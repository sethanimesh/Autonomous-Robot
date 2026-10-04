#!/usr/bin/env python3
"""Render saved genuine VLM observations; no inference or network access."""
import argparse
import json
from pathlib import Path
import sys
import textwrap

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))
sys.dont_write_bytecode = True


def validated_inputs(directory):
    """Reject stale saved responses before importing renderers or writing files.

    These helpers only read files and regenerate request descriptions. They do
    not construct cloud clients, run vision models, or contact robot services.
    """
    from scripts.diagnostics.run_visual_vlm import (
        MODES, EVIDENCE_KIND, checked_images, checked_routes, json_digest,
        read_json, task_plan,
    )
    from robot.mac.person_search_advisor import validate_search_observations
    from robot.jetson.navigation.navigation_reasoning import (
        validate_route_advice, validate_occlusion_advice,
    )
    from robot.mac.wardrobe_advisor import validate as validate_wardrobe

    manifest = read_json(directory/'manifest.json')
    local = read_json(directory/'local-results.json')
    vlm = read_json(directory/'vlm-results.json')
    images = checked_images(manifest)
    if (not isinstance(local, dict) or local.get('schema_version') != 1
            or local.get('evidence_kind') != 'actual_offline_inference_on_retained_real_images'
            or not isinstance(local.get('images'), list)):
        raise ValueError('Unsupported local inference report')
    seen = set()
    for row in local['images']:
        if (not isinstance(row, dict) or row.get('id') not in images or row['id'] in seen
                or row.get('sha256') != images[row['id']]['sha256']):
            raise ValueError('Local measurement source binding changed')
        seen.add(row['id'])
    if (not isinstance(vlm, dict) or vlm.get('schema_version') != 1
            or vlm.get('evidence_kind') != EVIDENCE_KIND
            or vlm.get('manifest_sha256') != json_digest(manifest)
            or not isinstance(vlm.get('tasks'), list)):
        raise ValueError('VLM report schema or manifest binding changed')
    routes = checked_routes(directory/'local-results.json', images)
    plans, completed, seen = {}, [], set()
    for task in vlm['tasks']:
        if (not isinstance(task, dict) or not isinstance(task.get('task_id'), str)
                or task['task_id'] in seen):
            raise ValueError('Invalid or duplicate VLM task')
        seen.add(task['task_id'])
        if task.get('status') != 'completed':
            continue
        model = task.get('model')
        if not isinstance(model, str) or task.get('provider') != 'gemini':
            raise ValueError('Completed task lacks a recorded Gemini model')
        if model not in plans:
            plans[model] = {row['task_id']: row for row in
                            task_plan(images, routes, MODES, model, manifest.get('framing_batches'))}
        expected = plans[model].get(task['task_id'])
        if (expected is None or task.get('mode') != expected['mode']
                or task.get('image_ids') != expected['image_ids']
                or task.get('input_context') != expected['context']
                or task.get('request_signature') != expected['request_signature']
                or task.get('request_sha256') != expected['request_sha256']
                or task.get('frame_sha256') != expected['request_signature']['frame_sha256']
                or task.get('prompt_sha256') != expected['request_signature']['prompt_sha256']):
            raise ValueError('Cached model, source, prompt, schema or context changed: '+task['task_id'])
        parsed = task.get('parsed_response')
        if task['mode'] == 'framing':
            validate_search_observations({'observations': parsed}, len(task['image_ids']))
        elif task['mode'] == 'route':
            validate_route_advice(parsed)
        elif task['mode'] == 'occlusion':
            validate_occlusion_advice(parsed)
        elif task['mode'] == 'wardrobe':
            validate_wardrobe(parsed, [], 'describe')
        completed.append(task)
    return manifest, local, completed


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--directory', type=Path, default=ROOT/'evaluation/visual-scenarios')
    args = parser.parse_args()
    directory = args.directory.resolve()
    manifest, local, completed = validated_inputs(directory)
    images = {row['id']: row for row in manifest['images']}
    measurements = {row['id']: row for row in local['images']}

    import os
    os.environ.setdefault('MPLCONFIGDIR', '/tmp/robot-visual-scenarios-mpl')
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    from matplotlib.patches import Polygon
    from PIL import Image
    from robot.jetson.navigation.image_corridors import route_corridors
    from robot.jetson.navigation.navigation_reasoning import fuse_route_evidence

    figures = directory/'figures'
    figures.mkdir(parents=True, exist_ok=True)
    summaries = []
    for task in completed:
        if task['mode'] != 'route':
            continue
        item = images[task['image_ids'][0]]
        measurement = measurements[item['id']]
        if measurement['sha256'] != item['sha256']:
            raise ValueError('Local measurement source binding changed')
        advice = task['parsed_response']
        # This preview intentionally has no fresh capture. It demonstrates the
        # intersection rule using one saved image, never a runtime approval.
        preview = fuse_route_evidence(measurement['route'], measurement['route'], advice)
        summaries.append(dict(id=item['id'], sha256=item['sha256'],
            evidence_kind='single_saved_frame_conditional_policy_preview',
            initial_and_fresh_local_source_are_same_image=True, fresh_scene_recheck=False,
            movement_authorized=False, vlm_response_task=task['task_id'], result=preview))
        image = Image.open(ROOT/item['path']).convert('RGB')
        fig, axes = plt.subplots(1, 2, figsize=(12.8, 6.6), dpi=140, gridspec_kw={'width_ratios': [1.12, 1]})
        fig.patch.set_facecolor('#F7F9FC')
        fig.suptitle(item['title']+' · VLM route review', x=.04, y=.97, ha='left', fontsize=17,
                     weight='bold', color='#16273A')
        fig.text(.04, .918, task['model']+' · actual saved-image response · advisory only', fontsize=10, color='#52667E')
        axes[0].imshow(image)
        axes[0].set_xticks([])
        axes[0].set_yticks([])
        axes[0].set_title('Exact image-space candidates sent to Gemini', fontsize=10, loc='left', pad=8)
        axes[1].axis('off')
        width, height = image.size
        colors = {'none': '#249C79', 'unknown': '#C58B20'}
        geometry = route_corridors(measurement['route']['floor_horizon_y'])
        by_name = {row['id']: row for row in advice['corridors']}
        for index, (name, corridor) in enumerate(zip(('left', 'center', 'right'), geometry)):
            value = by_name[name]
            color = colors.get(value['hazard'], '#C44949')
            vertices = [(corridor.top_center_x-corridor.top_half_width, corridor.top_y),
                        (corridor.top_center_x+corridor.top_half_width, corridor.top_y),
                        (corridor.bottom_center_x+corridor.bottom_half_width, corridor.bottom_y),
                        (corridor.bottom_center_x-corridor.bottom_half_width, corridor.bottom_y)]
            axes[0].add_patch(Polygon([(x*width, y*height) for x, y in vertices],
                                      facecolor=color, edgecolor=color, alpha=.27, linewidth=2))
            axes[0].text(corridor.bottom_center_x*width, .92*height, name[0].upper(), fontsize=11,
                          ha='center', color='white', weight='bold',
                          bbox=dict(facecolor=color, edgecolor='none', pad=3))
            y = .91-index*.255
            axes[1].text(.02, y, name.upper()+'  /  '+value['hazard'], transform=axes[1].transAxes,
                          fontsize=12, weight='bold', color=color)
            axes[1].text(.02, y-.055, 'Visibility: '+value['visibility'], transform=axes[1].transAxes,
                          fontsize=10, color='#52667E')
            axes[1].text(.02, y-.115, textwrap.fill(value['evidence'], 54), transform=axes[1].transAxes,
                          fontsize=10, va='top', linespacing=1.5, color='#16273A')
        axes[1].text(.02, .13, 'Preference: '+' → '.join(advice['preference']),
                      transform=axes[1].transAxes, fontsize=10, color='#16273A')
        axes[1].text(.02, .07, 'Near turn space: '+advice['near_turn_space'],
                      transform=axes[1].transAxes, fontsize=10, color='#52667E')
        decision = preview['decision']
        result = 'blocked' if decision['blocked'] else '{:+.0f}° / {:.0f} cm candidate'.format(
            decision['heading_degrees'], decision['distance_m']*100)
        fig.text(.04, .075, 'Conditional rule preview: '+result, fontsize=11, weight='bold', color='#16273A')
        fig.text(.04, .037, 'One saved frame reused for the preview · fresh scene not rechecked · movement disabled',
                 fontsize=9, color='#52667E')
        fig.subplots_adjust(left=.04, right=.97, top=.855, bottom=.14, wspace=.13)
        fig.savefig(figures/('vlm-route-'+item['id']+'.png'), facecolor=fig.get_facecolor())
        plt.close(fig)

    observations = {}
    for task in completed:
        if task['mode'] == 'framing':
            if len(task['parsed_response']) != len(task['image_ids']):
                raise ValueError('Framing observation order changed')
            observations.update(zip(task['image_ids'], task['parsed_response']))
    if observations:
        observed_images = [item for item in manifest['images'] if item['id'] in observations]
        columns = min(3, len(observed_images))
        rows = (len(observed_images) + columns - 1) // columns
        fig, axes = plt.subplots(rows, columns, figsize=(15.5, max(4, rows*3.6)), dpi=140,
                                 squeeze=False)
        fig.patch.set_facecolor('#F7F9FC')
        fig.suptitle('Recorded views · actual Gemini framing observations', x=.035, y=.98,
                     ha='left', fontsize=19, weight='bold', color='#16273A')
        fig.text(.035, .94, 'Independent image interpretations; human visibility is not recipient identity confirmation',
                 fontsize=11, color='#52667E')
        for ax in axes.flat:
            ax.axis('off')
        for ax, item in zip(axes.flat, observed_images):
            ax.axis('on')
            ax.imshow(Image.open(ROOT/item['path']).convert('RGB'))
            ax.set_xticks([])
            ax.set_yticks([])
            ax.set_title(item['title'], fontsize=10, loc='left', pad=7)
            value = observations.get(item['id'])
            if value:
                label = '{} · {}\nhuman: {} · framing: {}'.format(value['scene'], value['quality'],
                                                                     value['human_visible'], value['framing_hint'])
                ax.set_xlabel(label, fontsize=9, color='#52667E')
        fig.subplots_adjust(left=.035, right=.975, top=.885, bottom=.06, wspace=.10, hspace=.35)
        fig.savefig(figures/'vlm-framing-overview.png', facecolor=fig.get_facecolor())
        plt.close(fig)
    (directory/'fusion-previews.json').write_text(json.dumps(dict(schema_version=1,
        scope='conditional single-image illustration; not motion authorization', previews=summaries), indent=2)+'\n')
    print('Rendered {} actual route responses and framing overview; saved conditional rule previews.'.format(len(summaries)))


if __name__ == '__main__':
    main()
