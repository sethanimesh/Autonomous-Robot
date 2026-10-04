#!/usr/bin/env python3
"""Run pinned local vision models on retained images and render scientific figures.

No robot endpoints, ROS nodes or cloud adapters are started. Checkpoints must
already exist locally. Render-only mode uses the saved numerical outputs.
"""

import argparse
from datetime import datetime, timezone
import hashlib
import importlib.metadata
import json
import os
from pathlib import Path
import platform
import subprocess
import sys
import time


ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))
sys.dont_write_bytecode = True
os.environ['HF_HUB_OFFLINE'] = '1'
os.environ['TRANSFORMERS_OFFLINE'] = '1'
os.environ.setdefault('MPLCONFIGDIR', '/tmp/robot-visual-scenarios-mpl')

MODELS = {
    'segformer': {
        'name': 'nvidia/segformer-b0-finetuned-ade-512-512',
        'revision': '489d5cd81a0b59fab9b7ea758d3548ebe99677da',
        'weight_sha256': '6ae39addd01de6b1b8bde2cf677d43a5cd733424b8d186de3f95d1c51fee23f9',
        'source': 'https://huggingface.co/nvidia/segformer-b0-finetuned-ade-512-512',
    },
    'depth': {
        'name': 'depth-anything/Depth-Anything-V2-Metric-Indoor-Small-hf',
        'revision': '8078d68a9c75a972131914f6afd0c1723be0da7f',
        'weight_sha256': 'e990eb82fbf11b05b7813261196a2b841bdcf5a05f64396724a8987fa90504a3',
        'source': 'https://huggingface.co/depth-anything/Depth-Anything-V2-Metric-Indoor-Small-hf',
    },
    'yolox': {
        'name': 'YOLOX-s', 'revision': '0.1.1rc0',
        'weight_sha256': 'c5c2d13e59ae883e6af3b45daea64af4833a4951c92d116ec270d9ddbe998063',
        'source': 'https://github.com/Megvii-BaseDetection/YOLOX/releases/tag/0.1.1rc0',
    },
    'yunet': {
        'name': 'YuNet', 'revision': '2023mar',
        'weight_sha256': '8f2383e4dd3cfbb4553ea8718107fc0423210dc964f9f4280604804ed2552fa4',
        'source': 'https://github.com/opencv/opencv_zoo/tree/main/models/face_detection_yunet',
    },
}


def digest(path):
    result = hashlib.sha256()
    with Path(path).open('rb') as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b''):
            result.update(chunk)
    return result.hexdigest()


def verified_manifest(path):
    value = json.loads(path.read_text())
    if value.get('schema_version') != 1 or not value.get('images'):
        raise ValueError('Invalid or empty image manifest')
    seen = set()
    for item in value['images']:
        image = (ROOT / item['path']).resolve()
        if not image.is_relative_to(ROOT) or item['id'] in seen:
            raise ValueError('Invalid image path or duplicate ID')
        if digest(image) != item['sha256']:
            raise ValueError('Image digest changed: ' + item['id'])
        seen.add(item['id'])
    return value


def verify_weights(name, location):
    if not location:
        raise ValueError('Provide the local ' + name + ' checkpoint path')
    path = Path(location).resolve()
    if path.is_dir():
        path = path / 'model.safetensors'
    actual = digest(path)
    if actual != MODELS[name]['weight_sha256']:
        raise ValueError('Checkpoint digest differs from the pinned ' + name + ' weights')


def semantic_groups(labels, confidence, id2label, floor_ids, threshold):
    """Group genuine argmax classes for a readable fixed-color diagnostic plot."""
    import numpy as np
    groups = np.full(labels.shape, 4, dtype=np.uint8)
    groups[np.isin(labels, floor_ids)] = 0
    for key, name in id2label.items():
        label_id = int(key)
        if name == 'person':
            groups[labels == label_id] = 1
        elif name in ('chair', 'table', 'cabinet', 'armchair', 'seat', 'sofa', 'bed'):
            groups[labels == label_id] = 2
        elif name == 'ceiling':
            groups[labels == label_id] = 3
    groups[confidence < threshold] = 5
    return groups


def render(manifest, report, output):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    from matplotlib.colors import ListedColormap
    from matplotlib.patches import Circle, Patch, Polygon, Rectangle
    import numpy as np
    from PIL import Image
    from robot.jetson.navigation.image_corridors import route_corridors

    palette = ['#27A881', '#E6A444', '#7F8DA6', '#98BDE0', '#344D69', '#DFE4EA']
    group_names = ['Floor / rug', 'Person', 'Furniture', 'Ceiling', 'Other known', 'Low confidence']
    overview_rows = []
    figure_dir = output / 'figures'
    figure_dir.mkdir(parents=True, exist_ok=True)
    by_id = {row['id']: row for row in report['images']}
    for item in manifest['images']:
        row = by_id[item['id']]
        if row['sha256'] != item['sha256']:
            raise ValueError('Stored inference image binding changed')
        arrays_path = ROOT / row['arrays']
        if digest(arrays_path) != row['arrays_sha256']:
            raise ValueError('Stored inference arrays changed')
        with np.load(arrays_path, allow_pickle=False) as arrays:
            groups, depth = arrays['semantic_groups'], arrays['depth']
        image = Image.open(ROOT / item['path']).convert('RGB')
        fig, axes = plt.subplots(2, 2, figsize=(11.5, 9.1), dpi=140)
        fig.patch.set_facecolor('#F7F9FC')
        fig.suptitle(item['title'], x=.055, y=.98, ha='left', fontsize=17, weight='bold', color='#16273A')
        fig.text(.055, .935, 'Retained camera input · fresh offline inference · CPU · no robot motion', fontsize=9, color='#52667E')
        titles = ['Recorded camera input', 'YOLOX-s + YuNet 2023mar · CPU',
                  'SegFormer-B0 · confidence ≥ 0.65', 'Depth Anything V2 · unvalidated']
        for ax, title in zip(axes.flat, titles):
            ax.set_title(title, loc='left', fontsize=11, pad=8, color='#16273A')
            ax.set_xticks([])
            ax.set_yticks([])
            for spine in ax.spines.values():
                spine.set_color('#CCD5E0')
        axes[0, 0].imshow(image)
        axes[0, 1].imshow(image)
        for detection in row['person_detections']:
            x1, y1, x2, y2 = detection['box']
            axes[0, 1].add_patch(Rectangle((x1, y1), x2-x1, y2-y1, fill=False, edgecolor='#00E1D0', linewidth=1.8))
            axes[0, 1].text(x1, max(12, y1-5), 'person {:.2f}'.format(detection['score']), fontsize=8,
                            color='#002C32', bbox=dict(facecolor='#00E1D0', edgecolor='none', alpha=.88, pad=2))
        for detection in row['face_detections']:
            x, y, width, height = detection['box_xywh']
            axes[0, 1].add_patch(Rectangle((x, y), width, height, fill=False, edgecolor='#FC62CE', linewidth=1.8))
            for point in detection['landmarks']:
                axes[0, 1].add_patch(Circle(point, 2, facecolor='#FC62CE'))
            axes[0, 1].text(x, max(12, y-5), 'face {:.2f}'.format(detection['score']), fontsize=8,
                            color='#3A002B', bbox=dict(facecolor='#FC62CE', edgecolor='none', alpha=.88, pad=2))
        axes[1, 0].imshow(groups, cmap=ListedColormap(palette), vmin=-.5, vmax=5.5, interpolation='nearest')
        width, height = image.size
        corridors = route_corridors(row['route']['floor_horizon_y'])
        for name, corridor, evidence in zip(('L', 'C', 'R'), corridors, row['route']['evidence']):
            vertices = [(corridor.top_center_x-corridor.top_half_width, corridor.top_y),
                        (corridor.top_center_x+corridor.top_half_width, corridor.top_y),
                        (corridor.bottom_center_x+corridor.bottom_half_width, corridor.bottom_y),
                        (corridor.bottom_center_x-corridor.bottom_half_width, corridor.bottom_y)]
            axes[1, 0].add_patch(Polygon([(x*width, y*height) for x, y in vertices], fill=False,
                                               edgecolor='white', linewidth=1.1, linestyle='--'))
            axes[1, 0].text(corridor.bottom_center_x*width, .94*height,
                            '{} {:.0%}'.format(name, evidence['floor_fraction']), ha='center', fontsize=8,
                            color='white', bbox=dict(facecolor='#182938', edgecolor='none', alpha=.8, pad=2))
        axes[1, 0].legend(handles=[Patch(facecolor=c, label=n) for c, n in zip(palette, group_names)],
                           loc='upper left', fontsize=7, ncol=2, framealpha=.95)
        plotted = axes[1, 1].imshow(depth, cmap='magma_r', vmin=0, vmax=8, interpolation='nearest')
        colorbar = fig.colorbar(plotted, ax=axes[1, 1], fraction=.033, pad=.022)
        colorbar.ax.tick_params(labelsize=8)
        colorbar.set_label('Raw model depth; unvalidated scale', fontsize=8)
        decision = row['route']['decision']
        note = 'Local corridor proposal: ' + ('blocked' if decision['blocked'] else '{:+.0f}° / {:.0f} cm'.format(
            decision['heading_degrees'], decision['distance_m']*100))
        fig.text(.055, .028, note + ' · requires separate current-scene and VLM checks before movement', fontsize=9, color='#52667E')
        fig.subplots_adjust(left=.055, right=.945, top=.895, bottom=.075, hspace=.18, wspace=.13)
        path = figure_dir / (item['id']+'.png')
        fig.savefig(path, facecolor=fig.get_facecolor())
        plt.close(fig)
        row['figure'] = str(path.relative_to(ROOT))
        row['figure_sha256'] = digest(path)
        overview_rows.append((item, groups))

    # This overview compares actual semantic outputs with one fixed legend.
    fig, axes = plt.subplots(3, 4, figsize=(15, 9.5), dpi=140)
    fig.patch.set_facecolor('#F7F9FC')
    fig.suptitle('Twelve recorded views · SegFormer-B0 semantic evidence', x=.035, y=.98,
                 ha='left', fontsize=20, weight='bold', color='#16273A')
    for ax, (item, groups) in zip(axes.flat, overview_rows):
        ax.imshow(groups, cmap=ListedColormap(palette), vmin=-.5, vmax=5.5, interpolation='nearest')
        ax.set_title(item['title'], fontsize=10, loc='left')
        ax.axis('off')
    fig.legend(handles=[Patch(facecolor=c, label=n) for c, n in zip(palette, group_names)],
               loc='lower center', ncol=6, fontsize=10, frameon=False)
    fig.subplots_adjust(left=.035, right=.975, top=.91, bottom=.065, wspace=.08, hspace=.23)
    overview = figure_dir/'segmentation-overview.png'
    fig.savefig(overview, facecolor=fig.get_facecolor())
    plt.close(fig)
    report['overview'] = str(overview.relative_to(ROOT))
    report['overview_sha256'] = digest(overview)


def run(manifest, args, output):
    import cv2
    import numpy as np
    from PIL import Image
    import torch
    from transformers import AutoImageProcessor, AutoModelForDepthEstimation, SegformerForSemanticSegmentation
    from robot.mac.route_perception import RoutePerceptionEngine, validate_floor_label_ids
    from robot.mac.person_range import MetricDepthBackend
    from robot.jetson.perception.inference import OpenCVDnnBackend, PersonDetector
    from robot.jetson.perception.detections import select_person_detections

    for name, path in [('segformer', args.segformer_snapshot), ('depth', args.depth_snapshot),
                       ('yolox', args.yolox_onnx), ('yunet', args.yunet_onnx)]:
        verify_weights(name, path)
    torch.set_num_threads(4)
    cv2.setNumThreads(4)
    torch.manual_seed(0)
    torch.use_deterministic_algorithms(True)
    route = RoutePerceptionEngine(device='cpu')
    route.processor = AutoImageProcessor.from_pretrained(args.segformer_snapshot, local_files_only=True)
    route.model = SegformerForSemanticSegmentation.from_pretrained(args.segformer_snapshot, local_files_only=True).eval()
    route.torch, route.numpy, route.image_type = torch, np, Image
    validate_floor_label_ids(route.model.config.id2label, route.floor_ids)
    captured = {}
    handle = route.model.register_forward_hook(lambda _model, _inputs, value: captured.update(logits=value.logits))
    depth = MetricDepthBackend(device='cpu')
    depth.processor = AutoImageProcessor.from_pretrained(args.depth_snapshot, local_files_only=True)
    depth.model = AutoModelForDepthEstimation.from_pretrained(args.depth_snapshot, local_files_only=True).eval()
    person = PersonDetector(OpenCVDnnBackend(args.yolox_onnx, (640, 640)), 640, 640, .45, warmup_iterations=1)
    face = cv2.FaceDetectorYN.create(args.yunet_onnx, '', (640, 480), .60, .30, 5000,
                                    cv2.dnn.DNN_BACKEND_OPENCV, cv2.dnn.DNN_TARGET_CPU)
    model_metadata = json.loads(json.dumps(MODELS))
    for name, location in [('segformer', args.segformer_snapshot), ('depth', args.depth_snapshot)]:
        model_metadata[name]['configuration_sha256'] = {
            file.name: digest(file) for file in Path(location).glob('*.json')}
    model_metadata['yolox']['execution_backend'] = 'OpenCV DNN CPU; production preprocessing and decoding'
    model_metadata['yunet']['execution_backend'] = 'OpenCV FaceDetectorYN CPU; full-frame diagnostic'
    versions = {name: importlib.metadata.version(name) for name in ('torch', 'transformers', 'numpy', 'Pillow', 'matplotlib')}
    versions['opencv'] = cv2.__version__
    sources = ['robot/mac/route_perception.py', 'robot/mac/person_range.py',
               'robot/jetson/navigation/image_corridors.py', 'robot/jetson/navigation/local_planner.py',
               'robot/jetson/perception/inference.py', 'robot/jetson/perception/detections.py',
               'scripts/diagnostics/build_visual_scenarios.py']
    report = dict(schema_version=1, generated_at_utc=datetime.now(timezone.utc).isoformat(),
                  evidence_kind='actual_offline_inference_on_retained_real_images',
                  host=dict(os=platform.system(), architecture=platform.machine(), python=platform.python_version()),
                  device='cpu', precision='float32', versions=versions, models=model_metadata,
                  source_sha256={name: digest(ROOT/name) for name in sources},
                  git_base=subprocess.check_output(['git', 'rev-parse', 'HEAD'], cwd=ROOT, text=True).strip(),
                  segmentation_floor_labels={str(k): route.model.config.id2label[k] for k in route.floor_ids},
                  settings=dict(person_threshold=.45, person_nms_iou=.45, face_threshold=.60, face_nms_iou=.30,
                                semantic_confidence=.65, frame_annotation='retained source banners/overlays preserved'),
                  identity_verification_run=False, robot_motion_run=False, images=[])
    array_dir = output/'arrays'
    array_dir.mkdir(parents=True, exist_ok=True)
    for item in manifest['images']:
        source = ROOT/item['path']
        payload = source.read_bytes()
        image = Image.open(source).convert('RGB')
        if list(image.size) != [item['width'], item['height']]:
            raise ValueError('Manifest image size changed')
        local = route.infer(payload)
        with torch.inference_mode():
            logits = torch.nn.functional.interpolate(captured.pop('logits'), size=(image.height, image.width), mode='bilinear', align_corners=False)
            confidence, labels = logits.softmax(dim=1).max(dim=1)
        labels, confidence = labels[0].numpy().astype(np.uint8), confidence[0].numpy()
        groups = semantic_groups(labels, confidence, route.model.config.id2label, route.floor_ids, .65)
        began = time.perf_counter()
        predicted_depth = depth.infer(image)
        depth_ms = (time.perf_counter()-began)*1000
        bgr = cv2.imdecode(np.frombuffer(payload, np.uint8), cv2.IMREAD_COLOR)
        began = time.perf_counter()
        candidates, ratio = person.infer_candidates(bgr)
        detections = select_person_detections(candidates, 0, .45, .45, image.width, image.height, ratio)
        person_ms = (time.perf_counter()-began)*1000
        face.setInputSize(image.size)
        began = time.perf_counter()
        _, faces = face.detect(bgr)
        face_ms = (time.perf_counter()-began)*1000
        face_rows = [] if faces is None else [dict(box_xywh=[float(v) for v in value[:4]],
            landmarks=[[float(v) for v in value[k:k+2]] for k in range(4, 14, 2)], score=float(value[14])) for value in faces]
        arrays = array_dir/(item['id']+'.npz')
        np.savez_compressed(arrays, labels=labels, confidence=confidence.astype(np.float16),
                            semantic_groups=groups, depth=predicted_depth.astype(np.float32))
        ids, counts = np.unique(labels, return_counts=True)
        classes = sorted([dict(id=int(k), label=route.model.config.id2label[int(k)], fraction=float(count/labels.size))
                          for k, count in zip(ids, counts)], key=lambda value: value['fraction'], reverse=True)
        finite = predicted_depth[np.isfinite(predicted_depth)]
        row = dict(id=item['id'], sha256=item['sha256'], route=local,
                   person_detections=[dict(box=[d.x1, d.y1, d.x2, d.y2], score=d.score) for d in detections],
                   face_detections=face_rows, semantic_classes=classes,
                   semantic_group_fraction={name: float(np.mean(groups == k)) for k, name in enumerate(
                       ('floor', 'person', 'furniture', 'ceiling', 'other_known', 'low_confidence'))},
                   depth=dict(calibration_applied=False, physically_validated=False, finite_fraction=float(np.isfinite(predicted_depth).mean()),
                              percentiles={str(q): float(np.percentile(finite, q)) for q in (5, 50, 95)},
                              note='Pretrained metric model output; no independently verified physical range'),
                   timing_ms=dict(segmentation=local['inference_ms'], depth=round(depth_ms, 2),
                                  person=round(person_ms, 2), face=round(face_ms, 2)),
                   arrays=str(arrays.relative_to(ROOT)), arrays_sha256=digest(arrays))
        report['images'].append(row)
        (output/'local-results.json').write_text(json.dumps(report, indent=2, allow_nan=False)+'\n')
        print('{}: {} people, {} faces, floor {:.1%}, local route {}'.format(item['id'], len(detections),
              len(face_rows), row['semantic_group_fraction']['floor'], 'blocked' if local['decision']['blocked'] else 'candidate'), flush=True)
    handle.remove()
    person.close()
    return report


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--manifest', type=Path, default=ROOT/'evaluation/visual-scenarios/manifest.json')
    parser.add_argument('--output', type=Path, default=ROOT/'evaluation/visual-scenarios')
    parser.add_argument('--segformer-snapshot')
    parser.add_argument('--depth-snapshot')
    parser.add_argument('--yolox-onnx')
    parser.add_argument('--yunet-onnx')
    parser.add_argument('--render-only', action='store_true')
    args = parser.parse_args()
    manifest = verified_manifest(args.manifest.resolve())
    output = args.output.resolve()
    if not output.is_relative_to(ROOT):
        parser.error('Keep output inside this repository for relative artifact links')
    output.mkdir(parents=True, exist_ok=True)
    if args.render_only:
        report = json.loads((output/'local-results.json').read_text())
    else:
        report = run(manifest, args, output)
    render(manifest, report, output)
    (output/'local-results.json').write_text(json.dumps(report, indent=2, allow_nan=False)+'\n')
    print('Saved local inference records, numerical arrays and 13 scientific figures.', flush=True)


if __name__ == '__main__':
    main()
