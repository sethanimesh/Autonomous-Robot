"""Small dependency-free image checks for the loaded EV3 camera head.

Encoder movement proves that motor A turned; it does not prove that the camera
linkage followed or remained at the new angle.  These helpers reduce a raw ROS
image to a tiny luminance fingerprint and compare fingerprints before, after,
and shortly after a camera-head command.
"""

from statistics import median


class CameraMotionFeedbackError(RuntimeError):
    """Raised when live images do not prove a stable camera movement."""

    def __init__(self, message, report=None):
        super().__init__(message)
        self.report = report


def _positive_integer(name, value):
    value = int(value)
    if value <= 0:
        raise ValueError("{0} must be positive".format(name))
    return value


def frame_fingerprint(
    data,
    width,
    height,
    row_step,
    encoding="bgr8",
    columns=32,
    rows=24,
):
    """Return a compact luminance grid from an uncompressed RGB/BGR frame."""

    width = _positive_integer("frame width", width)
    height = _positive_integer("frame height", height)
    row_step = _positive_integer("frame row step", row_step)
    columns = _positive_integer("fingerprint columns", columns)
    rows = _positive_integer("fingerprint rows", rows)
    encoding = str(encoding).lower()
    if encoding not in ("bgr8", "rgb8"):
        raise ValueError("camera feedback requires bgr8 or rgb8 frames")
    if row_step < width * 3:
        raise ValueError("frame row step is smaller than its RGB payload")
    if len(data) < row_step * height:
        raise ValueError("camera frame payload is truncated")

    values = []
    for row in range(rows):
        y = min(height - 1, int((row + 0.5) * height / rows))
        base = y * row_step
        for column in range(columns):
            x = min(width - 1, int((column + 0.5) * width / columns))
            offset = base + x * 3
            first = int(data[offset])
            green = int(data[offset + 1])
            third = int(data[offset + 2])
            if encoding == "bgr8":
                blue, red = first, third
            else:
                red, blue = first, third
            values.append((77 * red + 150 * green + 29 * blue) >> 8)
    return {"columns": columns, "rows": rows, "values": tuple(values)}


def fingerprint_quality(fingerprint):
    """Describe whether a frame is bright and textured enough for feedback."""

    values = tuple(int(value) for value in fingerprint.get("values", ()))
    columns = int(fingerprint.get("columns", 0))
    rows = int(fingerprint.get("rows", 0))
    expected = columns * rows
    if not values or len(values) != expected:
        raise ValueError("camera fingerprint has invalid geometry")
    midpoint = float(median(values))
    contrast = sum(abs(value - midpoint) for value in values) / float(len(values))
    # Global contrast alone mistakes a smooth close-up gradient for a useful
    # scene. Second differences require repeated local detail/edges across the
    # image and reject the blurred support obstruction seen on the live head.
    details = []
    for row in range(rows):
        for column in range(columns):
            index = row * columns + column
            if 0 < column < columns - 1:
                details.append(
                    abs(
                        values[index - 1]
                        - 2 * values[index]
                        + values[index + 1]
                    )
                )
            if 0 < row < rows - 1:
                details.append(
                    abs(
                        values[index - columns]
                        - 2 * values[index]
                        + values[index + columns]
                    )
                )
    detail_mean = (
        sum(details) / float(len(details)) if details else 0.0
    )
    detail_fraction = (
        sum(value >= 12 for value in details) / float(len(details))
        if details
        else 0.0
    )
    return {
        "mean_luma": round(sum(values) / float(len(values)), 3),
        "median_luma": round(midpoint, 3),
        "mean_absolute_contrast": round(contrast, 3),
        "mean_second_difference": round(detail_mean, 3),
        "detail_fraction": round(detail_fraction, 4),
        "usable": (
            midpoint >= 12.0
            and contrast >= 3.0
            and detail_mean >= 12.0
            and detail_fraction >= 0.08
        ),
    }


def compare_fingerprints(before, after, pixel_delta=12):
    """Measure scene-wide change while ignoring a uniform exposure shift."""

    if (
        before.get("columns") != after.get("columns")
        or before.get("rows") != after.get("rows")
    ):
        raise ValueError("camera fingerprints have different geometry")
    first = tuple(int(value) for value in before.get("values", ()))
    second = tuple(int(value) for value in after.get("values", ()))
    if not first or len(first) != len(second):
        raise ValueError("camera fingerprints have invalid samples")
    threshold = _positive_integer("pixel delta", pixel_delta)
    deltas = [new - old for old, new in zip(first, second)]
    exposure_delta = float(median(deltas))
    residuals = [abs(delta - exposure_delta) for delta in deltas]
    return {
        "changed_fraction": round(
            sum(value >= threshold for value in residuals) / float(len(residuals)),
            4,
        ),
        "mean_absolute_delta": round(sum(residuals) / float(len(residuals)), 3),
        "exposure_delta": round(exposure_delta, 3),
    }


def verify_camera_step(
    before,
    after,
    settled,
    minimum_changed_fraction=0.18,
    minimum_mean_delta=7.0,
    maximum_settle_changed_fraction=0.25,
    maximum_settle_mean_delta=10.0,
    allow_textureless_view=False,
):
    """Require visible movement followed by a visually stable held pose."""

    before_quality = fingerprint_quality(before)
    after_quality = fingerprint_quality(after)
    settled_quality = fingerprint_quality(settled)
    transition = compare_fingerprints(before, after)
    drift = compare_fingerprints(after, settled)
    report = {
        "before": before_quality,
        "after": after_quality,
        "settled": settled_quality,
        "transition": transition,
        "post_move_drift": drift,
    }
    qualities = (after_quality, settled_quality)
    if not all(value["usable"] or (
        allow_textureless_view and 12 <= value["median_luma"] <= 245
    ) for value in qualities):
        raise CameraMotionFeedbackError(
            "camera view is dark, occluded, or lacks usable texture", report
        )
    if (
        transition["changed_fraction"] < float(minimum_changed_fraction)
        or transition["mean_absolute_delta"] < float(minimum_mean_delta)
    ):
        raise CameraMotionFeedbackError(
            "fresh frames do not prove that the camera linkage moved", report
        )
    if (
        drift["changed_fraction"] > float(maximum_settle_changed_fraction)
        or drift["mean_absolute_delta"] > float(maximum_settle_mean_delta)
    ):
        raise CameraMotionFeedbackError(
            "camera view continued moving after motor A reported stopped", report
        )
    if (
        transition["changed_fraction"] < drift["changed_fraction"] + 0.12
        or transition["mean_absolute_delta"] < drift["mean_absolute_delta"] + 5.0
    ):
        raise CameraMotionFeedbackError(
            "commanded camera motion is not distinct from post-move drift", report
        )
    report["verified"] = True
    return report
