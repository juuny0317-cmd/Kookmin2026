"""Shared helpers for the split camera perception pipelines."""

from typing import Iterable, Mapping, Sequence


def parse_name_list(value) -> list[str]:
    """Return a normalized class-name list from a ROS parameter value."""
    if value is None:
        return []
    if isinstance(value, str):
        items = value.split(',')
    elif isinstance(value, Iterable):
        items = value
    else:
        items = [value]
    return [str(item).strip() for item in items if str(item).strip()]


def normalize_model_names(model_names) -> dict[int, str]:
    """Normalize Ultralytics list/dict class names to an integer-key dict."""
    if isinstance(model_names, Mapping):
        return {int(key): str(name) for key, name in model_names.items()}
    if isinstance(model_names, Sequence):
        return {index: str(name) for index, name in enumerate(model_names)}
    raise TypeError(f'Unsupported model.names type: {type(model_names)!r}')


def resolve_class_ids(model_names, requested_names) -> list[int]:
    """Resolve requested class names and fail instead of silently dropping any."""
    normalized = normalize_model_names(model_names)
    requested = parse_name_list(requested_names)
    if not requested:
        return sorted(normalized)

    name_to_id = {name: class_id for class_id, name in normalized.items()}
    missing = [name for name in requested if name not in name_to_id]
    if missing:
        available = ', '.join(normalized[class_id] for class_id in sorted(normalized))
        raise ValueError(
            f'Model does not contain requested class(es): {missing}. '
            f'Available classes: [{available}]'
        )
    return [name_to_id[name] for name in requested]


def resolve_enabled_class_ids(
    model_names,
    requested_names,
    enable_checkerboard: bool = True,
) -> list[int]:
    """Apply optional-class switches without turning an empty filter into all."""
    normalized = normalize_model_names(model_names)
    requested = parse_name_list(requested_names)
    had_explicit_filter = bool(requested)
    if not enable_checkerboard:
        if had_explicit_filter:
            requested = [name for name in requested if name != 'checkerboard']
        else:
            requested = [
                name for name in normalized.values()
                if name != 'checkerboard'
            ]
    if had_explicit_filter and not requested:
        raise ValueError(
            'No inference classes remain after applying class options. '
            'The requested list contained only disabled classes.'
        )
    if not requested:
        if not enable_checkerboard:
            raise ValueError('No enabled inference classes exist in this model.')
        requested = list(normalized.values())
    if not requested:
        raise ValueError('No enabled inference classes exist in this model.')
    return resolve_class_ids(normalized, requested)


def stamp_to_ns(stamp) -> int:
    """Convert a builtin_interfaces/Time-like object to nanoseconds."""
    return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)


def source_is_fresh(
    reference_ns: int,
    source_ns: int,
    max_age_s: float,
) -> bool:
    """Return true only for a non-future source within the age limit."""
    if reference_ns <= 0 or source_ns <= 0 or max_age_s < 0.0:
        return False
    age_ns = int(reference_ns) - int(source_ns)
    return 0 <= age_ns <= int(float(max_age_s) * 1_000_000_000)


def scaled_bbox(
    bbox,
    source_width: int,
    source_height: int,
    target_width: int,
    target_height: int,
) -> tuple[int, int, int, int]:
    """Scale and clamp an xyxy bounding box between image coordinate systems."""
    if source_width <= 0 or source_height <= 0:
        raise ValueError('Source image dimensions must be positive')
    if target_width <= 0 or target_height <= 0:
        raise ValueError('Target image dimensions must be positive')

    sx = float(target_width) / float(source_width)
    sy = float(target_height) / float(source_height)
    xmin, ymin, xmax, ymax = bbox
    x1 = max(0, min(target_width - 1, int(round(float(xmin) * sx))))
    y1 = max(0, min(target_height - 1, int(round(float(ymin) * sy))))
    x2 = max(x1 + 1, min(target_width, int(round(float(xmax) * sx))))
    y2 = max(y1 + 1, min(target_height, int(round(float(ymax) * sy))))
    return x1, y1, x2, y2


def scaled_odd(value: float, scale: float, minimum: int = 3) -> int:
    """Scale a pixel kernel size while keeping it odd and valid for OpenCV."""
    result = max(int(minimum), int(float(value) * float(scale)))
    if result % 2 == 0:
        result += 1
    return result
