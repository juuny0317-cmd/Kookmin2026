from cam.perception_utils import (
    normalize_model_names,
    parse_name_list,
    resolve_class_ids,
    resolve_enabled_class_ids,
    scaled_bbox,
    scaled_odd,
    source_is_fresh,
)


def test_parse_and_resolve_class_names():
    names = {0: 'center_line', 1: 'obstacle_vehicle', 4: 'cone'}
    assert parse_name_list('center_line, cone') == ['center_line', 'cone']
    assert resolve_class_ids(names, 'center_line,cone') == [0, 4]


def test_resolve_class_names_rejects_missing_name():
    try:
        resolve_class_ids({0: 'center_line'}, 'cone')
    except ValueError as exc:
        assert 'cone' in str(exc)
    else:
        raise AssertionError('Missing class name was silently accepted')


def test_disabled_checkerboard_never_expands_to_all_classes():
    names = {0: 'center_line', 1: 'checkerboard'}
    try:
        resolve_enabled_class_ids(
            names, 'checkerboard', enable_checkerboard=False)
    except ValueError as exc:
        assert 'No inference classes remain' in str(exc)
    else:
        raise AssertionError('Disabled checkerboard expanded to every class')

    assert resolve_enabled_class_ids(
        names, '', enable_checkerboard=False) == [0]


def test_normalize_model_names_accepts_sequence():
    assert normalize_model_names(['lane', 'cone']) == {0: 'lane', 1: 'cone'}


def test_bbox_scaling_and_clamping():
    assert scaled_bbox((100, 120, 500, 480), 640, 480, 320, 240) == (
        50, 60, 250, 240)
    assert scaled_bbox((-20, -10, 800, 600), 640, 480, 320, 240) == (
        0, 0, 320, 240)


def test_input_kernel_scaling_stays_odd():
    assert scaled_odd(7, 0.5) == 3
    assert scaled_odd(13, 0.5) == 7


def test_source_freshness_rejects_future_and_stale_data():
    reference = 10_000_000_000
    assert source_is_fresh(reference, reference - 200_000_000, 0.35)
    assert not source_is_fresh(reference, reference - 400_000_000, 0.35)
    assert not source_is_fresh(reference, reference + 1, 0.35)
    assert not source_is_fresh(reference, 0, 0.35)
