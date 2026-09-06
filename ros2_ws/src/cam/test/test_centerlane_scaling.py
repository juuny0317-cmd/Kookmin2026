import math

from cam.centerlane_tracer import CenterlaneTracer


def test_centerlane_320_processing_constants_keep_physical_scale():
    tracer = CenterlaneTracer.__new__(CenterlaneTracer)
    tracer.BASE_INPUT_WIDTH = 640.0
    tracer.BASE_INPUT_HEIGHT = 480.0
    tracer.HEIGHT_THRESHOLD = 220
    tracer.POINT_MERGE_DISTANCE = 2.0
    tracer.SPLINE_SMOOTHING = 30.0
    tracer.params = {
        'adapt_method_val': 1,
        'block_size_val': 5,
        'c_constant_val': 5,
        'canny_thresh1': 50,
        'canny_thresh2': 150,
        'hough_threshold': 20,
        'hough_min_len': 20,
        'hough_max_gap': 5,
        'similarity_thresh': 70,
        'slope_thresh_x100': 13,
        'bottom_box_height_thresh': 10,
        'bottom_point_height_diff_thresh': 5,
        'angle_similarity_thresh': 20.0,
        'length_similarity_thresh': 0.6,
        'min_bbox_width': 5,
        'min_bbox_height': 5,
        'x_dist_thresh': 300,
        'rmse_thresh': 30,
    }

    scaled = tracer._scaled_processing_parameters(320, 240)

    assert scaled['block_size'] == 7
    assert scaled['height_threshold'] == 110
    assert scaled['hough_threshold'] == 10
    assert scaled['hough_min_len'] == 10
    assert scaled['hough_max_gap'] == 3
    assert math.isclose(scaled['similarity_thresh'], 35.0)
    assert math.isclose(scaled['rmse_thresh'], 15.0)
    assert math.isclose(scaled['x_dist_thresh'], 150.0)
    assert math.isclose(scaled['point_merge_distance'], 1.0)
    assert math.isclose(scaled['spline_smoothing'], 7.5)


def test_path_extensions_stop_at_first_image_boundary():
    tracer = CenterlaneTracer.__new__(CenterlaneTracer)
    path = [
        (118.0, 130.0),
        (152.0, 125.0),
        (319.0, 100.0),
    ]

    extended = tracer._extend_path_to_boundaries(
        path,
        frame_height=240,
        frame_width=320,
    )

    assert extended[0][0] == 0.0
    assert all(0.0 <= point[0] <= 319.0 for point in extended)
    assert all(0.0 <= point[1] <= 239.0 for point in extended)


def test_curve_points_are_clipped_to_image_before_extension():
    tracer = CenterlaneTracer.__new__(CenterlaneTracer)
    path = [
        (-5.0, 245.0),
        (120.0, 130.0),
        (325.0, 95.0),
    ]

    extended = tracer._extend_path_to_boundaries(
        path,
        frame_height=240,
        frame_width=320,
    )

    assert all(0.0 <= point[0] <= 319.0 for point in extended)
    assert all(0.0 <= point[1] <= 239.0 for point in extended)
