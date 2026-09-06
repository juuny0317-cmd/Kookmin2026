from pathlib import Path

import cam
from cam.perception_utils import resolve_enabled_class_ids
from cam.yolo_node import canonical_class_name, parse_class_name_aliases
from ultralytics import YOLO


def model_path(filename):
    return Path(cam.__file__).resolve().parent / filename


def test_lane_model_contains_center_line():
    model = YOLO(str(model_path('center_line_yolov10n_320_best.pt')))
    assert resolve_enabled_class_ids(model.names, 'center_line') == [0]


def test_scene_model_contains_required_classes():
    model = YOLO(str(model_path(
        'all_second_yolo_v2_yolov10n_640_best_e46.pt')))
    selected = resolve_enabled_class_ids(
        model.names,
        'cone,dynamic,green,left,red,static',
    )
    selected_names = {model.names[class_id] for class_id in selected}
    assert selected_names == {
        'cone',
        'dynamic',
        'green',
        'left',
        'red',
        'static',
    }


def test_scene_dynamic_alias_matches_obstacle_contract():
    names = {0: 'cone', 1: 'dynamic', 2: 'green', 3: 'left', 4: 'red'}
    assert parse_class_name_aliases(
        'dynamic=obstacle_vehicle', names) == {
            'dynamic': 'obstacle_vehicle',
        }
    assert canonical_class_name(
        'dynamic', {'dynamic': 'obstacle_vehicle'}) == 'obstacle_vehicle'
    assert canonical_class_name(
        'cone', {'dynamic': 'obstacle_vehicle'}) == 'cone'
    assert canonical_class_name(
        'static', {'dynamic': 'obstacle_vehicle'}) == 'static'
