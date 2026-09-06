#!/usr/bin/env python3
from __future__ import annotations

import math
import os
import shutil
import zipfile
from pathlib import Path

import yaml
import matplotlib.pyplot as plt
from matplotlib.patches import Polygon as MplPolygon
from shapely.geometry import LineString, Polygon, MultiPolygon, GeometryCollection, box
from shapely.affinity import rotate, translate
from shapely.ops import triangulate, unary_union

PKG_NAME = "square_intersection_gazebo_map"
WORLD_NAME = "square_intersection_loop"
OUT_ROOT = Path("/mnt/data/square_intersection_gazebo_map_detailed_build") / PKG_NAME
WORLD_DIR = OUT_ROOT / "worlds"
MESH_DIR = OUT_ROOT / "meshes"
CONFIG_DIR = OUT_ROOT / "config"
PREVIEW_DIR = OUT_ROOT / "preview"

# Main geometry
SIDE_LENGTH_M = 100.0
HALF_SIDE_M = SIDE_LENGTH_M / 2.0
ROAD_WIDTH_M = 7.0
LANE_WIDTH_M = 3.5
ROAD_HALF_M = ROAD_WIDTH_M / 2.0
CORNER_RADIUS_M = 12.0
CENTRAL_INTERSECTION_PAD_M = 14.0

# Road / marking heights
ASPHALT_TOP_Z = 0.04
ASPHALT_BOTTOM_Z = 0.0
YELLOW_Z = ASPHALT_TOP_Z + 0.0005
WHITE_Z = ASPHALT_TOP_Z + 0.0010

# Line widths
YELLOW_LINE_WIDTH_M = 0.075     # one strip of a double yellow centerline
YELLOW_DOUBLE_OFFSET_M = 0.11    # each yellow strip center is +/- this far from road centerline
EDGE_LINE_WIDTH_M = 0.15

# Detailed straight-road marking parameters from the reference image
DETAILED_SEGMENT_LENGTH_M = 48.0
CROSSWALK_U0_M = 47.0
CROSSWALK_U1_M = 48.0
CROSSWALK_STRIPE_W_M = 0.50
CROSSWALK_GAP_M = 0.50
DIAMOND_U_POSITIONS_M = [13.0, 40.5]
DIAMOND_LENGTH_M = 1.60
DIAMOND_WIDTH_M = 0.80
DIAMOND_LINE_WIDTH_M = 0.15
ARROW_CENTER_U_M = 34.0
ARROW_LENGTH_M = 3.0
ARROW_WIDTH_M = 0.60
STOP_LINE_U_M = 46.20
STOP_LINE_WIDTH_U_M = 0.30

# GPS origin
ORIGIN_LAT_DEG = 37.5665
ORIGIN_LON_DEG = 126.9780
ORIGIN_ELEV_M = 0.0


def prepare_dirs():
    if OUT_ROOT.exists():
        shutil.rmtree(OUT_ROOT)
    for d in [WORLD_DIR, MESH_DIR, CONFIG_DIR, PREVIEW_DIR]:
        d.mkdir(parents=True, exist_ok=True)


def rounded_square_centerline(side: float, radius: float, n_arc: int = 64):
    h, r = side / 2.0, radius
    pts = []
    pts.append((-h + r, -h))
    pts.append(( h - r, -h))
    cx, cy = h - r, -h + r
    for i in range(1, n_arc + 1):
        a = math.radians(-90 + 90 * i / n_arc)
        pts.append((cx + r * math.cos(a), cy + r * math.sin(a)))
    pts.append((h, h - r))
    cx, cy = h - r, h - r
    for i in range(1, n_arc + 1):
        a = math.radians(0 + 90 * i / n_arc)
        pts.append((cx + r * math.cos(a), cy + r * math.sin(a)))
    pts.append((-h + r, h))
    cx, cy = -h + r, h - r
    for i in range(1, n_arc + 1):
        a = math.radians(90 + 90 * i / n_arc)
        pts.append((cx + r * math.cos(a), cy + r * math.sin(a)))
    pts.append((-h, -h + r))
    cx, cy = -h + r, -h + r
    for i in range(1, n_arc + 1):
        a = math.radians(180 + 90 * i / n_arc)
        pts.append((cx + r * math.cos(a), cy + r * math.sin(a)))
    return pts


def iter_polygons(geom):
    if geom.is_empty:
        return
    if isinstance(geom, Polygon):
        yield geom
    elif isinstance(geom, MultiPolygon):
        for g in geom.geoms:
            yield g
    elif isinstance(geom, GeometryCollection):
        for g in geom.geoms:
            yield from iter_polygons(g)


def tx_local(u, v, origin, yaw):
    # local u is forward along the straight segment, local v is left of that direction
    c, s = math.cos(yaw), math.sin(yaw)
    x = origin[0] + u * c - v * s
    y = origin[1] + u * s + v * c
    return (x, y)


def rect_local(u0, u1, v0, v1, origin, yaw):
    return Polygon([
        tx_local(u0, v0, origin, yaw),
        tx_local(u1, v0, origin, yaw),
        tx_local(u1, v1, origin, yaw),
        tx_local(u0, v1, origin, yaw),
    ])


def polygon_local(points, origin, yaw):
    return Polygon([tx_local(u, v, origin, yaw) for u, v in points])


def diamond_outline(center_u, center_v, origin, yaw):
    L = DIAMOND_LENGTH_M
    W = DIAMOND_WIDTH_M
    outer_pts = [
        (center_u, center_v + W / 2.0),
        (center_u + L / 2.0, center_v),
        (center_u, center_v - W / 2.0),
        (center_u - L / 2.0, center_v),
    ]
    # Inner diamond shrunk by line width. Keep a minimum size.
    inner_L = max(0.05, L - 2.0 * DIAMOND_LINE_WIDTH_M)
    inner_W = max(0.05, W - 2.0 * DIAMOND_LINE_WIDTH_M)
    inner_pts = [
        (center_u, center_v + inner_W / 2.0),
        (center_u + inner_L / 2.0, center_v),
        (center_u, center_v - inner_W / 2.0),
        (center_u - inner_L / 2.0, center_v),
    ]
    outer = polygon_local(outer_pts, origin, yaw)
    inner = polygon_local(inner_pts, origin, yaw)
    return outer.difference(inner)


def arrow_polygon(center_u, center_v, direction_sign, origin, yaw):
    # Filled lane arrow. direction_sign=+1 points toward the central intersection; -1 points outward.
    L = ARROW_LENGTH_M
    W = ARROW_WIDTH_M
    head_len = L * 0.42
    shaft_len = L - head_len
    shaft_w = W * 0.32
    if direction_sign > 0:
        tail = center_u - L / 2.0
        tip = center_u + L / 2.0
        head_base = tip - head_len
        pts = [
            (tail, center_v - shaft_w / 2.0),
            (head_base, center_v - shaft_w / 2.0),
            (head_base, center_v - W / 2.0),
            (tip, center_v),
            (head_base, center_v + W / 2.0),
            (head_base, center_v + shaft_w / 2.0),
            (tail, center_v + shaft_w / 2.0),
        ]
    else:
        tail = center_u + L / 2.0
        tip = center_u - L / 2.0
        head_base = tip + head_len
        pts = [
            (tail, center_v - shaft_w / 2.0),
            (head_base, center_v - shaft_w / 2.0),
            (head_base, center_v - W / 2.0),
            (tip, center_v),
            (head_base, center_v + W / 2.0),
            (head_base, center_v + shaft_w / 2.0),
            (tail, center_v + shaft_w / 2.0),
        ]
    return polygon_local(pts, origin, yaw)


def detailed_straight_markings(origin, yaw):
    """Return white marking geometry for one 48 m straight-road module."""
    white_parts = []

    # Zebra crosswalk at the intersection-side end.
    v = -ROAD_HALF_M
    while v < ROAD_HALF_M - 1e-6:
        v0 = v
        v1 = min(v + CROSSWALK_STRIPE_W_M, ROAD_HALF_M)
        white_parts.append(rect_local(CROSSWALK_U0_M, CROSSWALK_U1_M, v0, v1, origin, yaw))
        v += CROSSWALK_STRIPE_W_M + CROSSWALK_GAP_M

    # Thin stop line before crosswalk, split by lane but still across most of the road.
    white_parts.append(rect_local(STOP_LINE_U_M, STOP_LINE_U_M + STOP_LINE_WIDTH_U_M,
                                  -ROAD_HALF_M + 0.25, ROAD_HALF_M - 0.25, origin, yaw))

    # Hollow diamond warnings: two in each lane, same as the reference straight road.
    lane_centers = [-LANE_WIDTH_M / 2.0, LANE_WIDTH_M / 2.0]
    for u in DIAMOND_U_POSITIONS_M:
        for lc in lane_centers:
            white_parts.append(diamond_outline(u, lc, origin, yaw))

    # Direction arrows: right-hand lane points to the intersection, opposite lane points outward.
    # local v=-1.75 is the right lane when travelling toward the intersection.
    white_parts.append(arrow_polygon(ARROW_CENTER_U_M, -LANE_WIDTH_M / 2.0, +1, origin, yaw))
    white_parts.append(arrow_polygon(ARROW_CENTER_U_M, +LANE_WIDTH_M / 2.0, -1, origin, yaw))

    return unary_union(white_parts).buffer(0)


def double_yellow_for_line(line: LineString):
    geoms = []
    # Shapely can return MultiLineString from offset. Buffer handles both.
    for side in [YELLOW_DOUBLE_OFFSET_M, -YELLOW_DOUBLE_OFFSET_M]:
        try:
            off = line.parallel_offset(abs(side), "left" if side > 0 else "right", join_style=2, resolution=16)
        except Exception:
            continue
        geoms.append(off.buffer(YELLOW_LINE_WIDTH_M / 2.0, cap_style=2, join_style=2, resolution=8))
    return unary_union(geoms).buffer(0)


def make_geometries():
    loop_pts = rounded_square_centerline(SIDE_LENGTH_M, CORNER_RADIUS_M)
    loop_line = LineString(loop_pts + [loop_pts[0]])
    loop_road = loop_line.buffer(ROAD_HALF_M, cap_style=2, join_style=2, resolution=16)

    vertical_center = LineString([(0.0, -HALF_SIDE_M), (0.0, HALF_SIDE_M)])
    horizontal_center = LineString([(-HALF_SIDE_M, 0.0), (HALF_SIDE_M, 0.0)])
    vertical_road = vertical_center.buffer(ROAD_HALF_M, cap_style=2, join_style=2, resolution=16)
    horizontal_road = horizontal_center.buffer(ROAD_HALF_M, cap_style=2, join_style=2, resolution=16)
    central_pad = box(-CENTRAL_INTERSECTION_PAD_M/2, -CENTRAL_INTERSECTION_PAD_M/2,
                      CENTRAL_INTERSECTION_PAD_M/2, CENTRAL_INTERSECTION_PAD_M/2)
    asphalt = unary_union([loop_road, vertical_road, horizontal_road, central_pad]).buffer(0)

    # Double yellow centerlines on the loop and both cross connectors.
    yellow_parts = [
        double_yellow_for_line(loop_line),
        double_yellow_for_line(vertical_center),
        double_yellow_for_line(horizontal_center),
    ]
    yellow = unary_union(yellow_parts).intersection(asphalt).buffer(0)

    # White edge lines trace the final asphalt boundary.
    white_edge_lines = asphalt.boundary.buffer(EDGE_LINE_WIDTH_M/2, cap_style=2, join_style=2, resolution=12)
    white_edge_lines = white_edge_lines.intersection(asphalt).buffer(0)

    # Four detailed straight modules around the central intersection.
    # Reference straight road is 48 m from start to crosswalk/intersection end.
    # The outer-loop centerline is at +/-50 m and the central pad edge is at +/-7 m,
    # so we place the local marking origin 5 m outside the loop centerline:
    #   local u=47~48 -> crosswalk at +/-8~+/-7 m, right before the central intersection pad.
    marking_origin_offset_m = 5.0
    approaches = [
        {"id": "south_to_center", "origin": (0.0, -HALF_SIDE_M - marking_origin_offset_m), "yaw": math.pi/2},
        {"id": "north_to_center", "origin": (0.0, HALF_SIDE_M + marking_origin_offset_m), "yaw": -math.pi/2},
        {"id": "east_to_center",  "origin": (HALF_SIDE_M + marking_origin_offset_m, 0.0), "yaw": math.pi},
        {"id": "west_to_center",  "origin": (-HALF_SIDE_M - marking_origin_offset_m, 0.0), "yaw": 0.0},
    ]
    white_detail_parts = [detailed_straight_markings(a["origin"], a["yaw"]) for a in approaches]
    white_details = unary_union(white_detail_parts).intersection(asphalt).buffer(0)

    return asphalt, yellow, white_edge_lines, white_details, loop_pts, approaches


def tri_normal(a, b, c):
    ux, uy, uz = b[0]-a[0], b[1]-a[1], b[2]-a[2]
    vx, vy, vz = c[0]-a[0], c[1]-a[1], c[2]-a[2]
    nx, ny, nz = uy*vz-uz*vy, uz*vx-ux*vz, ux*vy-uy*vx
    l = math.sqrt(nx*nx+ny*ny+nz*nz)
    if l < 1e-12:
        return (0, 0, 1)
    return (nx/l, ny/l, nz/l)


def add_tri(tris, a, b, c, want_up=True):
    n = tri_normal(a,b,c)
    if want_up and n[2] < 0:
        b, c = c, b
        n = tri_normal(a,b,c)
    if (not want_up) and n[2] > 0:
        b, c = c, b
        n = tri_normal(a,b,c)
    tris.append((n,a,b,c))


def geom_to_top_tris(geom, z):
    tris = []
    for poly in iter_polygons(geom):
        if poly.area < 1e-9:
            continue
        for t in triangulate(poly):
            if t.area < 1e-10:
                continue
            if not poly.covers(t):
                continue
            coords = list(t.exterior.coords)[:3]
            a,b,c = (coords[0][0],coords[0][1],z), (coords[1][0],coords[1][1],z), (coords[2][0],coords[2][1],z)
            add_tri(tris, a,b,c, want_up=True)
    return tris


def add_side_faces(tris, coords, z_top, z_bottom):
    coords = list(coords)
    if coords[0] != coords[-1]:
        coords.append(coords[0])
    for p0, p1 in zip(coords[:-1], coords[1:]):
        if abs(p1[0]-p0[0]) + abs(p1[1]-p0[1]) < 1e-9:
            continue
        a = (p0[0], p0[1], z_top)
        b = (p1[0], p1[1], z_top)
        c = (p1[0], p1[1], z_bottom)
        d = (p0[0], p0[1], z_bottom)
        tris.append((tri_normal(a,b,c), a,b,c))
        tris.append((tri_normal(a,c,d), a,c,d))


def geom_to_extruded_tris(geom, z_top, z_bottom):
    tris = []
    for poly in iter_polygons(geom):
        if poly.area < 1e-9:
            continue
        top = geom_to_top_tris(poly, z_top)
        tris.extend(top)
        for _,a,b,c in top:
            add_tri(tris, (a[0],a[1],z_bottom), (b[0],b[1],z_bottom), (c[0],c[1],z_bottom), want_up=False)
        add_side_faces(tris, poly.exterior.coords, z_top, z_bottom)
        for interior in poly.interiors:
            add_side_faces(tris, interior.coords, z_top, z_bottom)
    return tris


def write_stl(path, tris, name):
    with open(path, "w") as f:
        f.write(f"solid {name}\n")
        for n,a,b,c in tris:
            f.write(f"  facet normal {n[0]:.9g} {n[1]:.9g} {n[2]:.9g}\n")
            f.write("    outer loop\n")
            for v in (a,b,c):
                f.write(f"      vertex {v[0]:.9g} {v[1]:.9g} {v[2]:.9g}\n")
            f.write("    endloop\n")
            f.write("  endfacet\n")
        f.write(f"endsolid {name}\n")


def write_world():
    sdf = f'''<?xml version="1.0" ?>
<sdf version="1.9">
  <world name="{WORLD_NAME}">
    <gravity>0 0 -9.80665</gravity>

    <spherical_coordinates>
      <surface_model>EARTH_WGS84</surface_model>
      <world_frame_orientation>ENU</world_frame_orientation>
      <latitude_deg>{ORIGIN_LAT_DEG}</latitude_deg>
      <longitude_deg>{ORIGIN_LON_DEG}</longitude_deg>
      <elevation>{ORIGIN_ELEV_M}</elevation>
      <heading_deg>0</heading_deg>
    </spherical_coordinates>

    <physics name="default_physics" type="dart">
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1.0</real_time_factor>
      <real_time_update_rate>1000</real_time_update_rate>
    </physics>

    <scene>
      <ambient>0.45 0.45 0.45 1</ambient>
      <background>0.75 0.78 0.80 1</background>
      <shadows>true</shadows>
    </scene>

    <light name="sun" type="directional">
      <pose>0 0 80 0 0 0</pose>
      <cast_shadows>true</cast_shadows>
      <direction>-0.45 0.20 -0.87</direction>
      <diffuse>0.85 0.85 0.85 1</diffuse>
      <specular>0.2 0.2 0.2 1</specular>
    </light>

    <model name="asphalt_road_collision">
      <static>true</static>
      <pose>0 0 0 0 0 0</pose>
      <link name="road_link">
        <collision name="road_collision">
          <geometry><mesh><uri>model://{PKG_NAME}/meshes/road_asphalt.stl</uri><scale>1 1 1</scale></mesh></geometry>
          <surface>
            <friction><ode><mu>1.0</mu><mu2>1.0</mu2></ode></friction>
            <contact><ode><kp>10000000</kp><kd>1</kd></ode></contact>
          </surface>
        </collision>
        <visual name="road_visual">
          <geometry><mesh><uri>model://{PKG_NAME}/meshes/road_asphalt.stl</uri><scale>1 1 1</scale></mesh></geometry>
          <material>
            <ambient>0.050 0.050 0.050 1</ambient>
            <diffuse>0.070 0.070 0.070 1</diffuse>
            <specular>0.015 0.015 0.015 1</specular>
          </material>
        </visual>
      </link>
    </model>

    <model name="yellow_double_centerlines_visual_only">
      <static>true</static>
      <pose>0 0 0 0 0 0</pose>
      <link name="yellow_centerlines_link">
        <visual name="yellow_centerlines_visual">
          <geometry><mesh><uri>model://{PKG_NAME}/meshes/yellow_centerlines.stl</uri><scale>1 1 1</scale></mesh></geometry>
          <material>
            <ambient>1.0 0.76 0.0 1</ambient>
            <diffuse>1.0 0.76 0.0 1</diffuse>
            <specular>0.05 0.05 0.02 1</specular>
          </material>
        </visual>
      </link>
    </model>

    <model name="white_edge_lines_visual_only">
      <static>true</static>
      <pose>0 0 0 0 0 0</pose>
      <link name="white_edge_lines_link">
        <visual name="white_edge_lines_visual">
          <geometry><mesh><uri>model://{PKG_NAME}/meshes/white_edge_lines.stl</uri><scale>1 1 1</scale></mesh></geometry>
          <material>
            <ambient>1 1 1 1</ambient>
            <diffuse>1 1 1 1</diffuse>
            <specular>0.08 0.08 0.08 1</specular>
          </material>
        </visual>
      </link>
    </model>

    <model name="white_detailed_straight_markings_visual_only">
      <static>true</static>
      <pose>0 0 0 0 0 0</pose>
      <link name="white_details_link">
        <visual name="white_details_visual">
          <geometry><mesh><uri>model://{PKG_NAME}/meshes/white_straight_details.stl</uri><scale>1 1 1</scale></mesh></geometry>
          <material>
            <ambient>1 1 1 1</ambient>
            <diffuse>1 1 1 1</diffuse>
            <specular>0.08 0.08 0.08 1</specular>
          </material>
        </visual>
      </link>
    </model>
  </world>
</sdf>
'''
    (WORLD_DIR / "square_intersection_loop.world.sdf").write_text(sdf)


def write_configs(approaches):
    geometry = {
        "map_name": WORLD_NAME,
        "frame_id": "map",
        "coordinate_convention": {"x": "east_m", "y": "north_m", "z": "up_m"},
        "outer_loop": {
            "measurement_basis": "centerline overall bounding square",
            "side_length_m": SIDE_LENGTH_M,
            "centerline_extents": {"x_min_m": -HALF_SIDE_M, "x_max_m": HALF_SIDE_M, "y_min_m": -HALF_SIDE_M, "y_max_m": HALF_SIDE_M},
            "corner_radius_centerline_m": CORNER_RADIUS_M,
            "road_width_m": ROAD_WIDTH_M,
            "lane_width_m": LANE_WIDTH_M,
        },
        "central_intersection": {
            "center_x_m": 0.0,
            "center_y_m": 0.0,
            "cross_road_width_m": ROAD_WIDTH_M,
            "pad_size_m": CENTRAL_INTERSECTION_PAD_M,
        },
        "detailed_straight_sections": {
            "count": 4,
            "local_length_m": DETAILED_SEGMENT_LENGTH_M,
            "road_width_m": ROAD_WIDTH_M,
            "lane_width_m": LANE_WIDTH_M,
            "crosswalk_range_u_m": [CROSSWALK_U0_M, CROSSWALK_U1_M],
            "diamond_u_positions_m": DIAMOND_U_POSITIONS_M,
            "arrow_center_u_m": ARROW_CENTER_U_M,
            "collision": False,
            "approaches": approaches,
        },
        "road_surface": {
            "asphalt_top_z_m": ASPHALT_TOP_Z,
            "asphalt_bottom_z_m": ASPHALT_BOTTOM_Z,
            "collision": True,
            "friction_mu": 1.0,
        },
        "markings": {
            "yellow_double_centerline": True,
            "yellow_line_width_m": YELLOW_LINE_WIDTH_M,
            "yellow_double_offset_m": YELLOW_DOUBLE_OFFSET_M,
            "edge_line_width_m": EDGE_LINE_WIDTH_M,
            "yellow_z_m": YELLOW_Z,
            "white_z_m": WHITE_Z,
            "collision": False,
        },
    }
    origin = {
        "map_name": WORLD_NAME,
        "coordinate_system": {
            "type": "ENU_WGS84_LOCAL_TANGENT",
            "origin": {"latitude_deg": ORIGIN_LAT_DEG, "longitude_deg": ORIGIN_LON_DEG, "elevation_m": ORIGIN_ELEV_M},
            "x_axis": "east_m",
            "y_axis": "north_m",
            "z_axis": "up_m",
        },
    }
    spawn_points = {
        "map_name": WORLD_NAME,
        "frame_id": "map",
        "spawn_points": [
            {"id": "south_start", "description": "south connector road, facing north", "x": 0.0, "y": -50.0, "z": 0.30, "yaw": 1.5708},
            {"id": "north_start", "description": "north connector road, facing south", "x": 0.0, "y": 50.0, "z": 0.30, "yaw": -1.5708},
            {"id": "east_start", "description": "east connector road, facing west", "x": 50.0, "y": 0.0, "z": 0.30, "yaw": 3.14159},
            {"id": "west_start", "description": "west connector road, facing east", "x": -50.0, "y": 0.0, "z": 0.30, "yaw": 0.0},
        ],
    }
    for p, data in [(CONFIG_DIR/"map_geometry.yaml", geometry), (CONFIG_DIR/"map_origin.yaml", origin), (CONFIG_DIR/"spawn_points.yaml", spawn_points)]:
        with open(p, "w") as f:
            yaml.safe_dump(data, f, sort_keys=False, allow_unicode=True)


def write_docs():
    (OUT_ROOT / "model.config").write_text(f'''<?xml version="1.0"?>
<model>
  <name>{PKG_NAME}</name>
  <version>2.0</version>
  <sdf version="1.9">worlds/square_intersection_loop.world.sdf</sdf>
  <description>100 m square outer loop with central cross intersection and detailed straight sections.</description>
</model>
''')
    (OUT_ROOT / "README.md").write_text(f'''# {PKG_NAME}

Gazebo / `gz sim`용 중앙 십자 교차로 + 정사각형 외곽 순환도로 맵입니다.

이번 버전은 중앙 교차로 주변 4개의 직선 구간에 참고 이미지와 같은 상세 노면 마킹을 넣었습니다.

## 치수

- 외곽 순환도로 중심선 기준 한 변: {SIDE_LENGTH_M:.1f} m
- 중심선 좌표 범위: x/y = -50 m ~ +50 m
- 도로 폭: {ROAD_WIDTH_M:.1f} m
- 한 차선 폭: {LANE_WIDTH_M:.1f} m
- 외곽 순환도로 모서리 중심선 반지름: {CORNER_RADIUS_M:.1f} m
- 중앙 교차로 중심: x=0, y=0
- 상세 직선 구간 길이 기준: {DETAILED_SEGMENT_LENGTH_M:.1f} m

## 중앙 교차로 주변 4개 직선구간 마킹

각 방향 직선구간에 동일하게 적용했습니다.

- 교차로 쪽 끝 횡단보도
- 흰색 마름모 예고 마킹 4개
- 차선별 진행방향 화살표
- 이중 노란 중앙 실선
- 양쪽 흰색 가장자리 실선

## 충돌 설정

- 아스팔트: collision 있음
- 노란 중앙선 / 흰 실선 / 횡단보도 / 마름모 / 화살표 / 정지선: visual-only, collision 없음
- 노면표시는 z-fighting 방지를 위해 아스팔트보다 0.5~1.0 mm 위에 있습니다.
- 노면표시에는 collision이 없으므로 차량이 선이나 마킹을 밟아도 덜컹거리지 않습니다.

## 설치

```bash
mkdir -p ~/Competitions/2026/03_Kookmin_Autonomous_9th/gazebo_maps
cd ~/Competitions/2026/03_Kookmin_Autonomous_9th/gazebo_maps
unzip ~/Downloads/{PKG_NAME}.zip -d ~/Competitions/2026/03_Kookmin_Autonomous_9th/gazebo_maps
```

기존 폴더가 있으면 먼저 삭제하세요.

```bash
rm -rf ~/Competitions/2026/03_Kookmin_Autonomous_9th/gazebo_maps/{PKG_NAME}
unzip ~/Downloads/{PKG_NAME}.zip -d ~/Competitions/2026/03_Kookmin_Autonomous_9th/gazebo_maps
```

## 실행

```bash
export GZ_SIM_RESOURCE_PATH=$HOME/gazebo_maps:$GZ_SIM_RESOURCE_PATH
gz sim -r ~/Competitions/2026/03_Kookmin_Autonomous_9th/gazebo_maps/{PKG_NAME}/worlds/square_intersection_loop.world.sdf
```

## 차량 spawn 예시

```bash
ros2 run ros_gz_sim create \\
  -world square_intersection_loop \\
  -file /path/to/your_vehicle.sdf \\
  -name ego_vehicle \\
  -x 0.0 -y -50.0 -z 0.30 -Y 1.5708
```

## 재생성

```bash
cd ~/Competitions/2026/03_Kookmin_Autonomous_9th/gazebo_maps/{PKG_NAME}
python3 scripts/generate_square_intersection_map.py
```
''')
    (OUT_ROOT / "MEASUREMENTS.md").write_text(f'''# MEASUREMENTS

## Coordinate frame

- x: East [m]
- y: North [m]
- z: Up [m]
- Origin: central intersection center

## Outer loop

- Centerline bounding square side length: {SIDE_LENGTH_M:.3f} m
- Centerline extents: x = {-HALF_SIDE_M:.3f} ~ {HALF_SIDE_M:.3f}, y = {-HALF_SIDE_M:.3f} ~ {HALF_SIDE_M:.3f}
- Corner radius along centerline: {CORNER_RADIUS_M:.3f} m
- Road width: {ROAD_WIDTH_M:.3f} m
- Lane width: {LANE_WIDTH_M:.3f} m

## Detailed straight sections around central intersection

The four connector roads use the same local marking layout as the reference straight road.

- Local length used for markings: {DETAILED_SEGMENT_LENGTH_M:.3f} m
- Crosswalk local range: u={CROSSWALK_U0_M:.3f}~{CROSSWALK_U1_M:.3f} m
- Crosswalk stripe width: {CROSSWALK_STRIPE_W_M:.3f} m
- Crosswalk stripe gap: {CROSSWALK_GAP_M:.3f} m
- Diamond u positions: {DIAMOND_U_POSITIONS_M}
- Arrow center u: {ARROW_CENTER_U_M:.3f} m
- Double yellow line: two strips, each {YELLOW_LINE_WIDTH_M:.3f} m wide, offset +/- {YELLOW_DOUBLE_OFFSET_M:.3f} m from centerline

## Heights

- Asphalt top z: {ASPHALT_TOP_Z:.4f} m
- Asphalt bottom z: {ASPHALT_BOTTOM_Z:.4f} m
- Yellow marking visual z: {YELLOW_Z:.4f} m
- White marking visual z: {WHITE_Z:.4f} m
- Marking collision: false

## Recommended spawn

- x=0.0
- y=-50.0
- z=0.30
- yaw=1.5708
''')


def plot_geom(ax, geom, facecolor, edgecolor="none", alpha=1.0, linewidth=0.0):
    for _, a, b, c in geom_to_top_tris(geom, 0.0):
        ax.add_patch(MplPolygon([(a[0],a[1]),(b[0],b[1]),(c[0],c[1])], closed=True, facecolor=facecolor, edgecolor=edgecolor, alpha=alpha, linewidth=linewidth))


def make_preview(asphalt, yellow, edges, details):
    fig, ax = plt.subplots(figsize=(10,10), dpi=180)
    plot_geom(ax, asphalt, "#1c1c1c")
    plot_geom(ax, yellow, "#ffd000")
    plot_geom(ax, edges, "#ffffff")
    plot_geom(ax, details, "#ffffff")
    ax.scatter([0],[0], s=20, color="red")
    ax.text(2,2,"central\nintersection", color="red", fontsize=8)
    ax.scatter([0],[-50],s=25,color="#00aaff")
    ax.annotate("", xy=(0,-43), xytext=(0,-49), arrowprops=dict(arrowstyle="->", color="#00aaff", lw=1.5))
    ax.set_aspect("equal")
    ax.set_xlim(-62,62)
    ax.set_ylim(-62,62)
    ax.grid(True, linewidth=0.25, alpha=0.35)
    ax.set_xlabel("x East [m]")
    ax.set_ylabel("y North [m]")
    ax.set_title("Square loop + central intersection + detailed straight sections")
    fig.tight_layout()
    fig.savefig(PREVIEW_DIR / "preview.png")
    fig.savefig(PREVIEW_DIR / "square_intersection_loop_preview.png")
    plt.close(fig)


def copy_script_to_package():
    this_script = Path(__file__).resolve()
    dst = OUT_ROOT / "scripts"
    dst.mkdir(exist_ok=True)
    shutil.copy2(this_script, dst / "generate_square_intersection_map.py")


def zip_package():
    zip_path = Path("/mnt/data/square_intersection_gazebo_map_detailed_straights.zip")
    if zip_path.exists():
        zip_path.unlink()
    base = OUT_ROOT.parent
    with zipfile.ZipFile(zip_path, "w", zipfile.ZIP_DEFLATED) as z:
        for p in OUT_ROOT.rglob("*"):
            if p.is_file():
                z.write(p, p.relative_to(base))
    return zip_path


def main():
    prepare_dirs()
    asphalt, yellow, edges, details, _, approaches = make_geometries()
    road_tris = geom_to_extruded_tris(asphalt, ASPHALT_TOP_Z, ASPHALT_BOTTOM_Z)
    yellow_tris = geom_to_top_tris(yellow, YELLOW_Z)
    edge_tris = geom_to_top_tris(edges, WHITE_Z)
    detail_tris = geom_to_top_tris(details, WHITE_Z)
    write_stl(MESH_DIR / "road_asphalt.stl", road_tris, "road_asphalt")
    write_stl(MESH_DIR / "yellow_centerlines.stl", yellow_tris, "yellow_centerlines")
    write_stl(MESH_DIR / "white_edge_lines.stl", edge_tris, "white_edge_lines")
    write_stl(MESH_DIR / "white_straight_details.stl", detail_tris, "white_straight_details")
    write_world()
    write_configs(approaches)
    write_docs()
    copy_script_to_package()
    make_preview(asphalt, yellow, edges, details)
    zp = zip_package()
    print(f"Generated {OUT_ROOT}")
    print(f"Zip: {zp}")
    print(f"Triangles: road={len(road_tris)}, yellow={len(yellow_tris)}, edges={len(edge_tris)}, details={len(detail_tris)}")

if __name__ == "__main__":
    main()
