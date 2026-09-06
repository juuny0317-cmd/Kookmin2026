#!/usr/bin/env python3
"""Convert the Kookmin University R10 DXF track into a Gazebo SDF world.

The source drawing contains only closed POLYLINE entities.  DXF R10 does not
store drawing units, so the default conversion follows the source filename:
one drawing unit is interpreted as one millimetre.  No fit-to-size scaling is
performed.
"""

from __future__ import annotations

import argparse
import ctypes
import ctypes.util
import math
from dataclasses import dataclass
from html import escape
from pathlib import Path


DEFAULT_METERS_PER_UNIT = 0.001
DEFAULT_ARC_STEP_DEG = 1.0
ASPHALT_HEIGHT = 0.002
OVERLAY_HEIGHT = 0.00025
GROUND_MARGIN_M = 4.0
DASH_LENGTH_M = 0.20
DASH_WIDTH_M = 0.05

# Requested rendered road cross-section.  The DXF's main straight road averages
# 0.713 m between its two concrete edges. Expanding both edges equally keeps the
# original centreline fixed and makes the nominal grey road 0.800 m. Each white
# edge line is then placed wholly outside the grey road, making the requested
# white-to-white width 0.900 m.
SOURCE_NOMINAL_ROAD_WIDTH_M = 0.713
GRAY_ROAD_WIDTH_M = 0.800
WHITE_EDGE_WIDTH_M = 0.050
GRAY_EDGE_EXPANSION_M = (GRAY_ROAD_WIDTH_M - SOURCE_NOMINAL_ROAD_WIDTH_M) * 0.5

# Ordering of the source dash polygons along each centreline.  Gazebo can
# silently skip some of the curved, highly tessellated dash polygons.  Their
# centroids and path tangents are therefore retained while their render
# geometry is normalised to a small box that Gazebo renders reliably.
OUTER_DASH_PATH = (
    tuple(range(3, 31))
    + (32, 37, 40, 43, 45, 48, 51, 54, 57, 58, 61, 64, 67, 70, 73, 76, 77, 80, 83, 86, 89, 92, 100)
    + tuple(reversed((94, 95, 96, 97, 98, 99, 101, 102, 103, 104, 105, 93, 106, 107, 108, 109,
                      110, 111, 112, 113, 114, 115, 116, 117, 118, 119, 120)))
    + (91, 90, 87, 84, 81, 78, 74, 71, 68, 65, 62, 59, 55, 52, 49, 46, 42, 39, 38, 33, 31)
)
MIDDLE_DASH_PATH = (36, 41, 44, 47, 50, 53, 56, 60, 63, 66, 69, 72, 75, 79, 82, 85, 88)


@dataclass(frozen=True)
class Vertex:
    x: float
    y: float
    bulge: float = 0.0


@dataclass(frozen=True)
class Polyline:
    vertices: tuple[Vertex, ...]
    closed: bool


Point = tuple[float, float]


def _first(records: list[tuple[str, str]], code: str, default: str | None = None) -> str:
    for record_code, value in records:
        if record_code == code:
            return value
    if default is not None:
        return default
    raise ValueError(f"DXF record is missing required group code {code}")


def parse_polylines(path: Path) -> list[Polyline]:
    """Read closed 2D POLYLINE entities from an ASCII R10 DXF."""
    lines = path.read_text(encoding="utf-8-sig").splitlines()
    if len(lines) % 2:
        raise ValueError("DXF has an incomplete group-code/value pair")
    pairs = [(lines[index].strip(), lines[index + 1].strip()) for index in range(0, len(lines), 2)]

    polylines: list[Polyline] = []
    index = 0
    while index < len(pairs):
        if pairs[index] != ("0", "POLYLINE"):
            index += 1
            continue

        index += 1
        header: list[tuple[str, str]] = []
        while index < len(pairs) and pairs[index][0] != "0":
            header.append(pairs[index])
            index += 1

        vertices: list[Vertex] = []
        while index < len(pairs) and pairs[index] == ("0", "VERTEX"):
            index += 1
            records: list[tuple[str, str]] = []
            while index < len(pairs) and pairs[index][0] != "0":
                records.append(pairs[index])
                index += 1
            vertices.append(
                Vertex(
                    x=float(_first(records, "10")),
                    y=float(_first(records, "20")),
                    bulge=float(_first(records, "42", "0")),
                )
            )

        if index < len(pairs) and pairs[index] == ("0", "SEQEND"):
            index += 1

        flags = int(_first(header, "70", "0"))
        closed = bool(flags & 1)
        if not closed:
            raise ValueError(f"POLYLINE {len(polylines)} is open; this map requires closed outlines")
        if len(vertices) < 3:
            raise ValueError(f"POLYLINE {len(polylines)} has fewer than three vertices")
        polylines.append(Polyline(tuple(vertices), closed=True))

    if len(polylines) != 125:
        raise ValueError(f"Expected 125 closed polylines in the supplied map, found {len(polylines)}")
    return polylines


def _arc_segment(start: Vertex, end: Vertex, max_step_rad: float) -> list[Point]:
    """Return points after *start* through *end* for one DXF bulge segment."""
    dx = end.x - start.x
    dy = end.y - start.y
    chord = math.hypot(dx, dy)
    theta = 4.0 * math.atan(start.bulge)
    if chord == 0.0:
        return []
    if abs(theta) < 1e-12:
        return [(end.x, end.y)]

    midpoint_x = (start.x + end.x) * 0.5
    midpoint_y = (start.y + end.y) * 0.5
    center_offset = chord / (2.0 * math.tan(theta * 0.5))
    center_x = midpoint_x - dy / chord * center_offset
    center_y = midpoint_y + dx / chord * center_offset
    radius = math.hypot(start.x - center_x, start.y - center_y)
    start_angle = math.atan2(start.y - center_y, start.x - center_x)

    steps = max(1, math.ceil(abs(theta) / max_step_rad))
    fractions = {step / steps for step in range(1, steps + 1)}

    # Include axis extrema exactly.  Besides giving exact bounds, this avoids
    # clipping the outside of a curve when the world is centred and sized.
    for cardinal in (0.0, math.pi * 0.5, math.pi, math.pi * 1.5):
        for revolution in range(-2, 3):
            fraction = (cardinal + revolution * math.tau - start_angle) / theta
            if 0.0 < fraction < 1.0:
                fractions.add(fraction)

    points = [
        (
            center_x + radius * math.cos(start_angle + theta * fraction),
            center_y + radius * math.sin(start_angle + theta * fraction),
        )
        for fraction in sorted(fractions)
    ]
    points[-1] = (end.x, end.y)
    return points


def tessellate(polyline: Polyline, max_arc_step_deg: float) -> list[Point]:
    """Convert DXF bulge arcs to a Gazebo-compatible point polygon."""
    if not 0.05 <= max_arc_step_deg <= 45.0:
        raise ValueError("arc step must be between 0.05 and 45 degrees")
    max_step_rad = math.radians(max_arc_step_deg)
    vertices = polyline.vertices
    points: list[Point] = [(vertices[0].x, vertices[0].y)]
    for index, start in enumerate(vertices):
        end = vertices[(index + 1) % len(vertices)]
        points.extend(_arc_segment(start, end, max_step_rad))

    # The SDF polyline closes itself; do not duplicate its first point.
    if points and math.dist(points[0], points[-1]) < 1e-8:
        points.pop()
    return points


def bounds(points: list[Point]) -> tuple[float, float, float, float]:
    xs, ys = zip(*points)
    return min(xs), min(ys), max(xs), max(ys)


def offset_polygon(points: list[Point], distance: float) -> list[Point]:
    """Return a topology-safe GEOS offset of a simple source polygon.

    Positive distance expands and negative distance shrinks. GEOS resolves the
    self-intersections that a hand-built parallel-edge offset can create around
    the tight concave turns on the right side of this track. ``libgeos_c`` is a
    standard Ubuntu Jammy system library and is loaded through Python's ctypes,
    so no additional Python package is required.
    """
    if len(points) < 3:
        raise ValueError("cannot offset a polygon with fewer than three points")
    if abs(distance) < 1e-12:
        return list(points)

    library_name = ctypes.util.find_library("geos_c")
    if library_name is None:
        raise RuntimeError(
            "libgeos_c is required to generate non-self-intersecting road offsets")
    geos = ctypes.CDLL(library_name)
    pointer = ctypes.c_void_p

    geos.GEOS_init_r.argtypes = []
    geos.GEOS_init_r.restype = pointer
    geos.GEOS_finish_r.argtypes = [pointer]
    geos.GEOS_finish_r.restype = None
    geos.GEOSCoordSeq_create_r.argtypes = [pointer, ctypes.c_uint, ctypes.c_uint]
    geos.GEOSCoordSeq_create_r.restype = pointer
    geos.GEOSCoordSeq_setX_r.argtypes = [pointer, pointer, ctypes.c_uint, ctypes.c_double]
    geos.GEOSCoordSeq_setX_r.restype = ctypes.c_int
    geos.GEOSCoordSeq_setY_r.argtypes = [pointer, pointer, ctypes.c_uint, ctypes.c_double]
    geos.GEOSCoordSeq_setY_r.restype = ctypes.c_int
    geos.GEOSGeom_createLinearRing_r.argtypes = [pointer, pointer]
    geos.GEOSGeom_createLinearRing_r.restype = pointer
    geos.GEOSGeom_createPolygon_r.argtypes = [pointer, pointer, pointer, ctypes.c_uint]
    geos.GEOSGeom_createPolygon_r.restype = pointer
    geos.GEOSBufferWithStyle_r.argtypes = [
        pointer,
        pointer,
        ctypes.c_double,
        ctypes.c_int,
        ctypes.c_int,
        ctypes.c_int,
        ctypes.c_double,
    ]
    geos.GEOSBufferWithStyle_r.restype = pointer
    geos.GEOSGeomTypeId_r.argtypes = [pointer, pointer]
    geos.GEOSGeomTypeId_r.restype = ctypes.c_int
    geos.GEOSGetExteriorRing_r.argtypes = [pointer, pointer]
    geos.GEOSGetExteriorRing_r.restype = pointer
    geos.GEOSGeom_getCoordSeq_r.argtypes = [pointer, pointer]
    geos.GEOSGeom_getCoordSeq_r.restype = pointer
    geos.GEOSCoordSeq_getSize_r.argtypes = [pointer, pointer, ctypes.POINTER(ctypes.c_uint)]
    geos.GEOSCoordSeq_getSize_r.restype = ctypes.c_int
    geos.GEOSCoordSeq_getX_r.argtypes = [
        pointer, pointer, ctypes.c_uint, ctypes.POINTER(ctypes.c_double)]
    geos.GEOSCoordSeq_getX_r.restype = ctypes.c_int
    geos.GEOSCoordSeq_getY_r.argtypes = [
        pointer, pointer, ctypes.c_uint, ctypes.POINTER(ctypes.c_double)]
    geos.GEOSCoordSeq_getY_r.restype = ctypes.c_int
    geos.GEOSGeom_destroy_r.argtypes = [pointer, pointer]
    geos.GEOSGeom_destroy_r.restype = None

    context = geos.GEOS_init_r()
    source_polygon = None
    buffered_polygon = None
    try:
        # A GEOS linear ring repeats its first coordinate at the end.
        sequence = geos.GEOSCoordSeq_create_r(context, len(points) + 1, 2)
        if not sequence:
            raise RuntimeError("GEOS failed to allocate a road coordinate sequence")
        for index, (x, y) in enumerate(points + points[:1]):
            if not geos.GEOSCoordSeq_setX_r(context, sequence, index, x):
                raise RuntimeError("GEOS failed to set a road X coordinate")
            if not geos.GEOSCoordSeq_setY_r(context, sequence, index, y):
                raise RuntimeError("GEOS failed to set a road Y coordinate")
        ring = geos.GEOSGeom_createLinearRing_r(context, sequence)
        if not ring:
            raise RuntimeError("GEOS rejected a source road ring")
        # createPolygon takes ownership of the ring and its coordinate sequence.
        source_polygon = geos.GEOSGeom_createPolygon_r(context, ring, None, 0)
        if not source_polygon:
            raise RuntimeError("GEOS rejected a source road polygon")
        # Closed polygons do not use the cap style. Miter joins preserve the
        # densely tessellated DXF curves without adding thousands of tiny round
        # joins; GEOS still resolves concave self-intersections topologically.
        buffered_polygon = geos.GEOSBufferWithStyle_r(
            context, source_polygon, distance, 8, 1, 2, 4.0)
        if not buffered_polygon:
            raise RuntimeError(f"GEOS road offset failed for distance {distance}")
        # GEOS_POLYGON is type id 3. Splitting would mean the requested width is
        # too large for the supplied track geometry and must not be hidden.
        if geos.GEOSGeomTypeId_r(context, buffered_polygon) != 3:
            raise ValueError(
                f"road offset {distance} produced more than one polygon")

        exterior = geos.GEOSGetExteriorRing_r(context, buffered_polygon)
        sequence = geos.GEOSGeom_getCoordSeq_r(context, exterior)
        size = ctypes.c_uint()
        if not geos.GEOSCoordSeq_getSize_r(context, sequence, ctypes.byref(size)):
            raise RuntimeError("GEOS failed to report road offset coordinate count")
        result: list[Point] = []
        # Drop the repeated final coordinate required by the GEOS linear ring.
        for index in range(size.value - 1):
            x_value = ctypes.c_double()
            y_value = ctypes.c_double()
            if not geos.GEOSCoordSeq_getX_r(
                    context, sequence, index, ctypes.byref(x_value)):
                raise RuntimeError("GEOS failed to read a road X coordinate")
            if not geos.GEOSCoordSeq_getY_r(
                    context, sequence, index, ctypes.byref(y_value)):
                raise RuntimeError("GEOS failed to read a road Y coordinate")
            result.append((x_value.value, y_value.value))
        return result
    finally:
        if buffered_polygon:
            geos.GEOSGeom_destroy_r(context, buffered_polygon)
        if source_polygon:
            geos.GEOSGeom_destroy_r(context, source_polygon)
        geos.GEOS_finish_r(context)


def polygon_centroid(points: list[Point]) -> Point:
    """Return the area centroid of a simple closed polygon."""
    twice_area = 0.0
    weighted_x = 0.0
    weighted_y = 0.0
    for index, (x0, y0) in enumerate(points):
        x1, y1 = points[(index + 1) % len(points)]
        cross = x0 * y1 - x1 * y0
        twice_area += cross
        weighted_x += (x0 + x1) * cross
        weighted_y += (y0 + y1) * cross
    if abs(twice_area) < 1e-12:
        return (
            sum(x for x, _ in points) / len(points),
            sum(y for _, y in points) / len(points),
        )
    return weighted_x / (3.0 * twice_area), weighted_y / (3.0 * twice_area)


def material(rgba: str) -> str:
    return f"""<material>
              <ambient>{rgba}</ambient>
              <diffuse>{rgba}</diffuse>
              <specular>0.02 0.02 0.02 1</specular>
            </material>"""


ASPHALT = material("0.045 0.048 0.052 1")
WHITE = material("0.94 0.94 0.90 1")
YELLOW = material("1.00 0.72 0.02 1")
CONCRETE = material("0.32 0.33 0.34 1")


def polygon_visual(name: str, points: list[Point], z: float, height: float, appearance: str) -> str:
    point_xml = "\n".join(f"                <point>{x:.6f} {y:.6f}</point>" for x, y in points)
    return f"""          <visual name="{name}">
            <pose>0 0 {z:.6f} 0 0 0</pose>
            <cast_shadows>false</cast_shadows>
            <geometry>
              <polyline>
                <height>{height:.6f}</height>
{point_xml}
              </polyline>
            </geometry>
            {appearance}
          </visual>"""


def polygon_area(points: list[Point]) -> float:
    """Return signed area; positive is counter-clockwise."""
    return 0.5 * sum(
        x0 * y1 - x1 * y0
        for (x0, y0), (x1, y1) in zip(points, points[1:] + points[:1])
    )


def strip_visuals(
    name: str,
    road_edge: list[Point],
    concrete_edge: list[Point],
    expected_width: float,
    z: float,
    height: float,
    appearance: str,
    segments_per_visual: int = 12,
) -> list[str]:
    """Create an explicit marking strip from two corresponding offset rings.

    Rendering a large white infield and covering most of it with a concrete
    polygon can hide the remaining line in tight concave curves.  Small simple
    strip polygons contain only the requested 5 cm marking and triangulate
    reliably in Gazebo.
    """
    if len(road_edge) != len(concrete_edge):
        raise ValueError(
            f"{name} offset rings do not correspond: "
            f"{len(road_edge)} != {len(concrete_edge)}")
    inner = list(concrete_edge)
    if polygon_area(road_edge) * polygon_area(inner) < 0.0:
        inner.reverse()
    shift = min(range(len(inner)), key=lambda index: math.dist(road_edge[0], inner[index]))
    inner = inner[shift:] + inner[:shift]
    pair_distances = [math.dist(outer, inside) for outer, inside in zip(road_edge, inner)]
    if min(pair_distances) < expected_width * 0.90:
        raise ValueError(f"{name} strip locally narrower than requested")
    # A mitered 90-degree corner is sqrt(2) times the perpendicular line width.
    if max(pair_distances) > expected_width * 1.50:
        raise ValueError(f"{name} strip has an invalid offset correspondence")

    outer_closed = road_edge + road_edge[:1]
    inner_closed = inner + inner[:1]
    visuals: list[str] = []
    for start in range(0, len(road_edge), segments_per_visual):
        end = min(start + segments_per_visual, len(road_edge))
        strip = outer_closed[start:end + 1] + list(
            reversed(inner_closed[start:end + 1]))
        visuals.append(polygon_visual(
            f"{name}_{start:04d}", strip, z, height, appearance))
    return visuals


def box_visual(name: str, center: Point, yaw: float, z: float, appearance: str) -> str:
    return f"""          <visual name="{name}">
            <pose>{center[0]:.6f} {center[1]:.6f} {z:.6f} 0 0 {yaw:.6f}</pose>
            <cast_shadows>false</cast_shadows>
            <geometry>
              <box><size>{DASH_LENGTH_M:.6f} {DASH_WIDTH_M:.6f} {OVERLAY_HEIGHT:.6f}</size></box>
            </geometry>
            {appearance}
          </visual>"""


def dash_visuals(polygons: list[list[Point]]) -> list[str]:
    """Build uniform, reliable dash visuals at all source dash locations."""
    expected = set(range(3, 34)) | set(range(36, 121))
    ordered = set(OUTER_DASH_PATH) | set(MIDDLE_DASH_PATH)
    if ordered != expected or len(OUTER_DASH_PATH) + len(MIDDLE_DASH_PATH) != len(expected):
        raise ValueError("dash path ordering does not cover every source dash exactly once")

    visuals: list[str] = []
    for path, cyclic in ((OUTER_DASH_PATH, True), (MIDDLE_DASH_PATH, False)):
        centers = [polygon_centroid(polygons[index]) for index in path]
        for position, (index, center) in enumerate(zip(path, centers)):
            if position == 0 and not cyclic:
                previous = center
            else:
                previous = centers[(position - 1) % len(centers)]
            if position == len(centers) - 1 and not cyclic:
                following = center
            else:
                following = centers[(position + 1) % len(centers)]
            yaw = math.atan2(following[1] - previous[1], following[0] - previous[0])
            visuals.append(box_visual(f"dxf_{index:03d}_uniform_dash", center, yaw, 0.00325, YELLOW))
    return visuals


def world_text(
    source: Path,
    polylines: list[Polyline],
    meters_per_unit: float,
    max_arc_step_deg: float,
) -> str:
    if meters_per_unit <= 0.0:
        raise ValueError("meters per DXF unit must be positive")

    raw_polygons = [tessellate(polyline, max_arc_step_deg) for polyline in polylines]
    min_x, min_y, max_x, max_y = bounds(raw_polygons[0])
    center_x = (min_x + max_x) * 0.5
    center_y = (min_y + max_y) * 0.5

    def transform(points: list[Point]) -> list[Point]:
        # Preserve the DXF's +X/+Y orientation and only move its centre to the
        # Gazebo origin.  The uniform unit conversion does not distort shape.
        return [((x - center_x) * meters_per_unit, (y - center_y) * meters_per_unit) for x, y in points]

    polygons = [transform(points) for points in raw_polygons]
    width_m = (max_x - min_x) * meters_per_unit
    height_m = (max_y - min_y) * meters_per_unit
    ground_size_x = width_m + GROUND_MARGIN_M * 2.0
    ground_size_y = height_m + GROUND_MARGIN_M * 2.0

    # Build the requested cross-section by offsetting the original outer road
    # and two infield boundaries equally about the existing centreline:
    #   white 0.05 m | grey road 0.80 m | white 0.05 m
    # Polygon layering provides holes without requiring unsupported SDF holes.
    grey_offset = GRAY_EDGE_EXPANSION_M
    white_offset = grey_offset + WHITE_EDGE_WIDTH_M
    outer_white_edge = offset_polygon(polygons[0], white_offset)
    outer_grey_edge = offset_polygon(polygons[0], grey_offset)
    left_white_edge = offset_polygon(polygons[122], -grey_offset)
    right_white_edge = offset_polygon(polygons[124], -grey_offset)
    left_concrete = offset_polygon(polygons[122], -white_offset)
    right_concrete = offset_polygon(polygons[124], -white_offset)

    visuals = [
        polygon_visual("road_total_outer_white", outer_white_edge, 0.0, ASPHALT_HEIGHT, WHITE),
        polygon_visual("road_grey_surface_0800m", outer_grey_edge, 0.00200, OVERLAY_HEIGHT, ASPHALT),
        polygon_visual("road_left_concrete_infield", left_concrete, 0.00225, OVERLAY_HEIGHT, CONCRETE),
        polygon_visual("road_right_concrete_infield", right_concrete, 0.00225, OVERLAY_HEIGHT, CONCRETE),
    ]
    visuals.extend(strip_visuals(
        "road_left_inner_white_0050m",
        left_white_edge,
        left_concrete,
        WHITE_EDGE_WIDTH_M,
        0.00250,
        OVERLAY_HEIGHT,
        WHITE,
    ))
    visuals.extend(strip_visuals(
        "road_right_inner_white_0050m",
        right_white_edge,
        right_concrete,
        WHITE_EDGE_WIDTH_M,
        0.00250,
        OVERLAY_HEIGHT,
        WHITE,
    ))

    visuals.extend(dash_visuals(polygons))

    visual_xml = "\n".join(visuals)
    return f"""<?xml version="1.0" ?>
<sdf version="1.9">
  <!--
    Generated from {escape(source.name)}.
    Unit conversion: 1 DXF unit = {meters_per_unit:.9f} m (no fit-to-size scaling).
    Exact outer extent after DXF bulge expansion: {width_m:.6f} m x {height_m:.6f} m.
    All 125 source polylines are parsed; requested road-edge offsets replace the six source edge outlines.
    Nominal grey road width: {GRAY_ROAD_WIDTH_M:.3f} m.
    White edge line width: {WHITE_EDGE_WIDTH_M:.3f} m on each side, outside the grey road.
    Nominal total road width: {GRAY_ROAD_WIDTH_M + 2.0 * WHITE_EDGE_WIDTH_M:.3f} m.
    Centre dashes use every source location with a uniform 0.20 m x 0.05 m render shape.
    Road markings are visual-only and the complete map has one level collision plane.
  -->
  <world name="kookmin_dxf_track">
    <gravity>0 0 -9.80665</gravity>
    <physics name="default_physics" type="dart">
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1.0</real_time_factor>
      <real_time_update_rate>1000</real_time_update_rate>
    </physics>
    <scene>
      <ambient>0.48 0.48 0.48 1</ambient>
      <background>0.68 0.72 0.76 1</background>
      <shadows>true</shadows>
    </scene>
    <light name="sun" type="directional">
      <pose>0 0 30 0 0 0</pose>
      <direction>-0.45 0.25 -0.86</direction>
      <diffuse>0.85 0.85 0.85 1</diffuse>
      <specular>0.15 0.15 0.15 1</specular>
    </light>

    <model name="level_ground">
      <static>true</static>
      <link name="ground_link">
        <collision name="ground_collision">
          <geometry>
            <plane>
              <normal>0 0 1</normal>
              <size>{ground_size_x:.6f} {ground_size_y:.6f}</size>
            </plane>
          </geometry>
        </collision>
        <visual name="concrete_surround">
          <geometry>
            <plane>
              <normal>0 0 1</normal>
              <size>{ground_size_x:.6f} {ground_size_y:.6f}</size>
            </plane>
          </geometry>
          {CONCRETE}
        </visual>
      </link>
    </model>

    <model name="dxf_course_surface_and_markings">
      <static>true</static>
      <link name="course_link">
{visual_xml}
      </link>
    </model>
  </world>
</sdf>
"""


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("dxf", type=Path, help="source ASCII R10 DXF file")
    parser.add_argument("output", type=Path, help="world .sdf file to create")
    parser.add_argument(
        "--meters-per-unit",
        type=float,
        default=DEFAULT_METERS_PER_UNIT,
        help=f"metres represented by one DXF unit (default: {DEFAULT_METERS_PER_UNIT})",
    )
    parser.add_argument(
        "--arc-step-deg",
        type=float,
        default=DEFAULT_ARC_STEP_DEG,
        help=f"maximum tessellation angle for DXF bulge arcs (default: {DEFAULT_ARC_STEP_DEG})",
    )
    args = parser.parse_args()

    args.output.parent.mkdir(parents=True, exist_ok=True)
    output = world_text(args.dxf, parse_polylines(args.dxf), args.meters_per_unit, args.arc_step_deg)
    args.output.write_text(output, encoding="utf-8")


if __name__ == "__main__":
    main()
