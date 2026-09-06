import math
from dataclasses import dataclass

from mission_cone_drive.corridor_geometry import CorridorGeometry


@dataclass
class TrackedPoint:
    track_id: int
    x: float
    y: float
    hits: int = 1
    misses: int = 0
    updated: bool = True

    @property
    def point(self):
        return (self.x, self.y)


@dataclass(frozen=True)
class LidarCorridorResult:
    tracked_candidates: list
    accepted_cones: list
    left_cones: list
    right_cones: list
    midpoints: list
    pair_count: int
    state: str


class LidarCorridorFilter:
    """Track LiDAR objects and keep only a geometrically valid corridor."""

    def __init__(
        self,
        min_width_m=0.50,
        max_width_m=1.00,
        expected_width_m=0.80,
        cone_spacing_m=0.30,
        min_normal_alignment=0.45,
        max_pair_longitudinal_m=0.45,
        midpoint_reference_gate_m=0.45,
        association_distance_m=0.22,
        min_track_hits=2,
        max_track_misses=2,
        track_latest_weight=0.80,
        min_entry_pairs=2,
        min_pair_span_m=0.18,
        max_entry_width_spread_m=0.20,
        one_side_half_width_m=0.40,
        boundary_tolerance_m=0.18,
        min_one_side_points=2,
        two_point_support_confirm_frames=2,
        established_hold_frames=10,
    ):
        self.geometry = CorridorGeometry(
            min_width_m=min_width_m,
            max_width_m=max_width_m,
            expected_width_m=expected_width_m,
            cone_spacing_m=cone_spacing_m,
            min_normal_alignment=min_normal_alignment,
            max_pair_longitudinal_m=max_pair_longitudinal_m,
            midpoint_reference_gate_m=midpoint_reference_gate_m,
        )
        self.association_distance_m = max(
            float(association_distance_m), 0.05)
        self.min_track_hits = max(int(min_track_hits), 1)
        self.max_track_misses = max(int(max_track_misses), 0)
        self.track_latest_weight = min(
            max(float(track_latest_weight), 0.0), 1.0)
        self.min_entry_pairs = max(int(min_entry_pairs), 1)
        self.min_pair_span_m = max(float(min_pair_span_m), 0.0)
        self.max_entry_width_spread_m = max(
            float(max_entry_width_spread_m), 0.0)
        self.one_side_half_width_m = max(
            float(one_side_half_width_m), 0.10)
        self.boundary_tolerance_m = max(
            float(boundary_tolerance_m), 0.03)
        self.min_one_side_points = max(int(min_one_side_points), 2)
        self.two_point_support_confirm_frames = max(
            int(two_point_support_confirm_frames), 1)
        self.established_hold_frames = max(
            int(established_hold_frames), 0)
        self.cone_spacing_m = max(float(cone_spacing_m), 0.05)

        self.tracks = []
        self.next_track_id = 1
        self.reference_path = []
        self.established = False
        self.lost_frames = 0
        self.two_point_support_frames = 0

    def reset(self):
        self.tracks = []
        self.reference_path = []
        self.established = False
        self.lost_frames = 0
        self.two_point_support_frames = 0

    def update(self, candidate_points):
        points = self._unique_points(candidate_points)
        tracked = self._update_tracks(points)
        observation = self.geometry.extract(tracked, self.reference_path)

        pair_span = self._path_length(observation.paired_midpoints)
        pair_width_spread = self._value_spread(
            observation.paired_widths)
        strong_pairing = (
            observation.pair_count >= self.min_entry_pairs
            and pair_span >= self.min_pair_span_m
            and pair_width_spread <= self.max_entry_width_spread_m
        )

        if strong_pairing:
            left = observation.paired_left_cones
            right = observation.paired_right_cones
            midpoints = observation.paired_midpoints
            self.reference_path = self._sorted_points(midpoints)
            self.established = True
            self.lost_frames = 0
            self.two_point_support_frames = 0
            state = 'paired'
        elif self.established and len(self.reference_path) >= 2:
            left, right = self._select_one_side_support(tracked)
            support_count = len(left) + len(right)
            has_side = (
                len(left) >= self.min_one_side_points
                or len(right) >= self.min_one_side_points
            )
            if has_side and support_count == 2:
                self.two_point_support_frames += 1
                support_confirmed = (
                    self.two_point_support_frames
                    >= self.two_point_support_confirm_frames
                )
            else:
                self.two_point_support_frames = 0
                support_confirmed = has_side

            if support_confirmed:
                self.lost_frames = 0
                state = 'partial'
            else:
                self.lost_frames += 1
                left = []
                right = []
                state = 'holding'

            midpoints = []
            if self.lost_frames > self.established_hold_frames:
                self.established = False
                self.reference_path = []
                state = 'lost'
        else:
            left = []
            right = []
            midpoints = []
            self.two_point_support_frames = 0
            state = 'searching'

        if (
            self.established
            and len(midpoints) < 2
            and len(self.reference_path) >= 2
        ):
            estimated = self.geometry.estimate_center_from_one_side(
                left_cones=left,
                right_cones=right,
                reference_path=self.reference_path,
                half_width_m=self.one_side_half_width_m,
                min_points=self.min_one_side_points,
            )
            midpoints = self._unique_points(midpoints + estimated)

        accepted = self._unique_points(left + right)
        return LidarCorridorResult(
            tracked_candidates=tracked,
            accepted_cones=accepted,
            left_cones=left,
            right_cones=right,
            midpoints=midpoints,
            pair_count=observation.pair_count,
            state=state,
        )

    def _update_tracks(self, points):
        for track in self.tracks:
            track.updated = False

        possible_matches = []
        for track_index, track in enumerate(self.tracks):
            for point_index, point in enumerate(points):
                distance = self._distance(track.point, point)
                if distance <= self.association_distance_m:
                    possible_matches.append(
                        (distance, track_index, point_index))

        matched_tracks = set()
        matched_points = set()
        for _, track_index, point_index in sorted(possible_matches):
            if track_index in matched_tracks or point_index in matched_points:
                continue
            track = self.tracks[track_index]
            point = points[point_index]
            weight = self.track_latest_weight
            track.x = weight * point[0] + (1.0 - weight) * track.x
            track.y = weight * point[1] + (1.0 - weight) * track.y
            track.hits += 1
            track.misses = 0
            track.updated = True
            matched_tracks.add(track_index)
            matched_points.add(point_index)

        for track_index, track in enumerate(self.tracks):
            if track_index in matched_tracks:
                continue
            track.misses += 1
            track.hits = max(track.hits - 1, 0)

        for point_index, point in enumerate(points):
            if point_index in matched_points:
                continue
            self.tracks.append(TrackedPoint(
                track_id=self.next_track_id,
                x=point[0],
                y=point[1],
            ))
            self.next_track_id += 1

        self.tracks = [
            track for track in self.tracks
            if track.misses <= self.max_track_misses
        ]
        return self._sorted_points([
            track.point for track in self.tracks
            if track.updated and track.hits >= self.min_track_hits
        ])

    def _select_one_side_support(self, points):
        left_candidates = []
        right_candidates = []
        for point in points:
            center, _, normal, valid = self.geometry.reference_pose(
                point[0], self.reference_path)
            if not valid:
                continue
            offset = (
                point[0] - center[0],
                point[1] - center[1],
            )
            signed_distance = self._dot(offset, normal)
            width_error = abs(
                abs(signed_distance) - self.one_side_half_width_m)
            if width_error > self.boundary_tolerance_m:
                continue
            scored = (point, width_error)
            if signed_distance > 0.0:
                left_candidates.append(scored)
            else:
                right_candidates.append(scored)

        return (
            self._best_boundary_chain(left_candidates),
            self._best_boundary_chain(right_candidates),
        )

    def _best_boundary_chain(self, candidates):
        if not candidates:
            return []

        ordered = sorted(candidates, key=lambda item: item[0][0])
        minimum_step = max(self.cone_spacing_m * 0.30, 0.08)
        maximum_step = max(self.cone_spacing_m * 2.3, 0.60)
        rewards = [1.0 - item[1] for item in ordered]
        predecessors = [-1] * len(ordered)

        for current_index, (current, current_error) in enumerate(ordered):
            for previous_index in range(current_index):
                previous = ordered[previous_index][0]
                dx = current[0] - previous[0]
                distance = self._distance(current, previous)
                if dx < 0.03:
                    continue
                if distance < minimum_step or distance > maximum_step:
                    continue
                spacing_multiple = max(
                    1,
                    min(3, round(distance / self.cone_spacing_m)),
                )
                spacing_error = abs(
                    distance - spacing_multiple * self.cone_spacing_m
                ) / self.cone_spacing_m
                score = (
                    rewards[previous_index]
                    + 1.0
                    - current_error
                    - spacing_error * 0.35
                )
                if score > rewards[current_index]:
                    rewards[current_index] = score
                    predecessors[current_index] = previous_index

        index = max(range(len(ordered)), key=lambda item: rewards[item])
        chain = []
        while index >= 0:
            chain.append(ordered[index][0])
            index = predecessors[index]
        chain.reverse()
        if len(chain) < self.min_one_side_points:
            return []
        return chain

    @staticmethod
    def _dot(first, second):
        return first[0] * second[0] + first[1] * second[1]

    @staticmethod
    def _distance(first, second):
        return math.hypot(first[0] - second[0], first[1] - second[1])

    @staticmethod
    def _value_spread(values):
        if len(values) < 2:
            return 0.0
        return max(values) - min(values)

    @classmethod
    def _path_length(cls, points):
        ordered = cls._sorted_points(points)
        return sum(
            cls._distance(ordered[index - 1], ordered[index])
            for index in range(1, len(ordered))
        )

    @staticmethod
    def _sorted_points(points):
        return sorted(
            [(float(point[0]), float(point[1])) for point in points],
            key=lambda point: (point[0], point[1]),
        )

    @classmethod
    def _unique_points(cls, points):
        unique = []
        for point in cls._sorted_points(points):
            if any(cls._distance(point, other) < 1e-4 for other in unique):
                continue
            unique.append(point)
        return unique
