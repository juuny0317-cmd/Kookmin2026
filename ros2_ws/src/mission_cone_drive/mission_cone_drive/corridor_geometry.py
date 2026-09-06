import math
from dataclasses import dataclass


@dataclass(frozen=True)
class CorridorPair:
    left_index: int
    right_index: int
    left: tuple
    right: tuple
    midpoint: tuple
    score: float


@dataclass(frozen=True)
class CorridorObservation:
    left_cones: list
    right_cones: list
    paired_midpoints: list
    pair_count: int
    paired_left_cones: list
    paired_right_cones: list
    paired_widths: list


class CorridorGeometry:
    """Extract a cone corridor relative to its centerline, not vehicle y=0."""

    def __init__(
        self,
        min_width_m,
        max_width_m,
        expected_width_m,
        cone_spacing_m=0.30,
        side_deadband_m=0.04,
        min_normal_alignment=0.45,
        max_pair_longitudinal_m=0.45,
        midpoint_reference_gate_m=0.45,
    ):
        self.min_width_m = float(min_width_m)
        self.max_width_m = float(max_width_m)
        self.expected_width_m = float(expected_width_m)
        self.cone_spacing_m = max(float(cone_spacing_m), 0.05)
        self.side_deadband_m = max(float(side_deadband_m), 0.0)
        self.min_normal_alignment = float(min_normal_alignment)
        self.max_pair_longitudinal_m = max(
            float(max_pair_longitudinal_m), 0.05)
        self.midpoint_reference_gate_m = max(
            float(midpoint_reference_gate_m), 0.05)

    def extract(self, centers, reference_path):
        points = [(float(x), float(y)) for x, y in centers]
        reference = self._sorted_path(reference_path)
        reference_candidates = self._build_pair_candidates(
            points, reference)
        reference_selected = self._select_pair_chain(
            reference_candidates, reference)

        selected = reference_selected
        if reference:
            current_candidates = self._build_pair_candidates(points, [])
            current_selected = self._select_pair_chain(
                current_candidates, [])
            if (
                current_selected
                and len(current_selected) > len(reference_selected)
            ):
                selected = current_selected

        selected = self._orient_pairs_to_current_chain(
            selected, reference)

        paired_indices = set()
        left = []
        right = []
        midpoints = []
        for pair in selected:
            paired_indices.add(pair.left_index)
            paired_indices.add(pair.right_index)
            left.append(pair.left)
            right.append(pair.right)
            midpoints.append(pair.midpoint)

        classification_path = midpoints if len(midpoints) >= 2 else reference
        extra_left, extra_right = self._classify_unpaired(
            points,
            paired_indices,
            classification_path,
        )
        left.extend(extra_left)
        right.extend(extra_right)

        return CorridorObservation(
            left_cones=self._unique_points(left),
            right_cones=self._unique_points(right),
            paired_midpoints=self._unique_points(midpoints),
            pair_count=len(selected),
            paired_left_cones=self._unique_points(
                [pair.left for pair in selected]),
            paired_right_cones=self._unique_points(
                [pair.right for pair in selected]),
            paired_widths=[
                self._distance(pair.left, pair.right)
                for pair in selected
            ],
        )

    def estimate_center_from_one_side(
        self,
        left_cones,
        right_cones,
        reference_path,
        half_width_m,
        min_points,
    ):
        reference = self._sorted_path(reference_path)
        if len(reference) < 2:
            return []

        candidates = []
        if len(left_cones) >= min_points:
            path = self._offset_boundary(
                left_cones, reference, -float(half_width_m))
            candidates.append(path)
        if len(right_cones) >= min_points:
            path = self._offset_boundary(
                right_cones, reference, float(half_width_m))
            candidates.append(path)

        candidates = [path for path in candidates if path]
        if not candidates:
            return []

        def path_score(path):
            errors = []
            for point in path:
                center, _, _, _ = self.reference_pose(point[0], reference)
                errors.append(self._distance(point, center))
            continuity = sum(errors) / max(len(errors), 1)
            return continuity - 0.03 * len(path)

        return min(candidates, key=path_score)

    def reference_pose(self, x, path):
        points = self._sorted_path(path)
        if len(points) < 2:
            return (float(x), 0.0), (1.0, 0.0), (0.0, 1.0), False

        if x <= points[0][0]:
            first, second = points[0], points[1]
        elif x >= points[-1][0]:
            first, second = points[-2], points[-1]
        else:
            first, second = points[0], points[1]
            for index in range(len(points) - 1):
                if points[index][0] <= x <= points[index + 1][0]:
                    first, second = points[index], points[index + 1]
                    break

        dx = second[0] - first[0]
        dy = second[1] - first[1]
        if abs(dx) < 1e-6:
            ratio = 0.0
        else:
            ratio = (float(x) - first[0]) / dx
        center_y = first[1] + ratio * dy

        length = math.hypot(dx, dy)
        if length < 1e-6:
            tangent = (1.0, 0.0)
        else:
            tangent = (dx / length, dy / length)
        normal = (-tangent[1], tangent[0])
        return (float(x), center_y), tangent, normal, True

    def _build_pair_candidates(self, points, reference):
        candidates = []
        has_reference = len(reference) >= 2
        initial_alignment = max(0.30, self.min_normal_alignment - 0.15)

        for first_index in range(len(points)):
            for second_index in range(first_index + 1, len(points)):
                first = points[first_index]
                second = points[second_index]
                vector = (second[0] - first[0], second[1] - first[1])
                distance = math.hypot(vector[0], vector[1])
                if distance < 1e-6:
                    continue

                midpoint = (
                    (first[0] + second[0]) * 0.5,
                    (first[1] + second[1]) * 0.5,
                )
                center, tangent, normal, reference_valid = self.reference_pose(
                    midpoint[0], reference)
                longitudinal = abs(self._dot(vector, tangent))
                normal_width = abs(self._dot(vector, normal))
                normal_alignment = normal_width / distance

                if reference_valid:
                    measured_width = normal_width
                    if longitudinal > self.max_pair_longitudinal_m:
                        continue
                    if normal_alignment < self.min_normal_alignment:
                        continue
                else:
                    measured_width = distance
                    if normal_alignment < initial_alignment:
                        continue

                if measured_width < self.min_width_m:
                    continue
                if measured_width > self.max_width_m:
                    continue

                center_error = abs(self._dot(
                    (midpoint[0] - center[0], midpoint[1] - center[1]),
                    normal,
                ))
                if (
                    has_reference
                    and center_error > self.midpoint_reference_gate_m
                ):
                    continue

                first_side = self._dot(
                    (first[0] - midpoint[0], first[1] - midpoint[1]),
                    normal,
                )
                second_side = self._dot(
                    (second[0] - midpoint[0], second[1] - midpoint[1]),
                    normal,
                )
                if first_side > second_side:
                    left_index, left = first_index, first
                    right_index, right = second_index, second
                else:
                    left_index, left = second_index, second
                    right_index, right = first_index, first

                width_error = abs(
                    measured_width - self.expected_width_m
                ) / max(self.expected_width_m, 0.10)
                alignment_error = 1.0 - normal_alignment
                longitudinal_error = (
                    longitudinal / self.max_pair_longitudinal_m
                )
                score = (
                    width_error * 1.2
                    + alignment_error * 1.5
                    + center_error * 2.5
                    + longitudinal_error * 0.3
                )
                candidates.append(CorridorPair(
                    left_index=left_index,
                    right_index=right_index,
                    left=left,
                    right=right,
                    midpoint=midpoint,
                    score=score,
                ))

        return candidates

    def _select_pair_chain(self, candidates, reference):
        if not candidates:
            return []

        ordered = sorted(candidates, key=lambda pair: (
            pair.midpoint[0], pair.score))
        reward = 2.5
        best_scores = [reward - pair.score for pair in ordered]
        predecessors = [-1] * len(ordered)
        max_step = max(self.cone_spacing_m * 2.5, 0.65)

        for current_index, current in enumerate(ordered):
            for previous_index in range(current_index):
                previous = ordered[previous_index]
                if self._pairs_share_cone(previous, current):
                    continue

                dx = current.midpoint[0] - previous.midpoint[0]
                if dx < 0.06:
                    continue

                step_vector = (
                    current.midpoint[0] - previous.midpoint[0],
                    current.midpoint[1] - previous.midpoint[1],
                )
                step_distance = math.hypot(*step_vector)
                if step_distance < 0.08 or step_distance > max_step:
                    continue

                transition = self._pair_transition_metrics(
                    previous, current, step_vector, step_distance)
                if transition is None:
                    continue
                (
                    pair_alignment,
                    cross_alignment,
                    boundary_imbalance,
                ) = transition

                _, tangent, _, reference_valid = self.reference_pose(
                    (current.midpoint[0] + previous.midpoint[0]) * 0.5,
                    reference,
                )
                forward_alignment = (
                    self._dot(step_vector, tangent) / step_distance
                )
                if reference_valid and forward_alignment < 0.20:
                    continue

                spacing_multiple = max(
                    1,
                    min(3, round(step_distance / self.cone_spacing_m)),
                )
                expected_step = spacing_multiple * self.cone_spacing_m
                spacing_error = abs(
                    step_distance - expected_step) / self.cone_spacing_m
                transition_cost = (
                    spacing_error * 0.5
                    + (1.0 - max(forward_alignment, 0.0)) * 0.8
                    + (1.0 - pair_alignment) * 0.8
                    + cross_alignment * 0.8
                    + boundary_imbalance * 0.4
                )
                candidate_score = (
                    best_scores[previous_index]
                    + reward
                    - current.score
                    - transition_cost
                )
                if candidate_score > best_scores[current_index]:
                    best_scores[current_index] = candidate_score
                    predecessors[current_index] = previous_index

        end_index = max(range(len(ordered)), key=lambda i: best_scores[i])
        selected = []
        while end_index >= 0:
            selected.append(ordered[end_index])
            end_index = predecessors[end_index]
        selected.reverse()

        unique_chain = []
        used_indices = set()
        for pair in selected:
            indices = {pair.left_index, pair.right_index}
            if used_indices.intersection(indices):
                continue
            unique_chain.append(pair)
            used_indices.update(indices)
        return unique_chain

    def _pair_transition_metrics(
        self,
        previous,
        current,
        step_vector,
        step_distance,
    ):
        previous_normal = self._pair_normal(previous)
        current_normal = self._pair_normal(current)
        pair_alignment = self._dot(previous_normal, current_normal)
        if pair_alignment < 0.35:
            return None

        average_normal = (
            previous_normal[0] + current_normal[0],
            previous_normal[1] + current_normal[1],
        )
        average_length = math.hypot(*average_normal)
        if average_length < 1e-6:
            return None
        average_normal = (
            average_normal[0] / average_length,
            average_normal[1] / average_length,
        )
        step_unit = (
            step_vector[0] / step_distance,
            step_vector[1] / step_distance,
        )
        cross_alignment = abs(self._dot(step_unit, average_normal))
        if cross_alignment > 0.78:
            return None

        left_step = self._distance(previous.left, current.left)
        right_step = self._distance(previous.right, current.right)
        max_boundary_step = max(self.cone_spacing_m * 3.0, 0.95)
        if left_step > max_boundary_step or right_step > max_boundary_step:
            return None

        boundary_imbalance = (
            abs(left_step - right_step)
            / max(self.cone_spacing_m, 0.10)
        )
        return pair_alignment, cross_alignment, boundary_imbalance

    def _orient_pairs_to_current_chain(self, pairs, reference):
        if len(pairs) < 2:
            return pairs

        oriented = []
        for index, pair in enumerate(pairs):
            before = pairs[max(0, index - 1)].midpoint
            after = pairs[min(len(pairs) - 1, index + 1)].midpoint
            tangent = (
                after[0] - before[0],
                after[1] - before[1],
            )
            tangent_length = math.hypot(*tangent)
            if tangent_length < 1e-6:
                _, tangent, _, valid = self.reference_pose(
                    pair.midpoint[0], reference)
                if not valid:
                    oriented.append(pair)
                    continue
            else:
                tangent = (
                    tangent[0] / tangent_length,
                    tangent[1] / tangent_length,
                )

            normal = (-tangent[1], tangent[0])
            first_side = self._dot(
                (
                    pair.left[0] - pair.midpoint[0],
                    pair.left[1] - pair.midpoint[1],
                ),
                normal,
            )
            second_side = self._dot(
                (
                    pair.right[0] - pair.midpoint[0],
                    pair.right[1] - pair.midpoint[1],
                ),
                normal,
            )
            if first_side >= second_side:
                oriented.append(pair)
            else:
                oriented.append(CorridorPair(
                    left_index=pair.right_index,
                    right_index=pair.left_index,
                    left=pair.right,
                    right=pair.left,
                    midpoint=pair.midpoint,
                    score=pair.score,
                ))
        return oriented

    def _pair_normal(self, pair):
        vector = (
            pair.left[0] - pair.right[0],
            pair.left[1] - pair.right[1],
        )
        length = math.hypot(*vector)
        if length < 1e-6:
            return (0.0, 1.0)
        return (vector[0] / length, vector[1] / length)

    def _classify_unpaired(self, points, paired_indices, reference_path):
        if len(reference_path) < 2:
            return [], []

        left = []
        right = []
        for index, point in enumerate(points):
            if index in paired_indices:
                continue
            center, _, normal, _ = self.reference_pose(
                point[0], reference_path)
            signed_distance = self._dot(
                (point[0] - center[0], point[1] - center[1]),
                normal,
            )
            if signed_distance > self.side_deadband_m:
                left.append(point)
            elif signed_distance < -self.side_deadband_m:
                right.append(point)
        return left, right

    def _offset_boundary(self, boundary, reference, normal_offset):
        result = []
        for point in sorted(boundary, key=lambda item: item[0]):
            _, _, normal, valid = self.reference_pose(point[0], reference)
            if not valid:
                continue
            estimate = (
                point[0] + normal[0] * normal_offset,
                point[1] + normal[1] * normal_offset,
            )
            predicted, _, predicted_normal, _ = self.reference_pose(
                estimate[0], reference)
            lateral_error = abs(self._dot(
                (estimate[0] - predicted[0], estimate[1] - predicted[1]),
                predicted_normal,
            ))
            if lateral_error <= self.midpoint_reference_gate_m:
                result.append(estimate)
        return self._unique_points(result)

    @staticmethod
    def _pairs_share_cone(first, second):
        first_indices = {first.left_index, first.right_index}
        second_indices = {second.left_index, second.right_index}
        return bool(first_indices.intersection(second_indices))

    @staticmethod
    def _dot(first, second):
        return first[0] * second[0] + first[1] * second[1]

    @staticmethod
    def _distance(first, second):
        return math.hypot(first[0] - second[0], first[1] - second[1])

    @staticmethod
    def _sorted_path(path):
        return sorted(
            [(float(x), float(y)) for x, y in path],
            key=lambda point: point[0],
        )

    @staticmethod
    def _unique_points(points):
        unique = []
        for point in sorted(points, key=lambda item: (item[0], item[1])):
            if any(
                math.hypot(point[0] - other[0], point[1] - other[1]) < 1e-4
                for other in unique
            ):
                continue
            unique.append(point)
        return unique
