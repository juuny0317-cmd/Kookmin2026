"""Timed opposite-lane events for dynamic and static obstacles."""

from __future__ import annotations

from collections import deque
from dataclasses import dataclass
from enum import Enum


SHORTCUT_OBSTACLE_CONFLICT_STATES = frozenset({
    'PREPARE_LEFT',
    'FOLLOW_BRANCH',
    'EXIT_LEFT_COMMIT',
    'EXIT_HANDOFF',
    'FAILED',
})

SHORTCUT_OVERRIDE_RESET_PROTECTED_STATES = frozenset({
    'PREPARE_LEFT',
    'FOLLOW_BRANCH',
    'FAILED',
})


def shortcut_blocks_obstacle_event(state):
    """Return whether shortcut control must own the lane command."""
    return str(state).strip().upper() in SHORTCUT_OBSTACLE_CONFLICT_STATES


def shortcut_protects_override_from_reset(state):
    """Return whether an obstacle reset would erase a shortcut command."""
    normalized = str(state).strip().upper()
    return normalized in SHORTCUT_OVERRIDE_RESET_PROTECTED_STATES


def road_allows_obstacle_event(is_curve, allow_on_curve):
    """Allow obstacle avoidance on every road unless curve use is disabled."""
    return bool(allow_on_curve) or not bool(is_curve)


class DynamicLaneEventState(str, Enum):
    """Phases of one non-retriggering obstacle event."""

    IDLE = 'IDLE'
    APPROACH = 'APPROACH'
    ACTIVE = 'ACTIVE'
    WAIT_CLEAR = 'WAIT_CLEAR'


@dataclass(frozen=True)
class DynamicLaneEventConfig:
    """Independent speed, timing, and confirmation settings by kind."""

    duration_s: float = 5.0
    speed_confirm_min_matches: int = 2
    speed_confirm_window_frames: int = 3
    avoid_confirm_min_matches: int = 2
    avoid_confirm_window_frames: int = 3
    rearm_clear_frames: int = 5
    speed: float = 7.0
    static_duration_s: float = 4.0
    static_speed_confirm_min_matches: int = 2
    static_speed_confirm_window_frames: int = 3
    static_avoid_confirm_min_matches: int = 2
    static_avoid_confirm_window_frames: int = 3
    static_rearm_clear_frames: int = 5
    static_speed: float = 7.0


@dataclass(frozen=True)
class DynamicLaneEventDecision:
    """Lane command selected for the current perception frame."""

    state: DynamicLaneEventState
    command: str
    event_kind: str
    obstacle_lane: str
    target_lane: str
    elapsed_s: float
    speed_limit: float
    reason: str


class DynamicLaneEventMachine:
    """Latch the opposite lane using timing selected by obstacle kind."""

    TARGET_IOU_MATCH_THRESHOLD = 0.20

    OPPOSITE_COMMAND = {
        'left': ('right', 'go_right'),
        'right': ('left', 'go_left'),
    }
    OBSTACLE_KIND_BY_CLASS = {
        'dynamic': 'dynamic',
        'obstacle_vehicle': 'dynamic',
        'static': 'static',
    }

    def __init__(self, config=None):
        """Initialize an idle event machine."""
        self.config = config or DynamicLaneEventConfig()
        self.validate_confirmation_config()
        self.reset()

    def reset(self):
        """Clear the active event and confirmation history."""
        self.state = DynamicLaneEventState.IDLE
        self.speed_confirmation_history = deque(
            maxlen=self.maximum_confirmation_window())
        self.avoid_confirmation_history = deque(
            maxlen=self.maximum_confirmation_window())
        self.event_kind = 'unknown'
        self.obstacle_lane = 'unknown'
        self.target_lane = 'center'
        self.active_command = 'reset'
        self.state_entered_s = 0.0
        self.clear_count = 0
        self.last_timestamp_s = None
        self.last_reason = 'waiting_for_obstacle'

    @staticmethod
    def normalize_lane(value):
        """Return a supported lane label or unknown."""
        lane = str(value).strip().lower()
        return lane if lane in ('left', 'right') else 'unknown'

    @classmethod
    def normalize_obstacle_kind(cls, value):
        """Map detector class names to a distinct event kind."""
        return cls.OBSTACLE_KIND_BY_CLASS.get(
            str(value).strip().lower(), 'unknown')

    def duration_for(self, event_kind):
        """Return the configured hold duration for one event kind."""
        if event_kind == 'static':
            return max(0.0, float(self.config.static_duration_s))
        return max(0.0, float(self.config.duration_s))

    def speed_confirmation_for(self, event_kind):
        """Return speed-confirmation matches and window for one kind."""
        if event_kind == 'static':
            return (
                int(self.config.static_speed_confirm_min_matches),
                int(self.config.static_speed_confirm_window_frames),
            )
        return (
            int(self.config.speed_confirm_min_matches),
            int(self.config.speed_confirm_window_frames),
        )

    def avoid_confirmation_for(self, event_kind):
        """Return avoidance-confirmation matches and window for one kind."""
        if event_kind == 'static':
            return (
                int(self.config.static_avoid_confirm_min_matches),
                int(self.config.static_avoid_confirm_window_frames),
            )
        return (
            int(self.config.avoid_confirm_min_matches),
            int(self.config.avoid_confirm_window_frames),
        )

    def maximum_confirmation_window(self):
        """Return enough storage for every kind and decision history."""
        return max(
            1,
            int(self.config.speed_confirm_window_frames),
            int(self.config.avoid_confirm_window_frames),
            int(self.config.static_speed_confirm_window_frames),
            int(self.config.static_avoid_confirm_window_frames),
        )

    def validate_confirmation_config(self):
        """Reject impossible N-of-M confirmation settings."""
        settings = {
            'dynamic speed': self.speed_confirmation_for('dynamic'),
            'dynamic avoid': self.avoid_confirmation_for('dynamic'),
            'static speed': self.speed_confirmation_for('static'),
            'static avoid': self.avoid_confirmation_for('static'),
        }
        for name, (matches, window) in settings.items():
            if matches < 1 or window < 1 or matches > window:
                raise ValueError(
                    f'{name} confirmation requires '
                    f'1 <= min_matches <= window_frames; '
                    f'got {matches}-of-{window}')

    @staticmethod
    def normalize_bbox(value):
        """Return a valid float bounding box or None."""
        if value is None:
            return None
        try:
            xmin, ymin, xmax, ymax = (float(item) for item in value)
        except (TypeError, ValueError):
            return None
        if xmax <= xmin or ymax <= ymin:
            return None
        return xmin, ymin, xmax, ymax

    @classmethod
    def same_target(cls, first_bbox, second_bbox):
        """Associate nearby-frame detections using bounding-box IoU."""
        first = cls.normalize_bbox(first_bbox)
        second = cls.normalize_bbox(second_bbox)
        if first is None or second is None:
            # Pure-logic callers predating target association have no bbox.
            # Production passes a bbox for every eligible avoidance sample.
            return first is None and second is None
        intersection_width = max(
            0.0, min(first[2], second[2]) - max(first[0], second[0]))
        intersection_height = max(
            0.0, min(first[3], second[3]) - max(first[1], second[1]))
        intersection = intersection_width * intersection_height
        first_area = (first[2] - first[0]) * (first[3] - first[1])
        second_area = (second[2] - second[0]) * (second[3] - second[1])
        union = first_area + second_area - intersection
        return (
            union > 0.0
            and intersection / union >= cls.TARGET_IOU_MATCH_THRESHOLD
        )

    def record_speed_confirmation(self, eligible, event_kind):
        """Record one speed sample and evaluate its independent N-of-M."""
        observation = event_kind if eligible else None
        self.speed_confirmation_history.append(observation)
        matches_required, window_frames = self.speed_confirmation_for(
            event_kind)
        recent = list(self.speed_confirmation_history)[-window_frames:]
        matches = sum(item == event_kind for item in recent)
        return bool(eligible and matches >= matches_required), matches

    def record_avoid_confirmation(
            self, eligible, event_kind, lane, obstacle_bbox):
        """Confirm N-of-M observations of the same target and same lane."""
        observation = (
            (event_kind, lane, self.normalize_bbox(obstacle_bbox))
            if eligible else None
        )
        self.avoid_confirmation_history.append(observation)
        matches_required, window_frames = self.avoid_confirmation_for(
            event_kind)
        recent = list(self.avoid_confirmation_history)[-window_frames:]
        if not eligible:
            return False, 0
        current_bbox = observation[2]
        matches = sum(
            item is not None
            and item[0] == event_kind
            and item[1] == lane
            and self.same_target(item[2], current_bbox)
            for item in recent
        )
        return matches >= matches_required, matches

    def rearm_clear_frames_for(self, event_kind):
        """Return the configured clear count for one event kind."""
        if event_kind == 'static':
            return max(1, int(self.config.static_rearm_clear_frames))
        return max(1, int(self.config.rearm_clear_frames))

    def speed_for(self, event_kind):
        """Return the approach/avoidance/return speed cap for one kind."""
        if event_kind == 'static':
            return max(0.0, float(self.config.static_speed))
        return max(0.0, float(self.config.speed))

    def decision(self, timestamp_s):
        """Build the immutable output for the current state."""
        elapsed_s = 0.0
        if self.state == DynamicLaneEventState.ACTIVE:
            elapsed_s = max(0.0, float(timestamp_s) - self.state_entered_s)
        command = (
            self.active_command
            if self.state == DynamicLaneEventState.ACTIVE
            else 'reset'
        )
        speed_limit = (
            self.speed_for(self.event_kind)
            if self.state != DynamicLaneEventState.IDLE
            else -1.0
        )
        return DynamicLaneEventDecision(
            state=self.state,
            command=command,
            event_kind=self.event_kind,
            obstacle_lane=self.obstacle_lane,
            target_lane=self.target_lane,
            elapsed_s=elapsed_s,
            speed_limit=speed_limit,
            reason=self.last_reason,
        )

    def start_active(self, now_s, lane):
        """Latch the opposite lane after a confirmed close obstacle."""
        target_lane, command = self.OPPOSITE_COMMAND[lane]
        self.state = DynamicLaneEventState.ACTIVE
        self.state_entered_s = now_s
        self.obstacle_lane = lane
        self.target_lane = target_lane
        self.active_command = command
        self.speed_confirmation_history.clear()
        self.avoid_confirmation_history.clear()
        self.clear_count = 0
        self.last_reason = f'{self.event_kind}_opposite_lane_event_started'

    def update(
        self,
        timestamp_s,
        obstacle_lane='unknown',
        obstacle_kind='dynamic',
        object_present=False,
        speed_trigger_allowed=None,
        trigger_allowed=False,
        obstacle_bbox=None,
        new_target_present=False,
        blocked=False,
    ):
        """Advance one perception frame and return its lane command."""
        now_s = float(timestamp_s)
        if (
            self.last_timestamp_s is not None
            and now_s + 1e-6 < self.last_timestamp_s
        ):
            self.reset()
            self.last_timestamp_s = now_s
            self.last_reason = 'source_time_reset'
            return self.decision(now_s)
        self.last_timestamp_s = now_s

        lane = self.normalize_lane(obstacle_lane)
        event_kind = self.normalize_obstacle_kind(obstacle_kind)
        if speed_trigger_allowed is None:
            # Compatibility for callers which predate the APPROACH phase.
            speed_trigger_allowed = trigger_allowed
        if blocked:
            was_active = self.state != DynamicLaneEventState.IDLE
            self.reset()
            self.last_timestamp_s = now_s
            self.last_reason = (
                'blocked_event_cancelled' if was_active else 'blocked'
            )
            return self.decision(now_s)

        if self.state == DynamicLaneEventState.IDLE:
            eligible = (
                speed_trigger_allowed
                and object_present
                and event_kind != 'unknown'
            )
            avoid_eligible = (
                trigger_allowed
                and object_present
                and event_kind != 'unknown'
                and lane in ('left', 'right')
            )
            speed_confirmed, speed_matches = (
                self.record_speed_confirmation(eligible, event_kind))
            avoid_confirmed, avoid_matches = (
                self.record_avoid_confirmation(
                    avoid_eligible,
                    event_kind,
                    lane,
                    obstacle_bbox,
                )
            )
            if speed_confirmed:
                self.event_kind = event_kind
                self.state_entered_s = now_s
                self.obstacle_lane = (
                    lane if lane != 'unknown' else 'unknown')
                self.target_lane = 'center'
                self.active_command = 'reset'
                self.clear_count = 0
                if avoid_confirmed:
                    self.start_active(now_s, lane)
                else:
                    self.state = DynamicLaneEventState.APPROACH
                    self.last_reason = f'{event_kind}_approach_started'
            elif eligible:
                matches_required, window_frames = (
                    self.speed_confirmation_for(event_kind))
                self.last_reason = (
                    f'{event_kind}_approach_confirming_'
                    f'{speed_matches}_of_{matches_required}_in_'
                    f'{window_frames}')
            elif avoid_eligible:
                matches_required, window_frames = (
                    self.avoid_confirmation_for(event_kind))
                self.last_reason = (
                    f'{event_kind}_avoidance_buffering_'
                    f'{avoid_matches}_of_{matches_required}_in_'
                    f'{window_frames}')
            else:
                self.last_reason = (
                    'obstacle_not_eligible'
                    if object_present else 'waiting_for_obstacle'
                )

        elif self.state == DynamicLaneEventState.APPROACH:
            same_event = object_present and event_kind == self.event_kind
            if not same_event:
                self.record_avoid_confirmation(
                    False,
                    self.event_kind,
                    'unknown',
                    None,
                )
                self.clear_count += 1
                if self.clear_count >= self.rearm_clear_frames_for(
                        self.event_kind):
                    self.reset()
                    self.last_timestamp_s = now_s
                    self.last_reason = 'approach_cancelled_after_clear'
                else:
                    self.last_reason = 'holding_approach_through_dropout'
            else:
                self.clear_count = 0
                close_and_located = (
                    trigger_allowed and lane in ('left', 'right'))
                avoid_confirmed, avoid_matches = (
                    self.record_avoid_confirmation(
                        close_and_located,
                        event_kind,
                        lane,
                        obstacle_bbox,
                    )
                )
                if avoid_confirmed:
                    self.start_active(now_s, lane)
                elif close_and_located:
                    matches_required, window_frames = (
                        self.avoid_confirmation_for(self.event_kind))
                    self.last_reason = (
                        f'{self.event_kind}_avoidance_confirming_'
                        f'{avoid_matches}_of_{matches_required}_in_'
                        f'{window_frames}')
                else:
                    self.last_reason = f'{self.event_kind}_approaching'

        elif self.state == DynamicLaneEventState.ACTIVE:
            elapsed_s = max(0.0, now_s - self.state_entered_s)
            if elapsed_s >= self.duration_for(self.event_kind):
                self.state = DynamicLaneEventState.WAIT_CLEAR
                self.state_entered_s = now_s
                self.clear_count = 0
                self.last_reason = f'{self.event_kind}_event_complete'
            else:
                self.last_reason = 'holding_opposite_lane'

        elif self.state == DynamicLaneEventState.WAIT_CLEAR:
            if new_target_present:
                # LiDAR/camera tracking has positively separated a new target;
                # it need not disappear from YOLO before starting its own
                # ordinary N-of-M confirmation sequence.
                self.reset()
                self.last_timestamp_s = now_s
                self.last_reason = 'rearmed_for_new_target'
            else:
                self.clear_count = (
                    self.clear_count + 1 if not object_present else 0
                )
            if (
                self.state == DynamicLaneEventState.WAIT_CLEAR
                and self.clear_count >= self.rearm_clear_frames_for(
                    self.event_kind)
            ):
                self.reset()
                self.last_timestamp_s = now_s
                self.last_reason = 'rearmed_after_obstacle_cleared'
            elif self.state == DynamicLaneEventState.WAIT_CLEAR:
                self.last_reason = 'waiting_for_obstacle_to_clear'

        return self.decision(now_s)
