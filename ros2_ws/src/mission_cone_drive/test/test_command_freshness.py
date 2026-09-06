from mission_cone_drive.mission_manager_node import (
    MissionManagerNode,
    TimedMotorCommand,
)
import rclpy
from std_msgs.msg import Bool
from std_srvs.srv import Trigger


def test_timestamp_freshness_rejects_zero_stale_and_far_future():
    now_ns = 10_000_000_000
    assert not MissionManagerNode.timestamp_is_fresh(0, now_ns, 0.8, 0.05)
    assert not MissionManagerNode.timestamp_is_fresh(
        now_ns - 900_000_000, now_ns, 0.8, 0.05)
    assert not MissionManagerNode.timestamp_is_fresh(
        now_ns + 60_000_000, now_ns, 0.8, 0.05)
    assert MissionManagerNode.timestamp_is_fresh(
        now_ns - 100_000_000, now_ns, 0.8, 0.05)


def test_stamped_lane_command_requires_fresh_command_and_source():
    rclpy.init()
    node = MissionManagerNode()
    try:
        now_ns = node.now_ns()
        command = TimedMotorCommand(
            angle=10.0,
            speed=4.0,
            received_ns=now_ns,
            command_ns=now_ns,
            source_ns=now_ns,
            state_age_s=0.0,
            is_stamped=True,
        )
        assert node.command_is_fresh(command, now_ns, True)

        command.source_ns = now_ns - int(1.0e9)
        assert not node.command_is_fresh(command, now_ns, True)

        command.source_ns = now_ns
        command.command_ns = now_ns - int(1.0e9)
        assert not node.command_is_fresh(command, now_ns, True)

        command.command_ns = 0
        assert not node.command_is_fresh(command, now_ns, True)
        assert node.command_is_fresh(command, now_ns, False)
    finally:
        node.destroy_node()
        rclpy.shutdown()


def test_start_service_requires_vesc_and_fresh_lane_command():
    rclpy.init()
    node = MissionManagerNode()
    try:
        response = node.start_callback(
            Trigger.Request(), Trigger.Response())
        assert not response.success
        assert 'VESC not ready' in response.message
        assert 'fresh lane command unavailable' in response.message
        assert not node.active

        now_ns = node.now_ns()
        node.lane_command = TimedMotorCommand(
            angle=0.0,
            speed=4.0,
            received_ns=now_ns,
            command_ns=now_ns,
            source_ns=now_ns,
            is_stamped=True,
        )
        node.vesc_ready_callback(Bool(data=True))
        response = node.start_callback(
            Trigger.Request(), Trigger.Response())
        assert response.success
        assert node.active
        assert node.mode == node.LANE_MODE
    finally:
        node.destroy_node()
        rclpy.shutdown()


def test_stop_resets_run_state_and_requires_new_lane_command():
    rclpy.init()
    node = MissionManagerNode()
    try:
        now_ns = node.now_ns()
        node.active = True
        node.mode = node.CONE_MODE
        node.cone_completed = True
        node.cone_confirm_cycles = 3
        node.cone_suspect_until_ns = now_ns + int(1e9)
        node.cone_prearm_until_ns = now_ns + int(1e9)
        node.cone_prearm_votes.extend([True, True])
        node.lane_override = 'go_left'
        node.traffic_state = 'red'
        node.lane_command = TimedMotorCommand(
            received_ns=now_ns,
            command_ns=now_ns,
            source_ns=now_ns,
            is_stamped=True,
        )

        response = node.stop_callback(
            Trigger.Request(), Trigger.Response())
        assert response.success
        assert not node.active
        assert node.mode == node.LANE_MODE
        assert not node.cone_completed
        assert node.cone_confirm_cycles == 0
        assert node.cone_suspect_until_ns == 0
        assert node.cone_prearm_until_ns == 0
        assert not node.cone_prearm_votes
        assert node.lane_override == 'reset'
        assert node.traffic_state == 'unknown'
        assert node.lane_command.received_ns == 0
    finally:
        node.destroy_node()
        rclpy.shutdown()
