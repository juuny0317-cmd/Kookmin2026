from types import SimpleNamespace

import pytest

from mission_cone_drive.integrated_service_proxy import (
    IntegratedServiceProxy,
    parse_trigger_response,
)


def test_parse_success_response_with_trailing_warning():
    output = (
        'response:\n'
        'std_srvs.srv.Trigger_Response(success=True, '
        "message='Integrated drive started in LANE mode.')\n"
        'failed to shutdown: context already closed\n'
    )
    assert parse_trigger_response(output) == (
        True,
        'Integrated drive started in LANE mode.',
    )


def test_parse_rejected_response():
    output = (
        "std_srvs.srv.Trigger_Response(success=False, "
        "message='Start rejected: VESC not ready.')"
    )
    assert parse_trigger_response(output) == (
        False,
        'Start rejected: VESC not ready.',
    )


def test_parse_fast_client_json_response():
    output = (
        'warning before result\n'
        'XYCAR_TRIGGER_RESPONSE={"success": true, "message": "started"}\n'
    )
    assert parse_trigger_response(output) == (True, 'started')


def test_parse_missing_response_rejected():
    with pytest.raises(ValueError):
        parse_trigger_response('waiting for service')


@pytest.mark.parametrize(
    'name',
    ['/start_integrated_drive', '/_internal/start_2'],
)
def test_valid_absolute_service_names(name):
    assert IntegratedServiceProxy.validate_service_name(name) == name


@pytest.mark.parametrize('name', ['', 'relative', '/bad name', '/bad;name'])
def test_invalid_service_names(name):
    with pytest.raises(ValueError):
        IntegratedServiceProxy.validate_service_name(name)


def test_bridge_command_includes_all_drive_control_services():
    proxy = SimpleNamespace(
        internal_start_service='/internal/start',
        internal_stop_service='/internal/stop',
        internal_pause_service='/internal/pause',
        internal_resume_service='/internal/resume',
        internal_toggle_pause_service='/internal/toggle',
    )

    command = IntegratedServiceProxy.bridge_command(proxy, 'container-id')

    assert command[4] == '/opt/mission_overlay_entrypoint.sh'
    assert command[-5:] == [
        '/internal/start',
        '/internal/stop',
        '/internal/pause',
        '/internal/resume',
        '/internal/toggle',
    ]
