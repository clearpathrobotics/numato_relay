"""Check parameter initialization and interface setup without relay hardware."""

import ast
import inspect
from unittest.mock import Mock, call

from numato_relay import numato_relay

from numato_relay_interfaces.msg import GpioStates, RelayStates
from numato_relay_interfaces.srv import GetRelay, SetRelay, ToggleRelay

import pytest

import rclpy
from rclpy.serialization import deserialize_message, serialize_message


@pytest.fixture
def relay_factory(monkeypatch):
    """Create relay nodes with a mocked serial connection and clean them up."""
    serial = Mock()
    monkeypatch.setattr(numato_relay.serial, 'Serial', serial)
    nodes = []

    def create(args):
        rclpy.init(args=args)
        node = numato_relay.NumatoRelay()
        nodes.append(node)
        return node, serial

    yield create

    for node in nodes:
        node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()


@pytest.mark.parametrize('overrides', [
    {},
    {
        'port': '/dev/test-relay',
        'baud': 9600,
        'toggle_duration': 0.25,
        'relay_timer_period': 0.5,
        'gpio_timer_period': 4.5,
    },
])
def test_initialization(relay_factory, overrides):
    """Load every declared driver parameter and preserve interface defaults."""
    expected = {
        'port': '/dev/ttyACM0',
        'baud': 19200,
        'toggle_duration': 2.0,
        'relay_timer_period': 1.0,
        'gpio_timer_period': 3.0,
    }
    expected.update(overrides)
    args = ['--ros-args']
    for name, value in overrides.items():
        args.extend(['-p', f'{name}:={value}'])

    node, serial = relay_factory(args)

    for name, value in expected.items():
        assert node.get_parameter(name).value == value
        assert getattr(node, name) == value
    serial.assert_called_once_with(expected['port'], expected['baud'], timeout=1)
    assert node.relay_timer.timer_period_ns == int(expected['relay_timer_period'] * 1e9)
    assert node.gpio_timer.timer_period_ns == int(expected['gpio_timer_period'] * 1e9)
    assert node.relay_state_array == []
    assert node.gpio_state_array == []
    assert node.relay_publisher.topic_name == '/numato_relay/relay_states'
    assert node.gpio_publisher.topic_name == '/numato_relay/gpio_states'
    assert node.relay_publisher.msg_type is RelayStates
    assert node.gpio_publisher.msg_type is GpioStates
    for name in ('set_relay', 'get_relay', 'toggle_relay'):
        service = getattr(node, f'service_{name}')
        assert node.resolve_service_name(service.srv_name) == f'/numato_relay/{name}'


def test_toggle_uses_initialized_duration(relay_factory, monkeypatch):
    """Use the startup duration without accessing ROS parameters again."""
    node, _ = relay_factory(['--ros-args', '-p', 'toggle_duration:=0.25'])
    node.relay_state_array = [True]
    sleep = Mock()
    monkeypatch.setattr(numato_relay.time, 'sleep', sleep)
    monkeypatch.setattr(
        node, 'get_parameter', Mock(side_effect=AssertionError('Unexpected parameter lookup')))

    response = node.toggle_relay(
        ToggleRelay.Request(relay_channel=0, relay_state=True),
        ToggleRelay.Response(),
    )

    assert response.success
    assert response.relay_state
    assert node.toggle_duration == 0.25
    sleep.assert_called_once_with(0.25)


def test_parameter_access_is_only_in_init():
    """Keep all explicit parameter reads and writes inside the initializer."""
    source = ast.parse(inspect.getsource(numato_relay.NumatoRelay))
    for method in source.body[0].body:
        if not isinstance(method, ast.FunctionDef) or method.name == '__init__':
            continue
        for expression in ast.walk(method):
            if not isinstance(expression, ast.Call):
                continue
            if not isinstance(expression.func, ast.Attribute):
                continue
            assert expression.func.attr not in {
                'get_parameter', 'get_parameters', 'get_parameter_or',
                'get_parameters_by_prefix', 'set_parameter', 'set_parameters',
                'set_parameters_atomically',
            }, f'{method.name} accesses ROS parameters outside __init__'


@pytest.mark.parametrize('channel_count', [0, 1, 4, 8, 16, 32])
@pytest.mark.parametrize('kind,message_type', [('relay', RelayStates), ('gpio', GpioStates)])
def test_state_array_publishing(relay_factory, monkeypatch, channel_count, kind, message_type):
    """Publish every channel in order, including boards larger than eight channels."""
    node, _ = relay_factory([])
    states = [index % 3 == 0 for index in range(channel_count)]
    setattr(node, f'{kind}_state_array', list(states))
    publisher = Mock()
    monkeypatch.setattr(node, f'{kind}_publisher', publisher)
    update_gpio = Mock()
    monkeypatch.setattr(node, 'update_gpio', update_gpio)

    getattr(node, f'{kind}_timer_callback')()

    publisher.publish.assert_called_once()
    message, = publisher.publish.call_args.args
    assert isinstance(message, message_type)
    assert list(message.states) == states
    decoded = deserialize_message(serialize_message(message), message_type)
    assert list(decoded.states) == states
    if kind == 'gpio':
        update_gpio.assert_called_once_with()
    else:
        update_gpio.assert_not_called()

    getattr(node, f'{kind}_state_array').append(True)
    assert list(message.states) == states


def test_gpio_updates_preserve_channel_count(relay_factory):
    """Refresh GPIO values without inserting extra array entries."""
    node, serial = relay_factory([])
    node.gpio_state_array = [False, True]
    serial.return_value.read.side_effect = [
        b'gpio read 0\r\n1\r\n>', b'gpio read 1\r\n0\r\n>',
    ] * 2

    for _ in range(2):
        node.update_gpio()
        assert node.gpio_state_array == [True, False]


def test_serial_query(relay_factory):
    """Keep the write, fixed-size read, and flush locked and in order."""
    node, serial = relay_factory([])
    port = serial.return_value
    response = b'relay read 0\r\non\r\n>'
    port.read.return_value = response

    def check_locked(*args):
        assert node.serial_lock.locked()

    def read_locked(size):
        check_locked(size)
        return response

    port.write.side_effect = check_locked
    port.read.side_effect = read_locked
    port.flush.side_effect = check_locked

    assert node.serial_query('relay read 0 \n\r') == str(response)

    assert port.mock_calls == [
        call.write(b'relay read 0 \n\r'),
        call.read(node.SERIAL_READ_SIZE),
        call.flush(),
    ]
    assert not node.serial_lock.locked()


@pytest.mark.parametrize('operation', ['write', 'read', 'flush'])
def test_serial_query_releases_lock_on_error(relay_factory, operation):
    """Propagate serial failures while releasing the lock for later queries."""
    node, serial = relay_factory([])
    port = serial.return_value
    getattr(port, operation).side_effect = numato_relay.serial.SerialException('Serial failure')

    with pytest.raises(numato_relay.serial.SerialException, match='Serial failure'):
        node.serial_query('relay read 0 \n\r')

    assert not node.serial_lock.locked()
    getattr(port, operation).side_effect = None
    port.read.return_value = b'off'
    assert node.serial_query('relay read 0 \n\r') == str(b'off')


@pytest.mark.parametrize('kind,responses', [
    ('relay', [b'on', b'off', b'>']),
    ('gpio', [b'gpio read 0\r\n1\r\n>', b'gpio read 1\r\n0\r\n>', b'>']),
])
def test_channel_discovery_uses_serial_queries(relay_factory, kind, responses):
    """Preserve discovery commands, response parsing, and end-of-channel detection."""
    node, serial = relay_factory([])
    port = serial.return_value
    port.read.side_effect = responses

    getattr(node, f'read_{kind}')()

    assert getattr(node, f'{kind}_state_array') == [True, False]
    assert port.write.call_args_list == [
        call(f'{kind} read {index} \n\r'.encode('utf-8')) for index in range(3)
    ]
    assert port.read.call_args_list == [call(node.SERIAL_READ_SIZE)] * 3
    assert port.flush.call_count == 3


@pytest.mark.parametrize('response,expected', [
    (b'gpio read 2\r\n1\r\n>', True),
    (b'gpio read 2\r\n0\r\n>', False),
    (b'>', None),
    (b'', None),
])
def test_read_gpio_channel(relay_factory, response, expected):
    """Distinguish high, low, and unrecognized responses without changing stored state."""
    node, serial = relay_factory([])
    serial.return_value.read.return_value = response
    node.gpio_state_array = [True, False]

    assert node.read_gpio_channel(2) is expected

    serial.return_value.write.assert_called_once_with(b'gpio read 2 \n\r')
    assert node.gpio_state_array == [True, False]


def test_gpio_update_stops_at_unrecognized_channel(relay_factory):
    """Keep remaining channels unchanged when polling stops early."""
    node, serial = relay_factory([])
    node.gpio_state_array = [False, True, False]
    serial.return_value.read.side_effect = [b'gpio read 0\r\n1\r\n>', b'>']

    node.update_gpio()

    assert node.gpio_state_array == [True, True, False]
    assert serial.return_value.read.call_count == 2


@pytest.mark.parametrize('operation,service', [
    ('get_relay', GetRelay), ('set_relay', SetRelay), ('toggle_relay', ToggleRelay),
])
@pytest.mark.parametrize('states,channel', [
    ([], 0), ([True, False], 2), ([True], 255), ([True], 2**32 - 1),
])
def test_callbacks_reject_invalid_channels(
        relay_factory, monkeypatch, operation, service, states, channel):
    """Return consistent errors without serial access or relay state changes."""
    node, serial = relay_factory([])
    node.relay_state_array = list(states)
    logger = Mock()
    monkeypatch.setattr(node, 'get_logger', lambda: logger)
    response = service.Response(relay_state=True)
    response.success = True
    response.message = 'Previous response'

    result = getattr(node, operation)(service.Request(relay_channel=channel), response)

    assert result is response
    assert not result.relay_state
    assert not result.success
    assert result.message == (
        'The requested relay_channel is larger than what this Relay PCBA supports')
    logger.warning.assert_called_once()
    warning, = logger.warning.call_args.args
    assert operation in warning
    assert f'relay_channel {channel}' in warning
    assert node.relay_state_array == states
    assert serial.return_value.mock_calls == []


@pytest.mark.parametrize('operation,service', [
    ('get_relay', GetRelay), ('set_relay', SetRelay), ('toggle_relay', ToggleRelay),
])
@pytest.mark.parametrize('initial', [False, True])
def test_callbacks_accept_valid_channels(relay_factory, monkeypatch, operation, service, initial):
    """Preserve service results and serial writes for valid channels."""
    node, serial = relay_factory([])
    node.relay_state_array = [False, initial]
    sleep = Mock()
    monkeypatch.setattr(numato_relay.time, 'sleep', sleep)
    request = service.Request(relay_channel=1)
    if service is not GetRelay:
        request.relay_state = True

    result = getattr(node, operation)(request, service.Response())

    assert result.success
    assert result.message
    if service is GetRelay:
        assert result.relay_state is initial
        assert serial.return_value.mock_calls == []
    else:
        assert result.success
        assert result.relay_state
        assert result.message
        commands = []
        if operation == 'toggle_relay' and initial:
            commands = [call(b'relay off 1\n\r'), call(b'relay on 1\n\r')]
            sleep.assert_called_once_with(node.toggle_duration)
        elif not initial:
            commands = [call(b'relay on 1\n\r')]
        assert serial.return_value.write.call_args_list == commands
    if operation != 'toggle_relay' or not initial:
        sleep.assert_not_called()


@pytest.mark.parametrize('operation,service', [
    ('get_relay', GetRelay), ('set_relay', SetRelay), ('toggle_relay', ToggleRelay),
])
def test_service_contract_and_large_channel(relay_factory, operation, service):
    """Round-trip the common response and a channel above the old uint8 limit."""
    node, _ = relay_factory([])
    node.relay_state_array = [False] * 301
    request = service.Request(relay_channel=300)
    if service is not GetRelay:
        request.relay_state = True
    request = deserialize_message(serialize_message(request), service.Request)

    result = getattr(node, operation)(request, service.Response())
    result = deserialize_message(serialize_message(result), service.Response)

    assert service.Request.get_fields_and_field_types()['relay_channel'] == 'uint32'
    assert service.Response.get_fields_and_field_types() == {
        'success': 'boolean', 'relay_state': 'boolean', 'message': 'string',
    }
    assert result.success
    assert result.relay_state is (service is not GetRelay)
    assert result.message


@pytest.mark.parametrize('operation,service,initial,failed_write', [
    ('set_relay', SetRelay, False, 1),
    ('toggle_relay', ToggleRelay, False, 1),
    ('toggle_relay', ToggleRelay, True, 1),
    ('toggle_relay', ToggleRelay, True, 2),
])
@pytest.mark.parametrize('stage', ['write', 'flush'])
@pytest.mark.parametrize('error_type', [
    numato_relay.serial.SerialException,
    numato_relay.serial.SerialTimeoutException,
    OSError,
])
def test_callbacks_report_serial_errors(
        relay_factory, monkeypatch, operation, service, initial, failed_write, stage, error_type):
    """Fail explicitly and keep the last completed command's tracked state."""
    node, serial = relay_factory([])
    node.relay_state_array = [initial]
    monkeypatch.setattr(numato_relay.time, 'sleep', Mock())
    logger = Mock()
    monkeypatch.setattr(node, 'get_logger', lambda: logger)
    getattr(serial.return_value, stage).side_effect = (
        [None] * (failed_write - 1) + [error_type('Disconnected')])

    result = getattr(node, operation)(
        service.Request(relay_channel=0, relay_state=True),
        service.Response(success=True, relay_state=True, message='Previous response'),
    )

    assert not result.success
    assert not result.relay_state
    assert result.message == 'Serial communication failed: Disconnected'
    assert node.relay_state_array == [False if failed_write == 2 else initial]
    assert not node.serial_lock.locked()
    logger.error.assert_called_once()
    assert operation in logger.error.call_args.args[0]


def test_callback_does_not_hide_programming_errors(relay_factory, monkeypatch):
    """Only expected I/O failures are converted into failed service responses."""
    node, _ = relay_factory([])
    node.relay_state_array = [False]
    monkeypatch.setattr(node, 'write_relay', Mock(side_effect=RuntimeError('Unexpected failure')))

    with pytest.raises(RuntimeError, match='Unexpected failure'):
        node.set_relay(
            SetRelay.Request(relay_channel=0, relay_state=True), SetRelay.Response())
