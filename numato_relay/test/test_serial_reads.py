"""Verify serial read transactions without relay hardware."""

from unittest.mock import Mock, call

from numato_relay.numato_relay import NumatoRelay

import pytest

import rclpy

from serial import SerialException


@pytest.fixture
def relay(monkeypatch):
    """Create a node with a mocked serial port."""
    port = Mock()
    monkeypatch.setattr('numato_relay.numato_relay.serial.Serial', Mock(return_value=port))
    rclpy.init(args=[])
    node = None
    try:
        node = NumatoRelay()
        yield node, port
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()


def test_serial_query(relay):
    """Keep write, fixed-size read, and flush locked and ordered."""
    node, port = relay
    response = b'relay read 0\r\non\r\n>'

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
def test_serial_query_failure(relay, operation):
    """Propagate failures and release the lock so subsequent queries work."""
    node, port = relay
    error = SerialException('Serial failure')
    getattr(port, operation).side_effect = error

    with pytest.raises(SerialException) as caught:
        node.serial_query('gpio read 0 \n\r')

    assert caught.value is error
    assert not node.serial_lock.locked()
    getattr(port, operation).side_effect = None
    port.read.return_value = b'off'
    assert node.serial_query('relay read 0 \n\r') == str(b'off')


@pytest.mark.parametrize('kind,responses', [
    ('relay', [b'on', b'off', b'>']),
    ('gpio', [b'gpio read 0\r\n1\r\n>', b'gpio read 1\r\n0\r\n>', b'>']),
])
def test_discovery_reads(relay, kind, responses):
    """Preserve discovery commands, parsing, and termination."""
    node, port = relay
    port.read.side_effect = responses

    getattr(node, f'read_{kind}')()

    assert getattr(node, f'{kind}_state_array') == [True, False]
    assert port.write.call_args_list == [
        call(f'{kind} read {index} \n\r'.encode('utf-8')) for index in range(3)
    ]
    assert port.read.call_args_list == [call(node.SERIAL_READ_SIZE)] * 3
    assert port.flush.call_count == 3


def test_gpio_polling_reads(relay):
    """Preserve polling commands and existing insertion behavior."""
    node, port = relay
    node.gpio_state_array = [False, True]
    port.read.side_effect = [b'gpio read 0\r\n1\r\n>', b'gpio read 1\r\n0\r\n>']

    node.update_gpio()

    assert port.write.call_args_list == [
        call(b'gpio read 0 \n\r'), call(b'gpio read 1 \n\r'),
    ]
    assert port.read.call_args_list == [call(node.SERIAL_READ_SIZE)] * 2
    assert port.flush.call_count == 2
    assert node.gpio_state_array == [True, False, False, True]
