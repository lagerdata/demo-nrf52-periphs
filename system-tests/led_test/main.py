# pylint: disable=redefined-outer-name,missing-function-docstring

import sys
import time
import pytest
import serial
from pexpect_serial import SerialSpawn
import pigpio
from lager import lager


LED_PIN = 0

@pytest.fixture
def serial_spawn():
    with serial.Serial('/dev/ttyACM0', 115200, timeout=1) as ser:
        yield SerialSpawn(ser)

@pytest.fixture(scope='module')
def gateway():
    gw = lager.Lager()
    gw.gpio.set_mode(LED_PIN, pigpio.INPUT)
    yield gw
    gw.close()

@pytest.fixture(scope='module')
def device(gateway):
    device = gateway.connect("nrf52", interface="ftdi", transport="swd", speed="4000", force=True)
    device.reset(halt=True)
    hexfiles = [lager.Hexfile("app.hex")]
    device.flash(hexfiles=hexfiles)
    yield device
    device.close()

def test_led_on(gateway, device, serial_spawn):
    device.run()
    serial_spawn.sendline('l')
    serial_spawn.expect('Turning on LED 0.')
    led_state = gateway.gpio.read(LED_PIN)
    assert led_state == 1

def test_led_off(gateway, device, serial_spawn):
    device.run()
    serial_spawn.sendline('k')
    serial_spawn.expect('Turning off LED 0.')
    led_state = gateway.gpio.read(LED_PIN)
    assert led_state == 1

if __name__ == '__main__':
    sys.exit(pytest.main([__file__]))
