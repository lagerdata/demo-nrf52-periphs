# pylint: disable=redefined-outer-name,missing-function-docstring

import sys
import time
import pytest
import serial
import pigpio
from lager import lager


LED_PIN = 0

@pytest.fixture
def serial_port():
    with serial.Serial('/dev/ttyACM0', 115200, timeout=1) as ser:
        yield ser

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

def test_led_on(gateway, device, serial_port):
    device.run()
    serial_port.write(b'l')
    data = serial_port.read(32)
    #led_state = gateway.gpio.read(LED_PIN)
    led_state = gateway.pi.read(6)
    assert led_state == 0

def test_led_off(gateway, device, serial_port):
    device.run()
    serial_port.write(b'k')
    data = serial_port.read(32)
    #led_state = gateway.gpio.read(LED_PIN)
    led_state = gateway.pi.read(6)
    assert led_state == 2

if __name__ == '__main__':
    sys.exit(pytest.main([__file__]))
