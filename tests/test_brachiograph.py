from unittest import mock

import pytest

import brachiograph

@pytest.fixture
def br():
    with mock.patch("pigpio.pi"):
        with mock.patch("brachiograph.sleep"):
            br = brachiograph.BrachioGraph(9, 9)
        yield br


def test_plot_file():
    pass

def test_plot_lines():
    pass

def test_draw(br):
    with mock.patch("brachiograph.sleep"):
        br.draw(3,4)

def test_draw_test_pattern(br):
    with mock.patch("brachiograph.sleep"):
        br.test_pattern()

def test_box(br):
    with mock.patch("brachiograph.sleep"):
        br.box()

def test_centre():
    pass

def test_xy():
    pass

def test_set_angles():
    pass


def test_naive_angles_to_pulse_width():
    pass

def test_angles_to_pulse_width():
    pass

def test_set_pulse_widths():
    pass

def test_get_pulse_widths():
    pass

def test_park():
    pass

def test_quiet():
    pass

def test_xy_to_angles():
    pass

def test_angles_to_xy():
    pass

# no tests for drive / drive_xy, which are interactive
