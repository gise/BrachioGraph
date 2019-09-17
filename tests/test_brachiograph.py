from unittest import mock

import pytest

import brachiograph

@pytest.fixture
def br():
    with mock.patch("pigpio.pi"), mock.patch("brachiograph.sleep"):
        yield brachiograph.BrachioGraph(9, 9)


def test_plot_file(br):
    pass

def test_plot_lines(br):
    line1 = [[1,1], [2,2]]
    line2 = [[3,3], [4,4]]
    br.plot_lines([line1, line2], bounds=[-8,3,8,10])

def test_draw(br):
    br.draw(3,4)

def test_draw_test_pattern(br):
    br.test_pattern(bounds=[-8,3,8,10])

def test_box(br):
    br.box(bounds=[-8,3,8,10])

def test_centre(br):
    br.centre()

def test_xy(br):
    br.xy(3, 4)

def test_set_angles(br):
    pass

def test_naive_angles_to_pulse_width(br):
    pass

def test_angles_to_pulse_width(br):
    pass

def test_set_pulse_widths(br):
    pass

def test_get_pulse_widths(br):
    pass

def test_park(br):
    pass

def test_quiet(br):
    pass

def test_xy_to_angles(br):
    pass

def test_angles_to_xy(br):
    x, y = br.angles_to_xy(0, 90)
    assert x == pytest.approx(9)
    assert y == pytest.approx(9)

# no tests for drive / drive_xy, which are interactive
