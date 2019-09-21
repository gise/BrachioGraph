# coding=utf-8

from time import sleep
import readchar
import math
import numpy
import json

import pigpio
import tqdm


class Servo:
    def __init__(self, rpi, gpio_pin, angle_pws=(), zero_pw=1500):
        if angle_pws:
            array = numpy.array(angle_pws)
            self.angles_to_pw = numpy.poly1d(numpy.polyfit(array[:, 0], array[:, 1], 3))

        else:
            self.angles_to_pw = self._naive_angles_to_pulse_widths
            self.zero = zero_pw

        self.rpi = rpi
        self.gpio_pin = gpio_pin
        self.angle = None  # Unknown until set

        # the pulse frequency should be no higher than 100Hz - higher values could (supposedly) damage the servos
        rpi.set_PWM_frequency(gpio_pin, 50)

    def quiet(self):
        """Stop sending pulses to the servo"""
        self.rpi.set_servo_pulsewidth(self.gpio_pin, 0)

    def set_pulse_width(self, pw):
        """Manually set pulse width"""
        self.rpi.set_servo_pulsewidth(self.gpio_pin, pw)
        self.angle = None  # We don't know the angle if this is set manually

    def get_pulse_width(self):
        """
        Get current pulse width
        """
        return self.rpi.get_servo_pulsewidth(self.gpio_pin)

    def set_angle(self, new_angle, wait=0.3):
        """
        Change the servo angle

        new_angle: desired servo angle
        wait: amount of seconds to wait for the servo to stabilize
        """
        self.rpi.set_servo_pulsewidth(self.gpio_pin, self.angles_to_pw(new_angle))
        sleep(wait)
        self.angle = new_angle

    def _naive_angles_to_pulse_widths(self, angle):
        """
        Approximate pulse width with linear function

        angle: angle of servo
        """
        return (angle + 90) * 10 + self.zero


class BrachioGraphBase:

    # bounds: ...
    # pen: ...
    # shoulder: ...
    # elbow: ...

    # ----------------- drawing methods -----------------

    def plot_file(self, filename, wait=0.1, interpolate=10, bounds=None):
        """
        Plots json file input

        filename: the path to the file that should be plotted
        wait: seconds waited per centimeter
        interpolate: each centimeter is split into interpolate steps
        bounds: the box bounds describe a rectangle that we can safely draw in
                (minimum x, minimum y, maximum x, maximum y)
        """

        bounds = bounds or self.bounds

        if not bounds:
            raise TypeError("Missing bounds parameter (or missing bounds configuration")

        with open(filename, "r") as line_file:
            lines = json.load(line_file)

        self.plot_lines(
            lines=lines, wait=wait, interpolate=interpolate, bounds=bounds, flip=True
        )

    def plot_lines(
        self, lines=(), wait=0.1, interpolate=10, rotate=False, flip=False, bounds=None
    ):
        """
        Plots all line segments described in lines

        lines: a list of lines. Where each line is a list of points.
            Where each point is a pair of numbers
        wait: seconds waited per centimeter
        interpolate: each centimeter is split into interpolate steps
        rotate: should image be rotated 90 degrees to the right
        flip: should image be flipped over x axis
        bounds: the box bounds describe a rectangle that we can safely draw in
                (minimum x, minimum y, maximum x, maximum y)
        """
        bounds = bounds or self.bounds

        if not bounds:
            raise TypeError("Missing bounds parameter (or missing bounds configuration")

        # lines is a tuple itself containing a number of tuples, each of which contains a number of 2-tuples
        #
        # [                                                                                     # |
        #     [                                                                                 # |
        #         [3, 4],                               # |                                     # |
        #         [2, 4],                               # |                                     # |
        #         [1, 5],  #  a single point in a line  # |  a list of points defining a line   # |
        #         [3, 5],                               # |                                     # |
        #         [3, 7],                               # |                                     # |
        #     ],                                                                                # |
        #     [                                                                                 # |  all the lines
        #         [...],                                                                        # |
        #         [...],                                                                        # |
        #     ],                                                                                # |
        #     [                                                                                 # |
        #         [...],                                                                        # |
        #         [...],                                                                        # |
        #     ],                                                                                # |
        # ]                                                                                     # |

        # First, we create a pair of empty sets for all the x and y values in all of the lines of the plot data.

        x_values_in_lines = set()
        y_values_in_lines = set()

        # Loop over each line and all the points in each line, to get sets of all the x and y values:

        for line in lines:

            x_values_in_line, y_values_in_line = zip(*line)

            x_values_in_lines.update(x_values_in_line)
            y_values_in_lines.update(y_values_in_line)

        # Identify the minimum and maximum values.

        min_x, max_x = min(x_values_in_lines), max(x_values_in_lines)
        min_y, max_y = min(y_values_in_lines), max(y_values_in_lines)

        # Identify the range they span.

        x_range = max_x - min_x
        y_range = max_y - min_y

        x_mid_point = (max_x + min_x) / 2
        y_mid_point = (max_y + min_y) / 2

        box_x_range = bounds[2] - bounds[0]
        box_y_range = bounds[3] - bounds[1]

        box_x_mid_point = (bounds[0] + bounds[2]) / 2
        box_y_mid_point = (bounds[1] + bounds[3]) / 2

        # Get a 'divider' value for each range - the value by which we must divide all x and y so that they will
        # fit safely inside the drawing range of the plotter.

        # If both image and box are in portrait orientation, or both in landscape, we don't need to rotate the plot.

        if (x_range >= y_range and box_x_range >= box_y_range) or (
            x_range <= y_range and box_x_range <= box_y_range
        ):

            divider = max((x_range / box_x_range), (y_range / box_y_range))
            rotate = False

        else:

            divider = max((x_range / box_y_range), (y_range / box_x_range))
            rotate = True
            x_mid_point, y_mid_point = y_mid_point, x_mid_point

        # Now, divide each value, and take into account the offset from zero of each range

        for line in lines:

            for point in line:
                if rotate:
                    point[0], point[1] = point[1], point[0]

                x = point[0]
                x = (
                    x - x_mid_point
                )  # shift x values so that they have zero as their mid-point
                x = x / divider  # scale x values to fit in our box width
                x = (
                    x + box_x_mid_point
                )  # shift x values so that they have the box x midpoint as their endpoint

                if flip ^ rotate:
                    x = -x

                point[0] = x

                y = point[1]
                y = y - y_mid_point
                y = y / divider
                y = y + box_y_mid_point

                point[1] = y

        for line in tqdm.tqdm(lines, desc="Lines", leave=False):
            x, y = line[0]
            self.xy(x, y)
            for point in tqdm.tqdm(line[1:], desc="Segments", leave=False):
                x, y = point
                self.draw(x, y, wait=wait, interpolate=interpolate)

        self.park()
        self.quiet()

    def draw(self, x=0, y=0, wait=0.5, interpolate=10):
        """
        Draw a line from current position to x, y

        x: position to draw on x axis
        y: position to draw on y axis
        wait: seconds waited per centimeter
        interpolate: each centimeter is split into interpolate steps
        """
        self.xy(x=x, y=y, wait=wait, interpolate=interpolate, draw=True)

    def test_pattern(self, bounds=None, wait=1, interpolate=10, repeat=1):
        """
        Draw test pattern

        bounds: the box bounds describe a rectangle that we can safely draw in
                (minimum x, minimum y, maximum x, maximum y)
        wait: seconds waited per centimeter
        interpolate: each centimeter is split into interpolate steps
        repeat: number of times it draws the pattern
        """
        bounds = bounds or self.bounds

        if not bounds:
            raise TypeError("Missing bounds parameter (or missing bounds configuration")

        for r in tqdm.tqdm(tqdm.trange(repeat, desc="Iteration"), leave=False):

            for y in range(bounds[1], bounds[3], 2):

                self.xy(bounds[0], y, wait, interpolate)
                self.draw(bounds[2], y, wait, interpolate)
                self.xy(bounds[2], y + 1, wait, interpolate)
                self.draw(bounds[0], y + 1, wait, interpolate)

        self.pen.up()

        self.quiet()

    def box(self, bounds=None, wait=0.15, interpolate=10, repeat=1, reverse=False):
        """
        Draw rectangle defined by bounds

        bounds: the box bounds describe a rectangle that we can safely draw in
                (minimum x, minimum y, maximum x, maximum y)
        wait: seconds waited per centimeter
        interpolate: each centimeter is split into interpolate steps
        repeat: number of times it draws the pattern
        reverse: clockwise or counterclockwise
        """
        bounds = bounds or self.bounds

        if not bounds:
            raise TypeError("Missing bounds parameter (or missing bounds configuration")

        self.xy(bounds[0], bounds[1], wait, interpolate)

        for r in tqdm.tqdm(tqdm.trange(repeat), desc="Iteration", leave=False):

            if not reverse:

                self.draw(bounds[2], bounds[1], wait, interpolate)
                self.draw(bounds[2], bounds[3], wait, interpolate)
                self.draw(bounds[0], bounds[3], wait, interpolate)
                self.draw(bounds[0], bounds[1], wait, interpolate)

            else:

                self.draw(bounds[0], bounds[3], wait, interpolate)
                self.draw(bounds[2], bounds[3], wait, interpolate)
                self.draw(bounds[2], bounds[1], wait, interpolate)
                self.draw(bounds[0], bounds[1], wait, interpolate)

        self.pen.up()

        self.quiet()

    # ----------------- pen-moving methods -----------------

    def centre(self):
        """
        Moves pencil to center with the pen up.
        """

        if not self.bounds:
            raise TypeError("Missing bounds parameter (or missing bounds configuration")

        self.pen.up()
        self.xy(self.bounds[2] / 2, self.bounds[3] / 2)

        self.quiet()

    def xy(self, x=0, y=0, wait=0.1, interpolate=10, draw=False):
        """
        Moves the pen to the x, y position; optionally draws.

        x: position to draw on x axis
        y: position to draw on y axis
        wait: seconds waited per centimeter
        interpolate: each centimeter is split into interpolate steps
        draw: should we draw or just move the pen
        """
        if draw:
            self.pen.down()
        else:
            self.pen.up()

        (angle_shoulder, angle_elbow) = self.xy_to_angles(x, y)

        # if they are the same, we don't need to move anything
        if (angle_shoulder, angle_elbow) == (self.shoulder.angle, self.elbow.angle):

            # ensure the pantograph knows its x/y positions
            self.current_x = x
            self.current_y = y

            return

        # we assume the pantograph knows its x/y positions - if not, there could be
        # a sudden movement later

        # calculate how many steps we need for this move, and the x/y length of each
        (x_length, y_length) = (x - self.current_x, y - self.current_y)

        length = math.sqrt(x_length ** 2 + y_length ** 2)

        no_of_steps = int(length * interpolate) or 1

        if no_of_steps < 100:
            disable_tqdm = True
        else:
            disable_tqdm = False

        (length_of_step_x, length_of_step_y) = (
            x_length / no_of_steps,
            y_length / no_of_steps,
        )

        for step in tqdm.tqdm(
            range(no_of_steps), desc="Interpolation", leave=False, disable=disable_tqdm
        ):

            self.current_x = self.current_x + length_of_step_x
            self.current_y = self.current_y + length_of_step_y

            angle_shoulder, angle_elbow = self.xy_to_angles(
                self.current_x, self.current_y
            )

            self.set_angles(angle_shoulder, angle_elbow)

            if step + 1 < no_of_steps:
                sleep(length * wait / no_of_steps)

        sleep(length * wait / 10)

    def set_angles(self, angle_shoulder=0, angle_elbow=0):
        """
        Moves the two servo motors

        angle_shoulder: angle of arm (shoulder)
        angle_elbow: angle of arm (elbow)
        """
        self.shoulder.set_angle(angle_shoulder)
        self.elbow.set_angle(angle_elbow)

        # We record the angles, so that we know where the arms are for future reference.
        self.angle_1, self.angle_2 = angle_shoulder, angle_elbow

    def park(self):
        """
        Returns the plotter to initial position
        """
        self.pen.up()
        self.xy(-self.INNER_ARM, self.OUTER_ARM)

    def quiet(self):
        pass

    # ----------------- trigonometric methods -----------------

    # Every x/y position of the plotter corresponds to a pair of angles of the arms. These methods
    # calculate:
    #
    # the angles required to reach any x/y position
    # the x/y position represented by any pair of angles

    def xy_to_angles(self, x=0, y=0):
        """
        Convert x, y co-ordinates into motor angles

        x: position to draw on x axis
        y: position to draw on y axis
        """

        hypotenuse = math.sqrt(x ** 2 + y ** 2)
        hypotenuse_angle = math.asin(x / hypotenuse)

        inner_angle = math.acos(
            (hypotenuse ** 2 + self.INNER_ARM ** 2 - self.OUTER_ARM ** 2)
            / (2 * hypotenuse * self.INNER_ARM)
        )
        outer_angle = math.acos(
            (self.INNER_ARM ** 2 + self.OUTER_ARM ** 2 - hypotenuse ** 2)
            / (2 * self.INNER_ARM * self.OUTER_ARM)
        )

        shoulder_motor_angle = hypotenuse_angle - inner_angle
        elbow_motor_angle = math.pi - outer_angle

        return (math.degrees(shoulder_motor_angle), math.degrees(elbow_motor_angle))

    def angles_to_xy(self, shoulder_motor_angle, elbow_motor_angle):
        """
        Convert motor angles into x, y co-ordinates

        shoulder_motor_angle: angle for shoulder arm
        elbow_motor_angle: angle for elbow arm
        """
        elbow_motor_angle = math.radians(elbow_motor_angle)
        shoulder_motor_angle = math.radians(shoulder_motor_angle)

        hypotenuse = math.sqrt(
            (
                self.INNER_ARM ** 2
                + self.OUTER_ARM ** 2
                - 2
                * self.INNER_ARM
                * self.OUTER_ARM
                * math.cos(math.pi - elbow_motor_angle)
            )
        )
        base_angle = math.acos(
            (hypotenuse ** 2 + self.INNER_ARM ** 2 - self.OUTER_ARM ** 2)
            / (2 * hypotenuse * self.INNER_ARM)
        )
        inner_angle = base_angle + shoulder_motor_angle

        x = math.sin(inner_angle) * hypotenuse
        y = math.cos(inner_angle) * hypotenuse

        return (x, y)

    # ----------------- manual driving method -----------------

    def drive_xy(self):
        """
        Move the pen up/down and left/right using the keyboard
        """
        while True:
            key = readchar.readchar()

            if key == "0":
                return
            elif key == "a":
                self.current_x = self.current_x - 1
            elif key == "s":
                self.current_x = self.current_x + 1
            elif key == "A":
                self.current_x = self.current_x - 0.1
            elif key == "S":
                self.current_x = self.current_x + 0.1
            elif key == "k":
                self.current_y = self.current_y - 1
            elif key == "l":
                self.current_y = self.current_y + 1
            elif key == "K":
                self.current_y = self.current_y - 0.1
            elif key == "L":
                self.current_y = self.current_y + 0.1

            print(self.current_x, self.current_y)

            self.xy(self.current_x, self.current_y)


SHOULDER_GPIO = 14
ELBOW_GPIO = 15
PEN_GPIO = 18


class BrachioGraph(BrachioGraphBase):
    def __init__(
        self,
        inner_arm,  # the lengths of the arms
        outer_arm,
        bounds=None,  # the maximum rectangular drawing area
        servo_shoulder_angle_pws=[],  # pulse-widths for various angles
        servo_elbow_angle_pws=[],
        servo_shoulder_zero=1500,
        servo_elbow_zero=1500,
        pw_up=1500,  # pulse-widths for pen up/down
        pw_down=1100,
    ):

        # set the pantograph geometry
        self.INNER_ARM = inner_arm
        self.OUTER_ARM = outer_arm

        # the box bounds describe a rectangle that we can safely draw in
        self.bounds = bounds

        # instantiate this Raspberry Pi as a pigpio.pi() instance
        self.rpi = pigpio.pi()

        self.shoulder = Servo(
            self.rpi, SHOULDER_GPIO, servo_shoulder_angle_pws, servo_shoulder_zero
        )
        self.elbow = Servo(
            self.rpi, ELBOW_GPIO, servo_elbow_angle_pws, servo_elbow_zero
        )

        # create the pen object, and make sure the pen is up
        self.pen = Pen(ag=self, pw_up=pw_up, pw_down=pw_down)

        # Initialise the pantograph with the motors in the centre of their travel
        self.shoulder.set_angle(-90)
        self.elbow.set_angle(90)

        # Now the plotter is in a safe physical state.

        # Set the x and y position state, so it knows its current x/y position.
        self.current_x = -self.INNER_ARM
        self.current_y = self.OUTER_ARM

    def quiet(self):
        """
        Stop sending pulses to the servos

        servos: list of gpio pin numbers attached to the servos
        """

        for servo in (self.shoulder, self.elbow):
            servo.quiet()

        self.rpi.set_servo_pulsewidth(PEN_GPIO, 0)


class Pen:
    def __init__(
        self, ag, pw_up=1500, pw_down=1100, pin=PEN_GPIO, transition_time=0.25
    ):

        self.ag = ag
        self.pin = pin
        self.pw_up = pw_up
        self.pw_down = pw_down
        self.transition_time = transition_time

        self.rpi = pigpio.pi()
        self.rpi.set_PWM_frequency(self.pin, 50)

        self.up()
        sleep(0.3)
        self.down()
        sleep(0.3)
        self.up()
        sleep(0.3)

    def down(self):
        """
        Put pen down.
        """
        self.rpi.set_servo_pulsewidth(self.pin, self.pw_down)
        sleep(self.transition_time)

    def up(self):
        """
        Lift pen up.
        """
        self.rpi.set_servo_pulsewidth(self.pin, self.pw_up)
        sleep(self.transition_time)
