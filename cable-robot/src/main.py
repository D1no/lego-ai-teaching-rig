# Description for AI Assistance
# The following code is used to control a cable robot made out of lego technic and motors with position encoders by using a library called "pybricks" which runs on the lego mindstorms robot inventor hub. This robot is a parallel robot meaning, that it has four motors arranged in rectangular arrangement. These four motors sit in each corner of the rectangle and are connected to each other with a string of yarn. The top left motor is directly tied to the motor in the bottom right, and the motor in the top right is directly connected with the motor in the bottom left. The two crossing strings are connected with a knot to a disc. This means, that the motors can position this disc in a x and y coordinate system by pulling their end of the string with varying degrees to their corner. Each motor has a spring and a clutch for holding the string in tensioning while not overloading the motor by accident. In the beginning, it is unknown if the rectangular arrangement is square and how large it is. Therefore the range of travel is unknown. In the beginning, the string connecting this arrangement is also not tensioned and not in equal length to each motor. The goal of this code is to calibrate this tension array by finding valid travel ranges for each motor and then use this information to control the robot in a cartesian coordinate system going from 0 to 100 for each axis and is a safe zone of about 80% of the full size of the available space. The first goal is to move the disc to the center of that space. Next, move the disc clockwise to each corner in a circular pattern. Make sure that at any given moment, no motor is running faster than the MAX_SPEED_ANGLE_PER_SEC. The STALL_CLUTCH_DUTY_LIMIT is the value at which a motor stalls but still puts it end of the string under tension with its spring without triggering the safety clutch. This is useful for detecting limits by using the run_until_stalled method. The motor.angle() method provides an absolute angle the motor is at during its life. It is read from the motor encoder and is an absolute integer between that can be negative. It is useful to find and track travel ranges in each direction. Which direction of the motor winds in the string is written below. So keep in mind, that travel from the motors perspective can be flipped. The documentation for the pybricks library can be found here: https://docs.pybricks.com/en/latest/index.html where the relevant page for the motors is this page https://docs.pybricks.com/en/latest/pupdevices/motor.html and for multi tasking this page https://docs.pybricks.com/en/latest/tools/index.html .

from pybricks.hubs import InventorHub
from pybricks.parameters import Button, Color, Direction, Port, Side, Stop
from pybricks.pupdevices import ColorSensor, Motor, UltrasonicSensor
from pybricks.robotics import DriveBase
from pybricks.tools import StopWatch, multitask, run_task, wait

hub = InventorHub()

hub.light.blink(Color.GREEN, [500, 500])

# Model Constants
STALL_CLUTCH_DUTY_LIMIT = 20
MAX_SPEED_ANGLE_PER_SEC = 1000

###########################################################
# Motors
###########################################################

# ---------------------------------------------------------
# ↖ Top Left
top_left = Motor(Port.A)

# Direction of the motor
top_left_in = -1
top_left_out = top_left_in * -1

top_left_travel_min = None
top_left_travel_max = None
top_left_travel_mid = None
top_left_travel_left_neighbor = None
top_left_travel_right_neighbor = None

# ---------------------------------------------------------
# ↘ Bottom Right
bottom_right = Motor(Port.F)

# Direction of the motor
bottom_right_in = -1
bottom_right_out = bottom_right_in * -1

# Travel Range
bottom_right_travel_min = None
bottom_right_travel_max = None
bottom_right_travel_mid = None
bottom_right_travel_left_neighbor = None
bottom_right_travel_right_neighbor = None

# ---------------------------------------------------------
# ↗ Top Right
top_right = Motor(Port.B)

# Direction of the motor
top_right_in = 1
top_right_out = top_right_in * -1

# Travel Range
top_right_travel_min = None
top_right_travel_max = None
top_right_travel_mid = None
top_right_travel_left_neighbor = None
top_right_travel_right_neighbor = None

# ---------------------------------------------------------
# ↙ Bottom Left
bottom_left = Motor(Port.E)

# Direction of the motor
bottom_left_in = 1
bottom_left_out = bottom_left_in * -1

# Travel Range
bottom_left_travel_min = None
bottom_left_travel_max = None
bottom_left_travel_mid = None
bottom_left_travel_left_neighbor = None
bottom_left_travel_right_neighbor = None


###########################################################
# Solving Stages
###########################################################


# Stage 1: Make sure everything is in tension
async def bring_under_tension(speed: int):
    await multitask(
        top_left.run_until_stalled(
            speed=speed * top_left_in,
            then=Stop.HOLD,
            duty_limit=STALL_CLUTCH_DUTY_LIMIT,
        ),
        bottom_right.run_until_stalled(
            speed=speed * bottom_right_in,
            then=Stop.HOLD,
            duty_limit=STALL_CLUTCH_DUTY_LIMIT,
        ),
        top_right.run_until_stalled(
            speed=speed * top_right_in,
            then=Stop.HOLD,
            duty_limit=STALL_CLUTCH_DUTY_LIMIT,
        ),
        bottom_left.run_until_stalled(
            speed=speed * bottom_left_in,
            then=Stop.HOLD,
            duty_limit=STALL_CLUTCH_DUTY_LIMIT,
        ),
    )


# run_task(bring_under_tension(300))

###########################################################
# ChatGPTs Attempt
###########################################################
