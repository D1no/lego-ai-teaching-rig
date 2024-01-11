"""
Description Cable Robot (Testing AI Assistants)
Dino Scheidt <github@din.ooo>

Situation:
The following code should control a cable robot made out of lego technic and motors with position encoders by using a library called "pybricks" which runs on the lego mindstorms robot inventor hub. This robot is a parallel robot meaning, that it has four motors arranged in rectangular arrangement. These four motors sit in each corner of the rectangle and are connected to each other with a string of yarn. The top left motor is directly tied to the motor in the bottom right, and the motor in the top right is directly connected with the motor in the bottom left. The two crossing strings are connected with a knot to a disc. This means, that the motors can position this disc in a x and y coordinate system by pulling their end of the string with varying degrees to their corner. Each motor has a spring and a clutch for holding the string in tensioning while not overloading the motor by accident.

Complication:
In the beginning, it is unknown if the rectangular arrangement is square and how large it is. Therefore the range of travel is unknown. In the beginning, the string connecting this arrangement is also not tensioned and not guranteed to be in equal length to each motor.

Goal Definition:
 - The first goal is to move the disc to the center.
 - Next, move the disc clockwise to each corner in a circular pattern.

Constraints:
This code needs to calibrate this tension array by finding valid travel ranges for each motor and then use this information to control the robot in a cartesian coordinate system going from 0 to 100 for each axis and apply to a safe zone of about 80% of the full size of the available space.

Make sure that at any given moment, no motor is running faster than the MAX_SPEED_ANGLE_PER_SEC. During calibration, not faster than SPEED_MAX_ANGLE_PER_SEC_CALIBRATION. The STALL_TENSION_CLUTCH_DUTY_LIMIT is the value at which a motor stalls but still puts it end of the string under tension with its spring without triggering the safety clutch. The STALL_COLLISION_CLUTCH_DUTY_LIMIT is to be used when looking for minimum travel, i.e. when the disk is at danger to be pulled inside of anyone motor. The RELAX_TIME is the time in which the motors are relaxed, reeling out, to let the clutch settle. The RELAX_SETTLE_TIME is the time in which we simply wait for the springs inside of the mechanisms to recover. Acting too fast, will prematurely trigger stall detections in following steps.

In general, be conservative with speed during calibration and orientate yourself on the max speed during operating within the safe zone cartesian coordinate system. Since the space can be very rectangular, keep in mind that some motors need to be faster than others to compensate.

Tips:
Stall detection is useful for detecting limits by using the run_until_stalled method. The motor.angle() method provides an absolute angle the motor is at during its life. It is read from the motor encoder and is an absolute integer between that can be negative. It is useful to find and track travel ranges in each direction. Which direction of the motor winds in the string is written below. So keep in mind, that travel from the motors perspective can be flipped.

The documentation for the pybricks library can be found here: https://docs.pybricks.com/en/latest/index.html where the relevant page for the motors is this page https://docs.pybricks.com/en/latest/pupdevices/motor.html and for multi tasking this page https://docs.pybricks.com/en/latest/tools/index.html
"""  # noqa: E501 # pylint: disable=line-too-long

from pybricks.hubs import InventorHub
from pybricks.parameters import Color, Port, Stop
from pybricks.pupdevices import Motor
from pybricks.tools import multitask, run_task, wait

hub = InventorHub()

hub.light.blink(Color.GREEN, [500, 500])

# Model Constants
STALL_TENSION_CLUTCH_DUTY_LIMIT = 20
STALL_COLLISION_CLUTCH_DUTY_LIMIT = 15

SPEED_MAX_ANGLE_PER_SEC_CALIBRATION = 300
SPEED_MAX_ANGLE_PER_SEC = 1000

RELAX_TIME = 1500
RELAX_SETTLE_TIME = 500

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
top_left_travel_center = None
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
bottom_right_travel_center = None
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
top_right_travel_center = None
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
bottom_left_travel_center = None
bottom_left_travel_left_neighbor = None
bottom_left_travel_right_neighbor = None


def print_parameter_status(title: str = "Current Parameters"):
    print("")
    print(f"[{title}] ---------------------------------")
    print("")
    print("↖ Top Left")
    print("  - top_left_travel_min:", top_left_travel_min)
    print("  - top_left_travel_max:", top_left_travel_max)
    print("  - top_left_travel_center:", top_left_travel_center)
    print("  - top_left_travel_left_neighbor:", top_left_travel_left_neighbor)
    print("  - top_left_travel_right_neighbor:", top_left_travel_right_neighbor)
    print("")
    print("↘ Bottom Right")
    print("  - bottom_right_travel_min:", bottom_right_travel_min)
    print("  - bottom_right_travel_max:", bottom_right_travel_max)
    print("  - bottom_right_travel_center:", bottom_right_travel_center)
    print("  - bottom_right_travel_left_neighbor:", bottom_right_travel_left_neighbor)
    print("  - bottom_right_travel_right_neighbor:", bottom_right_travel_right_neighbor)
    print("")
    print("↗ Top Right")
    print("  - top_right_travel_min:", top_right_travel_min)
    print("  - top_right_travel_max:", top_right_travel_max)
    print("  - top_right_travel_center:", top_right_travel_center)
    print("  - top_right_travel_left_neighbor:", top_right_travel_left_neighbor)
    print("  - top_right_travel_right_neighbor:", top_right_travel_right_neighbor)
    print("")
    print("↙ Bottom Left")
    print("  - bottom_left_travel_min:", bottom_left_travel_min)
    print("  - bottom_left_travel_max:", bottom_left_travel_max)
    print("  - bottom_left_travel_center:", bottom_left_travel_center)
    print("  - bottom_left_travel_left_neighbor:", bottom_left_travel_left_neighbor)
    print("  - bottom_left_travel_right_neighbor:", bottom_left_travel_right_neighbor)
    print("")
    print("------------------------------------------------------")


###########################################################
# Solving Stages
###########################################################


# Stage 1.1: Make sure everything is in tension
async def bring_under_tension(speed: int = SPEED_MAX_ANGLE_PER_SEC_CALIBRATION):
    await multitask(
        top_left.run_until_stalled(
            speed=speed * top_left_in,
            then=Stop.HOLD,
            duty_limit=STALL_TENSION_CLUTCH_DUTY_LIMIT,
        ),
        bottom_right.run_until_stalled(
            speed=speed * bottom_right_in,
            then=Stop.HOLD,
            duty_limit=STALL_TENSION_CLUTCH_DUTY_LIMIT,
        ),
        top_right.run_until_stalled(
            speed=speed * top_right_in,
            then=Stop.HOLD,
            duty_limit=STALL_TENSION_CLUTCH_DUTY_LIMIT,
        ),
        bottom_left.run_until_stalled(
            speed=speed * bottom_left_in,
            then=Stop.HOLD,
            duty_limit=STALL_TENSION_CLUTCH_DUTY_LIMIT,
        ),
    )


print(
    "Stage 1.1 (START): Tensioning all motors from [TL, BR, TR, BL]",
    top_left.angle(),
    bottom_right.angle(),
    top_right.angle(),
    bottom_left.angle(),
)

run_task(bring_under_tension())

print(
    "Stage 1.1 (END): All motors tensioned to [TL, BR, TR, BL]",
    top_left.angle(),
    bottom_right.angle(),
    top_right.angle(),
    bottom_left.angle(),
)


# Stage 1.2: Relax all motors for a brief moment to relax the clutch
def relax_tension(
    speed: int = SPEED_MAX_ANGLE_PER_SEC_CALIBRATION,
    time: int = RELAX_TIME,
    add_wait: int = RELAX_SETTLE_TIME,
):
    top_left.run_time(
        speed=speed * top_left_out,
        time=time,
        then=Stop.COAST,
        wait=False,
    )
    bottom_right.run_time(
        speed=speed * bottom_right_out,
        time=time,
        then=Stop.COAST,
        wait=False,
    )
    top_right.run_time(
        speed=speed * top_right_out,
        time=time,
        then=Stop.COAST,
        wait=False,
    )
    bottom_left.run_time(
        speed=speed * bottom_left_out,
        time=time,
        then=Stop.COAST,
        wait=False,
    )

    wait(time + add_wait)


relax_tension()

print(
    "Stage 1.2 (END): All motors relaxed [TL, BR, TR, BL]",
    top_left.angle(),
    bottom_right.angle(),
    top_right.angle(),
    bottom_left.angle(),
)


# Stage 2.1: Go to top left zero position, stop all when top left is stalled
async def travel_to_top_left_zero_position(
    speed: int = SPEED_MAX_ANGLE_PER_SEC_CALIBRATION,
):
    await multitask(
        # Reel In
        top_left.run_until_stalled(
            speed=speed * top_left_in,
            then=Stop.HOLD,
            duty_limit=STALL_COLLISION_CLUTCH_DUTY_LIMIT,
        ),
        # Others Reel Out
        bottom_right.run_until_stalled(
            speed=speed * bottom_right_out,
            then=Stop.HOLD,
            duty_limit=STALL_COLLISION_CLUTCH_DUTY_LIMIT,
        ),
        top_right.run_until_stalled(
            speed=speed * top_right_out,
            then=Stop.HOLD,
            duty_limit=STALL_COLLISION_CLUTCH_DUTY_LIMIT,
        ),
        bottom_left.run_until_stalled(
            speed=speed * bottom_left_out,
            then=Stop.HOLD,
            duty_limit=STALL_COLLISION_CLUTCH_DUTY_LIMIT,
        ),
        # Stop all when top left is stalled
        race=True,
    )


print("Stage 2.1: Top Left Angle is at", top_left.angle())

run_task(travel_to_top_left_zero_position())

# Stage 2.2: Safe the top left zero position
top_left_travel_min = top_left.angle()

print("Stage 2.2: Top Left MIN Travel Angle is at", top_left_travel_min)

# Stage 2.3: Set top left to hold its current zero position
top_left.hold()

print("Stage 2.3: Top Left Angle is HOLDING")


# Stage 2.4: Reel in all other motors until stalled
async def tension_top_left_partners(
    speed: int = SPEED_MAX_ANGLE_PER_SEC_CALIBRATION,
):
    await multitask(
        bottom_right.run_until_stalled(
            speed=speed * bottom_right_in,
            then=Stop.HOLD,
            duty_limit=STALL_TENSION_CLUTCH_DUTY_LIMIT,
        ),
        top_right.run_until_stalled(
            speed=speed * top_right_in,
            then=Stop.HOLD,
            duty_limit=STALL_TENSION_CLUTCH_DUTY_LIMIT,
        ),
        bottom_left.run_until_stalled(
            speed=speed * bottom_left_in,
            then=Stop.HOLD,
            duty_limit=STALL_TENSION_CLUTCH_DUTY_LIMIT,
        ),
    )


print("Stage 2.4: Bottom Right Angle is at", bottom_right.angle())
print("Stage 2.4: Top Right Angle is at", top_right.angle())
print("Stage 2.4: Bottom Left Angle is at", bottom_left.angle())

run_task(tension_top_left_partners())

# Stage 2.5: Safe travel ranges for bottom right, top right and bottom left
bottom_right_travel_max = bottom_right.angle()

top_right_travel_right_neighbor = top_right.angle()
bottom_left_travel_left_neighbor = bottom_left.angle()

print("Stage 2.5: Bottom Right MAX Travel Angle is at", bottom_right_travel_max)
print(
    "Stage 2.5: Top Right - Right Neighbor Travel Angle is at",
    top_right_travel_right_neighbor,
)
print(
    "Stage 2.5: Bottom Left - Left Neighbor Travel Angle is at",
    bottom_left_travel_left_neighbor,
)

# Stage 2.6: Relax all motors
relax_tension()

print_parameter_status("Stage 2.6: Relaxed all motors")
