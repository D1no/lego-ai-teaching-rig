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
from pybricks.parameters import Color, Direction, Port, Stop
from pybricks.pupdevices import Motor
from pybricks.tools import multitask, run_task, wait

hub = InventorHub()

hub.light.blink(Color.GREEN, [500, 500])

# Model Constants

STALL_TENSION_CLUTCH_DUTY_LIMIT = 22
"""
Stall sensitivity before clutch triggers during tensioning.

Value   Ratio           Motor                           Reel
22      1:3   (1/3)     Black Gear 12 (32270)           -> Black Gear 36 (32498)
30      1:1.4 (5/7)     Light Yellow Gear 20 (18575)    -> Light Grey Gear 28 (46372)
35      1:1   (1/1)     Dark Grey Gear 24 (24505)       -> Dark Grey Gear 24 (24505)
"""

STALL_COLLISION_CLUTCH_DUTY_LIMIT = 18
"""
Stall sensitivity before clutch triggers when forcing disk into motor.

Value   Ratio           Motor                           Reel
18      1:3   (1/3)     Black Gear 12 (32270)           -> Black Gear 36 (32498)
25      1:1.4 (5/7)     Light Yellow Gear 20 (18575)    -> Light Grey Gear 28 (46372)
30      1:1   (1/1)     Dark Grey Gear 24 (24505)       -> Dark Grey Gear 24 (24505)
"""

SPEED_MAX_ANGLE_PER_SEC_CALIBRATION = 300
SPEED_MAX_ANGLE_PER_SEC = 1500

RELAX_TIME = 1000
RELAX_SETTLE_TIME = 500

###########################################################
# Motors
###########################################################

# Speed Direction Sign Multiplier
reel_in = 1
reel_out = reel_in * -1

# ---------------------------------------------------------
# ↖ Top Left
top_left = Motor(Port.A, positive_direction=Direction.COUNTERCLOCKWISE)

# Calibration Origin
top_left_calibration_initial = None
top_left_calibration_tensioned = None

# Travel Range
top_left_travel_min = None
top_left_travel_max = None
top_left_travel_center = None
top_left_travel_left_neighbor = None
top_left_travel_right_neighbor = None

# ---------------------------------------------------------
# ↘ Bottom Right
bottom_right = Motor(Port.F, positive_direction=Direction.COUNTERCLOCKWISE)

# Calibration Origin
bottom_right_calibration_initial = None
bottom_right_calibration_tensioned = None

# Travel Range
bottom_right_travel_min = None
bottom_right_travel_max = None
bottom_right_travel_center = None
bottom_right_travel_left_neighbor = None
bottom_right_travel_right_neighbor = None

# ---------------------------------------------------------
# ↗ Top Right
top_right = Motor(Port.B)

# Calibration Origin
top_right_calibration_initial = None
top_right_calibration_tensioned = None

# Travel Range
top_right_travel_min = None
top_right_travel_max = None
top_right_travel_center = None
top_right_travel_left_neighbor = None
top_right_travel_right_neighbor = None

# ---------------------------------------------------------
# ↙ Bottom Left
bottom_left = Motor(Port.E)

# Calibration Origin
bottom_left_calibration_initial = None
bottom_left_calibration_tensioned = None

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
    print("  > Current Angle:", top_left.angle())
    print("  - top_left_travel_min:", top_left_travel_min)
    print("  - top_left_travel_max:", top_left_travel_max)
    print("  - top_left_travel_center:", top_left_travel_center)
    print("  - top_left_travel_left_neighbor:", top_left_travel_left_neighbor)
    print("  - top_left_travel_right_neighbor:", top_left_travel_right_neighbor)
    print("  - top_left_calibration_initial:", top_left_calibration_initial)
    print("  - top_left_calibration_tensioned:", top_left_calibration_tensioned)
    print("")
    print("↘ Bottom Right")
    print("  > Current Angle:", bottom_right.angle())
    print("  - bottom_right_travel_min:", bottom_right_travel_min)
    print("  - bottom_right_travel_max:", bottom_right_travel_max)
    print("  - bottom_right_travel_center:", bottom_right_travel_center)
    print("  - bottom_right_travel_left_neighbor:", bottom_right_travel_left_neighbor)
    print("  - bottom_right_travel_right_neighbor:", bottom_right_travel_right_neighbor)
    print("  - bottom_right_calibration_initial:", bottom_right_calibration_initial)
    print("  - bottom_right_calibration_tensioned:", bottom_right_calibration_tensioned)
    print("")
    print("↗ Top Right")
    print("  > Current Angle:", top_right.angle())
    print("  - top_right_travel_min:", top_right_travel_min)
    print("  - top_right_travel_max:", top_right_travel_max)
    print("  - top_right_travel_center:", top_right_travel_center)
    print("  - top_right_travel_left_neighbor:", top_right_travel_left_neighbor)
    print("  - top_right_travel_right_neighbor:", top_right_travel_right_neighbor)
    print("  - top_right_calibration_initial:", top_right_calibration_initial)
    print("  - top_right_calibration_tensioned:", top_right_calibration_tensioned)
    print("")
    print("↙ Bottom Left")
    print("  > Current Angle:", bottom_left.angle())
    print("  - bottom_left_travel_min:", bottom_left_travel_min)
    print("  - bottom_left_travel_max:", bottom_left_travel_max)
    print("  - bottom_left_travel_center:", bottom_left_travel_center)
    print("  - bottom_left_travel_left_neighbor:", bottom_left_travel_left_neighbor)
    print("  - bottom_left_travel_right_neighbor:", bottom_left_travel_right_neighbor)
    print("  - bottom_left_calibration_initial:", bottom_left_calibration_initial)
    print("  - bottom_left_calibration_tensioned:", bottom_left_calibration_tensioned)
    print("")
    print("------------------------------------------------------")


###########################################################
# Monitoring
###########################################################


async def log_load(every_ms: int = 200):
    while True:
        print(
            "Load [TL, BR, TR, BL]",
            top_left.load(),
            bottom_right.load(),
            top_right.load(),
            bottom_left.load(),
        )

        await wait(every_ms)


def run_task_monitored(task):
    async def runner():
        await multitask(task, log_load(), race=True)

    run_task(runner())


###########################################################
# Solving Stages
###########################################################


# Stage 1.1: Make sure everything is in tension
async def bring_under_tension(speed: int = SPEED_MAX_ANGLE_PER_SEC_CALIBRATION):
    await multitask(
        top_left.run_until_stalled(
            speed=speed * reel_in,
            then=Stop.HOLD,
            duty_limit=STALL_TENSION_CLUTCH_DUTY_LIMIT,
        ),
        bottom_right.run_until_stalled(
            speed=speed * reel_in,
            then=Stop.HOLD,
            duty_limit=STALL_TENSION_CLUTCH_DUTY_LIMIT,
        ),
        top_right.run_until_stalled(
            speed=speed * reel_in,
            then=Stop.HOLD,
            duty_limit=STALL_TENSION_CLUTCH_DUTY_LIMIT,
        ),
        bottom_left.run_until_stalled(
            speed=speed * reel_in,
            then=Stop.HOLD,
            duty_limit=STALL_TENSION_CLUTCH_DUTY_LIMIT,
        ),
    )


top_left_calibration_initial = top_left.angle()
bottom_right_calibration_initial = bottom_right.angle()
top_right_calibration_initial = top_right.angle()
bottom_left_calibration_initial = bottom_left.angle()

print(
    "Stage 1.1 (START): Tensioning all motors from [TL, BR, TR, BL]",
    top_left_calibration_initial,
    bottom_right_calibration_initial,
    top_right_calibration_initial,
    bottom_left_calibration_initial,
)

run_task_monitored(bring_under_tension())

top_left_calibration_tensioned = top_left.angle()
bottom_right_calibration_tensioned = bottom_right.angle()
top_right_calibration_tensioned = top_right.angle()
bottom_left_calibration_tensioned = bottom_left.angle()

print(
    "Stage 1.1 (END): All motors tensioned to [TL, BR, TR, BL]",
    top_left_calibration_tensioned,
    bottom_right_calibration_tensioned,
    top_right_calibration_tensioned,
    bottom_left_calibration_tensioned,
)


# Stage 1.2: Relax all motors for a brief moment to relax the clutch
def relax_tension(
    speed: int = SPEED_MAX_ANGLE_PER_SEC_CALIBRATION,
    time: int = RELAX_TIME,
    add_wait: int = RELAX_SETTLE_TIME,
):
    top_left.run_time(
        speed=speed * reel_out,
        time=time,
        then=Stop.COAST,
        wait=False,
    )
    bottom_right.run_time(
        speed=speed * reel_out,
        time=time,
        then=Stop.COAST,
        wait=False,
    )
    top_right.run_time(
        speed=speed * reel_out,
        time=time,
        then=Stop.COAST,
        wait=False,
    )
    bottom_left.run_time(
        speed=speed * reel_out,
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


# Stage 2.1.1: Go to top left zero position, stop all when stalled
async def travel_to_top_left_zero_position(
    speed: int = SPEED_MAX_ANGLE_PER_SEC_CALIBRATION,
):
    await multitask(
        # Reel In
        top_left.run_until_stalled(
            speed=speed * reel_in,
            then=Stop.HOLD,
            duty_limit=STALL_COLLISION_CLUTCH_DUTY_LIMIT,
        ),
        # Others Reel Out
        bottom_right.run_until_stalled(
            speed=speed * reel_out,
            then=Stop.HOLD,
            duty_limit=STALL_COLLISION_CLUTCH_DUTY_LIMIT,
        ),
        top_right.run_until_stalled(
            speed=speed * reel_out,
            then=Stop.HOLD,
            duty_limit=STALL_COLLISION_CLUTCH_DUTY_LIMIT,
        ),
        bottom_left.run_until_stalled(
            speed=speed * reel_out,
            then=Stop.HOLD,
            duty_limit=STALL_COLLISION_CLUTCH_DUTY_LIMIT,
        ),
        # Stop all when one stalls
        race=True,
    )


print("Stage 2.1.1: Top Left Angle is at", top_left.angle())

run_task_monitored(travel_to_top_left_zero_position())

# Stage 2.1.2: Safe the top left zero position
top_left_travel_min = top_left.angle()

print("Stage 2.1.2: Top Left MIN Travel Angle is at", top_left_travel_min)

# Stage 2.1.3: Set top left to hold its current zero position
top_left.hold()

print("Stage 2.1.3: Top Left Angle is HOLDING")


# Stage 2.1.4: Reel in all other motors until stalled
async def tension_top_left_partners(
    speed: int = SPEED_MAX_ANGLE_PER_SEC_CALIBRATION,
):
    await multitask(
        bottom_right.run_until_stalled(
            speed=speed * reel_in,
            then=Stop.HOLD,
            duty_limit=STALL_TENSION_CLUTCH_DUTY_LIMIT,
        ),
        top_right.run_until_stalled(
            speed=speed * reel_in,
            then=Stop.HOLD,
            duty_limit=STALL_TENSION_CLUTCH_DUTY_LIMIT,
        ),
        bottom_left.run_until_stalled(
            speed=speed * reel_in,
            then=Stop.HOLD,
            duty_limit=STALL_TENSION_CLUTCH_DUTY_LIMIT,
        ),
    )


print("Stage 2.1.4: Bottom Right Angle is at", bottom_right.angle())
print("Stage 2.1.4: Top Right Angle is at", top_right.angle())
print("Stage 2.1.4: Bottom Left Angle is at", bottom_left.angle())

run_task_monitored(tension_top_left_partners())

# Stage 2.1.5: Safe travel ranges for bottom right, top right and bottom left
bottom_right_travel_max = bottom_right.angle()

top_right_travel_right_neighbor = top_right.angle()
bottom_left_travel_left_neighbor = bottom_left.angle()

print("Stage 2.1.5: Bottom Right MAX Travel Angle is at", bottom_right_travel_max)
print(
    "Stage 2.1.5: Top Right - Right Neighbor Travel Angle is at",
    top_right_travel_right_neighbor,
)
print(
    "Stage 2.1.5: Bottom Left - Left Neighbor Travel Angle is at",
    bottom_left_travel_left_neighbor,
)

# Stage 2.1.6: Relax all motors
relax_tension()

print_parameter_status("Stage 2.1.6: Relaxed all motors")


# Stage 2.1.7: Go back to tensioned calibration origin
async def move_to_tensioned_calibration_origin():
    await multitask(
        top_left.run_target(
            speed=SPEED_MAX_ANGLE_PER_SEC,
            target_angle=top_left_calibration_tensioned,
            then=Stop.COAST,
        ),
        bottom_right.run_target(
            speed=SPEED_MAX_ANGLE_PER_SEC,
            target_angle=bottom_right_calibration_tensioned,
            then=Stop.COAST,
        ),
        top_right.run_target(
            speed=SPEED_MAX_ANGLE_PER_SEC,
            target_angle=top_right_calibration_tensioned,
            then=Stop.COAST,
        ),
        bottom_left.run_target(
            speed=SPEED_MAX_ANGLE_PER_SEC,
            target_angle=bottom_left_calibration_tensioned,
            then=Stop.COAST,
        ),
    )


run_task_monitored(move_to_tensioned_calibration_origin())

print(
    "Stage 2.1.7 (END): All motors tensioned to [TL, BR, TR, BL]",
    top_left_calibration_tensioned,
    bottom_right_calibration_tensioned,
    top_right_calibration_tensioned,
    bottom_left_calibration_tensioned,
)

# Stage 2.1.8: Relax all motors at the tensioned calibration origin
relax_tension()

print_parameter_status(
    "Stage 2.1.8: Relaxed all motors at the tensioned calibration origin"
)


# Stage 2.2.1: Go to bottom right zero position, stop all when stalled
async def travel_to_bottom_right_zero_position(
    speed: int = SPEED_MAX_ANGLE_PER_SEC_CALIBRATION,
):
    await multitask(
        # Reel In
        bottom_right.run_until_stalled(
            speed=speed * reel_in,
            then=Stop.HOLD,
            duty_limit=STALL_COLLISION_CLUTCH_DUTY_LIMIT,
        ),
        # Others Reel Out
        top_left.run_until_stalled(
            speed=speed * reel_out,
            then=Stop.HOLD,
            duty_limit=STALL_COLLISION_CLUTCH_DUTY_LIMIT,
        ),
        top_right.run_until_stalled(
            speed=speed * reel_out,
            then=Stop.HOLD,
            duty_limit=STALL_COLLISION_CLUTCH_DUTY_LIMIT,
        ),
        bottom_left.run_until_stalled(
            speed=speed * reel_out,
            then=Stop.HOLD,
            duty_limit=STALL_COLLISION_CLUTCH_DUTY_LIMIT,
        ),
        # Stop all when one stalls
        race=True,
    )


print("Stage 2.2.1: Bottom Right Angle is at", bottom_right.angle())

run_task_monitored(travel_to_bottom_right_zero_position())

# Stage 2.2.2: Safe the bottom right zero position
bottom_right_travel_min = bottom_right.angle()

print("Stage 2.2.2: Bottom Right MIN Travel Angle is at", bottom_right_travel_min)

# Stage 2.2.3: Set bottom right to hold its current zero position
bottom_right.hold()

print("Stage 2.2.3: Bottom Right Angle is HOLDING")


# Stage 2.2.4: Reel in all other motors until stalled
async def tension_bottom_right_partners(
    speed: int = SPEED_MAX_ANGLE_PER_SEC_CALIBRATION,
):
    await multitask(
        top_left.run_until_stalled(
            speed=speed * reel_in,
            then=Stop.HOLD,
            duty_limit=STALL_TENSION_CLUTCH_DUTY_LIMIT,
        ),
        top_right.run_until_stalled(
            speed=speed * reel_in,
            then=Stop.HOLD,
            duty_limit=STALL_TENSION_CLUTCH_DUTY_LIMIT,
        ),
        bottom_left.run_until_stalled(
            speed=speed * reel_in,
            then=Stop.HOLD,
            duty_limit=STALL_TENSION_CLUTCH_DUTY_LIMIT,
        ),
    )


print("Stage 2.2.4: Top Left Angle is at", top_left.angle())
print("Stage 2.2.4: Top Right Angle is at", top_right.angle())
print("Stage 2.2.4: Bottom Left Angle is at", bottom_left.angle())

run_task_monitored(tension_bottom_right_partners())

# Stage 2.2.5: Safe travel ranges for top left, top right and bottom left
top_left_travel_max = top_left.angle()

top_right_travel_left_neighbor = top_right.angle()
bottom_left_travel_right_neighbor = bottom_left.angle()

print("Stage 2.2.5: Top Left MAX Travel Angle is at", top_left_travel_max)
print(
    "Stage 2.2.5: Top Right - Left Neighbor Travel Angle is at",
    top_right_travel_left_neighbor,
)
print(
    "Stage 2.2.5: Bottom Left - Right Neighbor Travel Angle is at",
    bottom_left_travel_right_neighbor,
)

# Stage 2.2.6: Relax all motors
relax_tension()

print_parameter_status("Stage 2.2.6: Relaxed all motors")

# Stage 2.2.7: Go back to tensioned calibration origin
run_task_monitored(move_to_tensioned_calibration_origin())

print(
    "Stage 2.2.7 (END): All motors tensioned to [TL, BR, TR, BL]",
    top_left_calibration_tensioned,
    bottom_right_calibration_tensioned,
    top_right_calibration_tensioned,
    bottom_left_calibration_tensioned,
)

# Stage 2.2.8: Relax all motors at the tensioned calibration origin
relax_tension()

print_parameter_status(
    "Stage 2.2.8: Relaxed all motors at the tensioned calibration origin"
)


# Stage 2.3.1: Go to bottom left zero position, stop all when stalled
async def travel_to_bottom_left_zero_position(
    speed: int = SPEED_MAX_ANGLE_PER_SEC_CALIBRATION,
):
    await multitask(
        # Reel In
        bottom_left.run_until_stalled(
            speed=speed * reel_in,
            then=Stop.HOLD,
            duty_limit=STALL_COLLISION_CLUTCH_DUTY_LIMIT,
        ),
        # Others Reel Out
        top_left.run_until_stalled(
            speed=speed * reel_out,
            then=Stop.HOLD,
            duty_limit=STALL_COLLISION_CLUTCH_DUTY_LIMIT,
        ),
        top_right.run_until_stalled(
            speed=speed * reel_out,
            then=Stop.HOLD,
            duty_limit=STALL_COLLISION_CLUTCH_DUTY_LIMIT,
        ),
        bottom_right.run_until_stalled(
            speed=speed * reel_out,
            then=Stop.HOLD,
            duty_limit=STALL_COLLISION_CLUTCH_DUTY_LIMIT,
        ),
        # Stop all when one stalls
        race=True,
    )


print("Stage 2.3.1: Bottom Left Angle is at", bottom_left.angle())

run_task_monitored(travel_to_bottom_left_zero_position())

# Stage 2.3.2: Safe the bottom left zero position
bottom_left_travel_min = bottom_left.angle()

print("Stage 2.3.2: Bottom Left MIN Travel Angle is at", bottom_left_travel_min)

# Stage 2.3.3: Set bottom left to hold its current zero position
bottom_right.hold()

print("Stage 2.3.3: Bottom Left Angle is HOLDING")


# Stage 2.3.4: Reel in all other motors until stalled
async def tension_bottom_left_partners(
    speed: int = SPEED_MAX_ANGLE_PER_SEC_CALIBRATION,
):
    await multitask(
        top_left.run_until_stalled(
            speed=speed * reel_in,
            then=Stop.HOLD,
            duty_limit=STALL_TENSION_CLUTCH_DUTY_LIMIT,
        ),
        top_right.run_until_stalled(
            speed=speed * reel_in,
            then=Stop.HOLD,
            duty_limit=STALL_TENSION_CLUTCH_DUTY_LIMIT,
        ),
        bottom_right.run_until_stalled(
            speed=speed * reel_in,
            then=Stop.HOLD,
            duty_limit=STALL_TENSION_CLUTCH_DUTY_LIMIT,
        ),
    )


print("Stage 2.3.4: Top Left Angle is at", top_left.angle())
print("Stage 2.3.4: Top Right Angle is at", top_right.angle())
print("Stage 2.3.4: Bottom Right Angle is at", bottom_right.angle())

run_task_monitored(tension_bottom_left_partners())

# Stage 2.3.5: Safe travel ranges for top left, top right and bottom left
top_right_travel_max = top_right.angle()

top_left_travel_right_neighbor = top_left.angle()
bottom_right_travel_left_neighbor = bottom_right.angle()

print("Stage 2.3.5: Top Right MAX Travel Angle is at", top_right_travel_max)
print(
    "Stage 2.3.5: Top Left - Right Neighbor Travel Angle is at",
    top_left_travel_right_neighbor,
)
print(
    "Stage 2.3.5: Bottom Right - Left Neighbor Travel Angle is at",
    bottom_right_travel_left_neighbor,
)

# Stage 2.3.6: Relax all motors
relax_tension()

print_parameter_status("Stage 2.3.6: Relaxed all motors")

# Stage 2.3.7: Go back to tensioned calibration origin
run_task_monitored(move_to_tensioned_calibration_origin())

print(
    "Stage 2.3.7 (END): All motors tensioned to [TL, BR, TR, BL]",
    top_left_calibration_tensioned,
    bottom_right_calibration_tensioned,
    top_right_calibration_tensioned,
    bottom_left_calibration_tensioned,
)

# Stage 2.3.8: Relax all motors at the tensioned calibration origin
relax_tension()

print_parameter_status(
    "Stage 2.3.8: Relaxed all motors at the tensioned calibration origin"
)


# Stage 2.4.1: Go to top right zero position, stop all when stalled
async def travel_to_top_right_zero_position(
    speed: int = SPEED_MAX_ANGLE_PER_SEC_CALIBRATION,
):
    await multitask(
        # Reel In
        top_right.run_until_stalled(
            speed=speed * reel_in,
            then=Stop.HOLD,
            duty_limit=STALL_COLLISION_CLUTCH_DUTY_LIMIT,
        ),
        # Others Reel Out
        top_left.run_until_stalled(
            speed=speed * reel_out,
            then=Stop.HOLD,
            duty_limit=STALL_COLLISION_CLUTCH_DUTY_LIMIT,
        ),
        bottom_left.run_until_stalled(
            speed=speed * reel_out,
            then=Stop.HOLD,
            duty_limit=STALL_COLLISION_CLUTCH_DUTY_LIMIT,
        ),
        bottom_right.run_until_stalled(
            speed=speed * reel_out,
            then=Stop.HOLD,
            duty_limit=STALL_COLLISION_CLUTCH_DUTY_LIMIT,
        ),
        # Stop all when one stalls
        race=True,
    )


print("Stage 2.4.1: Top Right Angle is at", top_right.angle())

run_task_monitored(travel_to_top_right_zero_position())

# Stage 2.4.2: Safe the top right zero position
top_right_travel_min = top_right.angle()

print("Stage 2.4.2: Top Right MIN Travel Angle is at", top_right_travel_min)

# Stage 2.4.3: Set top right to hold its current zero position
top_right.hold()

print("Stage 2.4.3: Top Right Angle is HOLDING")


# Stage 2.4.4: Reel in all other motors until stalled
async def tension_top_right_partners(
    speed: int = SPEED_MAX_ANGLE_PER_SEC_CALIBRATION,
):
    await multitask(
        top_left.run_until_stalled(
            speed=speed * reel_in,
            then=Stop.HOLD,
            duty_limit=STALL_TENSION_CLUTCH_DUTY_LIMIT,
        ),
        bottom_left.run_until_stalled(
            speed=speed * reel_in,
            then=Stop.HOLD,
            duty_limit=STALL_TENSION_CLUTCH_DUTY_LIMIT,
        ),
        bottom_right.run_until_stalled(
            speed=speed * reel_in,
            then=Stop.HOLD,
            duty_limit=STALL_TENSION_CLUTCH_DUTY_LIMIT,
        ),
    )


print("Stage 2.4.4: Top Left Angle is at", top_left.angle())
print("Stage 2.4.4: Bottom Left Angle is at", bottom_left.angle())
print("Stage 2.4.4: Bottom Right Angle is at", bottom_right.angle())

run_task_monitored(tension_top_right_partners())

# Stage 2.4.5: Safe travel ranges for top left, top right and bottom left
bottom_left_travel_max = bottom_left.angle()

top_left_travel_left_neighbor = top_left.angle()
bottom_right_travel_right_neighbor = bottom_right.angle()

print("Stage 2.4.5: Bottom Left MAX Travel Angle is at", bottom_left_travel_max)
print(
    "Stage 2.4.5: Top Left - Left Neighbor Travel Angle is at",
    top_left_travel_left_neighbor,
)
print(
    "Stage 2.4.5: Bottom Right - Right Neighbor Travel Angle is at",
    bottom_right_travel_right_neighbor,
)

# Stage 2.4.6: Relax all motors
relax_tension()

print_parameter_status("Stage 2.4.6: Relaxed all motors")

# Stage 2.4.7: Go back to tensioned calibration origin
run_task_monitored(move_to_tensioned_calibration_origin())

print(
    "Stage 2.4.7 (END): All motors tensioned to [TL, BR, TR, BL]",
    top_left_calibration_tensioned,
    bottom_right_calibration_tensioned,
    top_right_calibration_tensioned,
    bottom_left_calibration_tensioned,
)

# Stage 2.4.8: Relax all motors at the tensioned calibration origin
relax_tension()

print_parameter_status(
    "Stage 2.4.8: Relaxed all motors at the tensioned calibration origin"
)

# Stage 3.1: Claculate the center of the travel ranges
top_left_travel_center = (
    top_left_travel_max - top_left_travel_min
) / 2 + top_left_travel_min
bottom_right_travel_center = (
    bottom_right_travel_max - bottom_right_travel_min
) / 2 + bottom_right_travel_min

bottom_left_travel_center = (
    bottom_left_travel_max - bottom_left_travel_min
) / 2 + bottom_left_travel_min
top_right_travel_center = (
    top_right_travel_max - top_right_travel_min
) / 2 + top_right_travel_min


print_parameter_status("Stage 3.1: Claculate the center of the travel ranges")


# Stage 3.2: Go to the center of the travel ranges
async def travel_to_center(
    speed: int = SPEED_MAX_ANGLE_PER_SEC,
):
    await multitask(
        top_left.run_target(
            speed=speed,
            target_angle=top_left_travel_center,
            then=Stop.HOLD,
        ),
        bottom_right.run_target(
            speed=speed,
            target_angle=bottom_right_travel_center,
            then=Stop.HOLD,
        ),
        top_right.run_target(
            speed=speed,
            target_angle=top_right_travel_center,
            then=Stop.HOLD,
        ),
        bottom_left.run_target(
            speed=speed,
            target_angle=bottom_left_travel_center,
            then=Stop.HOLD,
        ),
    )


run_task_monitored(travel_to_center())

print_parameter_status("Stage 3.2: Go to the center of the travel ranges")
