from pybricks.hubs import InventorHub
from pybricks.parameters import Button, Color, Direction, Port, Side, Stop
from pybricks.pupdevices import ColorSensor, Motor, UltrasonicSensor
from pybricks.robotics import DriveBase
from pybricks.tools import StopWatch, multitask, run_task, wait

hub = InventorHub()

hub.light.blink(Color.GREEN, [500, 500])

# Model Constants
STALL_CLUTCH_DUTY_LIMIT = 20
MAX_SPEED_ANGLE_PER_SEC = 1500

# Motors
motor_tl = Motor(Port.A)
motor_br = Motor(Port.F)

motor_tr = Motor(Port.B)
motor_bl = Motor(Port.E)


# Start moving at 300 degrees per second.

# motor_tl.run(300)
# motor_br.run(300)


# Stage 1: Make sure everything is in tension
async def bring_under_tension():
    await multitask(
        motor_tl.run_until_stalled(
            speed=-300, then=Stop.HOLD, duty_limit=STALL_CLUTCH_DUTY_LIMIT
        ),
        motor_br.run_until_stalled(
            speed=-300, then=Stop.HOLD, duty_limit=STALL_CLUTCH_DUTY_LIMIT
        ),
        motor_tr.run_until_stalled(
            speed=300, then=Stop.HOLD, duty_limit=STALL_CLUTCH_DUTY_LIMIT
        ),
        motor_bl.run_until_stalled(
            speed=300, then=Stop.HOLD, duty_limit=STALL_CLUTCH_DUTY_LIMIT
        ),
    )


run_task(bring_under_tension())

# Display the angle and speed 50 times.
for i in range(100):
    # Read the angle (degrees) and speed (degrees per second).
    angle = motor_tl.angle()
    speed = motor_tl.speed()

    # Print the values.
    print(angle, speed)
    # print("Top Left Tension:", tl_tension)
    # print("Bottom Right Tension:", br_tension)
    # print("Top Right Tension:", tr_tension)
    # print("Bottom Left Tension:", bl_tension)

    # Wait some time so we can read what is displayed.
    wait(200)
