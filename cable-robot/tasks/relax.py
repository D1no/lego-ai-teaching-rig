# Reels out the tension array to relieve tension on the motors

from pybricks.hubs import InventorHub
from pybricks.parameters import Port
from pybricks.pupdevices import Motor
from pybricks.tools import wait

hub = InventorHub()

# Motors
motor_tl = Motor(Port.A)
motor_br = Motor(Port.F)

motor_tr = Motor(Port.B)
motor_bl = Motor(Port.E)

# Reel out
motor_tl.run(300)
motor_br.run(300)

motor_tr.run(-300)
motor_bl.run(-300)

# For two seconds

wait(2000)

print("Tension Array Relaxed")
