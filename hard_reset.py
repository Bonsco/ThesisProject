import time

from buildhat import Motor

left_motor = Motor('C')
right_motor = Motor('B')
camera_motor = Motor('D')

left_motor.start(1)
right_motor.start(1)
camera_motor.start(1)

time.sleep(1)

left_motor.stop()
right_motor.stop()
camera_motor.stop()
