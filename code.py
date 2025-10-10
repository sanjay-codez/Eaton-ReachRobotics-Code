import time
import pwmio
from adafruit_motor import servo
from adafruit_simplemath import map_range, constrain
from circuitpython_gizmo import Gizmo

# --- Setup ---
gizmo = Gizmo()

PWM_FREQ = 50
MIN_PULSE = 1000
MAX_PULSE = 2000
DEADZONE = 0.05
SPEED_SCALE = 1.0

# Left motor → Port 2, Right motor → Port 4
motor_left = servo.ContinuousServo(
    pwmio.PWMOut(gizmo.MOTOR_2, frequency=PWM_FREQ),
    min_pulse=MIN_PULSE,
    max_pulse=MAX_PULSE
)
motor_right = servo.ContinuousServo(
    pwmio.PWMOut(gizmo.MOTOR_4, frequency=PWM_FREQ),
    min_pulse=MIN_PULSE,
    max_pulse=MAX_PULSE
)

def apply_deadzone(v):
    return 0 if abs(v) < DEADZONE else v

# --- Main loop ---
while True:
    gizmo.refresh()

    # Read joystick axes
    forward = map_range(gizmo.axes.right_x, 0, 255, -1.0, 1.0)   # Up/down on left stick
    turn = map_range(gizmo.axes.left_y, 0, 255, -1.0, 1.0)     # Left/right on right stick

    # Apply deadzone
    forward = apply_deadzone(forward)
    turn = apply_deadzone(turn)

    # Combine forward/back + rotation
    left_speed = constrain((forward + turn) * SPEED_SCALE, -1.0, 1.0)
    right_speed = constrain((forward - turn) * SPEED_SCALE, -1.0, 1.0)

    # Drive motors
    motor_left.throttle = left_speed
    motor_right.throttle = right_speed

    time.sleep(0.02)
