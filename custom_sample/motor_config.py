# motor_config.py

from robstride_dynamics import Motor

CHANNEL = "can0"

MOTORS = {
    # Left leg
    "left_hip_yaw": Motor(
        id=0x1A,       # decimal 26
        model="rs-02",
    ),
    "left_hip_roll": Motor(
        id=0x1B,       # decimal 27
        model="rs-03",
    ),
    "left_hip_pitch": Motor(
        id=0x1C,       # decimal 28
        model="rs-03",
    ),
    "left_knee": Motor(
        id=0x1D,       # decimal 29
        model="rs-03",
    ),
    "left_ankle_pitch": Motor(
        id=0x1E,       # decimal 30
        model="rs-02",
    ),
    "left_ankle_roll": Motor(
        id=0x1F,       # decimal 31
        model="rs-00",
    ),

    # Right leg
    "right_hip_yaw": Motor(
        id=0x2A,       # decimal 42
        model="rs-02",
    ),
    "right_hip_roll": Motor(
        id=0x2B,       # decimal 43
        model="rs-03",
    ),
    "right_hip_pitch": Motor(
        id=0x2C,       # decimal 44
        model="rs-03",
    ),
    "right_knee": Motor(
        id=0x2D,       # decimal 45
        model="rs-03",
    ),
    "right_ankle_pitch": Motor(
        id=0x2E,       # decimal 46
        model="rs-02",
    ),
    "right_ankle_roll": Motor(
        id=0x2F,       # decimal 47
        model="rs-00",
    ),
}
