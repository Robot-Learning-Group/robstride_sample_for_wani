import time

from robstride_dynamics import RobstrideBus
from robstride_dynamics.protocol import CommunicationType

from motor_config import CHANNEL, MOTORS


ZERO_TOLERANCE = 0.02


bus = RobstrideBus(
    channel=CHANNEL,
    motors=MOTORS,
)

bus.connect()

try:
    for joint_name, motor in MOTORS.items():
        print(
            f"Zeroing {joint_name} "
            f"(ID 0x{motor.id:02X})"
        )

        # --------------------------------------------------
        # 1. Disable this motor
        # --------------------------------------------------
        bus.disable(joint_name)

        # --------------------------------------------------
        # 2. Set current position as mechanical zero
        # --------------------------------------------------
        bus.transmit(
            CommunicationType.SET_ZERO_POSITION,
            bus.host_id,
            motor.id,
            data=b"\x01\x00\x00\x00\x00\x00\x00\x00",
        )

        # Type-6 returns Type-2 operation-status feedback.
        # receive_status_frame() returns:
        # (position, velocity, torque, temperature)
        pos, _, _, _ = bus.receive_status_frame(joint_name)

        # --------------------------------------------------
        # 3. Verify position reported by the response
        # --------------------------------------------------
        if abs(pos) <= ZERO_TOLERANCE:
            print(f"  [OK] position = {pos:+.6f} rad")
        else:
            print(f"  [WARNING] position = {pos:+.6f} rad")

finally:
    bus.disconnect(
        disable_torque=False
    )
