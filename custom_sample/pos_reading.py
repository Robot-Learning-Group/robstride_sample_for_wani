import time

from robstride_dynamics import RobstrideBus, ParameterType
from motor_config import CHANNEL, MOTORS


READ_HZ = 5.0
DT = 1.0 / READ_HZ


bus = RobstrideBus(
    channel=CHANNEL,
    motors=MOTORS,
)

bus.connect()

try:
    print("Continuously reading mechanical positions.")
    print("Press Ctrl+C to stop.\n")

    while True:
        loop_start = time.perf_counter()

        print("\033[H\033[J", end="")  # clear terminal
        print("Mechanical positions")
        print("-" * 55)

        for joint_name, motor in MOTORS.items():
            try:
                pos = bus.read(
                    joint_name,
                    ParameterType.MECHANICAL_POSITION,
                )

                print(
                    f"{joint_name:<25} "
                    f"ID 0x{motor.id:02X}   "
                    f"{pos:+.6f} rad"
                )

            except Exception as e:
                print(
                    f"{joint_name:<25} "
                    f"ID 0x{motor.id:02X}   "
                    f"[READ ERROR] {e}"
                )

        elapsed = time.perf_counter() - loop_start
        remaining = DT - elapsed

        if remaining > 0:
            time.sleep(remaining)

except KeyboardInterrupt:
    print("\nStopped.")

finally:
    # IMPORTANT: don't disable motors here.
    # This script is intended to be read-only.
    bus.disconnect(disable_torque=False)
