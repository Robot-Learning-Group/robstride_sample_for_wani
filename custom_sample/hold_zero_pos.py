import time

from robstride_dynamics import RobstrideBus,ParameterType
from motor_config import CHANNEL, MOTORS


TARGET_POSITION = 0.0
VELOCITY_LIMIT = 0.3


bus = RobstrideBus(
    channel=CHANNEL,
    motors=MOTORS,
)

bus.connect()

enabled_motors = []

try:
    for joint_name, motor in MOTORS.items():
        print()
        print(
            f"Configuring {joint_name} "
            f"(ID 0x{motor.id:02X})"
        )

        # 1. Disable before mode change
        print("  Disabling")
        bus.disable(joint_name)

        # 2. Set CSP mode
        print("  Setting CSP mode")
        bus.set_run_mode(joint_name, 5)

        # 3. Enable torque
        print("  Enabling")
        bus.write(joint_name, ParameterType.CAN_TIMEOUT, 0)
        bus.enable(joint_name)
        enabled_motors.append(joint_name)

        # 4. Command target position
        print(f"  Commanding {TARGET_POSITION:+.3f} rad")
        bus.move_to_position_csp(
            joint_name,
            position=TARGET_POSITION,
            velocity_limit=VELOCITY_LIMIT,
        )

        print(f"  [OK] {joint_name}")

    print()
    print("All motors are enabled.")
    print(f"Target position = {TARGET_POSITION:+.3f} rad")
    print("Press Ctrl+C to disable all motors.")

    while True:
        time.sleep(1.0)

except KeyboardInterrupt:
    print("\nStopping...")

except Exception as e:
    print(f"\nERROR: {e}")

finally:
    print("Disabling motors...")

    for joint_name in enabled_motors:
        try:
            print(f"  Disabling {joint_name}")
            bus.disable(joint_name)

        except Exception as e:
            print(
                f"  Warning while disabling "
                f"{joint_name}: {e}"
            )

    bus.disconnect(disable_torque=False)

    print("RobstrideBus disconnected.")
