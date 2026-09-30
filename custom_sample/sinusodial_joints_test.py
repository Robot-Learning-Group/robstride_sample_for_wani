import math
import time
import sys
import select

from robstride_dynamics import RobstrideBus, ParameterType
from motor_config import CHANNEL, MOTORS

# ============================================================
# CSP control settings
# ============================================================

CONTROL_HZ = 100.0
DT = 1.0 / CONTROL_HZ

VELOCITY_LIMIT = 0.5


# ============================================================
# Startup -> zero settings
# ============================================================

# Current position -> 0 rad
ZEROING_TIME = 5.0

# Abort before enabling if startup position looks abnormal.
MAX_START_ABS_POSITION = 0.7

# Position tolerance after zeroing.
ZERO_TOLERANCE = 0.15


# ============================================================
# Periodic trajectory
# ============================================================

# 0 -> offset -> 0
#
# 0.40 Hz:
# one complete cycle = 2.5 s
MOTION_FREQUENCY_HZ = 0.250


# ============================================================
# Motion offset for ALL motors
#
# 0.0 means:
#   motor does NOT move,
#   but CSP command target=0 is still sent every cycle.
#
# positive:
#   0 -> +offset -> 0
#
# negative:
#   0 -> -offset -> 0
# ============================================================

MOTION_OFFSET = {
    # Left leg
    "left_hip_yaw":       0.0,
    "left_hip_roll":      0.0,
    "left_hip_pitch":     +0.2,
    "left_knee":          +0.4,
    "left_ankle_pitch":   -0.2,
    "left_ankle_roll":    0.0,

    # Right leg
    "right_hip_yaw":      0.0,
    "right_hip_roll":     0.0,
    "right_hip_pitch":    -0.2,
    "right_knee":         -0.4,
    "right_ankle_pitch":  +0.2,
    "right_ankle_roll":   0.0,
}


# ============================================================
# Monitoring
#
# IMPORTANT:
#
# This is only the EXTRA parameter read frequency used for
# terminal monitoring.
#
# move_to_position_csp() -> write() already waits for a
# Type-2 status response for every CSP command.
# ============================================================

READBACK_HZ = 2.0

READBACK_INTERVAL = max(
    1,
    int(CONTROL_HZ / READBACK_HZ),
)

MAX_TRACKING_ERROR = 0.4


# ============================================================
# Validate configuration
# ============================================================

for joint_name in MOTORS:

    if joint_name not in MOTION_OFFSET:
        raise RuntimeError(
            f"MOTION_OFFSET is missing '{joint_name}'."
        )


# ============================================================
# Smooth 0 -> 1 interpolation
# ============================================================

def smooth_step(x: float) -> float:
    return 0.5 - 0.5 * math.cos(math.pi * x)


# ============================================================
# Bus
# ============================================================

bus = RobstrideBus(
    channel=CHANNEL,
    motors=MOTORS,
)

bus.connect()

enabled_motors = []
start_positions = {}


try:

    # ========================================================
    # 1. Read initial positions of ALL motors
    # ========================================================

    print()
    print("Reading initial positions...")
    print("-" * 80)

    for joint_name, motor in MOTORS.items():

        position = bus.read(
            joint_name,
            ParameterType.MECHANICAL_POSITION,
        )

        start_positions[joint_name] = position

        print(
            f"{joint_name:<25}"
            f"ID 0x{motor.id:02X}   "
            f"{position:+.6f} rad"
        )

        if abs(position) > MAX_START_ABS_POSITION:

            raise RuntimeError(
                f"{joint_name}: suspicious startup position "
                f"{position:+.3f} rad.\n"
                "Refusing to enable motors."
            )


    # ========================================================
    # 2. Disable ALL motors first
    # ========================================================

    print()
    print("Disabling all motors...")
    print("-" * 80)

    for joint_name in MOTORS:

        print(f"  {joint_name}")

        bus.disable(joint_name)


    # ========================================================
    # 3. Configure ALL motors for CSP while disabled
    # ========================================================

    print()
    print("Configuring CSP mode...")
    print("-" * 80)

    for joint_name in MOTORS:

        print(
            f"  {joint_name:<25}"
            f"mode = CSP"
        )

        bus.set_run_mode(
            joint_name,
            5,
        )


    # ========================================================
    # 4. Preload safe target BEFORE enable
    #
    # This is intentionally a raw parameter write.
    #
    # move_to_position_csp() officially assumes that the
    # motor is already enabled.
    #
    # Here we want the target register to contain CURRENT
    # POSITION before enabling, to avoid a startup jump.
    # ========================================================

    print()
    print("Preloading safe startup targets...")
    print("-" * 80)

    for joint_name in MOTORS:

        # Set velocity limit before enabling
        bus.write(
            joint_name,
            ParameterType.VELOCITY_LIMIT,
            VELOCITY_LIMIT,
        )

        # Preload current measured position
        bus.write(
            joint_name,
            ParameterType.POSITION_TARGET,
            start_positions[joint_name],
        )

        print(
            f"  {joint_name:<25}"
            f"target={start_positions[joint_name]:+.4f} rad"
        )


    # ========================================================
    # 5. Enable ALL motors
    # ========================================================

    print()
    print("Enabling all motors...")
    print("-" * 80)

    for joint_name in MOTORS:

        bus.enable(joint_name)

        enabled_motors.append(
            joint_name
        )

        print(
            f"  {joint_name:<25}[OK]"
        )


    # ========================================================
    # 6. Immediately send first CSP command to ALL motors
    #
    # From this point onward, ALL position commands use
    # move_to_position_csp().
    # ========================================================

    print()
    print("Starting CSP command stream...")

    for joint_name in MOTORS:

        bus.move_to_position_csp(
            motor=joint_name,
            position=start_positions[joint_name],
        )


    # ========================================================
    # 7. Slowly move ALL motors to zero using CSP helper
    #
    # Every motor receives a CSP command every control cycle.
    # ========================================================

    print()
    print("Moving all motors slowly to zero using CSP...")
    print("Press Ctrl+C to stop.")
    print()

    zero_start_time = time.perf_counter()

    step = 0

    next_tick = time.perf_counter()


    while True:

        loop_start = time.perf_counter()

        elapsed = (
            loop_start
            - zero_start_time
        )

        alpha = min(
            elapsed / ZEROING_TIME,
            1.0,
        )

        blend = smooth_step(alpha)


        # ----------------------------------------------------
        # CSP command to ALL 12 motors
        # ----------------------------------------------------

        for joint_name in MOTORS:

            target = (
                start_positions[joint_name]
                * (1.0 - blend)
            )

            bus.move_to_position_csp(
                motor=joint_name,
                position=target,
            )


        # ----------------------------------------------------
        # Terminal progress only
        # ----------------------------------------------------

        if step % READBACK_INTERVAL == 0:

            print(
                f"\rMoving to zero: "
                f"{alpha * 100:5.1f} %",
                end="",
                flush=True,
            )


        if alpha >= 1.0:
            break


        # ----------------------------------------------------
        # Timing
        # ----------------------------------------------------

        step += 1

        next_tick += DT

        sleep_time = (
            next_tick
            - time.perf_counter()
        )

        if sleep_time > 0:

            time.sleep(
                sleep_time
            )

        else:

            loop_elapsed = (
                time.perf_counter()
                - loop_start
            )

            print(
                f"\n[WARNING] "
                f"Zeroing loop overrun: "
                f"{loop_elapsed * 1000:.2f} ms "
                f"(target={DT * 1000:.2f} ms)"
            )

            # Avoid accumulating timing delay forever
            next_tick = time.perf_counter()


    # ========================================================
    # 8. Verify zero position
    # ========================================================

    print()
    print()
    print("Verifying zero positions...")
    print("-" * 80)

    for joint_name in MOTORS:

        actual = bus.read(
            joint_name,
            ParameterType.MECHANICAL_POSITION,
        )

        print(
            f"{joint_name:<25}"
            f"{actual:+.6f} rad"
        )

        if abs(actual) > ZERO_TOLERANCE:

            raise RuntimeError(
                f"{joint_name}: failed to reach zero.\n"
                f"Measured position = "
                f"{actual:+.3f} rad"
            )

    # ========================================================
    # 9. STANDBY at zero position
    #
    # Keep sending CSP position=0 to ALL motors while waiting
    # for the user to press Enter.
    #
    # Enter  -> start experiment
    # Ctrl+C -> abort and disable motors
    # ========================================================

    print()
    print("=" * 80)
    print("STANDBY MODE")
    print()
    print("All motors are holding 0 rad using CSP.")
    print("CSP commands will continue to be sent during standby.")
    print()
    print("Press ENTER to start the experimental motion.")
    print("Press Ctrl+C to abort.")
    print("=" * 80)
    print()

    standby_start_time = time.perf_counter()

    next_tick = time.perf_counter()

    standby_step = 0


    while True:

        loop_start = time.perf_counter()


        # ----------------------------------------------------
        # Keep sending ZERO target to ALL motors.
        #
        # This is important because CAN_TIMEOUT is enabled.
        # ----------------------------------------------------

        for joint_name in MOTORS:

            bus.move_to_position_csp(
                motor=joint_name,
                position=0.0,
            )


        # ----------------------------------------------------
        # Check keyboard WITHOUT blocking CSP communication.
        #
        # Because the terminal is normally line-buffered,
        # this becomes true after the user presses Enter.
        # ----------------------------------------------------

        readable, _, _ = select.select(
            [sys.stdin],
            [],
            [],
            0,
        )

        if readable:

            # Remove the Enter/newline from stdin
            sys.stdin.readline()

            print()
            print("ENTER detected.")
            print("Starting experimental motion...")
            print()

            break


        # ----------------------------------------------------
        # Optional status message
        #
        # No additional CAN read is performed here.
        # ----------------------------------------------------

        if standby_step % int(CONTROL_HZ) == 0:

            standby_time = (
                time.perf_counter()
                - standby_start_time
            )

            print(
                f"\rSTANDBY: "
                f"holding all motors at 0 rad "
                f"({standby_time:.1f} s)",
                end="",
                flush=True,
            )


        standby_step += 1


        # ----------------------------------------------------
        # Maintain CSP command frequency
        # ----------------------------------------------------

        next_tick += DT

        sleep_time = (
            next_tick
            - time.perf_counter()
        )


        if sleep_time > 0:

            time.sleep(
                sleep_time
            )

        else:

            loop_elapsed = (
                time.perf_counter()
                - loop_start
            )

            print(
                f"\n[WARNING] "
                f"Standby loop overrun: "
                f"{loop_elapsed * 1000:.2f} ms "
                f"(target={DT * 1000:.2f} ms)"
            )

            # Do not accumulate scheduling delay
            next_tick = time.perf_counter()

    # ========================================================
    # 9. Periodic CSP trajectory
    #
    # EVERY motor receives move_to_position_csp() every cycle.
    #
    # MOTION_OFFSET == 0:
    #       position=0 every cycle
    #
    # MOTION_OFFSET != 0:
    #       0 -> offset -> 0
    # ========================================================

    print()
    print("Zero positions confirmed.")
    print()
    print("Starting CSP periodic trajectory.")
    print()

    print(
        f"Requested command rate : "
        f"{CONTROL_HZ:.1f} Hz"
    )

    print(
        f"Motion frequency       : "
        f"{MOTION_FREQUENCY_HZ:.3f} Hz"
    )

    print(
        f"Velocity limit         : "
        f"{VELOCITY_LIMIT:.3f} rad/s"
    )

    print()
    print("Motion offsets:")

    for joint_name in MOTORS:

        print(
            f"  {joint_name:<25}"
            f"{MOTION_OFFSET[joint_name]:+.4f} rad"
        )

    print()
    print("Press Ctrl+C to stop.")
    print()


    test_start_time = time.perf_counter()

    step = 0

    next_tick = time.perf_counter()


    while True:

        loop_start = time.perf_counter()

        t = (
            loop_start
            - test_start_time
        )


        # ====================================================
        # One-direction periodic trajectory
        #
        # phase:
        #
        # 0 -> 1 -> 0
        #
        # phase is never negative.
        # ====================================================

        phase = (
            0.5
            - 0.5
            * math.cos(
                2.0
                * math.pi
                * MOTION_FREQUENCY_HZ
                * t
            )
        )


        targets = {}


        # ====================================================
        # CSP command to EVERY motor
        # ====================================================

        for joint_name in MOTORS:

            target = (
                MOTION_OFFSET[joint_name]
                * phase
            )

            targets[joint_name] = target


            # =================================================
            # Official CSP helper
            # =================================================

            bus.move_to_position_csp(
                motor=joint_name,
                position=target,
            )


        # ====================================================
        # Slow EXTRA readback for logging
        #
        # NOTE:
        #
        # This is NOT the only CAN reception.
        #
        # Each move_to_position_csp() internally calls write(),
        # and write() receives a Type-2 status frame.
        #
        # These read() calls are additional Type-17 requests.
        # ====================================================

        if step % READBACK_INTERVAL == 0:

            print()
            print(
                f"--- "
                f"t={t:8.2f} s   "
                f"phase={phase:.3f} "
                f"---"
            )


            for joint_name in MOTORS:

                actual = bus.read(
                    joint_name,
                    ParameterType.MECHANICAL_POSITION,
                )

                target = targets[
                    joint_name
                ]

                error = abs(
                    actual
                    - target
                )


                print(
                    f"{joint_name:<25}"
                    f"target={target:+.4f}   "
                    f"actual={actual:+.4f}   "
                    f"error={error:.4f}"
                )


                if error > MAX_TRACKING_ERROR:

                    raise RuntimeError(
                        f"{joint_name}: "
                        f"tracking error too large.\n"
                        f"target = "
                        f"{target:+.3f} rad\n"
                        f"actual = "
                        f"{actual:+.3f} rad\n"
                        f"error  = "
                        f"{error:.3f} rad"
                    )


        # ====================================================
        # Timing
        # ====================================================

        step += 1

        next_tick += DT

        sleep_time = (
            next_tick
            - time.perf_counter()
        )


        if sleep_time > 0:

            time.sleep(
                sleep_time
            )

        else:

            loop_elapsed = (
                time.perf_counter()
                - loop_start
            )

            print(
                f"\n[WARNING] "
                f"Control loop overrun: "
                f"{loop_elapsed * 1000:.2f} ms "
                f"(target={DT * 1000:.2f} ms)"
            )

            # Do not accumulate scheduling delay
            next_tick = time.perf_counter()


except KeyboardInterrupt:

    print()
    print("Stopping...")


except Exception as e:

    print()
    print(
        f"ERROR: "
        f"{type(e).__name__}: {e}"
    )


finally:

    # ========================================================
    # Disable every successfully enabled motor
    # ========================================================

    print()
    print("Disabling motors...")

    for joint_name in enabled_motors:

        try:

            print(
                f"  {joint_name}"
            )

            bus.disable(
                joint_name
            )

        except Exception as e:

            print(
                f"  Disable warning: "
                f"{joint_name}: {e}"
            )


    bus.disconnect(
        disable_torque=False,
    )

    print("Disconnected.")
