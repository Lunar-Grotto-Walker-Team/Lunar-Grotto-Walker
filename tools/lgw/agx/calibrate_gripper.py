#!/usr/bin/env python3
"""Set current gripper position as new zero point via SDK call.

Strategy: disable -> calibrate -> re-enable arm.
Per pyAgxArm SDK: calibrate_gripper sends ArmMsgGripperCtrl(set_zero=0xAE),
which marks the current physical position as the new zero reference.
After this, gripper_status.homing_status becomes true and width readings
become calibrated.
"""
import sys
import time

from pyAgxArm import create_agx_arm_config, AgxArmFactory, ArmModel


def main(can_port: str = "can0") -> int:
    config = create_agx_arm_config(
        robot=ArmModel.PIPER, comm="can", channel=can_port
    )
    arm = AgxArmFactory.create_arm(config)
    print(f"[connect] trying CAN {can_port} ...")
    try:
        arm.connect()
    except Exception as e:
        print(f"[FAIL] arm.connect() raised: {type(e).__name__}: {e}")
        return 1
    if not arm.is_connected():
        print("[FAIL] is_connected() == False after connect")
        return 1
    print("[OK] CAN connected")

    try:
        gripper = arm.init_effector(arm.OPTIONS.EFFECTOR.AGX_GRIPPER)
    except Exception as e:
        print(f"[FAIL] init_effector: {type(e).__name__}: {e}")
        arm.disconnect()
        return 1
    if gripper is None:
        print("[FAIL] gripper is None")
        arm.disconnect()
        return 1
    # is_ok() may return False if gripper is currently disabled.
    # We proceed; later steps only need get_status / calibrate to work.
    print(f"[probe] gripper.is_ok()={gripper.is_ok()}")
    print("[OK] AGX_GRIPPER effector obtained")

    # Read state before
    time.sleep(0.2)
    for _ in range(40):
        gs = gripper.get_gripper_status()
        if gs is not None and gs.hz > 0:
            print(
                f"[BEFORE] width={gs.msg.value:.4f} force={gs.msg.force:.3f} "
                f"driver_enable={gs.msg.foc_status.driver_enable_status} "
                f"homing={gs.msg.foc_status.homing_status} mode={gs.msg.mode}"
            )
            break
        time.sleep(0.05)

    # Step 1: disable gripper driver (release servo lock)
    print("[STEP1] disable_gripper()")
    if not gripper.disable_gripper():
        print("[WARN] disable_gripper returned False, continue anyway")
    time.sleep(0.5)

    # Step 2: calibrate (set current as zero)
    print("[STEP2] calibrate_gripper(timeout=3.0)")
    ok = gripper.calibrate_gripper(timeout=3.0)
    print(f"[STEP2] result={ok}")
    if not ok:
        print("[FAIL] calibrate did not succeed")
        arm.disconnect()
        return 1

    # Wait for status update
    time.sleep(0.5)

    # Step 3: read state after
    print("[STEP3] after calibrate, read status")
    seen_homing = None
    for _ in range(40):
        gs = gripper.get_gripper_status()
        if gs is not None and gs.hz > 0:
            homing = gs.msg.foc_status.homing_status
            drv = gs.msg.foc_status.driver_enable_status
            print(
                f"[AFTER] width={gs.msg.value:.4f} force={gs.msg.force:.3f} "
                f"driver_enable={drv} homing={homing}"
            )
            if seen_homing is None or homing != seen_homing:
                seen_homing = homing
        time.sleep(0.1)

    arm.disconnect()
    print("[DONE] gripper calibration complete")
    return 0


if __name__ == "__main__":
    sys.exit(main(*sys.argv[1:]))
