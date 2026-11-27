#!/usr/bin/env python3
"""Direct SDK move_gripper_m test, bypassing ROS.

Compare ROS gripper_action_server path vs direct SDK to isolate
where the signal is lost.
"""
import sys
import time

from pyAgxArm import create_agx_arm_config, AgxArmFactory, ArmModel


def main(can_port: str = "can0") -> int:
    config = create_agx_arm_config(
        robot=ArmModel.PIPER, comm="can", channel=can_port
    )
    arm = AgxArmFactory.create_arm(config)
    arm.connect()
    gripper = arm.init_effector(arm.OPTIONS.EFFECTOR.AGX_GRIPPER)

    # Enable the arm joints (driver reset behavior)
    print("[STEP0] enable arm")
    arm.enable()
    time.sleep(0.5)

    print("[STEP1] disable gripper (clean state)")
    gripper.disable_gripper()
    time.sleep(0.3)

    print("[STEP2] calibrate (current pos = zero)")
    gripper.calibrate_gripper(timeout=2.0)
    time.sleep(0.5)

    print("[STEP3] read status before move")
    for _ in range(20):
        gs = gripper.get_gripper_status()
        if gs is not None and gs.hz > 0:
            print(
                f"[BEFORE] width={gs.msg.value:.5f} "
                f"force={gs.msg.force:.3f} "
                f"drv_en={gs.msg.foc_status.driver_enable_status} "
                f"homing={gs.msg.foc_status.homing_status}"
            )
            break
        time.sleep(0.05)

    # Move to 0.05m via SDK
    print("[STEP4] move_gripper_m(value=0.05, force=1.0) and watch for 2s")
    t0 = time.time()
    gripper.move_gripper_m(value=0.05, force=1.0)
    samples = []
    while time.time() - t0 < 2.0:
        gs = gripper.get_gripper_status()
        if gs is not None and gs.hz > 0:
            samples.append((time.time() - t0, gs.msg.value, gs.msg.force))
        time.sleep(0.1)
    print(f"[STEP4] got {len(samples)} samples:")
    for t, w, f in samples[:8]:
        print(f"  t={t:.2f}s width={w:.5f} force={f:.3f}")

    print("[STEP5] release to 0 (close)")
    gripper.move_gripper_m(value=0.0, force=1.0)
    time.sleep(2.0)
    for _ in range(20):
        gs = gripper.get_gripper_status()
        if gs is not None and gs.hz > 0:
            print(
                f"[AFTER ] width={gs.msg.value:.5f} "
                f"force={gs.msg.force:.3f} "
                f"drv_en={gs.msg.foc_status.driver_enable_status}"
            )
            break
        time.sleep(0.05)

    arm.disconnect()
    return 0


if __name__ == "__main__":
    sys.exit(main(*sys.argv[1:]))
