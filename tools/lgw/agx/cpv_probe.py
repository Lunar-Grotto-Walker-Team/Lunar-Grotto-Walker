#!/usr/bin/env python3
"""Read-only CPV capability probe.

Sends NO motion commands. It connects to the arm over CAN and reads the
CPV contour-velocity parameter for each joint (get_cpv_cv). A numeric
return means the firmware answers CPV queries; None/exception means the
CPV stack is not available on this arm/firmware.

Run inside self_l4t:
    source /my_ws/l4t/l4t_setup.sh
    python3 scripts/cpv_probe.py
"""
import sys
import time

from pyAgxArm import create_agx_arm_config, AgxArmFactory, PiperFW


def main() -> int:
    print("[probe] creating config (read-only, no motion)")
    cfg = create_agx_arm_config(
        robot="piper",
        comm="can",
        channel="can0",
        firmeware_version=PiperFW.V188,
    )
    arm = AgxArmFactory.create_arm(cfg)

    print("[probe] connecting ...")
    arm.connect()
    time.sleep(0.3)

    try:
        fw = arm.get_firmware()
        print(f"[probe] firmware = {fw}")
    except Exception as exc:  # noqa: BLE001 - probe only
        print(f"[probe] get_firmware failed: {exc}")

    ok = 0
    for joint in range(1, 7):
        try:
            cv = arm.get_cpv_cv(joint, timeout=0.8, min_interval=0.2)
            print(f"[probe] joint{joint} cpv_cv = {cv}")
            if cv is not None:
                ok += 1
        except Exception as exc:  # noqa: BLE001 - probe only
            print(f"[probe] joint{joint} cpv_cv ERROR: {exc}")

    print(f"[probe] numeric readings: {ok}/6")
    try:
        arm.disconnect()
    except Exception as exc:  # noqa: BLE001 - probe only
        print(f"[probe] disconnect warning: {exc}")

    return 0 if ok > 0 else 1


if __name__ == "__main__":
    sys.exit(main())
