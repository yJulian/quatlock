#!/usr/bin/env python3
# ---------------------------------------------------------------------------
# build_sw.py - Vitis Unified script: build the platform and the application
#
#   vitis -s sw/scripts/build_sw.py
#   vitis -s sw/scripts/build_sw.py -- --xsa <path.xsa>
#
# Output:
#   build/vitis/gimbal_plat/export/gimbal_plat/gimbal_plat.xpfm
#   build/vitis/gimbal_app/build/gimbal_app.elf
#
# The equivalent steps for the Vitis GUI are documented in the README.
# ---------------------------------------------------------------------------

import argparse
import os
import shutil
import sys

import vitis

HERE = os.path.dirname(os.path.abspath(__file__))
SW   = os.path.abspath(os.path.join(HERE, ".."))
ROOT = os.path.abspath(os.path.join(SW, ".."))

DEFAULT_XSA = os.path.join(ROOT, "build", "gimbal.xsa")
WORKSPACE   = os.path.join(ROOT, "build", "vitis")

PLATFORM = "gimbal_plat"
APP      = "gimbal_app"
CPU      = "psu_cortexa53_0"
DOMAIN   = "standalone_" + CPU


def platform_xpfm(workspace):
    """Path of the freshly built platform inside the workspace."""
    return os.path.join(workspace, PLATFORM, "export", PLATFORM,
                        PLATFORM + ".xpfm")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--xsa", default=DEFAULT_XSA)
    ap.add_argument("--workspace", default=WORKSPACE)
    ap.add_argument("--clean", action="store_true",
                    help="delete the workspace before building")
    args = ap.parse_args()

    if not os.path.isfile(args.xsa):
        sys.exit("XSA not found: %s\n"
                 "Run 'vivado -mode batch -source hw/scripts/build_hw.tcl' "
                 "first." % args.xsa)

    if args.clean and os.path.isdir(args.workspace):
        shutil.rmtree(args.workspace)
    os.makedirs(args.workspace, exist_ok=True)

    client = vitis.create_client()
    client.set_workspace(path=args.workspace)

    # ---- Platform -------------------------------------------------------
    print("== platform '%s' from %s" % (PLATFORM, args.xsa))
    client.create_platform_component(name=PLATFORM,
                                     hw_design=args.xsa,
                                     os="standalone",
                                     cpu=CPU,
                                     no_boot_bsp=True)
    plat = client.get_component(name=PLATFORM)
    plat.build()

    xpfm = platform_xpfm(args.workspace)
    if not os.path.isfile(xpfm):
        # fall back to a platform registered in the Vitis repositories
        xpfm = client.find_platform_in_repos(PLATFORM)
    print("   xpfm: %s" % xpfm)

    # ---- Application ----------------------------------------------------
    print("== application '%s'" % APP)
    comp = client.create_app_component(name=APP,
                                       platform=xpfm,
                                       domain=DOMAIN,
                                       template="empty_application")

    comp.import_files(from_loc=os.path.join(SW, "include"),
                      files=["gimbal_regs.h"],
                      dest_dir_in_cmp="src")
    comp.import_files(from_loc=os.path.join(SW, "drivers"),
                      files=["gimbal_ctrl.h", "gimbal_ctrl.c"],
                      dest_dir_in_cmp="src")
    comp.import_files(from_loc=os.path.join(SW, "app"),
                      files=["shell.h", "shell.c", "main.c"],
                      dest_dir_in_cmp="src")

    comp.build()

    print("== done")
    print("   ELF: %s" % os.path.join(args.workspace, APP, "build",
                                      APP + ".elf"))

    vitis.dispose()


if __name__ == "__main__":
    main()
