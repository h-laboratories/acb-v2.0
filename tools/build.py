#!/usr/bin/env python3
"""Build (and optionally DFU-flash) the ACB v2.0 firmware images with arduino-cli.

Targets
  bootloader  src/acb_can_bootloader   -> 0x08000000   (no USB, <= 28 KB)
  app         src/acb_v2.0_firmware    -> 0x08008000   (USB CDC serial, linked with flash_offset 0x8000)
  can_test    examples/can_test        -> 0x08000000   (stand-alone CAN test, replaces the bootloader!)

Examples
  python tools/build.py bootloader app          # build both into build/<target>/
  python tools/build.py bootloader --dfu        # build and write over USB DFU
  python tools/build.py app --dfu --run         # build, write at 0x08008000 and start
  python tools/build.py app --can --node 1      # build, then flash over CAN with acb_can_flash.py

Uses the arduino-cli bundled with the Arduino IDE if none is on PATH, and the
STM32 core's board options from the README (Generic G4 / G474RETx).
"""
import argparse
import os
import shutil
import subprocess
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
FQBN_BASE = "STMicroelectronics:stm32:GenG4:pnum=GENERIC_G474RETX,xserial=disabled"

TARGETS = {
    "bootloader": {
        "sketch": "src/acb_can_bootloader",
        "usb": "none",
        "props": {"upload.maximum_size": "28672"},        # must end before the boot-config page @0x7000
        "addr": 0x08000000,
    },
    "app": {
        "sketch": "src/acb_v2.0_firmware",
        "usb": "CDCgen",
        "props": {"build.flash_offset": "0x8000", "upload.maximum_size": "479232"},  # 0x75000: keep anti-cogging map (0x7D000) + EEPROM page (0x7F000)
        "addr": 0x08008000,
    },
    "can_test": {
        "sketch": "examples/can_test",
        "usb": "CDCgen",
        "props": {},
        "addr": 0x08000000,
    },
    "epc": {   # EPC91120 GaN inverter (STM32G431CBU6), programmed over SWD with an ST-LINK; serial on USART2/VCP
        "sketch": "src/epc91120_firmware",
        "usb": "none",
        "fqbn": "STMicroelectronics:stm32:GenG4:pnum=GENERIC_G431CBUX,xserial=generic",
        "props": {},
        "addr": 0x08000000,
    },
}


def find_arduino_cli():
    p = shutil.which("arduino-cli")
    if p:
        return p
    local = os.environ.get("LOCALAPPDATA", "")
    cand = os.path.join(local, "Programs", "Arduino IDE", "resources", "app", "lib", "backend", "resources", "arduino-cli.exe")
    if os.path.exists(cand):
        return cand
    sys.exit("arduino-cli not found (install it or the Arduino IDE 2.x)")


def find_cube_programmer():
    p = shutil.which("STM32_Programmer_CLI")
    if p:
        return p
    cand = r"C:\Program Files\STMicroelectronics\STM32Cube\STM32CubeProgrammer\bin\STM32_Programmer_CLI.exe"
    if os.path.exists(cand):
        return cand
    sys.exit("STM32_Programmer_CLI not found (install STM32CubeProgrammer)")


def build(target, cli):
    t = TARGETS[target]
    out = os.path.join(ROOT, "build", target)
    os.makedirs(out, exist_ok=True)
    fqbn = t.get("fqbn") or f"{FQBN_BASE},usb={t['usb']}"
    cmd = [cli, "compile", "--fqbn", fqbn, "--output-dir", out]
    for k, v in t["props"].items():
        cmd += ["--build-property", f"{k}={v}"]
    cmd.append(os.path.join(ROOT, t["sketch"]))
    print(f"== building {target}: {' '.join(cmd[1:])}")
    r = subprocess.run(cmd, capture_output=True, text=True)
    lines = [l for l in (r.stdout + r.stderr).splitlines()
             if "error" in l.lower() or l.startswith("Sketch uses") or l.startswith("Global variables")]
    print("\n".join(lines))
    if r.returncode != 0:
        print(r.stdout[-4000:], r.stderr[-4000:])
        sys.exit(f"build of {target} failed")
    name = os.path.basename(t["sketch"]) + ".ino.bin"
    bin_path = os.path.join(out, name)
    print(f"   -> {os.path.relpath(bin_path, ROOT)} ({os.path.getsize(bin_path)} bytes)")
    return bin_path


def dfu_flash(bin_path, addr, run):
    prog = find_cube_programmer()
    cmd = [prog, "-c", "port=usb1", "-w", bin_path, f"0x{addr:08X}", "-v"]
    if run:
        cmd.append("-g")
    print(f"== DFU: {' '.join(cmd[1:])}")
    r = subprocess.run(cmd, capture_output=True, text=True)
    keep = [l for l in r.stdout.splitlines() if any(k in l for k in ("Error", "error", "verified", "Start operation", "Download in Progress"))]
    print("\n".join(keep))
    if r.returncode != 0 or "verified successfully" not in r.stdout:
        print(r.stdout[-3000:])
        sys.exit("DFU flash failed (is the ACB in DFU mode?)")


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("targets", nargs="+", choices=sorted(TARGETS))
    ap.add_argument("--dfu", action="store_true", help="write each image over USB DFU at its address")
    ap.add_argument("--run", action="store_true", help="with --dfu: start the MCU after the last write")
    ap.add_argument("--can", action="store_true", help="app only: flash over CAN with tools/acb_can_flash.py")
    ap.add_argument("--node", default="1", help="with --can: target node id")
    args = ap.parse_args()

    cli = find_arduino_cli()
    bins = {t: build(t, cli) for t in args.targets}

    if args.dfu:
        for i, t in enumerate(args.targets):
            dfu_flash(bins[t], TARGETS[t]["addr"], run=args.run and i == len(args.targets) - 1)
    if args.can:
        if "app" not in bins:
            sys.exit("--can only applies to the app target")
        cmd = [sys.executable, os.path.join(ROOT, "tools", "acb_can_flash.py"), "--node", args.node, "flash", bins["app"]]
        print(f"== CAN: {' '.join(cmd[1:])}")
        sys.exit(subprocess.call(cmd))


if __name__ == "__main__":
    main()
