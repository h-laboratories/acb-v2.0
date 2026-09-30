#!/usr/bin/env python3
"""CAN wiring check between a USB CAN adapter (CANable / slcan) and an ACB v2.0.

Flash examples/can_test/can_test.ino to the ACB first. That sketch:
  * transmits a heartbeat on ID 0x123 every 2 s, and
  * echoes any frame received on 0x7E0 back on 0x7E1 with the same payload.

This script listens for the heartbeat, then sends pings and waits for pongs.

Interpretation
  heartbeat + pongs   -> CANH/CANL are wired correctly and the bit rate matches.
  nothing at all      -> most likely CANH/CANL swapped, a missing/duplicated
                         120 R termination, wrong bit rate, or the ACB is not
                         running can_test (still in DFU?). Swap H/L and re-run.

Usage
  python tools/can_wiring_test.py                 # auto-detect CANable, 500 kbit/s
  python tools/can_wiring_test.py --port COM5 --bitrate 500000 --listen 6 --pings 5
"""
import argparse
import sys
import time

import can
from serial.tools import list_ports

HEARTBEAT_ID = 0x123
PING_ID = 0x7E0
PONG_ID = 0x7E1

# USB VID:PID pairs for common slcan adapters. CANable 2.0 slcan is 16D0:117E.
KNOWN_ADAPTERS = {(0x16D0, 0x117E): "CANable 2.0 (slcan)",
                  (0xAD50, 0x60C4): "CANable 1.x (slcan)"}


def find_adapter_port():
    for p in list_ports.comports():
        name = KNOWN_ADAPTERS.get((p.vid, p.pid))
        if name:
            return p.device, name
    return None, None


def fmt(msg):
    data = " ".join(f"{b:02X}" for b in msg.data)
    return f"ID 0x{msg.arbitration_id:03X} len {msg.dlc} [{data}]"


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--port", help="serial port of the slcan adapter (default: auto-detect)")
    ap.add_argument("--bitrate", type=int, default=500000, help="CAN bit rate (default 500000)")
    ap.add_argument("--listen", type=float, default=6.0, help="seconds to listen for the 0x123 heartbeat")
    ap.add_argument("--pings", type=int, default=5, help="number of 0x7E0 pings to send")
    args = ap.parse_args()

    port, name = (args.port, "user-specified") if args.port else find_adapter_port()
    if not port:
        print("No slcan adapter found. Pass --port COMx.", file=sys.stderr)
        return 2

    print(f"Adapter : {port} ({name})")
    bus = can.Bus(interface="slcan", channel=port, bitrate=args.bitrate, ttyBaudrate=115200)
    try:
        try:
            hw, sw = bus.get_version(timeout=0.5)
            print(f"Firmware: hw={hw} sw={sw}")
        except Exception:
            pass
        print(f"Bit rate: {args.bitrate} bit/s")

        # ---- Phase 1: passive listen for the ACB heartbeat ----
        print(f"\n[1/2] Listening {args.listen:.0f} s for heartbeat 0x{HEARTBEAT_ID:03X} ...")
        heartbeats = 0
        others = 0
        t_end = time.monotonic() + args.listen
        while time.monotonic() < t_end:
            msg = bus.recv(timeout=0.2)
            if msg is None:
                continue
            if msg.arbitration_id == HEARTBEAT_ID:
                heartbeats += 1
                print(f"  heartbeat  {fmt(msg)}")
            else:
                others += 1
                print(f"  other      {fmt(msg)}")
        print(f"  -> {heartbeats} heartbeat(s), {others} other frame(s)")

        # ---- Phase 2: ping / pong round trip ----
        print(f"\n[2/2] Sending {args.pings} ping(s) on 0x{PING_ID:03X}, expecting echo on 0x{PONG_ID:03X} ...")
        pongs = 0
        latencies = []
        for i in range(args.pings):
            payload = bytes([0xA5, i & 0xFF, 0x00, 0x11, 0x22, 0x33, 0x44, 0x55])
            bus.send(can.Message(arbitration_id=PING_ID, data=payload, is_extended_id=False))
            t0 = time.monotonic()
            got = False
            while time.monotonic() - t0 < 1.0:
                msg = bus.recv(timeout=0.1)
                if msg is None:
                    continue
                if msg.arbitration_id == PONG_ID:
                    ok = bytes(msg.data) == payload
                    dt = (time.monotonic() - t0) * 1e3
                    latencies.append(dt)
                    pongs += 1 if ok else 0
                    print(f"  ping {i}: pong {fmt(msg)}  {'payload OK' if ok else 'PAYLOAD MISMATCH'}  {dt:.1f} ms")
                    got = True
                    break
            if not got:
                print(f"  ping {i}: no pong within 1 s")
            time.sleep(0.2)
        print(f"  -> {pongs}/{args.pings} pongs")

        # ---- Verdict ----
        print("\nVerdict:")
        if heartbeats and pongs == args.pings:
            print("  PASS - wiring is correct: heartbeat received and every ping was echoed.")
            return 0
        if heartbeats and pongs:
            print("  MOSTLY OK - bus works but some pings were lost. Check termination / noise.")
            return 0
        if heartbeats and not pongs:
            print("  PARTIAL - heartbeat seen but no echo. Physical layer is fine; is the ACB")
            print("            running the current can_test sketch (with 0x7E0 echo)?")
            return 1
        print("  FAIL - nothing received from the ACB.")
        print("         Likely CANH/CANL swapped on the USB adapter. Other causes: no 120 R")
        print("         termination, bit-rate mismatch, or the ACB is not running can_test.")
        return 1
    finally:
        bus.shutdown()


if __name__ == "__main__":
    sys.exit(main())
