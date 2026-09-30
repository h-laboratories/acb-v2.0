#!/usr/bin/env python3
"""Program an ACB v2.0 over CAN using the ACB CAN bootloader.

Requires a slcan USB adapter (CANable) and an ACB running the bootloader from
src/acb_can_bootloader plus an application that includes can_boot.cpp.
Protocol: src/acb_can_bootloader/acb_can_protocol.h.

Examples
  python tools/acb_can_flash.py scan
  python tools/acb_can_flash.py info  --node 1
  python tools/acb_can_flash.py flash build/app/acb_v2.0_firmware.ino.bin --node 1
  python tools/acb_can_flash.py enter --node 1          # reboot into bootloader and stay there
  python tools/acb_can_flash.py run   --node 1          # reset into the application
  python tools/acb_can_flash.py set-node-id 3 --node 1  # node 1 becomes node 3

If the application is not answering (bricked, wrong bitrate, still in DFU),
`flash`/`enter` keep sending ENTER while you power-cycle the ACB; the bootloader
listens for 300 ms after every reset.
"""
import argparse
import struct
import sys
import time
import zlib

import can
from serial.tools import list_ports

# ---- protocol constants (keep in sync with acb_can_protocol.h) ----
CMD_BASE, DATA_BASE, RESP_BASE = 0x600, 0x580, 0x680
BASE_MASK, NODE_MASK = 0x780, 0x07F
BROADCAST = 0

CMD_HELLO, CMD_PING, CMD_ERASE, CMD_WRITE_BEGIN, CMD_WRITE_DONE = 0x00, 0x01, 0x02, 0x03, 0x04
CMD_CRC, CMD_GO, CMD_SET_NODE_ID, CMD_INFO, CMD_ENTER = 0x05, 0x06, 0x07, 0x08, 0x7F
# application control
CMD_SET_MODE, CMD_SET_TARGET, CMD_ENABLE, CMD_SET_POLE_PAIRS, CMD_SET_VLIMIT, CMD_SAVE_CONFIG = 0x10, 0x11, 0x12, 0x13, 0x14, 0x15
CMD_GET_STATE, CMD_GET_TELEMETRY = 0x20, 0x21
CMD_DRV_CLEAR_FAULTS, CMD_DRV_WRITE_REG, CMD_DRV_FAULTS, CMD_DRV_READ_REG = 0x16, 0x17, 0x22, 0x23
CMD_SERIAL_STREAM = 0x18
CMD_RECALIBRATE, CMD_SET_ILIMIT = 0x19, 0x1A
CMD_SET_TORQUE_MODE, CMD_GET_CURRENTS, CMD_GET_DQ = 0x1B, 0x24, 0x25
CMD_DRV_SPI_CFG = 0x1C
CMD_COG_CALIB, CMD_COG_ENABLE, CMD_COG_SAVE, CMD_COG_STATUS, CMD_COG_GET = 0x1D, 0x1E, 0x1F, 0x26, 0x27
COG_STATES = {0: "idle", 1: "calibrating", 2: "done", 3: "aborted"}
TORQUE_MODES = {"voltage": 0, "dc_current": 1, "foc_current": 2}
DRV_FS1_BITS = {10: "FAULT", 9: "VDS_OCP", 8: "GDF", 7: "UVLO", 6: "OTSD",
                5: "VDS_HA", 4: "VDS_LA", 3: "VDS_HB", 2: "VDS_LB", 1: "VDS_HC", 0: "VDS_LC"}
DRV_FS2_BITS = {10: "SA_OC", 9: "SB_OC", 8: "SC_OC", 7: "OTW", 6: "CPUV",
                5: "VGS_HA", 4: "VGS_LA", 3: "VGS_HB", 2: "VGS_LB", 1: "VGS_HC", 0: "VGS_LC"}
DRV_REG_NAMES = {0: "FaultStatus1", 1: "VGSStatus2", 2: "DriverControl", 3: "GateDriveHS",
                 4: "GateDriveLS", 5: "OCPControl", 6: "CSAControl"}
MODES = {"torque": 0, "velocity": 1, "angle": 2, "velocity_openloop": 3, "angle_openloop": 4}
MODE_NAMES = {v: k for k, v in MODES.items()}
RPM_TO_RADS = 2 * 3.141592653589793 / 60

FLAG_APP_VALID, FLAG_IN_BOOTLOADER = 0x01, 0x02
STATUS = {0: "OK", 1: "bad argument / address range", 2: "flash erase/program failed",
          3: "bad state", 4: "readback/CRC mismatch", 5: "unsupported command",
          6: "node is running the application, not the bootloader"}

FLASH_BASE, APP_ADDR, APP_END = 0x08000000, 0x08008000, 0x0807F000
SRAM_BASE, SRAM_END = 0x20000000, 0x20020000
MAX_CHUNK = 2048

KNOWN_ADAPTERS = {(0x16D0, 0x117E): "CANable 2.0 (slcan)", (0xAD50, 0x60C4): "CANable 1.x (slcan)"}


class BootloaderError(Exception):
    pass


class ChunkTimeout(BootloaderError):
    """The bootloader never completed a chunk: data frames were probably dropped
    by the USB adapter's transmit queue. Recoverable by using smaller chunks."""


def find_adapter_port():
    for p in list_ports.comports():
        if (p.vid, p.pid) in KNOWN_ADAPTERS:
            return p.device, KNOWN_ADAPTERS[(p.vid, p.pid)]
    return None, None


def status_text(code):
    return STATUS.get(code, f"status {code}")


class AcbNode:
    def __init__(self, bus, node_id):
        self.bus = bus
        self.node = node_id

    # ---- low level ----
    def _send(self, can_id, data):
        self.bus.send(can.Message(arbitration_id=can_id, data=bytes(data), is_extended_id=False))

    def drain(self):
        while self.bus.recv(timeout=0) is not None:
            pass

    def cmd(self, cmd, payload=b"", node=None):
        node = self.node if node is None else node
        self._send(CMD_BASE | node, bytes([cmd]) + bytes(payload))

    def wait(self, cmds, timeout, node=None):
        """Return (node_id, data) of the first response whose byte0 is in cmds."""
        node = self.node if node is None else node
        deadline = time.monotonic() + timeout
        while True:
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                return None
            msg = self.bus.recv(timeout=min(remaining, 0.1))
            if msg is None or msg.is_extended_id or (msg.arbitration_id & BASE_MASK) != RESP_BASE:
                continue
            src = msg.arbitration_id & NODE_MASK
            if node != BROADCAST and src != node:
                continue
            data = bytes(msg.data)
            if data and data[0] in cmds:
                return src, data

    def request(self, cmd, payload=b"", timeout=1.0, expect=None):
        self.drain()
        self.cmd(cmd, payload)
        r = self.wait({cmd} if expect is None else expect, timeout)
        if r is None:
            raise BootloaderError(f"node {self.node}: no response to command 0x{cmd:02X}")
        return r[1]

    # ---- commands ----
    @staticmethod
    def parse_hello(data):
        if len(data) < 8:
            return None
        return {"proto": data[1], "version": data[2], "flags": data[3],
                "uid": struct.unpack_from("<I", data, 4)[0],
                "in_bootloader": bool(data[3] & FLAG_IN_BOOTLOADER),
                "app_valid": bool(data[3] & FLAG_APP_VALID)}

    def ping(self, timeout=0.3):
        self.drain()
        self.cmd(CMD_PING)
        r = self.wait({CMD_PING}, timeout)
        return self.parse_hello(r[1]) if r else None

    def info(self):
        d = self.request(CMD_INFO)
        if len(d) < 8:
            raise BootloaderError("short INFO response")
        flash_kb, page, app_kb = struct.unpack_from("<HHH", d, 1)
        return {"flash_kb": flash_kb, "page_size": page, "app_start": FLASH_BASE + app_kb * 1024, "dual_bank": bool(d[7])}

    def erase(self, addr, length, timeout=30.0):
        d = self.request(CMD_ERASE, struct.pack("<I", addr) + struct.pack("<I", length)[:3], timeout)
        if d[1] != 0:
            raise BootloaderError(f"erase failed: {status_text(d[1])}")

    def crc(self, addr, length, timeout=5.0):
        d = self.request(CMD_CRC, struct.pack("<I", addr) + struct.pack("<I", length)[:3], timeout)
        if d[1] != 0:
            raise BootloaderError(f"CRC failed: {status_text(d[1])}")
        return struct.unpack_from("<I", d, 2)[0]

    def go(self, stay=False):
        d = self.request(CMD_GO, bytes([1 if stay else 0]))
        if d[1] != 0:
            raise BootloaderError(f"GO failed: {status_text(d[1])}")

    def set_node_id(self, new_id):
        d = self.request(CMD_SET_NODE_ID, bytes([new_id]), timeout=3.0)
        if d[1] != 0:
            raise BootloaderError(f"set node id failed: {status_text(d[1])}")

    def write_chunk(self, addr, chunk, retries=3):
        """Write one chunk (<= MAX_CHUNK bytes, multiple of 8). Returns attempts used."""
        expected = zlib.crc32(chunk) & 0xFFFFFFFF
        last = "no response"
        for attempt in range(1, retries + 1):
            self.drain()
            self.cmd(CMD_WRITE_BEGIN, struct.pack("<IH", addr, len(chunk)))
            for i in range(0, len(chunk), 8):
                self._send(DATA_BASE | self.node, chunk[i:i + 8])
            r = self.wait({CMD_WRITE_BEGIN, CMD_WRITE_DONE}, 3.0)
            if r is None:
                last = "timeout waiting for WRITE_DONE (data frames lost?)"
                continue
            d = r[1]
            if d[0] == CMD_WRITE_BEGIN:
                raise BootloaderError(f"WRITE_BEGIN @0x{addr:08X} rejected: {status_text(d[1])}")
            if d[1] != 0:
                raise BootloaderError(f"write @0x{addr:08X} failed: {status_text(d[1])}")
            got = struct.unpack_from("<I", d, 2)[0]
            if got == expected:
                return attempt
            last = f"chunk CRC mismatch (got {got:08X}, want {expected:08X})"
        if last.startswith("timeout"):
            raise ChunkTimeout(f"write @0x{addr:08X}: {last}")
        raise BootloaderError(f"write @0x{addr:08X} failed after {retries} attempts: {last}")

    # ---- application control ----
    def ctl(self, cmd, payload=b"", timeout=1.0):
        d = self.request(cmd, payload, timeout)
        if cmd < 0x20 and d[1] != 0:
            raise BootloaderError(f"command 0x{cmd:02X} failed: {status_text(d[1])}")
        return d

    def require_app(self):
        p = self.ping(0.5)
        if not p:
            raise BootloaderError(f"node {self.node}: no response")
        if p["in_bootloader"]:
            raise BootloaderError(f"node {self.node} is in the bootloader; use `run` to start the application first")
        return p

    def get_state(self):
        d = self.ctl(CMD_GET_STATE)
        vel, ang = struct.unpack_from("<hi", d, 2)
        return {"mode": MODE_NAMES.get(d[1] & 0x0F, str(d[1] & 0x0F)), "enabled": bool(d[1] & 0x10),
                "velocity": vel / 100.0, "angle": ang / 1e4}

    def get_telemetry(self):
        d = self.ctl(CMD_GET_TELEMETRY)
        bus, tb, tm = struct.unpack_from("<Hhh", d, 1)
        return {"bus_v": bus / 100.0, "board_c": tb / 10.0, "mcu_c": tm / 10.0, "drv_fault": bool(d[7])}

    # ---- higher level ----
    def enter_bootloader(self, powercycle_wait=30.0, log=print):
        """Get the node into the bootloader. Returns the HELLO/PING dict."""
        p = self.ping()
        if p and p["in_bootloader"]:
            return p
        if p:
            log(f"node {self.node}: application answered, asking it to reboot into the bootloader ...")
            self.drain()
            self.cmd(CMD_ENTER)
            r = self.wait({CMD_HELLO}, 3.0)
            if r:
                return self.parse_hello(r[1])
            raise BootloaderError("application acknowledged ENTER but the bootloader never said HELLO")

        log(f"node {self.node}: no response. POWER-CYCLE the ACB now - the bootloader listens for 300 ms after reset.")
        log(f"          (sending ENTER for up to {powercycle_wait:.0f} s ...)")
        deadline = time.monotonic() + powercycle_wait
        while time.monotonic() < deadline:
            self.cmd(CMD_ENTER)
            r = self.wait({CMD_HELLO}, 0.05)
            if r:
                return self.parse_hello(r[1])
        raise BootloaderError("gave up waiting for the bootloader")


# ---- image checks ----
def check_image(image, force=False):
    problems = []
    if len(image) < 8:
        problems.append("file is too small to be a firmware image")
    else:
        sp, pc = struct.unpack_from("<II", image, 0)
        if not (SRAM_BASE < sp <= SRAM_END):
            problems.append(f"initial stack pointer 0x{sp:08X} is not in SRAM")
        if not (pc & 1) or not (APP_ADDR <= pc < APP_ADDR + len(image)):
            problems.append(f"reset vector 0x{pc:08X} is not inside 0x{APP_ADDR:08X}..; "
                            f"build the app with build.flash_offset=0x8000 (tools/build.py app)")
    if len(image) > APP_END - APP_ADDR:
        problems.append(f"image ({len(image)} bytes) does not fit in the application region ({APP_END - APP_ADDR} bytes)")
    if problems and not force:
        raise BootloaderError("refusing to flash:\n  - " + "\n  - ".join(problems))
    return problems


def pad8(b):
    return b + b"\xFF" * (-len(b) % 8)


APP_START_TIMEOUT = 20.0   # SimpleFOC's initFOC alignment makes the app slow to reach loop()


def wait_for_app(node, timeout=APP_START_TIMEOUT):
    """Ping until the application (not the bootloader) answers, or timeout.
    Returns (hello dict or None, seconds waited)."""
    t0 = time.monotonic()
    while time.monotonic() - t0 < timeout:
        p = node.ping(0.3)
        if p and not p["in_bootloader"]:
            return p, time.monotonic() - t0
        time.sleep(0.2)
    return None, time.monotonic() - t0


# ---- CLI ----
def open_bus(args):
    port, name = (args.port, "user-specified") if args.port else find_adapter_port()
    if not port:
        raise BootloaderError("no slcan adapter found; pass --port COMx")
    print(f"Adapter : {port} ({name}) @ {args.bitrate} bit/s")
    return can.Bus(interface="slcan", channel=port, bitrate=args.bitrate, ttyBaudrate=115200)


def describe(node_id, p):
    state = "bootloader" if p["in_bootloader"] else "application"
    return (f"node {node_id:3d}: {state:11s} v{p['version']}  proto {p['proto']}  "
            f"uid {p['uid']:08X}  app {'valid' if p['app_valid'] else 'INVALID'}")


def cmd_scan(bus, args):
    n = AcbNode(bus, BROADCAST)
    n.drain()
    n.cmd(CMD_PING, node=BROADCAST)
    seen = {}
    deadline = time.monotonic() + args.timeout
    while time.monotonic() < deadline:
        r = n.wait({CMD_PING}, deadline - time.monotonic(), node=BROADCAST)
        if r and r[0] not in seen:
            seen[r[0]] = AcbNode.parse_hello(r[1])
    if not seen:
        print("no nodes answered")
        return 1
    for nid in sorted(seen):
        print(describe(nid, seen[nid]))
    return 0


def cmd_info(bus, args):
    n = AcbNode(bus, args.node)
    p = n.ping(0.5)
    if not p:
        print(f"node {args.node}: no response")
        return 1
    print(describe(args.node, p))
    if p["in_bootloader"]:
        i = n.info()
        print(f"          flash {i['flash_kb']} KB, page {i['page_size']} B, {'dual' if i['dual_bank'] else 'single'} bank, "
              f"app @ 0x{i['app_start']:08X}")
    return 0


def cmd_enter(bus, args):
    n = AcbNode(bus, args.node)
    p = n.enter_bootloader(args.wait)
    print(describe(args.node, p))
    return 0


def cmd_run(bus, args):
    n = AcbNode(bus, args.node)
    p = n.ping(0.5)
    if not p:
        print(f"node {args.node}: no response")
        return 1
    n.go(stay=False)
    print(f"node {args.node}: reset issued")
    p, dt = wait_for_app(n)
    print(f"{describe(args.node, p)}  (up after {dt:.1f} s)" if p else f"node {args.node}: no answer within {APP_START_TIMEOUT:.0f} s")
    return 0


def cmd_set_node_id(bus, args):
    n = AcbNode(bus, args.node)
    n.enter_bootloader(args.wait)
    n.set_node_id(args.new_id)
    print(f"node {args.node} -> {args.new_id}; waiting for it to come back ...")
    m = AcbNode(bus, args.new_id)
    r = m.wait({CMD_HELLO}, 3.0)
    if not r:
        print("no HELLO from the new id (it may still be in the bootloader - try `info`)")
        return 1
    print(describe(args.new_id, AcbNode.parse_hello(r[1])))
    if not args.stay:
        m.go(stay=False)
        print("reset into application")
    return 0


def cmd_flash(bus, args):
    with open(args.image, "rb") as f:
        image = pad8(f.read())
    for w in check_image(image, args.force):
        print(f"WARNING: {w}")
    print(f"Image   : {args.image} ({len(image)} bytes, crc32 {zlib.crc32(image) & 0xFFFFFFFF:08X})")

    n = AcbNode(bus, args.node)
    t_start = time.monotonic()
    p = n.enter_bootloader(args.wait)
    print(describe(args.node, p))
    i = n.info()
    if i["app_start"] != APP_ADDR:
        raise BootloaderError(f"bootloader reports app start 0x{i['app_start']:08X}, tool expects 0x{APP_ADDR:08X}")

    print(f"Erasing {len(image)} bytes @ 0x{APP_ADDR:08X} ...", end=" ", flush=True)
    t = time.monotonic()
    n.erase(APP_ADDR, len(image))
    print(f"done ({time.monotonic() - t:.1f} s)")

    chunk = args.chunk
    if chunk % 8 or not 8 <= chunk <= MAX_CHUNK:
        raise BootloaderError(f"--chunk must be a multiple of 8 between 8 and {MAX_CHUNK}")
    print(f"Writing in {chunk}-byte chunks ...")
    t = time.monotonic()
    retries = 0
    next_pct = 10
    off = 0
    while off < len(image):
        part = image[off:off + chunk]
        try:
            retries += n.write_chunk(APP_ADDR + off, part) - 1
        except ChunkTimeout:
            if chunk <= 64:
                raise
            chunk //= 2
            print(f"  data frames lost (adapter TX queue overrun?) - retrying with {chunk}-byte chunks", flush=True)
            continue
        off += len(part)
        pct = off * 100 // len(image)
        if pct >= next_pct:
            print(f"  {pct:3d}%  0x{APP_ADDR + off:08X}", flush=True)
            next_pct += 10
    dt = time.monotonic() - t
    print(f"  written {len(image)} bytes in {dt:.1f} s ({len(image) / dt / 1024:.1f} KB/s, {retries} chunk retries)")

    if not args.no_verify:
        print("Verifying ...", end=" ", flush=True)
        want = zlib.crc32(image) & 0xFFFFFFFF
        got = n.crc(APP_ADDR, len(image))
        if got != want:
            raise BootloaderError(f"verify FAILED: device crc {got:08X}, image crc {want:08X}")
        print(f"OK (crc32 {got:08X})")

    if args.stay:
        print("Leaving the node in the bootloader (--stay).")
    else:
        n.go(stay=False)
        print("Reset into application ...", end=" ", flush=True)
        p, dt = wait_for_app(n)
        print(f"{describe(args.node, p)}  (up after {dt:.1f} s)" if p else f"no PING answer within {APP_START_TIMEOUT:.0f} s (is the application stuck?).")
    print(f"Total {time.monotonic() - t_start:.1f} s")
    return 0


def fmt_state(st):
    rpm = st["velocity"] / RPM_TO_RADS
    return (f"mode {st['mode']:18s} {'ENABLED ' if st['enabled'] else 'disabled'}  "
            f"vel {st['velocity']:8.2f} rad/s ({rpm:7.1f} rpm)  angle {st['angle']:9.3f} rad")


def fmt_telemetry(t):
    return f"bus {t['bus_v']:.2f} V  board {t['board_c']:.1f} C  mcu {t['mcu_c']:.1f} C  drv_fault {int(t['drv_fault'])}"


def cmd_ctl(bus, args):
    n = AcbNode(bus, args.node)
    n.require_app()
    c = args.command
    if c == "mode":
        n.ctl(CMD_SET_MODE, bytes([MODES[args.mode]]))
        print(f"mode -> {args.mode}")
    elif c == "target":
        v = args.value * RPM_TO_RADS if args.rpm else args.value
        n.ctl(CMD_SET_TARGET, struct.pack("<f", v))
        print(f"target -> {v:.4f}" + (f" rad/s ({args.value} rpm)" if args.rpm else ""))
    elif c == "enable":
        n.ctl(CMD_ENABLE, b"\x01")
        print("motor enabled")
    elif c == "disable":
        n.ctl(CMD_ENABLE, b"\x00")
        print("motor disabled")
    elif c == "pole-pairs":
        n.ctl(CMD_SET_POLE_PAIRS, bytes([args.pp]))
        print(f"pole pairs -> {args.pp} (runtime only; use save-config to persist)")
    elif c == "vlimit":
        n.ctl(CMD_SET_VLIMIT, struct.pack("<f", args.volts))
        print(f"voltage limit -> {args.volts} V")
    elif c == "save-config":
        n.ctl(CMD_SAVE_CONFIG, timeout=3.0)
        print("config saved to EEPROM")
    elif c == "state":
        print(fmt_state(n.get_state()))
    elif c == "telemetry":
        print(fmt_telemetry(n.get_telemetry()))
    elif c == "cog":
        def cog_status():
            d = n.ctl(CMD_COG_STATUS)
            idx, total = struct.unpack_from("<HH", d, 3)
            return {"state": COG_STATES.get(d[1], str(d[1])), "valid": bool(d[2] & 1), "enabled": bool(d[2] & 2),
                    "saved": bool(d[2] & 4), "index": idx, "n": total, "timeouts": d[7]}
        def fmt_cog(s):
            return (f"state {s['state']:<11} valid {int(s['valid'])} enabled {int(s['enabled'])} saved {int(s['saved'])}  "
                    f"index {s['index']}/{s['n']}  timeouts {s['timeouts']}")
        a = args.action
        if a == "status":
            print(fmt_cog(cog_status()))
        elif a == "calib":
            vel = max(1, min(255, int(round(args.vel * 100))))
            payload = bytes([1, max(1, min(255, args.pos_counts)), vel, max(1, min(255, args.dwell_ms)),
                             max(1, min(255, args.timeout_ms // 10))])
            n.ctl(CMD_COG_CALIB, payload)
            s = cog_status()
            print(f"anti-cogging calibration started: {s['n']} points, settle <= {args.pos_counts} count(s) & {args.vel} rad/s "
                  f"for {args.dwell_ms} ms, timeout {args.timeout_ms} ms per point")
            if args.watch:
                t0 = time.monotonic()
                while True:
                    time.sleep(2.0)
                    s = cog_status()
                    print(f"  t={time.monotonic()-t0:5.0f}s  {fmt_cog(s)}", flush=True)
                    if s["state"] != "calibrating":
                        break
                print("finished:", s["state"], "- use `cog save` (with the motor disabled) to persist it")
        elif a == "abort":
            n.ctl(CMD_COG_CALIB, b"\x00")
            print("calibration aborted")
        elif a in ("enable", "disable"):
            n.ctl(CMD_COG_ENABLE, b"\x01" if a == "enable" else b"\x00")
            print(f"anti-cogging feed-forward {a}d")
        elif a == "save":
            n.ctl(CMD_COG_SAVE)
            print("map saved to flash")
        elif a == "dump":
            total = cog_status()["n"]
            vals = []
            for i in range(0, total, 2):
                d = n.ctl(CMD_COG_GET, struct.pack("<H", i))
                v0, v1 = struct.unpack_from("<hh", d, 3)
                vals += [v0, v1]
            vals = vals[:total]
            if args.out:
                with open(args.out, "w") as f:
                    f.write("index,angle_deg,current_mA\n")
                    for i, v in enumerate(vals):
                        f.write(f"{i},{360.0*i/total:.4f},{v}\n")
                print(f"wrote {total} points to {args.out}")
            mean = sum(vals) / total
            rms = (sum((v - mean) ** 2 for v in vals) / total) ** 0.5
            print(f"{total} points: min {min(vals)} mA  max {max(vals)} mA  mean {mean:+.0f} mA  rms(about mean) {rms:.0f} mA")
    elif c == "ilimit":
        n.ctl(CMD_SET_ILIMIT, struct.pack("<f", args.amps))
        print(f"current limit -> {args.amps} A")
    elif c == "recalibrate":
        print("running sensor alignment (motor will twitch) ...")
        d = n.ctl(CMD_RECALIBRATE, struct.pack("<f", args.volts) if args.volts else b"", timeout=20.0)
        direction = struct.unpack_from("<b", d, 2)[0]
        zea = struct.unpack_from("<f", d, 3)[0]
        print(f"sensor_direction {direction:+d} ({'CW' if direction == 1 else 'CCW' if direction == -1 else 'UNKNOWN'})  "
              f"zero_electric_angle {zea:.4f} rad  (not saved; use save-config)")
        if direction == 0:
            print("alignment FAILED (direction unknown) - motor did not move? try --volts higher")
            return 1
    elif c == "torque-mode":
        n.ctl(CMD_SET_TORQUE_MODE, bytes([TORQUE_MODES[args.tmode]]))
        print(f"torque controller -> {args.tmode}")
    elif c == "currents":
        for _ in range(args.count):
            d = n.ctl(CMD_GET_CURRENTS); a, b, cc = struct.unpack_from("<hhh", d, 1)
            q = n.ctl(CMD_GET_DQ); iq, idd, uq = struct.unpack_from("<hhh", q, 1)
            print(f"phase a {a/1000:+.3f} A  b {b/1000:+.3f} A  c {cc/1000:+.3f} A  |  iq {iq/1000:+.3f} A  id {idd/1000:+.3f} A  uq {uq/1000:+.3f} V")
            time.sleep(0.2)
    elif c == "stream":
        n.ctl(CMD_SERIAL_STREAM, bytes([args.hz]))
        print(f"serial position stream -> {args.hz} Hz" if args.hz else "serial position stream off")
    elif c == "drv-spi":
        n.ctl(CMD_DRV_SPI_CFG, bytes([args.mode]) + struct.pack("<H", args.khz))
        print(f"DRV8323 SPI -> mode {args.mode}, {args.khz} kHz")
    elif c == "drv-faults":
        d = n.ctl(CMD_DRV_FAULTS)
        fs1, fs2 = struct.unpack_from("<HH", d, 2)
        f1 = [nm for b, nm in DRV_FS1_BITS.items() if fs1 & (1 << b)]
        f2 = [nm for b, nm in DRV_FS2_BITS.items() if fs2 & (1 << b)]
        print(f"nFAULT {'ASSERTED' if d[1] else 'clear'}  FaultStatus1 0x{fs1:03X} [{' '.join(f1) or '-'}]  "
              f"VGSStatus2 0x{fs2:03X} [{' '.join(f2) or '-'}]")
    elif c == "drv-clear":
        n.ctl(CMD_DRV_CLEAR_FAULTS)
        print("DRV8323 faults cleared")
    elif c == "drv-reg":
        if args.value is None:
            d = n.ctl(CMD_DRV_READ_REG, bytes([args.addr]))
            print(f"reg 0x{args.addr:02X} {DRV_REG_NAMES.get(args.addr, '?'):14s} = 0x{struct.unpack_from('<H', d, 2)[0]:03X}")
        else:
            d = n.ctl(CMD_DRV_WRITE_REG, bytes([args.addr]) + struct.pack("<H", args.value))
            print(f"reg 0x{args.addr:02X} {DRV_REG_NAMES.get(args.addr, '?'):14s} <- 0x{args.value:03X}, readback 0x{struct.unpack_from('<H', d, 3)[0]:03X}")
    elif c == "spin":
        v = args.rpm * RPM_TO_RADS
        mode = "velocity_openloop" if args.openloop else "velocity"
        if args.pole_pairs:
            n.ctl(CMD_SET_POLE_PAIRS, bytes([args.pole_pairs]))
        if args.vlimit is not None:
            n.ctl(CMD_SET_VLIMIT, struct.pack("<f", args.vlimit))
        n.ctl(CMD_SET_MODE, bytes([MODES[mode]]))
        n.ctl(CMD_SET_TARGET, struct.pack("<f", 0.0 if args.ramp > 0 else v))
        n.ctl(CMD_ENABLE, b"\x01")
        print(f"{mode}: target {args.rpm} rpm = {v:.3f} rad/s, ramp {args.ramp:.1f} s, monitoring {args.seconds:.0f} s ...")
        t0 = time.monotonic()
        last_print = -1.0
        while time.monotonic() - t0 < args.seconds:
            t = time.monotonic() - t0
            if args.ramp > 0 and t <= args.ramp + 0.1:
                n.ctl(CMD_SET_TARGET, struct.pack("<f", v * min(1.0, t / args.ramp)))
            if t - last_print >= 0.5:
                last_print = t
                st, tm = n.get_state(), n.get_telemetry()
                print(f"  t={t:4.1f}s  {fmt_state(st)}  |  {fmt_telemetry(tm)}", flush=True)
                if tm["drv_fault"]:
                    d = n.ctl(CMD_DRV_FAULTS)
                    fs1, fs2 = struct.unpack_from("<HH", d, 2)
                    f1 = [nm for bb, nm in DRV_FS1_BITS.items() if fs1 & (1 << bb)]
                    f2 = [nm for bb, nm in DRV_FS2_BITS.items() if fs2 & (1 << bb)]
                    print(f"  DRV8323 FAULT: {' '.join(f1 + f2) or 'nFAULT only'} - disabling")
                    n.ctl(CMD_SET_TARGET, struct.pack("<f", 0.0))
                    n.ctl(CMD_ENABLE, b"\x00")
                    return 1
            time.sleep(0.05)
        if args.stop:
            n.ctl(CMD_SET_TARGET, struct.pack("<f", 0.0))
            n.ctl(CMD_ENABLE, b"\x00")
            print("stopped and disabled")
        else:
            print("left running (use `disable` to stop)")
    return 0


def main():
    def add_common(p, suppress):
        # Common options are accepted both before and after the subcommand.
        d = (lambda v: argparse.SUPPRESS) if suppress else (lambda v: v)
        p.add_argument("--port", default=d(None), help="slcan adapter serial port (default: auto-detect CANable)")
        p.add_argument("--bitrate", type=int, default=d(500000))
        p.add_argument("--node", type=lambda s: int(s, 0), default=d(1), help="target node id 1..127 (default 1)")
        p.add_argument("--wait", type=float, default=d(30.0), help="seconds to wait for a power-cycle when the node is silent")

    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    add_common(ap, suppress=False)
    sub = ap.add_subparsers(dest="command", required=True)
    _add_parser = sub.add_parser

    def add_parser(*a, **k):
        p = _add_parser(*a, **k)
        add_common(p, suppress=True)
        return p
    sub.add_parser = add_parser

    s = sub.add_parser("scan", help="broadcast PING and list every node that answers")
    s.add_argument("--timeout", type=float, default=1.0)
    s.set_defaults(func=cmd_scan)

    s = sub.add_parser("info", help="PING one node (and INFO if it is in the bootloader)")
    s.set_defaults(func=cmd_info)

    s = sub.add_parser("enter", help="put the node into the bootloader and leave it there")
    s.set_defaults(func=cmd_enter)

    s = sub.add_parser("run", help="reset the node so it boots the application")
    s.set_defaults(func=cmd_run)

    s = sub.add_parser("set-node-id", help="change the node id stored in the boot-config page")
    s.add_argument("new_id", type=lambda v: int(v, 0))
    s.add_argument("--stay", action="store_true", help="stay in the bootloader afterwards")
    s.set_defaults(func=cmd_set_node_id)

    s = sub.add_parser("flash", help="erase, write and verify an application .bin, then run it")
    s.add_argument("image", help=".bin linked at 0x08008000 (tools/build.py app)")
    s.add_argument("--chunk", type=int, default=256,
                   help="bytes per WRITE (multiple of 8, <= 2048). 256 is safe for a CANable; larger values are "
                        "tried and halved automatically if the adapter drops frames")
    s.add_argument("--no-verify", action="store_true")
    s.add_argument("--stay", action="store_true", help="do not start the application afterwards")
    s.add_argument("--force", action="store_true", help="flash even if the image sanity checks fail")
    s.set_defaults(func=cmd_flash)

    g = sub.add_parser("mode", help="set control mode")
    g.add_argument("mode", choices=sorted(MODES))
    g.set_defaults(func=cmd_ctl)
    g = sub.add_parser("target", help="set target (rad/s, rad or torque units; --rpm converts rpm to rad/s)")
    g.add_argument("value", type=float)
    g.add_argument("--rpm", action="store_true")
    g.set_defaults(func=cmd_ctl)
    sub.add_parser("enable", help="enable the motor").set_defaults(func=cmd_ctl)
    sub.add_parser("disable", help="disable the motor").set_defaults(func=cmd_ctl)
    g = sub.add_parser("pole-pairs", help="set motor pole pairs (runtime; save-config to persist)")
    g.add_argument("pp", type=int)
    g.set_defaults(func=cmd_ctl)
    g = sub.add_parser("vlimit", help="set motor phase voltage limit in volts")
    g.add_argument("volts", type=float)
    g.set_defaults(func=cmd_ctl)
    sub.add_parser("save-config", help="persist config (pole pairs etc.) to EEPROM").set_defaults(func=cmd_ctl)
    sub.add_parser("state", help="read mode, enable, velocity and angle").set_defaults(func=cmd_ctl)
    sub.add_parser("telemetry", help="read bus voltage, temperatures, driver fault").set_defaults(func=cmd_ctl)
    g = sub.add_parser("cog", help="anti-cogging map: calib | abort | status | enable | disable | save | dump")
    g.add_argument("action", choices=["calib", "abort", "status", "enable", "disable", "save", "dump"])
    g.add_argument("--pos-counts", type=int, default=1, help="calib: settle window in encoder counts (default 1)")
    g.add_argument("--vel", type=float, default=0.1, help="calib: settle velocity threshold in rad/s (default 0.1)")
    g.add_argument("--dwell-ms", type=int, default=30, help="calib: settled time before a point is recorded (default 30)")
    g.add_argument("--timeout-ms", type=int, default=400, help="calib: per-point timeout, records anyway (default 400)")
    g.add_argument("--watch", action="store_true", help="calib: poll progress until finished")
    g.add_argument("--out", help="dump: write the map to this CSV file")
    g.set_defaults(func=cmd_ctl)
    g = sub.add_parser("ilimit", help="set motor current limit in amps")
    g.add_argument("amps", type=float)
    g.set_defaults(func=cmd_ctl)
    g = sub.add_parser("recalibrate", help="run SimpleFOC sensor alignment for the current pole-pair count")
    g.add_argument("--volts", type=float, help="alignment voltage (default: firmware setting, 1 V)")
    g.set_defaults(func=cmd_ctl)
    g = sub.add_parser("torque-mode", help="set torque controller: voltage, dc_current or foc_current")
    g.add_argument("tmode", choices=sorted(TORQUE_MODES))
    g.set_defaults(func=cmd_ctl)
    g = sub.add_parser("currents", help="read phase currents and FOC dq values")
    g.add_argument("--count", type=int, default=1)
    g.set_defaults(func=cmd_ctl)
    g = sub.add_parser("stream", help="print position/velocity on the ACB's USB serial port at N Hz (0 = off)")
    g.add_argument("hz", type=int)
    g.set_defaults(func=cmd_ctl)
    g = sub.add_parser("drv-spi", help="set SPI mode/clock used for DRV8323 register access (bring-up)")
    g.add_argument("mode", type=int, choices=[0, 1, 2, 3])
    g.add_argument("--khz", type=int, default=1000)
    g.set_defaults(func=cmd_ctl)
    sub.add_parser("drv-faults", help="read DRV8323 fault status registers").set_defaults(func=cmd_ctl)
    sub.add_parser("drv-clear", help="clear latched DRV8323 faults").set_defaults(func=cmd_ctl)
    g = sub.add_parser("drv-reg", help="read (or write with --value) a DRV8323 register 0x00..0x06")
    g.add_argument("addr", type=lambda v: int(v, 0))
    g.add_argument("--value", type=lambda v: int(v, 0))
    g.set_defaults(func=cmd_ctl)
    g = sub.add_parser("spin", help="spin at an rpm target and monitor")
    g.add_argument("--rpm", type=float, required=True)
    g.add_argument("--openloop", action="store_true", help="velocity_openloop instead of closed-loop velocity")
    g.add_argument("--pole-pairs", type=int)
    g.add_argument("--vlimit", type=float, help="phase voltage limit to apply first")
    g.add_argument("--seconds", type=float, default=5.0, help="how long to monitor")
    g.add_argument("--ramp", type=float, default=3.0, help="seconds to ramp the target from 0 (0 = step)")
    g.add_argument("--stop", action="store_true", help="stop and disable afterwards")
    g.set_defaults(func=cmd_ctl)

    args = ap.parse_args()
    if args.command == "set-node-id" and not 1 <= args.new_id <= 127:
        ap.error("new_id must be 1..127")
    if not 1 <= args.node <= 127:
        ap.error("--node must be 1..127")

    try:
        bus = open_bus(args)
    except BootloaderError as e:
        print(f"error: {e}", file=sys.stderr)
        return 2
    try:
        return args.func(bus, args)
    except BootloaderError as e:
        sys.stdout.flush()
        print(f"\nerror: {e}", file=sys.stderr)
        return 1
    finally:
        bus.shutdown()


if __name__ == "__main__":
    sys.exit(main())
