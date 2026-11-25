#!/usr/bin/env python3
"""Read-only OpenOCD -> MuJoCo real-time robotic arm calibration visualizer.

Connects to STM32 via OpenOCD TCL RPC (read-only `read_memory`, never halts or
writes MCU RAM), reads live `Now_Joint_Radian` and `Now_TCP_Pose`, and drives
MuJoCo kinematically (`mj_forward`) for intuitive axis sign / zero-offset checks.
"""

import argparse
import collections
import math
import os
import re
import shutil
import socket
import struct
import subprocess
import sys
import threading
import time
import xml.etree.ElementTree as ET
from pathlib import Path

# Support launching directly via `D:\conda_env\robotics\python.exe` without `conda activate`
if sys.platform == "win32":
    for _dll_dir in (
        Path(sys.prefix) / "Library" / "bin",
        Path(sys.prefix) / "DLLs",
        Path(sys.prefix),
    ):
        if _dll_dir.is_dir():
            os.add_dll_directory(str(_dll_dir))
            os.environ["PATH"] = str(_dll_dir) + os.pathsep + os.environ.get("PATH", "")

import mujoco
import mujoco.viewer
import numpy as np


DEFAULT_URDF = "urdf/urdf/M71_predev_with_mass.urdf"
DEFAULT_ELF = "MDK-ARM/output/ZLLC_2027.axf"
DEFAULT_JOINTS_SYM = "chariot.Test_Arm.Now_Joint_Radian"
DEFAULT_TCP_SYM = "chariot.Test_Arm.Now_TCP_Pose"

DEFAULT_GDB_FALLBACK = r"D:\STM32CubeCLT_1.22.0\GNU-tools-for-STM32\bin\arm-none-eabi-gdb.exe"
DEFAULT_OPENOCD_FALLBACK = r"C:\msys64\mingw64\bin\openocd.exe"


def rpy_to_mat(roll: float, pitch: float, yaw: float) -> np.ndarray:
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    return np.array([
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp,     cp * sr,                cp * cr],
    ], dtype=np.float64)


def rodrigues(axis: np.ndarray, angle: float) -> np.ndarray:
    x, y, z = float(axis[0]), float(axis[1]), float(axis[2])
    c, s = math.cos(angle), math.sin(angle)
    t = 1.0 - c
    return np.array([
        [t * x * x + c,     t * x * y - s * z, t * x * z + s * y],
        [t * x * y + s * z, t * y * y + c,     t * y * z - s * x],
        [t * x * z - s * y, t * y * z + s * x, t * z * z + c],
    ], dtype=np.float64)


def parse_urdf_kinematics(urdf_path: Path):
    """Extract ordered chain, joint limits, and tip fixed-frame transform for TCP lookup."""
    root = ET.parse(urdf_path).getroot()
    links = {el.attrib["name"] for el in root.findall("link")}
    children_of = {k: [] for k in links}
    has_parent = set()

    for j in root.findall("joint"):
        orig = j.find("origin")
        xyz = np.array([float(x) for x in orig.attrib.get("xyz", "0 0 0").split()], dtype=np.float64) if orig is not None else np.zeros(3)
        rpy = [float(x) for x in orig.attrib.get("rpy", "0 0 0").split()] if orig is not None else [0.0, 0.0, 0.0]
        ax_el = j.find("axis")
        axis = np.array([float(x) for x in ax_el.attrib.get("xyz", "0 0 1").split()], dtype=np.float64) if ax_el is not None else np.array([0.0, 0.0, 1.0])
        lim = j.find("limit")
        qmin = float(lim.attrib.get("lower", "-3.1416")) if lim is not None else -3.1416
        qmax = float(lim.attrib.get("upper", "3.1416")) if lim is not None else 3.1416

        jdata = {
            "name": j.attrib["name"],
            "type": j.attrib["type"],
            "parent": j.find("parent").attrib["link"],
            "child": j.find("child").attrib["link"],
            "xyz": xyz,
            "Rf": rpy_to_mat(*rpy),
            "axis": axis,
            "qmin": qmin,
            "qmax": qmax,
        }
        children_of[jdata["parent"]].append(jdata)
        has_parent.add(jdata["child"])

    roots = [l for l in links if l not in has_parent]
    if len(roots) != 1:
        raise RuntimeError(f"Expected single root link in {urdf_path}, got {roots}")

    chain = []
    cur = roots[0]
    while children_of[cur]:
        if len(children_of[cur]) > 1:
            raise RuntimeError(f"Branching URDF at {cur} is not supported")
        jdata = children_of[cur][0]
        chain.append(jdata)
        cur = jdata["child"]

    # Fold any trailing fixed joints after the last movable link into a single local offset
    movable_joints = [j for j in chain if j["type"] != "fixed"]
    last_movable_idx = max(i for i, j in enumerate(chain) if j["type"] != "fixed")
    anchor_body = chain[last_movable_idx]["child"]
    tcp_pos_local = np.zeros(3, dtype=np.float64)
    tcp_rot_local = np.eye(3, dtype=np.float64)
    for j in chain[last_movable_idx + 1:]:
        tcp_pos_local = tcp_pos_local + tcp_rot_local @ j["xyz"]
        tcp_rot_local = tcp_rot_local @ j["Rf"]

    return chain, movable_joints, anchor_body, tcp_pos_local, tcp_rot_local


def load_mujoco_from_urdf(urdf_path: Path):
    """Inject <mujoco> compiler settings in memory so STL meshes load directly."""
    urdf_abs = urdf_path.resolve()
    mesh_dir = (urdf_abs.parent.parent / "meshes").resolve()
    if not mesh_dir.is_dir():
        raise FileNotFoundError(f"Mesh directory not found: {mesh_dir}")

    root = ET.fromstring(urdf_abs.read_text(encoding="utf-8"))
    mj = ET.Element("mujoco")
    ET.SubElement(
        mj,
        "compiler",
        meshdir=str(mesh_dir).replace("\\", "/"),
        strippath="true",
        discardvisual="false",
        balanceinertia="true",
    )
    root.insert(0, mj)
    xml_str = ET.tostring(root, encoding="unicode")
    model = mujoco.MjModel.from_xml_string(xml_str)
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    return model, data


def find_tool(preferred: str, fallback: str) -> str:
    found = shutil.which(preferred)
    if found:
        return found
    if Path(fallback).is_file():
        return fallback
    raise FileNotFoundError(f"Could not find `{preferred}` on PATH or at `{fallback}`")


def resolve_symbol_addr(elf_path: Path, symbol: str, gdb_bin: str) -> int:
    if not elf_path.is_file():
        raise FileNotFoundError(f"ELF/AXF file not found: {elf_path}")

    cmd = [
        gdb_bin,
        "-batch",
        "-ex", f"file {elf_path.as_posix()}",
        "-ex", f"p/x &{symbol}",
    ]
    res = subprocess.run(cmd, capture_output=True, text=True, check=False)
    matches = re.findall(r"0x[0-9a-fA-F]+", res.stdout)
    if not matches:
        raise RuntimeError(
            f"Failed to resolve symbol `{symbol}` from {elf_path}.\n"
            f"GDB stdout: {res.stdout.strip()}\nGDB stderr: {res.stderr.strip()}"
        )
    return int(matches[-1], 16)


def parse_csv_floats(text: str, count: int, name: str, is_sign: bool = False) -> np.ndarray:
    parts = [p.strip() for p in text.split(",") if p.strip() != ""]
    if len(parts) != count:
        raise ValueError(f"`{name}` expects {count} comma-separated numbers, got {len(parts)} ({text!r})")
    vals = np.array([float(p) for p in parts], dtype=np.float64)
    if is_sign:
        for v in vals:
            if abs(abs(v) - 1.0) > 1e-6:
                raise ValueError(f"`--sign` entries must be +1 or -1, got {v}")
        vals = np.sign(vals)
    return vals


def parse_hex_words_to_floats(response: str, expected_words: int) -> np.ndarray:
    """Decode space/newline-separated 32-bit hex words from OpenOCD `read_memory` into float32."""
    tokens = re.findall(r"(?:0x)?([0-9a-fA-F]+)", response.strip())
    if len(tokens) != expected_words:
        raise ValueError(f"Expected {expected_words} hex words from OpenOCD, got {len(tokens)}: {response!r}")
    raw = b"".join(struct.pack("<I", int(tok, 16)) for tok in tokens)
    return np.frombuffer(raw, dtype="<f4").copy()


class OpenOCDTclClient:
    """Minimal read-only OpenOCD TCL RPC client (`\\x1a`-terminated frames)."""

    END_CHAR = b"\x1a"

    def __init__(self, host: str = "127.0.0.1", port: int = 6666, timeout: float = 1.0):
        self.host = host
        self.port = port
        self.timeout = timeout
        self.sock = None

    def connect(self):
        self.sock = socket.create_connection((self.host, self.port), timeout=self.timeout)
        self.sock.settimeout(self.timeout)
        self.sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)

    def close(self):
        if self.sock is not None:
            try:
                self.sock.close()
            finally:
                self.sock = None

    def reconnect(self):
        self.close()
        self.connect()

    def _command(self, cmd: str) -> str:
        if self.sock is None:
            raise RuntimeError("OpenOCD TCL socket is not connected")
        self.sock.sendall(cmd.encode("utf-8") + self.END_CHAR)
        chunks = []
        while True:
            data = self.sock.recv(4096)
            if not data:
                raise ConnectionError("OpenOCD TCL connection closed")
            end_idx = data.find(self.END_CHAR)
            if end_idx != -1:
                chunks.append(data[:end_idx])
                break
            chunks.append(data)
        return b"".join(chunks).decode("utf-8", errors="replace").strip()

    def read_floats(self, addr: int, count: int) -> np.ndarray:
        resp = self._command(f"read_memory 0x{addr:08x} 32 {count}")
        return parse_hex_words_to_floats(resp, count)


def _drain_stream(stream, log_buf: collections.deque, tag: str):
    try:
        for line in iter(stream.readline, ""):
            stripped = line.rstrip()
            if stripped:
                log_buf.append(f"[{tag}] {stripped}")
    except Exception:
        pass
    finally:
        try:
            stream.close()
        except Exception:
            pass


def launch_openocd(args):
    openocd_bin = find_tool(args.openocd_bin, DEFAULT_OPENOCD_FALLBACK)
    bin_path = Path(openocd_bin).resolve()
    env = os.environ.copy()
    env["PATH"] = str(bin_path.parent) + os.pathsep + env.get("PATH", "")

    cmd = [str(bin_path)]
    scripts_dir = bin_path.parent.parent / "share" / "openocd" / "scripts"
    if scripts_dir.is_dir():
        cmd.extend(["-s", str(scripts_dir)])

    if args.openocd_cfg:
        cmd.extend(["-f", args.openocd_cfg])
    else:
        cmd.extend(["-f", f"interface/{args.probe}.cfg"])
        if args.probe in ("cmsis-dap", "jlink"):
            cmd.extend(["-c", "transport select swd"])
        cmd.extend(["-f", f"target/{args.target}.cfg"])

    cmd.extend([
        "-c", f"adapter speed {args.speed}",
        "-c", "gdb_port disabled",
        "-c", "telnet_port disabled",
        "-c", f"tcl_port {args.port}",
        "-c", "init",
    ])

    proc = subprocess.Popen(
        cmd,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        text=True,
        bufsize=1,
        env=env,
    )

    # 后台线程持续排空 stdout/stderr，防止 Windows 4KB 管道写满阻塞 OpenOCD 事件循环
    log_buf = collections.deque(maxlen=40)
    threading.Thread(target=_drain_stream, args=(proc.stdout, log_buf, "ocd-out"), daemon=True).start()
    threading.Thread(target=_drain_stream, args=(proc.stderr, log_buf, "ocd-err"), daemon=True).start()

    deadline = time.monotonic() + 5.0
    while time.monotonic() < deadline:
        if proc.poll() is not None:
            time.sleep(0.05)
            logs = "\n".join(log_buf)
            raise RuntimeError(
                f"OpenOCD exited early (code={proc.returncode}).\n"
                f"Command: {' '.join(cmd)}\n"
                f"Output:\n{logs}\n"
                f"Hints: (1) Close Keil Debug session if open; (2) Verify --probe {args.probe}; "
                f"(3) Check SWD wiring & board power."
            )
        try:
            with socket.create_connection((args.host, args.port), timeout=0.2):
                return proc, log_buf
        except OSError:
            time.sleep(0.1)

    proc.terminate()
    logs = "\n".join(log_buf)
    raise RuntimeError(f"Timed out waiting for OpenOCD TCL port {args.host}:{args.port}\nOutput:\n{logs}")


def fk_reference(chain, q: np.ndarray) -> np.ndarray:
    R = np.eye(3, dtype=np.float64)
    P = np.zeros(3, dtype=np.float64)
    q_idx = 0
    for j in chain:
        P = P + R @ j["xyz"]
        R = R @ j["Rf"]
        if j["type"] != "fixed":
            R = R @ rodrigues(j["axis"], float(q[q_idx]))
            q_idx += 1
    return P


def run_self_test(args) -> int:
    urdf_path = Path(args.urdf)
    chain, movable, anchor_body, tcp_local, _ = parse_urdf_kinematics(urdf_path)
    dof = len(movable)
    sign = parse_csv_floats(args.sign, dof, "--sign", is_sign=True) if args.sign else np.ones(dof)
    offset = parse_csv_floats(args.offset, dof, "--offset") if args.offset else np.zeros(dof)
    if args.offset_deg:
        offset = offset + np.deg2rad(parse_csv_floats(args.offset_deg, dof, "--offset-deg"))

    model, data = load_mujoco_from_urdf(urdf_path)
    if model.nq != dof:
        raise AssertionError(f"MuJoCo nq={model.nq} != URDF dof={dof}")
    body_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, anchor_body)
    if body_id < 0:
        raise AssertionError(f"Anchor body `{anchor_body}` missing in MuJoCo model")

    # Verify MuJoCo FK + local TCP offset matches independent URDF FK across random configs
    rng = np.random.default_rng(42)
    max_err = 0.0
    for _ in range(50):
        q_raw = np.array([rng.uniform(j["qmin"], j["qmax"]) for j in movable], dtype=np.float64)
        q_vis = sign * q_raw + offset
        data.qpos[:dof] = q_vis
        mujoco.mj_forward(model, data)
        p_mj = data.xpos[body_id] + data.xmat[body_id].reshape(3, 3) @ tcp_local
        p_ref = fk_reference(chain, q_vis)
        err = float(np.linalg.norm(p_mj - p_ref))
        max_err = max(max_err, err)
    if max_err > 1e-5:
        raise AssertionError(f"MuJoCo TCP FK mismatch: max_err={max_err:.3e}")

    # Verify OpenOCD hex parser
    sample = np.array([0.125, -0.5, 1.25, -2.0, 0.75, -3.125], dtype="<f4")
    hex_str = " ".join(f"0x{w:08x}" for w in struct.unpack("<6I", sample.tobytes()))
    decoded = parse_hex_words_to_floats(hex_str, 6)
    if not np.allclose(decoded, sample, atol=0.0):
        raise AssertionError("OpenOCD hex decode mismatch")

    # Verify GDB symbol resolution when ELF exists
    elf_path = Path(args.elf)
    if elf_path.is_file():
        gdb_bin = find_tool(args.gdb_bin, DEFAULT_GDB_FALLBACK)
        j_addr = resolve_symbol_addr(elf_path, args.joints_symbol, gdb_bin)
        print(f"[SELF-TEST] Resolved {args.joints_symbol} -> 0x{j_addr:08x}")
        if not args.no_tcp:
            t_addr = resolve_symbol_addr(elf_path, args.tcp_symbol, gdb_bin)
            print(f"[SELF-TEST] Resolved {args.tcp_symbol} -> 0x{t_addr:08x}")
    else:
        print(f"[SELF-TEST] Skipping GDB check (ELF not found at {elf_path})")

    print(f"[SELF-TEST] PASS dof={dof}, sign={sign.tolist()}, offset_deg={np.rad2deg(offset).round(3).tolist()}, max_tcp_err={max_err:.3e} m")
    return 0


def parse_args():
    p = argparse.ArgumentParser(description="Read-only OpenOCD -> MuJoCo robotic arm calibration visualizer")
    p.add_argument("--urdf", default=DEFAULT_URDF, help=f"URDF path (default: {DEFAULT_URDF})")
    p.add_argument("--elf", default=DEFAULT_ELF, help=f"Keil .axf / ELF path (default: {DEFAULT_ELF})")
    p.add_argument("--joints-symbol", default=DEFAULT_JOINTS_SYM, help="Joint radian float[dof] symbol")
    p.add_argument("--tcp-symbol", default=DEFAULT_TCP_SYM, help="TCP pose struct (first 3 floats = xyz) symbol")
    p.add_argument("--joints-addr", default=None, help="Manual hex address override for joints (e.g. 0x24005fe8)")
    p.add_argument("--tcp-addr", default=None, help="Manual hex address override for TCP pose")
    p.add_argument("--no-tcp", action="store_true", help="Disable MCU TCP comparison marker")

    p.add_argument("--probe", default="cmsis-dap", choices=["cmsis-dap", "stlink", "jlink"], help="SWD probe type (default: cmsis-dap)")
    p.add_argument("--target", default="stm32h7x", help="OpenOCD target cfg name (default: stm32h7x)")
    p.add_argument("--speed", type=int, default=2000, help="SWD adapter clock in kHz (default: 2000)")
    p.add_argument("--host", default="127.0.0.1", help="OpenOCD TCL host (default: 127.0.0.1)")
    p.add_argument("--port", type=int, default=6666, help="OpenOCD TCL port (default: 6666)")
    p.add_argument("--connect-only", action="store_true", help="Connect to an already-running OpenOCD TCL server")
    p.add_argument("--openocd-bin", default="openocd", help="OpenOCD executable path")
    p.add_argument("--openocd-cfg", default=None, help="Custom OpenOCD .cfg file (overrides --probe/--target)")
    p.add_argument("--gdb-bin", default="arm-none-eabi-gdb", help="GDB executable for ELF symbol lookup")

    p.add_argument("--sign", default=None, help="Comma-separated +/-1 calibration preview signs (e.g. -1,1,1,1,1,1)")
    p.add_argument("--offset", default=None, help="Comma-separated zero offsets in radians added after sign")
    p.add_argument("--offset-deg", default=None, help="Comma-separated zero offsets in degrees added after sign")
    p.add_argument("--rate", type=float, default=50.0, help="Sampling & render loop rate in Hz (default: 50)")
    p.add_argument("--mock", action="store_true", help="Run offline with simulated joint motion (no hardware needed)")
    p.add_argument("--self-test", action="store_true", help="Run headless verification and exit")
    return p.parse_args()


def main() -> int:
    args = parse_args()
    if args.self_test:
        return run_self_test(args)

    urdf_path = Path(args.urdf)
    chain, movable, anchor_body, tcp_local, _ = parse_urdf_kinematics(urdf_path)
    dof = len(movable)

    sign = parse_csv_floats(args.sign, dof, "--sign", is_sign=True) if args.sign else np.ones(dof)
    offset = parse_csv_floats(args.offset, dof, "--offset") if args.offset else np.zeros(dof)
    if args.offset_deg:
        offset = offset + np.deg2rad(parse_csv_floats(args.offset_deg, dof, "--offset-deg"))

    model, data = load_mujoco_from_urdf(urdf_path)
    body_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, anchor_body)

    openocd_proc = None
    openocd_logs = None
    client = None
    joints_addr = None
    tcp_addr = None
    coalesced_span = None

    if not args.mock:
        if args.joints_addr:
            joints_addr = int(args.joints_addr, 0)
        else:
            gdb_bin = find_tool(args.gdb_bin, DEFAULT_GDB_FALLBACK)
            joints_addr = resolve_symbol_addr(Path(args.elf), args.joints_symbol, gdb_bin)

        if not args.no_tcp:
            if args.tcp_addr:
                tcp_addr = int(args.tcp_addr, 0)
            else:
                gdb_bin = find_tool(args.gdb_bin, DEFAULT_GDB_FALLBACK)
                tcp_addr = resolve_symbol_addr(Path(args.elf), args.tcp_symbol, gdb_bin)

            # 若关节数组与 TCP 位姿地址相邻（跨度 <= 512 字节且 4 字节对齐），合并为单次 read_memory 突发读取
            span_start = min(joints_addr, tcp_addr)
            span_end = max(joints_addr + dof * 4, tcp_addr + 3 * 4)
            span_bytes = span_end - span_start
            if ((joints_addr - span_start) % 4 == 0) and ((tcp_addr - span_start) % 4 == 0) and (span_bytes <= 512):
                coalesced_span = (
                    span_start,
                    span_bytes // 4,
                    (joints_addr - span_start) // 4,
                    (tcp_addr - span_start) // 4,
                )

        print(f"Resolved joints `{args.joints_symbol}` -> 0x{joints_addr:08x}")
        if tcp_addr is not None:
            print(f"Resolved TCP    `{args.tcp_symbol}` -> 0x{tcp_addr:08x}")
        if coalesced_span is not None:
            print(f"Coalesced read  0x{coalesced_span[0]:08x} ({coalesced_span[1]} words / request)")

        if not args.connect_only:
            print(f"Starting OpenOCD (probe={args.probe}, target={args.target}, swd={args.speed} kHz)...")
            openocd_proc, openocd_logs = launch_openocd(args)

        client = OpenOCDTclClient(host=args.host, port=args.port, timeout=1.0)
        client.connect()
        print(f"Connected to OpenOCD TCL on {args.host}:{args.port} (READ-ONLY mode).")

    if np.any(sign < 0.0) or np.any(np.abs(offset) > 1e-7):
        print(f"Calibration preview ACTIVE -> q_vis = sign * q_mcu + offset")
        print(f"  sign       = {sign.astype(int).tolist()}")
        print(f"  offset_deg = {np.rad2deg(offset).round(2).tolist()}")
    else:
        print("Calibration preview: sign=[+1]*6, offset=0 deg (1:1 raw MCU feed)")

    print("Markers -> GREEN sphere: MuJoCo URDF TCP | RED sphere: MCU Now_TCP_Pose.pos")
    print("Close the MuJoCo window or press Ctrl+C to exit.\n")

    period = 1.0 / max(args.rate, 1.0)
    t_start = time.monotonic()
    last_print = 0.0
    read_errors = 0
    q_mcu = np.zeros(dof, dtype=np.float64)
    tcp_mcu = None

    try:
        with mujoco.viewer.launch_passive(model, data) as viewer:
            while viewer.is_running():
                t0 = time.monotonic()
                if args.mock:
                    elapsed = t0 - t_start
                    for i, j in enumerate(movable):
                        mid = 0.5 * (j["qmin"] + j["qmax"])
                        amp = 0.35 * (j["qmax"] - j["qmin"])
                        q_mcu[i] = mid + amp * math.sin(0.6 * elapsed + 0.7 * i)
                    tcp_mcu = fk_reference(chain, q_mcu)
                else:
                    try:
                        if coalesced_span is not None:
                            span_addr, span_words, j_off, t_off = coalesced_span
                            buf = client.read_floats(span_addr, span_words)
                            q_raw = buf[j_off:j_off + dof].astype(np.float64)
                            t_raw = buf[t_off:t_off + 3].astype(np.float64)
                        else:
                            q_raw = client.read_floats(joints_addr, dof).astype(np.float64)
                            t_raw = client.read_floats(tcp_addr, 3).astype(np.float64) if tcp_addr is not None else None

                        if np.all(np.isfinite(q_raw)) and np.max(np.abs(q_raw)) < 50.0:
                            q_mcu = q_raw
                        if t_raw is not None and np.all(np.isfinite(t_raw)) and np.linalg.norm(t_raw) < 10.0:
                            tcp_mcu = t_raw
                    except Exception as exc:
                        read_errors += 1
                        if openocd_proc is not None and openocd_proc.poll() is not None:
                            recent = "\n".join(openocd_logs) if openocd_logs else ""
                            raise RuntimeError(f"OpenOCD terminated unexpectedly (code={openocd_proc.returncode}):\n{recent}")
                        if read_errors <= 3 or read_errors % 50 == 0:
                            print(f"\n[WARN #{read_errors}] OpenOCD read failed: {exc}")
                            if openocd_logs:
                                print("  last OpenOCD log:", openocd_logs[-1])
                        try:
                            client.reconnect()
                        except Exception:
                            pass
                        time.sleep(period)
                        continue

                q_vis = sign * q_mcu + offset
                with viewer.lock():
                    data.qpos[:dof] = q_vis
                    mujoco.mj_forward(model, data)
                    tcp_mj = data.xpos[body_id] + data.xmat[body_id].reshape(3, 3) @ tcp_local

                    ngeom = 0
                    if viewer.user_scn.maxgeom >= 1:
                        mujoco.mjv_initGeom(
                            viewer.user_scn.geoms[0],
                            type=mujoco.mjtGeom.mjGEOM_SPHERE,
                            size=np.array([0.010, 0.0, 0.0], dtype=np.float64),
                            pos=np.asarray(tcp_mj, dtype=np.float64),
                            mat=np.eye(3, dtype=np.float64).ravel(),
                            rgba=np.array([0.1, 0.9, 0.25, 0.9], dtype=np.float32),
                        )
                        ngeom = 1
                    if tcp_mcu is not None and viewer.user_scn.maxgeom >= 2:
                        mujoco.mjv_initGeom(
                            viewer.user_scn.geoms[1],
                            type=mujoco.mjtGeom.mjGEOM_SPHERE,
                            size=np.array([0.013, 0.0, 0.0], dtype=np.float64),
                            pos=np.asarray(tcp_mcu, dtype=np.float64),
                            mat=np.eye(3, dtype=np.float64).ravel(),
                            rgba=np.array([0.95, 0.2, 0.2, 0.55], dtype=np.float32),
                        )
                        ngeom = 2
                    viewer.user_scn.ngeom = ngeom
                viewer.sync()

                if t0 - last_print >= 0.2:
                    last_print = t0
                    deg_str = " ".join(f"J{i+1}:{math.degrees(v):+6.1f}°" for i, v in enumerate(q_vis))
                    if tcp_mcu is not None:
                        diff_mm = float(np.linalg.norm(tcp_mj - tcp_mcu)) * 1000.0
                        tcp_str = f"TCP_mj=({tcp_mj[0]:+.3f},{tcp_mj[1]:+.3f},{tcp_mj[2]:+.3f}) | ΔTCP={diff_mm:5.2f}mm"
                    else:
                        tcp_str = f"TCP_mj=({tcp_mj[0]:+.3f},{tcp_mj[1]:+.3f},{tcp_mj[2]:+.3f})"
                    print(f"\r{deg_str} | {tcp_str} | err={read_errors}", end="", flush=True)

                dt = time.monotonic() - t0
                if dt < period:
                    time.sleep(period - dt)
    except KeyboardInterrupt:
        print("\nInterrupted by user.")
    finally:
        print()
        if client is not None:
            client.close()
        if openocd_proc is not None and openocd_proc.poll() is None:
            openocd_proc.terminate()
            try:
                openocd_proc.wait(timeout=2.0)
            except subprocess.TimeoutExpired:
                openocd_proc.kill()
            print("OpenOCD terminated cleanly.")
    return 0


if __name__ == "__main__":
    try:
        sys.exit(main())
    except Exception as exc:
        print(f"Error: {exc}", file=sys.stderr)
        sys.exit(1)
