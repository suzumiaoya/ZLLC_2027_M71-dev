#!/usr/bin/env python3
"""Generate a C++ URDF model header compatible with alg_urdf_kinematics.h."""

import argparse
import math
import re
import sys
import xml.etree.ElementTree as ET
from pathlib import Path


def parse_vec(text: str, expected_len: int, default=None):
    if text is None:
        if default is not None:
            return list(default)
        raise ValueError("Missing required attribute vector")
    vals = [float(x) for x in text.strip().split()]
    if len(vals) != expected_len:
        raise ValueError(f"Expected {expected_len} floats, got {len(vals)} from {text!r}")
    return vals


def rpy_to_mat(roll: float, pitch: float, yaw: float):
    """URDF standard ZYX intrinsic / XYZ fixed rotation matrix (row-major 3x3)."""
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    return [
        cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr,
        sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr,
        -sp,     cp * sr,                cp * cr,
    ]


def sanitize_identifier(name: str) -> str:
    cleaned = re.sub(r"[^0-9A-Za-z_]", "_", name.strip())
    cleaned = re.sub(r"_+", "_", cleaned).strip("_")
    if not cleaned:
        cleaned = "URDF_Robot"
    if cleaned[0].isdigit():
        cleaned = "Model_" + cleaned
    return cleaned


def fmt_f(val: float) -> str:
    if abs(val) == 0.0:
        return "0.0f"
    s = f"{val:.9g}"
    if "." not in s and "e" not in s and "E" not in s:
        s += ".0"
    return s + "f"


def parse_urdf_chain(urdf_path: Path, base_link_arg: str = None, tip_link_arg: str = None):
    tree = ET.parse(urdf_path)
    root = tree.getroot()
    if root.tag != "robot":
        raise ValueError(f"Root tag must be <robot>, got <{root.tag}>")

    robot_name = root.attrib.get("name", urdf_path.stem)

    links = {}
    for link_el in root.findall("link"):
        name = link_el.attrib["name"]
        iner = link_el.find("inertial")
        mass = 0.0
        com = [0.0, 0.0, 0.0]
        inertia = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        if iner is not None:
            mass_el = iner.find("mass")
            if mass_el is not None:
                mass = float(mass_el.attrib.get("value", "0"))
            orig_el = iner.find("origin")
            if orig_el is not None and "xyz" in orig_el.attrib:
                com = parse_vec(orig_el.attrib["xyz"], 3)
            in_el = iner.find("inertia")
            if in_el is not None:
                inertia = [
                    float(in_el.attrib.get("ixx", "0")),
                    float(in_el.attrib.get("ixy", "0")),
                    float(in_el.attrib.get("ixz", "0")),
                    float(in_el.attrib.get("iyy", "0")),
                    float(in_el.attrib.get("iyz", "0")),
                    float(in_el.attrib.get("izz", "0")),
                ]
        links[name] = {
            "name": name,
            "mass": mass,
            "com": com,
            "inertia": inertia,
        }

    children_of = {k: [] for k in links}
    parent_joint_of = {}
    for joint_el in root.findall("joint"):
        jname = joint_el.attrib["name"]
        jtype = joint_el.attrib["type"]
        parent = joint_el.find("parent").attrib["link"]
        child = joint_el.find("child").attrib["link"]
        if parent not in links or child not in links:
            raise ValueError(f"Joint {jname!r} references unknown link ({parent!r} -> {child!r})")
        if child in parent_joint_of:
            raise ValueError(f"Link {child!r} has multiple parent joints (not a tree)")

        orig_el = joint_el.find("origin")
        xyz = parse_vec(orig_el.attrib.get("xyz"), 3, [0.0, 0.0, 0.0]) if orig_el is not None else [0.0, 0.0, 0.0]
        rpy = parse_vec(orig_el.attrib.get("rpy"), 3, [0.0, 0.0, 0.0]) if orig_el is not None else [0.0, 0.0, 0.0]

        ax_el = joint_el.find("axis")
        axis = parse_vec(ax_el.attrib.get("xyz"), 3, [0.0, 0.0, 1.0]) if ax_el is not None else [0.0, 0.0, 1.0]
        if jtype == "fixed":
            axis = [0.0, 0.0, 0.0]
        else:
            norm = math.sqrt(sum(a * a for a in axis))
            if norm < 1e-12:
                raise ValueError(f"Joint {jname!r} ({jtype}) has zero axis")
            axis = [a / norm for a in axis]

        lim_el = joint_el.find("limit")
        q_min, q_max = 0.0, 0.0
        if jtype == "revolute":
            if lim_el is None:
                raise ValueError(f"Revolute joint {jname!r} is missing <limit>")
            q_min = float(lim_el.attrib.get("lower", "0"))
            q_max = float(lim_el.attrib.get("upper", "0"))
        elif jtype == "continuous" and lim_el is not None:
            q_min = float(lim_el.attrib.get("lower", "0"))
            q_max = float(lim_el.attrib.get("upper", "0"))

        if jtype not in ("revolute", "continuous", "fixed"):
            raise ValueError(f"Unsupported joint type {jtype!r} on {jname!r}")

        jdata = {
            "name": jname,
            "type": jtype,
            "parent": parent,
            "child": child,
            "xyz": xyz,
            "rpy": rpy,
            "R_fixed": rpy_to_mat(*rpy),
            "axis": axis,
            "q_min": q_min,
            "q_max": q_max,
            "child_link": links[child],
        }
        children_of[parent].append(jdata)
        parent_joint_of[child] = jdata

    if base_link_arg is not None:
        if base_link_arg not in links:
            raise ValueError(f"Base link {base_link_arg!r} not found in URDF")
        base_link = base_link_arg
    else:
        roots = [name for name in links if name not in parent_joint_of]
        if len(roots) != 1:
            raise ValueError(f"Expected 1 root link, found {roots}; pass --base-link")
        base_link = roots[0]

    chain = []
    if tip_link_arg is not None:
        if tip_link_arg not in links:
            raise ValueError(f"Tip link {tip_link_arg!r} not found in URDF")
        cur = tip_link_arg
        visited = set()
        while cur != base_link:
            if cur in visited or cur not in parent_joint_of:
                raise ValueError(f"No chain from {base_link!r} to {tip_link_arg!r}")
            visited.add(cur)
            jdata = parent_joint_of[cur]
            chain.append(jdata)
            cur = jdata["parent"]
        chain.reverse()
    else:
        cur = base_link
        while children_of[cur]:
            if len(children_of[cur]) > 1:
                raise ValueError(f"Branching detected at link {cur!r}; specify --tip-link")
            jdata = children_of[cur][0]
            chain.append(jdata)
            cur = jdata["child"]

    if not chain:
        raise ValueError("Extracted URDF chain has 0 joints")

    return robot_name, base_link, chain[-1]["child"], chain


def generate_header(urdf_path: Path, out_path: Path, prefix: str, base_link: str, tip_link: str, chain) -> str:
    guard = f"{prefix.upper()}_URDF_MODEL_H"
    define_macro = f"{prefix.upper()}_URDF_MODEL_DEFINE"
    joint_enum_map = {
        "revolute": "URDF_Joint_Type_REVOLUTE",
        "continuous": "URDF_Joint_Type_CONTINUOUS",
        "fixed": "URDF_Joint_Type_FIXED",
    }

    dof = 0
    for j in chain:
        if j["type"] == "fixed":
            j["q_index"] = -1
        else:
            j["q_index"] = dof
            dof += 1

    joint_entries = []
    for idx, j in enumerate(chain):
        cl = j["child_link"]
        Rf = j["R_fixed"]
        entry = f"""    // [{idx}] {j['name']} ({j['parent']} -> {j['child']}, type={j['type']}, q_idx={j['q_index']})
    {{
        {joint_enum_map[j['type']]},
        {j['q_index']},
        {{{fmt_f(j['xyz'][0])}, {fmt_f(j['xyz'][1])}, {fmt_f(j['xyz'][2])}}},
        {{
            {fmt_f(Rf[0])}, {fmt_f(Rf[1])}, {fmt_f(Rf[2])},
            {fmt_f(Rf[3])}, {fmt_f(Rf[4])}, {fmt_f(Rf[5])},
            {fmt_f(Rf[6])}, {fmt_f(Rf[7])}, {fmt_f(Rf[8])}
        }},
        {{{fmt_f(j['axis'][0])}, {fmt_f(j['axis'][1])}, {fmt_f(j['axis'][2])}}},
        {fmt_f(j['q_min'])},
        {fmt_f(j['q_max'])},
        {fmt_f(cl['mass'])},
        {{{fmt_f(cl['com'][0])}, {fmt_f(cl['com'][1])}, {fmt_f(cl['com'][2])}}},
        {{{fmt_f(cl['inertia'][0])}, {fmt_f(cl['inertia'][1])}, {fmt_f(cl['inertia'][2])}, {fmt_f(cl['inertia'][3])}, {fmt_f(cl['inertia'][4])}, {fmt_f(cl['inertia'][5])}}}
    }}"""
        joint_entries.append(entry)

    joints_body = ",\n".join(joint_entries)

    content = f"""/**
 * @file {out_path.name}
 * @brief Auto-generated URDF model parameters from {urdf_path.name}
 *        Chain: {base_link} -> {tip_link} (joint_num={len(chain)}, dof={dof})
 *
 * Usage:
 *   - Include this header anywhere to reference `extern const Struct_URDF_Model {prefix}_URDF_Model;`
 *   - Define `#define {define_macro}` before including this header in EXACTLY ONE `.cpp` file.
 */

#ifndef {guard}
#define {guard}

#include "alg_urdf_kinematics.h"

static constexpr uint8_t {prefix}_URDF_JOINT_NUM = {len(chain)}U;
static constexpr uint8_t {prefix}_URDF_DOF = {dof}U;

extern const Struct_URDF_Joint {prefix}_URDF_Joints[{prefix}_URDF_JOINT_NUM];
extern const Struct_URDF_Model {prefix}_URDF_Model;

#ifdef {define_macro}

const Struct_URDF_Joint {prefix}_URDF_Joints[{prefix}_URDF_JOINT_NUM] = {{
{joints_body}
}};

const Struct_URDF_Model {prefix}_URDF_Model = {{
    {prefix}_URDF_JOINT_NUM,
    {prefix}_URDF_DOF,
    {prefix}_URDF_Joints
}};

#endif // {define_macro}

#endif // {guard}
"""
    return content


def main():
    parser = argparse.ArgumentParser(description="Generate C++ URDF model header for alg_urdf_kinematics / alg_urdf_dynamics.")
    parser.add_argument("--urdf", required=True, help="Path to source .urdf file")
    parser.add_argument("--out", required=True, help="Path to output C++ .h file")
    parser.add_argument("--prefix", default=None, help="C++ symbol prefix (defaults to URDF <robot name>)")
    parser.add_argument("--base-link", default=None, help="Optional root link name override")
    parser.add_argument("--tip-link", default=None, help="Optional tip link name override")
    args = parser.parse_args()

    urdf_path = Path(args.urdf)
    out_path = Path(args.out)

    robot_name, base_link, tip_link, chain = parse_urdf_chain(urdf_path, args.base_link, args.tip_link)
    prefix = sanitize_identifier(args.prefix if args.prefix else robot_name)

    header_text = generate_header(urdf_path, out_path, prefix, base_link, tip_link, chain)
    out_path.parent.mkdir(parents=True, exist_ok=True)
    out_path.write_text(header_text, encoding="utf-8", newline="\n")
    print(f"Generated {out_path} (prefix={prefix}, chain={base_link}->{tip_link}, joints={len(chain)})")


if __name__ == "__main__":
    try:
        main()
    except Exception as exc:
        print(f"Error: {exc}", file=sys.stderr)
        sys.exit(1)
