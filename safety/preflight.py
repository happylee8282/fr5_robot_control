"""Check the RV-4FL, nozzle, USD graph, glasses snapshot, and guard contract."""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
import re
import sys
import time
import xml.etree.ElementTree as ET

import yaml

from ..core.runtime_config import DEFAULT_CONFIG, load_config, output_path, resolve_path
from .non_contact_gate import validate_non_contact_config


EXPECTED_JOINTS = ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"]


def check(condition: bool, message: str) -> None:
    if not condition:
        raise RuntimeError(message)


def raw_config(path: Path) -> tuple[dict, Path]:
    config = yaml.safe_load(path.read_text(encoding="utf-8"))
    root = (path.parent / str(config["project"]["root"])).resolve()
    return config, root


def inspect_urdf(path: Path) -> dict:
    robot = ET.parse(path).getroot()
    joints = {item.attrib["name"]: item for item in robot.findall("joint")}
    links = {item.attrib["name"] for item in robot.findall("link")}
    check(all(name in joints for name in EXPECTED_JOINTS), "URDF is missing RV-4FL joints")
    check("nozzle_tcp" in links and "nozzle_link" in links, "URDF is missing nozzle links")
    tcp = joints.get("nozzle_tcp_joint")
    check(tcp is not None and tcp.attrib.get("type") == "fixed", "nozzle TCP joint is not fixed")
    origin = tcp.find("origin")
    check(origin is not None and origin.attrib.get("xyz") == "0 0 0.065", "nozzle TCP is not 65 mm")
    limits = {}
    for name in EXPECTED_JOINTS:
        limit = joints[name].find("limit")
        check(limit is not None, f"URDF joint limit missing: {name}")
        limits[name] = {
            "lower_rad": float(limit.attrib["lower"]),
            "upper_rad": float(limit.attrib["upper"]),
        }
    return {"robot": robot.attrib.get("name"), "joints": EXPECTED_JOINTS, "limits": limits}


def inspect_stage(path: Path) -> dict:
    text = path.read_text(encoding="utf-8")
    check('def Xform "rv4fl"' in text, "USD overlay has no RV-4FL prim")
    check('over "AI_Glass_Front"' in text, "USD overlay has no glasses override")
    check("PhysicsRigidBodyAPI" in text, "glasses have no RigidBodyAPI")
    check("bool physics:rigidBodyEnabled = 1" in text, "glasses rigid body is disabled")
    check('string inputs:topicName = "/isaac_joint_states"' in text, "joint-state topic mismatch")
    check('string inputs:topicName = "/isaac_joint_commands"' in text, "joint-command topic mismatch")
    check("</World/rv4fl/root_joint>" in text, "Action Graph does not target RV-4FL")
    robot_match = re.search(
        r'def Xform "rv4fl".*?double3 xformOp:translate = \(([^)]+)\)', text, re.S
    )
    check(robot_match is not None, "cannot read RV-4FL USD translation")
    translation = [float(item.strip()) for item in robot_match.group(1).split(",")]
    targets = {
        name: float(value)
        for name, value in re.findall(
            r'over "(joint[1-6])"\s*\{.*?drive:angular:physics:targetPosition = ([^\s]+)',
            text,
            re.S,
        )
    }
    states = {
        name: float(value)
        for name, value in re.findall(
            r'over "(joint[1-6])"\s*\{.*?state:angular:physics:position = ([^\s]+)',
            text,
            re.S,
        )
    }
    mismatches = {
        name: targets[name] - states[name]
        for name in EXPECTED_JOINTS
        if name in targets and name in states and not math.isclose(targets[name], states[name])
    }
    return {
        "robot_world_translation_m": translation,
        "drive_target_deg": targets,
        "saved_state_deg": states,
        "drive_target_minus_saved_state_deg": mismatches,
        "runner_action": "state and targets are reset to INITIAL_POSE before first Play",
    }


def runtime_status(config: dict, root: Path, config_path: Path) -> dict:
    try:
        loaded, _ = load_config(config_path)
        snapshot = loaded.get("runtime_snapshot", {})
    except RuntimeError as error:
        return {"ready": False, "reason": str(error)}
    status_path = resolve_path(root, config["glass_guard"]["status_file"])
    try:
        status = json.loads(status_path.read_text(encoding="utf-8"))
    except (FileNotFoundError, json.JSONDecodeError) as error:
        return {"ready": False, "snapshot": snapshot, "reason": str(error)}
    age = time.time() - float(status.get("updated_wall_time", 0.0))
    ready = (
        age <= float(config["glass_guard"]["status_timeout_s"])
        and bool(status.get("settled"))
        and not bool(status.get("kinematic", True))
        and bool(status.get("physics_sleeping", False))
        and not bool(status.get("violation"))
    )
    return {
        "ready": ready,
        "snapshot": snapshot,
        "monitor_state": status.get("state"),
        "monitor_age_s": age,
        "physics_sleeping": bool(status.get("physics_sleeping", False)),
        "violation": bool(status.get("violation")),
    }


def run(config_path: Path, require_runtime: bool) -> dict:
    config_path = config_path.expanduser().resolve()
    config, root = raw_config(config_path)
    check(config["project"]["joint_names"] == EXPECTED_JOINTS, "config joint order mismatch")
    stage_path = resolve_path(root, config["resources"]["stage"])
    urdf_path = resolve_path(root, config["resources"]["robot_urdf"])
    csv_path = resolve_path(root, config["input"]["step2_csv"])
    pcd_path = resolve_path(root, config["input"]["step2_pcd"])
    glass_stl = resolve_path(root, config["resources"]["glass_stl"])
    for path in (stage_path, urdf_path, csv_path, pcd_path, glass_stl):
        check(path.is_file(), f"required file is missing: {path}")
    urdf = inspect_urdf(urdf_path)
    stage = inspect_stage(stage_path)
    initial_pose_rad = [float(value) for value in config["execution"]["initial_pose_rad"]]
    check(len(initial_pose_rad) == len(EXPECTED_JOINTS), "INITIAL_POSE must contain six joints")
    initial_pose_deg = [math.degrees(value) for value in initial_pose_rad]
    for name, expected in zip(EXPECTED_JOINTS, initial_pose_deg):
        check(name in stage["drive_target_deg"], f"USD drive target is missing: {name}")
        check(name in stage["saved_state_deg"], f"USD saved state is missing: {name}")
        check(
            math.isclose(stage["drive_target_deg"][name], expected, abs_tol=1.0e-5),
            f"USD drive target does not match INITIAL_POSE: {name}",
        )
        check(
            math.isclose(stage["saved_state_deg"][name], expected, abs_tol=1.0e-5),
            f"USD saved state does not match INITIAL_POSE: {name}",
        )
    configured_base = [float(value) for value in config["frame_transform"]["robot_base_world_xyz_m"]]
    check(
        all(abs(a - b) <= 1.0e-9 for a, b in zip(configured_base, stage["robot_world_translation_m"])),
        "MoveIt world->base transform does not match the saved USD robot pose",
    )
    non_contact = validate_non_contact_config(config)
    runtime = runtime_status(config, root, config_path)
    if require_runtime:
        check(bool(runtime["ready"]), f"Isaac runtime is not ready: {runtime.get('reason', runtime)}")
    report = {
        "status": "READY" if runtime["ready"] else "STRUCTURE_PASS_RUNTIME_PENDING",
        "config": str(config_path),
        "inputs": {
            "step2_csv": str(csv_path),
            "step2_pcd": str(pcd_path),
            "glass_stl": str(glass_stl),
        },
        "urdf": urdf,
        "usd": stage,
        "non_contact_safety": non_contact,
        "runtime": runtime,
    }
    report_path = output_path(config, root, "cartesian_metrics").with_name("preflight_report.json")
    report_path.parent.mkdir(parents=True, exist_ok=True)
    report_path.write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
    print(json.dumps(report, indent=2))
    return report


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", type=Path, default=DEFAULT_CONFIG)
    parser.add_argument("--require-runtime", action="store_true")
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    try:
        run(args.config, args.require_runtime)
    except RuntimeError as error:
        print(f"PREFLIGHT FAILED: {error}", file=sys.stderr)
        raise SystemExit(2) from error


if __name__ == "__main__":
    main()
