"""Pure fail-closed helpers for the isolated QR SIM graph."""
from __future__ import annotations

import math
from typing import Any


def finite(values: list[float]) -> bool:
    return all(math.isfinite(value) for value in values)


def image_to_bgr(message: Any, numpy_module: Any) -> Any:
    """Strictly convert a tightly packed rgb8/bgr8 ROS Image to BGR."""
    if message.encoding not in ("rgb8", "bgr8") or message.is_bigendian or message.step != message.width * 3:
        raise ValueError("unsupported image layout")
    if message.width <= 0 or message.height <= 0 or len(message.data) != message.height * message.step:
        raise ValueError("invalid image dimensions or data length")
    image = numpy_module.frombuffer(message.data, dtype=numpy_module.uint8).reshape(message.height, message.width, 3)
    return image[:, :, ::-1].copy() if message.encoding == "rgb8" else image.copy()


def valid_camera_info(info: Any, image: Any, max_skew_s: float) -> bool:
    if info is None or len(info.K) != 9 or len(info.D) not in (4, 5, 8, 12, 14):
        return False
    if not finite(list(info.K) + list(info.D)) or info.K[0] <= 0 or info.K[4] <= 0:
        return False
    if info.header.frame_id != image.header.frame_id or info.width != image.width or info.height != image.height:
        return False
    return abs((info.header.stamp - image.header.stamp).to_sec()) <= max_skew_s


def update_stability(previous_payload: str | None, frames: int, payload: str | None) -> tuple[str | None, int]:
    """Only an immediately consecutive identical successful payload increments stability."""
    if payload is None:
        return None, 0
    return (payload, frames + 1) if previous_payload == payload else (payload, 1)


def normalized_quaternion(quaternion: tuple[float, float, float, float], tolerance: float = 1e-3) -> bool:
    if not finite(list(quaternion)):
        return False
    norm = math.sqrt(sum(value * value for value in quaternion))
    return norm > 1e-9 and abs(norm - 1.0) <= tolerance


def quaternion_multiply(left: tuple[float, float, float, float], right: tuple[float, float, float, float]) -> tuple[float, float, float, float]:
    lx, ly, lz, lw = left
    rx, ry, rz, rw = right
    return (lw * rx + lx * rw + ly * rz - lz * ry, lw * ry - lx * rz + ly * rw + lz * rx,
            lw * rz + lx * ry - ly * rx + lz * rw, lw * rw - lx * rx - ly * ry - lz * rz)


def rotation_matrix_quaternion(matrix: Any) -> tuple[float, float, float, float]:
    """Convert a finite 3x3 rotation matrix to normalized xyzw quaternion."""
    trace = float(matrix[0, 0] + matrix[1, 1] + matrix[2, 2])
    if trace > 0:
        scale = math.sqrt(trace + 1.0) * 2
        result = ((matrix[2, 1] - matrix[1, 2]) / scale, (matrix[0, 2] - matrix[2, 0]) / scale, (matrix[1, 0] - matrix[0, 1]) / scale, 0.25 * scale)
    elif matrix[0, 0] > matrix[1, 1] and matrix[0, 0] > matrix[2, 2]:
        scale = math.sqrt(1.0 + matrix[0, 0] - matrix[1, 1] - matrix[2, 2]) * 2
        result = (0.25 * scale, (matrix[0, 1] + matrix[1, 0]) / scale, (matrix[0, 2] + matrix[2, 0]) / scale, (matrix[2, 1] - matrix[1, 2]) / scale)
    elif matrix[1, 1] > matrix[2, 2]:
        scale = math.sqrt(1.0 + matrix[1, 1] - matrix[0, 0] - matrix[2, 2]) * 2
        result = ((matrix[0, 1] + matrix[1, 0]) / scale, 0.25 * scale, (matrix[1, 2] + matrix[2, 1]) / scale, (matrix[0, 2] - matrix[2, 0]) / scale)
    else:
        scale = math.sqrt(1.0 + matrix[2, 2] - matrix[0, 0] - matrix[1, 1]) * 2
        result = ((matrix[0, 2] + matrix[2, 0]) / scale, (matrix[1, 2] + matrix[2, 1]) / scale, 0.25 * scale, (matrix[1, 0] - matrix[0, 1]) / scale)
    norm = math.sqrt(sum(value * value for value in result))
    if not finite(list(result)) or norm <= 1e-9:
        raise ValueError("invalid rotation matrix")
    return tuple(value / norm for value in result)


def angle_error(actual: float, expected: float) -> float:
    return (actual - expected + math.pi) % (2.0 * math.pi) - math.pi


def quaternion_rpy(quaternion: tuple[float, float, float, float]) -> tuple[float, float, float]:
    if not normalized_quaternion(quaternion):
        raise ValueError("invalid quaternion")
    x, y, z, w = quaternion
    return (math.atan2(2*(w*x+y*z), 1-2*(x*x+y*y)), math.asin(max(-1.0, min(1.0, 2*(w*y-z*x)))), math.atan2(2*(w*z+x*y), 1-2*(y*y+z*z)))


def slot_match(position: tuple[float, float, float], rpy: tuple[float, float, float], slot: dict[str, Any], oracle: dict[str, float]) -> bool:
    expected = slot["xyz"]
    if not finite([*position, *rpy, *slot["rpy"]]):
        return False
    return (math.hypot(position[0] - expected[0], position[1] - expected[1]) <= oracle["xy_tolerance_m"] and abs(position[2] - expected[2]) <= oracle["z_tolerance_m"] and all(abs(angle_error(actual, target)) <= oracle["max_tilt_rad"] for actual, target in zip(rpy, slot["rpy"])))


def complete_unique(matches: dict[str, bool]) -> bool:
    return len(matches) == 4 and all(matches.values())


SIM_ASSUMPTION = "SIM_WORKING_ASSUMPTION"
CONFIG_GROUPS = ("frame_id", "qr_targets", "geometry", "block", "pick_poses", "trays", "slots", "detector", "oracle")

def load_sim_config(raw: dict[str, Any]) -> dict[str, Any]:
    """Unwrap only fully status-labelled SIM assumptions; reject partial schemas."""
    if raw.get("assumption_status") != SIM_ASSUMPTION:
        raise ValueError("root assumption status missing")
    result = {}
    for name in CONFIG_GROUPS:
        group = raw.get(name)
        if not isinstance(group, dict) or group.get("assumption_status") != SIM_ASSUMPTION or "value" not in group:
            raise ValueError("missing SIM assumption status for " + name)
        result[name] = group["value"]
    if not validate_scene_config(result):
        raise ValueError("invalid SIM QR scene config")
    return result


def rpy_quaternion(rpy: list[float]) -> tuple[float, float, float, float]:
    if len(rpy) != 3 or not finite([float(value) for value in rpy]):
        raise ValueError("invalid rpy")
    roll, pitch, yaw = (value / 2.0 for value in rpy)
    cr, sr, cp, sp, cy, sy = math.cos(roll), math.sin(roll), math.cos(pitch), math.sin(pitch), math.cos(yaw), math.sin(yaw)
    return (sr * cp * cy - cr * sp * sy, cr * sp * cy + sr * cp * sy, cr * cp * sy - sr * sp * cy, cr * cp * cy + sr * sp * sy)


def material_contract(model: str) -> tuple[str, str, str, str]:
    """Return the OGRE name plus script and texture resource directories."""
    suffix = model[-2:]
    return ("QrBlock%s/texture" % suffix, "materials/scripts", "materials/textures", "qr_%s.png" % suffix)


def render_scene_sdf(model: str, config: dict[str, Any], model_uri: str = "") -> str:
    """Render fixed SIM SDF from the sole geometry assumption source."""
    geometry, block = config["geometry"], config["block"]
    if model.startswith("qr_block_"):
        size, mass = block["size_m"], block["mass_kg"]
        material_name, script_dir, texture_dir, _ = material_contract(model)
        script_uri, texture_uri = model_uri + "/" + script_dir, model_uri + "/" + texture_dir
        return "<sdf version='1.6'><model name='%s'><static>false</static><link name='link'><inertial><mass>%s</mass><inertia><ixx>0.00002</ixx><iyy>0.00002</iyy><izz>0.00002</izz></inertia></inertial><collision name='collision'><geometry><box><size>%s %s %s</size></box></geometry></collision><visual name='body'><geometry><box><size>%s %s %s</size></box></geometry></visual><visual name='qr'><pose>0 0 %s 0 0 0</pose><geometry><box><size>%s %s %s</size></box></geometry><material><script><uri>%s</uri><uri>%s</uri><name>%s</name></script></material></visual></link></model></sdf>" % (model, mass, size, size, size, size, size, size, size / 2.0 + geometry["qr_visual_thickness_m"] / 2.0, block["qr_marker_size_m"], block["qr_marker_size_m"], geometry["qr_visual_thickness_m"], script_uri, texture_uri, material_name)
    size, height = geometry["tray_size_xy_m"], geometry["tray_height_m"]
    return "<sdf version='1.6'><model name='%s'><static>true</static><link name='link'><collision name='collision'><geometry><box><size>%s %s %s</size></box></geometry></collision><visual name='visual'><geometry><box><size>%s %s %s</size></box></geometry></visual></link></model></sdf>" % (model, size, size, height, size, size, height)


def _flatten_numeric(values: Any) -> list[float]:
    if isinstance(values, dict):
        return [value for item in values.values() for value in _flatten_numeric(item)]
    if isinstance(values, (list, tuple)):
        return [value for item in values for value in _flatten_numeric(item)]
    return [float(values)]


def validate_scene_config(config: dict[str, Any]) -> bool:
    try:
        targets, picks, slots = config["qr_targets"], config["pick_poses"], config["slots"]
        ids = [item["target_id"] for item in targets]
        if len(ids) != 4 or len(set(ids)) != 4 or set(ids) != set(picks) or set(ids) != {slot["target_id"] for slot in slots.values()}:
            return False
        geometry, block = config["geometry"], config["block"]
        numeric = _flatten_numeric([geometry, block, config["detector"], config["oracle"]])
        for pose in [*picks.values(), *config["trays"].values(), *slots.values()]:
            if not isinstance(pose.get("xyz"), list) or not isinstance(pose.get("rpy"), list) or len(pose["xyz"]) != 3 or len(pose["rpy"]) != 3:
                return False
            numeric.extend(_flatten_numeric(pose["xyz"] + pose["rpy"]))
        if not finite(numeric) or any(float(value) <= 0 for value in [geometry["tray_size_xy_m"], geometry["tray_height_m"], geometry["qr_visual_thickness_m"], block["size_m"], block["mass_kg"], block["qr_marker_size_m"]]):
            return False
        tray_top, expected_z = geometry["tray_height_m"] / 2.0, geometry["tray_height_m"] / 2.0 + block["size_m"] / 2.0
        if any(abs(pose["xyz"][2] - expected_z) > 1e-9 for pose in picks.values()) or any(abs(slot["xyz"][2] - expected_z) > 1e-9 for slot in slots.values()):
            return False
        half = geometry["tray_size_xy_m"] / 2.0 - block["size_m"] / 2.0
        pick_tray, place_tray = config["trays"]["pick"]["xyz"], config["trays"]["place"]["xyz"]
        picks_inside = all(abs(pose["xyz"][0] - pick_tray[0]) <= half and abs(pose["xyz"][1] - pick_tray[1]) <= half for pose in picks.values())
        slots_inside = all(abs(slot["xyz"][0] - place_tray[0]) <= half and abs(slot["xyz"][1] - place_tray[1]) <= half for slot in slots.values())
        return picks_inside and slots_inside
    except (KeyError, TypeError, ValueError):
        return False
