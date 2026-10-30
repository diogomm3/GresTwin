#!/usr/bin/env python3

import json
import math
import sys
import xml.etree.ElementTree as ET
from pathlib import Path


def fail(message):
    raise RuntimeError(message)


def vector(text, length, label):
    values = [float(value) for value in text.split()]
    if len(values) != length:
        fail(f"{label} must contain {length} values; got {text!r}")
    return values


def require_close(actual, expected, label, tolerance=1e-8):
    if not math.isclose(actual, expected, rel_tol=0.0, abs_tol=tolerance):
        fail(f"{label}: expected {expected}, got {actual}")


def collision_extents(model_path, ignored_collisions=None):
    root = ET.parse(model_path).getroot()
    bounds = [[math.inf, -math.inf] for _ in range(3)]
    for collision in root.findall(".//collision"):
        if collision.get("name") in (ignored_collisions or set()):
            continue
        box = collision.find("geometry/box/size")
        if box is None:
            continue
        sizes = vector(box.text, 3, "box size")
        pose = vector(
            collision.findtext("pose", default="0 0 0 0 0 0"),
            6,
            "collision pose",
        )
        if any(abs(angle) > 1e-10 for angle in pose[3:]):
            fail(f"Rotated box collision is not supported by extent check: {collision.get('name')}")
        for axis in range(3):
            bounds[axis][0] = min(bounds[axis][0], pose[axis] - sizes[axis] / 2)
            bounds[axis][1] = max(bounds[axis][1], pose[axis] + sizes[axis] / 2)
    if any(not math.isfinite(low) or not math.isfinite(high) for low, high in bounds):
        fail(f"No box collision geometry found in {model_path}")
    return [high - low for low, high in bounds]


def collision_bounds(model_root, model_pose, label):
    if any(abs(angle) > 1e-10 for angle in model_pose[3:]):
        fail(f"Rotated model is not supported by collision bounds check: {label}")
    bounds = []
    for collision in model_root.findall(".//collision"):
        size = collision.find("geometry/box/size")
        if size is None:
            fail(f"{label} collision is not a box: {collision.get('name')}")
        dimensions = vector(size.text, 3, f"{label} collision size")
        pose = vector(
            collision.findtext("pose", default="0 0 0 0 0 0"),
            6,
            f"{label} collision pose",
        )
        if any(abs(angle) > 1e-10 for angle in pose[3:]):
            fail(f"Rotated collision is not supported: {collision.get('name')}")
        center = [model_pose[axis] + pose[axis] for axis in range(3)]
        bounds.append(
            (
                collision.get("name"),
                [
                    (center[axis] - dimensions[axis] / 2,
                     center[axis] + dimensions[axis] / 2)
                    for axis in range(3)
                ],
            )
        )
    return bounds


def main():
    package_root = Path(__file__).resolve().parents[1]
    if not (package_root / "config" / "stage1_scene.json").is_file():
        from ament_index_python.packages import get_package_share_directory

        package_root = Path(get_package_share_directory("stage1_stoneware_sim"))
    config = json.loads(
        (package_root / "config" / "stage1_scene.json").read_text(encoding="utf-8")
    )
    world = ET.parse(
        package_root / "worlds" / "stage1_raw_stoneware.world.sdf"
    ).getroot()
    models = {
        include.findtext("name"): include
        for include in world.findall(".//include")
        if include.findtext("name")
    }
    if "stage1_stack_retainer" in models:
        fail("The removed stack-retainer model must not be included in the world")

    cylinder = ET.parse(
        package_root / "models" / "stage1_stoneware_cylinder" / "model.sdf"
    ).getroot()
    geometry = cylinder.find(".//collision/geometry/cylinder")
    if geometry is None:
        fail("Cylinder collision geometry is missing")
    require_close(float(geometry.findtext("length")), config["cylinder"]["length"], "cylinder length")
    require_close(
        float(geometry.findtext("radius")) * 2,
        config["cylinder"]["diameter"],
        "cylinder diameter",
    )
    collision_pose = vector(
        cylinder.findtext(".//collision/pose", default="0 0 0 0 0 0"),
        6,
        "cylinder collision pose",
    )
    require_close(collision_pose[4], math.pi / 2, "cylinder axis alignment")
    visual_geometry = cylinder.find(".//visual/geometry/cylinder")
    if visual_geometry is None:
        fail("Cylinder visual geometry is missing")
    require_close(
        float(visual_geometry.findtext("length")),
        float(geometry.findtext("length")),
        "visual/collision cylinder length",
    )
    require_close(
        float(visual_geometry.findtext("radius")),
        float(geometry.findtext("radius")),
        "visual/collision cylinder radius",
    )
    mass = float(cylinder.findtext(".//inertial/mass"))
    require_close(mass, config["cylinder"]["mass"], "cylinder mass")
    volume = math.pi * (config["cylinder"]["diameter"] / 2) ** 2 * config["cylinder"]["length"]
    require_close(
        mass,
        volume * config["cylinder"]["density_assumption"],
        "mass from configured density assumption",
        tolerance=1e-7,
    )
    expected_inertia = [
        mass * (config["cylinder"]["diameter"] / 2) ** 2 / 2,
        mass
        * (
            3 * (config["cylinder"]["diameter"] / 2) ** 2
            + config["cylinder"]["length"] ** 2
        )
        / 12,
    ]
    inertia = cylinder.find(".//inertial/inertia")
    require_close(float(inertia.findtext("ixx")), expected_inertia[0], "axial inertia", 1e-8)
    require_close(float(inertia.findtext("iyy")), expected_inertia[1], "transverse inertia", 1e-8)
    friction = float(cylinder.findtext(".//collision/surface/friction/bullet/friction"))
    require_close(friction, config["cylinder"]["friction"], "Bullet friction")
    rolling_friction = float(
        cylinder.findtext(".//collision/surface/friction/bullet/rolling_friction")
    )
    require_close(
        rolling_friction,
        config["cylinder"]["rolling_friction"],
        "rolling friction",
    )
    if cylinder.find(
        ".//sensor[@name='stoneware_contact_sensor']/contact/collision"
    ) is None:
        fail("Cylinder contact sensor is missing or not attached to its collision")
    restitution = float(
        cylinder.findtext(".//collision/surface/bounce/restitution_coefficient")
    )
    require_close(restitution, config["cylinder"]["restitution"], "restitution")

    pallet_model = ET.parse(
        package_root / "models" / "stage1_pallet" / "model.sdf"
    ).getroot()
    barrier_config = config["side_barriers"]
    barrier_names = {
        "side_barrier_negative_y_collision",
        "side_barrier_positive_y_collision",
    }
    pallet_dimensions = collision_extents(
        package_root / "models" / "stage1_pallet" / "model.sdf",
        ignored_collisions=barrier_names,
    )
    for axis, expected in enumerate(config["pallet"]["size"]):
        require_close(pallet_dimensions[axis], expected, f"pallet dimension axis {axis}")
    barriers = {
        collision.get("name"): collision
        for collision in pallet_model.findall(".//collision")
        if collision.get("name") in barrier_names
    }
    if barriers.keys() != barrier_names:
        fail(f"Expected two pallet side-barrier collisions, got {sorted(barriers)}")
    if pallet_model.findtext("./model/static") != "true":
        fail("Pallet and integrated side barriers must be static")
    expected_barrier_poses = [
        (0.0, offset, barrier_config["center_z"], 0.0, 0.0, 0.0)
        for offset in barrier_config["center_y_offsets"]
    ]
    for (name, collision), expected_pose in zip(
        sorted(barriers.items()), expected_barrier_poses
    ):
        size = vector(
            collision.findtext("geometry/box/size"),
            3,
            f"{name} size",
        )
        for axis, expected in enumerate(barrier_config["size"]):
            require_close(size[axis], expected, f"{name} dimension axis {axis}")
        pose = vector(collision.findtext("pose"), 6, f"{name} pose")
        for axis, expected in enumerate(expected_pose):
            require_close(pose[axis], expected, f"{name} pose axis {axis}")
        if any(abs(angle) > 1e-10 for angle in pose[3:]):
            fail(f"{name} must be axis-aligned")
    top_deck = pallet_model.find(".//collision[@name='top_deck_collision']")
    if top_deck is None:
        fail("Pallet top deck collision is missing")
    pallet_top = (
        float(top_deck.find("pose").text.split()[2])
        + float(top_deck.find("geometry/box/size").text.split()[2]) / 2
    )
    pallet_friction = float(
        top_deck.findtext("surface/friction/bullet/friction")
    )
    require_close(
        pallet_friction,
        config["pallet"]["top_surface_friction"],
        "pallet top-surface friction",
    )
    pallet_rolling_friction = float(
        top_deck.findtext("surface/friction/bullet/rolling_friction")
    )
    require_close(
        pallet_rolling_friction,
        config["pallet"]["top_surface_rolling_friction"],
        "pallet top-surface rolling friction",
    )
    pallet_restitution = float(
        top_deck.findtext("surface/bounce/restitution_coefficient")
    )
    require_close(
        pallet_restitution,
        config["pallet"]["top_surface_restitution"],
        "pallet top-surface restitution",
    )
    pallet_pose = vector(models["stage1_pallet"].findtext("pose"), 6, "pallet pose")
    for axis, expected in enumerate(config["pallet"]["pose"]):
        require_close(pallet_pose[axis], expected, f"pallet world pose axis {axis}")
    require_close(pallet_top + pallet_pose[2], config["pallet"]["size"][2], "pallet height")

    table_pose = vector(
        models["stage1_placement_table"].findtext("pose"), 6, "table pose"
    )
    for axis, expected in enumerate(config["table"]["pose"]):
        require_close(table_pose[axis], expected, f"table world pose axis {axis}")
    table_model = ET.parse(
        package_root / "models" / "stage1_placement_table" / "model.sdf"
    ).getroot()
    tabletop_pose = vector(
        table_model.find("./model/link/collision[@name='tabletop_collision']/pose").text,
        6,
        "tabletop pose",
    )
    tabletop_size = vector(
        table_model.find(
            "./model/link/collision[@name='tabletop_collision']/geometry/box/size"
        ).text,
        3,
        "tabletop size",
    )
    table_top = table_pose[2] + tabletop_pose[2] + tabletop_size[2] / 2
    table_dimensions = collision_extents(
        package_root / "models" / "stage1_placement_table" / "model.sdf"
    )
    for axis, expected in enumerate(config["table"]["size"]):
        require_close(table_dimensions[axis], expected, f"table dimension axis {axis}")
    for pallet_name, pallet_bounds in collision_bounds(
        pallet_model, pallet_pose, "pallet"
    ):
        for table_name, table_bounds in collision_bounds(
            table_model, table_pose, "table"
        ):
            if all(
                min(pallet_bounds[axis][1], table_bounds[axis][1])
                - max(pallet_bounds[axis][0], table_bounds[axis][0])
                > 1e-8
                for axis in range(3)
            ):
                fail(
                    f"Pallet collision {pallet_name} overlaps table collision "
                    f"{table_name}"
                )
    target_center_z = config["table"]["target_region"]["center"][2]
    target_model = world.find("./world/model[@name='placement_target_region']")
    if target_model is None:
        fail("Placement target region visual is missing")
    target_pose = vector(target_model.findtext("pose"), 6, "target region pose")
    for axis, expected in enumerate(config["table"]["target_region"]["center"]):
        require_close(target_pose[axis], expected, f"target region center axis {axis}")
    target_size = vector(
        target_model.findtext("./link/visual/geometry/box/size"),
        3,
        "target region size",
    )
    for axis, expected in enumerate(config["table"]["target_region"]["size"]):
        require_close(target_size[axis], expected, f"target region size axis {axis}")
    require_close(target_center_z - config["table"]["target_region"]["size"][2] / 2, table_top,
                  "target region lower surface")

    stack = []
    counts = []
    expected_counts = config["cylinder"]["level_counts_bottom_up"]
    for level in range(1, len(expected_counts) + 1):
        level_models = sorted(
            (
                (name, include)
                for name, include in models.items()
                if name.startswith(f"stoneware_l{level}_")
            ),
            key=lambda item: int(item[0].rsplit("_", 1)[1]),
        )
        counts.append(len(level_models))
        for name, include in level_models:
            pose = vector(include.findtext("pose"), 6, f"{name} pose")
            stack.append((level, name, pose))
    if counts != expected_counts:
        fail(f"Expected bottom-up level counts {expected_counts}, got {counts}")
    expected_total = sum(expected_counts)
    if len(stack) != expected_total:
        fail(f"Expected {expected_total} cylinders, got {len(stack)}")
    if sum(name.startswith("stoneware_l") for name in models) != expected_total:
        fail(f"World does not contain exactly {expected_total} named cylinder models")

    pallet_x, pallet_y, pallet_z = pallet_pose[:3]
    diameter = config["cylinder"]["diameter"]
    bottom_z = pallet_z + config["pallet"]["size"][2] + config["cylinder"]["diameter"] / 2
    barrier_height = barrier_config["size"][2]
    barrier_thickness = barrier_config["size"][1]
    barrier_bottom = pallet_z
    barrier_top = barrier_config["center_z"] + barrier_height / 2 + pallet_z
    pallet_half_length = config["pallet"]["size"][0] / 2
    pallet_half_width = config["pallet"]["size"][1] / 2
    for offset in barrier_config["center_y_offsets"]:
        require_close(
            abs(offset) + barrier_thickness / 2,
            pallet_half_width,
            "side barrier outer face / pallet edge",
        )
        require_close(
            barrier_config["center_z"] + barrier_height / 2,
            barrier_config["size"][2],
            "barrier height measured from pallet bottom",
        )
        require_close(
            barrier_config["center_z"] - barrier_height / 2,
            0.0,
            "barrier bottom at pallet bottom",
        )
        if barrier_config["size"][0] > config["pallet"]["size"][0]:
            fail("Side barrier extends beyond the pallet length")
    for level, name, pose in stack:
        level_count = expected_counts[level - 1]
        index = int(name.rsplit("_", 1)[1]) - 1
        expected_y = pallet_y - (level_count - 1) * diameter / 2 + index * diameter
        expected_z = bottom_z + (level - 1) * config["cylinder"]["vertical_pitch"]
        require_close(pose[0], pallet_x, f"{name} x center")
        require_close(pose[1], expected_y, f"{name} y center")
        require_close(
            pose[2],
            expected_z,
            f"{name} z center",
            tolerance=2e-8,
        )
        if abs(pose[1] - pallet_y) + diameter / 2 > pallet_half_width:
            fail(f"{name} extends beyond the pallet width")
        if abs(pose[0] - pallet_x) + config["cylinder"]["length"] / 2 > pallet_half_length:
            fail(f"{name} extends beyond the pallet length")
        if any(abs(angle) > 1e-10 for angle in pose[3:]):
            fail(f"{name} world pose is not parallel to the pallet length")
        cylinder_bottom = pose[2] - diameter / 2
        cylinder_top = pose[2] + diameter / 2
        if cylinder_bottom < pallet_top + pallet_z - 1e-8:
            fail(f"{name} penetrates the pallet top")
        if max(cylinder_bottom, barrier_bottom) < min(cylinder_top, barrier_top):
            for offset in barrier_config["center_y_offsets"]:
                inner_face = abs(offset) - barrier_thickness / 2
                cylinder_outer_edge = abs(pose[1] - pallet_y) + diameter / 2
                clearance = inner_face - cylinder_outer_edge
                if clearance < -1e-8:
                    fail(f"{name} penetrates a side barrier")

    bottom_level = [pose for level, _, pose in stack if level == 1]
    for offset in barrier_config["center_y_offsets"]:
        inner_face = abs(offset) - barrier_thickness / 2
        outer_edge = max(abs(pose[1] - pallet_y) + diameter / 2 for pose in bottom_level)
        actual_clearance = inner_face - outer_edge
        require_close(
            actual_clearance,
            barrier_config["clearance_to_outer_cylinders"],
            "outer-cylinder / side-barrier clearance",
        )

    for index, (_, name_a, pose_a) in enumerate(stack):
        for _, name_b, pose_b in stack[index + 1 :]:
            center_distance = math.hypot(pose_a[1] - pose_b[1], pose_a[2] - pose_b[2])
            if center_distance < diameter - 1e-8:
                fail(f"{name_a} and {name_b} overlap in the stack cross-section")

    for level in range(2, len(expected_counts) + 1):
        upper = [pose for current, _, pose in stack if current == level]
        lower = [pose for current, _, pose in stack if current == level - 1]
        for pose in upper:
            contacts = sum(
                math.isclose(
                    math.hypot(pose[1] - support[1], pose[2] - support[2]),
                    diameter,
                    rel_tol=0.0,
                    abs_tol=2e-8,
                )
                for support in lower
            )
            if contacts != 2:
                fail(f"Level {level} cylinder at y={pose[1]} has {contacts} supports, expected 2")

    if config["cylinder"]["level_counts_bottom_up"] != [8, 7, 6, 5, 4, 3, 2, 1]:
        fail("Pyramid counts no longer match the required 8/7/6/5/4/3/2/1 arrangement")
    if sum(counts) != 36:
        fail("Initial cylinder count is not 36")
    target = config["table"]["target_region"]
    target_x_limit = abs(target["center"][0]) + (
        target["size"][0] - config["cylinder"]["length"]
    ) / 2
    target_y_limit = abs(target["center"][1]) + (
        target["size"][1] - config["cylinder"]["diameter"]
    ) / 2
    target_distance = math.sqrt(
        target_x_limit**2
        + target_y_limit**2
        + (target["cylinder_center_z"] - config["robot"]["shoulder_height"]) ** 2
    )
    top_level_center_z = (
        bottom_z
        + (len(expected_counts) - 1) * config["cylinder"]["vertical_pitch"]
    )
    pickup_distance = math.dist(
        (pallet_x, pallet_y, top_level_center_z),
        (0.0, 0.0, config["robot"]["shoulder_height"]),
    )
    side_approach_distance = math.dist(
        (pallet_x, pallet_y - diameter / 2, top_level_center_z),
        (0.0, 0.0, config["robot"]["shoulder_height"]),
    )
    if target_distance > config["robot"]["nominal_reach"]:
        fail("The target-region envelope lies beyond nominal UR5e reach")

    print("PASS: pallet collision bounds are 1.0 x 0.85 x 0.25 m")
    print("PASS: two fixed 1.0 x 0.02 x 0.40 m side barriers fit the pallet")
    print("PASS: 36 cylinders, 0.90 m x 0.10 m, in 8/7/6/5/4/3/2/1 supported levels")
    print("PASS: cylinder collision, mass, inertia, friction and restitution are configured")
    print("PASS: stack has no overlaps or pallet/barrier penetrations; each upper cylinder has two supports")
    print(
        f"PASS: table collision bounds are 1.05 x 0.50 x 0.55 m; "
        f"target region is {config['table']['target_region']['size'][:2]} m"
    )
    print(
        f"PASS: pallet and table collision geometries do not overlap; "
        f"side-edge clearance is "
        f"{(table_pose[1] - config['table']['size'][1] / 2) - (pallet_pose[1] + config['pallet']['size'][1] / 2):.3f} m"
    )
    print(
        f"PASS: target envelope is within {config['robot']['nominal_reach']:.2f} m nominal reach"
    )
    if max(pickup_distance, side_approach_distance) > config["robot"]["nominal_reach"]:
        print(
            f"WARNING: top pickup estimate ({pickup_distance:.3f} m center, "
            f"{side_approach_distance:.3f} m side approach) exceeds nominal "
            f"UR5e reach of {config['robot']['nominal_reach']:.2f} m"
        )
    else:
        print(
            f"PASS: top pickup estimates ({pickup_distance:.3f} m center, "
            f"{side_approach_distance:.3f} m side approach) are within nominal "
            f"{config['robot']['nominal_reach']:.2f} m reach"
        )


if __name__ == "__main__":
    try:
        main()
    except (OSError, ValueError, KeyError, TypeError, ET.ParseError, RuntimeError) as error:
        print(f"FAIL: {error}", file=sys.stderr)
        sys.exit(1)
