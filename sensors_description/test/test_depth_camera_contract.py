#!/usr/bin/env python3

"""Render the shared RGB-D macro and verify its ROS optical-frame contract."""

import math
import pathlib
import shutil
import subprocess
import tempfile
import xml.etree.ElementTree as ET


MACRO_PATH = (
    pathlib.Path(__file__).resolve().parents[1]
    / "urdfs"
    / "sensors"
    / "depth_camera.gazebo.xacro"
)

XACRO_NS = "http://ros.org/wiki/xacro"


def _load_macro_body() -> ET.Element:
    """Parse the *production* xacro file directly (no xacro binary needed).

    This is a hermetic, deterministic proof that the shipped
    depth_camera.gazebo.xacro carries the truthful REP-103 optical topology
    and gz_frame_id binding, independent of whether a `xacro` renderer is
    installed in the environment.
    """
    tree = ET.parse(MACRO_PATH)
    macro = tree.getroot().find(f"{{{XACRO_NS}}}macro")
    assert macro is not None, "depth_camera macro not found in production xacro"
    return macro


def render_camera(namespace: str) -> ET.Element:
    wrapper = f"""<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="camera_contract">
  <xacro:include filename="{MACRO_PATH}"/>
  <link name="base_link"/>
  <xacro:depth_camera
      name="camera"
      namespace="{namespace}"
      update_rate="30"
      image_width="640"
      image_height="480"
      fov="1.047"
      near="0.1"
      far="10.0"
      parent_link="base_link"
      origin_xyz="0 0 0"
      origin_rpy="0 0 0"/>
</robot>
"""
    xacro_bin = (
        shutil.which("xacro")
        or (pathlib.Path("/opt/ros/jazzy/bin/xacro") if pathlib.Path("/opt/ros/jazzy/bin/xacro").exists() else None)
        or (pathlib.Path("/opt/ros/humble/bin/xacro") if pathlib.Path("/opt/ros/humble/bin/xacro").exists() else None)
        or "xacro"
    )
    import os
    env = dict(os.environ)
    env.pop("PYTHONNOUSERSITE", None)
    ros_paths = []
    for ros_dir in ["/opt/ros/jazzy", "/opt/ros/humble"]:
        if pathlib.Path(ros_dir).exists():
            bin_dir = f"{ros_dir}/bin"
            if bin_dir not in env.get("PATH", ""):
                env["PATH"] = f"{bin_dir}:{env.get('PATH', '')}"
            for sub in ["lib/python3.10/site-packages", "local/lib/python3.10/dist-packages", "lib/python3/dist-packages"]:
                p = f"{ros_dir}/{sub}"
                if pathlib.Path(p).exists():
                    ros_paths.append(p)
    if ros_paths:
        env["PYTHONPATH"] = ":".join(ros_paths) + ":" + env.get("PYTHONPATH", "")

    with tempfile.NamedTemporaryFile(mode="w", suffix=".urdf.xacro") as source:
        source.write(wrapper)
        source.flush()
        try:
            rendered = subprocess.run(
                [str(xacro_bin), source.name],
                check=True,
                capture_output=True,
                text=True,
                env=env,
            ).stdout
        except subprocess.CalledProcessError as e:
            raise RuntimeError(f"xacro failed (code {e.returncode}):\nSTDOUT: {e.stdout}\nSTDERR: {e.stderr}") from e
    return ET.fromstring(rendered)


def test_production_xacro_declares_rep103_optical_topology():
    """F1: the shipped xacro must define the static optical TF topology.

    Parses the real production file (not a self-declared expectation) and
    asserts the ${name}_optical_frame child, its fixed joint under
    ${name}_link, and the REP-103 rotation (rpy = -pi/2 0 -pi/2).
    """
    macro = _load_macro_body()

    optical_link = macro.find("./link[@name='${name}_optical_frame']")
    assert optical_link is not None, "missing ${name}_optical_frame link"

    joint = macro.find("./joint[@name='${name}_optical_joint']")
    assert joint is not None, "missing ${name}_optical_joint"
    assert joint.attrib.get("type") == "fixed"
    assert joint.find("parent").attrib["link"] == "${name}_link"
    assert joint.find("child").attrib["link"] == "${name}_optical_frame"

    # REP-103 body->optical rotation, expressed with xacro's pi property.
    rpy = joint.find("origin").attrib["rpy"].split()
    assert rpy == ["-${pi", "/", "2}", "0", "-${pi", "/", "2}"], (
        f"unexpected optical rotation: {joint.find('origin').attrib['rpy']!r}"
    )


def test_production_xacro_binds_sensor_frame_to_optical_not_map():
    """F2 / R03-AC3: the sensor must publish its truthful optical frame.

    The Gazebo sensor gz_frame_id must resolve to the declared
    camera_frame_id, and that property must be the *_optical_frame -- never
    a map/world frame -- so optical coordinates are not relabeled as map
    coordinates at the producer.
    """
    macro = _load_macro_body()

    gz_frame = macro.find(".//gz_frame_id")
    assert gz_frame is not None, "missing gz_frame_id on the RGB-D sensor"
    assert gz_frame.text == "${camera_frame_id}", (
        f"sensor frame must bind to the declared camera_frame_id, got "
        f"{gz_frame.text!r}"
    )

    # camera_frame_id must be defined as the optical frame in both the
    # namespaced and un-namespaced branches, and must not be a map frame.
    frame_values = [
        prop.attrib["value"]
        for prop in macro.iter(f"{{{XACRO_NS}}}property")
        if prop.attrib.get("name") == "camera_frame_id"
    ]
    assert frame_values, "camera_frame_id property is not declared"
    for value in frame_values:
        assert "_optical_frame" in value, (
            f"camera_frame_id must reference the optical frame, got {value!r}"
        )
        assert "map" not in value and "world" not in value, (
            f"camera_frame_id must not relabel to a map/world frame: {value!r}"
        )


def test_namespaced_camera_renders_truthful_optical_child_and_sensor_frame():
    robot = render_camera("ugv_1")

    assert robot.find("./link[@name='camera_link']") is not None
    assert robot.find("./link[@name='camera_optical_frame']") is not None

    joint = robot.find("./joint[@name='camera_optical_joint']")
    assert joint is not None
    assert joint.find("parent").attrib["link"] == "camera_link"
    assert joint.find("child").attrib["link"] == "camera_optical_frame"
    rpy = [float(value) for value in joint.find("origin").attrib["rpy"].split()]
    assert math.isclose(rpy[0], -math.pi / 2.0, abs_tol=1e-6)
    assert math.isclose(rpy[1], 0.0, abs_tol=1e-12)
    assert math.isclose(rpy[2], -math.pi / 2.0, abs_tol=1e-6)

    assert robot.find(".//gz_frame_id").text == "ugv_1/camera_optical_frame"


def test_unnamespaced_camera_uses_the_same_optical_contract():
    robot = render_camera("")
    assert robot.find(".//gz_frame_id").text == "camera_optical_frame"


def test_gz_world_plugin_system_prefix_resolution():
    import os
    import sys
    import importlib.util
    from unittest.mock import MagicMock

    sys.modules.setdefault("rclpy", MagicMock())
    sys.modules.setdefault("rclpy.node", MagicMock())
    sys.modules.setdefault("ament_index_python", MagicMock())
    sys.modules.setdefault("ament_index_python.packages", MagicMock())

    gz_world_path = (
        pathlib.Path(__file__).resolve().parents[2]
        / "wolf_description_utils"
        / "scripts"
        / "ros2"
        / "gz_world"
    )
    ns = {"__file__": str(gz_world_path), "__name__": "gz_world"}
    with open(gz_world_path, "r", encoding="utf-8") as f:
        exec(f.read(), ns)

    dummy_world = """<?xml version="1.0"?>
<sdf version="1.9">
  <world name="default">
  </world>
</sdf>
"""
    for distro, expected_filename in [
        ("jazzy", "gz-sim-air-pressure-system"),
        ("humble", "ignition-gazebo-air-pressure-system"),
    ]:
        with tempfile.NamedTemporaryFile(mode="w", suffix=".sdf", delete=False) as world_file:
            world_file.write(dummy_world)
            world_path = world_file.name

        try:
            original_distro = os.environ.get("ROS_DISTRO")
            os.environ["ROS_DISTRO"] = distro
            resolved_prefix = ns["resolve_system_plugin_prefix"](distro)
            plugin_filename = f"{resolved_prefix}-air-pressure-system"
            assert plugin_filename == expected_filename

            updated_world, _ = ns["_ensure_world_plugin"](
                world_path, "gz::sim::systems::AirPressure", plugin_filename
            )
            tree = ET.parse(updated_world)
            plugins = tree.findall(f".//plugin[@filename='{expected_filename}']")
            assert len(plugins) == 1
        finally:
            if original_distro is not None:
                os.environ["ROS_DISTRO"] = original_distro
            else:
                os.environ.pop("ROS_DISTRO", None)
            if os.path.exists(world_path):
                os.remove(world_path)

