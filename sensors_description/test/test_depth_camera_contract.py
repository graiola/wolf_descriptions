#!/usr/bin/env python3

"""Render the shared RGB-D macro and verify its ROS optical-frame contract."""

import math
import pathlib
import subprocess
import tempfile
import xml.etree.ElementTree as ET


MACRO_PATH = (
    pathlib.Path(__file__).resolve().parents[1]
    / "urdfs"
    / "sensors"
    / "depth_camera.gazebo.xacro"
)


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
    with tempfile.NamedTemporaryFile(mode="w", suffix=".urdf.xacro") as source:
        source.write(wrapper)
        source.flush()
        rendered = subprocess.run(
            ["xacro", source.name],
            check=True,
            capture_output=True,
            text=True,
        ).stdout
    return ET.fromstring(rendered)


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

