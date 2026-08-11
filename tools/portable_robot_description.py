#!/usr/bin/env python3
"""Print a URDF with local mesh paths exposed through a ROS package URI."""

import argparse
from pathlib import Path
import tempfile
import xml.etree.ElementTree as ET


PACKAGE_NAME = "g1_rh56e2_assets"
PREFIX = Path(tempfile.gettempdir()) / "g1_lio_ws_asset_prefix"


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--urdf", required=True)
    args = parser.parse_args()

    root = ET.parse(args.urdf).getroot()
    mesh_elements = root.findall(".//mesh[@filename]")
    local_paths = [Path(mesh.get("filename")) for mesh in mesh_elements]
    absolute_paths = [path for path in local_paths if path.is_absolute()]
    if not absolute_paths:
        print(ET.tostring(root, encoding="unicode"))
        return

    # The generated URDF stores paths ending in models/mujoco/meshes/....
    # Register models/mujoco as a tiny runtime ROS package, then give RViz
    # portable package:// resource names without altering the source URDF.
    try:
        asset_root = next(
            path.parents[index]
            for path in absolute_paths
            for index, parent in enumerate(path.parents)
            if parent.name == "mujoco" and path.is_relative_to(parent / "meshes")
        )
    except StopIteration as error:
        raise RuntimeError("could not locate the URDF's mujoco/meshes root") from error

    marker_dir = PREFIX / "share/ament_index/resource_index/packages"
    share_dir = PREFIX / "share" / PACKAGE_NAME
    marker_dir.mkdir(parents=True, exist_ok=True)
    (marker_dir / PACKAGE_NAME).touch()
    share_dir.parent.mkdir(parents=True, exist_ok=True)
    if share_dir.is_symlink() and share_dir.resolve() != asset_root.resolve():
        share_dir.unlink()
    if not share_dir.exists():
        share_dir.symlink_to(asset_root, target_is_directory=True)

    for mesh, path in zip(mesh_elements, local_paths):
        if path.is_absolute() and path.is_relative_to(asset_root):
            relative = path.relative_to(asset_root).as_posix()
            mesh.set("filename", f"package://{PACKAGE_NAME}/{relative}")

    print(ET.tostring(root, encoding="unicode"))


if __name__ == "__main__":
    main()
