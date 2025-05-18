#!/usr/bin/env python3

import argparse
import xml.etree.ElementTree as ET
from scipy.spatial.transform import Rotation as R
import numpy as np
import os

def parse_tracklet_xml(input_xml, output_txt):
    tree = ET.parse(input_xml)
    root = tree.getroot()
    poses_all = []

    # Process each tracklet
    for tracklet in root.findall(".//item"):
        poses = tracklet.find("poses")
        if poses is None:
            continue

        for pose in poses.findall("item"):
            tx = float(pose.find("tx").text)
            ty = float(pose.find("ty").text)
            tz = float(pose.find("tz").text)
            rx = float(pose.find("rx").text)
            ry = float(pose.find("ry").text)
            rz = float(pose.find("rz").text)

            # Convert Euler angles to rotation matrix
            rot = R.from_euler('xyz', [rx, ry, rz])
            rot_matrix = rot.as_matrix()

            # Build 3x4 matrix
            pose_matrix = np.hstack((rot_matrix, np.array([[tx], [ty], [tz]])))
            pose_flat = pose_matrix.flatten()
            poses_all.append(pose_flat)

    # Write poses to KITTI-format file
    with open(output_txt, 'w') as f:
        for pose in poses_all:
            f.write(' '.join(map(str, pose)) + '\n')

    print(f"[✓] Converted {len(poses_all)} poses to: {output_txt}")

def main():
    parser = argparse.ArgumentParser(description="Convert KITTI tracklet XML to KITTI trajectory format.")
    parser.add_argument("input_xml", help="Path to the tracklet_labels.xml file")
    parser.add_argument("output_txt", help="Output KITTI-style trajectory file")

    args = parser.parse_args()

    if not os.path.exists(args.input_xml):
        print(f"[✗] Error: File not found: {args.input_xml}")
        return

    parse_tracklet_xml(args.input_xml, args.output_txt)

if __name__ == "__main__":
    main()
