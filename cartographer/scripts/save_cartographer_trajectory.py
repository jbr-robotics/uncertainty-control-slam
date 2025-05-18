#!/usr/bin/env python3
"""
Dump Cartographer ROS 2 trajectory to a text file.
$ ./dump_traj.py 0 carto.tum
"""
import sys, rclpy
from rclpy.node import Node
from cartographer_ros_msgs.srv import TrajectoryQuery

class DumpTraj(Node):
    def __init__(self, tid, out):
        super().__init__('dump_traj')
        self.tid, self.out = tid, out
        self.cli = self.create_client(TrajectoryQuery, '/trajectory_query')
        if not self.cli.wait_for_service(timeout_sec=5.0):
            self.get_logger().error('trajectory_query not available'); return
        req = TrajectoryQuery.Request();  req.trajectory_id = tid
        future = self.cli.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        res = future.result()
        with open(out, 'w') as f:
            for pose in res.trajectory:
                t = pose.header.stamp.sec + pose.header.stamp.nanosec*1e-9
                p, q = pose.pose.position, pose.pose.orientation
                f.write(f"{t:.9f} {p.x} {p.y} {p.z} {q.x} {q.y} {q.z} {q.w}\n")
        self.get_logger().info(f"Wrote {len(res.trajectory)} poses to {out}")

if __name__ == '__main__':
    rclpy.init(); DumpTraj(int(sys.argv[1]), sys.argv[2]); rclpy.shutdown()
