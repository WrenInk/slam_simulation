#!/usr/bin/env python3
"""Drive the running arena using Nav2 actions; truth is read only for scoring."""
import argparse
import json
import math
from pathlib import Path
import time
import numpy as np
import rclpy
from rclpy.action import ActionClient
from geometry_msgs.msg import Twist
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import Odometry, OccupancyGrid
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import String
from std_srvs.srv import Trigger
from lifecycle_msgs.srv import GetState
from rclpy.qos import QoSProfile, DurabilityPolicy


def angle(value):
    return math.atan2(math.sin(value), math.cos(value))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=True)
    rclpy.init()
    node = rclpy.create_node('omni_navigation_evaluator')
    client = ActionClient(node, NavigateToPose, '/navigate_to_pose')
    state = dict(truth=None, cloud_frames=0, map_free_cells=0, collision_steps=0)
    traces, commands = [], []
    active = None
    def truth(msg):
        p, q = msg.pose.pose.position, msg.pose.pose.orientation
        yaw = math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))
        state['truth'] = np.array([p.x, p.y, yaw])
        if active is not None:
            traces.append([active, msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9, p.x, p.y, yaw])
    def command(msg):
        if active is not None:
            commands.append([active, msg.linear.x, msg.linear.y, msg.angular.z])
    def cloud(msg): state['cloud_frames'] += 1
    def grid(msg): state['map_free_cells'] = sum(v == 0 for v in msg.data)
    def status(msg): state['collision_steps'] = json.loads(msg.data)['collision_steps']
    node.create_subscription(Odometry, '/ground_truth/odom', truth, 10)
    node.create_subscription(Twist, '/cmd_vel', command, 10)
    node.create_subscription(PointCloud2, '/cloud_registered_body', cloud, 10)
    node.create_subscription(String, '/sim/status', status, 10)
    node.create_subscription(OccupancyGrid, '/map', grid,
        QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
    def until(predicate, timeout):
        deadline = time.monotonic()+timeout
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=.05)
            if predicate(): return
        raise TimeoutError('Omni navigation acceptance timed out')
    results, handles, error = [], [], None
    # Map origin is the startup IMU above the base at world (-3.8,-3.8).
    # The third direct segment crosses crate_a: Nav2 must route around it.
    goals = [('strafe', (0., 1.6, 0.)), ('forward', (1., 1.6, 0.)),
             ('around_crate', (4., 1.6, math.pi/4)), ('diagonal', (4., 4.6, math.pi/2))]
    try:
        until(lambda: client.server_is_ready() and state['truth'] is not None
              and state['cloud_frames'] > 15 and state['map_free_cells'] > 100, 100.)
        lifecycle = node.create_client(GetState, '/bt_navigator/get_state')
        if not lifecycle.wait_for_service(timeout_sec=10.): raise RuntimeError('Missing Nav2 lifecycle')
        deadline = time.monotonic()+45.
        while time.monotonic() < deadline:
            future = lifecycle.call_async(GetState.Request())
            try: until(future.done, 5.)
            except TimeoutError: continue
            if future.result().current_state.id == 3: break
        else: raise TimeoutError('Nav2 did not activate')
        for index, (name, target) in enumerate(goals):
            active = index
            goal = NavigateToPose.Goal()
            goal.pose.header.frame_id = 'map'
            goal.pose.pose.position.x, goal.pose.pose.position.y = target[:2]
            goal.pose.pose.orientation.z = math.sin(target[2]/2)
            goal.pose.pose.orientation.w = math.cos(target[2]/2)
            print('Navigating '+name+' '+str(target), flush=True)
            future = client.send_goal_async(goal)
            until(future.done, 15.)
            handle = future.result()
            if not handle.accepted: raise RuntimeError('Nav2 rejected '+name)
            handles.append(handle)
            future = handle.get_result_async()
            until(future.done, 150.)
            actual = state['truth']
            expected = np.array(target[:2])+[-3.8, -3.8]
            trace = np.array([t[2:] for t in traces if t[0] == index])
            distance = np.linalg.norm(np.diff(trace[:,:2],axis=0),axis=1).sum()
            near = np.flatnonzero(np.linalg.norm(trace[:,:2]-expected,axis=1) < .4)
            result = dict(name=name, goal_map=list(target), action_status=future.result().status,
                truth_xy_error_m=float(np.linalg.norm(actual[:2]-expected)),
                truth_yaw_error_rad=abs(angle(actual[2]-target[2])), actual_world=actual.tolist(),
                path_length_m=float(distance), elapsed_sim_s=float([t for t in traces if t[0]==index][-1][1]-[t for t in traces if t[0]==index][0][1]),
                yaw_error_at_40cm_rad=abs(angle(trace[near[0],2]-target[2])) if len(near) else None)
            if name == 'strafe':
                result['max_yaw_deviation_rad'] = float(np.max(np.abs(trace[:,2])))
                result['lateral_commands'] = sum(c[0]==index and abs(c[2]) > .05 for c in commands)
            if name == 'around_crate':
                result['max_detour_y_m'] = float(np.max(np.abs(trace[:,1]-expected[1])))
            results.append(result)
            print(json.dumps(result), flush=True)
            if future.result().status != 4: raise RuntimeError('Nav2 failed '+name)
        active = None
        service = node.create_client(Trigger, '/slam/save_map')
        if not service.wait_for_service(timeout_sec=5.): raise RuntimeError('Missing SLAM map saver')
        future = service.call_async(Trigger.Request()); until(future.done, 30.)
        if not future.result().success: raise RuntimeError(future.result().message)
    except Exception as exc:
        error = str(exc)
    finally:
        if error:
            for handle in handles:
                try:
                    future = handle.cancel_goal_async(); until(future.done, 3.)
                except Exception: pass
        passed = (error is None and len(results) == len(goals) and state['collision_steps'] == 0
            and all(r['action_status']==4 and r['truth_xy_error_m'] < .2 and r['truth_yaw_error_rad'] < .2 for r in results)
            and results[0].get('lateral_commands',0) > 10 and results[0].get('max_yaw_deviation_rad',99) < .15
            and results[2].get('max_detour_y_m',0) > .65)
        report = dict(passed=bool(passed), error=error, goals=results,
            streams={k:v for k,v in state.items() if k!='truth'},
            localization='MID360-like rays + measured IMU -> FAST-LIO2 -> LIO-SAM-style factor graph; no truth feedback',
            navigation='Nav2 SmacPlanner2D + MPPI Omni + velocity smoother',
            dynamics='Free chassis; equivalent mecanum roller traction, not a calibrated full roller model')
        (args.output/'navigation.json').write_text(json.dumps(report, indent=2))
        np.savetxt(args.output/'goal_trace.csv',traces,delimiter=',',header='goal,time,x,y,yaw')
        np.savetxt(args.output/'cmd_vel.csv',commands,delimiter=',',header='goal,vx,vy,wz')
        print(json.dumps(report), flush=True)
        node.destroy_node(); rclpy.shutdown()
    return 0 if passed else 1


if __name__ == '__main__':
    raise SystemExit(main())
