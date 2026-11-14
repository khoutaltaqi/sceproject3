"""One ROS 2 trial: acquire, drive, brake, settle, and request a tape measurement."""
import argparse
import hashlib
import json
import math
import platform
import sys
import time
from datetime import datetime
from pathlib import Path
import cv2
import numpy as np
from .common import EP, csv_file, folder, save, settings
from .measure import record
from .model import EKF, cubic, motion, tag_model, wrap
from .vision import Vision, draw


TRACE_FIELDS = ['time_s', 'state', 'v_cmd_mps', 'w_cmd_radps',
        'v_odom_mps', 'w_odom_radps', 'odom_distance_m', 'predicted_distance_m', 'camera_distance_m',
        'estimated_distance_m', 'range_innovation_m', 'bearing_innovation_rad', 'vision_nis',
        'vision_used', 'px_m', 'py_m', 'theta_rad', 'v_est_mps', 'w_est_radps', 'P_flat_json',
        'dt_s', 'frame_age_s', 'frame_number',
        'plan_time_s', 'x_ref_m', 'y_ref_m', 'v_ref_mps',
        'odom_px_m', 'odom_py_m', 'odom_theta_rad',
        'pred_px_m', 'pred_py_m', 'pred_theta_rad',
        'before_tag_px_m', 'before_tag_py_m', 'before_tag_theta_rad',
        'odom_correction_x_m', 'odom_correction_y_m', 'odom_correction_theta_rad',
        'tag_correction_x_m', 'tag_correction_y_m', 'tag_correction_theta_rad',
        'tracking_error_x_m', 'tracking_error_y_m', 'tracking_error_norm_m',
        'predicted_bearing_rad', 'camera_bearing_rad', 'estimated_bearing_rad',
        'range_residual_m', 'bearing_residual_rad',
        'pred_sigma_x_m', 'pred_sigma_y_m', 'pred_sigma_theta_rad',
        'sigma_x_m', 'sigma_y_m', 'sigma_theta_rad', 'odom_fresh', 'odom_used']


class Trial:
    def __init__(self, ep, c, vision, start, out, trace, updates=None):
        import rclpy
        from geometry_msgs.msg import Twist
        from nav_msgs.msg import Odometry
        self.node = rclpy.create_node('rm_phase3_trial')
        self.pub = self.node.create_publisher(Odometry, '/rm_phase3/estimate', 10)
        self.cmd_pub = self.node.create_publisher(Twist, '/rm_phase3/cmd_vel', 10)
        self.Odometry, self.Twist = Odometry, Twist
        self.ep, self.c, self.vision, self.start = ep, c, vision, start
        self.out, self.trace, self.filter = out, trace, EKF(c)
        self.updates, self.odom_pose = updates, np.zeros(3)
        self.landmark = np.array([start, 0.0])
        self.R = np.diag([vision.mapping['range_std_m']**2, c['bearing_std_rad']**2])
        self.began = self.previous = time.monotonic()
        self.state, self.reason, self.done = 'WAIT', '', False
        self.command, self.u = np.zeros(2), np.zeros(2)
        self.frame_number, self.odom_time, self.old_yaw = -1, 0, None
        self.stable, self.v, self.w = 0, 0.0, 0.0
        self.drive_at, self.stop_at, self.stall_at = None, None, None
        self.gap_at, self.reacquire = None, 0
        self.stationary, self.stationary_odom, self.writer, self.result = [], [], None, {}
        self.last_view = None
        self.length = start-c['stop_m']
        self.T = max(12.0, 1.5*self.length/c['max_speed_mps'])
        self.timer = self.node.create_timer(0.1, self.tick)

    def finish(self, reason, now):
        if self.state != 'SETTLE':
            self.reason = reason
            self.state = 'SETTLE'
            self.stop_at = now
            print('Braking:', reason)

    def publish(self):
        x, P = self.filter.x, self.filter.P
        msg = self.Odometry()
        msg.header.stamp = self.node.get_clock().now().to_msg()
        msg.header.frame_id, msg.child_frame_id = 'trial_origin', 'front_reference'
        msg.pose.pose.position.x, msg.pose.pose.position.y = float(x[0]), float(x[1])
        msg.pose.pose.orientation.z, msg.pose.pose.orientation.w = math.sin(x[2]/2), math.cos(x[2]/2)
        for i, row in enumerate([0, 1, 5]):
            for j, col in enumerate([0, 1, 5]):
                msg.pose.covariance[6*row+col] = float(P[i, j])
        msg.twist.twist.linear.x, msg.twist.twist.angular.z = float(x[3]), float(x[4])
        self.pub.publish(msg)
        cmd = self.Twist()
        cmd.linear.x, cmd.angular.z = map(float, self.command)
        self.cmd_pub.publish(cmd)

    def tick(self):
        try:
            self.step()
        except Exception as exc:
            self.result = {'status': 'ERROR', 'detail': str(exc), 'estimated_final_m': None}
            self.done = True
            self.ep.stop()

    def step(self):
        now = time.monotonic()
        dt, self.previous = now-self.previous, now
        if not 0 < dt < 0.3:
            raise RuntimeError(f'Control interval {dt:.3f}s exceeded watchdog')
        self.filter.predict(self.u, dt)
        predicted, P_predicted = self.filter.x.copy(), self.filter.P.copy()
        odom_update, tag_update = None, None
        data = self.ep.snapshot()
        fresh = all(k in data and now-data[k][0] < 0.3 for k in ['velocity', 'attitude'])
        arm = data.get('arm')
        arm_ok = arm and now-arm[0] < 0.8 and max(
            abs(arm[1][0]-self.c['arm_x_mm']), abs(arm[1][1]-self.c['arm_y_mm'])) <= 5
        if self.ep.fault or not arm_ok or not fresh:
            self.finish('SENSOR_OR_ARM_FAULT', now)
        if fresh:
            vt, velocity = data['velocity']
            at, attitude = data['attitude']
            if self.old_yaw is None:
                self.old_yaw = (at, attitude[0])
            if vt > self.odom_time and at > self.old_yaw[0]:
                self.v = float(velocity[3])  # SDK body forward velocity, same as Phase 2.
                self.w = float(-wrap(np.radians(attitude[0]-self.old_yaw[1]))/(at-self.old_yaw[0]))
                self.old_yaw, self.odom_time = (at, attitude[0]), vt
                if not np.isfinite([self.v, self.w]).all() or abs(self.w) > 2:
                    raise RuntimeError('Invalid odometry feedback')
                self.filter.odom(self.v, self.w)
                odom_update = self.filter.last_update
            # Tag-free baseline: lecture motion model with measured v/omega.
            self.odom_pose = motion(np.r_[self.odom_pose, self.v, self.w], [0, 0], dt)[0][:3]
            if abs(velocity[4]) > 0.05:
                self.finish('LATERAL_MOTION', now)
        before_tag = self.filter.x.copy()
        prior, prior_bearing = tag_model(before_tag, self.landmark)[0]
        item, z, accepted, innovation, nis = self.ep.frame, None, False, [None, None], None
        frame_age = None if item is None else now-item[0]
        if item and item[1] != self.frame_number and frame_age < 0.25:
            self.frame_number = item[1]
            z = self.vision.read(item[2])
            if z:
                accepted, innovation, nis = self.filter.tag(
                    [z['range_m'], z['bearing_rad']], self.landmark, self.R)
                tag_update = self.filter.last_update
        d, phi = tag_model(self.filter.x, self.landmark)[0]
        odom_distance = float(np.linalg.norm(self.landmark-self.odom_pose[:2]))
        v, w = 0.0, 0.0
        if self.state == 'WAIT':
            good = accepted and abs(z['range_m']-self.start) < 0.06 and abs(z['bearing_rad']) < np.radians(3)
            good = good and abs(self.v) < 0.01 and abs(self.w) < 0.03 and fresh and arm_ok
            self.stable = self.stable+1 if good else 0
            if self.stable >= 8:
                self.state, self.drive_at = 'DRIVE', now
                cv2.imwrite(str(self.out/'start.jpg'), draw(item[2], z, 'START'))
                print(f'DRIVE: start={self.start:.3f}m, stop={self.c["stop_m"]:.3f}m, T={self.T:.2f}s')
            elif now-self.began > 20:
                self.finish('START_ALIGNMENT_FAILED', now)
        if self.state == 'DRIVE':
            if not z or not accepted:
                self.gap_at = self.gap_at or now
                self.reacquire = 0
                if now-self.gap_at >= 0.6:
                    self.finish('TAG_LOST_OR_REJECTED', now)
            elif self.gap_at is not None and self.reacquire < 2:
                self.reacquire += 1  # Hold zero until three consecutive accepted frames.
            elif abs(phi) > np.radians(15):
                self.finish('HEADING_LIMIT', now)
            elif min(d, z['range_m']) <= self.c['stop_m']+self.c['stop_tolerance_m']:
                self.finish('ARRIVED', now)
            else:
                self.gap_at = None
                elapsed = now-self.drive_at
                sref, vff = cubic(elapsed, self.T, self.length)
                progress = self.start-d
                v = np.clip(vff+self.c['position_gain']*(sref-progress), 0, self.c['max_speed_mps'])
                margin = max(0.0, min(d, z['range_m'])-self.c['stop_m'])
                cap = min(0.8*margin, math.sqrt(2*self.c['max_accel_mps2']*margin))
                v = min(cap, np.clip(v, self.command[0]-self.c['max_accel_mps2']*dt,
                                          self.command[0]+self.c['max_accel_mps2']*dt))
                target_w = np.clip(self.c['bearing_gain']*phi, -self.c['max_turn_radps'], self.c['max_turn_radps'])
                dw = self.c['max_angular_accel_radps2']*dt
                w = float(np.clip(target_w, self.command[1]-dw, self.command[1]+dw))
                if self.command[0] > 0.025 and abs(self.v) < 0.008:
                    self.stall_at = self.stall_at or now
                else:
                    self.stall_at = None
                if self.stall_at and now-self.stall_at > 3:
                    self.finish('NO_MOTION_FEEDBACK', now)
                elif elapsed > self.T+25 or now-self.began > self.c['max_trial_s']:
                    self.finish('TRACKING_TIMEOUT', now)
        if self.state == 'SETTLE':
            v = w = 0.0
            settled = accepted and fresh and arm_ok and abs(self.v) < 0.01 and abs(self.w) < 0.03
            if settled and now-self.stop_at >= 1:
                self.stationary.append(float(d))
                self.stationary = self.stationary[-12:]
                self.stationary_odom.append(odom_distance)
                self.stationary_odom = self.stationary_odom[-12:]
            else:
                self.stationary.clear()
                self.stationary_odom.clear()
            if now-self.stop_at >= 2.2 and len(self.stationary) >= 10:
                final = float(np.median(self.stationary))
                status = self.reason
                if status == 'ARRIVED' and (abs(final-self.c['stop_m']) > 0.02 or abs(phi) > np.radians(3)):
                    status = 'STOP_OUTSIDE_TOLERANCE'
                self.result = {'status': status, 'estimated_final_m': final if self.drive_at else None,
                    'camera_final_m': z['range_m'], 'odom_final_m': float(np.median(self.stationary_odom)),
                    'odom_final_pose': self.odom_pose.tolist(),
                    'drive_start_time_s': self.drive_at-self.began if self.drive_at is not None else None,
                    'final_pose': self.filter.x.tolist(), 'final_covariance': self.filter.P.tolist(),
                    'final_samples': list(self.stationary), 'duration_s': now-self.began,
                    'planned_duration_s': self.T, 'trace_origin_monotonic_s': self.began,
                    'final_capture_at': datetime.now().astimezone().isoformat()}
                self.done = True
                cv2.imwrite(str(self.out/'finish.jpg'), draw(item[2], z, status))
            elif now-self.stop_at > 6:
                self.result = {'status': 'FINAL_CAPTURE_FAILED', 'brake_reason': self.reason,
                               'estimated_final_m': None}
                self.done = True
        if self.state == 'SETTLE':
            v = w = 0.0
        new = np.array([v, w], dtype=float)
        self.u = np.clip((new-self.command)/dt,
                        [-self.c['max_accel_mps2'], -self.c['max_angular_accel_radps2']],
                        [self.c['max_accel_mps2'], self.c['max_angular_accel_radps2']])
        self.command = new
        self.ep.command(v, w)
        self.publish()
        x = self.filter.x
        plan_t = None if self.drive_at is None else now-self.drive_at
        ref, vff = (None, None) if plan_t is None else cubic(plan_t, self.T, self.length)
        err_x, err_y = (None, None) if ref is None else (x[0]-ref, x[1])
        odom_delta, tag_delta = before_tag-predicted, x-before_tag
        odom_delta[2], tag_delta[2] = wrap(odom_delta[2]), wrap(tag_delta[2])
        if self.updates is not None:
            for sensor, update in [('odom', odom_update), ('tag', tag_update)]:
                if update is not None:
                    self.updates.writerow([now-self.began, sensor, json.dumps(update, allow_nan=False)])
        self.trace.writerow([now-self.began, self.state, v, w, self.v, self.w,
            odom_distance if fresh else None, prior, None if not z else z['range_m'], float(d),
            *innovation, nis, int(accepted), *x.tolist(),
            json.dumps(self.filter.P.flatten().tolist()), dt, frame_age, self.frame_number,
            plan_t, ref, None if ref is None else 0.0, vff,
            *(self.odom_pose.tolist() if fresh else [None]*3),
            *predicted[:3].tolist(), *before_tag[:3].tolist(),
            *odom_delta[:3].tolist(), *tag_delta[:3].tolist(),
            err_x, err_y, None if err_x is None else float(np.hypot(err_x, err_y)),
            prior_bearing, None if not z else z['bearing_rad'], float(phi),
            None if not z else z['range_m']-float(d),
            None if not z else float(wrap(z['bearing_rad']-phi)),
            *np.sqrt(np.maximum(0, np.diag(P_predicted)[:3])).tolist(),
            *np.sqrt(np.maximum(0, np.diag(self.filter.P)[:3])).tolist(), int(fresh),
            int(odom_update is not None)])
        if item:
            view = draw(item[2], z, f'{self.state} EKF={d:.3f}m')
            if self.writer is None:
                self.writer = cv2.VideoWriter(str(self.out/'trial.avi'),
                    cv2.VideoWriter_fourcc(*'MJPG'), 10, (view.shape[1], view.shape[0]))
                if not self.writer.isOpened():
                    raise RuntimeError('Cannot create trial.avi')
            self.writer.write(view)
            self.last_view = view

    def close(self):
        self.timer.cancel()
        if self.writer:
            self.writer.release()
        self.node.destroy_node()


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--start', type=float, help='Actual tape-measured starting distance, metres')
    a = p.parse_args()
    import rclpy
    c = settings()
    vision = Vision(c)
    if not vision.mapping.get('validated'):
        raise RuntimeError('Run calibrate verify successfully before a motion trial')
    actual_start = a.start if a.start is not None else float(input(
        'Place the fixed front reference 1.50 m from the tag. Enter actual tape distance [m]: '))
    if not np.isfinite(actual_start) or abs(actual_start-c['start_m']) > 0.02:
        raise ValueError('All trials must start at 1.50 m, within 2 cm')
    out, trial = folder('trial'), None
    save(out/'config.json', c)
    save(out/'camera.json', vision.cal)
    save(out/'distance.json', vision.mapping)
    sources = {f.name: hashlib.sha256(f.read_bytes()).hexdigest() for f in Path(__file__).parent.glob('*.py')}
    protocol = {'config': c, 'camera': vision.cal, 'distance': vision.mapping, 'sources': sources}
    experiment = hashlib.sha256(json.dumps(protocol, sort_keys=True).encode()).hexdigest()[:12]
    save(out/'software.json', {'python': sys.version, 'executable': sys.executable,
        'platform': platform.platform(), 'numpy': np.__version__, 'opencv': cv2.__version__, 'sources': sources})
    summary = {'kind': 'rm_phase3_trial', 'experiment_id': experiment, 'status': 'STARTUP',
        'trace_schema_version': 2, 'landmark_xy_m': [actual_start, 0.0],
        'planned_length_m': actual_start-c['stop_m'],
        'odometry_method': 'Lecture 11A integration of measured forward velocity and yaw-derived turn rate',
        'started_at': datetime.now().astimezone().isoformat(), 'start_actual_m': actual_start,
        'start_nominal_m': c['start_m'], 'stop_target_m': c['stop_m'], 'reference': c['reference'],
        'estimated_final_m': None, 'actual_final_m': None, 'error_m': None}
    save(out/'summary.json', summary)
    raw_file, raw = csv_file(out/'sensors.csv', ['monotonic_s', 'sensor', 'values_json'])
    trace_file, trace = csv_file(out/'trace.csv', TRACE_FIELDS)
    updates_file, updates = csv_file(out/'updates.csv', ['time_s', 'sensor', 'update_json'])
    rclpy.init()
    print('Trial folder:', out)
    try:
        with raw_file, trace_file, updates_file, EP(c, log=raw) as ep:
            ep.arm()
            trial = Trial(ep, c, vision, actual_start, out, trace, updates)
            while rclpy.ok() and not trial.done:
                rclpy.spin_once(trial.node, timeout_sec=0.2)
                trace_file.flush()
                updates_file.flush()
            summary.update(trial.result or {'status': 'INTERRUPTED'})
    except KeyboardInterrupt:
        summary['status'] = 'INTERRUPTED'
    except Exception as exc:
        summary.update(status='ERROR', detail=str(exc))
    finally:
        if trial:
            trial.close()
        if rclpy.ok():
            rclpy.shutdown()
        save(out/'summary.json', summary)
    print('Status:', summary['status'])
    if summary['estimated_final_m'] is not None:
        print(f"Saved final estimate: {summary['estimated_final_m']:.4f} m")
        try:
            text = input('Keep robot in place. Enter independent FINAL tape distance [m], or Enter to record later: ')
            if text.strip():
                record(out, float(text))
        except (EOFError, KeyboardInterrupt):
            pass
    print('Saved:', out)


if __name__ == '__main__':
    main()
