"""Paths, configuration, and the shared SDK connection."""
import csv
import json
import math
import os
import threading
import time
from datetime import datetime
from pathlib import Path
from queue import Empty


def home():
    source = Path(__file__).resolve().parents[1]
    return Path(os.environ.get('RM_PHASE3_HOME', source)).expanduser().resolve()


def settings():
    c = json.loads((home() / 'config.json').read_text())
    if c['connection'] not in ('ap', 'sta', 'rndis'):
        raise ValueError('connection must be ap, sta, or rndis')
    if not 0 < c['stop_m'] < c['start_m'] or c['tag_size_m'] <= 0:
        raise ValueError('Invalid start, stop, or tag size')
    return c


def save(path, data):
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    temp = path.with_suffix(path.suffix + '.tmp')
    temp.write_text(json.dumps(data, indent=2, allow_nan=False) + '\n')
    temp.replace(path)


def folder(kind, root='results'):
    stamp = datetime.now().astimezone().strftime('%Y%m%d_%H%M%S_%f%z')
    p = home() / root / f'{kind}_{stamp}'
    p.mkdir(parents=True, exist_ok=False)
    return p


class EP:
    """Latest uniquely numbered frames and timestamped SDK feedback."""
    def __init__(self, c, video=True, log=None):
        self.c, self.video, self.log = c, video, log
        self.data, self.frame, self.fault = {}, None, ''
        self.lock, self.subs = threading.Lock(), []
        self.running, self.streaming, self.ep = False, False, None
        self.thread = None

    def put(self, key, value):
        now = time.monotonic()
        with self.lock:
            self.data[key] = (now, tuple(value))
            if self.log:
                self.log.writerow([now, key, json.dumps(list(value))])

    def __enter__(self):
        from robomaster import robot
        try:
            self.ep = robot.Robot()
            self.ep.initialize(conn_type=self.c['connection'])
            self.ep.set_robot_mode(mode=robot.FREE)
            for key, obj, name, hz in [
                ('velocity', self.ep.chassis, 'velocity', 20),
                ('attitude', self.ep.chassis, 'attitude', 20),
                ('arm', self.ep.robotic_arm, 'position', 5),
            ]:
                if not getattr(obj, 'sub_' + name)(
                        freq=hz, callback=lambda v, k=key: self.put(k, v)):
                    raise RuntimeError(f'{key} subscription failed')
                self.subs.append((obj, name))
            if self.video:
                if self.ep.camera.start_video_stream(display=False, resolution='720p') is False:
                    raise RuntimeError('Camera stream failed')
                self.streaming = self.running = True
                self.thread = threading.Thread(target=self.grab, daemon=True)
                self.thread.start()
            return self
        except BaseException:
            self.close()
            raise

    def grab(self):
        n = 0
        while self.running:
            try:
                im = self.ep.camera.read_cv2_image(strategy='newest', timeout=0.3)
                if im is not None:
                    n += 1
                    self.frame = (time.monotonic(), n, im)
            except Empty:
                continue
            except Exception as exc:
                self.fault = str(exc)
                break

    def snapshot(self):
        with self.lock:
            return dict(self.data)

    def arm(self):
        x, y = self.c['arm_x_mm'], self.c['arm_y_mm']
        if not self.ep.robotic_arm.moveto(x=x, y=y).wait_for_completed(timeout=8):
            raise RuntimeError('Arm movement timed out')
        deadline = time.monotonic() + 3
        while time.monotonic() < deadline:
            item = self.snapshot().get('arm')
            if item and time.monotonic() - item[0] < 0.6:
                if max(abs(item[1][0] - x), abs(item[1][1] - y)) <= 5:
                    return item[1]
            time.sleep(0.05)
        raise RuntimeError('Arm feedback differs from (105, 60) mm')

    def command(self, v, w):
        self.ep.chassis.drive_speed(
            x=float(v),
            y=0,
            z=-math.degrees(w),
            timeout=0.3
    )

    def stop(self):
        if self.ep:
            ok = self.ep.chassis.drive_wheels(w1=0, w2=0, w3=0, w4=0, timeout=0.3)
            if ok is False:
                self.ep.chassis.drive_speed(x=0, y=0, z=0, timeout=0.3)

    def close(self):
        if not self.ep:
            return
        try:
            self.stop()
        except Exception:
            pass
        self.running = False
        if self.thread:
            self.thread.join(timeout=1)
        for obj, name in reversed(self.subs):
            try:
                getattr(obj, 'unsub_' + name)()
            except Exception:
                pass
        if self.streaming:
            try:
                self.ep.camera.stop_video_stream()
            except Exception:
                pass
        try:
            self.ep.close()
        except Exception:
            pass
        self.ep = None

    def __exit__(self, *_):
        self.close()


def csv_file(path, header):
    f = Path(path).open('w', newline='')
    w = csv.writer(f)
    w.writerow(header)
    return f, w
