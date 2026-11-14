"""Check Python, ROS 2, vision, and optional stationary robot feedback."""
import argparse
import importlib
import os
import platform
import sys
import time
from .common import EP, folder, save, settings


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--robot', action='store_true', help='Also connect, without moving the arm/chassis')
    a = p.parse_args()
    c, out = settings(), folder('check')
    report = {'python': sys.version, 'executable': sys.executable, 'platform': platform.platform(),
              'ROS_DISTRO': os.environ.get('ROS_DISTRO', '')}
    failed = []
    for name in ['numpy', 'cv2', 'matplotlib', 'rclpy', 'nav_msgs.msg', 'geometry_msgs.msg', 'robomaster.robot']:
        try:
            m = importlib.import_module(name)
            if name == 'numpy' and int(m.__version__.split('.')[0]) >= 2:
                raise RuntimeError('Use NumPy 1.26.4 with this Phase 2/ROS setup')
            if name == 'cv2':
                from .vision import detector
                detector()
                gui = next((s.strip() for s in m.getBuildInformation().splitlines() if 'GUI:' in s), '')
                if 'NONE' in gui or not (os.environ.get('DISPLAY') or os.environ.get('WAYLAND_DISPLAY')):
                    raise RuntimeError('Calibration needs desktop OpenCV with a display, not headless OpenCV')
            report[name] = {'ok': True, 'file': str(getattr(m, '__file__', '')),
                            'version': str(getattr(m, '__version__', ''))}
            print(f'PASS {name}')
        except Exception as exc:
            report[name] = {'ok': False, 'error': str(exc)}
            failed.append(name)
            print(f'FAIL {name}: {exc}')
    if a.robot and not failed:
        try:
            from .vision import detector
            with EP(c) as ep:
                time.sleep(2)
                data, frame = ep.snapshot(), ep.frame
                if not frame or time.monotonic()-frame[0] > 0.5:
                    raise RuntimeError('No fresh decoded camera frame. Check SDK codec/network.')
                for key in ['velocity', 'attitude', 'arm']:
                    if key not in data or time.monotonic()-data[key][0] > 0.5:
                        raise RuntimeError(f'No fresh {key} feedback')
                _, ids, _ = detector().detectMarkers(frame[2])
                report['robot'] = {'ok': True, 'firmware': ep.ep.get_version(),
                    'image_size': list(frame[2].shape[:2][::-1]),
                    'detected_ids': [] if ids is None else ids.flatten().tolist(),
                    'sensors': data}
                import cv2
                cv2.imwrite(str(out/'camera.jpg'), frame[2])
                print('PASS robot, camera, velocity, attitude, arm')
                print('Detected IDs:', report['robot']['detected_ids'])
        except Exception as exc:
            failed.append('robot')
            report['robot'] = {'ok': False, 'error': str(exc)}
            print(f'FAIL robot: {exc}')
    report['passed'] = not failed
    save(out/'environment.json', report)
    print('Saved:', out)
    if failed:
        raise SystemExit(1)


if __name__ == '__main__':
    main()
