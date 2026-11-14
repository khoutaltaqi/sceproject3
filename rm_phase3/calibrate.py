"""Camera intrinsics, tape-distance fitting, and independent validation."""
import argparse
import time
import cv2
import numpy as np
from .common import EP, csv_file, folder, home, save, settings
from .vision import Vision, draw


def camera(ep, c, out, cols, rows, square):
    pattern = (cols, rows)
    obj = np.zeros((cols*rows, 3), np.float32)
    obj[:, :2] = np.mgrid[:cols, :rows].T.reshape(-1, 2)*square
    objects, pixels, previous, number, size = [], [], None, -1, None
    last_frame = time.monotonic()
    print(f'Show a {cols}x{rows} INNER-corner board, {square*1000:g} mm squares.')
    print('Use varied tilts, distances and image positions. S: save. Q: solve after >=15 views.')
    while True:
        item = ep.frame
        if ep.fault:
            raise RuntimeError(ep.fault)
        if item is None or item[1] == number:
            if time.monotonic()-last_frame > 5:
                raise RuntimeError('No fresh camera frames for 5 seconds')
            time.sleep(0.02)
            continue
        received, number, im = item
        last_frame = time.monotonic()
        if time.monotonic()-received > 0.5:
            raise RuntimeError('Stale camera frame')
        gray = cv2.cvtColor(im, cv2.COLOR_BGR2GRAY)
        size = gray.shape[::-1]
        found, pts = cv2.findChessboardCornersSB(gray, pattern)
        view = im.copy()
        if found:
            cv2.drawChessboardCorners(view, pattern, pts, found)
        cv2.putText(view, f'{len(pixels)} saved | S save | Q solve', (20, 35),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
        cv2.imshow('Camera calibration', view)
        key = cv2.waitKey(20) & 255
        if key == ord('s') and found:
            if previous is not None and np.mean(np.linalg.norm(pts-previous, axis=2)) < 12:
                print('Change board pose before saving another view.')
                continue
            objects.append(obj.copy())
            pixels.append(pts.astype(np.float32))
            previous = pts.copy()
            cv2.imwrite(str(out/f'view_{len(pixels):02d}.jpg'), im)
            print('Saved view', len(pixels))
        if key == ord('q'):
            if len(pixels) >= 15:
                break
            print('Need at least 15 different views. Ctrl+C cancels.')
    rms, K, D, rvecs, tvecs = cv2.calibrateCamera(objects, pixels, size, None, None)
    per_view = [float(np.sqrt(np.mean(np.sum((cv2.projectPoints(obj, r, t, K, D)[0]-p)**2, axis=2))))
                for p, r, t in zip(pixels, rvecs, tvecs)]
    result = {'K': K.tolist(), 'D': D.flatten().tolist(), 'image_size': list(size),
              'rms_px': float(rms), 'per_view_rms_px': per_view, 'views': len(pixels),
              'pattern_inner_corners': [cols, rows], 'square_m': square,
              'arm_xy_mm': [c['arm_x_mm'], c['arm_y_mm']], 'source_folder': str(out)}
    save(out/'camera.json', result)
    if not np.isfinite(K).all() or rms > 1.5 or not 0 < K[0, 2] < size[0] or not 0 < K[1, 2] < size[1]:
        raise RuntimeError('Poor camera fit. Retake sharper, more varied views. Attempt saved.')
    save(home()/'calibration/camera.json', result)
    print(f'Camera RMS: {rms:.3f} px')


def collect(ep, vision, out, expected, writer):
    actual = float(input(f'Place reference at {expected:.2f} m. Enter actual tape distance in metres: '))
    if not np.isfinite(actual) or abs(actual-expected) > 0.025:
        raise ValueError('Place robot within 2.5 cm of the requested distance')
    samples, number, deadline = [], ep.frame[1] if ep.frame else -1, time.monotonic()+20
    while len(samples) < 30 and time.monotonic() < deadline:
        item = ep.frame
        if ep.fault:
            raise RuntimeError(ep.fault)
        if not item or item[1] == number:
            time.sleep(0.02)
            continue
        received, number, im = item
        if time.monotonic()-received > 0.3:
            continue
        z = vision.read(im)
        cv2.imshow('Distance calibration', draw(im, z, f'{expected:.2f}m: {len(samples)}/30'))
        if cv2.waitKey(10) & 255 == ord('q'):
            raise KeyboardInterrupt
        if z and abs(z['bearing_rad']) < np.radians(3):
            row = [actual, z['raw_x_m'], z['range_m'], z['bearing_rad'], z['rms_px']]
            samples.append(row)
            writer.writerow([received, *row])
            if len(samples) == 1:
                cv2.imwrite(str(out/f'distance_{expected:.2f}m.jpg'), draw(im, z))
        time.sleep(0.06)
    if len(samples) != 30:
        raise RuntimeError('Need 30 fresh, centred detections. Check ID, size, lighting, visibility.')
    a = np.asarray(samples)
    if np.std(a[:, 1], ddof=1) > 0.02:
        raise RuntimeError('Distance too unstable. Keep robot/tag stationary and improve lighting.')
    return {'actual_m': actual, 'raw_mean_m': float(a[:, 1].mean()),
            'estimated_mean_m': float(a[:, 2].mean()),
            'std_m': float(a[:, 2].std(ddof=1)), 'samples': 30}


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('mode', choices=['camera', 'distance', 'verify'])
    p.add_argument('--cols', type=int, default=9)
    p.add_argument('--rows', type=int, default=6)
    p.add_argument('--square', type=float, default=0.025, help='Checkerboard square side, metres')
    a = p.parse_args()
    c, out = settings(), folder(a.mode, 'calibration')
    save(out/'config.json', c)
    try:
        with EP(c) as ep:
            ep.arm()
            if a.mode == 'camera':
                camera(ep, c, out, a.cols, a.rows, a.square)
                return
            vision = Vision(c, distance=a.mode == 'verify')
            current = home()/'calibration/distance.json'
            if current.exists():
                import json
                old = json.loads(current.read_text())
                old['validated'] = False
                save(current, old)
            f, writer = csv_file(out/'samples.csv',
                ['monotonic_s', 'actual_m', 'raw_x_m', 'estimated_m', 'bearing_rad', 'rms_px'])
            with f:
                records = [collect(ep, vision, out, d, writer) for d in
                           ([0.20, 0.50, 1.00, 1.50] if a.mode == 'distance' else [0.20, 1.50])]
            save(out/'measurements.json', records)
            if a.mode == 'distance':
                raw = np.array([r['raw_mean_m'] for r in records])
                actual = np.array([r['actual_m'] for r in records])
                scale, offset = np.linalg.lstsq(np.c_[raw, np.ones(len(raw))], actual, rcond=None)[0]
                errors = scale*raw + offset - actual
                result = {'scale': float(scale), 'offset_m': float(offset),
                          'range_std_m': max(0.01, float(np.std(errors)),
                                            float(max(r['std_m'] for r in records)*abs(scale))),
                          'fit_errors_m': errors.tolist(), 'records': records,
                          'camera': vision.cal, 'geometry': vision.geometry(c),
                          'validated': False, 'source_folder': str(out)}
                save(out/'distance.json', result)
                if not 0.7 < scale < 1.3 or abs(offset) > 0.4 or max(abs(errors)) > 0.02:
                    raise RuntimeError('Poor distance fit. Check black-border size, pitch and reference point.')
            else:
                result = vision.mapping
                errors = [r['estimated_mean_m']-r['actual_m'] for r in records]
                result['validation'] = {'folder': str(out), 'records': records, 'errors_m': errors}
                result['validated'] = all(abs(e) <= c['distance_validation_tolerance_m'] for e in errors)
                result['range_std_m'] = max(result['range_std_m'], max(abs(e) for e in errors))
                save(out/'distance.json', result)
            save(home()/'calibration/distance.json', result)
            print('Distance calibration saved. Validated:', result['validated'])
            if a.mode == 'verify' and not result['validated']:
                raise RuntimeError('Validation failed. Review measurements and repeat distance calibration.')
    finally:
        cv2.destroyAllWindows()
        print('Saved:', out)


if __name__ == '__main__':
    main()
