"""A vertical 36h11 tag, camera intrinsics, and a measured distance mapping."""
import json
import cv2
import numpy as np
from .common import home


def detector():
    dictionary = cv2.aruco.getPredefinedDictionary(
        cv2.aruco.DICT_APRILTAG_36h11
    )

    class Detector:
        def detectMarkers(self, image):
            return cv2.aruco.detectMarkers(image, dictionary)

    return Detector()


class Vision:
    def __init__(self, c, distance=True):
        self.c, self.detector = c, detector()
        self.cal = json.loads((home()/'calibration/camera.json').read_text())
        if self.cal['arm_xy_mm'] != [c['arm_x_mm'], c['arm_y_mm']]:
            raise ValueError('Arm changed. Recalibrate camera.')
        self.K, self.D = np.array(self.cal['K']), np.array(self.cal['D'])
        s = c['tag_size_m']/2
        self.points = np.array([[-s, s, 0], [s, s, 0], [s, -s, 0], [-s, -s, 0]])
        self.mapping = None
        if distance:
            self.mapping = json.loads((home()/'calibration/distance.json').read_text())
            if self.mapping['camera'] != self.cal or self.mapping['geometry'] != self.geometry(c):
                raise ValueError('Calibration/config changed. Recalibrate distance.')

    @staticmethod
    def geometry(c):
        keys = ['tag_id', 'tag_size_m', 'arm_x_mm', 'arm_y_mm', 'reference',
                'camera_pitch_deg', 'camera_yaw_deg', 'camera_left_m']
        return {k: c[k] for k in keys}

    def read(self, image):
        height, width = image.shape[:2]
        if [width, height] != self.cal['image_size']:
            raise ValueError('Image resolution changed. Recalibrate camera.')
        corners, ids, _ = self.detector.detectMarkers(image)
        if ids is None or self.c['tag_id'] not in ids.flatten():
            return None
        p = corners[list(ids.flatten()).index(self.c['tag_id'])].reshape(4, 2)
        if (p.min(axis=0) < 5).any() or (p.max(axis=0) > [width-5, height-5]).any():
            return None
        ok, rv, tv = cv2.solvePnP(self.points, p, self.K, self.D, flags=cv2.SOLVEPNP_IPPE_SQUARE)
        if not ok or not np.isfinite(tv).all() or tv[2, 0] <= 0:
            return None
        reproj, _ = cv2.projectPoints(self.points, rv, tv, self.K, self.D)
        rms = float(np.sqrt(np.mean(np.sum((reproj.reshape(4, 2)-p)**2, axis=1))))
        if rms > self.c['max_reprojection_px']:
            return None
        xc, yc, zc = tv.flatten()
        pitch, yaw = np.radians([self.c['camera_pitch_deg'], self.c['camera_yaw_deg']])
        forward, left = np.cos(pitch)*zc - np.sin(pitch)*yc, -xc
        raw = np.array([np.cos(yaw)*forward-np.sin(yaw)*left,
                        np.sin(yaw)*forward+np.cos(yaw)*left])
        xy = raw.copy()
        if self.mapping:
            xy *= self.mapping['scale']
            xy[0] += self.mapping['offset_m']
        xy[1] += self.c['camera_left_m']
        if xy[0] <= 0:
            return None
        return {'raw_x_m': float(raw[0]), 'x_m': float(xy[0]), 'y_m': float(xy[1]),
                'range_m': float(np.linalg.norm(xy)),
                'bearing_rad': float(np.arctan2(xy[1], xy[0])), 'rms_px': rms, 'corners': p}


def draw(image, observation, text=''):
    out = image.copy()
    if observation:
        cv2.polylines(out, [observation['corners'].astype(np.int32)], True, (0, 255, 0), 2)
        text += f"  d={observation['range_m']:.3f}m  phi={np.degrees(observation['bearing_rad']):.1f}deg"
    cv2.putText(out, text, (20, 35), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0, 220, 255), 2)
    return out
