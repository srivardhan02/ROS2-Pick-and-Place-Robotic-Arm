"""ROS-independent QR detection + tracking (testable without rclpy).

Key idea: *finding* the QR (4 corners) and *decoding* it are separate.
Corner detection is far more tolerant to blur than decoding, so geometry
is tracked every frame and the decoded value is latched. A short hold
timer keeps the box alive through dropped frames, which removes the
"detect -> Searching -> detect" flicker.
"""
import time
from dataclasses import dataclass
from typing import Optional

import cv2
import numpy as np

try:
    from pyzbar.pyzbar import decode as zbar_decode
except ImportError:  # pragma: no cover
    zbar_decode = None


@dataclass
class QRResult:
    status: str                 # 'DECODED' | 'TRACKING' | 'SEARCHING'
    value: Optional[str]        # latched decoded text (may be None)
    corners: Optional[np.ndarray]  # 4x2 float32, TL,TR,BR,BL (smoothed)
    center: Optional[tuple]     # (cx, cy) pixels
    size: Optional[float]       # mean side length, pixels
    angle: Optional[float]      # rotation of top edge, degrees
    frame_size: tuple           # (w, h)
    # --- the brown cardboard box the QR is stuck on (what the gripper grips)
    box_corners: Optional[np.ndarray] = None  # 4x2, TL,TR,BR,BL (smoothed)
    box_center: Optional[tuple] = None        # (cx, cy) px
    box_size: Optional[tuple] = None          # (side_a, side_b) px, a=TL-TR edge
    box_angle: Optional[float] = None         # deg, normalised to [-45, 45)

    @property
    def offset_norm(self):
        """Center offset from image center, each axis in [-1, 1]."""
        if self.center is None:
            return None
        w, h = self.frame_size
        return ((self.center[0] - w / 2) / (w / 2),
                (self.center[1] - h / 2) / (h / 2))


def _order_corners(pts):
    """Return corners as TL, TR, BR, BL."""
    pts = np.asarray(pts, dtype=np.float32).reshape(4, 2)
    s = pts.sum(axis=1)
    d = np.diff(pts, axis=1).ravel()
    return np.array([pts[np.argmin(s)], pts[np.argmin(d)],
                     pts[np.argmax(s)], pts[np.argmax(d)]], dtype=np.float32)


def _valid_quad(c, frame_size):
    w, h = frame_size
    if c is None or not np.all(np.isfinite(c)):
        return False
    area = cv2.contourArea(c)
    if area < 40 * 40 or area > 0.9 * w * h:
        return False
    if not cv2.isContourConvex(c.reshape(-1, 1, 2)):
        return False
    sides = [np.linalg.norm(c[i] - c[(i + 1) % 4]) for i in range(4)]
    return min(sides) / max(sides) > 0.4  # roughly square


def detect_box(frame_bgr, qr_corners, brown_lo=(5, 35, 60),
               brown_hi=(35, 255, 230), grow=1.75):
    """Find the brown cardboard box around a QR label.

    Brown pixels near the QR + the QR corners are wrapped in one convex hull
    (the label hides most of the box top, so brown alone is only a ring).
    Returns 4x2 corners (TL,TR,BR,BL) or None.
    """
    h, w = frame_bgr.shape[:2]
    c = qr_corners.mean(axis=0)
    side = np.mean([np.linalg.norm(qr_corners[i] - qr_corners[(i + 1) % 4])
                    for i in range(4)])
    r = 0.5 * side * grow
    x0, y0 = int(max(c[0] - r, 0)), int(max(c[1] - r, 0))
    x1, y1 = int(min(c[0] + r, w)), int(min(c[1] + r, h))
    if x1 - x0 < 20 or y1 - y0 < 20:
        return None
    hsv = cv2.cvtColor(frame_bgr[y0:y1, x0:x1], cv2.COLOR_BGR2HSV)
    m = cv2.inRange(hsv, brown_lo, brown_hi)
    m = cv2.morphologyEx(m, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))
    n, lab, st, _ = cv2.connectedComponentsWithStats(m)
    keep = np.zeros(n, bool)
    keep[1:] = st[1:, cv2.CC_STAT_AREA] >= 60       # drop specks
    ys, xs = np.nonzero(keep[lab])
    if len(xs) < 150:                                # not enough cardboard seen
        return qr_corners.copy()                     # fall back to the label
    pts = np.column_stack([xs + x0, ys + y0]).astype(np.float32)
    hull = cv2.convexHull(np.vstack([pts, qr_corners.astype(np.float32)]))
    box = cv2.boxPoints(cv2.minAreaRect(hull))
    return _order_corners(box)


class QRTracker:
    def __init__(self, hold_sec=1.0, smooth=0.5, decode_every=1):
        self.hold_sec = hold_sec          # keep box after last sighting
        self.smooth = smooth              # EMA weight of the new measurement
        self.decode_every = decode_every
        self.detector = cv2.QRCodeDetector()
        self.wechat = None
        try:
            self.wechat = cv2.wechat_qrcode_WeChatQRCode()
        except Exception:
            pass
        self.clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8))

        self.corners = None
        self.box = None
        self.value = None
        self.last_seen = 0.0
        self.last_decoded = 0.0
        self.n = 0

    # ---------------- decoding helpers ----------------
    def _zbar(self, img):
        if zbar_decode is None:
            return None, None
        for r in zbar_decode(img):
            if r.type == 'QRCODE' and r.data:
                poly = (np.array([[p.x, p.y] for p in r.polygon], np.float32)
                        if len(r.polygon) == 4 else None)
                return r.data.decode('utf-8', errors='replace'), poly
        return None, None

    def _decode_image(self, gray, fast=False):
        """Try several cheap-to-expensive variants. -> (value, corners|None)

        fast=True is used for blind full-frame scans (no candidate yet).
        """
        variants = [gray, self.clahe.apply(gray)]
        for v in variants:
            val, poly = self._zbar(v)
            if val:
                return val, poly
        if fast:
            return None, None
        blur = cv2.GaussianBlur(variants[1], (0, 0), 3)
        variants.append(cv2.addWeighted(variants[1], 1.8, blur, -0.8, 0))  # unsharp
        val, poly = self._zbar(variants[2])
        if val:
            return val, poly
        if self.wechat is not None:
            try:
                res, pts = self.wechat.detectAndDecode(gray)
                if res and res[0]:
                    return res[0], np.asarray(pts[0], np.float32)
            except cv2.error:
                pass
        for v in variants[:2]:
            try:
                val, pts, _ = self.detector.detectAndDecode(v)
                if val:
                    return val, (pts.reshape(4, 2).astype(np.float32)
                                 if pts is not None else None)
            except cv2.error:
                pass
        # upscale: helps small / dense codes
        big = cv2.resize(variants[2], None, fx=2, fy=2,
                         interpolation=cv2.INTER_CUBIC)
        val, poly = self._zbar(big)
        if val:
            return val, (poly / 2.0 if poly is not None else None)
        return None, None

    def _detect_corners(self, gray):
        try:
            ok, pts = self.detector.detect(gray)
        except cv2.error:
            return None
        if not ok or pts is None:
            return None
        return pts.reshape(4, 2).astype(np.float32)

    def _decode_rectified(self, gray, corners):
        """Warp the found quad to a flat square, add quiet zone, retry decode."""
        S = 300
        dst = np.array([[0, 0], [S, 0], [S, S], [0, S]], np.float32)
        M = cv2.getPerspectiveTransform(corners, dst)
        w = cv2.warpPerspective(gray, M, (S, S), flags=cv2.INTER_CUBIC)
        w = cv2.copyMakeBorder(w, 40, 40, 40, 40, cv2.BORDER_CONSTANT, value=255)
        val, _ = self._decode_image(w)
        return val

    # ---------------- main entry ----------------
    def update(self, frame) -> QRResult:
        now = time.monotonic()
        h, w = frame.shape[:2]
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY) if frame.ndim == 3 else frame
        self.n += 1

        meas, value = None, None

        # 1) geometry: every frame (cheap, blur tolerant)
        c = self._detect_corners(gray)
        if c is not None:
            c = _order_corners(c)
            if _valid_quad(c, (w, h)):
                meas = c

        # 2) decode: crop around the QR when we know where it is
        if self.n % self.decode_every == 0:
            ref = meas if meas is not None else self.corners
            if ref is not None and (meas is not None or
                                    now - self.last_seen < self.hold_sec):
                x0, y0 = np.floor(ref.min(axis=0) - 25).astype(int)
                x1, y1 = np.ceil(ref.max(axis=0) + 25).astype(int)
                x0, y0 = max(x0, 0), max(y0, 0)
                x1, y1 = min(x1, w), min(y1, h)
                roi = gray[y0:y1, x0:x1]
                value, poly = self._decode_image(roi)
                if value and poly is not None:
                    poly = _order_corners(poly + np.array([x0, y0], np.float32))
                    if meas is None and _valid_quad(poly, (w, h)):
                        meas = poly
                if not value and meas is not None:
                    value = self._decode_rectified(gray, meas)
            elif self.n % 3 == 0:   # nothing known yet: cheap blind scan
                value, poly = self._decode_image(gray, fast=True)
                if value and poly is not None:
                    poly = _order_corners(poly)
                    if meas is None and _valid_quad(poly, (w, h)):
                        meas = poly

        # 3) update state
        if meas is not None:
            if self.corners is None or now - self.last_seen > self.hold_sec:
                self.corners = meas
            else:
                a = self.smooth
                self.corners = a * meas + (1 - a) * self.corners
            self.last_seen = now
            bx = detect_box(frame, meas) if frame.ndim == 3 else None
            if bx is not None:
                self.box = bx if self.box is None else (
                    self.smooth * bx + (1 - self.smooth) * self.box)
        if value:
            self.value = value
            self.last_decoded = now

        # 4) status with hold (this is what kills the flicker)
        alive = self.corners is not None and (now - self.last_seen) < self.hold_sec
        if not alive:
            self.corners, self.value, self.box = None, None, None
            return QRResult('SEARCHING', None, None, None, None, None, (w, h))

        cs = self.corners
        center = tuple(cs.mean(axis=0))
        size = float(np.mean([np.linalg.norm(cs[i] - cs[(i + 1) % 4])
                              for i in range(4)]))
        angle = float(np.degrees(np.arctan2(cs[1][1] - cs[0][1],
                                            cs[1][0] - cs[0][0])))
        status = 'DECODED' if (self.value and
                               now - self.last_decoded < 3.0) else 'TRACKING'
        res = QRResult(status, self.value, cs, center, size, angle, (w, h))
        if self.box is not None:
            b = self.box
            res.box_corners = b
            res.box_center = tuple(b.mean(axis=0))
            res.box_size = (float(np.linalg.norm(b[1] - b[0])),
                            float(np.linalg.norm(b[2] - b[1])))
            a = float(np.degrees(np.arctan2(b[1][1] - b[0][1],
                                            b[1][0] - b[0][0])))
            res.box_angle = (a + 45.0) % 90.0 - 45.0   # grip angle, [-45,45)
        return res
