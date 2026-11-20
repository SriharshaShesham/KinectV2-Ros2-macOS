"""Pure conversions from Kinect ROS messages to browser-friendly data.
No ROS imports here, so this is unit-testable anywhere."""

from typing import List, Optional

import numpy as np

NEAR_MM, FAR_MM = 500.0, 4500.0   # Kinect v2 usable depth range


def _jpeg(img, quality: int = 80) -> Optional[bytes]:
    import cv2
    ok, buf = cv2.imencode(".jpg", img, [cv2.IMWRITE_JPEG_QUALITY, quality])
    return buf.tobytes() if ok else None


def image_u16(msg) -> np.ndarray:
    """mono16 sensor_msgs/Image -> (h, w) uint16 (honours row padding)."""
    raw = np.frombuffer(bytes(msg.data), dtype="<u2")
    stride = msg.step // 2 if msg.step else msg.width
    return raw.reshape(msg.height, stride)[:, : msg.width].astype(np.uint16)


def passthrough_jpeg(msg) -> Optional[bytes]:
    """The driver already JPEG-encodes color: pass it through, no re-encode."""
    return bytes(msg.data)


def ir_u16_to_u8(ir: np.ndarray) -> np.ndarray:
    """IR has a huge dynamic range: sqrt squashes it, the 99th percentile
    maps to white so a far, dark scene still fills the range."""
    v = np.sqrt(ir.astype(np.float32))
    scale = 255.0 / max(float(np.percentile(v, 99)), 1.0)
    return (v * scale).clip(0, 255).astype(np.uint8)


def depth_u16_to_bgr(depth: np.ndarray) -> np.ndarray:
    """mm -> turbo colormap, near = warm, far = cool, no reading = black."""
    import cv2
    valid = depth > 0
    d = np.clip(depth, NEAR_MM, FAR_MM).astype(np.float32)
    u8 = (255 * (1.0 - (d - NEAR_MM) / (FAR_MM - NEAR_MM))).astype(np.uint8)
    color = cv2.applyColorMap(u8, cv2.COLORMAP_TURBO)
    color[~valid] = 0
    return color


def ir_to_jpeg(msg) -> Optional[bytes]:
    try:
        return _jpeg(ir_u16_to_u8(image_u16(msg)))
    except Exception:
        return None


def depth_to_jpeg(msg) -> Optional[bytes]:
    try:
        return _jpeg(depth_u16_to_bgr(image_u16(msg)))
    except Exception:
        return None


def bgr_to_jpeg(msg) -> Optional[bytes]:
    try:
        ch = 4 if msg.encoding in ("bgra8", "rgba8") else 3
        raw = np.frombuffer(bytes(msg.data), dtype=np.uint8)
        img = raw.reshape(msg.height, msg.step)[:, : msg.width * ch].reshape(msg.height, msg.width, ch)
        if msg.encoding.startswith("rgb"):
            img = img[..., [2, 1, 0]]
        return _jpeg(np.ascontiguousarray(img[..., :3]))
    except Exception:
        return None


def depth_at(depth: np.ndarray, u: int, v: int, radius: int = 3) -> Optional[float]:
    """Median of the valid readings in a small window: single time-of-flight
    pixels drop out at edges. None when nothing valid is near."""
    h, w = depth.shape
    win = depth[max(0, v - radius): min(h, v + radius + 1), max(0, u - radius): min(w, u + radius + 1)]
    good = win[win > 0]
    return float(np.median(good)) if good.size else None


def channel_levels(data, channels: int) -> List[float]:
    """Interleaved int16 PCM (bytes or list of 1-byte bytes, as rclpy gives
    ByteMultiArray.data) -> per-channel RMS in 0..1."""
    buf = b"".join(data) if isinstance(data, (list, tuple)) else bytes(data)
    pcm = np.frombuffer(buf, dtype="<i2")
    pcm = pcm[: (pcm.size // channels) * channels].reshape(-1, channels).astype(np.float32) / 32768.0
    rms = np.sqrt((pcm * pcm).mean(axis=0)) if pcm.size else np.zeros(channels)
    return [round(float(x), 5) for x in rms]
