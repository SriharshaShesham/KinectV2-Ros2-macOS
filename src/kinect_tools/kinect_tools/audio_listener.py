#!/usr/bin/env python3
"""
Direct audio viewer — captures from Kinect USB mic array via sounddevice.
No ROS2 topics involved. Shows 4-channel waveform + beamformed mono.
"""
import numpy as np
import cv2
import sounddevice as sd
import sys


def find_kinect_mic(name='Xbox NUI Sensor'):
    """Find the Kinect mic array device by name."""
    devices = sd.query_devices()
    for i, d in enumerate(devices):
        if name.lower() in d['name'].lower() and d['max_input_channels'] >= 4:
            return i
    return None


def main():
    device_id = find_kinect_mic()
    if device_id is None:
        print("ERROR: Kinect mic array not found. Available input devices:")
        for i, d in enumerate(sd.query_devices()):
            if d['max_input_channels'] > 0:
                print(f"  [{i}] {d['name']}  (ch={d['max_input_channels']}, rate={d['default_samplerate']})")
        sys.exit(1)

    sample_rate = 16000
    channels = 4
    buffer_seconds = 2
    buffer_size = sample_rate * buffer_seconds
    chunk = sample_rate // 20  # 50ms chunks

    # Ring buffer: (buffer_size, channels)
    ring = np.zeros((buffer_size, channels), dtype=np.float32)

    # Canvas
    W, H = 900, 500
    ch_height = H // (channels + 1)  # 4 channels + 1 mono row

    colors = [
        (0, 255, 0),    # mic 0 — green
        (0, 200, 255),  # mic 1 — yellow
        (255, 100, 0),  # mic 2 — blue
        (0, 0, 255),    # mic 3 — red
    ]

    print(f"Capturing from device [{device_id}]: {sd.query_devices(device_id)['name']}")
    print(f"  {channels} channels @ {sample_rate} Hz, chunk={chunk} frames")
    print("Press 'q' in the window to quit.")

    try:
        with sd.InputStream(device=device_id, samplerate=sample_rate,
                            channels=channels, dtype='float32',
                            blocksize=chunk) as stream:
            while True:
                data, overflowed = stream.read(chunk)
                if overflowed:
                    pass  # skip warning, just keep going

                # Shift ring buffer and append
                ring = np.roll(ring, -chunk, axis=0)
                ring[-chunk:] = data

                # Draw
                canvas = np.zeros((H, W, 3), dtype=np.uint8)

                # Downsample for display
                step = max(1, buffer_size // W)
                display_data = ring[::step][:W]

                # Draw each channel
                for ch in range(channels):
                    y_offset = ch * ch_height + ch_height // 2
                    # Center line
                    cv2.line(canvas, (0, y_offset), (W, y_offset), (40, 40, 40), 1)
                    # Label
                    cv2.putText(canvas, f"Mic {ch}", (5, ch * ch_height + 15),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.4, colors[ch], 1)

                    samples = display_data[:, ch]
                    scale = ch_height // 2 * 0.9
                    y_vals = (y_offset - samples * scale).astype(np.int32)
                    y_vals = np.clip(y_vals, ch * ch_height + 2, (ch + 1) * ch_height - 2)

                    for i in range(1, len(y_vals)):
                        cv2.line(canvas, (i - 1, y_vals[i - 1]), (i, y_vals[i]), colors[ch], 1)

                # Beamformed mono (bottom row)
                mono = np.mean(display_data, axis=1)
                y_offset = channels * ch_height + ch_height // 2
                cv2.line(canvas, (0, y_offset), (W, y_offset), (40, 40, 40), 1)
                cv2.putText(canvas, "Beam", (5, channels * ch_height + 15),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 255), 1)
                scale = ch_height // 2 * 0.9
                y_vals = (y_offset - mono * scale).astype(np.int32)
                y_vals = np.clip(y_vals, channels * ch_height + 2, H - 2)
                for i in range(1, len(y_vals)):
                    cv2.line(canvas, (i - 1, y_vals[i - 1]), (i, y_vals[i]), (255, 255, 255), 1)

                # Peak meter
                peak = np.max(np.abs(data))
                bar_w = int(min(peak, 1.0) * (W - 20))
                color = (0, 255, 0) if peak < 0.5 else (0, 200, 255) if peak < 0.8 else (0, 0, 255)
                cv2.rectangle(canvas, (10, H - 15), (10 + bar_w, H - 5), color, -1)
                cv2.putText(canvas, f"Peak: {peak:.3f}", (W - 120, H - 5),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.35, (150, 150, 150), 1)

                cv2.imshow('Kinect v2 Mic Array', canvas)
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break

    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()


if __name__ == '__main__':
    main()
