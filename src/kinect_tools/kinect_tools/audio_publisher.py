#!/usr/bin/env python3
"""
Kinect v2 Audio Publisher — captures real 4-channel audio from the Kinect mic array
via USB (sounddevice) and publishes:

  - kinect/audio/raw      (ByteMultiArray) — raw 4-channel int16 PCM
  - kinect/audio/mono      (ByteMultiArray) — beamformed mono channel
  - kinect/audio/doa       (Float32) — direction of arrival angle in degrees

DOA uses GCC-PHAT on mic pairs. Kinect v2 mic spacing is ~113mm linear array.
"""
import rclpy
from rclpy.node import Node
from std_msgs.msg import ByteMultiArray, MultiArrayDimension, Float32
import numpy as np
import sounddevice as sd
import os
import threading
import time


class AudioPublisher(Node):
    def __init__(self):
        super().__init__('audio_publisher')

        # Parameters
        self.declare_parameter('sample_rate', 16000)
        self.declare_parameter('channels', 4)
        self.declare_parameter('chunk_ms', 50)          # audio chunk duration in ms
        self.declare_parameter('device_name', 'Xbox NUI Sensor')
        # Outer pair (mic 0 to mic 3) is ~226 mm apart, NOT 113 mm: measured on speech
        # (2026-10-02) the 0-3 delay was a tight 5.5 samples — impossible at 113 mm
        # (max 5.3), so readings clipped to +-90 deg; the inner pairs add up to the
        # same total (1.19 + 1.12 + 3.00) in the Kinect's close-close-far layout.
        self.declare_parameter('mic_spacing_m', 0.226)

        self.sample_rate = self.get_parameter('sample_rate').value
        self.channels = self.get_parameter('channels').value
        self.chunk_ms = self.get_parameter('chunk_ms').value
        self.device_name = self.get_parameter('device_name').value
        self.mic_spacing_m = self.get_parameter('mic_spacing_m').value

        self.chunk_frames = self.sample_rate * self.chunk_ms // 1000  # samples per chunk

        # Publishers
        self.raw_pub = self.create_publisher(ByteMultiArray, 'kinect/audio/raw', 10)
        self.mono_pub = self.create_publisher(ByteMultiArray, 'kinect/audio/mono', 10)
        self.doa_pub = self.create_publisher(Float32, 'kinect/audio/doa', 10)

        # Find device
        self.device_id = self._find_device()
        if self.device_id is None:
            self.get_logger().error(
                f"Audio device '{self.device_name}' not found! Available devices:")
            for i, d in enumerate(sd.query_devices()):
                if d['max_input_channels'] > 0:
                    self.get_logger().error(f"  [{i}] {d['name']}")
            return

        self.get_logger().info(
            f"Audio publisher started: device='{self.device_name}' (id={self.device_id}), "
            f"rate={self.sample_rate}, channels={self.channels}, chunk={self.chunk_ms}ms"
        )

        # Speed of sound
        self.speed_of_sound = 343.0  # m/s

        # DOA gating: publish a direction only for chunks clearly above the
        # room's noise floor. Ungated, silent chunks (identical electrical
        # noise on all mics -> zero lag) reported "0 deg" ~2/3 of the time and
        # out-voted real speech in srotra's median, so it never turned
        # toward anyone (2026-09-30).
        self._noise_floor = None
        self.declare_parameter('doa_gate', 2.0)      # x noise floor (3.0 let almost no speech through)
        self.declare_parameter('doa_upsample', 16)   # sub-sample delay resolution (gcc method)
        # 'srp' (2026-10-22): SRP-PHAT over all 6 mic pairs on the float32
        # capture, 300-5000 Hz. Measured on noise bursts from one fixed spot:
        # srp agreed within 4 deg chunk to chunk; the old 'gcc' (mic 0-3 only,
        # on int16 audio that the quiet Kinect mics fill to only ~+-80 steps)
        # scattered -70..+82 deg and leaned to one side — the "always turns
        # left" in srotra's log. 'gcc' kept for rollback.
        self.declare_parameter('doa_method', 'srp')
        # Microsoft's published Kinect array layout (m, along the bar)
        self._mic_x = np.array([-0.113, 0.036, 0.076, 0.113])
        self._srp = None

        # Exit when orphaned: killing the kinect launch left this process
        # running with parent 1 — five stale copies held the mic for days and
        # wedged the live one (2026-09-30).
        self._parent = os.getppid()
        self._last_chunk = time.monotonic()
        self.create_timer(2.0, self._check_parent)

        # Start capture thread
        self.running = True
        self.capture_thread = threading.Thread(target=self._capture_loop, daemon=True)
        self.capture_thread.start()

    def _check_parent(self):
        if os.getppid() != self._parent:
            self.get_logger().warn('Parent launch is gone — exiting (no orphans).')
            os._exit(0)
        # A USB audio reset (the kinect driver does one at startup) can leave
        # stream.read() blocked forever — no exception to retry on. Exit and
        # let the launch file respawn this node with a fresh stream.
        if time.monotonic() - self._last_chunk > 6.0:
            self.get_logger().warn('No audio for 6 s — exiting for a clean respawn.')
            os._exit(3)

    def _find_device(self):
        """Find the audio device by name."""
        devices = sd.query_devices()
        for i, d in enumerate(devices):
            if self.device_name.lower() in d['name'].lower() and d['max_input_channels'] >= self.channels:
                return i
        return None

    def _capture_loop(self):
        """Capture forever, reopening the stream whenever it fails or goes dead.

        The kinect driver resets the sensor's USB audio as it starts — the
        stream this node opened at the same moment died, the thread exited,
        and DOA stayed silent until the next restart (2026-09-30). Now any
        error or 5 s of exact-zero audio reopens the stream.
        """
        while self.running and rclpy.ok():
            try:
                with sd.InputStream(
                    device=self.device_id,
                    samplerate=self.sample_rate,
                    channels=self.channels,
                    dtype='float32',
                    blocksize=self.chunk_frames,
                    latency='high',
                ) as stream:
                    self.get_logger().info("Audio stream opened (float32 capture).")
                    zero_chunks = 0
                    while self.running and rclpy.ok():
                        data_f32, overflowed = stream.read(self.chunk_frames)
                        self._last_chunk = time.monotonic()
                        if not data_f32.any():
                            zero_chunks += 1
                            if zero_chunks * self.chunk_ms >= 5000:
                                raise RuntimeError('5 s of exact-zero audio')
                            continue
                        zero_chunks = 0
                        data = np.clip(data_f32 * 32767.0, -32768, 32767).astype(np.int16)
                        self._publish_raw(data)
                        self._publish_mono(data)
                        self._publish_doa(data, data_f32)
            except Exception as e:
                self.get_logger().warn(f"Audio capture problem ({e}) — reopening in 2 s")
                time.sleep(2.0)
                self.device_id = self._find_device() or self.device_id

    def _publish_raw(self, data: np.ndarray):
        """Publish raw 4-channel audio as ByteMultiArray."""
        if self.raw_pub.get_subscription_count() == 0:
            return
        msg = ByteMultiArray()
        msg.layout.dim = [
            MultiArrayDimension(label='frames', size=data.shape[0], stride=data.shape[0] * data.shape[1] * 2),
            MultiArrayDimension(label='channels', size=data.shape[1], stride=data.shape[1] * 2),
        ]
        msg.data = data.tobytes()
        self.raw_pub.publish(msg)

    def _publish_mono(self, data: np.ndarray):
        """Publish delay-and-sum beamformed mono audio (focused forward)."""
        if self.mono_pub.get_subscription_count() == 0:
            return
        # Simple beamforming: average all 4 channels (delay-and-sum, broadside = forward)
        mono = np.mean(data.astype(np.float32), axis=1).astype(np.int16)
        msg = ByteMultiArray()
        msg.layout.dim = [
            MultiArrayDimension(label='frames', size=len(mono), stride=len(mono) * 2),
        ]
        msg.data = mono.tobytes()
        self.mono_pub.publish(msg)

    def _srp_tables(self, n_samples):
        import itertools
        n_fft = 1 << int(np.ceil(np.log2(n_samples)))
        f = np.fft.rfftfreq(n_fft, 1.0 / self.sample_rate)
        band = (f >= 300) & (f <= 5000)
        ang = np.arange(-90.0, 91.0, 2.0)
        pairs = list(itertools.combinations(range(min(4, self.channels)), 2))
        steer = {p: np.exp(2j * np.pi * f[band][None, :]
                           * ((self._mic_x[p[1]] - self._mic_x[p[0]])
                              * np.sin(np.radians(ang)) / self.speed_of_sound)[:, None])
                 for p in pairs}
        self._srp = (n_samples, n_fft, band, ang, pairs, steer, np.hanning(n_samples))

    def _srp_angle(self, x):
        """SRP-PHAT direction (deg) of one float32 chunk, in the SAME sign
        convention as the old gcc method (so srotra's doa_flip keeps meaning)."""
        if self._srp is None or self._srp[0] != len(x):
            self._srp_tables(len(x))
        _, n_fft, band, ang, pairs, steer, win = self._srp
        X = np.fft.rfft(x.astype(np.float64) * win[:, None], n_fft, axis=0)[band]
        P = np.zeros(len(ang))
        for i, j in pairs:
            cs = X[:, i] * np.conj(X[:, j])
            cs /= np.maximum(np.abs(cs), 1e-12)
            P += np.real((steer[(i, j)] * cs[None, :]).sum(1))
        return -float(ang[int(np.argmax(P))])

    def _publish_doa(self, data: np.ndarray, data_f32: np.ndarray = None):
        """Direction of arrival: SRP-PHAT on all pairs (default) or the old
        GCC-PHAT on the outer pair."""
        if self.doa_pub.get_subscription_count() == 0:
            return
        try:
            if self.get_parameter('doa_method').value == 'srp' and data_f32 is not None:
                rms = float(np.sqrt((data_f32[:, 0].astype(np.float64) ** 2).mean()))
                if self._noise_floor is None:
                    self._noise_floor = rms
                self._noise_floor = (min(rms, self._noise_floor * 1.002)
                                     if rms > 0 else self._noise_floor)
                if rms < self.get_parameter('doa_gate').value * max(self._noise_floor, 1e-7):
                    return
                msg = Float32()
                msg.data = self._srp_angle(data_f32)
                self.doa_pub.publish(msg)
                return
            # Use outermost pair (mic 0 and mic 3) for best resolution
            mic0 = data[:, 0].astype(np.float64)
            mic3 = data[:, min(3, data.shape[1] - 1)].astype(np.float64)

            rms = float(np.sqrt((mic0 ** 2).mean()))
            if self._noise_floor is None:
                self._noise_floor = rms
            # track the floor: falls fast, rises slowly (speech doesn't drag it up)
            self._noise_floor = min(rms, self._noise_floor * 1.002) if rms > 0 else self._noise_floor
            if rms < self.get_parameter('doa_gate').value * max(self._noise_floor, 1.0):
                return

            angle = self._gcc_phat(mic0, mic3, self.mic_spacing_m)
            if angle is not None:
                msg = Float32()
                msg.data = float(angle)
                self.doa_pub.publish(msg)
        except Exception:
            pass

    def _gcc_phat(self, sig1: np.ndarray, sig2: np.ndarray, mic_dist: float) -> float:
        """
        GCC-PHAT: estimate time delay between two signals, convert to angle.
        Returns angle in degrees (-90 to +90), where 0 = broadside (forward).
        """
        n = len(sig1) + len(sig2) - 1
        n_fft = 1
        while n_fft < n:
            n_fft <<= 1

        S1 = np.fft.rfft(sig1, n=n_fft)
        S2 = np.fft.rfft(sig2, n=n_fft)

        # Cross-power spectrum with phase transform
        R = S1 * np.conj(S2)
        magnitude = np.abs(R)
        magnitude[magnitude < 1e-10] = 1e-10
        R /= magnitude

        # Upsampled cross-correlation (zero-padded inverse FFT): the pair is
        # only ~5 samples apart at 16 kHz, so whole-sample lags could only say
        # 0/11/22/35/49/71/90 degrees.
        up = max(1, int(self.get_parameter('doa_upsample').value))
        cc = np.fft.irfft(R, n=n_fft * up)
        fs = self.sample_rate * up

        # Max delay in samples based on mic spacing
        max_delay_samples = int(mic_dist / self.speed_of_sound * fs) + 1
        max_delay_samples = min(max_delay_samples, (n_fft * up) // 2)

        # Search in valid range
        cc_shifted = np.concatenate([cc[-max_delay_samples:], cc[:max_delay_samples + 1]])
        peak_idx = np.argmax(np.abs(cc_shifted))
        delay_samples = peak_idx - max_delay_samples

        # Convert delay to angle
        delay_seconds = delay_samples / fs
        ratio = delay_seconds * self.speed_of_sound / mic_dist
        ratio = np.clip(ratio, -1.0, 1.0)
        angle_rad = np.arcsin(ratio)
        angle_deg = np.degrees(angle_rad)

        return angle_deg

    def destroy_node(self):
        self.running = False
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = AudioPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
