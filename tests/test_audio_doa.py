"""Direction of arrival in kinect_tools/audio_publisher.py on synthetic sound.

The publisher's math is tested on a simulated plane wave hitting the Kinect's
4-mic line array (Microsoft layout, metres along the bar)."""
import numpy as np
import pytest

pytest.importorskip("rclpy")
pytest.importorskip("sounddevice")
from kinect_tools.audio_publisher import AudioPublisher  # noqa: E402

FS, C = 16000, 343.0
MIC_X = np.array([-0.113, 0.036, 0.076, 0.113])


class _P:
    def __init__(self, v):
        self.value = v


def estimator(method="srp"):
    """An AudioPublisher with its DOA state but no ROS node or audio device."""
    ap = AudioPublisher.__new__(AudioPublisher)
    ap.sample_rate, ap.channels, ap.speed_of_sound = FS, 4, C
    ap.mic_spacing_m, ap._mic_x, ap._srp = 0.226, MIC_X, None
    params = {"doa_upsample": 16, "doa_method": method, "doa_gate": 2.0}
    ap.get_parameter = lambda name: _P(params[name])
    return ap


def plane_wave(theta_deg, n=800, seed=0):
    """Source at theta (positive = toward the channel-3 end): mic m hears it
    -x_m*sin(theta)/c earlier. Fractional delays via FFT phase shifts."""
    s = np.random.default_rng(seed).standard_normal(n * 4)
    S, f = np.fft.rfft(s), np.fft.rfftfreq(n * 4, 1 / FS)
    tau = -MIC_X * np.sin(np.radians(theta_deg)) / C
    x = np.stack([np.fft.irfft(S * np.exp(-2j * np.pi * f * t), n * 4) for t in tau], 1)
    return (x[n:2 * n] * 0.1).astype(np.float32)


@pytest.mark.parametrize("theta", [-60, -30, 0, 30, 60])
def test_srp_angle_magnitude(theta):
    assert abs(estimator()._srp_angle(plane_wave(theta))) == pytest.approx(abs(theta), abs=3)


@pytest.mark.parametrize("theta", [-60, -30, 0, 30, 60])
def test_gcc_angle_magnitude(theta):
    x = plane_wave(theta)
    ap = estimator("gcc")
    assert abs(ap._gcc_phat(x[:, 0].astype(float), x[:, 3].astype(float), 0.226)) \
        == pytest.approx(abs(theta), abs=4)


def test_sign_conventions_as_shipped():
    """Documents the shipped behaviour: srp reports a channel-3-side source as
    NEGATIVE, gcc as POSITIVE (see README, Known issues)."""
    x = plane_wave(40)
    assert estimator()._srp_angle(x) < 0
    assert estimator("gcc")._gcc_phat(x[:, 0].astype(float), x[:, 3].astype(float), 0.226) > 0


def test_srp_robust_to_noise():
    x = plane_wave(-35) + np.random.default_rng(9).standard_normal((800, 4)).astype(np.float32) * 0.02
    assert abs(estimator()._srp_angle(x)) == pytest.approx(35, abs=6)
