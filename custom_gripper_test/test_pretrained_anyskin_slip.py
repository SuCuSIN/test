import struct
from unittest.mock import Mock

import pytest

from manual_gripper_shell import AnySkinSerialStream
from pretrained_anyskin_slip import PollenSlipModel, corrected_input


def test_binary_model_xyz_does_not_change_legacy_control():
    stream = AnySkinSerialStream('unused', 115200, 5, True)
    floats = [value for i in range(5) for value in (25 + i, 100 + i, 200 + i, 300 + i)]
    packet = struct.pack('<20f', *floats) + b'\r\n'
    def read(size):
        stream._stop.set()
        return packet
    stream._serial = Mock(in_waiting=len(packet), read=read)
    stream._read_binary_burst_loop()
    assert stream.get_data()[0][1:] == [v for i in range(5) for v in (25 + i, 100 + i, 200 + i)]
    assert stream.model_data()[0][1:] == [v for i in range(5) for v in (100 + i, 200 + i, 300 + i)]
    assert stream.model_data()[0][0] == stream.get_data()[0][0]


def test_corrected_input_rejects_duplicate_stale_and_missing_zero():
    sensor = dict(xyz=list(range(15)), baseline=[0] * 15, received_pc_monotonic_s=1.0)
    assert corrected_input(sensor, 1.1) == list(range(15))
    with pytest.raises(ValueError, match='stale'):
        corrected_input(sensor, 2)
    sensor['baseline'] = None
    with pytest.raises(ValueError, match='15 XYZ'):
        corrected_input(sensor, 1.1)
    sensor.update(baseline=[0] * 15, xyz=[1, 2, 3] * 5)
    with pytest.raises(ValueError, match='duplicate'):
        corrected_input(sensor, 1.1)


def test_public_reference_prediction():
    model = PollenSlipModel()
    reference = [-56.100006103515625, -136.62001037597656, 254.6807861328125,
                 -50.46002197265625, -125.1300048828125, 57.5960693359375,
                 268.739990234375, -122.07000732421875, 269.152587890625,
                 -175.98001098632812, 187.52999877929688, 93.02490234375,
                 -101.79000854492188, 158.73001098632812, 44.77001953125]
    # Upstream pred.py asserts close to one at 1e-3; README's checkpoint
    # example is not bit-identical to the pinned HF safetensors artifact.
    assert model.score(reference) == pytest.approx(1.0, abs=1e-3)
    with pytest.raises(ValueError, match='non-finite'):
        model.score([float('nan')] * 15)
