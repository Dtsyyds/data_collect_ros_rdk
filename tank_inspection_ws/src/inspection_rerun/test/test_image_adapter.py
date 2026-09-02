from types import SimpleNamespace

import numpy as np
import pytest

from inspection_rerun.image_adapter import image_to_numpy, is_depth_encoding


def message(height, width, step, encoding, data, bigendian=False):
    return SimpleNamespace(height=height, width=width, step=step, encoding=encoding,
                           data=data, is_bigendian=bigendian)


def test_mono8_stays_two_dimensional():
    result = image_to_numpy(message(2, 3, 3, "mono8", bytes(range(6))))
    assert result.shape == (2, 3)
    assert result.dtype == np.uint8
    assert result.tolist() == [[0, 1, 2], [3, 4, 5]]


def test_row_padding_is_removed():
    result = image_to_numpy(message(2, 2, 4, "mono8", bytes([1, 2, 99, 99, 3, 4, 88, 88])))
    assert result.tolist() == [[1, 2], [3, 4]]


def test_bgr_and_bgra_are_converted_to_rgb_order():
    assert image_to_numpy(message(1, 1, 3, "bgr8", bytes([10, 20, 30]))).tolist() == [[[30, 20, 10]]]
    assert image_to_numpy(message(1, 1, 4, "bgra8", bytes([10, 20, 30, 40]))).tolist() == [[[30, 20, 10, 40]]]


def test_mono16_big_endian():
    result = image_to_numpy(message(1, 2, 4, "mono16", bytes([0x12, 0x34, 0xAB, 0xCD]), True))
    assert result.tolist() == [[0x1234, 0xABCD]]


def test_ros_16uc1_is_unsigned_depth():
    result = image_to_numpy(message(1, 2, 4, "16UC1", bytes([1, 0, 2, 0])))
    assert result.dtype == np.uint16
    assert result.tolist() == [[1, 2]]
    assert is_depth_encoding("16UC1")
    assert not is_depth_encoding("mono16")


def test_bad_length_is_rejected():
    with pytest.raises(ValueError, match="data length"):
        image_to_numpy(message(2, 2, 2, "mono8", bytes([1, 2, 3])))


def test_large_4096_by_3072_mono8_shape():
    result = image_to_numpy(message(3072, 4096, 4096, "mono8", bytes(4096 * 3072)))
    assert result.shape == (3072, 4096)
