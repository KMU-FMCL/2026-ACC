# Copyright 2026 QCar2 Team
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Tests for FrameTransform rotation matrix operations."""

import numpy as np

from qcar2_transforms.core.frame_transform import FrameTransform


def test_default_rotation_is_identity():
    """FrameTransform() with no args has an identity rotation matrix."""
    ft = FrameTransform()
    np.testing.assert_allclose(ft._rotation_matrix, np.eye(2), atol=1e-10)


def test_rotation_matrix_at_90_degrees():
    """Rotation of 90 degrees yields R(-90°) = [[0, 1], [-1, 0]]."""
    ft = FrameTransform(rotation=90)
    expected = np.array([[0.0, 1.0], [-1.0, 0.0]])
    np.testing.assert_allclose(ft._rotation_matrix, expected, atol=1e-10)


def test_rotation_degrees_property():
    """rotation_degrees property returns the value passed to the constructor."""
    ft = FrameTransform(rotation=45.0)
    assert ft.rotation_degrees == 45.0


def test_set_rotation_updates_matrix():
    """set_rotation(90) updates the internal rotation matrix correctly."""
    ft = FrameTransform()
    ft.set_rotation(90)
    expected = np.array([[0.0, 1.0], [-1.0, 0.0]])
    np.testing.assert_allclose(ft._rotation_matrix, expected, atol=1e-10)


def test_set_rotation_updates_property():
    """rotation_degrees property reflects the new angle after set_rotation."""
    ft = FrameTransform()
    ft.set_rotation(90)
    assert ft.rotation_degrees == 90.0
