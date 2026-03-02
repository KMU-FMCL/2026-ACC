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

"""Tests for FrameTransform forward transform (QLabs -> ROS)."""

import numpy as np

from qcar2_transforms.core.frame_transform import FrameTransform


def test_identity_transform_2d():
    """Identity transform leaves a 2D point unchanged."""
    ft = FrameTransform()
    result = ft.transform([3.0, 4.0])
    np.testing.assert_allclose(result, [3.0, 4.0], atol=1e-10)


def test_identity_transform_3d():
    """Identity transform leaves a 3D point unchanged."""
    ft = FrameTransform()
    result = ft.transform([3.0, 4.0, 5.0])
    np.testing.assert_allclose(result, [3.0, 4.0, 5.0], atol=1e-10)


def test_translation_only():
    """With rotation=0 and translation=(1,2), transform([0,0]) == [1.0, 2.0]."""
    ft = FrameTransform(rotation=0, translation=(1.0, 2.0))
    result = ft.transform([0.0, 0.0])
    np.testing.assert_allclose(result, [1.0, 2.0], atol=1e-10)


def test_rotation_90_degrees():
    """Verify rotation=90, no translation: transform([1,0]) == [0.0, 1.0]."""
    # R(-90°) = [[0, 1], [-1, 0]]; [1, 0] @ [[0, 1], [-1, 0]] = [0, 1]
    ft = FrameTransform(rotation=90)
    result = ft.transform([1.0, 0.0])
    np.testing.assert_allclose(result, [0.0, 1.0], atol=1e-10)


def test_rotation_and_translation():
    """Verify rotation=90, translation=(1,0): transform([0,0]) == [0.0, 1.0]."""
    # ([0,0] + [1,0]) @ R(-90°) = [1,0] @ [[0,1],[-1,0]] = [0, 1]
    ft = FrameTransform(rotation=90, translation=(1.0, 0.0))
    result = ft.transform([0.0, 0.0])
    np.testing.assert_allclose(result, [0.0, 1.0], atol=1e-10)


def test_3d_z_passthrough():
    """Z coordinate passes through unchanged when transforming a 3D point."""
    ft = FrameTransform(rotation=90)
    result = ft.transform([1.0, 0.0, 7.0])
    np.testing.assert_allclose(result, [0.0, 1.0, 7.0], atol=1e-10)
