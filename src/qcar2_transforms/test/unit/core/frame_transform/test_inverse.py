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

"""Tests for FrameTransform inverse transform (ROS -> QLabs)."""

import numpy as np

from qcar2_transforms.core.frame_transform import FrameTransform


def test_inverse_identity():
    """Identity transform inverse leaves a 2D point unchanged."""
    ft = FrameTransform()
    result = ft.inverse_transform([3.0, 4.0])
    np.testing.assert_allclose(result, [3.0, 4.0], atol=1e-10)


def test_roundtrip_2d():
    """inverse_transform(transform(p)) recovers the original 2D point."""
    ft = FrameTransform(rotation=45, translation=(2.0, -3.0))
    for point in ([1.0, 0.0], [0.0, 1.0], [3.0, 4.0], [-1.5, 2.5]):
        transformed = ft.transform(point)
        recovered = ft.inverse_transform(transformed)
        np.testing.assert_allclose(recovered, point, atol=1e-10)


def test_roundtrip_3d():
    """inverse_transform(transform(p)) recovers the original 3D point."""
    ft = FrameTransform(rotation=45, translation=(2.0, -3.0))
    for point in ([1.0, 0.0, 5.0], [0.0, 1.0, -2.0], [3.0, 4.0, 0.0]):
        transformed = ft.transform(point)
        recovered = ft.inverse_transform(transformed)
        np.testing.assert_allclose(recovered, point, atol=1e-10)


def test_inverse_known_value():
    """Verify rotation=90, translation=(1,0): inverse_transform([0,1]) == [0,0]."""
    # R(-90°)^T = [[0, -1], [1, 0]]
    # [0, 1] @ [[0, -1], [1, 0]] - [1, 0] = [1, 0] - [1, 0] = [0, 0]
    ft = FrameTransform(rotation=90, translation=(1.0, 0.0))
    result = ft.inverse_transform([0.0, 1.0])
    np.testing.assert_allclose(result, [0.0, 0.0], atol=1e-10)
