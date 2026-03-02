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

"""FrameTransform: 2D rigid-body transformation between QLabs and ROS frames."""

import numpy as np


class FrameTransform:
    """Encapsulates a 2D rigid-body transform between QLabs world and ROS map frame."""

    def __init__(self, rotation=0.0, translation=(0.0, 0.0)):
        """Initialize with rotation in degrees and translation (x, y)."""
        self._rotation_deg = float(rotation)
        self._translation = np.array(
            [float(translation[0]), float(translation[1])], dtype=float
        )
        self._rotation_matrix = self._make_rotation_matrix(self._rotation_deg)

    @staticmethod
    def _make_rotation_matrix(degrees):
        """Return 2x2 numpy rotation matrix for R(-degrees)."""
        theta = -np.deg2rad(degrees)
        c, s = np.cos(theta), np.sin(theta)
        return np.array([[c, -s], [s, c]], dtype=float)

    def set_rotation(self, degrees):
        """Recompute the internal rotation matrix from a new angle in degrees."""
        self._rotation_deg = float(degrees)
        self._rotation_matrix = self._make_rotation_matrix(self._rotation_deg)

    def set_translation(self, x, y):
        """Update the internal translation vector."""
        self._translation = np.array([float(x), float(y)], dtype=float)

    def transform(self, point):
        """Transform a 2D or 3D point from QLabs frame to ROS frame."""
        p = np.asarray(point, dtype=float)
        xy = (p[:2] + self._translation) @ self._rotation_matrix
        if p.shape[0] == 3:
            return np.array([xy[0], xy[1], p[2]])
        return xy

    def inverse_transform(self, point):
        """Apply inverse transform (ROS to QLabs) to a 2D or 3D point."""
        p = np.asarray(point, dtype=float)
        xy = p[:2] @ self._rotation_matrix.T - self._translation
        if p.shape[0] == 3:
            return np.array([xy[0], xy[1], p[2]])
        return xy

    @property
    def rotation_degrees(self):
        """Return rotation in degrees."""
        return self._rotation_deg

    @property
    def translation(self):
        """Return a copy of the translation vector as a numpy array."""
        return self._translation.copy()
