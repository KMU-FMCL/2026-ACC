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

"""Tests for FrameTransform translation vector operations."""

import numpy as np

from qcar2_transforms.core.frame_transform import FrameTransform


def test_default_translation_is_zero():
    """FrameTransform() with no args has a zero translation vector."""
    ft = FrameTransform()
    np.testing.assert_allclose(ft.translation, [0.0, 0.0], atol=1e-10)


def test_translation_property():
    """Translation property returns the value passed to the constructor."""
    ft = FrameTransform(translation=(3.0, 5.0))
    np.testing.assert_allclose(ft.translation, [3.0, 5.0], atol=1e-10)


def test_set_translation_updates_vector():
    """set_translation(1.0, 2.0) updates the translation to [1.0, 2.0]."""
    ft = FrameTransform()
    ft.set_translation(1.0, 2.0)
    np.testing.assert_allclose(ft.translation, [1.0, 2.0], atol=1e-10)
