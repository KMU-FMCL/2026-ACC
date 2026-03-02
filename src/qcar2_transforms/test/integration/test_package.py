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

"""Placeholder test to verify package scaffold."""

import qcar2_transforms


def test_version_exists():
    """Verify the package exposes a version string."""
    assert hasattr(qcar2_transforms, '__version__')
    assert isinstance(qcar2_transforms.__version__, str)
