# Copyright (c) 2026, nav2_joy authors
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

"""Launch-file helpers for nav2_joy.

This module is local to the ``nav2_joy`` package on purpose: it ships
small re-implementations of a few helpers that older apt-installed
``nav2_common`` releases don't expose. Importing from
``nav2_common.launch`` would otherwise fail on those systems
(``ImportError: cannot import name 'LaunchConfigAsBool'``).
"""

import launch
from launch import LaunchContext
from launch.substitutions import LaunchConfiguration
from launch.utilities import perform_substitutions


class LaunchConfigAsBool(launch.Substitution):
    """Convert a LaunchConfiguration value into ``'True'`` or ``'False'``.

    Accepts (case-insensitive) ``true / 1 / yes / on`` and
    ``false / 0 / no / off`` so users can pass ``--foo yes`` on the
    command line, then use the result in a ``PythonExpression`` or
    ``IfCondition``.
    """

    def __init__(self, name: str) -> None:
        super().__init__()
        self._config = LaunchConfiguration(name)

    def perform(self, context: LaunchContext) -> str:
        value = perform_substitutions(context, [self._config])
        return 'True' if value.strip().lower() in ('true', '1', 'yes', 'on') else 'False'

    def describe(self) -> str:
        return f'LaunchConfigAsBool({self._config.describe()})'
