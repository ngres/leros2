# Copyright 2026 Nicolas Gres
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     https://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Build a concrete component from its (draccus-parsed) config.

Each component config is a :class:`BaseComponentConfig` choice type; this module
maps every config class to the component class that consumes it, so a config-driven
robot/teleoperator can turn a list of parsed configs into live components.
"""

from .common.base import BaseComponent, BaseComponentConfig
from .joint_state import JointStateComponent, JointStateComponentConfig
from .joint_action import JointActionComponent, JointActionComponentConfig
from .pose_state import PoseStateComponent, PoseStateComponentConfig
from .pose_action import PoseActionComponent, PoseActionComponentConfig
from .wrench_state import WrenchStateComponent, WrenchStateComponentConfig
from .wrench_action import WrenchActionComponent, WrenchActionComponentConfig
from .parallel_gripper_action import (
    ParallelGripperActionComponent,
    ParallelGripperActionComponentConfig,
)
from .float_array_state import (
    FloatArrayStateComponent,
    FloatArrayStateComponentConfig,
)
from .float_array_action import (
    FloatArrayActionComponent,
    FloatArrayActionComponentConfig,
)
from .image import ImageComponent, ImageComponentConfig
from .compressed_image import (
    CompressedImageComponent,
    CompressedImageComponentConfig,
)


# Config class -> component class. Keys match the classes registered as
# ``BaseComponentConfig`` choice types (see each component module).
_COMPONENT_BY_CONFIG: dict[type[BaseComponentConfig], type[BaseComponent]] = {
    JointStateComponentConfig: JointStateComponent,
    JointActionComponentConfig: JointActionComponent,
    PoseStateComponentConfig: PoseStateComponent,
    PoseActionComponentConfig: PoseActionComponent,
    WrenchStateComponentConfig: WrenchStateComponent,
    WrenchActionComponentConfig: WrenchActionComponent,
    ParallelGripperActionComponentConfig: ParallelGripperActionComponent,
    FloatArrayStateComponentConfig: FloatArrayStateComponent,
    FloatArrayActionComponentConfig: FloatArrayActionComponent,
    ImageComponentConfig: ImageComponent,
    CompressedImageComponentConfig: CompressedImageComponent,
}


def make_component(config: BaseComponentConfig) -> BaseComponent:
    """Instantiate the component that consumes ``config``.

    Args:
        config: A parsed component config (a ``BaseComponentConfig`` choice type).

    Returns:
        The corresponding connected-capable component.

    Raises:
        ValueError: If no component is registered for the config's type.
    """
    try:
        component_cls = _COMPONENT_BY_CONFIG[type(config)]
    except KeyError:
        known = ", ".join(sorted(c.__name__ for c in _COMPONENT_BY_CONFIG))
        raise ValueError(
            f"No component registered for config type {type(config).__name__!r}. "
            f"Known config types: {known}."
        ) from None
    return component_cls(config)


def make_components(configs: list[BaseComponentConfig]) -> list[BaseComponent]:
    """Instantiate every component in ``configs`` (order preserved)."""
    return [make_component(config) for config in configs]
