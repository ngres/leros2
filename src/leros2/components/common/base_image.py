from abc import abstractmethod
from dataclasses import dataclass
from leros2.components.common.state import StateComponentConfig, StateComponent
from typing import TypeVar, Generic, Any


@dataclass
class ImageBaseComponentConfig(StateComponentConfig):
    """Configuration for the image component."""

    name: str
    width: int
    height: int


ConfigT = TypeVar("ConfigT", bound=ImageBaseComponentConfig)
MsgT = TypeVar("MsgT")


class ImageBaseComponent(StateComponent[ConfigT, MsgT], Generic[ConfigT, MsgT]):
    def __init__(self, config: ConfigT, msg_type: "type[MsgT]"):
        super().__init__(config, msg_type)

    @property
    def features(self) -> dict[str, tuple[int, int, int]]:
        return {self._config.name: (self._config.height, self._config.width, 3)}

    @abstractmethod
    def to_value(self, msg: MsgT) -> dict[str, Any]:
        raise NotImplementedError
