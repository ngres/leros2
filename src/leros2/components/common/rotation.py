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

"""Orientation representations for pose features.

ROS 2 always carries orientations as quaternions, but learned policies often
prefer a continuous representation. A :class:`RotationEncoding` owns both the
feature keys and the conversion to/from the quaternion on the wire, so a pose
component only has to pick one via its config and delegate.
"""

import math
from abc import ABC, abstractmethod
from collections.abc import Mapping, Sequence
from enum import Enum
from typing import Any, ClassVar

# Quaternion as ``(x, y, z, w)``, matching ``geometry_msgs/Quaternion``.
Quaternion = tuple[float, float, float, float]


class RotationRepresentation(str, Enum):
    """Orientation representation exposed by a pose component's features."""

    # ``<name>.quat.{x,y,z,w}``: the quaternion as published, no transform.
    QUATERNION = "quat"

    # ``<name>.rot6d.{r00,r10,r20,r01,r11,r21}``: the first two columns of the
    # rotation matrix, a continuous representation without the sign ambiguity
    # and wrap-around of quaternions/Euler angles.
    ROTATION_6D = "rot6d"

    @property
    def encoding(self) -> "RotationEncoding":
        """The encoding implementing this representation."""
        return _ENCODINGS[self]


class RotationEncoding(ABC):
    """Maps a quaternion onto the orientation features of a pose component.

    Subclasses define the key suffixes and the (invertible) transform; the
    key handling below is shared. Instances are stateless and reusable.
    """

    # Key segment between the component name and the suffix, e.g. ``quat`` in
    # ``pose.quat.x``.
    field: ClassVar[str]

    # Key suffixes, in the order returned by `from_quaternion`.
    suffixes: ClassVar[tuple[str, ...]]

    def keys(self, name: str) -> list[str]:
        """The feature keys of component ``name``, in encoded order."""
        return [f"{name}.{self.field}.{suffix}" for suffix in self.suffixes]

    def features(self, name: str) -> dict[str, type | tuple[type, ...]]:
        """The feature representation of component ``name``'s orientation."""
        return {key: float for key in self.keys(name)}

    def encode(self, name: str, quaternion: Quaternion) -> dict[str, float]:
        """Convert a quaternion into the features of component ``name``.

        Args:
            name: The pose component name used as the key prefix.
            quaternion: The orientation as ``(x, y, z, w)``.

        Returns:
            The orientation features.
        """
        return dict(zip(self.keys(name), self.from_quaternion(*quaternion)))

    def decode(self, name: str, features: Mapping[str, Any]) -> Quaternion:
        """Convert the features of component ``name`` back into a quaternion.

        Args:
            name: The pose component name used as the key prefix.
            features: The features to read the orientation from; extra keys are ignored.

        Returns:
            The orientation as ``(x, y, z, w)``.

        Raises:
            KeyError: If any orientation key is missing.
        """
        keys = self.keys(name)
        missing = [key for key in keys if key not in features]
        if missing:
            raise KeyError(f"Missing {self.field} keys for pose '{name}': {missing}")
        return self.to_quaternion([float(features[key]) for key in keys])

    @abstractmethod
    def from_quaternion(self, x: float, y: float, z: float, w: float) -> tuple[float, ...]:
        """Convert a quaternion into this representation's values (suffix order)."""
        raise NotImplementedError

    @abstractmethod
    def to_quaternion(self, values: Sequence[float]) -> Quaternion:
        """Convert this representation's values (suffix order) into a quaternion."""
        raise NotImplementedError


class QuaternionEncoding(RotationEncoding):
    """Passes the quaternion through unchanged."""

    field = "quat"
    suffixes = ("x", "y", "z", "w")

    def from_quaternion(self, x: float, y: float, z: float, w: float) -> tuple[float, ...]:
        return (x, y, z, w)

    def to_quaternion(self, values: Sequence[float]) -> Quaternion:
        x, y, z, w = values
        return (x, y, z, w)


class Rotation6DEncoding(RotationEncoding):
    """Represents the orientation as the first two columns of its rotation matrix.

    The third column is redundant (it is the cross product of the other two),
    so six values suffice; unlike quaternions this mapping is continuous, which
    makes it easier to regress. Decoding re-orthonormalizes the columns, since
    a policy's raw prediction is not guaranteed to be a valid rotation.
    """

    field = "rot6d"
    suffixes = ("r00", "r10", "r20", "r01", "r11", "r21")

    def from_quaternion(self, x: float, y: float, z: float, w: float) -> tuple[float, ...]:
        return (
            1 - 2 * (y * y + z * z),
            2 * (x * y + w * z),
            2 * (x * z - w * y),
            2 * (x * y - w * z),
            1 - 2 * (x * x + z * z),
            2 * (y * z + w * x),
        )

    def to_quaternion(self, values: Sequence[float]) -> Quaternion:
        r00, r10, r20, r01, r11, r21 = values
        (m00, m10, m20), (m01, m11, m21), (m02, m12, m22) = _orthonormalize(
            (r00, r10, r20), (r01, r11, r21)
        )

        trace = m00 + m11 + m22
        if trace > 0:
            s = math.sqrt(trace + 1.0) * 2  # s = 4 * w
            w = 0.25 * s
            x = (m21 - m12) / s
            y = (m02 - m20) / s
            z = (m10 - m01) / s
        elif m00 > m11 and m00 > m22:
            s = math.sqrt(1.0 + m00 - m11 - m22) * 2  # s = 4 * x
            w = (m21 - m12) / s
            x = 0.25 * s
            y = (m01 + m10) / s
            z = (m02 + m20) / s
        elif m11 > m22:
            s = math.sqrt(1.0 + m11 - m00 - m22) * 2  # s = 4 * y
            w = (m02 - m20) / s
            x = (m01 + m10) / s
            y = 0.25 * s
            z = (m12 + m21) / s
        else:
            s = math.sqrt(1.0 + m22 - m00 - m11) * 2  # s = 4 * z
            w = (m10 - m01) / s
            x = (m02 + m20) / s
            y = (m12 + m21) / s
            z = 0.25 * s

        return (x, y, z, w)


Vector3 = tuple[float, float, float]


def _orthonormalize(col0: Vector3, col1: Vector3) -> tuple[Vector3, Vector3, Vector3]:
    """Gram-Schmidt: turns two (possibly non-orthonormal) columns into a right-handed basis."""
    x0, y0, z0 = col0
    n0 = math.sqrt(x0 * x0 + y0 * y0 + z0 * z0)
    a0 = (x0 / n0, y0 / n0, z0 / n0)

    x1, y1, z1 = col1
    dot = x1 * a0[0] + y1 * a0[1] + z1 * a0[2]
    b1 = (x1 - dot * a0[0], y1 - dot * a0[1], z1 - dot * a0[2])
    n1 = math.sqrt(b1[0] * b1[0] + b1[1] * b1[1] + b1[2] * b1[2])
    a1 = (b1[0] / n1, b1[1] / n1, b1[2] / n1)

    a2 = (
        a0[1] * a1[2] - a0[2] * a1[1],
        a0[2] * a1[0] - a0[0] * a1[2],
        a0[0] * a1[1] - a0[1] * a1[0],
    )
    return a0, a1, a2


# Representation -> encoding. Add new representations by implementing a
# `RotationEncoding` and registering it here.
_ENCODINGS: dict[RotationRepresentation, RotationEncoding] = {
    RotationRepresentation.QUATERNION: QuaternionEncoding(),
    RotationRepresentation.ROTATION_6D: Rotation6DEncoding(),
}
