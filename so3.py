#!/usr/bin/env python
#
# Copyright (c) CTU -- All Rights Reserved
# Created on: 2023-07-4
#     Author: Vladimir Petrik <vladimir.petrik@cvut.cz>
#

"""Module for representing 3D rotation."""

from __future__ import annotations
import numpy as np
from numpy.typing import ArrayLike


class SO3:
    """This class represents an SO3 rotations internally represented by rotation
    matrix."""

    def __init__(self, rotation_matrix: ArrayLike | None = None) -> None:
        """Creates a rotation transformation from rot_vector."""
        super().__init__()
        self.rot: np.ndarray = (
            np.asarray(rotation_matrix) if rotation_matrix is not None else np.eye(3)
        )

    @staticmethod
    def exp(rot_vector: ArrayLike) -> SO3:
        """Compute SO3 transformation from a given rotation vector, i.e. exponential
        representation of the rotation."""
        v = np.asarray(rot_vector)
        assert v.shape == (3,)
        theta = np.linalg.norm(v)
        v_new = v/theta
        wx = v_new[0]
        wy = v_new[1]
        wz = v_new[2]
        skew_s_m = np.array([
            [0, -wz, wy],
            [wz, 0, -wx],
            [-wy, wx, 0]
        ])
        rodrig = np.identity(3) + np.sin(theta) * skew_s_m + (1 - np.cos(theta)) * (skew_s_m @ skew_s_m)
        t = SO3(rodrig)
        # todo HW01: implement Rodrigues' formula, t.rot = ...
        return t

    def log(self) -> np.ndarray:
        """Compute rotation vector from this SO3"""
        # todo HW01: implement computation of rotation vector from this SO3
        v = np.zeros(3)
        theta = 0
        if self == SO3(np.identity(3)):
            pass
        elif self.rot.trace() == -1:
            theta = np.pi
            r11 = self.rot[0, 0]
            r22 = self.rot[1,1]
            r33 = self.rot[2,2]
            if r33 != -1:
                v = (1 / np.sqrt(2 * (1 + r33))) * np.array([self.rot[0,2], self.rot[1,2], 1 + r33])
            elif r22 != -1:
                v = (1 / np.sqrt(2 * (1 + r22))) * np.array([self.rot[0,1], 1 + r22, self.rot[2,1]])
            elif r11 != -1:
                v = (1 / np.sqrt(2 * (1 + r11))) * np.array([1 + r11, self.rot[1,0], self.rot[2,0]])
        else:
            theta = np.arccos((1/2) * (self.rot.trace() - 1))
            skew_m = (1 / (2 * np.sin(theta))) * (self.rot - self.inverse().rot)
            v = np.array([skew_m[2,1], skew_m[0,2], skew_m[1,0]])
        return v * theta

    def __mul__(self, other: SO3) -> SO3:
        """Compose two rotations, i.e., self * other"""
        # todo: HW01: implement composition of two rotation.
        rot_new = self.rot @ other.rot
        return SO3(rot_new)

    def inverse(self) -> SO3:
        """Return inverse of the transformation."""
        # todo: HW01: implement inverse, do not use np.linalg.inverse()
        res = self.rot.T.copy()
        return SO3(res)

    def act(self, vector: ArrayLike) -> np.ndarray:
        """Rotate given vector by this transformation."""
        v = np.asarray(vector)
        assert v.shape == (3,)
        return self.rot @ v

    def __eq__(self, other: SO3) -> bool:
        """Returns true if two transformations are almost equal."""
        return np.allclose(self.rot, other.rot)

    @staticmethod
    def rx(angle: float) -> SO3:
        """Return rotation matrix around x axis."""
        # todo: HW1opt: implement rx
        s = np.sin(angle) 
        c = np.cos(angle)
        res = SO3(np.array([
            [1, 0, 0],
            [0, c, -s],
            [0, s, c]
        ]))
        return res
        raise NotImplementedError("RX needs to be implemented.")

    @staticmethod
    def ry(angle: float) -> SO3:
        """Return rotation matrix around y axis."""
        # todo: HW1opt: implement ry
        s = np.sin(angle) 
        c = np.cos(angle)
        res = SO3(np.array([
            [c, 0, s],
            [0, 1, 0],
            [-s, 0, c]
        ]))
        return res
        raise NotImplementedError("RY needs to be implemented.")

    @staticmethod
    def rz(angle: float) -> SO3:
        """Return rotation matrix around z axis."""
        # todo: HW1opt: implement rz
        s = np.sin(angle) 
        c = np.cos(angle)
        res = SO3(np.array([
            [c, -s, 0],
            [s, c, 0],
            [0, 0, 1]
        ]))
        return res
        raise NotImplementedError("RZ needs to be implemented.")

    @staticmethod
    def from_quaternion(q: ArrayLike) -> SO3:
        """Compute rotation from quaternion in a form [qx, qy, qz, qw]."""
        # todo: HW1opt: implement from quaternion
        qxyz = q[0:3]
        qw = q[3]
        print(qxyz, qw, np.linalg.norm(qxyz))
        rot = SO3().exp(2 * np.arccos(qw) * (qxyz / np.linalg.norm(qxyz)))
        print(rot)
        return rot
        raise NotImplementedError("From quaternion needs to be implemented")

    def to_quaternion(self) -> np.ndarray:
        """Compute quaternion from self."""
        # todo: HW1opt: implement to quaternion
        qw = 1/2 * np.sqrt(1 + self.rot.trace())
        qx = 1/(4*qw) * (self.rot[2,1] - self.rot[1,2])
        qy = 1/(4*qw) * (self.rot[0,2] - self.rot[2,0])
        qz = 1/(4*qw) * (self.rot[1,0] - self.rot[0,1])
        return np.array([qx,qy,qz,qw])
        raise NotImplementedError("To quaternion needs to be implemented")

    @staticmethod
    def from_angle_axis(angle: float, axis: ArrayLike) -> SO3:
        """Compute rotation from angle axis representation."""
        # todo: HW1opt: implement from angle axis
        wx,wy,wz = axis
        skew_s_m = np.array([
            [0, -wz, wy],
            [wz, 0, -wx],
            [-wy, wx, 0]
        ])
        rodrig = np.identity(3) + np.sin(angle) * skew_s_m + (1 - np.cos(angle)) * (skew_s_m @ skew_s_m)
        return SO3(rodrig)
        raise NotImplementedError("Needs to be implemented")

    def to_angle_axis(self) -> tuple[float, np.ndarray]:
        """Compute angle axis representation from self."""
        # todo: HW1opt: implement to angle axis
        v = np.zeros(3)
        theta = 0
        if self == SO3(np.identity(3)):
            pass
        elif self.rot.trace() == -1:
            theta = np.pi
            r11 = self.rot[0, 0]
            r22 = self.rot[1,1]
            r33 = self.rot[2,2]
            if r33 != -1:
                v = (1 / np.sqrt(2 * (1 + r33))) * np.array([self.rot[0,2], self.rot[1,2], 1 + r33])
            elif r22 != -1:
                v = (1 / np.sqrt(2 * (1 + r22))) * np.array([self.rot[0,1], 1 + r22, self.rot[2,1]])
            elif r11 != -1:
                v = (1 / np.sqrt(2 * (1 + r11))) * np.array([1 + r11, self.rot[1,0], self.rot[2,0]])
        else:
            theta = np.arccos((1/2) * (self.rot.trace() - 1))
            skew_m = (1 / (2 * np.sin(theta))) * (self.rot - self.inverse().rot)
            v = np.array([skew_m[2,1], skew_m[0,2], skew_m[1,0]])
        return [theta, v]
        raise NotImplementedError("Needs to be implemented")

    @staticmethod
    def from_euler_angles(angles: ArrayLike, seq: list[str]) -> SO3:
        """Compute rotation from euler angles defined by a given sequence.
        angles: is a three-dimensional array of angles
        seq: is a list of axis around which angles rotate, e.g. 'xyz', 'xzx', etc.
        """
        len_seq = len(seq)
        rot = SO3(np.identity(3))
        for i in range(1, len_seq + 1):
            if seq[len_seq - i] == "x":
                ax_rot = SO3().rx(angles[len_seq - i])
            if seq[len_seq - i] == "y":
                ax_rot = SO3().ry(angles[len_seq - i])
            if seq[len_seq - i] == "z":
                ax_rot = SO3().rz(angles[len_seq - i])

            rot = ax_rot * rot
        # todo: HW1opt: implement from euler angles
        return rot
        raise NotImplementedError("Needs to be implemented")

    def __hash__(self):
        return id(self)


if __name__ == "__main__":
    t = SO3(np.array([
        [0, 1, 0],
        [0, 0, 1],
        [1, 0, 0]
    ]))

    print(t.rot)
    t_i = t.inverse()
    print(t_i.rot)