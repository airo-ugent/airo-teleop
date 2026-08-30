"""Custom rotation utilities — a minimal scipy.spatial.transform.Rotation replacement."""

import numpy as np


class Rotation:
    """
    Minimal rotation class supporting quaternions, rotation vectors, and rotation matrices.
    Quaternion convention: [x, y, z, w] internally.
    """

    def __init__(self, quat: np.ndarray) -> None:
        self._quat = np.asarray(quat, dtype=float)
        norm = np.linalg.norm(self._quat)
        if norm > 0:
            self._quat = self._quat / norm

    # ── constructors ──────────────────────────────────────────────

    @classmethod
    def from_rotvec(cls, rotvec: np.ndarray) -> "Rotation":
        rotvec = np.asarray(rotvec, dtype=float)
        angle = np.linalg.norm(rotvec)
        if angle < 1e-8:
            quat = np.array([0.0, 0.0, 0.0, 1.0])
        else:
            axis = rotvec / angle
            half = angle / 2.0
            quat = np.array(
                [
                    axis[0] * np.sin(half),
                    axis[1] * np.sin(half),
                    axis[2] * np.sin(half),
                    np.cos(half),
                ]
            )
        return cls(quat)

    @classmethod
    def from_matrix(cls, matrix: np.ndarray) -> "Rotation":
        matrix = np.asarray(matrix, dtype=float)
        trace = np.trace(matrix)

        if trace > 0:
            s = np.sqrt(trace + 1.0) * 2
            qw, qx, qy, qz = (
                0.25 * s,
                (matrix[2, 1] - matrix[1, 2]) / s,
                (matrix[0, 2] - matrix[2, 0]) / s,
                (matrix[1, 0] - matrix[0, 1]) / s,
            )
        elif matrix[0, 0] > matrix[1, 1] and matrix[0, 0] > matrix[2, 2]:
            s = np.sqrt(1.0 + matrix[0, 0] - matrix[1, 1] - matrix[2, 2]) * 2
            qw, qx, qy, qz = (
                (matrix[2, 1] - matrix[1, 2]) / s,
                0.25 * s,
                (matrix[0, 1] + matrix[1, 0]) / s,
                (matrix[0, 2] + matrix[2, 0]) / s,
            )
        elif matrix[1, 1] > matrix[2, 2]:
            s = np.sqrt(1.0 + matrix[1, 1] - matrix[0, 0] - matrix[2, 2]) * 2
            qw, qx, qy, qz = (
                (matrix[0, 2] - matrix[2, 0]) / s,
                (matrix[0, 1] + matrix[1, 0]) / s,
                0.25 * s,
                (matrix[1, 2] + matrix[2, 1]) / s,
            )
        else:
            s = np.sqrt(1.0 + matrix[2, 2] - matrix[0, 0] - matrix[1, 1]) * 2
            qw, qx, qy, qz = (
                (matrix[1, 0] - matrix[0, 1]) / s,
                (matrix[0, 2] + matrix[2, 0]) / s,
                (matrix[1, 2] + matrix[2, 1]) / s,
                0.25 * s,
            )
        return cls(np.array([qx, qy, qz, qw]))

    @classmethod
    def from_quat(cls, quat: np.ndarray) -> "Rotation":
        return cls(quat)

    # ── conversions ───────────────────────────────────────────────

    def as_matrix(self) -> np.ndarray:
        qx, qy, qz, qw = self._quat
        return np.array(
            [
                [
                    1 - 2 * (qy * qy + qz * qz),
                    2 * (qx * qy - qz * qw),
                    2 * (qx * qz + qy * qw),
                ],
                [
                    2 * (qx * qy + qz * qw),
                    1 - 2 * (qx * qx + qz * qz),
                    2 * (qy * qz - qx * qw),
                ],
                [
                    2 * (qx * qz - qy * qw),
                    2 * (qy * qz + qx * qw),
                    1 - 2 * (qx * qx + qy * qy),
                ],
            ],
            dtype=float,
        )

    def as_rotvec(self) -> np.ndarray:
        qx, qy, qz, qw = self._quat
        if qw < 0:
            qx, qy, qz, qw = -qx, -qy, -qz, -qw
        angle = 2.0 * np.arccos(np.clip(abs(qw), 0.0, 1.0))
        sin_half = np.sqrt(1.0 - qw * qw)
        if sin_half < 1e-8:
            return 2.0 * np.array([qx, qy, qz])
        return angle * np.array([qx, qy, qz]) / sin_half

    def as_quat(self) -> np.ndarray:
        return self._quat.copy()

    # ── operations ────────────────────────────────────────────────

    def apply(self, vectors: np.ndarray, inverse: bool = False) -> np.ndarray:
        vectors = np.asarray(vectors, dtype=float)
        single = vectors.ndim == 1 and len(vectors) == 3
        if single:
            vectors = vectors.reshape(1, 3)
        mat = self.as_matrix().T if inverse else self.as_matrix()
        result = vectors @ mat.T
        return result.flatten() if single else result

    def inv(self) -> "Rotation":
        qx, qy, qz, qw = self._quat
        return Rotation(np.array([-qx, -qy, -qz, qw]))

    def __mul__(self, other: "Rotation") -> "Rotation":
        x1, y1, z1, w1 = other._quat
        x2, y2, z2, w2 = self._quat
        return Rotation(
            np.array(
                [
                    w2 * x1 + x2 * w1 + y2 * z1 - z2 * y1,
                    w2 * y1 - x2 * z1 + y2 * w1 + z2 * x1,
                    w2 * z1 + x2 * y1 - y2 * x1 + z2 * w1,
                    w2 * w1 - x2 * x1 - y2 * y1 - z2 * z1,
                ]
            )
        )
