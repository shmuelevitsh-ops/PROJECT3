from __future__ import annotations

from pathlib import Path
from typing import Literal, Sequence
from uuid import uuid4

import numpy as np
import nbtlib
from nbtlib import Compound, Short, Byte, ByteArray


AxisOrder = Literal["yzx", "xyz", "zyx"]


def _u8_tag(value: int) -> Byte:
    """NBT Byte is signed, but ClassicWorld treats some bytes as unsigned."""
    if not 0 <= value <= 255:
        raise ValueError(f"Byte value out of range 0..255: {value}")
    return Byte.from_unsigned(value)


def _u16_tag(value: int) -> Short:
    """NBT Short is signed, but ClassicWorld dimensions are read as unsigned 16-bit."""
    if not 0 <= value <= 65535:
        raise ValueError(f"Short value out of range 0..65535: {value}")
    return Short.from_unsigned(value)


def _byte_array_from_u8(values: np.ndarray) -> ByteArray:
    """
    Store raw 0..255 bytes in an NBT ByteArray.

    NBT ByteArray is signed int8 in nbtlib, so we view uint8 bytes as int8
    without changing the underlying byte values.
    """
    values = np.ascontiguousarray(values, dtype=np.uint8)
    return ByteArray(values.view(np.int8))


def _to_yzx(blocks: np.ndarray, axis_order: AxisOrder) -> np.ndarray:
    """
    Convert input array to shape (Y, Z, X).

    ClassiCube packs block index as:
        index = ((y * Z) + z) * X + x

    Therefore C-order flattening of a (Y, Z, X) array is correct.
    """
    if blocks.ndim != 3:
        raise ValueError(f"Expected a 3D array, got shape {blocks.shape}")

    if axis_order == "yzx":
        return blocks
    if axis_order == "xyz":
        return np.transpose(blocks, (1, 2, 0))  # X,Y,Z -> Y,Z,X
    if axis_order == "zyx":
        return np.transpose(blocks, (1, 0, 2))  # Z,Y,X -> Y,Z,X

    raise ValueError(f"Unsupported axis_order: {axis_order!r}")


def numpy_to_cw(
    blocks: np.ndarray,
    out_path: str | Path,
    *,
    axis_order: AxisOrder = "yzx",
    spawn: Sequence[int] | None = None,
    yaw: int = 0,
    pitch: int = 0,
    uuid_bytes: bytes | None = None,
) -> None:
    arr = np.asarray(blocks)

    if not np.issubdtype(arr.dtype, np.integer):
        raise TypeError(f"blocks must contain integer block IDs, got {arr.dtype}")

    arr = _to_yzx(arr, axis_order)
    y_size, z_size, x_size = map(int, arr.shape)

    if arr.size == 0:
        raise ValueError("blocks array must not be empty")

    min_id = int(arr.min())
    max_id = int(arr.max())
    if min_id < 0 or max_id > 65535:
        raise ValueError(f"Block IDs must be in 0..65535, got {min_id}..{max_id}")

    block_ids = np.ascontiguousarray(arr, dtype=np.uint16)

    low = (block_ids & 0xFF).astype(np.uint8).ravel(order="C")
    high = ((block_ids >> 8) & 0xFF).astype(np.uint8).ravel(order="C")

    if spawn is None:
        spawn_x = x_size // 2
        spawn_y = min(y_size - 1, max(0, y_size // 2))
        spawn_z = z_size // 2
    else:
        if len(spawn) != 3:
            raise ValueError("spawn must be a sequence of three integers: (x, y, z)")
        spawn_x, spawn_y, spawn_z = map(int, spawn)

    if uuid_bytes is None:
        uuid_bytes = uuid4().bytes
    if len(uuid_bytes) != 16:
        raise ValueError("uuid_bytes must be exactly 16 bytes")

    root = Compound({
        "UUID": _byte_array_from_u8(np.frombuffer(uuid_bytes, dtype=np.uint8)),
        "X": _u16_tag(x_size),
        "Y": _u16_tag(y_size),
        "Z": _u16_tag(z_size),
        "Spawn": Compound({
            "X": _u16_tag(spawn_x),
            "Y": _u16_tag(spawn_y),
            "Z": _u16_tag(spawn_z),
            "H": _u8_tag(yaw),
            "P": _u8_tag(pitch),
        }),
        "BlockArray": _byte_array_from_u8(low),
    })

    if max_id > 255:
        root["BlockArray2"] = _byte_array_from_u8(high)

    nbt_file = nbtlib.File(root, root_name="ClassicWorld")
    nbt_file.save(str(out_path), gzipped=True, byteorder="big")
