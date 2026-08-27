from __future__ import annotations

from pathlib import Path

import imageio.v2 as imageio
import numpy as np


def write_mp4(frames: list[np.ndarray], output_path: Path, fps: int = 30) -> Path:
    if not frames:
        raise ValueError("No frames available for video export")

    output_path.parent.mkdir(parents=True, exist_ok=True)
    with imageio.get_writer(output_path, fps=max(1, int(fps)), codec="libx264", quality=8) as writer:
        for frame in frames:
            array = np.asarray(frame)
            if array.ndim == 3 and array.shape[2] == 4:
                array = array[:, :, :3]
            if array.dtype != np.uint8:
                array = np.clip(array, 0, 255).astype(np.uint8)
            writer.append_data(array)
    return output_path
