"""Generated textures for the simulated field.

Each `make_*` function is pure: parameters in, a BGR `uint8` image out (BGR so
that `cv2.imwrite` and the rest of the vision code agree on channel order).
Writing to disk is the separate `write_png`.

Why generated rather than committed: the textures must track the parameters
(QR payloads, zone colours, banner green jitter per seed) and the repo should
not carry binary assets that can drift from the world they describe.
"""

from __future__ import annotations

from pathlib import Path

import cv2
import numpy as np
import numpy.typing as npt
import qrcode

from aero.core.errors import ConfigError
from aero.sim.world_params import Rgb

# All generators return 8-bit BGR images, the convention OpenCV writes and the
# vision code reads.
BgrImage = npt.NDArray[np.uint8]

# Low-resolution lattice used to give the ground texture large-scale blotches
# before fine per-pixel noise is added.
_BLOTCH_LATTICE_PX = 24
_BLOTCH_WEIGHT = 0.6
# Banner text layout, as fractions of the texture height.
_BANNER_TEXT_HEIGHT_FRACTION = 0.42
_BANNER_BORDER_FRACTION = 0.06


def rgb_to_bgr_255(rgb: Rgb) -> tuple[int, int, int]:
    """Convert a 0..1 RGB triple to an OpenCV 0..255 BGR tuple.

    Args:
        rgb: Red, green, blue in [0, 1].

    Returns:
        (blue, green, red) in [0, 255].
    """
    red, green, blue = (round(channel * 255) for channel in rgb)
    return (blue, green, red)


def make_qr_png(payload: str, size_px: int, quiet_zone_modules: int) -> BgrImage:
    """Render a QR code as a square BGR image.

    The module grid is rendered at one pixel per module and then upscaled with
    nearest-neighbour interpolation, so the result has hard edges and no
    anti-aliasing blur that would hurt decoding at altitude.

    Args:
        payload: String to encode.
        size_px: Side length of the output image in pixels.
        quiet_zone_modules: White border in modules (>= 4 per the QR spec).

    Returns:
        BGR image of shape (size_px, size_px, 3).

    Raises:
        ConfigError: The payload is empty or the parameters are unusable.
    """
    if not payload:
        raise ConfigError("QR payload must not be empty")
    if quiet_zone_modules < 1:
        raise ConfigError(f"quiet zone must be >= 1 module, got {quiet_zone_modules}")
    encoder = qrcode.QRCode(
        version=None,
        error_correction=qrcode.constants.ERROR_CORRECT_M,
        box_size=1,
        border=quiet_zone_modules,
    )
    encoder.add_data(payload)
    encoder.make(fit=True)
    matrix = np.array(encoder.get_matrix(), dtype=bool)
    if size_px < matrix.shape[0]:
        raise ConfigError(f"size_px {size_px} is smaller than the {matrix.shape[0]}-module QR grid")
    modules = np.where(matrix, 0, 255).astype(np.uint8)
    upscaled = cv2.resize(modules, (size_px, size_px), interpolation=cv2.INTER_NEAREST)
    return np.asarray(cv2.cvtColor(upscaled, cv2.COLOR_GRAY2BGR), dtype=np.uint8)


def make_ground_png(
    width_px: int,
    height_px: int,
    base_rgb: Rgb,
    noise_strength: float,
    seed: int,
) -> BgrImage:
    """Render a grass-like noisy ground texture.

    Large blotches plus fine grain, so a naive "is it green" detector cannot
    separate grass from the green banner.

    Args:
        width_px: Output width.
        height_px: Output height.
        base_rgb: Mean colour in [0, 1] per channel.
        noise_strength: Peak relative deviation, 0 for a flat colour.
        seed: Noise seed; the same seed always yields the same texture.

    Returns:
        BGR image of shape (height_px, width_px, 3).

    Raises:
        ConfigError: A dimension is not positive.
    """
    if width_px < 1 or height_px < 1:
        raise ConfigError(f"ground texture needs positive dimensions, got {width_px}x{height_px}")
    rng = np.random.default_rng(seed)
    lattice = rng.random((_BLOTCH_LATTICE_PX, _BLOTCH_LATTICE_PX), dtype=np.float32)
    blotches = np.asarray(
        cv2.resize(lattice, (width_px, height_px), interpolation=cv2.INTER_CUBIC),
        dtype=np.float32,
    )
    grain = rng.random((height_px, width_px), dtype=np.float32)
    # Centre both fields on zero so the mean colour stays `base_rgb`.
    variation = _BLOTCH_WEIGHT * (blotches - 0.5) + (1.0 - _BLOTCH_WEIGHT) * (grain - 0.5)
    scale = 1.0 + 2.0 * noise_strength * variation
    blue, green, red = rgb_to_bgr_255(base_rgb)
    image = np.empty((height_px, width_px, 3), dtype=np.float32)
    for index, channel_value in enumerate((blue, green, red)):
        image[:, :, index] = channel_value * scale
    return np.asarray(np.clip(image, 0, 255), dtype=np.uint8)


def make_banner_png(text: str, width_px: int, height_px: int, rgb: Rgb) -> BgrImage:
    """Render the green banner board with its caption.

    Args:
        text: Caption, e.g. "AeroTHON 2026".
        width_px: Output width.
        height_px: Output height.
        rgb: Background green in [0, 1] per channel (jittered per seed).

    Returns:
        BGR image of shape (height_px, width_px, 3).

    Raises:
        ConfigError: A dimension is not positive or the text is empty.
    """
    if width_px < 1 or height_px < 1:
        raise ConfigError(f"banner needs positive dimensions, got {width_px}x{height_px}")
    if not text:
        raise ConfigError("banner text must not be empty")
    image = np.empty((height_px, width_px, 3), dtype=np.uint8)
    image[:, :] = rgb_to_bgr_255(rgb)
    border_px = max(1, round(height_px * _BANNER_BORDER_FRACTION))
    cv2.rectangle(
        image,
        (border_px, border_px),
        (width_px - border_px - 1, height_px - border_px - 1),
        (255, 255, 255),
        thickness=max(1, border_px // 2),
    )
    font = cv2.FONT_HERSHEY_SIMPLEX
    target_text_height_px = height_px * _BANNER_TEXT_HEIGHT_FRACTION
    # Measure at scale 1 and rescale, so the caption fills the board predictably.
    (base_width, base_height), _ = cv2.getTextSize(text, font, 1.0, 2)
    scale_from_height = target_text_height_px / base_height
    usable_width_px = width_px - 4 * border_px
    scale = min(scale_from_height, usable_width_px / base_width)
    thickness = max(1, round(scale * 2))
    (text_width, text_height), baseline = cv2.getTextSize(text, font, scale, thickness)
    origin = ((width_px - text_width) // 2, (height_px + text_height - baseline) // 2)
    cv2.putText(image, text, origin, font, scale, (255, 255, 255), thickness, cv2.LINE_AA)
    return image


def write_png(path: Path, image: BgrImage) -> None:
    """Write a BGR image to `path`, creating parent directories.

    Args:
        path: Destination `.png` file.
        image: BGR image.

    Raises:
        ConfigError: OpenCV refused to write the file.
    """
    path.parent.mkdir(parents=True, exist_ok=True)
    if not cv2.imwrite(str(path), image):
        raise ConfigError(f"could not write texture {path}")
