"""Shared trinary occupancy maps and metric-preserving grid resampling."""
from pathlib import Path

import numpy as np
from PIL import Image
import yaml


DEFAULT_FREE_THRESH = 0.196
DEFAULT_OCCUPIED_THRESH = 0.65


def validate_resolution(resolution):
    resolution = float(resolution)
    if not np.isfinite(resolution) or resolution <= 0:
        raise ValueError('resolution must be finite and greater than zero')
    return resolution


def validate_thresholds(free_thresh, occupied_thresh):
    free_thresh, occupied_thresh = float(free_thresh), float(occupied_thresh)
    if not (np.isfinite(free_thresh) and np.isfinite(occupied_thresh) and
            0 < free_thresh < occupied_thresh < 1):
        raise ValueError('thresholds must satisfy 0 < free_thresh < occupied_thresh < 1')
    return free_thresh, occupied_thresh


def _snap_integer(values):
    """Ignore floating-point roundoff at coincident cell boundaries."""
    nearest = np.rint(values)
    return np.where(np.abs(values-nearest) < 1.e-10, nearest, values)


def resample_occupancy(values, source_resolution, resolution):
    """Cover the same metric extent, with occupied > unknown > free on overlap.

    The origin is unchanged. Output dimensions round up; cells extending beyond
    observed source coverage remain unknown unless they overlap an obstacle.
    """
    source_resolution = validate_resolution(source_resolution)
    resolution = validate_resolution(resolution)
    values = np.asarray(values)
    if values.ndim != 2 or not values.size:
        raise ValueError('occupancy grid must be a nonempty 2D array')
    if source_resolution == resolution:
        return values.copy()

    h, w = values.shape
    out_w, out_h = np.ceil(_snap_integer(
        np.array([w, h])*source_resolution/resolution)).astype(int)

    def bounds(size, count):
        edges = _snap_integer(np.arange(count+1)*resolution/source_resolution)
        lower = np.clip(np.floor(edges[:-1]).astype(int), 0, size)
        upper = np.clip(np.ceil(edges[1:]).astype(int), 0, size)
        return lower, upper, edges[1:] > size

    x0, x1, outside_x = bounds(w, out_w)
    y0, y1, outside_y = bounds(h, out_h)

    def any_overlap(mask):
        summed = np.pad(mask.astype(np.int64), ((1, 0), (1, 0)))
        summed = summed.cumsum(axis=0).cumsum(axis=1)
        count = (summed[y1[:, None], x1] - summed[y0[:, None], x1] -
                 summed[y1[:, None], x0] + summed[y0[:, None], x0])
        return count > 0

    unknown = any_overlap(values < 0) | outside_y[:, None] | outside_x
    occupied = any_overlap(values > 50)
    return np.where(occupied, 100, np.where(unknown, -1, 0)).astype(np.int8)


def _gray_levels(free_thresh, occupied_thresh):
    probability = 1. - np.arange(256)/255.
    candidates = np.flatnonzero((probability >= free_thresh) &
                               (probability <= occupied_thresh))
    if not len(candidates):
        raise ValueError('threshold interval cannot encode unknown cells in an 8-bit PGM')
    # Preserve the conventional gray205 when it decodes as unknown.
    unknown = 205 if 205 in candidates else int(candidates[
        np.argmin(np.abs(probability[candidates]-(free_thresh+occupied_thresh)/2))])
    free = 254 if probability[254] < free_thresh else 255
    return unknown, free


def save_occupancy_map(values, resolution, origin, yaml_path,
                       occupied_thresh=DEFAULT_OCCUPIED_THRESH,
                       free_thresh=DEFAULT_FREE_THRESH):
    """Write a PGM/YAML pair whose three cell states survive a reload."""
    resolution = validate_resolution(resolution)
    free_thresh, occupied_thresh = validate_thresholds(free_thresh, occupied_thresh)
    unknown_gray, free_gray = _gray_levels(free_thresh, occupied_thresh)
    origin = np.asarray(origin, dtype=float)
    if origin.shape != (3,) or not np.isfinite(origin).all():
        raise ValueError('origin must contain finite [x, y, yaw] values')
    values = np.asarray(values)
    if values.ndim != 2 or not values.size:
        raise ValueError('occupancy grid must be a nonempty 2D array')
    image = np.where(values < 0, unknown_gray,
                     np.where(values > 50, 0, free_gray)).astype(np.uint8)
    yaml_path = Path(yaml_path).resolve()
    pgm_path = yaml_path.with_suffix('.pgm')
    yaml_path.parent.mkdir(parents=True, exist_ok=True)
    Image.fromarray(np.flipud(image)).save(pgm_path)
    cfg = dict(image=pgm_path.name, resolution=resolution,
               origin=origin.tolist(), negate=0,
               occupied_thresh=occupied_thresh, free_thresh=free_thresh)
    yaml_path.write_text(yaml.safe_dump(cfg, sort_keys=False))
    return yaml_path, pgm_path


def load_occupancy_map(yaml_path):
    """Decode an image using its YAML probabilities, never a special gray value."""
    yaml_path = Path(yaml_path).resolve()
    cfg = yaml.safe_load(yaml_path.read_text())
    resolution = validate_resolution(cfg['resolution'])
    free_thresh, occupied_thresh = validate_thresholds(
        cfg.get('free_thresh', DEFAULT_FREE_THRESH),
        cfg.get('occupied_thresh', DEFAULT_OCCUPIED_THRESH))
    origin = np.asarray(cfg['origin'], dtype=float)
    if origin.shape != (3,) or not np.isfinite(origin).all():
        raise ValueError('origin must contain finite [x, y, yaw] values')
    image_path = yaml_path.parent / cfg['image']
    with Image.open(image_path) as image:
        pgm = np.flipud(np.asarray(image.convert('L')))
    occupancy = pgm/255. if cfg.get('negate', 0) else 1.-pgm/255.
    values = np.full(pgm.shape, -1, np.int8)
    values[occupancy < free_thresh] = 0
    values[occupancy > occupied_thresh] = 100
    return values, resolution, origin
