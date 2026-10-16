"""Gravity-aligned elevation terrain classification shared by map and local scan.

No absolute ground height or simulator geometry is used. Smooth inclined surfaces
are traversable; vertical returns, roughness and observed height discontinuities
remain obstacles. Unobserved cells remain unknown beyond a bounded sampling gap.
"""
from dataclasses import dataclass, fields
import numpy as np
from scipy import ndimage


@dataclass
class TerrainConfig:
    terrain_resolution: float = 0.10
    max_slope_degrees: float = 20.0
    max_step_height: float = 0.065
    max_roughness: float = 0.035
    ground_fill_distance: float = 0.12
    surface_layer_gap: float = 0.04

    @classmethod
    def from_node(cls, node):
        return cls(**{f.name: node.declare_parameter(f.name, f.default).value
                     for f in fields(cls)})


def clear_unknown_footprints(grid, origin, resolution, poses, length, width):
    """Fill the sensor's blind area swept by the robot, preserving all obstacles.

    Poses must be estimated BASE poses, with the IMU lever arm removed. A half
    cell margin includes cells intersecting the physical footprint boundary.
    """
    half = np.array([length, width])/2+resolution/2
    radius = float(np.linalg.norm(half))
    for pose in poses:
        center = pose[:2, 3]
        lower = np.maximum(0, np.floor((center-radius-origin)/resolution).astype(int))
        upper = np.minimum(grid.shape[::-1], np.ceil((center+radius-origin)/resolution).astype(int))
        x0, y0 = lower
        x1, y1 = upper
        if x1 <= x0 or y1 <= y0:
            continue
        ys, xs = np.mgrid[y0:y1, x0:x1]
        points = np.stack([origin[0]+(xs+.5)*resolution-center[0],
                           origin[1]+(ys+.5)*resolution-center[1]], axis=-1)
        yaw = np.arctan2(pose[1, 0], pose[0, 0])
        rotation = np.array([[np.cos(yaw), -np.sin(yaw)], [np.sin(yaw), np.cos(yaw)]])
        inside = np.all(np.abs(points @ rotation) <= half, axis=-1)
        patch = grid[y0:y1, x0:x1]
        patch[inside & (patch < 0)] = 0


@dataclass
class TerrainGrid:
    origin: np.ndarray
    resolution: float
    occupancy: np.ndarray
    elevation: np.ndarray
    slope: np.ndarray

    def obstacle_points(self):
        y, x = np.nonzero(self.occupancy == 100)
        return np.column_stack((self.origin[0]+(x+.5)*self.resolution,
                                self.origin[1]+(y+.5)*self.resolution,
                                self.elevation[y, x]))


def fit_surface(z, known, r, max_jump=None):
    # Fit z = a*dx + b*dy + c in each 3x3 elevation neighborhood. Masked
    # least squares handles boundaries; regularization only handles degenerate fits.
    shape = z.shape
    dy, dx = np.mgrid[-1:2, -1:2]*r
    basis = [dx, dy, np.ones((3, 3))]
    weight = known.astype(float)
    z_known = np.where(known, z, 0.)
    normal = np.zeros((*shape, 3, 3))
    rhs = np.zeros((*shape, 3))
    if max_jump is None:
        for i in range(3):
            rhs[..., i] = ndimage.correlate(z_known, basis[i], mode='constant')
            for j in range(3):
                normal[..., i, j] = ndimage.correlate(weight, basis[i]*basis[j], mode='constant')
        count = ndimage.correlate(weight, np.ones((3, 3)), mode='constant')
        squared = ndimage.correlate(z_known*z_known, np.ones((3, 3)), mode='constant')
    else:
        # Fit within a surface, not across a cliff into a different elevation
        # layer. Explicit pairwise step checks below still block BOTH edges.
        padded=np.pad(z_known,1);mask=np.pad(known,1)
        count=np.zeros(shape);squared=np.zeros(shape)
        for y in range(3):
            for x in range(3):
                sample=padded[y:y+shape[0],x:x+shape[1]]
                valid=mask[y:y+shape[0],x:x+shape[1]] & (np.abs(sample-z)<=max_jump)
                b=np.array([(x-1)*r,(y-1)*r,1.])
                normal += valid[...,None,None]*np.outer(b,b)
                rhs += (sample*valid)[...,None]*b
                count += valid;squared += sample*sample*valid
    normal += np.eye(3)*1.e-8
    plane = np.linalg.solve(normal, rhs)
    slope = np.arctan(np.hypot(plane[..., 0], plane[..., 1]))
    rough = np.sqrt(np.maximum(0., (squared-np.sum(plane*rhs, axis=-1))/np.maximum(count, 1)))
    return plane, slope, rough, count


def classify(points, cfg=None):
    cfg = cfg or TerrainConfig()
    points = np.asarray(points)
    points = points[np.isfinite(points).all(axis=1)]
    r = cfg.terrain_resolution
    if not len(points):
        return TerrainGrid(np.zeros(2), r, np.full((0, 0), -1, np.int8),
                           np.zeros((0, 0)), np.zeros((0, 0)))
    origin = np.floor(points[:, :2].min(axis=0)/r)*r-3*r
    wh = np.ceil((points[:, :2].max(axis=0)-origin)/r).astype(int)+4
    if np.max(wh) > 2000:
        raise ValueError('Terrain map exceeds 2000 cells per side; increase terrain_resolution')
    shape = (wh[1], wh[0])
    cells = np.floor((points[:, :2]-origin)/r).astype(int)
    ids = cells[:, 1]*wh[0]+cells[:, 0]
    low = np.full(np.prod(wh), np.inf)
    high = np.full(np.prod(wh), -np.inf)
    np.minimum.at(low, ids, points[:, 2])
    np.maximum.at(high, ids, points[:, 2])
    low, high = low.reshape(shape), high.reshape(shape)
    observed = np.isfinite(low)
    distance, nearest = ndimage.distance_transform_edt(~observed, return_indices=True)
    known = distance*r <= cfg.ground_fill_distance+1.e-6
    z = low[tuple(nearest)]
    # A thin ramp can expose the floor underneath to lateral rays. Separate
    # disconnected height clusters, and choose an upper sheet only when its
    # neighboring samples support a smooth, traversable surface. A vertical
    # wall (continuous height returns) must keep its full obstacle span.
    order = np.lexsort((points[:, 2], ids))
    sorted_ids, heights = ids[order], points[order, 2]
    jumps = np.diff(heights)
    gaps = (sorted_ids[1:] == sorted_ids[:-1]) & (jumps > cfg.surface_layer_gap)
    # Separate at the dominant gap. A small second gap above a well sampled
    # ramp is often one high return, not a replacement walking surface.
    largest_gap = np.full(np.prod(wh), -np.inf)
    np.maximum.at(largest_gap, sorted_ids[1:][gaps], jumps[gaps])
    gaps &= jumps == largest_gap[sorted_ids[1:]]
    upper_low = np.full(np.prod(wh), -np.inf)
    np.maximum.at(upper_low, sorted_ids[1:][gaps], heights[1:][gaps])
    upper_low = upper_low.reshape(shape)
    upper_span = np.where(observed, high, 0.)-np.where(np.isfinite(upper_low), upper_low, 0.)
    # Judge a separated upper layer by its median, not its single highest
    # return. One noisy high return must not demote an otherwise supported
    # ramp cell all the way to the floor below it and create a false cliff.
    # Cells without a separated layer retain their top as boundary support;
    # their full vertical span is still checked below.
    upper_points = heights >= upper_low.ravel()[sorted_ids]
    layer_counts = np.bincount(sorted_ids[upper_points], minlength=np.prod(wh))
    last = np.full(np.prod(wh), -1, dtype=int)
    np.maximum.at(last, sorted_ids, np.arange(len(sorted_ids)))
    has_layer = np.isfinite(upper_low).ravel()
    representative = high.ravel().copy()
    middle = last[has_layer]-(layer_counts[has_layer]-1)//2
    representative[has_layer] = heights[middle]
    top = representative.reshape(shape)[tuple(nearest)]
    fit_jump=cfg.max_step_height
    top_plane, top_slope, top_rough, top_count = fit_surface(top, known, r, fit_jump)
    sheet = (observed & np.isfinite(upper_low) & (top_count >= 6) &
             (top_slope <= np.deg2rad(cfg.max_slope_degrees)) &
             (top_rough <= cfg.max_roughness) &
             (np.abs(top-top_plane[...,2]) <= cfg.max_roughness) &
             (upper_span <= cfg.max_step_height))
    low = np.where(sheet, upper_low, low)
    # Resolve layers before testing isolation: a sparse ramp sample can
    # legitimately sit above neighboring cells' lower-floor returns.
    # A lone elevated return in an otherwise supported smooth floor is not
    # enough to infer a step and expand it into all neighboring cells. Keep its
    # own cell UNKNOWN, while fitting neighbors from the supported floor.
    # Continuous fences, walls and multi-cell steps retain obstacle evidence.
    counts = np.bincount(ids,minlength=np.prod(wh)).reshape(shape)
    padded = np.pad(np.where(observed,low,np.nan),1,constant_values=np.nan)
    neighbors = np.stack([padded[y:y+shape[0],x:x+shape[1]]
                         for y in range(3) for x in range(3)])
    # Fill absent neighbors with infinity for an order statistic without
    # all-NaN median warnings; at least six observed neighbors are required.
    support = np.isfinite(neighbors).sum(axis=0)
    ordered = np.sort(np.where(np.isfinite(neighbors),neighbors,np.inf),axis=0)
    median = np.take_along_axis(ordered,(np.maximum(support-1,0)//2)[None],axis=0)[0]
    finite_neighbors=np.isfinite(neighbors)
    safe_neighbors=np.where(finite_neighbors,neighbors,0.)
    similar=np.sum(finite_neighbors & (np.abs(safe_neighbors-
        np.where(np.isfinite(median),median,0.))<=cfg.max_roughness),axis=0)
    peers=np.sum(finite_neighbors & (np.abs(safe_neighbors-
        np.where(observed,low,0.))<=cfg.max_roughness),axis=0)
    isolated = observed & (counts<3) & (support>=6) & (similar>=6) & (peers==1) & (low>median+cfg.max_step_height)
    observed = observed & ~isolated
    distance, nearest = ndimage.distance_transform_edt(~observed, return_indices=True)
    known = distance*r <= cfg.ground_fill_distance+1.e-6
    z = low[tuple(nearest)]
    plane, slope, rough, _ = fit_surface(z, known, r, fit_jump)
    blocked = known & ((slope > np.deg2rad(cfg.max_slope_degrees)) |
                        (rough > cfg.max_roughness))
    # Return height above the cell's lowest sample: a thin wall or low fence
    # must not disappear just because a ground return exists in that same cell.
    # Use the measured local slope, not the maximum allowable robot slope.
    vertical_span = high-low
    allowance = (np.abs(plane[..., 0])+np.abs(plane[..., 1]))*r
    blocked |= observed & (vertical_span > cfg.max_step_height+np.minimum(allowance, .035))
    # A ramp changes elevation gradually. A curb/cliff changes it discontinuously;
    # mark BOTH sides, including negative obstacles when both surfaces were seen.
    for sy, sx in [(0, 1), (1, 0)]:
        pair = known & np.roll(known, (sy, sx), (0, 1))
        jump = np.abs(z-np.roll(z, (sy, sx), (0, 1)))
        edge = pair & (jump > cfg.max_step_height)
        blocked |= edge | np.roll(edge, (-sy, -sx), (0, 1))
    occupancy = np.full(shape, -1, np.int8)
    occupancy[known] = 0
    occupancy[blocked] = 100
    occupancy[isolated] = -1
    return TerrainGrid(origin, r, occupancy, np.where(known, z, np.nan), slope)


def pcd_to_occupancy(pcd_path, output_dir=None, resolution=0.05, name='map',
                      occupied_thresh=0.65, free_thresh=0.25):
    """Convert a 3D PCD point cloud into 2D Nav2 Occupancy Grid Map (.yaml + .pgm)."""
    from pathlib import Path
    from PIL import Image
    import yaml
    from .geometry import read_pcd

    pcd_path = Path(pcd_path).resolve()
    if not pcd_path.is_file():
        raise FileNotFoundError(f"PCD file not found: {pcd_path}")
    output_dir = Path(output_dir or pcd_path.parent).resolve()
    output_dir.mkdir(parents=True, exist_ok=True)

    points = read_pcd(pcd_path)
    terrain_res = max(resolution, 0.10)
    terrain = classify(points, TerrainConfig(terrain_resolution=terrain_res))
    scale = max(1, int(round(terrain.resolution / resolution)))
    grid = np.repeat(np.repeat(terrain.occupancy, scale, axis=0), scale, axis=1)
    origin = terrain.origin  # [min_x, min_y]
    height, width = grid.shape

    image = np.where(grid < 0, 205, np.where(grid > 50, 0, 254)).astype(np.uint8)
    pgm_path = output_dir / f"{name}.pgm"
    yaml_path = output_dir / f"{name}.yaml"

    Image.fromarray(np.flipud(image)).save(pgm_path)
    yaml_content = {
        'image': f"{name}.pgm",
        'resolution': float(resolution),
        'origin': [float(origin[0]), float(origin[1]), 0.0],
        'negate': 0,
        'occupied_thresh': float(occupied_thresh),
        'free_thresh': float(free_thresh)
    }
    yaml_path.write_text(yaml.safe_dump(yaml_content, sort_keys=False))
    return yaml_path, pgm_path

