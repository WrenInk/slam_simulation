"""SE(3), point cloud and ICP utilities, independent of ROS."""
import numpy as np
from scipy.spatial import cKDTree
from scipy.spatial.transform import Rotation


def transform(points, pose):
    return points @ pose[:3, :3].T + pose[:3, 3]


def inverse(pose):
    out = np.eye(4)
    out[:3, :3] = pose[:3, :3].T
    out[:3, 3] = -out[:3, :3] @ pose[:3, 3]
    return out


def matrix(x=0., y=0., z=0., yaw=0.):
    out = np.eye(4)
    out[:3, :3] = Rotation.from_euler('z', yaw).as_matrix()
    out[:3, 3] = [x, y, z]
    return out


def voxelize(points, size):
    points = np.asarray(points, dtype=np.float64).reshape(-1, 3)
    points = points[np.isfinite(points).all(axis=1)]
    if not len(points):
        return points
    _, ids = np.unique(np.floor(points / size).astype(np.int64), axis=0, return_index=True)
    return points[np.sort(ids)]


def supported_points(points, radius, min_neighbors):
    """Radius outlier removal for accumulated maps; count includes the point.

    This is a 3D sampling-confidence check, independent of surface height. The
    live obstacle scan is not filtered by this static-map integration rule.
    """
    points = np.asarray(points, dtype=np.float64).reshape(-1, 3)
    points = points[np.isfinite(points).all(axis=1)]
    if min_neighbors <= 1 or not len(points):
        return points
    if radius <= 0:
        raise ValueError('outlier radius must be positive')
    counts = cKDTree(points).query_ball_point(points, radius, return_length=True)
    return points[counts >= min_neighbors]


def icp(source, target, max_distance=0.7, max_iterations=60, min_overlap=0.55):
    """Point-to-point ICP in the common map frame, with explicit convergence gates.

    Return correction, mean squared inlier distance (m²), overlap, convergence.
    Never accepts a sparse, non-finite or exhausted fit.
    """
    correction = np.eye(4)
    if min(len(source), len(target)) < 30:
        return correction, float('inf'), 0., False
    tree = cKDTree(target)
    moving = source.copy()
    previous = np.inf
    converged = False
    for _ in range(max_iterations):
        distance, ids = tree.query(moving, workers=1)
        mask = distance < max_distance
        if mask.mean() < min_overlap or mask.sum() < 30:
            return correction, float('inf'), float(mask.mean()), False
        a, b = moving[mask], target[ids[mask]]
        ca, cb = a.mean(axis=0), b.mean(axis=0)
        u, _, vt = np.linalg.svd((a-ca).T @ (b-cb))
        rot = vt.T @ u.T
        if np.linalg.det(rot) < 0:
            vt[-1] *= -1
            rot = vt.T @ u.T
        step = np.eye(4)
        step[:3, :3], step[:3, 3] = rot, cb - rot @ ca
        moving = transform(moving, step)
        correction = step @ correction
        score = float(np.mean(distance[mask] ** 2))
        if abs(previous-score) < 1.e-7:
            converged = True
            break
        previous = score
    distance, _ = tree.query(moving, workers=1)
    mask = distance < max_distance
    overlap = float(mask.mean())
    score = float(np.mean(distance[mask] ** 2)) if mask.any() else float('inf')
    return correction, score, overlap, converged and overlap >= min_overlap


def write_pcd(path, points):
    points = np.asarray(points, dtype='<f4').reshape(-1, 3)
    header = (f'# .PCD v0.7\nVERSION 0.7\nFIELDS x y z\nSIZE 4 4 4\nTYPE F F F\n'
              f'COUNT 1 1 1\nWIDTH {len(points)}\nHEIGHT 1\nVIEWPOINT 0 0 0 1 0 0 0\n'
              f'POINTS {len(points)}\nDATA binary\n')
    with open(path, 'wb') as f:
        f.write(header.encode())
        f.write(points.tobytes())


def read_pcd(path):
    with open(path, 'rb') as f:
        header = []
        while True:
            line = f.readline().decode('ascii', errors='ignore').strip()
            header.append(line)
            if line.startswith('DATA'):
                data_type = line.split()[1].lower()
                break
        fields = {}
        for h in header:
            parts = h.split()
            if len(parts) >= 2:
                fields[parts[0].upper()] = parts[1:]
        num_points = int(fields.get('POINTS', [0])[0])
        field_names = [name.lower() for name in fields.get('FIELDS', [])]
        sizes = [int(s) for s in fields.get('SIZE', [])]
        point_step = sum(sizes) if sizes else 12

        if data_type == 'binary':
            raw = f.read()
            if field_names[:3] == ['x', 'y', 'z'] and point_step == 12:
                data = np.frombuffer(raw[:num_points * 12], dtype='<f4')
                return data.reshape(-1, 3).astype(np.float64)
            else:
                dtype_list = [(name, f'<f{size}') for name, size in zip(field_names, sizes)]
                arr = np.frombuffer(raw[:num_points * point_step], dtype=dtype_list)
                return np.column_stack([arr['x'], arr['y'], arr['z']]).astype(np.float64)
        elif data_type == 'ascii':
            data = np.loadtxt(f)
            return data[:num_points, :3].astype(np.float64)
        else:
            raise ValueError(f'Unsupported PCD data type: {data_type}')

