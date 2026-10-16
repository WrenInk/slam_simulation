import numpy as np
from scipy.spatial.transform import Rotation
from geometry_msgs.msg import Pose, TransformStamped
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header
from builtin_interfaces.msg import Time


def seconds(stamp):
    return stamp.sec + stamp.nanosec * 1.e-9


def stamp(t):
    ns = int(round(t*1.e9))
    return Time(sec=ns//1000000000, nanosec=ns % 1000000000)


def from_pose(p):
    out = np.eye(4)
    out[:3, :3] = Rotation.from_quat([p.orientation.x, p.orientation.y,
                                     p.orientation.z, p.orientation.w]).as_matrix()
    out[:3, 3] = [p.position.x, p.position.y, p.position.z]
    return out


def to_pose(m):
    p = Pose()
    p.position.x, p.position.y, p.position.z = map(float, m[:3, 3])
    p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w = map(
        float, Rotation.from_matrix(m[:3, :3]).as_quat())
    return p


def tf_msg(m, parent, child, time):
    p = to_pose(m)
    out = TransformStamped(header=Header(stamp=time, frame_id=parent), child_frame_id=child)
    out.transform.translation.x = p.position.x
    out.transform.translation.y = p.position.y
    out.transform.translation.z = p.position.z
    out.transform.rotation = p.orientation
    return out


def xyz(msg):
    endian = '>' if msg.is_bigendian else '<'
    fields = {f.name: f for f in msg.fields}
    # Honour row padding and point padding; upstream FAST-LIO publishes padded PCL points.
    dtype = np.dtype({'names': ['x','y','z'], 'formats': [endian+'f4']*3,
                      'offsets': [fields[n].offset for n in ['x','y','z']],
                      'itemsize': msg.point_step})
    array = np.ndarray((msg.height, msg.width), dtype=dtype, buffer=msg.data,
                       strides=(msg.row_step, msg.point_step))
    out = np.column_stack([array[n].ravel() for n in ['x','y','z']])
    return out[np.isfinite(out).all(axis=1)]


def cloud_msg(points, frame, time):
    points = np.asarray(points, dtype='<f4').reshape(-1, 3)
    return PointCloud2(header=Header(stamp=time, frame_id=frame), height=1, width=len(points),
                      fields=[PointField(name=n, offset=i*4, datatype=PointField.FLOAT32, count=1)
                              for i, n in enumerate(['x','y','z'])],
                      is_bigendian=False, point_step=12, row_step=12*len(points),
                      is_dense=True, data=points.tobytes())
