"""Standalone relative crossing direction estimator (2.5D Euclidean clustering).

Not a robot classifier. IDs are temporary. Works best with a stationary observer.
"""
import json
import math
import numpy as np
from scipy.spatial import cKDTree
from scipy.optimize import linear_sum_assignment
import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import String
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point
from tf2_ros import Buffer, TransformListener, TransformException


def quaternion_rot(q):
    x, y, z, w = q.x, q.y, q.z, q.w
    n = x*x + y*y + z*z + w*w
    if n < 1e-12:
        return np.eye(3)
    s = 2.0/n
    return np.array([
        [1-s*(y*y+z*z), s*(x*y-z*w), s*(x*z+y*w)],
        [s*(x*y+z*w), 1-s*(x*x+z*z), s*(y*z-x*w)],
        [s*(x*z-y*w), s*(y*z+x*w), 1-s*(x*x+y*y)]])


def cloud_xyz(msg):
    # Humble sensor_msgs_py returns a structured array for read_points_numpy.
    try:
        a = point_cloud2.read_points_numpy(msg, field_names=('x','y','z'), skip_nans=True)
        return np.asarray(a, dtype=np.float64).reshape((-1, 3))
    except (AttributeError, TypeError, ValueError):
        return np.array(list(point_cloud2.read_points(
            msg, field_names=('x','y','z'), skip_nans=True)),
            dtype=np.float64).reshape((-1, 3))


def cluster_indices(xyz, radius, min_size, max_size):
    """Connected components in 3D radius-neighborhood graph (not ground segmentation)."""
    n = len(xyz)
    if n == 0:
        return []
    tree = cKDTree(xyz)
    parent = np.arange(n)
    sizes = np.ones(n, dtype=np.int32)

    def find(i):
        while parent[i] != i:
            parent[i] = parent[parent[i]]
            i = parent[i]
        return i

    for i, j in tree.query_pairs(radius, output_type='ndarray'):
        ri, rj = find(int(i)), find(int(j))
        if ri != rj:
            if sizes[ri] < sizes[rj]:
                ri, rj = rj, ri
            parent[rj] = ri
            sizes[ri] += sizes[rj]

    groups = {}
    for i in range(n):
        root = find(i)
        groups.setdefault(root, []).append(i)
    return [np.asarray(v, dtype=np.int32) for v in groups.values()
            if min_size <= len(v) <= max_size]


class PassageDetector(Node):
    def __init__(self):
        super().__init__('passage_detector')
        settings = {
            'input_topic': '/livox/lidar', 'base_frame': 'base_link',
            'min_range_m': 0.4, 'max_range_m': 8.0,
            'min_z_m': 0.12, 'max_z_m': 1.8,
            'self_radius_m': 0.65, 'voxel_m': 0.10,
            'cluster_tolerance_m': 0.28, 'min_cluster_points': 5,
            'max_cluster_points': 3000, 'max_cluster_width_m': 1.8,
            'max_points_after_voxel': 6000,
            'association_distance_m': 0.85, 'max_missed_s': 1.0,
            'min_confirmed_hits': 3, 'x_threshold_m': 0.6,
            'side_min_m': 0.35, 'side_max_m': 2.5,
            'min_crossing_duration_s': 0.35,
            'max_crossing_duration_s': 12.0,
            'tf_timeout_s': 0.15,
        }
        for name, default in settings.items():
            self.declare_parameter(name, default)
        self.p = {name: self.get_parameter(name).value for name in settings}
        self.tf_buffer = Buffer()
        self.listener = TransformListener(self.tf_buffer, self)
        self.tracks = {}
        self.next_id = 1
        self.last_marker_ids = set()
        self.last_warning_ns = 0
        self.events = self.create_publisher(String, '/passing_direction', 10)
        self.tracked = self.create_publisher(String, '/tracked_objects', 10)
        self.markers = self.create_publisher(MarkerArray, '/tracked_objects_markers', 10)
        self.sub = self.create_subscription(PointCloud2, self.p['input_topic'], self.callback, 10)
        self.get_logger().info('Listening: %s, base frame: %s' % (
            self.p['input_topic'], self.p['base_frame']))

    def warn_throttled(self, msg):
        now = self.get_clock().now().nanoseconds
        if now - self.last_warning_ns > 3_000_000_000:
            self.get_logger().warn(msg)
            self.last_warning_ns = now

    def callback(self, msg):
        stamp = rclpy.time.Time.from_msg(msg.header.stamp)
        t = stamp.nanoseconds / 1e9
        try:
            if msg.header.frame_id != self.p['base_frame']:
                tf = self.tf_buffer.lookup_transform(
                    self.p['base_frame'], msg.header.frame_id, stamp,
                    timeout=Duration(seconds=self.p['tf_timeout_s']))
                rot = quaternion_rot(tf.transform.rotation)
                trans = np.array([tf.transform.translation.x,
                                  tf.transform.translation.y,
                                  tf.transform.translation.z])
            else:
                rot, trans = np.eye(3), np.zeros(3)
        except TransformException as exc:
            self.warn_throttled('TF unavailable (%s -> %s): %s' % (
                msg.header.frame_id, self.p['base_frame'], str(exc)))
            return
        try:
            xyz = cloud_xyz(msg)
        except Exception as exc:
            self.warn_throttled('PointCloud2 conversion error: %s' % str(exc))
            return
        if not len(xyz):
            return
        xyz = xyz @ rot.T + trans
        finite = np.isfinite(xyz).all(axis=1)
        xyz = xyz[finite]
        d = np.linalg.norm(xyz[:, :2], axis=1)
        p = self.p
        valid = ((d >= max(p['min_range_m'], p['self_radius_m'])) &
                 (d <= p['max_range_m']) &
                 (xyz[:, 2] >= p['min_z_m']) & (xyz[:, 2] <= p['max_z_m']))
        xyz = xyz[valid]
        if not len(xyz):
            self.update([], t, msg.header.stamp)
            return
        # One point per voxel, deterministic sampling to bound CPU cost.
        vox = np.floor(xyz / max(p['voxel_m'], 0.01)).astype(np.int64)
        _, keep = np.unique(vox, axis=0, return_index=True)
        xyz = xyz[np.sort(keep)]
        cap = max(100, int(p['max_points_after_voxel']))
        if len(xyz) > cap:
            xyz = xyz[np.linspace(0, len(xyz)-1, cap, dtype=np.int64)]
        clusters = cluster_indices(xyz, p['cluster_tolerance_m'],
                                   p['min_cluster_points'], p['max_cluster_points'])
        detections = []
        for inds in clusters:
            points = xyz[inds]
            extent = np.ptp(points[:, :2], axis=0)
            if np.max(extent) > p['max_cluster_width_m']:
                continue
            detections.append(np.median(points[:, :2], axis=0))
        self.update(detections, t, msg.header.stamp)

    def update(self, detections, t, ros_stamp):
        p = self.p
        existing_ids = list(self.tracks)
        matched_ids, matched_det = set(), set()
        if existing_ids and detections:
            predicted = np.array([
                self.tracks[i]['xy'] + self.tracks[i]['vel'] *
                min(max(0.0, t-self.tracks[i]['seen']), 0.5)
                for i in existing_ids])
            obs = np.asarray(detections)
            costs = np.linalg.norm(predicted[:, None, :] - obs[None, :, :], axis=2)
            # Gate before assignment with a large finite penalty.
            costs[costs > p['association_distance_m']] = 1e6
            rows, cols = linear_sum_assignment(costs)
            for a, b in zip(rows, cols):
                if costs[a, b] >= 1e6:
                    continue
                ident = existing_ids[a]
                track = self.tracks[ident]
                xy = obs[b]
                dt = t - track['seen']
                if dt > 0.001:
                    velocity = (xy - track['xy']) / dt
                    track['vel'] = 0.55*track['vel'] + 0.45*velocity
                track['xy'] = xy
                track['seen'] = t
                track['hits'] += 1
                matched_ids.add(ident)
                matched_det.add(int(b))
                self.check_passage(ident, track, t)
        for idx, xy in enumerate(detections):
            if idx in matched_det:
                continue
            ident = self.next_id
            self.next_id += 1
            self.tracks[ident] = {
                'xy': np.asarray(xy), 'vel': np.zeros(2),
                'seen': t, 'hits': 1, 'origin_side': 0,
                'origin_time': t, 'event_sent': False,
                'side_seen': False,
            }
        for ident in list(self.tracks):
            if t - self.tracks[ident]['seen'] > p['max_missed_s']:
                del self.tracks[ident]
        self.publish(t, ros_stamp)

    def check_passage(self, ident, track, t):
        p = self.p
        if track['event_sent'] or track['hits'] < p['min_confirmed_hits']:
            return
        x, y = track['xy']
        on_side = p['side_min_m'] <= abs(y) <= p['side_max_m']
        side = -1 if x < -p['x_threshold_m'] else (1 if x > p['x_threshold_m'] else 0)
        if not on_side:
            # Retain origin across short excursions outside the side corridor;
            # events require the destination to be inside it.
            return
        track['side_seen'] = True
        if track['origin_side'] == 0:
            if side:
                track['origin_side'] = side
                track['origin_time'] = t
            return
        dt = t-track['origin_time']
        if dt > p['max_crossing_duration_s']:
            track['origin_side'] = side
            track['origin_time'] = t
            return
        if side == -track['origin_side'] and dt >= p['min_crossing_duration_s']:
            direction = ('REAR_TO_FRONT' if track['origin_side'] == -1
                         else 'FRONT_TO_REAR')
            payload = {'object_id': ident, 'direction': direction,
                       'relative_motion': True, 'time_sec': t,
                       'x_m': round(float(x), 3), 'y_m': round(float(y), 3)}
            self.events.publish(String(data=json.dumps(payload)))
            self.get_logger().info('PASSAGE: %s (ID %d)' % (direction, ident))
            track['event_sent'] = True

    def publish(self, t, ros_stamp):
        active = {ident: tr for ident, tr in self.tracks.items()
                  if t-tr['seen'] < 0.15 and tr['hits'] >= self.p['min_confirmed_hits']}
        payload = {'frame_id': self.p['base_frame'], 'time_sec': t, 'objects': [
            {'id': ident, 'x': round(float(tr['xy'][0]), 3),
             'y': round(float(tr['xy'][1]), 3),
             'vx': round(float(tr['vel'][0]), 3),
             'vy': round(float(tr['vel'][1]), 3),
             'hits': tr['hits']} for ident, tr in active.items()]}
        self.tracked.publish(String(data=json.dumps(payload)))
        markers = MarkerArray()
        for ident in self.last_marker_ids - set(active):
            m = Marker()
            m.header.frame_id = self.p['base_frame']
            m.header.stamp = ros_stamp
            m.ns = 'tracked_objects'
            m.id = int(ident)
            m.action = Marker.DELETE
            markers.markers.append(m)
        for ident, tr in active.items():
            m = Marker()
            m.header.frame_id = self.p['base_frame']
            m.header.stamp = ros_stamp
            m.ns = 'tracked_objects'
            m.id = int(ident)
            m.type = Marker.TEXT_VIEW_FACING
            m.action = Marker.ADD
            m.pose.position.x = float(tr['xy'][0])
            m.pose.position.y = float(tr['xy'][1])
            m.pose.position.z = 1.6
            m.pose.orientation.w = 1.0
            m.scale.z = 0.25
            m.color.r, m.color.g, m.color.b, m.color.a = 0.1, 1.0, 0.5, 1.0
            m.text = 'ID %d' % ident
            markers.markers.append(m)
        self.last_marker_ids = set(active)
        self.markers.publish(markers)


def main(args=None):
    rclpy.init(args=args)
    node = PassageDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
