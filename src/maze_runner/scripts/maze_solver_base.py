#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import os
import sys
_WORKSPACE_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..', '..'))
_ROSCONSOLE_CFG = os.path.join(_WORKSPACE_ROOT, 'rosconsole_silent_tf.conf')
if os.path.exists(_ROSCONSOLE_CFG) and ('ROSCONSOLE_CONFIG_FILE' not in os.environ):
    os.environ['ROSCONSOLE_CONFIG_FILE'] = _ROSCONSOLE_CFG
# tf2内部重复TF告警来自console_bridge，需单独降级日志级别
os.environ.setdefault('CONSOLE_BRIDGE_LOG_LEVEL', 'error')
os.environ.setdefault('CONSOLE_BRIDGE_log_level', 'error')


class _FilteredConsoleStream(object):
    """Drop known TF repeated-data spam lines for this process only."""

    def __init__(self, raw):
        self._raw = raw
        self._buf = ''

    def write(self, data):
        if not data:
            return 0
        self._buf += data
        while '\n' in self._buf:
            line, self._buf = self._buf.split('\n', 1)
            if self._drop(line):
                continue
            self._raw.write(line + '\n')
        return len(data)

    def flush(self):
        if self._buf and (not self._drop(self._buf)):
            self._raw.write(self._buf)
        self._buf = ''
        self._raw.flush()

    def _drop(self, line):
        return ('TF_REPEATED_DATA' in line) or ('buffer_core.cpp' in line)


if not isinstance(sys.stderr, _FilteredConsoleStream):
    sys.stderr = _FilteredConsoleStream(sys.stderr)
if not isinstance(sys.stdout, _FilteredConsoleStream):
    sys.stdout = _FilteredConsoleStream(sys.stdout)

import rospy
import numpy as np
import math
import json
from datetime import datetime
from collections import deque
import tf2_ros
from geometry_msgs.msg import Twist, Point
from nav_msgs.msg import Odometry, OccupancyGrid
from sensor_msgs.msg import LaserScan, Image
from std_msgs.msg import String
try:
    from cv_bridge import CvBridge, CvBridgeError
except Exception:  # cv_bridge may fail to load (ABI/OpenCV mismatch)
    CvBridge = None
    CvBridgeError = Exception
try:
    import cv2
except Exception:
    cv2 = None
from tf.transformations import euler_from_quaternion


class MazeSolver:
    def __init__(self):
        rospy.init_node('maze_solver', anonymous=True)
        self.vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.scan_sub = rospy.Subscriber('/scan', LaserScan, self.scan_callback)
        self.odom_sub = rospy.Subscriber('/odom', Odometry, self.odom_callback)

        # 颜色探测：相机输入与检测结果输出
        self.image_topic = rospy.get_param('~image_topic', '/camera/rgb/image_raw')
        self.image_sub = rospy.Subscriber(self.image_topic, Image, self.image_callback, queue_size=1)
        self.target_pub = rospy.Publisher('/detected_targets', String, queue_size=10)
        self.bridge = CvBridge() if CvBridge is not None else None
        self._warned_no_bridge = False

        # 机器人位姿与激光
        self.current_pos = Point()
        self.current_yaw = 0.0
        self.odom_received = False
        self.scan_msg = None
        self.laser_ranges = []
        self.turn_dir = 1.0
        self.rate = rospy.Rate(10)

        # 控制参数
        self.safe_distance = 0.45
        self.max_linear = 0.30
        self.max_angular = 1.2
        self.goal_tolerance = 0.28
        self.yaw_tolerance = 0.35

        # 保底墙跟随
        self.wall_dist = 0.45
        self.wall_kp = 1.5
        self.wall_linear = 0.18

        # Bug导航状态
        self.nav_state = "GOTO"
        self._wall_hit_dist = None
        self._wall_follow_enter_time = 0.0

        # 路径与frontier状态
        self.current_frontier = None
        self.path_waypoints = []
        self.wp_index = 0
        self.waypoint_step = 3
        self.waypoint_reach_tol = 0.22
        self.replan_interval = 2.0
        self.last_plan_time = 0.0
        self.min_path_cells = 10

        # frontier选取参数
        self.frontier_min_cluster = 6
        self.frontier_blacklist = deque(maxlen=100)  # (x, y, ts)
        self.blacklist_radius = 1.5
        self.blacklist_ttl_sec = 60.0
        self.recent_frontiers = deque(maxlen=20)  # (x, y)
        self.revisit_radius = 1.0
        self.last_forced_switch_time = 0.0
        self.forced_switch_cooldown = 3.0
        self.force_escape_until = 0.0

        # 地图膨胀参数：把靠墙太近的free栅格视为不可通行，减少“理论可达/实际过不去”
        self.robot_radius_m = 0.18
        self.clearance_m = 0.12
        self._inflate_radius_cells = -1
        self._inflate_offsets = []

        # 完成判定：连续多次无未知栅格才停
        self.no_unknown_count = 0
        self.no_unknown_need = 20  # 10Hz下约2秒

        # 卡死检测与恢复
        self._pose_hist = deque(maxlen=40)
        self._stuck_dist_eps = 0.03
        self._stuck_time_sec = 6.0
        self._recovery_until = 0.0
        self._stuck_event_count = 0
        self.max_stuck_before_replan = 2

        # 局部最优脱困模式
        self.local_minima_until = 0.0
        self.local_minima_count = 0

        # 进展监测：长时间无位移则强制换frontier
        self._last_progress_pose = None
        self._last_progress_time = 0.0
        self.progress_eps = 0.06
        self.no_progress_timeout = 8.0

        # 地图
        self.map_data = None
        self.map_info = None
        self.map_sub = rospy.Subscriber("/map", OccupancyGrid, self.map_callback)

        # TF：优先使用map坐标系位姿做规划
        self.tf_buffer = tf2_ros.Buffer(rospy.Duration(10.0))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        self.map_pose_valid = False
        self.map_pos_x = 0.0
        self.map_pos_y = 0.0
        self.map_yaw = 0.0

        # ========== 颜色探测配置 ==========
        self.enable_color_detection = rospy.get_param('~enable_color_detection', True)
        self.camera_hfov_deg = float(rospy.get_param('~camera_hfov_deg', 62.0))
        self.min_blob_area = float(rospy.get_param('~min_blob_area', 450.0))
        self.detection_merge_radius = float(rospy.get_param('~detection_merge_radius', 0.45))
        default_report_dir = os.path.join(_WORKSPACE_ROOT, '巡逻报告')
        self.report_dir = rospy.get_param('~report_dir', default_report_dir)
        run_tag = datetime.now().strftime('%Y%m%d_%H%M%S')
        default_json = os.path.join(self.report_dir, 'patrol_report_%s.json' % run_tag)
        default_md = os.path.join(self.report_dir, 'patrol_report_%s.md' % run_tag)
        self.report_output_path = rospy.get_param('~report_output_path', default_json)
        self.report_autosave_sec = float(rospy.get_param('~report_autosave_sec', 5.0))
        self._last_report_save_time = 0.0
        self.target_z = float(rospy.get_param('~target_z', 0.1))
        self._final_report_printed = False
        self.markdown_report_path = rospy.get_param('~markdown_report_path', default_md)

        # 返航触发：未知区域长时间变化很小（B方案）
        self.unknown_stable_window_sec = float(rospy.get_param('~unknown_stable_window_sec', 120.0))
        self.unknown_stable_min_runtime_sec = float(rospy.get_param('~unknown_stable_min_runtime_sec', 90.0))
        self.unknown_stable_delta_cells = int(rospy.get_param('~unknown_stable_delta_cells', 60))
        self.unknown_history = deque(maxlen=3000)  # (time, unknown_cells)
        self.start_time = rospy.get_time()
        self.return_mode = False
        self.return_completed = False
        self.frontier_missing_since = 0.0
        self.return_requires_no_frontier_sec = float(rospy.get_param('~return_requires_no_frontier_sec', 45.0))

        # 起点位姿（A方案：程序启动后的初始位姿）
        self.home_pose = None  # (x, y, yaw)
        self.home_reach_tol = float(rospy.get_param('~home_reach_tol', 0.25))
        self.home_yaw_tol = float(rospy.get_param('~home_yaw_tol', 0.20))

        # Bug墙跟随超时脱困：避免沿障碍持续打转
        self.wall_follow_timeout_sec = float(rospy.get_param('~wall_follow_timeout_sec', 6.0))
        self.wall_no_improve_eps = float(rospy.get_param('~wall_no_improve_eps', 0.06))
        self.wall_enter_count = 0
        self.wall_enter_limit = int(rospy.get_param('~wall_enter_limit', 3))

        # 闭合回路返航判定（D方案）：轨迹闭环 + 包围框已探明比例高
        self.enable_loop_return = bool(rospy.get_param('~enable_loop_return', True))
        self.loop_close_dist = float(rospy.get_param('~loop_close_dist', 0.35))
        self.loop_min_path_len = float(rospy.get_param('~loop_min_path_len', 8.0))
        self.loop_min_points_gap = int(rospy.get_param('~loop_min_points_gap', 25))
        self.loop_known_ratio_thresh = float(rospy.get_param('~loop_known_ratio_thresh', 0.86))
        self.loop_bbox_min_area = float(rospy.get_param('~loop_bbox_min_area', 6.0))
        self.loop_no_frontier_sec = float(rospy.get_param('~loop_no_frontier_sec', 20.0))
        self.loop_pose_hist = deque(maxlen=2400)  # (x, y)
        self.loop_cumlen_hist = deque(maxlen=2400)  # cumulative length at sample
        self.loop_path_total = 0.0
        self.loop_sample_step = float(rospy.get_param('~loop_sample_step', 0.10))
        self._loop_return_logged = False

        # 聚类参数（任务二：按颜色分别做空间聚类）
        self.cluster_radius = float(rospy.get_param('~cluster_radius', 2.00))
        self.cluster_merge_radius = float(rospy.get_param('~cluster_merge_radius', 3.20))

        # 按颜色记录目标：{color: [{x,y,ts,px,py,area}, ...]}
        self.detected_targets = {
            'green': [],
            'red': [],
            'yellow': [],
            'blue': [],
        }

        # HSV阈值（OpenCV范围: H[0,179], S/V[0,255]）
        self.color_hsv_ranges = {
            'green': [((35, 70, 70), (85, 255, 255))],
            'yellow': [((20, 90, 90), (35, 255, 255))],
            'blue': [((90, 80, 60), (130, 255, 255))],
            # 红色跨越色环两端
            'red': [((0, 90, 70), (10, 255, 255)), ((170, 90, 70), (179, 255, 255))],
        }

        rospy.on_shutdown(self._on_shutdown)

    def image_callback(self, msg):
        if not self.enable_color_detection:
            return

        frame_rgb = self._image_msg_to_rgb(msg)
        if frame_rgb is None or frame_rgb.size == 0:
            return

        h, w = frame_rgb.shape[:2]
        if w <= 0:
            return

        # 使用map位姿（若可用）否则回退到odom
        if self.map_pose_valid:
            rx, ry, ryaw = self.map_pos_x, self.map_pos_y, self.map_yaw
        else:
            rx, ry, ryaw = float(self.current_pos.x), float(self.current_pos.y), float(self.current_yaw)

        found_any = False
        for color_name in ('red', 'green', 'yellow', 'blue'):
            mask = self._build_color_mask_numpy(frame_rgb, color_name)
            if mask is None:
                continue

            cx, cy, area = self._largest_component_stats(mask)
            if area < self.min_blob_area:
                continue

            # 简单方位估计：像素偏移 -> 水平角度
            pixel_offset = (cx - (w / 2.0)) / (w / 2.0)
            bearing = math.radians(self.camera_hfov_deg * 0.5) * pixel_offset

            # 简单距离估计：面积越大越近（经验模型）
            est_dist = 2200.0 / max(1.0, math.sqrt(area))
            est_dist = float(max(0.45, min(3.0, est_dist)))

            tx = rx + est_dist * math.cos(ryaw + bearing)
            ty = ry + est_dist * math.sin(ryaw + bearing)

            if self._register_target(color_name, tx, ty, cx, cy, area):
                found_any = True

        if found_any:
            self._publish_target_report()

    def _image_msg_to_rgb(self, msg):
        """Convert sensor_msgs/Image to RGB ndarray, without requiring cv2."""
        # Preferred path: cv_bridge (if available)
        if self.bridge is not None:
            try:
                return self.bridge.imgmsg_to_cv2(msg, desired_encoding='rgb8')
            except CvBridgeError:
                pass
            except Exception:
                pass

        # Manual fallback for common raw image encodings
        try:
            h = int(msg.height)
            w = int(msg.width)
            if h <= 0 or w <= 0:
                return None

            enc = (msg.encoding or '').lower()
            buf = np.frombuffer(msg.data, dtype=np.uint8)

            if enc == 'rgb8':
                expected = h * w * 3
                if buf.size < expected:
                    return None
                return buf[:expected].reshape((h, w, 3))

            if enc == 'bgr8':
                expected = h * w * 3
                if buf.size < expected:
                    return None
                bgr = buf[:expected].reshape((h, w, 3))
                return bgr[:, :, ::-1]

            if enc == 'mono8':
                expected = h * w
                if buf.size < expected:
                    return None
                gray = buf[:expected].reshape((h, w))
                return np.repeat(gray[:, :, None], 3, axis=2)

            # Unknown encoding
            if not self._warned_no_bridge:
                rospy.logwarn('无法解码图像编码: %s (cv_bridge不可用或转换失败)', msg.encoding)
                self._warned_no_bridge = True
            return None
        except Exception:
            return None

    def _build_color_mask_numpy(self, frame_rgb, color_name):
        """Simple robust color masks in RGB space for Gazebo-like scenes."""
        r = frame_rgb[:, :, 0].astype(np.int16)
        g = frame_rgb[:, :, 1].astype(np.int16)
        b = frame_rgb[:, :, 2].astype(np.int16)

        if color_name == 'red':
            return (r > 120) & (r > g + 35) & (r > b + 35)
        if color_name == 'green':
            return (g > 110) & (g > r + 25) & (g > b + 25)
        if color_name == 'blue':
            return (b > 100) & (b > r + 25) & (b > g + 25)
        if color_name == 'yellow':
            return (r > 120) & (g > 120) & (b < 110) & (np.abs(r - g) < 70)
        return None

    def _largest_component_stats(self, mask):
        """Return centroid (cx, cy) and area of the largest 8-connected component."""
        if mask is None:
            return 0.0, 0.0, 0.0

        h, w = mask.shape
        visited = np.zeros((h, w), dtype=np.uint8)
        ys, xs = np.nonzero(mask)
        if xs.size == 0:
            return 0.0, 0.0, 0.0

        best_area = 0
        best_sum_x = 0.0
        best_sum_y = 0.0

        neighbors = [
            (-1, -1), (-1, 0), (-1, 1),
            (0, -1),           (0, 1),
            (1, -1),  (1, 0),  (1, 1),
        ]

        for x0, y0 in zip(xs, ys):
            if visited[y0, x0]:
                continue
            if not mask[y0, x0]:
                continue

            q = deque()
            q.append((x0, y0))
            visited[y0, x0] = 1

            area = 0
            sum_x = 0.0
            sum_y = 0.0

            while q:
                x, y = q.popleft()
                area += 1
                sum_x += float(x)
                sum_y += float(y)

                for dx, dy in neighbors:
                    nx = x + dx
                    ny = y + dy
                    if nx < 0 or ny < 0 or nx >= w or ny >= h:
                        continue
                    if visited[ny, nx] or (not mask[ny, nx]):
                        continue
                    visited[ny, nx] = 1
                    q.append((nx, ny))

            if area > best_area:
                best_area = area
                best_sum_x = sum_x
                best_sum_y = sum_y

        if best_area <= 0:
            return 0.0, 0.0, 0.0

        return (best_sum_x / best_area), (best_sum_y / best_area), float(best_area)

    def _register_target(self, color_name, x, y, px, py, area):
        target_list = self.detected_targets[color_name]
        for item in target_list:
            if math.hypot(item['x'] - x, item['y'] - y) < self.detection_merge_radius:
                # 已有目标则轻量更新位置（指数平滑）
                alpha = 0.35
                item['x'] = alpha * x + (1.0 - alpha) * item['x']
                item['y'] = alpha * y + (1.0 - alpha) * item['y']
                item['ts'] = rospy.get_time()
                item['px'] = float(px)
                item['py'] = float(py)
                item['area'] = float(area)
                return False

        target_list.append({
            'x': float(x),
            'y': float(y),
            'z': float(self.target_z),
            'color': str(color_name),
            'ts': rospy.get_time(),
            'px': float(px),
            'py': float(py),
            'area': float(area),
        })
        rospy.loginfo("Target detected! Location: (%.2f, %.2f, %.2f) color=%s", x, y, self.target_z, color_name)
        return True

    def _publish_target_report(self):
        report = self._build_target_report()
        self.target_pub.publish(String(data=json.dumps(report, ensure_ascii=False)))

        now = rospy.get_time()
        if (now - self._last_report_save_time) >= self.report_autosave_sec:
            self._save_report_to_file(report)
            self._last_report_save_time = now

    def _build_target_report(self):
        clusters = self._cluster_all_targets()
        cluster_summary = {
            'red': 0,
            'green': 0,
            'yellow': 0,
            'blue': 0,
            'total': 0,
        }
        for c in clusters:
            color = c['color']
            if color in cluster_summary:
                cluster_summary[color] += 1
        cluster_summary['total'] = len(clusters)

        return {
            'stamp': rospy.get_time(),
            'raw_summary': {
                'green': len(self.detected_targets['green']),
                'red': len(self.detected_targets['red']),
                'yellow': len(self.detected_targets['yellow']),
                'blue': len(self.detected_targets['blue']),
            },
            'summary': cluster_summary,
            'targets_raw': self.detected_targets,
            'targets_clustered': clusters,
            'frame': 'map_or_odom_fallback',
        }

    def _save_report_to_file(self, report):
        try:
            out_dir = os.path.dirname(self.report_output_path)
            if out_dir:
                os.makedirs(out_dir, exist_ok=True)
            with open(self.report_output_path, 'w', encoding='utf-8') as f:
                json.dump(report, f, ensure_ascii=False, indent=2)

            self._save_markdown_report(report)
        except Exception as e:
            rospy.logwarn_throttle(5.0, '写入探测报告失败: %s', e)

    def _save_markdown_report(self, report):
        try:
            out_dir = os.path.dirname(self.markdown_report_path)
            if out_dir:
                os.makedirs(out_dir, exist_ok=True)

            clusters = report.get('targets_clustered', [])
            stamp = float(report.get('stamp', rospy.get_time()))
            time_text = datetime.fromtimestamp(stamp).strftime('%Y-%m-%d %H:%M:%S')
            summary = report.get('summary', {})

            lines = []
            lines.append('# Patrol Report')
            lines.append('')
            lines.append('- Generated at: %s' % time_text)
            lines.append('- Total clustered targets: %d' % int(summary.get('total', len(clusters))))
            lines.append('- By color: red=%d, green=%d, yellow=%d, blue=%d' % (
                int(summary.get('red', 0)),
                int(summary.get('green', 0)),
                int(summary.get('yellow', 0)),
                int(summary.get('blue', 0)),
            ))
            lines.append('')
            lines.append('## Objects')
            lines.append('')
            for c in clusters:
                cid = int(c.get('id', 0))
                color = str(c.get('color', 'unknown'))
                cx = float(c.get('x', 0.0))
                cy = float(c.get('y', 0.0))
                cz = float(c.get('z', self.target_z))
                cnt = int(c.get('points_count', 0))
                lines.append('### Object %d' % cid)
                lines.append('- Color: %s' % color)
                lines.append('- Center: (%.2f, %.2f, %.2f)' % (cx, cy, cz))
                lines.append('- Points: %d' % cnt)
                lines.append('- Raw detections:')
                members = c.get('members', [])
                for i, m in enumerate(members, 1):
                    lines.append('  - %d) (%.2f, %.2f, %.2f)' % (
                        i,
                        float(m.get('x', 0.0)),
                        float(m.get('y', 0.0)),
                        float(m.get('z', self.target_z)),
                    ))
                lines.append('')

            with open(self.markdown_report_path, 'w', encoding='utf-8') as f:
                f.write('\n'.join(lines) + '\n')
        except Exception as e:
            rospy.logwarn_throttle(5.0, '写入Markdown报告失败: %s', e)

    def _on_shutdown(self):
        """Stop robot and flush final target report on node shutdown."""
        self._stop_robot()
        try:
            report = self._build_target_report()
            self._save_report_to_file(report)
            self._print_final_report(report)
            rospy.loginfo('最终探测报告已保存: %s', self.report_output_path)
            rospy.loginfo('最终巡逻Markdown报告已保存: %s', self.markdown_report_path)
        except Exception:
            pass

    def _cluster_color_targets(self, color, points):
        if not points:
            return []

        clusters = []
        visited = [False] * len(points)

        for i in range(len(points)):
            if visited[i]:
                continue

            q = deque([i])
            visited[i] = True
            group = []

            while q:
                cur = q.popleft()
                group.append(cur)
                x1 = float(points[cur].get('x', 0.0))
                y1 = float(points[cur].get('y', 0.0))

                for j in range(len(points)):
                    if visited[j]:
                        continue
                    x2 = float(points[j].get('x', 0.0))
                    y2 = float(points[j].get('y', 0.0))
                    if math.hypot(x1 - x2, y1 - y2) <= self.cluster_radius:
                        visited[j] = True
                        q.append(j)

            group_pts = [points[idx] for idx in group]
            cx = sum(float(p.get('x', 0.0)) for p in group_pts) / float(len(group_pts))
            cy = sum(float(p.get('y', 0.0)) for p in group_pts) / float(len(group_pts))
            cz = sum(float(p.get('z', self.target_z)) for p in group_pts) / float(len(group_pts))
            ts_first = min(float(p.get('ts', 0.0)) for p in group_pts)
            ts_last = max(float(p.get('ts', 0.0)) for p in group_pts)

            clusters.append({
                'color': color,
                'x': float(cx),
                'y': float(cy),
                'z': float(cz),
                'points_count': int(len(group_pts)),
                'first_seen': float(ts_first),
                'last_seen': float(ts_last),
                'members': group_pts,
            })

        return clusters

    def _cluster_all_targets(self):
        all_clusters = []
        for color in ('red', 'green', 'yellow', 'blue'):
            pts = self.detected_targets.get(color, [])
            all_clusters.extend(self._cluster_color_targets(color, pts))

        # 第二阶段：同色簇中心再合并，进一步消除单次观测抖动导致的“碎簇”
        all_clusters = self._merge_clusters_by_center(all_clusters)

        all_clusters.sort(key=lambda c: c.get('first_seen', 0.0))
        for idx, c in enumerate(all_clusters, 1):
            c['id'] = idx
        return all_clusters

    def _merge_clusters_by_center(self, clusters):
        if not clusters:
            return []

        out = []
        used = [False] * len(clusters)
        for i in range(len(clusters)):
            if used[i]:
                continue

            base = clusters[i]
            color = base.get('color', 'unknown')
            members = list(base.get('members', []))
            used[i] = True

            changed = True
            while changed:
                changed = False
                cx = sum(float(m.get('x', 0.0)) for m in members) / float(max(1, len(members)))
                cy = sum(float(m.get('y', 0.0)) for m in members) / float(max(1, len(members)))

                for j in range(len(clusters)):
                    if used[j]:
                        continue
                    c = clusters[j]
                    if c.get('color', 'unknown') != color:
                        continue
                    jx = float(c.get('x', 0.0))
                    jy = float(c.get('y', 0.0))
                    ci = self._cluster_span_radius(members)
                    cj = self._cluster_span_radius(c.get('members', []))
                    adaptive_merge = self.cluster_merge_radius + 0.6 * max(ci, cj)
                    if math.hypot(cx - jx, cy - jy) <= adaptive_merge:
                        used[j] = True
                        members.extend(c.get('members', []))
                        changed = True

            cx = sum(float(m.get('x', 0.0)) for m in members) / float(max(1, len(members)))
            cy = sum(float(m.get('y', 0.0)) for m in members) / float(max(1, len(members)))
            cz = sum(float(m.get('z', self.target_z)) for m in members) / float(max(1, len(members)))
            ts_first = min(float(m.get('ts', 0.0)) for m in members) if members else 0.0
            ts_last = max(float(m.get('ts', 0.0)) for m in members) if members else 0.0

            out.append({
                'color': color,
                'x': float(cx),
                'y': float(cy),
                'z': float(cz),
                'points_count': int(len(members)),
                'first_seen': float(ts_first),
                'last_seen': float(ts_last),
                'members': members,
            })

        return out

    def _cluster_span_radius(self, members):
        if not members:
            return 0.0
        cx = sum(float(m.get('x', 0.0)) for m in members) / float(len(members))
        cy = sum(float(m.get('y', 0.0)) for m in members) / float(len(members))
        r = 0.0
        for m in members:
            r = max(r, math.hypot(float(m.get('x', 0.0)) - cx, float(m.get('y', 0.0)) - cy))
        return r

    def _print_final_report(self, report=None):
        if self._final_report_printed:
            return
        self._final_report_printed = True

        if report is None:
            report = self._build_target_report()

        all_targets = report.get('targets_clustered', [])
        rospy.loginfo("=== Patrol Report ===")
        rospy.loginfo("Number of targets detected: %d", len(all_targets))
        rospy.loginfo("List of target locations:")
        for idx, t in enumerate(all_targets, 1):
            rospy.loginfo("  %d. color=%s (%.2f, %.2f, %.2f)", idx, t['color'], t['x'], t['y'], t['z'])

    def _capture_home_pose_once(self):
        if self.home_pose is not None:
            return
        if not self.odom_received:
            return

        px, py = self._planning_pose_xy()
        pyaw = self._planning_yaw()
        self.home_pose = (float(px), float(py), float(pyaw))
        rospy.loginfo('已记录起点位姿: (%.2f, %.2f, %.2f)', self.home_pose[0], self.home_pose[1], self.home_pose[2])

    def _update_unknown_history(self):
        if self.map_data is None:
            return
        now = rospy.get_time()
        unknown_cells = int(np.count_nonzero(self.map_data == -1))
        self.unknown_history.append((now, unknown_cells))

        while self.unknown_history and (now - self.unknown_history[0][0]) > self.unknown_stable_window_sec:
            self.unknown_history.popleft()

    def _update_loop_history(self):
        px, py = self._planning_pose_xy()
        if not self.loop_pose_hist:
            self.loop_pose_hist.append((float(px), float(py)))
            self.loop_cumlen_hist.append(0.0)
            self.loop_path_total = 0.0
            return

        lx, ly = self.loop_pose_hist[-1]
        step = math.hypot(float(px) - lx, float(py) - ly)
        if step < self.loop_sample_step:
            return

        self.loop_path_total += step
        self.loop_pose_hist.append((float(px), float(py)))
        self.loop_cumlen_hist.append(float(self.loop_path_total))

    def _bbox_known_ratio(self, min_x, max_x, min_y, max_y):
        if self.map_data is None or self.map_info is None:
            return 0.0
        p0 = self.world_to_grid(min_x, min_y)
        p1 = self.world_to_grid(max_x, max_y)
        if p0 is None or p1 is None:
            return 0.0

        gx0, gy0 = p0
        gx1, gy1 = p1
        x0, x1 = min(gx0, gx1), max(gx0, gx1)
        y0, y1 = min(gy0, gy1), max(gy0, gy1)
        if x1 <= x0 or y1 <= y0:
            return 0.0

        patch = self.map_data[y0:y1 + 1, x0:x1 + 1]
        total = patch.size
        if total <= 0:
            return 0.0
        known = int(np.count_nonzero(patch != -1))
        return float(known) / float(total)

    def _should_start_return_by_loop(self):
        if (not self.enable_loop_return) or (self.map_data is None):
            return False
        now = rospy.get_time()
        if (now - self.start_time) < self.unknown_stable_min_runtime_sec:
            return False
        if self.frontier_missing_since <= 0.0:
            return False
        if (now - self.frontier_missing_since) < self.loop_no_frontier_sec:
            return False

        n = len(self.loop_pose_hist)
        if n < (self.loop_min_points_gap + 2):
            return False

        end_idx = n - 1
        ex, ey = self.loop_pose_hist[end_idx]
        close_idx = -1

        for i in range(0, end_idx - self.loop_min_points_gap):
            ix, iy = self.loop_pose_hist[i]
            if math.hypot(ex - ix, ey - iy) > self.loop_close_dist:
                continue
            path_len = self.loop_cumlen_hist[end_idx] - self.loop_cumlen_hist[i]
            if path_len >= self.loop_min_path_len:
                close_idx = i
                break

        if close_idx < 0:
            return False

        loop_pts = list(self.loop_pose_hist)[close_idx:end_idx + 1]
        xs = [p[0] for p in loop_pts]
        ys = [p[1] for p in loop_pts]
        min_x, max_x = min(xs), max(xs)
        min_y, max_y = min(ys), max(ys)
        bbox_area = max(0.0, (max_x - min_x) * (max_y - min_y))
        if bbox_area < self.loop_bbox_min_area:
            return False

        known_ratio = self._bbox_known_ratio(min_x, max_x, min_y, max_y)
        if known_ratio < self.loop_known_ratio_thresh:
            return False

        if not self._loop_return_logged:
            rospy.loginfo('闭合回路返航触发: known_ratio=%.2f bbox_area=%.2f', known_ratio, bbox_area)
            self._loop_return_logged = True
        return True

    def _should_start_return(self):
        if self._should_start_return_by_loop():
            return True
        if (rospy.get_time() - self.start_time) < self.unknown_stable_min_runtime_sec:
            return False
        if len(self.unknown_history) < 2:
            return False
        if self.frontier_missing_since <= 0.0:
            return False
        if (rospy.get_time() - self.frontier_missing_since) < self.return_requires_no_frontier_sec:
            return False

        oldest_t, oldest_u = self.unknown_history[0]
        newest_t, newest_u = self.unknown_history[-1]
        if (newest_t - oldest_t) < self.unknown_stable_window_sec * 0.9:
            return False

        reduced = int(oldest_u - newest_u)
        return reduced <= self.unknown_stable_delta_cells

    def _navigate_return_home(self):
        if self.home_pose is None:
            self._capture_home_pose_once()
        if self.home_pose is None:
            return False

        hx, hy, hyaw = self.home_pose
        self.goal_tolerance = self.home_reach_tol
        reached = self.drive_to_target((hx, hy))
        if not reached:
            return False

        yaw_err = self._normalize_angle(hyaw - self._planning_yaw())
        if abs(yaw_err) <= self.home_yaw_tol:
            self.vel_pub.publish(Twist())
            return True

        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = max(-0.7, min(0.7, 1.6 * yaw_err))
        self.vel_pub.publish(twist)
        return False

    def scan_callback(self, data):
        self.scan_msg = data
        self.laser_ranges = data.ranges

    def _normalize_angle(self, angle):
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle

    def _sanitize_range(self, r):
        if r is None:
            return float('inf')
        if (not math.isfinite(r)) or r <= 0.0:
            return float('inf')
        return r

    def get_min_range(self, angle_center_rad, angle_width_rad):
        """取某个角度扇区内的最小有效距离（更稳健，避免固定索引导致的越界/分辨率差异）。"""
        if self.scan_msg is None or not self.scan_msg.ranges:
            return float('inf')

        angle_min = self.scan_msg.angle_min
        angle_inc = self.scan_msg.angle_increment
        if angle_inc == 0.0:
            return float('inf')

        center_idx = int(round((angle_center_rad - angle_min) / angle_inc))
        half_span = int(max(0, round((angle_width_rad / 2.0) / angle_inc)))

        start_idx = max(0, center_idx - half_span)
        end_idx = min(len(self.scan_msg.ranges) - 1, center_idx + half_span)

        min_r = float('inf')
        for idx in range(start_idx, end_idx + 1):
            min_r = min(min_r, self._sanitize_range(self.scan_msg.ranges[idx]))
        return min_r

    def odom_callback(self, data):
        self.current_pos.x = data.pose.pose.position.x
        self.current_pos.y = data.pose.pose.position.y
        quat = data.pose.pose.orientation
        quat_list = [quat.x, quat.y, quat.z, quat.w]
        _, _, yaw = euler_from_quaternion(quat_list)
        self.current_yaw = yaw
        self.odom_received = True

    def _update_map_pose_from_tf(self):
        """尝试获取 map->base_* 位姿；失败时回退到/odom位姿。"""
        base_candidates = ("base_footprint", "base_link")
        for base in base_candidates:
            try:
                trans = self.tf_buffer.lookup_transform("map", base, rospy.Time(0), rospy.Duration(0.05))
                self.map_pos_x = float(trans.transform.translation.x)
                self.map_pos_y = float(trans.transform.translation.y)
                q = trans.transform.rotation
                _, _, yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])
                self.map_yaw = float(yaw)
                self.map_pose_valid = True
                return
            except Exception:
                continue
        self.map_pose_valid = False

    def _planning_pose_xy(self):
        if self.map_pose_valid:
            return (self.map_pos_x, self.map_pos_y)
        return (float(self.current_pos.x), float(self.current_pos.y))

    def _planning_yaw(self):
        if self.map_pose_valid:
            return float(self.map_yaw)
        return float(self.current_yaw)

    def _prune_blacklist(self, now):
        while self.frontier_blacklist and (now - self.frontier_blacklist[0][2]) > self.blacklist_ttl_sec:
            self.frontier_blacklist.popleft()

    def _is_blacklisted(self, target_point, now):
        self._prune_blacklist(now)
        tx, ty = float(target_point[0]), float(target_point[1])
        for bx, by, _ in self.frontier_blacklist:
            if math.hypot(tx - bx, ty - by) < self.blacklist_radius:
                return True
        return False

    def _blacklist_target(self, target_point, reason):
        if target_point is None:
            return
        now = rospy.get_time()
        tx, ty = float(target_point[0]), float(target_point[1])
        if self._is_blacklisted((tx, ty), now):
            return
        self.frontier_blacklist.append((tx, ty, now))
        rospy.logwarn("目标加入黑名单(%.2f, %.2f): %s", tx, ty, reason)

    def _blacklist_pose(self, x, y, reason):
        now = rospy.get_time()
        tx, ty = float(x), float(y)
        if self._is_blacklisted((tx, ty), now):
            return
        self.frontier_blacklist.append((tx, ty, now))
        rospy.logwarn("位置加入黑名单(%.2f, %.2f): %s", tx, ty, reason)

    def _stop_robot(self):
        """节点退出时发零速，避免Gazebo保持最后一次速度命令。"""
        stop = Twist()
        try:
            for _ in range(3):
                self.vel_pub.publish(stop)
        except Exception:
            pass

    def avoid_obstacle(self):
        twist = Twist()
        front_range = self.get_min_range(0.0, math.radians(30.0))
        left_range = self.get_min_range(math.pi / 2.0, math.radians(30.0))
        right_range = self.get_min_range(-math.pi / 2.0, math.radians(30.0))

        if front_range < self.safe_distance:
            twist.linear.x = 0.0
            twist.angular.z = self.turn_dir * 0.9
            if left_range < right_range:
                self.turn_dir = 1.0
            else:
                self.turn_dir = -1.0
        else:
            twist.linear.x = 0.24
            twist.angular.z = 0.0
        self.vel_pub.publish(twist)

    def map_callback(self, msg):
        self.map_info = msg.info
        self.map_data = np.array(msg.data, dtype=np.int16).reshape((self.map_info.height, self.map_info.width))

    def grid_to_world(self, grid_x, grid_y):
        if self.map_info is None:
            return None
        res = self.map_info.resolution
        world_x = self.map_info.origin.position.x + (grid_x + 0.5) * res
        world_y = self.map_info.origin.position.y + (grid_y + 0.5) * res
        return (world_x, world_y)

    def world_to_grid(self, world_x, world_y):
        if self.map_info is None:
            return None
        res = self.map_info.resolution
        gx = int(math.floor((world_x - self.map_info.origin.position.x) / res))
        gy = int(math.floor((world_y - self.map_info.origin.position.y) / res))
        if gx < 0 or gy < 0 or gx >= self.map_info.width or gy >= self.map_info.height:
            return None
        return (gx, gy)

    def _is_free(self, gx, gy):
        return int(self.map_data[gy][gx]) == 0

    def _is_free_mask(self, gx, gy, free_mask):
        return bool(free_mask[gy][gx])

    def _is_unknown(self, gx, gy):
        return int(self.map_data[gy][gx]) == -1

    def _has_unknown_neighbor(self, gx, gy):
        for dy in (-1, 0, 1):
            for dx in (-1, 0, 1):
                if dx == 0 and dy == 0:
                    continue
                nx, ny = gx + dx, gy + dy
                if nx < 0 or ny < 0 or nx >= self.map_info.width or ny >= self.map_info.height:
                    continue
                if self._is_unknown(nx, ny):
                    return True
        return False

    def _nearest_free_seed(self, gx, gy, max_radius=10):
        if gx is None or gy is None:
            return None
        if self._is_free(gx, gy):
            return (gx, gy)
        for r in range(1, max_radius + 1):
            for dy in range(-r, r + 1):
                for dx in range(-r, r + 1):
                    if abs(dx) != r and abs(dy) != r:
                        continue
                    nx, ny = gx + dx, gy + dy
                    if nx < 0 or ny < 0 or nx >= self.map_info.width or ny >= self.map_info.height:
                        continue
                    if self._is_free(nx, ny):
                        return (nx, ny)
        return None

    def _nearest_free_seed_mask(self, gx, gy, free_mask, max_radius=10):
        if gx is None or gy is None:
            return None
        if self._is_free_mask(gx, gy, free_mask):
            return (gx, gy)
        for r in range(1, max_radius + 1):
            for dy in range(-r, r + 1):
                for dx in range(-r, r + 1):
                    if abs(dx) != r and abs(dy) != r:
                        continue
                    nx, ny = gx + dx, gy + dy
                    if nx < 0 or ny < 0 or nx >= self.map_info.width or ny >= self.map_info.height:
                        continue
                    if self._is_free_mask(nx, ny, free_mask):
                        return (nx, ny)
        return None

    def _ensure_inflate_offsets(self):
        if self.map_info is None or self.map_info.resolution <= 0.0:
            return
        radius_cells = int(math.ceil((self.robot_radius_m + self.clearance_m) / self.map_info.resolution))
        if radius_cells == self._inflate_radius_cells and self._inflate_offsets:
            return

        self._inflate_radius_cells = radius_cells
        offsets = []
        rr2 = radius_cells * radius_cells
        for dy in range(-radius_cells, radius_cells + 1):
            for dx in range(-radius_cells, radius_cells + 1):
                if (dx * dx + dy * dy) <= rr2:
                    offsets.append((dx, dy))
        self._inflate_offsets = offsets

    def _build_free_mask(self):
        """构建膨胀后可通行栅格：只允许足够远离障碍的free单元通过。"""
        if self.map_data is None or self.map_info is None:
            return None

        self._ensure_inflate_offsets()

        free_mask = (self.map_data == 0)
        occ = np.argwhere(self.map_data > 50)
        if occ.size == 0 or not self._inflate_offsets:
            return free_mask

        h, w = free_mask.shape
        for oy, ox in occ:
            for dx, dy in self._inflate_offsets:
                nx = int(ox + dx)
                ny = int(oy + dy)
                if 0 <= nx < w and 0 <= ny < h:
                    free_mask[ny, nx] = False
        return free_mask

    def _is_recent_frontier(self, world_point):
        if world_point is None:
            return False
        tx, ty = float(world_point[0]), float(world_point[1])
        for fx, fy in self.recent_frontiers:
            if math.hypot(tx - fx, ty - fy) < self.revisit_radius:
                return True
        return False

    def _reachable_frontiers(self):
        """BFS遍历可达free区域，返回frontier集合、父节点树、距离表、seed。"""
        if self.map_data is None or self.map_info is None:
            return None, None, None, None

        free_mask = self._build_free_mask()
        if free_mask is None:
            return None, None, None, None

        px, py = self._planning_pose_xy()
        start = self.world_to_grid(px, py)
        if start is None:
            return None, None, None, None

        seed = self._nearest_free_seed_mask(start[0], start[1], free_mask, max_radius=20)
        if seed is None:
            return None, None, None, None

        q = deque([seed])
        parent = {seed: None}
        dist = {seed: 0}
        frontiers = set()

        while q:
            gx, gy = q.popleft()
            if self._has_unknown_neighbor(gx, gy):
                frontiers.add((gx, gy))

            for dx, dy in ((1, 0), (-1, 0), (0, 1), (0, -1)):
                nx, ny = gx + dx, gy + dy
                if nx < 0 or ny < 0 or nx >= self.map_info.width or ny >= self.map_info.height:
                    continue
                if (nx, ny) in parent:
                    continue
                if self._is_free_mask(nx, ny, free_mask):
                    parent[(nx, ny)] = (gx, gy)
                    dist[(nx, ny)] = dist[(gx, gy)] + 1
                    q.append((nx, ny))

        return frontiers, parent, dist, seed

    def _cluster_frontiers(self, frontier_cells):
        if not frontier_cells:
            return []
        remaining = set(frontier_cells)
        clusters = []
        while remaining:
            root = remaining.pop()
            q = deque([root])
            cluster = [root]
            while q:
                cx, cy = q.popleft()
                for dx in (-1, 0, 1):
                    for dy in (-1, 0, 1):
                        if dx == 0 and dy == 0:
                            continue
                        nb = (cx + dx, cy + dy)
                        if nb in remaining:
                            remaining.remove(nb)
                            q.append(nb)
                            cluster.append(nb)
            clusters.append(cluster)
        return clusters

    def _path_from_parent(self, parent, goal_cell):
        path = []
        cur = goal_cell
        while cur is not None:
            path.append(cur)
            cur = parent.get(cur)
        path.reverse()
        return path

    def _path_to_waypoints(self, grid_path):
        if not grid_path:
            return []
        waypoints = []
        for i in range(0, len(grid_path), self.waypoint_step):
            wp = self.grid_to_world(grid_path[i][0], grid_path[i][1])
            if wp is not None:
                waypoints.append(wp)
        end_wp = self.grid_to_world(grid_path[-1][0], grid_path[-1][1])
        if end_wp is not None:
            if not waypoints:
                waypoints.append(end_wp)
            else:
                last = waypoints[-1]
                if math.hypot(last[0] - end_wp[0], last[1] - end_wp[1]) > 1e-3:
                    waypoints.append(end_wp)
        return waypoints

    def _plan_frontier_path(self):
        """选择最优可达frontier簇并返回(frontier_world, waypoints)。"""
        if self.map_data is None or self.map_info is None:
            return None, None

        frontiers, parent, dist, _ = self._reachable_frontiers()
        if frontiers is None:
            return None, None
        if not frontiers:
            return None, None

        clusters = self._cluster_frontiers(frontiers)
        if not clusters:
            return None, None

        usable_clusters = [c for c in clusters if len(c) >= self.frontier_min_cluster]
        if not usable_clusters:
            usable_clusters = clusters

        best = None
        best_score = -1e18
        now = rospy.get_time()
        candidates = []

        for cluster in usable_clusters:
            rep = min(cluster, key=lambda c: dist.get(c, 10**9))
            d = float(dist.get(rep, 10**9))
            rep_world = self.grid_to_world(rep[0], rep[1])
            if rep_world is None:
                continue
            # 黑名单目标直接禁选，避免反复选回同一区域
            if self._is_blacklisted(rep_world, now):
                continue
            # 大簇优先、距离次优先；最近访问过的区域降权
            revisit_penalty = 8.0 if self._is_recent_frontier(rep_world) else 0.0
            score = (2.5 * len(cluster)) - (0.18 * d) - revisit_penalty
            candidates.append((score, rep, rep_world))

            if score > best_score:
                best_score = score
                best = rep

        if not candidates:
            return None, None

        candidates.sort(key=lambda x: x[0], reverse=True)

        # 第一轮：严格跳过最近区域；第二轮：放宽，避免无目标可选
        for strict_recent in (True, False):
            for _, rep, rep_world in candidates:
                if strict_recent and self._is_recent_frontier(rep_world):
                    continue

                path_grid = self._path_from_parent(parent, rep)
                if len(path_grid) < self.min_path_cells and len(candidates) > 1:
                    continue

                waypoints = self._path_to_waypoints(path_grid)
                if len(waypoints) < 2 and len(candidates) > 1:
                    continue

                return rep_world, waypoints

        return None, None

    def _check_and_handle_stuck(self):
        now = rospy.get_time()
        px, py = self._planning_pose_xy()
        self._pose_hist.append((now, px, py))
        if len(self._pose_hist) < self._pose_hist.maxlen:
            return False

        t0, x0, y0 = self._pose_hist[0]
        t1, x1, y1 = self._pose_hist[-1]
        if (t1 - t0) < self._stuck_time_sec:
            return False

        moved = math.hypot(x1 - x0, y1 - y0)
        if moved < self._stuck_dist_eps and now >= self._recovery_until:
            # 进入短暂恢复：原地旋转，改变turn_dir避免重复
            self.turn_dir *= -1.0
            self._recovery_until = now + 2.0
            self._stuck_event_count += 1
            rospy.logwarn("检测到疑似卡死(%.3fm/%.1fs)，进入恢复旋转", moved, (t1 - t0))
            return True
        return False

    def _update_progress(self):
        now = rospy.get_time()
        px, py = self._planning_pose_xy()
        if self._last_progress_pose is None:
            self._last_progress_pose = (px, py)
            self._last_progress_time = now
            return
        moved = math.hypot(px - self._last_progress_pose[0], py - self._last_progress_pose[1])
        if moved > self.progress_eps:
            self._last_progress_pose = (px, py)
            self._last_progress_time = now

    def _no_progress(self):
        if self._last_progress_time <= 0.0:
            return False
        return (rospy.get_time() - self._last_progress_time) > self.no_progress_timeout

    def _publish_recovery(self):
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = self.turn_dir * 0.7
        self.vel_pub.publish(twist)

    def _publish_escape_motion(self):
        """强制脱困动作：短暂后退并转向，脱离局部循环。"""
        twist = Twist()
        # 先后退再前探，配合较大角速度离开局部势阱
        phase = int(rospy.get_time() * 2.0) % 4
        if phase in (0, 1):
            twist.linear.x = -0.06
        else:
            twist.linear.x = 0.05
        twist.angular.z = self.turn_dir * 1.0
        self.vel_pub.publish(twist)

    def explore_without_map(self):
        """无/map时的保底探索：右手沿墙 + 遇障转向。"""
        # 右侧与前方距离
        front = self.get_min_range(0.0, math.radians(25.0))
        right = self.get_min_range(-math.pi / 2.0, math.radians(35.0))
        front_right = self.get_min_range(-math.pi / 4.0, math.radians(35.0))

        twist = Twist()
        if front < self.safe_distance:
            twist.linear.x = 0.0
            twist.angular.z = 0.8
        else:
            twist.linear.x = self.wall_linear
            # 没墙时倾向右转去“找墙”
            if not math.isfinite(right) or right == float('inf'):
                twist.angular.z = -0.4
            else:
                err = self.wall_dist - right
                twist.angular.z = max(-self.max_angular, min(self.max_angular, -self.wall_kp * err))
            # 如果右前太近，稍微左拐
            if front_right < (self.wall_dist * 0.9):
                twist.angular.z = max(twist.angular.z, 0.4)

        self.vel_pub.publish(twist)

    def _goal_bearing_and_dist(self, target_point):
        tx, ty = float(target_point[0]), float(target_point[1])
        px, py = self._planning_pose_xy()
        dx = tx - px
        dy = ty - py
        dist = math.hypot(dx, dy)
        desired_yaw = math.atan2(dy, dx)
        yaw = self._planning_yaw()
        bearing = self._normalize_angle(desired_yaw - yaw)
        return bearing, dist

    def _is_goal_direction_clear(self, bearing, dist):
        # 用指向目标方向的扇区，判断是否“近似直线可达”
        r = self.get_min_range(bearing, math.radians(12.0))
        if not math.isfinite(r):
            return True
        return r > max(0.0, dist - 0.15)

    def navigate_bug_to_target(self, target_point):
        if target_point is None:
            return False

        bearing, dist = self._goal_bearing_and_dist(target_point)
        if dist < self.goal_tolerance:
            stop = Twist()
            self.vel_pub.publish(stop)
            self.nav_state = "GOTO"
            self._wall_hit_dist = None
            return True

        front = self.get_min_range(0.0, math.radians(25.0))

        now = rospy.get_time()
        if self.nav_state == "GOTO":
            if front < self.safe_distance:
                self.nav_state = "WALL"
                self._wall_hit_dist = dist
                self._wall_follow_enter_time = now
                self.wall_enter_count += 1
                if (not self.return_mode) and (self.current_frontier is not None) and self.wall_enter_count >= self.wall_enter_limit:
                    rospy.logwarn("同一目标多次触发WALL，加入黑名单并切换目标")
                    self.turn_dir *= -1.0
                    self._blacklist_target(self.current_frontier, "多次触发WALL")
                    px, py = self._planning_pose_xy()
                    self._blacklist_pose(px, py, "WALL频发当前位置")
                    self.current_frontier = None
                    self.path_waypoints = []
                    self.wp_index = 0
                    self.force_escape_until = now + 3.5
                    self.nav_state = "GOTO"
                    self._wall_hit_dist = None
                    self.wall_enter_count = 0
                    return False
            else:
                twist = Twist()
                k_ang = 1.4
                twist.angular.z = max(-self.max_angular, min(self.max_angular, k_ang * bearing))
                if abs(bearing) > self.yaw_tolerance:
                    twist.linear.x = 0.0
                else:
                    # 前方更空旷时加速，靠近障碍时自动降速
                    if front > 1.0:
                        twist.linear.x = min(self.max_linear, 0.30)
                    elif front > 0.7:
                        twist.linear.x = min(self.max_linear, 0.24)
                    else:
                        twist.linear.x = min(self.max_linear, 0.16)
                self.vel_pub.publish(twist)
                return False

        right = self.get_min_range(-math.pi / 2.0, math.radians(35.0))
        front_right = self.get_min_range(-math.pi / 4.0, math.radians(35.0))

        twist = Twist()
        if front < self.safe_distance:
            twist.linear.x = 0.0
            twist.angular.z = 1.1
        else:
            twist.linear.x = self.wall_linear
            if not math.isfinite(right) or right == float('inf'):
                twist.angular.z = -0.6
            else:
                err = self.wall_dist - right
                twist.angular.z = max(-self.max_angular, min(self.max_angular, -self.wall_kp * err))
            if front_right < (self.wall_dist * 0.9):
                twist.angular.z = max(twist.angular.z, 0.8)

        self.vel_pub.publish(twist)

        # 墙跟随过久且没有实质逼近目标：判定为局部绕圈，触发脱困与换目标
        if self.nav_state == "WALL" and self._wall_hit_dist is not None:
            wall_elapsed = now - self._wall_follow_enter_time
            if wall_elapsed > self.wall_follow_timeout_sec:
                rospy.logwarn("墙跟随超时且无明显进展，触发脱困重规划")
                self.turn_dir *= -1.0
                self.force_escape_until = now + 3.0
                self.nav_state = "GOTO"
                self._wall_hit_dist = None
                self.wall_enter_count = 0
                if (not self.return_mode) and (self.current_frontier is not None):
                    self._blacklist_target(self.current_frontier, "WALL超时绕圈")
                    px, py = self._planning_pose_xy()
                    self._blacklist_pose(px, py, "WALL超时当前位置")
                    self.current_frontier = None
                    self.path_waypoints = []
                    self.wp_index = 0
                    self.local_minima_count += 1
                    self.local_minima_until = now + min(6.0, 2.0 + 0.8 * self.local_minima_count)
                return False

        if self._wall_hit_dist is not None:
            clear = self._is_goal_direction_clear(bearing, dist)
            improving = dist < (self._wall_hit_dist - 0.10)
            if (now - self._wall_follow_enter_time) > 1.0 and clear and improving and front > (self.safe_distance + 0.05):
                self.nav_state = "GOTO"
                self._wall_hit_dist = None
                self.wall_enter_count = 0

        return False

    def drive_to_target(self, target_point):
        return self.navigate_bug_to_target(target_point)

    def run(self):
        rospy.loginfo("=== Patrol task started ===")
        while not rospy.is_shutdown():
            self._update_map_pose_from_tf()
            self._update_progress()
            self._update_loop_history()
            self._capture_home_pose_once()

            if self.scan_msg is None or len(self.laser_ranges) == 0:
                rospy.logwarn_throttle(2.0, "等待激光雷达数据...")
                self.rate.sleep()
                continue

            if not self.odom_received:
                rospy.logwarn_throttle(2.0, "等待/odom数据...")
                self.rate.sleep()
                continue

            now = rospy.get_time()
            if now < self._recovery_until:
                self._publish_recovery()
                self.rate.sleep()
                continue

            if now < self.force_escape_until:
                self._publish_escape_motion()
                self.rate.sleep()
                continue

            if now < self.local_minima_until:
                self._publish_escape_motion()
                self.rate.sleep()
                continue

            if self.map_data is None or self.map_info is None:
                rospy.logwarn_throttle(2.0, "等待/map数据...（地图未就绪时不会判定探索完成）")
                self.explore_without_map()
                self._check_and_handle_stuck()
                self.rate.sleep()
                continue

            self._update_unknown_history()

            if (not self.return_mode) and self._should_start_return():
                self.return_mode = True
                self.path_waypoints = []
                self.current_frontier = None
                self.wp_index = 0
                self.nav_state = "GOTO"
                self._wall_hit_dist = None
                rospy.loginfo('未知区域在较长时间内几乎不再变化，开始返航到起点...')

            if self.return_mode and (not self.return_completed):
                done = self._navigate_return_home()
                if done:
                    self.return_completed = True
                    self.vel_pub.publish(Twist())
                    rospy.loginfo('巡逻任务圆满完成')
                    report = self._build_target_report()
                    self._save_report_to_file(report)
                    self._print_final_report(report)
                    break
                self.rate.sleep()
                continue

            has_unknown = bool(np.any(self.map_data == -1))
            if has_unknown:
                self.no_unknown_count = 0
            else:
                self.no_unknown_count += 1

            if self.no_unknown_count >= self.no_unknown_need:
                self.vel_pub.publish(Twist())
                rospy.loginfo("Returning to start...")
                self._print_final_report()
                rospy.loginfo("探索完成，小车已停止")
                break

            need_replan = (not self.path_waypoints) or (self.wp_index >= len(self.path_waypoints))
            # 仅在无路径/无进展时重规划，避免目标频繁跳变造成局部振荡
            need_replan = need_replan or (((now - self.last_plan_time) > self.replan_interval) and self.current_frontier is None)
            need_replan = need_replan or self._no_progress()

            if self._no_progress() and self.current_frontier is not None and (now - self.last_forced_switch_time) > self.forced_switch_cooldown:
                self._blacklist_target(self.current_frontier, "长时间无进展，强制换目标")
                self.last_forced_switch_time = now
                self.current_frontier = None
                self.path_waypoints = []
                self.wp_index = 0
                self.nav_state = "GOTO"
                self._wall_hit_dist = None
                self.wall_enter_count = 0
                self.force_escape_until = now + 2.0
                self._last_progress_pose = self._planning_pose_xy()
                self._last_progress_time = now
                # 不在这里清零stuck计数，让连续卡死策略保持敏感

            if need_replan:
                frontier_world, waypoints = self._plan_frontier_path()
                self.last_plan_time = now

                if frontier_world is not None and waypoints:
                    self.frontier_missing_since = 0.0
                    old_frontier = self.current_frontier
                    self.current_frontier = frontier_world
                    self.recent_frontiers.append((frontier_world[0], frontier_world[1]))
                    self.path_waypoints = waypoints
                    self.wp_index = 0
                    self.nav_state = "GOTO"
                    self.wall_enter_count = 0
                    # 只有目标明显改变时才重置卡死计数，避免同一目标无限重试
                    if old_frontier is None or math.hypot(old_frontier[0] - frontier_world[0], old_frontier[1] - frontier_world[1]) > 0.35:
                        self._stuck_event_count = 0
                    rospy.loginfo_throttle(1.0, "选择frontier目标：(%.2f, %.2f), 路径点:%d",
                                           frontier_world[0], frontier_world[1], len(waypoints))
                else:
                    if has_unknown:
                        if self.frontier_missing_since <= 0.0:
                            self.frontier_missing_since = now
                        # 没有可规划frontier时持续探索，避免提前停机
                        self.explore_without_map()
                        stuck_event = self._check_and_handle_stuck()
                        if stuck_event and self.current_frontier is not None and self._stuck_event_count >= self.max_stuck_before_replan:
                            self._blacklist_target(self.current_frontier, "无可规划路径且连续卡死")
                            self.current_frontier = None
                            self.path_waypoints = []
                            self.wp_index = 0
                            self._stuck_event_count = 0
                        self.rate.sleep()
                        continue

            if not self.path_waypoints or self.wp_index >= len(self.path_waypoints):
                if has_unknown and self.frontier_missing_since <= 0.0:
                    self.frontier_missing_since = now
                self.explore_without_map()
                self._check_and_handle_stuck()
                self.rate.sleep()
                continue

            target_wp = self.path_waypoints[self.wp_index]
            reached = self.drive_to_target(target_wp)
            if reached:
                self.wp_index += 1
                self._stuck_event_count = 0
                if self.wp_index >= len(self.path_waypoints):
                    self.current_frontier = None
                    self.path_waypoints = []
                    self.wp_index = 0
                    self._stuck_event_count = 0

            stuck_event = self._check_and_handle_stuck()
            if stuck_event and self.current_frontier is not None and self._stuck_event_count >= self.max_stuck_before_replan:
                self._blacklist_target(self.current_frontier, "连续卡死，触发重规划")
                self.current_frontier = None
                self.path_waypoints = []
                self.wp_index = 0
                self.nav_state = "GOTO"
                self._wall_hit_dist = None
                self.wall_enter_count = 0
                self._stuck_event_count = 0
                self.local_minima_count += 1
                # 显式进入脱困模式，先脱离局部最优再重新选frontier
                self.local_minima_until = now + min(6.0, 2.0 + 0.8 * self.local_minima_count)

            self.rate.sleep()

if __name__ == '__main__':
    try:
        solver = MazeSolver()
        solver.run()
    except rospy.ROSInterruptException:
        rospy.loginfo("程序中断")
    except Exception as e:
        rospy.logerr("运行出错: %s", e)
