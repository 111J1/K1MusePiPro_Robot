#!/usr/bin/env python3
import math
import random
import time
from dataclasses import dataclass
from typing import List, Optional, Tuple

import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import Point, PoseStamped
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import OccupancyGrid
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.time import Time
from std_msgs.msg import String
from tf2_ros import Buffer, TransformException, TransformListener
from visualization_msgs.msg import Marker, MarkerArray


@dataclass
class Candidate:
    x: float
    y: float
    source: str
    information: int
    frontier_x: Optional[float] = None
    frontier_y: Optional[float] = None
    score: float = 0.0


@dataclass
class FailedGoal:
    x: float
    y: float
    stamp: float


@dataclass
class VisitedSpot:
    x: float
    y: float
    stamp: float


class GridView:
    def __init__(self, msg: OccupancyGrid):
        self.msg = msg
        self.width = int(msg.info.width)
        self.height = int(msg.info.height)
        self.resolution = float(msg.info.resolution)
        self.origin_x = float(msg.info.origin.position.x)
        self.origin_y = float(msg.info.origin.position.y)
        self.data = msg.data

    def in_cell_bounds(self, mx: int, my: int) -> bool:
        return 0 <= mx < self.width and 0 <= my < self.height

    def index(self, mx: int, my: int) -> int:
        return my * self.width + mx

    def value(self, mx: int, my: int) -> int:
        if not self.in_cell_bounds(mx, my):
            return 100
        return int(self.data[self.index(mx, my)])

    def world_to_cell(self, x: float, y: float) -> Optional[Tuple[int, int]]:
        mx = int(math.floor((x - self.origin_x) / self.resolution))
        my = int(math.floor((y - self.origin_y) / self.resolution))
        if not self.in_cell_bounds(mx, my):
            return None
        return mx, my

    def cell_to_world(self, mx: int, my: int) -> Tuple[float, float]:
        return (
            self.origin_x + (mx + 0.5) * self.resolution,
            self.origin_y + (my + 0.5) * self.resolution,
        )


class RrtFrontierExplorer(Node):
    def __init__(self):
        super().__init__("rrt_frontier_explorer")

        self.map_topic = self.declare_parameter("map_topic", "/map").value
        self.global_frame = self.declare_parameter("global_frame", "map").value
        self.base_frame = self.declare_parameter("base_frame", "base_footprint").value
        self.send_goals = self._param_bool("send_goals", False)
        self.min_x = self._param_float("min_x", math.nan)
        self.max_x = self._param_float("max_x", math.nan)
        self.min_y = self._param_float("min_y", math.nan)
        self.max_y = self._param_float("max_y", math.nan)

        self.occupied_threshold = self._param_int("occupied_threshold", 65)
        self.frontier_stride = max(1, self._param_int("frontier_stride", 2))
        self.cluster_radius = self._param_float("cluster_radius", 0.35)
        self.info_radius = self._param_float("info_radius", 1.0)
        self.info_multiplier = self._param_float("info_multiplier", 0.10)
        self.distance_weight = self._param_float("distance_weight", 0.65)
        self.min_goal_distance = self._param_float("min_goal_distance", 0.45)
        self.max_goal_distance = self._param_float("max_goal_distance", 6.0)
        self.viewpoint_min_distance = self._param_float("viewpoint_min_distance", 0.28)
        self.viewpoint_max_distance = self._param_float("viewpoint_max_distance", 0.90)
        self.viewpoint_distance_step = self._param_float("viewpoint_distance_step", 0.15)
        self.goal_reached_distance = self._param_float("goal_reached_distance", 0.20)
        self.min_goal_active_sec = self._param_float("min_goal_active_sec", 15.0)
        self.distance_goal_completion_sec = self._param_float("distance_goal_completion_sec", 1.0)
        self.goal_timeout_sec = self._param_float("goal_timeout_sec", 75.0)
        self.replan_period_sec = self._param_float("replan_period_sec", 5.0)
        self.eta = self._param_float("eta", 0.5)
        self.rrt_samples_per_cycle = self._param_int("rrt_samples_per_cycle", 120)
        self.failed_goal_radius = self._param_float("failed_goal_radius", 0.55)
        self.failed_goal_ttl_sec = self._param_float("failed_goal_ttl_sec", 120.0)
        self.visited_goal_radius = self._param_float("visited_goal_radius", 0.45)
        self.visited_frontier_block_radius = self._param_float("visited_frontier_block_radius", 0.35)
        self.visited_frontier_radius = self._param_float("visited_frontier_radius", 0.75)
        self.visited_ttl_sec = self._param_float("visited_ttl_sec", 900.0)
        self.visited_history_limit = max(1, self._param_int("visited_history_limit", 120))
        self.novelty_radius = self._param_float("novelty_radius", 2.0)
        self.novelty_weight = self._param_float("novelty_weight", 1.2)
        self.frontier_novelty_weight = self._param_float("frontier_novelty_weight", 0.8)
        self.visited_goal_penalty = self._param_float("visited_goal_penalty", 2.0)
        self.visited_frontier_penalty = self._param_float("visited_frontier_penalty", 2.0)
        self.path_visit_min_separation = self._param_float("path_visit_min_separation", 0.40)
        self.unvisited_free_stride = max(1, self._param_int("unvisited_free_stride", 5))
        self.unvisited_free_limit = max(0, self._param_int("unvisited_free_limit", 80))
        self.unvisited_free_min_novelty = self._param_float("unvisited_free_min_novelty", 0.9)
        self.unvisited_free_min_info = self._param_int("unvisited_free_min_info", 3)
        self.robot_clearance = self._param_float("robot_clearance", 0.28)
        self.unknown_clearance = self._param_float("unknown_clearance", 0.08)
        self.relaxed_safety_when_stuck = self._param_bool("relaxed_safety_when_stuck", True)
        self.relaxed_robot_clearance = self._param_float("relaxed_robot_clearance", 0.22)
        self.relaxed_unknown_clearance = self._param_float("relaxed_unknown_clearance", 0.0)
        self.max_candidates = self._param_int("max_candidates", 120)
        self.done_no_candidate_cycles = self._param_int("done_no_candidate_cycles", 6)
        self.completion_detector_enabled = self._param_bool("completion_detector_enabled", True)
        self.completion_repeat_cycles = self._param_int("completion_repeat_cycles", 6)
        self.completion_min_information = self._param_int("completion_min_information", 8)
        self.completion_min_score = self._param_float("completion_min_score", 0.5)
        self.completion_min_safe_candidates = self._param_int("completion_min_safe_candidates", 2)
        self.completion_min_viewpoints = self._param_int("completion_min_viewpoints", 3)
        self.completion_visited_reject_ratio = self._param_float("completion_visited_reject_ratio", 0.70)
        self.completion_min_visited_goals = self._param_int("completion_min_visited_goals", 3)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.nav_client = ActionClient(self, NavigateToPose, "navigate_to_pose")

        self.map: Optional[OccupancyGrid] = None
        self.grid: Optional[GridView] = None
        self.last_plan_time = 0.0
        self.active_goal_xy: Optional[Tuple[float, float]] = None
        self.active_goal_candidate: Optional[Candidate] = None
        self.active_goal_handle = None
        self.goal_start_time = 0.0
        self.active_goal_close_since: Optional[float] = None
        self.failed_goals: List[FailedGoal] = []
        self.visited_goals: List[VisitedSpot] = []
        self.visited_frontiers: List[VisitedSpot] = []
        self.last_best: Optional[Candidate] = None
        self.goal_future = None
        self.result_future = None
        self.no_candidate_cycles = 0
        self.completion_candidate_cycles = 0
        self.exploration_done = False
        self.completion_reason = ""
        self.last_stats = {}

        self.map_sub = self.create_subscription(
            OccupancyGrid, self.map_topic, self._map_callback, 10
        )
        self.marker_pub = self.create_publisher(MarkerArray, "exploration_markers", 10)
        self.status_pub = self.create_publisher(String, "exploration_status", 10)
        self.timer = self.create_timer(1.0, self._tick)

        self.random = random.Random(7)
        mode = "ACTIVE goals enabled" if self.send_goals else "DRY RUN markers only"
        self.get_logger().info(
            "RRT frontier explorer ready (%s, boundary x=[%.2f, %.2f] y=[%.2f, %.2f])"
            % (mode, self.min_x, self.max_x, self.min_y, self.max_y)
        )

    def _param_bool(self, name: str, default: bool) -> bool:
        self.declare_parameter(name, default)
        value = self.get_parameter(name).value
        if isinstance(value, str):
            return value.strip().lower() in ("1", "true", "yes", "on")
        return bool(value)

    def _param_float(self, name: str, default: float) -> float:
        self.declare_parameter(name, default)
        value = self.get_parameter(name).value
        try:
            return float(value)
        except (TypeError, ValueError):
            return default

    def _param_int(self, name: str, default: int) -> int:
        self.declare_parameter(name, default)
        value = self.get_parameter(name).value
        try:
            return int(value)
        except (TypeError, ValueError):
            return default

    def _map_callback(self, msg: OccupancyGrid):
        if msg.info.width == 0 or msg.info.height == 0 or not msg.data:
            return
        self.map = msg
        self.grid = GridView(msg)

    def _tick(self):
        if not self._valid_boundary():
            self._publish_status("idle: invalid exploration boundary; pass min_x/max_x/min_y/max_y")
            return
        if self.grid is None:
            self._publish_status("idle: waiting for /map")
            return

        pose = self._robot_pose()
        if pose is None:
            self._publish_status("idle: waiting for TF map -> base_footprint")
            return

        self._expire_failed_goals()
        self._expire_visited_spots()
        self._record_pose_visit(pose)
        if self.exploration_done:
            self._publish_status(self._completion_status("done"))
            return

        if self._active_goal_done_by_distance(pose):
            self._mark_active_goal_visited("goal reached by distance")

        if self._goal_timed_out():
            self._mark_active_goal_failed("goal timeout")

        now = time.monotonic()
        if self.active_goal_xy is not None:
            distance = math.hypot(
                pose[0] - self.active_goal_xy[0],
                pose[1] - self.active_goal_xy[1],
            )
            close_for = 0.0
            if self.active_goal_close_since is not None:
                close_for = now - self.active_goal_close_since
            self._publish_status(
                "navigating: target=(%.2f, %.2f) distance=%.2f age=%.1f close_for=%.1f"
                % (
                    self.active_goal_xy[0],
                    self.active_goal_xy[1],
                    distance,
                    now - self.goal_start_time,
                    close_for,
                )
            )
            return
        if now - self.last_plan_time < self.replan_period_sec:
            return
        self.last_plan_time = now

        candidates = self._make_candidates(pose)
        if not candidates:
            self.no_candidate_cycles += 1
            self._publish_markers([], None)
            status = (
                "searching: no safe viewpoint cycle=%d/%d "
                "raw=%d frontiers=%d viewpoints=%d unvisited_free=%d "
                "relaxed_used=%d "
                "rejected_obstacle=%d rejected_unknown=%d "
                "rejected_failed=%d rejected_distance=%d rejected_visited_goal=%d "
                "rejected_visited_frontier=%d rejected_no_info=%d "
                "visited_goals=%d visited_frontiers=%d"
            ) % (
                self.no_candidate_cycles,
                self.done_no_candidate_cycles,
                self.last_stats.get("raw_candidates", 0),
                self.last_stats.get("frontiers", 0),
                self.last_stats.get("viewpoints", 0),
                self.last_stats.get("unvisited_free", 0),
                self.last_stats.get("relaxed_used", 0),
                self.last_stats.get("rejected_obstacle", 0),
                self.last_stats.get("rejected_unknown", 0),
                self.last_stats.get("rejected_failed", 0),
                self.last_stats.get("rejected_distance", 0),
                self.last_stats.get("rejected_visited_goal", 0),
                self.last_stats.get("rejected_visited_frontier", 0),
                self.last_stats.get("rejected_no_info", 0),
                len(self.visited_goals),
                len(self.visited_frontiers),
            )
            if self.no_candidate_cycles >= self.done_no_candidate_cycles:
                self.exploration_done = True
                self.completion_reason = "no_safe_viewpoint"
                status = self._completion_status("done")
            self._publish_status(status)
            return

        self.no_candidate_cycles = 0
        best = max(candidates, key=lambda item: item.score)
        self.last_best = best
        self._publish_markers(candidates, best)

        completion_ready, completion_detail = self._update_completion_detector(candidates, best)
        if completion_ready:
            self.exploration_done = True
            self.completion_reason = completion_detail
            self._publish_status(self._completion_status("done"))
            return

        self._publish_status(
            "candidate: target=(%.2f, %.2f) frontier=(%.2f, %.2f) score=%.2f "
            "source=%s accepted=%d raw=%d frontiers=%d viewpoints=%d unvisited_free=%d "
            "relaxed_used=%d rejected_obstacle=%d "
            "rejected_unknown=%d rejected_failed=%d rejected_distance=%d "
            "rejected_visited_goal=%d rejected_visited_frontier=%d "
            "rejected_no_info=%d completion_cycle=%d/%d visited_goals=%d "
            "visited_frontiers=%d send_goals=%s"
            % (
                best.x,
                best.y,
                best.frontier_x if best.frontier_x is not None else best.x,
                best.frontier_y if best.frontier_y is not None else best.y,
                best.score,
                best.source,
                len(candidates),
                self.last_stats.get("raw_candidates", 0),
                self.last_stats.get("frontiers", 0),
                self.last_stats.get("viewpoints", 0),
                self.last_stats.get("unvisited_free", 0),
                self.last_stats.get("relaxed_used", 0),
                self.last_stats.get("rejected_obstacle", 0),
                self.last_stats.get("rejected_unknown", 0),
                self.last_stats.get("rejected_failed", 0),
                self.last_stats.get("rejected_distance", 0),
                self.last_stats.get("rejected_visited_goal", 0),
                self.last_stats.get("rejected_visited_frontier", 0),
                self.last_stats.get("rejected_no_info", 0),
                self.completion_candidate_cycles,
                self.completion_repeat_cycles,
                len(self.visited_goals),
                len(self.visited_frontiers),
                self.send_goals,
            )
        )
        if self.send_goals:
            self._send_goal(best)

    def _update_completion_detector(
        self,
        candidates: List[Candidate],
        best: Candidate,
    ) -> Tuple[bool, str]:
        if not self.completion_detector_enabled:
            self.completion_candidate_cycles = 0
            return False, "disabled"
        if len(self.visited_goals) < self.completion_min_visited_goals:
            self.completion_candidate_cycles = 0
            return False, "warmup"

        raw_candidates = max(1, self.last_stats.get("raw_candidates", 0))
        visited_rejected = (
            self.last_stats.get("rejected_visited_goal", 0)
            + self.last_stats.get("rejected_visited_frontier", 0)
        )
        visited_reject_ratio = visited_rejected / raw_candidates

        low_gain = (
            best.information < self.completion_min_information
            or best.score < self.completion_min_score
        )
        few_safe_candidates = len(candidates) < self.completion_min_safe_candidates
        few_viewpoints = (
            self.last_stats.get("viewpoints", 0) < self.completion_min_viewpoints
            and self.last_stats.get("unvisited_free", 0) == 0
        )
        mostly_visited = visited_reject_ratio >= self.completion_visited_reject_ratio

        signals = []
        if low_gain:
            signals.append("low_gain")
        if few_safe_candidates:
            signals.append("few_safe_candidates")
        if few_viewpoints:
            signals.append("few_viewpoints")
        if mostly_visited:
            signals.append("mostly_visited")

        if signals:
            self.completion_candidate_cycles += 1
        else:
            self.completion_candidate_cycles = 0

        detail = (
            "%s cycles=%d/%d best_info=%d best_score=%.2f accepted=%d "
            "raw=%d viewpoints=%d unvisited_free=%d visited_reject_ratio=%.2f"
        ) % (
            "+".join(signals) if signals else "active",
            self.completion_candidate_cycles,
            self.completion_repeat_cycles,
            best.information,
            best.score,
            len(candidates),
            raw_candidates,
            self.last_stats.get("viewpoints", 0),
            self.last_stats.get("unvisited_free", 0),
            visited_reject_ratio,
        )
        return self.completion_candidate_cycles >= self.completion_repeat_cycles, detail

    def _completion_status(self, prefix: str) -> str:
        best_text = "none"
        if self.last_best is not None:
            best_text = (
                "target=(%.2f, %.2f) source=%s score=%.2f info=%d"
                % (
                    self.last_best.x,
                    self.last_best.y,
                    self.last_best.source,
                    self.last_best.score,
                    self.last_best.information,
                )
            )
        return (
            "%s: exploration complete reason=%s %s raw=%d frontiers=%d "
            "viewpoints=%d unvisited_free=%d accepted_cycle=%d/%d "
            "visited_goals=%d visited_frontiers=%d"
        ) % (
            prefix,
            self.completion_reason,
            best_text,
            self.last_stats.get("raw_candidates", 0),
            self.last_stats.get("frontiers", 0),
            self.last_stats.get("viewpoints", 0),
            self.last_stats.get("unvisited_free", 0),
            self.completion_candidate_cycles,
            self.completion_repeat_cycles,
            len(self.visited_goals),
            len(self.visited_frontiers),
        )

    def _valid_boundary(self) -> bool:
        values = (self.min_x, self.max_x, self.min_y, self.max_y)
        return (
            all(math.isfinite(value) for value in values)
            and self.min_x < self.max_x
            and self.min_y < self.max_y
        )

    def _robot_pose(self) -> Optional[Tuple[float, float]]:
        try:
            tf = self.tf_buffer.lookup_transform(
                self.global_frame,
                self.base_frame,
                Time(),
                timeout=Duration(seconds=0.2),
            )
        except TransformException as exc:
            self.get_logger().warn(
                "Waiting for TF %s -> %s: %s"
                % (self.global_frame, self.base_frame, exc),
                throttle_duration_sec=5.0,
            )
            return None
        return (float(tf.transform.translation.x), float(tf.transform.translation.y))

    def _make_candidates(self, pose: Tuple[float, float]) -> List[Candidate]:
        assert self.grid is not None
        self.last_stats = {
            "raw_candidates": 0,
            "frontiers": 0,
            "viewpoints": 0,
            "unvisited_free": 0,
            "rejected_obstacle": 0,
            "rejected_unknown": 0,
            "rejected_failed": 0,
            "rejected_distance": 0,
            "rejected_visited_goal": 0,
            "rejected_visited_frontier": 0,
            "rejected_no_info": 0,
            "relaxed_attempted": 0,
            "relaxed_used": 0,
        }
        candidates = self._frontier_candidates()
        candidates.extend(self._rrt_candidates(pose))
        candidates.extend(self._unvisited_free_candidates(pose))
        self.last_stats["raw_candidates"] = len(candidates)

        filtered = self._filter_candidates(candidates, pose, relaxed=False, update_stats=True)
        if not filtered and self.relaxed_safety_when_stuck and candidates:
            self.last_stats["relaxed_attempted"] = 1
            filtered = self._filter_candidates(candidates, pose, relaxed=True, update_stats=False)
            if filtered:
                self.last_stats["relaxed_used"] = 1
        filtered = self._cluster_candidates(filtered)
        return filtered[:self.max_candidates]

    def _filter_candidates(
        self,
        candidates: List[Candidate],
        pose: Tuple[float, float],
        relaxed: bool,
        update_stats: bool,
    ) -> List[Candidate]:
        filtered = []
        for candidate in candidates:
            if not self._inside_boundary(candidate.x, candidate.y):
                continue
            if self._too_close_to_failed(candidate.x, candidate.y):
                if update_stats:
                    self.last_stats["rejected_failed"] += 1
                continue
            if not relaxed and self._too_close_to_visited_goal(candidate.x, candidate.y):
                if update_stats:
                    self.last_stats["rejected_visited_goal"] += 1
                continue
            frontier_visited_distance = self._frontier_distance_to_visited(candidate)
            if not relaxed and frontier_visited_distance < self.visited_frontier_block_radius:
                if update_stats:
                    self.last_stats["rejected_visited_frontier"] += 1
                continue
            distance = math.hypot(candidate.x - pose[0], candidate.y - pose[1])
            if distance < self.min_goal_distance or distance > self.max_goal_distance:
                if update_stats:
                    self.last_stats["rejected_distance"] += 1
                continue
            safe, reason = self._is_safe_goal(candidate.x, candidate.y, relaxed=relaxed)
            if not safe:
                if update_stats and reason in self.last_stats:
                    self.last_stats[reason] += 1
                continue
            info = self._information_gain(candidate.x, candidate.y)
            if info <= 0:
                if update_stats:
                    self.last_stats["rejected_no_info"] += 1
                continue
            candidate.information = info
            candidate.score = self._score_candidate(
                candidate,
                distance,
                frontier_visited_distance,
            )
            if relaxed:
                candidate.score -= 0.5
            filtered.append(candidate)
        return filtered

    def _frontier_candidates(self) -> List[Candidate]:
        assert self.grid is not None
        candidates = []
        for my in range(1, self.grid.height - 1, self.frontier_stride):
            for mx in range(1, self.grid.width - 1, self.frontier_stride):
                value = self.grid.value(mx, my)
                if value < 0 or value >= self.occupied_threshold:
                    continue
                wx, wy = self.grid.cell_to_world(mx, my)
                if not self._inside_boundary(wx, wy):
                    continue
                if self._has_unknown_neighbor(mx, my):
                    self.last_stats["frontiers"] += 1
                    candidates.extend(self._viewpoints_for_frontier(mx, my, wx, wy))
        return candidates

    def _viewpoints_for_frontier(self, mx: int, my: int, wx: float, wy: float) -> List[Candidate]:
        direction = self._unknown_direction(mx, my)
        if direction is None:
            return []

        ux, uy = direction
        candidates = []
        distance = self.viewpoint_min_distance
        step = max(self.viewpoint_distance_step, 0.05)
        while distance <= self.viewpoint_max_distance + 1e-6:
            vx = wx - ux * distance
            vy = wy - uy * distance
            if self._inside_boundary(vx, vy) and self._is_free(vx, vy):
                candidates.append(Candidate(vx, vy, "viewpoint", 1, wx, wy))
                self.last_stats["viewpoints"] += 1
            distance += step
        return candidates

    def _unknown_direction(self, mx: int, my: int) -> Optional[Tuple[float, float]]:
        assert self.grid is not None
        ux = 0.0
        uy = 0.0
        count = 0
        wx, wy = self.grid.cell_to_world(mx, my)
        for dy in (-1, 0, 1):
            for dx in (-1, 0, 1):
                if dx == 0 and dy == 0:
                    continue
                if self.grid.value(mx + dx, my + dy) < 0:
                    nx, ny = self.grid.cell_to_world(mx + dx, my + dy)
                    ux += nx - wx
                    uy += ny - wy
                    count += 1
        if count == 0:
            return None
        norm = math.hypot(ux, uy)
        if norm < 1e-6:
            return None
        return ux / norm, uy / norm

    def _rrt_candidates(self, pose: Tuple[float, float]) -> List[Candidate]:
        assert self.grid is not None
        tree = [pose]
        candidates = []
        for _ in range(max(0, self.rrt_samples_per_cycle)):
            sample = (
                self.random.uniform(self.min_x, self.max_x),
                self.random.uniform(self.min_y, self.max_y),
            )
            nearest = min(tree, key=lambda item: math.hypot(sample[0] - item[0], sample[1] - item[1]))
            theta = math.atan2(sample[1] - nearest[1], sample[0] - nearest[0])
            new_point = (
                nearest[0] + self.eta * math.cos(theta),
                nearest[1] + self.eta * math.sin(theta),
            )
            if not self._inside_boundary(new_point[0], new_point[1]):
                continue
            if not self._is_free(new_point[0], new_point[1]):
                continue
            if not self._segment_is_free(nearest, new_point):
                continue
            tree.append(new_point)
            if self._information_gain(new_point[0], new_point[1]) > 0:
                candidates.append(Candidate(new_point[0], new_point[1], "rrt_viewpoint", 1))
        return candidates

    def _unvisited_free_candidates(self, pose: Tuple[float, float]) -> List[Candidate]:
        assert self.grid is not None
        if self.unvisited_free_limit <= 0 or not self.visited_goals:
            return []

        candidates = []
        stride = max(self.unvisited_free_stride, 1)
        for my in range(1, self.grid.height - 1, stride):
            for mx in range(1, self.grid.width - 1, stride):
                wx, wy = self.grid.cell_to_world(mx, my)
                if not self._inside_boundary(wx, wy):
                    continue
                if not self._is_free(wx, wy):
                    continue
                if math.hypot(wx - pose[0], wy - pose[1]) < self.min_goal_distance:
                    continue
                novelty = self._distance_to_nearest(
                    self.visited_goals,
                    wx,
                    wy,
                    self.novelty_radius,
                )
                if novelty < self.unvisited_free_min_novelty:
                    continue
                info = self._information_gain(wx, wy)
                if info < self.unvisited_free_min_info:
                    continue
                candidates.append(Candidate(wx, wy, "unvisited_free", info))

        candidates.sort(
            key=lambda item: (
                self._distance_to_nearest(
                    self.visited_goals,
                    item.x,
                    item.y,
                    self.novelty_radius,
                ),
                item.information,
            ),
            reverse=True,
        )
        candidates = candidates[:self.unvisited_free_limit]
        self.last_stats["unvisited_free"] = len(candidates)
        return candidates

    def _score_candidate(
        self,
        candidate: Candidate,
        distance: float,
        frontier_visited_distance: float,
    ) -> float:
        target_novelty = self._distance_to_nearest(
            self.visited_goals,
            candidate.x,
            candidate.y,
            self.novelty_radius,
        )
        frontier_novelty = 0.0
        if candidate.frontier_x is not None and candidate.frontier_y is not None:
            frontier_novelty = self._distance_to_nearest(
                self.visited_frontiers,
                candidate.frontier_x,
                candidate.frontier_y,
                self.novelty_radius,
            )

        goal_penalty = max(0.0, self.visited_goal_radius - target_novelty) * self.visited_goal_penalty
        frontier_penalty = 0.0
        if frontier_visited_distance < self.visited_frontier_radius:
            frontier_penalty = (
                self.visited_frontier_radius - frontier_visited_distance
            ) * self.visited_frontier_penalty

        source_bonus = 0.0
        if candidate.source == "unvisited_free":
            source_bonus = 0.8
        elif candidate.source == "rrt_viewpoint":
            source_bonus = 0.2

        return (
            candidate.information * self.info_multiplier
            + target_novelty * self.novelty_weight
            + frontier_novelty * self.frontier_novelty_weight
            + source_bonus
            - distance * self.distance_weight
            - goal_penalty
            - frontier_penalty
        )

    def _cluster_candidates(self, candidates: List[Candidate]) -> List[Candidate]:
        ordered = sorted(candidates, key=lambda item: item.score, reverse=True)
        clustered = []
        radius = max(self.cluster_radius, 1e-3)
        for candidate in ordered:
            if any(math.hypot(candidate.x - item.x, candidate.y - item.y) < radius for item in clustered):
                continue
            clustered.append(candidate)
        return clustered

    def _inside_boundary(self, x: float, y: float) -> bool:
        return self.min_x <= x <= self.max_x and self.min_y <= y <= self.max_y

    def _has_unknown_neighbor(self, mx: int, my: int) -> bool:
        assert self.grid is not None
        for dy in (-1, 0, 1):
            for dx in (-1, 0, 1):
                if dx == 0 and dy == 0:
                    continue
                if self.grid.value(mx + dx, my + dy) < 0:
                    return True
        return False

    def _is_free(self, x: float, y: float) -> bool:
        assert self.grid is not None
        cell = self.grid.world_to_cell(x, y)
        if cell is None:
            return False
        value = self.grid.value(cell[0], cell[1])
        return 0 <= value < self.occupied_threshold

    def _is_safe_goal(self, x: float, y: float, relaxed: bool = False) -> Tuple[bool, str]:
        assert self.grid is not None
        cell = self.grid.world_to_cell(x, y)
        if cell is None:
            return False, "rejected_obstacle"
        robot_clearance = self.relaxed_robot_clearance if relaxed else self.robot_clearance
        unknown_clearance = self.relaxed_unknown_clearance if relaxed else self.unknown_clearance
        obstacle_radius_cells = max(1, int(math.ceil(robot_clearance / self.grid.resolution)))
        unknown_radius_cells = max(0, int(math.ceil(unknown_clearance / self.grid.resolution)))
        cx, cy = cell
        for my in range(cy - obstacle_radius_cells, cy + obstacle_radius_cells + 1):
            for mx in range(cx - obstacle_radius_cells, cx + obstacle_radius_cells + 1):
                if not self.grid.in_cell_bounds(mx, my):
                    return False, "rejected_obstacle"
                wx, wy = self.grid.cell_to_world(mx, my)
                if math.hypot(wx - x, wy - y) > robot_clearance:
                    continue
                if self.grid.value(mx, my) >= self.occupied_threshold:
                    return False, "rejected_obstacle"
        if unknown_clearance <= 0.0:
            return True, ""
        for my in range(cy - unknown_radius_cells, cy + unknown_radius_cells + 1):
            for mx in range(cx - unknown_radius_cells, cx + unknown_radius_cells + 1):
                if not self.grid.in_cell_bounds(mx, my):
                    return False, "rejected_unknown"
                wx, wy = self.grid.cell_to_world(mx, my)
                if math.hypot(wx - x, wy - y) > unknown_clearance:
                    continue
                if self.grid.value(mx, my) < 0:
                    return False, "rejected_unknown"
        return True, ""

    def _segment_is_free(self, start: Tuple[float, float], end: Tuple[float, float]) -> bool:
        assert self.grid is not None
        distance = math.hypot(end[0] - start[0], end[1] - start[1])
        steps = max(1, int(math.ceil(distance / max(self.grid.resolution, 1e-3))))
        for i in range(steps + 1):
            t = i / steps
            x = start[0] + (end[0] - start[0]) * t
            y = start[1] + (end[1] - start[1]) * t
            if not self._is_free(x, y):
                return False
        return True

    def _information_gain(self, x: float, y: float) -> int:
        assert self.grid is not None
        cell = self.grid.world_to_cell(x, y)
        if cell is None:
            return 0
        radius_cells = max(1, int(math.ceil(self.info_radius / self.grid.resolution)))
        cx, cy = cell
        unknown_count = 0
        for my in range(cy - radius_cells, cy + radius_cells + 1):
            for mx in range(cx - radius_cells, cx + radius_cells + 1):
                if not self.grid.in_cell_bounds(mx, my):
                    continue
                wx, wy = self.grid.cell_to_world(mx, my)
                if math.hypot(wx - x, wy - y) > self.info_radius:
                    continue
                if self.grid.value(mx, my) < 0:
                    unknown_count += 1
        return unknown_count

    def _too_close_to_failed(self, x: float, y: float) -> bool:
        return any(
            math.hypot(x - item.x, y - item.y) < self.failed_goal_radius
            for item in self.failed_goals
        )

    def _too_close_to_visited_goal(self, x: float, y: float) -> bool:
        return any(
            math.hypot(x - item.x, y - item.y) < self.visited_goal_radius
            for item in self.visited_goals
        )

    def _frontier_distance_to_visited(self, candidate: Candidate) -> float:
        if candidate.frontier_x is None or candidate.frontier_y is None:
            return self.novelty_radius
        if not self.visited_frontiers:
            return self.novelty_radius
        return self._distance_to_nearest(
            self.visited_frontiers,
            candidate.frontier_x,
            candidate.frontier_y,
            self.novelty_radius,
        )

    def _distance_to_nearest(
        self,
        spots: List[VisitedSpot],
        x: float,
        y: float,
        cap: float,
    ) -> float:
        if not spots:
            return 0.0
        return min(
            cap,
            min(math.hypot(x - item.x, y - item.y) for item in spots),
        )

    def _expire_failed_goals(self):
        now = time.monotonic()
        self.failed_goals = [
            item for item in self.failed_goals
            if now - item.stamp < self.failed_goal_ttl_sec
        ]

    def _record_pose_visit(self, pose: Tuple[float, float]):
        if self.path_visit_min_separation <= 0.0:
            return
        if any(
            math.hypot(pose[0] - item.x, pose[1] - item.y) < self.path_visit_min_separation
            for item in self.visited_goals
        ):
            return
        self.visited_goals.append(VisitedSpot(pose[0], pose[1], time.monotonic()))
        self.visited_goals = self.visited_goals[-self.visited_history_limit:]

    def _expire_visited_spots(self):
        now = time.monotonic()
        self.visited_goals = [
            item for item in self.visited_goals
            if now - item.stamp < self.visited_ttl_sec
        ][-self.visited_history_limit:]
        self.visited_frontiers = [
            item for item in self.visited_frontiers
            if now - item.stamp < self.visited_ttl_sec
        ][-self.visited_history_limit:]

    def _send_goal(self, candidate: Candidate, reset_timer: bool = True):
        if not self.nav_client.server_is_ready():
            if not self.nav_client.wait_for_server(timeout_sec=0.1):
                self._publish_status("idle: waiting for Nav2 navigate_to_pose action")
                return

        goal = NavigateToPose.Goal()
        goal.pose = PoseStamped()
        goal.pose.header.frame_id = self.global_frame
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = candidate.x
        goal.pose.pose.position.y = candidate.y
        goal.pose.pose.orientation.w = 1.0

        self.active_goal_xy = (candidate.x, candidate.y)
        self.active_goal_candidate = candidate
        if reset_timer:
            self.goal_start_time = time.monotonic()
            self.active_goal_close_since = None
        self.goal_future = self.nav_client.send_goal_async(
            goal,
            feedback_callback=self._feedback_callback,
        )
        self.goal_future.add_done_callback(self._goal_response_callback)
        self.get_logger().info(
            "Sent exploration goal x=%.2f y=%.2f score=%.2f source=%s"
            % (candidate.x, candidate.y, candidate.score, candidate.source)
        )

    def _goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self._mark_active_goal_failed("goal rejected")
            return
        self.active_goal_handle = goal_handle
        self.result_future = goal_handle.get_result_async()
        self.result_future.add_done_callback(self._result_callback)

    def _feedback_callback(self, feedback):
        _ = feedback

    def _result_callback(self, future):
        result = future.result()
        status = result.status
        if status == GoalStatus.STATUS_SUCCEEDED:
            pose = self._robot_pose()
            if pose is not None and self.active_goal_xy is not None:
                distance = math.hypot(
                    pose[0] - self.active_goal_xy[0],
                    pose[1] - self.active_goal_xy[1],
                )
                if distance > self.goal_reached_distance:
                    self.get_logger().warn(
                        "Nav2 reported success %.2fm from exploration target; resending same goal"
                        % distance
                    )
                    self.active_goal_handle = None
                    self._resend_active_goal()
                    return
            self.get_logger().info("Exploration goal succeeded")
        else:
            self._mark_active_goal_failed("goal failed status=%d" % status)
            return
        self._mark_active_goal_visited("goal succeeded")

    def _resend_active_goal(self):
        if self.active_goal_candidate is None or self.active_goal_xy is None:
            self._mark_active_goal_failed("missing active goal candidate")
            return
        self._send_goal(self.active_goal_candidate, reset_timer=False)

    def _active_goal_done_by_distance(self, pose: Tuple[float, float]) -> bool:
        if self.active_goal_xy is None:
            return False
        now = time.monotonic()
        distance = math.hypot(pose[0] - self.active_goal_xy[0], pose[1] - self.active_goal_xy[1])
        if distance >= self.goal_reached_distance:
            self.active_goal_close_since = None
            return False
        if now - self.goal_start_time < self.min_goal_active_sec:
            return False
        if self.active_goal_close_since is None:
            self.active_goal_close_since = now
            return False
        return now - self.active_goal_close_since >= self.distance_goal_completion_sec

    def _goal_timed_out(self) -> bool:
        return (
            self.active_goal_xy is not None
            and self.goal_timeout_sec > 0.0
            and time.monotonic() - self.goal_start_time > self.goal_timeout_sec
        )

    def _mark_active_goal_failed(self, reason: str):
        if self.active_goal_xy is not None:
            self.failed_goals.append(FailedGoal(
                self.active_goal_xy[0],
                self.active_goal_xy[1],
                time.monotonic(),
            ))
        self.get_logger().warn("Exploration goal marked failed: %s" % reason)
        self.active_goal_xy = None
        self.active_goal_candidate = None
        self.active_goal_handle = None
        self.active_goal_close_since = None

    def _mark_active_goal_visited(self, reason: str):
        if self.active_goal_xy is None:
            return
        now = time.monotonic()
        self.visited_goals.append(VisitedSpot(self.active_goal_xy[0], self.active_goal_xy[1], now))
        if self.active_goal_candidate is not None:
            frontier_x = self.active_goal_candidate.frontier_x
            frontier_y = self.active_goal_candidate.frontier_y
            if frontier_x is not None and frontier_y is not None:
                self.visited_frontiers.append(VisitedSpot(frontier_x, frontier_y, now))
        self.visited_goals = self.visited_goals[-self.visited_history_limit:]
        self.visited_frontiers = self.visited_frontiers[-self.visited_history_limit:]
        self.get_logger().info("Exploration goal marked visited: %s" % reason)
        self.active_goal_xy = None
        self.active_goal_candidate = None
        self.active_goal_handle = None
        self.active_goal_close_since = None

    def _publish_status(self, text: str):
        msg = String()
        msg.data = text
        self.status_pub.publish(msg)

    def _publish_markers(self, candidates: List[Candidate], best: Optional[Candidate]):
        markers = MarkerArray()
        now = self.get_clock().now().to_msg()

        delete_marker = Marker()
        delete_marker.header.frame_id = self.global_frame
        delete_marker.header.stamp = now
        delete_marker.ns = "k1_exploration"
        delete_marker.action = Marker.DELETEALL
        markers.markers.append(delete_marker)

        points = Marker()
        points.header.frame_id = self.global_frame
        points.header.stamp = now
        points.ns = "k1_exploration"
        points.id = 1
        points.type = Marker.POINTS
        points.action = Marker.ADD
        points.scale.x = 0.08
        points.scale.y = 0.08
        points.color.r = 0.1
        points.color.g = 0.7
        points.color.b = 1.0
        points.color.a = 0.85
        for candidate in candidates:
            point = Point()
            point.x = candidate.x
            point.y = candidate.y
            point.z = 0.05
            points.points.append(point)
        markers.markers.append(points)

        if best is not None:
            target = Marker()
            target.header.frame_id = self.global_frame
            target.header.stamp = now
            target.ns = "k1_exploration"
            target.id = 2
            target.type = Marker.SPHERE
            target.action = Marker.ADD
            target.pose.position.x = best.x
            target.pose.position.y = best.y
            target.pose.position.z = 0.1
            target.scale.x = 0.22
            target.scale.y = 0.22
            target.scale.z = 0.08
            target.color.r = 1.0
            target.color.g = 0.5
            target.color.b = 0.0
            target.color.a = 0.95
            markers.markers.append(target)

        self.marker_pub.publish(markers)


def main(args=None):
    rclpy.init(args=args)
    node = RrtFrontierExplorer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
