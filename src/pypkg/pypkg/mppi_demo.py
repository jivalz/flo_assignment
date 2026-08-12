#!/usr/bin/env python3
from numpy import float64
from rclpy.validate_namespace import validate_namespace
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import Twist, PointStamped, Point, PoseStamped
from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import Marker, MarkerArray
import os
import csv
import math 
import numpy as np
from scipy.interpolate import CubicSpline
from scipy.spatial import cKDTree
from scipy.spatial.distance import cdist
from scipy.ndimage import gaussian_filter1d

def euler_from_quat(x: float, y: float, z: float, w: float):
    t3 = 2.0 * (w*z + x*y)
    t4 = 1.0 - 2.0 * (y*y + z*z)
    return float(math.atan2(t3,t4))
def wrap_angle(angle: np.ndarray):
    return (angle + np.pi) % (2.0 * np.pi) - np.pi

#transform from the lidar frame to world frame(map)
# NOTE: max_range widened 1.0 -> 1.8 (see scan_callback below for the call site).
# At v_max=0.5 m/s, 1.0m only gave ~2s of warning before contact; 1.8m gives ~3.6s,
# more in line with the planning horizon, without going as wide as the old 3.5m.
def laserscan_to_world_frame(scan:LaserScan, rb_x : float,rb_y : float,rb_yaw : float, max_range : float=1.8,min_range : float=0.05,) -> np.ndarray:
    ranges = np.array(scan.ranges , dtype = np.float64)
    angles = scan.angle_min + np.arange(len(ranges)) * scan.angle_increment
    valid = np.isfinite(ranges) & (ranges>=min_range) & (ranges<=max_range)
    r = ranges[valid]
    phi = angles[valid]
    if r.size == 0:
        return np.empty((0,2),dtype=np.float64)
    local_x = r*np.cos(phi)
    local_y = r*np.sin(phi)
    cos_y = math.cos(rb_yaw)
    sin_y = math.sin(rb_yaw)
    world_x = rb_x + cos_y * local_x - sin_y * local_y
    world_y = rb_y + sin_y * local_x + cos_y * local_y
    return np.column_stack([world_x, world_y])

class referencepath:
    def __init__(self, waypoints_xy:np.ndarray):
        if len(waypoints_xy)<2 :
            raise ValueError("ref path require atleast 2 waypoint")
        
        diffs = np.diff(waypoints_xy,axis=0)
        dists = np.hypot(diffs[:,0], diffs[:,1])
        keep = np.concatenate(([True],dists >1e-4))
        pts = waypoints_xy[keep]

        segment_lengths = np.hypot(np.diff(pts[:,0]),np.diff(pts[:,1]))
        self.s_waypoints = np.concatenate(([0.0], np.cumsum(segment_lengths)))
        self.total_length = self.s_waypoints[-1]

        self.cs_x = CubicSpline(self.s_waypoints,pts[:,0], bc_type = 'natural')
        self.cs_y = CubicSpline(self.s_waypoints, pts[:,1],bc_type = 'natural')
        
        n_samples = max(200, int(self.total_length * 20))
        self.s_dense = np.linspace(0.0, self.total_length,n_samples)
        self.dense_xy  = np.column_stack([self.cs_x(self.s_dense), self.cs_y(self.s_dense)])

        dx_ds = self.cs_x.derivative()(self.s_dense)
        dy_ds = self.cs_y.derivative()(self.s_dense)
        tangent_norms = np.hypot(dx_ds, dy_ds) + 1e-6
        self.dense_tangent_x = dx_ds / tangent_norms
        self.dense_tangent_y = dy_ds / tangent_norms
        self.kdtree = cKDTree(self.dense_xy)

    def eval(self, s :float):
        s_clamp = np.clip(s,0.0, self.total_length)
        return float(self.cs_x(s_clamp)), float(self.cs_y(s_clamp))
    def find_closest_idx(self, x:float , y:float):
        _, idx  = self.kdtree.query([x,y])
        return int(idx)

#mppi solver
class mppi:
    def __init__(
        self, ref_path: referencepath,
        K: int  = 100, #rollout samples
        N: int = 30, #prediction horizon
        dt:float = 0.05, #value of h
        lambda_: float = 1.0,# const value of temp for the softmax
        sig_v:float= 0.3, # noise for v
        sig_w:float = 1.0, #noise for w
        v_max:float = 0.5, 
        w_max:float = 1.0,
        wprogress:float = 10.0,
        wtracking:float = 15.0,
        wheading:float  = 5.0,
        wcollision:float = 100.0, 
        wproximity:float = 10.0, 
        wsmoothness:float = 2.0,
        wstall:float = 3.0,   # NEW: small direct reward for raw forward speed.
                               # progress_cost is 0 whenever v=0 (dx=dy=0), which makes
                               # "stand still and spin" a zero-cost option whenever the
                               # robot's heading is off from the path -- exactly the
                               # "look at the wall, yaw left/right, no progress" failure
                               # mode. This term makes standing still never free, without
                               # needing to touch tracking/heading/collision weights.
        obs_threshold:float = 0.35,
        sigma_prox:float = 0.5 #soft threshold 
    ):
        self.ref_path = ref_path
        self.K = K
        self.N = N
        self.dt = dt
        self.lambda_ = lambda_
        self.sig_v = sig_v
        self.sig_w = sig_w
        self.v_max = v_max
        self.w_max = w_max
        self.wprogress = wprogress
        self.wtracking = wtracking
        self.wheading = wheading
        self.wcollision = wcollision
        self.wproximity = wproximity
        self.wsmoothness = wsmoothness
        self.wstall = wstall
        self.obs_threshold = obs_threshold
        self.sigma_prox = sigma_prox

        self.u_nominal = np.zeros((N,2), dtype=float64)
        self.u_nominal[:,0] = 0.2 #initial linear velocity guess
    
    def diffdrive_odometry_model(self,states:np.ndarray,controls:np.ndarray):
        v = controls[:,0]
        w = controls[:,1]
        yaw = states[:,2]
        x_next = states[:,0] + v*np.cos(yaw)*self.dt
        y_next = states[:,1] + v*np.sin(yaw)*self.dt
        yaw_next = wrap_angle(yaw + w*self.dt)

        return np.stack([x_next, y_next, yaw_next], axis = 1)
    
    def rollout(self, x0:np.ndarray):
        epsilon_v = np.random.randn(self.K, self.N)
        epsilon_w = np.random.randn(self.K, self.N)
        #smoothen the noise
        epsilon_v = gaussian_filter1d(epsilon_v, sigma = 5.0, axis = 1)
        epsilon_w = gaussian_filter1d(epsilon_w, sigma = 10.0, axis = 1)
        
        # Rescale noise back to desired standard deviations so it actually explores!
        epsilon_v = (epsilon_v / (np.std(epsilon_v) + 1e-8)) * self.sig_v
        epsilon_w = (epsilon_w / (np.std(epsilon_w) + 1e-8)) * self.sig_w
        
        del_u = np.stack([epsilon_v, epsilon_w], axis = 2)
        del_u[0] = 0.0
        #keeping the control sequence under the bots physical constraints
        u_sampled = self.u_nominal[None, :, :] + del_u
        u_sampled[:,:,0] = np.clip(u_sampled[:,:,0], 0.0, self.v_max)
        u_sampled[:,:,1] = np.clip(u_sampled[:,:,1], -self.w_max, self.w_max)
        #sample the next states
        x_sampled = np.empty((self.K, self.N + 1, 3), dtype = float64)
        x_sampled[:,0,:] = x0[None,:]
        for t in range(self.N):
            x_sampled[:,t+1,:] = self.diffdrive_odometry_model(x_sampled[:,t,:], u_sampled[:,t,:])
        return u_sampled, x_sampled, del_u

    def calculate_cost(self, x_sampled:np.ndarray, u_sampled:np.ndarray, scans:np.ndarray, current_idx: int):
        pts = x_sampled[:,1:,:2].reshape(self.K*self.N, 2)

        # Define a local window on the path to prevent snapping to outgoing lanes or crossing paths
        # At 20 pts/m, a 150 point window covers 7.5 meters, matching the N=150 (7.5s) prediction horizon.
        window_start = max(0, current_idx - 10)
        window_end = min(len(self.ref_path.dense_xy), current_idx + 150)
        local_path = self.ref_path.dense_xy[window_start:window_end]
        
        # Fast local search using cdist instead of instantiating a new KDTree
        dists = cdist(pts, local_path)
        closest_idx = np.argmin(dists, axis=1) + window_start

        # Dynamically reduce tracking and heading weights if an obstacle is detected nearby
        current_wtracking = self.wtracking
        current_wheading = self.wheading
        
        dist_to_obs = float('inf')
        if scans.shape[0] > 0:
            current_pos = x_sampled[0, 0, :2].reshape(1, 2)
            dist_to_obs = cdist(current_pos, scans).min()

        tx = self.ref_path.dense_tangent_x[closest_idx].reshape(self.K,self.N)
        ty = self.ref_path.dense_tangent_y[closest_idx].reshape(self.K,self.N)

        #trajectory tracking component of cost fxn
        ref_pts = self.ref_path.dense_xy[closest_idx]
        
        lateral_dist = np.linalg.norm(pts - ref_pts, axis=1).reshape(self.K,self.N)
        if dist_to_obs < 1.5:
            # Deadband: Allow up to 1.5 meters of lateral freedom to detour around obstacles
            lateral_dist = np.maximum(0.0, lateral_dist - 1.5)
            
        tracking_cost = self.wtracking * np.sum(lateral_dist**2, axis = 1)
        
        #heading err and cost
        yaw = x_sampled[:,1:,2].reshape(self.K, self.N)
        
        # Lookahead point for heading (aim at a clear point on the trajectory)
        lookahead_idx = np.clip(closest_idx + 20, 0, len(self.ref_path.dense_xy)-1)
        lookahead_pts = self.ref_path.dense_xy[lookahead_idx]
        dx_target = lookahead_pts[:,0] - pts[:,0]
        dy_target = lookahead_pts[:,1] - pts[:,1]
        path_yaw = np.arctan2(dy_target, dx_target).reshape(self.K, self.N)
        
        heading_error = np.abs(wrap_angle(yaw - path_yaw))
        if dist_to_obs < 1.5:
            # Deadband: Allow up to 45 degrees (0.78 rad) of heading freedom to detour
            heading_error = np.maximum(0.0, heading_error - 0.78)
            
        heading_cost = self.wheading * np.sum(heading_error**2,axis = 1)

        #progress cost
        dx = x_sampled[:,1:,0] - x_sampled[:,:-1, 0]
        dy = x_sampled[:,1:,1] - x_sampled[:,:-1, 1]
        progress_perstep = dx*tx + dy*ty
        progress_cost = -self.wprogress*np.sum(progress_perstep,axis = 1)

        # NEW: anti-stall term. progress_cost above is 0 whenever v=0 (dx=dy=0), which
        # makes "stand still and spin" a genuine zero-cost option whenever heading is
        # off from the path -- e.g. staring at a wall dead-ahead. This rewards raw
        # forward speed directly, independent of tangent alignment, so standing still
        # is never free. Kept small relative to wprogress/wtracking/wcollision so it
        # doesn't override legitimate slow-down-for-safety behavior near obstacles.
        v_cmd_perstep = u_sampled[:, :, 0]
        stall_cost = -self.wstall * np.sum(v_cmd_perstep, axis=1)

        #obstacle collision and proximity cost
        if scans.shape[0] > 0:
            dists = cdist(pts,scans)
            min_dist_perstep = dists.min(axis=1).reshape(self.K, self.N)
            collision = (min_dist_perstep < self.obs_threshold).astype(float)
            proximity = np.exp(- min_dist_perstep / self.sigma_prox)
            collision_cost = self.wcollision * np.sum(collision, axis = 1)
            proximity_cost = self.wproximity * np.sum(proximity, axis=1)
        else:
            collision_cost = 0.0
            proximity_cost = 0.0
        
        #smoothness or jerk reduction
        u_diff = u_sampled[:,1:, :] - u_sampled[:,:-1,:]
        smoothness_cost = self.wsmoothness * (np.sum(u_diff**2, axis= (1,2)))

        #total cost
        final_cost = tracking_cost + heading_cost + progress_cost + stall_cost + collision_cost + proximity_cost + smoothness_cost
        return final_cost
    
    def solve(self,x0:np.ndarray,scans:np.ndarray):
        current_idx = self.ref_path.find_closest_idx(x0[0], x0[1])
        u_sampled, x_sampled, del_u = self.rollout(x0)
        final_cost = self.calculate_cost(x_sampled, u_sampled, scans, current_idx)
        rho = np.min(final_cost)
        wk = np.exp(-(1/self.lambda_) * (final_cost - rho))
        wk = wk / (np.sum(wk) + 1e-10)
        weighte_del_u = np.einsum('k,kni->ni',wk,del_u)
        self.u_nominal += weighte_del_u
        self.u_nominal[:,0] = np.clip(self.u_nominal[:,0], 0.0, self.v_max)
        self.u_nominal[:,1] = np.clip(self.u_nominal[:,1], -self.w_max, self.w_max)
        v_cmd, w_cmd = float(self.u_nominal[0,0]), float(self.u_nominal[0,1])

        best_idx = np.argmin(final_cost)
        x_optimal = x_sampled[best_idx]
        self.u_nominal = np.roll(self.u_nominal, -1, axis = 0)
        self.u_nominal[-1] = self.u_nominal[-2]
        return v_cmd, w_cmd, x_optimal, x_sampled[:40]        

    def reset(self):
        self.u_nominal = np.zeros((self.N,2),dtype=np.float64)
        self.u_nominal[:,0] = 0.2

class mppinode(Node):
    def __init__(self):
        super().__init__('mppi_node')
        
        # VERY IMPORTANT: Enable sim time so our timestamps match Gazebo and RViz!
        self.set_parameters([rclpy.parameter.Parameter('use_sim_time', rclpy.Parameter.Type.BOOL, True)])
        
        self.declare_parameter('waypoint_file', '')
        self.declare_parameter('control_rate', 20.0)
        self.declare_parameter('v_max', 0.5)
        self.declare_parameter('w_max', 1.0)
        self.declare_parameter('r_safe', 0.35)
        wp_file = self.get_parameter('waypoint_file').value
        if not wp_file:
            wp_file = os.path.expanduser('~/assignment/src/nav/waypoints/waypoint5.csv')
            # if not os.path.exists(wp_file):
            #     wp_file = os.path.expanduser('~/assignment/src/nav/waypoints/waypoint4.csv')

        try:
            waypoints = []
            with open(wp_file, 'r') as f:
                reader = csv.reader(f)
                header = next(reader, None)
                for row in reader:
                    if len(row) >= 2:
                        x_val = float(row[-2]) if len(row) > 2 else float(row[0])
                        y_val = float(row[-1]) if len(row) > 2 else float(row[1])
                        waypoints.append([x_val, y_val])

            waypoints_arr = np.array(waypoints, dtype=np.float64)
            self.ref_path = referencepath(waypoints_arr)
            self.get_logger().info(f"Loaded {len(waypoints_arr)} waypoints from: {wp_file}")
        except Exception as e:
            self.get_logger().error(f"Failed to load waypoints from '{wp_file}': {e}")
            raise e

        # Initialize MPPI Solver
        self.solver = mppi(
            ref_path=self.ref_path,
            K=450,
            N=100,  
            dt=0.05,
            lambda_=9.0,  
            sig_v=0.3,    
            sig_w=0.3,
            wprogress=40.0, 
            wheading=3.0,   
            wsmoothness=10.0,  
            wtracking=7.5,   
            wcollision=5000.0, 
            wproximity=10.0, 
            wstall=10.0,       # NEW -- increased to allow dodging on N=100
            v_max=self.get_parameter('v_max').value,
            w_max=self.get_parameter('w_max').value,
            obs_threshold=self.get_parameter('r_safe').value
        )

        # State Variables
        self.robot_state = None  # [x, y, yaw]
        self.scan_pts = np.empty((0, 2))

        # Publishers
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.opt_path_pub = self.create_publisher(Path, '/mppi/optimal_path', 10)
        self.ref_path_pub = self.create_publisher(Path, '/mppi/ref_path', 10)
        self.rollouts_pub = self.create_publisher(MarkerArray, '/mppi/rollouts', 10)

        # Subscribers
        self.create_subscription(Odometry, '/odom', self._odom_callback, 10)
        self.create_subscription(LaserScan, '/scan', self._scan_callback, 10)

        # Control Loop Timer
        rate = self.get_parameter('control_rate').value
        self.create_timer(1.0 / rate, self._control_loop)
        self.create_timer(2.0, self._publish_reference_path)

    def _odom_callback(self, msg: Odometry):
        pos = msg.pose.pose.position
        ori = msg.pose.pose.orientation
        yaw = euler_from_quat(ori.x, ori.y, ori.z, ori.w)
        self.robot_state = np.array([pos.x, pos.y, yaw])

    def _scan_callback(self, msg: LaserScan):
        if self.robot_state is None:
            return
        rx, ry, ryaw = self.robot_state
        # max_range widened 1.0 -> 1.8 (see comment on laserscan_to_world_frame above)
        self.scan_pts = laserscan_to_world_frame(msg, rx, ry, ryaw, max_range=1.8, min_range=0.1)

    def _control_loop(self):
        if self.robot_state is None:
            return

        # Solve MPPI optimization problem
        v_cmd, w_cmd, x_opt, x_rollouts = self.solver.solve(self.robot_state, self.scan_pts)

        # Publish velocity command to robot
        cmd = Twist()
        cmd.linear.x = v_cmd
        cmd.angular.z = w_cmd
        self.get_logger().info(f"Publishing: v={v_cmd:.3f}, w={w_cmd:.3f}")
        self.cmd_pub.publish(cmd)

        # Publish RViz Visualizations
        self._publish_visualizations(x_opt, x_rollouts)

    def _publish_visualizations(self, x_opt: np.ndarray, x_rollouts: np.ndarray):
        stamp = self.get_clock().now().to_msg()

        # 1. Publish Optimal Path Trajectory
        opt_msg = Path()
        opt_msg.header.stamp = stamp
        opt_msg.header.frame_id = 'odom'
        for pt in x_opt:
            ps = PoseStamped()
            ps.header = opt_msg.header
            ps.pose.position.x = pt[0]
            ps.pose.position.y = pt[1]
            opt_msg.poses.append(ps)
        self.opt_path_pub.publish(opt_msg)

        # 2. Publish Candidate Trajectory Rollouts as RViz Markers
        ma = MarkerArray()
        m = Marker()
        m.header.stamp = stamp
        m.header.frame_id = 'odom'
        m.id = 0
        m.type = Marker.LINE_LIST
        m.action = Marker.ADD
        m.scale.x = 0.01  # Thin line width
        m.color.a = 0.15  # Semi-transparent green
        m.color.g = 1.0

        for k in range(x_rollouts.shape[0]):
            for i in range(x_rollouts.shape[1] - 1):
                p1 = Point(x=x_rollouts[k, i, 0], y=x_rollouts[k, i, 1], z=0.0)
                p2 = Point(x=x_rollouts[k, i+1, 0], y=x_rollouts[k, i+1, 1], z=0.0)
                m.points.append(p1)
                m.points.append(p2)
        ma.markers.append(m)
        self.rollouts_pub.publish(ma)

    def _publish_reference_path(self):
        path_msg = Path()
        path_msg.header.frame_id = 'odom'
        path_msg.header.stamp = self.get_clock().now().to_msg()
        for pt in self.ref_path.dense_xy:
            ps = PoseStamped()
            ps.header = path_msg.header
            ps.pose.position.x = pt[0]
            ps.pose.position.y = pt[1]
            path_msg.poses.append(ps)
        self.ref_path_pub.publish(path_msg)


def main(args=None):
    rclpy.init(args=args)
    node = mppinode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()

if __name__ == '__main__':
    main()