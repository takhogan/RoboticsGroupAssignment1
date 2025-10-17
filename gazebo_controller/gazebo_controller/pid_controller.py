import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Odometry
from tf2_ros import Buffer, TransformListener
import math
import numpy as np
from rclpy.duration import Duration

class DiffDrivePID(Node):
    """
    A PD Controller for differential drive robots that uses TF (map -> chassis) 
    to track the robot's pose and calculate velocity commands to reach a goal.
    """
    def __init__(self):
        super().__init__('diffdrive_pid')

        # --- Parameters (declare as ROS parameters for easy tuning) ---
        self.declare_parameter('rate_hz', 30.0)
        self.declare_parameter('kp_linear', 0.5)
        self.declare_parameter('kd_linear', 0.05)
        self.declare_parameter('kp_angular', 1.0)
        self.declare_parameter('kd_angular', 0.1)
        self.declare_parameter('max_linear_vel', 1.0)
        self.declare_parameter('max_angular_vel', 1.5)
        self.declare_parameter('dist_tolerance', 0.1)
        self.declare_parameter('angle_tolerance', 0.05)
        
        self.rate_hz = self.get_parameter('rate_hz').value
        self.period = 1.0 / self.rate_hz

        # PD Controller Gains
        self.kp_linear = self.get_parameter('kp_linear').value
        self.kd_linear = self.get_parameter('kd_linear').value
        self.kp_angular = self.get_parameter('kp_angular').value
        self.kd_angular = self.get_parameter('kd_angular').value

        # Maximum allowed velocities
        self.max_linear_vel = self.get_parameter('max_linear_vel').value
        self.max_angular_vel = self.get_parameter('max_angular_vel').value
        
        # Goal/Error Tolerances
        self.dist_tolerance = self.get_parameter('dist_tolerance').value
        self.angle_tolerance = self.get_parameter('angle_tolerance').value
        
        # --- State variables ---
        self.goal_pose = None
        self.final_turn = False 
        self.current_pos = np.array([0.0, 0.0, 0.0])  # [x, y, theta]
        self.previous_pos = np.array([0.0, 0.0, 0.0])
        self.prev_error_dist = 0.0
        self.prev_error_angle = 0.0
        self.tf_available = False
        
        # Frame names
        self.TARGET_FRAME_ID = 'map' 
        self.SOURCE_FRAME_ID = 'chassis'

        # --- ROS 2 Setup ---
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)

        # TF listener setup
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # Subscribers
        self.goal_sub = self.create_subscription(
            PoseStamped, 
            'goal_pose', 
            self.goal_pose_callback, 
            10
        )
        
        # Optional: Subscribe to odometry for debugging
        self.odom_sub = self.create_subscription(
            Odometry,
            'odom',
            self.odom_callback,
            10
        )
        
        # Control loop timer
        self.timer = self.create_timer(self.period, self.control_loop)
        
        self.get_logger().info('Differential Drive PID Controller Started')
        self.get_logger().info(f'  kp_linear: {self.kp_linear}, kd_linear: {self.kd_linear}')
        self.get_logger().info(f'  kp_angular: {self.kp_angular}, kd_angular: {self.kd_angular}')
        self.get_logger().info(f'  max_linear_vel: {self.max_linear_vel} m/s')
        self.get_logger().info(f'  max_angular_vel: {self.max_angular_vel} rad/s')
    
    def odom_callback(self, msg):
        """Optional callback to monitor odometry (for debugging)"""
        pass  # Can add debugging info here if needed
    
    def goal_pose_callback(self, msg):
        """Processes a new goal pose from the /goal_pose topic."""
        
        # Convert quaternion to euler angle (we only need yaw/theta)
        q = msg.pose.orientation
        _, _, theta = self.quaternion_to_euler(q.x, q.y, q.z, q.w)
        
        # Set goal position
        self.goal_pose = np.array([msg.pose.position.x, msg.pose.position.y, theta]) 

        # Reset previous errors when a new goal is set
        self.prev_error_dist = 0.0
        self.prev_error_angle = 0.0
        self.final_turn = False
        
        self.get_logger().info(
            f"New goal received: x={self.goal_pose[0]:.2f}, "
            f"y={self.goal_pose[1]:.2f}, theta={np.degrees(self.goal_pose[2]):.1f}°"
        )

    def update_pos(self):
        """Updates the robot's current position by looking up the 'map'->'chassis' TF."""
        
        try:
            # Use latest available transform
            t = self.tf_buffer.lookup_transform(
                self.TARGET_FRAME_ID,
                self.SOURCE_FRAME_ID,
                rclpy.time.Time(),
                timeout=Duration(seconds=0.1)
            )
            
            # Mark TF as available
            if not self.tf_available:
                self.tf_available = True
                self.get_logger().info(f'TF {self.TARGET_FRAME_ID}->{self.SOURCE_FRAME_ID} now available')
            
        except Exception as e:
            if self.tf_available:
                self.get_logger().warn(f'TF lookup failed: {str(e)}')
            else:
                # Only log once at debug level during startup
                self.get_logger().debug(f'Waiting for TF {self.TARGET_FRAME_ID}->{self.SOURCE_FRAME_ID}...')
            return False
        
        # Update previous position
        self.previous_pos = np.copy(self.current_pos)

        # Update current position
        self.current_pos[0] = t.transform.translation.x
        self.current_pos[1] = t.transform.translation.y

        # Update current orientation (Yaw)
        q = t.transform.rotation
        _, _, theta = self.quaternion_to_euler(q.x, q.y, q.z, q.w)
        self.current_pos[2] = theta
        
        return True
    
    def control_loop(self):
        """
        Main control loop for PD logic. Calculates errors and publishes Twist commands.
        """

        if self.goal_pose is None:
            # No goal set - publish zero velocity
            self.publish_cmd_vel(0.0, 0.0)
            return
        
        # Update current position and verify success
        if not self.update_pos():
            # If update failed (e.g., TF error), stop the robot
            self.publish_cmd_vel(0.0, 0.0)
            return
            
        x, y, theta = self.current_pos
        x_g, y_g, theta_g = self.goal_pose
        dt = self.period

        # --- PRIMARY NAVIGATION ERRORS ---

        # 1. Distance error (Euclidean distance to goal position)
        dist_error = math.sqrt((x_g - x)**2 + (y_g - y)**2)

        # 2. Heading error: Angle between robot's current heading and line to goal
        goal_angle = math.atan2(y_g - y, x_g - x)
        heading_error = self.normalize_angle(goal_angle - theta)

        # 3. Final angle error: Difference between current and desired final heading
        final_theta_error = self.normalize_angle(theta_g - theta)

        # --- GOAL REACHED CHECK & FINAL TURN ---
        if dist_error < self.dist_tolerance: 
            
            if not self.final_turn:
                self.final_turn = True
                self.get_logger().info(
                    f"Position reached! Distance: {dist_error:.3f}m. "
                    f"Starting final turn to {np.degrees(theta_g):.1f}°"
                )

            if abs(final_theta_error) < self.angle_tolerance:
                self.get_logger().info(
                    f"Goal Reached! Final error - dist: {dist_error:.3f}m, "
                    f"angle: {np.degrees(final_theta_error):.2f}°"
                )
                self.goal_pose = None
                self.final_turn = False
                self.publish_cmd_vel(0.0, 0.0)
                return
        
        v_x_cmd = 0.0
        v_th_cmd = 0.0
        
        if self.final_turn:
            # Final rotation phase: PD-controller on final_theta_error
            d_error_angle = (final_theta_error - self.prev_error_angle) / dt
            v_th_cmd = self.kp_angular * final_theta_error + self.kd_angular * d_error_angle
            self.prev_error_angle = final_theta_error
        
        else:
            # Approach phase: Control based on heading_error and dist_error
            
            # --- ANGULAR VELOCITY CONTROL (PD on Heading Error) ---
            d_error_angle = (heading_error - self.prev_error_angle) / dt
            v_th_cmd = self.kp_angular * heading_error + self.kd_angular * d_error_angle
            self.prev_error_angle = heading_error

            # --- LINEAR VELOCITY CONTROL (PD on Distance Error) ---
            # Stop linear motion if heading error is too large (pure rotation)
            heading_threshold = math.pi / 4  # 45 degrees
            
            if abs(heading_error) > heading_threshold:
                v_x_cmd = 0.0
                # Reset distance error tracking when not moving forward
                self.prev_error_dist = dist_error
            else:
                # Scale linear velocity based on heading alignment
                # Full speed when aligned, reduced when slightly misaligned
                heading_scale = max(0.0, 1.0 - abs(heading_error) / heading_threshold)
                
                # D-term: Rate of change of distance error
                d_error_dist = (dist_error - self.prev_error_dist) / dt
                
                # PD Controller for linear speed
                v_x_cmd = (self.kp_linear * dist_error + self.kd_linear * d_error_dist) * heading_scale
                self.prev_error_dist = dist_error
        
        # --- COMMAND CLAMPING ---
        v_x_cmd = np.clip(v_x_cmd, -self.max_linear_vel, self.max_linear_vel)
        v_th_cmd = np.clip(v_th_cmd, -self.max_angular_vel, self.max_angular_vel)
        
        # Publish commands
        self.publish_cmd_vel(v_x_cmd, v_th_cmd)

    def publish_cmd_vel(self, v_x, v_th):
        """Creates and publishes a Twist message."""
        twist = Twist()
        twist.linear.x = float(v_x)
        twist.angular.z = float(v_th)
        self.cmd_vel_pub.publish(twist)

    @staticmethod
    def normalize_angle(angle):
        """Normalize angle to [-pi, pi]"""
        return math.atan2(math.sin(angle), math.cos(angle))

    @staticmethod
    def quaternion_to_euler(x, y, z, w):
        """Convert a quaternion to Euler angles (roll, pitch, yaw)"""
        t0 = 2.0 * (w * x + y * z)
        t1 = 1.0 - 2.0 * (x * x + y * y)
        roll = math.atan2(t0, t1)

        t2 = 2.0 * (w * y - z * x)
        t2 = max(-1.0, min(1.0, t2))  # Clamp to [-1, 1]
        pitch = math.asin(t2)

        t3 = 2.0 * (w * z + x * y)
        t4 = 1.0 - 2.0 * (y * y + z * z)
        yaw = math.atan2(t3, t4)

        return roll, pitch, yaw

def main(args=None):
    rclpy.init(args=args)
    node = DiffDrivePID()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()