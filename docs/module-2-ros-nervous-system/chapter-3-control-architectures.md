---
title: Chapter 3 - Control Architectures
sidebar_label: Chapter 3 - Control Architectures
---

# Chapter 3: Control Architectures

This chapter explores control architectures for humanoid robots within the ROS 2 framework, focusing on how different control patterns coordinate robot behavior and achieve complex tasks.

## Learning Objectives

After completing this chapter, you will be able to:

- Design and implement hierarchical control architectures for humanoid robots
- Apply different control patterns to various humanoid robot subsystems
- Integrate perception, planning, and control in a unified framework
- Implement fault-tolerant control architectures
- Evaluate the performance and stability of control systems

## Introduction to Control Architectures

Control architectures define how a robot's intelligence is organized and how decisions are made to achieve desired behaviors. In humanoid robotics, control architectures must handle complex interactions between many degrees of freedom while maintaining stability and achieving goals.

### Hierarchical Control Structure

Humanoid robots typically employ hierarchical control structures:

- **Task Level**: High-level task planning and execution
- **Motion Level**: Motion planning and trajectory generation
- **Trajectory Level**: Desired trajectories and reference generation
- **Servo Level**: Low-level feedback control for actuators

### Control Architecture Characteristics

Effective control architectures exhibit:

- **Modularity**: Components can be developed and tested independently
- **Scalability**: Ability to handle increasing complexity
- **Robustness**: Tolerance to sensor noise and disturbances
- **Real-time Capability**: Meeting timing constraints
- **Maintainability**: Ease of debugging and modification

## Classical Control Architectures

### Subsumption Architecture

A behavior-based architecture where simple behaviors subsume complex ones:

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

class SubsumptionController(Node):
    def __init__(self):
        super().__init__('subsumption_controller')

        # Publishers for different behaviors
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # Subscribers for sensor data
        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        self.emergency_stop_sub = self.create_subscription(Bool, '/emergency_stop', self.emergency_callback, 10)

        # Behavior priorities (higher value = higher priority)
        self.behavior_priorities = {
            'emergency_stop': 100,
            'collision_avoidance': 90,
            'goal_seeking': 50,
            'wander': 10
        }

        self.current_behavior = 'wander'
        self.obstacle_distance = float('inf')
        self.emergency_active = False

        # Timer for behavior arbitration
        self.arbitration_timer = self.create_timer(0.05, self.arbitrate_behaviors)

    def scan_callback(self, msg):
        # Simple obstacle detection
        if len(msg.ranges) > 0:
            self.obstacle_distance = min(msg.ranges)

    def emergency_callback(self, msg):
        self.emergency_active = msg.data

    def arbitrate_behaviors(self):
        if self.emergency_active:
            self.current_behavior = 'emergency_stop'
            self.execute_emergency_stop()
        elif self.obstacle_distance < 0.5:  # 50cm threshold
            self.current_behavior = 'collision_avoidance'
            self.execute_collision_avoidance()
        else:
            self.current_behavior = 'goal_seeking'  # Simplified for example
            self.execute_goal_seeking()

    def execute_emergency_stop(self):
        cmd = Twist()
        cmd.linear.x = 0.0
        cmd.angular.z = 0.0
        self.cmd_vel_pub.publish(cmd)

    def execute_collision_avoidance(self):
        cmd = Twist()
        cmd.linear.x = 0.0
        cmd.angular.z = 0.5  # Turn away from obstacle
        self.cmd_vel_pub.publish(cmd)

    def execute_goal_seeking(self):
        cmd = Twist()
        cmd.linear.x = 0.2
        cmd.angular.z = 0.0
        self.cmd_vel_pub.publish(cmd)
```

### Three-Layer Architecture

A layered approach with distinct control layers:

- **Behavior Layer**: High-level goals and plans
- **Executive Layer**: Task sequencing and execution
- **Reactive Layer**: Immediate response to environment

### Blackboard Architecture

A shared data space for different control modules:

- **Knowledge Sources**: Independent modules contributing to solution
- **Blackboard**: Shared workspace for data exchange
- **Control Component**: Coordinates module execution

## Modern Control Approaches in ROS 2

### Behavior Trees

Behavior trees provide a modern approach to organizing robot behavior:

```xml
<!-- behavior_tree.xml -->
<root main_tree_to_execute="MainTree">
    <BehaviorTree ID="MainTree">
        <Sequence name="RootSequence">
            <CheckBatteryCondition battery_threshold="20"/>
            <Fallback name="MainActions">
                <Sequence name="EmergencyStop">
                    <CheckEmergencyCondition/>
                    <StopRobotAction/>
                </Sequence>
                <Sequence name="NormalOperation">
                    <CheckGoalReceivedCondition/>
                    <MoveToGoalAction/>
                </Sequence>
                <WanderAction/>
            </Fallback>
        </Sequence>
    </BehaviorTree>
</root>
```

### State Machines

State machines provide clear behavior organization:

```python
from enum import Enum
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class HumanoidState(Enum):
    IDLE = "idle"
    WALKING = "walking"
    BALANCING = "balancing"
    STANDING_UP = "standing_up"
    EMERGENCY_STOP = "emergency_stop"

class StateMachineController(Node):
    def __init__(self):
        super().__init__('state_machine_controller')
        self.current_state = HumanoidState.IDLE
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # Timer for state machine update
        self.state_timer = self.create_timer(0.1, self.update_state_machine)

    def update_state_machine(self):
        # Check transition conditions
        new_state = self.evaluate_transitions()

        if new_state != self.current_state:
            self.exit_current_state(self.current_state)
            self.enter_new_state(new_state)
            self.current_state = new_state

        # Execute current state behavior
        self.execute_current_state()

    def evaluate_transitions(self):
        # Example transition logic
        if self.emergency_detected():
            return HumanoidState.EMERGENCY_STOP
        elif self.balance_lost():
            return HumanoidState.BALANCING
        elif self.goal_received() and self.ready_to_walk():
            return HumanoidState.WALKING
        elif self.standing():
            return HumanoidState.IDLE
        else:
            return self.current_state

    def execute_current_state(self):
        if self.current_state == HumanoidState.IDLE:
            self.execute_idle()
        elif self.current_state == HumanoidState.WALKING:
            self.execute_walking()
        elif self.current_state == HumanoidState.BALANCING:
            self.execute_balancing()
        # ... handle other states

    def execute_idle(self):
        cmd = Twist()
        cmd.linear.x = 0.0
        cmd.angular.z = 0.0
        self.cmd_vel_pub.publish(cmd)

    def execute_walking(self):
        cmd = Twist()
        cmd.linear.x = 0.3  # Forward at 0.3 m/s
        cmd.angular.z = 0.0
        self.cmd_vel_pub.publish(cmd)

    def execute_balancing(self):
        # Implement balancing control
        pass

    def emergency_detected(self):
        # Check for emergency conditions
        return False

    def balance_lost(self):
        # Check balance stability
        return False

    def goal_received(self):
        # Check if navigation goal is set
        return False

    def ready_to_walk(self):
        # Check if robot is ready to walk
        return True

    def standing(self):
        # Check if robot is standing
        return True
```

## Control Architecture Patterns

### Centralized Architecture

All control decisions made by a central authority:

- **Pros**: Coordinated, globally optimal decisions
- **Cons**: Single point of failure, computational bottleneck
- **Best for**: Small systems with tight coordination needs

### Distributed Architecture

Control decisions distributed among multiple nodes:

- **Pros**: Robust, scalable, modular
- **Cons**: Coordination challenges, potential conflicts
- **Best for**: Large systems with many components

### Hybrid Architecture

Combination of centralized and distributed elements:

- **Pros**: Best of both worlds, flexible
- **Cons**: Complexity in design and coordination
- **Best for**: Complex humanoid systems

### Example: Distributed Joint Control

```python
# JointControllerNode - runs on each actuator
import rclpy
from rclpy.node import Node
from control_msgs.msg import JointControllerState
from std_msgs.msg import Float64

class JointController(Node):
    def __init__(self, joint_name):
        super().__init__(f'{joint_name}_controller')
        self.joint_name = joint_name
        self.position = 0.0
        self.velocity = 0.0
        self.effort = 0.0

        # PID controller parameters
        self.kp = 10.0
        self.ki = 1.0
        self.kd = 0.1

        # Subscribers
        self.command_sub = self.create_subscription(
            Float64, f'/joint_commands/{joint_name}',
            self.command_callback, 10
        )

        self.state_sub = self.create_subscription(
            JointControllerState, f'/joint_states/{joint_name}',
            self.state_callback, 10
        )

        # Publishers
        self.control_pub = self.create_publisher(Float64, f'/joint_controls/{joint_name}', 10)

        # Control timer
        self.control_timer = self.create_timer(0.005, self.control_loop)  # 200 Hz

        self.target_position = 0.0
        self.integral_error = 0.0
        self.previous_error = 0.0

    def command_callback(self, msg):
        self.target_position = msg.data

    def state_callback(self, msg):
        self.position = msg.process_value
        self.velocity = msg.process_value_dot

    def control_loop(self):
        # Compute PID control
        error = self.target_position - self.position
        self.integral_error += error * 0.005  # dt = 0.005s
        derivative_error = (error - self.previous_error) / 0.005

        control_output = (self.kp * error +
                         self.ki * self.integral_error +
                         self.kd * derivative_error)

        self.previous_error = error

        # Publish control signal
        cmd_msg = Float64()
        cmd_msg.data = control_output
        self.control_pub.publish(cmd_msg)
```

### Centralized Whole-Body Controller

```python
# WholeBodyControllerNode - centralized control
import rclpy
from rclpy.node import Node
import numpy as np
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray

class WholeBodyController(Node):
    def __init__(self):
        super().__init__('whole_body_controller')

        # Robot model (simplified for example)
        self.joint_names = [
            'hip_pitch_left', 'hip_roll_left', 'hip_yaw_left',
            'knee_left', 'ankle_pitch_left', 'ankle_roll_left',
            'hip_pitch_right', 'hip_roll_right', 'hip_yaw_right',
            'knee_right', 'ankle_pitch_right', 'ankle_roll_right'
        ]

        self.num_joints = len(self.joint_names)
        self.current_positions = np.zeros(self.num_joints)
        self.current_velocities = np.zeros(self.num_joints)
        self.target_positions = np.zeros(self.num_joints)

        # Subscribers
        self.joint_state_sub = self.create_subscription(
            JointState, '/joint_states', self.joint_state_callback, 10
        )

        # Publishers
        self.joint_command_pub = self.create_publisher(
            Float64MultiArray, '/joint_commands', 10
        )

        # Control timer
        self.control_timer = self.create_timer(0.002, self.control_loop)  # 500 Hz

    def joint_state_callback(self, msg):
        for i, name in enumerate(self.joint_names):
            try:
                idx = msg.name.index(name)
                self.current_positions[i] = msg.position[idx]
                if idx < len(msg.velocity):
                    self.current_velocities[i] = msg.velocity[idx]
            except ValueError:
                # Joint name not found in message
                pass

    def control_loop(self):
        # Compute whole-body control law (simplified)
        self.target_positions = self.compute_target_positions()

        # Apply constraints (balance, joint limits, etc.)
        constrained_targets = self.apply_constraints(self.target_positions)

        # Publish joint commands
        cmd_msg = Float64MultiArray()
        cmd_msg.data = constrained_targets.tolist()
        self.joint_command_pub.publish(cmd_msg)

    def compute_target_positions(self):
        # Simplified whole-body control computation
        # In practice, this would use kinematic/dynamic models
        return self.target_positions + 0.001 * np.sin(self.get_clock().now().nanoseconds / 1e9)  # Small demo movement

    def apply_constraints(self, targets):
        # Apply joint limits and balance constraints
        # This is where whole-body inverse kinematics would be applied
        return np.clip(targets, -np.pi, np.pi)  # Simplified joint limits
```

## Perception-Action Integration

### Sensor Integration Framework

Integrating multiple sensors for control decisions:

```python
class SensorIntegrationNode(Node):
    def __init__(self):
        super().__init__('sensor_integration')

        # Sensor subscribers
        self.imu_sub = self.create_subscription(Imu, '/imu/data', self.imu_callback, 10)
        self.joint_state_sub = self.create_subscription(JointState, '/joint_states', self.joint_state_callback, 10)
        self.force_torque_sub = self.create_subscription(WrenchStamped, '/ft_sensor', self.ft_callback, 10)

        # Publisher for fused state
        self.state_pub = self.create_publisher(HumanoidState, '/robot_state', 10)

        # State estimation variables
        self.orientation = [0.0, 0.0, 0.0, 1.0]  # Quaternion
        self.angular_velocity = [0.0, 0.0, 0.0]
        self.joint_positions = []
        self.contact_force = [0.0, 0.0, 0.0]

        # State estimation timer
        self.estimation_timer = self.create_timer(0.01, self.estimate_state)

    def estimate_state(self):
        # Fuse sensor data to estimate robot state
        # In practice, this would use Kalman filters or other state estimation techniques
        fused_state = self.perform_sensor_fusion()

        # Publish fused state
        state_msg = HumanoidState()
        # Populate with fused state data
        self.state_pub.publish(state_msg)

    def perform_sensor_fusion(self):
        # Implementation of sensor fusion algorithm
        # e.g., Extended Kalman Filter, Unscented Kalman Filter, or particle filter
        pass
```

### Feedback Control with Sensing

Closing the perception-action loop:

```python
class FeedbackController(Node):
    def __init__(self):
        super().__init__('feedback_controller')

        # Subscribe to fused robot state
        self.state_sub = self.create_subscription(
            HumanoidState, '/robot_state', self.state_callback, 10
        )

        # Publisher for control commands
        self.cmd_pub = self.create_publisher(Float64MultiArray, '/joint_commands', 10)

        # Control parameters
        self.balance_control_enabled = True
        self.target_com_position = [0.0, 0.0, 0.8]  # Target center of mass

        # Control timer
        self.control_timer = self.create_timer(0.005, self.control_loop)

    def state_callback(self, msg):
        self.current_state = msg

    def control_loop(self):
        if self.balance_control_enabled:
            # Compute balance control commands based on current state
            control_commands = self.compute_balance_control(self.current_state)

            # Publish control commands
            cmd_msg = Float64MultiArray()
            cmd_msg.data = control_commands
            self.cmd_pub.publish(cmd_msg)

    def compute_balance_control(self, state):
        # Implement balance control algorithm
        # e.g., Linear Inverted Pendulum Mode (LIPM), Capture Point, or MPC
        pass
```

## Planning-Action Integration

### Motion Planning Integration

Connecting planning and control:

```python
class PlanningControlInterface(Node):
    def __init__(self):
        super().__init__('planning_control_interface')

        # Planning service client
        self.plan_client = self.create_client(GetMotionPlan, '/get_motion_plan')

        # Trajectory follower client
        self.traj_client = self.create_client(FollowTrajectory, '/follow_trajectory')

        # Current state subscriber
        self.state_sub = self.create_subscription(HumanoidState, '/robot_state', self.state_callback, 10)

        # Planning timer
        self.planning_timer = self.create_timer(5.0, self.request_plan)  # Plan every 5 seconds

    def request_plan(self):
        if self.plan_client.service_is_ready():
            request = GetMotionPlan.Request()
            # Set planning parameters based on current state
            request.start_state = self.current_state
            request.goal_state = self.compute_next_goal()

            future = self.plan_client.call_async(request)
            future.add_done_callback(self.plan_response_callback)

    def plan_response_callback(self, future):
        try:
            response = future.result()
            if response.success:
                # Execute planned trajectory
                self.execute_trajectory(response.trajectory)
        except Exception as e:
            self.get_logger().error(f'Planning service call failed: {e}')

    def execute_trajectory(self, trajectory):
        if self.traj_client.service_is_ready():
            request = FollowTrajectory.Request()
            request.trajectory = trajectory

            future = self.traj_client.call_async(request)
            future.add_done_callback(self.trajectory_response_callback)
```

## Control System Stability

### Stability Analysis

Ensuring control system stability:

```python
class StabilityMonitor(Node):
    def __init__(self):
        super().__init__('stability_monitor')

        # Subscribers
        self.state_sub = self.create_subscription(HumanoidState, '/robot_state', self.state_callback, 10)
        self.control_sub = self.create_subscription(JointCommand, '/joint_commands', self.control_callback, 10)

        # Stability monitoring timer
        self.monitor_timer = self.create_timer(0.1, self.check_stability)

        self.stability_metrics = {}
        self.is_stable = True

    def check_stability(self):
        # Compute stability metrics
        balance_margin = self.compute_balance_margin()
        control_effort = self.compute_control_effort()
        state_variance = self.compute_state_variance()

        # Check stability criteria
        self.is_stable = (balance_margin > 0.05 and  # At least 5cm stability margin
                         control_effort < 100.0 and   # Reasonable control effort
                         state_variance < 1.0)        # Stable state

        if not self.is_stable:
            self.handle_instability()

    def compute_balance_margin(self):
        # Compute distance to stability boundary (e.g., Zero Moment Point)
        pass

    def compute_control_effort(self):
        # Compute norm of control signals
        pass

    def compute_state_variance(self):
        # Compute variance of state variables over time window
        pass

    def handle_instability(self):
        # Implement emergency stabilization procedures
        self.get_logger().warn('Instability detected! Initiating stabilization...')
        # Send stabilization commands
```

## Fault-Tolerant Control

### Fault Detection and Isolation

Implementing fault tolerance in control systems:

```python
class FaultTolerantController(Node):
    def __init__(self):
        super().__init__('fault_tolerant_controller')

        # Monitor various system components
        self.joint_monitors = {}
        self.sensor_monitors = {}
        self.actuator_monitors = {}

        # Initialize monitors for each joint
        for joint_name in self.joint_names:
            self.joint_monitors[joint_name] = JointMonitor(joint_name)

        # Fault response timer
        self.fault_check_timer = self.create_timer(0.05, self.check_faults)

    def check_faults(self):
        # Check for any component failures
        active_faults = []

        for joint_name, monitor in self.joint_monitors.items():
            fault_status = monitor.check_status()
            if fault_status['is_faulted']:
                active_faults.append({
                    'component': f'joint_{joint_name}',
                    'type': fault_status['fault_type'],
                    'severity': fault_status['severity']
                })

        if active_faults:
            self.handle_faults(active_faults)

    def handle_faults(self, faults):
        for fault in faults:
            if fault['severity'] == 'critical':
                self.initiate_emergency_procedure()
            elif fault['severity'] == 'warning':
                self.adjust_control_strategy()
            elif fault['severity'] == 'info':
                self.log_event(fault)

    def initiate_emergency_procedure(self):
        # Implement emergency safe state
        pass

    def adjust_control_strategy(self):
        # Adjust control to accommodate faults
        # e.g., redistribute load, use backup actuators
        pass
```

### Graceful Degradation

Maintaining functionality despite component failures:

```python
class GracefulDegradationController(Node):
    def __init__(self):
        super().__init__('graceful_degradation_controller')

        # Define functionality levels
        self.functional_levels = {
            'full': ['walk', 'balance', 'manipulate', 'perceive'],
            'degraded': ['balance', 'perceive'],  # Can maintain balance and perceive
            'minimal': ['stabilize'],  # Can only maintain basic stability
            'emergency': ['shutdown']  # Emergency shutdown required
        }

        self.current_level = 'full'

    def update_functionality_level(self, available_components):
        # Determine highest possible functionality level based on available components
        if self.can_perform('walk', available_components):
            self.current_level = 'full'
        elif self.can_perform('balance', available_components):
            self.current_level = 'degraded'
        elif self.can_perform('stabilize', available_components):
            self.current_level = 'minimal'
        else:
            self.current_level = 'emergency'

        # Switch to appropriate control mode
        self.switch_control_mode(self.current_level)

    def can_perform(self, capability, available_components):
        # Check if capability can be performed with available components
        pass

    def switch_control_mode(self, level):
        # Switch control algorithms based on current capability level
        if level == 'full':
            self.activate_full_control()
        elif level == 'degraded':
            self.activate_degraded_control()
        elif level == 'minimal':
            self.activate_minimal_control()
        elif level == 'emergency':
            self.activate_emergency_shutdown()
```

## Learning Activities

### Activity 1: Behavior Tree Implementation

1. Implement a simple behavior tree for humanoid walking
2. Include conditions for obstacle detection, balance maintenance, and goal achievement
3. Test the behavior tree with simulated sensor inputs
4. Evaluate the effectiveness of different behavior combinations

### Activity 2: Distributed Control

1. Create multiple nodes for different control aspects (position, balance, perception)
2. Connect them using ROS 2 messaging
3. Test coordination between nodes
4. Analyze performance under communication delays

### Activity 3: Fault Tolerance

1. Simulate a joint failure in a control system
2. Implement fault detection mechanisms
3. Design a graceful degradation strategy
4. Test system behavior with the simulated fault

## Advanced Control Topics

### Adaptive Control

Adapting to changing conditions and parameters:

- **Model Reference Adaptive Control**: Adjusting to match reference model
- **Self-Tuning Regulators**: Automatic parameter adjustment
- **Gain Scheduling**: Adjusting gains based on operating conditions
- **Learning Control**: Improving performance over time

### Optimal Control

Optimizing control performance:

- **Model Predictive Control (MPC)**: Optimizing over prediction horizon
- **Linear Quadratic Regulator (LQR)**: Optimal control for linear systems
- **Nonlinear Model Predictive Control**: For nonlinear humanoid dynamics
- **Reinforcement Learning**: Learning optimal control policies

### Robust Control

Handling uncertainties and disturbances:

- **H-infinity Control**: Optimizing worst-case performance
- **Sliding Mode Control**: Robust to matched uncertainties
- **Gain Scheduling**: Handling operating point variations
- **Robust Optimization**: Optimizing against parameter uncertainties

## Summary

Control architectures form the decision-making backbone of humanoid robots, determining how the robot processes information, makes decisions, and executes actions. Effective control architectures must balance modularity, performance, robustness, and maintainability while meeting the demanding real-time requirements of humanoid robots.

The choice of control architecture significantly impacts the robot's behavior, performance, and ability to handle complex tasks. Modern humanoid robotics increasingly employs hybrid approaches that combine the benefits of different architectural patterns.

## Further Reading

- Siciliano, B., & Khatib, O. (2016). Springer Handbook of Robotics
- "Robotics: Control, Sensing, Vision, and Intelligence" by Fu, Gonzalez, and Lee
- "Applied Nonlinear Control" by Slotine and Li
- "Introduction to Autonomous Manipulation" by Torras, Ibáñez, and Sandoval