---
title: Chapter 2 - Message Passing
sidebar_label: Chapter 2 - Message Passing
---

# Chapter 2: Message Passing

This chapter explores message passing patterns in ROS 2, focusing on how information flows through humanoid robot systems to coordinate complex behaviors.

## Learning Objectives

After completing this chapter, you will be able to:

- Implement various message passing patterns in ROS 2
- Design efficient communication architectures for humanoid robots
- Apply Quality of Service (QoS) policies for different message types
- Optimize message passing for real-time humanoid robot control
- Evaluate the performance of different communication patterns

## Introduction to Message Passing in ROS 2

Message passing is the fundamental communication mechanism in ROS 2, enabling nodes to exchange information asynchronously. In humanoid robotics, message passing orchestrates the complex interactions between sensors, controllers, and actuators.

### The Publish-Subscribe Pattern

The publish-subscribe pattern enables one-to-many communication:

- **Publishers**: Send messages to topics without knowing subscribers
- **Subscribers**: Receive messages from topics without knowing publishers
- **Topics**: Named channels for message routing
- **Messages**: Typed data structures exchanged between nodes

### Asynchronous Communication

ROS 2 communication is inherently asynchronous:

- **Decoupled Timing**: Publishers and subscribers operate independently
- **Buffering**: Messages are buffered to handle timing differences
- **Queue Management**: Configurable queue sizes for message buffering
- **Callback Processing**: Messages processed in callback functions

## Message Types and Structures

### Standard Message Types

ROS 2 provides standard message types for common robotics applications:

#### Sensor Messages

- **sensor_msgs**: Camera, LIDAR, IMU, and other sensor data
- **geometry_msgs**: Position, orientation, and motion data
- **nav_msgs**: Navigation-related information
- **diagnostic_msgs**: System diagnostic information

#### Control Messages

- **std_msgs**: Basic data types (int, float, string, etc.)
- **trajectory_msgs**: Trajectory and path specifications
- **control_msgs**: Controller state and command messages
- **actionlib_msgs**: Action-related messages

### Custom Message Types

Creating custom messages for humanoid robot applications:

```c
# JointCommand.msg
string[] joint_names
float64[] positions
float64[] velocities
float64[] efforts

# HumanoidState.msg
Header header
geometry_msgs/Pose base_pose
float64[] joint_positions
float64[] joint_velocities
float64[] joint_efforts
geometry_msgs/Wrench[] foot_forces
sensor_msgs/Imu imu_data

# WalkingPattern.msg
string pattern_type
float64[] step_positions
float64[] step_timings
float64[] support_phases
```

### Message Definition Best Practices

Designing effective message types:

- **Minimal Data**: Include only necessary information
- **Clear Semantics**: Well-defined meaning for each field
- **Forward Compatibility**: Plan for future extensions
- **Performance Considerations**: Optimize for bandwidth and processing

## Communication Patterns for Humanoid Robots

### Sensor Data Broadcasting

Humanoid robots broadcast sensor data to multiple consumers:

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from std_msgs.msg import Float64MultiArray

class SensorNode(Node):
    def __init__(self):
        super().__init__('sensor_node')

        # Publisher for IMU data (many consumers)
        self.imu_publisher = self.create_publisher(
            Imu,
            '/imu/data',
            10  # Queue size
        )

        # Publisher for joint encoder data
        self.joint_publisher = self.create_publisher(
            Float64MultiArray,
            '/joint_states',
            10
        )

        # Timer for periodic sensor data publication
        self.timer = self.create_timer(
            0.01,  # 100 Hz for sensor data
            self.publish_sensor_data
        )

    def publish_sensor_data(self):
        # Publish IMU data
        imu_msg = Imu()
        # ... populate message with sensor data ...
        self.imu_publisher.publish(imu_msg)

        # Publish joint states
        joint_msg = Float64MultiArray()
        # ... populate with joint positions ...
        self.joint_publisher.publish(joint_msg)
```

### Control Command Distribution

Distributing control commands to multiple actuators:

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray

class ControllerNode(Node):
    def __init__(self):
        super().__init__('controller_node')

        # Publisher for joint commands
        self.joint_cmd_publisher = self.create_publisher(
            Float64MultiArray,
            '/joint_commands',
            1  # Minimal queue for real-time control
        )

        # Timer for control loop
        self.control_timer = self.create_timer(
            0.005,  # 200 Hz for control
            self.compute_and_publish_commands
        )

    def compute_and_publish_commands(self):
        # Compute control commands
        commands = self.compute_control_signals()

        # Publish commands
        cmd_msg = Float64MultiArray()
        cmd_msg.data = commands
        self.joint_cmd_publisher.publish(cmd_msg)
```

### Coordination Messages

Higher-level coordination between subsystems:

```python
# Custom message for task coordination
# TaskCommand.msg
string task_type
string[] target_joints
float64[] target_positions
duration execution_time
bool blocking

class TaskScheduler(Node):
    def __init__(self):
        super().__init__('task_scheduler')

        # Publisher for task commands
        self.task_publisher = self.create_publisher(
            TaskCommand,
            '/task_commands',
            5  # Queue for upcoming tasks
        )

        # Subscriber for task status
        self.status_subscriber = self.create_subscription(
            String,
            '/task_status',
            self.task_status_callback,
            5
        )
```

## Quality of Service (QoS) Policies

### QoS Configuration

QoS policies control communication behavior:

```python
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy

# High-frequency sensor data (best-effort, volatile)
SENSOR_QOS = QoSProfile(
    depth=5,
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.VOLATILE,
    history=HistoryPolicy.KEEP_LAST
)

# Critical control commands (reliable, transient)
CONTROL_QOS = QoSProfile(
    depth=1,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
    history=HistoryPolicy.KEEP_LAST
)

# Configuration parameters (durable, keep-all)
CONFIG_QOS = QoSProfile(
    depth=10,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
    history=HistoryPolicy.KEEP_ALL
)
```

### QoS for Different Message Types

Applying QoS based on message criticality:

#### Critical Control Messages

- **Reliability**: RELIABLE - guaranteed delivery
- **Durability**: TRANSIENT_LOCAL - available to late joiners
- **History**: KEEP_LAST with minimal depth
- **Depth**: 1 for real-time control

#### Sensor Data

- **Reliability**: BEST_EFFORT for high-frequency data
- **Durability**: VOLATILE for real-time data
- **History**: KEEP_LAST with appropriate depth
- **Depth**: Based on processing delay tolerance

#### Configuration Data

- **Reliability**: RELIABLE for important parameters
- **Durability**: TRANSIENT_LOCAL for availability
- **History**: KEEP_ALL for complete history
- **Depth**: Large enough for complete history

### Advanced QoS Settings

Fine-tuning communication for humanoid robots:

```python
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSReliabilityPolicy
from rclpy.qos import QoSProfile, QoSLivelinessPolicy

# High-performance control (low latency)
HIGH_PERFORMANCE_CONTROL = QoSProfile(
    depth=1,
    reliability=QoSReliabilityPolicy.RELIABLE,
    durability=QoSDurabilityPolicy.VOLATILE,
    history=QoSHistoryPolicy.KEEP_LAST,
    deadline=Duration(seconds=0, nanoseconds=5000000),  # 5ms deadline
    liveliness=QoSLivelinessPolicy.AUTOMATIC,
    lifespan=Duration(seconds=1)
)

# Reliable state synchronization
RELIABLE_STATE_SYNC = QoSProfile(
    depth=10,
    reliability=QoSReliabilityPolicy.RELIABLE,
    durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
    history=QoSHistoryPolicy.KEEP_LAST,
    deadline=Duration(seconds=1),
    liveliness=QoSLivelinessPolicy.MANUAL_BY_TOPIC,
    liveliness_lease_duration=Duration(seconds=10)
)
```

## Message Passing Patterns

### Fan-Out Pattern

One publisher, many subscribers:

```python
class SensorHub(Node):
    def __init__(self):
        super().__init__('sensor_hub')

        # Single publisher for sensor data
        self.sensor_publisher = self.create_publisher(
            HumanoidState,
            '/robot_state',
            10
        )

        # Multiple subscribers will receive the same data
        # - Balance controller
        # - Navigation system
        # - Diagnostic system
        # - Visualization tools
```

### Fan-In Pattern

Many publishers, one subscriber:

```python
class CentralController(Node):
    def __init__(self):
        super().__init__('central_controller')

        # Multiple sensor inputs
        self.imu_subscriber = self.create_subscription(
            Imu,
            '/imu/data',
            self.imu_callback,
            SENSOR_QOS
        )

        self.joint_subscriber = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_callback,
            SENSOR_QOS
        )

        self.lidar_subscriber = self.create_subscription(
            LaserScan,
            '/scan',
            self.lidar_callback,
            SENSOR_QOS
        )

    def sensor_fusion_callback(self):
        # Combine all sensor inputs for control decisions
        fused_state = self.fuse_sensors()
        control_command = self.compute_control(fused_state)
        # ... publish control command ...
```

### Pipeline Pattern

Sequential message processing:

```python
# Node 1: Raw sensor processing
class SensorProcessor(Node):
    def __init__(self):
        super().__init__('sensor_processor')
        self.pub = self.create_publisher(SensorProcessed, '/sensor_processed', 10)

# Node 2: Perception
class PerceptionNode(Node):
    def __init__(self):
        super().__init__('perception_node')
        self.sub = self.create_subscription(SensorProcessed, '/sensor_processed', self.callback, 10)
        self.pub = self.create_publisher(PerceptionResult, '/perception_result', 10)

# Node 3: Planning
class PlannerNode(Node):
    def __init__(self):
        super().__init__('planner_node')
        self.sub = self.create_subscription(PerceptionResult, '/perception_result', self.callback, 10)
        self.pub = self.create_publisher(MotionPlan, '/motion_plan', 10)
```

## Performance Optimization

### Message Serialization

Optimizing message serialization for humanoid robots:

- **Compact Representations**: Use efficient data structures
- **Binary Formats**: Leverage binary serialization
- **Compression**: Consider compression for large messages
- **Pre-allocation**: Pre-allocate message objects

### Bandwidth Management

Managing network bandwidth:

- **Sampling Rates**: Appropriate rates for different data types
- **Filtering**: Remove unnecessary data before transmission
- **Aggregation**: Combine related data into single messages
- **Prioritization**: Prioritize critical messages

### Memory Management

Efficient memory usage:

```python
class OptimizedNode(Node):
    def __init__(self):
        super().__init__('optimized_node')

        # Pre-allocate message objects
        self.preallocated_msg = JointCommand()
        self.preallocated_msg.joint_names = ['joint1', 'joint2', 'joint3']  # Fixed size

        # Use message pools to reduce allocation
        self.message_pool = []
        for _ in range(10):
            self.message_pool.append(HumanoidState())

        self.pool_index = 0

    def reuse_message(self):
        # Reuse pre-allocated message
        msg = self.message_pool[self.pool_index]
        self.pool_index = (self.pool_index + 1) % len(self.message_pool)
        # Reset message content before reuse
        return msg
```

## Real-Time Considerations

### Deterministic Communication

Ensuring deterministic message passing:

- **Deadline Contracts**: Messages must arrive within deadlines
- **Jitter Minimization**: Consistent timing for control loops
- **Priority Scheduling**: Critical messages get priority
- **Memory Locking**: Prevent page faults during real-time operation

### Timing Constraints

Meeting timing requirements for humanoid robots:

- **Control Loop Timing**: Typically 100Hz-1kHz for control
- **Sensor Update Rates**: Match sensor capabilities
- **Communication Delays**: Account for network and processing delays
- **Synchronization**: Coordinate timing across nodes

## Error Handling and Fault Tolerance

### Communication Failures

Handling communication failures gracefully:

```python
class RobustSubscriber(Node):
    def __init__(self):
        super().__init__('robust_subscriber')

        self.subscription = self.create_subscription(
            JointState,
            '/joint_states',
            self.safe_callback,
            10
        )

        # Timer to monitor message arrival
        self.last_message_time = self.get_clock().now()
        self.monitor_timer = self.create_timer(1.0, self.check_communication_health)

    def safe_callback(self, msg):
        try:
            # Process message safely
            self.process_joint_data(msg)
            self.last_message_time = self.get_clock().now()
        except Exception as e:
            self.get_logger().error(f'Error processing message: {e}')
            # Continue operation with last known good state

    def check_communication_health(self):
        current_time = self.get_clock().now()
        time_since_last = current_time - self.last_message_time

        if time_since_last.nanoseconds > 2e9:  # 2 seconds
            self.get_logger().warn('Communication timeout detected')
            # Implement recovery strategy
```

### Graceful Degradation

Implementing graceful degradation:

- **Fallback Modes**: Safe states when communication fails
- **Reduced Functionality**: Continue with degraded performance
- **Recovery Procedures**: Automatic recovery when possible
- **User Notification**: Inform operators of degraded state

## Advanced Message Passing Techniques

### Shared Memory

Using shared memory for high-performance communication:

- **Intra-process Communication**: Direct memory sharing between nodes in same process
- **Inter-process Communication**: Shared memory between processes
- **Zero-Copy Communication**: Eliminate memory copying overhead
- **Real-Time Performance**: Reduce latency and jitter

### Multicast Communication

Efficient broadcasting to multiple recipients:

- **UDP Multicast**: Network-level multicast for sensor data
- **DDS Multicast**: Middleware-level multicast support
- **Load Balancing**: Distribute processing across multiple nodes
- **Redundancy**: Multiple receivers for critical data

## Debugging and Monitoring

### Message Inspection

Tools for inspecting message passing:

- **ros2 topic echo**: View message contents in real-time
- **ros2 topic hz**: Monitor message frequency
- **ros2 bag**: Record and replay messages
- **Custom monitoring tools**: Application-specific inspection

### Performance Monitoring

Monitoring communication performance:

```python
class PerformanceMonitor(Node):
    def __init__(self):
        super().__init__('performance_monitor')

        self.message_times = {}
        self.message_counts = {}

        # Monitor specific topics
        self.monitored_topics = ['/joint_commands', '/sensor_data', '/control_status']

        for topic in self.monitored_topics:
            self.create_subscription(
                String,  # Generic type for monitoring
                topic,
                lambda msg, t=topic: self.monitor_message(t),
                1
            )

    def monitor_message(self, topic_name):
        current_time = self.get_clock().now()

        if topic_name not in self.message_times:
            self.message_times[topic_name] = current_time
            self.message_counts[topic_name] = 0
        else:
            interval = current_time - self.message_times[topic_name]
            self.message_times[topic_name] = current_time
            self.message_counts[topic_name] += 1

            # Log performance metrics
            self.get_logger().debug(f'{topic_name} interval: {interval.nanoseconds / 1e6:.2f}ms')
```

## Learning Activities

### Activity 1: QoS Optimization

1. Create a publisher with default QoS settings
2. Measure message delivery performance
3. Adjust QoS settings based on requirements
4. Compare performance with different QoS configurations

### Activity 2: Message Pipeline

1. Implement a three-node pipeline for sensor processing
2. Use appropriate QoS settings for each connection
3. Monitor message flow and timing
4. Optimize for minimal latency

### Activity 3: Fault Tolerance

1. Implement error handling in a message passing system
2. Simulate communication failures
3. Test graceful degradation mechanisms
4. Verify system recovery procedures

## Summary

Message passing in ROS 2 provides the communication backbone for humanoid robot systems. By understanding and properly applying publish-subscribe patterns, Quality of Service policies, and performance optimization techniques, you can build robust, efficient communication systems for complex humanoid robots.

The key to effective message passing lies in matching communication patterns to application requirements, applying appropriate QoS settings, and implementing proper error handling and monitoring.

## Further Reading

- ROS 2 QoS Documentation: https://docs.ros.org/en/rolling/Concepts/About-Quality-of-Service-Settings.html
- DDS Quality of Service: https://www.omg.org/spec/DDS/1.4/
- Real-Time Communication in Robotics: "Real-Time Systems for Robotics" by Shin and Krishna
- Performance Optimization: "High-Performance Communication for Robotics" by various authors