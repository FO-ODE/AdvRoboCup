## Core Navigation Modules

### 1. `adjust_pose_to_object.py` - Object Alignment System
- **Main Function**: Automatically adjusts the robot's position relative to a target point.
- **Workflow**:
  - Automatically calls the `/adv_robocup/sam2clip/trigger` service to obtain the target point.
  - Listens to the `/adv_robocup/object_position` topic to receive target coordinates.
  - Performs coordinate transformations (any frame → `base_link` → `map`).
  - Calculates the robot's target position: **retreats a safe distance backward from the target point along the robot's orientation.**
  - Sends a navigation goal to `move_base` and automatically shuts down upon task completion.
- **Features**: Timestamp synchronization, multi-coordinate frame support, automatic service triggering.

### 2. `person_follower.py` - Person Following System
- **Main Function**: Enables the robot to follow a specific person.
- **Working Principle**:
  - Subscribes to the `/adv_robocup/waving_person/position` topic.
  - Calculates the distance to the person, maintaining a safe following distance of 1.3m.
  - Automatically orients towards the person being followed.
  - **One-shot Navigation**: The node automatically shuts down after reaching the target.

### 3. `move_to_pose.py` - Basic Pose Navigation
- **Main Function**: Moves the robot to a specified pose.
- **Supported Modes**:
  - `preset`: Moves to a predefined location.
  - `custom`: Specifies a custom location via ROS parameters.
  - `cancel`: Cancels the current navigation goal.
- **Application Scenarios**: Basic navigation, pose calibration.

## Head Control Modules

### 4. `head_tracking.py` - Head Target Tracking
- **Main Function**: Tracks a specified target point with the head.
- **Working Characteristics**:
  - Subscribes to `/adv_robocup/waving_person/position` for target tracking.
  - Performs horizontal tracking (pan) only, with the vertical angle fixed at 0.
  - **One-shot Action**: Automatically shuts down after tracking the first target.
  - **Angle Limits**: Within a ±90 degree range.

### 5. `head_scan.py` - Head Scanning System
- **Main Function**: Performs left-right head scanning for searching.
- **Control Method**:
  - Receives control commands via the `/head_scan_command` topic.
  - Supported Commands: `start`, `stop`, `center`.
- **Scan Modes**:
  - `continuous`: Continuous scanning mode.
  - `single`: Single-shot scanning mode.
- **Scan Parameters**: ±60 degree range, 10 seconds per scan, 1 second dwell time.

## System Characteristics

### Technical Architecture
- **ROS Node-based Design**: Each function is an independent node, facilitating modular combination and reuse.
- **Action Client Pattern**: Uses `move_base` and `FollowJointTrajectory` actions.
- **Coordinate Transformations**: Full TF2 support for automatic transformation across multiple coordinate frames.
- **Exception Handling**: Comprehensive error handling and status feedback.

### Operating Modes
- **One-shot Tasks**: Most nodes automatically shut down after completing their task.
- **Real-time Feedback**: Detailed log output and status monitoring.
- **Parameter-driven Configuration**: Key parameters can be dynamically configured via ROS parameters.
