# Robotics Research Plan: Autonomous Pick-and-Place with SO-ARM101

This document outlines a multi-phase research plan to develop and evaluate autonomous pick-and-place capabilities using the SO-ARM101 robotic platform, integrated with the LeRobot framework. The research progresses through four phases: arm-only manipulation, base-only navigation, combined arm-base operations, and dual-arm coordination. Each phase builds on the previous, culminating in a sophisticated bimanual system.

## Phase 1: Arm-Only Manipulation (“Ball to Cup”)

**Objective:** Achieve precise pick-and-place of a ball into a cup using a stationary robotic arm, trained via joystick teleoperation demonstrations.

**Sensors:**
- **Wrist RGB Camera (cam_wrist):** Primary vision input for grasping.
- **Head Depth Camera (cam_head_depth):** Optional depth data for enhanced perception.
- Camera intrinsics and extrinsics recorded in `info.json` per LeRobot dataset specifications.

**Control:**
- Action space: `[Δq1..Δq6, gripper_cmd]`, representing joint position deltas and binary gripper commands (open/close).
- Joystick mapping:
  - Left stick: End-effector (EE) XY motion via inverse kinematics.
  - Right stick: Z-axis and yaw control.
  - Bumpers: Roll and pitch adjustments.
  - A/B buttons: Start/stop recording.
  - X/Y buttons: Gripper open/close.
- Alternative: Lightweight ROS interface for end-effector velocity teleoperation using “lerobot-ros.”

**Data Collection:**
- Utilize LeRobot’s real-robot pipeline for recording, training, and evaluation.
- Collect 200–500 short episodes (10–20 seconds each), capturing variations in ball and cup positions.
- Each episode includes approach, grasp, lift, and place actions.

**Policy and Training:**
- Employ Behavioral Cloning (BC) with the Action Chunking Transformer (ACT) policy as a baseline.
- Example command: `lerobot-train --dataset.repo_id=<your/so_arm101_ballcup> --policy.type=act --output_dir=outputs/act_ballcup`.

**Success Criteria:**
- Achieve ≥80% success rate on held-out test scenarios with minor distractor and background variations.

## Phase 2: Base-Only Navigation (“Go to Ball / Go to Cup”)

**Objective:** Train the omnidirectional base to navigate to optimal pre-grasp and pre-place poses for the arm.

**Sensors:**
- **Head RGB-D Camera:** Primary input for target localization.
- Optional AprilTag on cup for early training stability.
- Observations: `[x, y, θ]` (odometry) and target pose in base frame, stored in dataset.

**Control:**
- Action space: `[vx, vy, ω]` (linear and angular velocities) at 10–20 Hz.

**Data Collection:**
- Teleoperate the base to pre-grasp and pre-place poses using fixed offsets from ball/cup positions.
- Record episodes using LeRobot’s pipeline.

**Policy:**
- Train a navigation policy via BC to output base velocities until within a tolerance radius of the target pose.
- Leverage LeRobot’s dataset and policy tools for consistency.

## Phase 3: Integrated Arm and Base Operation

**Objective:** Combine arm and base policies into a hierarchical system, with an optional upgrade to a Vision-Language-Action (VLA) policy.

**Controller Architecture:**
- **Hierarchical Finite-State Machine (FSM):**
  - States: `SEARCH → NAV_TO_BALL → PICK → NAV_TO_CUP → PLACE → DONE`.
  - Navigation states invoke Phase 2 base policy; pick/place states invoke Phase 1 arm policy.
- **VLA Upgrade (Future):**
  - Replace FSM with a VLA policy to select skills based on vision and language context.
  - Reuse trained ACT skills as primitives within the VLA framework.

**Training:**
- Start with FSM for rapid integration, then fine-tune end-to-end or incorporate VLA selector.

## Phase 4: Dual-Arm Coordination on Shared Base

**Objective:** Extend to bimanual tasks with both arms on the SO-ARM101 platform, coordinating with the base.

**Hardware:**
- SO-ARM101: Two 6-DOF arms with bus servos, mounted on an omnidirectional base.
- Sensors:
  - **Head:** One RGB-D camera.
  - **Wrists:** One RGB camera per arm (cam_wrist_left, cam_wrist_right).
  - Calibrate intrinsics, extrinsics, and hand-eye transformations (`T_cam→ee`) for all cameras, stored in `info.json`.

**Control:**
- Action space: Concatenation of `[A_left: Δq1..Δq6, gripL]`, `[A_right: Δq1..Δq6, gripR]`, and `[A_base: vx, vy, ω]`.
- Initial tasks: Bimanual primitives (e.g., left arm stabilizes cup, right arm places ball).
- Progress to cooperative grasps with constraint-based coordination.

**Coordination:**
- Use a hierarchical selector (rule-based or VLA) to assign skills to each arm and base.
- Start with time-boxed sequencing, then advance to constraint-based coordination (e.g., maintaining relative wrist poses).

**Training:**
- Train individual arm primitives separately, then collect a bimanual dataset.
- Fine-tune a multi-head policy with separate MLP heads per arm, sharing a visual backbone.

## Dataset Structure (All Phases)

Each phase’s dataset is organized as a repository (local or Hugging Face Hub) with the following structure:
- **Observations:**
  - Per arm: `qpos`, `qvel`, `gripper_state`.
  - Base (if applicable): `base_pose`.
  - Cameras: `cam_head_rgb`, `cam_head_depth` (optional), `cam_wrist_left_rgb`, `cam_wrist_right_rgb`.
- **Actions:** Arrays matching the control vector per timestep.
- **Timestamps:** Monotonic time for synchronization.
- **info.json:**
  - Action definitions (e.g., “joint position deltas + binary gripper”).
  - Camera intrinsics, extrinsics, and hand-eye transformations.
  - Units and control rates.

## Teleoperation Framework

- **Gamepad Interface:** Use a ROS teleop bridge to map joystick inputs to end-effector velocity or joint jog commands, integrated with LeRobot’s control and recording pipeline.
- Follow LeRobot’s standard workflow: record → train → evaluate.

## Calibration and Synchronization

1. **Base-to-Arm Transform:** Measure `T_base→arm` for each arm.
2. **Hand-Eye Calibration:** Compute `T_ee→cam` for wrist cameras using AprilTag or charuco boards.
3. **Head Camera Extrinsics:** Determine `T_base→cam_head`.
4. **Time Synchronization:** Ensure all data streams use a monotonic clock or corrected offsets.
5. **Latency Logging:** Record per-sensor latency in `info.json` for aligned training data.

## Safety and Reliability

- Implement speed and torque limits per phase.
- Bind emergency stop to a dedicated joystick button.
- Define “no-go” zones (20–30 cm radius around cup) during base motion.
- Gripper interlocks: Require positive ball contact (via depth or contact switch) before closing.

## Research Questions

1. How effectively can Behavioral Cloning with ACT policies achieve ≥80% success in arm-only pick-and-place tasks under varying environmental conditions?
2. What is the impact of AprilTag-based localization on the stability and convergence speed of base navigation training in Phase 2?
3. How does a hierarchical FSM compare to a VLA policy in terms of task success rate and adaptability in integrated arm-base operations (Phase 3)?
4. What are the performance trade-offs between time-boxed sequencing and constraint-based coordination in bimanual tasks during Phase 4?
5. How does the inclusion of depth data from the head camera influence the robustness of the arm’s grasping policy across all phases?

## References

1. [LeRobot Dataset Format - phospho starter pack documentation](https://docs.phospho.ai/learn/lerobot-dataset)
2. [Lerobot-ros: a lightweight interface for controlling ROS-based robotic arms](https://discourse.openrobotics.org/t/lerobot-ros-a-lightweight-interface-for-controlling-ros-based-robotic-arms-using-lerobot/49420)
3. [Getting Started with Real-World Robots - Hugging Face](https://huggingface.co/docs/lerobot/en/getting_started_real_world_robot)
4. [Imitation Learning on Real-World Robots - Hugging Face](https://huggingface.co/docs/lerobot/en/il_robots)
5. [LeRobot: Making AI for Robotics more accessible - GitHub](https://github.com/huggingface/lerobot)
6. [LeRobot - Hugging Face](https://huggingface.co/lerobot)
7. [LeRobot – Lowering the entry barrier to AI for robotics - YouTube](https://www.youtube.com/watch?pp=0gcJCfwAo7VqN5tD&v=L0uxfZMlkag)
8. [Vision Language Action Models (VLA) & Policies for Robots](https://learnopencv.com/vision-language-action-models-lerobot-policy/)
9. [TheRobotStudio/SO-ARM100 - GitHub](https://github.com/TheRobotStudio/SO-ARM100)
10. [Technical features and advanced details of the SO-ARM101 robot](https://en.hwlibre.com/Technical-features-and-advanced-details-of-the-SO-ARM101-robot%3A-open-source-robotics-within-everyone%27s-reach./)
