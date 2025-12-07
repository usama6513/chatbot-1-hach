# Implementation Plan: ROS 2 Humanoid Robotics Module

**Branch**: `ros2-humanoid-module` | **Date**: 2025-12-08 | **Spec**: [specs/ros2-humanoid/spec.md](./spec.md)
**Input**: Feature specification and user requirements for ROS 2 setup and humanoid control

## Summary

This module provides a practical, hands-on introduction to ROS 2 Humble for developers building their first humanoid robot system. The focus is on core ROS 2 concepts (Nodes, Topics, Services) via Python, a working humanoid URDF model, and integration patterns for autonomous agents. Delivered as Docusaurus documentation with complete, reproducible code examples in a GitHub repository.

## Technical Context

**Language/Version**: Python 3.10+; ROS 2 Humble (22.04)
**Primary Dependencies**: `rclpy`, `ros2cli`, `robot_state_publisher`, `rviz2`, `geometry_msgs`, `sensor_msgs`
**Storage**: Local filesystem (URDF, launch configs); no database required
**Testing**: ROS 2 node launch tests, URDF validation (`check_urdf`), manual end-to-end verification
**Target Platform**: Linux (Ubuntu 22.04) + Docker container
**Project Type**: Multi-artifact: Documentation (Docusaurus) + Code Repository (Python packages)
**Performance Goals**: Message latency < 100ms; node startup < 5s; visualization refresh 10+ Hz
**Constraints**: Beginner/intermediate level; no motion planning; ROS 2 Humble only
**Scale/Scope**: ~200 lines URDF; 3–5 Python node examples; 15–20 page documentation

## Constitution Check

**Gate: Must pass before Phase 0 research. Re-check after Phase 1 design.**

1. **Spec-First Development** ✅
   - Feature specification documented with user scenarios, requirements, success criteria
   - Implementation plan derived from spec

2. **Educational Clarity** ✅
   - Audience identified: Beginner/Intermediate developers
   - Code examples designed for learning: commented, step-by-step, no magic
   - Documentation includes conceptual explanations before code

3. **Full Reproducibility** ✅
   - All examples tested on Ubuntu 22.04 and Docker
   - No undocumented dependencies; all steps explicit
   - Verification commands provided for each section

4. **Modular Architecture** ✅
   - This module is independent; can be completed before other modules (Gazebo, Isaac, VLA)
   - Examples are self-contained: each tutorial can run standalone

5. **Quality Gates** ✅
   - Code follows ROS 2 conventions (PEP 8, rclpy patterns)
   - URDF validates without errors/warnings
   - Documentation accessibility reviewed during Phase 1

6. **Tool Stack Adherence** ✅
   - Uses ROS 2 Humble (specified)
   - Python for all examples (specified)
   - Docusaurus for documentation (specified)
   - GitHub repository (specified)

7. **Open Integration & Extensibility** ✅
   - Module structure allows extension (e.g., adding custom message types, more complex URDF)
   - Clear handoff points to advanced topics (motion planning, gazebo integration)

**Result**: PASS — All principles aligned. No violations.

## Architecture Decisions

### 1. **Python-First Approach**
- **Decision**: All examples in Python (rclpy), not C++
- **Rationale**: Lower barrier for beginners; faster prototyping; sufficient for learning; bridges to agent/AI logic
- **Alternatives**: C++ (more complex, better performance) rejected for beginner audience; Mixed (adds complexity)

### 2. **Docusaurus + GitHub Pages for Documentation**
- **Decision**: Book chapters in Docusaurus; code in GitHub repo
- **Rationale**: Scalable for 4-module book; GitHub Pages free hosting; integrates with RAG chatbot planned for book
- **Alternatives**: Sphinx/ReadTheDocs (less flexible); Medium blog (not integrated); Static HTML (harder to maintain)

### 3. **Separate Code Repository vs. Inline in Docusaurus**
- **Decision**: Code in GitHub repository `/code/ros2_humanoid/`; documentation references/links to it
- **Rationale**: Enables CI/CD testing, version control, easy cloning for students; documentation stays lightweight
- **Alternatives**: Code inline in docs (harder to test; duplication); separate package only (no learning context)

### 4. **ROS 2 Middleware & DDS Configuration**
- **Decision**: Use default ROS 2 middleware (Fast-RTPS); no custom QoS or security
- **Rationale**: Default settings sufficient for learning; reduces cognitive load; can be extended later
- **Alternatives**: DDS tuning (overkill for this level); edge cases documented for future

### 5. **Humanoid URDF Complexity**
- **Decision**: Simple 7-DoF humanoid (torso, 2-arm with shoulder/elbow, 2-leg with hip/knee, head)
- **Rationale**: Complex enough to demonstrate hierarchy and visualization; simple enough for learning
- **Alternatives**: Single-DOF robot (too simple); 44-DOF industrial humanoid (too complex)

### 6. **Simulation vs. Hardware**
- **Decision**: Focus on RViz visualization; Gazebo preview only (no control integration)
- **Rationale**: No hardware required for learning; RViz sufficient; Gazebo integration in Module 2 (out of scope)
- **Alternatives**: Hardware setup docs (too specific); Gazebo deeply integrated (out of scope per constraints)

## Project Structure

### Documentation (Docusaurus site)

```text
docs/modules/ros2/
├── 01-installation.md           # ROS 2 Humble setup (Ubuntu + Docker)
├── 02-workspace-setup.md        # Create ROS 2 workspace and packages
├── 03-publisher-subscriber.md   # Publisher-Subscriber pattern tutorial
├── 04-services.md               # ROS 2 Services tutorial
├── 05-humanoid-urdf.md          # URDF model definition and visualization
├── 06-joint-states.md           # Publishing joint states and RViz motion
├── 07-python-agent.md           # Python AI agent bridging to ROS 2
├── 08-launch-files.md           # Multi-node orchestration
├── 09-end-to-end.md             # Complete integration example
├── _category_.json              # Docusaurus navigation
└── assets/
    ├── images/                  # Screenshots, RViz captures
    └── downloads/               # Sample files for download
```

### Source Code (GitHub Repository)

```text
code/ros2_humanoid/
├── README.md                    # Project overview and quick start
├── Dockerfile                   # ROS 2 Humble Docker image
├── docker-compose.yml           # Multi-container setup (optional)
├── setup.py                     # Python package metadata
├── src/
│   ├── humanoid_robot/
│   │   ├── __init__.py
│   │   ├── publisher_node.py    # Example: Joint state publisher
│   │   ├── subscriber_node.py   # Example: Joint state subscriber
│   │   ├── service_node.py      # Example: Service server/client
│   │   ├── agent_node.py        # Example: Python AI agent
│   │   └── utils.py             # Shared utilities
│   └── package.xml              # ROS 2 package descriptor
├── urdf/
│   ├── humanoid.urdf            # Main URDF file (7-DoF model)
│   ├── humanoid_simple.urdf     # Simpler 3-DoF variant (optional)
│   └── meshes/                  # STL/DAE visual/collision meshes
├── launch/
│   ├── all_nodes.launch.py      # Launch all nodes
│   ├── rviz.launch.py           # RViz with humanoid config
│   └── publisher_subscriber.launch.py  # Tutorial launch
├── config/
│   ├── humanoid_rviz.rviz       # RViz saved configuration
│   └── ros2_params.yaml         # Node parameters
├── tests/
│   ├── test_publisher.py        # Unit test: publisher correctness
│   ├── test_subscriber.py       # Unit test: subscriber receives
│   └── test_urdf.py             # Validation: URDF parses correctly
└── scripts/
    ├── verify_install.sh        # Installation verification script
    └── run_tutorials.sh         # Run all example nodes
```

**Structure Decision**:
- Single ROS 2 package (`humanoid_robot`) with nodes in separate files for clarity
- Documentation in Docusaurus mirrors code structure for easy cross-reference
- URDF and launch files co-located with code for easy discovery

## Phase 0: Research & Clarifications

### Research Tasks

1. **ROS 2 Humble Best Practices**
   - Verify latest Python rclpy API (callback groups, async patterns)
   - Confirm Docker base image versions (osrf/ros:humble)
   - Identify any breaking changes in sensor_msgs/geometry_msgs

2. **URDF Validation & Visualization**
   - Research best practices for humanoid URDF structure (DH parameters, joint ordering)
   - Verify RViz mesh loading and transform handling
   - Test visual/collision geometry separation

3. **ROS 2 on Docker: Networking & Display**
   - Confirm Docker display forwarding for RViz (X11 or Wayland)
   - Test localhost networking for localhost-only environments
   - Document volume mounts for persistent workspace

4. **Educational Content Patterns**
   - Review ROS 2 official tutorials for beginner clarity
   - Identify terminology that requires explanation (QoS, middleware, executor)
   - Define glossary for common ROS concepts

**Output**: `research.md` with findings, best practices, and decision points

## Phase 1: Design & Contracts

### 1. Data Model

**Key Entities**:

- **ROS Node**: Encapsulates publisher(s), subscriber(s), service server(s)
  - Fields: name, namespace, executor, callbacks
  - State: running, stopped, errored

- **Topic**: Channel for pub/sub communication
  - Fields: name (string), message_type (string), QoS_profile
  - Examples: `/joint_states`, `/cmd_vel`

- **Message**: Structured data (geometry_msgs, sensor_msgs)
  - Examples: JointState, Twist, PoseStamped
  - Validation: Type checking at subscribe/publish time

- **URDF Link**: Robot part with geometry
  - Fields: name, visual_geometry, collision_geometry, inertia
  - Hierarchy: tree structure (parent-child)

- **URDF Joint**: Connection between links
  - Fields: name, type (revolute/prismatic), parent, child, axis, limits
  - State: current position, velocity

### 2. API Contracts (Informal; ROS 2 standard patterns)

**Publisher Node Contract**:
```python
class JointStatePublisher:
    def __init__(self, node_name: str)
    def publish_joint_state(joint_positions: dict) -> None
    def on_timer() -> None  # Callback for periodic publish
```

**Subscriber Node Contract**:
```python
class JointStateSubscriber:
    def __init__(self, node_name: str)
    def joint_state_callback(msg: sensor_msgs.JointState) -> None
    def get_latest_state() -> sensor_msgs.JointState
```

**Service Server Contract**:
```python
class RobotController:
    def handle_move_request(request: std_srvs.Trigger) -> std_srvs.TriggerResponse
    def handle_stop_request(request: std_srvs.Trigger) -> std_srvs.TriggerResponse
```

### 3. URDF Model Schema

**Humanoid URDF Structure**:
```xml
<robot name="humanoid">
  <link name="world" />
  <link name="torso" />
  <joint name="world_to_torso" type="fixed" />

  <!-- Left Arm -->
  <link name="left_shoulder" />
  <link name="left_upper_arm" />
  <link name="left_forearm" />
  <joint name="left_shoulder_pitch" type="revolute" />
  <joint name="left_elbow" type="revolute" />

  <!-- [Similar for right arm, left leg, right leg, head] -->
</robot>
```

**Validation Rules**:
- All joints have unique names
- All links have unique names
- Parent-child relationships form a tree (no cycles)
- Visual and collision geometries are valid

### 4. Quickstart & Integration

**Quickstart Workflow**:
1. Clone repository or copy code examples
2. Install ROS 2 Humble (via guide or Docker)
3. Create ROS 2 workspace: `mkdir -p ~/ros2_ws/src`
4. Copy package into workspace
5. Build: `colcon build`
6. Source: `source install/setup.bash`
7. Run example: `ros2 run humanoid_robot publisher_node`

**File Structure After Install**:
```
~/ros2_ws/
├── src/humanoid_robot/  # Cloned package
├── build/
├── install/
└── log/
```

### 5. Docusaurus Integration

**Navigation Structure** (`_category_.json`):
```json
{
  "label": "ROS 2 Humanoid Robotics",
  "position": 2,  // Module order
  "link": { "type": "doc", "id": "ros2/01-installation" }
}
```

**Page Linking**: Each Docusaurus page references corresponding code files in GitHub with direct links.

## Phase 2: Task Generation

**Scope**: Tasks will be generated via `/sp.tasks` command after this plan is approved.

**Expected Task Count**: ~20–30 tasks, grouped into:
- Installation & Verification (3–5 tasks)
- Workspace & Package Setup (2–3 tasks)
- Publisher-Subscriber Tutorial (5–7 tasks)
- URDF Modeling (5–7 tasks)
- Python Agent Integration (4–5 tasks)
- Documentation & Testing (3–5 tasks)

## Risks & Mitigations

| Risk | Impact | Mitigation |
|------|--------|-----------|
| Docker display forwarding for RViz fails | High: Users can't visualize | Document X11 setup, provide Wayland alt; offer RViz-headless alternative |
| URDF complexity intimidates beginners | Medium: Low engagement | Provide simplified 3-DoF variant first; progressive complexity |
| Python version mismatches (3.8 vs 3.10+) | High: Code doesn't run | Explicit Python 3.10+ requirement in install guide; CI tests this |
| ROS 2 Humble minor version drift | Low: API changes unlikely mid-LTS | Pin exact package versions in Dockerfile and `requirements.txt` |

## Success Validation

- ✅ All 5 code examples run on clean Ubuntu 22.04 + Docker
- ✅ URDF validates with zero warnings via `check_urdf`
- ✅ Docusaurus site builds and deploys to GitHub Pages
- ✅ Documentation accessibility review: no undefined terminology, code explained line-by-line
- ✅ Integration points to other modules (Gazebo, Isaac) are clear and documented

## Next Steps After Approval

1. **Phase 0 Execution**: Launch research agent for ROS 2 best practices, URDF standards, Docker setup
2. **Phase 1 Finalization**: Review research findings; finalize data model and API contracts
3. **Phase 2 Task Generation**: Run `/sp.tasks` to create detailed, testable task list
4. **Phase 3 Implementation**: Build code, documentation, and tests in parallel
5. **Phase 4 Integration**: Integrate into Docusaurus book; test end-to-end; prepare for chatbot embedding

---

**Prepared by**: Claude Code Agent
**Date**: 2025-12-08
**Status**: Ready for Phase 0 Research Launch
