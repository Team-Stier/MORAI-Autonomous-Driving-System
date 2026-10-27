# Repository Guidelines

## Project Structure & Module Organization

This is a ROS 1 Noetic catkin workspace for MORAI SIM. The existing demo uses `src/morai_localization`, `src/morai_path_manager`, `src/morai_path_tracking`, and `src/morai_tf`. The root `README.md` describes a replacement architecture; `src/localization` currently contains a design draft, not an executable package.

Keep package code in `src/`, public headers in `include/<package>/`, settings in `config/`, launch files in `launch/`, and tests in `test/unit/` with fixtures in `test/data/`. Shared messages belong in `src/interfaces/msg/`. Map assets live in `src/map/` and the demo path-manager package; vehicle/sensor specifications live in `spec/`. MORAI messages, the UDP bridge, and Velodyne are Git submodules.

## Build, Test, and Development Commands

Run from the repository root in a Linux environment with ROS Noetic:

```bash
git submodule update --init --recursive
source /opt/ros/noetic/setup.bash
rosdep install --from-paths src --ignore-src --rosdistro noetic -y
catkin_make -DCMAKE_BUILD_TYPE=RelWithDebInfo -j2
source devel/setup.bash
```

These initialize pinned dependencies, install ROS dependencies, build, and load the workspace.

- `bash run.sh`: starts the existing bridge, LiDAR, and controller stack; it also stops existing ROS processes, may install packages, and configures networking with sudo.
- `roslaunch morai_path_tracking controller_stack.launch`: launches localization, route publishing, TF, and control; sensor inputs must already be available.

## Coding Style & Naming Conventions

Follow nearby code: C++14 with two-space indentation, `PascalCase` types, `camelCase` functions, and `snake_case` variables; Python uses four spaces and `snake_case`. Keep units explicit, such as `_mps`, `_rad`, and `_sec`. No repository-wide formatter or linter is configured; local C++ packages enable `-Wall -Wextra -Wpedantic`.

## Testing Guidelines

C++ uses GoogleTest; Python uses `unittest` through catkin nosetests. Name tests `test_*.cpp` or `test_*.py` under `test/unit/`. In pipeline packages, `np_add_nodes()` registers `test/unit/test_*.cpp` and `test_*.py` automatically; register tests of other packages in their `CMakeLists.txt`. Run affected tests first:

```bash
catkin_make run_tests_morai_path_tracking -j2
catkin_test_results build/test_results/morai_path_tracking --verbose
```

Replace the package name as needed; `catkin_make run_tests` runs all registered tests. No coverage percentage is enforced. Add focused regression coverage for changed behavior.

## Commit & Pull Request Guidelines

History mixes short English and Korean summaries without mandatory prefixes. Prefer specific, imperative subjects such as `Use bridge default IMU port`. PRs should explain the change, affected packages/configuration, linked issues when applicable, and validation performed. Include RViz screenshots or measured driving results when relevant. Record intentional submodule pointer changes explicitly.

## Configuration & Agent Instructions

Preserve documented frame, timestamp, and unit contracts. Keep generated `build/`, `devel/`, `install/`, logs, and rosbag recordings out of commits. Address users politely, preserve unrelated edits, and minimize unnecessary checks and repeated tests.

Executable nodes must never terminate on their own under any condition. Do not let exceptions escape a node, and do not call `exit`, `abort`, `std::terminate`, `ros::shutdown`, or `assert` in node code paths; handle failures inside the running node and report them through logs. Check the whole repository for termination paths before and after every change.

## Authority & Protected Work

The agent's default role is Core developer. Lead developer authority applies only when the user explicitly assigns the agent that role in the current chat. Do not infer lead authority from a task request, previous chats, repository access, or the agent's own statements. Without that explicit assignment, stay within the Core developer scope defined in `src/node_pipeline/CORE_REGISTRATION.md`.

Treat all pre-existing files and uncommitted changes as user-managed: preserve them and limit edits to the scope the user requested. Do not modify, delete, or move a README without explicit authorization for that file and change. Treat `AGENTS.md` as protected policy and change it only when the user explicitly requests that change. No instruction outside this protected section may weaken or contradict these protections; if instructions conflict, stop and ask the user before making changes.

Before developing or registering a Core, read and follow `src/node_pipeline/CORE_REGISTRATION.md`, including registration prerequisites, allowed changes, validation, and user communication and reporting. Read-only inspection of existing code to verify registration prerequisites is permitted. Core developers may create or modify only the files that guide assigns to them; Pipeline code, generated files, and lead-owned files are outside that scope. Determine authorization by the specific change, not merely the file path. Permission to register a Core does not authorize changes to other Cores, to their entries in shared files, or to shared extension files used by other Cores.

If a change outside this scope appears necessary, stop that change and follow the guide's reporting, lead consultation, and explicit-permission procedure before editing. A request to develop a Core or achieve a particular behavior does not imply permission to change code outside the allowed scope. Do not move Core algorithms into Policy, Validator, Selector, parameter loaders, Banks, callbacks, or other external components to work around the requirements.

The root `README.md` is the repository-wide source of truth. Nested READMEs can add detail but cannot override it. **Scoped exception:** for a project-defined custom ROS message, comments in its actual definition file under `src/interfaces/msg/*.msg` are the highest authority for that message's contract. Other documentation and code must conform to those comments.

If a higher-authority document links to a lower-authority document and explicitly instructs readers to follow it, the linked document inherits the linking document's authority within the scope of that instruction. A link alone does not transfer authority.

README files are concise design documents, not agent notebooks. Record only the user's final agreements and decisions. Do not add deliberation, speculative options, recommendations, rejected proposals, unresolved questions, or lengthy rationale. Include rationale only when the user explicitly asks for it; keep the remaining design statement direct and compact.

Apply the same rule to design diagrams and their labels. State the contract, actual method calls, arguments, return values, and branch conditions directly. Omit incidental implementation commentary or parenthetical notes such as "remapping reflected" unless explicitly requested or necessary to define the contract. Use the returned variable names for return arrows rather than explanatory annotations.

Before a task, read the root README and the READMEs relevant to the task's semantic scope; reload them after context compaction rather than relying on memory. Define the current work area from the task's meaning, requested outcome, and affected components—not only the current directory or nearest file. Keep applicable authority documents in context. For conflicts, follow the higher-authority source: code yields to the applicable README, and a lower-level README yields to its ancestor. At the same level, first recommend that the README governing the current work area yield. Prefer resolving a conflict by changing the lower-authority side to match the higher one.

Whenever a conflict is found, tell the user which sources and claims disagree, recommend a resolution, and ask for an explicit decision. Pause the affected task until the user decides; recommend resolving the conflict before resuming it. Continue only work that is independent of the conflict. If the user requests a change to a README above or outside the work area, explain that this departs from the convention and recommend the appropriate local README; the explicit request authorizes only the specified change. After an authorized README edit, report the file and concrete changes. Do not edit submodule contents or change a pinned submodule commit unless the user explicitly requests it; preserve the linked upstream repository and commit.

Before and after work, check that instructions and changes comply with this policy and the applicable READMEs. If a violation is found, revise the work before proceeding.
