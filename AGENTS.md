# AGENTS.md

## Project Scope

This project is the autonomous inventory robot system based on ROS2 Humble.

The only package that may be modified without explicit user confirmation is:

    /home/wheeltec/wheeltec_ros2/src/agv_inventory_system

Do not modify files outside this package unless the user explicitly confirms the exact target path.

Forbidden external packages include, but are not limited to:

- /home/wheeltec/wheeltec_ros2/src/wheeltec_robot_nav2
- /home/wheeltec/wheeltec_ros2/src/turn_on_wheeltec_robot
- /home/wheeltec/wheeltec_ros2/src/wheeltec_cartographer
- /home/wheeltec/wheeltec_ros2/install
- system files under /opt, /usr, /etc, or /lib

## Required Workflow Before Any Code Modification

Before modifying code, first locate the relevant files, symbols, parameters, and line numbers.

Use commands like:

    cd /home/wheeltec/wheeltec_ros2/src/agv_inventory_system

When the target file, function name, parameter name, or line number is already known, prefer precise
local reads and targeted grep first:

    grep -n "function_name" src/mission_manager_node.cpp
    grep -n "parameter_name" config/inventory_system.yaml
    sed -n '120,220p' src/mission_manager_node.cpp

Only use broad recursive searches when the target file or symbol is not yet known:

    grep -R "keyword" -n src include config
    grep -R "StateName" -n src include config

After the search, summarize:

1. Which files are relevant.
2. Which line numbers contain the current logic.
3. Which functions, states, parameters, or topics will be changed.
4. Which old logic should be removed.

Do not modify files before this analysis is complete.

## Codex Token Usage Policy

Use a lightweight workflow by default to reduce unnecessary Codex quota consumption.

- When the target file, function name, parameter name, or line number is already known, prefer local
  reads and precise grep instead of broad project-wide searches.
- Do not default to large recursive grep commands or full state-machine analysis when the relevant
  scope is already clear.
- When a plan has already been confirmed in the previous turn, do not repeat the full planning
  analysis before implementing it; re-check only the context needed for a safe edit.
- Keep reports concise by default, focusing on changed files, important line numbers, verification
  results, and remaining risks.
- Do not sacrifice correctness to save tokens while writing code. Necessary context reading,
  implementation, cleanup of obsolete logic, compile-error fixes, and build verification must still
  be done when needed.
- Safety rules remain higher priority than token savings, including allowed-path restrictions,
  preserving user changes, keeping lift and scanning interfaces, and avoiding sim/mock/fake/
  temporary/placeholder names as formal interface names.

## Code Modification Rules

When modifying code:

- Prefer small, focused changes.
- Do not only append new logic while leaving obsolete logic active.
- Remove unused old parameters, variables, branches, states, and comments when they become invalid.
- Keep the state machine readable and avoid duplicated control paths.
- Preserve existing topic names, service names, and parameter names unless the user explicitly asks to rename them.
- Do not introduce large new dependencies unless the user explicitly approves.

## Old Code Cleanup Rule

When implementing a new behavior that replaces an old behavior, the old behavior must be removed in the same modification.

Do not keep both the old and new logic active unless the user explicitly asks to preserve both paths.

Before editing, identify which old code will become obsolete. After editing, use grep to confirm that obsolete parameters, states, functions, service names, and comments have been removed.

Examples of obsolete code that should be removed:

- old parameters no longer used by the real workflow
- old test-only branches that conflict with the final logic
- duplicated state-machine branches
- placeholder, mock, simulated, or temporary logic that has been replaced
- comments describing behavior that is no longer true

## Naming Rules

Use final production-style names even if the current implementation is temporarily simplified internally.

Avoid names such as:

- sim
- mock
- fake
- temporary
- placeholder as a permanent interface name

For example, use names like:

- lift_controller
- scan_controller
- inventory_executor

The internal behavior may be simplified during testing, but the external interface should match the final real system design.

## Inventory Robot Workflow Context

The main autonomous inventory workflow is:

1. Web or service sends target inventory task.
2. Robot navigates along the corridor.
3. Front camera recognizes cabinet numbers.
4. Robot tracks the target cabinet.
5. Robot waits for or requests an accessible cabinet gap.
6. Robot detects the gap using LiDAR.
7. Robot enters the gap.
8. Robot performs inventory scanning.
9. Robot exits the gap.
10. Robot continues to the next target or finishes the mission.

The main state machine may include:

- IDLE
- CORRIDOR_NAV
- TARGET_TRACKING
- SEARCH_GAP
- WAITING_GAP
- ENTERING_GAP
- INVENTORYING
- RETURNING
- DONE
- ERROR

Do not break this overall workflow unless the user explicitly asks for a redesign.

## Lift and Scanning Rules

Do not delete lift-pole or scanning-related interfaces.

Even if the current implementation only uses an internal test process, keep the final real interface structure available.

The lift and scanning logic should be independent from the main mission state machine as much as possible.

The main mission manager should call clear interfaces such as:

- start inventory scan
- move lift to target level
- trigger scanner
- report scan result
- finish cabinet inventory

Do not hard-code all scanning behavior directly into the main navigation logic.

## Warehouse Layout Context

The warehouse has two movable cabinet rows and a central corridor.

Cabinets:

- Upper or left row: cabinets 1-18
- Lower or right row: cabinets 19-36

Special single cabinets:

- 1
- 18
- 19
- 36

Paired physical cabinet units:

- 2/3, 4/5, 6/7, 8/9, 10/11, 12/13, 14/15, 16/17
- 20/21, 22/23, 24/25, 26/27, 28/29, 30/31, 32/33, 34/35

The maximum accessible gap is measured between adjacent physical cabinet units, not inside a paired cabinet.

Do not treat the internal boundary of a paired cabinet as a real physical gap.

## Build and Test Commands

After modifying C++ or configuration files, run:

    cd /home/wheeltec/wheeltec_ros2

    colcon build --packages-select agv_inventory_system

If the build fails, report:

1. The exact error message.
2. The file and line number.
3. The likely cause.
4. The proposed fix.

Do not claim success unless the build actually passes.

## Git Safety

Before making changes, check the working tree:

    git status --short

After changes, summarize:

1. Modified files.
2. Added files.
3. Deleted old code or parameters.
4. Build result.
5. Any remaining risks or TODO items.

Do not overwrite user changes.

## Communication Style

When reporting results to the user, use Chinese.

Be concise but specific.

For code changes, always report concrete file paths and line numbers when possible.

Avoid vague statements such as "optimized the logic" without explaining what changed.
