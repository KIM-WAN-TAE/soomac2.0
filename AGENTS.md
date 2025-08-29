# Repository Guidelines

## Project Structure & Module Organization
- `src/`: ROS 2 packages
  - `dongsoo_cpp_pkg/`: C++ nodes (`src/*.cpp`, `CMakeLists.txt`, `package.xml`).
  - `dongsoo_py_pkg/`: Python nodes (`dongsoo_py_pkg/*.py`), tests in `src/dongsoo_py_pkg/test/`.
  - `dongsoo_description/`: robot config and assets (`config/*.json`).
  - `dongsoo_interfaces/`: ROS 2 interface definitions (`srv/*.srv`).
  - `finger_test/`: example/test package.
- `build/`, `install/`, `log/`: colcon outputs (generated).
- `etc/`, `shared_context/`, `work_log/`: local configs, context, and notes.

## Build, Test, and Development Commands
- Setup ROS 2: `source /opt/ros/humble/setup.bash`
- Build all: `colcon build --symlink-install`
- Overlay env: `source install/setup.bash`
- Run example node: `ros2 run dongsoo_py_pkg trajectory_test`
- Test all: `colcon test` (adds `--event-handlers console_cohesion+` for clearer logs)
- Python tests only: `pytest -q src/dongsoo_py_pkg`
- Lint-only (Python): `pytest -m "flake8 or pep257" -q src/dongsoo_py_pkg`

## Coding Style & Naming Conventions
- Python: PEP 8, 4-space indent; functions/modules `snake_case`, classes `CapWords`.
- Docstrings: PEP 257; include brief summary and arguments/returns.
- C++: follow ROS 2/ament norms; types `CamelCase`, functions/vars `snake_case`; keep headers in `include/` (when added) and sources in `src/`.
- Lint/format: flake8 and pydocstyle run via tests; keep imports ordered and lines ≤ 120 chars.

## Testing Guidelines
- Frameworks: `pytest` for Python; C++ via ament/`ctest`.
- Location: `src/<package>/test/`, file pattern `test_*.py`.
- Coverage: include tests for new logic and edge cases; prefer small, deterministic unit tests. Add launch/integration tests where ROS graph is required.

## Commit & Pull Request Guidelines
- Commits: concise, imperative summaries (e.g., "Add IK service handler"); add context in body and reference issues.
- Scope: separate logical changes per commit; avoid bundling formatting with behavior.
- PRs: include description, affected packages, test steps/commands, and relevant `ros2` logs or screenshots. Ensure `colcon build && colcon test` pass locally.

## Security & Configuration Tips
- Avoid hardcoding device paths or credentials; use ROS 2 params/config files under `dongsoo_description/config/`.
- Validate external JSON before use and handle file I/O errors gracefully.
