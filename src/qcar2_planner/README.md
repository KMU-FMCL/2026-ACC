# qcar2_planner

PH curve-based path planning library for the QCar2 autonomous vehicle platform.

Provides mathematically exact path planning using Pythagorean Hodograph (PH) quintic curves. The library computes polynomial arc lengths, rational unit normals, and exact offset curves — properties that hold analytically rather than through numerical approximation.

## Features

- **Bernstein polynomial arithmetic** — construction, evaluation (de Casteljau), degree elevation, differentiation, and multiplication
- **Quintic PH curve interpolation** — G1 Hermite data to PH quintic with exact arc length
- **Offset curves** — constant or variable-distance offsets using RBF interpolation
- **Result-based error handling** — all fallible operations return `playground::core::Result<T>`; no exceptions, no silent failures

## Requirements

| Requirement | Version |
|-------------|---------|
| CMake | 3.25+ |
| C++ | 17 |
| ROS 2 | Humble (or compatible) |
| colcon | any |

Dependencies are fetched automatically at configure time via CMake FetchContent.

## Building

Source your ROS 2 environment and build with colcon from the workspace root:

```bash
cd /home/user/qcar2_ws
source /opt/ros/${ROS_DISTRO}/setup.bash
colcon build
```

Optional build flags:

| Flag | Default | Effect |
|------|---------|--------|
| `ENABLE_COVERAGE` | OFF | Instrument for gcovr coverage (target: 80%) |
| `ENFORCE_FORMAT` | OFF | Run clang-format as a pre-build step |
| `ENFORCE_LINT` | OFF | Run clang-tidy as a pre-build step |
| `CI_MODE` | OFF | Validate all files (not just changed files) |

Example with coverage:

```bash
colcon build --cmake-args -DENABLE_COVERAGE=ON
```

## Testing

Run all tests (includes unit, integration, and linter checks):

```bash
cd /home/user/qcar2_ws
source /opt/ros/${ROS_DISTRO}/setup.bash
AMENT_CPPCHECK_ALLOW_SLOW_VERSIONS=1 colcon test --ctest-args '--output-on-failure'
```

View detailed results:

```bash
colcon test-result --all --verbose
```

Test targets follow the naming convention:

```
qcar2_planner__test__{type}__{path}
```

Example: `qcar2_planner__test__unit__core__bernstein__polynomial`

## API Overview

All public types live under the `qcar2_planner::core` namespace, subdivided by module.

### Bernstein polynomial — `qcar2_planner::core::bernstein`

```cpp
#include <qcar2_planner/core/bernstein/polynomial.hpp>

using namespace qcar2_planner::core::bernstein;

// Construction and evaluation
auto p = Polynomial({1.0, 2.0, 3.0});          // degree-2
auto value = p.Evaluate(0.5);                   // de Casteljau evaluation

// Arithmetic (same degree)
auto q = Polynomial({0.5, 1.5, 2.5});
auto sum = p + q;

// Degree-changing operations (static members)
auto elevated   = Polynomial::Elevate(p);        // degree-2 -> degree-3
auto derivative = Polynomial::Differentiate(p);  // degree-2 -> degree-1
auto product    = Polynomial::Multiply(p, q);    // degree-2 x degree-2 -> degree-4
```

### PH quintic curve — `qcar2_planner::core::hodograph`

```cpp
#include <qcar2_planner/core/hodograph/quintic.hpp>
#include <qcar2_planner/core/hodograph/planar_point.hpp>

using namespace qcar2_planner::core::hodograph;

PlanarPoint p_s{0.0, 0.0};
PlanarPoint p_e{10.0, 5.0};
PlanarPoint t_s{1.0, 0.0};   // unit tangent at start
PlanarPoint t_e{0.0, 1.0};   // unit tangent at end

// Interpolate — returns Result<Quintic>
auto result = Quintic::Interpolate(p_s, p_e, t_s, t_e);
if (result.IsFailure()) {
  // handle error
  return;
}
auto curve = std::move(result.Value());

auto position = curve.Evaluate(0.5);   // point at t=0.5
auto tangent  = curve.Tangent(0.5);    // unit tangent at t=0.5
auto normal   = curve.Normal(0.5);     // unit normal (left of direction)
auto length   = curve.TotalLength();   // exact polynomial arc length
```

The factory produces four solution branches from sign choices in the PH preimage. The default overload selects the branch with the shortest arc length. An explicit branch (1–4) can be requested via `Quintic::Interpolate(p_s, p_e, t_s, t_e, branch)`.

### Offset curve — `qcar2_planner::core::offset`

```cpp
#include <qcar2_planner/core/offset/curve.hpp>

using namespace qcar2_planner::core::offset;

// Constant offset
auto curve_result = hodograph::Quintic::Interpolate(p_s, p_e, t_s, t_e);
auto curve = std::move(curve_result.Value());

auto offset_result = Curve::Create(std::move(curve), 2.0);
auto offset = std::move(offset_result.Value());
auto point  = offset.Evaluate(0.5);

// Variable offset via RBF interpolation
auto interp_result = rbf::Interpolator::Fit(centers, offsets, epsilon);
auto interp = std::move(interp_result.Value());

auto curve2_result = hodograph::Quintic::Interpolate(p_s, p_e, t_s, t_e);
auto curve2 = std::move(curve2_result.Value());

auto var_offset_result = Curve::Create(std::move(curve2), std::move(interp));
auto var_offset = std::move(var_offset_result.Value());
auto adaptive_point = var_offset.Evaluate(0.3);
```

### RBF interpolator — `qcar2_planner::core::rbf`

```cpp
#include <qcar2_planner/core/rbf/interpolator.hpp>

using namespace qcar2_planner::core::rbf;

std::vector<double> centers = {0.0, 0.25, 0.5, 0.75, 1.0};
std::vector<double> offsets = {0.0, 1.5, 3.0, 1.5, 0.0};
double epsilon = 4.0;

auto result = Interpolator::Fit(centers, offsets, epsilon);
auto interp = std::move(result.Value());
auto d = interp.Evaluate(0.3);  // offset distance at xi=0.3
```

## Project Structure

```
qcar2_planner/
├── CMakeLists.txt
├── package.xml
├── library/
│   ├── include/qcar2_planner/core/   # Public headers
│   └── source/core/                  # Implementations (Pimpl details here)
├── test/
│   ├── unit/core/                    # Unit tests (mirrors source layout)
│   └── integration/                  # Integration tests
├── validation/                       # Ament linter targets
├── dependency/
│   ├── dependencies.cmake            # Single source of truth for all deps
│   ├── externals/                    # FetchContent deps (read-only, SYSTEM)
│   └── internals/                    # Modifiable deps (git-tracked)
└── design/                           # Feature requirement documents
```

## Dependencies

| Library | Version | Purpose |
|---------|---------|---------|
| playground | v0.1.0 | Core infrastructure (Error, Result, Logger, unit types) |
| Eigen3 | 5.0.0 | Linear algebra (VectorXd, matrix operations) |
| Catch2 | v3.5.2 | Testing framework |
| FakeIt | 2.5.0 | C++ mocking framework |
| spdlog | v1.14.1 | Logging |
| fkYAML | v0.3.12 | YAML parsing |
| doxygen-awesome-css | v2.3.4 | Documentation theme |

All dependencies are declared in `dependency/dependencies.cmake` and fetched automatically by CMake.

## Development Workflow

1. Branch from `develop`: `git checkout -b feat/<name>`
2. Write a test, then write the implementation (TDD)
3. Build and test until all checks pass
4. Open a PR to `develop` (squash merge after approval)

Commits follow semantic format: `<type>(<scope>): <description>`

Test coverage must not drop below 80%.

## License

GPL-3.0-only. See [LICENSE](LICENSE).
