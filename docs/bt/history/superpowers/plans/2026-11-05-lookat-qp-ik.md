# Lookat QP IK Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Add a QP-based lookat solver behind the existing solver interface, selectable by config, with the current DLS posture family as the fallback.

**Architecture:** `lookat_solver.hpp` defines one interface both solvers implement; `lookat_solver_router` calls the configured solver (QP by default after A/B, DLS today) and falls back to DLS whenever the QP path throws, is infeasible, or fails the post-solve guards. The QP itself is a 6-variable problem (Δq) solved with vendored qpmad, reusing the KDL chain already inside `PostureFamily` for FK and the 6×6 Jacobian.

**Tech Stack:** C++17, KDL (already linked), Eigen (already linked), qpmad (header-only, vendored), ROS 2 Humble, gtest run directly from `build/lgw_bt_planner/`.

**Spec:** `docs/superpowers/specs/2026-11-05-lookat-qp-ik-design.md`

## Global Constraints

- Control tick 100 Hz; per-tick solve budget `posture_max_overrun_us = 4000`.
- Joint margin `joint_margin = 0.15`; Servo halts at limit − 0.10, so every output joint must satisfy `lo + 0.15 <= q <= hi - 0.15`.
- Aim tolerance `aim_tol_rad = 0.035`; condition number ceiling `cond_max = 50`.
- Camera envelope `cam_min_height_m = 0.20`, `cam_max_radius_m = 0.40`.
- Default config stays `lookat_solver: dls` until the A/B acceptance run passes; `dls` mode must be behaviourally identical to today.
- DLS is never deleted; it is the fallback and the A/B control.
- Tests run as `./build/lgw_bt_planner/test_<name>` (no `colcon test` in this container).

## Review Focus

- QP output violating `cond`/camera envelope because those are soft/post-checked, not hard constraints — every output must pass the same FK checks DLS uses.
- First solve after takeover has no previous pose: the QP must still produce a legal absolute target.
- Degenerate target on the base z axis must throw exactly like DLS (`DegenerateTargetThrows`), never emit a NaN pose.
- Targets that jump to the side/rear (the reason for this work) must be reachable within the per-tick step limit, without a discontinuity spike.
- Unknown/typo'd `lookat_solver` value must fall back to DLS with a warning, never crash at startup.

---

### Task 1: Solver interface + router (no behaviour change)

**Files:**
- Create: `src/lgw_bt_planner/include/lgw_bt_planner/lookat_solver.hpp`
- Create: `src/lgw_bt_planner/include/lgw_bt_planner/lookat_solver_router.hpp`
- Create: `src/lgw_bt_planner/src/lookat_solver_router.cpp`
- Modify: `src/lgw_bt_planner/include/lgw_bt_planner/lookat_posture_family.hpp` (add `solve()` override + `jacobian()`)
- Modify: `src/lgw_bt_planner/src/lookat_posture_family.cpp`
- Modify: `src/lgw_bt_planner/CMakeLists.txt` (add `lookat_solver_router.cpp` to `lgw_posture`)
- Test: `src/lgw_bt_planner/tests/test_lookat_solver_router.cpp` (new), CMake `ament_add_gtest`

**Interfaces:**
- Produces:
  ```cpp
  namespace lgw_bt_planner {
  class LookatSolver {
  public:
    virtual ~LookatSolver() = default;
    virtual PostureResult solve(
      const tf2::Vector3 & p_target,
      const tf2::Matrix3x3 & R_cam_in_tip,
      const tf2::Vector3 & t_cam_in_tip,
      const PostureParams & params) = 0;
  };
  enum class SolverKind { kDls, kQp };
  SolverKind parse_solver_kind(const std::string & value, SolverKind fallback);
  class LookatSolverRouter {
  public:
    LookatSolverRouter(LookatSolver & dls, LookatSolver * qp,
                       SolverKind kind, bool fallback_dls);
    PostureResult solve(const tf2::Vector3 & p_target,
                        const tf2::Matrix3x3 & R_cam_in_tip,
                        const tf2::Vector3 & t_cam_in_tip,
                        const PostureParams & params);
    std::string last_path() const;   // "dls" | "qp" | "qp->dls"
  };
  }
  ```
- `PostureFamily` gains `solve(...)` (forwards to `compute`) and
  `bool jacobian(const std::array<double,6> & q, std::array<std::array<double,6>,6> & J, std::string & error) const;`

- [ ] **Step 1: Write the failing test**

```cpp
TEST(SolverRouter, DlsKindUsesDls)
{
  // fake solvers recording which one ran
  ...
  EXPECT_EQ(router.solve(t, R, tc, PostureParams{}).status, "ok");
  EXPECT_EQ(router.last_path(), "dls");
}

TEST(SolverRouter, UnknownValueFallsBackToDls)
{
  EXPECT_EQ(parse_solver_kind("qpp", SolverKind::kDls), SolverKind::kDls);
  EXPECT_EQ(parse_solver_kind("qp", SolverKind::kDls), SolverKind::kQp);
}
```

- [ ] **Step 2: Run it to verify it fails**

Run: `./build/lgw_bt_planner/test_lookat_solver_router`
Expected: build error / `parse_solver_kind` not declared.

- [ ] **Step 3: Implement the interface, the router and `PostureFamily::solve`/`jacobian`**

`jacobian` returns the KDL 6×6 Jacobian transposed into row-major
`J[joint][dof]` (translation rows 0-2, rotation rows 3-5) — the QP task
Jacobian is built from finite differences on top of `cameraPose`, so
`jacobian()` is only needed by later telemetry; keep it if already trivial,
otherwise drop it and let Task 3 use finite differences alone.

- [ ] **Step 4: Run tests to verify they pass**

Run: `./build/lgw_bt_planner/test_lookat_solver_router && ./build/lgw_bt_planner/test_lookat_posture_family`
Expected: PASS both, 15 posture tests unchanged.

- [ ] **Step 5: Commit**

```bash
git add src/lgw_bt_planner/include/lgw_bt_planner/lookat_solver.hpp \
        src/lgw_bt_planner/include/lgw_bt_planner/lookat_solver_router.hpp \
        src/lgw_bt_planner/src/lookat_solver_router.cpp \
        src/lgw_bt_planner/tests/test_lookat_solver_router.cpp \
        src/lgw_bt_planner/CMakeLists.txt
git commit -m "refactor(lgw): add a lookat solver interface and router"
```

---

### Task 2: Vendor qpmad and prove it builds

**Files:**
- Create: `src/lgw_bt_planner/thirdparty/qpmad/` (headers from the upstream release, LICENSE included)
- Test: `src/lgw_bt_planner/tests/test_qpmad_smoke.cpp`

**Interfaces:**
- Produces: `#include "qpmad/solver.h"` available to `lgw_posture`.

- [ ] **Step 1: Write the failing test**

```cpp
TEST(QpmadSmoke, MinimisesWithBoxAndLinearConstraint)
{
  // min 0.5*x'x - [1,1]x, s.t. 0 <= x <= 2, x0 + x1 >= 1
  Eigen::MatrixXd H = Eigen::MatrixXd::Identity(2, 2);
  Eigen::VectorXd g(2); g << -1.0, -1.0;
  ... qpmad solve ...
  EXPECT_NEAR(x(0), 0.5, 1e-6);
  EXPECT_NEAR(x(1), 0.5, 1e-6);
}
```

- [ ] **Step 2: Run to verify it fails**

Run: `./build/lgw_bt_planner/test_qpmad_smoke`
Expected: `qpmad/solver.h: No such file`.

- [ ] **Step 3: Vendor the headers, add the include dir, record the LICENSE**

Copy the upstream `include/qpmad/` tree; add
`target_include_directories(lgw_posture PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/thirdparty/qpmad/include)`.
Record the upstream commit hash and license in `thirdparty/qpmad/README.md`.

- [ ] **Step 4: Run to verify it passes**

Run: `./build/lgw_bt_planner/test_qpmad_smoke`
Expected: PASS.

- [ ] **Step 5: Commit**

```bash
git add src/lgw_bt_planner/thirdparty/qpmad src/lgw_bt_planner/tests/test_qpmad_smoke.cpp src/lgw_bt_planner/CMakeLists.txt
git commit -m "build(lgw): vendor qpmad"
```

---

### Task 3: QP core — task Jacobian, box constraints, solve

**Files:**
- Create: `src/lgw_bt_planner/include/lgw_bt_planner/lookat_qp_solver.hpp`
- Create: `src/lgw_bt_planner/src/lookat_qp_solver.cpp`
- Test: `src/lgw_bt_planner/tests/test_lookat_qp_solver.cpp`

**Interfaces:**
- Consumes: `PostureFamily::cameraPose()` for FK, `PostureParams`, `qpmad`.
- Produces:
  ```cpp
  struct QpSolverParams {
    double w_aim{1.0};
    double roll_weight{0.25};
    double w_smooth{0.01};
    double w_posture{0.001};
    double step_max_rad{0.2};
    int    max_linearizations{2};
  };
  class QpPostureSolver final : public LookatSolver {
  public:
    bool initialize(const std::string & urdf, bool is_file,
                    const std::string & base_link, const std::string & tip_link,
                    std::string & error);
    PostureResult solve(const tf2::Vector3 & p_target,
                        const tf2::Matrix3x3 & R_cam_in_tip,
                        const tf2::Vector3 & t_cam_in_tip,
                        const PostureParams & params) override;
    void set_params(const QpSolverParams & p);
  };
  ```
- `solve()` builds `e` exactly like `PostureFamily::solveFree` (axis cross product
  + `0.25 * roll`), the 4×6 task Jacobian by finite differences (h = 1e-4),
  solves the QP for `Δq`, then produces `tcp_desired` the same way the family does.

- [ ] **Step 1: Write the failing test**

```cpp
TEST(QpSolver, AimsAtGroundTarget)
{
  QpPostureSolver qp; initialize(...);
  const auto r = qp.solve(tf2::Vector3(0.75, 0.0, -0.37), R, t, PostureParams{});
  EXPECT_TRUE(r.ok);
  EXPECT_LT(r.aim_error_rad, PostureParams{}.aim_tol_rad);
  EXPECT_LE(r.cond, PostureParams{}.cond_max);
}

TEST(QpSolver, RespectsJointMarginsAndStep)
{
  // every output joint inside [lo+0.15, hi-0.15]
  // from a fixed previous pose, |q_out - q_prev| <= step_max + 1e-9
}
```

- [ ] **Step 2: Run to verify it fails**

Run: `./build/lgw_bt_planner/test_lookat_qp_solver`
Expected: build error, `QpPostureSolver` not defined.

- [ ] **Step 3: Implement the QP**

Hessian `H = 2*(w_aim·JᵀJ + w_smooth·I + w_posture·I)`; gradient
`g = -2*(w_aim·Jᵀe + w_posture·(q_prev - q_ref))`; box
`[max(lo+m, q_prev-step), min(hi-m, q_prev+step)]`; solve with
`qpmad::Solver`. Seed `q_prev` from the previous solution, or from
`posture_j2_nominal_rad` + pinned j3/j5 on the first call. Convert the
result to `PostureResult` with the exact `tcp_desired` formula used by the
family.

- [ ] **Step 4: Run to verify it passes**

Run: `./build/lgw_bt_planner/test_lookat_qp_solver`
Expected: PASS.

- [ ] **Step 5: Commit**

```bash
git add src/lgw_bt_planner/include/lgw_bt_planner/lookat_qp_solver.hpp src/lgw_bt_planner/src/lookat_qp_solver.cpp src/lgw_bt_planner/tests/test_lookat_qp_solver.cpp src/lgw_bt_planner/CMakeLists.txt
git commit -m "feat(lgw): QP lookat solver core"
```

---

### Task 4: Camera envelope as linearised constraints + FK post-check

**Files:**
- Modify: `src/lgw_bt_planner/src/lookat_qp_solver.cpp`
- Test: `src/lgw_bt_planner/tests/test_lookat_qp_solver.cpp`

**Interfaces:**
- Produces: every `solve()` result passes the same checks DLS applies,
  including one re-linearisation when the first solve violates them.

- [ ] **Step 1: Write the failing test**

```cpp
TEST(QpSolver, NeverExitsTheCameraEnvelope)
{
  // 40 random reachable targets: FK the result, assert
  // cam.z >= 0.20 - 1e-9 and radius <= 0.40 + 1e-9 or the result is !ok.
}

TEST(QpSolver, DegenerateTargetThrows)
{
  EXPECT_THROW(qp.solve(tf2::Vector3(0.0, 0.0, -0.37), R, t, PostureParams{}),
               std::runtime_error);
}
```

- [ ] **Step 2: Run to verify they fail**

Run: `./build/lgw_bt_planner/test_lookat_qp_solver --gtest_filter='*Envelope*:*Degenerate*'`
Expected: envelope violation or no throw.

- [ ] **Step 3: Implement the constraints and the post-check**

Add `p_z + J_zΔq >= 0.20` and `n·(p_xy + J_xyΔq) <= 0.40` with
`n = p_xy / ||p_xy||`; after solving, FK the result; on violation re-solve
once from the new point (`max_linearizations`); still failing → return
`ok=false, status="out_of_envelope"`. On `||p_target.xy|| < 1e-3` throw the
same `std::runtime_error` the family throws.

- [ ] **Step 4: Run to verify they pass**

Run: `./build/lgw_bt_planner/test_lookat_qp_solver`
Expected: PASS.

- [ ] **Step 5: Commit**

```bash
git add src/lgw_bt_planner/src/lookat_qp_solver.cpp src/lgw_bt_planner/tests/test_lookat_qp_solver.cpp
git commit -m "feat(lgw): constrain the QP lookat solve to the camera envelope"
```

---

### Task 5: Fallback path and node wiring

**Files:**
- Modify: `src/lgw_bt_planner/src/servo_node.cpp` (construct both solvers + router; params `lookat_solver`, `lookat_qp_fallback_dls`)
- Modify: `src/lgw_bt_planner/config/servo_node.yaml`
- Test: `src/lgw_bt_planner/tests/test_lookat_solver_router.cpp`

**Interfaces:**
- Consumes: `LookatSolverRouter`, `QpPostureSolver` from Tasks 1/3.
- Produces: `solver_status_` string logged with the posture telemetry (`dls|qp|qp->dls`).

- [ ] **Step 1: Write the failing test**

```cpp
TEST(SolverRouter, QpFailureFallsBackToDls)
{
  // QP fake throws / returns !ok; router returns the DLS result
  EXPECT_EQ(router.last_path(), "qp->dls");
  EXPECT_TRUE(result.ok);
}

TEST(SolverRouter, QpFailureWithoutFallbackPropagates)
{
  // fallback_dls = false → the QP result (ok=false) is returned unchanged
}
```

- [ ] **Step 2: Run to verify it fails**

Run: `./build/lgw_bt_planner/test_lookat_solver_router`
Expected: FAIL, fallback not implemented.

- [ ] **Step 3: Implement the fallback and wire the node**

Router: catch exceptions from the QP, return DLS when
`fallback_dls && qp != nullptr && (threw || !r.ok)`; set `last_path()`
accordingly. Node: parse `lookat_solver` (`dls` default, unknown → warn +
`dls`), build `QpPostureSolver` only when `qp` is selected, and log
`[posture] solve ... path=<last_path()>`. Expose QP weights
(`qp_w_aim`, `qp_w_smooth`, `qp_w_posture`, `qp_step_max_rad`).

- [ ] **Step 4: Run to verify it passes**

Run: `./build/lgw_bt_planner/test_lookat_solver_router` and the full suite
(`for t in build/lgw_bt_planner/test_*; do $t; done`)
Expected: all PASS.

- [ ] **Step 5: Commit**

```bash
git add src/lgw_bt_planner/src/servo_node.cpp src/lgw_bt_planner/config/servo_node.yaml src/lgw_bt_planner/tests/test_lookat_solver_router.cpp
git commit -m "feat(lgw): wire the QP lookat solver behind a config switch with DLS fallback"
```

---

### Task 6: Coverage/continuity A/B and acceptance

**Files:**
- Create: `scripts/lookat_solver_ab.py` (C++-backed sweep driver, or a gtest sweep behind an env var)
- Modify: `docs/superpowers/specs/2026-11-05-lookat-qp-ik-design.md` (§7 results)

- [ ] **Step 1: Write the sweep test/tool**

Azimuth −180…+180° step 5°, distances 0.35/0.5/0.75/1.0/1.3/1.6 m,
heights +0.15/0/−0.2/−0.37; for each solver report: ok %, cond p95,
max tcp step between adjacent azimuths, solve time mean/max.

- [ ] **Step 2: Run for dls**

Run: `python3 scripts/lookat_solver_ab.py --solver dls`
Expected: ±90° coverage 100 %; the ~+119° warm-start failure reproduced.

- [ ] **Step 3: Run for qp**

Run: `python3 scripts/lookat_solver_ab.py --solver qp`
Expected: ±90° 100 %; ±175° ≥ 95 %; no envelope/cond violations; step < 5 mm
within reachable bands; mean < 0.3 ms.

- [ ] **Step 4: Record the numbers in the spec §7 and decide the default**

Only switch `lookat_solver` default to `qp` if every §7 criterion passes;
otherwise keep `dls` and file the gaps.

- [ ] **Step 5: Commit**

```bash
git add scripts/lookat_solver_ab.py docs/superpowers/specs/2026-11-05-lookat-qp-ik-design.md
git commit -m "test(lgw): lookat solver coverage and continuity A/B"
```
