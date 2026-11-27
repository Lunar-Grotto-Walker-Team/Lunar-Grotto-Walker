// src/lookat_solver.cpp — solver selection helpers (see lookat_solver.hpp).

#include "lgw_bt_planner/lookat_solver.hpp"

namespace lgw_bt_planner
{

SolverKind parse_solver_kind(const std::string & value, SolverKind fallback)
{
  if (value == "dls") {
    return SolverKind::kDls;
  }
  if (value == "qp") {
    return SolverKind::kQp;
  }
  return fallback;
}

}  // namespace lgw_bt_planner
