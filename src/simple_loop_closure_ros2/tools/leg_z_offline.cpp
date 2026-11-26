// Offline port check for LegZIntegrator.
//
// Reads the exact per-sample inputs dumped by
// `scripts/large/leg_z_reference.py --dump-inputs`, runs the production
// integrator, and writes `t,z`.  This removes ROS callback timing from the
// comparison with the Python reference.
//
// Usage:
//   leg_z_offline <inputs.csv> <out.csv> [pitch_deg] [roll_deg] [spread]
//                  [settle] [att_timeout] [att_jump]
//                  [force_on] [force_off] [single_force] [max_step_dz]

#include <cstdlib>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

#include <Eigen/Core>

#include "simple_loop_closure/leg_z_integrator.hpp"

namespace {

std::vector<double> splitCsv(const std::string& line) {
  std::vector<double> out;
  std::stringstream ss(line);
  std::string tok;
  while (std::getline(ss, tok, ',')) {
    out.push_back(std::stod(tok));
  }
  return out;
}

}  // namespace

int main(int argc, char** argv) {
  if (argc < 3) {
    std::cerr << "usage: leg_z_offline <inputs.csv> <out.csv> "
                 "[pitch_deg] [roll_deg] [spread] [settle] [att_timeout] "
                 "[att_jump] [force_on] [force_off] [single_force] "
                 "[max_step_dz]\n";
    return 2;
  }

  simple_loop_closure::LegZConfig cfg;
  if (argc > 3) cfg.pitch_offset_deg = std::stod(argv[3]);
  if (argc > 4) cfg.roll_offset_deg = std::stod(argv[4]);
  if (argc > 5) cfg.consensus_spread = std::stod(argv[5]);
  if (argc > 6) cfg.settle_s = std::stod(argv[6]);
  if (argc > 7) cfg.attitude_timeout = std::stod(argv[7]);
  if (argc > 8) cfg.attitude_jump_deg = std::stod(argv[8]);
  if (argc > 9) cfg.force_on = std::stod(argv[9]);
  if (argc > 10) cfg.force_off = std::stod(argv[10]);
  if (argc > 11) cfg.single_force = std::stod(argv[11]);
  if (argc > 12) cfg.max_step_dz = std::stod(argv[12]);

  std::ifstream in(argv[1]);
  if (!in) {
    std::cerr << "cannot open " << argv[1] << "\n";
    return 1;
  }
  std::ofstream out(argv[2]);
  if (!out) {
    std::cerr << "cannot open " << argv[2] << "\n";
    return 1;
  }
  out << "t,z\n";

  simple_loop_closure::LegZIntegrator integrator(cfg);
  std::string line;
  std::getline(in, line);  // header
  size_t rows = 0;
  while (std::getline(in, line)) {
    if (line.empty()) continue;
    const std::vector<double> v = splitCsv(line);
    if (v.size() < 28) {
      std::cerr << "bad row (" << v.size() << " cols)\n";
      return 1;
    }
    simple_loop_closure::LegSample s;
    s.stamp = v[0];
    for (int i = 0; i < 4; ++i) {
      s.foot[i] = Eigen::Vector3d(v[1 + 3 * i], v[2 + 3 * i],
                                  v[3 + 3 * i]);
      s.force[i] = v[13 + i];
    }
    simple_loop_closure::LegAttitude att;
    att.stamp = v[17];
    att.valid = v[18] > 0.5;
    att.R << v[19], v[20], v[21], v[22], v[23], v[24], v[25], v[26], v[27];
    integrator.update(s, att);
    out << s.stamp << ',' << integrator.z() << '\n';
    ++rows;
  }
  std::cout << "rows=" << rows << " z=" << integrator.z()
            << " used=" << integrator.diagnostics().steps_used
            << " skipped=" << integrator.diagnostics().steps_skipped << '\n';
  return 0;
}
