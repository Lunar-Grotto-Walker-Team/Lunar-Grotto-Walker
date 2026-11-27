// vui_bridge_node.cpp
//
// rclcpp service that shells out to a one-shot SDK helper subprocess
// (`go2_vui_set_brightness_helper`) for every SetVuiBrightness call. The
// bridge process itself NEVER links the Unitree SDK, so it never loads
// libddscxx.so.0 — only the helper subprocess does, in its own address
// space. That keeps rclcpp's rmw_cyclonedds_cpp (which statically carries
// its own libddscxx 0.10.5) completely isolated from the SDK's vendored
// libddscxx 0.10.2, avoiding the in-process double-participant crash that
// blew up the original in-process SetLedBrightnessNode.
//
// Rationale + process-boundary diagram: srv/SetVuiBrightness.srv header.
//
// Performance note: every service call pays one popen+fork+exec+SDK
// ChannelFactory::Init round-trip (~80-150 ms cold, ~20-50 ms warm). The
// BT demo fires at most ~6 brightness changes in a 5 s tree tick, so this
// overhead is invisible next to the 1 s SetTimeout the helper inherits
// from the SDK anyway. If a tighter loop ever needs it, the helper can
// be replaced by a long-running daemon speaking the SDK RPC, but the
// subprocess approach keeps the failure mode (helper crash) cleanly
// contained inside vui_bridge's service response.

#include <array>
#include <cerrno>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>
#include <sys/wait.h>

#include <ament_index_cpp/get_package_prefix.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/rclcpp.hpp>

#include "lgw_vui_bridge/srv/set_vui_brightness.hpp"

namespace
{
// Network interface + brightness range — mirrors the pattern in
// lgw_bt_planner/set_led_brightness_node.cpp:50-63,50-75 which this node
// replaces. ETH_INTERFACE_NAME env override lets operators point the
// helper at a different dog subnet without recompiling (e.g. during dog
// re-IP under docker exec -e).
constexpr const char* kDefaultNetworkInterface = "eth10";
constexpr int kBrightnessMin = 0;
constexpr int kBrightnessMax = 10;

const std::string& getNetworkInterface()
{
  static const std::string cached = []() -> std::string {
    if (const char* env_iface = std::getenv("ETH_INTERFACE_NAME");
        env_iface && env_iface[0] != '\0')
    {
      return std::string(env_iface);
    }
    return std::string(kDefaultNetworkInterface);
  }();
  return cached;
}

// Locate the helper binary by walking from this package's install prefix
// (resolved at runtime via ament_index_cpp) to lib/<pkg>/. Works for both
// the install tree and a devel overlay because ament_index_cpp honors
// AMENT_PREFIX_PATH.
std::string resolveHelperPath()
{
  // get_package_prefix returns the install root (e.g. .../install/
  // lgw_vui_bridge), so /lib/<pkg>/<helper> is the canonical sibling
  // layout matching the CMakeLists install(TARGETS ... RUNTIME
  // DESTINATION lib/${PROJECT_NAME}) line.
  const std::string pkg_prefix =
    ament_index_cpp::get_package_prefix("lgw_vui_bridge");
  return pkg_prefix + "/lib/lgw_vui_bridge/go2_vui_set_brightness_helper";
}

// popen() wrapper. Captures stdout into stdout_out, surfaces popen/pclose
// failures into error_out, and returns the helper's WEXITSTATUS (or -1
// on shell-level failure). The helper self-bounds to ~1 s via its
// SDK SetTimeout, so we don't need to add wall-clock polling here.
int runHelper(const std::string& helper_path,
              const std::string& iface, int level,
              std::string& stdout_out, std::string& error_out)
{
  stdout_out.clear();
  error_out.clear();

  const std::string cmd = helper_path + " " + iface + " " + std::to_string(level);
  FILE* pipe = ::popen(cmd.c_str(), "r");
  if (pipe == nullptr) {
    error_out = std::string("popen failed: ") + std::strerror(errno);
    return -1;
  }
  std::array<char, 256> buf{};
  while (::fgets(buf.data(), buf.size(), pipe) != nullptr) {
    stdout_out.append(buf.data());
  }
  const int pclose_rc = ::pclose(pipe);
  if (pclose_rc == -1) {
    error_out = std::string("pclose failed: ") + std::strerror(errno);
    return -1;
  }
  if (!WIFEXITED(pclose_rc)) {
    error_out = "helper killed by signal " + std::to_string(WTERMSIG(pclose_rc));
    return -1;
  }
  return WEXITSTATUS(pclose_rc);
}

// Translate helper exit code to a human-readable phase label so the
// bridge's log line and the BT FAILURE message both surface which
// step of the Set→Get→verify pipeline tripped. The labels match
// go2_vui_set_brightness_helper.cpp: exit codes 0/1/2/3/4.
const char* helperPhaseLabel(int exit_code)
{
  switch (exit_code) {
    case 0:  return "ok_verified";
    case 1:  return "set_fail";      // SetBrightness RPC failed (link /
                                     // firmware hard reject, ret=100 etc.)
    case 2:  return "verify_mismatch";  // Set ok, Get returned a different
                                       // value (firmware dropped Set or
                                       // level out of range)
    case 3:  return "verify_get_fail";  // Set ok, Get RPC itself failed
                                       // (link down between Set and Get)
    case 4:  return "cli_usage_error";
    default: return "unknown";
  }
}
}  // namespace

int main(int argc, char** argv)
{
  // rclcpp::init must happen before Node construction but AFTER we've
  // resolved the helper path — and the helper path resolution depends on
  // nothing rcl-related, so the order is trivial.
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("vui_bridge");

  const std::string helper_path = resolveHelperPath();
  const auto logger = node->get_logger();
  RCLCPP_INFO(logger,
    "vui_bridge ready: iface='%s' (override via ETH_INTERFACE_NAME env), "
    "helper='%s', service /set_vui_brightness live. Bridge process owns "
    "rmw_cyclonedds_cpp ONLY; SDK DomainParticipant lives in helper "
    "subprocess — no in-process libddscxx.so.0 collision.",
    getNetworkInterface().c_str(), helper_path.c_str());

  auto service = node->create_service<lgw_vui_bridge::srv::SetVuiBrightness>(
    "/set_vui_brightness",
    [&logger, &helper_path](
        const std::shared_ptr<
            lgw_vui_bridge::srv::SetVuiBrightness::Request> request,
        std::shared_ptr<
            lgw_vui_bridge::srv::SetVuiBrightness::Response> response)
    {
      const int level = request->level;
      if (level < kBrightnessMin || level > kBrightnessMax) {
        // Same policy as the prior in-process node: warn but forward —
        // firmware behavior is undefined out-of-range, but the BT
        // operator relies on visual confirmation either way.
        RCLCPP_WARN(logger,
          "SetVuiBrightness out-of-range level=%d (valid [%d,%d]); "
          "calling helper anyway (matches prior in-process node policy).",
          level, kBrightnessMin, kBrightnessMax);
      }

      std::string stdout_out;
      std::string error_out;
      const int rc = runHelper(helper_path, getNetworkInterface(), level,
                               stdout_out, error_out);
      if (rc == 0) {
        RCLCPP_INFO(logger,
          "SetVuiBrightness level=%d → helper ok (phase=%s, stdout='%s')",
          level, helperPhaseLabel(rc), stdout_out.c_str());
        response->success = true;
        response->message = "ok";
        return;
      }
      // Non-zero exit: surface what we know. The BT node sees success=
      // false and goes through its FAILURE branch (see
      // set_led_brightness_node.hpp for the FAILURE → FAILURE subtree
      // wiring), so the operator gets a visible log entry without
      // crashing bt_runner.
      RCLCPP_WARN(logger,
        "SetVuiBrightness level=%d → helper exit=%d (phase=%s), "
        "err='%s', stdout='%s' (dog power? eth10 link? EDU screen? "
        "out-of-range level? vui_bridge_node alive in launch?)",
        level, rc, helperPhaseLabel(rc),
        error_out.c_str(), stdout_out.c_str());
      response->success = false;
      response->message = std::string("helper exit ") +
                           std::to_string(rc) +
                           " (" + helperPhaseLabel(rc) + ")" +
                           (error_out.empty() ? std::string() :
                            (": " + error_out));
    });

  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
