// go2_vui_set_brightness_helper.cpp
//
// One-shot CLI wrapper around Unitree Go2 VuiClient with Set+Get+verify
// semantics. Spawned by vui_bridge_node per SetVuiBrightness service call
// to keep the SDK DomainParticipant lifecycle in a separate OS process —
// see feedback_unitree_sdk_cyclonedds_process_isolation.md for the
// architectural rationale.
//
// CLI:
//   go2_vui_set_brightness_helper <network_interface> <level>
//   <level>: int in [0, 10] (10 = brightest per go2_vui_client.cpp:32
//            `++level %= 11`); out-of-range is forwarded to firmware
//            unchanged and the verify step will likely fail to match.
//
// Exit codes (v2, 2026-10-15 Set+Get+verify rewrite):
//   0  Set+Get+verify all OK (GetBrightness returned <level>)
//   1  SetBrightness returned non-zero (link down / firmware rejected
//      the SET; e.g. ret=100 from SDK indicates transient transport or
//      state-machine reject that the next retry may or may not see)
//   2  Set OK but GetBrightness didn't converge to <level> after retry
//      budget (firmware-applied value differs — either the request was
//      silently dropped, or level is out of firmware range)
//   3  Set OK but GetBrightness RPC itself failed (no readback — link
//      down between Set and Get)
//   4  CLI usage error
//
// Verify loop rationale (2026-10-15):
//   After SetBrightness returns 0, give firmware 50 ms to apply, then
//   poll GetBrightness up to kVerifyMaxAttempts times with kVerifySleepUs
//   between. Any attempt that returns the expected value short-circuits
//   to SUCCESS. The retry window absorbs the ~50-150 ms transient
//   ret=100 errors observed when the dog is mid state-machine
//   transition (post-StandUp Euler clear, post-BalanceStand re-entry,
//   etc.) — a single Set+Get pair would have failed those; the retry
//   lets the dog settle.
//
// stdout: "ok level=<level> verified on attempt <N>\n" on full success
//         (N is 1-based; first attempt = 1)
// stderr: human-readable error including both set_ret and get value
//         on every non-zero return, so vui_bridge_node can surface
//         "set ok verify fail" / "set fail" / etc. in its log.

#include <cstdlib>
#include <iostream>
#include <string>
#include <unistd.h>

#include <unitree/robot/channel/channel_factory.hpp>
#include <unitree/robot/go2/vui/vui_client.hpp>

namespace
{
// How long to wait after SetBrightness returns 0 before the first
// GetBrightness poll. The Go2 VUI handler applies the value to the EDU
// screen backlight within a few hundred microseconds on a quiescent
// dog, but during a state-machine transition (post-Euler clear, mid
// re-balance, etc.) the apply path can be delayed by tens of ms while
// the actuator loop runs.
constexpr int kVerifyGraceUs = 50'000;  // 50 ms initial grace
constexpr int kVerifyMaxAttempts = 5;
constexpr int kVerifySleepUs = 50'000;  // 50 ms between polls
// SetBrightness retry budget. The dog can transiently reject the
// VUI service call (SDK ret=100 observed 2026-10-15) when the state
// machine is mid-transition — e.g. just after a SetBodyPose clear or
// re-entering BalanceStand. Retrying with a small backoff absorbs
// most of these without bubbling FAILURE up to the BT.
constexpr int kSetMaxAttempts = 3;
constexpr int kSetSleepUs = 50'000;  // 50 ms between Set attempts
}  // namespace

int main(int argc, char** argv)
{
  if (argc < 3) {
    std::cerr << "Usage: " << argv[0]
              << " <network_interface> <level>" << std::endl;
    return 4;
  }

  const std::string network_interface = argv[1];
  const int level = std::atoi(argv[2]);

  // Match go2_vui_client.cpp:14-21: Init domain on eth10 (or override),
  // 1.0s SetTimeout to bound each synchronous RPC.
  unitree::robot::ChannelFactory::Instance()->Init(0, network_interface);
  unitree::robot::go2::VuiClient vc;
  vc.SetTimeout(1.0f);
  vc.Init();

  // ---- 1. SetBrightness (with retry on transient ret != 0) --------------
  // First attempt is cheap (~50-100 ms RPC); the retry loop only fires
  // when the dog is mid state-transition and the VUI service handler
  // has briefly dropped the request. After kSetMaxAttempts, the
  // helper gives up and returns exit 1 — a persistent Set failure
  // (link down, dog powered off, eth10 unplugged) is a real problem
  // the operator should see in the BT log.
  int32_t set_ret = -1;
  int set_attempt = 0;
  for (set_attempt = 1; set_attempt <= kSetMaxAttempts; ++set_attempt) {
    set_ret = vc.SetBrightness(level);
    if (set_ret == 0) {
      break;
    }
    if (set_attempt < kSetMaxAttempts) {
      usleep(kSetSleepUs);
    }
  }
  if (set_ret != 0) {
    std::cerr << "fail set level=" << level
              << " ret=" << set_ret
              << " after " << kSetMaxAttempts
              << " attempts (verify skipped — no Set confirmed). "
              << "Likely causes: eth10 link down, dog powered off, "
              << "or VUI service in extended state-machine reject."
              << std::endl;
    return 1;
  }

  // ---- 2. Initial grace for firmware to apply ---------------------------
  usleep(kVerifyGraceUs);

  // ---- 3. Verify by GetBrightness retry loop ----------------------------
  int value = -1;
  int get_ret = -1;
  int attempt = 0;
  for (attempt = 1; attempt <= kVerifyMaxAttempts; ++attempt) {
    get_ret = vc.GetBrightness(value);
    if (get_ret == 0 && value == level) {
      // Firmware confirmed the applied value matches the request.
      std::cout << "ok level=" << level
                << " verified on attempt " << attempt << std::endl;
      return 0;
    }
    if (attempt < kVerifyMaxAttempts) {
      usleep(kVerifySleepUs);
    }
  }

  // ---- 4. Failure reporting ---------------------------------------------
  // Two distinct failure modes, both exit 2/3:
  //   - get_ret != 0: Get RPC itself failed → exit 3 (link down
  //     mid-verify, or VUI service stopped responding). The Set
  //     already went through so the value is "applied" but unknown.
  //   - get_ret == 0 but value != level: Firmware readback is reachable
  //     but reports a different value → exit 2 (firmware silently
  //     dropped the Set, OR level is out of supported range and
  //     firmware clamped it to a different value).
  if (get_ret != 0) {
    std::cerr << "fail verify_get level=" << level
              << " set_ok=true get_ret=" << get_ret
              << " after " << kVerifyMaxAttempts
              << " attempts" << std::endl;
    return 3;
  }
  std::cerr << "fail verify_mismatch level=" << level
            << " set_ok=true got=" << value
            << " after " << kVerifyMaxAttempts
            << " attempts (firmware didn't apply the Set, or "
               "level is out of supported range)" << std::endl;
  return 2;
}
