// transit_main.cpp — 进程入口。
//
// 单独成文件是为了让节点类可以被 gtest 直接链接（否则 main() 会和
// gtest_main 冲突）。

#include <csignal>
#include <memory>

#include <rclcpp/rclcpp.hpp>

#include "go2_transit/transit_node.hpp"

namespace
{
volatile std::sig_atomic_t g_interrupted = 0;

void handle_signal(int)
{
  g_interrupted = 1;
}
}  // namespace

int main(int argc, char ** argv)
{
  // 自己接管 SIGINT：rclcpp 默认处理器会立刻 shutdown()，而在那之前必须
  // 把在飞的 nav2 动作取消掉，否则节点退出后狗还会继续往目标走。
  rclcpp::InitOptions init_options;
  init_options.shutdown_on_signal = false;
  rclcpp::init(argc, argv, init_options);
  std::signal(SIGINT, handle_signal);
  std::signal(SIGTERM, handle_signal);

  auto node = std::make_shared<go2_transit::TransitNode>();
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);

  while (rclcpp::ok() && g_interrupted == 0) {
    // spin_some() 是"收一次当下就绪的工作": 没有工作时立即返回, 放在 while
    // 里就是忙等 —— 2026-11-20 实测空闲时它占用 ~23% 一个 CPU 核, 每秒被唤醒
    // 7000+ 次(Jetson 上白白吃掉一个核的 1/4)。spin_once(timeout) 会阻塞到
    // 有工作或超时, 空闲 CPU 接近 0, 20 Hz tick 与 2 Hz status 照常跑。
    executor.spin_once(std::chrono::milliseconds(50));
  }

  node->cancel_all_on_shutdown(executor);

  executor.remove_node(node);
  node.reset();
  rclcpp::shutdown();
  return 0;
}
