#include "lgw_bt_planner/xacro_condition_filter.hpp"

#include <regex>
#include <vector>

namespace lgw_bt_planner
{

namespace
{
const std::regex kEvalWrapper(R"(^\s*\$\(eval\s+([\s\S]*)\)\s*$)");
const std::regex kClause(R"(^\s*([A-Za-z_]\w*)\s*(==|!=)\s*'([^']*)'\s*$)");

std::string trim(const std::string& s)
{
  const auto begin = s.find_first_not_of(" \t\r\n");
  if (begin == std::string::npos) {
    return "";
  }
  const auto end = s.find_last_not_of(" \t\r\n");
  return s.substr(begin, end - begin + 1);
}

std::string lookup(
  const std::string& ident, const std::string& arm_type,
  const std::string& effector_type, const std::string& revo2_type)
{
  if (ident == "arm_type") return arm_type;
  if (ident == "effector_type") return effector_type;
  if (ident == "revo2_type") return revo2_type;
  return {};  // unknown → caller treats as failure
}

}  // namespace

bool eval_xacro_condition(
  const std::string& expr, const std::string& arm_type,
  const std::string& effector_type, const std::string& revo2_type)
{
  std::smatch m;
  if (!std::regex_match(expr, m, kEvalWrapper)) {
    return false;  // not an $(eval ...) expression — fail closed
  }
  const std::string body = m[1].str();

  // Split on "and" (only operator this xacro uses).
  std::vector<std::string> clauses;
  {
    std::size_t pos = 0;
    while (true) {
      const std::size_t and_pos = body.find(" and ", pos);
      if (and_pos == std::string::npos) {
        clauses.push_back(trim(body.substr(pos)));
        break;
      }
      clauses.push_back(trim(body.substr(pos, and_pos - pos)));
      pos = and_pos + 5;
    }
  }

  for (const auto& clause : clauses) {
    std::smatch cm;
    if (!std::regex_match(clause, cm, kClause)) {
      return false;  // `or`, comparisons between idents, etc. — fail closed
    }
    const std::string ident = cm[1].str();
    const std::string op = cm[2].str();
    const std::string expected = cm[3].str();
    const std::string value = lookup(ident, arm_type, effector_type, revo2_type);
    if (value.empty()) {
      return false;  // unknown identifier
    }
    const bool equal = (value == expected);
    if ((op == "==" && !equal) || (op == "!=" && equal)) {
      return false;
    }
  }
  return true;
}

std::string strip_inactive_xacro_blocks(
  const std::string& content, const std::string& arm_type,
  const std::string& effector_type, const std::string& revo2_type)
{
  static const std::regex kValueAttr(R"(value\s*=\s*"([^"]*)\")");

  std::string out;
  out.reserve(content.size());

  // One entry per open conditional: whether its BRANCH is taken. The block is
  // emitted only when this and every enclosing entry are true.
  std::vector<bool> stack;

  auto emitting = [&stack]() {
    for (bool active : stack) {
      if (!active) return false;
    }
    return true;
  };

  std::size_t pos = 0;
  while (pos < content.size()) {
    const std::size_t if_pos = content.find("<xacro:if", pos);
    const std::size_t un_pos = content.find("<xacro:unless", pos);
    std::size_t open_pos;
    bool is_if = true;
    if (if_pos == std::string::npos) {
      open_pos = un_pos;
      is_if = false;
    } else if (un_pos == std::string::npos) {
      open_pos = if_pos;
    } else if (if_pos <= un_pos) {
      open_pos = if_pos;
    } else {
      open_pos = un_pos;
      is_if = false;
    }

    const std::size_t close_if = content.find("</xacro:if>", pos);
    const std::size_t close_un = content.find("</xacro:unless>", pos);
    std::size_t close_pos;
    bool close_is_if = true;
    if (close_if == std::string::npos) {
      close_pos = close_un;
      close_is_if = false;
    } else if (close_un == std::string::npos) {
      close_pos = close_if;
    } else if (close_if <= close_un) {
      close_pos = close_if;
    } else {
      close_pos = close_un;
      close_is_if = false;
    }

    // Whichever comes first: an opening tag or a closing tag.
    if (open_pos != std::string::npos &&
        (close_pos == std::string::npos || open_pos < close_pos))
    {
      if (emitting()) {
        out.append(content, pos, open_pos - pos);
      }
      const std::size_t tag_end = content.find('>', open_pos);
      if (tag_end == std::string::npos) {
        break;
      }
      const std::string tag = content.substr(open_pos, tag_end - open_pos + 1);
      std::smatch vm;
      bool cond = false;
      if (std::regex_search(tag, vm, kValueAttr)) {
        cond = eval_xacro_condition(
          vm[1].str(), arm_type, effector_type, revo2_type);
      }
      if (!is_if) {
        cond = !cond;
      }
      stack.push_back(cond);
      pos = tag_end + 1;
      continue;
    }

    if (close_pos != std::string::npos) {
      if (emitting()) {
        out.append(content, pos, close_pos - pos);
      }
      if (!stack.empty()) {
        stack.pop_back();
      }
      pos = close_pos + (close_is_if ? std::string("</xacro:if>").size()
                                     : std::string("</xacro:unless>").size());
      continue;
    }

    // No more tags.
    if (emitting()) {
      out.append(content, pos, std::string::npos);
    }
    break;
  }
  return out;
}

}  // namespace lgw_bt_planner
