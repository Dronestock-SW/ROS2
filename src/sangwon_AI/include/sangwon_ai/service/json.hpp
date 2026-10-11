#pragma once
// JSON comes from the already pinned BehaviorTree.CPP distribution.
#include <behaviortree_cpp/contrib/json.hpp>
#include <openssl/sha.h>
#include <iomanip>
#include <set>
#include <sstream>

namespace sangwon::service {
using Json = nlohmann::json;
double utc_parse(std::string text);
inline Json parse(const std::string& text) {
  std::vector<std::set<std::string>> keys;
  return Json::parse(text, [&keys](int depth, Json::parse_event_t event, Json& value) {
    if(depth>32) throw std::invalid_argument("JSON_NESTING_LIMIT");
    if (event == Json::parse_event_t::object_start) keys.emplace_back();
    if (event == Json::parse_event_t::key && !keys.back().insert(value.get<std::string>()).second)
      throw std::invalid_argument("DUPLICATE_JSON_KEY");
    if (event == Json::parse_event_t::object_end) keys.pop_back();
    return true;
  });
}
inline std::string digest(const std::string& text) {
  unsigned char bytes[SHA256_DIGEST_LENGTH];
  SHA256(reinterpret_cast<const unsigned char*>(text.data()), text.size(), bytes);
  std::ostringstream out;
  for (auto b : bytes) out << std::hex << std::setw(2) << std::setfill('0') << unsigned(b);
  return out.str();
}
inline void require(bool yes, const std::string& code) {
  if (!yes) throw std::invalid_argument(code);
}
}  // namespace sangwon::service
