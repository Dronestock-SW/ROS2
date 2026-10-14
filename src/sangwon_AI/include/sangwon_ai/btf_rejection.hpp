#pragma once
#include "sangwon_ai/service/json.hpp"
#include <cstdint>
#include <string>
namespace sangwon::field {
// Stop progress on a rejected candidate. Delayed diagnostics cannot revoke a
// newer accepted pose. Only actual pose publication releases this latch.
struct BtfRejection {
  std::int64_t decision_ns{}, rejected_ns{};
  std::string reason;
  void update(const sangwon::service::Json& j,std::int64_t now_ns) {
    try {
      if(j.at("schema")!=1||!j.at("published").is_boolean()||j.at("published").get<bool>())return;
      const auto stamp=j.at("decision_stamp_ns").get<std::int64_t>();
      if(stamp<=decision_ns||stamp>now_ns||now_ns-stamp>250000000)return;
      auto sample=j.value("stamp_ns",std::int64_t(0));
      if(sample<=0)sample=j.at("input_received_ros_ns").get<std::int64_t>();
      if(sample<=0||sample>stamp||sample<=rejected_ns)return;
      reason=j.value("pose_blocked_reason",j.value("reason",std::string("rejected")));
      decision_ns=stamp;rejected_ns=sample;
    } catch(const std::exception&) { /* Malformed reports never fabricate data. */ }
  }
  bool blocked(std::int64_t accepted_pose_ns)const {return rejected_ns>accepted_pose_ns;}
};
}
