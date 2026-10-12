// Physical-input mission decisions. No ROS, FC transport, simulated vehicle,
// attitude output or vertical trajectory. The sole executor applies intents.
#include "sangwon_ai/geometry.hpp"
#include "sangwon_ai/lidar_relative.hpp"
#include "sangwon_ai/service/json.hpp"
#include <behaviortree_cpp/bt_factory.h>
#include <iostream>
#include <memory>
#include <limits>

using namespace sangwon;
using namespace sangwon::service;
namespace {
constexpr const char* schema = "sangwon-field-ai/1";
constexpr const char* backend = "sangwon_cpp_bt";
double number(const Json& x, double lo, double hi) {
  require(x.is_number() && !x.is_boolean(), "FINITE_NUMBER_REQUIRED");
  const double n = x.get<double>();
  require(std::isfinite(n) && n >= lo && n <= hi, "NUMBER_OUTSIDE_LIMIT");
  return n;
}
Vec3 point(const Json& x) {
  require(x.is_array() && x.size() == 2, "XY_REQUIRED");
  return {number(x[0], -1000, 1000), number(x[1], -1000, 1000), 0};
}
Json xy_json(Vec3 p) { return Json::array({p.x, p.y}); }
bool flag(const Json& x, const char* key) {
  return x.contains(key) && x.at(key).is_boolean() && x.at(key).get<bool>();
}
bool age(const Json& s, const char* key, double max) {
  if(!s.contains(key) || !s.at(key).is_number() || s.at(key).is_boolean()) return false;
  const double n = s.at(key).get<double>();
  return std::isfinite(n) && n >= 0 && n <= max;
}
struct Task { Vec3 xy; double dwell; };

class MissionAI {
 public:
  explicit MissionAI(const Json& in) {
    execution_ = in.at("execution_id").get<std::string>();
    require(!execution_.empty() && execution_.size() <= 128, "EXECUTION_ID_REQUIRED");
    const auto& cfg = in.at("settings");
    auto lidar_config=cfg.value("lidar_gap",Json::object());
    lidar_config["map_y_axis_sign"]=cfg.value("map_y_axis_sign",1);
    lidar_ = std::make_unique<observe::LidarGapSupport>(lidar_config);
    require(cfg.at("command_frame") == "local_xy" && cfg.at("execution_profile") == "anchor_trial",
            "BOUNDED_LOCAL_XY_PROFILE_REQUIRED");
    layout_ = cfg.at("layout_id").get<std::string>();
    tag_ = cfg.at("drone_id").get<std::string>();
    require((tag_ == "5" || tag_ == "6") && !layout_.empty(), "IDENTITY_REQUIRED");
    require(cfg.at("bounds_xy_m").is_array() && cfg.at("bounds_xy_m").size() == 4, "BOUNDS_REQUIRED");
    for(std::size_t i = 0; i < 4; ++i) bounds_[i] = number(cfg.at("bounds_xy_m").at(i), -1000, 1000);
    require(bounds_[0] < bounds_[1] && bounds_[2] < bounds_[3], "INVALID_BOUNDS");
    speed_ = number(cfg.at("speed_m_s"), .001, .25);
    max_leg_ = number(cfg.at("max_leg_m"), .001, 2);
    pose_age_ = number(cfg.at("pose_timeout_s"), .001, .2);
    settle_ = number(cfg.at("takeoff_settle_s"), .001, 30);
    leg_timeout_ = number(cfg.at("leg_timeout_s"), .001, 120);
    takeoff_timeout_ = number(cfg.at("takeoff_timeout_s"), .001, 120);
    home_ = stream_ = goal_ = point(in.at("home_xy"));
    require(inside(home_), "LAUNCH_OUTSIDE_BOUNDS");
    height_ = number(in.at("height_reference_m"), -100, 100);
    const double climb = number(cfg.at("expected_mis_takeoff_alt_m"), .1, 10);
    require(number(in.at("ceiling_height_m"), .1, 100) >= climb + .25, "CEILING_CLEARANCE_REQUIRED");
    const auto& tasks = in.at("waypoints");
    require(tasks.is_array() && !tasks.empty() && tasks.size() <= 20, "ONE_TO_TWENTY_TASKS_REQUIRED");
    Vec3 previous = home_;
    for(const auto& t : tasks) {
      require(t.at("type") == "waypoint" || t.at("type") == "hover", "XY_HOVER_TASKS_ONLY");
      require(!t.contains("z") && (!t.contains("yaw_deg") || t.at("yaw_deg").is_null()), "VERTICAL_OR_YAW_TASK_FORBIDDEN");
      const auto p = point(t.at("xy"));
      require(inside(p) && xy(p - previous) <= max_leg_ + 1e-9, "TASK_OUTSIDE_TRIAL_ROUTE");
      tasks_.push_back({p, number(t.at("dwell_s"), 0, 30)});
      previous = p;
    }
    visited_.push_back(home_);
    plan_ = "cpp-bt-local-xy:" + digest(in.dump());
    factory_.registerSimpleCondition("ShouldYield", [this](BT::TreeNode&) {
      return yielding_ ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
    });
    factory_.registerSimpleCondition("ShouldLand", [this](BT::TreeNode&) {
      return phase_ == "LANDING" ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
    });
    factory_.registerSimpleAction("Yield", [this](BT::TreeNode&) {
      phase_ = "PILOT_OVERRIDE"; return BT::NodeStatus::SUCCESS;
    });
    factory_.registerSimpleAction("Land", [this](BT::TreeNode&) {
      const auto& s = input_.at("state");
      if(flag(input_, "landing_confirmed") && flag(s, "connected") && !flag(s, "armed")
          && s.at("landed") == 1 && age(s, "state_age_s", 1) && age(s, "landed_age_s", 1))
        phase_ = failure_.empty() && completed_ == tasks_.size() ? "END" : "FAILED";
      return BT::NodeStatus::SUCCESS;
    });
    factory_.registerSimpleAction("Execute", [this](BT::TreeNode&) {
      advance(); return BT::NodeStatus::SUCCESS;
    });
    tree_ = factory_.createTreeFromText(R"(
      <root BTCPP_format="4" main_tree_to_execute="FieldMission">
        <BehaviorTree ID="FieldMission"><ReactiveFallback>
          <ReactiveSequence><ShouldYield/><Yield/></ReactiveSequence>
          <ReactiveSequence><ShouldLand/><Land/></ReactiveSequence>
          <Execute/>
        </ReactiveFallback></BehaviorTree>
      </root>)");
  }
  Json prepared() const { return {{"phase", phase_}, {"plan_ref", plan_},
    {"execution_id", execution_}, {"home_xy", xy_json(home_)}, {"waypoint_count", tasks_.size()},
    {"validation_scope", "BOUNDED_LOCAL_XY_ONLY"}, {"obstacle_map_checked", false}}; }
  Json release(const Json& in) {
    require(in.at("execution_id") == execution_ && in.at("plan_ref") == plan_, "MISSION_CONTEXT_MISMATCH");
    const auto outcome = in.at("outcome").get<std::string>();
    require(outcome == "PILOT_OVERRIDE" || outcome == "UNCONFIRMED" || outcome == "FAILED", "NON_SUCCESS_RELEASE_REQUIRED");
    yielding_ = true; phase_ = outcome; failure_ = in.at("reason").get<std::string>();
    auto out = prepared(); out["reason"] = failure_; return out;
  }
  Json step(const Json& in) {
    require(in.at("execution_id") == execution_ && in.at("plan_ref") == plan_, "MISSION_CONTEXT_MISMATCH");
    now_ = number(in.at("sample_time_s"), 0, 1e12);
    require(!last_time_ || now_ >= *last_time_, "BACKWARD_SAMPLE_TIME");
    dt_ = last_time_ ? now_ - *last_time_ : .05;
    last_time_ = now_; input_ = in;
    const auto& s = in.at("state");
    const auto executor = in.at("executor_phase").get<std::string>();
    lidar_status_ = lidar_->update(s,now_);
    observation_gap_ = (!age(s,"uwb_age_s",.25) || !age(s,"bridge_observation_age_s",.35));
    gap_supported_ = observation_gap_ && flag(lidar_status_,"approved")
        && (executor=="TAKING_OFF" || executor=="MOVING" || executor=="DWELL" || executor=="RETURNING")
        && age(s,"uwb_age_s",.8) && age(s,"bridge_observation_age_s",.8);
    if(executor == "PILOT_OVERRIDE" || flag(in, "pilot_override")) yielding_ = true;
    if(flag(s, "connected") && age(s, "state_age_s", 1)) {
      const auto mode = s.at("mode").get<std::string>();
      const bool allowed = executor == "PREPARING" ? (mode == "POSCTL" || mode == "AUTO.LOITER" || mode == "OFFBOARD")
          : executor == "LANDING" ? (mode == "OFFBOARD" || mode == "AUTO.LAND")
          : executor == "GROUND_RECOVERY" ? (mode == "POSCTL" || mode == "AUTO.LOITER" || mode == "OFFBOARD")
          : mode == "OFFBOARD";
      if(!allowed) yielding_ = true;
    }
    if(executor == "UNCONFIRMED") { yielding_ = true; failure_ = "COMMAND_EFFECT_UNKNOWN"; }
    if(executor == "LANDING") request_land(in.value("executor_failure", std::string()));
    if(executor == "GROUND_RECOVERY" || executor == "FAILED") { failure_ = in.value("executor_failure", std::string("EXECUTOR_FAILED")); phase_ = "FAILED"; }
    if(phase_ != "END" && phase_ != "FAILED" && phase_ != "PILOT_OVERRIDE" && phase_ != "LANDING") {
      if(!flag(s, "connected") || !age(s, "state_age_s", 2.5)) request_land("FC_STATE_UNAVAILABLE");
      else if(dt_ > .2 + 1e-9) request_land("AI_SAMPLE_GAP");
      else if(!flag(s, "estimator_valid") || !flag(s, "vertical_estimator_valid") || !age(s, "estimator_age_s", .5)) request_land("ESTIMATOR_UNAVAILABLE");
      else if(!age(s, "pose_age_s", pose_age_)) request_land("PX4_POSE_UNAVAILABLE");
      else if(flag(lidar_status_,"enabled") && lidar_status_.at("reason") == "UWB_LIDAR_RECOVERY_DISAGREEMENT") request_land("UWB_LIDAR_RECOVERY_DISAGREEMENT");
      else if(!age(s, "uwb_age_s", .25) && !gap_supported_) request_land("UWB_UNAVAILABLE");
      else if(!flag(s, "bridge_ready") || !flag(s, "alignment_matches") || (!age(s, "bridge_observation_age_s", .35) && !gap_supported_)) request_land("OBSERVATION_BRIDGE_UNAVAILABLE");
      else if(!age(s, "battery_age_s", 3) || number(s.at("battery"), 0, 1) < .15) request_land("BATTERY_UNAVAILABLE_OR_LOW");
    }
    if(phase_ != "END" && phase_ != "FAILED" && phase_ != "PILOT_OVERRIDE") tree_.tickOnce();
    Json out = prepared();
    out.update({{"lidar_support",lidar_status_},{"observation_gap",observation_gap_},{"gap_supported",gap_supported_},
      {"sample_time_s", now_}, {"expires_s", now_ + .15}, {"goal_xy", xy_json(goal_)},
      {"stream_xy", xy_json(stream_)}, {"completed_waypoints", completed_}, {"waypoint_index", index_},
      {"returning", returning_}, {"home_verified", home_verified_}, {"leg_sequence", leg_},
      {"reason", failure_.empty() ? reason_ : failure_}, {"stable_s", stable_since_ ? now_ - *stable_since_ : 0}});
    return out;
  }
 private:
  BT::BehaviorTreeFactory factory_; BT::Tree tree_;
  std::unique_ptr<observe::LidarGapSupport> lidar_;
  Json lidar_status_; bool observation_gap_{}, gap_supported_{};
  Json input_; std::string execution_, layout_, tag_, plan_, phase_{"WAIT_TAKEOFF"}, failure_, reason_;
  double bounds_[4]{}, speed_{}, max_leg_{}, pose_age_{}, settle_{}, height_{}, leg_timeout_{}, takeoff_timeout_{};
  double now_{}, dt_{}, entered_{}, takeoff_started_{};
  std::optional<double> last_time_, stable_since_;
  std::vector<Task> tasks_; std::vector<Vec3> visited_, return_path_;
  Vec3 home_, stream_, goal_; std::size_t index_{}, completed_{}, return_index_{};
  unsigned leg_{}; bool returning_{}, home_verified_{}, yielding_{};
  bool inside(Vec3 p) const { return p.x >= bounds_[0] && p.x <= bounds_[1] && p.y >= bounds_[2] && p.y <= bounds_[3]; }
  void request_land(const std::string& why) {
    if(phase_ == "END" || phase_ == "FAILED") return;
    if(!why.empty() && failure_.empty()) failure_ = why;
    phase_ = "LANDING"; stable_since_.reset();
  }
  void choose_goal(Vec3 p) {
    require(inside(p), "GOAL_OUTSIDE_BOUNDS"); goal_ = p;
    phase_ = returning_ ? "RETURNING" : "MOVING";
    entered_ = now_; stable_since_.reset(); ++leg_;
  }
  bool stable(bool ok, double seconds) {
    if(!ok) { stable_since_.reset(); return false; }
    if(!stable_since_) stable_since_ = now_;
    return now_ - *stable_since_ + 1e-9 >= seconds;
  }
  void begin_return(Vec3 current) {
    returning_ = true; return_path_.clear();
    if(xy(visited_.back() - home_) < .01) return_path_.push_back(home_);
    else return_path_.assign(visited_.rbegin(), visited_.rend());
    while(return_path_.size() > 1 && xy(return_path_.front() - current) < .15) return_path_.erase(return_path_.begin());
    return_index_ = 0; choose_goal(return_path_.front());
  }
  void advance() {
    const auto executor = input_.at("executor_phase").get<std::string>();
    const auto& s = input_.at("state");
    if(phase_ == "WAIT_TAKEOFF" && executor != "TAKING_OFF" && executor != "HANDOFF") return;
    const Vec3 current = point(s.at("xy")), measured = point(s.at("uwb_xy"));
    const double velocity = xy(point(s.at("velocity_xy")));
    const double vertical = std::abs(number(s.at("vertical_speed_m_s"), -20, 20));
    const double height_error = std::abs(number(s.at("altitude_enu_m"), -100, 100) - height_);
    if(phase_ == "WAIT_TAKEOFF") {
      if(!takeoff_started_) takeoff_started_ = now_;
      if(xy(current - home_) > .6) { request_land("TAKEOFF_HORIZONTAL_DRIFT"); return; }
      if(now_ - takeoff_started_ > takeoff_timeout_) { request_land("TAKEOFF_TIMEOUT"); return; }
      if(gap_supported_) { stable_since_.reset(); reason_="LIDAR_SUPPORTED_TAKEOFF_WAIT_FOR_UWB"; return; }
      if(stable(flag(s, "armed") && s.at("landed") == 2 && age(s, "feedback_age_s", .5)
          && height_error <= .15 && vertical <= .1, settle_)) choose_goal(tasks_.front().xy);
      return;
    }
    if(!flag(s, "armed") || s.at("landed") != 2 || !age(s, "feedback_age_s", .5)) { request_land("AIRBORNE_OR_TARGET_FEEDBACK_LOST"); return; }
    if(input_.value("command", std::string()) == "return_to_home" && !returning_) begin_return(current);
    if(input_.value("command", std::string()) == "land") { request_land("OPERATOR_LAND"); return; }
    if(gap_supported_) {
      // Preserve the last streamed XY. No arrival, dwell, home, or END credit.
      stable_since_.reset(); reason_="LIDAR_SUPPORTED_FIXED_TARGET_WAIT_FOR_UWB";
      if(now_-entered_>leg_timeout_+tasks_.at(std::min(index_,tasks_.size()-1)).dwell) request_land("OBSERVATION_RECOVERY_TIMEOUT");
      return;
    }
    if(phase_ == "MOVING" || phase_ == "RETURNING") {
      const double distance = xy(goal_ - stream_);
      if(distance > 0) stream_ = stream_ + (goal_ - stream_) * (std::min(distance, speed_ * dt_) / distance);
      if(xy(current - stream_) > .5) { request_land("LOCAL_PATH_TRACKING_ERROR"); return; }
      if(now_ - entered_ > leg_timeout_) { request_land("LOCAL_LEG_TIMEOUT"); return; }
      if(stable(xy(goal_ - stream_) <= .01 && xy(current - goal_) <= .15 && xy(measured - goal_) <= .25 && velocity <= .1, 1)) {
        if(returning_) {
          if(++return_index_ == return_path_.size()) { home_verified_ = true; reason_ = "HOME_ARRIVAL_VERIFIED"; request_land(""); }
          else choose_goal(return_path_.at(return_index_));
        } else { visited_.push_back(goal_); phase_ = "DWELL"; entered_ = now_; stable_since_.reset(); }
      }
    } else if(phase_ == "DWELL") {
      if(xy(current - goal_) > .5) { request_land("HOVER_HORIZONTAL_DRIFT"); return; }
      if(now_ - entered_ > leg_timeout_ + tasks_.at(index_).dwell) { request_land("HOVER_STABILITY_TIMEOUT"); return; }
      if(stable(xy(current - goal_) <= .15 && xy(measured - goal_) <= .25 && velocity <= .1 && height_error <= .15 && vertical <= .1, tasks_.at(index_).dwell)) {
        ++completed_; ++index_;
        if(index_ == tasks_.size()) begin_return(current); else choose_goal(tasks_.at(index_).xy);
      }
    }
  }
};
}

int main(int argc, char** argv) {
  if(argc != 2 || std::string(argv[1]) != "--stdio") return 2;
  std::unique_ptr<MissionAI> mission;
  std::string session; std::uint64_t previous = 0;
  for(std::string line; std::getline(std::cin, line);) {
    Json response = {{"schema", schema}, {"backend", backend}, {"fc_transport", false}};
    try {
      require(line.size() <= 262144, "MESSAGE_TOO_LARGE");
      const auto req = parse(line);
      require(req.at("schema") == schema, "SCHEMA_MISMATCH");
      const auto seq = req.at("sequence").get<std::uint64_t>();
      const auto context = req.at("session_id").get<std::string>();
      require(!context.empty() && context.size() <= 128 && seq > previous, "SESSION_OR_SEQUENCE_INVALID");
      if(session.empty()) session = context;
      require(context == session, "SESSION_MISMATCH"); previous = seq;
      response.update({{"sequence", seq}, {"session_id", session}});
      const auto method = req.at("method").get<std::string>();
      if(method == "hello") response["payload"] = {{"tree", "FieldMission"}, {"mission_scope", "LOCAL_XY_HOVER_RETURN"}};
      else if(method == "prepare") {
        require(!mission, "ONE_EXECUTION_PER_PROCESS");
        mission = std::make_unique<MissionAI>(req.at("payload")); response["payload"] = mission->prepared();
      } else if(method == "step") {
        require(bool(mission), "MISSION_NOT_PREPARED"); response["payload"] = mission->step(req.at("payload"));
      } else if(method == "release") {
        require(bool(mission), "MISSION_NOT_PREPARED"); response["payload"] = mission->release(req.at("payload"));
      } else throw std::invalid_argument("UNKNOWN_METHOD");
      response["ok"] = true;
    } catch(const std::exception& e) {
      response.update({{"ok", false}, {"code", e.what()}});
    }
    std::cout << response.dump() << '\n' << std::flush;
  }
}
