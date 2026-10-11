#pragma once
#include "sangwon_ai/trajectory.hpp"
#include "sangwon_ai/scan.hpp"
#include <memory>

namespace sangwon {
class Runtime {
 public:
  explicit Runtime(Config config = {});
  ~Runtime();
  Runtime(const Runtime&)=delete;
  Runtime& operator=(const Runtime&)=delete;
  bool start(const Mission&, const State&, double now);
  bool command(Command command);
  void control_fault(const std::string& reason,bool released);
  std::optional<Intent> tick(const State&,double now);
  Phase phase() const { return phase_; }
  std::size_t visited() const { return visited_; }
  bool returning() const { return return_lock_; }
  bool paused() const { return paused_; }
  bool recovering() const { return recovering_; }
  double max_overshoot_m() const { return max_overshoot_; }
  const std::string& reason() const { return reason_; }
  const std::string& boot_session() const { return boot_; }
  const std::vector<std::string>& events() const { return events_; }
  std::uint64_t route_revision() const { return route_revision_; }
  const ScanAction* scan_action() const { return scan_.get(); }
  const std::vector<ScanResult>& scan_results() const { return scan_results_; }
  const std::optional<ScanDecision>& scan_decision() const { return scan_decision_; }
 private:
  struct TreeImpl;
  std::unique_ptr<TreeImpl> bt_;
  Config c_; Mission mission_; State state_;
  std::unique_ptr<ScanAction> scan_;
  std::vector<ScanResult> scan_results_;
  bool scan_movement_complete_{};
  std::optional<ScanDecision> scan_decision_;
  std::string boot_{"replay-boot-1"},branch_,reason_;
  std::vector<std::string> events_;
  Phase phase_{Phase::Idle};
  Pose home_,hold_,goal_;
  std::vector<Pose> history_,return_path_;
  std::vector<Vec3> navigation_path_;
  std::size_t navigation_index_{};
  Pose blocked_target_;
  double plan_retry_s_{};
  bool navigation_active_{},map_blocked_{};
  std::uint64_t route_revision_{};
  std::optional<double> navigation_deadline_active_;
  std::optional<Pose> return_vertical_goal_;
  std::size_t visited_{},return_index_{};
  std::uint64_t generation_{1},sequence_{},frame_{1};
  double now_{},last_tick_{},active_s_{},phase_start_active_{},acquire_start_{};
  double timeout_s_{},best_error_{},progress_active_{},max_overshoot_{};
  double segment_start_active_{};
  std::optional<double> stable_since_,recovery_since_,recovery_stable_,blocked_since_;
  std::optional<double> prestream_since_,last_prestream_;
  double native_takeoff_since_{},handoff_since_{};
  bool native_completed_{},control_released_{};
  std::string control_fault_;
  std::optional<Segment> segment_;
  std::optional<Intent> output_;
  bool started_{},paused_{},recovering_{},return_lock_{},land_lock_{},manual_lock_{};
  bool cancel_{},land_command_{},resume_{},recovery_interrupted_{};
  bool motion_initialized_{};
  bool motion_tracks_yaw_{};
  void update_safety();
  bool choose(const std::string&) const;
  void run(const std::string&);
  void halt(const std::string&);
  void emit(IntentKind,Pose);
  void enter(Phase);
  void request_return(const std::string&);
  void request_land(const std::string&);
  void move_mission();
  bool move_to(Pose,double speed,bool require_yaw=true,const Config* trajectory_config=nullptr);
  bool move_route(Pose,double speed,bool require_yaw=true,bool face_path=false);
  void block_route(Pose,const std::string&);
  void retry_route();
  void suspend_motion();
  bool stable(Pose,double duration);
  void next_waypoint();
};
} // namespace sangwon
