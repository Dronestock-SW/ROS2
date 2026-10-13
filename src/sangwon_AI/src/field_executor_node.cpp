// Sole FC writer. Heavy scan/JSON/BT work runs outside the 20 Hz sender group.
#include "sangwon_ai/field_executor.hpp"
#include "sangwon_ai/writer_lock.hpp"
#include <rclcpp/rclcpp.hpp>
#include <mavros_msgs/msg/state.hpp>
#include <mavros_msgs/msg/extended_state.hpp>
#include <mavros_msgs/msg/estimator_status.hpp>
#include <mavros_msgs/msg/position_target.hpp>
#include <mavros_msgs/msg/rc_in.hpp>
#include <mavros_msgs/msg/timesync_status.hpp>
#include <mavros_msgs/srv/set_mode.hpp>
#include <mavros_msgs/srv/command_bool.hpp>
#include <mavros_msgs/srv/command_tol.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <sensor_msgs/msg/battery_state.hpp>
#include <std_msgs/msg/string.hpp>
#include <openssl/rand.h>
#include <fstream>
#include <mutex>
#include <deque>
#include <thread>
#include <array>
#include <atomic>
#include <condition_variable>
#include <fcntl.h>
#include <unistd.h>

using namespace sangwon::field;
double steady_s() { return std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count(); }
double wall_s() { return std::chrono::duration<double>(std::chrono::system_clock::now().time_since_epoch()).count(); }
struct Sample {
  bool connected{},armed{},estimator{},vertical{},battery_ok{},pose_ok{};
  int landed{};std::string mode;std::array<double,2> xy{},velocity{},uwb{};
  double z{},vz{},yaw{},battery{},state_at{-1e9},landed_at{-1e9},pose_at{-1e9},est_at{-1e9},battery_at{-1e9},rc_at{-1e9},uwb_at{-1e9},feedback_at{-1e9};
  std::array<bool,9> estimator_flags{};std::array<int64_t,8> source_ns{},received_ns{};int64_t uwb_ns{};std::vector<uint16_t> rc;
  int64_t remote_ns{},offset_ns{},timesync_received_ns{};double rtt_ms{};
};
struct View { bool stream{},pending{};unsigned token{};std::string phase;double x{},y{},z{},yaw{},expires{};std::vector<uint16_t> baseline;std::vector<int> switches; };
class ExecutorNode final:public rclcpp::Node {
 public:
  explicit ExecutorNode(Json config):Node("sangwon_field_executor"),outer_(std::move(config)),cfg_(outer_.at("mission")),core_(cfg_) {
    cfg_["execute"]=outer_.at("output_enabled");core_=FieldExecutor(cfg_);
    const int domain=std::stoi(std::getenv("ROS_DOMAIN_ID")?std::getenv("ROS_DOMAIN_ID"):"-1");
    const bool test=flag(outer_,"test_mode");const auto local=std::getenv("ROS_LOCALHOST_ONLY");
    require(test ? domain==99 && local && std::string(local)=="1" : domain==(cfg_.at("drone_id")=="5"?1:2),"DOMAIN_OR_TEST_ISOLATION_MISMATCH");
    require(cfg_.at("command_frame")=="local_xy" && cfg_.at("mission_ai")=="cpp_bt","CPP_LOCAL_XY_REQUIRED");
    unsigned char nonce[16];require(RAND_bytes(nonce,16)==1,"RANDOM_FAILED");for(auto b:nonce){std::ostringstream o;o<<std::hex<<std::setw(2)<<std::setfill('0')<<unsigned(b);session_+=o.str();}
    if(flag(cfg_,"execute")) lock_=std::make_unique<sangwon::WriterLock>(domain);
    const auto record=outer_.at("record_file").get<std::string>();require(!record.empty() && record.front()=='/',"ABSOLUTE_RECORD_FILE_REQUIRED");
    require(!std::ifstream(record).good(),"RECORD_ALREADY_EXISTS");log_.open(record);require(bool(log_),"RECORD_UNAVAILABLE");
    record_fd_=open(record.c_str(),O_WRONLY|O_APPEND|O_CLOEXEC|O_NOFOLLOW);require(record_fd_>=0,"RECORD_SYNC_UNAVAILABLE");
    command_fd_=open((record+".commands").c_str(),O_WRONLY|O_CREAT|O_EXCL|O_CLOEXEC|O_NOFOLLOW,0600);require(command_fd_>=0,"COMMAND_RECORD_UNAVAILABLE");
    std::ifstream self_binary("/proc/self/exe",std::ios::binary);std::string binary_bytes((std::istreambuf_iterator<char>(self_binary)),std::istreambuf_iterator<char>());
    log_<<Json({{"type","runtime_manifest"},{"record_schema","flight-evidence/2"},{"executor_owner","cpp_executor"},{"settings",cfg_},
      {"binary_sha256",digest(binary_bytes)},{"pid",getpid()},{"sample_ros_ns",now().nanoseconds()},{"sample_monotonic_ns",int64_t(steady_s()*1e9)}}).dump()<<'\n';log_.flush();
    fast_=create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    work_=create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    rclcpp::SubscriptionOptions f;f.callback_group=fast_;rclcpp::SubscriptionOptions w;w.callback_group=work_;
    const auto qos=rclcpp::SensorDataQoS();
    timesync_sub_=create_subscription<mavros_msgs::msg::TimesyncStatus>("/mavros/timesync_status",qos,[this](mavros_msgs::msg::TimesyncStatus::ConstSharedPtr m){std::lock_guard<std::mutex> g(fast_mutex_);sample_.remote_ns=m->remote_timestamp_ns;sample_.offset_ns=m->estimated_offset_ns;sample_.rtt_ms=m->round_trip_time_ms;sample_.timesync_received_ns=int64_t(steady_s()*1e9);},f);
    state_sub_=create_subscription<mavros_msgs::msg::State>("/mavros/state",qos,[this](mavros_msgs::msg::State::ConstSharedPtr m){const auto t=source(m->header.stamp);std::lock_guard<std::mutex> g(fast_mutex_);if(t<=sample_.state_at)return;sample_.state_at=t;sample_.source_ns[0]=int64_t(m->header.stamp.sec)*1000000000+m->header.stamp.nanosec;sample_.received_ns[0]=int64_t(steady_s()*1e9);sample_.connected=m->connected;sample_.armed=m->armed;sample_.mode=m->mode;},f);
    landed_sub_=create_subscription<mavros_msgs::msg::ExtendedState>("/mavros/extended_state",qos,[this](mavros_msgs::msg::ExtendedState::ConstSharedPtr m){const auto t=source(m->header.stamp);std::lock_guard<std::mutex> g(fast_mutex_);if(t<=sample_.landed_at)return;sample_.landed_at=t;sample_.source_ns[1]=int64_t(m->header.stamp.sec)*1000000000+m->header.stamp.nanosec;sample_.received_ns[1]=int64_t(steady_s()*1e9);sample_.landed=m->landed_state;},f);
    est_sub_=create_subscription<mavros_msgs::msg::EstimatorStatus>("/mavros/estimator_status",qos,[this](mavros_msgs::msg::EstimatorStatus::ConstSharedPtr m){const auto t=source(m->header.stamp);std::lock_guard<std::mutex> g(fast_mutex_);if(t<=sample_.est_at)return;sample_.est_at=t;sample_.source_ns[2]=int64_t(m->header.stamp.sec)*1000000000+m->header.stamp.nanosec;sample_.received_ns[2]=int64_t(steady_s()*1e9);sample_.estimator=m->attitude_status_flag&&m->velocity_horiz_status_flag&&m->pos_horiz_rel_status_flag&&(!m->const_pos_mode_status_flag||m->pred_pos_horiz_rel_status_flag)&&!m->gps_glitch_status_flag&&!m->accel_error_status_flag;sample_.vertical=m->velocity_vert_status_flag&&m->pos_vert_abs_status_flag;sample_.estimator_flags={m->attitude_status_flag,m->velocity_horiz_status_flag,m->velocity_vert_status_flag,m->pos_horiz_rel_status_flag,m->pos_vert_abs_status_flag,m->const_pos_mode_status_flag,m->pred_pos_horiz_rel_status_flag,m->gps_glitch_status_flag,m->accel_error_status_flag};},f);
    rc_sub_=create_subscription<mavros_msgs::msg::RCIn>("/mavros/rc/in",qos,[this](mavros_msgs::msg::RCIn::ConstSharedPtr m){const auto t=source(m->header.stamp);std::lock_guard<std::mutex> g(fast_mutex_);if(t<=sample_.rc_at)return;sample_.rc_at=t;sample_.source_ns[4]=int64_t(m->header.stamp.sec)*1000000000+m->header.stamp.nanosec;sample_.received_ns[4]=int64_t(steady_s()*1e9);sample_.rc=m->channels;},f);
    battery_sub_=create_subscription<sensor_msgs::msg::BatteryState>("/mavros/battery",qos,[this](sensor_msgs::msg::BatteryState::ConstSharedPtr m){const auto t=source(m->header.stamp);std::lock_guard<std::mutex> g(fast_mutex_);if(t<=sample_.battery_at)return;sample_.battery_at=t;sample_.source_ns[3]=int64_t(m->header.stamp.sec)*1000000000+m->header.stamp.nanosec;sample_.received_ns[3]=int64_t(steady_s()*1e9);sample_.battery=m->percentage;sample_.battery_ok=m->present&&std::isfinite(m->voltage)&&m->voltage>0&&std::isfinite(m->percentage)&&m->percentage>=0&&m->percentage<=1;},f);
    odom_sub_=create_subscription<nav_msgs::msg::Odometry>("/mavros/local_position/odom",qos,[this](nav_msgs::msg::Odometry::ConstSharedPtr m){odom(*m);},f);
    uwb_sub_=create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>("/uwb/btf_pose",qos,[this](geometry_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr m){const auto t=source(m->header.stamp);auto p=m->pose.pose.position;if(m->header.frame_id!="uwb_map"||!std::isfinite(p.x)||!std::isfinite(p.y))return;std::lock_guard<std::mutex> g(fast_mutex_);if(t<=sample_.uwb_at)return;sample_.uwb_at=t;sample_.source_ns[6]=int64_t(m->header.stamp.sec)*1000000000+m->header.stamp.nanosec;sample_.received_ns[6]=int64_t(steady_s()*1e9);sample_.uwb={p.x,p.y};sample_.uwb_ns=int64_t(m->header.stamp.sec)*1000000000+m->header.stamp.nanosec;},f);
    feedback_sub_=create_subscription<mavros_msgs::msg::PositionTarget>("/mavros/setpoint_raw/target_local",qos,[this](mavros_msgs::msg::PositionTarget::ConstSharedPtr m){const auto t=source(m->header.stamp);std::lock_guard<std::mutex> g(fast_mutex_);if(!view_.stream||m->coordinate_frame!=1||(m->type_mask&7)||std::hypot(m->position.x-view_.x,m->position.y-view_.y)>.08||std::abs(m->position.z-view_.z)>.02)return;if(t>sample_.feedback_at){sample_.feedback_at=t;sample_.source_ns[7]=int64_t(m->header.stamp.sec)*1000000000+m->header.stamp.nanosec;sample_.received_ns[7]=int64_t(steady_s()*1e9);}},f);
    input_sub_=create_subscription<std_msgs::msg::String>("/flight/cpp_input",rclcpp::QoS(1),[this](std_msgs::msg::String::ConstSharedPtr m){try{require(m->data.size()<=262144,"OBSERVER_INPUT_TOO_LARGE");auto j=Json::parse(m->data);require(j.at("schema")=="sangwon-cpp-observer/1"&&j.at("executor_session")==session_&&j.at("settings")==cfg_,"OBSERVER_CONTEXT_MISMATCH");double t=number(j.at("sample_time_s"),0,1e12);require(steady_s()-t>=0&&steady_s()-t<=.2,"OBSERVER_TIME_INVALID");auto value=std::make_shared<const Json>(std::move(j));std::lock_guard<std::mutex> g(input_mutex_);require(t>incoming_at_,"BACKWARD_OBSERVER_INPUT");incoming_=std::move(value);incoming_at_=t;incoming_error_.clear();}catch(const std::exception& e){std::lock_guard<std::mutex> g(input_mutex_);incoming_error_=e.what();}},w);
    bridge_sub_=create_subscription<std_msgs::msg::String>(cfg_.at("bridge_topic").get<std::string>(),10,[this](std_msgs::msg::String::ConstSharedPtr m){bridge(*m);},w);
    pub_=create_publisher<mavros_msgs::msg::PositionTarget>("/mavros/setpoint_raw/local",qos);
    status_=create_publisher<std_msgs::msg::String>("/flight_state",rclcpp::SensorDataQoS().keep_last(1));result_=create_publisher<std_msgs::msg::String>("/mission_result",10);
    mode_=create_client<mavros_msgs::srv::SetMode>("/mavros/set_mode",rmw_qos_profile_services_default,fast_);
    arm_=create_client<mavros_msgs::srv::CommandBool>("/mavros/cmd/arming",rmw_qos_profile_services_default,fast_);
    land_=create_client<mavros_msgs::srv::CommandTOL>("/mavros/cmd/land",rmw_qos_profile_services_default,fast_);
    send_timer_=create_wall_timer(std::chrono::milliseconds(50),[this]{send();},fast_);
    status_worker_=std::thread([this]{status_loop();});record_worker_=std::thread([this]{record_loop();});command_worker_=std::thread([this]{command_loop();});
    worker_=std::thread([this]{auto next=std::chrono::steady_clock::now();while(!stop_.load()&&rclcpp::ok()){next+=std::chrono::milliseconds(50);try{tick();}catch(const std::exception& e){if(rclcpp::ok())std::cerr<<"NATIVE_WORKER_STOPPED: "<<e.what()<<'\n';stop_.store(true);break;}std::this_thread::sleep_until(next);if(std::chrono::steady_clock::now()>next+std::chrono::milliseconds(200))next=std::chrono::steady_clock::now();}});
  }
  ~ExecutorNode() override {stop_.store(true);io_cv_.notify_all();if(worker_.joinable())worker_.join();if(status_worker_.joinable())status_worker_.join();if(record_worker_.joinable())record_worker_.join();if(command_worker_.joinable())command_worker_.join();if(record_fd_>=0)close(record_fd_);if(command_fd_>=0)close(command_fd_);}
 private:
  Json outer_,cfg_,input_,bridge_state_=Json::object();FieldExecutor core_;std::string session_,last_input_error_,previous_phase_;
  std::unique_ptr<sangwon::WriterLock> lock_;std::ofstream log_;std::mutex fast_mutex_;Sample sample_;View view_;
  int record_fd_{-1},command_fd_{-1};
  std::atomic<bool> stop_{false};std::thread worker_;std::mutex input_mutex_;
  std::thread status_worker_,record_worker_,command_worker_;std::mutex io_mutex_;std::condition_variable io_cv_;
  std::shared_ptr<const Json> outgoing_;std::deque<Json> events_,commands_;
  std::shared_ptr<const Json> last_evidence_,record_out_;double last_recorded_{-1e9};unsigned sample_sequence_{};std::atomic<bool> record_fault_{false},completion_persisted_{false};
  std::shared_ptr<const Json> incoming_,incoming_bridge_;double incoming_at_{-1e9},incoming_bridge_at_{-1e9};std::string incoming_error_;
  std::deque<std::tuple<unsigned,bool,double>> acks_;std::string fault_;bool pilot_fault_{};double input_at_{-1e9},bridge_at_{-1e9},last_send_{-1},max_send_gap_{};unsigned sent_{};
  rclcpp::CallbackGroup::SharedPtr fast_,work_;rclcpp::TimerBase::SharedPtr send_timer_,work_timer_;
  rclcpp::Publisher<mavros_msgs::msg::PositionTarget>::SharedPtr pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_,result_;
  rclcpp::Subscription<mavros_msgs::msg::State>::SharedPtr state_sub_;
  rclcpp::Subscription<mavros_msgs::msg::ExtendedState>::SharedPtr landed_sub_;
  rclcpp::Subscription<mavros_msgs::msg::EstimatorStatus>::SharedPtr est_sub_;
  rclcpp::Subscription<mavros_msgs::msg::RCIn>::SharedPtr rc_sub_;
  rclcpp::Subscription<mavros_msgs::msg::TimesyncStatus>::SharedPtr timesync_sub_;
  rclcpp::Subscription<sensor_msgs::msg::BatteryState>::SharedPtr battery_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr uwb_sub_;
  rclcpp::Subscription<mavros_msgs::msg::PositionTarget>::SharedPtr feedback_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr input_sub_,bridge_sub_;
  rclcpp::Client<mavros_msgs::srv::SetMode>::SharedPtr mode_;rclcpp::Client<mavros_msgs::srv::CommandBool>::SharedPtr arm_;rclcpp::Client<mavros_msgs::srv::CommandTOL>::SharedPtr land_;
  double source(const builtin_interfaces::msg::Time& t)const {const double a=now().seconds()-(t.sec+1e-9*t.nanosec);return t.sec>0&&a>=0&&a<=3?steady_s()-a:-1e9;}
  std::array<double,2> map(double x,double y,bool vector=false) const {if(!vector){x-=cfg_.at("enu_offset_x_m").get<double>();y-=cfg_.at("enu_offset_y_m").get<double>();}double a=cfg_.at("enu_yaw_deg").get<double>()*M_PI/180;return {std::cos(a)*x+std::sin(a)*y,cfg_.at("map_y_axis_sign").get<int>()*(-std::sin(a)*x+std::cos(a)*y)};}
  void odom(const nav_msgs::msg::Odometry& m) {
    const double t=source(m.header.stamp);const auto p=m.pose.pose.position;const auto q=m.pose.pose.orientation;auto v=m.twist.twist.linear;
    bool good=m.header.frame_id=="map"&&std::isfinite(p.x)&&std::isfinite(p.y)&&std::isfinite(p.z)&&std::isfinite(v.x)&&std::isfinite(v.y)&&std::isfinite(v.z)&&std::isfinite(q.w)&&std::abs(q.w*q.w+q.x*q.x+q.y*q.y+q.z*q.z-1)<.002;
    if(m.child_frame_id=="base_link") {const auto u=v;v.x=(1-2*(q.y*q.y+q.z*q.z))*u.x+2*(q.x*q.y-q.z*q.w)*u.y+2*(q.x*q.z+q.y*q.w)*u.z;v.y=2*(q.x*q.y+q.z*q.w)*u.x+(1-2*(q.x*q.x+q.z*q.z))*u.y+2*(q.y*q.z-q.x*q.w)*u.z;v.z=2*(q.x*q.z-q.y*q.w)*u.x+2*(q.y*q.z+q.x*q.w)*u.y+(1-2*(q.x*q.x+q.y*q.y))*u.z;}
    else if(m.child_frame_id!="map")good=false;
    std::lock_guard<std::mutex> g(fast_mutex_);if(t<=sample_.pose_at)return;sample_.pose_at=t;sample_.source_ns[5]=int64_t(m.header.stamp.sec)*1000000000+m.header.stamp.nanosec;sample_.received_ns[5]=int64_t(steady_s()*1e9);sample_.pose_ok=good;sample_.xy=map(p.x,p.y);sample_.velocity=map(v.x,v.y,true);sample_.z=p.z;sample_.vz=v.z;sample_.yaw=std::remainder(cfg_.at("map_y_axis_sign").get<int>()*(std::atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))*180/M_PI-cfg_.at("enu_yaw_deg").get<double>()),360.);
  }
  void bridge(const std_msgs::msg::String& m) {
    try {
      require(m.data.size()<=65536,"BRIDGE_TOO_LARGE");auto b=Json::parse(m.data);auto c=b.at("settings");const bool local=cfg_.at("bridge_topic")=="/uwb/local_odometry_status";
      bool match=c.at("tag_id")==cfg_.at("drone_id")&&flag(c,"layout_confirmed")&&c.at("source_frame")=="uwb_map"&&c.value("map_y_axis_sign",1)==cfg_.at("map_y_axis_sign");
      for(const auto* k:{"enu_yaw_deg","enu_offset_x_m","enu_offset_y_m","expected_ev_delay_ms"})match=match&&std::abs(number(c.at(k),-1000,1000)-number(cfg_.at(k),-1000,1000))<1e-6;
      for(const auto* a:{"x","y","z"})match=match&&std::abs(number(c.at(std::string("antenna_body_frd_")+a+"_m"),-10,10)-number(cfg_.at(std::string("expected_ev_pos_")+a+"_m"),-10,10))<1e-6;
      const auto stamp=b.at(local?"last_sample_stamp_ns":"last_observation_stamp_ns").get<int64_t>();
      const int count=local?b.at("counts").at("published").get<int>():b.at("published").get<int>();
      const bool clock=!local || (!cfg_.at("bridge_host_boot_id").get<std::string>().empty()&&c.at("host_boot_id")==cfg_.at("bridge_host_boot_id")&&flag(b,"timesync_ready")&&age(b,"last_send_age_s",cfg_.value("bridge_observation_timeout_s",.35)));
      if(!local)match=match&&c.at("input_source")=="btf_xy"&&!flag(c,"ground_only");
      const bool boot_match=!local||c.at("host_boot_id")==cfg_.at("bridge_host_boot_id");
      auto value=std::make_shared<const Json>(Json{{"bridge_ready",clock&&b.at("gate")=="ready"},{"alignment_matches",match},{"bridge_published",count},{"bridge_stamp_ns",stamp},
        {"bridge_gate",b.at("gate")},{"bridge_send_age_s",b.value("last_send_age_s",Json())},{"bridge_timesync_ready",b.value("timesync_ready",Json())},
        {"bridge_parameters",b.value("parameters",Json::object())},{"bridge_contract_error",!boot_match?"bridge_host_boot_changed":!match?"observation_and_command_alignment_mismatch":""},
        {"bridge_report_stamp_ns",b.value("status_stamp_ns",Json())},{"bridge_report_monotonic_ns",b.value("status_monotonic_ns",Json())},{"bridge_last_send_monotonic_ns",b.value("last_send_monotonic_ns",Json())},
        {"bridge_position_ned_xy_m",b.value("last_position_ned_xy_m",Json())},{"bridge_odometry_time_usec",b.value("last_odometry_time_usec",Json())}});
      std::lock_guard<std::mutex> g(input_mutex_);incoming_bridge_=std::move(value);incoming_bridge_at_=steady_s();
    }catch(const std::exception&){auto value=std::make_shared<const Json>(Json{{"bridge_ready",false},{"alignment_matches",false},{"bridge_gate","malformed_report"},{"bridge_contract_error",""}});std::lock_guard<std::mutex> g(input_mutex_);incoming_bridge_=std::move(value);incoming_bridge_at_=steady_s();}
  }
  Json snapshot(double t) {
    Sample s;View view;{std::lock_guard<std::mutex> g(fast_mutex_);s=sample_;view=view_;}
    Json out=input_.is_object()&&input_.contains("state")?input_.at("state"):Json::object();
    // Metadata age is added, never reset to arrival or executor time.
    const double elapsed=t-input_at_;for(auto& kv:out.items())if(kv.key().size()>=6&&kv.key().substr(kv.key().size()-6)=="_age_s"&&kv.value().is_number())kv.value()=kv.value().get<double>()+elapsed;
    if(out.contains("lidar_scan")&&out.at("lidar_scan").is_object()) {auto& scan=out["lidar_scan"];for(const auto* key:{"age_s","first_beam_age_s","attitude_age_s","anchor_uwb_age_s"})if(scan.contains(key)&&scan[key].is_number())scan[key]=scan[key].get<double>()+elapsed;}
    out.update({{"connected",s.connected},{"armed",s.armed},{"landed",s.landed},{"mode",s.mode},{"xy",s.pose_ok?Json(s.xy):Json::array()}, {"velocity_xy",Json(s.velocity)}, {"altitude_enu_m",s.z},{"vertical_speed_m_s",s.vz},{"yaw_deg",s.yaw},{"estimator_valid",s.estimator},{"vertical_estimator_valid",s.vertical},
      {"state_age_s",t-s.state_at},{"landed_age_s",t-s.landed_at},{"pose_age_s",t-s.pose_at},{"estimator_age_s",t-s.est_at},{"battery",s.battery_ok?Json(s.battery):Json()}, {"battery_age_s",t-s.battery_at},
      {"uwb_xy",Json(s.uwb)},{"uwb_stamp_ns",s.uwb_ns},{"uwb_age_s",t-s.uwb_at},{"feedback_age_s",t-s.feedback_at},{"rc_channels",s.rc},{"rc_age_s",t-s.rc_at},
      {"source_stamp_ns",Json(s.source_ns)},{"received_monotonic_ns",Json(s.received_ns)},{"source_order",Json::array({"state","landed","estimator","battery","rc","pose","uwb","target_feedback"})},{"local_setpoint_ready",pub_->get_subscription_count()==1},{"command_services_ready",mode_->service_is_ready()&&arm_->service_is_ready()&&land_->service_is_ready()}});
    bool rc=s.rc.size()>=4;for(size_t i=0;i<std::min<size_t>(4,s.rc.size());++i)rc=rc&&s.rc[i]>=800&&s.rc[i]<=2200;out["rc_valid"]=rc;
    bool changed=false;for(int ch:view.switches)if(ch>0&&size_t(ch)<=s.rc.size()&&size_t(ch)<=view.baseline.size()&&std::abs(int(s.rc[ch-1])-int(view.baseline[ch-1]))>=100)changed=true;
    out["rc_switch_changed"]=t-s.rc_at<=1&&changed;
    out["estimator_flags"]=Json(s.estimator_flags);out["estimator_flag_order"]=Json::array({"attitude","velocity_horizontal","velocity_vertical","position_horizontal_relative","position_vertical_absolute","constant_position","predicted_position_horizontal_relative","gps_glitch","accel_error"});
    out["timesync"]=Json{{"remote_timestamp_ns",s.remote_ns},{"estimated_offset_ns",s.offset_ns},{"round_trip_time_ms",s.rtt_ms},{"received_monotonic_ns",s.timesync_received_ns}};
    if(!bridge_state_.empty()){out.update(bridge_state_);out["bridge_age_s"]=t-bridge_at_;if(bridge_state_.contains("bridge_stamp_ns"))out["bridge_observation_age_s"]=now().seconds()-bridge_state_.at("bridge_stamp_ns").get<double>()/1e9;}
    return out;
  }
  void tick() {
    std::shared_ptr<const Json> input,bridge;double at,bridge_at;std::string input_error;
    {std::lock_guard<std::mutex> g(input_mutex_);input=incoming_;at=incoming_at_;bridge=incoming_bridge_;bridge_at=incoming_bridge_at_;input_error=incoming_error_;}
    if(input && at>input_at_){input_=*input;input_at_=at;}
    if(bridge && bridge_at>bridge_at_){bridge_state_=*bridge;bridge_at_=bridge_at;}
    last_input_error_=std::move(input_error);
    const double t=steady_s();Json s=snapshot(t);std::deque<std::tuple<unsigned,bool,double>> acks;std::string fault;bool pilot;
    {std::lock_guard<std::mutex> g(fast_mutex_);acks.swap(acks_);fault.swap(fault_);pilot=pilot_fault_;pilot_fault_=false;}
    for(auto [token,accepted,at]:acks){core_.command_result(token,accepted,at);std::lock_guard<std::mutex> g(io_mutex_);events_.push_back(Json{{"type","command_ack"},{"token",token},{"accepted",accepted},{"sample_monotonic_ns",int64_t(at*1e9)},{"executor_owner","cpp_executor"}});}
    try {
      Json request=input_.is_object()?input_.value("request",Json()):Json();
      if(!request.is_null()&&request.at("executor_session")!=session_)request=Json();
      // A consumed request may remain in metadata; only START is idempotent.
      if(!request.is_null()&&request.value("action",std::string())!="start"&&request.at("request_id")==last_control_)request=Json();
      if(!request.is_null()&&request.value("action",std::string())!="start")last_control_=request.at("request_id").get<std::string>();
      if(!fault.empty())core_.interrupt(fault,pilot,t,s);
      else core_.tick(s,t,request,wall_s());
    }catch(const std::exception& e){core_.interrupt(std::string("native_input_invalid:")+e.what(),false,t,s);}
    View v;v.stream=core_.streaming()&&flag(cfg_,"execute");v.pending=core_.pending();v.token=core_.pending_token();v.phase=core_.phase;v.expires=t+.15;
    double angle=cfg_.at("enu_yaw_deg").get<double>()*M_PI/180;auto xy=core_.target();const double y=cfg_.at("map_y_axis_sign").get<int>()*xy.y;
    v.x=std::cos(angle)*xy.x-std::sin(angle)*y+cfg_.at("enu_offset_x_m").get<double>();v.y=std::sin(angle)*xy.x+std::cos(angle)*y+cfg_.at("enu_offset_y_m").get<double>();v.z=core_.height();v.yaw=(cfg_.at("enu_yaw_deg").get<double>()+cfg_.at("map_y_axis_sign").get<int>()*core_.yaw())*M_PI/180;
    double gap;unsigned sent;
    {std::lock_guard<std::mutex> g(fast_mutex_);if(view_.baseline.empty()&&v.stream){v.baseline=sample_.rc;if(s.contains("rc_switch_channels"))v.switches=s.at("rc_switch_channels").get<std::vector<int>>();}else{v.baseline=view_.baseline;v.switches=view_.switches;}view_=std::move(v);gap=max_send_gap_;sent=sent_;}
    auto out=core_.status();out.update({{"executor_session",session_},{"test_mode",flag(outer_,"test_mode")},{"physical_flight",!flag(outer_,"test_mode")&&flag(cfg_,"execute")},{"output_enabled",flag(cfg_,"execute")},{"input_error",last_input_error_},{"input_age_s",t-input_at_},{"max_sender_gap_s",gap},{"setpoints_sent",sent},{"fc_connected",s.value("connected",false)},{"fc_armed",s.value("armed",false)},{"fc_mode",s.value("mode",std::string())},{"fc_landed",s.value("landed",0)}});
    out["px4_map_xy_m"]=s.at("xy");
    out["bridge_observation_age_s"]=s.value("bridge_observation_age_s",Json());
    out["raw_uwb_age_s"]=s.value("uwb_age_s",Json());
    out["target_applied"]=age(s,"feedback_age_s",.5);
    out["observer_metadata_delayed"]=t-input_at_>2.5;
    out["web_report_delayed"]=!age(s,"web_age_s",cfg_.value("web_timeout_s",2.));
    out["observation_status"]=observation_status(s,cfg_.value("bridge_observation_timeout_s",.35));
    out["record_schema"]="flight-evidence/2";
    out["sample_sequence"]=++sample_sequence_;
    out["executor_owner"]="cpp_executor";
    out["sample_monotonic_ns"]=int64_t(t*1e9);out["sample_ros_ns"]=now().nanoseconds();out["sample_wall_ns"]=int64_t(wall_s()*1e9);
    out["input_snapshot"]=s;
    out["land_request_owner"]=core_.phase=="LANDING"?"cpp_executor":"";
    out["sender_target_enu"]=Json{{"x",std::cos(angle)*xy.x-std::sin(angle)*y+cfg_.at("enu_offset_x_m").get<double>()},
      {"y",std::sin(angle)*xy.x+std::cos(angle)*y+cfg_.at("enu_offset_y_m").get<double>()},{"z",core_.height()},{"yaw_rad",(cfg_.at("enu_yaw_deg").get<double>()+cfg_.at("map_y_axis_sign").get<int>()*core_.yaw())*M_PI/180}};
    out["preflight"]=input_.is_object()?input_.value("preflight",Json::object()):Json::object();
    if(t-input_at_>2.5)out["preflight"]["checked_inputs_passed"]=false;
    auto value=std::make_shared<const Json>(std::move(out));
    {std::lock_guard<std::mutex> g(io_mutex_);outgoing_=value;record_out_=value;last_evidence_=value;for(const auto& a:core_.actions)commands_.push_back(a);if(previous_phase_!=core_.phase){auto transition=*value;transition["type"]="transition";events_.push_back(std::move(transition));previous_phase_=core_.phase;}}
    io_cv_.notify_all();
  }
  void io_fault(const std::string& reason) {std::lock_guard<std::mutex> g(fast_mutex_);if(fault_.empty())fault_=reason;}
  void status_loop() {
    while(!stop_.load()&&rclcpp::ok()){
      std::shared_ptr<const Json> out;
      {std::unique_lock<std::mutex> g(io_mutex_);io_cv_.wait(g,[this]{return stop_.load()||outgoing_;});if(stop_.load())break;out.swap(outgoing_);}
      try{
        auto value=*out;const auto state=value.at("state").get<std::string>();
        const bool terminal=state=="END"||state=="FAILED"||state=="UNCONFIRMED"||state=="PILOT_OVERRIDE";
        value["recording_ok"]=!record_fault_;
        if(record_fault_){value["mission_complete"]=false;value["work_outcome"]="UNVERIFIED";if(terminal){value["state"]="FAILED";value["phase"]="FAILED";value["reason"]="native_completion_record_failed";}}
        else if(terminal&&!completion_persisted_){value["mission_complete"]=false;value["record_pending"]=true;
          if(state=="END"){value["state"]="LANDING";value["phase"]="LANDING";value["reason"]="completion_record_pending";}}
        std_msgs::msg::String msg;msg.data=value.dump();status_->publish(msg);
      }catch(const std::exception& e){if(rclcpp::ok())io_fault(std::string("native_status_transport_failed:")+e.what());}
    }
  }
  void record_loop() {
    while(!stop_.load()&&rclcpp::ok()) {
      std::shared_ptr<const Json> out;std::deque<Json> events;
      {std::unique_lock<std::mutex> g(io_mutex_);io_cv_.wait(g,[this]{return stop_.load()||record_out_||!events_.empty();});if(stop_.load())break;out.swap(record_out_);events.swap(events_);}
      try {
        // Heartbeats are latest-value telemetry. Terminal results retain the
        // reliable topic, outside both flight execution and command delivery.
        for(const auto& e:events){const auto raw=e.dump();log_<<raw<<'\n';log_.flush();require(bool(log_),"native_event_record_failed");
          const auto state=e.value("state",std::string());
          if(state=="END"||state=="FAILED"||state=="UNCONFIRMED"||state=="PILOT_OVERRIDE"){
            require(fsync(record_fd_)==0,"native_completion_record_sync_failed");
            completion_persisted_=true;
            std_msgs::msg::String msg;msg.data=raw;result_->publish(msg);
          }
        }
        if(out){
          const double sample=out->at("sample_monotonic_ns").get<double>()/1e9;
          if(sample-last_recorded_>=.1){auto row=*out;row["type"]="flight_sample";row["written_monotonic_ns"]=int64_t(steady_s()*1e9);log_<<row.dump()<<'\n';log_.flush();require(bool(log_),"native_sample_record_failed");last_recorded_=sample;}
        }
      }catch(const std::exception& e){record_fault_=true;if(rclcpp::ok())io_fault(std::string("native_record_or_status_failed:")+e.what());}
    }
  }
  void command_loop() {
    while(!stop_.load()&&rclcpp::ok()) {
      Json action;
      {std::unique_lock<std::mutex> g(io_mutex_);io_cv_.wait(g,[this]{return stop_.load()||!commands_.empty();});if(stop_.load())break;action=std::move(commands_.front());commands_.pop_front();}
      try {
        std::shared_ptr<const Json> evidence;{std::lock_guard<std::mutex> g(io_mutex_);evidence=last_evidence_;}
        auto raw=Json({{"type","command_before_dispatch"},{"action",action},{"steady_s",steady_s()},{"executor_owner","cpp_executor"},{"trigger",evidence?*evidence:Json()}}).dump()+"\n";
        size_t offset=0;while(offset<raw.size()){auto n=write(command_fd_,raw.data()+offset,raw.size()-offset);require(n>0,"COMMAND_RECORD_WRITE_FAILED");offset+=size_t(n);}
        require(fsync(command_fd_)==0,"COMMAND_RECORD_SYNC_FAILED");
        // A slow fsync cannot dispatch a command from an expired/cancelled
        // execution. The core keeps updating while this worker waits.
        View v;{std::lock_guard<std::mutex> g(fast_mutex_);v=view_;}
        if(!v.pending||v.token!=action.at("token").get<unsigned>()||steady_s()>v.expires)continue;
        dispatch(action);
      }catch(const std::exception& e){if(rclcpp::ok())io_fault(std::string("native_command_transport_failed:")+e.what());}
    }
  }
  std::string last_control_;
  void send() {
    const double t=steady_s();View v;Sample s;{std::lock_guard<std::mutex> g(fast_mutex_);v=view_;s=sample_;}
    if(!v.stream)return;
    const bool mode=v.phase=="PREPARING"?(s.mode=="POSCTL"||s.mode=="AUTO.LOITER"||s.mode=="OFFBOARD"):s.mode=="OFFBOARD";
    bool rc=false;for(int ch:v.switches)if(ch>0&&size_t(ch)<=s.rc.size()&&size_t(ch)<=v.baseline.size()&&std::abs(int(s.rc[ch-1])-int(v.baseline[ch-1]))>=100)rc=true;
    const bool pilot=(s.connected&&t-s.state_at<=cfg_.value("state_timeout_s",2.5)&&!mode)||(t-s.rc_at<=1&&rc);
    // A stale planner cannot advance the target. This immutable View remains
    // an approved position hold while direct FC health and pilot authority
    // remain valid; output cadence is independent of planner/report cadence.
    const bool healthy=s.connected&&t-s.state_at<=2.5&&t-s.pose_at<=.2&&s.pose_ok&&s.estimator&&s.vertical;
    if(pilot||!healthy){
      const std::string why=pilot?"pilot_priority_native_sender":!s.connected||t-s.state_at>2.5?"native_sender_fc_state_expired":!s.pose_ok||t-s.pose_at>.2?"native_sender_fc_pose_expired":"native_sender_fc_estimator_expired";
      std::lock_guard<std::mutex> g(fast_mutex_);if(pilot||fault_.empty())fault_=why;pilot_fault_=pilot_fault_||pilot;return;
    }
    mavros_msgs::msg::PositionTarget m;m.header.stamp=now();m.header.frame_id="map";m.coordinate_frame=1;m.type_mask=2552;m.position.x=v.x;m.position.y=v.y;m.position.z=v.z;m.yaw=v.yaw;pub_->publish(m);
    std::lock_guard<std::mutex> g(fast_mutex_);if(last_send_>=0)max_send_gap_=std::max(max_send_gap_,t-last_send_);last_send_=t;++sent_;
  }
  void ack(unsigned token,bool accepted){std::lock_guard<std::mutex> g(fast_mutex_);acks_.emplace_back(token,accepted,steady_s());}
  void dispatch(const Json& action) {
    require(flag(cfg_,"execute"),"EXECUTION_DISABLED");const auto kind=action.at("kind").get<std::string>();const auto token=action.at("token").get<unsigned>();
    // Recheck live authority after durable recording and immediately before
    // handing a command to MAVROS; callbacks may run during fsync/BT work.
    Sample live;View view;{std::lock_guard<std::mutex> g(fast_mutex_);live=sample_;view=view_;}
    const bool mode_ok=kind=="arm"?live.mode=="OFFBOARD":kind=="land"?live.mode=="OFFBOARD"||live.mode=="AUTO.LAND":live.mode=="POSCTL"||live.mode=="AUTO.LOITER"||live.mode=="OFFBOARD";
    bool rc_changed=false;for(int ch:view.switches)if(ch>0&&size_t(ch)<=live.rc.size()&&size_t(ch)<=view.baseline.size()&&std::abs(int(live.rc[ch-1])-int(view.baseline[ch-1]))>=100)rc_changed=true;
    if(!mode_ok||rc_changed){std::lock_guard<std::mutex> g(fast_mutex_);fault_="pilot_priority_before_native_command";pilot_fault_=true;acks_.emplace_back(token,false,steady_s());return;}
    if(!live.connected||steady_s()-live.state_at>cfg_.value("state_timeout_s",2.5)){ack(token,false);return;}
    if(kind=="offboard_mode"||kind=="position_mode") {if(!mode_->service_is_ready()){ack(token,false);return;}auto req=std::make_shared<mavros_msgs::srv::SetMode::Request>();req->custom_mode=kind=="offboard_mode"?"OFFBOARD":"POSCTL";mode_->async_send_request(req,[this,token](rclcpp::Client<mavros_msgs::srv::SetMode>::SharedFuture f){try{ack(token,f.get()->mode_sent);}catch(...){/* Unknown effect: command timeout latches UNCONFIRMED. */}});}
    else if(kind=="arm") {if(!arm_->service_is_ready()){ack(token,false);return;}auto req=std::make_shared<mavros_msgs::srv::CommandBool::Request>();req->value=true;arm_->async_send_request(req,[this,token](rclcpp::Client<mavros_msgs::srv::CommandBool>::SharedFuture f){try{auto r=f.get();ack(token,r->success&&r->result==0);}catch(...){}});}
    else if(kind=="land") {if(!land_->service_is_ready()){ack(token,false);return;}auto req=std::make_shared<mavros_msgs::srv::CommandTOL::Request>();req->yaw=req->latitude=req->longitude=req->altitude=std::numeric_limits<double>::quiet_NaN();land_->async_send_request(req,[this,token](rclcpp::Client<mavros_msgs::srv::CommandTOL>::SharedFuture f){try{auto r=f.get();ack(token,r->success&&r->result==0);}catch(...){}});}
    else require(false,"UNKNOWN_NATIVE_ACTION");
  }
};
int main(int argc,char** argv) {
  try {require(argc>=2,"CONFIG_FILE_REQUIRED");std::ifstream file(argv[1]);require(bool(file),"CONFIG_UNAVAILABLE");Json cfg;file>>cfg;rclcpp::init(argc,argv);auto node=std::make_shared<ExecutorNode>(cfg);rclcpp::executors::MultiThreadedExecutor exec(rclcpp::ExecutorOptions(),3);exec.add_node(node);exec.spin();rclcpp::shutdown();}
  catch(const std::exception& e){std::cerr<<e.what()<<'\n';return 2;}
}
