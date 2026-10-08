// Explicit START only. No setpoint publisher or parameter writer.
#include "sangwon_ai/native_hover.hpp"
#include "sangwon_ai/writer_lock.hpp"
#include "sangwon_ai/service/json.hpp"
#include <rclcpp/rclcpp.hpp>
#include <mavros_msgs/msg/state.hpp>
#include <mavros_msgs/msg/extended_state.hpp>
#include <mavros_msgs/msg/estimator_status.hpp>
#include <mavros_msgs/msg/rc_in.hpp>
#include <mavros_msgs/msg/status_text.hpp>
#include <mavros_msgs/srv/set_mode.hpp>
#include <mavros_msgs/srv/command_bool.hpp>
#include <mavros_msgs/srv/param_pull.hpp>
#include <rcl_interfaces/srv/get_parameters.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sys/file.h>
#include <sys/socket.h>
#include <sys/stat.h>
#include <sys/un.h>
#include <fcntl.h>
#include <unistd.h>
#include <openssl/rand.h>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <chrono>
#include <cstring>
#include <array>

using sangwon::service::Json;
using sangwon::service::require;
using sangwon::bench::Action;
namespace fs=std::filesystem;
double steady() { return std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count(); }
std::string random_id() {
  unsigned char b[16]; require(RAND_bytes(b,16)==1,"RANDOM_FAILED");
  std::ostringstream s; for(auto c:b) s<<std::hex<<std::setw(2)<<std::setfill('0')<<unsigned(c); return s.str();
}
class HoverNode : public rclcpp::Node {
 public:
  explicit HoverNode(const Json& c):Node("sangwon_native_hover"), config_(c) {
    profile_=c.at("profile").get<std::string>();
    require(profile_=="SITL" || profile_=="FLIGHT","INVALID_PROFILE");
    const char* domain=std::getenv("ROS_DOMAIN_ID");
    require(domain && std::stoi(domain)==c.at("ros_domain_id").get<int>(),"DOMAIN_MISMATCH");
    require(profile_!="SITL" || std::stoi(domain)==173,"SITL_REQUIRES_DOMAIN_173");
    require(profile_!="FLIGHT" || std::stoi(domain)==2,"TAG_B_REQUIRES_DOMAIN_2");
    controller_.enabled=c.at("output_enabled").get<bool>();
    require(!controller_.enabled || profile_=="SITL" || c.at("rc_auto_mode_handoff_verified").get<bool>(),"RC_HANDOFF_NOT_VERIFIED");
    if(controller_.enabled) writer_lock_=std::make_unique<sangwon::WriterLock>(std::stoi(domain));
    controller_.height=c.at("takeoff_height_m").get<double>();
    require(std::isfinite(controller_.height) && controller_.height>=0.3 && controller_.height<=2.0,"INVALID_HEIGHT");
    state_dir_=c.at("state_dir").get<std::string>();
    require(state_dir_.is_absolute(),"STATE_DIR_ABSOLUTE_REQUIRED");
    fs::create_directories(state_dir_); fs::permissions(state_dir_,fs::perms::owner_all);
    lock_=open((state_dir_/"writer.lock").c_str(),O_CREAT|O_RDWR|O_CLOEXEC,0600);
    require(lock_>=0 && flock(lock_,LOCK_EX|LOCK_NB)==0,"WRITER_ALREADY_RUNNING");
    socket_path_=(state_dir_/"hover.sock").string();
    sockaddr_un addr{}; addr.sun_family=AF_UNIX;
    require(socket_path_.size()<sizeof(addr.sun_path),"IPC_PATH_TOO_LONG");
    std::strcpy(addr.sun_path,socket_path_.c_str());
    if(fs::exists(socket_path_)) { require(fs::is_socket(socket_path_),"IPC_PATH_CONFLICT"); fs::remove(socket_path_); }
    server_=socket(AF_UNIX,SOCK_SEQPACKET|SOCK_NONBLOCK|SOCK_CLOEXEC,0);
    require(server_>=0 && bind(server_,reinterpret_cast<sockaddr*>(&addr),sizeof(addr))==0 && listen(server_,8)==0,"IPC_BIND_FAILED");
    log_.open(state_dir_/"events.jsonl",std::ios::app); require(bool(log_),"LOG_UNAVAILABLE");
    session_=random_id(); event({{"event","boot"},{"session",session_},{"profile",profile_},{"output_enabled",controller_.enabled}});
    const auto q=rclcpp::SensorDataQoS();
    state_sub_=create_subscription<mavros_msgs::msg::State>("/mavros/state",q,[this](mavros_msgs::msg::State::ConstSharedPtr m){
      sample_.connected=m->connected; sample_.armed=m->armed; sample_.mode=m->mode; state_at_=stamp(m->header.stamp,2.5); });
    extended_sub_=create_subscription<mavros_msgs::msg::ExtendedState>("/mavros/extended_state",q,[this](mavros_msgs::msg::ExtendedState::ConstSharedPtr m){
      sample_.landed=m->landed_state==1; landed_at_=stamp(m->header.stamp,1.5); });
    odom_sub_=create_subscription<nav_msgs::msg::Odometry>("/mavros/local_position/odom",q,[this](nav_msgs::msg::Odometry::ConstSharedPtr m){
      auto p=m->pose.pose.position; auto v=m->twist.twist.linear;
      sample_.x=p.x; sample_.y=p.y; sample_.z=p.z; sample_.speed=std::sqrt(v.x*v.x+v.y*v.y+v.z*v.z);
      pose_at_=stamp(m->header.stamp,.3);
      if(!std::isfinite(p.x)||!std::isfinite(p.y)||!std::isfinite(p.z)||!std::isfinite(sample_.speed)) pose_at_=-1e9;
    });
    estimator_sub_=create_subscription<mavros_msgs::msg::EstimatorStatus>("/mavros/estimator_status",q,[this](mavros_msgs::msg::EstimatorStatus::ConstSharedPtr m){
      estimator_at_=stamp(m->header.stamp,2.5); const_pos_=m->const_pos_mode_status_flag;
      estimator_good_=m->attitude_status_flag && m->velocity_horiz_status_flag && m->velocity_vert_status_flag &&
        (m->pos_horiz_abs_status_flag || m->pos_horiz_rel_status_flag) && m->pos_vert_abs_status_flag &&
        // PX4 v1.17 sets CONST_POS at rest even with healthy GNSS aiding.
        // PRED_POS identifies active horizontal aiding; finite local XYZ alone does not.
        (m->pred_pos_horiz_rel_status_flag || m->pred_pos_horiz_abs_status_flag) && !m->accel_error_status_flag && !m->gps_glitch_status_flag;
    });
    rc_sub_=create_subscription<mavros_msgs::msg::RCIn>("/mavros/rc/in",q,[this](mavros_msgs::msg::RCIn::ConstSharedPtr m){
      rc_at_=stamp(m->header.stamp,.5); rc_good_=m->channels.size()>=8 && m->rssi!=0;
      for(size_t i=0;i<8 && i<m->channels.size();++i) { rc_[i]=m->channels[i]; rc_good_=rc_good_ && rc_[i]>=800 && rc_[i]<=2200; }
    });
    text_sub_=create_subscription<mavros_msgs::msg::StatusText>("/mavros/statustext/recv",q,[this](mavros_msgs::msg::StatusText::ConstSharedPtr m){ last_text_=m->text; event({{"event","px4_text"},{"severity",m->severity},{"text",m->text}}); });
    mode_=create_client<mavros_msgs::srv::SetMode>("/mavros/set_mode");
    arm_=create_client<mavros_msgs::srv::CommandBool>("/mavros/cmd/arming");
    params_=create_client<rcl_interfaces::srv::GetParameters>("/mavros/param/get_parameters");
    pull_=create_client<mavros_msgs::srv::ParamPull>("/mavros/param/pull");
    timer_=create_wall_timer(std::chrono::milliseconds(50),[this]{ tick(); });
  }
  ~HoverNode() override { for(auto& p:peers_) close(p.first); if(server_>=0)close(server_); if(!socket_path_.empty())unlink(socket_path_.c_str()); if(lock_>=0)close(lock_); }
 private:
  std::unique_ptr<sangwon::WriterLock> writer_lock_;
  Json config_; std::string profile_,session_,socket_path_,last_text_,last_phase_,last_reply_; fs::path state_dir_;
  int server_{-1},lock_{-1}; std::vector<std::pair<int,double>> peers_; std::ofstream log_;
  sangwon::bench::NativeHover controller_; sangwon::bench::Sample sample_;
  double state_at_{-1e9},landed_at_{-1e9},pose_at_{-1e9},estimator_at_{-1e9},rc_at_{-1e9},param_at_{-1e9},param_try_{-1e9},command_at_{-1e9},status_at_{};
  bool estimator_good_{},rc_good_{},const_pos_{},pulled_{},pull_pending_{},param_pending_{},command_pending_{},param_good_{};
  std::array<int,8> rc_{},baseline_{}; unsigned request_id_{};
  rclcpp::Subscription<mavros_msgs::msg::State>::SharedPtr state_sub_;
  rclcpp::Subscription<mavros_msgs::msg::ExtendedState>::SharedPtr extended_sub_;
  rclcpp::Subscription<mavros_msgs::msg::EstimatorStatus>::SharedPtr estimator_sub_;
  rclcpp::Subscription<mavros_msgs::msg::RCIn>::SharedPtr rc_sub_;
  rclcpp::Subscription<mavros_msgs::msg::StatusText>::SharedPtr text_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Client<mavros_msgs::srv::SetMode>::SharedPtr mode_;
  rclcpp::Client<mavros_msgs::srv::CommandBool>::SharedPtr arm_;
  rclcpp::Client<mavros_msgs::srv::ParamPull>::SharedPtr pull_;
  rclcpp::Client<rcl_interfaces::srv::GetParameters>::SharedPtr params_;
  rclcpp::TimerBase::SharedPtr timer_;
  double stamp(const builtin_interfaces::msg::Time& t,double limit) {
    double age=now().seconds()-(double(t.sec)+1e-9*t.nanosec);
    return t.sec>0 && age>=-.05 && age<=limit ? steady():-1e9;
  }
  void event(Json e) { e["steady_s"]=steady(); log_<<e.dump()<<'\n'; log_.flush(); if(!log_)controller_.rejected("LOG_WRITE_FAILED"); }
  void refresh(double t) {
    sample_.fresh=t-state_at_<2.5 && t-landed_at_<1.5 && t-pose_at_<.3;
    sample_.estimator_ok=estimator_good_ && t-estimator_at_<2.5;
    sample_.rc_ok=rc_good_ && t-rc_at_<.5;
    sample_.rc_stick=false; sample_.rc_switch=false;
    if(controller_.used) {
      for(size_t i=0;i<4;++i) sample_.rc_stick=sample_.rc_stick || std::abs(rc_[i]-baseline_[i])>300;
      for(auto i:{4,6,7}) sample_.rc_switch=sample_.rc_switch || std::abs(rc_[i]-baseline_[i])>200;
    }
  }
  std::string readiness() const {
    auto r=controller_.readiness(sample_); if(!r.empty())return r;
    if(!param_good_ || steady()-param_at_>3) return "PX4_PARAMETERS_NOT_CONFIRMED";
    if(!mode_->service_is_ready() || !arm_->service_is_ready())return "MAVROS_COMMAND_UNAVAILABLE";
    // A held extreme stick must not become the takeover baseline.
    for(size_t i=0;i<4;++i) if(std::abs(rc_[i]-1500)>100)return "CENTER_RC_STICKS";
    return {};
  }
  Json status() const { return {{"session",session_},{"profile",profile_},{"output_enabled",controller_.enabled},
    {"phase",controller_.phase},{"reason",controller_.reason},{"start_blocker",readiness()},
    {"takeoff_height_m",controller_.height},{"hover_seconds",controller_.hover_s},{"mode",sample_.mode},
    {"armed",sample_.armed},{"landed",sample_.landed},{"telemetry_fresh",sample_.fresh},{"px4_estimator_ok",sample_.estimator_ok},
    {"const_pos_mode",const_pos_},{"rc_ok",sample_.rc_ok},{"xyz_px4_enu_m",{sample_.x,sample_.y,sample_.z}},
    {"parameter_readback_ok",param_good_},{"last_reply",last_reply_},{"px4_text",last_text_},{"uwb_fusion_verified",false}}; }
  Json handle(const Json& q) {
    const auto method=q.at("method").get<std::string>();
    if(method=="status")return status();
    require(q.at("session")==session_,"STALE_SESSION");
    if(method=="start") {
      require(q.at("confirm") == "TAKEOFF_HOVER_2S_LAND","EXPLICIT_START_REQUIRED");
      auto why=readiness(); require(why.empty(),why);
      baseline_=rc_; require(controller_.start(sample_,steady()),controller_.reason);
      event({{"event","human_start"},{"session",session_}}); return status();
    }
    if(method=="land") { require(controller_.active(),"NO_ACTIVE_RUN"); controller_.land(steady()); event({{"event","operator_land"}}); return status(); }
    throw std::invalid_argument("UNKNOWN_METHOD");
  }
  void ipc(double t) {
    // Packet IPC never blocks the 20Hz sequence loop, including stalled clients.
    if(peers_.size()<8) {
      int fd=accept4(server_,nullptr,nullptr,SOCK_NONBLOCK|SOCK_CLOEXEC);
      if(fd>=0) { ucred cred{}; socklen_t n=sizeof(cred); if(getsockopt(fd,SOL_SOCKET,SO_PEERCRED,&cred,&n)==0 && cred.uid==geteuid())peers_.push_back({fd,t}); else close(fd); }
    }
    for(auto p=peers_.begin();p!=peers_.end();) {
      char buf[4097]; auto n=recv(p->first,buf,sizeof(buf),MSG_DONTWAIT|MSG_TRUNC);
      if(n<0 && t-p->second<1) { ++p; continue; }
      if(n>0) { Json reply; try { require(n<=4096,"MESSAGE_TOO_LARGE"); reply={{"ok",true},{"status",handle(sangwon::service::parse(std::string(buf,size_t(n))))}}; }
        catch(const std::exception& e) { reply={{"ok",false},{"error",e.what()},{"status",status()}}; }
        auto out=reply.dump(); send(p->first,out.data(),out.size(),MSG_DONTWAIT|MSG_NOSIGNAL);
      }
      close(p->first); p=peers_.erase(p);
    }
  }
  void parameters(double t) {
    if(!pulled_) {
      if(!pull_pending_ && pull_->service_is_ready()) {
        auto p=std::make_shared<mavros_msgs::srv::ParamPull::Request>(); p->force_pull=true; pull_pending_=true;
        pull_->async_send_request(p,[this](rclcpp::Client<mavros_msgs::srv::ParamPull>::SharedFuture f){ pulled_=f.get()->success; pull_pending_=false; });
      }
      return;
    }
    if(param_pending_ || t-param_try_<1 || !params_->service_is_ready())return;
    param_try_=t; param_pending_=true;
    auto p=std::make_shared<rcl_interfaces::srv::GetParameters::Request>();
    p->names={"MIS_TAKEOFF_ALT","COM_TAKEOFF_ACT","RC_MAP_ROLL","RC_MAP_PITCH","RC_MAP_THROTTLE","RC_MAP_YAW","RC_MAP_FLTMODE","RC_MAP_KILL_SW","RC_MAP_ARM_SW"};
    params_->async_send_request(p,[this](rclcpp::Client<rcl_interfaces::srv::GetParameters>::SharedFuture f){
      const auto v=f.get()->values; param_pending_=false; param_good_=v.size()==9;
      if(param_good_) {
        param_good_=v[0].type==3 && std::abs(v[0].double_value-controller_.height)<.01;
        const int expected[]={0,1,2,3,4,5,7,8};
        for(size_t i=1;i<9;++i)param_good_=param_good_ && v[i].type==2 && v[i].integer_value==expected[i-1];
      }
      param_at_=steady();
    });
  }
  void output(Action a,double t) {
    if(a==Action::None || !controller_.enabled)return;
    if(command_pending_) {
      if(t-command_at_<=3 && a!=Action::Position)return;
      mode_->prune_pending_requests(); arm_->prune_pending_requests(); command_pending_=false;
      controller_.rejected("COMMAND_REPLY_TIMEOUT");
    }
    command_at_=t; command_pending_=true; const auto id=++request_id_;
    event({{"event","command"},{"id",id},{"action",int(a)}});
    if(a==Action::Arm) {
      if(!arm_->service_is_ready()) { command_pending_=false; controller_.rejected("ARM_SERVICE_LOST"); return; }
      auto p=std::make_shared<mavros_msgs::srv::CommandBool::Request>(); p->value=true;
      arm_->async_send_request(p,[this,id](rclcpp::Client<mavros_msgs::srv::CommandBool>::SharedFuture f){command_pending_=false; auto r=f.get(); last_reply_="ARM ACK "+std::to_string(r->result); event({{"event","reply"},{"id",id},{"success",r->success},{"result",r->result}}); if(!r->success)controller_.rejected("ARM_REJECTED"); });
    } else {
      if(!mode_->service_is_ready()) { command_pending_=false; controller_.rejected("MODE_SERVICE_LOST"); return; }
      auto p=std::make_shared<mavros_msgs::srv::SetMode::Request>();
      p->custom_mode=a==Action::Takeoff?"AUTO.TAKEOFF":a==Action::Land?"AUTO.LAND":"POSCTL";
      mode_->async_send_request(p,[this,id](rclcpp::Client<mavros_msgs::srv::SetMode>::SharedFuture f){command_pending_=false; auto r=f.get(); last_reply_=r->mode_sent?"MODE SENT (await actual state)":"MODE NOT SENT"; event({{"event","reply"},{"id",id},{"mode_sent",r->mode_sent}}); if(!r->mode_sent)controller_.rejected("MODE_NOT_SENT"); });
    }
  }
  void tick() {
    auto t=steady(); refresh(t); parameters(t); ipc(t); output(controller_.tick(sample_,t),t);
    if(last_phase_!=controller_.phase) { last_phase_=controller_.phase; event({{"event","phase"},{"status",status()}}); }
    if(t-status_at_>.5) { status_at_=t; event({{"event","telemetry"},{"status",status()}}); }
  }
};
int main(int argc,char** argv) {
  try {
    require(argc==3 && std::string(argv[1])=="--config","Usage: sangwon_native_hover --config FILE");
    umask(0077); std::ifstream in(argv[2]); require(bool(in),"CONFIG_UNAVAILABLE");
    std::string text{std::istreambuf_iterator<char>(in),std::istreambuf_iterator<char>()}; auto c=sangwon::service::parse(text);
    rclcpp::init(argc,argv); rclcpp::spin(std::make_shared<HoverNode>(c)); rclcpp::shutdown(); return 0;
  } catch(const std::exception& e) { std::cerr<<e.what()<<'\n'; return 2; }
}
