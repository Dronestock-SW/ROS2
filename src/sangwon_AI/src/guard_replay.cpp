// Linux-only pipe fixture. No ROS, MAVLink, network, serial or actuator access.
#include "sangwon_ai/guard.hpp"
#include <chrono>
#include <iostream>
#include <sstream>
#include <poll.h>
#include <unistd.h>

namespace {
double mono() {
  return std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();
}
void complete(std::istringstream& in) {
  in>>std::ws;
  if(!in.eof()) throw std::invalid_argument("Trailing fixture input");
}
}
int main(int argc,char** argv) {
  using namespace sangwon;
  if(argc!=2||std::string(argv[1])!="--replay-only") {
    std::cerr<<"Requires --replay-only; hardware transport is not implemented\n"; return 2;
  }
  try {
    FlightGuard guard({},"pipe-fixture","replay-pipe-boot",1,1);
    State state; bool ready=false; std::string buffer;
    OutputKind last=OutputKind::None;
    std::cout.precision(17); std::cout<<"READY "<<mono()<<'\n'<<std::flush;
    for(;;) {
      pollfd fd{STDIN_FILENO,POLLIN,0};
      const int result=poll(&fd,1,20);
      if(result<0) throw std::runtime_error("poll failed");
      if(result>0&&(fd.revents&(POLLIN|POLLHUP))) {
        char bytes[4096]; const auto count=read(STDIN_FILENO,bytes,sizeof(bytes));
        if(count<=0) break;
        buffer.append(bytes,static_cast<std::size_t>(count));
        if(buffer.size()>8192) throw std::invalid_argument("Fixture input too large");
        std::size_t end;
        while((end=buffer.find('\n'))!=std::string::npos) {
          std::istringstream input(buffer.substr(0,end)); buffer.erase(0,end+1);
          std::string type; input>>type;
          if(type=="QUIT") { complete(input); return 0; }
          if(type=="STATE") {
            double timestamp=0;
            if(!(input>>timestamp)||!std::isfinite(timestamp)) throw std::invalid_argument("Bad state");
            complete(input);
            // Synthetic constant hover: only a test source can generate this fixture.
            state.pose.p={0,0,1}; state.mode=Mode::Offboard;
            state.armed=true; state.landed=false; state.connected=state.can_hold=true;
            state.pose_quality=state.yaw_quality=Quality::Valid;
            state.pose_time_s=state.mode_time_s=state.quality_time_s=timestamp; ready=true;
          } else if(type=="INTENT") {
            double timestamp=0; std::uint64_t sequence=0;
            if(!(input>>timestamp>>sequence)) throw std::invalid_argument("Bad intent");
            complete(input);
            Intent i{1,"pipe-fixture","replay-pipe-boot",1,sequence,1,1,IntentKind::Hold,
                     {{0,0,1},0},timestamp,timestamp+0.5};
            if(!guard.accept(i,mono())) std::cout<<"REJECTED\n"<<std::flush;
          } else throw std::invalid_argument("Unknown fixture record");
        }
      }
      if(ready) {
        const double now=mono(); const auto out=guard.tick(state,now);
        if(out.kind!=last) {
          last=out.kind;
          std::cout<<"OUTPUT "<<static_cast<int>(out.kind)<<' '<<now<<' '<<out.reason<<'\n'<<std::flush;
        }
      }
    }
  } catch(const std::exception& e) { std::cerr<<e.what()<<'\n'; return 1; }
  return 0;
}
