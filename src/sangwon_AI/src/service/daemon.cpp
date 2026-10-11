#include "sangwon_ai/service/engine.hpp"
#include <sys/file.h>
#include <sys/socket.h>
#include <sys/stat.h>
#include <sys/un.h>
#include <poll.h>
#include <unistd.h>
#include <fcntl.h>
#include <signal.h>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <cstring>

namespace fs=std::filesystem;
using namespace sangwon::service;
namespace {
volatile sig_atomic_t running=1;
void stop(int) { running=0; }
std::string read_file(const fs::path& path) {
  std::ifstream in(path); require(bool(in),"FILE_UNAVAILABLE");
  return {std::istreambuf_iterator<char>(in),std::istreambuf_iterator<char>()};
}
void atomic_status(const fs::path& path,const Json& status) {
  const auto tmp=path.string()+".tmp";
  { std::ofstream out(tmp); out<<status.dump(2)<<'\n'; out.flush(); require(bool(out),"STATUS_WRITE_FAILED"); }
  fs::rename(tmp,path);
}
Json exchange(int fd,Engine& engine) {
  std::string text; char chunk[8192];
  const double end=steady_seconds()+0.5;
  while(text.find('\n')==std::string::npos) {
    const int remaining=int((end-steady_seconds())*1000); require(remaining>0,"IPC_TIMEOUT");
    pollfd p{fd,POLLIN,0}; require(poll(&p,1,remaining)>0,"IPC_TIMEOUT");
    const auto n=recv(fd,chunk,sizeof(chunk),0); require(n>0,"IPC_CLOSED"); text.append(chunk,std::size_t(n));
    require(text.size()<=2*1024*1024,"IPC_MESSAGE_TOO_LARGE");
  }
  const auto split=text.find('\n'); require(split==text.size()-1,"IPC_ONE_REQUEST_PER_CONNECTION");
  auto request=parse(text);
  require(request.at("ipc_version")==1,"UNSUPPORTED_IPC_VERSION");
  return {{"ipc_version",1},{"request_id",request.at("request_id")},{"ok",true},
    {"payload",engine.handle(request.at("method"),request.value("payload",Json::object()))}};
}
}
int main(int argc,char** argv) {
  try {
    require(argc==3 && std::string(argv[1])=="--config","Usage: sangwon_companiond --config FILE");
    umask(0077); signal(SIGTERM,stop); signal(SIGINT,stop); signal(SIGPIPE,SIG_IGN);
    const auto config_path=fs::canonical(argv[2]); auto config=parse(read_file(config_path));
    const fs::path root=config_path.parent_path().parent_path();
    fs::path state=fs::weakly_canonical(root/config.at("state_dir").get<std::string>());
    // All writable state must remain below this package's private runtime directory.
    const auto prefix=(root/".runtime").string()+"/";
    require(state.string().rfind(prefix,0)==0,"STATE_PATH_OUTSIDE_RUNTIME");
    fs::create_directories(state); fs::permissions(state,fs::perms::owner_all);
    const int lock=open((state/"core.lock").c_str(),O_RDWR|O_CREAT,0600);
    require(lock>=0 && flock(lock,LOCK_EX|LOCK_NB)==0,"CORE_ALREADY_RUNNING");
    const std::string socket_path=(state/"core.sock").string();
    sockaddr_un addr{}; addr.sun_family=AF_UNIX;
    require(socket_path.size()<sizeof(addr.sun_path),"IPC_PATH_TOO_LONG");
    std::strcpy(addr.sun_path,socket_path.c_str());
    if(fs::exists(socket_path)) { require(fs::is_socket(socket_path),"IPC_PATH_CONFLICT"); fs::remove(socket_path); }
    const int server=socket(AF_UNIX,SOCK_STREAM|SOCK_CLOEXEC,0); require(server>=0,"IPC_SOCKET_FAILED");
    require(bind(server,reinterpret_cast<sockaddr*>(&addr),sizeof(addr))==0 && listen(server,16)==0,"IPC_BIND_FAILED");
    auto boot=read_file("/proc/sys/kernel/random/boot_id"); while(!boot.empty() && boot.back()=='\n') boot.pop_back();
    Engine engine(config,boot,(state/"ledger.sqlite3").string());
    double next=steady_seconds(),next_status=0;
    std::cout<<"sangwon_companiond profile="<<config.at("profile")<<" physical_output=false\n"<<std::flush;
    while(running) {
      const auto now=steady_seconds();
      if(now>=next) { engine.tick(now); next=now+0.05; }
      if(now>=next_status) { atomic_status(state/"core_status.json",engine.status()); next_status=now+0.5; }
      pollfd p{server,POLLIN,0}; const int ready=poll(&p,1,20);
      if(ready>0 && (p.revents&POLLIN)) {
        const int client=accept4(server,nullptr,nullptr,SOCK_CLOEXEC);
        if(client<0) continue;
        Json response;
        try { response=exchange(client,engine); }
        catch(const std::invalid_argument& e) { response={{"ipc_version",1},{"ok",false},{"code",e.what()}}; }
        catch(const Json::exception&) { response={{"ipc_version",1},{"ok",false},{"code","INVALID_JSON_MESSAGE"}}; }
        const auto data=response.dump()+"\n";
        // Nonblocking bounded response. The client treats a partial reply as failed.
        std::size_t sent=0;
        while(sent<data.size()) {
          const auto n=send(client,data.data()+sent,data.size()-sent,MSG_NOSIGNAL|MSG_DONTWAIT);
          if(n<=0) break;
          sent+=std::size_t(n);
        }
        close(client);
      }
    }
    close(server); fs::remove(socket_path); close(lock); return 0;
  } catch(const std::exception& e) {
    std::cerr<<"companiond stopped: "<<e.what()<<'\n'; return 2;
  }
}
