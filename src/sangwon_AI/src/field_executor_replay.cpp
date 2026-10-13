#include "sangwon_ai/field_executor.hpp"
int main() {
  using namespace sangwon::field;
  std::unique_ptr<FieldExecutor> executor;
  for(std::string line;std::getline(std::cin,line);) {
    try {
      const auto in=Json::parse(line);const auto method=in.at("method").get<std::string>();
      if(method=="configure")executor=std::make_unique<FieldExecutor>(in.at("settings"));
      else {
        require(bool(executor),"CONFIGURE_REQUIRED");
        if(method=="ack")executor->command_result(in.at("token").get<unsigned>(),flag(in,"accepted"),number(in.at("now_s"),0,1e12));
        else if(method=="queued")executor->command_queued(in.at("token").get<unsigned>());
        else if(method=="dispatched")executor->command_dispatched(in.at("token").get<unsigned>(),number(in.at("now_s"),0,1e12));
        else if(method=="tick")executor->tick(in.at("state"),number(in.at("now_s"),0,1e12),in.value("request",Json()),in.value("wall_s",0.));
        else require(false,"UNKNOWN_METHOD");
      }
      auto out=executor->status();out["actions"]=executor->actions;out["streaming"]=executor->streaming();std::cout<<Json({{"ok",true},{"payload",out}}).dump()<<std::endl;
    } catch(const std::exception& e) { std::cout<<Json({{"ok",false},{"error",e.what()}}).dump()<<std::endl; }
  }
}
