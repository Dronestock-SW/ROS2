#include "btf/core.hpp"
#include "btf/freeze.hpp"
#include "json_io.hpp"
#include <nlohmann/json.hpp>
#include <chrono>
#include <fstream>
#include <iostream>
#include <stdexcept>

using Json=nlohmann::json;
namespace {
using namespace btf::json_io;
Json freeze(btf::FreezePolicy& policy,const Json& row) {
  const auto health=row.value("fc_health",std::string("unknown"));
  btf::FcHealth fc;
  if(health=="healthy")fc=btf::FcHealth::Healthy;
  else if(health=="invalid")fc=btf::FcHealth::Invalid;
  else if(health=="unknown")fc=btf::FcHealth::Unknown;
  else throw std::invalid_argument("invalid_fc_health");
  std::optional<btf::XY> target;
  if(row.contains("approved_target_xy_m")&&!row["approved_target_xy_m"].is_null())
    target=array<double,2>(row.at("approved_target_xy_m"));
  const auto d=policy.tick(timestamp(row.at("now_monotonic_us")),fc,target);
  return {{"state",btf::state_name(d.state)},{"pause_mission",d.pause_mission},
    {"stream_target",d.stream_target},{"target_xy_m",nullable(d.target_xy)},
    {"defer_to_fc_failsafe",d.defer_to_fc_failsafe},{"reason",d.reason},
    {"last_observation_xy_m",nullable(d.last_observation_xy)},
    {"last_observation_monotonic_us",nullable(d.last_observation_monotonic_us)}};
}
}  // namespace
int main(int argc,char** argv) {
  if(argc!=3){std::cerr<<"Usage: btf_replay CONFIG.json NORMALIZED_FRAMES.jsonl\n";return 2;}
  try {
    std::ifstream cf(argv[1]), input(argv[2]);
    if(!cf||!input)throw std::runtime_error("input_file_unavailable");
    Json j;cf>>j;btf::Processor processor(config(j));
    btf::FreezePolicy policy(j.value("freeze_observation_max_age_us",150000));
    std::string line;std::size_t number=0;bool failed=false;
    while(std::getline(input,line)) {
      ++number;Json out={{"line",number},{"external_output_allowed",false},{"flight_valid",false}};
      try {
        const auto row=Json::parse(line);const auto kind=row.value("type",std::string("frame"));
        if(kind=="reset") {processor.reset();policy.reset();out["reset"]=true;}
        else if(kind=="tick")out["freeze"]=freeze(policy,row);
        else if(kind=="frame") {
          const auto f=frame(row);const auto before=std::chrono::steady_clock::now();
          const auto d=processor.process(f);
          const auto elapsed=std::chrono::steady_clock::now()-before;
          out["compute_ms"]=std::chrono::duration<double,std::milli>(elapsed).count();
          out["measurement"]=decision(d);
          if(row.contains("now_monotonic_us")) {
            policy.observe(d,timestamp(row.at("source_monotonic_us")),timestamp(row.at("now_monotonic_us")));
            out["freeze"]=freeze(policy,row);
          }
        } else throw std::invalid_argument("unknown_record_type");
      } catch(const std::exception& e) {
        // A malformed row cannot leave a previous valid measurement eligible.
        processor.reset();policy.reset();out["error"]=e.what();failed=true;
      }
      std::cout<<out.dump()<<'\n';
    }
    if(input.bad()||!std::cout)throw std::runtime_error("replay_io_error");
    return failed?1:0;
  } catch(const std::exception& e){std::cerr<<e.what()<<'\n';return 2;}
}
