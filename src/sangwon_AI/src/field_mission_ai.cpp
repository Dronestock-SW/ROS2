#include "sangwon_ai/field_mission.hpp"
using namespace sangwon::field;
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
