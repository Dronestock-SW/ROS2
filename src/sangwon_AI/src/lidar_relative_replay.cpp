#include "sangwon_ai/lidar_relative.hpp"
#include <iostream>
int main() {
  using sangwon::service::Json;
  std::string line;
  try {
    if(!std::getline(std::cin,line)) return 2;
    sangwon::observe::LidarGapSupport support(Json::parse(line));
    while(std::getline(std::cin,line)) {
      if(line.size()>262144) throw std::invalid_argument("ROW_TOO_LARGE");
      const auto row=Json::parse(line);
      std::cout << support.update(row.at("state"),row.at("sample_time_s").get<double>()).dump() << std::endl;
    }
  } catch(const std::exception& e) { std::cerr << e.what() << '\n';return 2; }
}
