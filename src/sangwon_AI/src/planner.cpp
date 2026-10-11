#include "sangwon_ai/planner.hpp"
#include <chrono>
#include <limits>
#include <map>
#include <queue>
#include <tuple>

namespace sangwon {
void PlannerLimits::validate() const {
  if(!std::isfinite(resolution_m)||resolution_m<0.02||resolution_m>1
      ||!std::isfinite(max_length_m)||max_length_m<=0||max_length_m>1000
      ||!std::isfinite(budget_s)||budget_s<=0||budget_s>0.1
      ||max_nodes<4||max_nodes>65536||max_expansions<1||max_expansions>max_nodes)
    throw std::invalid_argument("INVALID_PLANNER_LIMITS");
}
PlannedRoute plan_horizontal_route(const FlightSpace& space,Vec3 start,Vec3 goal,double h,double v,const PlannerLimits& limits) {
  limits.validate();PlannedRoute result;
  if(!finite(start)||!finite(goal)||!std::isfinite(h)||!std::isfinite(v)||h<0||v<0
      ||std::abs(start.z-goal.z)>1e-8) {result.code="PLAN_INVALID_INPUT";return result;}
  if(!space.boundary_provided||space.boundary.empty()) {result.code="PLAN_BOUNDARY_REQUIRED";return result;}
  if(*space_segment_error(space,start,start,h,v)) {result.code="PLAN_START_INVALID";return result;}
  if(*space_segment_error(space,goal,goal,h,v)) {result.code="PLAN_GOAL_INVALID";return result;}
  if(xy(goal-start)>limits.max_length_m) {result.code="PLAN_LENGTH_LIMIT";return result;}
  const auto began=std::chrono::steady_clock::now();
  const auto expired=[&] {return std::chrono::duration<double>(std::chrono::steady_clock::now()-began).count()>=limits.budget_s;};
  if(!*space_segment_error(space,start,goal,h,v)) {result.points={start,goal};result.length_m=xy(goal-start);result.code="OK";return result;}
  using Key=std::pair<int,int>;
  struct Node {double g{std::numeric_limits<double>::infinity()};Key parent{};bool closed{};};
  using Candidate=std::tuple<double,double,Key>;
  std::priority_queue<Candidate,std::vector<Candidate>,std::greater<Candidate>> queue;
  std::map<Key,Node> nodes;
  const auto position=[&](Key key) {return start+Vec3{key.first*limits.resolution_m,key.second*limits.resolution_m,0};};
  const Key root{0,0};nodes[root].g=0;queue.emplace(xy(goal-start),0,root);
  std::optional<Key> end;
  while(!queue.empty()) {
    if(expired()) {result.code="PLAN_TIME_BUDGET";return result;}
    const auto [cost,g,key]=queue.top();queue.pop();(void)cost;
    auto& current=nodes.at(key);if(current.closed||g!=current.g) continue;
    if(result.expanded>=limits.max_expansions) {result.code="PLAN_EXPANSION_LIMIT";return result;}
    current.closed=true;++result.expanded;const auto p=position(key);
    if(xy(goal-p)<=1.5*limits.resolution_m&&g+xy(goal-p)<=limits.max_length_m
        &&!*space_segment_error(space,p,goal,h,v)) {end=key;break;}
    for(int dx=-1;dx<=1;++dx) for(int dy=-1;dy<=1;++dy) {
      if((dx==0&&dy==0)||expired()) continue;
      const Key next{key.first+dx,key.second+dy};const auto q=position(next);
      const auto next_g=g+xy(q-p);
      if(next_g+xy(goal-q)>limits.max_length_m||*space_segment_error(space,p,q,h,v)) continue;
      auto found=nodes.find(next);
      if(found==nodes.end()) {
        if(nodes.size()>=limits.max_nodes) {result.code="PLAN_NODE_LIMIT";return result;}
        found=nodes.emplace(next,Node{}).first;
      }
      auto& node=found->second;
      if(!node.closed&&next_g<node.g) {node.g=next_g;node.parent=key;queue.emplace(next_g+xy(goal-q),next_g,next);}
    }
  }
  if(!end) {result.code=expired()?"PLAN_TIME_BUDGET":"PLAN_NO_ROUTE_AT_RESOLUTION";return result;}
  std::vector<Vec3> raw{goal};auto key=*end;
  for(unsigned count=0;;++count) {
    if(count>=limits.max_nodes) {result.code="PLAN_INTERNAL_ERROR";return result;}
    raw.push_back(position(key));if(key==root) break;key=nodes.at(key).parent;
  }
  std::reverse(raw.begin(),raw.end());result.points={start};
  // Greedy shortcutting is optional. Budget exhaustion returns the already
  // proven grid route, never an unchecked shortcut or an incomplete route.
  for(std::size_t i=0;i+1<raw.size();) {
    auto next=i+1;
    for(auto j=raw.size()-1;j>i+1&&!expired();--j)
      if(!*space_segment_error(space,raw[i],raw[j],h,v)) {next=j;break;}
    result.length_m+=xy(raw[next]-result.points.back());result.points.push_back(raw[next]);i=next;
  }
  result.code="OK";return result;
}
} // namespace sangwon
