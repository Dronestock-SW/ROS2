// One in-process owner; no pipe, ROS publisher, FC transport or background thread.
#include "json_io.hpp"
#include <chrono>
#include <memory>
#include <string>
#ifdef _WIN32
#define BTF_EXPORT extern "C" __declspec(dllexport)
#else
#define BTF_EXPORT extern "C" __attribute__((visibility("default")))
#endif
namespace {
using Json=nlohmann::json;
struct Handle { std::unique_ptr<btf::Processor> processor; std::string reply; };
}
BTF_EXPORT int btf_abi_version() noexcept { return 1; }
BTF_EXPORT void* btf_create() noexcept { try { return new Handle; } catch(...) { return nullptr; } }
BTF_EXPORT void btf_destroy(void* ptr) noexcept { delete static_cast<Handle*>(ptr); }
// Reply belongs to this handle, valid only until its next call. Python copies it
// under the same lock as request/destroy. Errors invalidate all numerical state.
BTF_EXPORT const char* btf_request(void* ptr,const char* data) noexcept {
  auto* h=static_cast<Handle*>(ptr);
  if(!h)return "{\"error\":\"null_handle\"}";
  try {
    if(!data)throw std::invalid_argument("null_request");
    const auto j=Json::parse(data); const auto action=j.at("action").get<std::string>();
    Json out;
    if(action=="configure") {
      h->processor=std::make_unique<btf::Processor>(btf::json_io::config(j.at("config")));
      out={{"configured",true},{"abi",1}};
    } else {
      if(!h->processor)throw std::runtime_error("processor_unconfigured");
      if(action=="reset") {
        h->processor->reset_models(j.value("preserve_motion",false),j.value("preserve_height_conflict",true));
        out={{"reset",true}};
      } else if(action=="process") {
        const auto f=btf::json_io::frame(j.at("frame"));
        const auto started=std::chrono::steady_clock::now();
        out=btf::json_io::decision(h->processor->process(f));
        out["compute_ms"]=std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-started).count();
      } else throw std::invalid_argument("unknown_action");
    }
    h->reply=out.dump();
  } catch(const std::exception& e) {
    h->processor.reset();
    try { h->reply=Json({{"error",e.what()}}).dump(); } catch(...) { return "{\"error\":\"native_exception\"}"; }
  } catch(...) { h->processor.reset(); return "{\"error\":\"native_exception\"}"; }
  return h->reply.c_str();
}
