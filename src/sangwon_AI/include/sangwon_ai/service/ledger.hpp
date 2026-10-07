#pragma once
#include "sangwon_ai/service/json.hpp"
#include <sqlite3.h>

namespace sangwon::service {
class Ledger {
 public:
  explicit Ledger(const std::string& path);
  ~Ledger();
  Ledger(const Ledger&) = delete;
  Ledger& operator=(const Ledger&) = delete;
  void begin();
  void commit();
  void rollback();
  Json command(const std::string& id) const;
  void put_command(const std::string& id, const std::string& canonical, const Json& result);
  Json meta(const std::string& key, Json fallback = nullptr) const;
  void set_meta(const std::string& key, const Json& value);
  void enqueue(const std::string& key, const std::string& route, const Json& body);
  Json pending(const std::string& after_key="") const;
  void acknowledge(const std::string& key, const std::string& sha256);
 private:
  sqlite3* db_{};
  void exec(const char* sql);
};
}  // namespace sangwon::service
