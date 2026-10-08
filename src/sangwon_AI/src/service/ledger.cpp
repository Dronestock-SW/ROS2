#include "sangwon_ai/service/ledger.hpp"

namespace sangwon::service {
namespace {
struct Statement {
  sqlite3_stmt* s{};
  explicit Statement(sqlite3* db, const char* sql) {
    if (sqlite3_prepare_v2(db, sql, -1, &s, nullptr) != SQLITE_OK)
      throw std::runtime_error("LEDGER_PREPARE_FAILED");
  }
  ~Statement() { sqlite3_finalize(s); }
  void bind(int n, const std::string& text) {
    if (sqlite3_bind_text(s, n, text.c_str(), int(text.size()), SQLITE_TRANSIENT) != SQLITE_OK)
      throw std::runtime_error("LEDGER_BIND_FAILED");
  }
  int step() {
    const int rc = sqlite3_step(s);
    if (rc != SQLITE_ROW && rc != SQLITE_DONE) throw std::runtime_error("LEDGER_WRITE_FAILED");
    return rc;
  }
  std::string text(int col) const {
    const auto* p = sqlite3_column_text(s, col);
    return p ? reinterpret_cast<const char*>(p) : "";
  }
};
}
Ledger::Ledger(const std::string& path) {
  if (sqlite3_open_v2(path.c_str(), &db_, SQLITE_OPEN_READWRITE | SQLITE_OPEN_CREATE, nullptr) != SQLITE_OK)
    throw std::runtime_error("LEDGER_OPEN_FAILED");
  sqlite3_busy_timeout(db_, 250);
  exec("PRAGMA journal_mode=WAL; PRAGMA synchronous=FULL;");
  Statement version(db_, "PRAGMA user_version");
  require(version.step() == SQLITE_ROW && sqlite3_column_int(version.s, 0) <= 1, "LEDGER_VERSION_UNSUPPORTED");
  exec("CREATE TABLE IF NOT EXISTS commands(id TEXT PRIMARY KEY, canonical TEXT NOT NULL, result TEXT NOT NULL);"
       "CREATE TABLE IF NOT EXISTS meta(key TEXT PRIMARY KEY,value TEXT NOT NULL);"
       "CREATE TABLE IF NOT EXISTS outbox(key TEXT PRIMARY KEY,route TEXT NOT NULL,body TEXT NOT NULL,sha TEXT NOT NULL,acked INTEGER NOT NULL DEFAULT 0);"
       "PRAGMA user_version=1;");
}
Ledger::~Ledger() { if (db_) sqlite3_close(db_); }
void Ledger::exec(const char* sql) {
  if (sqlite3_exec(db_, sql, nullptr, nullptr, nullptr) != SQLITE_OK) throw std::runtime_error("LEDGER_TRANSACTION_FAILED");
}
void Ledger::begin() { exec("BEGIN IMMEDIATE"); }
void Ledger::commit() { exec("COMMIT"); }
void Ledger::rollback() { exec("ROLLBACK"); }
Json Ledger::command(const std::string& id) const {
  Statement q(db_, "SELECT canonical,result FROM commands WHERE id=?"); q.bind(1, id);
  if (q.step() == SQLITE_DONE) return nullptr;
  return Json{{"canonical", q.text(0)}, {"result", parse(q.text(1))}};
}
void Ledger::put_command(const std::string& id, const std::string& canonical, const Json& result) {
  Statement q(db_, "INSERT INTO commands(id,canonical,result) VALUES(?,?,?) ON CONFLICT(id) DO UPDATE SET result=excluded.result");
  q.bind(1,id); q.bind(2,canonical); q.bind(3,result.dump()); q.step();
}
Json Ledger::meta(const std::string& key, Json fallback) const {
  Statement q(db_, "SELECT value FROM meta WHERE key=?"); q.bind(1,key);
  return q.step() == SQLITE_ROW ? parse(q.text(0)) : fallback;
}
void Ledger::set_meta(const std::string& key, const Json& value) {
  Statement q(db_, "INSERT INTO meta(key,value) VALUES(?,?) ON CONFLICT(key) DO UPDATE SET value=excluded.value");
  q.bind(1,key); q.bind(2,value.dump()); q.step();
}
void Ledger::enqueue(const std::string& key, const std::string& route, const Json& body) {
  const auto content = body.dump();
  Statement q(db_, "INSERT OR IGNORE INTO outbox(key,route,body,sha) VALUES(?,?,?,?)");
  q.bind(1,key); q.bind(2,route); q.bind(3,content); q.bind(4,digest(content)); q.step();
  Statement verify(db_, "SELECT sha FROM outbox WHERE key=?"); verify.bind(1,key);
  require(verify.step() == SQLITE_ROW && verify.text(0) == digest(content), "OUTBOX_ID_CONFLICT");
}
Json Ledger::pending(const std::string& after_key) const {
  if(!after_key.empty()) {
    Statement cursor(db_,"SELECT key FROM outbox WHERE key=?");cursor.bind(1,after_key);
    require(cursor.step()==SQLITE_ROW,"INVALID_OUTBOX_CURSOR");
  }
  Json rows = Json::array();
  Statement q(db_, "SELECT key,route,body,sha FROM outbox WHERE acked=0 AND (?='' OR rowid>(SELECT rowid FROM outbox WHERE key=?)) ORDER BY rowid LIMIT 32");
  q.bind(1,after_key);q.bind(2,after_key);
  while (q.step() == SQLITE_ROW) rows.push_back({{"key",q.text(0)}, {"route",q.text(1)}, {"body",parse(q.text(2))}, {"raw_body",q.text(2)}, {"sha256",q.text(3)}});
  return rows;
}
void Ledger::acknowledge(const std::string& key, const std::string& sha256) {
  Statement q(db_, "UPDATE outbox SET acked=1 WHERE key=? AND sha=?");
  q.bind(1,key); q.bind(2,sha256); q.step();
}
}  // namespace sangwon::service
