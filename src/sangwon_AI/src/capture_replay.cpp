// File-only executable. It cannot create ROS clients, publishers or FC links.
#include "sangwon_ai/capture_observations.hpp"
#include <openssl/evp.h>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <memory>

using sangwon::service::Json;
using sangwon::service::require;
namespace fs = std::filesystem;
namespace {
// Bound memory even for damaged or untrusted capture rows. Stop at the first
// malformed/truncated row; a partial capture must never receive a success report.
bool line(std::istream& input, std::string& text) {
  text.clear(); char c;
  while (input.get(c)) {
    if (c == '\n') return true;
    require(text.size() < 1024*1024, "CAPTURE_LINE_TOO_LARGE"); text.push_back(c);
  }
  require(input.eof(), "CAPTURE_READ_FAILED");
  require(text.empty(), "CAPTURE_TRUNCATED_LINE"); return false;
}
std::string hash_final(EVP_MD_CTX* context) {
  unsigned char bytes[EVP_MAX_MD_SIZE]; unsigned count{};
  require(EVP_DigestFinal_ex(context, bytes, &count) == 1, "HASH_FAILED");
  std::ostringstream out;
  for (unsigned i = 0; i < count; ++i) out << std::hex << std::setw(2) << std::setfill('0') << unsigned(bytes[i]);
  return out.str();
}
}  // namespace
int main(int argc, char** argv) {
  try {
    require(argc == 3 && std::string(argv[1]) == "--capture", "Usage: sangwon_capture_replay --capture DIRECTORY");
    const auto root = fs::canonical(argv[2]);
    require(fs::file_size(root / "manifest.json") <= 1024*1024, "MANIFEST_TOO_LARGE");
    std::ifstream manifest(root / "manifest.json", std::ios::binary);
    require(bool(manifest), "MANIFEST_UNAVAILABLE");
    const std::string manifest_bytes{std::istreambuf_iterator<char>(manifest), std::istreambuf_iterator<char>()};
    require(manifest.eof() || bool(manifest), "MANIFEST_READ_FAILED");
    sangwon::observe::CaptureObservations observations(sangwon::service::parse(manifest_bytes));
    const auto file = root / "events.jsonl";
    const auto initial_size = fs::file_size(file); const auto initial_mtime = fs::last_write_time(file);
    require(initial_size > 0, "CAPTURE_EMPTY");
    std::ifstream input(file, std::ios::binary); require(bool(input), "CAPTURE_UNAVAILABLE");
    std::unique_ptr<EVP_MD_CTX, decltype(&EVP_MD_CTX_free)> hash(EVP_MD_CTX_new(), &EVP_MD_CTX_free);
    require(hash && EVP_DigestInit_ex(hash.get(), EVP_sha256(), nullptr) == 1, "HASH_FAILED");
    std::string row; std::uint64_t bytes{};
    while (line(input, row)) {
      require(!row.empty(), "CAPTURE_EMPTY_ROW");
      require(EVP_DigestUpdate(hash.get(), row.data(), row.size()) == 1
        && EVP_DigestUpdate(hash.get(), "\n", 1) == 1, "HASH_FAILED");
      bytes += row.size()+1;
      observations.ingest(sangwon::service::parse(row));
    }
    require(bytes == initial_size && fs::file_size(file) == initial_size
      && fs::last_write_time(file) == initial_mtime, "CAPTURE_CHANGED_DURING_REPLAY");
    auto report = observations.snapshot(observations.last_receipt_ns());
    report["input"] = {{"manifest_sha256", sangwon::service::digest(manifest_bytes)},
      {"events_sha256", hash_final(hash.get())}, {"events_bytes", bytes}};
    report["limitation"] = "Sample validity is not alignment, timing, EKF fusion or flight readiness. Raw omitted topics remain in the capture.";
    std::cout << report.dump(2) << '\n'; return 0;
  } catch (const std::invalid_argument& e) {
    // Only fixed contract codes/usage text are thrown as invalid_argument.
    std::cerr << "CAPTURE_REPLAY_FAILED: " << e.what() << '\n'; return 2;
  } catch (const std::exception&) {
    // No partial report or raw sensor payload on failure.
    std::cerr << "CAPTURE_REPLAY_FAILED: inspect immutable manifest and complete JSONL input\n";
    return 2;
  }
}
