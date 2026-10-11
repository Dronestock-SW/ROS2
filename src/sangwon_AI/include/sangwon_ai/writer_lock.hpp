#pragma once
#include <fcntl.h>
#include <sys/file.h>
#include <sys/stat.h>
#include <unistd.h>
#include <stdexcept>
#include <string>

namespace sangwon {
// The same fixed path and flock protocol as drone_mission.writer_lock.
class WriterLock {
 public:
  explicit WriterLock(int domain) {
    const std::string scope=(domain==1 || domain==2) ? "flight" : "test-"+std::to_string(domain);
    const auto path="/tmp/dronestock-px4-writer-"+scope+".lock";
    fd_=open(path.c_str(),O_CREAT|O_RDWR|O_CLOEXEC|O_NOFOLLOW,0600);
    struct stat info{};
    if(fd_<0 || fstat(fd_,&info)!=0 || !S_ISREG(info.st_mode)
       || info.st_uid!=geteuid() || info.st_nlink!=1 || flock(fd_,LOCK_EX|LOCK_NB)!=0) {
      if(fd_>=0) close(fd_);
      fd_=-1;
      throw std::runtime_error("PX4_WRITER_ALREADY_RUNNING_OR_LOCK_UNAVAILABLE");
    }
  }
  ~WriterLock() { if(fd_>=0) close(fd_); } // Keep the inode, release only the descriptor.
  WriterLock(const WriterLock&)=delete;
  WriterLock& operator=(const WriterLock&)=delete;
 private:
  int fd_{-1};
};
}
