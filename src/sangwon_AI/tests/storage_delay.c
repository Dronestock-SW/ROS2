/* Test-only LD_PRELOAD shim. Never built into an executor. */
#define _GNU_SOURCE
#include <dlfcn.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

int fsync(int fd) {
  static int (*real_sync)(int);
  static unsigned commands;
  if (!real_sync) real_sync=dlsym(RTLD_NEXT,"fsync");
  const char *domain=getenv("ROS_DOMAIN_ID"), *local=getenv("ROS_LOCALHOST_ONLY");
  if (domain && local && !strcmp(domain,"99") && !strcmp(local,"1")) {
    char link[64],path[4096];snprintf(link,sizeof(link),"/proc/self/fd/%d",fd);
    ssize_t n=readlink(link,path,sizeof(path)-1);
    if(n>0){path[n]=0;
      if(strstr(path,"native-events.jsonl.commands")) {
        ++commands;
        const char *marker=getenv("STORAGE_TEST_MARKER");
        if(marker){FILE *f=fopen(marker,"w");if(f){fprintf(f,"%u",commands);fclose(f);}}
        usleep(4000000);
      } else if(strstr(path,"native-events.jsonl")) usleep(4000000);
    }
  }
  return real_sync(fd);
}
