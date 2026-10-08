"""Cross-process and cross-language lock tests, without ROS or a flight device."""
import os
from pathlib import Path
import shutil
import subprocess
import sys

import pytest

from drone_mission.writer_lock import WriterLock, writer_lock_path

pytestmark = pytest.mark.skipif(sys.platform != 'linux', reason='Linux flock contract')


def test_both_physical_domains_share_owner_but_local_simulation_is_separate():
    assert writer_lock_path(1) == writer_lock_path(2)
    assert writer_lock_path(99) != writer_lock_path(1)


def test_cpp_and_python_exclude_each_other_and_recover_after_process_exit(tmp_path):
    compiler = shutil.which('g++')
    if not compiler:
        pytest.skip('C++ compiler unavailable')
    source = tmp_path/'writer.cpp'
    source.write_text('''#include "sangwon_ai/writer_lock.hpp"
#include <iostream>
int main(int argc,char** argv) {
  try { sangwon::WriterLock lock(std::stoi(argv[1]));
    std::cout << "held" << std::endl;
    if(argc>2) { char c; std::cin.get(c); }
    return 0;
  } catch(...) { return 23; }
}
''', encoding='utf-8')
    include = Path(__file__).parents[2]/'sangwon_AI/include'
    binary = tmp_path/'writer'
    subprocess.run([compiler,'-std=c++17','-I'+str(include),str(source),'-o',str(binary)],check=True)
    domain = 10000 + os.getpid()
    lock = WriterLock(domain)
    try:
        assert subprocess.run([str(binary),str(domain)],capture_output=True).returncode == 23
    finally:
        lock.close()
    child = subprocess.Popen([str(binary),str(domain),'wait'],stdin=subprocess.PIPE,stdout=subprocess.PIPE,text=True)
    try:
        assert child.stdout.readline().strip() == 'held'
        with pytest.raises(BlockingIOError):
            WriterLock(domain)
    finally:
        child.kill()
        child.wait(timeout=5)
    recovered = WriterLock(domain)
    recovered.close()
