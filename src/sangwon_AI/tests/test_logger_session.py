import importlib.util
from pathlib import Path
import sys
from types import SimpleNamespace as NS
import pytest

OPS=Path(__file__).parents[1]/'ops'
sys.path.insert(0,str(OPS))
spec=importlib.util.spec_from_file_location('logger_session',OPS/'px4_logger_session.py')
tool=importlib.util.module_from_spec(spec);spec.loader.exec_module(tool)


def test_only_logger_commands_and_readbacks_are_exposed():
    assert set(tool.COMMANDS)-set(tool.READ_QUERIES)=={'logger on','logger off'}
    for command in ('param set SDLOG_MODE 2','commander arm','reboot','logger stop'):
        with pytest.raises(RuntimeError,match='not allowed'):tool.LoggerSession.query(NS(),command)


@pytest.mark.parametrize('state,stamp,expected',[(1,1_000_000_000,True),(2,1_000_000_000,False),
    (1,0,False),(1,4_000_000_000,False)])
def test_ground_guard_requires_fresh_source_timestamp(state,stamp,expected):
    m=NS(landed_state=state,header=NS(stamp=NS(sec=stamp//10**9,nanosec=stamp%10**9)))
    assert tool.fresh_ground(m,2_000_000_000)==expected


def test_logger_growth_units_and_sd_free_space():
    assert tool.parse_logger('Not logging')['logging'] is False
    a=tool.parse_logger('Full File Logging Running:\nLog file: /fs/microsd/log/2026-10-11/03_00_00.ulg\nWrote 90.0 KiB')
    b=tool.parse_logger('Full File Logging Running:\nLog file: /fs/microsd/log/2026-10-11/03_00_00.ulg\nWrote 0.2 MiB')
    assert a['logging'] and a['path']==b['path'] and b['written_bytes_approx']>a['written_bytes_approx']
    assert tool.sd_free_bytes('32768 243747 194469 49278 /fs/microsd')==32768*49278
    with pytest.raises(RuntimeError):tool.sd_free_bytes('0 0 0 0 /proc')
