#!/usr/bin/env python3
"""Compile the patched real driver against API doubles. No ROS, replay or device I/O."""
import argparse
from pathlib import Path
import subprocess


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--px4-source', type=Path, required=True)
    parser.add_argument('--compiler', type=Path, required=True)
    parser.add_argument('--zig', action='store_true', help='Use `zig c++` instead of a C++ compiler')
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    source = args.px4_source.resolve() / 'src/drivers/distance_sensor/tf02pro'
    fixtures = Path(__file__).resolve().parents[1] / 'test/firmware'
    output = args.output.resolve()
    includes = output / 'include'
    for header in ('px4_platform_common/px4_config.h', 'px4_platform_common/i2c_spi_buses.h',
                   'px4_platform_common/getopt.h', 'px4_platform_common/module.h',
                   'lib/drivers/rangefinder/PX4Rangefinder.hpp', 'drivers/device/i2c.h',
                   'drivers/drv_hrt.h', 'lib/perf/perf_counter.h',
                   'uORB/Subscription.hpp', 'uORB/topics/vehicle_status.h'):
        target = includes / header
        target.parent.mkdir(parents=True, exist_ok=True)
        target.write_text('#include "px4_test.hpp"\n', encoding='utf-8')
    executable = output / 'tfmini_plus_unit.exe'
    command = [str(args.compiler.resolve())] + (['c++'] if args.zig else [])
    command += ['-std=c++17', '-Wall', '-Wextra', '-Werror', '-I', str(includes),
                '-I', str(fixtures), '-I', str(source)]
    subprocess.run(command + ['-c', str(source / 'tf02pro_main.cpp'),
                             '-o', str(output / 'tf02pro_main.o')], check=True)
    subprocess.run(command + [str(source / 'TF02PRO.cpp'), str(fixtures / 'test_tfmini_plus.cpp'),
                             '-o', str(executable)], check=True)
    subprocess.run([str(executable)], check=True)


if __name__ == '__main__':
    main()
