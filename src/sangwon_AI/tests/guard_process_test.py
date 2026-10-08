"""Independent C++ watchdog test using Linux monotonic timestamps and pipes."""
import selectors
import subprocess
import sys
import time


def main():
    process = subprocess.Popen([sys.argv[1], "--replay-only"], stdin=subprocess.PIPE,
                               stdout=subprocess.PIPE, stderr=subprocess.PIPE,
                               text=True, encoding="utf-8", bufsize=1)
    selector = selectors.DefaultSelector()
    selector.register(process.stdout, selectors.EVENT_READ)
    try:
        assert selector.select(3), "guard startup timeout"
        ready = process.stdout.readline().split()
        assert ready[0] == "READY"
        assert abs(float(ready[1]) - time.monotonic()) < 1, "clock domain mismatch"
        issued = time.monotonic()
        process.stdin.write(f"STATE {issued:.9f}\nINTENT {issued:.9f} 1\n")
        process.stdin.flush()
        kinds = []
        land_time = None
        while time.monotonic() - issued < 2:
            # Vehicle input stays alive; BT intent producer has stopped.
            process.stdin.write(f"STATE {time.monotonic():.9f}\n")
            process.stdin.flush()
            for _key, _event in selector.select(0.02):
                line = process.stdout.readline().split()
                assert line, "guard exited"
                if line[0] == "OUTPUT":
                    kinds.append(int(line[1]))
                    if int(line[1]) == 4:  # OutputKind::RequestLand
                        land_time = float(line[2])
                        assert line[3] == "INTENT_EXPIRED"
                        break
            if land_time is not None:
                break
        assert 1 in kinds, "no valid position output before producer stop"
        assert land_time is not None, "independent watchdog did not request Land"
        elapsed = land_time - issued
        assert 0.5 <= elapsed < 1.5, f"unexpected watchdog delay: {elapsed}"
        print(f"PASS independent_guard lease_s=0.5 observed_land_s={elapsed:.3f}")
        process.stdin.write("QUIT\n")
        process.stdin.flush()
        assert process.wait(timeout=3) == 0
    finally:
        selector.close()
        if process.poll() is None:
            process.kill()
            process.wait()


if __name__ == "__main__":
    main()
