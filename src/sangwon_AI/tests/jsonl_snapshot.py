"""Read a snapshot of an append-only journal without treating its tail as a row."""
import json
from pathlib import Path


def read_complete_jsonl(path):
    data = Path(path).read_bytes()
    # An active ofstream may have written only part of the next JSON row (or
    # UTF-8 character). Validate every completed row; only the unfinished tail
    # is excluded. A malformed newline-terminated row still fails the test.
    end = data.rfind(b'\n')
    return [json.loads(line) for line in data[:end+1].splitlines()]
