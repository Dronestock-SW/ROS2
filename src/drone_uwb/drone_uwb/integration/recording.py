"""Session file ownership; received bytes and processing results stay separate."""
from contextlib import ExitStack
import json
from pathlib import Path


def open_record_files(directory, metadata):
    """Return raw/received/decisions/status streams; caller closes all streams.

    The session directory must be new. Failed creation leaves evidence on disk
    but closes every opened stream; retry with a new session directory.
    """
    encoded = json.dumps(metadata, ensure_ascii=False, allow_nan=False, indent=2) + '\n'
    target = Path(directory)
    target.mkdir(parents=True, exist_ok=False)
    raw, processed = target / 'raw', target / 'processed'
    raw.mkdir()
    processed.mkdir()
    with ExitStack() as stack:
        streams = {
            'raw': stack.enter_context((raw / 'serial.raw').open('xb')),
            'received': stack.enter_context((raw / 'received.jsonl').open('x', encoding='utf-8')),
            'decisions': stack.enter_context((processed / 'decisions.jsonl').open('x', encoding='utf-8')),
            'status': stack.enter_context((processed / 'node_status.jsonl').open('x', encoding='utf-8')),
        }
        with (raw / 'node_metadata.json').open('x', encoding='utf-8') as stream:
            stream.write(encoded)
        stack.pop_all()
    return streams
