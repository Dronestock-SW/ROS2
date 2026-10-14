import json

import pytest

from jsonl_snapshot import read_complete_jsonl


def test_snapshot_excludes_in_progress_json_and_utf8_tail(tmp_path):
    path = tmp_path/'live.jsonl'
    path.write_bytes(b'{"state":"END"}\n{"name":"' + bytes([0xe3]))
    assert read_complete_jsonl(path) == [{'state':'END'}]
    path.write_text('{"state":"END"}\n{"name":"finished"}\n', encoding='utf-8')
    assert read_complete_jsonl(path) == [{'state':'END'}, {'name':'finished'}]


def test_snapshot_rejects_corrupt_completed_row(tmp_path):
    path = tmp_path/'bad.jsonl'
    path.write_bytes(b'{"state":"END"}\n{"broken":}\n')
    with pytest.raises(json.JSONDecodeError):
        read_complete_jsonl(path)


def test_unterminated_only_row_is_not_completion_evidence(tmp_path):
    path = tmp_path/'partial.jsonl'
    path.write_bytes(b'{"state":"END"}')
    assert read_complete_jsonl(path) == []
