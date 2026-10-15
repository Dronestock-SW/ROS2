"""Import the pinned upstream sources without modifying the supplied archive."""
import argparse
import hashlib
import json
from pathlib import Path, PurePosixPath
import zipfile

ZIP_SHA256 = '6d75f7254644409913ac1b6f1075f51e5ee8ff77a9c27f8c257ee7383b71e112'
PREFIX = 'uwb-multitag-research-main/experiments/tdma40-radiofix3/projects/'


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('archive', type=Path)
    parser.add_argument('--destination', type=Path, required=True)
    args = parser.parse_args()
    digest = hashlib.file_digest(args.archive.open('rb'), 'sha256').hexdigest()
    if digest != ZIP_SHA256:
        raise SystemExit('Archive SHA-256 does not match the approved baseline')
    destination = args.destination.resolve()
    if any((destination / name).exists() for name in ('tag', 'anchor', 'common')):
        raise SystemExit('Refusing to overwrite existing firmware sources')
    entries = []
    with zipfile.ZipFile(args.archive) as archive:
        for item in archive.infolist():
            if item.is_dir() or not item.filename.startswith(PREFIX):
                continue
            relative = PurePosixPath(item.filename[len(PREFIX):])
            if relative.is_absolute() or '..' in relative.parts:
                raise ValueError('Unsafe ZIP member')
            if relative.parts[0] not in ('tag', 'anchor', 'common'):
                continue
            data = archive.read(item)
            target = destination.joinpath(*relative.parts)
            target.parent.mkdir(parents=True, exist_ok=True)
            target.write_bytes(data)
            entries.append({'path': str(relative), 'bytes': len(data),
                            'sha256': hashlib.sha256(data).hexdigest()})
    manifest = {'archive_sha256': digest, 'archive_prefix': PREFIX,
                'files': entries, 'files_count': len(entries),
                'note': 'Import hashes, before single-tag changes. Preserve vendor licenses.'}
    (destination / 'UPSTREAM_SOURCE.json').write_text(
        json.dumps(manifest, indent=2, ensure_ascii=False) + '\n', encoding='utf-8')
    print(json.dumps({'imported': len(entries), 'bytes': sum(e['bytes'] for e in entries)}))


if __name__ == '__main__':
    main()
