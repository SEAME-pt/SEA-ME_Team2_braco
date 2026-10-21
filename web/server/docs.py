"""Discover and read every Markdown file in the repository, grouped by topic."""

from pathlib import Path

REPO = Path(__file__).resolve().parents[2]

# Directories never searched (dependencies, caches, VCS data)
SKIP_DIRS = {
    '.git',
    '.venv',
    'venv',
    'node_modules',
    '__pycache__',
    '.pytest_cache',
    '.mypy_cache',
    'build',
    'install',
    'log',
}

# (group label, matcher) in display order; the first matching rule wins
GROUPS = [
    ('Getting started', lambda p: p in ('README.md', 'docs/dev-setup.md')),
    ('Safety', lambda p: p == 'docs/safety.md'),
    ('Technical reference', lambda p: p == 'docs/protocol_commands.md'),
    ('Architecture and decisions', lambda p: p.startswith('docs/ADR-')),
    ('Inventory', lambda p: p.startswith('docs/inventory/')),
    ('Web page (braco)', lambda p: p.startswith('web/')),
    ('Docker and ROS 2', lambda p: p.startswith(('docker/', 'ros2_ws/'))),
    ('Contributing', lambda p: p.startswith('.github/')),
    ('Code', lambda p: p.startswith(('libs/', 'playground/', 'tests/'))),
]
OTHER = 'Other'
ORDER = [g for g, _ in GROUPS] + [OTHER]


def _group(rel):
    return next((name for name, match in GROUPS if match(rel)), OTHER)


def _title(path):
    try:
        with path.open(encoding='utf-8') as f:
            for line in f:
                if line.startswith('# '):
                    return line[2:].strip()
    except OSError:
        pass
    return path.stem.replace('-', ' ').replace('_', ' ')


def _files():
    """Markdown files in the repo as {repo-relative posix path: Path}."""
    found = {}
    for path in sorted(REPO.rglob('*.md')):
        parts = path.relative_to(REPO).parts
        # skip dependency/cache dirs and hidden dirs other than .github
        if any(p in SKIP_DIRS or (p.startswith('.') and p != '.github') for p in parts[:-1]):
            continue
        if path.is_file():
            found['/'.join(parts)] = path
    return found


def list_docs():
    items = [
        {'path': rel, 'title': _title(path), 'group': _group(rel)}
        for rel, path in _files().items()
    ]
    items.sort(key=lambda d: (ORDER.index(d['group']), d['path'] != 'README.md', d['path']))
    return items


def read_doc(rel):
    """Return the Markdown of `rel`, or None if it is not a listed document."""
    path = _files().get(rel)
    return path.read_text(encoding='utf-8') if path else None
