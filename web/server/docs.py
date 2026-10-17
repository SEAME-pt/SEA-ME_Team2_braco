"""List and read the Markdown files under docs/ (and the top-level READMEs)."""

from pathlib import Path

REPO = Path(__file__).resolve().parents[2]
DOCS = REPO / "docs"

# Extra Markdown files outside docs/ worth showing, as (published path, file)
EXTRA = {
    "docker/README.md": REPO / "docker" / "README.md",
    "web/README.md": REPO / "web" / "README.md",
}


def _title(path):
    try:
        with path.open(encoding="utf-8") as f:
            for line in f:
                if line.startswith("# "):
                    return line[2:].strip()
    except OSError:
        pass
    return path.stem.replace("-", " ").replace("_", " ")


def list_docs():
    items = []
    for path in sorted(DOCS.rglob("*.md")):
        rel = path.relative_to(DOCS).as_posix()
        items.append({"path": "docs/" + rel, "title": _title(path),
                      "group": rel.split("/")[0] if "/" in rel else "docs"})
    for rel, path in EXTRA.items():
        if path.is_file():
            items.append({"path": rel, "title": _title(path), "group": "outros"})
    return items


def read_doc(rel):
    """Return the Markdown of `rel`, or None if it is not a listed document."""
    if rel in EXTRA:
        path = EXTRA[rel]
    elif rel.startswith("docs/") and rel.endswith(".md"):
        path = (REPO / rel).resolve()
        if DOCS.resolve() not in path.parents:
            return None
    else:
        return None
    if not path.is_file():
        return None
    return path.read_text(encoding="utf-8")
