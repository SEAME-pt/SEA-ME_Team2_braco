"""Sprint data from the GitHub project: milestones, SPRINT issues, sub-issues, merged PRs.

Uses only the standard library. Results are cached for CACHE_SECONDS so the
page stays well under the unauthenticated limit of 60 requests per hour.
Set GITHUB_TOKEN to raise the limit (or if the repository becomes private).
"""

import json
import os
import re
import threading
import time
import urllib.error
import urllib.request

REPO = os.environ.get("GITHUB_REPO", "SEAME-pt/SEA-ME_Team2_braco")
API = "https://api.github.com/repos/" + REPO
CACHE_SECONDS = 300

_cache = {"time": 0.0, "data": None}
_lock = threading.Lock()


def _get(url):
    req = urllib.request.Request(url, headers={
        "Accept": "application/vnd.github+json",
        "User-Agent": "braco-dashboard",
    })
    token = os.environ.get("GITHUB_TOKEN")
    if token:
        req.add_header("Authorization", "Bearer " + token)
    with urllib.request.urlopen(req, timeout=10) as resp:
        return json.loads(resp.read().decode("utf-8"))


def _get_all(url):
    items, page = [], 1
    while True:
        sep = "&" if "?" in url else "?"
        batch = _get("{}{}per_page=100&page={}".format(url, sep, page))
        items.extend(batch)
        if len(batch) < 100:
            return items
        page += 1


def _issue(raw):
    return {
        "number": raw["number"],
        "title": raw["title"],
        "state": raw["state"],
        "url": raw["html_url"],
        "assignees": [a["login"] for a in raw.get("assignees") or []],
        "labels": [l["name"] for l in raw.get("labels") or []],
        "milestone": (raw.get("milestone") or {}).get("number"),
        "created_at": raw.get("created_at"),
        "closed_at": raw.get("closed_at"),
    }


def _pull(raw):
    return {
        "number": raw["number"],
        "title": raw["title"],
        "url": raw["html_url"],
        "author": (raw.get("user") or {}).get("login"),
        "labels": [l["name"] for l in raw.get("labels") or []],
        "milestone": (raw.get("milestone") or {}).get("number"),
        "merged_at": raw["merged_at"],
    }


SPRINT_RE = re.compile(r"^\s*SPRINT\s+(\d+)\s*\|\s*(.*)$")


def _fetch():
    milestones = [{
        "number": m["number"],
        "title": m["title"],
        "description": m.get("description") or "",
        "state": m["state"],
        "due_on": m.get("due_on"),
        "open": m["open_issues"],
        "closed": m["closed_issues"],
        "url": m["html_url"],
    } for m in _get_all(API + "/milestones?state=all")]
    milestones.sort(key=lambda m: m["number"])

    issues = [i for i in _get_all(API + "/issues?state=all") if "pull_request" not in i]

    # Merged pull requests are the "evolutions" shown on each sprint page
    pulls = [_pull(p) for p in _get_all(API + "/pulls?state=closed") if p.get("merged_at")]
    pulls.sort(key=lambda p: p["merged_at"])

    sprints = []
    for raw in issues:
        match = SPRINT_RE.match(raw["title"])
        if not match:
            continue
        sprint = _issue(raw)
        sprint["sprint"] = int(match.group(1))
        sprint["name"] = match.group(2).strip()
        sprint["body"] = raw.get("body") or ""
        sprint["milestone_title"] = (raw.get("milestone") or {}).get("title")
        try:
            subs = _get_all("{}/issues/{}/sub_issues".format(API, raw["number"]))
            sprint["sub_issues"] = [_issue(s) for s in subs]
        except urllib.error.HTTPError:
            sprint["sub_issues"] = []
        sprints.append(sprint)
    sprints.sort(key=lambda s: s["sprint"])

    return {
        "repo": REPO,
        "fetched": time.time(),
        "milestones": milestones,
        "sprints": sprints,
        "issues": [_issue(i) for i in issues],
        "pulls": pulls,
    }


def sprints(force=False):
    """Cached sprint data. On a GitHub error, return the last good copy with `error` set."""
    with _lock:
        fresh = time.time() - _cache["time"] < CACHE_SECONDS
        if _cache["data"] is not None and fresh and not force:
            return _cache["data"]
        try:
            data = _fetch()
            _cache.update(time=time.time(), data=data)
            return data
        except Exception as e:
            if _cache["data"] is not None:
                return dict(_cache["data"], error=str(e))
            return {"repo": REPO, "error": str(e), "milestones": [], "sprints": [], "issues": [], "pulls": []}
