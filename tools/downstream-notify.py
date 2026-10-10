#!/usr/bin/env python3
#
# Copyright (C) 2009-2026, Kees Verruijt, Harlingen, The Netherlands.
#
# Tell downstream projects about a new CANboat release.
#
# The projects to tell are listed in .github/downstream-projects.toml, which
# only maintainers change: a project asks with an issue, a maintainer checks
# the request and adds it there. When a release is tagged, this script opens an
# issue in every listed repository that asked for that kind of release (major,
# or major and minor; patch releases notify nobody). The issue summarises what
# changed in the published databases, as tools/contract.py classifies it, and
# links the release notes.
#
# A repository that already has an issue for this version is skipped, so the
# script can be re-run safely.
#
# Usage:
#   tools/downstream-notify.py --tag v8.4.0 [--previous v8.3.0] [--dry-run]
#                              [--projects .github/downstream-projects.toml]
#
# Needs git (with the tags fetched) and the GitHub CLI `gh`. Opening issues in
# other repositories needs a classic token with the public_repo scope in
# GH_TOKEN; --dry-run only reads.
#
# Pure Python 3 standard library (matches tools/contract.py); 3.11 or later,
# for tomllib.

import argparse
import json
import os
import re
import subprocess
import sys
import tempfile
import tomllib

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import contract  # noqa: E402

PROJECTS = os.path.join(HERE, "..", ".github", "downstream-projects.toml")
REPO_NAME_RE = re.compile(r"^[A-Za-z0-9_.-]+/[A-Za-z0-9_.-]+$")
CONTRACTS = (
    ("NMEA 2000", "docs/canboat.json"),
    ("SAE J1939", "docs/canboat-j1939.json"),
    ("Quick PCS", "docs/canboat-quick.json"),
)
# GitHub refuses issue bodies over 65536 characters; stay well clear.
MAX_BODY = 60000
MAX_ITEMS = 40

VERSION_RE = re.compile(r"^v(\d+)\.(\d+)\.(\d+)$")


def run(cmd, check=True):
    r = subprocess.run(cmd, capture_output=True, text=True)
    if check and r.returncode != 0:
        raise RuntimeError("%s failed: %s" % (" ".join(cmd), r.stderr.strip()))
    return r


# --------------------------------------------------------------------------- #
# Versions
# --------------------------------------------------------------------------- #

def parse_version(tag):
    m = VERSION_RE.match(tag)
    return tuple(int(x) for x in m.groups()) if m else None


def previous_release(tag):
    """The highest final release tag (vX.Y.Z, no pre-release) below `tag`."""
    this = parse_version(tag)
    tags = run(["git", "tag", "-l", "v*"]).stdout.split()
    older = [(parse_version(t), t) for t in tags if parse_version(t) and parse_version(t) < this]
    return max(older)[1] if older else None


def release_level(tag, previous):
    new, old = parse_version(tag), parse_version(previous)
    if new[0] != old[0]:
        return "major"
    if new[1] != old[1]:
        return "minor"
    return "patch"


# --------------------------------------------------------------------------- #
# Projects
# --------------------------------------------------------------------------- #

def load_projects(path):
    """[(repo, wants)] from the projects file; `wants` is "major" (major
    releases only) or "minor" (major and minor). Raises ValueError on an entry
    that is not well formed, so a typo fails the run instead of silently
    dropping a project."""
    with open(path, "rb") as fh:
        data = tomllib.load(fh)
    unknown = set(data) - {"project"}
    if unknown:
        raise ValueError("%s: unknown top-level %s (projects are [[project]] tables)"
                         % (path, ", ".join(sorted(unknown))))
    out = []
    for n, p in enumerate(data.get("project", []), 1):
        unknown = set(p) - {"repo", "releases", "request"}
        if unknown:
            raise ValueError("%s: project %d: unknown key(s) %s" % (path, n, ", ".join(sorted(unknown))))
        repo, wants = p.get("repo", ""), p.get("releases", "")
        if not REPO_NAME_RE.match(repo):
            raise ValueError("%s: project %d: repo %r is not owner/repo" % (path, n, repo))
        if wants not in ("major", "minor"):
            raise ValueError("%s: %s: releases must be \"major\" or \"minor\", not %r"
                             % (path, repo, wants))
        out.append((repo, wants))
    return out


def wants_release(wants, level):
    return level == "major" and wants in ("major", "minor") or level == "minor" and wants == "minor"


# --------------------------------------------------------------------------- #
# What changed
# --------------------------------------------------------------------------- #

def contract_at(ref, path):
    r = run(["git", "show", "%s:%s" % (ref, path)], check=False)
    return json.loads(r.stdout) if r.returncode == 0 else None


def database_changes(previous, tag):
    """Markdown summarising the contract changes between two releases."""
    parts = []
    for name, path in CONTRACTS:
        old, new = contract_at(previous, path), contract_at(tag, path)
        if new is None:
            continue
        if old is None:
            parts.append("**%s** (`%s`) is new in this release." % (name, path))
            continue
        changes = contract.diff_signatures(contract.extract_signature(old), contract.extract_signature(new))
        changes = [c for c in changes if c.severity != "cosmetic"]
        if not changes:
            continue
        lines = ["**%s** (`%s`):" % (name, path), ""]
        for severity, heading in (
            ("breaking", "Breaking"),
            ("minor", "Changed (re-check decoders and fixtures)"),
            ("additive", "Added"),
        ):
            group = [c for c in changes if c.severity == severity]
            if not group:
                continue
            lines.append("- %s: %d" % (heading, len(group)))
            for c in group[:MAX_ITEMS]:
                lines.append("  - %s: %s" % (c.subject, c.detail))
            if len(group) > MAX_ITEMS:
                lines.append("  - … and %d more" % (len(group) - MAX_ITEMS))
        parts.append("\n".join(lines))
    if not parts:
        return "No changes to the published databases beyond descriptions and comments."
    return "\n\n".join(parts)


def release_notes(canboat_repo, tag):
    r = run(["gh", "release", "view", tag, "-R", canboat_repo, "--json", "body,url"], check=False)
    if r.returncode != 0:
        return None, "https://github.com/%s/releases/tag/%s" % (canboat_repo, tag)
    data = json.loads(r.stdout)
    return data.get("body") or None, data["url"]


def issue_title(tag, level):
    return "CANboat %s released (%s release)" % (tag, level)


def issue_body(canboat_repo, tag, previous, level, changes, notes, notes_url):
    head = [
        "CANboat [%s](%s) is out, a **%s** release (previous release: %s)."
        % (tag, notes_url, level, previous),
        "",
        "You are getting this because this repository signed up for CANboat release "
        "notifications.",
        "",
        "## What changed in the databases",
        "",
        changes,
    ]
    tail = [
        "",
        "---",
        "To change which releases you hear about, or to stop, comment here or open an "
        "issue in %s. This issue was opened by CANboat's release workflow; feel free "
        "to close it once you have updated." % canboat_repo,
    ]
    # The database summary may take at most half the issue; the release notes
    # get what is left, and are left out when that is too little to be useful.
    more = "\n\n… (truncated; see the release notes)"
    if len(changes) > MAX_BODY // 2:
        head[-1] = changes[: MAX_BODY // 2] + more
    body = "\n".join(head)
    tail = "\n".join(tail)
    if notes:
        room = MAX_BODY - len(body) - len(tail) - len("\n\n## Release notes\n\n") - len(more)
        if room >= 500:
            notes = notes if len(notes) <= room else notes[:room] + more
            body += "\n\n## Release notes\n\n" + notes
    return body + tail


# --------------------------------------------------------------------------- #
# Main
# --------------------------------------------------------------------------- #

def already_notified(repo, title):
    """True or False, or None when the search failed: then nobody knows, and
    opening another issue could make a duplicate."""
    r = run(["gh", "issue", "list", "-R", repo, "--state", "all", "--search", "%s in:title" % title,
             "--json", "title", "--jq", ".[].title"], check=False)
    if r.returncode != 0:
        return None
    return title in r.stdout.splitlines()


def main(argv=None):
    p = argparse.ArgumentParser(description="Tell downstream projects about a CANboat release.")
    p.add_argument("--tag", required=True, help="the release tag, vX.Y.Z")
    p.add_argument("--previous", help="the release before it (default: the previous vX.Y.Z tag)")
    p.add_argument("--repo", default="canboat/canboat", help="the CANboat repository")
    p.add_argument("--projects", default=PROJECTS, help="the projects file (default: %(default)s)")
    p.add_argument("--dry-run", action="store_true", help="print what would be done; write nothing")
    args = p.parse_args(argv)

    if not parse_version(args.tag):
        print("%s is not a final release tag (vX.Y.Z); nothing to do." % args.tag)
        return 0
    previous = args.previous or previous_release(args.tag)
    if not previous:
        print("No release before %s; nothing to compare with." % args.tag)
        return 0
    level = release_level(args.tag, previous)
    print("%s after %s: a %s release." % (args.tag, previous, level))
    if level == "patch":
        print("Patch releases notify nobody.")
        return 0

    projects = load_projects(args.projects)
    targets = [repo for repo, wants in projects if wants_release(wants, level)]
    summary = []
    print("%d project(s), %d of them for a %s release." % (len(projects), len(targets), level))

    changes = database_changes(previous, args.tag) if targets else ""
    notes, notes_url = release_notes(args.repo, args.tag) if targets else (None, "")
    title = issue_title(args.tag, level)
    failed = 0
    for repo in targets:
        notified = already_notified(repo, title)
        if notified is None:
            failed += 1
            print("%s: could not search its issues; not sent, to avoid a duplicate" % repo)
            summary.append("| %s | failed: could not check for an existing issue |" % repo)
            continue
        if notified:
            print("%s: already has \"%s\", skipped" % (repo, title))
            summary.append("| %s | already notified |" % repo)
            continue
        body = issue_body(args.repo, args.tag, previous, level, changes, notes, notes_url)
        if args.dry_run:
            print("\n=== would open in %s: %s\n%s" % (repo, title, body))
            summary.append("| %s | dry run |" % repo)
            continue
        with tempfile.NamedTemporaryFile("w", suffix=".md", delete=False) as fh:
            fh.write(body)
        r = run(["gh", "issue", "create", "-R", repo, "--title", title, "--body-file", fh.name], check=False)
        os.unlink(fh.name)
        if r.returncode != 0:
            failed += 1
            print("%s: FAILED: %s" % (repo, r.stderr.strip()))
            summary.append("| %s | failed: %s |" % (repo, r.stderr.strip()[:120].replace("|", "/")))
            continue
        url = r.stdout.strip()
        print("%s: %s" % (repo, url))
        summary.append("| %s | %s |" % (repo, url))

    step_summary = os.environ.get("GITHUB_STEP_SUMMARY")
    if step_summary:
        with open(step_summary, "a", encoding="utf-8") as fh:
            fh.write("## Downstream notifications for %s (%s release)\n\n" % (args.tag, level))
            fh.write("| Repository | Result |\n|---|---|\n")
            fh.write("\n".join(summary) + "\n")
    return 1 if failed else 0


if __name__ == "__main__":
    sys.exit(main())
