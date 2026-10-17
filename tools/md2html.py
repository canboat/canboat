#!/usr/bin/env python3
"""
md2html.py - a small, dependency-free Markdown to HTML converter.

Portable: uses only the Python 3 standard library, so it runs on macOS and
Linux (and Windows, though the Makefile does not require it there). It is not a
full CommonMark implementation - it supports the subset used by canboat's
doc/ and research/ documents, which `make markdown-docs` turns into the
GitHub Pages site in docs/:

  - ATX headings (# .. ######) with GitHub-style slug ids for #anchor links
  - paragraphs
  - fenced code blocks (``` ... ```), optional language label
  - GitHub-flavoured tables (| a | b | with a |---|---| separator row)
  - unordered (-, *, +) and ordered (1.) lists, nested by indentation, with
    paragraphs of their own after a blank line
  - blockquotes (> ), and GitHub's alerts (> [!NOTE], > [!WARNING], ...)
  - horizontal rules (---, ***, ___)
  - inline: `code`, **bold**, *italic*/_italic_, [text](url), bare URLs
  - in a table cell, `\\|` for a | that does not end the cell

A relative link is made to work on the site: to a .md file that is also
converted (doc/*.md, research/*.md), it goes to that page; to a file in docs/,
to that file; to anything else in the repository, to the file on GitHub.

Usage:
  python3 md2html.py INPUT.md [-o OUTPUT.html] [--title TITLE]
"""
import sys
import os
import re
import argparse
import html

STYLE = """
:root { color-scheme: light dark; }
body { font-family: -apple-system, Segoe UI, Roboto, Helvetica, Arial, sans-serif;
       line-height: 1.55; max-width: 50rem; margin: 2rem auto; padding: 0 1rem;
       color: #1a1a1a; background: #fff; }
@media (prefers-color-scheme: dark) { body { color: #ddd; background: #1a1a1a; } }
h1, h2, h3, h4 { line-height: 1.25; margin-top: 1.6em; }
h1 { border-bottom: 2px solid #8884; padding-bottom: .2em; }
h2 { border-bottom: 1px solid #8884; padding-bottom: .2em; }
code { font-family: ui-monospace, SFMono-Regular, Menlo, Consolas, monospace;
       background: #8881; padding: .1em .3em; border-radius: 4px; font-size: .9em; }
pre { background: #8881; padding: .8em 1em; border-radius: 6px; overflow-x: auto; }
pre code { background: none; padding: 0; }
table { border-collapse: collapse; margin: 1em 0; display: block; overflow-x: auto; }
th, td { border: 1px solid #8886; padding: .35em .7em; text-align: left; }
th { background: #8882; }
blockquote { border-left: 4px solid #8884; margin: 1em 0; padding: .2em 1em; color: #666; }
a { color: #1a73e8; }
nav { font-size: .9em; margin-bottom: 1rem; }
.alert { color: inherit; }
.alert-title { font-weight: bold; margin-bottom: .3em; }
.alert-note, .alert-tip { border-left-color: #1a73e8; }
.alert-important { border-left-color: #8250df; }
.alert-warning { border-left-color: #bf8700; }
.alert-caution { border-left-color: #cf222e; }
""".strip()

PAGE = """<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>{title}</title>
<style>
{style}
</style>
</head>
<body>
<nav><a href="index.html">CANboat documentation</a></nav>
{body}
</body>
</html>
"""


def slug(text):
    s = re.sub(r"<[^>]+>", "", text).lower()
    s = re.sub(r"[^\w\s-]", "", s)
    return re.sub(r"[\s_]+", "-", s).strip("-")


REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
GITHUB = "https://github.com/canboat/canboat/blob/master/"
CONVERTED = ("doc", "research")  # their *.md become docs/*.html
SOURCE_DIR = ""  # the input's directory, relative to REPO; set by main()


def site_link(url):
    """Where a link in a converted document goes on the site."""
    if re.match(r"^[a-z][a-z0-9+.-]*:", url, re.I) or url.startswith(("#", "/")):
        return url
    path, frag = (url.split("#", 1) + [""])[:2]
    frag = "#" + frag if frag else ""
    target = os.path.normpath(os.path.join(SOURCE_DIR, path)).replace(os.sep, "/")
    top, _, rest = target.partition("/")
    if top in CONVERTED and "/" not in rest and rest.endswith(".md"):
        return rest[:-3] + ".html" + frag
    if top == "docs" and rest:
        return rest + frag
    if target.startswith(".."):
        return url
    if os.path.isdir(os.path.join(REPO, target)):
        return GITHUB.replace("/blob/", "/tree/") + target.rstrip("/") + frag
    return GITHUB + target + frag


def inline(text):
    """Render inline markdown to HTML on a single line of source text."""
    # Pull out code spans first so their contents are not reformatted.
    spans = []

    def stash(m):
        spans.append(html.escape(m.group(1)))
        return "\x00%d\x00" % (len(spans) - 1)

    text = re.sub(r"`([^`]+)`", stash, text)
    text = html.escape(text, quote=False)
    # [text](url)
    text = re.sub(r"\[([^\]]+)\]\(([^)\s]+)\)",
                  lambda m: '<a href="%s">%s</a>' % (html.escape(site_link(m.group(2)), quote=True), m.group(1)),
                  text)
    # bare URLs
    text = re.sub(r"(?<![\"=>])(https?://[^\s<>()]+)",
                  lambda m: '<a href="%s">%s</a>' % (m.group(1), m.group(1)), text)
    text = re.sub(r"\*\*(.+?)\*\*", r"<strong>\1</strong>", text)
    text = re.sub(r"(?<!\w)[_*]([^_*]+)[_*](?!\w)", r"<em>\1</em>", text)
    # restore code spans
    text = re.sub(r"\x00(\d+)\x00", lambda m: "<code>%s</code>" % spans[int(m.group(1))], text)
    return text


def split_cells(row):
    """A table row's cells: a | in a code span or written \\| is no divider."""
    row = row.strip()
    if row.startswith("|"):
        row = row[1:]
    if row.endswith("|") and not row.endswith("\\|"):
        row = row[:-1]
    cells, cur, code, i = [], "", False, 0
    while i < len(row):
        c = row[i]
        if c == "\\" and row[i + 1:i + 2] == "|":
            cur += "|"
            i += 2
            continue
        if c == "`":
            code = not code
        if c == "|" and not code:
            cells.append(cur.strip())
            cur = ""
        else:
            cur += c
        i += 1
    cells.append(cur.strip())
    return cells


def render_table(rows):
    cells = split_cells
    head = cells(rows[0])
    out = ["<table>", "<thead><tr>"]
    out += ["<th>%s</th>" % inline(c) for c in head]
    out += ["</tr></thead>", "<tbody>"]
    for row in rows[2:]:
        out.append("<tr>" + "".join("<td>%s</td>" % inline(c) for c in cells(row)) + "</tr>")
    out += ["</tbody>", "</table>"]
    return "\n".join(out)


def is_table_sep(line):
    return bool(re.match(r"^\s*\|?[\s:|-]*-[\s:|-]*\|?\s*$", line)) and "-" in line


LIST_RE = re.compile(r"^(\s*)([-*+]|\d+\.)\s+")
BLOCK_START = re.compile(r"^(#{1,6}\s|```|\s*>|\s*([-*_])(\s*\2){2,}\s*$)")


def indent(line):
    return len(line) - len(line.lstrip(" "))


def render_list(lines, i):
    """The list starting at lines[i]: its HTML, and the line after it.

    An item holds every line indented past its marker, blank lines included
    when the item goes on after them, so a nested list or a second paragraph
    stays in it. Its contents are converted as a document of their own."""
    n = len(lines)
    first = LIST_RE.match(lines[i])
    base, ordered = len(first.group(1)), first.group(2)[0].isdigit()
    items = []
    while i < n:
        m = LIST_RE.match(lines[i])
        if not m or len(m.group(1)) != base or m.group(2)[0].isdigit() != ordered:
            break
        body, loose = [lines[i][m.end():]], False
        i += 1
        while i < n:
            line = lines[i]
            if not line.strip():
                j = i
                while j < n and not lines[j].strip():
                    j += 1
                if j < n and indent(lines[j]) > base:
                    body += [""] * (j - i)
                    loose = True
                    i = j
                    continue
                if j < n and LIST_RE.match(lines[j]) and indent(lines[j]) == base:
                    i = j  # the next item, after a blank line
                break
            if indent(line) > base:
                body.append(line[min(indent(line), m.end()):])
                i += 1
            elif not LIST_RE.match(line) and not BLOCK_START.match(line) and body[-1].strip():
                body.append(line.strip())  # a lazy continuation line
                i += 1
            else:
                break
        items.append("<li>%s</li>" % convert("\n".join(body), tight=not loose))
    tag = "ol" if ordered else "ul"
    return "<%s>\n%s\n</%s>" % (tag, "\n".join(items), tag), i


def convert(md, tight=False):
    """Markdown to HTML; `tight` leaves paragraphs unwrapped, as in a list item."""
    lines = md.replace("\r\n", "\n").split("\n")
    out = []
    i, n = 0, len(lines)
    while i < n:
        line = lines[i]
        # fenced code
        m = re.match(r"^```\s*(\w*)\s*$", line)
        if m:
            i += 1
            buf = []
            while i < n and not lines[i].startswith("```"):
                buf.append(html.escape(lines[i]))
                i += 1
            i += 1  # closing fence
            cls = ' class="language-%s"' % m.group(1) if m.group(1) else ""
            out.append("<pre><code%s>%s</code></pre>" % (cls, "\n".join(buf)))
            continue
        # blank
        if line.strip() == "":
            i += 1
            continue
        # heading
        m = re.match(r"^(#{1,6})\s+(.*?)\s*#*$", line)
        if m:
            lvl = len(m.group(1))
            txt = inline(m.group(2))
            out.append('<h%d id="%s">%s</h%d>' % (lvl, slug(m.group(2)), txt, lvl))
            i += 1
            continue
        # horizontal rule
        if re.match(r"^\s*([-*_])(\s*\1){2,}\s*$", line):
            out.append("<hr>")
            i += 1
            continue
        # table (current line has |, next line is a separator)
        if "|" in line and i + 1 < n and is_table_sep(lines[i + 1]):
            tbl = [line, lines[i + 1]]
            i += 2
            while i < n and "|" in lines[i] and lines[i].strip():
                tbl.append(lines[i])
                i += 1
            out.append(render_table(tbl))
            continue
        # blockquote
        if line.lstrip().startswith(">"):
            buf = []
            while i < n and lines[i].lstrip().startswith(">"):
                buf.append(re.sub(r"^\s*>\s?", "", lines[i]))
                i += 1
            alert = re.match(r"^\[!(NOTE|TIP|IMPORTANT|WARNING|CAUTION)\]\s*$", buf[0]) if buf else None
            if alert:
                kind = alert.group(1).lower()
                out.append('<blockquote class="alert alert-%s">\n<p class="alert-title">%s</p>\n%s\n</blockquote>'
                           % (kind, kind.capitalize(), convert("\n".join(buf[1:]))))
            else:
                out.append("<blockquote>\n%s\n</blockquote>" % convert("\n".join(buf)))
            continue
        # lists
        if LIST_RE.match(line):
            html_list, i = render_list(lines, i)
            out.append(html_list)
            continue
        # paragraph: gather until blank or block start
        buf = [line]
        i += 1
        while i < n and lines[i].strip() and not re.match(
                r"^(#{1,6}\s|```|\s*([-*+]|\d+\.)\s|\s*>|\s*([-*_])(\s*\3){2,}\s*$)", lines[i]) \
                and not ("|" in lines[i] and i + 1 < n and is_table_sep(lines[i + 1])):
            buf.append(lines[i])
            i += 1
        text = inline(" ".join(s.strip() for s in buf))
        out.append(text if tight else "<p>%s</p>" % text)
    return "\n".join(out)


def main():
    ap = argparse.ArgumentParser(description="Convert Markdown to a standalone HTML page.")
    ap.add_argument("input")
    ap.add_argument("-o", "--output", help="output file (default: stdout)")
    ap.add_argument("--title", help="page title (default: first heading or file name)")
    args = ap.parse_args()

    global SOURCE_DIR
    SOURCE_DIR = os.path.relpath(os.path.dirname(os.path.abspath(args.input)), REPO)

    with open(args.input, "r", encoding="utf-8") as f:
        md = f.read()
    body = convert(md)
    title = args.title
    if not title:
        m = re.search(r"^#\s+(.*)", md, re.M)
        title = m.group(1).strip() if m else args.input
    page = PAGE.format(title=html.escape(title), style=STYLE, body=body)
    if args.output:
        with open(args.output, "w", encoding="utf-8") as f:
            f.write(page)
    else:
        sys.stdout.write(page)


if __name__ == "__main__":
    main()
