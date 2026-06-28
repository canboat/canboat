#!/usr/bin/env python3
"""
md2html.py - a small, dependency-free Markdown to HTML converter.

Portable: uses only the Python 3 standard library, so it runs on macOS and
Linux (and Windows, though the Makefile does not require it there). It is not a
full CommonMark implementation - it supports the subset used by canboat's
research/ documents:

  - ATX headings (# .. ######) with GitHub-style slug ids for #anchor links
  - paragraphs
  - fenced code blocks (``` ... ```), optional language label
  - GitHub-flavoured tables (| a | b | with a |---|---| separator row)
  - unordered (-, *, +) and ordered (1.) lists, one level of nesting
  - blockquotes (> )
  - horizontal rules (---, ***, ___)
  - inline: `code`, **bold**, *italic*/_italic_, [text](url), bare URLs

Usage:
  python3 md2html.py INPUT.md [-o OUTPUT.html] [--title TITLE]
"""
import sys
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
{body}
</body>
</html>
"""


def slug(text):
    s = re.sub(r"<[^>]+>", "", text).lower()
    s = re.sub(r"[^\w\s-]", "", s)
    return re.sub(r"[\s_]+", "-", s).strip("-")


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
                  lambda m: '<a href="%s">%s</a>' % (html.escape(m.group(2), quote=True), m.group(1)),
                  text)
    # bare URLs
    text = re.sub(r"(?<![\"=>])(https?://[^\s<>()]+)",
                  lambda m: '<a href="%s">%s</a>' % (m.group(1), m.group(1)), text)
    text = re.sub(r"\*\*([^*]+)\*\*", r"<strong>\1</strong>", text)
    text = re.sub(r"(?<!\w)[_*]([^_*]+)[_*](?!\w)", r"<em>\1</em>", text)
    # restore code spans
    text = re.sub(r"\x00(\d+)\x00", lambda m: "<code>%s</code>" % spans[int(m.group(1))], text)
    return text


def render_table(rows):
    cells = lambda row: [c.strip() for c in row.strip().strip("|").split("|")]
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


def convert(md):
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
            out.append("<blockquote>\n%s\n</blockquote>" % convert("\n".join(buf)))
            continue
        # lists
        if re.match(r"^\s*([-*+]|\d+\.)\s+", line):
            ordered = bool(re.match(r"^\s*\d+\.\s+", line))
            tag = "ol" if ordered else "ul"
            items = []
            while i < n and re.match(r"^\s*([-*+]|\d+\.)\s+", lines[i]):
                item = re.sub(r"^\s*([-*+]|\d+\.)\s+", "", lines[i])
                # gather continuation/indented sub-lines into the same item
                i += 1
                while i < n and lines[i].strip() and not re.match(r"^\s*([-*+]|\d+\.)\s+", lines[i]) \
                        and lines[i].startswith((" ", "\t")):
                    item += " " + lines[i].strip()
                    i += 1
                items.append("<li>%s</li>" % inline(item))
            out.append("<%s>\n%s\n</%s>" % (tag, "\n".join(items), tag))
            continue
        # paragraph: gather until blank or block start
        buf = [line]
        i += 1
        while i < n and lines[i].strip() and not re.match(
                r"^(#{1,6}\s|```|\s*([-*+]|\d+\.)\s|\s*>|\s*([-*_])(\s*\3){2,}\s*$)", lines[i]) \
                and not ("|" in lines[i] and i + 1 < n and is_table_sep(lines[i + 1])):
            buf.append(lines[i])
            i += 1
        out.append("<p>%s</p>" % inline(" ".join(s.strip() for s in buf)))
    return "\n".join(out)


def main():
    ap = argparse.ArgumentParser(description="Convert Markdown to a standalone HTML page.")
    ap.add_argument("input")
    ap.add_argument("-o", "--output", help="output file (default: stdout)")
    ap.add_argument("--title", help="page title (default: first heading or file name)")
    args = ap.parse_args()

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
