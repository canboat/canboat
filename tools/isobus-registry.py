#!/usr/bin/env python3
"""
isobus-registry.py - scrape the official ISO 11783 / SAE J1939 manufacturer
code registry from isobus.net.

The registry is published free by VDMA at

    https://www.isobus.net/isobus/manufacturerCode

and is the same data as the paywalled SAE J1939 Digital Annex. There is no CSV
or API, but pagination is URL-driven and the table markup is regular, so it
scrapes cleanly. Wireshark's epan/dissectors/data-isobus.c mirrors the same
table in pre-parsed form, but it is GPL-2.0-or-later and therefore not a usable
source here; scrape the registry itself.

This is the J1939 side of the split described in github issue #837. Marine
manufacturer codes are a *different* allocation of the same 11-bit space,
administered by the NMEA, and live in database/lookups/MANUFACTURER_CODE.yaml.
Do not feed this output into that file.

Usage:
    tools/isobus-registry.py --yaml  > database/lookups/J1939_MANUFACTURER_CODE.yaml
    tools/isobus-registry.py --tsv   > /tmp/isobus.tsv
    tools/isobus-registry.py --cache /tmp/isobus-html --tsv

Dependencies: Python 3 standard library only, matching the rest of tools/.
"""

import argparse
import html
import os
import re
import sys
import time
import urllib.request

BASE = "https://www.isobus.net/isobus/manufacturerCode/index"
QUERY = "?ManufacturerCode_sort=value&ManufacturerCode_page={page}"
USER_AGENT = "canboat-isobus-registry/1.0 (+https://github.com/canboat/canboat)"

# <td>code</td><td>name</td><td>location</td><td class="button-column">…
ROW = re.compile(
    r"<td>(\d+)</td>\s*<td>(.*?)</td>\s*<td>(.*?)</td>\s*<td class=",
    re.S,
)


def fetch(page, cache_dir=None, delay=1.0):
    """One results page, from the cache when present."""
    if cache_dir:
        path = os.path.join(cache_dir, f"page{page:02d}.html")
        if os.path.exists(path):
            with open(path, encoding="utf-8") as fp:
                return fp.read()
    url = BASE + QUERY.format(page=page)
    req = urllib.request.Request(url, headers={"User-Agent": USER_AGENT})
    with urllib.request.urlopen(req, timeout=60) as resp:
        text = resp.read().decode("utf-8", "replace")
    if cache_dir:
        os.makedirs(cache_dir, exist_ok=True)
        with open(os.path.join(cache_dir, f"page{page:02d}.html"), "w",
                  encoding="utf-8") as fp:
            fp.write(text)
    time.sleep(delay)  # the registry is a courtesy; do not hammer it
    return text


def clean(cell):
    """Strip tags, unescape entities, collapse whitespace."""
    return re.sub(r"\s+", " ", html.unescape(re.sub(r"<[^>]*>", "", cell))).strip()


def scrape(cache_dir=None, delay=1.0, max_pages=40):
    """Every {code: (name, location)}.

    Stops on the first page that contributes no new code. Asking for a page
    past the end does not return an empty table - the site clamps the number
    and re-serves the last page - so "no rows" is the wrong stop condition and
    would spend a request per remaining page.
    """
    out = {}
    for page in range(1, max_pages + 1):
        rows = ROW.findall(fetch(page, cache_dir, delay))
        added = 0
        for code, name, location in rows:
            code = int(code)
            name, location = clean(name), clean(location)
            if not name:
                continue
            if code in out:
                if out[code][0] != name:
                    print(f"warning: code {code} listed twice: "
                          f"{out[code][0]!r} then {name!r}", file=sys.stderr)
                continue
            out[code] = (name, location)
            added += 1
        print(f"page {page}: {len(rows)} rows, {added} new ({len(out)} total)",
              file=sys.stderr)
        if not added:
            break
    return out


def yaml_scalar(s):
    """Quote only where the canonical style in database/lookups/ requires it."""
    if (s != s.strip() or s == ""
            or re.search(r"[:#]\s|\s#|^[-?&*!|>%@`\"']|[:\[\]{},]$", s)
            or s.lower() in ("yes", "no", "true", "false", "null", "~")):
        return '"' + s.replace("\\", "\\\\").replace('"', '\\"') + '"'
    return s


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    fmt = ap.add_mutually_exclusive_group(required=True)
    fmt.add_argument("--yaml", action="store_true",
                     help="emit a keel lookup file for J1939_MANUFACTURER_CODE")
    fmt.add_argument("--tsv", action="store_true",
                     help="emit code<TAB>name<TAB>location")
    ap.add_argument("--cache", metavar="DIR",
                    help="read/write the raw HTML here, so reruns need no network")
    ap.add_argument("--delay", type=float, default=1.0,
                    help="seconds between requests (default 1.0)")
    args = ap.parse_args()

    reg = scrape(args.cache, args.delay)
    if not reg:
        print("isobus-registry: no rows scraped; the page layout may have "
              "changed", file=sys.stderr)
        return 1

    if args.tsv:
        for code in sorted(reg):
            name, location = reg[code]
            print(f"{code}\t{name}\t{location}")
        return 0

    print("name: J1939_MANUFACTURER_CODE")
    print("kind: pair")
    print("bits: 11")
    print("note: Scraped from https://www.isobus.net/isobus/manufacturerCode by"
          " tools/isobus-registry.py. This is the ISO 11783 / SAE J1939"
          " allocation; the NMEA 2000 marine allocation of the same 11-bit"
          " space is MANUFACTURER_CODE. See github issue #837.")
    print("values:")
    for code in sorted(reg):
        print(f"  {code}: {yaml_scalar(reg[code][0])}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
