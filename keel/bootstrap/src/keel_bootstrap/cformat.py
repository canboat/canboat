"""C printf-compatible formatting helpers.

The canboat.xml emitter must reproduce analyzer-explain.c's printf output
byte-for-byte (DESIGN.md, migration step 1), so floats are formatted with
the exact semantics of C's %g / %.15g and strings are escaped exactly like
printXML() in analyzer-explain.c.
"""

from __future__ import annotations


def c_g(value: float) -> str:
    """C printf %g (6 significant digits, trailing zeros stripped)."""
    return "%g" % value


def c_15g(value: float) -> str:
    """C printf %.15g."""
    return "%.15g" % value


def xml_escape(text: str) -> str:
    """Port of printXML(): escapes & < > " -- and deliberately NOT the
    apostrophe, even though lookup attribute values are single-quoted.
    No current name contains one; the validator must keep it that way."""
    return (
        text.replace("&", "&amp;").replace("<", "&lt;").replace(">", "&gt;").replace('"', "&quot;")
    )
