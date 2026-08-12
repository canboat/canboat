#!/bin/sh
#
# Fixup XSLTPROC output to contain proper HTML.
#

sed -e '1s/.*/<!DOCTYPE HTML>/
s!\(<script[^/]*\)/>!\1></script>!
s!<span/>!<span></span>!g'
