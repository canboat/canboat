# Security Policy

## Reporting a vulnerability

Please report security vulnerabilities privately, so a fix can be
prepared before the details are public.

Use GitHub's private vulnerability reporting for this repository:
open the [Security Advisories](https://github.com/canboat/canboat/security/advisories/new)
page and file a report. If that is unavailable to you, email
Kees Verruijt at <kees@verruijt.net> instead.

Please include:

- the affected version or commit,
- the affected file and, where you can, the line,
- a short description of the impact,
- a minimal input or steps that reproduce it.

Do not open a public issue or pull request for a vulnerability, and do
not publish the details, until a fix has been released.

## Scope

CANboat parses NMEA 2000 and J1939 data, sometimes from a network
source (for example an `analyzer` reading from a `maretron-ipg` or
`actisense` gateway over TCP). Input that a gateway or the bus can
carry is in scope: a crash, a memory-safety error, or a hang triggered
by such input is a vulnerability worth reporting.

## Supported versions

Fixes land on `master` and go out in the next release. Only the latest
release is supported; please reproduce against `master` where you can.
