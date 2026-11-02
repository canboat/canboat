# CZone proprietary PGN contribution research

This feature branch is based on upstream `canboat/canboat` master at `4f232faf00efc6cd07d56cb3ef7efddd216e98bb`.

Candidate PGNs: 130817 (AC), 130822 (DC), 130825 (Control X Plus). Source captures and schema validation are staged and green in [research PR #4](https://github.com/mattsmitchell/signalk-czone-circuits/pull/4), workflow [41383317492](https://github.com/mattsmitchell/signalk-czone-circuits/actions/runs/41383317492).

**Important:** The separate research workflow stages replacement YAML into a disposable upstream checkout; it has not yet transferred the candidate definitions or their generated outputs to this branch. Before submitting an upstream PR, reconcile existing 130817 semantics and preserve other BEP variants, add the capture-backed YAML, run `keel/keel check`, `make generated`, `make pr` in this fork, and commit generated artifacts.

No changes to the active Signal K CZone beta are involved.
