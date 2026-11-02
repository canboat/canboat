# Coexistence test result — blocked by duplicate match sets

GitHub Actions run: https://github.com/mattsmitchell/canboat/actions/runs/41384777574

On 2026-11-02, the workflow staged all three CZone candidate YAML files alongside the unchanged upstream BEP variants. The staging step succeeded. `keel/keel check` failed with **6 errors and 0 warnings**.

| PGN | Keel R20 | Keel R40 |
|---|---|---|
| 130817 | Match set `[0=295,2=4]` duplicates `bepMarineCzoneStatusExtended`; later variant unreachable | Sample selects existing variant, not candidate |
| 130822 | Match set `[0=295,2=4]` duplicates `bepMarineProprietaryPgn130822`; later variant unreachable | Sample selects existing variant, not candidate |
| 130825 | Match set `[0=295,2=4]` duplicates `bepMarineProprietaryPgn130825`; later variant unreachable | Sample selects existing variant, not candidate |

**Conclusion:** Merely adding candidate definitions as additional variants cannot work. Changing `variantOrder` does not create a unique match predicate. The common `27 99` prefix is the BEP manufacturer/industry header, not a CZone-specific discriminator.

**Next investigation:** determine whether actual CZone captures expose a stable, protocol-justified discriminator (e.g. a constant page/opcode) that is not also present in other BEP devices. Do not turn dynamic module ID or page values into invented constants. Alternatively propose a maintainer-approved revision of existing BEP variants with explicit compatibility notes. Payload length alone has not been demonstrated to be supported as a variant match condition.

**Decision:** do not promote into `database/pgns/` yet. The measured 130817 AC current calibration remains valid for the observed SugarShack device. Existing BEP definitions, fork `master`, upstream, and the active Signal K beta remain untouched.
