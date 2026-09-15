"""Resolve legacy solver frequency indices to raw RINEX observation bands.

``freq_ix0``/``freq_ix1`` are solver-facing indices into the historical
frequency table.  RINEX observation identifiers use constellation-specific
band digits instead.  The two domains intentionally have no positional or
Hz-based relationship, so this module is the single boundary between them.
"""

from collections.abc import Mapping, Sequence
from typing import Dict, List


SYSTEM_TO_RINEX = {
    "GPS": "G",
    "GLO": "R",
    "GAL": "E",
    "BDS": "C",
    "QZS": "J",
    "SBS": "S",
}
RINEX_TO_SYSTEM = {rinex: system for system, rinex in SYSTEM_TO_RINEX.items()}


# Legacy frequency-table metadata.  The key is the solver constellation name
# and the nested key is the legacy ``freq_ix`` value.  Values are *raw RINEX
# band digits*, never frequencies and never decoder slots.
#
# BDS entries follow GREAT-MSF's frequency plan (src/LibGnut/gutils/gnss.cpp:78
# orders the BDS bands B1I, B2I, B3I, ... with B1I=BAND_2 and B3I=BAND_6):
# freq_ix 0 (the old generic first slot) resolves to B1I (C2I, raw band 2)
# and freq_ix 6 (the extended-table entry used by the GREAT campus01 single
# frequency baseline) resolves to B3I (C6I, raw band 6).  Datasets that need
# a different pairing must declare ``raw_band_priority`` explicitly.
LEGACY_FREQ_BAND_METADATA = {
    "GPS": {0: 1, 1: 2, 2: 5, 3: 6, 4: 7, 5: 8},
    "GLO": {0: 1, 1: 2, 4: 1, 5: 2},
    "GAL": {0: 1, 1: 2, 2: 5, 3: 6, 4: 7},
    "BDS": {0: 2, 1: 7, 2: 5, 3: 7, 4: 6, 5: 7, 6: 6},
    "QZS": {0: 1, 1: 2, 2: 5, 3: 6},
    "SBS": {0: 1, 2: 5},
}


# Raw RINEX band digits accepted for each constellation.  Keeping this
# domain explicit makes unknown raw bands fail at configuration time rather
# than silently falling back to a column position.
RAW_BAND_DOMAIN = {
    "G": frozenset({1, 2, 5, 6, 7, 8}),
    "R": frozenset({1, 2, 3, 4, 6}),
    "E": frozenset({1, 5, 6, 7, 8}),
    "C": frozenset({1, 2, 5, 6, 7, 8}),
    "J": frozenset({1, 2, 5, 6, 7, 8}),
    "S": frozenset({1, 5}),
}


# Compatibility conversion for the current simplifier API.  It is kept here
# so sensors pass a derived normalized priority, never ``freq_ix`` itself.
# The next reader slice can consume ``resolved_raw_band_priority`` directly.
RAW_TO_NORMALIZED_BAND = {
    "G": {1: 0, 2: 1, 5: 2, 6: 4, 7: 3, 8: 4},
    "R": {1: 0, 2: 1, 3: 2, 4: 3, 6: 4},
    "E": {1: 0, 5: 2, 6: 4, 7: 3, 8: 4},
    "C": {1: 0, 2: 1, 5: 2, 6: 4, 7: 3, 8: 4},
    "J": {1: 0, 2: 1, 5: 2, 6: 4, 7: 3, 8: 4},
    "S": {1: 0, 5: 2},
}


# Decoder-facing defaults are expressed in raw RINEX band digits.  A caller
# may provide a different ordered mapping for each stream (for example the
# legacy GAL/BDS pair versus GREAT's GAL/BDS pair).
#
# The BDS order follows GREAT: B3I (C6I, raw band 6) first — the frequency
# the GREAT campus01 single-frequency baseline actually solves with — then
# B1I (C2I, raw band 2).  B1C (band 1) is BDS-3-only and absent from many
# receivers (phones included), so it is not part of the default pair.
DEFAULT_RAW_BAND_PRIORITY = {
    "G": [1, 2],
    "R": [1, 2],
    "E": [1, 7],
    "C": [6, 2],
    "J": [1, 2],
}


def _system_char(system: str) -> str:
    """Return the canonical one-character RINEX system identifier."""
    if not isinstance(system, str):
        raise ValueError(
            f"raw_band_priority system must be a string, got {system!r}"
        )
    value = system.strip().upper()
    if value in SYSTEM_TO_RINEX:
        return SYSTEM_TO_RINEX[value]
    if value in RINEX_TO_SYSTEM:
        return value
    raise ValueError(f"raw_band_priority has unknown system '{system}'")


def _validate_raw_priority(
    raw_priority, *, expected_band_count=None
) -> Dict[str, List[int]]:
    """Validate and canonicalize an explicit raw-band priority mapping."""
    if not isinstance(raw_priority, Mapping):
        raise ValueError(
            "raw_band_priority must be a mapping keyed by GNSS system"
        )

    canonical: Dict[str, List[int]] = {}
    for system, bands in raw_priority.items():
        rinex_system = _system_char(system)
        if rinex_system in canonical:
            raise ValueError(
                f"raw_band_priority has duplicate system '{rinex_system}'"
            )
        if isinstance(bands, (str, bytes)) or not isinstance(bands, Sequence):
            raise ValueError(
                f"raw_band_priority {rinex_system} must be an ordered list"
            )
        if not bands:
            raise ValueError(
                f"raw_band_priority {rinex_system} must not be empty"
            )
        if (expected_band_count is not None
                and len(bands) != expected_band_count):
            raise ValueError(
                f"raw_band_priority {rinex_system} length must match "
                f"nf={expected_band_count}: expected exactly "
                f"{expected_band_count} band(s), got {len(bands)}"
            )

        resolved_bands = []
        for band in bands:
            if isinstance(band, bool) or not isinstance(band, int):
                raise ValueError(
                    f"raw_band_priority {rinex_system} raw band must be an integer, "
                    f"got {band!r}"
                )
            if band not in RAW_BAND_DOMAIN[rinex_system]:
                raise ValueError(
                    f"raw_band_priority unknown raw band for {rinex_system}: {band}"
                )
            if band in resolved_bands:
                raise ValueError(
                    f"raw_band_priority duplicate raw band for {rinex_system}: {band}"
                )
            resolved_bands.append(band)
        canonical[rinex_system] = resolved_bands
    return canonical


def _legacy_systems(freq_ix0, freq_ix1):
    """Yield legacy system names in configuration insertion order."""
    seen = set()
    for table in (freq_ix0, freq_ix1):
        if not isinstance(table, Mapping):
            raise ValueError("freq_ix0/freq_ix1 must be mappings keyed by GNSS system")
        for system in table:
            if system not in seen:
                seen.add(system)
                yield system


def _resolve_legacy_pair(system, freq_ix0, freq_ix1) -> List[int]:
    """Resolve the configured legacy indices for one system."""
    if system not in SYSTEM_TO_RINEX:
        raise ValueError(f"cannot resolve legacy system '{system}' to a raw RINEX band")
    metadata = LEGACY_FREQ_BAND_METADATA[system]
    bands = []
    for field, table in (("freq_ix0", freq_ix0), ("freq_ix1", freq_ix1)):
        if system not in table:
            continue
        index = table[system]
        if isinstance(index, bool):
            raise ValueError(
                f"cannot resolve legacy {system} {field} {index!r} to a raw RINEX band"
            )
        try:
            is_index = int(index)
        except (TypeError, ValueError):
            raise ValueError(
                f"cannot resolve legacy {system} {field} {index!r} to a raw RINEX band"
            ) from None
        if is_index != index or is_index not in metadata:
            raise ValueError(
                f"cannot resolve legacy {system} freq_ix {index!r} to a raw RINEX band"
            )
        band = metadata[is_index]
        if band in bands:
            raise ValueError(
                f"resolved raw-band priority duplicate raw band for {system} "
                f"({SYSTEM_TO_RINEX[system]}): {band} from {field}"
            )
        bands.append(band)
    return bands


def resolve_raw_band_priority(gnss_cfg: Mapping) -> Dict[str, List[int]]:
    """Resolve a GNSS config into canonical raw RINEX band priorities.

    Explicit ``raw_band_priority`` entries are canonicalized first and take
    precedence.  Systems without an explicit entry are resolved from the
    legacy ``(system, freq_ix)`` metadata.  The result is keyed by RINEX
    system character (``E``, ``C``, ``R`` ...), with list order preserved.
    """
    if not isinstance(gnss_cfg, Mapping):
        raise ValueError("gnss configuration must be a mapping")

    freq_ix0 = gnss_cfg.get("freq_ix0", {})
    freq_ix1 = gnss_cfg.get("freq_ix1", {})
    raw_priority = gnss_cfg.get("raw_band_priority")
    if raw_priority is None:
        explicit = {}
    else:
        nf = gnss_cfg.get("nf")
        expected_band_count = None if nf is None else int(nf)
        explicit = _validate_raw_priority(
            raw_priority, expected_band_count=expected_band_count
        )
    resolved: Dict[str, List[int]] = dict(explicit)

    for system in _legacy_systems(freq_ix0, freq_ix1):
        rinex_system = _system_char(system)
        if rinex_system in resolved:
            continue
        bands = _resolve_legacy_pair(system, freq_ix0, freq_ix1)
        if bands:
            resolved[rinex_system] = bands
    return resolved


def raw_band_priority_to_simplifier_priority(
    raw_priority: Mapping[str, Sequence[int]],
) -> Dict[str, List[int]]:
    """Convert validated raw bands to the current simplifier's band ranks."""
    canonical = _validate_raw_priority(raw_priority)
    normalized = {}
    for system, bands in canonical.items():
        normalized[system] = [RAW_TO_NORMALIZED_BAND[system][band] for band in bands]
    return normalized


def raw_band_priority_to_slot_mapping(
    raw_priority: Mapping[str, Sequence[int]] = None,
) -> Dict[str, Dict[int, int]]:
    """Derive a decoder slot map from each stream's ordered raw bands.

    The first configured raw band is always decoder slot 0, the second slot 1,
    and so on.  No global constellation-specific slot table is consulted.
    """
    if raw_priority is None:
        raw_priority = DEFAULT_RAW_BAND_PRIORITY
    canonical = _validate_raw_priority(raw_priority)
    return {
        system: {band: slot for slot, band in enumerate(bands)}
        for system, bands in canonical.items()
    }
