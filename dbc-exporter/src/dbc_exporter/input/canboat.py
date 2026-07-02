"""
Canboat Json file reader and parser
"""

import datetime
import json
import logging
from typing import Dict, List, Set
import warnings

from cantools.database import Database
from cantools.database import Message
from cantools.database import Signal
from cantools.database.conversion import BaseConversion


def get_unique_id(id_: str, reserved_ids: Set[str]) -> str:
    """
    Returns a unique ID for a field.
    """
    if id_ not in reserved_ids:
        reserved_ids.add(id_)
        return id_

    i = 1
    while True:
        new_id = f"{id_}_{i}"
        if new_id not in reserved_ids:
            reserved_ids.add(new_id)
            return new_id
        i += 1


class Field:
    def __init__(self, field_dict: dict, lookups: dict, reserved_field_ids: set):
        self.order: int = field_dict["Order"]
        self.id: str = get_unique_id(field_dict["Id"], reserved_field_ids)
        self.name: str = field_dict["Name"]
        self.description: str = (
            field_dict["Description"] if "Description" in field_dict else None
        )
        self.bit_length: int = field_dict["BitLength"]
        self.bit_offset: int = field_dict["BitOffset"]
        self.bit_start: int = field_dict["BitStart"]
        self.unit: str = field_dict["Unit"] if "Unit" in field_dict else None
        self.type: str = field_dict["FieldType"] if "FieldType" in field_dict else None
        self.resolution: float = (
            field_dict["Resolution"] if "Resolution" in field_dict else 1
        )
        self.signed: bool = field_dict["Signed"] if "Signed" in field_dict else None

        lookup = field_dict["LookupEnumeration"] if "LookupEnumeration" in field_dict else None
        if lookup in lookups:
            self.enum_values: Dict[int, str] = {
                d["Value"]: d["Name"] for d in lookups[lookup]
            }
        else:
            self.enum_values = None

        self.multiplexer: bool = False
        self.multiplexer_ids: List[int] = None


class CanboatReader:
    def __init__(self, input_file) -> None:
        self._input_file = input_file
        self._json_data = json.load(self._input_file)

    def get_database(self):
        """
        Return a cantools Database object
        """

        # get current date and time in ISO 8601 format
        date_str = datetime.datetime.now().isoformat()

        version_str = f"Canboat export {date_str}"

        return Database(messages=list(self.messages), version=version_str, strict=True)

    @property
    def messages(self):
        """
        Generator for cantools Message objects
        """
        pgns = {}
        for pgn in self._json_data["PGNs"]:
            if "Fallback" in pgn:
                continue
            else:
                pgn_number = pgn["PGN"]
                pgns.setdefault(pgn_number, []).append(pgn)

        lookups = {}
        for lookup in self._json_data["LookupEnumerations"]:
            lookups[lookup["Name"]] = lookup["EnumValues"]

        for pgn_number, variants in pgns.items():
            if len(variants) > 1:
                msg = self._variants_to_multiplexed_message(pgn_number, variants, lookups)
                if msg is not None:
                    yield msg
                elif any(v["Type"] == "Fast" for v in variants):
                    # cannot multiplex, but it is a fast packet: emit the opaque representation
                    yield self._generic_fast_message(pgn_number, variants, lookups)
                else:
                    logging.warning("Could not multiplex PGN %d, ignoring", pgn_number)
                    continue
            else:
                msg = self._json_to_message(variants[0], lookups)
                if msg is not None:
                    yield msg
                else:
                    logging.warning("Could not process PGN %d, ignoring", pgn_number)
                    continue

    def _json_to_message(self, json_data: dict, lookups: dict) -> Message:
        """
        Convert a canboat json message to a cantools Message
        """

        logging.debug("Processing message %s", json_data["Id"])

        pgn_number = json_data["PGN"]

        raw_id = json_data["Id"]
        pgn_id = f"PGN_{pgn_number}_{raw_id}"

        logging.debug("Processing PGN %d (%s)", pgn_number, pgn_id)

        pgn_type = json_data["Type"] if "Type" in json_data else None

        pgn_length = json_data["Length" if "Length" in json_data else "MinLength"]

        if pgn_type and pgn_type == "ISO":
            logging.warning(
                "PGN %d %s: ISO packets not yet supported",
                pgn_number,
                pgn_id,
            )
            return None

        if (pgn_type and pgn_type == "Fast") or (pgn_length > 8):
            return self._fast_to_message(json_data, lookups)

        # fall back to single

        return self._single_to_message(json_data, lookups)

    def _fast_to_message(self, json_data, lookups):
        """
        Convert a fast packet PGN to a cantools Message
        """
        pgn_number = json_data["PGN"]
        return self._opaque_fast_message(
            pgn_number,
            f"PGN_{pgn_number}_{json_data['Id']}",
            json_data["Description"],
            json_data["Length" if "Length" in json_data else "MinLength"],
            lookups,
        )

    def _generic_fast_message(self, pgn_number, variants, lookups):
        """
        Multi-variant fast packet PGNs (proprietary media, database and autopilot messages)
        span several CAN frames and cannot be field-mapped, nor multiplexed (the manufacturer
        header is not at a fixed position once the sequence counter is prepended). Represent the
        PGN with the same opaque layout used for a single fast packet so it is at least present.
        """
        length = max(v["Length" if "Length" in v else "MinLength"] for v in variants)
        return self._opaque_fast_message(
            pgn_number, f"PGN_{pgn_number}_fast", "Fast packet (multiple variants)", length, lookups
        )

    def _opaque_fast_message(self, pgn_number, pgn_id, description, length, lookups):
        """
        Build the generic "sequence counter + opaque data" message used for fast packets, which
        DBC cannot otherwise represent.
        """
        first_field = {
            "Order": 1,
            "Id": "fastPacketSequenceCounter",
            "Name": "Fast packet sequence counter",
            "BitLength": 8,
            "BitOffset": 0,
            "BitStart": 0,
            "FieldType": "NUMBER",
            "Signed": False,
        }

        data_field = {
            "Order": 1,
            "Id": "fastPacketData",
            "Name": "Fast packet data",
            "BitLength": 56,
            "BitOffset": 8,
            "BitStart": 0,
            "Signed": False,
        }

        reserved_field_ids = set()
        signals = [
            self._field_to_signal(Field(first_field, lookups, reserved_field_ids)),
            self._field_to_signal(Field(data_field, lookups, reserved_field_ids)),
        ]

        return self._pgn_to_message(pgn_number, pgn_id, description, length, signals)

    def _single_to_message(self, json_data: dict, lookups: dict) -> Message:
        """
        Convert a single frame PGN to a cantools Message
        """
        pgn_number = json_data["PGN"]
        pgn_id = f"PGN_{pgn_number}_{json_data['Id']}"
        pgn_description = json_data["Description"]
        pgn_length = json_data["Length"]

        # pgn_complete = json_data["Complete"] if "Complete" in json_data else None

        pgn_repeating_fields = json_data["RepeatingFieldsSet1Size"] if "RepeatingFieldsSet1Size" in json_data else 0

        # check the offset and length of the last field to verify it fits in the message
        if not self._validate_data_length(json_data["Fields"], pgn_length):
            logging.warning(
                "PGN %d %s fields overflow the defined message length (%d)",
                pgn_number,
                pgn_id,
                pgn_length,
            )
            return None

        reserved_field_ids = set()
        pgn_signals = [
            self._field_to_signal(Field(field, lookups, reserved_field_ids))
            for field in json_data["Fields"]
        ]

        if pgn_repeating_fields:
            pgn_signals = self._repeat(pgn_signals, pgn_repeating_fields)

        return self._pgn_to_message(
            pgn_number, pgn_id, pgn_description, pgn_length, pgn_signals
        )

    def _validate_data_length(self, fields, pgn_length):
        """
        Check that the total length of the fields is less or equal to the PGN length
        """
        last_field = fields[-1]
        return last_field["BitOffset"] + last_field["BitLength"] <= pgn_length * 8

    # ---- Multiplexed (multi-variant) PGN support ---------------------------------------

    @staticmethod
    def _composite_match(variant):
        """
        The leading Manufacturer (11 bits) + Reserved (2 bits) + Industry (3 bits) fields form a
        fixed 2-byte block; collapse them into a single 16-bit multiplexor value. The Reserved
        bits are all ones on the wire. Returns None when the variant does not have this standard
        proprietary header (so it cannot take part in manufacturer multiplexing).
        """
        fields = variant["Fields"]
        if len(fields) < 3:
            return None
        manuf, industry = fields[0], fields[2]
        if (
            manuf.get("BitOffset") != 0 or manuf.get("BitLength") != 11 or "Match" not in manuf
            or industry.get("BitOffset") != 13 or industry.get("BitLength") != 3 or "Match" not in industry
        ):
            return None
        return manuf["Match"] | (0b11 << 11) | (industry["Match"] << 13)

    @staticmethod
    def _secondary_matches(variant):
        """
        The proprietary sub-protocol discriminators: match fields beyond the 2-byte
        manufacturer/industry header, keyed by bit offset (Proprietary ID, Message ID, Command,
        Report, Event, ...).
        """
        return {
            f["BitOffset"]: f for f in variant["Fields"] if "Match" in f and f["BitOffset"] >= 16
        }

    def _variants_to_multiplexed_message(self, pgn_number, variants, lookups):
        """
        Merge several variants of the same PGN into a (possibly nested) multiplexed message.

        Level 0 multiplexes on the 16-bit manufacturer/industry header; deeper levels nest on the
        proprietary match fields. Variants that cannot be field-mapped (fast/ISO) or that are a
        less-specific "catch-all" of a sibling are dropped with a warning.
        """
        single = [v for v in variants if v["Type"] == "Single"]
        if len(single) < len(variants):
            # The non-single (fast/ISO) variants cannot be field-mapped; the caller falls back to
            # an opaque fast-packet message, so this is informational rather than a warning.
            logging.info(
                "PGN %d: %d non-single-frame variant(s) not field-mapped", pgn_number, len(variants) - len(single)
            )
        if len(single) < 2:
            return None

        usable = [v for v in single if self._composite_match(v) is not None]
        if len(usable) < len(single):
            # Not every variant carries the proprietary manufacturer header (e.g. ISO Transport
            # Protocol on PGN 60416, keyed on a Group Function Code). Fall back to flat
            # multiplexing on the first match field.
            return self._simple_multiplex(pgn_number, single, lookups)

        reserved_ids = set()
        manuf_names = {d["Value"]: d["Name"] for d in lookups.get("MANUFACTURER_CODE", [])}

        # Level-0 multiplexor: the 16-bit manufacturer/industry header.
        header = {
            "Order": 1, "Id": "manufacturerIndustry", "Name": "Manufacturer and industry",
            "BitOffset": 0, "BitLength": 16, "BitStart": 0, "FieldType": "NUMBER", "Signed": False,
        }
        header_field = Field(header, lookups, reserved_ids)
        header_field.multiplexer = True

        groups = {}
        for v in usable:
            groups.setdefault(self._composite_match(v), []).append(v)
        header_field.enum_values = {
            comp: manuf_names.get(comp & 0x7FF, str(comp & 0x7FF)) for comp in groups
        }
        header_signal = self._field_to_signal(header_field)

        signals = [header_signal]
        for comp, group in groups.items():
            signals.extend(
                self._emit_variant_group(group, header_signal, comp, set(), reserved_ids, lookups, pgn_number)
            )

        return self._pgn_to_message(
            pgn_number,
            f"PGN_{pgn_number}_multiplexed",
            "Multiplexed message",
            max(v["Length" if "Length" in v else "MinLength"] for v in usable),
            signals,
        )

    def _simple_multiplex(self, pgn_number, variants, lookups):
        """
        Flat multiplexing for non-proprietary multi-variant PGNs: the first field is the
        multiplexor and every variant is selected by its unique match value. Returns None when
        the variants cannot be told apart by that first field.
        """
        reserved_ids = set()
        by_match = {}
        for v in variants:
            field0 = v["Fields"][0]
            if "Match" not in field0:
                logging.warning("PGN %d: variant %s has no leading match field; cannot multiplex", pgn_number, v["Id"])
                return None
            if field0["Match"] in by_match:
                logging.warning("PGN %d: match value %d is not unique; cannot multiplex", pgn_number, field0["Match"])
                return None
            by_match[field0["Match"]] = v

        mux_field = Field(variants[0]["Fields"][0], lookups, reserved_ids)
        mux_field.multiplexer = True
        mux_field.enum_values = {m: v["Id"] for m, v in by_match.items()}
        mux_signal = self._field_to_signal(mux_field)

        signals = [mux_signal]
        for match, variant in by_match.items():
            for f in variant["Fields"][1:]:
                if "BitLength" not in f:
                    continue
                field = Field(f, lookups, reserved_ids)
                field.id = get_unique_id(f"m{match}_{field.id}", reserved_ids)
                field.multiplexer_ids = [match]
                signals.append(self._field_to_signal(field, mux_signal.name))

        return self._pgn_to_message(
            pgn_number,
            f"PGN_{pgn_number}_multiplexed",
            "Multiplexed message",
            max(v["Length" if "Length" in v else "MinLength"] for v in variants),
            signals,
        )

    def _emit_variant_group(self, variants, parent_signal, parent_value, consumed, reserved_ids, lookups, pgn_number):
        """
        Emit the signals for variants that all share the multiplexor values on the path so far
        (`parent_signal` == `parent_value`, plus the offsets in `consumed`). Recurses, nesting a
        further multiplexor on the next shared, differing match field.
        """
        if len(variants) == 1:
            return self._leaf_signals(variants[0], parent_signal, parent_value, consumed, reserved_ids, lookups)

        disc, variants = self._pick_discriminator(variants, consumed, pgn_number)
        if disc is None:
            for v in variants[1:]:
                logging.warning("PGN %d: variant %s is indistinguishable; dropping", pgn_number, v["Id"])
            return self._leaf_signals(variants[0], parent_signal, parent_value, consumed, reserved_ids, lookups)
        if len(variants) == 1:
            return self._leaf_signals(variants[0], parent_signal, parent_value, consumed, reserved_ids, lookups)

        sub_groups = {}
        for v in variants:
            sub_groups.setdefault(self._secondary_matches(v)[disc]["Match"], []).append(v)

        mux_field = Field(self._secondary_matches(variants[0])[disc], lookups, reserved_ids)
        mux_field.id = get_unique_id(f"m{parent_value}_{mux_field.id}", reserved_ids)
        mux_field.multiplexer = True
        mux_field.multiplexer_ids = [parent_value]
        mux_field.enum_values = {val: grp[0]["Id"] for val, grp in sub_groups.items()}
        mux_signal = self._field_to_signal(mux_field, parent_signal.name)

        signals = [mux_signal]
        deeper = consumed | {disc}
        for val, grp in sub_groups.items():
            signals.extend(
                self._emit_variant_group(grp, mux_signal, val, deeper, reserved_ids, lookups, pgn_number)
            )
        return signals

    def _pick_discriminator(self, variants, consumed, pgn_number):
        """
        Choose the next bit offset to multiplex on. Returns (offset, variants_to_keep), or
        (None, [one variant]) when no further split is possible. Drops catch-all variants (no
        further match field) and, only if the survivors are misaligned, the minority that do not
        carry the chosen offset.
        """
        def sec(v):
            return set(self._secondary_matches(v)) - consumed

        specific = [v for v in variants if sec(v)]
        for v in variants:
            if not sec(v):
                logging.warning("PGN %d: dropping catch-all variant %s", pgn_number, v["Id"])
        if len(specific) <= 1:
            return (None, specific or [variants[0]])

        common = set.intersection(*[sec(v) for v in specific])
        for off in sorted(common):
            if len({self._secondary_matches(v)[off]["Match"] for v in specific}) > 1:
                return (off, specific)

        # No offset shared by every survivor: pick the most common one, drop the rest.
        counts = {}
        for v in specific:
            for off in sec(v):
                counts[off] = counts.get(off, 0) + 1
        best = max(counts, key=lambda o: (counts[o], -o))
        keep = [v for v in specific if best in self._secondary_matches(v)]
        for v in specific:
            if best not in self._secondary_matches(v):
                logging.warning("PGN %d: dropping misaligned variant %s", pgn_number, v["Id"])
        if len({self._secondary_matches(v)[best]["Match"] for v in keep}) <= 1:
            return (None, keep)
        return (best, keep)

    def _leaf_signals(self, variant, parent_signal, parent_value, consumed, reserved_ids, lookups):
        """
        Emit the data signals for a single variant: every field past the 2-byte header that was
        not already consumed as a multiplexor, multiplexed under `parent_value`.
        """
        signals = []
        for f in variant["Fields"]:
            if f["BitOffset"] < 16 or f["BitOffset"] in consumed:
                continue
            if "BitLength" not in f:
                # variable-length trailing field (string / dynamic); not a fixed-width DBC signal
                continue
            field = Field(f, lookups, reserved_ids)
            field.id = get_unique_id(f"m{parent_value}_{field.id}", reserved_ids)
            field.multiplexer_ids = [parent_value]
            signals.append(self._field_to_signal(field, parent_signal.name))
        return signals

    def _repeat(self, signals, repeating_fields):
        """
        Repeat the `repeating_fields` last signals until the PGN is exhausted
        """
        warnings.warn("Repeating fields not implemented")

        return signals

    def _field_to_signal(
        self, field: Field, multiplexer_signal: Signal = None
    ) -> Signal:
        """
        Convert a canboat Field to a cantools Signal
        """

        logging.debug(
            "Processing field %s (%d, %d)", field.id, field.bit_offset, field.bit_length
        )

        # cantools >= 39 bundles scale/offset/choices into a conversion object
        # instead of taking them as direct Signal arguments.
        conversion = BaseConversion.factory(
            scale=field.resolution, choices=field.enum_values
        )

        return Signal(
            name=field.id,
            start=field.bit_offset,
            length=field.bit_length,
            byte_order="little_endian",
            is_signed=field.signed,
            conversion=conversion,
            unit=field.unit,
            comment=field.description,
            is_multiplexer=field.multiplexer,
            multiplexer_ids=field.multiplexer_ids,
            multiplexer_signal=multiplexer_signal,
        )

    def _pgn_to_message(
        self,
        number: int,
        id_: str,
        description: str,
        length: int,
        signals: List[Signal],
    ):
        """
        Convert a canboat PGN to a cantools Message
        """

        frame_id = self._pgn_to_frame_id(number)

        logging.debug("frame_id = %s", frame_id)

        # FIXME: how about fast packets etc?

        message = Message(
            frame_id=frame_id,
            name=id_,
            length=8,
            signals=signals,
            comment=description,
            is_extended_frame=True,
            protocol="j1939",
        )

        return message

    def _pgn_to_frame_id(self, pgn: int) -> int:
        """
        Convert a canboat PGN to a cantools Frame ID
        """

        priority = 6
        reserved = 0
        data_page = (pgn >> 16) & 1
        pdu_format = pgn >> 8 & 255
        pdu_specific = pgn & 255
        source_address = 254

        priority_bits = priority << 26
        reserved_bits = reserved << 25
        data_page_bits = data_page << 24
        pdu_format_bits = pdu_format << 16
        pdu_specific_bits = pdu_specific << 8
        source_address_bits = source_address

        frame_id = (
            priority_bits
            | reserved_bits
            | data_page_bits
            | pdu_format_bits
            | pdu_specific_bits
            | source_address_bits
        )

        return frame_id
