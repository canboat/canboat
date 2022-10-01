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
    def __init__(self, field_dict: dict, reserved_field_ids: set):
        self.order: int = field_dict["Order"]
        self.id: str = get_unique_id(field_dict["Id"], reserved_field_ids)
        self.name: str = field_dict["Name"]
        self.description: str = (
            field_dict["Description"] if "Description" in field_dict else None
        )
        self.bit_length: int = field_dict["BitLength"]
        self.bit_offset: int = field_dict["BitOffset"]
        self.bit_start: int = field_dict["BitStart"]
        self.units: str = field_dict["Units"] if "Units" in field_dict else None
        self.type: str = field_dict["Type"] if "Type" in field_dict else None
        self.resolution: float = (
            field_dict["Resolution"] if "Resolution" in field_dict else 1
        )
        self.signed: bool = field_dict["Signed"] if "Resolution" in field_dict else None
        if self.type == "Lookup table" and "EnumValues" in field_dict:
            self.enum_values: Dict[int, str] = {
                d["value"]: d["name"] for d in field_dict["EnumValues"]
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
            pgn_number = pgn["PGN"]
            pgns.setdefault(pgn_number, []).append(pgn)

        for pgn_number, variants in pgns.items():
            if len(variants) > 1:
                msg = self._variants_to_multiplexed_message(pgn_number, variants)
                if msg is not None:
                    yield msg
                else:
                    logging.warning("Could not multiplex PGN %d, ignoring", pgn_number)
                    continue
            else:
                msg = self._json_to_message(variants[0])
                if msg is not None:
                    yield msg
                else:
                    logging.warning("Could not process PGN %d, ignoring", pgn_number)
                    continue

    def _json_to_message(self, json_data: dict) -> Message:
        """
        Convert a canboat json message to a cantools Message
        """

        logging.debug("Processing message %s", json_data["Id"])

        pgn_number = json_data["PGN"]

        raw_id = json_data["Id"]
        pgn_id = f"PGN_{pgn_number}_{raw_id}"

        logging.debug("Processing PGN %d (%s)", pgn_number, pgn_id)

        pgn_type = json_data["Type"] if "Type" in json_data else None

        pgn_length = json_data["Length"]

        if pgn_type and pgn_type == "ISO":
            logging.warning(
                "PGN %d %s: ISO packets not yet supported",
                pgn_number,
                pgn_id,
            )
            return None

        if (pgn_type and pgn_type == "Fast") or (pgn_length > 8):
            return self._fast_to_message(json_data)

        # fall back to single

        return self._single_to_message(json_data)

    def _fast_to_message(self, json_data):
        """
        Convert a fast packet PGN to a cantools Message
        """

        pgn_number = json_data["PGN"]
        pgn_id = f"PGN_{pgn_number}_{json_data['Id']}"
        pgn_description = json_data["Description"]
        pgn_length = json_data["Length"]

        # fast packets cannot be represented by DBC; replace the data definition
        # with a generic message representation

        first_field = {
            "Order": 1,
            "Id": "fastPacketSequenceCounter",
            "Name": "Fast packet sequence counter",
            "BitLength": 8,
            "BitOffset": 0,
            "BitStart": 0,
            "Type": "Integer",
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
            self._field_to_signal(Field(first_field, reserved_field_ids)),
            self._field_to_signal(Field(data_field, reserved_field_ids)),
        ]

        return self._pgn_to_message(
            pgn_number, pgn_id, pgn_description, pgn_length, signals
        )

    def _single_to_message(self, json_data: dict) -> Message:
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
            self._field_to_signal(Field(field, reserved_field_ids))
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

    def _variants_to_multiplexed_message(self, pgn_number, variants):
        """
        Merge several variants of the same PGN into a multiplexed message
        """

        # validate that the variants have the same multiplex field

        if not self._validate_multiplex_field(variants):
            logging.warning("Multiplex fields differ for PGN %d", pgn_number)
            return None

        # generate the message data
        pgn_id = f"PGN_{pgn_number}_multiplexed"
        pgn_description = "Multiplexed message"
        pgn_length = max([v["Length"] for v in variants])

        valid_variants = [v for v in variants if v["Length"] < 8 and v["Type"] != "Fast"]

        if len(valid_variants) < len(variants):
            logging.warning(
                "PGN %d: Ignoring fast packet variants", pgn_number
            )
            return None

        signals = []
        reserved_field_ids = set()

        # get the multiplexing signal
        multiplex_field = Field(valid_variants[0]["Fields"][0], reserved_field_ids)
        multiplex_choices = {
            variant["Fields"][0]["Match"]: variant["Id"] for variant in valid_variants if "Match" in variant["Fields"][0]
        }
        multiplex_field.enum_values = multiplex_choices
        multiplex_field.multiplexer = True
        multiplex_signal = self._field_to_signal(multiplex_field)
        signals.append(multiplex_signal)

        # get the signals for each variant

        for variant in valid_variants:
            variant_signals = self._get_variant_signals(
                variant, multiplex_signal, reserved_field_ids
            )
            signals.extend(variant_signals)

        # create the multiplexed message
        return self._pgn_to_message(
            pgn_number, pgn_id, pgn_description, pgn_length, signals
        )

    def _get_variant_signals(
        self, variant: dict, multiplexer_signal: Signal, reserved_field_ids: Set[int]
    ):
        """
        Get the signals for a variant of a multiplexed message
        """

        if "Match" not in variant["Fields"][0]:
            return []
        variant_match = variant["Fields"][0]["Match"]
        variant_signals = []
        for json_field in variant["Fields"][1:]:
            field = Field(json_field, reserved_field_ids)
            # add the message id to the multiplexed field id
            field.id = f"m{variant_match}_{field.id}"
            field.multiplexer_ids = [variant_match]
            signal = self._field_to_signal(field, multiplexer_signal)
            variant_signals.append(signal)

        return variant_signals

    def _validate_multiplex_field(self, variants):
        """
        Check that the variants have a valid multiplex field

        Multiplex fields need to have the same id, same bit offset and length,
        and different values. It is assumed that the first field is always
        the multiplexor.
        """

        ignore_variants = set()

        for variant in variants:
            variant_field0 = variant["Fields"][0]
            if "Match" not in variant_field0:
                logging.warning("No match field in variant %s; ignoring", variant["Id"])
                ignore_variants.add(variant["Id"])

        ref = variants[0]
        ref_field0 = ref["Fields"][0]
        if "Match" not in ref_field0:
            return False
        seen_values = {ref_field0["Match"]}
        for variant in variants[1:]:
            if variant["Id"] in ignore_variants:
                continue
            variant_field0 = variant["Fields"][0]
            if ref_field0["Id"] != variant_field0["Id"]:
                logging.warning(
                    "Multiplex variants %s and %s have different ids",
                    ref["Id"],
                    variant["Id"],
                )
                return False
            if variant_field0["Match"] in seen_values:
                logging.warning(
                    "Multiplex field value %d is not unique",
                    variant_field0["Match"],
                )
                return False
            seen_values.add(variant_field0["Match"])
            if ref_field0["BitOffset"] != variant_field0["BitOffset"]:
                logging.warning(
                    "Multiplex fields have different bit offsets (%d != %d)",
                    ref_field0["BitOffset"],
                    variant_field0["BitOffset"],
                )
                return False
            if ref_field0["BitLength"] != variant_field0["BitLength"]:
                logging.warning(
                    "Multiplex fields have different bit lengths (%d != %d)",
                    ref_field0["BitLength"],
                    variant_field0["BitLength"],
                )
                return False
        return True

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

        # FIXME: implement type awareness and enums (choices)

        return Signal(
            name=field.id,
            start=field.bit_offset,
            length=field.bit_length,
            byte_order="little_endian",
            is_signed=field.signed,
            scale=field.resolution,
            unit=field.units,
            comment=field.description,
            choices=field.enum_values,
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
