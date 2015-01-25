/*

Analyzes NMEA 2000 PGNs.

(C) 2009-2014, Kees Verruijt, Harlingen, The Netherlands.

This file is part of CANboat.

CANboat is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

CANboat is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with CANboat.  If not, see <http://www.gnu.org/licenses/>.

*/

#define UINT16_OUT_OF_RANGE (MAX_UINT16 - 1)
#define UINT16_UNKNOWN (MAX_UINT16)

#define PROGRAM_DATE "$Date: 2012-03-14 16:30:42 +0100 (wo, 14 mrt 2012) $"
#define PROGRAM_REV  "$Rev: 249 $"

/*
 * Notes on the NMEA 2000 packet structure
 * ---------------------------------------
 *
 * http://www.nmea.org/Assets/pgn059392.pdf tells us that:
 * - All messages shall set the reserved bit in the CAN ID field to zero on transmit.
 * - Data field reserve bits or reserve bytes shall be filled with ones. i.e. a reserve
 *   byte will be set to a hex value of FF, a single reservie bit would be set to a value of 1.
 * - Data field extra bytes shall be illed with a hex value of FF.
 * - If the PGN in a Command or Request is not recognized by the destination it shall
 *   reply with the PGN 059392 ACK or NACK message using a destination specific address.
 *
 */

/*
 * NMEA 2000 uses the 8 'data' bytes as follows:
 * data[0] is an 'order' that increments, or not (depending a bit on implementation).
 * If the size of the packet <= 7 then the data follows in data[1..7]
 * If the size of the packet > 7 then the next byte data[1] is the size of the payload
 * and data[0] is divided into 5 bits index into the fast packet, and 3 bits 'order
 * that increases.
 * This means that for 'fast packets' the first bucket (sub-packet) contains 6 payload
 * bytes and 7 for remaining. Since the max index is 31, the maximal payload is
 * 6 + 31 * 7 = 223 bytes
 */

/*
 * Some packets include a "SID", explained by Maretron as follows:
 * SID: The sequence identifier field is used to tie related PGNs together. For example,
 * the DST100 will transmit identical SIDs for Speed (PGN 128259) and Water depth
 * (128267) to indicate that the readings are linked together (i.e., the data from each
 * PGN was taken at the same time although reported at slightly different times).
 */

#define FASTPACKET_INDEX (0)
#define FASTPACKET_SIZE (1)
#define FASTPACKET_BUCKET_0_SIZE (6)
#define FASTPACKET_BUCKET_N_SIZE (7)
#define FASTPACKET_BUCKET_0_OFFSET (2)
#define FASTPACKET_BUCKET_N_OFFSET (1)
#define FASTPACKET_MAX_INDEX (0x1f)
#define FASTPACKET_MAX_SIZE (FASTPACKET_BUCKET_0_SIZE + FASTPACKET_BUCKET_N_SIZE * (FASTPACKET_MAX_INDEX - 1))

#define Pi (3.141592654)
#define RadianToDegree (360.0 / 2 / Pi)
#define BYTES(x) ((x)*(8))

#define RES_LAT_LONG_PRECISION (10000000) /* 1e7 */
#define RES_LAT_LONG (1.0e-7)
#define RES_LAT_LONG_64 (1.0e-16)

typedef struct
{
  char * name;
  uint32_t size;     /* Size in bits. All fields are contiguous in message; use 'reserved' fields to fill in empty bits. */
# define LEN_VARIABLE (0)
  double resolution; /* Either a positive real value or one of the following RES_ special values */
# define RES_NOTUSED (0)
# define RES_DEGREES (1e-4 * RadianToDegree)
# define RES_ROTATION (1e-3/32.0 * RadianToDegree)
# define RES_ASCII (-1.0)
# define RES_LATITUDE (-2.0)
# define RES_LONGITUDE (-3.0)
# define RES_DATE (-4.0)
# define RES_TIME (-5.0)
# define RES_TEMPERATURE (-6.0)
# define RES_6BITASCII (-7.0)            /* Actually not used in N2K, only in N183 AIS */
# define RES_INTEGER (-8.0)
# define RES_LOOKUP (-9.0)
# define RES_BINARY (-10.0)
# define RES_MANUFACTURER (-11.0)
# define RES_STRING (-12.0)
# define RES_FLOAT (-13.0)
# define RES_PRESSURE (-14.0)
# define RES_STRINGLZ (-15.0)            /* ASCII string starting with length byte and terminated by zero byte */
# define MAX_RESOLUTION_LOOKUP 15

  bool hasSign; /* Is the value signed, e.g. has both positive and negative values? */
  char * units; /* String containing the 'Dimension' (e.g. s, h, m/s, etc.) unless it starts with , in which
                 * case it contains a set of lookup values.
                 */
  char * description;
  int32_t offset; /* Only used for SAE J1939 values with sign; these are in Offset/Excess-K notation instead
                   * of two's complement as used by NMEA 2000.
                   * See http://en.wikipedia.org/wiki/Offset_binary
                   */
  char * camelName; /* Filled by C, no need to set in initializers. */
} Field;

typedef struct
{
  const char * name;
  const char * resolution;
} Resolution;

const Resolution types[MAX_RESOLUTION_LOOKUP] =
{ { "ASCII text", 0 }
, { "Latitude", 0 }
, { "Longitude", 0 }
, { "Date", "1" }
, { "Time", "0.0001" }
, { "Temperature", "0.01" }
, { "6 Bit ASCII text", 0 }
, { "Integer", "1" }
, { "Lookup table", 0 }
, { "Binary data", 0 }
, { "Manufacturer code", 0 }
, { "String with start/stop byte", 0 }
, { "IEEE Float", 0 }
, { "Pressure", 0 }
, { "ASCII string starting with length byte", 0 }
};


#define LOOKUP_INDUSTRY_CODE (",4=Marine")

#define LOOKUP_SHIP_TYPE ( \
          ",0=unavailable" \
          ",20=Wing In Ground,29=Wing In Ground (no other information)" \
          ",30=Fishing,31=Towing,32=Towing exceeds 200m or wider than 25m,33=Engaged in dredging or underwater operations,34=Engaged in diving operations" \
          ",35=Engaged in military operations,36=Sailing,37=Pleasure" \
          ",40=High speed craft,71=High speed craft carrying dangerous goods,72=High speed craft hazard cat B,73=High speed craft hazard cat C,74=High speed craft hazard cat D,79=High speed craft (no additional information)" \
          ",50=Pilot vessel,51=SAR,52=Tug,53=Port tender,54=Anti-pollution,55=Law enforcement,56=Spare,57=Spare #2,58=Medical,59=RR Resolution No.18" \
          ",60=Passenger ship,69=Passenger ship (no additional information)" \
          ",70=Cargo ship,71=Cargo ship carrying dangerous goods,72=Cargo ship hazard cat B,73=Cargo ship hazard cat C,74=Cargo ship hazard cat D,79=Cargo ship (no additional information)" \
          ",80=Tanker,81=Tanker carrying dangerous goods,82=Tanker hazard cat B,83=Tanker hazard cat C,84=Tanker hazard cat D,89=Tanker (no additional information)" \
          ",90=Other,91=Other carrying dangerous goods,92=Other hazard cat B,93=Other hazard cat C,94=Other hazard cat D,99=Other (no additional information)" \
          )

/* http://www.nmea.org/Assets/20120726%20nmea%202000%20class%20&%20function%20codes%20v%202.00.pdf */
#define LOOKUP_DEVICE_CLASS ( \
          ",0=Reserved for 2000 Use" \
          ",10=System tools" \
          ",20=Safety systems" \
          ",25=Internetwork device" \
          ",30=Electrical Distribution" \
          ",35=Electrical Generation" \
          ",40=Steering and Control surfaces" \
          ",50=Propulsion" \
          ",60=Navigation" \
          ",70=Communication" \
          ",75=Sensor Communication Interface" \
          ",80=Instrumentation/general systems" \
          ",85=External Environment" \
          ",90=Internal Environment" \
          ",100=Deck + cargo + fishing equipment systems" \
          ",120=Display" \
          ",125=Entertainment" \
          )

#define LOOKUP_REPEAT_INDICATOR ( \
          ",0=Initial,1=First retransmission,2=Second retransmission,3=Final retransmission" \
          )

#define LOOKUP_AIS_TRANSCEIVER ( \
          ",0=Channel A VDL reception" \
          ",1=Channel B VDL reception" \
          ",2=Channel A VDL transmission" \
          ",3=Channel B VDL transmission" \
          ",4=Own information not broadcast" \
          ",5=Reserved" \
          )

#define LOOKUP_ENGINE_INSTANCE ( ",0=Single Engine or Dual Engine Port,1=Dual Engine Starboard" )

#define LOOKUP_GEAR_STATUS ( ",0=Forward,1=Neutral,2=Reverse,3=Unknown" )

#define LOOKUP_POSITION_ACCURACY ( ",0=Low,1=High" )

#define LOOKUP_RAIM_FLAG ( ",0=not in use,1=in use" )

#define LOOKUP_TIME_STAMP ( ",60=Not available,61=Manual input mode,62=Dead reckoning mode,63=Positioning system is inoperative" )

#define LOOKUP_GNS_AIS ( ",0=undefined,1=GPS,2=GLONASS,3=GPS+GLONASS,4=Loran-C,5=Chayka,6=integrated,7=surveyed,8=Galileo" )
#define LOOKUP_GNS ( ",0=GPS,1=GLONASS,2=GPS+GLONASS,3=GPS+SBAS/WAAS,4=GPS+SBAS/WAAS+GLONASS,5=Chayka,6=integrated,7=surveyed,8=Galileo" )

#define LOOKUP_GNS_METHOD ( ",0=no GNSS,1=GNSS fix,2=DGNSS fix,3=Precise GNSS,4=RTK Fixed Integer,5=RTK float,6=Estimated (DR) mode,7=Manual Input,8=Simulate mode" )

#define LOOKUP_GNS_INTEGRITY ( ",0=No integrity checking,1=Safe,2=Caution" )

#define LOOKUP_SYSTEM_TIME ( ",0=GPS,1=GLONASS,2=Radio Station,3=Local Cesium clock,4=Local Rubidium clock,5=Local Crystal clock" )

#define LOOKUP_MAGNETIC_VARIATION ( \
            ",0=Manual" \
            ",1=Automatic Chart"\
            ",2=Automatic Table"\
            ",3=Automatic Calculation"\
            ",4=WMM 2000"\
            ",5=WMM 2005"\
            ",6=WMM 2010"\
            ",7=WMM 2015"\
            ",8=WMM 2020"\
            )

#define LOOKUP_RESIDUAL_MODE ( ",0=Autonomous,1=Differential enhanced,2=Estimated,3=Simulator,4=Manual" )

#define LOOKUP_WIND_REFERENCE ( ",0=True (ground referenced to North),1=Magnetic (ground referenced to Magnetic North),2=Apparent,3=True (boat referenced),4=True (water referenced)" )

#define LOOKUP_YES_NO ( ",0=No,1=Yes" )
#define LOOKUP_OK_WARNING ( ",0=OK,1=Warning" )

#define LOOKUP_DIRECTION_REFERENCE ( ",0=True,1=Magnetic" )

#define LOOKUP_NAV_STATUS ( \
            ",0=Under way using engine" \
            ",1=At anchor" \
            ",2=Not under command" \
            ",3=Restricted manoeuverability" \
            ",4=Constrained by her draught" \
            ",5=Moored" \
            ",6=Aground" \
            ",7=Engaged in Fishing" \
            ",8=Under way sailing" )

#define LOOKUP_POWER_FACTOR ( ",0=Leading,1=Lagging,2=Error" )

#define LOOKUP_TEMPERATURE_SOURCE ( \
    ",0=Sea Temperature" \
    ",1=Outside Temperature" \
    ",2=Inside Temperature" \
    ",3=Engine Room Temperature" \
    ",4=Main Cabin Temperature" \
    ",5=Live Well Temperature" \
    ",6=Bait Well Temperature" \
    ",7=Refridgeration Temperature" \
    ",8=Heating System Temperature" \
    ",9=Freezer Temperature" )

#define ACTISENSE_BEM 0x40000 /* Actisense specific fake PGNs */

typedef struct
{
  char     * description;
  uint32_t   pgn;
  bool       known;             /* Are we pretty sure we've got all fields with correct definitions? */
  uint32_t   size;              /* (Minimal) size of this PGN. Helps to determine fast/single frame and initial malloc */
  uint32_t   repeatingFields;   /* How many fields at the end repeat until the PGN is exhausted? */
  Field      fieldList[28];     /* Note fixed # of fields; increase if needed. RepeatingFields support means this is enough for now. */
  uint32_t   fieldCount;        /* Filled by C, no need to set in initializers. */
  char     * camelDescription;  /* Filled by C, no need to set in initializers. */
} Pgn;


Pgn pgnList[] =
{

{ "Unknown PGN", 0, false, 8, 0,
  { { "Manufacturer Code", 11, RES_MANUFACTURER, false, 0, "" }
  , { "Reserved", 2, 1, false, 0, "" }
  , { "Industry Code", 3, RES_LOOKUP, false, LOOKUP_INDUSTRY_CODE, "" }
  , { 0 }
  }
}

  /************ Protocol PGNs ************/
  /* http://www.nmea.org/Assets/july%202010%20nmea2000_v1-301_app_b_pgn_field_list.pdf */
  /* http://www.maretron.com/products/pdf/J2K100-Data_Sheet.pdf */
  /* http://www.nmea.org/Assets/pgn059392.pdf */
  /* http://www8.garmin.com/manuals/GPSMAP4008_NMEA2000NetworkFundamentals.pdf */
  /* http://www.furunousa.com/Furuno/Doc/0/8JT2BMDDIB249FCNUK64DKLV67/GP330B%20NMEA%20PGNs.pdf */
  /* http://www.nmea.org/Assets/20140710%20nmea-2000-060928%20iso%20address%20claim%20pgn%20corrigendum.pdf */
,
{ "ISO Acknowledgement", 59392, true, 8, 0,
  { { "Control", BYTES(1), RES_LOOKUP, false, ",0=ACK,1=NAK,2=Access Denied,3=Address Busy", "" }
  , { "Group Function", BYTES(1), 1, false, 0, "" }
  , { "Reserved", 24, RES_BINARY, false, 0, "Reserved" }
  , { "PGN", 24, RES_INTEGER, false, 0, "Parameter Group Number of requested information" }
  , { 0 }
  }
}

,
{ "ISO Request", 59904, true, 3, 0,
  { { "PGN", 24, RES_INTEGER, false, 0, "" }
  , { 0 }
  }
}

,
{ "ISO Address Claim", 60928, true, 8, 0,
  { { "Unique Number", 21, RES_BINARY, false, 0, "ISO Identity Number" }
  , { "Manufacturer Code", 11, RES_MANUFACTURER, false, 0, "" }
  , { "Device Instance Lower", 3, 1, false, 0, "ISO ECU Instance" }
  , { "Device Instance Upper", 5, 1, false, 0, "ISO Function Instance" }
  , { "Device Function", 8, 1, false, 0, "ISO Function" }
  , { "Reserved", 1, RES_BINARY, false, 0, "" }
  , { "Device Class", 7, RES_LOOKUP, false, LOOKUP_DEVICE_CLASS, "" }
  , { "System Instance", 4, 1, false, 0, "ISO Device Class Instance" }
  , { "Industry Group", 3, RES_LOOKUP, false, LOOKUP_INDUSTRY_CODE, "" }
  , { "Reserved", 1, 1, false, 0, "ISO Self Configurable" }
  , { 0 }
  }
}

  /* The following probably have the wrong Proprietary ID */
,
{ "Seatalk: Wireless Keypad Light Control", 61184, false, 0x08, 0,
  { { "Manufacturer Code", 11, RES_MANUFACTURER, false, "=1851", "Raymarine" }
  , { "Reserved", 2, 1, false, 0, "" }
  , { "Industry Code", 3, RES_LOOKUP, false, "=4", "Marine Industry" }
  , { "Proprietary ID", BYTES(1), RES_INTEGER, false, "=1", "Wireless Keypad Light Control" }
  , { "Variant", BYTES(1), 1, false, 0, "" }
  , { "Wireless Setting", BYTES(1), 1, false, 0, "" }
  , { "Wired Setting", BYTES(1), 1, false, 0, "" }
  }
}

,
{ "Seatalk: Wireless Keypad Light Control", 61184, false, 0x08, 0,
  { { "Manufacturer Code", 11, RES_MANUFACTURER, false, "=1851", "Raymarine" }
  , { "Reserved", 2, 1, false, 0, "" }
  , { "Industry Code", 3, RES_LOOKUP, false, "=4", "Marine Industry" }
  , { "PID", BYTES(1), 1, false, 0, "" }
  , { "Variant", BYTES(1), 1, false, 0, "" }
  , { "Beep Control", BYTES(1), 1, false, 0, "" }
  }
}

,
{ "ISO: Manu. Proprietary single-frame addressed", 61184, false, 8, 0,
  { { "Manufacturer Code", 11, RES_MANUFACTURER, false, 0, "" }
  , { "Reserved", 2, 1, false, 0, "" }
  , { "Industry Code", 3, RES_LOOKUP, false, LOOKUP_INDUSTRY_CODE, "" }
  , { 0 }
  }
}

/* Maretron ACM 100 manual documents PGN 65001-65030 */

,
{ "Bus #1 Phase C Basic AC Quantities", 65001, false, 8, 0,
  { { "Line-Line AC RMS Voltage", BYTES(2), 1, false, "V", "" }
  , { "Line-Neutral AC RMS Voltage", BYTES(2), 1, false, "V", "" }
  , { "AC Frequency", BYTES(2), 1/128, false, "Hz", "" }
  , { 0 }
  }
}

,
{ "Bus #1 Phase B Basic AC Quantities", 65002, false, 8, 0,
  { { "Line-Line AC RMS Voltage", BYTES(2), 1, false, "V", "" }
  , { "Line-Neutral AC RMS Voltage", BYTES(2), 1, false, "V", "" }
  , { "AC Frequency", BYTES(2), 1/128, false, "Hz", "" }
  , { 0 }
  }
}

,
{ "Bus #1 Phase A Basic AC Quantities", 65003, false, 8, 0,
  { { "Line-Line AC RMS Voltage", BYTES(2), 1, false, "V", "" }
  , { "Line-Neutral AC RMS Voltage", BYTES(2), 1, false, "V", "" }
  , { "AC Frequency", BYTES(2), 1/128, false, "Hz", "" }
  , { 0 }
  }
}

,
{ "Bus #1 Average Basic AC Quantities", 65004, false, 8, 0,
  { { "Line-Line AC RMS Voltage", BYTES(2), 1, false, "V", "" }
  , { "Line-Neutral AC RMS Voltage", BYTES(2), 1, false, "V", "" }
  , { "AC Frequency", BYTES(2), 1/128, false, "Hz", "" }
  , { 0 }
  }
}

,
{ "Utility Total AC Energy", 65005, false, 8, 0,
  { { "Total Energy Export", BYTES(4), 1, false, "kWh", "" }
  , { "Total Energy Import", BYTES(4), 1, false, "kWh", "" }
  , { 0 }
  }
}

,
{ "Utility Phase C AC Reactive Power", 65006, false, 8, 0,
  { { "Reactive Power", BYTES(2), 1, false, "VAr", "" }
  , { "Power Factor", BYTES(2), 1/16384, false, 0, "" }
  , { "Power Factor Lagging", 2, RES_LOOKUP, false, LOOKUP_POWER_FACTOR, "" }
  , { 0 }
  }
}

,
{ "Utility Phase C AC Power", 65007, false, 8, 0,
  { { "Real Power", BYTES(4), 1, true, "W", "", -2000000000 }
  , { "Apparent Power", BYTES(4), 1, true, "VA", "", -2000000000 }
  , { 0 }
  }
}

,
{ "Utility Phase C Basic AC Quantities", 65008, false, 8, 0,
  { { "Line-Line AC RMS Voltage", BYTES(2), 1, false, "V", "" }
  , { "Line-Neutral AC RMS Voltage", BYTES(2), 1, false, "V", "" }
  , { "AC Frequency", BYTES(2), 1/128, false, "Hz", "" }
  , { "AC RMS Current", BYTES(2), 1, false, "A", "" }
  , { 0 }
  }
}

,
{ "Utility Phase B AC Reactive Power", 65009, false, 8, 0,
  { { "Reactive Power", BYTES(2), 1, false, "VAr", "" }
  , { "Power Factor", BYTES(2), 1/16384, false, 0, "" }
  , { "Power Factor Lagging", 2, RES_LOOKUP, false, LOOKUP_POWER_FACTOR, "" }
  , { 0 }
  }
}

,
{ "Utility Phase B AC Power", 65010, false, 8, 0,
  { { "Real Power", BYTES(4), 1, true, "W", "", -2000000000 }
  , { "Apparent Power", BYTES(4), 1, true, "VA", "", -2000000000 }
  , { 0 }
  }
}

,
{ "Utility Phase B Basic AC Quantities", 65011, false, 8, 0,
  { { "Line-Line AC RMS Voltage", BYTES(2), 1, false, "V", "" }
  , { "Line-Neutral AC RMS Voltage", BYTES(2), 1, false, "V", "" }
  , { "AC Frequency", BYTES(2), 1/128, false, "Hz", "" }
  , { "AC RMS Current", BYTES(2), 1, false, "A", "" }
  , { 0 }
  }
}

,
{ "Utility Phase A AC Reactive Power", 65012, false, 8, 0,
  { { "Reactive Power", BYTES(4), 1, true, "VAr", "", -2000000000 }
  , { "Power Factor", BYTES(2), 1/16384, true, 0, "" }
  , { "Power Factor Lagging", 2, RES_LOOKUP, false, LOOKUP_POWER_FACTOR, "" }
  , { 0 }
  }
}

,
{ "Utility Phase A AC Power", 65013, false, 8, 0,
  { { "Real Power", BYTES(4), 1, true, "W", "", -2000000000 }
  , { "Apparent Power", BYTES(4), 1, true, "VA", "", -2000000000 }
  , { 0 }
  }
}

,
{ "Utility Phase A Basic AC Quantities", 65014, false, 8, 0,
  { { "Line-Line AC RMS Voltage", BYTES(2), 1, false, "V", "" }
  , { "Line-Neutral AC RMS Voltage", BYTES(2), 1, false, "V", "" }
  , { "AC Frequency", BYTES(2), 1/128, false, "Hz", "" }
  , { "AC RMS Current", BYTES(2), 1, false, "A", "" }
  , { 0 }
  }
}

,
{ "Utility Total AC Reactive Power", 65015, false, 8, 0,
  { { "Reactive Power", BYTES(4), 1, true, "VAr", "", -2000000000 }
  , { "Power Factor", BYTES(2), 1/16384, false, 0, "" }
  , { "Power Factor Lagging", 2, RES_LOOKUP, false, LOOKUP_POWER_FACTOR, "" }
  , { 0 }
  }
}

,
{ "Utility Total AC Power", 65016, false, 8, 0,
  { { "Real Power", BYTES(4), 1, true, "W", "", -2000000000 }
  , { "Apparent Power", BYTES(4), 1, true, "VA", "", -2000000000 }
  , { 0 }
  }
}

,
{ "Utility Average Basic AC Quantities", 65017, false, 8, 0,
  { { "Line-Line AC RMS Voltage", BYTES(2), 1, false, "V", "" }
  , { "Line-Neutral AC RMS Voltage", BYTES(2), 1, false, "V", "" }
  , { "AC Frequency", BYTES(2), 1/128, false, "Hz", "" }
  , { "AC RMS Current", BYTES(2), 1, false, "A", "" }
  , { 0 }
  }
}

,
{ "Generator Total AC Energy", 65018, false, 8, 0,
  { { "Total Energy Export", BYTES(4), 1, false, "kWh", "" }
  , { "Total Energy Import", BYTES(4), 1, false, "kWh", "" }
  , { 0 }
  }
}

,
{ "Generator Phase C AC Reactive Power", 65019, false, 8, 0,
  { { "Reactive Power", BYTES(2), 1, false, "VAr", "", -2000000000 }
  , { "Power Factor", BYTES(2), 1/16384, false, 0, "" }
  , { "Power Factor Lagging", 2, RES_LOOKUP, false, LOOKUP_POWER_FACTOR, "" }
  , { 0 }
  }
}

,
{ "Generator Phase C AC Power", 65020, false, 8, 0,
  { { "Real Power", BYTES(2), 1, false, "W", "", -2000000000 }
  , { "Apparent Power", BYTES(2), 1, false, "VA", "", -2000000000 }
  , { 0 }
  }
}

,
{ "Generator Phase C Basic AC Quantities", 65021, false, 8, 0,
  { { "Line-Line AC RMS Voltage", BYTES(2), 1, false, "V", "" }
  , { "Line-Neutral AC RMS Voltage", BYTES(2), 1, false, "V", "" }
  , { "AC Frequency", BYTES(2), 1/128, false, "Hz", "" }
  , { "AC RMS Current", BYTES(2), 1, false, "A", "" }
  , { 0 }
  }
}

,
{ "Generator Phase B AC Reactive Power", 65022, false, 8, 0,
  { { "Reactive Power", BYTES(2), 1, false, "VAr", "", -2000000000 }
  , { "Power Factor", BYTES(2), 1/16384, false, 0, "" }
  , { "Power Factor Lagging", 2, RES_LOOKUP, false, LOOKUP_POWER_FACTOR, "" }
  , { 0 }
  }
}

,
{ "Generator Phase B AC Power", 65023, false, 8, 0,
  { { "Real Power", BYTES(2), 1, false, "W", "", -2000000000 }
  , { "Apparent Power", BYTES(2), 1, false, "VA", "", -2000000000 }
  , { 0 }
  }
}

,
{ "Generator Phase B Basic AC Quantities", 65024, false, 8, 0,
  { { "Line-Line AC RMS Voltage", BYTES(2), 1, false, "V", "" }
  , { "Line-Neutral AC RMS Voltage", BYTES(2), 1, false, "V", "" }
  , { "AC Frequency", BYTES(2), 1/128, false, "Hz", "" }
  , { "AC RMS Current", BYTES(2), 1, false, "A", "" }
  , { 0 }
  }
}

,
{ "Generator Phase A AC Reactive Power", 65025, false, 8, 0,
  { { "Reactive Power", BYTES(2), 1, false, "VAr", "", -2000000000 }
  , { "Power Factor", BYTES(2), 1/16384, false, 0, "" }
  , { "Power Factor Lagging", 2, RES_LOOKUP, false, LOOKUP_POWER_FACTOR, "" }
  , { 0 }
  }
}

,
{ "Generator Phase A AC Power", 65026, false, 8, 0,
  { { "Real Power", BYTES(2), 1, false, "W", "", -2000000000 }
  , { "Apparent Power", BYTES(2), 1, false, "VA", "", -2000000000 }
  , { 0 }
  }
}

,
{ "Generator Phase A Basic AC Quantities", 65027, false, 8, 0,
  { { "Line-Line AC RMS Voltage", BYTES(2), 1, false, "V", "" }
  , { "Line-Neutral AC RMS Voltage", BYTES(2), 1, false, "V", "" }
  , { "AC Frequency", BYTES(2), 1/128, false, "Hz", "" }
  , { "AC RMS Current", BYTES(2), 1, false, "A", "" }
  , { 0 }
  }
}

,
{ "Generator Total AC Reactive Power", 65028, false, 8, 0,
  { { "Reactive Power", BYTES(2), 1, false, "VAr", "", -2000000000 }
  , { "Power Factor", BYTES(2), 1/16384, false, 0, "" }
  , { "Power Factor Lagging", 2, RES_LOOKUP, false, LOOKUP_POWER_FACTOR, "" }
  , { 0 }
  }
}

,
{ "Generator Total AC Power", 65029, false, 8, 0,
  { { "Real Power", BYTES(2), 1, false, "W", "", -2000000000 }
  , { "Apparent Power", BYTES(2), 1, false, "VA", "", -2000000000 }
  , { 0 }
  }
}

,
{ "Generator Average Basic AC Quantities", 65030, false, 8, 0,
  { { "Line-Line AC RMS Voltage", BYTES(2), 1, false, "V", "" }
  , { "Line-Neutral AC RMS Voltage", BYTES(2), 1, false, "V", "" }
  , { "AC Frequency", BYTES(2), 1/128, false, "Hz", "" }
  , { "AC RMS Current", BYTES(2), 1, false, "A", "" }
  , { 0 }
  }
}

,
{ "ISO Commanded Address", 65240, false, 8, 0,
  /* ISO 11783 defined this message to provide a mechanism for assigning a network address to a node. The NAME information in the
  data portion of the message must match the name information of the node whose network address is to be set. */
  { { "Unique Number", 21, RES_BINARY, false, 0, "ISO Identity Number" }
  , { "Manufacturer Code", 11, 1, false, 0, "" }
  , { "Device Instance Lower", 4, 1, false, 0, "ISO ECU Instance" }
  , { "Device Instance Upper", 4, 1, false, 0, "ISO Function Instance" }
  , { "Device Function", BYTES(1), 1, false, 0, "ISO Function" }
  , { "Reserved", 1 , 1, false, 0, "" }
  , { "Device Class", 7, RES_LOOKUP, false, LOOKUP_DEVICE_CLASS, "" }
  , { "System Instance", 4, 1, false, 0, "ISO Device Class Instance" }
  , { "Reserved", 2, 1, false, 0, "" }
  , { "Industry Code", 3, RES_LOOKUP, false, LOOKUP_INDUSTRY_CODE, "" }
  , { "Reserved", 3, 1, false, 0, "ISO Self Configurable" }
  , { "New Source Address", BYTES(1), 1, false, 0, "" }
  , { 0 }
  }
}

  /* ISO 11783: 65,280 to 65,535 (0xFF00 to 0xFFFF): Proprietary PDU-2 messages */

,
{ "ISO: Manu. Proprietary single-frame non-addressed", 65280, false, 8, 0,
  { { 0 }
  }
}

  /* http://www.airmartechnology.com/uploads/installguide/DST200UserlManual.pdf */
,
{ "Airmar: Boot State Acknowledgment", 65285, true, 8, 0,
  { { "Manufacturer Code", 11, RES_MANUFACTURER, false, "=135", "Airmar" }
  , { "Reserved", 2, 1, false, 0, "" }
  , { "Industry Code", 3, RES_LOOKUP, false, "=4", "Marine Industry" }
  , { "Boot State", 4, RES_LOOKUP, false, ",0=in Startup Monitor,1=running Bootloader,2=running Application", "" }
  , { 0 }
  }
}

,
{ "Lowrance: Temperature", 65285, false, 8, 0,
  { { "Manufacturer Code", 11, RES_MANUFACTURER, false, "=140", "" }
  , { "Reserved", 2, 1, false, 0, "" }
  , { "Industry Code", 3, RES_LOOKUP, false, LOOKUP_INDUSTRY_CODE, "" }
  , { "Temperature Instance", 4, 1, false, 0, "" }
  , { "Temperature Source", 4, 1, false, 0, "" }
  , { "Actual Temperature", BYTES(2), RES_TEMPERATURE, false, "K", "" }
  , { 0 }
  }
}

  /* http://www.airmartechnology.com/uploads/installguide/DST200UserlManual.pdf */
,
{ "Airmar: Boot State Request", 65286, true, 8, 0,
  { { "Manufacturer Code", 11, RES_MANUFACTURER, false, "=135", "Airmar" }
  , { "Reserved", 2, 1, false, 0, "" }
  , { "Industry Code", 3, RES_LOOKUP, false, LOOKUP_INDUSTRY_CODE, "" }
  , { 0 }
  }
}

  /* http://www.airmartechnology.com/uploads/installguide/DST200UserlManual.pdf */
,
{ "Airmar: Access Level", 65287, true, 8, 0,
  { { "Manufacturer Code", 11, RES_MANUFACTURER, false, "=135", "Airmar" }
  , { "Reserved", 2, 1, false, 0, "" }
  , { "Industry Code", 3, RES_LOOKUP, false, "=4", "Marine Industry" }
  , { "Format Code", 3, RES_LOOKUP, false, ",1=Format code 1", "" }
  , { "Access Level", 3, RES_LOOKUP, false, ",0=Locked,1=unlocked level 1,2=unlocked level 2", "" }
  , { "Reserved", 2, 1, false, 0, "" }
  , { "Access Seed/Key", BYTES(4), RES_INTEGER, false, 0, "When transmitted, it provides a seed for an unlock operation. It is used to provide the key during PGN 126208." }
  , { 0 }
  }
}

,
{ "Simnet: Configure Temperature Sensor", 65287, false, 0x08, 0,
  { { "Manufacturer Code", 11, RES_MANUFACTURER, false, "=1857", "Simrad" }
  , { "Reserved", 2, 1, false, 0, "" }
  , { "Industry Code", 3, RES_LOOKUP, false, LOOKUP_INDUSTRY_CODE, "" }
  }
}

,
{ "Simnet: Trim Tab Sensor Calibration", 65289, false, 0x08, 0,
  { { "Manufacturer Code", 11, RES_MANUFACTURER, false, "=1857", "Simrad" }
  , { "Reserved", 2, 1, false, 0, "" }
  , { "Industry Code", 3, RES_LOOKUP, false, LOOKUP_INDUSTRY_CODE, "" }
  }
}

,
{ "Simnet: Paddle Wheel Speed Configuration", 65290, false, 0x08, 0,
  { { "Manufacturer Code", 11, RES_MANUFACTURER, false, "=1857", "Simrad" }
  , { "Reserved", 2, 1, false, 0, "" }
  , { "Industry Code", 3, RES_LOOKUP, false, LOOKUP_INDUSTRY_CODE, "" }
  }
}

,
{ "Simnet: Clear Fluid Level Warnings", 65292, false, 0x08, 0,
  { { "Manufacturer Code", 11, RES_MANUFACTURER, false, "=1857", "Simrad" }
  , { "Reserved", 2, 1, false, 0, "" }
  , { "Industry Code", 3, RES_LOOKUP, false, LOOKUP_INDUSTRY_CODE, "" }
  }
}

,
{ "Simnet: LGC-2000 Configuration", 65293, false, 0x08, 0,
  { { "Manufacturer Code", 11, RES_MANUFACTURER, false, "=1857", "Simrad" }
  , { "Reserved", 2, 1, false, 0, "" }
  , { "Industry Code", 3, RES_LOOKUP, false, LOOKUP_INDUSTRY_CODE, "" }
  }
}

,
{ "Simnet: Reprogram Status", 65325, false, 0x08, 0,
  { { "Manufacturer Code", 11, RES_MANUFACTURER, false, "=1857", "Simrad" }
  , { "Reserved", 2, 1, false, 0, "" }
  , { "Industry Code", 3, RES_LOOKUP, false, LOOKUP_INDUSTRY_CODE, "" }
  }
}

,
{ "Simnet: Autopilot Mode", 65341, false, 0x08, 0,
  { { "Manufacturer Code", 11, RES_MANUFACTURER, false, "=1857", "Simrad" }
  , { "Reserved", 2, 1, false, 0, "" }
  , { "Industry Code", 3, RES_LOOKUP, false, LOOKUP_INDUSTRY_CODE, "" }
  }
}

,
{ "Seatalk: Keypad Message", 65371, false, 0x08, 0,
  { { "Manufacturer Code", 11, RES_MANUFACTURER, false, "=1851", "Raymarine" }
  , { "Reserved", 2, 1, false, 0, "" }
  , { "Industry Code", 3, RES_LOOKUP, false, "=4", "Marine Industry" }
  , { "Proprietary ID", BYTES(1), 1, false, 0, "" }
  , { "First key", BYTES(1), 1, false, 0, "" }
  , { "Second key", BYTES(1), 1, false, 0, "" }
  , { "First key state", 2, 1, false, 0, "" }
  , { "Second key state", 2, 1, false, 0, "" }
  , { "Reserved", 4, 1, false, 0, "" }
  , { "Encoder Position", BYTES(1), 1, false, 0, "" }
  }
}

,
{ "SeaTalk: Keypad Heartbeat", 65374, false, 0x08, 0,
  { { "Manufacturer Code", 11, RES_MANUFACTURER, false, "=1851", "Raymarine" }
  , { "Reserved", 2, 1, false, 0, "" }
  , { "Industry Code", 3, RES_LOOKUP, false, "=4", "Marine Industry" }
  , { "Proprietary ID", BYTES(1), 1, false, 0, "" }
  , { "Variant", BYTES(1), 1, false, 0, "" }
  , { "Status", BYTES(1), 1, false, 0, "" }
  }
}

  /* http://www.airmartechnology.com/uploads/installguide/DST200UserlManual.pdf */
,
{ "Airmar: Depth Quality Factor", 65408, false, 8, 0,
  { { "Manufacturer Code", 11, RES_MANUFACTURER, false, "=135", "Airmar" }
  , { "Reserved", 2, 1, false, 0, "" }
  , { "Industry Code", 3, RES_LOOKUP, false, "=4", "Marine Industry" }
  , { "SID", BYTES(1), 1, false, 0, "" }
  , { "Depth Quality Factor", 4, RES_LOOKUP, false, ",0=No Depth Lock", "" }
  , { 0 }
  }
}

  /* http://www.airmartechnology.com/uploads/installguide/DST200UserlManual.pdf */
,
{ "Airmar: Device Information", 65410, false, 8, 0,
  { { "Manufacturer Code", 11, RES_MANUFACTURER, false, 0, "" }
  , { "Reserved", 2, 1, false, 0, "" }
  , { "Industry Code", 3, RES_LOOKUP, false, LOOKUP_INDUSTRY_CODE, "" }
  , { "SID", BYTES(1), 1, false, 0, "" }
  , { "Internal Device Temperature", BYTES(2), RES_TEMPERATURE, false, "K", "" }
  , { "Supply Voltage", BYTES(2), 0.01, false, "V", "" }
  , { "Reserved", BYTES(1), 1, false, 0, "" }
  , { 0 }
  }
}

,
{ "Simnet: Autopilot Mode", 65480, false, 0x08, 0,
  { { "Manufacturer Code", 11, RES_MANUFACTURER, false, "=1857", "Simrad" }
  , { "Reserved", 2, 1, false, 0, "" }
  , { "Industry Code", 3, RES_LOOKUP, false, LOOKUP_INDUSTRY_CODE, "" }
  , { 0 }
  }
}

  /* http://www.maretron.com/support/manuals/DST100UM_1.2.pdf */
,
{ "NMEA - Request group function", 126208, true, 8, 2,
  { { "Function Code", BYTES(1), RES_INTEGER, false, "=0", "Request" }
  , { "PGN", BYTES(3), RES_INTEGER, false, 0, "Requested PGN" }
  , { "Transmission interval", BYTES(4), 1, false, 0, "" }
  , { "Transmission interval offset", BYTES(2), 1, false, 0, "" }
  , { "# of Requested Parameters", 8, 1, false, 0, "How many parameter pairs will follow" }
  , { "Parameter Index", 8, RES_INTEGER, false, 0, "Parameter index" }
  , { "Parameter Value", LEN_VARIABLE, RES_INTEGER, false, 0, "Parameter value, variable length" }
  , { 0 }
  }
}

,
{ "NMEA - Command group function", 126208, true, 8, 2,
  { { "Function Code", BYTES(1), RES_INTEGER, false, "=1", "Command" }
  , { "PGN", BYTES(3), RES_INTEGER, false, 0, "Commanded or requested PGN" }
  , { "Priority", 4, 1, false, 0, ",8=Leave priority unchanged" }
  , { "Reserved", 4, 1, false, 0, "" }
  , { "# of Commanded Parameters", BYTES(1), 1, false, 0, "How many parameter pairs will follow" }
  , { "Parameter Index", BYTES(1), RES_INTEGER, false, 0, "Parameter index" }
  , { "Parameter Value", LEN_VARIABLE, RES_INTEGER, false, 0, "Parameter value, variable length" }
  , { 0 }
  }
}

,
{ "NMEA - Acknowledge group function", 126208, true, 8, 1,
  { { "Function Code", BYTES(1), RES_INTEGER, false, "=2", "Acknowledge" }
  , { "PGN", 24, RES_INTEGER, false, 0, "Commanded or requested PGN" }
  , { "PGN error code", 4, 1, false, 0, "" }
  , { "Transmission interval/Priority error code", 4, 1, false, 0, "" }
  , { "# of Commanded Parameters", 8, 1, false, 0, "" }
  , { "Parameter Error", 8, RES_INTEGER, false, 0, "" }
  , { 0 }
  }
}

  /************ RESPONSE TO REQUEST PGNS **************/

,
{ "Maretron: Slave Response", 126270, false, 8, 0,
  { { "Manufacturer Code", 11, RES_MANUFACTURER, false, 0, "" }
  , { "Reserved", 2, 1, false, 0, "" }
  , { "Industry Code", 3, RES_LOOKUP, false, LOOKUP_INDUSTRY_CODE, "" }
  , { "Product code", BYTES(2), 1, false, 0, "0x1b2=SSC200" }
  , { "Software code", BYTES(2), 1, false, 0, "" }
  , { "Command", BYTES(1), 1, false, 0, "0x50=Deviation calibration result" }
  , { "Status", BYTES(1), 1, false, 0, "" }
  , { 0 }
  }
}

,
{ "PGN List (Transmit and Receive)", 126464, false, 8, 1,
  { { "Function Code", BYTES(1), RES_LOOKUP, false, ",0=Transmit PGN list,1=Receive PGN list", "Transmit or receive PGN Group Function Code" }
  , { "PGN", 24, RES_INTEGER, false, 0, }
  , { 0 }
  }
}

  /* http://www.airmartechnology.com/uploads/installguide/PB200UserManual.pdf */
,
{ "Airmar: Attitude Offset", 126720, true, 9, 0,
  { { "Manufacturer Code", 11, RES_MANUFACTURER, false, "=135", "Airmar" }
  , { "Reserved", 2, RES_NOTUSED, false, 0, "" }
  , { "Industry Code", 3, RES_LOOKUP, false, "=4", "Marine Industry" }
  , { "Proprietary ID", BYTES(1), RES_INTEGER, false, "=32", "Attitude Offsets" }
  , { "Azimuth offset", BYTES(2), RES_DEGREES, true, "deg", "Positive: sensor rotated to port, negative: sensor rotated to starboard" }
  , { "Pitch offset", BYTES(2), RES_DEGREES, true, "deg", "Positive: sensor tilted to bow, negative: sensor tilted to stern" }
  , { "Roll offset", BYTES(2), RES_DEGREES, true, "deg", "Positive: sensor tilted to port, negative: sensor tilted to starboard" }
  , { 0 }
  }
}

  /* http://www.airmartechnology.com/uploads/installguide/PB200UserManual.pdf */
,
{ "Airmar: Calibrate Compass", 126720, true, 24, 0,
  { { "Manufacturer Code", 11, RES_MANUFACTURER, false, "=135", "Airmar" }
  , { "Reserved", 2, RES_NOTUSED, false, 0, "" }
  , { "Industry Code", 3, RES_LOOKUP, false, "=4", "Marine Industry" }
  , { "Proprietary ID", BYTES(1), RES_INTEGER, false, "=33", "Calibrate Compass" }
  , { "Calibrate Function", BYTES(1), RES_LOOKUP, false, ",0=Normal/cancel calibration,1=Enter calibration mode,2=Reset calibration to 0,3=Verify,4=Reset compass to defaults,5=Reset damping to defaults", "" }
  , { "Calibration Status", BYTES(1), RES_LOOKUP, false, ",0=Queried,1=Passed,2=Failed - timeout,3=Failed - tilt error,4=Failed - other,5=In progress", "" }
  , { "Verify Score", BYTES(1), RES_INTEGER, false, 0, "TBD" }
  , { "X-axis gain value", BYTES(2), 0.01, true, 0, "default 100, range 50 to 500" }
  , { "Y-axis gain value", BYTES(2), 0.01, true, 0, "default 100, range 50 to 500" }
  , { "Z-axis gain value", BYTES(2), 0.01, true, 0, "default 100, range 50 to 500" }
  , { "X-axis linear offset", BYTES(2), 0.01, true, "Tesla", "default 0, range -320.00 to 320.00" }
  , { "Y-axis linear offset", BYTES(2), 0.01, true, "Tesla", "default 0, range -320.00 to 320.00" }
  , { "Z-axis linear offset", BYTES(2), 0.01, true, "Tesla", "default 0, range -320.00 to 320.00" }
  , { "X-axis angular offset", BYTES(2), 0.1, true, "deg", "default 0, range 0 to 3600" }
  , { "Pitch and Roll damping", BYTES(2), 0.05, true, "s", "default 30, range 0 to 200" }
  , { "Compass/Rate gyro damping", BYTES(2), 0.05, true, "s", "default -30, range -2400 to 2400, negative indicates rate gyro is to be used in compass calculations" }
  , { 0 }
  }
}

  /* http://www.airmartechnology.com/uploads/installguide/PB200UserManual.pdf */
,
{ "Airmar: True Wind Options", 126720, true, 6, 0,//FIXME Single Frame: No
  { { "Manufacturer Code", 11, RES_MANUFACTURER, false, "=135", "Airmar" }
  , { "Reserved", 2, RES_NOTUSED, false, 0, "" }
  , { "Industry Code", 3, RES_LOOKUP, false, "=4", "Marine Industry" }
  , { "Proprietary ID", BYTES(1), RES_INTEGER, false, "=34", "True Wind Options" }
  , { "COG substition for HDG", 2, RES_LOOKUP, false, ",0=Use HDG only,1=Allow COG to replace HDG", "Allow use of COG when HDG not available?" }
  , { "Calibration Status", BYTES(1), RES_LOOKUP, false, ",0=Queried,1=Passed,2=Failed - timeout,3=Failed - tilt error,4=Failed - other,5=In progress", "" }
  , { "Verify Score", BYTES(1), RES_INTEGER, false, 0, "TBD" }
  , { "X-axis gain value", BYTES(2), 0.01, true, 0, "default 100, range 50 to 500" }
  , { "Y-axis gain value", BYTES(2), 0.01, true, 0, "default 100, range 50 to 500" }
  , { "Z-axis gain value", BYTES(2), 0.01, true, 0, "default 100, range 50 to 500" }
  , { "X-axis linear offset", BYTES(2), 0.01, true, "Tesla", "default 0, range -320.00 to 320.00" }
  , { "Y-axis linear offset", BYTES(2), 0.01, true, "Tesla", "default 0, range -320.00 to 320.00" }
  , { "Z-axis linear offset", BYTES(2), 0.01, true, "Tesla", "default 0, range -320.00 to 320.00" }
  , { "X-axis angular offset", BYTES(2), 0.1, true, "deg", "default 0, range 0 to 3600" }
  , { "Pitch and Roll damping", BYTES(2), 0.05, true, "s", "default 30, range 0 to 200" }
  , { "Compass/Rate gyro damping", BYTES(2), 0.05, true, "s", "default -30, range -2400 to 2400, negative indicates rate gyro is to be used in compass calculations" }
  , { 0 }
  }
}

  /* http://www.airmartechnology.com/uploads/installguide/DST200UserlManual.pdf */
,
{ "Airmar: Simulate Mode", 126720, true, 6, 0,//FIXME Single Frame: No
  { { "Manufacturer Code", 11, RES_MANUFACTURER, false, "=135", "Airmar" }
  , { "Reserved", 2, RES_NOTUSED, false, 0, "" }
  , { "Industry Code", 3, RES_LOOKUP, false, "=4", "Marine Industry" }
  , { "Proprietary ID", BYTES(1), RES_INTEGER, false, "=35", "Simulate Mode" }
  , { "Simulate Mode", 2, RES_LOOKUP, false, ",0=Off,1=On", "" }
  , { "Reserved", 22, RES_BINARY, false, 0, "Reserved" }
  , { 0 }
  }
}

  /* http://www.airmartechnology.com/uploads/installguide/DST200UserlManual.pdf */
,
{ "Airmar: Calibrate Depth", 126720, true, 6, 0,//FIXME Single Frame: No
  { { "Manufacturer Code", 11, RES_MANUFACTURER, false, "=135", "Airmar" }
  , { "Reserved", 2, RES_NOTUSED, false, 0, "" }
  , { "Industry Code", 3, RES_LOOKUP, false, "=4", "Marine Industry" }
  , { "Proprietary ID", BYTES(1), RES_INTEGER, false, "=40", "Calibrate Depth" }
  , { "Speed of Sound Mode", BYTES(2), 0.1, false, "m/s", "actual allowed range is 1350.0 to 1650.0 m/s" }
  , { "Reserved", 8, RES_BINARY, false, 0, "Reserved" }
  , { 0 }
  }
}

  /* http://www.airmartechnology.com/uploads/installguide/DST200UserlManual.pdf */
,
{ "Airmar: Calibrate Speed", 126720, true, 12, 2,
  { { "Manufacturer Code", 11, RES_MANUFACTURER, false, "=135", "Airmar" }
  , { "Reserved", 2, RES_NOTUSED, false, 0, "" }
  , { "Industry Code", 3, RES_LOOKUP, false, "=4", "Marine Industry" }
  , { "Proprietary ID", BYTES(1), RES_INTEGER, false, "=41", "Calibrate Speed" }
  , { "Number of pairs of data points", BYTES(1), RES_INTEGER, false, 0, "actual range is 0 to 25. 254=restore default speed curve"
  }
  , { "Input frequency", BYTES(2), 0.1, false, "Hz", "" }
  , { "Output speed", BYTES(2), 0.01, false, "m/s", "" }
  , { 0 }
  }
}

  /* http://www.airmartechnology.com/uploads/installguide/DST200UserlManual.pdf */
,
{ "Airmar: Calibrate Temperature", 126720, true, 6, 2,//FIXME Single Frame: No
  { { "Manufacturer Code", 11, RES_MANUFACTURER, false, "=135", "Airmar" }
  , { "Reserved", 2, RES_NOTUSED, false, 0, "" }
  , { "Industry Code", 3, RES_LOOKUP, false, "=4", "Marine Industry" }
  , { "Proprietary ID", BYTES(1), RES_INTEGER, false, "=42", "Calibrate Temperature" }
  , { "Temperature instance", 2, RES_LOOKUP, false, ",0=Device Sensor,1=Onboard Water Sensor,2=Optional Water Sensor", "" }
  , { "Reserved", 6, RES_BINARY, false, 0, "Reserved" }
  , { "Temperature offset", BYTES(2), 0.1, false, "Hz", "" }
  , { "Temperature offset", BYTES(2), 0.001, true, "K", "actual range is -9.999 to +9.999 K" }
  , { 0 }
  }
}

  /* http://www.airmartechnology.com/uploads/installguide/DST200UserlManual.pdf */
,
{ "Airmar: Speed Filter", 126720, true, 8, 2,//FIXME Single Frame: No (type 0: NDB=6; type 1:NDB=8)
  { { "Manufacturer Code", 11, RES_MANUFACTURER, false, "=135", "Airmar" }
  , { "Reserved", 2, RES_NOTUSED, false, 0, "" }
  , { "Industry Code", 3, RES_LOOKUP, false, "=4", "Marine Industry" }
  , { "Proprietary ID", BYTES(1), RES_INTEGER, false, "=43", "Speed Filter" }
  , { "Filter type", 4, RES_LOOKUP, false, ",0=no filter,1=basic IIR filter", "" }
  , { "Reserved", 4, RES_BINARY, false, 0, "Reserved" }
  , { "Sample interval", BYTES(2), 0.01, false, "s", "" }
  , { "Filter duration", BYTES(2), 0.01, false, "s", "" }
  , { 0 }
  }
}

  /* http://www.airmartechnology.com/uploads/installguide/DST200UserlManual.pdf */
,
{ "Airmar: Temperature Filter", 126720, true, 8, 2,//FIXME Single Frame: No (type 0: NDB=6; type 1:NDB=8)
  { { "Manufacturer Code", 11, RES_MANUFACTURER, false, "=135", "Airmar" }
  , { "Reserved", 2, RES_NOTUSED, false, 0, "" }
  , { "Industry Code", 3, RES_LOOKUP, false, "=4", "Marine Industry" }
  , { "Proprietary ID", BYTES(1), RES_INTEGER, false, "=44", "Temperature Filter" }
  , { "Filter type", 4, RES_LOOKUP, false, ",0=no filter,1=basic IIR filter,15=data not available", "" }
  , { "Reserved", 4, RES_BINARY, false, 0, "Reserved" }
  , { "Sample interval", BYTES(2), 0.01, false, "s", "" }
  , { "Filter duration", BYTES(2), 0.01, false, "s", "" }
  , { 0 }
  }
}

  /* http://www.airmartechnology.com/uploads/installguide/DST200UserlManual.pdf */
,
{ "Airmar: NMEA 2000 options", 126720, true, 6, 2,//FIXME Single Frame: No
  { { "Manufacturer Code", 11, RES_MANUFACTURER, false, "=135", "Airmar" }
  , { "Reserved", 2, RES_NOTUSED, false, 0, "" }
  , { "Industry Code", 3, RES_LOOKUP, false, "=4", "Marine Industry" }
  , { "Proprietary ID", BYTES(1), RES_INTEGER, false, "=46", "NMEA 2000 options" }
  , { "Transmission Interval", 2, RES_LOOKUP, false, ",0=Measure Interval,1=Requested by user,2=reserved,3=data not available", "" }
  , { "Reserved", 22, RES_BINARY, false, 0, "Reserved" }
  , { 0 }
  }
}

,
{ "Airmar: Addressable Multi-Frame", 126720, true, 4, 0,//FIXME Single Frame: No
  { { "Manufacturer Code", 11, RES_MANUFACTURER, false, "=135", "Airmar" }
  , { "Reserved", 2, RES_NOTUSED, false, 0, "" }
  , { "Industry Code", 3, RES_LOOKUP, false, "=4", "Marine Industry" }
  , { "Proprietary ID", BYTES(1), RES_INTEGER, false, 0, "" }
  , { 0 }
  }
}

,
{ "Manufacturer Proprietary: Addressable Multi-Frame", 126720, true, 3, 0,//FIXME Single Frame: No
  { { "Manufacturer Code", 11, RES_MANUFACTURER, false, 0, "" }
  , { "Reserved", 2, RES_NOTUSED, false, 0, "" }
  , { "Industry Code", 3, RES_LOOKUP, false, LOOKUP_INDUSTRY_CODE, "" }
  , { 0 }
  }
}

,
{ "Alert", 126983, false, 8, 0,
  { { 0 }
  }
}

,
{ "Alert Response", 126984, false, 8, 0,
  { { 0 }
  }
}

,
{ "Alert Text", 126985, false, 8, 0,
  { { 0 }
  }
}

,
{ "Alert Configuration", 126986, false, 8, 0,
  { { 0 }
  }
}

,
{ "Alert Threshold", 126987, false, 8, 0,
  { { 0 }
  }
}

,
{ "Alert Value", 126988, false, 8, 0,
  { { 0 }
  }
}

  /* http://www.maretron.com/support/manuals/GPS100UM_1.2.pdf */
,
{ "System Time", 126992, true, 8, 0,
  { { "SID", BYTES(1), 1, false, 0, "" }
  , { "Source", 4, RES_LOOKUP, false, LOOKUP_SYSTEM_TIME, "" }
  , { "Reserved", 4, RES_BINARY, false, 0, "Reserved" }
  , { "Date", BYTES(2), RES_DATE, false, "days", "Days since January 1, 1970" }
  , { "Time", BYTES(4), RES_TIME, false, "s", "Seconds since midnight" }
  , { 0 }
  }
}

  /* http://www.nmea.org/Assets/20130905%20nmea%202000%20heartbeat%20amendment%20final.pdf */
,
{ "Heartbeat", 126993, true, 8, 0,
  { { "Data transmit offset", BYTES(2), 0.01, false, "s", "Offset in transmit time from time of request command: 0x0 = transmit immediately, 0xFFFF = Do not change offset." }
  , { "Sequence Counter", BYTES(1), RES_INTEGER, false, 0, "" }
  , { "Reserved", BYTES(3), RES_BINARY, false, 0, "Reserved" }
  , { 0 }
  }
}

,
{ "Product Information", 126996, false, 0x86, 0,
  { { "NMEA 2000 Version", BYTES(2), 1, false, 0, "" }
  , { "Product Code", BYTES(2), 1, false, 0, "" }
  , { "Model ID", BYTES(32), RES_ASCII, false, 0, "" }
  , { "Software Version Code", BYTES(32), RES_ASCII, false, 0, "" }
  , { "Model Version", BYTES(32), RES_ASCII, false, 0, "" }
  , { "Model Serial Code", BYTES(32), RES_ASCII, false, 0, "" }
  , { "Certification Level", BYTES(1), 1, false, 0, "" }
  , { "Load Equivalency", BYTES(1), 1, false, 0, "" }
  , { 0 }
  }
}

,
{ "Configuration Information", 126998, false, 0x2a, 0,
  { { "Station ID", BYTES(2), 1, false, 0, "" }
  , { "Station Name", BYTES(2), 1, false, 0, "" }
  , { "A", BYTES(2), 1, false, 0, "" }
  , { "Manufacturer", BYTES(36), RES_ASCII, false, 0, "" }
  , { "Installation Description #1", BYTES(2), 1, false, 0, "" }
  , { "Installation Description #2", BYTES(2), 1, false, 0, "" }
  , { 0 }
  }
}

  /************ PERIODIC DATA PGNs **************/
  /* http://www.nmea.org/Assets/july%202010%20nmea2000_v1-301_app_b_pgn_field_list.pdf */
  /* http://www.maretron.com/support/manuals/USB100UM_1.2.pdf */
  /* http://www8.garmin.com/manuals/GPSMAP4008_NMEA2000NetworkFundamentals.pdf */

  /* http://www.nmea.org/Assets/20130906%20nmea%202000%20%20man%20overboard%20notification%20%28mob%29%20pgn%20127233%20amendment.pdf */
,
{ "Man Overboard Notification", 127233, true, 35, 0,
  { { "SID", BYTES(1), 1, false, 0, "" }
  , { "MOB Emitter ID", BYTES(4), RES_INTEGER, false, 0, "Identifier for each MOB emitter, unique to the vessel" }
  , { "Man Overboard Status", 3, RES_LOOKUP, false, "0=MOB Emitter Activated,1=Manual on-board MOB Button Activation,2=Test Mode,3=MOB Not Active", "" }
  , { "Reserved", 5, 1, false, 0, "" }
  , { "Activation Time", BYTES(4), RES_TIME, false, "s", "Time of day (UTC) when MOB was activated" }
  , { "Position Source", 3, RES_LOOKUP, false, "0=Position estimated by the Vessel,1=Position reported by MOB emitter", "" }
  , { "Reserved", 5, 1, false, 0, "" }
  , { "Position Date", BYTES(2), RES_DATE, false, "", "Date of MOB position" }
  , { "Position Time", BYTES(4), RES_TIME, false, "s", "Time of day of MOB position (UTC)" }
  , { "Latitude", BYTES(4), RES_LATITUDE, true, "deg", "" }
  , { "Longitude", BYTES(4), RES_LONGITUDE, true, "deg", "" }
  , { "COG Reference", 2, RES_LOOKUP, false, LOOKUP_DIRECTION_REFERENCE, "" }
  , { "Reserved", 6, 1, false, 0, "" }
  , { "COG", BYTES(2), RES_DEGREES, false, "deg", "" }
  , { "SOG", BYTES(2), 0.01, false, "m/s", "" }
  , { "MMSI of vessel of origin", BYTES(4), RES_INTEGER, false, "MMSI", "" }
  , { "MOB Emitter Battery Status", 3, RES_LOOKUP, false, "0=Good,1=Low", "" }
  , { "Reserved", 5, 1, false, 0, "" }
  , { 0 }
  }
}

,
{ "Heading/Track control", 127237, false, 0x15, 0,
  { { "Rudder Limit Exceeded", 2, 1, false, 0, "" }
  , { "Off-Heading Limit Exceeded", 2, 1, false, 0, "" }
  , { "Off-Track Limit Exceeded", 2, 1, false, 0, "" }
  , { "Override", 2, 1, false, 0, "" }
  , { "Steering Mode", 4, 1, false, 0, "" }
  , { "Turn Mode", 4, 1, false, 0, "" }
  , { "Heading Reference", 3, 1, false, 0, "" }
  , { "Reserved", 3, 1, false, 0, "" }
  , { "Commanded Rudder Direction", 2, 1, false, 0, "" }
  , { "Commanded Rudder Angle", BYTES(2), RES_DEGREES, true, "deg", "" }
  , { "Heading-To-Steer (Course)", BYTES(2), RES_DEGREES, false, "deg", "" }
  , { "Track", BYTES(2), RES_DEGREES, false, "deg", "" }
  , { "Rudder Limit", BYTES(2), RES_DEGREES, false, "deg", "" }
  , { "Off-Heading Limit", BYTES(2), RES_DEGREES, false, "deg", "" }
  , { "Radius of Turn Order", BYTES(2), RES_DEGREES, true, "deg", "" }
  , { "Rate of Turn Order", BYTES(2), RES_ROTATION, true, "deg/s", "" }
  , { "Off-Track Limit", BYTES(2), 1, true, "m", "" }
  , { "Vessel Heading", BYTES(2), RES_DEGREES, false, "deg", "" }
  , { 0 }
  }
}


  /* http://www.maretron.com/support/manuals/RAA100UM_1.0.pdf */
  /* Haven't actually seen this value yet, lengths are guesses */
,
{ "Rudder", 127245, false, 8, 0,
  { { "Instance", BYTES(1), 1, false, 0, "" }
  , { "Direction Order", 2, 1, false, 0, "" }
  , { "Reserved", 6, RES_BINARY, false, 0, "Reserved" }
  , { "Angle Order", BYTES(2), RES_DEGREES, true, "deg", "" }
  , { "Position", BYTES(2), RES_DEGREES, true, "deg", "" }
  , { 0 }
  }
}

  /* NMEA + Simrad AT10 */
  /* http://www.maretron.com/support/manuals/SSC200UM_1.7.pdf */
  /* molly_rose_E80start.kees */
,
{ "Vessel Heading", 127250, true, 8, 0,
  { { "SID", BYTES(1), 1, false, 0, "" }
  , { "Heading", BYTES(2), RES_DEGREES, false, "deg", "" }
  , { "Deviation", BYTES(2), RES_DEGREES, true, "deg", "" }
  , { "Variation", BYTES(2), RES_DEGREES, true, "deg", "" }
  , { "Reference", 2, RES_LOOKUP, false, LOOKUP_DIRECTION_REFERENCE, "" }
  , { 0 }
  }
}

  /* http://www.maretron.com/support/manuals/SSC200UM_1.7.pdf */
  /* Lengths observed from Simrad RC42 */
,
{ "Rate of Turn", 127251, true, 5, 0,
  { { "SID", BYTES(1), 1, false, 0, "" }
  , { "Rate", BYTES(4), RES_ROTATION * 0.0001, true, "deg/s", "" }
  , { 0 }
  }
}

,
{ "Attitude", 127257, true, 7, 0,
  { { "SID", BYTES(1), 1, false, 0, "" }
  , { "Yaw", BYTES(2), RES_ROTATION, true, "deg/s", "" }
  , { "Pitch", BYTES(2), RES_ROTATION, true, "deg/s", "" }
  , { "Roll", BYTES(2), RES_ROTATION, true, "deg/s", "" }
  , { 0 }
  }
}

  /* NMEA + Simrad AT10 */
  /* http://www.maretron.com/support/manuals/GPS100UM_1.2.pdf */
,
{ "Magnetic Variation", 127258, true, 6, 0,
  { { "SID", BYTES(1), 1, false, 0, "" }
  , { "Source", 4, RES_LOOKUP, false, LOOKUP_MAGNETIC_VARIATION, "" }
  , { "Reserved", 4, RES_BINARY, false, 0, "Reserved" }
  , { "Age of service", BYTES(2), RES_DATE, false, "days", "Days since January 1, 1970" }
  , { "Variation", BYTES(2), RES_DEGREES, true, "deg", "" }
  , { 0 }
  }
}
  /* Engine group PGNs all derived PGN Numbers from */
  /* http://www.maretron.com/products/pdf/J2K100-Data_Sheet.pdf */
  /* http://www.floscan.com/html/blue/NMEA2000.php */

,
{ "Engine Parameters, Rapid Update", 127488, true, 8, 0,
  { { "Engine Instance", BYTES(1), RES_LOOKUP, false, LOOKUP_ENGINE_INSTANCE, "" }
  , { "Engine Speed", BYTES(2), RES_INTEGER, false, "rpm", "" }
  , { "Engine Boost Pressure", BYTES(2), RES_PRESSURE, false, "hPa", "" }
  , { "Engine Tilt/Trim", BYTES(1), 1, true, "", "" }
  , { 0 }
  }
}

,
{ "Engine Parameters, Dynamic", 127489, true, 26, 0,
  { { "Engine Instance", BYTES(1), RES_LOOKUP, false, LOOKUP_ENGINE_INSTANCE, "" }
  , { "Oil pressure", BYTES(2), RES_PRESSURE, false, 0, "hPa" }
  , { "Oil temperature", BYTES(2), RES_TEMPERATURE, false, "K", "" }
  , { "Temperature", BYTES(2), RES_TEMPERATURE, false, "K", "" }
  , { "Alternator Potential", BYTES(2), 0.01, false, "V", "" }
  , { "Fuel Rate", BYTES(2), 0.1, true, "L/h", "" }
  , { "Total Engine hours", BYTES(4), 1.0, false, "s", "" }
  , { "Coolant Pressure", BYTES(2), RES_PRESSURE, false, "hPa", "" }
  , { "Fuel Pressure", BYTES(2), 1, false, 0, "" }
  , { "Reserved", BYTES(1), 1, false, 0, "" }
  , { "Discrete Status 1", BYTES(2), RES_INTEGER, false, 0, "" }
  , { "Discrete Status 2", BYTES(2), RES_INTEGER, false, 0, "" }
  , { "Percent Engine Load", BYTES(1), RES_INTEGER, true, "%", "" }
  , { "Percent Engine Torque", BYTES(1), RES_INTEGER, true, "%", "" }
  , { 0 }
  }
}

,
{ "Transmission Parameters, Dynamic", 127493, true, 8, 0,
  { { "Engine Instance", 8, RES_LOOKUP, false, LOOKUP_ENGINE_INSTANCE, "" }
  , { "Transmission Gear", 2, RES_LOOKUP, false, LOOKUP_GEAR_STATUS, "" }
  , { "Reserved", 6, RES_NOTUSED, false, 0, "" }
  , { "Oil pressure", BYTES(2), RES_PRESSURE, false, "hPa", "" }
  , { "Oil temperature", BYTES(2), RES_TEMPERATURE, false, "K", "" }
  , { "Discrete Status 1", BYTES(1), RES_INTEGER, false, 0, "" }
  , { "Reserved", BYTES(1), RES_NOTUSED, false, 0, "" }
  , { 0 }
  }
}

,
{ "Trip Parameters, Vessel", 127496, true, 10, 0,
  { { "Time to Empty", BYTES(4), 0.001, false, "s", "" }
  , { "Distance to Empty", BYTES(4), 0.01, false, "m", "" }
  , { "Estimated Fuel Remaining", BYTES(2), 1, false, "L", "" }
  , { "Trip Run Time", BYTES(4), 0.001, false, "s", "" }
  , { 0 }
  }
}

,
{ "Trip Parameters, Engine", 127497, true, 9, 0,
  { { "Engine Instance", BYTES(1), RES_LOOKUP, false, LOOKUP_ENGINE_INSTANCE, "" }
  , { "Trip Fuel Used", BYTES(2), 1, false, "L", "" }
  , { "Fuel Rate, Average", BYTES(2), 0.1, true, "L/h", "" }
  , { "Fuel Rate, Economy", BYTES(2), 0.1, true, "L/h", "" }
  , { "Instantaneous Fuel Economy", BYTES(2), 0.1, true, "L/h", "" }
  , { 0 }
  }
}

,
{ "Engine Parameters, Static", 127498, true, 8, 0,
  { { "Engine Instance", BYTES(1), RES_LOOKUP, false, LOOKUP_ENGINE_INSTANCE, "" }
  , { "Rated Engine Speed", BYTES(2), 1, false, 0, "" }
  , { "VIN", BYTES(1), 1, false, 0, "" }
  , { "Software ID", BYTES(2), 1, false, 0, "" }
  , { 0 }
  }
}

,
{ "Binary Switch Bank Status", 127501, false, 8, 1,
  { { "Indicator Bank Instance", BYTES(1), 1, false, 0, "" }
  , { "Indicator", 2, RES_LOOKUP, false, ",0=Off,1=On", "" }
  , { 0 }
  }
}

,
{ "Switch Bank Control", 127502, false, 8, 1,
  { { "Switch Bank Instance", BYTES(1), 1, false, 0, "" }
  , { "Switch", 2, RES_LOOKUP, false, ",0=Off,1=On", "" }
  , { 0 }
  }
}

  /* http://www.nmea.org/Assets/nmea-2000-corrigendum-1-2010-1.pdf */
,
{ "AC Input Status", 127503, true, 8, 10,
  { { "AC Instance", BYTES(1), 1, false, 0, "" }
  , { "Number of Lines", BYTES(1), 1, false, 0, "" }

  , { "Line", 2, RES_LOOKUP, false, ",0=Line 1,1=Line 2,2=Line 3,3=Reserved", "" }
  , { "Acceptability", 2, RES_LOOKUP, false, ",0=Bad Level,1=Bad Frequency,2=Being Qualified,3=Good", "" }
  , { "Reserved", 4, 1, false, 0, "" }
  , { "Voltage", BYTES(2), 0.01, false, "V", "" }
  , { "Current", BYTES(2), 0.1, false, "A", "" }
  , { "Frequency", BYTES(2), 0.01, false, "Hz", "" }
  , { "Breaker Size", BYTES(2), 0.1, false, "A", "" }
  , { "Real Power", BYTES(4), RES_INTEGER, false, "W", "" }
  , { "Reactive Power", BYTES(4), RES_INTEGER, false, "VAR", "" }
  , { "Power Factor", BYTES(1), 0.01, false, "Cos Phi", "" }
  , { 0 }
  }
}

  /* http://www.nmea.org/Assets/nmea-2000-corrigendum-1-2010-1.pdf */
,
{ "AC Output Status", 127504, true, 8, 10,
  { { "AC Instance", BYTES(1), 1, false, 0, "" }
  , { "Number of Lines", BYTES(1), 1, false, 0, "" }

  , { "Line", 2, RES_LOOKUP, false, ",0=Line 1,1=Line 2,2=Line 3", "" }
  , { "Waveform", 3, RES_LOOKUP, false, ",0=Sine Wave,1=Modified Sine Wave,6=Error,7=Data Not Available", "" }
  , { "Reserved", 3, 1, false, 0, "" }
  , { "Voltage", BYTES(2), 0.01, false, "V", "" }
  , { "Current", BYTES(2), 0.1, false, "A", "" }
  , { "Frequency", BYTES(2), 0.01, false, "Hz", "" }
  , { "Breaker Size", BYTES(2), 0.1, false, "A", "" }
  , { "Real Power", BYTES(4), RES_INTEGER, false, "W", "" }
  , { "Reactive Power", BYTES(4), RES_INTEGER, false, "VAR", "" }
  , { "Power Factor", BYTES(1), 0.01, false, "Cos Phi", "" }
  , { 0 }
  }
}

  /* http://www.maretron.com/support/manuals/TLA100UM_1.2.pdf */
  /* Observed from EP65R */
,
{ "Fluid Level", 127505, true, 7, 0,
  { { "Instance", 4, 1, false, 0, "" }
  , { "Type", 4, RES_LOOKUP, false, ",0=Fuel,1=Water,2=Gray water,3=Live well,4=Oil,5=Black water", "" }
  , { "Level", BYTES(2), 100.0/25000, false, "%", "" }
  , { "Capacity", BYTES(4), 0.1, false, "L", "" }
  , { 0 }
  }
}

,
{ "DC Detailed Status", 127506, false, 8, 0,
  { { "SID", BYTES(1), 1, false, 0, "" }
  , { "DC Instance", BYTES(1), 1, false, 0, "" }
  , { "DC Type", BYTES(1), 1, false, 0, "" }
  , { "State of Charge", BYTES(1), 1, false, 0, "" }
  , { "State of Health", BYTES(1), 1, false, 0, "" }
  , { "Time Remaining", BYTES(2), 1, false, 0, "" }
  , { "Ripple Voltage", BYTES(2), 0.01, false, "V", "" }
  , { 0 }
  }
}

,
{ "Charger Status", 127507, false, 8, 0,
  { { "Charger Instance", BYTES(1), 1, false, 0, "" }
  , { "Battery Instance", BYTES(1), 1, false, 0, "" }
  , { "Operating State", BYTES(1), 1, false, 0, "" }
  , { "Charge Mode", BYTES(1), 1, false, 0, "" }
  , { "Charger Enable/Disable", 2, 1, false, 0, "" }
  , { "Equalization Pending", 2, 1, false, 0, "" }
  , { "Reserved", 4, 1, false, 0, "" }
  , { "Equalization Time Remaining", BYTES(2), 1, false, 0, "" }
  , { 0 }
  }
}

,
{ "Battery Status", 127508, true, 8, 0,
  { { "Battery Instance", BYTES(1), 1, false, 0, "" }
  , { "Voltage", BYTES(2), 0.01, true, "V", "" }
  , { "Current", BYTES(2), 0.1, true, "A", "" }
  , { "Temperature", BYTES(2), RES_TEMPERATURE, false, "K", "" }
  , { "SID", BYTES(1), 1, false, 0, "" }
  , { 0 }
  }
}

,
{ "Inverter Status", 127509, false, 8, 0,
  { { "Inverter Instance", BYTES(1), 1, false, 0, "" }
  , { "AC Instance", BYTES(1), 1, false, 0, "" }
  , { "DC Instance", BYTES(1), 1, false, 0, "" }
  , { "Operating State", 4, RES_LOOKUP, false, ",0=Standby,1=On", "" }
  , { "Inverter", 2, RES_LOOKUP, false, ",0=Standby,1=On", "" }
  , { 0 }
  }
}

,
{ "Charger Configuration Status", 127510, false, 8, 0,
  { { "Charger Instance", BYTES(1), 1, false, 0, "" }
  , { "Battery Instance", BYTES(1), 1, false, 0, "" }
  , { "Charger Enable/Disable", 2, 1, false, 0, "" }
  , { "Reserved", 6, 1, false, 0, "" }
  , { "Charge Current Limit", BYTES(2), 0.1, false, "A", "" }
  , { "Charging Algorithm", BYTES(1), 1, false, 0, "" }
  , { "Charger Mode", BYTES(1), 1, false, 0, "" }
  , { "Estimated Temperature", BYTES(2), RES_TEMPERATURE, false, 0, "When no sensor present" }
  , { "Equalize One Time Enable/Disable", 4, 1, false, 0, "" }
  , { "Over Charge Enable/Disable", 4, 1, false, 0, "" }
  , { "Equalize Time", BYTES(2), 1, false, 0, "" }
  , { 0 }
  }
}

,
{ "Inverter Configuration Status", 127511, false, 8, 0,
  { { "Inverter Instance", BYTES(1), 1, false, 0, "" }
  , { "AC Instance", BYTES(1), 1, false, 0, "" }
  , { "DC Instance", BYTES(1), 1, false, 0, "" }
  , { "Inverter Enable/Disable", 2, 1, false, 0, "" }
  , { "Inverter Mode", BYTES(1), 1, false, 0, "" }
  , { "Load Sense Enable/Disable", BYTES(1), 1, false, 0, "" }
  , { "Load Sense Power Threshold", BYTES(1), 1, false, 0, "" }
  , { "Load Sense Interval", BYTES(1), 1, false, 0, "" }
  , { 0 }
  }
}

,
{ "AGS Configuration Status", 127512, false, 8, 0,
  { { "AGS Instance", BYTES(1), 1, false, 0, "" }
  , { "Generator Instance", BYTES(1), 1, false, 0, "" }
  , { "AGS Mode", BYTES(1), 1, false, 0, "" }
  , { 0 }
  }
}

,
{ "Battery Configuration Status", 127513, false, 8, 0,
  { { "Battery Instance", BYTES(1), 1, false, 0, "" }
  , { "Battery Type", BYTES(1), 1, false, 0, "" }
  , { "Supports Equalization", 2, 1, false, 0, "" }
  , { "Reserved", 6, 1, false, 0, "" }
  , { "Nominal Voltage", BYTES(2), 0.01, false, "V", "" }
  , { "Chemistry", BYTES(1), 1, false, 0, "" }
  , { "Capacity", BYTES(2), 1, false, 0, "" }
  , { "Temperature Coefficient", BYTES(2), 1, false, 0, "" }
  , { "Peukert Exponent", BYTES(2), 1, false, 0, "" }
  , { "Charge Efficiency Factor", BYTES(2), 1, false, 0, "" }
  , { 0 }
  }
}

,
{ "AGS Status", 127514, false, 8, 0,
  { { "AGS Instance", BYTES(1), 1, false, 0, "" }
  , { "Generator Instance", BYTES(1), 1, false, 0, "" }
  , { "AGS Operating State", BYTES(1), 1, false, 0, "" }
  , { "Generator State", BYTES(1), 1, false, 0, "" }
  , { "Generator On Reason", BYTES(1), 1, false, 0, "" }
  , { "Generator Off Reason", BYTES(1), 1, false, 0, "" }
  , { 0 }
  }
}

  /* http://www.maretron.com/support/manuals/DST100UM_1.2.pdf */
,
{ "Speed", 128259, true, 6, 0,
  { { "SID", BYTES(1), 1, false, 0, "" }
  , { "Speed Water Referenced", BYTES(2), 0.01, false, "m/s", "" }
  , { "Speed Ground Referenced", BYTES(2), 0.01, false, "m/s", "" }
  , { "Speed Water Referenced Type", 4, RES_LOOKUP, false, 0, "" }
  , { 0 }
  }
}

  /* http://www.maretron.com/support/manuals/DST100UM_1.2.pdf */
,
{ "Water Depth", 128267, true, 5, 0,
  { { "SID", BYTES(1), 1, false, 0, "" }
  , { "Depth", BYTES(4), 0.01, false, "m", "Depth below transducer" }
  , { "Offset", BYTES(2), 0.001, true, "m", "Distance between transducer and surface (positive) or keel (negative)" }
  , { 0 }
  }
}

  /* http://www.nmea.org/Assets/nmea-2000-digital-interface-white-paper.pdf */
,
{ "Distance Log", 128275, true, 14, 0,
  { { "Date", BYTES(2), RES_DATE, false, "days", "Timestamp of last reset in Days since January 1, 1970" }
  , { "Time", BYTES(4), RES_TIME, false, "s", "Timestamp of last reset Seconds since midnight" }
  , { "Log", BYTES(4), 1, false, "m", "Total cumulative distance" }
  , { "Trip Log", BYTES(4), 1, false, "m", "Distance since last reset" }
  , { 0 }
  }
}

,
{ "Tracked Target Data", 128520, true, 27, 0,
  { { "SID", BYTES(1), 1, false, 0, "" }
  , { "Target ID #", BYTES(1), 1, false, 0, "Number of route, waypoint, event, mark, etc." }
  , { "Track Status", 2, RES_LOOKUP, false, ",0=Cancelled,1=Acquiring,2=Tracking,3=Lost", "" }
  , { "Reported Target", 1, RES_LOOKUP, false, LOOKUP_YES_NO, "" }
  , { "Target Acquisition", 1, RES_LOOKUP, false, ",0=Manual,1=Automatic", "" }
  , { "Bearing Reference", 2, RES_LOOKUP, false, LOOKUP_DIRECTION_REFERENCE, "" }
  , { "Reserved", 2, 1, false, 0, "" }
  , { "Bearing", BYTES(2), RES_DEGREES, false, "deg", "" }
  , { "Distance", BYTES(4), 0.001, false, "m", "" }
  , { "Course", BYTES(2), RES_DEGREES, false, "deg", "" }
  , { "Speed", BYTES(2), 0.01, false, "m/s", "" }
  , { "CPA", BYTES(4), 0.01, false, "m", "" }
  , { "TCPA", BYTES(4), 0.001, false, "s", "negative = time elapsed since event, positive = time to go" }
  , { "UTC of Fix", BYTES(4), RES_TIME, false, "s", "Seconds since midnight" }
  , { "Name", BYTES(255), RES_ASCII, false, 0, "" }
  , { 0 }
  }
}

,
{ "Position, Rapid Update", 129025, true, 8, 0,
  { { "Latitude", BYTES(4), RES_LATITUDE, true, "deg", "" }
  , { "Longitude", BYTES(4), RES_LONGITUDE, true, "deg", "" }
  , { 0 }
  }
}

  /* http://www.maretron.com/support/manuals/GPS100UM_1.2.pdf */
,
{ "COG & SOG, Rapid Update", 129026, true, 8, 0,
  { { "SID", BYTES(1), 1, false, 0, "" }
  , { "COG Reference", 2, RES_LOOKUP, false, LOOKUP_DIRECTION_REFERENCE, "" }
  , { "Reserved", 6, RES_BINARY, false, 0, "Reserved" }
  , { "COG", BYTES(2), RES_DEGREES, false, "deg", "" }
  , { "SOG", BYTES(2), 0.01, false, "m/s", "" }
  , { "Reserved", BYTES(2), RES_BINARY, false, 0, "Reserved" }
  , { 0 }
  }
}

,
{ "Position Delta, Rapid Update", 129027, false, 8, 0,
  { { "SID", BYTES(1), 1, false, 0, "" }
  , { "Time Delta", BYTES(2), 1, false, 0, "" }
  , { "Latitude Delta", BYTES(2), 1, true, 0, "" }
  , { "Longitude Delta", BYTES(2), 1, true, 0, "" }
  , { 0 }
  }
}

,
{ "Altitude Delta, Rapid Update", 129028, false, 8, 0,
  { { "SID", BYTES(1), 1, false, 0, "" }
  , { "Time Delta", BYTES(2), 1, true, 0, "" }
  , { "GNSS Quality", 2, 1, false, 0, "" }
  , { "Direction", 2, 1, false, 0, "" }
  , { "Reserved", 4, RES_BINARY, false, 0, "Reserved" }
  , { "Course Over Ground", BYTES(4), RES_DEGREES, false, "deg", "" }
  , { "Altitude Delta", BYTES(2), 1, true, 0, "" }
  , { 0 }
  }
}


  /* http://www.maretron.com/support/manuals/GPS100UM_1.2.pdf */
,
{ "GNSS Position Data", 129029, true, 51, 3,
  { { "SID", BYTES(1), 1, false, 0, "" }
  , { "Date", BYTES(2), RES_DATE, false, "days", "Days since January 1, 1970" }
  , { "Time", BYTES(4), RES_TIME, false, "s", "Seconds since midnight" }
  , { "Latitude", BYTES(8), RES_LATITUDE, true, "deg", "" }
  , { "Longitude", BYTES(8), RES_LONGITUDE, true, "deg", "" }
  , { "Altitude", BYTES(8), 1e-6, true, "m", "" }
  , { "GNSS type", 4, RES_LOOKUP, false, LOOKUP_GNS, "" }
  , { "Method", 4, RES_LOOKUP, false, LOOKUP_GNS_METHOD, "" }
  , { "Integrity", 2, RES_LOOKUP, false, LOOKUP_GNS_INTEGRITY, "" }
  , { "Reserved", 6, RES_BINARY, false, 0, "Reserved" }
  , { "Number of SVs", BYTES(1), 1, false, 0, "Number of satellites used in solution" }
  , { "HDOP", BYTES(2), 0.01, true, 0, "Horizontal dilution of precision" }
  , { "PDOP", BYTES(2), 0.01, true, 0, "Probable dilution of precision" }
  , { "Geoidal Separation", BYTES(2), 0.01, true, "m", "Geoidal Separation" }
  , { "Reference Stations", BYTES(1), 1, false, 0, "Number of reference stations" }
  , { "Reference Station Type", 4, RES_LOOKUP, false, LOOKUP_GNS, "" }
  , { "Reference Station ID", 12, 1, false, "" }
  , { "Age of DGNSS Corrections", BYTES(2), 0.01, false, "s", "" }
  , { 0 }
  }
}

,
{ "Time & Date", 129033, true, 8, 0,
  { { "Date", BYTES(2), RES_DATE, false, "days", "Days since January 1, 1970" }
  , { "Time", BYTES(4), RES_TIME, false, "s", "Seconds since midnight" }
  , { "Local Offset", BYTES(2), RES_INTEGER, true, "minutes", "Minutes" }
  , { 0 }
  }
}

,
{ "AIS Class A Position Report", 129038, true, 27, 0,
  { { "Message ID", 6, 1, false, 0, "" }
  , { "Repeat Indicator", 2, RES_LOOKUP, false, LOOKUP_REPEAT_INDICATOR, "" }
  , { "User ID", BYTES(4), RES_INTEGER, false, "MMSI", "" }
  , { "Longitude", BYTES(4), RES_LATITUDE, true, "deg", "" }
  , { "Latitude", BYTES(4), RES_LONGITUDE, true, "deg", "" }
  , { "Position Accuracy", 1, RES_LOOKUP, false, LOOKUP_POSITION_ACCURACY, "" }
  , { "RAIM", 1, RES_LOOKUP, false, LOOKUP_RAIM_FLAG, "" }
  , { "Time Stamp", 6, RES_LOOKUP, false, LOOKUP_TIME_STAMP, "0-59 = UTC second when the report was generated" }
  , { "COG", BYTES(2), RES_DEGREES, false, "deg", "" }
  , { "SOG", BYTES(2), 0.01, false, "m/s", "" }
  , { "Communication State", 19, RES_BINARY, false, 0, "Information used by the TDMA slot allocation algorithm and synchronization information" }
  , { "AIS Transceiver information", 5, RES_LOOKUP, false, LOOKUP_AIS_TRANSCEIVER, "" }
  , { "Heading", BYTES(2), RES_DEGREES, false, "deg", "True heading" }
  , { "Rate of Turn", BYTES(2), RES_ROTATION, true, "deg/s", "" }
  , { "Nav Status", BYTES(1), RES_LOOKUP, false, LOOKUP_NAV_STATUS, "" }
  , { "Reserved for Regional Applications", BYTES(1), 1, false, 0, "" }
  , { "Spare", BYTES(1), 1, false, 0, "" }
  , { 0 }
  }
}

,
{ "AIS Class B Position Report", 129039, true, 0x1a, 0,
  { { "Message ID", 6, 1, false, 0, "" }
  , { "Repeat Indicator", 2, RES_LOOKUP, false, LOOKUP_REPEAT_INDICATOR, "" }
  , { "User ID", BYTES(4), RES_INTEGER, false, "MMSI", "" }
  , { "Longitude", BYTES(4), RES_LATITUDE, true, "deg", "" }
  , { "Latitude", BYTES(4), RES_LONGITUDE, true, "deg", "" }
  , { "Position Accuracy", 1, RES_LOOKUP, false, LOOKUP_POSITION_ACCURACY, "" }
  , { "RAIM", 1, RES_LOOKUP, false, LOOKUP_RAIM_FLAG, "" }
  , { "Time Stamp", 6, RES_LOOKUP, false, LOOKUP_TIME_STAMP, "0-59 = UTC second when the report was generated" }
  , { "COG", BYTES(2), RES_DEGREES, false, "deg", "" }
  , { "SOG", BYTES(2), 0.01, false, "m/s", "" }
  , { "Communication State", 19, RES_BINARY, false, 0, "Information used by the TDMA slot allocation algorithm and synchronization information" }
  , { "AIS Transceiver information", 5, RES_LOOKUP, false, LOOKUP_AIS_TRANSCEIVER, "" }
  , { "Heading", BYTES(2), RES_DEGREES, false, "deg", "True heading" }
  , { "Regional Application", BYTES(1), 1, false, 0, "" }
  , { "Regional Application", 2, 1, false, 0, "" }
  , { "Unit type", 1, RES_LOOKUP, false, ",0=SOTDMA,1=CS", "" }
  , { "Integrated Display", 1, RES_LOOKUP, false, LOOKUP_YES_NO, "Whether the unit can show messages 12 and 14" }
  , { "DSC", 1, RES_LOOKUP, false, LOOKUP_YES_NO, "" }
  , { "Band", 1, RES_LOOKUP, false, ",0=top 525 kHz of marine band,1=entire marine band", "" }
  , { "Can handle Msg 22", 1, RES_LOOKUP, false, LOOKUP_YES_NO, "Whether device supports message 22" }
  , { "AIS mode", 1, RES_LOOKUP, false, ",0=Autonomous,1=Assigned", "" }
  , { "AIS communication state", 1, RES_LOOKUP, false, ",0=SOTDMA,1=ITDMA", "" }
  , { 0 }
  }
}

,
{ "AIS Class B Extended Position Report", 129040, true, 33, 0,
  { { "Message ID", 6, 1, false, 0, "" }
  , { "Repeat Indicator", 2, RES_LOOKUP, false, LOOKUP_REPEAT_INDICATOR, "" }
  , { "User ID", BYTES(4), RES_INTEGER, false, "MMSI", "" }
  , { "Longitude", BYTES(4), RES_LATITUDE, true, "deg", "" }
  , { "Latitude", BYTES(4), RES_LONGITUDE, true, "deg", "" }
  , { "Position Accuracy", 1, RES_LOOKUP, false, LOOKUP_POSITION_ACCURACY, "" }
  , { "AIS RAIM flag", 1, RES_LOOKUP, false, LOOKUP_RAIM_FLAG, "" }
  , { "Time Stamp", 6, RES_LOOKUP, false, LOOKUP_TIME_STAMP, "0-59 = UTC second when the report was generated" }
  , { "COG", BYTES(2), RES_DEGREES, false, "deg", "" }
  , { "SOG", BYTES(2), 0.01, false, "m/s", "" }
  , { "Regional Application", BYTES(1), 1, false, 0, "" }
  , { "Regional Application", 4, 1, false, 0, "" }
  , { "Reserved", 4, RES_BINARY, false, 0, "reserved" }
  , { "Type of ship", BYTES(1), RES_LOOKUP, false, LOOKUP_SHIP_TYPE, "" }
  , { "True Heading", BYTES(2), RES_DEGREES, false, "deg", "" }
  , { "Reserved", 4, RES_BINARY, false, 0, "" }
  , { "GNSS type", 4, RES_LOOKUP, false, LOOKUP_GNS_AIS, "" }
  , { "Length", BYTES(2), 0.1, false, "m", "" }
  , { "Beam", BYTES(2), 0.1, false, "m", "" }
  , { "Position reference from Starboard", BYTES(2), 0.1, false, "m", "" }
  , { "Position reference from Bow", BYTES(2), 0.1, false, "m", "" }
  , { "Name", BYTES(20), RES_ASCII, false, 0, "0=unavailable" }
  , { "DTE", 1, RES_LOOKUP, false, ",0=Available,1=Not available", "" }
  , { "AIS mode", 1, 1, false, ",0=Autonomous,1=Assigned", "" }
  , { "Reserved", 4, RES_BINARY, false, 0, "" }
  , { "AIS Transceiver information", 5, RES_LOOKUP, false, LOOKUP_AIS_TRANSCEIVER, "" }
  , { 0 }
  }
}

,
{ "Datum", 129044, true, 24, 0,
  { { "Local Datum", BYTES(4), RES_ASCII, false, 0, "defined in IHO Publication S-60, Appendices B and C."
              " First three chars are datum ID as per IHO tables."
              " Fourth char is local datum subdivision code." }
  , { "Delta Latitude", BYTES(4), RES_LATITUDE, true, "deg", "" }
  , { "Delta Longitude", BYTES(4), RES_LONGITUDE, true, "deg", "" }
  , { "Delta Altitude", BYTES(4), 1e-6, true, "m", "" }
  , { "Reference Datum", BYTES(4), RES_ASCII, false, 0, "defined in IHO Publication S-60, Appendices B and C."
              " First three chars are datum ID as per IHO tables."
              " Fourth char is local datum subdivision code." }
  , { 0 }
  }
}

,
{ "User Datum", 129045, true, 37, 0,
  { { "Delta X", BYTES(4), 0.01, true, "m", "Delta shift in X axis from WGS 84" }
  , { "Delta Y", BYTES(4), 0.01, true, "m", "Delta shift in Y axis from WGS 84" }
  , { "Delta Z", BYTES(4), 0.01, true, "m", "Delta shift in Z axis from WGS 84" }
  , { "Rotation in X", BYTES(4), RES_FLOAT, true, 0, "Rotational shift in X axis from WGS 84. Rotations presented use the geodetic sign convention.  When looking along the positive axis towards the origin, counter-clockwise rotations are positive." }
  , { "Rotation in Y", BYTES(4), RES_FLOAT, true, 0, "Rotational shift in Y axis from WGS 84. Rotations presented use the geodetic sign convention.  When looking along the positive axis towards the origin, counter-clockwise rotations are positive." }
  , { "Rotation in Z", BYTES(4), RES_FLOAT, true, 0, "Rotational shift in Z axis from WGS 84. Rotations presented use the geodetic sign convention.  When looking along the positive axis towards the origin, counter-clockwise rotations are positive." }
  , { "Scale", BYTES(4), RES_FLOAT, true, "ppm", "Scale factor expressed in parts-per-million" }
  , { "Ellipsoid Semi-major Axis", BYTES(4), 0.01, true, "m", "Semi-major axis (a) of the User Datum ellipsoid" }
  , { "Ellipsoid Flattening Inverse", BYTES(4), RES_FLOAT, true, 0, "Flattening (1/f) of the User Datum ellipsoid" }
  , { "Datum Name", BYTES(4), RES_ASCII, false, 0, "4 character code from IHO Publication S-60,Appendices B and C."
              " First three chars are datum ID as per IHO tables."
              " Fourth char is local datum subdivision code." }
  , { 0 }
  }
}

,
{ "Cross Track Error", 129283, false, 6, 0,
  { { "SID", BYTES(1), 1, false, 0, "" }
  , { "XTE mode", 4, RES_LOOKUP, false, LOOKUP_RESIDUAL_MODE, "" }
  , { "Reserved", 2, RES_BINARY, false, 0, "reserved" }
  , { "Navigation Terminated", 2, RES_LOOKUP, false, LOOKUP_YES_NO, "" }
  , { "XTE", BYTES(4), 0.01, true, "m", "" }
  , { 0 }
  }
}

,
{ "Navigation Data", 129284, true, 0x22, 0,
  { { "SID", BYTES(1), 1, false, 0, "" }
  , { "Distance to Waypoint", BYTES(4), 0.01, false, "m", "" }
  , { "Course/Bearing reference", 2, RES_LOOKUP, false, LOOKUP_DIRECTION_REFERENCE, "" }
  , { "Perpendicular Crossed", 2, RES_LOOKUP, false, LOOKUP_YES_NO, "" }
  , { "Arrival Circle Entered", 2, RES_LOOKUP, false, LOOKUP_YES_NO, "" }
  , { "Calculation Type", 2, RES_LOOKUP, false, ",0=Great Circle,1=Rhumb Line", "" }
  , { "ETA Time", BYTES(4), RES_TIME, false, "s", "Seconds since midnight" }
  , { "ETA Date", BYTES(2), RES_DATE, false, "days", "Days since January 1, 1970" }
  , { "Bearing, Origin to Destination Waypoint", BYTES(2), RES_DEGREES, false, "deg", "" }
  , { "Bearing, Position to Destination Waypoint", BYTES(2), RES_DEGREES, false, "deg", "" }
  , { "Origin Waypoint Number", BYTES(4), 1, false, 0, "" }
  , { "Destination Waypoint Number", BYTES(4), 1, false, 0, "" }
  , { "Destination Latitude", BYTES(4), RES_LATITUDE, true, "deg", "" }
  , { "Destination Longitude", BYTES(4), RES_LONGITUDE, true, "deg", "" }
  , { "Waypoint Closing Velocity", BYTES(2), 0.01, true, "m/s", "" }
  , { 0 }
  }
}

,
{ "Navigation - Route/WP Information", 129285, true, 8, 4,
  { { "Start RPS#", BYTES(2), 1, false, 0, "" }
  , { "nItems", BYTES(2), 1, false, 0, "" }
  , { "Database ID", BYTES(2), 1, false, 0, "" }
  , { "Route ID", BYTES(2), 1, false, 0, "" }
  , { "Navigation direction in route", 2, 1, false, 0, "" }
  , { "Supplementary Route/WP data available", 2, 1, false, 0, "" }
  , { "Reserved", 4, RES_BINARY, false, 0, "Reserved" }
  , { "Route Name", BYTES(255), RES_STRING, false, 0, "" }
  , { "Reserved", BYTES(1), RES_BINARY, false, 0, "Reserved" }
  , { "WP ID", BYTES(2), 1, false, 0, "" }
  , { "WP Name", BYTES(255), RES_STRING, false, 0, "" }
  , { "WP Latitude", BYTES(4), RES_LATITUDE, true, "deg", "" }
  , { "WP Longitude", BYTES(4), RES_LONGITUDE, true, "deg", "" }
  , { 0 }
  }
}

,
{ "Set & Drift, Rapid Update", 129291, true, 8, 0,
  { { "SID", BYTES(1), 1, false, 0, "" }
  , { "Set Reference", 2, RES_LOOKUP, false, LOOKUP_DIRECTION_REFERENCE, "" }
  , { "Reserved", 6, RES_BINARY, false, 0, "Reserved" }
  , { "Set", BYTES(2), RES_DEGREES, false, "deg", "" }
  , { "Drift", BYTES(2), 0.01, false, "m/s", "" }
  , { 0 }
  }
}

,
{ "Navigation - Route / Time to+from Mark", 129301, true, 10, 0,
  { { "SID", BYTES(1), 1, false, 0, "" }
  , { "Time to mark", BYTES(4), 0.001, true, "s", "negative = elapsed since event, positive = time to go" }
  , { "Mark Type", 4, RES_LOOKUP, false, ",0=Collision,1=Turning point,2=Reference,3=Wheelover,4=Waypoint", "" }
  , { "Reserved", 4, RES_BINARY, false, 0, "Reserved" }
  , { "Mark ID", BYTES(4), 1, false, 0, "" }
  , { 0 }
  }
}

,
{ "Bearing and Distance between two Marks", 129302, false, 8, 0,
  { { "SID", BYTES(1), 1, false, 0, "" }
  , { "Bearing Reference", 4, RES_LOOKUP, false, 0, "" }
  , { "Calculation Type", 2, RES_LOOKUP, false, 0, "" }
  , { "Reserved", 2, RES_BINARY, false, 0, "Reserved" }
  , { "Bearing, Origin to Destination", BYTES(2), RES_DEGREES, false, "deg", "" }
  , { "Distance", BYTES(4), 0.01, false, "m", "" }
  , { "Origin Mark Type", 4, RES_LOOKUP, false, 0, "" }
  , { "Destination Mark Type", 4, RES_LOOKUP, false, 0, "" }
  , { "Origin Mark ID", BYTES(4), 1, false, 0, "" }
  , { "Destination Mark ID", BYTES(4), 1, false, 0, "" }
  , { 0 }
  }
}

  /* http://www.maretron.com/support/manuals/GPS100UM_1.2.pdf */
  /* Haven't seen this yet (no way to send PGN 059904 yet) so lengths unknown */
,
{ "GNSS Control Status", 129538, false, 0, 0,
  { { "SV Elevation Mask", BYTES(2), 1, false, 0, "Will not use SV below this elevation" }
  , { "PDOP Mask", BYTES(2), 0.01, false, 0, "Will not report position above this PDOP" }
  , { "PDOP Switch", BYTES(2), 0.01, false, 0, "Will report 2D position above this PDOP" }
  , { "SNR Mask", BYTES(2), 0.01, false, 0, "Will not use SV below this SNR" }
  , { "GNSS Mode (desired)", 3, RES_LOOKUP, false, ",0=1D,1=2D,2=3D,3=Auto,4=Reserved,5=Reserved,6=Error", "" }
  , { "DGNSS Mode (desired)", 3, RES_LOOKUP, false, ",0=no SBAS,1=SBAS,3=SBAS", "" }
  , { "Position/Velocity Filter", 2, 1, false, 0, "" }
  , { "Max Correction Age", BYTES(2), 1, false, 0, "" }
  , { "Antenna Altitude for 2D Mode", BYTES(2), 0.01, false, "m", "" }
  , { "Use Antenna Altitude for 2D Mode", 2, RES_LOOKUP, false, ",0=use last 3D height,1=Use antenna altitude", "" }
  , { 0 }
  }
}

  /* http://www.maretron.com/support/manuals/GPS100UM_1.2.pdf */
,
{ "GNSS DOPs", 129539, true, 8, 0,
  { { "SID", BYTES(1), 1, false, 0, "" }
  , { "Desired Mode", 3, RES_LOOKUP, false, ",0=1D,1=2D,2=3D,3=Auto,4=Reserved,5=Reserved,6=Error", "" }
  , { "Actual Mode", 3, RES_LOOKUP, false, ",0=1D,1=2D,2=3D,3=Auto,4=Reserved,5=Reserved,6=Error", "" }
  , { "Reserved", 2, RES_BINARY, false, 0, "Reserved" }
  , { "HDOP", BYTES(2), 0.01, true, 0, "Horizontal dilution of precision" }
  , { "VDOP", BYTES(2), 0.01, true, 0, "Vertical dilution of precision" }
  , { "TDOP", BYTES(2), 0.01, false, 0, "Time dilution of precision" }
  , { 0 }
  }
}

,
{ "GNSS Sats in View", 129540, true, 0xff, 7,
  { { "SID", BYTES(1), 1, false, 0, "" }
  , { "Mode", 2, RES_LOOKUP, false, ",3=Range residuals used to calculate position", "" }
  , { "Reserved", 6, RES_BINARY, false, 0, "Reserved" }
  , { "Sats in View", BYTES(1), 1, false, 0, "" }
  , { "PRN", BYTES(1), 1, false, 0, "" }
  , { "Elevation", BYTES(2), RES_DEGREES, false, "deg", "" }
  , { "Azimuth", BYTES(2), RES_DEGREES, false, "deg", "" }
  , { "SNR", BYTES(2), 0.01, false, "dB", "" }
  , { "Range residuals", BYTES(4), 1, true, 0, "" }
  , { "Status", 4, RES_LOOKUP, false, ",0=Not tracked,1=Tracked,2=Used,3=Not tracked+Diff,4=Tracked+Diff,5=Used+Diff", "" }
  , { "Reserved", 4, RES_BINARY, false, 0, "Reserved" }
  , { 0 }
  }
}

,
{ "GPS Almanac Data", 129541, false, 8, 0,
  { { "PRN", BYTES(1), 1, false, 0, "" }
  , { "GPS Week number", BYTES(1), 1, false, 0, "" }
  , { "SV Health Bits", BYTES(1), 1, false, 0, "" }
  , { "Eccentricity", BYTES(1), 1, false, 0, "" }
  , { "Almanac Reference Time", BYTES(1), 1, false, 0, "" }
  , { "Inclination Angle", BYTES(1), 1, false, 0, "" }
  , { "Right of Right Ascension", BYTES(1), 1, false, 0, "" }
  , { "Root of Semi-major Axis", BYTES(1), 1, false, 0, "" }
  , { "Argument of Perigee", BYTES(1), 1, false, 0, "" }
  , { "Longitude of Ascension Node", BYTES(1), 1, false, 0, "" }
  , { "Mean Anomaly", BYTES(1), 1, false, 0, "" }
  , { "Clock Parameter 1", BYTES(1), 1, false, 0, "" }
  , { "Clock Parameter 2", BYTES(1), 1, false, 0, "" }
  , { 0 }
  }
}

,
{ "GNSS Pseudorange Noise Statistics", 129542, false, 8, 0,
  { { "SID", BYTES(1), 1, false, 0, "" }
  , { "RMS of Position Uncertainty", BYTES(2), 1, false, 0, "" }
  , { "STD of Major axis", BYTES(1), 1, false, 0, "" }
  , { "STD of Minor axis", BYTES(1), 1, false, 0, "" }
  , { "Orientation of Major axis", BYTES(1), 1, false, 0, "" }
  , { "STD of Lat Error", BYTES(1), 1, false, 0, "" }
  , { "STD of Lon Error", BYTES(1), 1, false, 0, "" }
  , { "STD of Alt Error", BYTES(1), 1, false, 0, "" }
  , { 0 }
  }
}

,
{ "GNSS RAIM Output", 129545, false, 8, 0,
  { { "SID", BYTES(1), 1, false, 0, "" }
  , { "Integrity flag", 4, 1, false, 0, "" }
  , { "Reserved", 4, RES_BINARY, false, 0, "Reserved" }
  , { "Latitude expected error", BYTES(1), 1, false, 0, "" }
  , { "Longitude expected error", BYTES(1), 1, false, 0, "" }
  , { "Altitude expected error", BYTES(1), 1, false, 0, "" }
  , { "SV ID of most likely failed sat", BYTES(1), 1, false, 0, "" }
  , { "Probability of missed detection", BYTES(1), 1, false, 0, "" }
  , { "Estimate of pseudorange bias", BYTES(1), 1, false, 0, "" }
  , { "Std Deviation of bias", BYTES(1), 1, false, 0, "" }
  , { 0 }
  }
}

,
{ "GNSS RAIM Settings", 129546, false, 8, 0,
  { { "Radial Position Error Maximum Threshold", BYTES(1), 1, false, 0, "" }
  , { "Probability of False Alarm", BYTES(1), 1, false, 0, "" }
  , { "Probability of Missed Detection", BYTES(1), 1, false, 0, "" }
  , { "Pseudorange Residual Filtering Time Constant", BYTES(1), 1, false, 0, "" }
  , { 0 }
  }
}

,
{ "GNSS Pseudorange Error Statistics", 129547, false, 8, 0,
  { { "SID", BYTES(1), 1, false, 0, "" }
  , { "RMS Std Dev of Range Inputs", BYTES(2), 1, false, 0, "" }
  , { "Std Dev of Major error ellipse", BYTES(1), 1, false, 0, "" }
  , { "Std Dev of Minor error ellipse", BYTES(1), 1, false, 0, "" }
  , { "Orientation of error ellipse", BYTES(1), 1, false, 0, "" }
  , { "Std Dev Lat Error", BYTES(1), 1, false, 0, "" }
  , { "Std Dev Lon Error", BYTES(1), 1, false, 0, "" }
  , { "Std Dev Alt Error", BYTES(1), 1, false, 0, "" }
  , { 0 }
  }
}

,
{ "DGNSS Corrections", 129549, false, 8, 0,
  { { "SID", BYTES(1), 1, false, 0, "" }
  , { "Reference Station ID", BYTES(2), 1, false, 0, "" }
  , { "Reference Station Type", BYTES(2), 1, false, 0, "" }
  , { "Time of corrections", BYTES(1), 1, false, 0, "" }
  , { "Station Health", BYTES(1), 1, false, 0, "" }
  , { "Reserved Bits", BYTES(1), 1, false, 0, "" }
  , { "Satellite ID", BYTES(1), 1, false, 0, "" }
  , { "PRC", BYTES(1), 1, false, 0, "" }
  , { "RRC", BYTES(1), 1, false, 0, "" }
  , { "UDRE", BYTES(1), 1, false, 0, "" }
  , { "IOD", BYTES(1), 1, false, 0, "" }
  , { 0 }
  }
}

,
{ "GNSS Differential Correction Receiver Interface", 129550, false, 8, 0,
  { { "Channel", BYTES(1), 1, false, 0, "" }
  , { "Frequency", BYTES(1), 1, false, 0, "" }
  , { "Serial Interface Bit Rate", BYTES(1), 1, false, 0, "" }
  , { "Serial Interface Detection Mode", BYTES(1), 1, false, 0, "" }
  , { "Differential Source", BYTES(1), 1, false, 0, "" }
  , { "Differential Operation Mode", BYTES(1), 1, false, 0, "" }
  , { 0 }
  }
}

,
{ "GNSS Differential Correction Receiver Signal", 129551, false, 8, 0,
  { { "SID", BYTES(1), 1, false, 0, "" }
  , { "Channel", BYTES(1), 1, false, 0, "" }
  , { "Signal Strength", BYTES(1), 1, false, 0, "" }
  , { "Signal SNR", BYTES(1), 1, false, 0, "" }
  , { "Frequency", BYTES(1), 1, false, 0, "" }
  , { "Station Type", BYTES(1), 1, false, 0, "" }
  , { "Station ID", BYTES(1), 1, false, 0, "" }
  , { "Differential Signal Bit Rate", BYTES(1), 1, false, 0, "" }
  , { "Differential Signal Detection Mode", BYTES(1), 1, false, 0, "" }
  , { "Used as Correction Source", BYTES(1), 1, false, 0, "" }
  , { "Reserved", BYTES(1), 1, false, 0, "Reserved" }
  , { "Differential Source", BYTES(1), 1, false, 0, "" }
  , { "Time since Last Sat Differential Sync", BYTES(1), 1, false, 0, "" }
  , { "Satellite Service ID No.", BYTES(1), 1, false, 0, "" }
  , { 0 }
  }
}

,
{ "GLONASS Almanac Data", 129556, false, 8, 0,
  { { "PRN", BYTES(1), 1, false, 0, "" }
  , { "NA", BYTES(1), 1, false, 0, "" }
  , { "CnA", BYTES(1), 1, false, 0, "" }
  , { "HnA", BYTES(1), 1, false, 0, "" }
  , { "(epsilon)nA", BYTES(1), 1, false, 0, "" }
  , { "(deltaTnA)DOT", BYTES(1), 1, false, 0, "" }
  , { "(omega)nA", BYTES(1), 1, false, 0, "" }
  , { "(delta)TnA", BYTES(1), 1, false, 0, "" }
  , { "tnA", BYTES(1), 1, false, 0, "" }
  , { "(lambda)nA", BYTES(1), 1, false, 0, "" }
  , { "(delta)inA", BYTES(1), 1, false, 0, "" }
  , { "tcA", BYTES(1), 1, false, 0, "" }
  , { "tnA", BYTES(1), 1, false, 0, "" }
  , { 0 }
  }
}

,
{ "AIS DGNSS Broadcast Binary Message", 129792, false, 8, 0,
  { { "Message ID", BYTES(1), 1, false, 0, "" }
  , { "Repeat Indicator", BYTES(1), 1, false, 0, "" }
  , { "Source ID", BYTES(1), 1, false, 0, "" }
  , { "NMEA 2000 Reserved", BYTES(1), 1, false, 0, "" }
  , { "AIS Tranceiver Information", BYTES(1), 1, false, 0, "" }
  , { "Spare", BYTES(1), 1, false, 0, "" }
  , { "Longitude", BYTES(1), 1, false, 0, "" }
  , { "Latitude", BYTES(1), 1, false, 0, "" }
  , { "NMEA 2000 Reserved", BYTES(1), 1, false, 0, "" }
  , { "Spare", BYTES(1), 1, false, 0, "" }
  , { "Number of Bits in Binary Data Field", BYTES(1), 1, false, 0, "" }
  , { "Binary Data", BYTES(8), RES_BINARY, false, 0, "" }
  , { 0 }
  }
}

,
{ "AIS UTC and Date Report", 129793, false, 8, 0,
  { { "Message ID", 6, 1, false, 0, "" }
  , { "Repeat Indicator", 2, RES_LOOKUP, false, LOOKUP_REPEAT_INDICATOR, "" }
  , { "User ID", BYTES(4), RES_INTEGER, false, "MMSI", "" }
  , { "Longitude", BYTES(4), RES_LONGITUDE, true, "deg", "" }
  , { "Latitude", BYTES(4), RES_LATITUDE, true, "deg", "" }
  , { "Position Accuracy", 1, RES_LOOKUP, false, ",0=Low,1=High", "" }
  , { "RAIM", 1, RES_LOOKUP, false, ",0=not in use,1=in use", "" }
  , { "Reserved", 6, RES_BINARY, false, 0, "NMEA reserved to align next data on byte boundary" }
  , { "Position Time", BYTES(4), RES_TIME, false, "s", "Seconds since midnight" }
  , { "Communication State", 19, RES_BINARY, false, 0, "Information used by the TDMA slot allocation algorithm and synchronization information" }
  , { "AIS Transceiver information", 5, RES_LOOKUP, false, LOOKUP_AIS_TRANSCEIVER, "" }
  , { "Position Date", BYTES(2), RES_DATE, false, "days", "Days since January 1, 1970" }
  , { "Reserved", 4, RES_BINARY, false, 0, "NMEA reserved to align next data on byte boundary" }
  , { "GNSS type", 4, RES_LOOKUP, false, LOOKUP_GNS_AIS, "" }
  , { "Spare", BYTES(1), RES_BINARY, false, 0, "" }
  , { 0 }
  }
}

  /* http://www.navcen.uscg.gov/enav/ais/AIS_messages.htm */
,
{ "AIS Class A Static and Voyage Related Data", 129794, true, 0x18, 0,
  { { "Message ID", 6, 1, false, 0, "" }
  , { "Repeat indicator", 2, RES_LOOKUP, false, LOOKUP_REPEAT_INDICATOR, "" }
  , { "User ID", BYTES(4), RES_INTEGER, false, "MMSI", "" }
  , { "IMO number", BYTES(4), RES_INTEGER, false, 0, "0=unavailable" }
  , { "Callsign", BYTES(7), RES_ASCII, false, 0, "0=unavailable" }
  , { "Name", BYTES(20), RES_ASCII, false, 0, "0=unavailable" }
  , { "Type of ship", BYTES(1), RES_LOOKUP, false, LOOKUP_SHIP_TYPE, "" }
  , { "Length", BYTES(2), 0.1, false, "m", "" }
  , { "Beam", BYTES(2), 0.1, false, "m", "" }
  , { "Position reference from Starboard", BYTES(2), 0.1, false, "m", "" }
  , { "Position reference from Bow", BYTES(2), 0.1, false, "m", "" }
  , { "ETA Date", BYTES(2), RES_DATE, false, "days", "Days since January 1, 1970" }
  , { "ETA Time", BYTES(4), RES_TIME, false, "s", "Seconds since midnight" }
  , { "Draft", BYTES(2), 0.01, false, "m", "" }
  , { "Destination", BYTES(20), RES_ASCII, false, 0, "0=unavailable" }
  , { "AIS version indicator", 2, RES_LOOKUP, false, ",0=ITU-R M.1371-1,1=ITU-R M.1371-3", "" }
  , { "GNSS type", 4, RES_LOOKUP, false, LOOKUP_GNS_AIS, "" }
  , { "DTE", 1, RES_LOOKUP, false, ",0=available,1=not available", "" }
  , { "Reserved", 1, RES_BINARY, false, 0, "reserved" }
  , { "AIS Transceiver information", 5, RES_LOOKUP, false, LOOKUP_AIS_TRANSCEIVER, "" }
  , { 0 }
  }
}

,
{ "AIS Addressed Binary Message", 129795, true, 13, 0,
  { { "Message ID", 6, 1, false, 0, "" }
  , { "Repeat Indicator", 2, RES_LOOKUP, false, LOOKUP_REPEAT_INDICATOR, "" }
  , { "Source ID", BYTES(4), RES_INTEGER, false, "MMSI", "" }
  , { "Reserved", 1, RES_BINARY, false, 0, "reserved" }
  , { "AIS Transceiver information", 5, RES_LOOKUP, false, LOOKUP_AIS_TRANSCEIVER, "" }
  , { "Sequence Number", 2, 1, false, 0, "" }
  , { "Destination ID", BYTES(4), RES_INTEGER, false, "MMSI", "" }
  , { "Reserved", 6, RES_BINARY, false, 0, "reserved" }
  , { "Retransmit flag", 1, 1, false, 0, "" }
  , { "Reserved", 1, RES_BINARY, false, 0, "reserved" }
  , { "Number of Bits in Binary Data Field", BYTES(2), RES_INTEGER, false, 0, "" }
  , { "Binary Data", BYTES(8), RES_BINARY, false, 0, "" }
  , { 0 }
  }
}

,
{ "AIS Acknowledge", 129796, true, 12, 0,
  { { "Message ID", 6, 1, false, 0, "" }
  , { "Repeat Indicator", 2, RES_LOOKUP, false, LOOKUP_REPEAT_INDICATOR, "" }
  , { "Source ID", BYTES(4), 1, false, "MMSI", "" }
  , { "Reserved", 1, RES_BINARY, false, 0, "reserved" }
  , { "AIS Transceiver information", 5, RES_LOOKUP, false, LOOKUP_AIS_TRANSCEIVER, "" }
  , { "Reserved", 2, RES_BINARY, false, 0, "reserved" }
  , { "Destination ID #1", BYTES(4), 1, false, 0, "" }
  , { "Sequence Number for ID 1", 2, RES_BINARY, false, 0, "reserved" }
  , { "Reserved", 6, RES_BINARY, false, 0, "reserved" }
  , { "Sequence Number for ID n", 2, RES_BINARY, false, 0, "reserved" }
  , { 0 }
  }
}

,
{ "AIS Binary Broadcast Message", 129797, true, 8, 0,
  { { "Message ID", 6, 1, false, 0, "" }
  , { "Repeat Indicator", 2, RES_LOOKUP, false, LOOKUP_REPEAT_INDICATOR, "" }
  , { "Source ID", BYTES(4), 1, false, 0, "" }
  , { "Reserved", 1, RES_BINARY, false, 0, "reserved" }
  , { "AIS Transceiver information", 5, RES_LOOKUP, false, LOOKUP_AIS_TRANSCEIVER, "" }
  , { "Reserved", 2, RES_BINARY, false, 0, "reserved" }
  , { "Number of Bits in Binary Data Field", BYTES(2), 1, false, 0, "" }
  , { "Binary Data", BYTES(255), RES_BINARY, false, 0, "" }
  , { 0 }
  }
}

,
{ "AIS SAR Aircraft Position Report", 129798, false, 8, 0,
  { { "Message ID", 6, 1, false, 0, "" }
  , { "Repeat indicator", 2, RES_LOOKUP, false, LOOKUP_REPEAT_INDICATOR, "" }
  , { "User ID", BYTES(4), RES_INTEGER, false, "MMSI", "" }
  , { "Longitude", BYTES(4), RES_LONGITUDE, true, "deg", "" }
  , { "Latitude", BYTES(4), RES_LATITUDE, true, "deg", "" }
  , { "Position Accuracy", 1, RES_LOOKUP, false, LOOKUP_POSITION_ACCURACY, "" }
  , { "RAIM", 1, RES_LOOKUP, false, LOOKUP_RAIM_FLAG, "" }
  , { "Time Stamp", 6, RES_LOOKUP, false, LOOKUP_TIME_STAMP, "0-59 = UTC second when the report was generated" }
  , { "COG", BYTES(2), RES_DEGREES, false, "deg", "" }
  , { "SOG", BYTES(2), 0.1, false, "m/s", "" }
  , { "Communication State", 19, RES_BINARY, false, 0, "Information used by the TDMA slot allocation algorithm and synchronization information" }
  , { "AIS Transceiver information", 5, RES_LOOKUP, false, LOOKUP_AIS_TRANSCEIVER, "" }
  , { "Altitude", BYTES(8), 1e-6, true, "m", "" }
  , { "Reserved for Regional Applications", BYTES(1), 1, false, 0, "" }
  , { "DTE", 1, RES_LOOKUP, false, ",0=Available,1=Not available", "" }
  , { "Reserved", 7, RES_BINARY, false, 0, "reserved" }
  , { 0 }
  }
}

,
{ "Radio Frequency/Mode/Power", 129799, false, 9, 0,
  { { "Rx Frequency", BYTES(4), 10, false, "Hz", "" }
  , { "Tx Frequency", BYTES(4), 10, false, "Hz", "" }
  , { "Radio Channel", BYTES(1), 1, false, 0, "" }
  , { "Tx Power", BYTES(1), 1, false, 0, "" }
  , { "Mode", BYTES(1), 1, false, 0, "" }
  , { "Channel Bandwidth", BYTES(1), 1, false, 0, "" }
  , { 0 }
  }
}

,
{ "AIS UTC/Date Inquiry", 129800, false, 8, 0,
  { { "Message ID", 6, 1, false, 0, "" }
  , { "Repeat Indicator", 2, RES_LOOKUP, false, LOOKUP_REPEAT_INDICATOR, "" }
  , { "Source ID", 30, 1, false, 0, "" }
  , { "Reserved", 2, RES_BINARY, false, 0, "reserved" }
  , { "AIS Transceiver information", 5, RES_LOOKUP, false, LOOKUP_AIS_TRANSCEIVER, "" }
  , { "Reserved", 3, RES_BINARY, false, 0, "reserved" }
  , { "Destination ID", 30, 1, false, 0, "" }
  , { "Reserved", 2, RES_BINARY, false, 0, "reserved" }
  , { 0 }
  }
}

,
{ "AIS Addressed Safety Related Message", 129801, true, 12, 0,
  { { "Message ID", 6, 1, false, 0, "" }
  , { "Repeat Indicator", 2, RES_LOOKUP, false, LOOKUP_REPEAT_INDICATOR, "" }
  , { "Source ID", BYTES(4), 1, false, "MMSI", "" }
  , { "Reserved", 1, RES_BINARY, false, 0, "reserved" }
  , { "AIS Transceiver information", 5, RES_LOOKUP, false, LOOKUP_AIS_TRANSCEIVER, "" }
  , { "Sequence Number", 2, 1, false, 0, "" }
  , { "Destination ID", BYTES(4), 1, false, "MMSI", "" }
  , { "Reserved", 6, RES_BINARY, false, 0, "reserved" }
  , { "Retransmit flag", 1, 1, false, 0, "" }
  , { "Reserved", 1, RES_BINARY, false, 0, "reserved" }
  , { "Safety Related Text", BYTES(255), RES_ASCII, false, 0, "" }
  , { 0 }
  }
}

,
{ "AIS Safety Related Broadcast Message", 129802, false, 8, 0,
  { { "Message ID", 6, 1, false, 0, "" }
  , { "Repeat Indicator", 2, RES_LOOKUP, false, LOOKUP_REPEAT_INDICATOR, "" }
  , { "Source ID", 30, RES_INTEGER, false, 0, "" }
  , { "Reserved", 2, RES_BINARY, false, 0, "reserved" }
  , { "AIS Transceiver information", 5, RES_LOOKUP, false, LOOKUP_AIS_TRANSCEIVER, "" }
  , { "Reserved", 3, RES_BINARY, false, 0, "reserved" }
  , { "Safety Related Text", BYTES(36), RES_ASCII, false, 0, "" }
  , { 0 }
  }
}

,
{ "AIS Interrogation", 129803, false, 8, 8,
  { { "Message ID", 6, 1, false, 0, "" }
  , { "Repeat Indicator", 2, RES_LOOKUP, false, LOOKUP_REPEAT_INDICATOR, "" }
  , { "Source ID", 30, RES_INTEGER, false, 0, "" }
  , { "Reserved", 2, RES_BINARY, false, 0, "reserved" }
  , { "AIS Transceiver information", 5, RES_LOOKUP, false, LOOKUP_AIS_TRANSCEIVER, "" }
  , { "Reserved", 3, RES_BINARY, false, 0, "reserved" }

  , { "Destination ID", 30, RES_INTEGER, false, 0, "" }
  , { "Reserved", 2, RES_BINARY, false, 0, "reserved" }
  , { "Message ID A", BYTES(1), RES_INTEGER, false, 0, "" }
  , { "Slot Offset A", 14, RES_INTEGER, false, 0, "" }
  , { "Reserved", 2, RES_BINARY, false, 0, "reserved" }
  , { "Message ID B", BYTES(1), RES_INTEGER, false, 0, "" }
  , { "Slot Offset B", 14, RES_INTEGER, false, 0, "" }
  , { "Reserved", 2, RES_BINARY, false, 0, "reserved" }
  , { 0 }
  }
}

,
{ "AIS Assignment Mode Command", 129804, true, 23, 3,
  { { "Message ID", 6, 1, false, 0, "" }
  , { "Repeat Indicator", 2, RES_LOOKUP, false, LOOKUP_REPEAT_INDICATOR, "" }
  , { "Source ID", BYTES(4), RES_INTEGER, false, "MMSI", "" }
  , { "Reserved", 1, RES_BINARY, false, 0, "reserved" }
  , { "AIS Transceiver information", 5, RES_LOOKUP, false, LOOKUP_AIS_TRANSCEIVER, "" }
  , { "Reserved", 2, RES_BINARY, false, 0, "reserved" }

  , { "Destination ID", BYTES(4), RES_INTEGER, false, "MMSI", "" }
  , { "Offset", BYTES(2), RES_INTEGER, false, 0, "" }
  , { "Increment", BYTES(2), RES_INTEGER, false, 0, "" }
  , { 0 }
  }
}

,
{ "AIS Data Link Management Message", 129805, false, 8, 4,
  { { "Message ID", 6, 1, false, 0, "" }
  , { "Repeat Indicator", 2, RES_LOOKUP, false, LOOKUP_REPEAT_INDICATOR, "" }
  , { "Source ID", 30, RES_INTEGER, false, 0, "" }
  , { "Reserved", 2, RES_BINARY, false, 0, "reserved" }
  , { "AIS Transceiver information", 5, RES_LOOKUP, false, LOOKUP_AIS_TRANSCEIVER, "" }
  , { "Reserved", 3, RES_BINARY, false, 0, "reserved" }

  , { "Offset", 10, RES_INTEGER, false, 0, "" }
  , { "Number of Slots", BYTES(1), RES_INTEGER, false, 0, "" }
  , { "Timeout", BYTES(1), RES_INTEGER, false, 0, "" }
  , { "Increment", BYTES(1), RES_INTEGER, false, 0, "" }
  , { 0 }
  }
}

,
{ "AIS Channel Management", 129806, false, 8, 0,
  { { "Message ID", 6, 1, false, 0, "" }
  , { "Repeat Indicator", 2, RES_LOOKUP, false, LOOKUP_REPEAT_INDICATOR, "" }
  , { "Source ID", 30, RES_INTEGER, false, 0, "" }
  , { "Reserved", 2, RES_BINARY, false, 0, "reserved" }
  , { "AIS Transceiver information", 5, RES_LOOKUP, false, LOOKUP_AIS_TRANSCEIVER, "" }
  , { "Reserved", 3, RES_BINARY, false, 0, "reserved" }
  , { "Channel A", 7, 1, false, 0, "" }
  , { "Channel B", 7, 1, false, 0, "" }
  , { "Reserved", 2, RES_BINARY, false, 0, "reserved" }
  , { "Power", BYTES(1), 1, false, 0, "reserved" }
  , { "Tx/Rx Mode", BYTES(1), RES_INTEGER, false, 0, "" }
  , { "North East Longitude Corner 1", BYTES(4), RES_LONGITUDE, true, "deg", "" }
  , { "North East Latitude Corner 1", BYTES(4), RES_LATITUDE, true, "deg", "" }
  , { "South West Longitude Corner 1", BYTES(4), RES_LONGITUDE, true, "deg", "" }
  , { "South West Latitude Corner 2", BYTES(4), RES_LATITUDE, true, "deg", "" }
  , { "Reserved", 6, RES_BINARY, false, 0, "reserved" }
  , { "Addressed or Broadcast Message Indicator", 2, 1, false, 0, "" }
  , { "Channel A Bandwidth", 7, RES_INTEGER, false, 0, "" }
  , { "Channel B Bandwidth", 7, RES_INTEGER, false, 0, "" }
  , { "Reserved", 2, RES_BINARY, false, 0, "reserved" }
  , { "Transitional Zone Size", BYTES(1), 1, false, 0, "" }
  , { 0 }
  }
}

,
{ "AIS Class B Group Assignment", 129807, false, 8, 0,
  { { "Message ID", 6, 1, false, 0, "" }
  , { "Repeat Indicator", 2, RES_LOOKUP, false, LOOKUP_REPEAT_INDICATOR, "" }
  , { "Source ID", 30, RES_INTEGER, false, 0, "" }
  , { "Reserved", 2, RES_BINARY, false, 0, "reserved" }
  , { "Tx/Rx Mode", 2, RES_INTEGER, false, 0, "" }
  , { "Reserved", 6, RES_BINARY, false, 0, "reserved" }
  , { "North East Longitude Corner 1", BYTES(4), RES_LONGITUDE, true, "deg", "" }
  , { "North East Latitude Corner 1", BYTES(4), RES_LATITUDE, true, "deg", "" }
  , { "South West Longitude Corner 1", BYTES(4), RES_LONGITUDE, true, "deg", "" }
  , { "South West Latitude Corner 2", BYTES(4), RES_LATITUDE, true, "deg", "" }
  , { "Station Type", BYTES(1), 1, false, 0, "" }
  , { "Reserved", 2, RES_BINARY, false, 0, "reserved" }
  , { "Ship and Cargo Filter", 6, 1, false, 0, "" }
  , { "Reserved", 2, RES_BINARY, false, 0, "reserved" }
  , { "Reporting Interval", BYTES(2), 1, false, 0, "" }
  , { "Quiet Time", BYTES(2), 1, false, 0, "" }
  , { 0 }
  }
}

,
{ "DSC Call Information", 129808, false, 8, 2,
  { { "DSC Format Symbol", BYTES(1), 1, false, 0, "" }
  , { "DSC Category Symbol", BYTES(1), 1, false, 0, "" }
  , { "DSC Message Address", BYTES(1), 1, false, 0, "" }
  , { "Nature of Distress or 1st Telecommand", BYTES(1), 1, false, 0, "" }
  , { "Subsequent Communication Mode or 2nd Telecommand", BYTES(1), 1, false, 0, "" }
  , { "Proposed Rx Frequency/Channel", BYTES(1), 1, false, 0, "" }
  , { "Proposed Tx Frequency/Channel", BYTES(1), 1, false, 0, "" }
  , { "Telephone Number", BYTES(1), 1, false, 0, "" }
  , { "Latitude of Vessel Reported", BYTES(4), RES_LATITUDE, true, "deg", "" }
  , { "Longitude of Vessel Reported", BYTES(4), RES_LONGITUDE, true, "deg", "" }
  , { "Time of Position", BYTES(4), RES_TIME, false, "s", "Seconds since midnight" }
  , { "User ID of Ship In Distress", BYTES(4), RES_INTEGER, false, "MMSI", "" }
  , { "DSC EOS Symbol", BYTES(1), 1, false, 0, "" }
  , { "Expansion Enabled", BYTES(1), 1, false, 0, "" }
  , { "Calling Rx Frequency/Channel", BYTES(1), 1, false, 0, "" }
  , { "Calling Tx Frequency/Channel", BYTES(1), 1, false, 0, "" }
  , { "Time of Receipt", BYTES(4), RES_TIME, false, "s", "Seconds since midnight" }
  , { "Date of Receipt", BYTES(2), RES_DATE, false, "days", "Days since January 1, 1970" }
  , { "DSC Equipment Assigned Message ID", BYTES(1), 1, false, 0, "" }
  , { "DSC Expansion Field Symbol", BYTES(1), 1, false, 0, "" }
  , { "DSC Expansion Field Data", BYTES(1), 1, false, 0, "" }
  , { 0 }
  }
}

,
{ "AIS Class B static data (msg 24 Part A)", 129809, false, 20 + 4 + 1, 0,
  { { "Message ID", 6, 1, false, 0, "" }
  , { "Repeat indicator", 2, RES_LOOKUP, false, LOOKUP_REPEAT_INDICATOR, "" }
  , { "User ID", BYTES(4), RES_INTEGER, false, "MMSI", "" }
  , { "Name", BYTES(20), RES_ASCII, false, 0, "" }
  , { 0 }
  }
}

,
{ "AIS Class B static data (msg 24 Part B)", 129810, false, 0x25 - 4, 0,
  { { "Message ID", 6, 1, false, 0, "" }
  , { "Repeat indicator", 2, RES_LOOKUP, false, LOOKUP_REPEAT_INDICATOR, "" }
  , { "User ID", BYTES(4), RES_INTEGER, false, "MMSI", "" }
  , { "Type of ship", BYTES(1), RES_LOOKUP, false, LOOKUP_SHIP_TYPE, "" }
  , { "Vendor ID", BYTES(7), RES_ASCII, false, 0, "" }
  , { "Callsign", BYTES(7), RES_ASCII, false, 0, "0=unavailable" }
  , { "Length", BYTES(2), 0.1, false, "m", "" }
  , { "Beam", BYTES(2), 0.1, false, "m", "" }
  , { "Position reference from Starboard", BYTES(2), 0.1, false, "m", "" }
  , { "Position reference from Bow", BYTES(2), 0.1, false, "m", "" }
  , { "Mothership User ID", BYTES(4), RES_INTEGER, false, "MMSI", "MMSI of mother ship sent by daughter vessels" }
  , { "Reserved", 2, RES_BINARY, false, 0, "reserved" }
  , { "Spare", 6, RES_INTEGER, false, 0, "0=unavailable" }
  , { 0 }
  }
}

,
{ "Label", 130060, false, 0, 0,
  { { 0 }
  }
}

,
{ "Channel Source Configuration", 130061, false, 0, 0,
  { { 0 }
  }
}

,
{ "Route and WP Service - Database List", 130064, false, 8, 9,
  { { "Start Database ID", BYTES(1), 1, false, 0, "" }
  , { "nItems", BYTES(1), 1, false, 0, "" }
  , { "Number of Databases Available", BYTES(1), 1, false, 0, "" }

  , { "Database ID", BYTES(1), 1, false, 0, "" }
  , { "Database Name", BYTES(8), RES_ASCII, false, 0, "" }
  , { "Database Timestamp", BYTES(4), RES_TIME, false, "s", "Seconds since midnight" }
  , { "Database Datestamp", BYTES(2), RES_DATE, false, "days", "Days since January 1, 1970" }
  , { "WP Position Resolution", 6, 1, false, 0, "" }
  , { "Reserved", 2, RES_BINARY, false, 0, "reserved" }
  , { "Number of Routes in Database", BYTES(2), 1, false, 0, "" }
  , { "Number of WPs in Database", BYTES(2), 1, false, 0, "" }
  , { "Number of Bytes in Database", BYTES(2), 1, false, 0, "" }
  , { 0 }
  }
}

,
{ "Route and WP Service - Route List", 130065, false, 8, 6,
  { { "Start Route ID", BYTES(1), 1, false, 0, "" }
  , { "nItems", BYTES(1), 1, false, 0, "" }
  , { "Number of Routes in Database", BYTES(1), 1, false, 0, "" }

  , { "Database ID", BYTES(1), 1, false, 0, "" }
  , { "Route ID", BYTES(1), 1, false, 0, "" }
  , { "Route Name", BYTES(8), RES_ASCII, false, 0, "" }
  , { "Reserved", 4, RES_BINARY, false, 0, "reserved" }
  , { "WP Identification Method", 2, 1, false, 0, "" }
  , { "Route Status", 2, 1, false, 0, "" }
  , { 0 }
  }
}

,
{ "Route and WP Service - Route/WP-List Attributes", 130066, false, 8, 0,
  { { "Database ID", BYTES(1), 1, false, 0, "" }
  , { "Route ID", BYTES(1), 1, false, 0, "" }
  , { "Route/WP-List Name", BYTES(8), RES_ASCII, false, 0, "" }
  , { "Route/WP-List Timestamp", BYTES(4), RES_TIME, false, "s", "Seconds since midnight" }
  , { "Route/WP-List Datestamp", BYTES(2), RES_DATE, false, "days", "Days since January 1, 1970" }
  , { "Change at Last Timestamp", BYTES(1), 1, false, 0, "" }
  , { "Number of WPs in the Route/WP-List", BYTES(2), 1, false, 0, "" }
  , { "Critical supplementary parameters", BYTES(1), 1, false, 0, "" }
  , { "Navigation Method", 2, 1, false, 0, "" }
  , { "WP Identification Method", 2, 1, false, 0, "" }
  , { "Route Status", 2, 1, false, 0, "" }
  , { "XTE Limit for the Route", BYTES(2), 1, false, 0, "" }
  , { 0 }
  }
}

,
{ "Route and WP Service - Route - WP Name & Position", 130067, false, 8, 4,
  { { "Start RPS#", BYTES(1), 1, false, 0, "" }
  , { "nItems", BYTES(1), 1, false, 0, "" }
  , { "Number of WPs in the Route/WP-List", BYTES(2), 1, false, 0, "" }
  , { "Database ID", BYTES(1), 1, false, 0, "" }
  , { "Route ID", BYTES(1), 1, false, 0, "" }

  , { "WP ID", BYTES(1), 1, false, 0, "" }
  , { "WP Name", BYTES(8), RES_ASCII, false, 0, "" }
  , { "WP Latitude", BYTES(4), RES_LATITUDE, true, "deg", "" }
  , { "WP Longitude", BYTES(4), RES_LONGITUDE, true, "deg", "" }
  , { 0 }
  }
}

,
{ "Route and WP Service - Route - WP Name", 130068, false, 8, 2,
  { { "Start RPS#", BYTES(1), 1, false, 0, "" }
  , { "nItems", BYTES(1), 1, false, 0, "" }
  , { "Number of WPs in the Route/WP-List", BYTES(2), 1, false, 0, "" }
  , { "Database ID", BYTES(1), 1, false, 0, "" }
  , { "Route ID", BYTES(1), 1, false, 0, "" }
  , { "WP ID", BYTES(1), 1, false, 0, "" }
  , { "WP Name", BYTES(8), RES_ASCII, false, 0, "" }
  , { 0 }
  }
}

,
{ "Route and WP Service - XTE Limit & Navigation Method", 130069, false, 8, 6,
  { { "Start RPS#", BYTES(1), 1, false, 0, "" }
  , { "nItems", BYTES(1), 1, false, 0, "" }
  , { "Number of WPs with a specific XTE Limit or Nav. Method", BYTES(2), 1, false, 0, "" }

  , { "Database ID", BYTES(1), 1, false, 0, "" }
  , { "Route ID", BYTES(1), 1, false, 0, "" }
  , { "RPS#", BYTES(1), 1, false, 0, "" }
  , { "XTE limit in the leg after WP", BYTES(2), 1, false, 0, "" }
  , { "Nav. Method in the leg after WP", 4, 1, false, 0, "" }
  , { "Reserved", 4, RES_BINARY, false, 0, "" }
  , { 0 }
  }
}

,
{ "Route and WP Service - WP Comment", 130070, false, 8, 2,
  { { "Start ID", BYTES(1), 1, false, 0, "" }
  , { "nItems", BYTES(1), 1, false, 0, "" }
  , { "Number of WPs with Comments", BYTES(2), 1, false, 0, "" }
  , { "Database ID", BYTES(1), 1, false, 0, "" }
  , { "Route ID", BYTES(1), 1, false, 0, "" }

  , { "WP ID / RPS#", BYTES(1), 1, false, 0, "" }
  , { "Comment", BYTES(8), RES_ASCII, false, 0, "" }
  , { 0 }
  }
}

,
{ "Route and WP Service - Route Comment", 130071, false, 8, 2,
  { { "Start Route ID", BYTES(1), 1, false, 0, "" }
  , { "nItems", BYTES(1), 1, false, 0, "" }
  , { "Number of Routes with Comments", BYTES(2), 1, false, 0, "" }
  , { "Database ID", BYTES(1), 1, false, 0, "" }

  , { "Route ID", BYTES(1), 1, false, 0, "" }
  , { "Comment", BYTES(8), RES_ASCII, false, 0, "" }
  , { 0 }
  }
}

,
{ "Route and WP Service - Database Comment", 130072, false, 8, 2,
  { { "Start Database ID", BYTES(1), 1, false, 0, "" }
  , { "nItems", BYTES(1), 1, false, 0, "" }
  , { "Number of Databases with Comments", BYTES(2), 1, false, 0, "" }

  , { "Database ID", BYTES(1), 1, false, 0, "" }
  , { "Comment", BYTES(8), RES_ASCII, false, 0, "" }
  , { 0 }
  }
}

,
{ "Route and WP Service - Radius of Turn", 130073, false, 8, 2,
  { { "Start RPS#", BYTES(1), 1, false, 0, "" }
  , { "nItems", BYTES(1), 1, false, 0, "" }
  , { "Number of WPs with a specific Radius of Turn", BYTES(2), 1, false, 0, "" }
  , { "Database ID", BYTES(1), 1, false, 0, "" }
  , { "Route ID", BYTES(1), 1, false, 0, "" }

  , { "RPS#", BYTES(1), 1, false, 0, "" }
  , { "Radius of Turn", BYTES(2), 1, false, 0, "" }
  , { 0 }
  }
}

,
{ "Route and WP Service - WP List - WP Name & Position", 130074, false, 8, 4,
  { { "Start WP ID", BYTES(1), 1, false, 0, "" }
  , { "nItems", BYTES(1), 1, false, 0, "" }
  , { "Number of valid WPs in the WP-List", BYTES(2), 1, false, 0, "" }
  , { "Database ID", BYTES(1), 1, false, 0, "" }
  , { "Reserved", BYTES(1), 1, false, 0, "reserved" }

  , { "WP ID", BYTES(1), 1, false, 0, "" }
  , { "WP Name", BYTES(8), RES_ASCII, false, 0, "" }
  , { "WP Latitude", BYTES(4), RES_LATITUDE, true, "deg", "" }
  , { "WP Longitude", BYTES(4), RES_LONGITUDE, true, "deg", "" }
  , { 0 }
  }
}

  /* http://askjackrabbit.typepad.com/ask_jack_rabbit/page/7/ */
,
{ "Wind Data", 130306, true, 6, 0,
  { { "SID", BYTES(1), 1, false, 0, "" }
  , { "Wind Speed", BYTES(2), 0.01, false, "m/s", "" }
  , { "Wind Angle", BYTES(2), RES_DEGREES, false, "deg", "" }
  , { "Reference", 3, RES_LOOKUP, false, LOOKUP_WIND_REFERENCE, "" }
  , { 0 }
  }
}

  /* Water temperature, Transducer Measurement */
,
{ "Environmental Parameters", 130310, true, 7, 0,
  { { "SID", BYTES(1), 1, false, 0, "" }
  , { "Water Temperature", BYTES(2), RES_TEMPERATURE, false, "K", "" }
  , { "Outside Ambient Air Temperature", BYTES(2), RES_TEMPERATURE, false, "K", "" }
  , { "Atmospheric Pressure", BYTES(2), RES_PRESSURE, false, "hPa", "" }
  , { 0 }
  }
}

,
{ "Environmental Parameters", 130311, true, 8, 0,
  { { "SID", BYTES(1), 1, false, 0, "" }
  , { "Temperature Instance", 6, RES_LOOKUP, false, ",0=Sea,1=Outside,2=Inside,3=Engine room,4=Main Cabin", "" }
  , { "Humidity Instance", 2, RES_LOOKUP, false, ",0=Inside,1=Outside", "" }
  , { "Temperature", BYTES(2), RES_TEMPERATURE, false, "K", "" }
  , { "Humidity", BYTES(2), 100.0/25000, true, "%", "" }
  , { "Atmospheric Pressure", BYTES(2), RES_PRESSURE, false, "hPa", "" }
  , { 0 }
  }
}

,
{ "Temperature", 130312, true, 8, 0,
  { { "SID", BYTES(1), 1, false, 0, "" }
  , { "Temperature Instance", BYTES(1), 1, false, 0, "" }
  , { "Temperature Source", BYTES(1), RES_LOOKUP, false, LOOKUP_TEMPERATURE_SOURCE, "" }
  , { "Actual Temperature", BYTES(2), RES_TEMPERATURE, false, "K", "" }
  , { "Set Temperature", BYTES(2), RES_TEMPERATURE, false, "K", "" }
  , { 0 }
  }
}

,
{ "Humidity", 130313, true, 8, 0,
  { { "SID", BYTES(1), 1, false, 0, "" }
  , { "Humidity Instance", BYTES(1), 1, false, 0, "" }
  , { "Humidity Source", BYTES(1), 1, false, 0, "" }
  , { "Actual Humidity", BYTES(2), 100.0/25000, true, "%", "" }
  , { "Set Humidity", BYTES(2), 100.0/25000, true, "%", "" }
  , { 0 }
  }
}

,
{ "Actual Pressure", 130314, false, 8, 0,
  { { "SID", BYTES(1), 1, false, 0, "" }
  , { "Pressure Instance", 4, 1, false, 0, "" }
  , { "Pressure Source", 4, 1, false, 0, "" }
  , { "Pressure", BYTES(2), 1, false, "", "" }
  , { 0 }
  }
}

,
{ "Set Pressure", 130315, true, 8, 0,
  { { "SID", BYTES(1), 1, false, 0, "" }
  , { "Pressure Instance", BYTES(1), 1, false, 0, "" }
  , { "Pressure Source", BYTES(1), RES_LOOKUP, false, 0, ",0=Atmospheric,1=Water,2=Steam,3=Compressed Air,4=Hydraulic" }
  , { "Pressure", BYTES(4), 2.1*100000, false, "kPa", "" }
  , { 0 }
  }
}

,
{ "Temperature Extended Range", 130316, false, 8, 0,
  { { 0 }
  }
}

,
{ "Tide Station Data", 130320, true, 20, 0,
  { { "Mode", 4, RES_LOOKUP, false, LOOKUP_RESIDUAL_MODE, "" }
  , { "Tide Tendency", 2, RES_LOOKUP, false, ",0=Falling,1=Rising", "" }
  , { "Reserved", 2, RES_BINARY, false, 0, "" }
  , { "Measurement Date", BYTES(2), RES_DATE, false, "days", "Days since January 1, 1970" }
  , { "Measurement Time", BYTES(4), RES_TIME, false, "s", "Seconds since midnight" }
  , { "Station Latitude", BYTES(4), RES_LATITUDE, true, "deg", "" }
  , { "Station Longitude", BYTES(4), RES_LONGITUDE, true, "deg", "" }
  , { "Tide Level", BYTES(2), 0.001, true, "m", "Relative to MLLW" }
  , { "Tide Level standard deviation", BYTES(2), 0.01, false, "m", "" }
  , { "Station ID", BYTES(2), RES_STRING, false, 0, "" }
  , { "Station Name", BYTES(2), RES_STRING, false, 0, "" }
  , { 0 }
  }
}

,
{ "Salinity Station Data", 130321, true, 22, 0,
  { { "Mode", 4, RES_LOOKUP, false, LOOKUP_RESIDUAL_MODE, "" }
  , { "Reserved", 4, RES_BINARY, false, 0, "" }
  , { "Measurement Date", BYTES(2), RES_DATE, false, "days", "Days since January 1, 1970" }
  , { "Measurement Time", BYTES(4), RES_TIME, false, "s", "Seconds since midnight" }
  , { "Station Latitude", BYTES(4), RES_LATITUDE, true, "deg", "" }
  , { "Station Longitude", BYTES(4), RES_LONGITUDE, true, "deg", "" }
  , { "Salinity", BYTES(4), RES_FLOAT, true, "ppt", "The average Salinity of ocean water is about 35 grams of salts per kilogram of sea water (g/kg), usually written as 35 ppt which is read as 35 parts per thousand." }
  , { "Water Temperature", BYTES(2), RES_TEMPERATURE, false, "K", "" }
  , { "Station ID", BYTES(2), RES_STRING, false, 0, "" }
  , { "Station Name", BYTES(2), RES_STRING, false, 0, "" }
  , { 0 }
  }
}

,
{ "Current Station Data", 130322, false, 8, 0,
  { { "Mode", 4, 1, false, 0, "" }
  , { "Reserved", 4, RES_BINARY, false, 0, "" }
  , { "Measurement Date", BYTES(2), RES_DATE, false, "days", "Days since January 1, 1970" }
  , { "Measurement Time", BYTES(4), RES_TIME, false, "s", "Seconds since midnight" }
  , { "Station Latitude", BYTES(4), RES_LATITUDE, true, "deg", "" }
  , { "Station Longitude", BYTES(4), RES_LONGITUDE, true, "deg", "" }
  , { "Measurement Depth", BYTES(4), 0.01, false, "m", "Depth below transducer" }
  , { "Current speed", BYTES(2), 0.01, false, "m/s", "" }
  , { "Current flow direction", BYTES(2), RES_DEGREES, false, "deg", "" }
  , { "Water Temperature", BYTES(2), RES_TEMPERATURE, false, "K", "" }
  , { "Station ID", BYTES(2), RES_STRING, false, 0, "" }
  , { "Station Name", BYTES(2), RES_STRING, false, 0, "" }
  , { 0 }
  }
}

,
{ "Meteorological Station Data", 130323, false, 0x1e, 0,
  { { "Mode", 4, 1, false, 0, "" }
  , { "Reserved", 4, RES_BINARY, false, 0, "" }
  , { "Measurement Date", BYTES(2), RES_DATE, false, "days", "Days since January 1, 1970" }
  , { "Measurement Time", BYTES(4), RES_TIME, false, "s", "Seconds since midnight" }
  , { "Station Latitude", BYTES(4), RES_LATITUDE, true, "deg", "" }
  , { "Station Longitude", BYTES(4), RES_LONGITUDE, true, "deg", "" }
  , { "Wind Speed", BYTES(2), 0.01, false, "m/s", "" }
  , { "Wind Direction", BYTES(2), RES_DEGREES, false, "deg", "" }
  , { "Wind Reference", 3, RES_LOOKUP, false, LOOKUP_WIND_REFERENCE, "" }
  , { "Reserved", 5, RES_BINARY, false, "", "reserved" }
  , { "Wind Gusts", BYTES(2), 0.01, false, "m/s", "" }
  , { "Atmospheric Pressure", BYTES(2), RES_PRESSURE, false, "hPa", "" }
  , { "Ambient Temperature", BYTES(2), RES_TEMPERATURE, false, "K", "" }
  , { "Station ID", BYTES(2), RES_STRING, false, 0, "" }
  , { "Station Name", BYTES(2), RES_STRING, false, 0, "" }
  , { 0 }
  }
}

,
{ "Moored Buoy Station Data", 130324, false, 8, 0,
  { { "Mode", 4, 1, false, 0, "" }
  , { "Reserved", 4, RES_BINARY, false, 0, "" }
  , { "Measurement Date", BYTES(2), RES_DATE, false, "days", "Days since January 1, 1970" }
  , { "Measurement Time", BYTES(4), RES_TIME, false, "s", "Seconds since midnight" }
  , { "Station Latitude", BYTES(4), RES_LATITUDE, true, "deg", "" }
  , { "Station Longitude", BYTES(4), RES_LONGITUDE, true, "deg", "" }
  , { "Wind Speed", BYTES(2), 0.01, false, "m/s", "" }
  , { "Wind Direction", BYTES(2), RES_DEGREES, false, "deg", "" }
  , { "Wind Reference", 3, RES_LOOKUP, false, LOOKUP_WIND_REFERENCE, "" }
  , { "Reserved", 5, RES_BINARY, false, "", "reserved" }
  , { "Wind Gusts", BYTES(2), 0.01, false, "m/s", "" }
  , { "Wave Height", BYTES(2), 1, false, 0, "" }
  , { "Dominant Wave Period", BYTES(2), 1, false, 0, "" }
  , { "Atmospheric Pressure", BYTES(2), RES_PRESSURE, false, "hPa", "" }
  , { "Pressure Tendency Rate", BYTES(2), 1, false, "", "" }
  , { "Air Temperature", BYTES(2), RES_TEMPERATURE, false, "K", "" }
  , { "Water Temperature", BYTES(2), RES_TEMPERATURE, false, "K", "" }
  , { "Station ID", BYTES(8), RES_ASCII, false, 0, "" }
  , { 0 }
  }
}

,
{ "Payload Mass", 130560, false, 0, 0,
  { { 0 }
  }
}

  /* http://www.nmea.org/Assets/20130905%20amendment%20at%202000%20201309051%20watermaker%20input%20setting%20and%20status%20pgn%20130567.pdf

  This PGN may be requested or used to command and configure a number of Watermaker controls. The Command Group Function PGN 126208
  is used perform the following: start/stop a production, start/stop rinse or flush operation , start/stop low and high pressure
  pump and perform an emergency stop. The Request Group Function PGN 126208 or ISO Request PGN 059904 may be used to request this
  PGN. This PGN also provides Watermaker status and measurement information. The PGN is broadcast periodically.

  */
,
{ "Watermaker Input Setting and Status", 130567, true, 24, 0,
  { { "Watermaker Operating State", 6, RES_LOOKUP, false, "0=Stopped,1=Starting,2=Running,3=Stopping,4=Flushing,5=Rinsing,6=Initiating,7=Manual Mode,62=Error,63=Unavailable", "" }
  , { "Production Start/Stop", 2, RES_LOOKUP, false, LOOKUP_YES_NO, "" }
  , { "Rinse Start/Stop", 2, RES_LOOKUP, false, LOOKUP_YES_NO, "" }
  , { "Low Pressure Pump Status", 2, RES_LOOKUP, false, LOOKUP_YES_NO, "" }
  , { "High Pressure Pump Status", 2, RES_LOOKUP, false, LOOKUP_YES_NO, "" }
  , { "Emergency Stop", 2, RES_LOOKUP, false, LOOKUP_YES_NO, "" }
  , { "Product Solenoid Valve Status", 2, RES_LOOKUP, false, LOOKUP_OK_WARNING, "" }
  , { "Flush Mode Status", 2, RES_LOOKUP, false, LOOKUP_YES_NO, "" }
  , { "Salinity Status", 2, RES_LOOKUP, false, LOOKUP_OK_WARNING, "" }
  , { "Sensor Status", 2, RES_LOOKUP, false, LOOKUP_OK_WARNING, "" }
  , { "Oil Change Indicator Status", 2, RES_LOOKUP, false, LOOKUP_OK_WARNING, "" }
  , { "Filter Status", 2, RES_LOOKUP, false, LOOKUP_OK_WARNING, "" }
  , { "System Status", 2, RES_LOOKUP, false, LOOKUP_OK_WARNING, "" }
  , { "Reserved", 2, RES_BINARY, false, 0, "Reserved" }
  , { "Salinity", BYTES(2), RES_INTEGER, false, "ppm", "" }
  , { "Product Water Temperature", BYTES(2), RES_TEMPERATURE, false, "K", "" }
  , { "Pre-filter Pressure", BYTES(2), RES_PRESSURE, false, "hPa", "" }
  , { "Post-filter Pressure", BYTES(2), RES_PRESSURE, false, "hPa", "" }
  , { "Feed Pressure", BYTES(2), RES_PRESSURE, true, "kPa", "" }
  , { "System High Pressure", BYTES(2), RES_PRESSURE, false, "kPa", "" }
  , { "Product Water Flow", BYTES(2), 0.1, true, "L/h", "" }
  , { "Brine Water Flow", BYTES(2), 0.1, true, "L/h", "" }
  , { "Run Time", BYTES(4), RES_INTEGER, false, "s", "" }
  , { 0 }
  }
}

,
{ "Small Craft Status", 130576, true, 2, 0,
  { { "Port trim tab", BYTES(1), 1, true, 0, "" }
  , { "Starboard trim tab", BYTES(1), 1, true, 0, "" }
  , { 0 }
  }
}

,
{ "Direction Data", 130577, true, 14, 0,
  { { "Data Mode", 4, RES_LOOKUP, false, LOOKUP_RESIDUAL_MODE, "" }
  , { "COG Reference", 2, RES_LOOKUP, false, LOOKUP_DIRECTION_REFERENCE, "" }
  , { "Reserved", 2, RES_BINARY, false, 0, "Reserved" }
  , { "SID", BYTES(1), 1, false, 0, "" }
    /* So far, 2 bytes. Very sure of this given molly rose data */
  , { "COG", BYTES(2), RES_DEGREES, false, "deg", "" }
  , { "SOG", BYTES(2), 0.01, false, "m/s", "" }
  , { "Heading", BYTES(2), RES_DEGREES, false, "deg", "" }
  , { "Speed through Water", BYTES(2), 0.01, false, "m/s", "" }
  , { "Set", BYTES(2), RES_DEGREES, false, "deg", "" }
  , { "Drift", BYTES(2), 0.01, false, "m/s", "" }
  , { 0 }
  }
}

,
{ "Vessel Speed Components", 130578, true, 12, 0,
  { { "Longitudinal Speed, Water-referenced", BYTES(2), 0.001, true, "m/s", "" }
  , { "Transverse Speed, Water-referenced", BYTES(2), 0.001, true, "m/s", "" }
  , { "Longitudinal Speed, Ground-referenced", BYTES(2), 0.001, true, "m/s", "" }
  , { "Transverse Speed, Ground-referenced", BYTES(2), 0.001, true, "m/s", "" }
  , { "Stern Speed, Water-referenced", BYTES(2), 0.001, true, "m/s", "" }
  , { "Stern Speed, Ground-referenced", BYTES(2), 0.001, true, "m/s", "" }
  , { 0 }
  }
}

,
{ "SonicHub: Init #2", 130816, false, 9, 0,
  { { "Manufacturer Code", 11, RES_MANUFACTURER, false, "=275", "Navico" }
  , { "Reserved", 2, 1, false, 0, "" }
  , { "Industry Code", 3, RES_LOOKUP, false, LOOKUP_INDUSTRY_CODE, "" }
  , { "Reserved", BYTES(1), 1, false, 0, "" }
  , { "Proprietary ID", BYTES(1), RES_LOOKUP, false, "=1", "Init #2" }
  , { "Control", BYTES(1), RES_LOOKUP, false, ",0=Set,128=Ack", "" }
  , { "A", BYTES(2), RES_INTEGER, false, 0, "" }
  , { "B", BYTES(2), RES_INTEGER, false, 0, "" }
  , { 0 }
  }
}

,
{ "SonicHub: AM Radio", 130816, false, 0x40, 0,
  { { "Manufacturer Code", 11, RES_MANUFACTURER, false, "=275", "Navico" }
  , { "Reserved", 2, 1, false, 0, "" }
  , { "Industry Code", 3, RES_LOOKUP, false, LOOKUP_INDUSTRY_CODE, "" }
  , { "Reserved", BYTES(1), 1, false, 0, "" }
  , { "Proprietary ID", BYTES(1), RES_LOOKUP, false, "=4", "AM Radio" }
  , { "Control", BYTES(1), RES_LOOKUP, false, ",0=Set,128=Ack", "" }
  , { "Item", BYTES(1), RES_LOOKUP, false, ",1=Seeking up,2=Tuned,3=Seeking down", "" }
  , { "Frequency", BYTES(4), 0.001, false, "kHz", "" }
  , { "Noise level", 2, 1, false, 0, "" }  // Not sure about this
  , { "Signal level", 4, 1, false, 0, "" } // ... and this, doesn't make complete sense compared to display
  , { "Reserved", 2, 1, false, 0, "" }
  , { "Text", BYTES(32), RES_STRINGLZ, false, 0, "" }
  , { 0 }
  }
}

,
{ "SonicHub: Zone info", 130816, false, 6, 0,
  { { "Manufacturer Code", 11, RES_MANUFACTURER, false, "=275", "Navico" }
  , { "Reserved", 2, 1, false, 0, "" }
  , { "Industry Code", 3, RES_LOOKUP, false, LOOKUP_INDUSTRY_CODE, "" }
  , { "Reserved", BYTES(1), 1, false, 0, "" }
  , { "Proprietary ID", BYTES(1), RES_LOOKUP, false, "=5", "Zone info" }
  , { "Control", BYTES(1), RES_LOOKUP, false, ",0=Set,128=Ack", "" }
  , { "Zone", BYTES(1), RES_INTEGER, false, 0, "" }
  , { 0 }
  }
}

,
{ "SonicHub: Source", 130816, false, 0x40, 0,
  { { "Manufacturer Code", 11, RES_MANUFACTURER, false, "=275", "Navico" }
  , { "Reserved", 2, 1, false, 0, "" }
  , { "Industry Code", 3, RES_LOOKUP, false, LOOKUP_INDUSTRY_CODE, "" }
  , { "Reserved", BYTES(1), 1, false, 0, "" }
  , { "Proprietary ID", BYTES(1), RES_LOOKUP, false, "=6", "Source" }
  , { "Control", BYTES(1), RES_LOOKUP, false, ",0=Set,128=Ack", "" }
  , { "Source", BYTES(1), RES_LOOKUP, false, ",0=AM,1=FM,2=iPod,3=USB,4=AUX,5=AUX 2,6=Mic", "" }
  , { 0 }
  }
}

,
{ "SonicHub: Source List", 130816, false, 0x40, 0,
  { { "Manufacturer Code", 11, RES_MANUFACTURER, false, "=275", "Navico" }
  , { "Reserved", 2, 1, false, 0, "" }
  , { "Industry Code", 3, RES_LOOKUP, false, LOOKUP_INDUSTRY_CODE, "" }
  , { "Reserved", BYTES(1), 1, false, 0, "" }
  , { "Proprietary ID", BYTES(1), RES_LOOKUP, false, "=8", "Source list" }
  , { "Control", BYTES(1), RES_LOOKUP, false, ",0=Set,128=Ack", "" }
  , { "Source ID", BYTES(1), RES_INTEGER, false, 0, "" }
  , { "A", 8, RES_INTEGER, false, 0, "" }
  , { "Text", BYTES(32), RES_STRINGLZ, false, 0, "" }
  , { 0 }
  }
}

,
{ "SonicHub: Control", 130816, false, 0x40, 0,
  { { "Manufacturer Code", 11, RES_MANUFACTURER, false, "=275", "Navico" }
  , { "Reserved", 2, 1, false, 0, "" }
  , { "Industry Code", 3, RES_LOOKUP, false, LOOKUP_INDUSTRY_CODE, "" }
  , { "Reserved", BYTES(1), 1, false, 0, "" }
  , { "Proprietary ID", BYTES(1), RES_LOOKUP, false, "=9", "Control" }
  , { "Control", BYTES(1), RES_LOOKUP, false, ",0=Set,128=Ack", "" }
  , { "Item", BYTES(1), RES_LOOKUP, false, ",1=Mute on,2=Mute off", "" }
  , { 0 }
  }
}

,
{ "SonicHub: Unknown", 130816, false, 0x40, 0,
  { { "Manufacturer Code", 11, RES_MANUFACTURER, false, "=275", "Navico" }
  , { "Reserved", 2, 1, false, 0, "" }
  , { "Industry Code", 3, RES_LOOKUP, false, LOOKUP_INDUSTRY_CODE, "" }
  , { "Reserved", BYTES(1), 1, false, 0, "" }
  , { "Proprietary ID", BYTES(1), RES_LOOKUP, false, "=9", "Unknown" }
  , { "Control", BYTES(1), RES_LOOKUP, false, ",0=Set,128=Ack", "" }
  , { "A", 8, RES_INTEGER, false, 0, "" }
  , { "B", 8, RES_INTEGER, false, 0, "" }
  , { 0 }
  }
}

,
{ "SonicHub: FM Radio", 130816, false, 0x40, 0,
  { { "Manufacturer Code", 11, RES_MANUFACTURER, false, "=275", "Navico" }
  , { "Reserved", 2, 1, false, 0, "" }
  , { "Industry Code", 3, RES_LOOKUP, false, LOOKUP_INDUSTRY_CODE, "" }
  , { "Reserved", BYTES(1), 1, false, 0, "" }
  , { "Proprietary ID", BYTES(1), RES_LOOKUP, false, "=12", "FM Radio" }
  , { "Control", BYTES(1), RES_LOOKUP, false, ",0=Set,128=Ack", "" }
  , { "Item", BYTES(1), RES_LOOKUP, false, ",1=Seeking up,2=Tuned,3=Seeking down", "" }
  , { "Frequency", BYTES(4), 0.001, false, "kHz", "" }
  , { "Noise level", 2, 1, false, 0, "" }  // Not sure about this
  , { "Signal level", 4, 1, false, 0, "" } // ... and this, doesn't make complete sense compared to display
  , { "Reserved", 2, 1, false, 0, "" }
  , { "Text", BYTES(32), RES_STRINGLZ, false, 0, "" }
  , { 0 }
  }
}

,
{ "SonicHub: Playlist", 130816, false, 0x40, 0,
  { { "Manufacturer Code", 11, RES_MANUFACTURER, false, "=275", "Navico" }
  , { "Reserved", 2, 1, false, 0, "" }
  , { "Industry Code", 3, RES_LOOKUP, false, LOOKUP_INDUSTRY_CODE, "" }
  , { "Reserved", BYTES(1), 1, false, 0, "" }
  , { "Proprietary ID", BYTES(1), RES_LOOKUP, false, "=13", "Playlist" }
  , { "Control", BYTES(1), RES_LOOKUP, false, ",0=Set,128=Ack", "" }
  , { "Item", BYTES(1), RES_LOOKUP, false, ",1=Report,4=Next Song,6=Previous Song", "" }
  , { "A", BYTES(1), RES_INTEGER, false, 0, "" }
  , { "Current Track", BYTES(4), RES_INTEGER, false, 0, "" }
  , { "Tracks", BYTES(4), RES_INTEGER, false, 0, "" }
  , { "Length", BYTES(4), 0.001, false, "s", "" }
  , { "Position in track", BYTES(4), 0.001, false, "s", "" }
  , { 0 }
  }
}

,
{ "SonicHub: Track", 130816, false, 0x40, 0,
  { { "Manufacturer Code", 11, RES_MANUFACTURER, false, "=275", "Navico" }
  , { "Reserved", 2, 1, false, 0, "" }
  , { "Industry Code", 3, RES_LOOKUP, false, LOOKUP_INDUSTRY_CODE, "" }
  , { "Reserved", BYTES(1), 1, false, 0, "" }
  , { "Proprietary ID", BYTES(1), RES_LOOKUP, false, "=14", "Track" }
  , { "Control", BYTES(1), RES_LOOKUP, false, ",0=Set,128=Ack", "" }
  , { "Item", BYTES(4), RES_INTEGER, false, 0, "" }
  , { "Text", BYTES(32), RES_STRINGLZ, false, 0, "" }
  , { 0 }
  }
}

,
{ "SonicHub: Artist", 130816, false, 0x40, 0,
  { { "Manufacturer Code", 11, RES_MANUFACTURER, false, "=275", "Navico" }
  , { "Reserved", 2, 1, false, 0, "" }
  , { "Industry Code", 3, RES_LOOKUP, false, LOOKUP_INDUSTRY_CODE, "" }
  , { "Reserved", BYTES(1), 1, false, 0, "" }
  , { "Proprietary ID", BYTES(1), RES_LOOKUP, false, "=15", "Artist" }
  , { "Control", BYTES(1), RES_LOOKUP, false, ",0=Set,128=Ack", "" }
  , { "Item", BYTES(4), RES_INTEGER, false, 0, "" }
  , { "Text", BYTES(32), RES_STRINGLZ, false, 0, "" }
  , { 0 }
  }
}

,
{ "SonicHub: Album", 130816, false, 0x40, 0,
  { { "Manufacturer Code", 11, RES_MANUFACTURER, false, "=275", "Navico" }
  , { "Reserved", 2, 1, false, 0, "" }
  , { "Industry Code", 3, RES_LOOKUP, false, LOOKUP_INDUSTRY_CODE, "" }
  , { "Reserved", BYTES(1), 1, false, 0, "" }
  , { "Proprietary ID", BYTES(1), RES_LOOKUP, false, "=16", "Album" }
  , { "Control", BYTES(1), RES_LOOKUP, false, ",0=Set,128=Ack", "" }
  , { "Item", BYTES(4), RES_INTEGER, false, 0, "" }
  , { "Text", BYTES(32), RES_STRINGLZ, false, 0, "" }
  , { 0 }
  }
}

,
{ "SonicHub: Menu Item", 130816, false, 0x40, 0,
  { { "Manufacturer Code", 11, RES_MANUFACTURER, false, "=275", "Navico" }
  , { "Reserved", 2, 1, false, 0, "" }
  , { "Industry Code", 3, RES_LOOKUP, false, LOOKUP_INDUSTRY_CODE, "" }
  , { "Reserved", BYTES(1), 1, false, 0, "" }
  , { "Proprietary ID", BYTES(1), RES_LOOKUP, false, "=19", "Menu Item" }
  , { "Control", BYTES(1), RES_LOOKUP, false, ",0=Set,128=Ack", "" }
  , { "Item", BYTES(4), RES_INTEGER, false, 0, "" }
  , { "C", BYTES(1), 1, false, 0, "" }
  , { "D", BYTES(1), 1, false, 0, "" }
  , { "E", BYTES(1), 1, false, 0, "" }
  , { "Text", BYTES(32), RES_STRINGLZ, false, 0, "" }
  , { 0 }
  }
}

,
{ "SonicHub: Zones", 130816, false, 0x40, 0,
  { { "Manufacturer Code", 11, RES_MANUFACTURER, false, "=275", "Navico" }
  , { "Reserved", 2, 1, false, 0, "" }
  , { "Industry Code", 3, RES_LOOKUP, false, LOOKUP_INDUSTRY_CODE, "" }
  , { "Reserved", BYTES(1), 1, false, 0, "" }
  , { "Proprietary ID", BYTES(1), RES_LOOKUP, false, "=20", "Zones" }
  , { "Control", BYTES(1), RES_LOOKUP, false, ",0=Set,128=Ack", "" }
  , { "Zones", BYTES(1), RES_INTEGER, false, 0, "" }
  , { 0 }
  }
}

,
{ "SonicHub: Max Volume", 130816, false, 0x40, 0,
  { { "Manufacturer Code", 11, RES_MANUFACTURER, false, "=275", "Navico" }
  , { "Reserved", 2, 1, false, 0, "" }
  , { "Industry Code", 3, RES_LOOKUP, false, LOOKUP_INDUSTRY_CODE, "" }
  , { "Reserved", BYTES(1), 1, false, 0, "" }
  , { "Proprietary ID", BYTES(1), RES_LOOKUP, false, "=23", "Max Volume" }
  , { "Control", BYTES(1), RES_LOOKUP, false, ",0=Set,128=Ack", "" }
  , { "Zone", BYTES(1), RES_LOOKUP, false, ",0=Zone 1,1=Zone 2,2=Zone 3", "" }
  , { "Level", BYTES(1), RES_INTEGER, false, 0, "" }
  , { 0 }
  }
}

,
{ "SonicHub: Volume", 130816, false, 0x40, 0,
  { { "Manufacturer Code", 11, RES_MANUFACTURER, false, "=275", "Navico" }
  , { "Reserved", 2, 1, false, 0, "" }
  , { "Industry Code", 3, RES_LOOKUP, false, LOOKUP_INDUSTRY_CODE, "" }
  , { "Reserved", BYTES(1), 1, false, 0, "" }
  , { "Proprietary ID", BYTES(1), RES_LOOKUP, false, "=24", "Volume" }
  , { "Control", BYTES(1), RES_LOOKUP, false, ",0=Set,128=Ack", "" }
  , { "Zone", BYTES(1), RES_LOOKUP, false, ",0=Zone 1,1=Zone 2,2=Zone 3", "" }
  , { "Level", BYTES(1), RES_INTEGER, false, 0, "" }
  , { 0 }
  }
}

,
{ "SonicHub: Init #1", 130816, false, 0x40, 0,
  { { "Manufacturer Code", 11, RES_MANUFACTURER, false, "=275", "Navico" }
  , { "Reserved", 2, 1, false, 0, "" }
  , { "Industry Code", 3, RES_LOOKUP, false, LOOKUP_INDUSTRY_CODE, "" }
  , { "Reserved", BYTES(1), 1, false, 0, "" }
  , { "Proprietary ID", BYTES(1), RES_LOOKUP, false, "=25", "Init #1" }
  , { "Control", BYTES(1), RES_LOOKUP, false, ",0=Set,128=Ack", "" }
  , { 0 }
  }
}

,
{ "SonicHub: Position", 130816, true, 0x40, 0,
  { { "Manufacturer Code", 11, RES_MANUFACTURER, false, "=275", "Navico" }
  , { "Reserved", 2, 1, false, 0, "" }
  , { "Industry Code", 3, RES_LOOKUP, false, LOOKUP_INDUSTRY_CODE, "" }
  , { "Reserved", BYTES(1), 1, false, 0, "" }
  , { "Proprietary ID", BYTES(1), RES_LOOKUP, false, "=48", "Position" }
  , { "Control", BYTES(1), RES_LOOKUP, false, ",0=Set,128=Ack", "" }
  , { "Position", BYTES(4), 0.001, false, "s", "" }
  , { 0 }
  }
}

,
{ "SonicHub: Init #3", 130816, false, 9, 0,
  { { "Manufacturer Code", 11, RES_MANUFACTURER, false, "=275", "Navico" }
  , { "Reserved", 2, 1, false, 0, "" }
  , { "Industry Code", 3, RES_LOOKUP, false, LOOKUP_INDUSTRY_CODE, "" }
  , { "Reserved", BYTES(1), 1, false, 0, "" }
  , { "Proprietary ID", BYTES(1), RES_LOOKUP, false, "=50", "Init #3" }
  , { "Control", BYTES(1), RES_LOOKUP, false, ",0=Set,128=Ack", "" }
  , { "A", BYTES(1), RES_INTEGER, false, 0, "" }
  , { "B", BYTES(1), RES_INTEGER, false, 0, "" }
  , { 0 }
  }
}

,
{ "Simrad: Text Message", 130816, false, 0x40, 0,
  { { "Manufacturer Code", 11, RES_MANUFACTURER, false, 0, "" }
  , { "Reserved", 2, 1, false, 0, "" }
  , { "Industry Code", 3, RES_LOOKUP, false, LOOKUP_INDUSTRY_CODE, "" }
  , { "Reserved", BYTES(1), 1, false, 0, "" }
  , { "Proprietary ID", BYTES(1), RES_LOOKUP, false, "=50", "Init #3" }
  , { "A", BYTES(1), 1, false, 0, "" }
  , { "B", BYTES(1), 1, false, 0, "" }
  , { "C", BYTES(1), 1, false, 0, "" }
  , { "SID", BYTES(1), 1, false, 0, "" }
  , { "Prio", BYTES(1), 1, false, 0, "" }
  , { "Text", BYTES(32), RES_ASCII, false, 0, "" }
  , { 0 }
  }
}

,
{ "Navico: Product Information", 130817, false, 0x0e, 0,
  { { "Manufacturer Code", 11, RES_MANUFACTURER, false, 0, "" }
  , { "Reserved", 2, 1, false, 0, "" }
  , { "Industry Code", 3, RES_LOOKUP, false, LOOKUP_INDUSTRY_CODE, "" }
  , { "Product Code", BYTES(2), RES_INTEGER, false, 0, "" }
  , { "Model", BYTES(32), RES_ASCII, false, 0, "" }
  , { "A", BYTES(1), 1, false, 0, "" }
  , { "B", BYTES(1), 1, false, 0, "" }
  , { "C", BYTES(1), 1, false, 0, "" }
  , { "Firmware version", BYTES(10), RES_ASCII, false, 0, "" }
  , { "Firmware date", BYTES(32), RES_ASCII, false, 0, "" }
  , { "Firmware time", BYTES(32), RES_ASCII, false, 0, "" }
  , { 0 }
  }
}

,
{ "Simnet: Reprogram Data", 130818, false, 0x08, 0,
  { { "Manufacturer Code", 11, RES_MANUFACTURER, false, "=1857", "Simrad" }
  , { "Reserved", 2, 1, false, 0, "" }
  , { "Industry Code", 3, RES_LOOKUP, false, LOOKUP_INDUSTRY_CODE, "" }
  , { "Version", BYTES(2), RES_INTEGER, false, 0, "" }
  , { "Sequence", BYTES(2), RES_INTEGER, false, 0, "" }
  , { "Data", BYTES(255), RES_BINARY, false, 0, "" }
  , { 0 }
  }
}

,
{ "Simnet: Request Reprogram", 130819, false, 0x08, 0,
  { { "Manufacturer Code", 11, RES_MANUFACTURER, false, "=1857", "Simrad" }
  , { "Reserved", 2, 1, false, 0, "" }
  , { "Industry Code", 3, RES_LOOKUP, false, LOOKUP_INDUSTRY_CODE, "" }
  , { 0 }
  }
}

,
{ "Simnet: Reprogram Status", 130820, false, 0x08, 0,
  { { "Manufacturer Code", 11, RES_MANUFACTURER, false, "=1857", "Simrad" }
  , { "Reserved", 2, 1, false, 0, "" }
  , { "Industry Code", 3, RES_LOOKUP, false, LOOKUP_INDUSTRY_CODE, "" }
  , { "Reserved", BYTES(1), 1, false, 0, "" }
  , { "Status", BYTES(1), 1, false, 0, "" }
  , { "Reserved", BYTES(3), 1, false, 0, "" }
  , { 0 }
  }
}

/* M/V Dirona */
,
{ "Furuno: Unknown", 130820, false, 0x08, 0,
  { { "Manufacturer Code", 11, RES_MANUFACTURER, false, "=1855", "Furuno" }
  , { "Reserved", 2, 1, false, 0, "" }
  , { "Industry Code", 3, RES_LOOKUP, false, LOOKUP_INDUSTRY_CODE, "" }
  , { "A", BYTES(1), 1, false, 0, "" }
  , { "B", BYTES(1), 1, false, 0, "" }
  , { "C", BYTES(1), 1, false, 0, "" }
  , { "D", BYTES(1), 1, false, 0, "" }
  , { "E", BYTES(1), 1, false, 0, "" }
  , { 0 }
  }
}

/* Fusion */
,
{ "Fusion: Source Name", 130820, false, 13, 0,
  { { "Manufacturer Code", 11, RES_MANUFACTURER, false, "=419", "Fusion" }
  , { "Reserved", 2, 1, false, 0, "" }
  , { "Industry Code", 3, RES_LOOKUP, false, LOOKUP_INDUSTRY_CODE, "" }
  , { "Message ID", BYTES(1), 1, false, "=2", "" }
  , { "A", BYTES(1), 1, false, 0, "" }
  , { "B", BYTES(1), 1, false, 0, "" }
  , { "C", BYTES(1), 1, false, 0, "" }
  , { "D", BYTES(1), 1, false, 0, "" }
  , { "E", BYTES(1), 1, false, 0, "" }
  , { "Source", BYTES(5), RES_STRINGLZ, false, 0, "" }
  , { 0 }
  }
}

,
{ "Fusion: Track", 130820, false, 0x20, 0,
  { { "Manufacturer Code", 11, RES_MANUFACTURER, false, "=419", "Fusion" }
  , { "Reserved", 2, 1, false, 0, "" }
  , { "Industry Code", 3, RES_LOOKUP, false, LOOKUP_INDUSTRY_CODE, "" }
  , { "Message ID", BYTES(1), 1, false, "=5", "" }
  , { "A", BYTES(1), 1, false, 0, "" }
  , { "B", BYTES(5), 1, false, 0, "" }
  , { "Track", BYTES(10), RES_STRINGLZ, false, 0, "" }
  , { 0 }
  }
}

,
{ "Fusion: Artist", 130820, false, 0x20, 0,
  { { "Manufacturer Code", 11, RES_MANUFACTURER, false, "=419", "Fusion" }
  , { "Reserved", 2, 1, false, 0, "" }
  , { "Industry Code", 3, RES_LOOKUP, false, LOOKUP_INDUSTRY_CODE, "" }
  , { "Message ID", BYTES(1), 1, false, "=6", "" }
  , { "A", BYTES(1), 1, false, 0, "" }
  , { "B", BYTES(5), 1, false, 0, "" }
  , { "Artist", BYTES(10), RES_STRINGLZ, false, 0, "" }
  , { 0 }
  }
}

,
{ "Fusion: Album", 130820, false, 0x20, 0,
  { { "Manufacturer Code", 11, RES_MANUFACTURER, false, "=419", "Fusion" }
  , { "Reserved", 2, 1, false, 0, "" }
  , { "Industry Code", 3, RES_LOOKUP, false, LOOKUP_INDUSTRY_CODE, "" }
  , { "Message ID", BYTES(1), 1, false, "=7", "" }
  , { "A", BYTES(1), 1, false, 0, "" }
  , { "B", BYTES(5), 1, false, 0, "" }
  , { "Album", BYTES(10), RES_STRINGLZ, false, 0, "" }
  , { 0 }
  }
}

,
{ "Fusion: Play Progress", 130820, false, 9, 0,
  { { "Manufacturer Code", 11, RES_MANUFACTURER, false, "=419", "Fusion" }
  , { "Reserved", 2, 1, false, 0, "" }
  , { "Industry Code", 3, RES_LOOKUP, false, LOOKUP_INDUSTRY_CODE, "" }
  , { "Message ID", BYTES(1), 1, false, "=9", "" }
  , { "A", BYTES(1), 1, false, 0, "" }
  , { "B", BYTES(1), 1, false, 0, "" }
  , { "Progress", BYTES(3), 0.001, false, "s", "" }
  , { 0 }
  }
}

,
{ "Fusion: AM/FM Station", 130820, false, 0x0A, 0,
  { { "Manufacturer Code", 11, RES_MANUFACTURER, false, "=419", "Fusion" }
  , { "Reserved", 2, 1, false, 0, "" }
  , { "Industry Code", 3, RES_LOOKUP, false, LOOKUP_INDUSTRY_CODE, "" }
  , { "Message ID", BYTES(1), 1, false, "=11", "" }
  , { "A", BYTES(3), 1, false, 0, "" }
  , { "Frequency", BYTES(4), 1, false, "Hz", "" }
  , { "B", BYTES(1), 1, false, 0, "" }
  , { "Track", BYTES(10), RES_STRINGLZ, false, 0, "" }
  , { 0 }
  }
}

,
{ "Fusion: VHF", 130820, false, 9, 0,
  { { "Manufacturer Code", 11, RES_MANUFACTURER, false, "=419", "Fusion" }
  , { "Reserved", 2, 1, false, 0, "" }
  , { "Industry Code", 3, RES_LOOKUP, false, LOOKUP_INDUSTRY_CODE, "" }
  , { "Message ID", BYTES(1), 1, false, "=12", "" }
  , { "A", BYTES(1), 1, false, 0, "" }
  , { "B", BYTES(1), 1, false, 0, "" }
  , { "Channel", BYTES(1), 1, false, 0, "" }
  , { "D", BYTES(3), 1, false, 0, "" }
  , { 0 }
  }
}

,
{ "Fusion: Squelch", 130820, false, 6, 0,
  { { "Manufacturer Code", 11, RES_MANUFACTURER, false, "=419", "Fusion" }
  , { "Reserved", 2, 1, false, 0, "" }
  , { "Industry Code", 3, RES_LOOKUP, false, LOOKUP_INDUSTRY_CODE, "" }
  , { "Message ID", BYTES(1), 1, false, "=13", "" }
  , { "A", BYTES(1), 1, false, 0, "" }
  , { "B", BYTES(1), 1, false, 0, "" }
  , { "Squelch", BYTES(1), 1, false, 0, "" }
  , { 0 }
  }
}

,
{ "Fusion: Scan", 130820, false, 6, 0,
  { { "Manufacturer Code", 11, RES_MANUFACTURER, false, "=419", "Fusion" }
  , { "Reserved", 2, 1, false, 0, "" }
  , { "Industry Code", 3, RES_LOOKUP, false, LOOKUP_INDUSTRY_CODE, "" }
  , { "Message ID", BYTES(1), 1, false, "=14", "" }
  , { "A", BYTES(1), 1, false, 0, "" }
  , { "B", BYTES(1), 1, false, 0, "" }
  , { "Scan", BYTES(1), RES_LOOKUP, false, ",0=Off,1=Scan", "" }
  , { 0 }
  }
}

,
{ "Fusion: Menu Item", 130820, false, 23, 0,
  { { "Manufacturer Code", 11, RES_MANUFACTURER, false, "=419", "Fusion" }
  , { "Reserved", 2, 1, false, 0, "" }
  , { "Industry Code", 3, RES_LOOKUP, false, LOOKUP_INDUSTRY_CODE, "" }
  , { "Message ID", BYTES(1), 1, false, "=17", "" }
  , { "A", BYTES(1), 1, false, 0, "" }
  , { "B", BYTES(1), 1, false, 0, "" }
  , { "Line", BYTES(1), 1, false, 0, "" }
  , { "E", BYTES(1), 1, false, 0, "" }
  , { "F", BYTES(1), 1, false, 0, "" }
  , { "G", BYTES(1), 1, false, 0, "" }
  , { "H", BYTES(1), 1, false, 0, "" }
  , { "I", BYTES(1), 1, false, 0, "" }
  , { "Text", BYTES(5), RES_STRINGLZ, false, 0, "" }
  , { 0 }
  }
}

,
{ "Fusion: Replay", 130820, false, 23, 0,
  { { "Manufacturer Code", 11, RES_MANUFACTURER, false, "=419", "Fusion" }
  , { "Reserved", 2, 1, false, 0, "" }
  , { "Industry Code", 3, RES_LOOKUP, false, LOOKUP_INDUSTRY_CODE, "" }
  , { "Message ID", BYTES(1), 1, false, "=20", "" }
  , { "A", BYTES(1), 1, false, 0, "" }
  , { "Mode", BYTES(1), RES_LOOKUP, false, ",9=USB Repeat,10=USB Shuffle,12=iPod Repeat,13=iPod Shuffle", "" }
  , { "C", BYTES(3), 1, false, 0, "" }
  , { "D", BYTES(1), 1, false, 0, "" }
  , { "E", BYTES(1), 1, false, 0, "" }
  , { "Status", BYTES(1), RES_LOOKUP, false, ",0=Off,1=One/Track,2=All/Album", "" }
  , { "H", BYTES(1), 1, false, 0, "" }
  , { "I", BYTES(1), 1, false, 0, "" }
  , { "J", BYTES(1), 1, false, 0, "" }
  , { 0 }
  }
}

,
{ "Fusion: Mute", 130820, false, 5, 0,
  { { "Manufacturer Code", 11, RES_MANUFACTURER, false, "=419", "Fusion" }
  , { "Reserved", 2, 1, false, 0, "" }
  , { "Industry Code", 3, RES_LOOKUP, false, LOOKUP_INDUSTRY_CODE, "" }
  , { "Message ID", BYTES(1), 1, false, "=23", "" }
  , { "A", BYTES(1), 1, false, 0, "" }
  , { "Mute", BYTES(1), RES_LOOKUP, false, ",1=Muted,2=Not Muted", "" }
  , { 0 }
  }
}

,
// Range: 0 to +24
{ "Fusion: Sub Volume", 130820, false, 8, 0,
  { { "Manufacturer Code", 11, RES_MANUFACTURER, false, "=419", "Fusion" }
  , { "Reserved", 2, 1, false, 0, "" }
  , { "Industry Code", 3, RES_LOOKUP, false, LOOKUP_INDUSTRY_CODE, "" }
  , { "Message ID", BYTES(1), 1, false, "=26", "" }
  , { "A", BYTES(1), 1, false, 0, "" }
  , { "Zone 1", BYTES(1), 1, false, "vol", "" }
  , { "Zone 2", BYTES(1), 1, false, "vol", "" }
  , { "Zone 3", BYTES(1), 1, false, "vol", "" }
  , { "Zone 4", BYTES(1), 1, false, "vol", "" }
  , { 0 }
  }
}

,
// Range: -15 to +15
{ "Fusion: Tone", 130820, false, 8, 0,
  { { "Manufacturer Code", 11, RES_MANUFACTURER, false, "=419", "Fusion" }
  , { "Reserved", 2, 1, false, 0, "" }
  , { "Industry Code", 3, RES_LOOKUP, false, LOOKUP_INDUSTRY_CODE, "" }
  , { "Message ID", BYTES(1), 1, false, "=27", "" }
  , { "A", BYTES(1), 1, false, 0, "" }
  , { "B", BYTES(1), 1, false, 0, "" }
  , { "Bass", BYTES(1), 1, true, "vol", "" }
  , { "Mid", BYTES(1), 1, true, "vol", "" }
  , { "Treble", BYTES(1), 1, true, "vol", "" }
  , { 0 }
  }
}

,
{ "Fusion: Volume", 130820, false, 0x0A, 0,
  { { "Manufacturer Code", 11, RES_MANUFACTURER, false, "=419", "Fusion" }
  , { "Reserved", 2, 1, false, 0, "" }
  , { "Industry Code", 3, RES_LOOKUP, false, LOOKUP_INDUSTRY_CODE, "" }
  , { "Message ID", BYTES(1), 1, false, "=29", "" }
  , { "A", BYTES(1), 1, false, 0, "" }
  , { "Zone 1", BYTES(1), 1, false, "vol", "" }
  , { "Zone 2", BYTES(1), 1, false, "vol", "" }
  , { "Zone 3", BYTES(1), 1, false, "vol", "" }
  , { "Zone 4", BYTES(1), 1, false, "vol", "" }
  , { 0 }
  }
}

,
{ "Fusion: Transport", 130820, false, 5, 0,
  { { "Manufacturer Code", 11, RES_MANUFACTURER, false, "=419", "Fusion" }
  , { "Reserved", 2, 1, false, 0, "" }
  , { "Industry Code", 3, RES_LOOKUP, false, LOOKUP_INDUSTRY_CODE, "" }
  , { "Message ID", BYTES(1), 1, false, "=32", "" }
  , { "A", BYTES(1), 1, false, 0, "" }
  , { "Transport", BYTES(1), RES_LOOKUP, false, ",1=Paused", "" }
  , { 0 }
  }
}

/* M/V Dirona */
,
{ "Furuno: Unknown", 130821, false, 0x0c, 0,
  { { "Manufacturer Code", 11, RES_MANUFACTURER, false, "=1855", "Furuno" }
  , { "Reserved", 2, 1, false, 0, "" }
  , { "Industry Code", 3, RES_LOOKUP, false, LOOKUP_INDUSTRY_CODE, "" }
  , { "SID", BYTES(1), 1, false, 0, "" }
  , { "A", BYTES(1), 1, false, 0, "" }
  , { "B", BYTES(1), 1, false, 0, "" }
  , { "C", BYTES(1), 1, false, 0, "" }
  , { "D", BYTES(1), 1, false, 0, "" }
  , { "E", BYTES(1), 1, false, 0, "" }
  , { "F", BYTES(1), 1, false, 0, "" }
  , { "G", BYTES(1), 1, false, 0, "" }
  , { "H", BYTES(1), 1, false, 0, "" }
  , { "I", BYTES(1), 1, false, 0, "" }
  , { 0 }
  }
}

/* Uwe Lovas has seen this from EP-70R */
,
{ "Lowrance: unknown", 130827, false, 10, 0,
  { { "Manufacturer Code", 11, RES_MANUFACTURER, false, "=140", "Lowrance" }
  , { "Reserved", 2, 1, false, 0, "" }
  , { "Industry Code", 3, RES_LOOKUP, false, LOOKUP_INDUSTRY_CODE, "" }
  , { "A", BYTES(1), 1, false, 0, "" }
  , { "B", BYTES(1), 1, false, 0, "" }
  , { "C", BYTES(1), 1, false, 0, "" }
  , { "D", BYTES(1), 1, false, 0, "" }
  , { "E", BYTES(2), 1, false, 0, "" }
  , { "F", BYTES(2), 1, false, 0, "" }
  , { 0 }
  }
}

,
{ "Simnet: Set Serial Number", 130828, false, 0x08, 0,
  { { "Manufacturer Code", 11, RES_MANUFACTURER, false, "=1857", "Simrad" }
  , { "Reserved", 2, 1, false, 0, "" }
  , { "Industry Code", 3, RES_LOOKUP, false, LOOKUP_INDUSTRY_CODE, "" }
  , { 0 }
  }
}

,
{ "Suzuki: Engine and Storage Device Config", 130831, false, 0x08, 0,
  { { "Manufacturer Code", 11, RES_MANUFACTURER, false, 0, "" }
  , { "Reserved", 2, 1, false, 0, "" }
  , { "Industry Code", 3, RES_LOOKUP, false, LOOKUP_INDUSTRY_CODE, "" }
  , { 0 }
  }
}

,
{ "Simnet: Fuel Used - High Resolution", 130832, false, 0x08, 0,
  { { "Manufacturer Code", 11, RES_MANUFACTURER, false, "=1857", "Simrad" }
  , { "Reserved", 2, 1, false, 0, "" }
  , { "Industry Code", 3, RES_LOOKUP, false, LOOKUP_INDUSTRY_CODE, "" }
  , { 0 }
  }
}

,
{ "Simnet: Engine and Tank Configuration", 130834, false, 0x08, 0,
  { { "Manufacturer Code", 11, RES_MANUFACTURER, false, "=1857", "Simrad" }
  , { "Reserved", 2, 1, false, 0, "" }
  , { "Industry Code", 3, RES_LOOKUP, false, LOOKUP_INDUSTRY_CODE, "" }
  , { 0 }
  }
}

,
{ "Simnet: Set Engine and Tank Configuration", 130835, false, 0x08, 0,
  { { "Manufacturer Code", 11, RES_MANUFACTURER, false, "=1857", "Simrad" }
  , { "Reserved", 2, 1, false, 0, "" }
  , { "Industry Code", 3, RES_LOOKUP, false, LOOKUP_INDUSTRY_CODE, "" }
  , { 0 }
  }
}

  /* Seen when HDS8 configures EP65R */
,
{ "Simnet: Fluid Level Sensor Configuration", 130836, false, 0x0e, 0,
  { { "Manufacturer Code", 11, RES_MANUFACTURER, false, 0, "" }
  , { "Reserved", 2, 1, false, 0, "" }
  , { "Industry Code", 3, RES_LOOKUP, false, LOOKUP_INDUSTRY_CODE, "" }
  , { "C", BYTES(1), 1, false, 0, "" }
  , { "Device", BYTES(1), RES_INTEGER, false, 0, "" }
  , { "Instance", BYTES(1), 1, false, 0, "" }
  , { "F", 1 * 4, 1, false, 0, "" }
  , { "Tank type", 1 * 4, RES_LOOKUP, false, ",0=Fuel,1=Water,2=Gray water,3=Live well,4=Oil,5=Black water", "" }
  , { "Capacity", BYTES(4), 0.1, false, 0, "" }
  , { "G", BYTES(1), 1, false, 0, "" }
  , { "H", BYTES(2), 1, true, 0, "" }
  , { "I", BYTES(1), 1, true, 0, "" }
  , { 0 }
  }
}

,
{ "Simnet: Fuel Flow Turbine Configuration", 130837, false, 0x08, 0,
  { { "Manufacturer Code", 11, RES_MANUFACTURER, false, "=1857", "Simrad" }
  , { "Reserved", 2, 1, false, 0, "" }
  , { "Industry Code", 3, RES_LOOKUP, false, LOOKUP_INDUSTRY_CODE, "" }
  , { 0 }
  }
}

,
{ "Simnet: Fluid Level Warning", 130838, false, 0x08, 0,
  { { "Manufacturer Code", 11, RES_MANUFACTURER, false, "=1857", "Simrad" }
  , { "Reserved", 2, 1, false, 0, "" }
  , { "Industry Code", 3, RES_LOOKUP, false, LOOKUP_INDUSTRY_CODE, "" }
  , { 0 }
  }
}

,
{ "Simnet: Pressure Sensor Configuration", 130839, false, 0x08, 0,
  { { "Manufacturer Code", 11, RES_MANUFACTURER, false, "=1857", "Simrad" }
  , { "Reserved", 2, 1, false, 0, "" }
  , { "Industry Code", 3, RES_LOOKUP, false, LOOKUP_INDUSTRY_CODE, "" }
  , { 0 }
  }
}

,
{ "Simnet: Data User Group Configuration", 130840, false, 0x08, 0,
  { { "Manufacturer Code", 11, RES_MANUFACTURER, false, "=1857", "Simrad" }
  , { "Reserved", 2, 1, false, 0, "" }
  , { "Industry Code", 3, RES_LOOKUP, false, LOOKUP_INDUSTRY_CODE, "" }
  , { 0 }
  }
}

/* Where did this come from ?
,
{ "Simnet: DSC Message", 130842, false, 0x08, 0,
  { { "Manufacturer Code", 11, RES_MANUFACTURER, false, "=1857", "Simrad" }
  , { "Reserved", 2, 1, false, 0, "" }
  , { "Industry Code", 3, RES_LOOKUP, false, LOOKUP_INDUSTRY_CODE, "" }
  , { 0 }
  }
}
*/

,
{ "Simnet: AIS Class B static data (msg 24 Part A)", 130842, false, 0x1d, 0,
  { { "Manufacturer Code", 11, RES_MANUFACTURER, false, "=1857", "Simrad" }
  , { "Reserved", 2, 1, false, 0, "" }
  , { "Industry Code", 3, RES_LOOKUP, false, LOOKUP_INDUSTRY_CODE, "" }
  , { "Message ID", 6, 1, false, "=0", "Msg 24 Part A" }
  , { "Repeat indicator", 2, RES_LOOKUP, false, LOOKUP_REPEAT_INDICATOR, "" }
  , { "D", BYTES(1), 1, false, 0, "" }
  , { "E", BYTES(1), 1, false, 0, "" }
  , { "User ID", BYTES(4), RES_INTEGER, false, "MMSI", "" }
  , { "Name", BYTES(20), RES_ASCII, false, 0, "" }
  , { 0 }
  }
}

,
{ "Simnet: AIS Class B static data (msg 24 Part B)", 130842, false, 0x25, 0,
  { { "Manufacturer Code", 11, RES_MANUFACTURER, false, "=1857", "Simrad" }
  , { "Reserved", 2, 1, false, 0, "" }
  , { "Industry Code", 3, RES_LOOKUP, false, LOOKUP_INDUSTRY_CODE, "" }
  , { "Message ID", 6, 1, false, "=1", "Msg 24 Part B" }
  , { "Repeat indicator", 2, RES_LOOKUP, false, LOOKUP_REPEAT_INDICATOR, "" }
  , { "D", BYTES(1), 1, false, 0, "" }
  , { "E", BYTES(1), 1, false, 0, "" }
  , { "User ID", BYTES(4), RES_INTEGER, false, "MMSI", "" }
  , { "Type of ship", BYTES(1), RES_LOOKUP, false, LOOKUP_SHIP_TYPE, "" }
  , { "Vendor ID", BYTES(7), RES_ASCII, false, 0, "" }
  , { "Callsign", BYTES(7), RES_ASCII, false, 0, "0=unavailable" }
  , { "Length", BYTES(2), 0.1, false, "m", "" }
  , { "Beam", BYTES(2), 0.1, false, "m", "" }
  , { "Position reference from Starboard", BYTES(2), 0.1, false, "m", "" }
  , { "Position reference from Bow", BYTES(2), 0.1, false, "m", "" }
  , { "Mothership User ID", BYTES(4), RES_INTEGER, false, "MMSI", "Id of mother ship sent by daughter vessels" }
  , { "", 2, 1, false, 0, "" }
  , { "Spare", 6, RES_INTEGER, false, 0, "0=unavailable" }
  , { 0 }
  }
}

,
{ "Simnet: Sonar Status, Frequency and DSP Voltage", 130843, false, 0x08, 0,
  { { "Manufacturer Code", 11, RES_MANUFACTURER, false, "=1857", "Simrad" }
  , { "Reserved", 2, 1, false, 0, "" }
  , { "Industry Code", 3, RES_LOOKUP, false, LOOKUP_INDUSTRY_CODE, "" }
  , { 0 }
  }
}

,
{ "Simnet: Parameter Handle", 130845, false, 0x0e, 0,
  { { "Manufacturer Code", 11, RES_MANUFACTURER, false, "=1857", "Simrad" }
  , { "Reserved", 2, 1, false, 0, "" }
  , { "Industry Code", 3, RES_LOOKUP, false, LOOKUP_INDUSTRY_CODE, "" }
  , { "Message ID", 6, 1, false, 0, "" }
  , { "Repeat indicator", 2, RES_LOOKUP, false, LOOKUP_REPEAT_INDICATOR, "" }
  , { "D", BYTES(1), 1, false, 0, "" }
  , { "Group", BYTES(1), 1, false, 0, "" }
  , { "F", BYTES(1), 1, false, 0, "" }
  , { "G", BYTES(1), 1, false, 0, "" }
  , { "H", BYTES(1), 1, false, 0, "" }
  , { "I", BYTES(1), 1, false, 0, "" }
  , { "J", BYTES(1), 1, false, 0, "" }
  , { "Backlight", BYTES(1), RES_LOOKUP, false, ",1=Day Mode,4=Night Mode,11=Level 1,22=Level 2,33=Level 3,44=Level 4,55=Level 5,66=Level 6,77=Level 7,88=Level 8,99=Level 9", "" }
  , { "L", BYTES(2), 1, false, 0, "" }
  , { 0 }
  }
}

,
{ "SeaTalk: Node Statistics", 130847, false, 0, 0,
  { { "Manufacturer Code", 11, RES_MANUFACTURER, false, "=1851", "Raymarine" }
  , { "Software Release", BYTES(2), 1, false, 0, "" }
  , { "Development Version", BYTES(1), 1, false, 0, "" }
  , { "Product Code", BYTES(2), 1, false, 0, "" }
  , { "Year", BYTES(1), 1, false, 0, "" }
  , { "Month", BYTES(1), 1, false, 0, "" }
  , { "Device Number", BYTES(2), 1, false, 0, "" }
  , { "Node Voltage", BYTES(2), 0.01, false, "V", "" }
  , { 0 }
  }
}

#define LOOKUP_SIMNET_AP_EVENTS ( \
          ",6=Standby" \
          ",9=Auto mode" \
          ",10=Nav mode" \
          ",13=Non Follow Up mode" \
          ",15=Wind mode" \
          ",26=Change Course" \
          )

#define LOOKUP_SIMNET_DIRECTION ( \
          ",2=Port" \
          ",3=Starboard" \
          ",4=Left rudder (port)" \
          ",5=Right rudder (starboard)" \
          )

,
{ "Simnet: Event Command: AP command", 130850, false, 12, 0,
  { { "Manufacturer Code", 11, RES_MANUFACTURER, false, "=1857", "Simrad" }
  , { "Reserved", 2, 1, false, 0, "" }
  , { "Industry Code", 3, RES_LOOKUP, false, LOOKUP_INDUSTRY_CODE, "" }
  , { "Proprietary ID", BYTES(1), RES_LOOKUP, false, "=2", "AP command" }
  , { "B", BYTES(2), 1, false, 0, "" }
  , { "Controlling Device", BYTES(1), 1, false, 0, "" }
  , { "Event", BYTES(2), RES_LOOKUP, false, LOOKUP_SIMNET_AP_EVENTS, "" }
  , { "Direction", BYTES(1), RES_LOOKUP, false, LOOKUP_SIMNET_DIRECTION, "" }
  , { "Angle", BYTES(2), RES_DEGREES, false, "deg", "" }
  , { "G", BYTES(1), 1, false, 0, "" }
  , { 0 }
  }
}

,
{ "Simnet: Event Command: Alarm?", 130850, false, 12, 0,
  { { "Manufacturer Code", 11, RES_MANUFACTURER, false, "=1857", "Simrad" }
  , { "Reserved", 2, 1, false, 0, "" }
  , { "Industry Code", 3, RES_LOOKUP, false, LOOKUP_INDUSTRY_CODE, "" }
  , { "A", BYTES(2), 1, false, 0, "" }
  , { "Proprietary ID", BYTES(1), RES_LOOKUP, false, "=1", "Alarm command" }
  , { "C", BYTES(1), 1, false, 0, "" }
  , { "Alarm", BYTES(2), RES_LOOKUP, false, ",57=Raise,56=Clear", "" }
  , { "Message ID", BYTES(2), RES_INTEGER, false, 0, "" }
  , { "F", BYTES(1), 1, false, 0, "" }
  , { "G", BYTES(1), 1, false, 0, "" }
  , { 0 }
  }
}

,
{ "Simnet: Event Command: Unknown", 130850, false, 12, 0,
  { { "Manufacturer Code", 11, RES_MANUFACTURER, false, "=1857", "Simrad" }
  , { "Reserved", 2, 1, false, 0, "" }
  , { "Industry Code", 3, RES_LOOKUP, false, LOOKUP_INDUSTRY_CODE, "" }
  , { "A", BYTES(2), 1, false, 0, "" }
  , { "Proprietary ID", BYTES(1), RES_LOOKUP, false, "=1", "Alarm command" }
  , { "B", BYTES(1), 1, false, 0, "" }
  , { "C", BYTES(2), 1, false, 0, "" }
  , { "D", BYTES(2), 1, false, 0, "" }
  , { "E", BYTES(2), 1, false, 0, "" }
  , { 0 }
  }
}

,
{ "Simnet: Event Reply: AP command", 130851, false, 12, 0,
  { { "Manufacturer Code", 11, RES_MANUFACTURER, false, "=1857", "Simrad" }
  , { "Reserved", 2, 1, false, 0, "" }
  , { "Industry Code", 3, RES_LOOKUP, false, LOOKUP_INDUSTRY_CODE, "" }
  , { "Proprietary ID", BYTES(1), RES_LOOKUP, false, "=2", "AP command" }
  , { "B", BYTES(2), 1, false, 0, "" }
  , { "Controlling Device", BYTES(1), 1, false, 0, "" }
  , { "Event", BYTES(2), RES_LOOKUP, false, LOOKUP_SIMNET_AP_EVENTS, "" }
  , { "Direction", BYTES(1), RES_LOOKUP, false, LOOKUP_SIMNET_DIRECTION, "" }
  , { "Angle", BYTES(2), RES_DEGREES, false, "deg", "" }
  , { "G", BYTES(1), 1, false, 0, "" }
  , { 0 }
  }
}

,
{ "Simnet: Alarm Message", 130856, false, 0x08, 0,
  { { "Manufacturer Code", 11, RES_MANUFACTURER, false, "=1857", "Simrad" }
  , { "Reserved", 2, 1, false, 0, "" }
  , { "Industry Code", 3, RES_LOOKUP, false, LOOKUP_INDUSTRY_CODE, "" }
  , { "Message ID", BYTES(2), 1, false, 0, "" }
  , { "B", BYTES(1), 1, false, 0, "" }
  , { "C", BYTES(1), 1, false, 0, "" }
  , { "Text", BYTES(255), RES_ASCII, false, 0, "" }
  , { 0 }
  }
}

,
{ "Airmar: Additional Weather Data", 130880, false, 0x1e, 0,
  { { "Manufacturer Code", 11, RES_MANUFACTURER, false, 0, "" }
  , { "Reserved", 2, 1, false, 0, "" }
  , { "Industry Code", 3, RES_LOOKUP, false, LOOKUP_INDUSTRY_CODE, "" }
  , { "C", BYTES(1), 1, false, 0, "" }
  , { "Apparent Windchill Temperature", BYTES(2), RES_TEMPERATURE, false, "K", "" }
  , { "True Windchill Temperature", BYTES(2), RES_TEMPERATURE, false, "K", "" }
  , { "Dewpoint", BYTES(2), RES_TEMPERATURE, false, "K", "" }
  , { 0 }
  }
}

,
{ "Airmar: Heater Control", 130881, false, 0x9, 0,
  { { "Manufacturer Code", 11, RES_MANUFACTURER, false, 0, "" }
  , { "Reserved", 2, 1, false, 0, "" }
  , { "Industry Code", 3, RES_LOOKUP, false, LOOKUP_INDUSTRY_CODE, "" }
  , { "C", BYTES(1), 1, false, 0, "" }
  , { "Plate Temperature", BYTES(2), RES_TEMPERATURE, false, "K", "" }
  , { "Air Temperature", BYTES(2), RES_TEMPERATURE, false, "K", "" }
  , { "Dewpoint", BYTES(2), RES_TEMPERATURE, false, "K", "" }
  , { 0 }
  }
}

,
{ "Airmar: POST", 130944, false, 0x8, 0,
  { { "Manufacturer Code", 11, RES_MANUFACTURER, false, 0, "" }
  , { "Reserved", 2, 1, false, 0, "" }
  , { "Industry Code", 3, RES_LOOKUP, false, LOOKUP_INDUSTRY_CODE, "" }
  , { "Control", 4, RES_LOOKUP, false, ",0=Report previous values,1=Generate new values", "" }
  , { "Reserved", 7, 1, false, 0, "" }
  , { "Number of ID/test result pairs to follow", BYTES(1), RES_INTEGER, false, 0, "" }

  , { "Test ID", BYTES(1), RES_LOOKUP, false, ",1=Format Code,2=Factory EEPROM,3=User EEPROM,4=Water Temp Sensor,5=Sonar Transceiver,6=Speed sensor,7=Internal temperature sensor,8=Battery voltage sensor", "See Airmar docs for table of IDs and failure codes; these lookup values are for DST200" }
  , { "Test result", BYTES(1), RES_LOOKUP, false, ",0=Pass", "Values other than 0 are failure codes" }
  , { 0 }
  }
}

,
{ "Actisense: Operating mode", ACTISENSE_BEM + 0x11, false, 0x0e, 0,
  { { "SID", BYTES(1), 1, false, 0, "" }
  , { "Model ID", BYTES(2), RES_INTEGER, false, 0, "" }
  , { "Serial ID", BYTES(4), RES_INTEGER, false, 0, "" }
  , { "Error ID", BYTES(4), RES_INTEGER, false, 0, "" }
  , { "Operating Mode", BYTES(2), 1, false, 0, "" }
  , { 0 }
  }
}

,
{ "Actisense: Startup status", ACTISENSE_BEM + 0xf0, false, 0x0f, 0,
  { { "SID", BYTES(1), 1, false, 0, "" }
  , { "Model ID", BYTES(2), RES_INTEGER, false, 0, "" }
  , { "Serial ID", BYTES(4), RES_INTEGER, false, 0, "" }
  , { "Error ID", BYTES(4), RES_INTEGER, false, 0, "" }
  , { "Firmware version", BYTES(2), 0.001, false, 0, "" }
  , { "Reset status", BYTES(1), 1, false, 0, "" }
  , { "A", BYTES(1), 1, false, 0, "" }
  , { 0 }
  }
}

,
{ "Actisense: System status", ACTISENSE_BEM + 0xf2, false, 0x22, 0,
  { { "SID", BYTES(1), 1, false, 0, "" }
  , { "Model ID", BYTES(2), RES_INTEGER, false, 0, "" }
  , { "Serial ID", BYTES(4), RES_INTEGER, false, 0, "" }
  , { "Error ID", BYTES(4), RES_INTEGER, false, 0, "" }
  , { "Indi channel count", BYTES(1), 1, false, 0, "" }
  , { "Ch1 Rx Bandwidth", BYTES(1), 1, false, 0, "" }
  , { "Ch1 Rx Load", BYTES(1), 1, false, 0, "" }
  , { "Ch1 Rx Filtered", BYTES(1), 1, false, 0, "" }
  , { "Ch1 Rx Dropped", BYTES(1), 1, false, 0, "" }
  , { "Ch1 Tx Bandwidth", BYTES(1), 1, false, 0, "" }
  , { "Ch1 Tx Load", BYTES(1), 1, false, 0, "" }
  , { "Ch2 Rx Bandwidth", BYTES(1), 1, false, 0, "" }
  , { "Ch2 Rx Load", BYTES(1), 1, false, 0, "" }
  , { "Ch2 Rx Filtered", BYTES(1), 1, false, 0, "" }
  , { "Ch2 Rx Dropped", BYTES(1), 1, false, 0, "" }
  , { "Ch2 Tx Bandwidth", BYTES(1), 1, false, 0, "" }
  , { "Ch2 Tx Load", BYTES(1), 1, false, 0, "" }
  , { "Uni channel count", BYTES(1), 1, false, 0, "" }
  , { "Ch1 Bandwidth", BYTES(1), 1, false, 0, "" }
  , { "Ch1 Deleted", BYTES(1), 1, false, 0, "" }
  , { "Ch1 BufferLoading", BYTES(1), 1, false, 0, "" }
  , { "Ch1 PointerLoading", BYTES(1), 1, false, 0, "" }
  , { "Ch2 Bandwidth", BYTES(1), 1, false, 0, "" }
  , { "Ch2 Deleted", BYTES(1), 1, false, 0, "" }
  , { "Ch2 BufferLoading", BYTES(1), 1, false, 0, "" }
  , { "Ch2 PointerLoading", BYTES(1), 1, false, 0, "" }
  , { 0 }
  }
}

,
{ "Actisense: ?", ACTISENSE_BEM + 0xf4, false, 17, 0,
  { { "SID", BYTES(1), 1, false, 0, "" }
  , { "Model ID", BYTES(2), RES_INTEGER, false, 0, "" }
  , { "Serial ID", BYTES(4), RES_INTEGER, false, 0, "" }
  , { 0 }
  }
}

}
;

typedef struct
{
  char * name;
  size_t id;
} Company;

/* http://www.nmea.org/Assets/20121212%20nmea%202000%20registration%20list.pdf */
Company companyList[] =
{ { "Volvo Penta", 174 }
, { "Actia Corporation", 199 }
, { "Actisense", 273 }
, { "Aetna Engineering/Fireboy-Xintex", 215 }
, { "Airmar", 135 }
, { "Alltek", 459 }
, { "B&G", 381 }
, { "Beede Electrical", 185 }
, { "BEP", 295 }
, { "Beyond Measure", 396 }
, { "Blue Water Data", 148 }
, { "Evinrude/Bombardier" , 163 }
, { "Camano Light", 384 }
, { "Capi 2", 394 }
, { "Carling", 176 }
, { "CPac", 165 }
, { "Coelmo", 286 }
, { "ComNav", 404 }
, { "Cummins", 440 }
, { "Dief", 329 }
, { "Digital Yacht", 437 }
, { "Disenos Y Technologia", 201 }
, { "DNA Group", 211 }
, { "Egersund Marine", 426 }
, { "Electronic Design", 373 }
, { "Em-Trak", 427 }
, { "EMMI Network", 224 }
, { "Empirbus", 304 }
, { "eRide", 243 }
, { "Faria Instruments", 1863 }
, { "Fischer Panda", 356 }
, { "Floscan", 192 }
, { "Furuno", 1855 }
, { "Fusion", 419 }
, { "FW Murphy", 78 }
, { "Garmin", 229 }
, { "Geonav", 385 }
, { "Glendinning", 378 }
, { "GME / Standard", 475 }
, { "Groco", 272 }
, { "Hamilton Jet", 283 }
, { "Hemisphere GPS", 88 }
, { "Honda", 257 }
, { "Hummingbird", 467 }
, { "ICOM", 315 }
, { "JRC", 1853 }
, { "Kvasar", 1859 }
, { "Kohler", 85 }
, { "Korea Maritime University", 345 }
, { "LCJ Capteurs", 499 }
, { "Litton", 1858 }
, { "Livorsi", 400 }
, { "Lowrance", 140 }
, { "Maretron", 137 }
, { "MBW", 307 }
, { "Mastervolt", 355 }
, { "Mercury", 144 }
, { "MMP", 1860 }
, { "Mystic Valley Comms", 198 }
, { "Nautibus", 147 }
, { "Navico", 275 }
, { "Navionics", 1852 }
, { "Naviop", 503 }
, { "Nobeltec", 193 }
, { "Noland", 517 }
, { "Northern Lights", 374 }
, { "Northstar", 1854 }
, { "Novatel", 305 }
, { "Offshore Systems", 161 }
, { "Qwerty", 328 }
, { "Parker Hannifin", 451 }
, { "Raymarine", 1851 }
, { "Rolls Royce", 370 }
, { "SailorMade/Tetra", 235 }
, { "San Giorgio", 460 }
, { "Sea Cross", 471 }
, { "Sea Recovery", 285 }
, { "Yamaha", 1862 }
, { "Simrad", 1857 }
, { "Sitex", 470 }
, { "Sleipner", 306 }
, { "Teleflex", 1850 }
, { "Thrane and Thrane", 351 }
, { "Tohatsu", 431 }
, { "Transas", 518 }
, { "Trimble", 1856 }
, { "True Heading", 422 }
, { "Twin Disc", 80 }
, { "Vector Cantech", 1861 }
, { "Veethree", 466 }
, { "Vertex", 421 }
, { "Vesper", 504 }
, { "Victron", 358 }
, { "Watcheye", 493 }
, { "Westerbeke", 154 }
, { "Xantrex", 168 }
, { "Yacht Monitoring Solutions", 233 }
, { "Yanmar", 172 }
, { "ZF", 228 }
};

