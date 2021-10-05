/*

Analyzes NMEA 2000 PGNs.

(C) 2009-2021, Kees Verruijt, Harlingen, The Netherlands.

This file is part of CANboat.

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

    http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.

*/

#include "common.h"

#define RES_LAT_LONG_PRECISION (10000000) /* 1e7 */
#define RES_LAT_LONG (1.0e-7)
#define RES_LAT_LONG_64 (1.0e-16)
#define RES_PERCENTAGE (100.0 / 25000.0)

typedef struct
{
  char *   name;
  uint32_t size; /* Size in bits. All fields are contiguous in message; use 'reserved' fields to fill in empty bits. */
#define LEN_VARIABLE (0)
  double resolution; /* Either a positive real value or one of the following RES_ special values */
#define RES_NOTUSED (0)
#define RES_RADIANS (1e-4)
#define RES_ROTATION (1e-3 / 32.0)
#define RES_HIRES_ROTATION (1e-6 / 32.0)
#define RES_ASCII (-1.0)
#define RES_LATITUDE (-2.0)
#define RES_LONGITUDE (-3.0)
#define RES_DATE (-4.0)
#define RES_TIME (-5.0)
#define RES_TEMPERATURE (-6.0)
#define RES_6BITASCII (-7.0) /* Actually not used in N2K, only in N183 AIS */
#define RES_INTEGER (-8.0)
#define RES_LOOKUP (-9.0)
#define RES_BINARY (-10.0)
#define RES_MANUFACTURER (-11.0)
#define RES_STRING (-12.0)
#define RES_FLOAT (-13.0)
#define RES_PRESSURE (-14.0)
#define RES_STRINGLZ (-15.0) /* ASCII string starting with length byte and terminated by zero byte */
#define RES_STRINGLAU (-16.0) /* ASCII or UNICODE string starting with length byte and ASCII/Unicode byte */
#define RES_DECIMAL (-17.0)
#define RES_BITFIELD (-18.0)
#define RES_TEMPERATURE_HIGH (-19.0)
#define RES_TEMPERATURE_HIRES (-20.0)
#define RES_PRESSURE_HIRES (-21.0)
#define RES_VARIABLE (-22.0)
#define MAX_RESOLUTION_LOOKUP 22

  bool  hasSign; /* Is the value signed, e.g. has both positive and negative values? */
  char *units;   /* String containing the 'Dimension' (e.g. s, h, m/s, etc.) unless it starts with , in which
                  * case it contains a set of lookup values.
                  */
  char *  description;
  int32_t offset;  /* Only used for SAE J1939 values with sign; these are in Offset/Excess-K notation instead
                    * of two's complement as used by NMEA 2000.
                    * See http://en.wikipedia.org/wiki/Offset_binary
                    */
  char *camelName; /* Filled by C, no need to set in initializers. */
} Field;

typedef struct
{
  const char *name;
  const char *resolution;
} Resolution;

static const Resolution types[MAX_RESOLUTION_LOOKUP] = {{"ASCII text", 0},
                                                        {"Latitude", 0},
                                                        {"Longitude", 0},
                                                        {"Date", "1"},
                                                        {"Time", "0.0001"},
                                                        {"Temperature", "0.01"},
                                                        {"6 Bit ASCII text", 0},
                                                        {"Integer", "1"},
                                                        {"Lookup table", 0},
                                                        {"Binary data", 0},
                                                        {"Manufacturer code", 0},
                                                        {"String with start/stop byte", 0},
                                                        {"IEEE Float", 0},
                                                        {"Pressure", 0},
                                                        {"ASCII string starting with length byte", 0},
                                                        {"ASCII or UNICODE string starting with length and control byte", 0},
                                                        {"Decimal encoded number", 0},
                                                        {"Bitfield", 0},
                                                        {"Temperature", "0.1"},
                                                        {"Temperature (hires)", "0.001"},
                                                        {"Pressure (hires)", "0.1"}};

#define LOOKUP_INDUSTRY_CODE (",0=Global,1=Highway,2=Agriculture,3=Construction,4=Marine,5=Industrial")

#define LOOKUP_SHIP_TYPE                                                                                                           \
  (",0=unavailable"                                                                                                                \
   ",20=Wing In Ground,29=Wing In Ground (no other information)"                                                                   \
   ",30=Fishing,31=Towing,32=Towing exceeds 200m or wider than 25m,33=Engaged in dredging or underwater operations,34=Engaged in " \
   "diving operations"                                                                                                             \
   ",35=Engaged in military operations,36=Sailing,37=Pleasure"                                                                     \
   ",40=High speed craft,41=High speed craft carrying dangerous goods,42=High speed craft hazard cat B,43=High speed craft "       \
   "hazard cat C,44=High speed craft hazard cat D,49=High speed craft (no additional information)"                                 \
   ",50=Pilot vessel,51=SAR,52=Tug,53=Port tender,54=Anti-pollution,55=Law enforcement,56=Spare,57=Spare #2,58=Medical,59=RR "     \
   "Resolution No.18"                                                                                                              \
   ",60=Passenger ship,69=Passenger ship (no additional information)"                                                              \
   ",70=Cargo ship,71=Cargo ship carrying dangerous goods,72=Cargo ship hazard cat B,73=Cargo ship hazard cat C,74=Cargo ship "    \
   "hazard cat D,79=Cargo ship (no additional information)"                                                                        \
   ",80=Tanker,81=Tanker carrying dangerous goods,82=Tanker hazard cat B,83=Tanker hazard cat C,84=Tanker hazard cat D,89=Tanker " \
   "(no additional information)"                                                                                                   \
   ",90=Other,91=Other carrying dangerous goods,92=Other hazard cat B,93=Other hazard cat C,94=Other hazard cat D,99=Other (no "   \
   "additional information)")

/* http://www.nmea.org/Assets/20120726%20nmea%202000%20class%20&%20function%20codes%20v%202.00.pdf */
#define LOOKUP_DEVICE_CLASS                        \
  (",0=Reserved for 2000 Use"                      \
   ",10=System tools"                              \
   ",20=Safety systems"                            \
   ",25=Internetwork device"                       \
   ",30=Electrical Distribution"                   \
   ",35=Electrical Generation"                     \
   ",40=Steering and Control surfaces"             \
   ",50=Propulsion"                                \
   ",60=Navigation"                                \
   ",70=Communication"                             \
   ",75=Sensor Communication Interface"            \
   ",80=Instrumentation/general systems"           \
   ",85=External Environment"                      \
   ",90=Internal Environment"                      \
   ",100=Deck + cargo + fishing equipment systems" \
   ",120=Display"                                  \
   ",125=Entertainment")

#define LOOKUP_REPEAT_INDICATOR (",0=Initial,1=First retransmission,2=Second retransmission,3=Final retransmission")

#define LOOKUP_AIS_TRANSCEIVER        \
  (",0=Channel A VDL reception"       \
   ",1=Channel B VDL reception"       \
   ",2=Channel A VDL transmission"    \
   ",3=Channel B VDL transmission"    \
   ",4=Own information not broadcast" \
   ",5=Reserved")

#define LOOKUP_AIS_ASSIGNED_MODE  \
  (",0=Autonomous and continuous" \
   ",1=Assigned mode")

#define LOOKUP_ATON_TYPE                                 \
  (",0=Default: Type of AtoN not specified"              \
   ",1=Reference point"                                   \
   ",2=RACON"                                            \
   ",3=Fixed structure off-shore"                        \
   ",4=Reserved for future use"                          \
   ",5=Fixed light: without sectors"                     \
   ",6=Fixed light: with sectors"                        \
   ",7=Fixed leading light front"                        \
   ",8=Fixed leading light rear"                         \
   ",9=Fixed beacon: cardinal N"                         \
   ",10=Fixed beacon: cardinal E"                        \
   ",11=Fixed beacon: cardinal S"                        \
   ",12=Fixed beacon: cardinal W"                        \
   ",13=Fixed beacon: port hand"                         \
   ",14=Fixed beacon: starboard hand"                    \
   ",15=Fixed beacon: preferred channel port hand"       \
   ",16=Fixed beacon: preferred channel starboard hand"  \
   ",17=Fixed beacon: isolated danger"                   \
   ",18=Fixed beacon: safe water"                        \
   ",19=Fixed beacon: special mark"                      \
   ",20=Floating AtoN: cardinal N"                       \
   ",21=Floating AtoN: cardinal E"                       \
   ",22=Floating AtoN: cardinal S"                       \
   ",23=Floating AtoN: cardinal W"                       \
   ",24=Floating AtoN: port hand mark"                   \
   ",25=Floating AtoN: starboard hand mark"              \
   ",26=Floating AtoN: preferred channel port hand"      \
   ",27=Floating AtoN: preferred channel starboard hand" \
   ",28=Floating AtoN: isolated danger"                  \
   ",29=Floating AtoN: safe water"                       \
   ",30=Floating AtoN: special mark"                     \
   ",31=Floating AtoN: light vessel/LANBY/rigs")

#define LOOKUP_AIS_SPECIAL_MANEUVER     \
  (",0=Not available"                   \
   ",1=Not engaged in special maneuver" \
   ",2=Engaged in special maneuver"     \
   ",3=Reserved")

#define LOOKUP_POSITION_FIX_DEVICE   \
  (",0=Default: undefined"           \
   ",1=GPS"                          \
   ",2=GLONASS"                      \
   ",3=Combined GPS/GLONASS"         \
   ",4=Loran-C"                      \
   ",5=Chayka"                       \
   ",6=Integrated navigation system" \
   ",7=Surveyed"                     \
   ",8=Galileo"                      \
   ",15=Internal GNSS")

#define LOOKUP_ENGINE_INSTANCE (",0=Single Engine or Dual Engine Port,1=Dual Engine Starboard")

// http://www.osukl.com/wp-content/uploads/2015/04/3155-UM.pdf
#define LOOKUP_ENGINE_STATUS_1                                                                                                     \
  (",0=Check Engine,1=Over Temperature,2=Low Oil Pressure,3=Low Oil Level,4=Low Fuel Pressure,5=Low System Voltage,6=Low Coolant " \
   "Level,7=Water Flow,8=Water In Fuel,9=Charge Indicator,10=Preheat Indicator,11=High Boost Pressure,12=Rev Limit "               \
   "Exceeded,13=EGR System,14=Throttle Position Sensor,15=Emergency Stop")
#define LOOKUP_ENGINE_STATUS_2                                                                                           \
  (",0=Warning Level 1,1=Warning Level 2,2=Power Reduction,3=Maintenance Needed,4=Engine Comm Error,5=Sub or Secondary " \
   "Throttle,6=Neutral Start Protect,7=Engine Shutting Down")

#define LOOKUP_GEAR_STATUS (",0=Forward,1=Neutral,2=Reverse")

#define LOOKUP_POSITION_ACCURACY (",0=Low,1=High")

#define LOOKUP_RAIM_FLAG (",0=not in use,1=in use")

#define LOOKUP_TIME_STAMP (",60=Not available,61=Manual input mode,62=Dead reckoning mode,63=Positioning system is inoperative")

#define LOOKUP_GNS_AIS (",0=undefined,1=GPS,2=GLONASS,3=GPS+GLONASS,4=Loran-C,5=Chayka,6=integrated,7=surveyed,8=Galileo")
#define LOOKUP_GNS \
  (",0=GPS,1=GLONASS,2=GPS+GLONASS,3=GPS+SBAS/WAAS,4=GPS+SBAS/WAAS+GLONASS,5=Chayka,6=integrated,7=surveyed,8=Galileo")

#define LOOKUP_GNS_METHOD                                                                                             \
  (",0=no GNSS,1=GNSS fix,2=DGNSS fix,3=Precise GNSS,4=RTK Fixed Integer,5=RTK float,6=Estimated (DR) mode,7=Manual " \
   "Input,8=Simulate mode")

#define LOOKUP_GNS_INTEGRITY (",0=No integrity checking,1=Safe,2=Caution")

#define LOOKUP_SYSTEM_TIME (",0=GPS,1=GLONASS,2=Radio Station,3=Local Cesium clock,4=Local Rubidium clock,5=Local Crystal clock")

#define LOOKUP_MAGNETIC_VARIATION \
  (",0=Manual"                    \
   ",1=Automatic Chart"           \
   ",2=Automatic Table"           \
   ",3=Automatic Calculation"     \
   ",4=WMM 2000"                  \
   ",5=WMM 2005"                  \
   ",6=WMM 2010"                  \
   ",7=WMM 2015"                  \
   ",8=WMM 2020")

#define LOOKUP_RESIDUAL_MODE (",0=Autonomous,1=Differential enhanced,2=Estimated,3=Simulator,4=Manual")

#define LOOKUP_WIND_REFERENCE                                                                                       \
  (",0=True (ground referenced to North),1=Magnetic (ground referenced to Magnetic North),2=Apparent,3=True (boat " \
   "referenced),4=True (water referenced)")

#define LOOKUP_WATER_REFERENCE (",0=Paddle wheel,1=Pitot tube,2=Doppler,3=Correlation (ultra sound),4=Electro Magnetic")

#define LOOKUP_YES_NO \
  (",0=No" \
   ",1=Yes") /* Note that Error and Unknown are automatically decoded */
#define LOOKUP_OK_WARNING (",0=OK,1=Warning")
#define LOOKUP_OFF_ON \
  (",0=Off"           \
   ",1=On") /* Note that Error and Unknown are automatically decoded */

#define LOOKUP_DIRECTION_REFERENCE (",0=True,1=Magnetic,2=Error,3=Null")

#define LOOKUP_DIRECTION_RUDDER (",0=No Order,1=Move to starboard,2=Move to port")

#define LOOKUP_NAV_STATUS                    \
  (",0=Under way using engine"               \
   ",1=At anchor"                            \
   ",2=Not under command"                    \
   ",3=Restricted maneuverability"           \
   ",4=Constrained by her draught"           \
   ",5=Moored"                               \
   ",6=Aground"                              \
   ",7=Engaged in Fishing"                   \
   ",8=Under way sailing"                    \
   ",9=Hazardous material - High Speed"      \
   ",10=Hazardous material - Wing in Ground" \
   ",14=AIS-SART")

#define LOOKUP_POWER_FACTOR (",0=Leading,1=Lagging,2=Error")

#define LOOKUP_TEMPERATURE_SOURCE           \
  (",0=Sea Temperature"                     \
   ",1=Outside Temperature"                 \
   ",2=Inside Temperature"                  \
   ",3=Engine Room Temperature"             \
   ",4=Main Cabin Temperature"              \
   ",5=Live Well Temperature"               \
   ",6=Bait Well Temperature"               \
   ",7=Refrigeration Temperature"           \
   ",8=Heating System Temperature"          \
   ",9=Dew Point Temperature"               \
   ",10=Apparent Wind Chill Temperature"    \
   ",11=Theoretical Wind Chill Temperature" \
   ",12=Heat Index Temperature"             \
   ",13=Freezer Temperature"                \
   ",14=Exhaust Gas Temperature")

#define LOOKUP_HUMIDITY_SOURCE \
  (",0=Inside"                 \
   ",1=Outside")

#define LOOKUP_PRESSURE_SOURCE \
  (",0=Atmospheric"            \
   ",1=Water"                  \
   ",2=Steam"                  \
   ",3=Compressed Air"         \
   ",4=Hydraulic"              \
   ",5=Filter"                 \
   ",6=AltimeterSetting"       \
   ",7=Oil"                    \
   ",8=Fuel")
#define LOOKUP_DSC_FORMAT     \
  (",102=Geographical area"   \
   ",112=Distress"            \
   ",114=Common interest"     \
   ",116=All ships"           \
   ",120=Individual stations" \
   ",121=Non-calling purpose" \
   ",123=Individual station automatic")

#define LOOKUP_DSC_CATEGORY \
  (",100=Routine"           \
   ",108=Safety"            \
   ",110=Urgency"           \
   ",112=Distress")

#define LOOKUP_DSC_NATURE     \
  (",100=Fire"                \
   ",101=Flooding"            \
   ",102=Collision"           \
   ",103=Grounding"           \
   ",104=Listing"             \
   ",105=Sinking"             \
   ",106=Disabled and adrift" \
   ",107=Undesignated"        \
   ",108=Abandoning ship"     \
   ",109=Piracy"              \
   ",110=Man overboard"       \
   ",112=EPIRB emission")

#define LOOKUP_DSC_FIRST_TELECOMMAND                      \
  (",100=F3E/G3E All modes TP"                            \
   ",101=F3E/G3E duplex TP"                               \
   ",103=Polling"                                         \
   ",104=Unable to comply"                                \
   ",105=End of call"                                     \
   ",106=Data"                                            \
   ",109=J3E TP"                                          \
   ",110=Distress acknowledgement"                        \
   ",112=Distress relay"                                  \
   ",113=F1B/J2B TTY-FEC"                                 \
   ",115=F1B/J2B TTY-ARQ"                                 \
   ",118=Test"                                            \
   ",121=Ship position or location registration updating" \
   ",126=No information")

#define LOOKUP_DSC_SECOND_TELECOMMAND                                   \
  (",100=No reason given"                                               \
   ",101=Congestion at MSC"                                             \
   ",102=Busy"                                                          \
   ",103=Queue indication"                                              \
   ",104=Station barred"                                                \
   ",105=No operator available"                                         \
   ",106=Operator temporarily unavailable"                              \
   ",107=Equipment disabled"                                            \
   ",108=Unable to use proposed channel"                                \
   ",109=Unable to use proposed mode"                                   \
   ",110=Ships and aircraft of States not parties to an armed conflict" \
   ",111=Medical transports"                                            \
   ",112=Pay phone/public call office"                                  \
   ",113=Fax/data"                                                      \
   ",126=No information")

#define LOOKUP_DSC_EXPANSION_DATA           \
  (",100=Enhanced position"                 \
   ",101=Source and datum of position"      \
   ",102=SOG"                               \
   ",103=COG"                               \
   ",104=Additional station identification" \
   ",105=Enhanced geographic area"          \
   ",106=Number of persons on board")

#define LOOKUP_SEATALK_ALARM_STATUS          \
  (",0=Alarm condition not met"              \
   ",1=Alarm condition met and not silenced" \
   ",2=Alarm condition met and silenced")

#define LOOKUP_SEATALK_ALARM_ID                                   \
  (",0=No Alarm"                                                  \
   ",1=Shallow Depth"                                             \
   ",2=Deep Depth"                                                \
   ",3=Shallow Anchor"                                            \
   ",4=Deep Anchor"                                               \
   ",5=Off Course"                                                \
   ",6=AWA High"                                                  \
   ",7=AWA Low"                                                   \
   ",8=AWS High"                                                  \
   ",9=AWS Low"                                                   \
   ",10=TWA High"                                                 \
   ",11=TWA Low"                                                  \
   ",12=TWS High"                                                 \
   ",13=TWS Low"                                                  \
   ",14=WP Arrival"                                               \
   ",15=Boat Speed High"                                          \
   ",16=Boat Speed Low"                                           \
   ",17=Sea Temp High"                                            \
   ",18=Sea Temp Low"                                             \
   ",19=Pilot Watch"                                              \
   ",20=Pilot Off Course"                                         \
   ",21=Pilot Wind Shift"                                         \
   ",22=Pilot Low Battery"                                        \
   ",23=Pilot Last Minute Of Watch"                               \
   ",24=Pilot No NMEA Data"                                       \
   ",25=Pilot Large XTE"                                          \
   ",26=Pilot NMEA DataError"                                     \
   ",27=Pilot CU Disconnected"                                    \
   ",28=Pilot Auto Release"                                       \
   ",29=Pilot Way Point Advance"                                  \
   ",30=Pilot Drive Stopped"                                      \
   ",31=Pilot Type Unspecified"                                   \
   ",32=Pilot Calibration Required"                               \
   ",33=Pilot Last Heading"                                       \
   ",34=Pilot No Pilot"                                           \
   ",35=Pilot Route Complete"                                     \
   ",36=Pilot Variable Text"                                      \
   ",37=GPS Failure"                                              \
   ",38=MOB"                                                      \
   ",39=Seatalk1 Anchor"                                          \
   ",40=Pilot Swapped Motor Power"                                \
   ",41=Pilot Standby Too Fast To Fish"                           \
   ",42=Pilot No GPS Fix"                                         \
   ",43=Pilot No GPS COG"                                         \
   ",44=Pilot Start Up"                                           \
   ",45=Pilot Too Slow"                                           \
   ",46=Pilot No Compass"                                         \
   ",47=Pilot Rate Gyro Fault"                                    \
   ",48=Pilot Current Limit"                                      \
   ",49=Pilot Way Point Advance Port"                             \
   ",50=Pilot Way Point Advance Stbd"                             \
   ",51=Pilot No Wind Data"                                       \
   ",52=Pilot No Speed Data"                                      \
   ",53=Pilot Seatalk Fail1"                                      \
   ",54=Pilot Seatalk Fail2"                                      \
   ",55=Pilot Warning Too Fast To Fish"                           \
   ",56=Pilot Auto Dockside Fail"                                 \
   ",57=Pilot Turn Too Fast"                                      \
   ",58=Pilot No Nav Data"                                        \
   ",59=Pilot Lost Waypoint Data"                                 \
   ",60=Pilot EEPROM Corrupt"                                     \
   ",61=Pilot Rudder Feedback Fail"                               \
   ",62=Pilot Autolearn Fail1"                                    \
   ",63=Pilot Autolearn Fail2"                                    \
   ",64=Pilot Autolearn Fail3"                                    \
   ",65=Pilot Autolearn Fail4"                                    \
   ",66=Pilot Autolearn Fail5"                                    \
   ",67=Pilot Autolearn Fail6"                                    \
   ",68=Pilot Warning Cal Required"                               \
   ",69=Pilot Warning OffCourse"                                  \
   ",70=Pilot Warning XTE"                                        \
   ",71=Pilot Warning Wind Shift"                                 \
   ",72=Pilot Warning Drive Short"                                \
   ",73=Pilot Warning Clutch Short"                               \
   ",74=Pilot Warning Solenoid Short"                             \
   ",75=Pilot Joystick Fault"                                     \
   ",76=Pilot No Joystick Data"                                   \
   ",77=not assigned"                                             \
   ",78=not assigned"                                             \
   ",79=not assigned"                                             \
   ",80=Pilot Invalid Command"                                    \
   ",81=AIS TX Malfunction"                                       \
   ",82=AIS Antenna VSWR fault"                                   \
   ",83=AIS Rx channel 1 malfunction"                             \
   ",84=AIS Rx channel 2 malfunction"                             \
   ",85=AIS No sensor position in use"                            \
   ",86=AIS No valid SOG information"                             \
   ",87=AIS No valid COG information"                             \
   ",88=AIS 12V alarm"                                            \
   ",89=AIS 6V alarm"                                             \
   ",90=AIS Noise threshold exceeded channel A"                   \
   ",91=AIS Noise threshold exceeded channel B"                   \
   ",92=AIS Transmitter PA fault"                                 \
   ",93=AIS 3V3 alarm"                                            \
   ",94=AIS Rx channel 70 malfunction"                            \
   ",95=AIS Heading lost/invalid"                                 \
   ",96=AIS internal GPS lost"                                    \
   ",97=AIS No sensor position"                                   \
   ",98=AIS Lock failure"                                         \
   ",99=AIS Internal GGA timeout"                                 \
   ",100=AIS Protocol stack restart"                              \
   ",101=Pilot No IPS communications"                             \
   ",102=Pilot Power-On or Sleep-Switch Reset While Engaged     " \
   ",103=Pilot Unexpected Reset While Engaged"                    \
   ",104=AIS Dangerous Target"                                    \
   ",105=AIS Lost Target"                                         \
   ",106=AIS Safety Related Message (used to silence)"            \
   ",107=AIS Connection Lost"                                     \
   ",108=No Fix")

#define LOOKUP_SEATALK_ALARM_GROUP \
  (",0=Instrument"                 \
   ",1=Autopilot"                  \
   ",2=Radar"                      \
   ",3=Chart Plotter"              \
   ",4=AIS")

#define LOOKUP_ENTERTAINMENT_ZONE \
  (",0=All zones"                 \
   ",1=Zone 1"                    \
   ",2=Zone 2"                    \
   ",3=Zone 3"                    \
   ",4=Zone 4")

#define LOOKUP_ENTERTAINMENT_SOURCE \
  (",0=Vessel alarm"                \
   ",1=AM"                          \
   ",2=FM"                          \
   ",3=Weather"                     \
   ",4=DAB"                         \
   ",5=Aux"                         \
   ",6=USB"                         \
   ",7=CD"                          \
   ",8=MP3"                         \
   ",9=Apple iOS"                   \
   ",10=Android"                    \
   ",11=Bluetooth"                  \
   ",12=Sirius XM"                  \
   ",13=Pandora"                    \
   ",14=Spotify"                    \
   ",15=Slacker"                    \
   ",16=Songza"                     \
   ",17=Apple Radio"                \
   ",18=Last FM"                    \
   ",19=Ethernet"                   \
   ",20=Video MP4"                  \
   ",21=Video DVD"                  \
   ",22=Video BluRay"               \
   ",23=HDMI"                       \
   ",24=Video")

#define LOOKUP_ENTERTAINMENT_PLAY_STATUS \
  (",0=Play"                             \
   ",1=Pause"                            \
   ",2=Stop"                             \
   ",3=FF (1x)"                          \
   ",4=FF (2x)"                          \
   ",5=FF (3x)"                          \
   ",6=FF (4x)"                          \
   ",7=RW (1x)"                          \
   ",8=RW (2x)"                          \
   ",9=RW (3x)"                          \
   ",10=RW (4x)"                         \
   ",11=Skip ahead"                      \
   ",12=Skip back"                       \
   ",13=Jog ahead"                       \
   ",14=Jog back"                        \
   ",15=Seek up"                         \
   ",16=Seek down"                       \
   ",17=Scan up"                         \
   ",18=Scan down"                       \
   ",19=Tune up"                         \
   ",20=Tune down"                       \
   ",21=Slow motion (.75x)"              \
   ",22=Slow motion (.5x)"               \
   ",23=Slow motion (.25x)"              \
   ",24=Slow motion (.125x)"             \
   ",25=Source renaming")

#define LOOKUP_ENTERTAINMENT_REPEAT_STATUS \
  (",0=Off"                                \
   ",1=One"                                \
   ",2=All")

#define LOOKUP_ENTERTAINMENT_SHUFFLE_STATUS \
  (",0=Off"                                 \
   ",1=Play queue"                          \
   ",2=All")

#define LOOKUP_ENTERTAINMENT_LIKE_STATUS \
  (",0=None"                             \
   ",1=Thumbs up"                        \
   ",2=Thumbs down")

#define LOOKUP_ENTERTAINMENT_TYPE \
  (",0=File"                      \
   ",1=Playlist Name"             \
   ",2=Genre Name"                \
   ",3=Album Name"                \
   ",4=Artist Name"               \
   ",5=Track Name"                \
   ",6=Station Name"              \
   ",7=Station Number"            \
   ",8=Favourite Number"          \
   ",9=Play Queue"                \
   ",10=Content Info")

#define LOOKUP_ENTERTAINMENT_GROUP \
  (",0=File"                       \
   ",1=Playlist Name"              \
   ",2=Genre Name"                 \
   ",3=Album Name"                 \
   ",4=Artist Name"                \
   ",5=Track Name"                 \
   ",6=Station Name"               \
   ",7=Station Number"             \
   ",8=Favourite Number"           \
   ",9=Play Queue"                 \
   ",10=Content Info")

#define LOOKUP_ENTERTAINMENT_CHANNEL \
  (",0=All channels"                 \
   ",1=Stereo full range"            \
   ",2=Stereo front"                 \
   ",3=Stereo back"                  \
   ",4=Stereo surround"              \
   ",5=Center"                       \
   ",6=Subwoofer"                    \
   ",7=Front left"                   \
   ",8=Front right"                  \
   ",9=Back left"                    \
   ",10=Back right"                  \
   ",11=Surround left"               \
   ",12=Surround right")

#define LOOKUP_ENTERTAINMENT_EQ \
  (",0=Flat"                    \
   ",1=Rock"                    \
   ",2=Hall"                    \
   ",3=Jazz"                    \
   ",4=Pop"                     \
   ",5=Live"                    \
   ",6=Classic"                 \
   ",7=Vocal"                   \
   ",8=Arena"                   \
   ",9=Cinema"                  \
   ",10=Custom")

#define LOOKUP_ENTERTAINMENT_FILTER \
  (",0=Full range"                  \
   ",1=High pass"                   \
   ",2=Low pass"                    \
   ",3=Band pass"                   \
   ",4=Notch filter")

#define LOOKUP_ALERT_TYPE  \
  (",0=Reserved"           \
   ",1=Emergency Alarm"    \
   ",2=Alarm"              \
   ",3=Reserved"           \
   ",4=Reserved"           \
   ",5=Warning"            \
   ",6=Reserved"           \
   ",7=Reserved"           \
   ",8=Caution"            \
   ",13=Reserved"          \
   ",14=Data out of range" \
   ",15=Data not available")

#define LOOKUP_ALERT_CATEGORY \
  (",0=Navigational"          \
   ",1=Technical"             \
   ",13=Reserved"             \
   ",14=Data out of range"    \
   ",15=Data not available")

#define LOOKUP_ALERT_TRIGGER_CONDITION \
  (",0=Manual"                         \
   ",1=Auto"                           \
   ",2=Test"                           \
   ",3=Disabled"                       \
   ",13=Reserved"                      \
   ",14=Data out of range"             \
   ",15=Data not available")

#define LOOKUP_ALERT_THRESHOLD_STATUS \
  (",0=Normal"                        \
   ",1=Threshold Exceeded"            \
   ",2=Extreme Threshold Exceeded"    \
   ",3=Low Threshold Exceeded"        \
   ",4=Acknowledged"                  \
   ",5=Awaiting Acknowledge"          \
   ",253=Reserved"                    \
   ",254=Data out of range"           \
   ",255=Data not available")

#define LOOKUP_ALERT_STATE   \
  (",0=Disabled"             \
   ",1=Normal"               \
   ",2=Active"               \
   ",3=Silenced"             \
   ",4=Acknowledged"         \
   ",5=Awaiting Acknowledge" \
   ",253=Reserved"           \
   ",254=Data out of range"  \
   ",255=Data not available")

#define LOOKUP_ALERT_LANGUAGE_ID \
  (",0=English (US)"             \
   ",1=English (UK)"             \
   ",2=Arabic"                   \
   ",3=Chinese (simplified)"     \
   ",4=Croatian"                 \
   ",5=Danish"                   \
   ",6=Dutch"                    \
   ",7=Finnish"                  \
   ",8=French"                   \
   ",9=German"                   \
   ",10=Greek"                   \
   ",11=Italian"                 \
   ",12=Japanese"                \
   ",13=Korean"                  \
   ",14=Norwegian"               \
   ",15=Polish"                  \
   ",16=Portuguese"              \
   ",17=Russian"                 \
   ",18=Spanish"                 \
   ",19=Swedish")

#define LOOKUP_ALERT_RESPONSE_COMMAND \
  (",0=Acknowledge"                   \
   ",1=Temporary Silence"             \
   ",2=Test Command off"              \
   ",3=Test Command on")

#define LOOKUP_CONVERTER_STATE \
  (",0=Off"                    \
   ",1=Low Power Mode"         \
   ",2=Fault"                  \
   ",3=Bulk"                   \
   ",4=Absorption"             \
   ",5=Float"                  \
   ",6=Storage"                \
   ",7=Equalize"               \
   ",8=Pass thru"              \
   ",9=Inverting"              \
   ",10=Assisting")

#define LOOKUP_THRUSTER_DIRECTION_CONTROL \
  (",0=Off"                               \
   ",1=Ready"                             \
   ",2=To Port"                           \
   ",3=To Starboard")

#define LOOKUP_THRUSTER_RETRACT_CONTROL \
  (",0=Off"                             \
   ",1=Extend"                          \
   ",2=Retract"                         \
   ",3=Reserved")

#define LOOKUP_THRUSTER_CONTROL_EVENTS       \
  (",0=Another device controlling thruster"  \
   ",1=Boat speed too fast to safely use thruster")

#define LOOKUP_THRUSTER_MOTOR_TYPE \
  (",0=12VDC"                      \
   ",1=24VDC"                      \
   ",2=48VDC"                      \
   ",3=24VAC"                      \
   ",4=Hydraulic")

#define LOOKUP_THRUSTER_MOTOR_EVENTS       \
  (",0=Motor over temperature cutout"  \
   ",1=Motor over current cutout"  \
   ",2=Low oil level warning"  \
   ",3=Oil over temperature warning"  \
   ",4=Controller under voltage cutout"  \
   ",5=Manufacturer defined")

typedef enum PacketComplete
{
  PACKET_COMPLETE              = 0,
  PACKET_FIELDS_UNKNOWN        = 1,
  PACKET_FIELD_LENGTHS_UNKNOWN = 2,
  PACKET_PRECISION_UNKNOWN     = 4,
  PACKET_LOOKUPS_UNKNOWN       = 8,
  PACKET_NOT_SEEN              = 16
} PacketComplete;

#define PACKET_INCOMPLETE (PACKET_FIELDS_UNKNOWN | PACKET_FIELD_LENGTHS_UNKNOWN | PACKET_PRECISION_UNKNOWN)
#define PACKET_INCOMPLETE_LOOKUP (PACKET_INCOMPLETE | PACKET_LOOKUPS_UNKNOWN)

typedef enum PacketType
{
  PACKET_SINGLE,
  PACKET_FAST,
  PACKET_ISO11783
} PacketType;

typedef struct
{
  char *     description;
  uint32_t   pgn;
  uint16_t   complete;        /* Either PACKET_COMPLETE or bit values set for various unknown items */
  PacketType type;            /* Single, Fast or ISO11783 */
  uint32_t   size;            /* (Minimal) size of this PGN. Helps to determine initial malloc */
  uint32_t   repeatingFields; /* How many fields at the end repeat until the PGN is exhausted? */
  Field      fieldList[30]; /* Note fixed # of fields; increase if needed. RepeatingFields support means this is enough for now. */
  uint32_t   fieldCount;    /* Filled by C, no need to set in initializers. */
  char *     camelDescription; /* Filled by C, no need to set in initializers. */
  bool       unknownPgn;       /* true = this is a catch-all for unknown PGNs */
} Pgn;

// Returns the first pgn that matches the given id, or 0 if not found.
Pgn *searchForPgn(int pgn);

// Returns a pointer (potentially invalid) to the first pgn that does not match "first".
Pgn *endPgn(Pgn *first);

Pgn *getMatchingPgn(int pgnId, uint8_t *dataStart, int length);

bool printPgn(RawMessage *msg, uint8_t *dataStart, int length, bool showData, bool showJson);
void checkPgnList(void);

Field *getField(uint32_t pgn, uint32_t field);
void   extractNumber(const Field *field, uint8_t *data, size_t startBit, size_t bits, int64_t *value, int64_t *maxValue);

int parseRawFormatPlain(char *msg, RawMessage *m, bool showJson);
int parseRawFormatFast(char *msg, RawMessage *m, bool showJson);
int parseRawFormatAirmar(char *msg, RawMessage *m, bool showJson);
int parseRawFormatChetco(char *msg, RawMessage *m, bool showJson);
int parseRawFormatGarminCSV(char *msg, RawMessage *m, bool showJson, bool absolute);
int parseRawFormatYDWG02(char *msg, RawMessage *m, bool showJson);

#ifdef GLOBALS
Pgn pgnList[] = {

    /* PDU1 (addressed) single-frame PGN range 0E800 to 0xEEFF (59392 - 61183) */

    {"Unknown single-frame addressed",
     0,
     PACKET_INCOMPLETE,
     PACKET_SINGLE,
     8,
     0,
     {{"Data", BYTES(8), RES_BINARY, false, 0, ""}, {0}},
     0,
     0,
     true}

    /************ Protocol PGNs ************/
    /* http://www.nmea.org/Assets/july%202010%20nmea2000_v1-301_app_b_pgn_field_list.pdf */
    /* http://www.maretron.com/products/pdf/J2K100-Data_Sheet.pdf */
    /* http://www.nmea.org/Assets/pgn059392.pdf */
    /* http://www8.garmin.com/manuals/GPSMAP4008_NMEA2000NetworkFundamentals.pdf */
    /* http://www.furunousa.com/Furuno/Doc/0/8JT2BMDDIB249FCNUK64DKLV67/GP330B%20NMEA%20PGNs.pdf */
    /* http://www.nmea.org/Assets/20140710%20nmea-2000-060928%20iso%20address%20claim%20pgn%20corrigendum.pdf */
    ,
    {"ISO Acknowledgement",
     59392,
     PACKET_COMPLETE,
     PACKET_SINGLE,
     8,
     0,
     {{"Control", BYTES(1), RES_LOOKUP, false, ",0=ACK,1=NAK,2=Access Denied,3=Address Busy", ""},
      {"Group Function", BYTES(1), 1, false, 0, ""},
      {"Reserved", 24, RES_BINARY, false, 0, "Reserved"},
      {"PGN", 24, RES_INTEGER, false, 0, "Parameter Group Number of requested information"},
      {0}}}

    ,
    {"ISO Request", 59904, PACKET_COMPLETE, PACKET_SINGLE, 3, 0, {{"PGN", 24, RES_INTEGER, false, 0, ""}, {0}}}

    /* For a good explanation of ISO 11783 transport protocol (as used in J1939) see
     * http://www.simmasoftware.com/j1939-presentation.pdf
     *
     * First: Transmit a RTS message to the specific address that says:
     *   1. I'm about to send the following PGN in multiple packets.
     *   2. I'm sending X amount of data.
     *   3. I'm sending Y number of packets.
     *   4. I can send Z number of packets at once.
     * Second: Wait for CTS: CTS says:
     *   1. I can receive M number of packets at once.
     *   2. Start sending with sequence number N.
     * Third: Send data. Then repeat steps starting with #2. When all data sent, wait for ACK.
     */

    // ISO 11783 defines this PGN as part of the transport protocol method used for transmitting messages that have 9 or more data
    // bytes. This PGN represents a single packet of a multipacket message.
    ,
    {"ISO Transport Protocol, Data Transfer",
     60160,
     PACKET_COMPLETE,
     PACKET_SINGLE,
     8,
     1,
     {{"SID", BYTES(1), 1, false, 0, ""}, {"Data", BYTES(7), 1, false, 0, ""}, {0}}}

    // ''ISO 11783 defines this group function PGN as part of the transport protocol method used for transmitting messages that have
    // 9 or more data bytes. This PGN's role in the transport process is determined by the group function value found in the first
    // data byte of the PGN.''
    ,
    {"ISO Transport Protocol, Connection Management - Request To Send",
     60416,
     PACKET_COMPLETE,
     PACKET_SINGLE,
     8,
     1,
     {{"Group Function Code", BYTES(1), 1, false, "=16", "RTS"},
      {"Message size", BYTES(2), 1, false, 0, "bytes"},
      {"Packets", BYTES(1), 1, false, 0, "packets"},
      {"Packets reply", BYTES(1), 1, false, 0, "packets sent in response to CTS"} // This one is still mysterious to me...
      ,
      {"PGN", BYTES(3), RES_INTEGER, false, 0, "PGN"},
      {0}}},
    {"ISO Transport Protocol, Connection Management - Clear To Send",
     60416,
     PACKET_COMPLETE,
     PACKET_SINGLE,
     8,
     1,
     {{"Group Function Code", BYTES(1), 1, false, "=17", "CTS"},
      {"Max packets", BYTES(1), 1, false, 0, "packets before waiting for next CTS"},
      {"Next SID", BYTES(1), 1, false, 0, "packet"},
      {"Reserved", BYTES(2), RES_BINARY, false, 0, ""},
      {"PGN", BYTES(3), RES_INTEGER, false, 0, "PGN"},
      {0}}},
    {"ISO Transport Protocol, Connection Management - End Of Message",
     60416,
     PACKET_COMPLETE,
     PACKET_SINGLE,
     8,
     1,
     {{"Group Function Code", BYTES(1), 1, false, "=19", "EOM"},
      {"Total message size", BYTES(2), 1, false, 0, "bytes"},
      {"Total number of packets received", BYTES(1), 1, false, 0, "packets"},
      {"Reserved", BYTES(1), RES_BINARY, false, 0, ""},
      {"PGN", BYTES(3), RES_INTEGER, false, 0, "PGN"},
      {0}}},
    {"ISO Transport Protocol, Connection Management - Broadcast Announce",
     60416,
     PACKET_COMPLETE,
     PACKET_SINGLE,
     8,
     1,
     {{"Group Function Code", BYTES(1), 1, false, "=32", "BAM"},
      {"Message size", BYTES(2), 1, false, 0, "bytes"},
      {"Packets", BYTES(1), 1, false, 0, "frames"},
      {"Reserved", BYTES(1), RES_BINARY, false, 0, ""},
      {"PGN", BYTES(3), RES_INTEGER, false, 0, "PGN"},
      {0}}},
    {"ISO Transport Protocol, Connection Management - Abort",
     60416,
     PACKET_COMPLETE,
     PACKET_SINGLE,
     8,
     1,
     {{"Group Function Code", BYTES(1), 1, false, "=255", "Abort"},
      {"Reason", BYTES(1), RES_BINARY, false, 0, ""},
      {"Reserved", BYTES(2), RES_BINARY, false, 0, ""},
      {"PGN", BYTES(3), RES_INTEGER, false, 0, "PGN"},
      {0}}}

    ,
    {"ISO Address Claim",
     60928,
     PACKET_COMPLETE,
     PACKET_SINGLE,
     8,
     0,
     {{"Unique Number", 21, RES_BINARY, false, 0, "ISO Identity Number"},
      {"Manufacturer Code", 11, RES_MANUFACTURER, false, 0, ""},
      {"Device Instance Lower", 3, 1, false, 0, "ISO ECU Instance"},
      {"Device Instance Upper", 5, 1, false, 0, "ISO Function Instance"},
      {"Device Function", 8, 1, false, 0, "ISO Function"},
      {"Reserved", 1, RES_BINARY, false, 0, ""},
      {"Device Class", 7, RES_LOOKUP, false, LOOKUP_DEVICE_CLASS, ""},
      {"System Instance", 4, 1, false, 0, "ISO Device Class Instance"},
      {"Industry Group", 3, RES_LOOKUP, false, LOOKUP_INDUSTRY_CODE, ""},
      {"Reserved", 1, RES_BINARY, false, 0, "ISO Self Configurable"},
      {0}}}

    /* PDU1 (addressed) single-frame PGN range 0EF00 to 0xEFFF (61184 - 61439) */

    /* The following probably have the wrong Proprietary ID */
    ,
    {"Seatalk: Wireless Keypad Light Control",
     61184,
     PACKET_INCOMPLETE,
     PACKET_SINGLE,
     0x08,
     0,
     {{"Manufacturer Code", 11, RES_MANUFACTURER, false, "=1851", "Raymarine"},
      {"Reserved", 2, RES_NOTUSED, false, 0, ""},
      {"Industry Code", 3, RES_LOOKUP, false, "=4", "Marine Industry"},
      {"Proprietary ID", BYTES(1), RES_INTEGER, false, "=1", "Wireless Keypad Light Control"},
      {"Variant", BYTES(1), 1, false, 0, ""},
      {"Wireless Setting", BYTES(1), 1, false, 0, ""},
      {"Wired Setting", BYTES(1), 1, false, 0, ""},
      {0}}}

    ,
    {"Seatalk: Wireless Keypad Light Control",
     61184,
     PACKET_INCOMPLETE,
     PACKET_SINGLE,
     0x08,
     0,
     {{"Manufacturer Code", 11, RES_MANUFACTURER, false, "=1851", "Raymarine"},
      {"Reserved", 2, RES_NOTUSED, false, 0, ""},
      {"Industry Code", 3, RES_LOOKUP, false, "=4", "Marine Industry"},
      {"PID", BYTES(1), 1, false, 0, ""},
      {"Variant", BYTES(1), 1, false, 0, ""},
      {"Beep Control", BYTES(1), 1, false, 0, ""},
      {0}}}

    ,
    {"Victron Battery Register",
     61184,
     PACKET_INCOMPLETE,
     PACKET_SINGLE,
     0x08,
     0,
     {{"Manufacturer Code", 11, RES_MANUFACTURER, false, "=358", "Victron"},
      {"Reserved", 2, RES_NOTUSED, false, 0, ""},
      {"Industry Code", 3, RES_LOOKUP, false, "=4", "Marine Industry"},
      {"Register Id", BYTES(2), 1, false, 0, ""},
      {"Payload", BYTES(4), 1, false, 0, ""},
      {0}}}

    ,
    {"Manufacturer Proprietary single-frame addressed",
     61184,
     PACKET_INCOMPLETE,
     PACKET_SINGLE,
     8,
     0,
     {{"Manufacturer Code", 11, RES_MANUFACTURER, false, 0, ""},
      {"Reserved", 2, RES_NOTUSED, false, 0, ""},
      {"Industry Code", 3, RES_LOOKUP, false, LOOKUP_INDUSTRY_CODE, ""},
      {"Data", BYTES(6), RES_BINARY, false, 0, ""},
      {0}},
     0,
     0,
     true}

    /* PDU2 non-addressed single-frame PGN range 0xF000 - 0xFEFF (61440 - 65279) */

    ,
    {"Unknown single-frame non-addressed",
     61440,
     PACKET_INCOMPLETE,
     PACKET_SINGLE,
     8,
     0,
     {{"Manufacturer Code", 11, RES_MANUFACTURER, false, 0, ""},
      {"Reserved", 2, RES_NOTUSED, false, 0, ""},
      {"Industry Code", 3, RES_LOOKUP, false, LOOKUP_INDUSTRY_CODE, ""},
      {"Data", BYTES(6), RES_BINARY, false, 0, ""},
      {0}},
     0,
     0,
     true}

    /* Maretron ACM 100 manual documents PGN 65001-65030 */

    ,
    {"Bus #1 Phase C Basic AC Quantities",
     65001,
     PACKET_COMPLETE,
     PACKET_SINGLE,
     8,
     0,
     {{"Line-Line AC RMS Voltage", BYTES(2), 1, false, "V", ""},
      {"Line-Neutral AC RMS Voltage", BYTES(2), 1, false, "V", ""},
      {"AC Frequency", BYTES(2), 1 / 128.0, false, "Hz", ""},
      {0}}}

    ,
    {"Bus #1 Phase B Basic AC Quantities",
     65002,
     PACKET_COMPLETE,
     PACKET_SINGLE,
     8,
     0,
     {{"Line-Line AC RMS Voltage", BYTES(2), 1, false, "V", ""},
      {"Line-Neutral AC RMS Voltage", BYTES(2), 1, false, "V", ""},
      {"AC Frequency", BYTES(2), 1 / 128.0, false, "Hz", ""},
      {0}}}

    ,
    {"Bus #1 Phase A Basic AC Quantities",
     65003,
     PACKET_COMPLETE,
     PACKET_SINGLE,
     8,
     0,
     {{"Line-Line AC RMS Voltage", BYTES(2), 1, false, "V", ""},
      {"Line-Neutral AC RMS Voltage", BYTES(2), 1, false, "V", ""},
      {"AC Frequency", BYTES(2), 1 / 128.0, false, "Hz", ""},
      {0}}}

    ,
    {"Bus #1 Average Basic AC Quantities",
     65004,
     PACKET_INCOMPLETE,
     PACKET_SINGLE,
     8,
     0,
     {{"Line-Line AC RMS Voltage", BYTES(2), 1, false, "V", ""},
      {"Line-Neutral AC RMS Voltage", BYTES(2), 1, false, "V", ""},
      {"AC Frequency", BYTES(2), 1 / 128.0, false, "Hz", ""},
      {0}}}

    ,
    {"Utility Total AC Energy",
     65005,
     PACKET_PRECISION_UNKNOWN,
     PACKET_SINGLE,
     8,
     0,
     {{"Total Energy Export", BYTES(4), 1, false, "kWh", ""}, {"Total Energy Import", BYTES(4), 1, false, "kWh", ""}, {0}}}

    ,
    {"Utility Phase C AC Reactive Power",
     65006,
     PACKET_COMPLETE,
     PACKET_SINGLE,
     8,
     0,
     {{"Reactive Power", BYTES(2), 1, false, "var", ""},
      {"Power Factor", BYTES(2), 1 / 16384, false, 0, ""},
      {"Power Factor Lagging", 2, RES_LOOKUP, false, LOOKUP_POWER_FACTOR, ""},
      {0}}}

    ,
    {"Utility Phase C AC Power",
     65007,
     PACKET_COMPLETE,
     PACKET_SINGLE,
     8,
     0,
     {{"Real Power", BYTES(4), 1, true, "W", "", -2000000000}, {"Apparent Power", BYTES(4), 1, true, "VA", "", -2000000000}, {0}}}

    ,
    {"Utility Phase C Basic AC Quantities",
     65008,
     PACKET_COMPLETE,
     PACKET_SINGLE,
     8,
     0,
     {{"Line-Line AC RMS Voltage", BYTES(2), 1, false, "V", ""},
      {"Line-Neutral AC RMS Voltage", BYTES(2), 1, false, "V", ""},
      {"AC Frequency", BYTES(2), 1 / 128.0, false, "Hz", ""},
      {"AC RMS Current", BYTES(2), 1, false, "A", ""},
      {0}}}

    ,
    {"Utility Phase B AC Reactive Power",
     65009,
     PACKET_COMPLETE,
     PACKET_SINGLE,
     8,
     0,
     {{"Reactive Power", BYTES(2), 1, false, "var", ""},
      {"Power Factor", BYTES(2), 1 / 16384, false, 0, ""},
      {"Power Factor Lagging", 2, RES_LOOKUP, false, LOOKUP_POWER_FACTOR, ""},
      {0}}}

    ,
    {"Utility Phase B AC Power",
     65010,
     PACKET_COMPLETE,
     PACKET_SINGLE,
     8,
     0,
     {{"Real Power", BYTES(4), 1, true, "W", "", -2000000000}, {"Apparent Power", BYTES(4), 1, true, "VA", "", -2000000000}, {0}}}

    ,
    {"Utility Phase B Basic AC Quantities",
     65011,
     PACKET_COMPLETE,
     PACKET_SINGLE,
     8,
     0,
     {{"Line-Line AC RMS Voltage", BYTES(2), 1, false, "V", ""},
      {"Line-Neutral AC RMS Voltage", BYTES(2), 1, false, "V", ""},
      {"AC Frequency", BYTES(2), 1 / 128.0, false, "Hz", ""},
      {"AC RMS Current", BYTES(2), 1, false, "A", ""},
      {0}}}

    ,
    {"Utility Phase A AC Reactive Power",
     65012,
     PACKET_COMPLETE,
     PACKET_SINGLE,
     8,
     0,
     {{"Reactive Power", BYTES(4), 1, true, "var", "", -2000000000},
      {"Power Factor", BYTES(2), 1 / 16384, true, 0, ""},
      {"Power Factor Lagging", 2, RES_LOOKUP, false, LOOKUP_POWER_FACTOR, ""},
      {0}}}

    ,
    {"Utility Phase A AC Power",
     65013,
     PACKET_COMPLETE,
     PACKET_SINGLE,
     8,
     0,
     {{"Real Power", BYTES(4), 1, true, "W", "", -2000000000}, {"Apparent Power", BYTES(4), 1, true, "VA", "", -2000000000}, {0}}}

    ,
    {"Utility Phase A Basic AC Quantities",
     65014,
     PACKET_COMPLETE,
     PACKET_SINGLE,
     8,
     0,
     {{"Line-Line AC RMS Voltage", BYTES(2), 1, false, "V", ""},
      {"Line-Neutral AC RMS Voltage", BYTES(2), 1, false, "V", ""},
      {"AC Frequency", BYTES(2), 1 / 128.0, false, "Hz", ""},
      {"AC RMS Current", BYTES(2), 1, false, "A", ""},
      {0}}}

    ,
    {"Utility Total AC Reactive Power",
     65015,
     PACKET_COMPLETE,
     PACKET_SINGLE,
     8,
     0,
     {{"Reactive Power", BYTES(4), 1, true, "var", "", -2000000000},
      {"Power Factor", BYTES(2), 1 / 16384, false, 0, ""},
      {"Power Factor Lagging", 2, RES_LOOKUP, false, LOOKUP_POWER_FACTOR, ""},
      {0}}}

    ,
    {"Utility Total AC Power",
     65016,
     PACKET_COMPLETE,
     PACKET_SINGLE,
     8,
     0,
     {{"Real Power", BYTES(4), 1, true, "W", "", -2000000000}, {"Apparent Power", BYTES(4), 1, true, "VA", "", -2000000000}, {0}}}

    ,
    {"Utility Average Basic AC Quantities",
     65017,
     PACKET_COMPLETE,
     PACKET_SINGLE,
     8,
     0,
     {{"Line-Line AC RMS Voltage", BYTES(2), 1, false, "V", ""},
      {"Line-Neutral AC RMS Voltage", BYTES(2), 1, false, "V", ""},
      {"AC Frequency", BYTES(2), 1 / 128.0, false, "Hz", ""},
      {"AC RMS Current", BYTES(2), 1, false, "A", ""},
      {0}}}

    ,
    {"Generator Total AC Energy",
     65018,
     PACKET_PRECISION_UNKNOWN,
     PACKET_SINGLE,
     8,
     0,
     {{"Total Energy Export", BYTES(4), 1, false, "kWh", ""}, {"Total Energy Import", BYTES(4), 1, false, "kWh", ""}, {0}}}

    ,
    {"Generator Phase C AC Reactive Power",
     65019,
     PACKET_COMPLETE,
     PACKET_SINGLE,
     8,
     0,
     {{"Reactive Power", BYTES(2), 1, false, "var", "", -2000000000},
      {"Power Factor", BYTES(2), 1 / 16384, false, 0, ""},
      {"Power Factor Lagging", 2, RES_LOOKUP, false, LOOKUP_POWER_FACTOR, ""},
      {0}}}

    ,
    {"Generator Phase C AC Power",
     65020,
     PACKET_COMPLETE,
     PACKET_SINGLE,
     8,
     0,
     {{"Real Power", BYTES(2), 1, false, "W", "", -2000000000}, {"Apparent Power", BYTES(2), 1, false, "VA", "", -2000000000}, {0}}}

    ,
    {"Generator Phase C Basic AC Quantities",
     65021,
     PACKET_COMPLETE,
     PACKET_SINGLE,
     8,
     0,
     {{"Line-Line AC RMS Voltage", BYTES(2), 1, false, "V", ""},
      {"Line-Neutral AC RMS Voltage", BYTES(2), 1, false, "V", ""},
      {"AC Frequency", BYTES(2), 1 / 128.0, false, "Hz", ""},
      {"AC RMS Current", BYTES(2), 1, false, "A", ""},
      {0}}}

    ,
    {"Generator Phase B AC Reactive Power",
     65022,
     PACKET_COMPLETE,
     PACKET_SINGLE,
     8,
     0,
     {{"Reactive Power", BYTES(2), 1, false, "var", "", -2000000000},
      {"Power Factor", BYTES(2), 1 / 16384, false, 0, ""},
      {"Power Factor Lagging", 2, RES_LOOKUP, false, LOOKUP_POWER_FACTOR, ""},
      {0}}}

    ,
    {"Generator Phase B AC Power",
     65023,
     PACKET_COMPLETE,
     PACKET_SINGLE,
     8,
     0,
     {{"Real Power", BYTES(2), 1, false, "W", "", -2000000000}, {"Apparent Power", BYTES(2), 1, false, "VA", "", -2000000000}, {0}}}

    ,
    {"Generator Phase B Basic AC Quantities",
     65024,
     PACKET_COMPLETE,
     PACKET_SINGLE,
     8,
     0,
     {{"Line-Line AC RMS Voltage", BYTES(2), 1, false, "V", ""},
      {"Line-Neutral AC RMS Voltage", BYTES(2), 1, false, "V", ""},
      {"AC Frequency", BYTES(2), 1 / 128.0, false, "Hz", ""},
      {"AC RMS Current", BYTES(2), 1, false, "A", ""},
      {0}}}

    ,
    {"Generator Phase A AC Reactive Power",
     65025,
     PACKET_COMPLETE,
     PACKET_SINGLE,
     8,
     0,
     {{"Reactive Power", BYTES(2), 1, false, "var", "", -2000000000},
      {"Power Factor", BYTES(2), 1 / 16384, false, 0, ""},
      {"Power Factor Lagging", 2, RES_LOOKUP, false, LOOKUP_POWER_FACTOR, ""},
      {0}}}

    ,
    {"Generator Phase A AC Power",
     65026,
     PACKET_COMPLETE,
     PACKET_SINGLE,
     8,
     0,
     {{"Real Power", BYTES(4), 1, false, "W", "", -2000000000}, {"Apparent Power", BYTES(4), 1, false, "VA", "", -2000000000}, {0}}}

    ,
    {"Generator Phase A Basic AC Quantities",
     65027,
     PACKET_COMPLETE,
     PACKET_SINGLE,
     8,
     0,
     {{"Line-Line AC RMS Voltage", BYTES(2), 1, false, "V", ""},
      {"Line-Neutral AC RMS Voltage", BYTES(2), 1, false, "V", ""},
      {"AC Frequency", BYTES(2), 1 / 128.0, false, "Hz", ""},
      {"AC RMS Current", BYTES(2), 1, false, "A", ""},
      {0}}}

    ,
    {"Generator Total AC Reactive Power",
     65028,
     PACKET_COMPLETE,
     PACKET_SINGLE,
     8,
     0,
     {{"Reactive Power", BYTES(2), 1, false, "var", "", -2000000000},
      {"Power Factor", BYTES(2), 1 / 16384, false, 0, ""},
      {"Power Factor Lagging", 2, RES_LOOKUP, false, LOOKUP_POWER_FACTOR, ""},
      {0}}}

    ,
    {"Generator Total AC Power",
     65029,
     PACKET_COMPLETE,
     PACKET_SINGLE,
     8,
     0,
     {{"Real Power", BYTES(4), 1, false, "W", "", -2000000000}, {"Apparent Power", BYTES(4), 1, false, "VA", "", -2000000000}, {0}}}

    ,
    {"Generator Average Basic AC Quantities",
     65030,
     PACKET_COMPLETE,
     PACKET_SINGLE,
     8,
     0,
     {{"Line-Line AC RMS Voltage", BYTES(2), 1, false, "V", ""},
      {"Line-Neutral AC RMS Voltage", BYTES(2), 1, false, "V", ""},
      {"AC Frequency", BYTES(2), 1 / 128.0, false, "Hz", ""},
      {"AC RMS Current", BYTES(2), 1, false, "A", ""},
      {0}}}

    ,
    {"ISO Commanded Address",
     65240,
     PACKET_COMPLETE,
     PACKET_ISO11783,
     9,
     0,
     /* ISO 11783 defined this message to provide a mechanism for assigning a network address to a node. The NAME information in the
     data portion of the message must match the name information of the node whose network address is to be set. */
     {{"Unique Number", 21, RES_BINARY, false, 0, "ISO Identity Number"},
      {"Manufacturer Code", 11, 1, false, 0, ""},
      {"Device Instance Lower", 3, 1, false, 0, "ISO ECU Instance"},
      {"Device Instance Upper", 5, 1, false, 0, "ISO Function Instance"},
      {"Device Function", BYTES(1), 1, false, 0, "ISO Function"},
      {"Reserved", 1, RES_BINARY, false, 0, ""},
      {"Device Class", 7, RES_LOOKUP, false, LOOKUP_DEVICE_CLASS, ""},
      {"System Instance", 4, 1, false, 0, "ISO Device Class Instance"},
      {"Industry Code", 3, RES_LOOKUP, false, LOOKUP_INDUSTRY_CODE, ""},
      {"Reserved", 1, RES_BINARY, false, 0, "ISO Self Configurable"},
      {"New Source Address", BYTES(1), 1, false, 0, ""},
      {0}}}

    /* proprietary PDU2 (non addressed) single-frame range 0xFF00 to 0xFFFF (65280 - 65535) */

    ,
    {"Furuno: Heave",
     65280,
     PACKET_INCOMPLETE,
     PACKET_SINGLE,
     0x08,
     0,
     {{"Manufacturer Code", 11, RES_MANUFACTURER, false, "=1855", "Furuno"},
      {"Reserved", 2, RES_NOTUSED, false, 0, ""},
      {"Industry Code", 3, RES_LOOKUP, false, "=4", "Marine Industry"},
      {"Heave", BYTES(4), 0.001, true, "m", ""},
      {"Reserved", BYTES(2), RES_NOTUSED, false, 0, ""},
      {0}}}

    ,
    {"Manufacturer Proprietary single-frame non-addressed",
     65280,
     PACKET_INCOMPLETE,
     PACKET_SINGLE,
     8,
     0,
     {{"Manufacturer Code", 11, RES_MANUFACTURER, false, 0, ""},
      {"Reserved", 2, RES_NOTUSED, false, 0, ""},
      {"Industry Code", 3, RES_LOOKUP, false, LOOKUP_INDUSTRY_CODE, ""},
      {"Data", BYTES(6), RES_BINARY, false, 0, ""},
      {0}},
     0,
     0,
     true}

    ,
    {"Maretron: Proprietary DC Breaker Current",
     65284,
     PACKET_COMPLETE,
     PACKET_SINGLE,
     8,
     0,
     {{"Manufacturer Code", 11, RES_MANUFACTURER, false, "=137", "Maretron"},
      {"Reserved", 2, RES_NOTUSED, false, 0, ""},
      {"Industry Code", 3, RES_LOOKUP, false, "=4", "Marine Industry"},
      {"Bank Instance", BYTES(1), 1, false, 0, ""},
      {"Indicator Number", BYTES(1), 1, false, 0, ""},
      {"Breaker Current", BYTES(2), 0.1, true, "A", ""},
      {"Reserved", BYTES(2), 1, false, 0, ""},
      {0}}}

    /* http://www.airmartechnology.com/uploads/installguide/DST200UserlManual.pdf */
    ,
    {"Airmar: Boot State Acknowledgment",
     65285,
     PACKET_COMPLETE,
     PACKET_SINGLE,
     8,
     0,
     {{"Manufacturer Code", 11, RES_MANUFACTURER, false, "=135", "Airmar"},
      {"Reserved", 2, RES_NOTUSED, false, 0, ""},
      {"Industry Code", 3, RES_LOOKUP, false, "=4", "Marine Industry"},
      {"Boot State", 4, RES_LOOKUP, false, ",0=in Startup Monitor,1=running Bootloader,2=running Application", ""},
      {0}}}

    ,
    {"Lowrance: Temperature",
     65285,
     PACKET_COMPLETE,
     PACKET_SINGLE,
     8,
     0,
     {{"Manufacturer Code", 11, RES_MANUFACTURER, false, "=140", "Lowrance"},
      {"Reserved", 2, RES_NOTUSED, false, 0, ""},
      {"Industry Code", 3, RES_LOOKUP, false, "=4", "Marine Industry"},
      {"Temperature Source", BYTES(1), RES_LOOKUP, false, LOOKUP_TEMPERATURE_SOURCE, ""},
      {"Actual Temperature", BYTES(2), RES_TEMPERATURE, false, "K", ""},
      {0}}}

    ,
    {"Chetco: Dimmer",
     65286,
     PACKET_INCOMPLETE_LOOKUP,
     PACKET_SINGLE,
     8,
     0,
     {{"Manufacturer Code", 11, RES_MANUFACTURER, false, "=409", "Chetco"},
      {"Reserved", 2, RES_NOTUSED, false, 0, ""},
      {"Industry Code", 3, 1, false, 0, ""},
      {"Instance", BYTES(1), 1, false, 0, ""},
      {"Dimmer1", BYTES(1), 1, false, 0, ""},
      {"Dimmer2", BYTES(1), 1, false, 0, ""},
      {"Dimmer3", BYTES(1), 1, false, 0, ""},
      {"Dimmer4", BYTES(1), 1, false, 0, ""},
      {"Control", BYTES(1), 1, false, 0, ""},
      {0}}}

    /* http://www.airmartechnology.com/uploads/installguide/DST200UserlManual.pdf */
    ,
    {"Airmar: Boot State Request",
     65286,
     PACKET_COMPLETE,
     PACKET_SINGLE,
     8,
     0,
     {{"Manufacturer Code", 11, RES_MANUFACTURER, false, "=135", "Airmar"},
      {"Reserved", 2, RES_NOTUSED, false, 0, ""},
      {"Industry Code", 3, RES_LOOKUP, false, "=4", "Marine Industry"},
      {0}}}

    /* http://www.airmartechnology.com/uploads/installguide/DST200UserlManual.pdf */
    ,
    {"Airmar: Access Level",
     65287,
     PACKET_COMPLETE,
     PACKET_SINGLE,
     8,
     0,
     {{"Manufacturer Code", 11, RES_MANUFACTURER, false, "=135", "Airmar"},
      {"Reserved", 2, RES_NOTUSED, false, 0, ""},
      {"Industry Code", 3, RES_LOOKUP, false, "=4", "Marine Industry"},
      {"Format Code", 3, RES_LOOKUP, false, ",1=Format code 1", ""},
      {"Access Level", 3, RES_LOOKUP, false, ",0=Locked,1=unlocked level 1,2=unlocked level 2", ""},
      {"Reserved", 2, RES_BINARY, false, 0, ""},
      {"Access Seed/Key",
       BYTES(4),
       RES_INTEGER,
       false,
       0,
       "When transmitted, it provides a seed for an unlock operation. It is used to provide the key during PGN 126208."},
      {0}}}

    ,
    {"Simnet: Configure Temperature Sensor",
     65287,
     PACKET_INCOMPLETE,
     PACKET_SINGLE,
     0x08,
     0,
     {{"Manufacturer Code", 11, RES_MANUFACTURER, false, "=1857", "Simrad"},
      {"Reserved", 2, RES_NOTUSED, false, 0, ""},
      {"Industry Code", 3, RES_LOOKUP, false, "=4", "Marine Industry"},
      {0}}}

    ,
    {"Seatalk: Alarm",
     65288,
     PACKET_INCOMPLETE,
     PACKET_SINGLE,
     0x08,
     0,
     {{"Manufacturer Code", 11, RES_MANUFACTURER, false, "=1851", "Raymarine"},
      {"Reserved", 2, RES_NOTUSED, false, 0, ""},
      {"Industry Code", 3, RES_LOOKUP, false, "=4", "Marine Industry"},
      {"SID", BYTES(1), RES_BINARY, false, 0, ""},
      {"Alarm Status", BYTES(1), RES_LOOKUP, false, LOOKUP_SEATALK_ALARM_STATUS, ""},
      {"Alarm ID", BYTES(1), RES_LOOKUP, false, LOOKUP_SEATALK_ALARM_ID, ""},
      {"Alarm Group", BYTES(1), RES_LOOKUP, false, LOOKUP_SEATALK_ALARM_GROUP, ""},
      {"Alarm Priority", BYTES(2), RES_BINARY, false, 0, ""},
      {0}}},
    {"Simnet: Trim Tab Sensor Calibration",
     65289,
     PACKET_INCOMPLETE,
     PACKET_SINGLE,
     0x08,
     0,
     {{"Manufacturer Code", 11, RES_MANUFACTURER, false, "=1857", "Simrad"},
      {"Reserved", 2, RES_NOTUSED, false, 0, ""},
      {"Industry Code", 3, RES_LOOKUP, false, "=4", "Marine Industry"},
      {0}}}

    ,
    {"Simnet: Paddle Wheel Speed Configuration",
     65290,
     PACKET_INCOMPLETE,
     PACKET_SINGLE,
     0x08,
     0,
     {{"Manufacturer Code", 11, RES_MANUFACTURER, false, "=1857", "Simrad"},
      {"Reserved", 2, RES_NOTUSED, false, 0, ""},
      {"Industry Code", 3, RES_LOOKUP, false, "=4", "Marine Industry"},
      {0}}}

    ,
    {"Simnet: Clear Fluid Level Warnings",
     65292,
     PACKET_INCOMPLETE,
     PACKET_SINGLE,
     0x08,
     0,
     {{"Manufacturer Code", 11, RES_MANUFACTURER, false, "=1857", "Simrad"},
      {"Reserved", 2, RES_NOTUSED, false, 0, ""},
      {"Industry Code", 3, RES_LOOKUP, false, "=4", "Marine Industry"},
      {0}}}

    ,
    {"Simnet: LGC-2000 Configuration",
     65293,
     PACKET_INCOMPLETE,
     PACKET_SINGLE,
     0x08,
     0,
     {{"Manufacturer Code", 11, RES_MANUFACTURER, false, "=1857", "Simrad"},
      {"Reserved", 2, RES_NOTUSED, false, 0, ""},
      {"Industry Code", 3, RES_LOOKUP, false, "=4", "Marine Industry"},
      {0}}}

    ,
    {"Navico: Wireless Battery Status",
     65309,
     PACKET_INCOMPLETE,
     PACKET_SINGLE,
     0x08,
     0,
     {{"Manufacturer Code", 11, RES_MANUFACTURER, false, "=275", "Navico"},
      {"Reserved", 2, RES_NOTUSED, false, 0, ""},
      {"Industry Code", 3, RES_LOOKUP, false, "=4", "Marine Industry"},
      {"Status", BYTES(1), 1, false, 0, ""},
      {"Battery Status", BYTES(1), 1, false, "%", ""},
      {"Battery Charge Status", BYTES(1), 1, false, "%", ""},
      {"Reserved", BYTES(3), 1, false, 0, ""},
      {0}}}

    ,
    {"Navico: Wireless Signal Status",
     65312,
     PACKET_FIELDS_UNKNOWN,
     PACKET_SINGLE,
     0x08,
     0,
     {{"Manufacturer Code", 11, RES_MANUFACTURER, false, "=275", "Navico"},
      {"Reserved", 2, RES_NOTUSED, false, 0, ""},
      {"Industry Code", 3, RES_LOOKUP, false, "=4", "Marine Industry"},
      {"Unknown", BYTES(1), 1, false, 0, ""},
      {"Signal Strength", BYTES(1), 1, false, "%", ""},
      {"Reserved", BYTES(3), 1, false, 0, ""},
      {0}}}

    ,
    {"Simnet: Reprogram Status",
     65325,
     PACKET_INCOMPLETE,
     PACKET_SINGLE,
     0x08,
     0,
     {{"Manufacturer Code", 11, RES_MANUFACTURER, false, "=1857", "Simrad"},
      {"Reserved", 2, RES_NOTUSED, false, 0, ""},
      {"Industry Code", 3, RES_LOOKUP, false, "=4", "Marine Industry"},
      {0}}}

    ,
    {"Simnet: Autopilot Mode",
     65341,
     PACKET_INCOMPLETE,
     PACKET_SINGLE,
     0x08,
     0,
     {{"Manufacturer Code", 11, RES_MANUFACTURER, false, "=1857", "Simrad"},
      {"Reserved", 2, RES_NOTUSED, false, 0, ""},
      {"Industry Code", 3, RES_LOOKUP, false, "=4", "Marine Industry"},
      {0}}}

    ,
    {"Seatalk: Pilot Wind Datum",
     65345,
     PACKET_INCOMPLETE,
     PACKET_SINGLE,
     0x08,
     0,
     {{"Manufacturer Code", 11, RES_MANUFACTURER, false, "=1851", "Raymarine"},
      {"Reserved", 2, RES_NOTUSED, false, 0, ""},
      {"Industry Code", 3, RES_LOOKUP, false, "=4", "Marine Industry"},
      {"Wind Datum", BYTES(2), RES_RADIANS, false, "rad", ""},
      {"Rolling Average Wind Angle", BYTES(2), RES_RADIANS, false, "rad", ""},
      {"Reserved", BYTES(2), 1, false, 0, ""},
      {0}}},
    {"Seatalk: Pilot Heading",
     65359,
     PACKET_INCOMPLETE,
     PACKET_SINGLE,
     0x08,
     0,
     {{"Manufacturer Code", 11, RES_MANUFACTURER, false, "=1851", "Raymarine"},
      {"Reserved", 2, RES_NOTUSED, false, 0, ""},
      {"Industry Code", 3, RES_LOOKUP, false, "=4", "Marine Industry"},
      {"SID", BYTES(1), RES_BINARY, false, 0, ""},
      {"Heading True", BYTES(2), RES_RADIANS, false, "rad", ""},
      {"Heading Magnetic", BYTES(2), RES_RADIANS, false, "rad", ""},
      {"Reserved", BYTES(1), 1, false, 0, ""},
      {0}}}

    ,
    {"Seatalk: Pilot Locked Heading",
     65360,
     PACKET_COMPLETE,
     PACKET_SINGLE,
     0x08,
     0,
     {{"Manufacturer Code", 11, RES_MANUFACTURER, false, "=1851", "Raymarine"},
      {"Reserved", 2, RES_NOTUSED, false, 0, ""},
      {"Industry Code", 3, RES_LOOKUP, false, "=4", "Marine Industry"},
      {"SID", BYTES(1), RES_BINARY, false, 0, ""},
      {"Target Heading True", BYTES(2), RES_RADIANS, false, "rad", ""},
      {"Target Heading Magnetic", BYTES(2), RES_RADIANS, false, "rad", ""},
      {"Reserved", BYTES(1), RES_BINARY, false, 0, ""},
      {0}}}

    ,
    {"Seatalk: Silence Alarm",
     65361,
     PACKET_COMPLETE,
     PACKET_SINGLE,
     0x08,
     0,
     {{"Manufacturer Code", 11, RES_MANUFACTURER, false, "=1851", "Raymarine"},
      {"Reserved", 2, RES_NOTUSED, false, 0, ""},
      {"Industry Code", 3, RES_LOOKUP, false, "=4", "Marine Industry"},
      {"Alarm ID", BYTES(1), RES_LOOKUP, false, LOOKUP_SEATALK_ALARM_ID, ""},
      {"Alarm Group", BYTES(1), RES_LOOKUP, false, LOOKUP_SEATALK_ALARM_GROUP, ""},
      {"Reserved", 32, RES_BINARY, false, 0, ""},
      {0}}}

    ,
    {"Seatalk: Keypad Message",
     65371,
     PACKET_INCOMPLETE,
     PACKET_SINGLE,
     0x08,
     0,
     {{"Manufacturer Code", 11, RES_MANUFACTURER, false, "=1851", "Raymarine"},
      {"Reserved", 2, RES_NOTUSED, false, 0, ""},
      {"Industry Code", 3, RES_LOOKUP, false, "=4", "Marine Industry"},
      {"Proprietary ID", BYTES(1), 1, false, 0, ""},
      {"First key", BYTES(1), 1, false, 0, ""},
      {"Second key", BYTES(1), 1, false, 0, ""},
      {"First key state", 2, 1, false, 0, ""},
      {"Second key state", 2, 1, false, 0, ""},
      {"Reserved", 4, RES_BINARY, false, 0, ""},
      {"Encoder Position", BYTES(1), 1, false, 0, ""},
      {0}}}

    ,
    {"SeaTalk: Keypad Heartbeat",
     65374,
     PACKET_INCOMPLETE,
     PACKET_SINGLE,
     0x08,
     0,
     {{"Manufacturer Code", 11, RES_MANUFACTURER, false, "=1851", "Raymarine"},
      {"Reserved", 2, RES_NOTUSED, false, 0, ""},
      {"Industry Code", 3, RES_LOOKUP, false, "=4", "Marine Industry"},
      {"Proprietary ID", BYTES(1), 1, false, 0, ""},
      {"Variant", BYTES(1), 1, false, 0, ""},
      {"Status", BYTES(1), 1, false, 0, ""},
      {0}}}

    ,
    {"Seatalk: Pilot Mode",
     65379,
     PACKET_INCOMPLETE,
     PACKET_SINGLE,
     0x08,
     0,
     {{"Manufacturer Code", 11, RES_MANUFACTURER, false, "=1851", "Raymarine"},
      {"Reserved", 2, RES_NOTUSED, false, 0, ""},
      {"Industry Code", 3, RES_LOOKUP, false, "=4", "Marine Industry"},
      {"Pilot Mode", BYTES(1), RES_BINARY, false, 0, ""},
      {"Sub Mode", BYTES(1), RES_BINARY, false, 0, ""},
      {"Pilot Mode Data", BYTES(1), RES_BINARY, false, 0, ""},
      {"Reserved", BYTES(3), RES_BINARY, false, 0, ""},
      {0}}}
    /* http://www.airmartechnology.com/uploads/installguide/DST200UserlManual.pdf */
    ,
    {"Airmar: Depth Quality Factor",
     65408,
     PACKET_INCOMPLETE,
     PACKET_SINGLE,
     8,
     0,
     {{"Manufacturer Code", 11, RES_MANUFACTURER, false, "=135", "Airmar"},
      {"Reserved", 2, RES_NOTUSED, false, 0, ""},
      {"Industry Code", 3, RES_LOOKUP, false, "=4", "Marine Industry"},
      {"SID", BYTES(1), 1, false, 0, ""},
      {"Depth Quality Factor", 4, RES_LOOKUP, false, ",0=No Depth Lock", ""},
      {0}}}

    /* http://www.airmartechnology.com/uploads/installguide/DST200UserlManual.pdf */
    ,
    {"Airmar: Speed Pulse Count",
     65409,
     PACKET_COMPLETE,
     PACKET_SINGLE,
     8,
     0,
     {{"Manufacturer Code", 11, RES_MANUFACTURER, false, "=135", "Airmar"},
      {"Reserved", 2, RES_NOTUSED, false, 0, ""},
      {"Industry Code", 3, RES_LOOKUP, false, "=4", "Marine Industry"},
      {"SID", BYTES(1), 1, false, 0, ""},
      {"Duration of interval", BYTES(2), 0.001, false, "s", ""},
      {"Number of pulses received", BYTES(2), 1, false, 0, ""},
      {"Reserved", BYTES(1), RES_NOTUSED, false, 0, ""},
      {0}}}

    /* http://www.airmartechnology.com/uploads/installguide/DST200UserlManual.pdf */
    ,
    {"Airmar: Device Information",
     65410,
     PACKET_INCOMPLETE,
     PACKET_SINGLE,
     8,
     0,
     {{"Manufacturer Code", 11, RES_MANUFACTURER, false, "=135", "Airmar"},
      {"Reserved", 2, RES_NOTUSED, false, 0, ""},
      {"Industry Code", 3, RES_LOOKUP, false, "=4", "Marine Industry"},
      {"SID", BYTES(1), 1, false, 0, ""},
      {"Internal Device Temperature", BYTES(2), RES_TEMPERATURE, false, "K", ""},
      {"Supply Voltage", BYTES(2), 0.01, false, "V", ""},
      {"Reserved", BYTES(1), RES_BINARY, false, 0, ""},
      {0}}}

    ,
    {"Simnet: Autopilot Mode",
     65480,
     PACKET_INCOMPLETE,
     PACKET_SINGLE,
     0x08,
     0,
     {{"Manufacturer Code", 11, RES_MANUFACTURER, false, "=1857", "Simrad"},
      {"Reserved", 2, RES_NOTUSED, false, 0, ""},
      {"Industry Code", 3, RES_LOOKUP, false, "=4", "Marine Industry"},
      {0}}}

    /* PDU1 (addressed) fast-packet PGN range 0x10000 to 0x1EEFF (65536 - 126719) */
    ,
    {"Unknown fast-packet addressed",
     65536,
     PACKET_INCOMPLETE_LOOKUP,
     PACKET_FAST,
     255,
     0,
     {{"Data", BYTES(255), RES_BINARY, false, 0, ""}, {0}},
     0,
     0,
     true}

    /* http://www.maretron.com/support/manuals/DST100UM_1.2.pdf */
    /* http://www.nmea.org/Assets/20140109%20nmea-2000-corrigendum-tc201401031%20pgn%20126208.pdf */
    ,
    {"NMEA - Request group function",
     126208,
     PACKET_COMPLETE,
     PACKET_FAST,
     12,
     2,
     {{"Function Code", BYTES(1), RES_INTEGER, false, "=0", "Request"},
      {"PGN", BYTES(3), RES_INTEGER, false, 0, "Requested PGN"},
      {"Transmission interval", BYTES(4), 0.001, false, "s", ""},
      {"Transmission interval offset", BYTES(2), 0.01, false, "s", ""},
      {"# of Parameters", BYTES(1), 1, false, 0, "How many parameter pairs will follow"},
      {"Parameter", BYTES(1), RES_INTEGER, false, 0, "Parameter index"},
      {"Value", LEN_VARIABLE, RES_VARIABLE, false, 0, "Parameter value, variable length"},
      {0}}}

    ,
    {"NMEA - Command group function",
     126208,
     PACKET_COMPLETE,
     PACKET_FAST,
     8,
     2,
     {{"Function Code", BYTES(1), RES_INTEGER, false, "=1", "Command"},
      {"PGN", BYTES(3), RES_INTEGER, false, 0, "Commanded PGN"},
      {"Priority", 4, RES_LOOKUP, 0, ",8=Leave priority unchanged,9=Reset to default"},
      {"Reserved", 4, RES_BINARY, false, 0, ""},
      {"# of Parameters", BYTES(1), 1, false, 0, "How many parameter pairs will follow"},
      {"Parameter", BYTES(1), RES_INTEGER, false, 0, "Parameter index"},
      {"Value", LEN_VARIABLE, RES_VARIABLE, false, 0, "Parameter value, variable length"},
      {0}}}

    ,
    {"NMEA - Acknowledge group function",
     126208,
     PACKET_COMPLETE,
     PACKET_FAST,
     8,
     1,
     {{"Function Code", BYTES(1), RES_INTEGER, false, "=2", "Acknowledge"},
      {"PGN", 24, RES_INTEGER, false, 0, "Commanded PGN"},
      {"PGN error code",
       4,
       RES_LOOKUP,
       false,
       ",0=Acknowledge,1=PGN not supported,2=PGN not available,3=Access denied,4=Not supported,5=Tag not supported,6=Read or Write "
       "not supported",
       ""},
      {"Transmission interval/Priority error code",
       4,
       RES_LOOKUP,
       false,
       ",0=Acknowledge,1=Transmit Interval/Priority not supported,2=Transmit Interval to low,3=Access denied,4=Not supported",
       ""},
      {"# of Parameters", 8, 1, false, 0, ""},
      {"Parameter",
       4,
       RES_LOOKUP,
       false,
       ",0=Acknowledge,1=Invalid parameter field,2=Temporary error,3=Parameter out of range,4=Access denied,5=Not supported,6=Read "
       "or Write not supported",
       ""},
      {0}}}

    ,
    {"NMEA - Read Fields group function",
     126208,
     PACKET_INCOMPLETE,
     PACKET_FAST,
     8,
     102,
     {{"Function Code", BYTES(1), RES_INTEGER, false, "=3", "Read Fields"},
      {"PGN", 24, RES_INTEGER, false, 0, "Commanded PGN"},
      {"Manufacturer Code", 11, RES_MANUFACTURER, false, 0, ""} // TODO: Only in PGN when field PGN is proprietary. Sigh.
      ,
      {"Reserved", 2, RES_NOTUSED, false, 0, ""} // TODO: Only in PGN when field PGN is proprietary. Sigh.
      ,
      {"Industry Code", 3, RES_LOOKUP, false, LOOKUP_INDUSTRY_CODE, ""} // TODO: Only in PGN when field PGN is proprietary. Sigh.
      ,
      {"Unique ID", 8, RES_INTEGER, false, 0, ""},
      {"# of Selection Pairs", 8, 1, false, 0, ""},
      {"# of Parameters", 8, 1, false, 0, ""},
      {"Selection Parameter", BYTES(1), RES_INTEGER, false, 0, ""},
      {"Selection Value", LEN_VARIABLE, RES_VARIABLE, false, 0, ""},
      {"Parameter", BYTES(1), RES_INTEGER, false, 0, ""},
      {0}}}

    /* The following won't work when analyzing non-proprietary PGNs */
    ,
    {"NMEA - Read Fields reply group function",
     126208,
     PACKET_COMPLETE,
     PACKET_FAST,
     8,
     202,
     {{"Function Code", BYTES(1), RES_INTEGER, false, "=4", "Read Fields Reply"},
      {"PGN", 24, RES_INTEGER, false, 0, "Commanded PGN"},
      {"Manufacturer Code",
       11,
       RES_MANUFACTURER,
       false,
       0,
       "Only for proprietary PGNs"} // TODO: Only in PGN when field PGN is proprietary. Sigh.
      ,
      {"Reserved", 2, RES_NOTUSED, false, 0, "Only for proprietary PGNs"} // TODO: Only in PGN when field PGN is proprietary. Sigh.
      ,
      {"Industry Code",
       3,
       RES_LOOKUP,
       false,
       LOOKUP_INDUSTRY_CODE,
       "Only for proprietary PGNs"} // TODO: Only in PGN when field PGN is proprietary. Sigh.
      ,
      {"Unique ID", 8, RES_INTEGER, false, 0, ""},
      {"# of Selection Pairs", 8, 1, false, 0, ""},
      {"# of Parameters", 8, 1, false, 0, ""},
      {"Selection Parameter", BYTES(1), RES_INTEGER, false, 0, ""},
      {"Selection Value", LEN_VARIABLE, RES_VARIABLE, false, 0, ""},
      {"Parameter", BYTES(1), RES_INTEGER, false, 0, ""},
      {"Value", LEN_VARIABLE, RES_VARIABLE, false, 0, ""},
      {0}}}

    /* The following won't work when analyzing non-proprietary PGNs */
    ,
    {"NMEA - Write Fields group function",
     126208,
     PACKET_COMPLETE,
     PACKET_FAST,
     8,
     202,
     {{"Function Code", BYTES(1), RES_INTEGER, false, "=5", "Write Fields"},
      {"PGN", 24, RES_INTEGER, false, 0, "Commanded PGN"},
      {"Manufacturer Code",
       11,
       RES_MANUFACTURER,
       false,
       0,
       "Only for proprietary PGNs"} // TODO: Only in PGN when field PGN is proprietary. Sigh.
      ,
      {"Reserved", 2, RES_NOTUSED, false, 0, "Only for proprietary PGNs"} // TODO: Only in PGN when field PGN is proprietary. Sigh.
      ,
      {"Industry Code",
       3,
       RES_LOOKUP,
       false,
       LOOKUP_INDUSTRY_CODE,
       "Only for proprietary PGNs"} // TODO: Only in PGN when field PGN is proprietary. Sigh.
      ,
      {"Unique ID", 8, RES_INTEGER, false, 0, ""},
      {"# of Selection Pairs", 8, 1, false, 0, ""},
      {"# of Parameters", 8, 1, false, 0, ""},
      {"Selection Parameter", BYTES(1), RES_INTEGER, false, 0, ""},
      {"Selection Value", LEN_VARIABLE, RES_VARIABLE, false, 0, ""},
      {"Parameter", BYTES(1), RES_INTEGER, false, 0, ""},
      {"Value", LEN_VARIABLE, RES_VARIABLE, false, 0, ""},
      {0}}}

    /* The following won't work when analyzing non-proprietary PGNs */
    ,
    {"NMEA - Write Fields reply group function",
     126208,
     PACKET_COMPLETE,
     PACKET_FAST,
     8,
     202,
     {{"Function Code", BYTES(1), RES_INTEGER, false, "=6", "Write Fields Reply"},
      {"PGN", 24, RES_INTEGER, false, 0, "Commanded PGN"},
      {"Manufacturer Code",
       11,
       RES_MANUFACTURER,
       false,
       0,
       "Only for proprietary PGNs"} // TODO: Only in PGN when field PGN is proprietary. Sigh.
      ,
      {"Reserved", 2, RES_NOTUSED, false, 0, "Only for proprietary PGNs"} // TODO: Only in PGN when field PGN is proprietary. Sigh.
      ,
      {"Industry Code",
       3,
       RES_LOOKUP,
       false,
       LOOKUP_INDUSTRY_CODE,
       "Only for proprietary PGNs"} // TODO: Only in PGN when field PGN is proprietary. Sigh.
      ,
      {"Unique ID", 8, RES_INTEGER, false, 0, ""},
      {"# of Selection Pairs", 8, 1, false, 0, ""},
      {"# of Parameters", 8, 1, false, 0, ""},
      {"Selection Parameter", BYTES(1), RES_INTEGER, false, 0, ""},
      {"Selection Value", LEN_VARIABLE, RES_VARIABLE, false, 0, ""},
      {"Parameter", BYTES(1), RES_INTEGER, false, 0, ""},
      {"Value", LEN_VARIABLE, RES_VARIABLE, false, 0, ""},
      {0}}}

    /************ RESPONSE TO REQUEST PGNS **************/

    ,
    {"PGN List (Transmit and Receive)",
     126464,
     PACKET_COMPLETE,
     PACKET_FAST,
     8,
     1,
     {{"Function Code",
       BYTES(1),
       RES_LOOKUP,
       false,
       ",0=Transmit PGN list,1=Receive PGN list",
       "Transmit or receive PGN Group Function Code"},
      {
          "PGN",
          24,
          RES_INTEGER,
          false,
          0,
      },
      {0}}}

    /* proprietary PDU1 (addressed) fast-packet PGN range 0x1EF00 to 0x1EFFF (126720 - 126975) */

    ,
    {"Seatalk1: Pilot Mode",
     126720,
     PACKET_INCOMPLETE,
     PACKET_FAST,
     21,
     0,
     {{"Manufacturer Code", 11, RES_MANUFACTURER, false, "=1851", "Raymarine"},
      {"Reserved", 2, RES_NOTUSED, false, 0, ""},
      {"Industry Code", 3, RES_LOOKUP, false, "=4", "Marine Industry"},
      {"Proprietary ID", BYTES(2), RES_INTEGER, false, "=33264", "0x81f0"},
      {"command", BYTES(1), RES_INTEGER, false, "=132", "0x84"},
      {"Unknown 1", BYTES(3), RES_BINARY, false, 0, ""},
      {"Pilot Mode", BYTES(1), RES_INTEGER, false, ",64=Standby,66=Auto,70=Wind,74=Track", ""},
      {"Sub Mode", BYTES(1), RES_INTEGER, false, 0, ""},
      {"Pilot Mode Data", BYTES(1), RES_BINARY, false, 0, ""},
      {"Unknown 2", BYTES(10), RES_BINARY, false, 0, ""},
      {0}}}

    ,
    {"Fusion: Media Control",
     126720,
     PACKET_INCOMPLETE,
     PACKET_FAST,
     6,
     0,
     {{"Manufacturer Code", 11, RES_MANUFACTURER, false, "=419", "Fusion"},
      {"Reserved", 2, RES_NOTUSED, false, 0, ""},
      {"Industry Code", 3, RES_LOOKUP, false, "=4", "Marine Industry"},
      {"Proprietary ID", BYTES(1), RES_INTEGER, false, "=3", "Media Control"},
      {"Unknown", BYTES(1), RES_INTEGER, false, 0, ""},
      {"Source ID", BYTES(1), RES_INTEGER, false, 0, ""},
      {"Command", BYTES(1), RES_LOOKUP, false, ",1=Play,2=Pause,4=Next,6=Prev", ""},
      {0}}}

    ,
    {"Fusion: Sirius Control",
     126720,
     PACKET_INCOMPLETE,
     PACKET_FAST,
     7,
     0,
     {{"Manufacturer Code", 11, RES_MANUFACTURER, false, "=419", "Fusion"},
      {"Reserved", 2, RES_NOTUSED, false, 0, ""},
      {"Industry Code", 3, RES_LOOKUP, false, "=4", "Marine Industry"},
      {"Proprietary ID", BYTES(1), RES_INTEGER, false, "=30", "Sirius Control"},
      {"Unknown", BYTES(1), RES_INTEGER, false, 0, ""},
      {"Source ID", BYTES(1), RES_INTEGER, false, 0, ""},
      {"Command", BYTES(1), RES_LOOKUP, false, ",1=Next,2=Prev", ""},
      {0}}}

    ,
    {"Fusion: Request Status",
     126720,
     PACKET_INCOMPLETE,
     PACKET_FAST,
     3,
     0,
     {{"Manufacturer Code", 11, RES_MANUFACTURER, false, "=419", "Fusion"},
      {"Reserved", 2, RES_NOTUSED, false, 0, ""},
      {"Industry Code", 3, RES_LOOKUP, false, "=4", "Marine Industry"},
      {"Proprietary ID", BYTES(1), RES_INTEGER, false, "=1", "Request Status"},
      {"Unknown", BYTES(1), RES_INTEGER, false, 0, ""},
      {0}}}

    ,
    {"Fusion: Set Source",
     126720,
     PACKET_INCOMPLETE,
     PACKET_FAST,
     3,
     0,
     {{"Manufacturer Code", 11, RES_MANUFACTURER, false, "=419", "Fusion"},
      {"Reserved", 2, RES_NOTUSED, false, 0, ""},
      {"Industry Code", 3, RES_LOOKUP, false, "=4", "Marine Industry"},
      {"Proprietary ID", BYTES(1), RES_INTEGER, false, "=2", "Set Source"},
      {"Unknown", BYTES(1), RES_INTEGER, false, 0, ""},
      {"Source ID", BYTES(1), RES_INTEGER, false, 0, ""},
      {0}}}

    ,
    {"Fusion: Mute",
     126720,
     PACKET_INCOMPLETE,
     PACKET_FAST,
     3,
     0,
     {{"Manufacturer Code", 11, RES_MANUFACTURER, false, "=419", "Fusion"},
      {"Reserved", 2, RES_NOTUSED, false, 0, ""},
      {"Industry Code", 3, RES_LOOKUP, false, "=4", "Marine Industry"},
      {"Proprietary ID", BYTES(1), RES_INTEGER, false, "=17", "Mute"},
      {"Command", BYTES(1), RES_LOOKUP, false, ",1=Mute On,2=Mute Off", ""},
      {0}}}

    ,
    {"Fusion: Set Zone Volume",
     126720,
     PACKET_INCOMPLETE,
     PACKET_FAST,
     6,
     0,
     {{"Manufacturer Code", 11, RES_MANUFACTURER, false, "=419", "Fusion"},
      {"Reserved", 2, RES_NOTUSED, false, 0, ""},
      {"Industry Code", 3, RES_LOOKUP, false, "=4", "Marine Industry"},
      {"Proprietary ID", BYTES(1), RES_INTEGER, false, "=24", "Set Zone Volume"},
      {"Unknown", BYTES(1), RES_INTEGER, false, 0, ""},
      {"Zone", BYTES(1), RES_INTEGER, false, 0, ""},
      {"Volume", BYTES(1), RES_INTEGER, false, 0, ""},
      {0}}}

    ,
    {"Fusion: Set All Volumes",
     126720,
     PACKET_INCOMPLETE,
     PACKET_FAST,
     9,
     0,
     {{"Manufacturer Code", 11, RES_MANUFACTURER, false, "=419", "Fusion"},
      {"Reserved", 2, RES_NOTUSED, false, 0, ""},
      {"Industry Code", 3, RES_LOOKUP, false, "=4", "Marine Industry"},
      {"Proprietary ID", BYTES(1), RES_INTEGER, false, "=25", "Set All Volumes"},
      {"Unknown", BYTES(1), RES_INTEGER, false, 0, ""},
      {"Zone1", BYTES(1), RES_INTEGER, false, 0, ""},
      {"Zone2", BYTES(1), RES_INTEGER, false, 0, ""},
      {"Zone3", BYTES(1), RES_INTEGER, false, 0, ""},
      {"Zone4", BYTES(1), RES_INTEGER, false, 0, ""},
      {0}}}

    /* Seatalk1 code from http://thomasknauf.de/rap/seatalk2.htm */
    ,
    {"Seatalk1: Keystroke",
     126720,
     PACKET_INCOMPLETE,
     PACKET_FAST,
     21,
     0,
     {{"Manufacturer Code", 11, RES_MANUFACTURER, false, "=1851", "Raymarine"},
      {"Reserved", 2, RES_NOTUSED, false, 0, ""},
      {"Industry Code", 3, RES_LOOKUP, false, "=4", "Marine Industry"},
      {"Proprietary ID", BYTES(2), RES_INTEGER, false, "=33264", "0x81f0"},
      {"command", BYTES(1), RES_BINARY, false, "=134", "0x86"},
      {"device", BYTES(1), RES_LOOKUP, false, ",33=S100", ""},
      {"key",
       BYTES(2),
       RES_LOOKUP,
       false,
       ",64005=-1,63495=+1,64770=Standby,65025=Auto,64515=Wind,56355=Track,63240=+10,63750=-10,56865=-1 and -10,56610=+1 and +10",
       ""},
      {"Unknown data",
       BYTES(14),
       RES_BINARY,
       false,
       0,
       ""} // xx xx xx xx xx c1 c2 cd 64 80 d3 42 f1 c8 (if xx=0xff =>working or xx xx xx xx xx = [A5 FF FF FF FF | 00 00 00 FF FF |
           // FF FF FF FF FF | 42 00 F8 02 05])
      ,
      {0}}}

    ,
    {"Seatalk1: Device Identification",
     126720,
     PACKET_INCOMPLETE,
     PACKET_FAST,
     8,
     0,
     {{"Manufacturer Code", 11, RES_MANUFACTURER, false, "=1851", "Raymarine"},
      {"Reserved", 2, RES_NOTUSED, false, 0, ""},
      {"Industry Code", 3, RES_LOOKUP, false, "=4", "Marine Industry"},
      {"Proprietary ID", BYTES(2), RES_INTEGER, false, "=33264", "0x81f0"},
      {"command", BYTES(1), RES_BINARY, false, "=144", "0x90"},
      {"Reserved", BYTES(1), RES_NOTUSED, false, 0, ""} // 0x00
      ,
      {"device", BYTES(1), RES_LOOKUP, false, ",3=S100,5=Course Computer", ""},
      {0}}}

    /* http://www.airmartechnology.com/uploads/installguide/PB200UserManual.pdf */
    ,
    {"Airmar: Attitude Offset",
     126720,
     PACKET_COMPLETE,
     PACKET_FAST,
     9,
     0,
     {{"Manufacturer Code", 11, RES_MANUFACTURER, false, "=135", "Airmar"},
      {"Reserved", 2, RES_NOTUSED, false, 0, ""},
      {"Industry Code", 3, RES_LOOKUP, false, "=4", "Marine Industry"},
      {"Proprietary ID", BYTES(1), RES_INTEGER, false, "=32", "Attitude Offsets"},
      {"Azimuth offset",
       BYTES(2),
       RES_RADIANS,
       true,
       "rad",
       "Positive: sensor rotated to port, negative: sensor rotated to starboard"},
      {"Pitch offset", BYTES(2), RES_RADIANS, true, "rad", "Positive: sensor tilted to bow, negative: sensor tilted to stern"},
      {"Roll offset", BYTES(2), RES_RADIANS, true, "rad", "Positive: sensor tilted to port, negative: sensor tilted to starboard"},
      {0}}}

    /* http://www.airmartechnology.com/uploads/installguide/PB200UserManual.pdf */
    ,
    {"Airmar: Calibrate Compass",
     126720,
     PACKET_FIELDS_UNKNOWN,
     PACKET_FAST,
     24,
     0,
     {{"Manufacturer Code", 11, RES_MANUFACTURER, false, "=135", "Airmar"},
      {"Reserved", 2, RES_NOTUSED, false, 0, ""},
      {"Industry Code", 3, RES_LOOKUP, false, "=4", "Marine Industry"},
      {"Proprietary ID", BYTES(1), RES_INTEGER, false, "=33", "Calibrate Compass"},
      {"Calibrate Function",
       BYTES(1),
       RES_LOOKUP,
       false,
       ",0=Normal/cancel calibration,1=Enter calibration mode,2=Reset calibration to 0,3=Verify,4=Reset compass to "
       "defaults,5=Reset damping to defaults",
       ""},
      {"Calibration Status",
       BYTES(1),
       RES_LOOKUP,
       false,
       ",0=Queried,1=Passed,2=Failed - timeout,3=Failed - tilt error,4=Failed - other,5=In progress",
       ""},
      {"Verify Score", BYTES(1), RES_INTEGER, false, 0, "TBD"},
      {"X-axis gain value", BYTES(2), 0.01, true, 0, "default 100, range 50 to 500"},
      {"Y-axis gain value", BYTES(2), 0.01, true, 0, "default 100, range 50 to 500"},
      {"Z-axis gain value", BYTES(2), 0.01, true, 0, "default 100, range 50 to 500"},
      {"X-axis linear offset", BYTES(2), 0.01, true, "Tesla", "default 0, range -320.00 to 320.00"},
      {"Y-axis linear offset", BYTES(2), 0.01, true, "Tesla", "default 0, range -320.00 to 320.00"},
      {"Z-axis linear offset", BYTES(2), 0.01, true, "Tesla", "default 0, range -320.00 to 320.00"},
      {"X-axis angular offset", BYTES(2), 0.1, true, "deg", "default 0, range 0 to 3600"},
      {"Pitch and Roll damping", BYTES(2), 0.05, true, "s", "default 30, range 0 to 200"},
      {"Compass/Rate gyro damping",
       BYTES(2),
       0.05,
       true,
       "s",
       "default -30, range -2400 to 2400, negative indicates rate gyro is to be used in compass calculations"},
      {0}}}

    /* http://www.airmartechnology.com/uploads/installguide/PB200UserManual.pdf */
    ,
    {"Airmar: True Wind Options",
     126720,
     PACKET_INCOMPLETE,
     PACKET_FAST,
     6,
     0,
     {{"Manufacturer Code", 11, RES_MANUFACTURER, false, "=135", "Airmar"},
      {"Reserved", 2, RES_NOTUSED, false, 0, ""},
      {"Industry Code", 3, RES_LOOKUP, false, "=4", "Marine Industry"},
      {"Proprietary ID", BYTES(1), RES_INTEGER, false, "=34", "True Wind Options"},
      {"COG substitution for HDG",
       2,
       RES_LOOKUP,
       false,
       ",0=Use HDG only,1=Allow COG to replace HDG",
       "Allow use of COG when HDG not available?"},
      {"Calibration Status",
       BYTES(1),
       RES_LOOKUP,
       false,
       ",0=Queried,1=Passed,2=Failed - timeout,3=Failed - tilt error,4=Failed - other,5=In progress",
       ""},
      {"Verify Score", BYTES(1), RES_INTEGER, false, 0, "TBD"},
      {"X-axis gain value", BYTES(2), 0.01, true, 0, "default 100, range 50 to 500"},
      {"Y-axis gain value", BYTES(2), 0.01, true, 0, "default 100, range 50 to 500"},
      {"Z-axis gain value", BYTES(2), 0.01, true, 0, "default 100, range 50 to 500"},
      {"X-axis linear offset", BYTES(2), 0.01, true, "Tesla", "default 0, range -320.00 to 320.00"},
      {"Y-axis linear offset", BYTES(2), 0.01, true, "Tesla", "default 0, range -320.00 to 320.00"},
      {"Z-axis linear offset", BYTES(2), 0.01, true, "Tesla", "default 0, range -320.00 to 320.00"},
      {"X-axis angular offset", BYTES(2), 0.1, true, "deg", "default 0, range 0 to 3600"},
      {"Pitch and Roll damping", BYTES(2), 0.05, true, "s", "default 30, range 0 to 200"},
      {"Compass/Rate gyro damping",
       BYTES(2),
       0.05,
       true,
       "s",
       "default -30, range -2400 to 2400, negative indicates rate gyro is to be used in compass calculations"},
      {0}}}

    /* http://www.airmartechnology.com/uploads/installguide/DST200UserlManual.pdf */
    ,
    {"Airmar: Simulate Mode",
     126720,
     PACKET_COMPLETE,
     PACKET_FAST,
     6,
     0,
     {{"Manufacturer Code", 11, RES_MANUFACTURER, false, "=135", "Airmar"},
      {"Reserved", 2, RES_NOTUSED, false, 0, ""},
      {"Industry Code", 3, RES_LOOKUP, false, "=4", "Marine Industry"},
      {"Proprietary ID", BYTES(1), RES_INTEGER, false, "=35", "Simulate Mode"},
      {"Simulate Mode", 2, RES_LOOKUP, false, LOOKUP_OFF_ON, ""},
      {"Reserved", 22, RES_BINARY, false, 0, "Reserved"},
      {0}}}

    /* http://www.airmartechnology.com/uploads/installguide/DST200UserlManual.pdf */
    ,
    {"Airmar: Calibrate Depth",
     126720,
     PACKET_COMPLETE,
     PACKET_FAST,
     6,
     0,
     {{"Manufacturer Code", 11, RES_MANUFACTURER, false, "=135", "Airmar"},
      {"Reserved", 2, RES_NOTUSED, false, 0, ""},
      {"Industry Code", 3, RES_LOOKUP, false, "=4", "Marine Industry"},
      {"Proprietary ID", BYTES(1), RES_INTEGER, false, "=40", "Calibrate Depth"},
      {"Speed of Sound Mode", BYTES(2), 0.1, false, "m/s", "actual allowed range is 1350.0 to 1650.0 m/s"},
      {"Reserved", 8, RES_BINARY, false, 0, "Reserved"},
      {0}}}

    /* http://www.airmartechnology.com/uploads/installguide/DST200UserlManual.pdf */
    ,
    {"Airmar: Calibrate Speed",
     126720,
     PACKET_COMPLETE,
     PACKET_FAST,
     12,
     2,
     {{"Manufacturer Code", 11, RES_MANUFACTURER, false, "=135", "Airmar"},
      {"Reserved", 2, RES_NOTUSED, false, 0, ""},
      {"Industry Code", 3, RES_LOOKUP, false, "=4", "Marine Industry"},
      {"Proprietary ID", BYTES(1), RES_INTEGER, false, "=41", "Calibrate Speed"},
      {"Number of pairs of data points",
       BYTES(1),
       RES_INTEGER,
       false,
       0,
       "actual range is 0 to 25. 254=restore default speed curve"},
      {"Input frequency", BYTES(2), 0.1, false, "Hz", ""},
      {"Output speed", BYTES(2), 0.01, false, "m/s", ""},
      {0}}}

    /* http://www.airmartechnology.com/uploads/installguide/DST200UserlManual.pdf */
    ,
    {"Airmar: Calibrate Temperature",
     126720,
     PACKET_COMPLETE,
     PACKET_FAST,
     6,
     2,
     {{"Manufacturer Code", 11, RES_MANUFACTURER, false, "=135", "Airmar"},
      {"Reserved", 2, RES_NOTUSED, false, 0, ""},
      {"Industry Code", 3, RES_LOOKUP, false, "=4", "Marine Industry"},
      {"Proprietary ID", BYTES(1), RES_INTEGER, false, "=42", "Calibrate Temperature"},
      {"Temperature instance", 2, RES_LOOKUP, false, ",0=Device Sensor,1=Onboard Water Sensor,2=Optional Water Sensor", ""},
      {"Reserved", 6, RES_BINARY, false, 0, "Reserved"},
      {"Temperature offset", BYTES(2), 0.001, true, "K", "actual range is -9.999 to +9.999 K"},
      {0}}}

    /* http://www.airmartechnology.com/uploads/installguide/DST200UserlManual.pdf */
    ,
    {"Airmar: Speed Filter",
     126720,
     PACKET_COMPLETE,
     PACKET_FAST,
     8,
     2,
     {{"Manufacturer Code", 11, RES_MANUFACTURER, false, "=135", "Airmar"},
      {"Reserved", 2, RES_NOTUSED, false, 0, ""},
      {"Industry Code", 3, RES_LOOKUP, false, "=4", "Marine Industry"},
      {"Proprietary ID", BYTES(1), RES_INTEGER, false, "=43", "Speed Filter"},
      {"Filter type", 4, RES_LOOKUP, false, ",0=no filter,1=basic IIR filter", ""},
      {"Reserved", 4, RES_BINARY, false, 0, "Reserved"},
      {"Sample interval", BYTES(2), 0.01, false, "s", ""},
      {"Filter duration", BYTES(2), 0.01, false, "s", ""},
      {0}}}

    /* http://www.airmartechnology.com/uploads/installguide/DST200UserlManual.pdf */
    ,
    {"Airmar: Temperature Filter",
     126720,
     PACKET_COMPLETE,
     PACKET_FAST,
     8,
     2,
     {{"Manufacturer Code", 11, RES_MANUFACTURER, false, "=135", "Airmar"},
      {"Reserved", 2, RES_NOTUSED, false, 0, ""},
      {"Industry Code", 3, RES_LOOKUP, false, "=4", "Marine Industry"},
      {"Proprietary ID", BYTES(1), RES_INTEGER, false, "=44", "Temperature Filter"},
      {"Filter type", 4, RES_LOOKUP, false, ",0=no filter,1=basic IIR filter,15=data not available", ""},
      {"Reserved", 4, RES_BINARY, false, 0, "Reserved"},
      {"Sample interval", BYTES(2), 0.01, false, "s", ""},
      {"Filter duration", BYTES(2), 0.01, false, "s", ""},
      {0}}}

    /* http://www.airmartechnology.com/uploads/installguide/DST200UserlManual.pdf */
    ,
    {"Airmar: NMEA 2000 options",
     126720,
     PACKET_COMPLETE,
     PACKET_FAST,
     6,
     2,
     {{"Manufacturer Code", 11, RES_MANUFACTURER, false, "=135", "Airmar"},
      {"Reserved", 2, RES_NOTUSED, false, 0, ""},
      {"Industry Code", 3, RES_LOOKUP, false, "=4", "Marine Industry"},
      {"Proprietary ID", BYTES(1), RES_INTEGER, false, "=46", "NMEA 2000 options"},
      {"Transmission Interval",
       2,
       RES_LOOKUP,
       false,
       ",0=Measure Interval,1=Requested by user,2=reserved,3=data not available",
       ""},
      {"Reserved", 22, RES_BINARY, false, 0, "Reserved"},
      {0}}}

    ,
    {"Airmar: Addressable Multi-Frame",
     126720,
     PACKET_INCOMPLETE,
     PACKET_FAST,
     4,
     0,
     {{"Manufacturer Code", 11, RES_MANUFACTURER, false, "=135", "Airmar"},
      {"Reserved", 2, RES_NOTUSED, false, 0, ""},
      {"Industry Code", 3, RES_LOOKUP, false, "=4", "Marine Industry"},
      {"Proprietary ID", BYTES(1), RES_INTEGER, false, 0, ""},
      {0}}}

    ,
    {"Maretron: Slave Response",
     126720,
     PACKET_LOOKUPS_UNKNOWN,
     PACKET_FAST,
     8,
     0,
     {{"Manufacturer Code", 11, RES_MANUFACTURER, false, "=137", "Maretron"},
      {"Reserved", 2, RES_NOTUSED, false, 0, ""},
      {"Industry Code", 3, RES_LOOKUP, false, "=4", "Marine Industry"},
      {"Product code", BYTES(2), 1, false, 0, "0x1b2=SSC200"},
      {"Software code", BYTES(2), 1, false, 0, ""},
      {"Command", BYTES(1), 1, false, 0, "0x50=Deviation calibration result"},
      {"Status", BYTES(1), 1, false, 0, ""},
      {0}}}

    ,
    {"Manufacturer Proprietary fast-packet addressed",
     126720,
     PACKET_INCOMPLETE,
     PACKET_FAST,
     223,
     0,
     {{"Manufacturer Code", 11, RES_MANUFACTURER, false, 0, ""},
      {"Reserved", 2, RES_NOTUSED, false, 0, ""},
      {"Industry Code", 3, RES_LOOKUP, false, LOOKUP_INDUSTRY_CODE, ""},
      {"Data", BYTES(221), RES_BINARY, false, 0, ""},
      {0}},
     0,
     0,
     true}

    /* PDU2 (non addressed) fast packet PGN range 0x1F000 to 0x1FEFF (126976 - 130815) */
    ,
    {"Unknown fast-packet non-addressed",
     126976,
     PACKET_INCOMPLETE,
     PACKET_FAST,
     255,
     0,
     {{"Data", BYTES(255), RES_BINARY, false, 0, ""}, {0}},
     0,
     0,
     true}

    ,
    {"Alert",
     126983,
     PACKET_COMPLETE,
     PACKET_FAST,
     28,
     0,
     {{"Alert Type", 4, RES_LOOKUP, false, LOOKUP_ALERT_TYPE, ""},
      {"Alert Category", 4, RES_LOOKUP, false, LOOKUP_ALERT_CATEGORY, ""},
      {"Alert System", BYTES(1), 1, false, 0, ""},
      {"Alert Sub-System", BYTES(1), 1, false, 0, ""},
      {"Alert ID", BYTES(2), 1, false, 0, ""},
      {"Data Source Network ID NAME", BYTES(8), 1, false, 0, ""},
      {"Data Source Instance", BYTES(1), 1, false, 0, ""},
      {"Data Source Index-Source", BYTES(1), 1, false, 0, ""},
      {"Alert Occurrence Number", BYTES(1), 1, false, 0, ""},
      {"Temporary Silence Status", 1, RES_LOOKUP, false, ",0=Not Temporary Silence,1=Temporary Silence", ""},
      {"Acknowledge Status", 1, RES_LOOKUP, false, ",0=Not Acknowledged,1=Acknowledged", ""},
      {"Escalation Status", 1, RES_LOOKUP, false, ",0=Not Escalated,1=Escalated", ""},
      {"Temporary Silence Support", 1, RES_LOOKUP, false, ",0=Not Supported,1=Supported", ""},
      {"Acknowledge Support", 1, RES_LOOKUP, false, ",0=Not Supported,1=Supported", ""},
      {"Escalation Support", 1, RES_LOOKUP, false, ",0=Not Supported,1=Supported", ""},
      {"NMEA Reserved", 2, RES_BINARY, false, 0, ""},
      {"Acknowledge Source Network ID NAME", BYTES(8), 1, false, 0, ""},
      {"Trigger Condition", 4, RES_LOOKUP, false, LOOKUP_ALERT_TRIGGER_CONDITION, ""},
      {"Threshold Status", 4, RES_LOOKUP, false, LOOKUP_ALERT_THRESHOLD_STATUS, ""},
      {"Alert Priority", BYTES(1), 1, false, 0, ""},
      {"Alert State", BYTES(1), RES_LOOKUP, false, LOOKUP_ALERT_STATE, ""},
      {0}}}

    ,
    {"Alert Response",
     126984,
     PACKET_COMPLETE,
     PACKET_FAST,
     25,
     0,
     {{"Alert Type", 4, RES_LOOKUP, false, LOOKUP_ALERT_TYPE, ""},
      {"Alert Category", 4, RES_LOOKUP, false, LOOKUP_ALERT_CATEGORY, ""},
      {"Alert System", BYTES(1), 1, false, 0, ""},
      {"Alert Sub-System", BYTES(1), 1, false, 0, ""},
      {"Alert ID", BYTES(2), 1, false, 0, ""},
      {"Data Source Network ID NAME", BYTES(8), 1, false, 0, ""},
      {"Data Source Instance", BYTES(1), 1, false, 0, ""},
      {"Data Source Index-Source", BYTES(1), 1, false, 0, ""},
      {"Alert Occurrence Number", BYTES(1), 1, false, 0, ""},
      {"Acknowledge Source Network ID NAME", BYTES(8), 1, false, 0, ""},
      {"Response Command", 2, RES_LOOKUP, false, LOOKUP_ALERT_RESPONSE_COMMAND, ""},
      {"NMEA Reserved", 6, RES_BINARY, false, 0, ""},
      {0}}}

    ,
    {"Alert Text",
     126985,
     PACKET_COMPLETE,
     PACKET_FAST,
     49,
     0,
     {{"Alert Type", 4, RES_LOOKUP, false, LOOKUP_ALERT_TYPE, ""},
      {"Alert Category", 4, RES_LOOKUP, false, LOOKUP_ALERT_CATEGORY, ""},
      {"Alert System", BYTES(1), 1, false, 0, ""},
      {"Alert Sub-System", BYTES(1), 1, false, 0, ""},
      {"Alert ID", BYTES(2), 1, false, 0, ""},
      {"Data Source Network ID NAME", BYTES(8), 1, false, 0, ""},
      {"Data Source Instance", BYTES(1), 1, false, 0, ""},
      {"Data Source Index-Source", BYTES(1), 1, false, 0, ""},
      {"Alert Occurrence Number", BYTES(1), 1, false, 0, ""},
      {"Language ID", BYTES(1), RES_LOOKUP, false, LOOKUP_ALERT_LANGUAGE_ID, ""},
      {"Alert Text Description", BYTES(16), RES_STRINGLAU, false, 0, ""},
      {"Alert Location Text Description", BYTES(16), RES_STRINGLAU, false, 0, ""},
      {0}}}

    ,
    {"Alert Configuration", 126986, PACKET_INCOMPLETE, PACKET_FAST, 8, 0, {{0}}}

    ,
    {"Alert Threshold", 126987, PACKET_INCOMPLETE, PACKET_FAST, 8, 0, {{0}}}

    ,
    {"Alert Value", 126988, PACKET_INCOMPLETE, PACKET_FAST, 8, 0, {{0}}}

    /* http://www.maretron.com/support/manuals/GPS100UM_1.2.pdf */
    ,
    {"System Time",
     126992,
     PACKET_COMPLETE,
     PACKET_SINGLE,
     8,
     0,
     {{"SID", BYTES(1), 1, false, 0, ""},
      {"Source", 4, RES_LOOKUP, false, LOOKUP_SYSTEM_TIME, ""},
      {"Reserved", 4, RES_BINARY, false, 0, "Reserved"},
      {"Date", BYTES(2), RES_DATE, false, "days", "Days since January 1, 1970"},
      {"Time", BYTES(4), RES_TIME, false, "s", "Seconds since midnight"},
      {0}}}

    /* http://www.nmea.org/Assets/20140102%20nmea-2000-126993%20heartbeat%20pgn%20corrigendum.pdf */
    /* http://www.nmea.org/Assets/20190624%20NMEA%20Heartbeat%20Information%20Amendment%20AT%2020190623HB.pdf */
    ,
    {"Heartbeat",
     126993,
     PACKET_COMPLETE,
     PACKET_SINGLE,
     8,
     0,
     {{"Data transmit offset",
       BYTES(2),
       0.001,
       false,
       "s",
       "Offset in transmit time from time of request command: 0x0 = transmit immediately, 0xFFFF = Do not change offset."},
      {"Sequence Counter", BYTES(1), RES_INTEGER, false, 0, ""},
      {"Controller 1 State", 2, RES_LOOKUP, false, ",0=Error Active,1=Error Passive,2=Bus Off,3=Not Available", ""},
      {"Controller 2 State", 2, RES_LOOKUP, false, ",0=Error Active,1=Error Passive,2=Bus Off,3=Not Available", ""},
      {"Equipment Status", 2, RES_LOOKUP, false, ",0=Operational,1=Fault,2=Reserved,3=Not Available", ""},
      {"Reserved", 34, RES_BINARY, false, 0, "Reserved"},
      {0}}}

    ,
    {"Product Information",
     126996,
     PACKET_COMPLETE,
     PACKET_FAST,
     0x86,
     0,
     {{"NMEA 2000 Version", BYTES(2), 1, false, 0, ""},
      {"Product Code", BYTES(2), 1, false, 0, ""},
      {"Model ID", BYTES(32), RES_ASCII, false, 0, ""},
      {"Software Version Code", BYTES(32), RES_ASCII, false, 0, ""},
      {"Model Version", BYTES(32), RES_ASCII, false, 0, ""},
      {"Model Serial Code", BYTES(32), RES_ASCII, false, 0, ""},
      {"Certification Level", BYTES(1), 1, false, 0, ""},
      {"Load Equivalency", BYTES(1), 1, false, 0, ""},
      {0}}}

    ,
    {"Configuration Information",
     126998,
     PACKET_COMPLETE,
     PACKET_FAST,
     0x2a,
     0,
     {{"Installation Description #1", BYTES(2), RES_STRINGLAU, false, 0, ""},
      {"Installation Description #2", BYTES(2), RES_STRINGLAU, false, 0, ""},
      {"Manufacturer Information", BYTES(2), RES_STRINGLAU, false, 0, ""},
      {0}}}

    /************ PERIODIC DATA PGNs **************/
    /* http://www.nmea.org/Assets/july%202010%20nmea2000_v1-301_app_b_pgn_field_list.pdf */
    /* http://www.maretron.com/support/manuals/USB100UM_1.2.pdf */
    /* http://www8.garmin.com/manuals/GPSMAP4008_NMEA2000NetworkFundamentals.pdf */

    /* http://www.nmea.org/Assets/20130906%20nmea%202000%20%20man%20overboard%20notification%20%28mob%29%20pgn%20127233%20amendment.pdf
     */
    ,
    {"Man Overboard Notification",
     127233,
     PACKET_COMPLETE,
     PACKET_FAST,
     35,
     0,
     {{"SID", BYTES(1), 1, false, 0, ""},
      {"MOB Emitter ID", BYTES(4), RES_INTEGER, false, 0, "Identifier for each MOB emitter, unique to the vessel"},
      {"Man Overboard Status",
       3,
       RES_LOOKUP,
       false,
       ",0=MOB Emitter Activated,1=Manual on-board MOB Button Activation,2=Test Mode,3=MOB Not Active",
       ""},
      {"Reserved", 5, RES_BINARY, false, 0, ""},
      {"Activation Time", BYTES(4), RES_TIME, false, "s", "Time of day (UTC) when MOB was activated"},
      {"Position Source", 3, RES_LOOKUP, false, ",0=Position estimated by the Vessel,1=Position reported by MOB emitter", ""},
      {"Reserved", 5, RES_BINARY, false, 0, ""},
      {"Position Date", BYTES(2), RES_DATE, false, "", "Date of MOB position"},
      {"Position Time", BYTES(4), RES_TIME, false, "s", "Time of day of MOB position (UTC)"},
      {"Latitude", BYTES(4), RES_LATITUDE, true, "deg", ""},
      {"Longitude", BYTES(4), RES_LONGITUDE, true, "deg", ""},
      {"COG Reference", 2, RES_LOOKUP, false, LOOKUP_DIRECTION_REFERENCE, ""},
      {"Reserved", 6, RES_BINARY, false, 0, ""},
      {"COG", BYTES(2), RES_RADIANS, false, "rad", ""},
      {"SOG", BYTES(2), 0.01, false, "m/s", ""},
      {"MMSI of vessel of origin", BYTES(4), RES_INTEGER, false, "MMSI", ""},
      {"MOB Emitter Battery Status", 3, RES_LOOKUP, false, ",0=Good,1=Low", ""},
      {"Reserved", 5, RES_BINARY, false, 0, ""},
      {0}}}

    ,
    {"Heading/Track control",
     127237,
     PACKET_COMPLETE,
     PACKET_FAST,
     0x15,
     0,
     {{"Rudder Limit Exceeded", 2, RES_LOOKUP, false, LOOKUP_YES_NO, ""},
      {"Off-Heading Limit Exceeded", 2, RES_LOOKUP, false, LOOKUP_YES_NO, ""},
      {"Off-Track Limit Exceeded", 2, RES_LOOKUP, false, LOOKUP_YES_NO, ""},
      {"Override", 2, RES_LOOKUP, false, LOOKUP_YES_NO, ""},
      {"Steering Mode",
       3,
       RES_LOOKUP,
       false,
       ",0=Main Steering,1=Non-Follow-up Device,10=Follow-up Device,11=Heading Control Standalone,100=Heading Control,101=Track "
       "Control",
       ""},
      {"Turn Mode", 3, RES_LOOKUP, false, ",0=Rudder Limit controlled,1=turn rate controlled,10=radius controlled", ""},
      {"Heading Reference", 2, RES_LOOKUP, false, LOOKUP_DIRECTION_REFERENCE, ""},
      {"Reserved", 5, RES_BINARY, false, 0, ""},
      {"Commanded Rudder Direction", 3, RES_LOOKUP, false, LOOKUP_DIRECTION_RUDDER, ""},
      {"Commanded Rudder Angle", BYTES(2), RES_RADIANS, true, "rad", ""},
      {"Heading-To-Steer (Course)", BYTES(2), RES_RADIANS, false, "rad", ""},
      {"Track", BYTES(2), RES_RADIANS, false, "rad", ""},
      {"Rudder Limit", BYTES(2), RES_RADIANS, false, "rad", ""},
      {"Off-Heading Limit", BYTES(2), RES_RADIANS, false, "rad", ""},
      {"Radius of Turn Order", BYTES(2), RES_RADIANS, true, "rad", ""},
      {"Rate of Turn Order", BYTES(2), RES_ROTATION, true, "rad/s", ""},
      {"Off-Track Limit", BYTES(2), 1, true, "m", ""},
      {"Vessel Heading", BYTES(2), RES_RADIANS, false, "rad", ""},
      {0}}}

    /* http://www.maretron.com/support/manuals/RAA100UM_1.0.pdf */
    ,
    {"Rudder",
     127245,
     PACKET_COMPLETE,
     PACKET_SINGLE,
     8,
     0,
     {{"Instance", BYTES(1), 1, false, 0, ""},
      {"Direction Order", 3, RES_LOOKUP, false, LOOKUP_DIRECTION_RUDDER, ""},
      {"Reserved", 5, RES_BINARY, false, 0, "Reserved"},
      {"Angle Order", BYTES(2), RES_RADIANS, true, "rad", ""},
      {"Position", BYTES(2), RES_RADIANS, true, "rad", ""},
      {"Reserved", BYTES(2), RES_BINARY, false, 0, "Reserved"},
      {0}}}

    /* NMEA + Simrad AT10 */
    /* http://www.maretron.com/support/manuals/SSC200UM_1.7.pdf */
    /* molly_rose_E80start.kees */
    ,
    {"Vessel Heading",
     127250,
     PACKET_COMPLETE,
     PACKET_SINGLE,
     8,
     0,
     {{"SID", BYTES(1), 1, false, 0, ""},
      {"Heading", BYTES(2), RES_RADIANS, false, "rad", ""},
      {"Deviation", BYTES(2), RES_RADIANS, true, "rad", ""},
      {"Variation", BYTES(2), RES_RADIANS, true, "rad", ""},
      {"Reference", 2, RES_LOOKUP, false, LOOKUP_DIRECTION_REFERENCE, ""},
      {"Reserved", 6, RES_BINARY, false, 0, "Reserved"},
      {0}}}

    /* http://www.maretron.com/support/manuals/SSC200UM_1.7.pdf */
    /* Lengths observed from Simrad RC42 */
    ,
    {"Rate of Turn",
     127251,
     PACKET_COMPLETE,
     PACKET_SINGLE,
     5,
     0,
     {{"SID", BYTES(1), 1, false, 0, ""}, {"Rate", BYTES(4), RES_HIRES_ROTATION, true, "rad/s", ""}, {0}}}

    ,
    {"Heave",
     127252,
     PACKET_INCOMPLETE,
     PACKET_SINGLE,
     0x08,
     0,
     {{"SID", BYTES(1), 1, false, 0, ""},
      {"Heave", BYTES(2), 0.01, true, "m", ""},
      {"Reserved", BYTES(5), RES_NOTUSED, false, 0, ""},
      {0}}}

    ,
    {"Attitude",
     127257,
     PACKET_COMPLETE,
     PACKET_SINGLE,
     7,
     0,
     {{"SID", BYTES(1), 1, false, 0, ""},
      {"Yaw", BYTES(2), RES_RADIANS, true, "rad", ""},
      {"Pitch", BYTES(2), RES_RADIANS, true, "rad", ""},
      {"Roll", BYTES(2), RES_RADIANS, true, "rad", ""},
      {0}}}

    /* NMEA + Simrad AT10 */
    /* http://www.maretron.com/support/manuals/GPS100UM_1.2.pdf */
    ,
    {"Magnetic Variation",
     127258,
     PACKET_COMPLETE,
     PACKET_SINGLE,
     6,
     0,
     {{"SID", BYTES(1), 1, false, 0, ""},
      {"Source", 4, RES_LOOKUP, false, LOOKUP_MAGNETIC_VARIATION, ""},
      {"Reserved", 4, RES_BINARY, false, 0, "Reserved"},
      {"Age of service", BYTES(2), RES_DATE, false, "days", "Days since January 1, 1970"},
      {"Variation", BYTES(2), RES_RADIANS, true, "rad", ""},
      {0}}}

    /* Engine group PGNs all derived PGN Numbers from              */
    /* http://www.maretron.com/products/pdf/J2K100-Data_Sheet.pdf  */
    /* http://www.floscan.com/html/blue/NMEA2000.php               */
    /* http://www.osukl.com/wp-content/uploads/2015/04/3155-UM.pdf */
    ,
    {"Engine Parameters, Rapid Update",
     127488,
     PACKET_COMPLETE,
     PACKET_SINGLE,
     8,
     0,
     {{"Instance", BYTES(1), RES_LOOKUP, false, LOOKUP_ENGINE_INSTANCE, ""},
      {"Speed", BYTES(2), 0.25, false, "rpm", ""},
      {"Boost Pressure", BYTES(2), RES_PRESSURE, false, "hPa", ""},
      {"Tilt/Trim", BYTES(1), 1, true, "", ""},
      {"Reserved", BYTES(2), RES_BINARY, false, 0, ""},
      {0}}}

    // http://www.osukl.com/wp-content/uploads/2015/04/3155-UM.pdf
    ,
    {"Engine Parameters, Dynamic",
     127489,
     PACKET_COMPLETE,
     PACKET_FAST,
     26,
     0,
     {{"Instance", BYTES(1), RES_LOOKUP, false, LOOKUP_ENGINE_INSTANCE, ""},
      {"Oil pressure", BYTES(2), RES_PRESSURE, false, "hPa", ""},
      {"Oil temperature", BYTES(2), RES_TEMPERATURE_HIGH, false, "K", ""},
      {"Temperature", BYTES(2), RES_TEMPERATURE, false, "K", ""},
      {"Alternator Potential", BYTES(2), 0.01, true, "V", ""},
      {"Fuel Rate", BYTES(2), 0.1, true, "L/h", ""},
      {"Total Engine hours", BYTES(4), 1.0, false, "s", ""},
      {"Coolant Pressure", BYTES(2), RES_PRESSURE, false, "hPa", ""},
      {"Fuel Pressure", BYTES(2), 1, false, "kPa", ""},
      {"Reserved", BYTES(1), RES_BINARY, false, 0, ""},
      {"Discrete Status 1", BYTES(2), RES_BITFIELD, false, LOOKUP_ENGINE_STATUS_1, ""},
      {"Discrete Status 2", BYTES(2), RES_BITFIELD, false, LOOKUP_ENGINE_STATUS_2, ""},
      {"Percent Engine Load", BYTES(1), RES_INTEGER, true, "%", ""},
      {"Percent Engine Torque", BYTES(1), RES_INTEGER, true, "%", ""},
      {0}}}

    ,
    {"Transmission Parameters, Dynamic",
     127493,
     PACKET_COMPLETE,
     PACKET_SINGLE,
     8,
     0,
     {{"Instance", 8, RES_LOOKUP, false, LOOKUP_ENGINE_INSTANCE, ""},
      {"Transmission Gear", 2, RES_LOOKUP, false, LOOKUP_GEAR_STATUS, ""},
      {"Reserved", 6, RES_BINARY, false, 0, ""},
      {"Oil pressure", BYTES(2), RES_PRESSURE, false, "hPa", ""},
      {"Oil temperature", BYTES(2), RES_TEMPERATURE_HIGH, false, "K", ""},
      {"Discrete Status 1", BYTES(1), RES_INTEGER, false, 0, ""},
      {"Reserved", BYTES(1), RES_BINARY, false, 0, ""},
      {0}}}

    ,
    {"Trip Parameters, Vessel",
     127496,
     PACKET_COMPLETE,
     PACKET_FAST,
     10,
     0,
     {{"Time to Empty", BYTES(4), 0.001, false, "s", ""},
      {"Distance to Empty", BYTES(4), 0.01, false, "m", ""},
      {"Estimated Fuel Remaining", BYTES(2), 1, false, "L", ""},
      {"Trip Run Time", BYTES(4), 0.001, false, "s", ""},
      {0}}}

    ,
    {"Trip Parameters, Engine",
     127497,
     PACKET_COMPLETE,
     PACKET_FAST,
     9,
     0,
     {{"Instance", BYTES(1), RES_LOOKUP, false, LOOKUP_ENGINE_INSTANCE, ""},
      {"Trip Fuel Used", BYTES(2), 1, false, "L", ""},
      {"Fuel Rate, Average", BYTES(2), 0.1, true, "L/h", ""},
      {"Fuel Rate, Economy", BYTES(2), 0.1, true, "L/h", ""},
      {"Instantaneous Fuel Economy", BYTES(2), 0.1, true, "L/h", ""},
      {0}}}

    ,
    {"Engine Parameters, Static",
     127498,
     PACKET_COMPLETE,
     PACKET_FAST,
     52,
     0,
     {{"Instance", BYTES(1), RES_LOOKUP, false, LOOKUP_ENGINE_INSTANCE, ""},
      {"Rated Engine Speed", BYTES(2), 0.25, false, 0, "rpm"},
      {"VIN", BYTES(17), RES_ASCII, false, 0, ""},
      {"Software ID", BYTES(32), RES_ASCII, false, 0, ""},
      {0}}}

    ,
    {"Load Controller Connection State/Control",
     127500,
     PACKET_COMPLETE,
     PACKET_FAST,
     10,
     0,
     {{"Sequence ID", BYTES(1), RES_INTEGER, false, 0, ""},
      {"Connection ID", BYTES(1), RES_INTEGER, false, 0, ""},
      {"State", BYTES(1), RES_INTEGER, false, 0, ""},
      {"Status", BYTES(1), RES_INTEGER, false, 0, ""},
      {"Operational Status & Control", BYTES(1), RES_INTEGER, false, 0, ""},
      {"PWM Duty Cycle", BYTES(1), RES_INTEGER, false, 0, ""},
      {"TimeON", BYTES(2), RES_INTEGER, false, 0, ""},
      {"TimeOFF", BYTES(2), RES_INTEGER, false, 0, ""},
      {0}}}

    ,
    {"Binary Switch Bank Status",
     127501,
     PACKET_COMPLETE,
     PACKET_SINGLE,
     8,
     0,
     {{"Instance", BYTES(1), 1, false, 0, ""},
      {"Indicator1", 2, RES_LOOKUP, false, LOOKUP_OFF_ON, ""},
      {"Indicator2", 2, RES_LOOKUP, false, LOOKUP_OFF_ON, ""},
      {"Indicator3", 2, RES_LOOKUP, false, LOOKUP_OFF_ON, ""},
      {"Indicator4", 2, RES_LOOKUP, false, LOOKUP_OFF_ON, ""},
      {"Indicator5", 2, RES_LOOKUP, false, LOOKUP_OFF_ON, ""},
      {"Indicator6", 2, RES_LOOKUP, false, LOOKUP_OFF_ON, ""},
      {"Indicator7", 2, RES_LOOKUP, false, LOOKUP_OFF_ON, ""},
      {"Indicator8", 2, RES_LOOKUP, false, LOOKUP_OFF_ON, ""},
      {"Indicator9", 2, RES_LOOKUP, false, LOOKUP_OFF_ON, ""},
      {"Indicator10", 2, RES_LOOKUP, false, LOOKUP_OFF_ON, ""},
      {"Indicator11", 2, RES_LOOKUP, false, LOOKUP_OFF_ON, ""},
      {"Indicator12", 2, RES_LOOKUP, false, LOOKUP_OFF_ON, ""},
      {"Indicator13", 2, RES_LOOKUP, false, LOOKUP_OFF_ON, ""},
      {"Indicator14", 2, RES_LOOKUP, false, LOOKUP_OFF_ON, ""},
      {"Indicator15", 2, RES_LOOKUP, false, LOOKUP_OFF_ON, ""},
      {"Indicator16", 2, RES_LOOKUP, false, LOOKUP_OFF_ON, ""},
      {"Indicator17", 2, RES_LOOKUP, false, LOOKUP_OFF_ON, ""},
      {"Indicator18", 2, RES_LOOKUP, false, LOOKUP_OFF_ON, ""},
      {"Indicator19", 2, RES_LOOKUP, false, LOOKUP_OFF_ON, ""},
      {"Indicator20", 2, RES_LOOKUP, false, LOOKUP_OFF_ON, ""},
      {"Indicator21", 2, RES_LOOKUP, false, LOOKUP_OFF_ON, ""},
      {"Indicator22", 2, RES_LOOKUP, false, LOOKUP_OFF_ON, ""},
      {"Indicator23", 2, RES_LOOKUP, false, LOOKUP_OFF_ON, ""},
      {"Indicator24", 2, RES_LOOKUP, false, LOOKUP_OFF_ON, ""},
      {"Indicator25", 2, RES_LOOKUP, false, LOOKUP_OFF_ON, ""},
      {"Indicator26", 2, RES_LOOKUP, false, LOOKUP_OFF_ON, ""},
      {"Indicator27", 2, RES_LOOKUP, false, LOOKUP_OFF_ON, ""},
      {"Indicator28", 2, RES_LOOKUP, false, LOOKUP_OFF_ON, ""},
      {0}}},
    {"Switch Bank Control",
     127502,
     PACKET_COMPLETE,
     PACKET_SINGLE,
     8,
     0,
     {{"Instance", BYTES(1), 1, false, 0, ""},   {"Switch1", 2, RES_LOOKUP, false, LOOKUP_OFF_ON, ""},
      {"Switch2", 2, RES_LOOKUP, false, LOOKUP_OFF_ON, ""},  {"Switch3", 2, RES_LOOKUP, false, LOOKUP_OFF_ON, ""},
      {"Switch4", 2, RES_LOOKUP, false, LOOKUP_OFF_ON, ""},  {"Switch5", 2, RES_LOOKUP, false, LOOKUP_OFF_ON, ""},
      {"Switch6", 2, RES_LOOKUP, false, LOOKUP_OFF_ON, ""},  {"Switch7", 2, RES_LOOKUP, false, LOOKUP_OFF_ON, ""},
      {"Switch8", 2, RES_LOOKUP, false, LOOKUP_OFF_ON, ""},  {"Switch9", 2, RES_LOOKUP, false, LOOKUP_OFF_ON, ""},
      {"Switch10", 2, RES_LOOKUP, false, LOOKUP_OFF_ON, ""}, {"Switch11", 2, RES_LOOKUP, false, LOOKUP_OFF_ON, ""},
      {"Switch12", 2, RES_LOOKUP, false, LOOKUP_OFF_ON, ""}, {"Switch13", 2, RES_LOOKUP, false, LOOKUP_OFF_ON, ""},
      {"Switch14", 2, RES_LOOKUP, false, LOOKUP_OFF_ON, ""}, {"Switch15", 2, RES_LOOKUP, false, LOOKUP_OFF_ON, ""},
      {"Switch16", 2, RES_LOOKUP, false, LOOKUP_OFF_ON, ""}, {"Switch17", 2, RES_LOOKUP, false, LOOKUP_OFF_ON, ""},
      {"Switch18", 2, RES_LOOKUP, false, LOOKUP_OFF_ON, ""}, {"Switch19", 2, RES_LOOKUP, false, LOOKUP_OFF_ON, ""},
      {"Switch20", 2, RES_LOOKUP, false, LOOKUP_OFF_ON, ""}, {"Switch21", 2, RES_LOOKUP, false, LOOKUP_OFF_ON, ""},
      {"Switch22", 2, RES_LOOKUP, false, LOOKUP_OFF_ON, ""}, {"Switch23", 2, RES_LOOKUP, false, LOOKUP_OFF_ON, ""},
      {"Switch24", 2, RES_LOOKUP, false, LOOKUP_OFF_ON, ""}, {"Switch25", 2, RES_LOOKUP, false, LOOKUP_OFF_ON, ""},
      {"Switch26", 2, RES_LOOKUP, false, LOOKUP_OFF_ON, ""}, {"Switch27", 2, RES_LOOKUP, false, LOOKUP_OFF_ON, ""},
      {"Switch28", 2, RES_LOOKUP, false, LOOKUP_OFF_ON, ""}, {0}}}

    /* http://www.nmea.org/Assets/nmea-2000-corrigendum-1-2010-1.pdf */
    ,
    {"AC Input Status",
     127503,
     PACKET_COMPLETE,
     PACKET_FAST,
     2 + 3 * 18,
     10,
     {{"Instance", BYTES(1), 1, false, 0, ""},
      {"Number of Lines", BYTES(1), 1, false, 0, ""}

      ,
      {"Line", 2, RES_LOOKUP, false, ",0=Line 1,1=Line 2,2=Line 3,3=Reserved", ""},
      {"Acceptability", 2, RES_LOOKUP, false, ",0=Bad Level,1=Bad Frequency,2=Being Qualified,3=Good", ""},
      {"Reserved", 4, RES_BINARY, false, 0, ""},
      {"Voltage", BYTES(2), 0.01, false, "V", ""},
      {"Current", BYTES(2), 0.1, false, "A", ""},
      {"Frequency", BYTES(2), 0.01, false, "Hz", ""},
      {"Breaker Size", BYTES(2), 0.1, false, "A", ""},
      {"Real Power", BYTES(4), RES_INTEGER, false, "W", ""},
      {"Reactive Power", BYTES(4), RES_INTEGER, false, "VAR", ""},
      {"Power Factor", BYTES(1), 0.01, false, "Cos Phi", ""},
      {0}}}

    /* http://www.nmea.org/Assets/nmea-2000-corrigendum-1-2010-1.pdf */
    ,
    {"AC Output Status",
     127504,
     PACKET_COMPLETE,
     PACKET_SINGLE,
     2 + 3 * 18,
     10,
     {{"Instance", BYTES(1), 1, false, 0, ""},
      {"Number of Lines", BYTES(1), 1, false, 0, ""}

      ,
      {"Line", 2, RES_LOOKUP, false, ",0=Line 1,1=Line 2,2=Line 3", ""},
      {"Waveform", 3, RES_LOOKUP, false, ",0=Sine Wave,1=Modified Sine Wave", ""},
      {"Reserved", 3, RES_BINARY, false, 0, ""},
      {"Voltage", BYTES(2), 0.01, false, "V", ""},
      {"Current", BYTES(2), 0.1, false, "A", ""},
      {"Frequency", BYTES(2), 0.01, false, "Hz", ""},
      {"Breaker Size", BYTES(2), 0.1, false, "A", ""},
      {"Real Power", BYTES(4), RES_INTEGER, false, "W", ""},
      {"Reactive Power", BYTES(4), RES_INTEGER, false, "VAR", ""},
      {"Power Factor", BYTES(1), 0.01, false, "Cos Phi", ""},
      {0}}}

    /* http://www.maretron.com/support/manuals/TLA100UM_1.2.pdf */
    /* Observed from EP65R */
    ,
    {"Fluid Level",
     127505,
     PACKET_COMPLETE,
     PACKET_SINGLE,
     8,
     0,
     {{"Instance", 4, 1, false, 0, ""},
      {"Type", 4, RES_LOOKUP, false, ",0=Fuel,1=Water,2=Gray water,3=Live well,4=Oil,5=Black water", ""},
      {"Level", BYTES(2), RES_PERCENTAGE, false, "%", ""},
      {"Capacity", BYTES(4), 0.1, false, "L", ""},
      {"Reserved", BYTES(1), RES_BINARY, false, 0, "Reserved"},
      {0}}}

    ,
    {"DC Detailed Status",
     127506,
     PACKET_COMPLETE,
     PACKET_FAST,
     11,
     0,
     {{"SID", BYTES(1), 1, false, 0, ""},
      {"Instance", BYTES(1), 1, false, 0, ""},
      {"DC Type", BYTES(1), RES_LOOKUP, false, ",0=Battery,1=Alternator,2=Convertor,3=Solar Cell,4=Wind Generator", ""},
      {"State of Charge", BYTES(1), 1, false, 0, ""},
      {"State of Health", BYTES(1), 1, false, 0, ""},
      {"Time Remaining", BYTES(2), 1, false, 0, ""},
      {"Ripple Voltage", BYTES(2), 0.01, false, "V", ""},
      {"Amp Hours", BYTES(2), 3600, false, "C", ""},
      {0}}}

    // http://www.osukl.com/wp-content/uploads/2015/04/3155-UM.pdf
    ,
    {"Charger Status",
     127507,
     PACKET_COMPLETE,
     PACKET_FAST,
     6,
     0,
     {{"Instance", BYTES(1), 1, false, 0, ""},
      {"Battery Instance", BYTES(1), 1, false, 0, ""},
      {"Operating State",
       4,
       RES_LOOKUP,
       false,
       ",0=Not charging,1=Bulk,2=Absorption,3=Overcharge,4=Equalise,5=Float,6=No Float,7=Constant VI,8=Disabled,9=Fault",
       ""},
      {"Charge Mode", 4, RES_LOOKUP, false, ",0=Standalone,1=Primary,2=Secondary,3=Echo", ""},
      {"Enabled", 2, RES_LOOKUP, false, LOOKUP_OFF_ON, ""},
      {"Equalization Pending", 2, RES_LOOKUP, false, LOOKUP_OFF_ON, ""},
      {"Reserved", 4, RES_BINARY, false, 0, ""},
      {"Equalization Time Remaining", BYTES(2), 1, false, 0, ""},
      {0}}}

    ,
    {"Battery Status",
     127508,
     PACKET_COMPLETE,
     PACKET_SINGLE,
     8,
     0,
     {{"Instance", BYTES(1), 1, false, 0, ""},
      {"Voltage", BYTES(2), 0.01, true, "V", ""},
      {"Current", BYTES(2), 0.1, true, "A", ""},
      {"Temperature", BYTES(2), RES_TEMPERATURE, false, "K", ""},
      {"SID", BYTES(1), 1, false, 0, ""},
      {0}}}

    /* https://www.nmea.org/Assets/20140102%20nmea-2000-127509%20pgn%20corrigendum.pdf */
    ,
    {"Inverter Status",
     127509,
     PACKET_COMPLETE,
     PACKET_SINGLE,
     4,
     0,
     {{"Instance", BYTES(1), 1, false, 0, ""},
      {"AC Instance", BYTES(1), 1, false, 0, ""},
      {"DC Instance", BYTES(1), 1, false, 0, ""},
      {"Operating State", 4, RES_LOOKUP, false, ",0=Invert,1=AC Passthru,2=Load Sense,3=Fault,4=Disabled,14=Error,15=Data Not Available", ""},
      {"Inverter Enable/Disable", 2, RES_LOOKUP, false, "0=Disabled,1=Enabled,2=Error,3=Unknown", ""},
      {"Reserved", 2, RES_BINARY, false, 0, "Reserved"},
      {0}}}

    ,
    {"Charger Configuration Status",
     127510,
     PACKET_INCOMPLETE,
     PACKET_FAST,
     13,
     0,
     {{"Instance", BYTES(1), 1, false, 0, ""},
      {"Battery Instance", BYTES(1), 1, false, 0, ""},
      {"Charger Enable/Disable", 2, 1, false, 0, ""},
      {"Reserved", 6, RES_BINARY, false, 0, ""},
      {"Charge Current Limit", BYTES(2), 0.1, false, "A", ""},
      {"Charging Algorithm", BYTES(1), 1, false, 0, ""},
      {"Charger Mode", BYTES(1), 1, false, 0, ""},
      {"Estimated Temperature", BYTES(2), RES_TEMPERATURE, false, 0, "When no sensor present"},
      {"Equalize One Time Enable/Disable", 4, 1, false, 0, ""},
      {"Over Charge Enable/Disable", 4, 1, false, 0, ""},
      {"Equalize Time", BYTES(2), 1, false, 0, ""},
      {0}}}

    ,
    {"Inverter Configuration Status",
     127511,
     PACKET_INCOMPLETE,
     PACKET_SINGLE,
     8,
     0,
     {{"Instance", BYTES(1), 1, false, 0, ""},
      {"AC Instance", BYTES(1), 1, false, 0, ""},
      {"DC Instance", BYTES(1), 1, false, 0, ""},
      {"Inverter Enable/Disable", 2, 1, false, 0, ""},
      {"Inverter Mode", BYTES(1), 1, false, 0, ""},
      {"Load Sense Enable/Disable", BYTES(1), 1, false, 0, ""},
      {"Load Sense Power Threshold", BYTES(1), 1, false, 0, ""},
      {"Load Sense Interval", BYTES(1), 1, false, 0, ""},
      {0}}}

    ,
    {"AGS Configuration Status",
     127512,
     PACKET_INCOMPLETE,
     PACKET_SINGLE,
     8,
     0,
     {{"Instance", BYTES(1), 1, false, 0, ""},
      {"Generator Instance", BYTES(1), 1, false, 0, ""},
      {"AGS Mode", BYTES(1), 1, false, 0, ""},
      {0}}}

    /* #143, @ksltd writes that it is definitely 10 bytes and that
     * nominal voltage is a lookup, Peukert Exponent and Charge Efficiency
     * are 8 bits. It follows that Temperature Coefficient must be 8 bits
     * as well to fit in 10 bytes.
     *
     * I'm now actually following https://github.com/ttlappalainen/NMEA2000/
     * The Supports Equalization is 2 bits, Battery Type, Chemistry and
     * Nominal voltage are all 4 bits. Capacity and Peukert are both 2 bytes.
     * but this only adds up to 8 bytes... Maybe the 10 was as this is transmitted
     * as FAST pgn?
     */
    ,
    {"Battery Configuration Status",
     127513,
     PACKET_INCOMPLETE,
     PACKET_FAST,
     10,
     0,
     {{"Instance", BYTES(1), 1, false, 0, ""},
      {"Battery Type", 4, RES_LOOKUP, false, ",0=Flooded,1=Gel,2=AGM", ""},
      {"Supports Equalization", 2, RES_LOOKUP, false, ",0=Not Supported,1=Supported", ""},
      {"Reserved", 2, RES_BINARY, false, 0, ""},
      {"Nominal Voltage", 4, RES_LOOKUP, false, ",0=6V,1=12V,2=24V,3=32V,4=36V,5=42V,6=48V", ""},
      {"Chemistry", 4, RES_LOOKUP, false, ",0=LeadAcid,1=Li,2=Nicad,3=Zno,4=NiMH", ""},
      {"Capacity", BYTES(2), 1, false, "C", ""},
      {"Temperature Coefficient", BYTES(1), RES_INTEGER, false, "%", ""},
      {"Peukert Exponent", BYTES(1), 0.002, false, 0, "Possibly in Excess-1 notation"},
      {"Charge Efficiency Factor", BYTES(1), 1, false, "%", ""},
      {0}}}

    ,
    {"AGS Status",
     127514,
     PACKET_INCOMPLETE,
     PACKET_SINGLE,
     8,
     0,
     {{"Instance", BYTES(1), 1, false, 0, ""},
      {"Generator Instance", BYTES(1), 1, false, 0, ""},
      {"AGS Operating State", BYTES(1), 1, false, 0, ""},
      {"Generator State", BYTES(1), 1, false, 0, ""},
      {"Generator On Reason", BYTES(1), 1, false, 0, ""},
      {"Generator Off Reason", BYTES(1), 1, false, 0, ""},
      {0}}}

    ,
    {"AC Power / Current - Phase A",
     127744,
     PACKET_COMPLETE,
     PACKET_SINGLE,
     8,
     0,
     {{"SID", BYTES(1), 1, false, 0, ""},
      {"Connection Number", BYTES(1), 1, false, 0, ""},
      {"AC RMS Current", BYTES(2), 0.1, false, "A", ""},
      {"Power", BYTES(4), 1, true, "W", ""},
      {0}}}

    ,
    {"AC Power / Current - Phase B",
     127745,
     PACKET_COMPLETE,
     PACKET_SINGLE,
     8,
     0,
     {{"SID", BYTES(1), 1, false, 0, ""},
      {"Connection Number", BYTES(1), 1, false, 0, ""},
      {"AC RMS Current", BYTES(2), 0.1, false, "A", ""},
      {"Power", BYTES(4), 1, true, "W", ""},
      {0}}}

    ,
    {"AC Power / Current - Phase C",
     127746,
     PACKET_COMPLETE,
     PACKET_SINGLE,
     8,
     0,
     {{"SID", BYTES(1), 1, false, 0, ""},
      {"Connection Number", BYTES(1), 1, false, 0, ""},
      {"AC RMS Current", BYTES(2), 0.1, false, "A", ""},
      {"Power", BYTES(4), 1, true, "W", ""},
      {0}}}

    ,
    {"Converter Status",
     127750,
     PACKET_COMPLETE,
     PACKET_SINGLE,
     0x08,
     0,
     {{"SID", BYTES(1), RES_BINARY, false, 0, ""},
      {"Connection Number", BYTES(1), 1, false, 0, ""},
      {"Operating State", BYTES(1), RES_LOOKUP, false, LOOKUP_CONVERTER_STATE, ""},
      {"Temperature State", 2, RES_LOOKUP, false, ",0=Ok,1=Warning,2=Over Temperature,3=Not Available", ""},
      {"Overload State", 2, RES_LOOKUP, false, ",0=Ok,1=Warning,2=Overload,3=Not Available", ""},
      {"Low DC Voltage State", 2, RES_LOOKUP, false, ",0=Ok,1=Warning,2=DC voltage too low,3=Not Available", ""},
      {"Ripple State", 2, RES_LOOKUP, false, ",0=Ok,1=Warning,2=Ripple Too High,3=Not Available", ""},
      {"Reserved", BYTES(4), RES_NOTUSED, false, 0, ""},
      {0}}}

    ,
    {"DC Voltage/Current",
     127751,
     PACKET_COMPLETE,
     PACKET_SINGLE,
     0x08,
     0,
     {{"SID", BYTES(1), RES_BINARY, false, 0, ""},
      {"Connection Number", BYTES(1), 1, false, 0, ""},
      {"DC Voltage", BYTES(2), 0.1, false, "V", ""},
      {"DC Current", BYTES(3), 0.01, true, "A", ""},
      {"Reserved", BYTES(1), RES_NOTUSED, false, 0, ""},
      {0}}}

    /* https://www.nmea.org/Assets/20170204%20nmea%202000%20leeway%20pgn%20final.pdf */
    ,
    {"Leeway Angle",
     128000,
     PACKET_COMPLETE,
     PACKET_SINGLE,
     8,
     0,
     {{"SID", BYTES(1), 1, false, 0, ""},
      {"Leeway Angle", BYTES(2), RES_RADIANS, true, "rad", ""},
      {"Reserved", BYTES(5), RES_BINARY, false, 0, ""},
      {0}}}

    ,
    {"Thruster Control Status",
     128006,
     PACKET_COMPLETE,
     PACKET_SINGLE,
     8,
     0,
     {{"SID", BYTES(1), 1, false, 0, ""},
      {"Identifier", BYTES(1), 1, false, 0, ""},
      {"Direction Control", 4, RES_LOOKUP, false, LOOKUP_THRUSTER_DIRECTION_CONTROL, ""},
      {"Power Enabled", 2, RES_LOOKUP, false, LOOKUP_OFF_ON, ""},
      {"Retract Control", 2, RES_LOOKUP, false, LOOKUP_THRUSTER_RETRACT_CONTROL, ""},
      {"Speed Control", BYTES(1), RES_PERCENTAGE, false, "%", ""},
      {"Control Events", BYTES(1), RES_BITFIELD, false, LOOKUP_THRUSTER_CONTROL_EVENTS, ""},
      {"Command Timeout", BYTES(1), 1e-3, false, 0, ""},
      {"Azimuth Control", BYTES(2), RES_RADIANS, false, "rad", ""},
      {0}}}

    ,
    {"Thruster Information",
     128007,
     PACKET_COMPLETE,
     PACKET_SINGLE,
     8,
     0,
     {{"Identifier", BYTES(1), 1, false, 0, ""},
      {"Motor Type", 4, RES_LOOKUP, false, LOOKUP_THRUSTER_MOTOR_TYPE, ""},
      {"Reserved", 4, RES_BINARY, false, 0, ""},
      {"Power Rating", BYTES(2), 1, false, "W", ""},
      {"Maximum Temperature Rating", BYTES(2), RES_TEMPERATURE, false, "K", ""},
      {"Maximum Rotational Speed", BYTES(2), 0.25, false, "rpm", ""},
      {0}}}

    ,
    {"Thruster Motor Status",
     128008,
     PACKET_COMPLETE,
     PACKET_SINGLE,
     8,
     0,
     {{"SID", BYTES(1), 1, false, 0, ""},
      {"Identifier", BYTES(1), 1, false, 0, ""},
      {"Motor Events", BYTES(1), RES_BITFIELD, false, LOOKUP_THRUSTER_MOTOR_EVENTS, ""},
      {"Current", BYTES(1), 1, false, "A", ""},
      {"Temperature", BYTES(2), RES_TEMPERATURE, false, "K", ""},
      {"Operating Time", BYTES(2), 1, false, "minutes", ""},
      {0}}}
    
    /* http://www.maretron.com/support/manuals/DST100UM_1.2.pdf */
    ,
    {"Speed",
     128259,
     PACKET_COMPLETE,
     PACKET_SINGLE,
     8,
     0,
     {{"SID", BYTES(1), 1, false, 0, ""},
      {"Speed Water Referenced", BYTES(2), 0.01, false, "m/s", ""},
      {"Speed Ground Referenced", BYTES(2), 0.01, false, "m/s", ""},
      {"Speed Water Referenced Type", BYTES(1), RES_LOOKUP, false, LOOKUP_WATER_REFERENCE, ""},
      {"Speed Direction", 4, 1, false, 0, ""},
      {"Reserved", 12, RES_BINARY, false, 0, ""},
      {0}}}

    /* http://www.maretron.com/support/manuals/DST100UM_1.2.pdf */
    ,
    {"Water Depth",
     128267,
     PACKET_COMPLETE,
     PACKET_SINGLE,
     8,
     0,
     {{"SID", BYTES(1), 1, false, 0, ""},
      {"Depth", BYTES(4), 0.01, false, "m", "Depth below transducer"},
      {"Offset", BYTES(2), 0.001, true, "m", "Distance between transducer and surface (positive) or keel (negative)"},
      {"Range", BYTES(1), 10, false, "m", "Max measurement range"},
      {0}}}

    /* http://www.nmea.org/Assets/nmea-2000-digital-interface-white-paper.pdf */
    ,
    {"Distance Log",
     128275,
     PACKET_COMPLETE,
     PACKET_FAST,
     14,
     0,
     {{"Date", BYTES(2), RES_DATE, false, "days", "Timestamp of last reset in Days since January 1, 1970"},
      {"Time", BYTES(4), RES_TIME, false, "s", "Timestamp of last reset Seconds since midnight"},
      {"Log", BYTES(4), 1, false, "m", "Total cumulative distance"},
      {"Trip Log", BYTES(4), 1, false, "m", "Distance since last reset"},
      {0}}}

    ,
    {"Tracked Target Data",
     128520,
     PACKET_NOT_SEEN,
     PACKET_FAST,
     27,
     0,
     {{"SID", BYTES(1), 1, false, 0, ""},
      {"Target ID #", BYTES(1), 1, false, 0, "Number of route, waypoint, event, mark, etc."},
      {"Track Status", 2, RES_LOOKUP, false, ",0=Cancelled,1=Acquiring,2=Tracking,3=Lost", ""},
      {"Reported Target", 1, RES_LOOKUP, false, LOOKUP_YES_NO, ""},
      {"Target Acquisition", 1, RES_LOOKUP, false, ",0=Manual,1=Automatic", ""},
      {"Bearing Reference", 2, RES_LOOKUP, false, LOOKUP_DIRECTION_REFERENCE, ""},
      {"Reserved", 2, RES_BINARY, false, 0, ""},
      {"Bearing", BYTES(2), RES_RADIANS, false, "rad", ""},
      {"Distance", BYTES(4), 0.001, false, "m", ""},
      {"Course", BYTES(2), RES_RADIANS, false, "rad", ""},
      {"Speed", BYTES(2), 0.01, false, "m/s", ""},
      {"CPA", BYTES(4), 0.01, false, "m", ""},
      {"TCPA", BYTES(4), 0.001, false, "s", "negative = time elapsed since event, positive = time to go"},
      {"UTC of Fix", BYTES(4), RES_TIME, false, "s", "Seconds since midnight"},
      {"Name", BYTES(255), RES_ASCII, false, 0, ""},
      {0}}}

    ,
    /* https://www.nmea.org/Assets/20190613%20windlass%20amendment,%20128776,%20128777,%20128778.pdf */
    {"Windlass Control Status",
     128776,
     PACKET_COMPLETE,
     PACKET_SINGLE,
     7,
     0,
     {{"SID", BYTES(1), 1, false, 0, ""},
      {"Windlass ID", BYTES(1), 1, false, 0, ""},
      {"Windlass Direction Control", 2, RES_LOOKUP, false, ",0=Off,1=Down,2=Up", ""},
      {"Anchor Docking Control", 2, RES_LOOKUP, false, LOOKUP_OFF_ON, ""},
      {"Speed Control Type", 2, RES_LOOKUP, false, ",0=Single Speed,1=Dual Speed,2=Proportional Speed", ""},
      {"Reserved", 2, RES_BINARY, false, 0, "Reserved"},
      {"Speed Control",
       BYTES(1),
       RES_BINARY,
       false,
       0,
       "0=Off,Single speed:1-100=On,Dual Speed:1-49=Slow/50-100=Fast,Proportional:10-100"},
      {"Power Enable", 2, RES_LOOKUP, false, LOOKUP_OFF_ON, ""},
      {"Mechanical Lock", 2, RES_LOOKUP, false, LOOKUP_OFF_ON, ""},
      {"Deck and Anchor Wash", 2, RES_LOOKUP, false, LOOKUP_OFF_ON, ""},
      {"Anchor Light", 2, RES_LOOKUP, false, LOOKUP_OFF_ON, ""},
      {"Command Timeout",
       BYTES(1),
       0.005,
       false,
       "s",
       "If timeout elapses the thruster stops operating and reverts to static mode"},
      {"Windlass Control Events", 4, RES_BITFIELD, false, ",0=Another device controlling windlass", ""},
      {"Reserved", 4, RES_BINARY, false, 0, "Reserved"},
      {0}}}

    ,
    /* https://www.nmea.org/Assets/20190613%20windlass%20amendment,%20128776,%20128777,%20128778.pdf */
    {"Anchor Windlass Operating Status",
     128777,
     PACKET_COMPLETE,
     PACKET_SINGLE,
     8,
     0,
     {{"SID", BYTES(1), 1, false, 0, ""},
      {"Windlass ID", BYTES(1), 1, false, 0, ""},
      {"Windlass Direction Control", 2, RES_LOOKUP, false, ",0=Off,1=Down,2=Up", ""},
      {"Windlass Motion Status", 2, RES_LOOKUP, false, ",0=Windlass stopped,1=Deployment occurring,2=Retrieval occurring", ""},
      {"Rode Type Status", 2, RES_LOOKUP, false, ",0=Chain presently detected,1=Rope presently detected,2=Error", ""},
      {"Reserved", 2, RES_BINARY, false, 0, "Reserved"},
      {"Rode Counter Value", BYTES(2), 0.1, false, "m", ""},
      {"Windlass Line Speed", BYTES(2), 0.01, false, "m/s", ""},
      {"Anchor Docking Status", 2, RES_LOOKUP, false, ",0=Not docked,1=Fully docked,2=Error", ""},
      {"Windlass Operating Events",
       6,
       RES_BITFIELD,
       false,
       ",0=System error,1=Sensor error,2=No windlass motion detected,3=Retrieval docking distance reached,4=End or rode reached",
       ""},
      {0}}}

    ,
    /* https://www.nmea.org/Assets/20190613%20windlass%20amendment,%20128776,%20128777,%20128778.pdf */
    {"Anchor Windlass Monitoring Status",
     128778,
     PACKET_COMPLETE,
     PACKET_SINGLE,
     8,
     0,
     {{"SID", BYTES(1), 1, false, 0, ""},
      {"Windlass ID", BYTES(1), 1, false, 0, ""},
      {"Windlass Monitoring Events",
       8,
       RES_BITFIELD,
       false,
       ",0=Controller under voltage cut-out,1=Controller over current cut-out,2=Controller over temperature cut-out,3=Manufacturer "
       "defined",
       ""},
      {"Controller voltage", BYTES(1), 0.2, false, "V", ""},
      {"Motor current", BYTES(1), 1, false, "A", ""},
      {"Total Motor Time", BYTES(2), 60, false, "s", ""},
      {"Reserved", BYTES(1), RES_BINARY, false, 0, "Reserved"},
      {0}}}

    ,
    {"Position, Rapid Update",
     129025,
     PACKET_COMPLETE,
     PACKET_SINGLE,
     8,
     0,
     {{"Latitude", BYTES(4), RES_LATITUDE, true, "deg", ""}, {"Longitude", BYTES(4), RES_LONGITUDE, true, "deg", ""}, {0}}}

    /* http://www.maretron.com/support/manuals/GPS100UM_1.2.pdf */
    ,
    {"COG & SOG, Rapid Update",
     129026,
     PACKET_COMPLETE,
     PACKET_SINGLE,
     8,
     0,
     {{"SID", BYTES(1), 1, false, 0, ""},
      {"COG Reference", 2, RES_LOOKUP, false, LOOKUP_DIRECTION_REFERENCE, ""},
      {"Reserved", 6, RES_BINARY, false, 0, "Reserved"},
      {"COG", BYTES(2), RES_RADIANS, false, "rad", ""},
      {"SOG", BYTES(2), 0.01, false, "m/s", ""},
      {"Reserved", BYTES(2), RES_BINARY, false, 0, "Reserved"},
      {0}}}

    ,
    {"Position Delta, Rapid Update",
     129027,
     PACKET_NOT_SEEN,
     PACKET_SINGLE,
     8,
     0,
     {{"SID", BYTES(1), 1, false, 0, ""},
      {"Time Delta", BYTES(2), 1, false, 0, ""},
      {"Latitude Delta", BYTES(2), 1, true, 0, ""},
      {"Longitude Delta", BYTES(2), 1, true, 0, ""},
      {0}}}

    ,
    {"Altitude Delta, Rapid Update",
     129028,
     PACKET_NOT_SEEN,
     PACKET_SINGLE,
     8,
     0,
     {{"SID", BYTES(1), 1, false, 0, ""},
      {"Time Delta", BYTES(2), 1, true, 0, ""},
      {"GNSS Quality", 2, 1, false, 0, ""},
      {"Direction", 2, 1, false, 0, ""},
      {"Reserved", 4, RES_BINARY, false, 0, "Reserved"},
      {"COG", BYTES(2), RES_RADIANS, false, "rad", ""},
      {"Altitude Delta", BYTES(2), 1, true, 0, ""},
      {0}}}

    /* http://www.maretron.com/support/manuals/GPS100UM_1.2.pdf */
    ,
    {"GNSS Position Data",
     129029,
     PACKET_COMPLETE,
     PACKET_FAST,
     43,
     3,
     {{"SID", BYTES(1), 1, false, 0, ""},
      {"Date", BYTES(2), RES_DATE, false, "days", "Days since January 1, 1970"},
      {"Time", BYTES(4), RES_TIME, false, "s", "Seconds since midnight"},
      {"Latitude", BYTES(8), RES_LATITUDE, true, "deg", ""},
      {"Longitude", BYTES(8), RES_LONGITUDE, true, "deg", ""},
      {"Altitude", BYTES(8), 1e-6, true, "m", "Altitude referenced to WGS-84"},
      {"GNSS type", 4, RES_LOOKUP, false, LOOKUP_GNS, ""},
      {"Method", 4, RES_LOOKUP, false, LOOKUP_GNS_METHOD, ""},
      {"Integrity", 2, RES_LOOKUP, false, LOOKUP_GNS_INTEGRITY, ""},
      {"Reserved", 6, RES_BINARY, false, 0, "Reserved"},
      {"Number of SVs", BYTES(1), 1, false, 0, "Number of satellites used in solution"},
      {"HDOP", BYTES(2), 0.01, true, 0, "Horizontal dilution of precision"},
      {"PDOP", BYTES(2), 0.01, true, 0, "Positional dilution of precision"},
      {"Geoidal Separation", BYTES(4), 0.01, true, "m", "Geoidal Separation"},
      {"Reference Stations", BYTES(1), 1, false, 0, "Number of reference stations"},
      {"Reference Station Type", 4, RES_LOOKUP, false, LOOKUP_GNS, ""},
      {"Reference Station ID", 12, 1, false, ""},
      {"Age of DGNSS Corrections", BYTES(2), 0.01, false, "s", ""},
      {0}}}

    ,
    {"Time & Date",
     129033,
     PACKET_COMPLETE,
     PACKET_SINGLE,
     8,
     0,
     {{"Date", BYTES(2), RES_DATE, false, "days", "Days since January 1, 1970"},
      {"Time", BYTES(4), RES_TIME, false, "s", "Seconds since midnight"},
      {"Local Offset", BYTES(2), RES_INTEGER, true, "minutes", "Minutes"},
      {0}}}

    ,
    {"AIS Class A Position Report",
     129038,
     PACKET_COMPLETE,
     PACKET_FAST,
     28,
     0,
     {{"Message ID", 6, 1, false, 0, ""},
      {"Repeat Indicator", 2, RES_LOOKUP, false, LOOKUP_REPEAT_INDICATOR, ""},
      {"User ID", BYTES(4), RES_INTEGER, false, "MMSI", ""},
      {"Longitude", BYTES(4), RES_LATITUDE, true, "deg", ""},
      {"Latitude", BYTES(4), RES_LONGITUDE, true, "deg", ""},
      {"Position Accuracy", 1, RES_LOOKUP, false, LOOKUP_POSITION_ACCURACY, ""},
      {"RAIM", 1, RES_LOOKUP, false, LOOKUP_RAIM_FLAG, ""},
      {"Time Stamp", 6, RES_LOOKUP, false, LOOKUP_TIME_STAMP, "0-59 = UTC second when the report was generated"},
      {"COG", BYTES(2), RES_RADIANS, false, "rad", ""},
      {"SOG", BYTES(2), 0.01, false, "m/s", ""},
      {"Communication State",
       19,
       RES_BINARY,
       false,
       0,
       "Information used by the TDMA slot allocation algorithm and synchronization information"},
      {"AIS Transceiver information", 5, RES_LOOKUP, false, LOOKUP_AIS_TRANSCEIVER, ""},
      {"Heading", BYTES(2), RES_RADIANS, false, "rad", "True heading"},
      {"Rate of Turn", BYTES(2), RES_ROTATION, true, "rad/s", ""},
      {"Nav Status", 4, RES_LOOKUP, false, LOOKUP_NAV_STATUS, ""},
      {"Special Maneuver Indicator", 2, RES_LOOKUP, false, LOOKUP_AIS_SPECIAL_MANEUVER, ""},
      {"Reserved", 2, RES_BINARY, false, 0, "reserved"},
      {"AIS Spare", 3, RES_BINARY, false, 0, ""},
      {"Reserved", 5, RES_BINARY, false, 0, "reserved"},
      {"Sequence ID", BYTES(1), RES_INTEGER, false, 0, ""},
      {0}}}

    ,
    {"AIS Class B Position Report",
     129039,
     PACKET_COMPLETE,
     PACKET_FAST,
     0x1a,
     0,
     {{"Message ID", 6, 1, false, 0, ""},
      {"Repeat Indicator", 2, RES_LOOKUP, false, LOOKUP_REPEAT_INDICATOR, ""},
      {"User ID", BYTES(4), RES_INTEGER, false, "MMSI", ""},
      {"Longitude", BYTES(4), RES_LATITUDE, true, "deg", ""},
      {"Latitude", BYTES(4), RES_LONGITUDE, true, "deg", ""},
      {"Position Accuracy", 1, RES_LOOKUP, false, LOOKUP_POSITION_ACCURACY, ""},
      {"RAIM", 1, RES_LOOKUP, false, LOOKUP_RAIM_FLAG, ""},
      {"Time Stamp", 6, RES_LOOKUP, false, LOOKUP_TIME_STAMP, "0-59 = UTC second when the report was generated"},
      {"COG", BYTES(2), RES_RADIANS, false, "rad", ""},
      {"SOG", BYTES(2), 0.01, false, "m/s", ""},
      {"Communication State",
       19,
       RES_BINARY,
       false,
       0,
       "Information used by the TDMA slot allocation algorithm and synchronization information"},
      {"AIS Transceiver information", 5, RES_LOOKUP, false, LOOKUP_AIS_TRANSCEIVER, ""},
      {"Heading", BYTES(2), RES_RADIANS, false, "rad", "True heading"},
      {"Regional Application", BYTES(1), 1, false, 0, ""},
      {"Regional Application", 2, 1, false, 0, ""},
      {"Unit type", 1, RES_LOOKUP, false, ",0=SOTDMA,1=CS", ""},
      {"Integrated Display", 1, RES_LOOKUP, false, LOOKUP_YES_NO, "Whether the unit can show messages 12 and 14"},
      {"DSC", 1, RES_LOOKUP, false, LOOKUP_YES_NO, ""},
      {"Band", 1, RES_LOOKUP, false, ",0=top 525 kHz of marine band,1=entire marine band", ""},
      {"Can handle Msg 22", 1, RES_LOOKUP, false, LOOKUP_YES_NO, "Whether device supports message 22"},
      {"AIS mode", 1, RES_LOOKUP, false, ",0=Autonomous,1=Assigned", ""},
      {"AIS communication state", 1, RES_LOOKUP, false, ",0=SOTDMA,1=ITDMA", ""},
      {0}}}

    ,
    {"AIS Class B Extended Position Report",
     129040,
     PACKET_COMPLETE,
     PACKET_FAST,
     33,
     0,
     {{"Message ID", 6, 1, false, 0, ""},
      {"Repeat Indicator", 2, RES_LOOKUP, false, LOOKUP_REPEAT_INDICATOR, ""},
      {"User ID", BYTES(4), RES_INTEGER, false, "MMSI", ""},
      {"Longitude", BYTES(4), RES_LATITUDE, true, "deg", ""},
      {"Latitude", BYTES(4), RES_LONGITUDE, true, "deg", ""},
      {"Position Accuracy", 1, RES_LOOKUP, false, LOOKUP_POSITION_ACCURACY, ""},
      {"AIS RAIM flag", 1, RES_LOOKUP, false, LOOKUP_RAIM_FLAG, ""},
      {"Time Stamp", 6, RES_LOOKUP, false, LOOKUP_TIME_STAMP, "0-59 = UTC second when the report was generated"},
      {"COG", BYTES(2), RES_RADIANS, false, "rad", ""},
      {"SOG", BYTES(2), 0.01, false, "m/s", ""},
      {"Regional Application", BYTES(1), 1, false, 0, ""},
      {"Regional Application", 4, 1, false, 0, ""},
      {"Reserved", 4, RES_BINARY, false, 0, "reserved"},
      {"Type of ship", BYTES(1), RES_LOOKUP, false, LOOKUP_SHIP_TYPE, ""},
      {"True Heading", BYTES(2), RES_RADIANS, false, "rad", ""},
      {"Reserved", 4, RES_BINARY, false, 0, ""},
      {"GNSS type", 4, RES_LOOKUP, false, LOOKUP_GNS_AIS, ""},
      {"Length", BYTES(2), 0.1, false, "m", ""},
      {"Beam", BYTES(2), 0.1, false, "m", ""},
      {"Position reference from Starboard", BYTES(2), 0.1, false, "m", ""},
      {"Position reference from Bow", BYTES(2), 0.1, false, "m", ""},
      {"Name", BYTES(20), RES_ASCII, false, 0, ",0=unavailable"},
      {"DTE", 1, RES_LOOKUP, false, ",0=Available,1=Not available", ""},
      {"AIS mode", 1, 1, false, ",0=Autonomous,1=Assigned", ""},
      {"Reserved", 4, RES_BINARY, false, 0, ""},
      {"AIS Transceiver information", 5, RES_LOOKUP, false, LOOKUP_AIS_TRANSCEIVER, ""},
      {0}}}

    ,
    {"AIS Aids to Navigation (AtoN) Report",
     129041,
     PACKET_COMPLETE,
     PACKET_FAST,
     60,
     0,
     {{"Message ID", 6, 1, false, 0, ""},
      {"Repeat Indicator", 2, RES_LOOKUP, false, LOOKUP_REPEAT_INDICATOR, ""},
      {"User ID", BYTES(4), RES_INTEGER, false, "MMSI", ""},
      {"Longitude", BYTES(4), RES_LONGITUDE, true, "deg", ""},
      {"Latitude", BYTES(4), RES_LATITUDE, true, "deg", ""},
      {"Position Accuracy", 1, RES_LOOKUP, false, LOOKUP_POSITION_ACCURACY, ""},
      {"AIS RAIM Flag", 1, RES_LOOKUP, false, LOOKUP_RAIM_FLAG, ""},
      {"Time Stamp", 6, RES_LOOKUP, false, LOOKUP_TIME_STAMP, "0-59 = UTC second when the report was generated"},
      {"Length/Diameter", BYTES(2), 0.1, false, "m", ""},
      {"Beam/Diameter", BYTES(2), 0.1, false, "m", ""},
      {"Position Reference from Starboard Edge", BYTES(2), 0.1, false, "m", ""},
      {"Position Reference from True North Facing Edge", BYTES(2), 0.1, false, "m", ""},
      {"AtoN Type", 5, RES_LOOKUP, false, LOOKUP_ATON_TYPE, ""},
      {"Off Position Indicator", 1, RES_LOOKUP, false, LOOKUP_YES_NO, ""},
      {"Virtual AtoN Flag", 1, RES_LOOKUP, false, LOOKUP_YES_NO, ""},
      {"Assigned Mode Flag", 1, RES_LOOKUP, false, LOOKUP_AIS_ASSIGNED_MODE, ""},
      {"AIS Spare", 1, RES_BINARY, false, 0, ""},
      {"Position Fixing Device Type", 4, RES_LOOKUP, false, LOOKUP_POSITION_FIX_DEVICE, ""},
      {"Reserved", 3, RES_BINARY, false, 0, ""},
      {"AtoN Status", 8, RES_BINARY, false, 0, "00000000 = default"},
      {"AIS Transceiver information", 5, RES_LOOKUP, false, LOOKUP_AIS_TRANSCEIVER, ""},
      {"Reserved", 3, RES_BINARY, false, 0, ""},
      {"AtoN Name", BYTES(34), RES_STRINGLAU, false, 0, ""},
      {0}}}

    ,
    {"Datum",
     129044,
     PACKET_COMPLETE,
     PACKET_FAST,
     20,
     0,
     {{"Local Datum",
       BYTES(4),
       RES_ASCII,
       false,
       0,
       "defined in IHO Publication S-60, Appendices B and C."
       " First three chars are datum ID as per IHO tables."
       " Fourth char is local datum subdivision code."},
      {"Delta Latitude", BYTES(4), RES_LATITUDE, true, "deg", ""},
      {"Delta Longitude", BYTES(4), RES_LONGITUDE, true, "deg", ""},
      {"Delta Altitude", BYTES(4), 1e-6, true, "m", ""},
      {"Reference Datum",
       BYTES(4),
       RES_ASCII,
       false,
       0,
       "defined in IHO Publication S-60, Appendices B and C."
       " First three chars are datum ID as per IHO tables."
       " Fourth char is local datum subdivision code."},
      {0}}}

    ,
    {"User Datum",
     129045,
     PACKET_COMPLETE,
     PACKET_FAST,
     37,
     0,
     {{"Delta X", BYTES(4), 0.01, true, "m", "Delta shift in X axis from WGS 84"},
      {"Delta Y", BYTES(4), 0.01, true, "m", "Delta shift in Y axis from WGS 84"},
      {"Delta Z", BYTES(4), 0.01, true, "m", "Delta shift in Z axis from WGS 84"},
      {"Rotation in X",
       BYTES(4),
       RES_FLOAT,
       true,
       0,
       "Rotational shift in X axis from WGS 84. Rotations presented use the geodetic sign convention.  When looking along the "
       "positive axis towards the origin, counter-clockwise rotations are positive."},
      {"Rotation in Y",
       BYTES(4),
       RES_FLOAT,
       true,
       0,
       "Rotational shift in Y axis from WGS 84. Rotations presented use the geodetic sign convention.  When looking along the "
       "positive axis towards the origin, counter-clockwise rotations are positive."},
      {"Rotation in Z",
       BYTES(4),
       RES_FLOAT,
       true,
       0,
       "Rotational shift in Z axis from WGS 84. Rotations presented use the geodetic sign convention.  When looking along the "
       "positive axis towards the origin, counter-clockwise rotations are positive."},
      {"Scale", BYTES(4), RES_FLOAT, true, "ppm", "Scale factor expressed in parts-per-million"},
      {"Ellipsoid Semi-major Axis", BYTES(4), 0.01, true, "m", "Semi-major axis (a) of the User Datum ellipsoid"},
      {"Ellipsoid Flattening Inverse", BYTES(4), RES_FLOAT, true, 0, "Flattening (1/f) of the User Datum ellipsoid"},
      {"Datum Name",
       BYTES(4),
       RES_ASCII,
       false,
       0,
       "4 character code from IHO Publication S-60,Appendices B and C."
       " First three chars are datum ID as per IHO tables."
       " Fourth char is local datum subdivision code."},
      {0}}}

    ,
    {"Cross Track Error",
     129283,
     PACKET_COMPLETE,
     PACKET_SINGLE,
     8,
     0,
     {{"SID", BYTES(1), 1, false, 0, ""},
      {"XTE mode", 4, RES_LOOKUP, false, LOOKUP_RESIDUAL_MODE, ""},
      {"Reserved", 2, RES_BINARY, false, 0, "reserved"},
      {"Navigation Terminated", 2, RES_LOOKUP, false, LOOKUP_YES_NO, ""},
      {"XTE", BYTES(4), 0.01, true, "m", ""},
      {"Reserved", BYTES(2), RES_BINARY, false, 0, "Reserved"},
      {0}}}

    ,
    {"Navigation Data",
     129284,
     PACKET_COMPLETE,
     PACKET_FAST,
     0x22,
     0,
     {{"SID", BYTES(1), 1, false, 0, ""},
      {"Distance to Waypoint", BYTES(4), 0.01, false, "m", ""},
      {"Course/Bearing reference", 2, RES_LOOKUP, false, LOOKUP_DIRECTION_REFERENCE, ""},
      {"Perpendicular Crossed", 2, RES_LOOKUP, false, LOOKUP_YES_NO, ""},
      {"Arrival Circle Entered", 2, RES_LOOKUP, false, LOOKUP_YES_NO, ""},
      {"Calculation Type", 2, RES_LOOKUP, false, ",0=Great Circle,1=Rhumb Line", ""},
      {"ETA Time", BYTES(4), RES_TIME, false, "s", "Seconds since midnight"},
      {"ETA Date", BYTES(2), RES_DATE, false, "days", "Days since January 1, 1970"},
      {"Bearing, Origin to Destination Waypoint", BYTES(2), RES_RADIANS, false, "rad", ""},
      {"Bearing, Position to Destination Waypoint", BYTES(2), RES_RADIANS, false, "rad", ""},
      {"Origin Waypoint Number", BYTES(4), 1, false, 0, ""},
      {"Destination Waypoint Number", BYTES(4), 1, false, 0, ""},
      {"Destination Latitude", BYTES(4), RES_LATITUDE, true, "deg", ""},
      {"Destination Longitude", BYTES(4), RES_LONGITUDE, true, "deg", ""},
      {"Waypoint Closing Velocity", BYTES(2), 0.01, true, "m/s", ""},
      {0}}}

    ,
    {"Navigation - Route/WP Information",
     129285,
     PACKET_COMPLETE,
     PACKET_FAST,
     233,
     4,
     {{"Start RPS#", BYTES(2), 1, false, 0, ""},
      {"nItems", BYTES(2), 1, false, 0, ""},
      {"Database ID", BYTES(2), 1, false, 0, ""},
      {"Route ID", BYTES(2), 1, false, 0, ""},
      {"Navigation direction in route", 2, 1, false, 0, ""},
      {"Supplementary Route/WP data available", 2, 1, false, 0, ""},
      {"Reserved", 4, RES_BINARY, false, 0, "Reserved"},
      {"Route Name", BYTES(255), RES_STRING, false, 0, ""},
      {"Reserved", BYTES(1), RES_BINARY, false, 0, "Reserved"},
      {"WP ID", BYTES(2), 1, false, 0, ""},
      {"WP Name", BYTES(255), RES_STRING, false, 0, ""},
      {"WP Latitude", BYTES(4), RES_LATITUDE, true, "deg", ""},
      {"WP Longitude", BYTES(4), RES_LONGITUDE, true, "deg", ""},
      {0}}}

    ,
    {"Set & Drift, Rapid Update",
     129291,
     PACKET_NOT_SEEN,
     PACKET_SINGLE,
     8,
     0,
     {{"SID", BYTES(1), 1, false, 0, ""},
      {"Set Reference", 2, RES_LOOKUP, false, LOOKUP_DIRECTION_REFERENCE, ""},
      {"Reserved", 6, RES_BINARY, false, 0, "Reserved"},
      {"Set", BYTES(2), RES_RADIANS, false, "rad", ""},
      {"Drift", BYTES(2), 0.01, false, "m/s", ""},
      {0}}}

    ,
    {"Navigation - Route / Time to+from Mark",
     129301,
     PACKET_INCOMPLETE | PACKET_NOT_SEEN,
     PACKET_FAST,
     10,
     0,
     {{"SID", BYTES(1), 1, false, 0, ""},
      {"Time to mark", BYTES(4), 0.001, true, "s", "negative = elapsed since event, positive = time to go"},
      {"Mark Type", 4, RES_LOOKUP, false, ",0=Collision,1=Turning point,2=Reference,3=Wheelover,4=Waypoint", ""},
      {"Reserved", 4, RES_BINARY, false, 0, "Reserved"},
      {"Mark ID", BYTES(4), 1, false, 0, ""},
      {0}}}

    ,
    {"Bearing and Distance between two Marks",
     129302,
     PACKET_INCOMPLETE | PACKET_NOT_SEEN,
     PACKET_FAST,
     8,
     0,
     {{"SID", BYTES(1), 1, false, 0, ""},
      {"Bearing Reference", 4, RES_LOOKUP, false, 0, ""},
      {"Calculation Type", 2, RES_LOOKUP, false, 0, ""},
      {"Reserved", 2, RES_BINARY, false, 0, "Reserved"},
      {"Bearing, Origin to Destination", BYTES(2), RES_RADIANS, false, "rad", ""},
      {"Distance", BYTES(4), 0.01, false, "m", ""},
      {"Origin Mark Type", 4, RES_LOOKUP, false, 0, ""},
      {"Destination Mark Type", 4, RES_LOOKUP, false, 0, ""},
      {"Origin Mark ID", BYTES(4), 1, false, 0, ""},
      {"Destination Mark ID", BYTES(4), 1, false, 0, ""},
      {0}}}

    /* http://www.maretron.com/support/manuals/GPS100UM_1.2.pdf */
    /* Haven't seen this yet (no way to send PGN 059904 yet) so lengths unknown */
    ,
    {"GNSS Control Status",
     129538,
     PACKET_INCOMPLETE | PACKET_NOT_SEEN,
     PACKET_FAST,
     13,
     0,
     {{"SV Elevation Mask", BYTES(2), 1, false, 0, "Will not use SV below this elevation"},
      {"PDOP Mask", BYTES(2), 0.01, false, 0, "Will not report position above this PDOP"},
      {"PDOP Switch", BYTES(2), 0.01, false, 0, "Will report 2D position above this PDOP"},
      {"SNR Mask", BYTES(2), 0.01, false, 0, "Will not use SV below this SNR"},
      {"GNSS Mode (desired)", 3, RES_LOOKUP, false, ",0=1D,1=2D,2=3D,3=Auto,4=Reserved,5=Reserved,6=Error", ""},
      {"DGNSS Mode (desired)", 3, RES_LOOKUP, false, ",0=no SBAS,1=SBAS,3=SBAS", ""},
      {"Position/Velocity Filter", 2, 1, false, 0, ""},
      {"Max Correction Age", BYTES(2), 1, false, 0, ""},
      {"Antenna Altitude for 2D Mode", BYTES(2), 0.01, false, "m", ""},
      {"Use Antenna Altitude for 2D Mode", 2, RES_LOOKUP, false, ",0=use last 3D height,1=Use antenna altitude", ""},
      {0}}}

    /* http://www.maretron.com/support/manuals/GPS100UM_1.2.pdf */
    ,
    {"GNSS DOPs",
     129539,
     PACKET_COMPLETE,
     PACKET_SINGLE,
     8,
     0,
     {{"SID", BYTES(1), 1, false, 0, ""},
      {"Desired Mode", 3, RES_LOOKUP, false, ",0=1D,1=2D,2=3D,3=Auto", ""},
      {"Actual Mode", 3, RES_LOOKUP, false, ",0=1D,1=2D,2=3D,3=Auto", ""},
      {"Reserved", 2, RES_BINARY, false, 0, "Reserved"},
      {"HDOP", BYTES(2), 0.01, true, 0, "Horizontal dilution of precision"},
      {"VDOP", BYTES(2), 0.01, true, 0, "Vertical dilution of precision"},
      {"TDOP", BYTES(2), 0.01, true, 0, "Time dilution of precision"},
      {0}}}

    ,
    {"GNSS Sats in View",
     129540,
     PACKET_COMPLETE,
     PACKET_FAST,
     233,
     7,
     {{"SID", BYTES(1), 1, false, 0, ""},
      {"Mode", 2, RES_LOOKUP, false, ",3=Range residuals used to calculate position", ""},
      {"Reserved", 6, RES_BINARY, false, 0, "Reserved"},
      {"Sats in View", BYTES(1), 1, false, 0, ""},
      {"PRN", BYTES(1), 1, false, 0, ""},
      {"Elevation", BYTES(2), RES_RADIANS, false, "rad", ""},
      {"Azimuth", BYTES(2), RES_RADIANS, false, "rad", ""},
      {"SNR", BYTES(2), 0.01, false, "dB", ""},
      {"Range residuals", BYTES(4), 1, true, 0, ""},
      {"Status", 4, RES_LOOKUP, false, ",0=Not tracked,1=Tracked,2=Used,3=Not tracked+Diff,4=Tracked+Diff,5=Used+Diff", ""},
      {"Reserved", 4, RES_BINARY, false, 0, "Reserved"},
      {0}}}

    ,
    {"GPS Almanac Data",
     129541,
     PACKET_INCOMPLETE,
     PACKET_FAST,
     26,
     0,
     {{"PRN", BYTES(1), RES_INTEGER, false, 0, ""},
      {"GPS Week number", BYTES(2), RES_INTEGER, false, 0, ""},
      {"SV Health Bits", BYTES(1), RES_BINARY, false, 0, ""},
      {"Eccentricity", BYTES(2), 1e-21, false, "m/m", ""},
      {"Almanac Reference Time", BYTES(1), 1e12, false, "s", ""},
      {"Inclination Angle", BYTES(2), 1e-19, true, "semi-circle", ""},
      {"Rate of Right Ascension", BYTES(2), 1e-38, true, "semi-circle/s", ""},
      {"Root of Semi-major Axis", BYTES(3), 1e-11, false, "sqrt(m)", ""},
      {"Argument of Perigee", BYTES(3), 1e-23, true, "semi-circle", ""},
      {"Longitude of Ascension Node", BYTES(3), 1e-23, true, "semi-circle", ""},
      {"Mean Anomaly", BYTES(3), 1e-23, true, "semi-circle", ""},
      {"Clock Parameter 1", 11, 1e-20, true, "s", ""},
      {"Clock Parameter 2", 11, 1e-38, true, "s/s", ""},
      {"Reserved", 2, RES_BINARY, false, 0, "Reserved"},
      {0}}}

    ,
    {"GNSS Pseudorange Noise Statistics",
     129542,
     PACKET_INCOMPLETE | PACKET_NOT_SEEN,
     PACKET_FAST,
     9,
     0,
     {{"SID", BYTES(1), 1, false, 0, ""},
      {"RMS of Position Uncertainty", BYTES(2), 1, false, 0, ""},
      {"STD of Major axis", BYTES(1), 1, false, 0, ""},
      {"STD of Minor axis", BYTES(1), 1, false, 0, ""},
      {"Orientation of Major axis", BYTES(1), 1, false, 0, ""},
      {"STD of Lat Error", BYTES(1), 1, false, 0, ""},
      {"STD of Lon Error", BYTES(1), 1, false, 0, ""},
      {"STD of Alt Error", BYTES(1), 1, false, 0, ""},
      {0}}}

    ,
    {"GNSS RAIM Output",
     129545,
     PACKET_INCOMPLETE,
     PACKET_FAST,
     9,
     0,
     {{"SID", BYTES(1), 1, false, 0, ""},
      {"Integrity flag", 4, 1, false, 0, ""},
      {"Reserved", 4, RES_BINARY, false, 0, "Reserved"},
      {"Latitude expected error", BYTES(1), 1, false, 0, ""},
      {"Longitude expected error", BYTES(1), 1, false, 0, ""},
      {"Altitude expected error", BYTES(1), 1, false, 0, ""},
      {"SV ID of most likely failed sat", BYTES(1), 1, false, 0, ""},
      {"Probability of missed detection", BYTES(1), 1, false, 0, ""},
      {"Estimate of pseudorange bias", BYTES(1), 1, false, 0, ""},
      {"Std Deviation of bias", BYTES(1), 1, false, 0, ""},
      {0}}}

    ,
    {"GNSS RAIM Settings",
     129546,
     PACKET_INCOMPLETE,
     PACKET_SINGLE,
     8,
     0,
     {{"Radial Position Error Maximum Threshold", BYTES(1), 1, false, 0, ""},
      {"Probability of False Alarm", BYTES(1), 1, false, 0, ""},
      {"Probability of Missed Detection", BYTES(1), 1, false, 0, ""},
      {"Pseudorange Residual Filtering Time Constant", BYTES(1), 1, false, 0, ""},
      {0}}}

    ,
    {"GNSS Pseudorange Error Statistics",
     129547,
     PACKET_INCOMPLETE | PACKET_NOT_SEEN,
     PACKET_FAST,
     9,
     0,
     {{"SID", BYTES(1), 1, false, 0, ""},
      {"RMS Std Dev of Range Inputs", BYTES(2), 1, false, 0, ""},
      {"Std Dev of Major error ellipse", BYTES(1), 1, false, 0, ""},
      {"Std Dev of Minor error ellipse", BYTES(1), 1, false, 0, ""},
      {"Orientation of error ellipse", BYTES(1), 1, false, 0, ""},
      {"Std Dev Lat Error", BYTES(1), 1, false, 0, ""},
      {"Std Dev Lon Error", BYTES(1), 1, false, 0, ""},
      {"Std Dev Alt Error", BYTES(1), 1, false, 0, ""},
      {0}}}

    ,
    {"DGNSS Corrections",
     129549,
     PACKET_INCOMPLETE | PACKET_NOT_SEEN,
     PACKET_FAST,
     13,
     0,
     {{"SID", BYTES(1), 1, false, 0, ""},
      {"Reference Station ID", BYTES(2), 1, false, 0, ""},
      {"Reference Station Type", BYTES(2), 1, false, 0, ""},
      {"Time of corrections", BYTES(1), 1, false, 0, ""},
      {"Station Health", BYTES(1), 1, false, 0, ""},
      {"Reserved Bits", BYTES(1), RES_BINARY, false, 0, ""},
      {"Satellite ID", BYTES(1), 1, false, 0, ""},
      {"PRC", BYTES(1), 1, false, 0, ""},
      {"RRC", BYTES(1), 1, false, 0, ""},
      {"UDRE", BYTES(1), 1, false, 0, ""},
      {"IOD", BYTES(1), 1, false, 0, ""},
      {0}}}

    ,
    {"GNSS Differential Correction Receiver Interface",
     129550,
     PACKET_INCOMPLETE | PACKET_NOT_SEEN,
     PACKET_FAST,
     8,
     0,
     {{"Channel", BYTES(1), 1, false, 0, ""},
      {"Frequency", BYTES(1), 1, false, 0, ""},
      {"Serial Interface Bit Rate", BYTES(1), 1, false, 0, ""},
      {"Serial Interface Detection Mode", BYTES(1), 1, false, 0, ""},
      {"Differential Source", BYTES(1), 1, false, 0, ""},
      {"Differential Operation Mode", BYTES(1), 1, false, 0, ""},
      {0}}}

    ,
    {"GNSS Differential Correction Receiver Signal",
     129551,
     PACKET_INCOMPLETE | PACKET_NOT_SEEN,
     PACKET_FAST,
     8,
     0,
     {{"SID", BYTES(1), 1, false, 0, ""},
      {"Channel", BYTES(1), 1, false, 0, ""},
      {"Signal Strength", BYTES(1), 1, false, 0, ""},
      {"Signal SNR", BYTES(1), 1, false, 0, ""},
      {"Frequency", BYTES(1), 1, false, 0, ""},
      {"Station Type", BYTES(1), 1, false, 0, ""},
      {"Station ID", BYTES(1), 1, false, 0, ""},
      {"Differential Signal Bit Rate", BYTES(1), 1, false, 0, ""},
      {"Differential Signal Detection Mode", BYTES(1), 1, false, 0, ""},
      {"Used as Correction Source", BYTES(1), 1, false, 0, ""},
      {"Reserved", BYTES(1), RES_BINARY, false, 0, "Reserved"},
      {"Differential Source", BYTES(1), 1, false, 0, ""},
      {"Time since Last Sat Differential Sync", BYTES(1), 1, false, 0, ""},
      {"Satellite Service ID No.", BYTES(1), 1, false, 0, ""},
      {0}}}

    ,
    {"GLONASS Almanac Data",
     129556,
     PACKET_INCOMPLETE | PACKET_NOT_SEEN,
     PACKET_FAST,
     8,
     0,
     {{"PRN", BYTES(1), 1, false, 0, ""},
      {"NA", BYTES(1), 1, false, 0, ""},
      {"CnA", BYTES(1), 1, false, 0, ""},
      {"HnA", BYTES(1), 1, false, 0, ""},
      {"(epsilon)nA", BYTES(1), 1, false, 0, ""},
      {"(deltaTnA)DOT", BYTES(1), 1, false, 0, ""},
      {"(omega)nA", BYTES(1), 1, false, 0, ""},
      {"(delta)TnA", BYTES(1), 1, false, 0, ""},
      {"tnA", BYTES(1), 1, false, 0, ""},
      {"(lambda)nA", BYTES(1), 1, false, 0, ""},
      {"(delta)inA", BYTES(1), 1, false, 0, ""},
      {"tcA", BYTES(1), 1, false, 0, ""},
      {"tnA", BYTES(1), 1, false, 0, ""},
      {0}}}

    ,
    {"AIS DGNSS Broadcast Binary Message",
     129792,
     PACKET_INCOMPLETE | PACKET_NOT_SEEN,
     PACKET_FAST,
     8,
     0,
     {{"Message ID", 6, 1, false, 0, ""},
      {"Repeat Indicator", 2, 1, false, 0, ""},
      {"Source ID", BYTES(4), 1, false, "MMSI", ""},
      {"NMEA 2000 Reserved", BYTES(1), RES_BINARY, false, 0, ""},
      {"AIS Transceiver Information", BYTES(1), 1, false, 0, ""},
      {"Spare", BYTES(1), 1, false, 0, ""},
      {"Longitude", BYTES(4), 1, false, 0, ""},
      {"Latitude", BYTES(4), 1, false, 0, ""},
      {"NMEA 2000 Reserved", BYTES(1), RES_BINARY, false, 0, ""},
      {"Spare", BYTES(1), 1, false, 0, ""},
      {"Number of Bits in Binary Data Field", BYTES(1), 1, false, 0, ""},
      {"Binary Data", BYTES(8), RES_BINARY, false, 0, ""},
      {0}}}

    ,
    {"AIS UTC and Date Report",
     129793,
     PACKET_INCOMPLETE,
     PACKET_FAST,
     0x1a,
     0,
     {{"Message ID", 6, 1, false, 0, ""},
      {"Repeat Indicator", 2, RES_LOOKUP, false, LOOKUP_REPEAT_INDICATOR, ""},
      {"User ID", BYTES(4), RES_INTEGER, false, "MMSI", ""},
      {"Longitude", BYTES(4), RES_LONGITUDE, true, "deg", ""},
      {"Latitude", BYTES(4), RES_LATITUDE, true, "deg", ""},
      {"Position Accuracy", 1, RES_LOOKUP, false, ",0=Low,1=High", ""},
      {"RAIM", 1, RES_LOOKUP, false, ",0=not in use,1=in use", ""},
      {"Reserved", 6, RES_BINARY, false, 0, "NMEA reserved to align next data on byte boundary"},
      {"Position Time", BYTES(4), RES_TIME, false, "s", "Seconds since midnight"},
      {"Communication State",
       19,
       RES_BINARY,
       false,
       0,
       "Information used by the TDMA slot allocation algorithm and synchronization information"},
      {"AIS Transceiver information", 5, RES_LOOKUP, false, LOOKUP_AIS_TRANSCEIVER, ""},
      {"Position Date", BYTES(2), RES_DATE, false, "days", "Days since January 1, 1970"},
      {"Reserved", 4, RES_BINARY, false, 0, "NMEA reserved to align next data on byte boundary"},
      {"GNSS type", 4, RES_LOOKUP, false, LOOKUP_GNS_AIS, ""},
      {"Spare", BYTES(1), RES_BINARY, false, 0, ""},
      {0}}}

    /* http://www.navcen.uscg.gov/enav/ais/AIS_messages.htm */
    ,
    {"AIS Class A Static and Voyage Related Data",
     129794,
     PACKET_COMPLETE,
     PACKET_FAST,
     0x18,
     0,
     {{"Message ID", 6, 1, false, 0, ""},
      {"Repeat indicator", 2, RES_LOOKUP, false, LOOKUP_REPEAT_INDICATOR, ""},
      {"User ID", BYTES(4), RES_INTEGER, false, "MMSI", ""},
      {"IMO number", BYTES(4), RES_INTEGER, false, 0, ",0=unavailable"},
      {"Callsign", BYTES(7), RES_ASCII, false, 0, ",0=unavailable"},
      {"Name", BYTES(20), RES_ASCII, false, 0, ",0=unavailable"},
      {"Type of ship", BYTES(1), RES_LOOKUP, false, LOOKUP_SHIP_TYPE, ""},
      {"Length", BYTES(2), 0.1, false, "m", ""},
      {"Beam", BYTES(2), 0.1, false, "m", ""},
      {"Position reference from Starboard", BYTES(2), 0.1, false, "m", ""},
      {"Position reference from Bow", BYTES(2), 0.1, false, "m", ""},
      {"ETA Date", BYTES(2), RES_DATE, false, "days", "Days since January 1, 1970"},
      {"ETA Time", BYTES(4), RES_TIME, false, "s", "Seconds since midnight"},
      {"Draft", BYTES(2), 0.01, false, "m", ""},
      {"Destination", BYTES(20), RES_ASCII, false, 0, ",0=unavailable"},
      {"AIS version indicator", 2, RES_LOOKUP, false, ",0=ITU-R M.1371-1,1=ITU-R M.1371-3", ""},
      {"GNSS type", 4, RES_LOOKUP, false, LOOKUP_GNS_AIS, ""},
      {"DTE", 1, RES_LOOKUP, false, ",0=available,1=not available", ""},
      {"Reserved", 1, RES_BINARY, false, 0, "reserved"},
      {"AIS Transceiver information", 5, RES_LOOKUP, false, LOOKUP_AIS_TRANSCEIVER, ""},
      {0}}}

    ,
    {"AIS Addressed Binary Message",
     129795,
     PACKET_COMPLETE,
     PACKET_FAST,
     13,
     0,
     {{"Message ID", 6, 1, false, 0, ""},
      {"Repeat Indicator", 2, RES_LOOKUP, false, LOOKUP_REPEAT_INDICATOR, ""},
      {"Source ID", BYTES(4), RES_INTEGER, false, "MMSI", ""},
      {"Reserved", 1, RES_BINARY, false, 0, "reserved"},
      {"AIS Transceiver information", 5, RES_LOOKUP, false, LOOKUP_AIS_TRANSCEIVER, ""},
      {"Sequence Number", 2, 1, false, 0, ""},
      {"Destination ID", BYTES(4), RES_INTEGER, false, "MMSI", ""},
      {"Reserved", 6, RES_BINARY, false, 0, "reserved"},
      {"Retransmit flag", 1, 1, false, 0, ""},
      {"Reserved", 1, RES_BINARY, false, 0, "reserved"},
      {"Number of Bits in Binary Data Field", BYTES(2), RES_INTEGER, false, 0, ""},
      {"Binary Data", BYTES(8), RES_BINARY, false, 0, ""},
      {0}}}

    ,
    {"AIS Acknowledge",
     129796,
     PACKET_COMPLETE,
     PACKET_FAST,
     12,
     0,
     {{"Message ID", 6, 1, false, 0, ""},
      {"Repeat Indicator", 2, RES_LOOKUP, false, LOOKUP_REPEAT_INDICATOR, ""},
      {"Source ID", BYTES(4), 1, false, "MMSI", ""},
      {"Reserved", 1, RES_BINARY, false, 0, "reserved"},
      {"AIS Transceiver information", 5, RES_LOOKUP, false, LOOKUP_AIS_TRANSCEIVER, ""},
      {"Reserved", 2, RES_BINARY, false, 0, "reserved"},
      {"Destination ID #1", BYTES(4), 1, false, 0, ""},
      {"Sequence Number for ID 1", 2, RES_BINARY, false, 0, "reserved"},
      {"Reserved", 6, RES_BINARY, false, 0, "reserved"},
      {"Sequence Number for ID n", 2, RES_BINARY, false, 0, "reserved"},
      {0}}}

    ,
    {"AIS Binary Broadcast Message",
     129797,
     PACKET_COMPLETE,
     PACKET_FAST,
     233,
     0,
     {{"Message ID", 6, 1, false, 0, ""},
      {"Repeat Indicator", 2, RES_LOOKUP, false, LOOKUP_REPEAT_INDICATOR, ""},
      {"Source ID", BYTES(4), 1, false, 0, ""},
      {"Reserved", 1, RES_BINARY, false, 0, "reserved"},
      {"AIS Transceiver information", 5, RES_LOOKUP, false, LOOKUP_AIS_TRANSCEIVER, ""},
      {"Reserved", 2, RES_BINARY, false, 0, "reserved"},
      {"Number of Bits in Binary Data Field", BYTES(2), 1, false, 0, ""},
      {"Binary Data", BYTES(255), RES_BINARY, false, 0, ""},
      {0}}}

    ,
    {"AIS SAR Aircraft Position Report",
     129798,
     PACKET_INCOMPLETE | PACKET_NOT_SEEN,
     PACKET_FAST,
     8,
     0,
     {{"Message ID", 6, 1, false, 0, ""},
      {"Repeat indicator", 2, RES_LOOKUP, false, LOOKUP_REPEAT_INDICATOR, ""},
      {"User ID", BYTES(4), RES_INTEGER, false, "MMSI", ""},
      {"Longitude", BYTES(4), RES_LONGITUDE, true, "deg", ""},
      {"Latitude", BYTES(4), RES_LATITUDE, true, "deg", ""},
      {"Position Accuracy", 1, RES_LOOKUP, false, LOOKUP_POSITION_ACCURACY, ""},
      {"RAIM", 1, RES_LOOKUP, false, LOOKUP_RAIM_FLAG, ""},
      {"Time Stamp", 6, RES_LOOKUP, false, LOOKUP_TIME_STAMP, "0-59 = UTC second when the report was generated"},
      {"COG", BYTES(2), RES_RADIANS, false, "rad", ""},
      {"SOG", BYTES(2), 0.1, false, "m/s", ""},
      {"Communication State",
       19,
       RES_BINARY,
       false,
       0,
       "Information used by the TDMA slot allocation algorithm and synchronization information"},
      {"AIS Transceiver information", 5, RES_LOOKUP, false, LOOKUP_AIS_TRANSCEIVER, ""},
      {"Altitude", BYTES(8), 1e-6, true, "m", ""},
      {"Reserved for Regional Applications", BYTES(1), RES_BINARY, false, 0, ""},
      {"DTE", 1, RES_LOOKUP, false, ",0=Available,1=Not available", ""},
      {"Reserved", 7, RES_BINARY, false, 0, "reserved"},
      {0}}}

    ,
    {"Radio Frequency/Mode/Power",
     129799,
     PACKET_INCOMPLETE,
     PACKET_FAST,
     9,
     0,
     {{"Rx Frequency", BYTES(4), 10, false, "Hz", ""},
      {"Tx Frequency", BYTES(4), 10, false, "Hz", ""},
      {"Radio Channel", BYTES(1), 1, false, 0, ""},
      {"Tx Power", BYTES(1), 1, false, 0, ""},
      {"Mode", BYTES(1), 1, false, 0, ""},
      {"Channel Bandwidth", BYTES(1), 1, false, 0, ""},
      {0}}}

    ,
    {"AIS UTC/Date Inquiry",
     129800,
     PACKET_INCOMPLETE,
     PACKET_FAST,
     8,
     0,
     {{"Message ID", 6, 1, false, 0, ""},
      {"Repeat Indicator", 2, RES_LOOKUP, false, LOOKUP_REPEAT_INDICATOR, ""},
      {"Source ID", 30, 1, false, 0, ""},
      {"Reserved", 2, RES_BINARY, false, 0, "reserved"},
      {"AIS Transceiver information", 5, RES_LOOKUP, false, LOOKUP_AIS_TRANSCEIVER, ""},
      {"Reserved", 3, RES_BINARY, false, 0, "reserved"},
      {"Destination ID", 30, 1, false, 0, ""},
      {"Reserved", 2, RES_BINARY, false, 0, "reserved"},
      {0}}}

    ,
    {"AIS Addressed Safety Related Message",
     129801,
     PACKET_INCOMPLETE,
     PACKET_FAST,
     12,
     0,
     {{"Message ID", 6, 1, false, 0, ""},
      {"Repeat Indicator", 2, RES_LOOKUP, false, LOOKUP_REPEAT_INDICATOR, ""},
      {"Source ID", 30, 1, false, "MMSI", ""},
      {"Reserved", 2, RES_BINARY, false, 0, "reserved"},
      {"AIS Transceiver information", 5, RES_LOOKUP, false, LOOKUP_AIS_TRANSCEIVER, ""},
      {"Sequence Number", 2, 1, false, 0, ""},
      {"Destination ID", 30, 1, false, "MMSI", ""},
      {"Reserved", 2, RES_BINARY, false, 0, "reserved"},
      {"Retransmit flag", 1, 1, false, 0, ""},
      {"Reserved", 7, RES_BINARY, false, 0, "reserved"},
      {"Safety Related Text", BYTES(255), RES_ASCII, false, 0, ""},
      {0}}}

    ,
    {"AIS Safety Related Broadcast Message",
     129802,
     PACKET_INCOMPLETE,
     PACKET_FAST,
     8,
     0,
     {{"Message ID", 6, 1, false, 0, ""},
      {"Repeat Indicator", 2, RES_LOOKUP, false, LOOKUP_REPEAT_INDICATOR, ""},
      {"Source ID", 30, RES_INTEGER, false, 0, ""},
      {"Reserved", 2, RES_BINARY, false, 0, "reserved"},
      {"AIS Transceiver information", 5, RES_LOOKUP, false, LOOKUP_AIS_TRANSCEIVER, ""},
      {"Reserved", 3, RES_BINARY, false, 0, "reserved"},
      {"Safety Related Text", BYTES(36), RES_ASCII, false, 0, ""},
      {0}}}

    ,
    {"AIS Interrogation",
     129803,
     PACKET_INCOMPLETE,
     PACKET_SINGLE,
     8,
     8,
     {{"Message ID", 6, 1, false, 0, ""},
      {"Repeat Indicator", 2, RES_LOOKUP, false, LOOKUP_REPEAT_INDICATOR, ""},
      {"Source ID", 30, RES_INTEGER, false, 0, ""},
      {"Reserved", 2, RES_BINARY, false, 0, "reserved"},
      {"AIS Transceiver information", 5, RES_LOOKUP, false, LOOKUP_AIS_TRANSCEIVER, ""},
      {"Reserved", 3, RES_BINARY, false, 0, "reserved"}

      ,
      {"Destination ID", 30, RES_INTEGER, false, 0, ""},
      {"Reserved", 2, RES_BINARY, false, 0, "reserved"},
      {"Message ID A", BYTES(1), RES_INTEGER, false, 0, ""},
      {"Slot Offset A", 14, RES_INTEGER, false, 0, ""},
      {"Reserved", 2, RES_BINARY, false, 0, "reserved"},
      {"Message ID B", BYTES(1), RES_INTEGER, false, 0, ""},
      {"Slot Offset B", 14, RES_INTEGER, false, 0, ""},
      {"Reserved", 2, RES_BINARY, false, 0, "reserved"},
      {0}}}

    ,
    {"AIS Assignment Mode Command",
     129804,
     PACKET_INCOMPLETE | PACKET_NOT_SEEN,
     PACKET_FAST,
     23,
     3,
     {{"Message ID", 6, 1, false, 0, ""},
      {"Repeat Indicator", 2, RES_LOOKUP, false, LOOKUP_REPEAT_INDICATOR, ""},
      {"Source ID", 30, RES_INTEGER, false, "MMSI", ""},
      {"Reserved", 2, RES_BINARY, false, 0, "reserved"},
      {"AIS Transceiver information", 5, RES_LOOKUP, false, LOOKUP_AIS_TRANSCEIVER, ""},
      {"Reserved", 3, RES_BINARY, false, 0, "reserved"}

      ,
      {"Destination ID", BYTES(4), RES_INTEGER, false, "MMSI", ""},
      {"Offset", BYTES(2), RES_INTEGER, false, 0, ""},
      {"Increment", BYTES(2), RES_INTEGER, false, 0, ""},
      {0}}}

    ,
    {"AIS Data Link Management Message",
     129805,
     PACKET_INCOMPLETE,
     PACKET_FAST,
     8,
     4,
     {{"Message ID", 6, 1, false, 0, ""},
      {"Repeat Indicator", 2, RES_LOOKUP, false, LOOKUP_REPEAT_INDICATOR, ""},
      {"Source ID", 30, RES_INTEGER, false, 0, ""},
      {"Reserved", 2, RES_BINARY, false, 0, "reserved"},
      {"AIS Transceiver information", 5, RES_LOOKUP, false, LOOKUP_AIS_TRANSCEIVER, ""},
      {"Reserved", 3, RES_BINARY, false, 0, "reserved"}

      ,
      {"Offset", 10, RES_INTEGER, false, 0, ""},
      {"Number of Slots", BYTES(1), RES_INTEGER, false, 0, ""},
      {"Timeout", BYTES(1), RES_INTEGER, false, 0, ""},
      {"Increment", BYTES(1), RES_INTEGER, false, 0, ""},
      {0}}}

    ,
    {"AIS Channel Management",
     129806,
     PACKET_INCOMPLETE,
     PACKET_FAST,
     8,
     0,
     {{"Message ID", 6, 1, false, 0, ""},
      {"Repeat Indicator", 2, RES_LOOKUP, false, LOOKUP_REPEAT_INDICATOR, ""},
      {"Source ID", 30, RES_INTEGER, false, 0, ""},
      {"Reserved", 2, RES_BINARY, false, 0, "reserved"},
      {"AIS Transceiver information", 5, RES_LOOKUP, false, LOOKUP_AIS_TRANSCEIVER, ""},
      {"Reserved", 3, RES_BINARY, false, 0, "reserved"},
      {"Channel A", 7, 1, false, 0, ""},
      {"Channel B", 7, 1, false, 0, ""},
      {"Reserved", 2, RES_BINARY, false, 0, "reserved"},
      {"Power", BYTES(1), 1, false, 0, "reserved"},
      {"Tx/Rx Mode", BYTES(1), RES_INTEGER, false, 0, ""},
      {"North East Longitude Corner 1", BYTES(4), RES_LONGITUDE, true, "deg", ""},
      {"North East Latitude Corner 1", BYTES(4), RES_LATITUDE, true, "deg", ""},
      {"South West Longitude Corner 1", BYTES(4), RES_LONGITUDE, true, "deg", ""},
      {"South West Latitude Corner 2", BYTES(4), RES_LATITUDE, true, "deg", ""},
      {"Reserved", 6, RES_BINARY, false, 0, "reserved"},
      {"Addressed or Broadcast Message Indicator", 2, 1, false, 0, ""},
      {"Channel A Bandwidth", 7, RES_INTEGER, false, 0, ""},
      {"Channel B Bandwidth", 7, RES_INTEGER, false, 0, ""},
      {"Reserved", 2, RES_BINARY, false, 0, "reserved"},
      {"Transitional Zone Size", BYTES(1), 1, false, 0, ""},
      {0}}}

    ,
    {"AIS Class B Group Assignment",
     129807,
     PACKET_INCOMPLETE,
     PACKET_FAST,
     8,
     0,
     {{"Message ID", 6, 1, false, 0, ""},
      {"Repeat Indicator", 2, RES_LOOKUP, false, LOOKUP_REPEAT_INDICATOR, ""},
      {"Source ID", 30, RES_INTEGER, false, 0, ""},
      {"Reserved", 2, RES_BINARY, false, 0, "reserved"},
      {"Tx/Rx Mode", 2, RES_INTEGER, false, 0, ""},
      {"Reserved", 6, RES_BINARY, false, 0, "reserved"},
      {"North East Longitude Corner 1", BYTES(4), RES_LONGITUDE, true, "deg", ""},
      {"North East Latitude Corner 1", BYTES(4), RES_LATITUDE, true, "deg", ""},
      {"South West Longitude Corner 1", BYTES(4), RES_LONGITUDE, true, "deg", ""},
      {"South West Latitude Corner 2", BYTES(4), RES_LATITUDE, true, "deg", ""},
      {"Station Type", 6, 1, false, 0, ""},
      {"Reserved", 2, RES_BINARY, false, 0, "reserved"},
      {"Ship and Cargo Filter", 6, 1, false, 0, ""},
      {"Reserved", 2, RES_BINARY, false, 0, "reserved"},
      {"Reporting Interval", BYTES(2), 1, false, 0, ""},
      {"Quiet Time", BYTES(2), 1, false, 0, ""},
      {0}}}

    /* http://www.nmea.org/Assets/2000_20150328%20dsc%20technical%20corrigendum%20database%20version%202.100.pdf */
    /* This is like the worst PGN ever.
     * 1. The "Nature of Distress or 1st Telecommand' field meaning depends on the 'DSC Category'.
     * 2. The "Message address" (the 'to' field) meaning depends on the 'DSC format'.
     * 3. Field 12 'MMSI of ship in destress' may have multiple interpretations.
     * 4. Field 22 'DSC expansion field data' depends on field 21 for its meaning.
     * 5. It contains a variable length field 'Telephone number', which means that bit offsets for subsequent fields
     *    depend on this field's length.
     *
     * We solve #1 here by having two definitions.
     */

    ,
    {"DSC Distress Call Information",
     129808,
     PACKET_INCOMPLETE | PACKET_NOT_SEEN,
     PACKET_FAST,
     8,
     2,
     {{"DSC Format", BYTES(1), RES_LOOKUP, false, LOOKUP_DSC_FORMAT, ""},
      {"DSC Category", BYTES(1), RES_LOOKUP, false, "=112", "Distress"},
      {"DSC Message Address", BYTES(5), RES_DECIMAL, false, 0, "MMSI, Geographic Area or blank"},
      {"Nature of Distress", BYTES(1), RES_LOOKUP, false, LOOKUP_DSC_NATURE, ""},
      {"Subsequent Communication Mode or 2nd Telecommand", BYTES(1), RES_LOOKUP, false, LOOKUP_DSC_SECOND_TELECOMMAND, ""},
      {"Proposed Rx Frequency/Channel", BYTES(6), RES_ASCII, false, 0, ""},
      {"Proposed Tx Frequency/Channel", BYTES(6), RES_ASCII, false, 0, ""},
      {"Telephone Number", BYTES(2), RES_STRINGLAU, false, 0, ""},
      {"Latitude of Vessel Reported",
       BYTES(4),
       RES_LATITUDE,
       true,
       "deg",
       "offset depends on previous field, as do all following fields"},
      {"Longitude of Vessel Reported", BYTES(4), RES_LONGITUDE, true, "deg", ""},
      {"Time of Position", BYTES(4), RES_TIME, false, "s", "Seconds since midnight"},
      {"MMSI of Ship In Distress", BYTES(5), RES_DECIMAL, false, "MMSI", ""},
      {"DSC EOS Symbol", BYTES(1), 1, false, 0, ""},
      {"Expansion Enabled", 2, RES_LOOKUP, false, LOOKUP_YES_NO, ""},
      {"Reserved", 6, RES_BINARY, false, 0, "reserved"},
      {"Calling Rx Frequency/Channel", BYTES(6), RES_ASCII, false, 0, ""},
      {"Calling Tx Frequency/Channel", BYTES(6), RES_ASCII, false, 0, ""},
      {"Time of Receipt", BYTES(4), RES_TIME, false, "s", "Seconds since midnight"},
      {"Date of Receipt", BYTES(2), RES_DATE, false, "days", "Days since January 1, 1970"},
      {"DSC Equipment Assigned Message ID", BYTES(2), 1, false, 0, ""},
      {"DSC Expansion Field Symbol", BYTES(1), RES_LOOKUP, false, LOOKUP_DSC_EXPANSION_DATA, ""},
      {"DSC Expansion Field Data", BYTES(2), RES_STRINGLAU, false, 0, ""},
      {0}}}

    ,
    {"DSC Call Information",
     129808,
     PACKET_INCOMPLETE | PACKET_NOT_SEEN,
     PACKET_FAST,
     8,
     2,
     {{"DSC Format Symbol", BYTES(1), RES_LOOKUP, false, LOOKUP_DSC_FORMAT, ""},
      {"DSC Category Symbol", BYTES(1), RES_LOOKUP, false, LOOKUP_DSC_CATEGORY, ""},
      {"DSC Message Address", BYTES(5), RES_DECIMAL, false, 0, "MMSI, Geographic Area or blank"},
      {"1st Telecommand", BYTES(1), RES_LOOKUP, false, LOOKUP_DSC_FIRST_TELECOMMAND, ""},
      {"Subsequent Communication Mode or 2nd Telecommand", BYTES(1), RES_LOOKUP, false, LOOKUP_DSC_SECOND_TELECOMMAND, ""},
      {"Proposed Rx Frequency/Channel", BYTES(6), RES_ASCII, false, 0, ""},
      {"Proposed Tx Frequency/Channel", BYTES(6), RES_ASCII, false, 0, ""},
      {"Telephone Number", BYTES(2), RES_STRINGLAU, false, 0, ""},
      {"Latitude of Vessel Reported",
       BYTES(4),
       RES_LATITUDE,
       true,
       "deg",
       "offset depends on previous field, as do all following fields"},
      {"Longitude of Vessel Reported", BYTES(4), RES_LONGITUDE, true, "deg", ""},
      {"Time of Position", BYTES(4), RES_TIME, false, "s", "Seconds since midnight"},
      {"MMSI of Ship In Distress", BYTES(5), RES_DECIMAL, false, "MMSI", ""},
      {"DSC EOS Symbol", BYTES(1), 1, false, 0, ""},
      {"Expansion Enabled", 2, RES_LOOKUP, false, LOOKUP_YES_NO, ""},
      {"Reserved", 6, RES_BINARY, false, 0, "reserved"},
      {"Calling Rx Frequency/Channel", BYTES(6), RES_ASCII, false, 0, ""},
      {"Calling Tx Frequency/Channel", BYTES(6), RES_ASCII, false, 0, ""},
      {"Time of Receipt", BYTES(4), RES_TIME, false, "s", "Seconds since midnight"},
      {"Date of Receipt", BYTES(2), RES_DATE, false, "days", "Days since January 1, 1970"},
      {"DSC Equipment Assigned Message ID", BYTES(2), 1, false, 0, ""},
      {"DSC Expansion Field Symbol", BYTES(1), RES_LOOKUP, false, LOOKUP_DSC_EXPANSION_DATA, ""},
      {"DSC Expansion Field Data", BYTES(2), RES_STRINGLAU, false, 0, ""},
      {0}}}

    ,
    {"AIS Class B static data (msg 24 Part A)",
     129809,
     PACKET_COMPLETE,
     PACKET_FAST,
     27,
     0,
     {{"Message ID", 6, 1, false, 0, ""},
      {"Repeat indicator", 2, RES_LOOKUP, false, LOOKUP_REPEAT_INDICATOR, ""},
      {"User ID", BYTES(4), RES_INTEGER, false, "MMSI", ""},
      {"Name", BYTES(20), RES_ASCII, false, 0, ""},
      {"AIS Transceiver information", 5, RES_LOOKUP, false, LOOKUP_AIS_TRANSCEIVER, ""},
      {"Reserved", 3, RES_BINARY, false, 0, "reserved"},
      {"Sequence ID", BYTES(1), RES_INTEGER, false, 0, ""},
      {0}}}

    ,
    {"AIS Class B static data (msg 24 Part B)",
     129810,
     PACKET_COMPLETE,
     PACKET_FAST,
     34,
     0,
     {{"Message ID", 6, 1, false, 0, ""},
      {"Repeat indicator", 2, RES_LOOKUP, false, LOOKUP_REPEAT_INDICATOR, ""},
      {"User ID", BYTES(4), RES_INTEGER, false, "MMSI", ""},
      {"Type of ship", BYTES(1), RES_LOOKUP, false, LOOKUP_SHIP_TYPE, ""},
      {"Vendor ID", BYTES(7), RES_ASCII, false, 0, ""},
      {"Callsign", BYTES(7), RES_ASCII, false, 0, ",0=unavailable"},
      {"Length", BYTES(2), 0.1, false, "m", ""},
      {"Beam", BYTES(2), 0.1, false, "m", ""},
      {"Position reference from Starboard", BYTES(2), 0.1, false, "m", ""},
      {"Position reference from Bow", BYTES(2), 0.1, false, "m", ""},
      {"Mothership User ID", BYTES(4), RES_INTEGER, false, "MMSI", "MMSI of mother ship sent by daughter vessels"},
      {"Reserved", 2, RES_BINARY, false, 0, "reserved"},
      {"Spare", 6, RES_INTEGER, false, 0, ",0=unavailable"},
      {"AIS Transceiver information", 5, RES_LOOKUP, false, LOOKUP_AIS_TRANSCEIVER, ""},
      {"Reserved", 3, RES_BINARY, false, 0, "reserved"},
      {"Sequence ID", BYTES(1), RES_INTEGER, false, 0, ""},
      {0}}}

    ,
    {"Label", 130060, PACKET_INCOMPLETE | PACKET_NOT_SEEN, PACKET_FAST, 0, 0, {{0}}}

    ,
    {"Channel Source Configuration", 130061, PACKET_INCOMPLETE | PACKET_NOT_SEEN, PACKET_FAST, 0, 0, {{0}}}

    ,
    {"Route and WP Service - Database List",
     130064,
     PACKET_INCOMPLETE | PACKET_NOT_SEEN,
     PACKET_FAST,
     8,
     9,
     {{"Start Database ID", BYTES(1), 1, false, 0, ""},
      {"nItems", BYTES(1), 1, false, 0, ""},
      {"Number of Databases Available", BYTES(1), 1, false, 0, ""}

      ,
      {"Database ID", BYTES(1), 1, false, 0, ""},
      {"Database Name", BYTES(8), RES_ASCII, false, 0, ""},
      {"Database Timestamp", BYTES(4), RES_TIME, false, "s", "Seconds since midnight"},
      {"Database Datestamp", BYTES(2), RES_DATE, false, "days", "Days since January 1, 1970"},
      {"WP Position Resolution", 6, 1, false, 0, ""},
      {"Reserved", 2, RES_BINARY, false, 0, "reserved"},
      {"Number of Routes in Database", BYTES(2), 1, false, 0, ""},
      {"Number of WPs in Database", BYTES(2), 1, false, 0, ""},
      {"Number of Bytes in Database", BYTES(2), 1, false, 0, ""},
      {0}}}

    ,
    {"Route and WP Service - Route List",
     130065,
     PACKET_INCOMPLETE | PACKET_NOT_SEEN,
     PACKET_FAST,
     8,
     6,
     {{"Start Route ID", BYTES(1), 1, false, 0, ""},
      {"nItems", BYTES(1), 1, false, 0, ""},
      {"Number of Routes in Database", BYTES(1), 1, false, 0, ""}

      ,
      {"Database ID", BYTES(1), 1, false, 0, ""},
      {"Route ID", BYTES(1), 1, false, 0, ""},
      {"Route Name", BYTES(8), RES_ASCII, false, 0, ""},
      {"Reserved", 4, RES_BINARY, false, 0, "reserved"},
      {"WP Identification Method", 2, 1, false, 0, ""},
      {"Route Status", 2, 1, false, 0, ""},
      {0}}}

    ,
    {"Route and WP Service - Route/WP-List Attributes",
     130066,
     PACKET_INCOMPLETE | PACKET_NOT_SEEN,
     PACKET_FAST,
     8,
     0,
     {{"Database ID", BYTES(1), 1, false, 0, ""},
      {"Route ID", BYTES(1), 1, false, 0, ""},
      {"Route/WP-List Name", BYTES(8), RES_ASCII, false, 0, ""},
      {"Route/WP-List Timestamp", BYTES(4), RES_TIME, false, "s", "Seconds since midnight"},
      {"Route/WP-List Datestamp", BYTES(2), RES_DATE, false, "days", "Days since January 1, 1970"},
      {"Change at Last Timestamp", BYTES(1), 1, false, 0, ""},
      {"Number of WPs in the Route/WP-List", BYTES(2), 1, false, 0, ""},
      {"Critical supplementary parameters", BYTES(1), 1, false, 0, ""},
      {"Navigation Method", 2, 1, false, 0, ""},
      {"WP Identification Method", 2, 1, false, 0, ""},
      {"Route Status", 2, 1, false, 0, ""},
      {"XTE Limit for the Route", BYTES(2), 1, false, 0, ""},
      {0}}}

    ,
    {"Route and WP Service - Route - WP Name & Position",
     130067,
     PACKET_INCOMPLETE | PACKET_NOT_SEEN,
     PACKET_FAST,
     8,
     4,
     {{"Start RPS#", BYTES(1), 1, false, 0, ""},
      {"nItems", BYTES(1), 1, false, 0, ""},
      {"Number of WPs in the Route/WP-List", BYTES(2), 1, false, 0, ""},
      {"Database ID", BYTES(1), 1, false, 0, ""},
      {"Route ID", BYTES(1), 1, false, 0, ""}

      ,
      {"WP ID", BYTES(1), 1, false, 0, ""},
      {"WP Name", BYTES(8), RES_ASCII, false, 0, ""},
      {"WP Latitude", BYTES(4), RES_LATITUDE, true, "deg", ""},
      {"WP Longitude", BYTES(4), RES_LONGITUDE, true, "deg", ""},
      {0}}}

    ,
    {"Route and WP Service - Route - WP Name",
     130068,
     PACKET_INCOMPLETE | PACKET_NOT_SEEN,
     PACKET_FAST,
     8,
     2,
     {{"Start RPS#", BYTES(1), 1, false, 0, ""},
      {"nItems", BYTES(1), 1, false, 0, ""},
      {"Number of WPs in the Route/WP-List", BYTES(2), 1, false, 0, ""},
      {"Database ID", BYTES(1), 1, false, 0, ""},
      {"Route ID", BYTES(1), 1, false, 0, ""},
      {"WP ID", BYTES(1), 1, false, 0, ""},
      {"WP Name", BYTES(8), RES_ASCII, false, 0, ""},
      {0}}}

    ,
    {"Route and WP Service - XTE Limit & Navigation Method",
     130069,
     PACKET_INCOMPLETE | PACKET_NOT_SEEN,
     PACKET_FAST,
     8,
     6,
     {{"Start RPS#", BYTES(1), 1, false, 0, ""},
      {"nItems", BYTES(1), 1, false, 0, ""},
      {"Number of WPs with a specific XTE Limit or Nav. Method", BYTES(2), 1, false, 0, ""}

      ,
      {"Database ID", BYTES(1), 1, false, 0, ""},
      {"Route ID", BYTES(1), 1, false, 0, ""},
      {"RPS#", BYTES(1), 1, false, 0, ""},
      {"XTE limit in the leg after WP", BYTES(2), 1, false, 0, ""},
      {"Nav. Method in the leg after WP", 4, 1, false, 0, ""},
      {"Reserved", 4, RES_BINARY, false, 0, ""},
      {0}}}

    ,
    {"Route and WP Service - WP Comment",
     130070,
     PACKET_INCOMPLETE | PACKET_NOT_SEEN,
     PACKET_FAST,
     8,
     2,
     {{"Start ID", BYTES(1), 1, false, 0, ""},
      {"nItems", BYTES(1), 1, false, 0, ""},
      {"Number of WPs with Comments", BYTES(2), 1, false, 0, ""},
      {"Database ID", BYTES(1), 1, false, 0, ""},
      {"Route ID", BYTES(1), 1, false, 0, ""}

      ,
      {"WP ID / RPS#", BYTES(1), 1, false, 0, ""},
      {"Comment", BYTES(8), RES_ASCII, false, 0, ""},
      {0}}}

    ,
    {"Route and WP Service - Route Comment",
     130071,
     PACKET_INCOMPLETE | PACKET_NOT_SEEN,
     PACKET_FAST,
     8,
     2,
     {{"Start Route ID", BYTES(1), 1, false, 0, ""},
      {"nItems", BYTES(1), 1, false, 0, ""},
      {"Number of Routes with Comments", BYTES(2), 1, false, 0, ""},
      {"Database ID", BYTES(1), 1, false, 0, ""}

      ,
      {"Route ID", BYTES(1), 1, false, 0, ""},
      {"Comment", BYTES(8), RES_ASCII, false, 0, ""},
      {0}}}

    ,
    {"Route and WP Service - Database Comment",
     130072,
     PACKET_INCOMPLETE | PACKET_NOT_SEEN,
     PACKET_FAST,
     8,
     2,
     {{"Start Database ID", BYTES(1), 1, false, 0, ""},
      {"nItems", BYTES(1), 1, false, 0, ""},
      {"Number of Databases with Comments", BYTES(2), 1, false, 0, ""}

      ,
      {"Database ID", BYTES(1), 1, false, 0, ""},
      {"Comment", BYTES(8), RES_ASCII, false, 0, ""},
      {0}}}

    ,
    {"Route and WP Service - Radius of Turn",
     130073,
     PACKET_INCOMPLETE | PACKET_NOT_SEEN,
     PACKET_FAST,
     8,
     2,
     {{"Start RPS#", BYTES(1), 1, false, 0, ""},
      {"nItems", BYTES(1), 1, false, 0, ""},
      {"Number of WPs with a specific Radius of Turn", BYTES(2), 1, false, 0, ""},
      {"Database ID", BYTES(1), 1, false, 0, ""},
      {"Route ID", BYTES(1), 1, false, 0, ""}

      ,
      {"RPS#", BYTES(1), 1, false, 0, ""},
      {"Radius of Turn", BYTES(2), 1, false, 0, ""},
      {0}}}

    ,
    {"Route and WP Service - WP List - WP Name & Position",
     130074,
     PACKET_INCOMPLETE | PACKET_NOT_SEEN,
     PACKET_FAST,
     8,
     4,
     {{"Start WP ID", BYTES(1), 1, false, 0, ""},
      {"nItems", BYTES(1), 1, false, 0, ""},
      {"Number of valid WPs in the WP-List", BYTES(2), 1, false, 0, ""},
      {"Database ID", BYTES(1), 1, false, 0, ""},
      {"Reserved", BYTES(1), RES_BINARY, false, 0, "reserved"}

      ,
      {"WP ID", BYTES(1), 1, false, 0, ""},
      {"WP Name", BYTES(8), RES_ASCII, false, 0, ""},
      {"WP Latitude", BYTES(4), RES_LATITUDE, true, "deg", ""},
      {"WP Longitude", BYTES(4), RES_LONGITUDE, true, "deg", ""},
      {0}}}

    /* http://askjackrabbit.typepad.com/ask_jack_rabbit/page/7/ */
    ,
    {"Wind Data",
     130306,
     PACKET_COMPLETE,
     PACKET_SINGLE,
     8,
     0,
     {{"SID", BYTES(1), 1, false, 0, ""},
      {"Wind Speed", BYTES(2), 0.01, false, "m/s", ""},
      {"Wind Angle", BYTES(2), RES_RADIANS, false, "rad", ""},
      {"Reference", 3, RES_LOOKUP, false, LOOKUP_WIND_REFERENCE, ""},
      {"Reserved", 5 + BYTES(2), RES_BINARY, false, 0, ""},
      {0}}}

    /* Water temperature, Transducer Measurement */
    ,
    {"Environmental Parameters",
     130310,
     PACKET_COMPLETE,
     PACKET_SINGLE,
     8,
     0,
     {{"SID", BYTES(1), 1, false, 0, ""},
      {"Water Temperature", BYTES(2), RES_TEMPERATURE, false, "K", ""},
      {"Outside Ambient Air Temperature", BYTES(2), RES_TEMPERATURE, false, "K", ""},
      {"Atmospheric Pressure", BYTES(2), RES_PRESSURE, false, "hPa", ""},
      {"Reserved", BYTES(1), RES_BINARY, false, 0, ""},
      {0}}}

    ,
    {"Environmental Parameters",
     130311,
     PACKET_COMPLETE,
     PACKET_SINGLE,
     8,
     0,
     {{"SID", BYTES(1), 1, false, 0, ""},
      {"Temperature Source", 6, RES_LOOKUP, false, LOOKUP_TEMPERATURE_SOURCE, ""},
      {"Humidity Source", 2, RES_LOOKUP, false, LOOKUP_HUMIDITY_SOURCE, ""},
      {"Temperature", BYTES(2), RES_TEMPERATURE, false, "K", ""},
      {"Humidity", BYTES(2), RES_PERCENTAGE, true, "%", ""},
      {"Atmospheric Pressure", BYTES(2), RES_PRESSURE, false, "hPa", ""},
      {0}}}

    ,
    {"Temperature",
     130312,
     PACKET_COMPLETE,
     PACKET_SINGLE,
     8,
     0,
     {{"SID", BYTES(1), 1, false, 0, ""},
      {"Instance", BYTES(1), 1, false, 0, ""},
      {"Source", BYTES(1), RES_LOOKUP, false, LOOKUP_TEMPERATURE_SOURCE, ""},
      {"Actual Temperature", BYTES(2), RES_TEMPERATURE, false, "K", ""},
      {"Set Temperature", BYTES(2), RES_TEMPERATURE, false, "K", ""},
      {0}}}

    ,
    {"Humidity",
     130313,
     PACKET_COMPLETE,
     PACKET_SINGLE,
     8,
     0,
     {{"SID", BYTES(1), 1, false, 0, ""},
      {"Instance", BYTES(1), 1, false, 0, ""},
      {"Source", BYTES(1), RES_LOOKUP, false, LOOKUP_HUMIDITY_SOURCE, ""},
      {"Actual Humidity", BYTES(2), RES_PERCENTAGE, true, "%", ""},
      {"Set Humidity", BYTES(2), RES_PERCENTAGE, true, "%", ""},
      {0}}}

    ,
    {"Actual Pressure",
     130314,
     PACKET_COMPLETE,
     PACKET_SINGLE,
     8,
     0,
     {{"SID", BYTES(1), 1, false, 0, ""},
      {"Instance", BYTES(1), 1, false, 0, ""},
      {"Source", BYTES(1), RES_LOOKUP, false, LOOKUP_PRESSURE_SOURCE, ""},
      {"Pressure", BYTES(4), RES_PRESSURE_HIRES, true, "dPa", ""},
      {0}}}

    ,
    {"Set Pressure",
     130315,
     PACKET_COMPLETE,
     PACKET_SINGLE,
     8,
     0,
     {{"SID", BYTES(1), 1, false, 0, ""},
      {"Instance", BYTES(1), 1, false, 0, ""},
      {"Source", BYTES(1), RES_LOOKUP, false, LOOKUP_PRESSURE_SOURCE, ""},
      {"Pressure", BYTES(4), RES_PRESSURE_HIRES, false, "dPa", ""},
      {0}}}

    ,
    {"Temperature Extended Range",
     130316,
     PACKET_COMPLETE,
     PACKET_SINGLE,
     8,
     0,
     {{"SID", BYTES(1), 1, false, 0, ""},
      {"Instance", BYTES(1), 1, false, 0, ""},
      {"Source", BYTES(1), RES_LOOKUP, false, LOOKUP_TEMPERATURE_SOURCE, ""},
      {"Temperature", BYTES(3), RES_TEMPERATURE_HIRES, false, "K", ""},
      {"Set Temperature", BYTES(2), RES_TEMPERATURE_HIGH, false, "K", ""},
      {0}}}

    ,
    {"Tide Station Data",
     130320,
     PACKET_COMPLETE,
     PACKET_FAST,
     20,
     0,
     {{"Mode", 4, RES_LOOKUP, false, LOOKUP_RESIDUAL_MODE, ""},
      {"Tide Tendency", 2, RES_LOOKUP, false, ",0=Falling,1=Rising", ""},
      {"Reserved", 2, RES_BINARY, false, 0, ""},
      {"Measurement Date", BYTES(2), RES_DATE, false, "days", "Days since January 1, 1970"},
      {"Measurement Time", BYTES(4), RES_TIME, false, "s", "Seconds since midnight"},
      {"Station Latitude", BYTES(4), RES_LATITUDE, true, "deg", ""},
      {"Station Longitude", BYTES(4), RES_LONGITUDE, true, "deg", ""},
      {"Tide Level", BYTES(2), 0.001, true, "m", "Relative to MLLW"},
      {"Tide Level standard deviation", BYTES(2), 0.01, false, "m", ""},
      {"Station ID", BYTES(2), RES_STRING, false, 0, ""},
      {"Station Name", BYTES(2), RES_STRING, false, 0, ""},
      {0}}}

    ,
    {"Salinity Station Data",
     130321,
     PACKET_COMPLETE | PACKET_NOT_SEEN,
     PACKET_FAST,
     22,
     0,
     {{"Mode", 4, RES_LOOKUP, false, LOOKUP_RESIDUAL_MODE, ""},
      {"Reserved", 4, RES_BINARY, false, 0, ""},
      {"Measurement Date", BYTES(2), RES_DATE, false, "days", "Days since January 1, 1970"},
      {"Measurement Time", BYTES(4), RES_TIME, false, "s", "Seconds since midnight"},
      {"Station Latitude", BYTES(4), RES_LATITUDE, true, "deg", ""},
      {"Station Longitude", BYTES(4), RES_LONGITUDE, true, "deg", ""},
      {"Salinity",
       BYTES(4),
       RES_FLOAT,
       true,
       "ppt",
       "The average Salinity of ocean water is about 35 grams of salts per kilogram of sea water (g/kg), usually written as 35 ppt "
       "which is read as 35 parts per thousand."},
      {"Water Temperature", BYTES(2), RES_TEMPERATURE, false, "K", ""},
      {"Station ID", BYTES(2), RES_STRING, false, 0, ""},
      {"Station Name", BYTES(2), RES_STRING, false, 0, ""},
      {0}}}

    ,
    {"Current Station Data",
     130322,
     PACKET_INCOMPLETE | PACKET_NOT_SEEN,
     PACKET_FAST,
     8,
     0,
     {{"Mode", 4, 1, false, 0, ""},
      {"Reserved", 4, RES_BINARY, false, 0, ""},
      {"Measurement Date", BYTES(2), RES_DATE, false, "days", "Days since January 1, 1970"},
      {"Measurement Time", BYTES(4), RES_TIME, false, "s", "Seconds since midnight"},
      {"Station Latitude", BYTES(4), RES_LATITUDE, true, "deg", ""},
      {"Station Longitude", BYTES(4), RES_LONGITUDE, true, "deg", ""},
      {"Measurement Depth", BYTES(4), 0.01, false, "m", "Depth below transducer"},
      {"Current speed", BYTES(2), 0.01, false, "m/s", ""},
      {"Current flow direction", BYTES(2), RES_RADIANS, false, "rad", ""},
      {"Water Temperature", BYTES(2), RES_TEMPERATURE, false, "K", ""},
      {"Station ID", BYTES(2), RES_STRING, false, 0, ""},
      {"Station Name", BYTES(2), RES_STRING, false, 0, ""},
      {0}}}

    ,
    {"Meteorological Station Data",
     130323,
     PACKET_INCOMPLETE | PACKET_NOT_SEEN,
     PACKET_FAST,
     0x1e,
     0,
     {{"Mode", 4, 1, false, 0, ""},
      {"Reserved", 4, RES_BINARY, false, 0, ""},
      {"Measurement Date", BYTES(2), RES_DATE, false, "days", "Days since January 1, 1970"},
      {"Measurement Time", BYTES(4), RES_TIME, false, "s", "Seconds since midnight"},
      {"Station Latitude", BYTES(4), RES_LATITUDE, true, "deg", ""},
      {"Station Longitude", BYTES(4), RES_LONGITUDE, true, "deg", ""},
      {"Wind Speed", BYTES(2), 0.01, false, "m/s", ""},
      {"Wind Direction", BYTES(2), RES_RADIANS, false, "rad", ""},
      {"Wind Reference", 3, RES_LOOKUP, false, LOOKUP_WIND_REFERENCE, ""},
      {"Reserved", 5, RES_BINARY, false, "", "reserved"},
      {"Wind Gusts", BYTES(2), 0.01, false, "m/s", ""},
      {"Atmospheric Pressure", BYTES(2), RES_PRESSURE, false, "hPa", ""},
      {"Ambient Temperature", BYTES(2), RES_TEMPERATURE, false, "K", ""},
      {"Station ID", BYTES(257), RES_STRING, false, 0, ""},
      {"Station Name", BYTES(257), RES_STRING, false, 0, ""},
      {0}}}

    ,
    {"Moored Buoy Station Data",
     130324,
     PACKET_INCOMPLETE | PACKET_NOT_SEEN,
     PACKET_FAST,
     8,
     0,
     {{"Mode", 4, 1, false, 0, ""},
      {"Reserved", 4, RES_BINARY, false, 0, ""},
      {"Measurement Date", BYTES(2), RES_DATE, false, "days", "Days since January 1, 1970"},
      {"Measurement Time", BYTES(4), RES_TIME, false, "s", "Seconds since midnight"},
      {"Station Latitude", BYTES(4), RES_LATITUDE, true, "deg", ""},
      {"Station Longitude", BYTES(4), RES_LONGITUDE, true, "deg", ""},
      {"Wind Speed", BYTES(2), 0.01, false, "m/s", ""},
      {"Wind Direction", BYTES(2), RES_RADIANS, false, "rad", ""},
      {"Wind Reference", 3, RES_LOOKUP, false, LOOKUP_WIND_REFERENCE, ""},
      {"Reserved", 5, RES_BINARY, false, "", "reserved"},
      {"Wind Gusts", BYTES(2), 0.01, false, "m/s", ""},
      {"Wave Height", BYTES(2), 1, false, 0, ""},
      {"Dominant Wave Period", BYTES(2), 1, false, 0, ""},
      {"Atmospheric Pressure", BYTES(2), RES_PRESSURE, false, "hPa", ""},
      {"Pressure Tendency Rate", BYTES(2), 1, false, "", ""},
      {"Air Temperature", BYTES(2), RES_TEMPERATURE, false, "K", ""},
      {"Water Temperature", BYTES(2), RES_TEMPERATURE, false, "K", ""},
      {"Station ID", BYTES(8), RES_ASCII, false, 0, ""},
      {0}}}

    ,
    {"Payload Mass", 130560, PACKET_INCOMPLETE | PACKET_NOT_SEEN, PACKET_FAST, 0, 0, {{0}}}

    /* http://www.nmea.org/Assets/20130905%20amendment%20at%202000%20201309051%20watermaker%20input%20setting%20and%20status%20pgn%20130567.pdf

    This PGN may be requested or used to command and configure a number of Watermaker controls. The Command Group Function PGN
    126208 is used perform the following: start/stop a production, start/stop rinse or flush operation , start/stop low and high
    pressure pump and perform an emergency stop. The Request Group Function PGN 126208 or ISO Request PGN 059904 may be used to
    request this PGN. This PGN also provides Watermaker status and measurement information. The PGN is broadcast periodically.

    */
    ,
    {"Watermaker Input Setting and Status",
     130567,
     PACKET_COMPLETE,
     PACKET_FAST,
     24,
     0,
     {{"Watermaker Operating State",
       6,
       RES_LOOKUP,
       false,
       ",0=Stopped,1=Starting,2=Running,3=Stopping,4=Flushing,5=Rinsing,6=Initiating,7=Manual Mode,62=Error,63=Unavailable",
       ""},
      {"Production Start/Stop", 2, RES_LOOKUP, false, LOOKUP_YES_NO, ""},
      {"Rinse Start/Stop", 2, RES_LOOKUP, false, LOOKUP_YES_NO, ""},
      {"Low Pressure Pump Status", 2, RES_LOOKUP, false, LOOKUP_YES_NO, ""},
      {"High Pressure Pump Status", 2, RES_LOOKUP, false, LOOKUP_YES_NO, ""},
      {"Emergency Stop", 2, RES_LOOKUP, false, LOOKUP_YES_NO, ""},
      {"Product Solenoid Valve Status", 2, RES_LOOKUP, false, LOOKUP_OK_WARNING, ""},
      {"Flush Mode Status", 2, RES_LOOKUP, false, LOOKUP_YES_NO, ""},
      {"Salinity Status", 2, RES_LOOKUP, false, LOOKUP_OK_WARNING, ""},
      {"Sensor Status", 2, RES_LOOKUP, false, LOOKUP_OK_WARNING, ""},
      {"Oil Change Indicator Status", 2, RES_LOOKUP, false, LOOKUP_OK_WARNING, ""},
      {"Filter Status", 2, RES_LOOKUP, false, LOOKUP_OK_WARNING, ""},
      {"System Status", 2, RES_LOOKUP, false, LOOKUP_OK_WARNING, ""},
      {"Reserved", 2, RES_BINARY, false, 0, "Reserved"},
      {"Salinity", BYTES(2), RES_INTEGER, false, "ppm", ""},
      {"Product Water Temperature", BYTES(2), RES_TEMPERATURE, false, "K", ""},
      {"Pre-filter Pressure", BYTES(2), RES_PRESSURE, false, "hPa", ""},
      {"Post-filter Pressure", BYTES(2), RES_PRESSURE, false, "hPa", ""},
      {"Feed Pressure", BYTES(2), RES_PRESSURE, true, "kPa", ""},
      {"System High Pressure", BYTES(2), RES_PRESSURE, false, "kPa", ""},
      {"Product Water Flow", BYTES(2), 0.1, true, "L/h", ""},
      {"Brine Water Flow", BYTES(2), 0.1, true, "L/h", ""},
      {"Run Time", BYTES(4), RES_INTEGER, false, "s", ""},
      {0}}}

    /* https://www.nmea.org/Assets/20160725%20corrigenda%20pgn%20130569%20published.pdf */
    ,
    {"Current Status and File",
     130569,
     PACKET_INCOMPLETE | PACKET_NOT_SEEN,
     PACKET_FAST,
     233,
     0,
     {{"Zone", BYTES(1), RES_LOOKUP, false, LOOKUP_ENTERTAINMENT_ZONE, ""},
      {"Source", 8, RES_LOOKUP, false, LOOKUP_ENTERTAINMENT_SOURCE, ""},
      {"Number", BYTES(1), RES_INTEGER, false, 0, "Source number per type"},
      {"ID", BYTES(4), RES_INTEGER, false, 0, "Unique file ID"},
      {"Play status", BYTES(1), RES_LOOKUP, false, LOOKUP_ENTERTAINMENT_PLAY_STATUS, ""},
      {"Elapsed Track Time", BYTES(2), RES_TIME, false, "s", ""},
      {"Track Time", BYTES(2), RES_TIME, false, "s", ""},
      {"Repeat Status", 4, RES_LOOKUP, false, LOOKUP_ENTERTAINMENT_REPEAT_STATUS, ""},
      {"Shuffle Status", 4, RES_LOOKUP, false, LOOKUP_ENTERTAINMENT_SHUFFLE_STATUS, ""},
      {"Save Favorite Number", BYTES(1), RES_INTEGER, false, 0, "Used to command AV to save current station as favorite"},
      {"Play Favorite Number", BYTES(2), RES_INTEGER, false, 0, "Used to command AV to play indicated favorite station"},
      {"Thumbs Up/Down", BYTES(1), RES_INTEGER, false, LOOKUP_ENTERTAINMENT_LIKE_STATUS, ""},
      {"Signal Strength", BYTES(1), RES_INTEGER, false, "%", ""},
      {"Radio Frequency", BYTES(4), 10, false, "Hz", ""},
      {"HD Frequency Multicast", BYTES(1), RES_INTEGER, false, 0, "Digital sub channel"},
      {"Delete Favorite Number", BYTES(1), RES_INTEGER, false, 0, "Used to command AV to delete current station as favorite"},
      {"Total Number of Tracks", BYTES(2), RES_INTEGER, false, 0, ""},
      {0}}}

    /* https://www.nmea.org/Assets/20160715%20corrigenda%20entertainment%20pgns%20.pdf */

    ,
    {"Library Data File",
     130570,
     PACKET_INCOMPLETE | PACKET_NOT_SEEN,
     PACKET_FAST,
     233,
     0,
     {{"Source", 8, RES_LOOKUP, false, LOOKUP_ENTERTAINMENT_SOURCE, ""},
      {"Number", BYTES(1), RES_INTEGER, false, 0, "Source number per type"},
      {"ID", BYTES(4), RES_INTEGER, false, 0, "Unique file ID"},
      {"Type", BYTES(1), RES_LOOKUP, false, LOOKUP_ENTERTAINMENT_TYPE, ""},
      {"Name", BYTES(2), RES_STRINGLAU, false, 0, ""},
      {"Track", BYTES(2), RES_INTEGER, false, 0, ""},
      {"Station", BYTES(2), RES_INTEGER, false, 0, ""},
      {"Favorite", BYTES(1), RES_INTEGER, false, 0, ""},
      {"Radio frequency", BYTES(4), 10., false, "Hz", ""},
      {"HD Frequency", BYTES(1), RES_INTEGER, false, 0, ""},
      {"Zone", BYTES(1), RES_LOOKUP, false, LOOKUP_ENTERTAINMENT_ZONE, ""},
      {"In play queue", 2, RES_LOOKUP, false, LOOKUP_YES_NO, ""},
      {"Lock status", 2, RES_LOOKUP, false, ",0=Unlocked,1=Locked", "Sirius XM only"},
      {"Reserved", 4, RES_BINARY, false, 0, "Reserved"},
      {"Artist", BYTES(2), RES_STRINGLAU, false, 0, ""},
      {"Album", BYTES(2), RES_STRINGLAU, false, 0, ""},
      {"Station", BYTES(2), RES_STRINGLAU, false, 0, ""},
      {0}}}

    ,
    {"Library Data Group",
     130571,
     PACKET_INCOMPLETE | PACKET_NOT_SEEN,
     PACKET_FAST,
     233,
     2,
     {
         {"Source", 8, RES_LOOKUP, false, LOOKUP_ENTERTAINMENT_SOURCE, ""},
         {"Number", BYTES(1), RES_INTEGER, false, 0, "Source number per type"},
         {"Zone", BYTES(1), RES_LOOKUP, false, LOOKUP_ENTERTAINMENT_ZONE, ""},
         {"Group ID", BYTES(4), RES_INTEGER, false, 0, "Unique group ID"},
         {"ID offset", BYTES(2), RES_INTEGER, false, 0, "First ID in this PGN"},
         {"ID count", BYTES(2), RES_INTEGER, false, 0, "Number of IDs in this PGN"},
         {"Total ID count", BYTES(2), RES_INTEGER, false, 0, "Total IDs in group"},
         {"ID type", BYTES(1), RES_LOOKUP, false, ",0=Group,1=File,2=Encrypted group,3=Encrypted file", ""},
         {"ID", BYTES(4), RES_INTEGER, false, 0, ""},
         {"Name", BYTES(2), RES_STRINGLAU, false, 0, ""}
         // TODO: Add support for extra fields *after* the repeating fields.
         // The NMEA, in all its wisdom, suddenly feels a repeating field PGN can act to different rules. Sigh.
         // , { "Artist", BYTES(2), RES_STRINGLAU, false, 0, "" }
     }}

    ,
    {"Library Data Search",
     130572,
     PACKET_INCOMPLETE | PACKET_NOT_SEEN,
     PACKET_FAST,
     233,
     0,
     {{"Source", 8, RES_LOOKUP, false, LOOKUP_ENTERTAINMENT_SOURCE, ""},
      {"Number", BYTES(1), RES_INTEGER, false, 0, "Source number per type"},
      {"Group ID", BYTES(4), RES_INTEGER, false, 0, "Unique group ID"},
      {"Group type 1", BYTES(1), RES_LOOKUP, false, LOOKUP_ENTERTAINMENT_GROUP, ""},
      {"Group name 1", BYTES(2), RES_STRINGLAU, false, 0, ""},
      {"Group type 2", BYTES(1), RES_LOOKUP, false, LOOKUP_ENTERTAINMENT_GROUP, ""},
      {"Group name 2", BYTES(2), RES_STRINGLAU, false, 0, ""},
      {"Group type 3", BYTES(1), RES_LOOKUP, false, LOOKUP_ENTERTAINMENT_GROUP, ""},
      {"Group name 3", BYTES(2), RES_STRINGLAU, false, 0, ""},
      {0}}}

    ,
    {"Supported Source Data",
     130573,
     PACKET_INCOMPLETE | PACKET_NOT_SEEN,
     PACKET_FAST,
     233,
     10,
     {{"ID offset", BYTES(2), RES_INTEGER, false, 0, "First ID in this PGN"},
      {"ID count", BYTES(2), RES_INTEGER, false, 0, "Number of IDs in this PGN"},
      {"Total ID count", BYTES(2), RES_INTEGER, false, 0, "Total IDs in group"},
      {"ID", BYTES(1), RES_INTEGER, false, 0, "Source ID"},
      {"Source", 8, RES_LOOKUP, false, LOOKUP_ENTERTAINMENT_SOURCE, ""},
      {"Number", BYTES(1), RES_INTEGER, false, 0, "Source number per type"},
      {"Name", BYTES(2), RES_STRINGLAU, false, 0, ""},
      {"Play support", BYTES(2), RES_BITFIELD, false, LOOKUP_ENTERTAINMENT_PLAY_STATUS, ""},
      {"Browse support", BYTES(2), RES_BITFIELD, false, LOOKUP_ENTERTAINMENT_GROUP, ""},
      {"Thumbs support", 2, RES_LOOKUP, false, LOOKUP_YES_NO, ""},
      {"Connected", 2, RES_LOOKUP, false, LOOKUP_YES_NO, ""},
      {"Repeat support", 2, RES_BITFIELD, false, ",1=Song,2=Play Queue", ""},
      {"Shuffle support", 2, RES_BITFIELD, false, ",1=Play Queue,2=All", ""},
      {0}}}

    ,
    {"Supported Zone Data",
     130574,
     PACKET_INCOMPLETE | PACKET_NOT_SEEN,
     PACKET_FAST,
     233,
     2,
     {{"First zone ID", BYTES(1), RES_INTEGER, false, 0, "First Zone in this PGN"},
      {"Zone count", BYTES(1), RES_INTEGER, false, 0, "Number of Zones in this PGN"},
      {"Total zone count", BYTES(1), RES_INTEGER, false, 0, "Total Zones supported by this device"},
      {"Zone ID", BYTES(1), RES_LOOKUP, false, LOOKUP_ENTERTAINMENT_ZONE, ""},
      {"Name", BYTES(2), RES_STRINGLAU, false, 0, ""},
      {0}}}

    ,
    {"Small Craft Status",
     130576,
     PACKET_INCOMPLETE | PACKET_NOT_SEEN,
     PACKET_SINGLE,
     2,
     0,
     {{"Port trim tab", BYTES(1), 1, true, 0, ""}, {"Starboard trim tab", BYTES(1), 1, true, 0, ""}, {0}}}

    ,
    {"Direction Data",
     130577,
     PACKET_INCOMPLETE | PACKET_NOT_SEEN,
     PACKET_FAST,
     14,
     0,
     {{"Data Mode", 4, RES_LOOKUP, false, LOOKUP_RESIDUAL_MODE, ""},
      {"COG Reference", 2, RES_LOOKUP, false, LOOKUP_DIRECTION_REFERENCE, ""},
      {"Reserved", 2, RES_BINARY, false, 0, "Reserved"},
      {"SID", BYTES(1), 1, false, 0, ""}
      /* So far, 2 bytes. Very sure of this given molly rose data */
      ,
      {"COG", BYTES(2), RES_RADIANS, false, "rad", ""},
      {"SOG", BYTES(2), 0.01, false, "m/s", ""},
      {"Heading", BYTES(2), RES_RADIANS, false, "rad", ""},
      {"Speed through Water", BYTES(2), 0.01, false, "m/s", ""},
      {"Set", BYTES(2), RES_RADIANS, false, "rad", ""},
      {"Drift", BYTES(2), 0.01, false, "m/s", ""},
      {0}}}

    ,
    {"Vessel Speed Components",
     130578,
     PACKET_COMPLETE,
     PACKET_FAST,
     12,
     0,
     {{"Longitudinal Speed, Water-referenced", BYTES(2), 0.001, true, "m/s", ""},
      {"Transverse Speed, Water-referenced", BYTES(2), 0.001, true, "m/s", ""},
      {"Longitudinal Speed, Ground-referenced", BYTES(2), 0.001, true, "m/s", ""},
      {"Transverse Speed, Ground-referenced", BYTES(2), 0.001, true, "m/s", ""},
      {"Stern Speed, Water-referenced", BYTES(2), 0.001, true, "m/s", ""},
      {"Stern Speed, Ground-referenced", BYTES(2), 0.001, true, "m/s", ""},
      {0}}}

    ,
    {"System Configuration",
     130579,
     PACKET_FIELD_LENGTHS_UNKNOWN | PACKET_NOT_SEEN,
     PACKET_FAST,
     (48 / 8 + 2),
     0,
     {{"Power", 2, RES_LOOKUP, false, LOOKUP_YES_NO, ""},
      {"Default Settings",
       2,
       RES_LOOKUP,
       false,
       ",0=Save current settings as user default,1=Load user default,2=Load Manufacturer default",
       ""},
      {"Tuner regions",
       4,
       RES_LOOKUP,
       false,
       ",0=USA,1=Europe,2=Asia,3=Middle East,4=Latin America,5=Australia,6=Russia,7=Japan",
       ""},
      {"Max favorites", BYTES(1), RES_INTEGER, false, 0, ""},
      {"Video protocols", 4, RES_BITFIELD, false, ",0=PAL,1=NTSC", ""},
      {"Reserved", 44, RES_BINARY, false, 0, "Reserved"},
      {0}}}

    ,
    {"System Configuration (deprecated)",
     130580,
     PACKET_FIELD_LENGTHS_UNKNOWN | PACKET_NOT_SEEN,
     PACKET_FAST,
     2,
     0,
     {{"Power", 2, RES_LOOKUP, false, LOOKUP_YES_NO, ""},
      {"Default Settings",
       2,
       RES_LOOKUP,
       false,
       ",0=Save current settings as user default,1=Load user default,2=Load Manufacturer default",
       ""},
      {"Tuner regions",
       4,
       RES_LOOKUP,
       false,
       ",0=USA,1=Europe,2=Asia,3=Middle East,4=Latin America,5=Australia,6=Russia,7=Japan",
       ""},
      {"Max favorites", BYTES(1), RES_INTEGER, false, 0, ""},
      {0}}}

    ,
    {"Zone Configuration (deprecated)",
     130581,
     PACKET_FIELD_LENGTHS_UNKNOWN | PACKET_NOT_SEEN,
     PACKET_FAST,
     14,
     2,
     {{"First zone ID", BYTES(1), RES_INTEGER, false, 0, "First Zone in this PGN"},
      {"Zone count", BYTES(1), RES_INTEGER, false, 0, "Number of Zones in this PGN"},
      {"Total zone count", BYTES(1), RES_INTEGER, false, 0, "Total Zones supported by this device"},
      {"Zone ID", BYTES(1), RES_LOOKUP, false, LOOKUP_ENTERTAINMENT_ZONE, ""},
      {"Zone name", BYTES(2), RES_STRINGLAU, false, 0, ""},
      {0}}}

    ,
    {"Zone Volume",
     130582,
     PACKET_FIELD_LENGTHS_UNKNOWN | PACKET_NOT_SEEN,
     PACKET_FAST,
     4,
     0,
     {{"Zone ID", BYTES(1), RES_LOOKUP, false, LOOKUP_ENTERTAINMENT_ZONE, ""},
      {"Volume", BYTES(1), RES_INTEGER, false, "%", ""},
      {"Volume change", 2, RES_LOOKUP, false, ",0=Up,1=Down", "Write only"},
      {"Mute", 2, RES_LOOKUP, false, LOOKUP_YES_NO, ""},
      {"Reserved", 4, RES_BINARY, false, 0, "Reserved"},
      {"Channel", 8, RES_LOOKUP, false, LOOKUP_ENTERTAINMENT_CHANNEL, ""},
      {0}}}

    ,
    {"Available Audio EQ presets",
     130583,
     PACKET_FIELD_LENGTHS_UNKNOWN | PACKET_NOT_SEEN,
     PACKET_FAST,
     233,
     2,
     {{"First preset", BYTES(1), RES_INTEGER, false, 0, "First preset in this PGN"},
      {"Preset count", BYTES(1), RES_INTEGER, false, 0, ""},
      {"Total preset count", BYTES(1), RES_INTEGER, false, 0, ""},
      {"Preset type", BYTES(1), RES_LOOKUP, false, LOOKUP_ENTERTAINMENT_EQ, ""},
      {"Preset name", BYTES(2), RES_STRINGLAU, false, 0, ""},
      {0}}}

    ,
    {"Available Bluetooth addresses",
     130584,
     PACKET_FIELD_LENGTHS_UNKNOWN | PACKET_NOT_SEEN,
     PACKET_FAST,
     233,
     3,
     {{"First address", BYTES(1), RES_INTEGER, false, 0, "First address in this PGN"},
      {"Address count", BYTES(1), RES_INTEGER, false, 0, ""},
      {"Total address count", BYTES(1), RES_INTEGER, false, 0, ""},
      {"Bluetooth address", BYTES(6), RES_INTEGER, false, 0, ""},
      {"Status", BYTES(1), RES_LOOKUP, false, ",0=Connected,1=Not connected,2=Not paired", ""},
      {"Device name", BYTES(2), RES_STRINGLAU, false, 0, ""},
      {"Signal strength", BYTES(1), RES_INTEGER, false, "%", ""},
      {0}}}

    ,
    {"Bluetooth source status",
     130585,
     PACKET_FIELD_LENGTHS_UNKNOWN | PACKET_NOT_SEEN,
     PACKET_FAST,
     233,
     0,
     {{"Source number", BYTES(1), RES_INTEGER, false, 0, ""},
      {"Status", 4, RES_LOOKUP, false, ",0=Reserved,1=Connected,2=Connecting,3=Not connected", ""},
      {"Forget device", 2, RES_LOOKUP, false, LOOKUP_YES_NO, ""},
      {"Discovering", 2, RES_LOOKUP, false, LOOKUP_YES_NO, ""},
      {"Bluetooth address", BYTES(6), RES_INTEGER, false, 0, ""},
      {0}}}

    ,
    {"Zone Configuration",
     130586,
     PACKET_FIELD_LENGTHS_UNKNOWN | PACKET_NOT_SEEN,
     PACKET_FAST,
     14,
     2,
     {{"Zone ID", BYTES(1), RES_LOOKUP, false, LOOKUP_ENTERTAINMENT_ZONE, ""},
      {"Volume limit", BYTES(1), RES_INTEGER, false, "%", ""},
      {"Fade", BYTES(1), RES_INTEGER, true, "%", ""},
      {"Balance", BYTES(1), RES_INTEGER, true, "%", ""},
      {"Sub volume", BYTES(1), RES_INTEGER, true, "%", ""},
      {"EQ - Treble", BYTES(1), RES_INTEGER, true, "%", ""},
      {"EQ - Mid range", BYTES(1), RES_INTEGER, true, "%", ""},
      {"EQ - Bass", BYTES(1), RES_INTEGER, true, "%", ""},
      {"Preset type", BYTES(1), RES_LOOKUP, false, LOOKUP_ENTERTAINMENT_EQ, ""},
      {"Audio filter", BYTES(1), RES_LOOKUP, false, LOOKUP_ENTERTAINMENT_FILTER, ""},
      {"High pass filter frequency", BYTES(2), RES_INTEGER, false, "Hz", ""},
      {"Low pass filter frequency", BYTES(2), RES_INTEGER, false, "Hz", ""},
      {"Channel", 8, RES_LOOKUP, false, LOOKUP_ENTERTAINMENT_CHANNEL, ""},
      {0}}}

    /* proprietary PDU2 (non addressed) fast packet PGN range 0x1FF00 to 0x1FFFF (130816 - 131071) */
    ,
    {"SonicHub: Init #2",
     130816,
     PACKET_INCOMPLETE | PACKET_NOT_SEEN,
     PACKET_FAST,
     9,
     0,
     {{"Manufacturer Code", 11, RES_MANUFACTURER, false, "=275", "Navico"},
      {"Reserved", 2, RES_NOTUSED, false, 0, ""},
      {"Industry Code", 3, RES_LOOKUP, false, "=4", "Marine Industry"},
      {"Reserved", BYTES(1), RES_NOTUSED, false, 0, ""},
      {"Proprietary ID", BYTES(1), RES_LOOKUP, false, "=1", "Init #2"},
      {"Control", BYTES(1), RES_LOOKUP, false, ",0=Set,128=Ack", ""},
      {"A", BYTES(2), RES_INTEGER, false, 0, ""},
      {"B", BYTES(2), RES_INTEGER, false, 0, ""},
      {0}}}

    ,
    {"SonicHub: AM Radio",
     130816,
     PACKET_INCOMPLETE | PACKET_NOT_SEEN,
     PACKET_FAST,
     0x40,
     0,
     {{"Manufacturer Code", 11, RES_MANUFACTURER, false, "=275", "Navico"},
      {"Reserved", 2, RES_NOTUSED, false, 0, ""},
      {"Industry Code", 3, RES_LOOKUP, false, "=4", "Marine Industry"},
      {"Reserved", BYTES(1), RES_NOTUSED, false, 0, ""},
      {"Proprietary ID", BYTES(1), RES_LOOKUP, false, "=4", "AM Radio"},
      {"Control", BYTES(1), RES_LOOKUP, false, ",0=Set,128=Ack", ""},
      {"Item", BYTES(1), RES_LOOKUP, false, ",1=Seeking up,2=Tuned,3=Seeking down", ""},
      {"Frequency", BYTES(4), 0.001, false, "kHz", ""},
      {"Noise level", 2, 1, false, 0, ""} // Not sure about this
      ,
      {"Signal level", 4, 1, false, 0, ""} // ... and this, doesn't make complete sense compared to display
      ,
      {"Reserved", 2, RES_BINARY, false, 0, ""},
      {"Text", BYTES(32), RES_STRINGLZ, false, 0, ""},
      {0}}}

    ,
    {"SonicHub: Zone info",
     130816,
     PACKET_INCOMPLETE,
     PACKET_FAST,
     6,
     0,
     {{"Manufacturer Code", 11, RES_MANUFACTURER, false, "=275", "Navico"},
      {"Reserved", 2, RES_NOTUSED, false, 0, ""},
      {"Industry Code", 3, RES_LOOKUP, false, "=4", "Marine Industry"},
      {"Reserved", BYTES(1), RES_NOTUSED, false, 0, ""},
      {"Proprietary ID", BYTES(1), RES_LOOKUP, false, "=5", "Zone info"},
      {"Control", BYTES(1), RES_LOOKUP, false, ",0=Set,128=Ack", ""},
      {"Zone", BYTES(1), RES_INTEGER, false, 0, ""},
      {0}}}

    ,
    {"SonicHub: Source",
     130816,
     PACKET_INCOMPLETE,
     PACKET_FAST,
     0x40,
     0,
     {{"Manufacturer Code", 11, RES_MANUFACTURER, false, "=275", "Navico"},
      {"Reserved", 2, RES_NOTUSED, false, 0, ""},
      {"Industry Code", 3, RES_LOOKUP, false, "=4", "Marine Industry"},
      {"Reserved", BYTES(1), RES_NOTUSED, false, 0, ""},
      {"Proprietary ID", BYTES(1), RES_LOOKUP, false, "=6", "Source"},
      {"Control", BYTES(1), RES_LOOKUP, false, ",0=Set,128=Ack", ""},
      {"Source", BYTES(1), RES_LOOKUP, false, ",0=AM,1=FM,2=iPod,3=USB,4=AUX,5=AUX 2,6=Mic", ""},
      {0}}}

    ,
    {"SonicHub: Source List",
     130816,
     PACKET_INCOMPLETE,
     PACKET_FAST,
     0x40,
     0,
     {{"Manufacturer Code", 11, RES_MANUFACTURER, false, "=275", "Navico"},
      {"Reserved", 2, RES_NOTUSED, false, 0, ""},
      {"Industry Code", 3, RES_LOOKUP, false, "=4", "Marine Industry"},
      {"Reserved", BYTES(1), RES_NOTUSED, false, 0, ""},
      {"Proprietary ID", BYTES(1), RES_LOOKUP, false, "=8", "Source list"},
      {"Control", BYTES(1), RES_LOOKUP, false, ",0=Set,128=Ack", ""},
      {"Source ID", BYTES(1), RES_INTEGER, false, 0, ""},
      {"A", 8, RES_INTEGER, false, 0, ""},
      {"Text", BYTES(32), RES_STRINGLZ, false, 0, ""},
      {0}}}

    ,
    {"SonicHub: Control",
     130816,
     PACKET_INCOMPLETE,
     PACKET_FAST,
     0x40,
     0,
     {{"Manufacturer Code", 11, RES_MANUFACTURER, false, "=275", "Navico"},
      {"Reserved", 2, RES_NOTUSED, false, 0, ""},
      {"Industry Code", 3, RES_LOOKUP, false, "=4", "Marine Industry"},
      {"Reserved", BYTES(1), RES_NOTUSED, false, 0, ""},
      {"Proprietary ID", BYTES(1), RES_LOOKUP, false, "=9", "Control"},
      {"Control", BYTES(1), RES_LOOKUP, false, ",0=Set,128=Ack", ""},
      {"Item", BYTES(1), RES_LOOKUP, false, ",1=Mute on,2=Mute off", ""},
      {0}}}

    ,
    {"SonicHub: Unknown",
     130816,
     PACKET_INCOMPLETE,
     PACKET_FAST,
     0x40,
     0,
     {{"Manufacturer Code", 11, RES_MANUFACTURER, false, "=275", "Navico"},
      {"Reserved", 2, RES_NOTUSED, false, 0, ""},
      {"Industry Code", 3, RES_LOOKUP, false, "=4", "Marine Industry"},
      {"Reserved", BYTES(1), RES_NOTUSED, false, 0, ""},
      {"Proprietary ID", BYTES(1), RES_LOOKUP, false, "=9", "Unknown"},
      {"Control", BYTES(1), RES_LOOKUP, false, ",0=Set,128=Ack", ""},
      {"A", 8, RES_INTEGER, false, 0, ""},
      {"B", 8, RES_INTEGER, false, 0, ""},
      {0}}}

    ,
    {"SonicHub: FM Radio",
     130816,
     PACKET_INCOMPLETE,
     PACKET_FAST,
     0x40,
     0,
     {{"Manufacturer Code", 11, RES_MANUFACTURER, false, "=275", "Navico"},
      {"Reserved", 2, RES_NOTUSED, false, 0, ""},
      {"Industry Code", 3, RES_LOOKUP, false, "=4", "Marine Industry"},
      {"Reserved", BYTES(1), RES_NOTUSED, false, 0, ""},
      {"Proprietary ID", BYTES(1), RES_LOOKUP, false, "=12", "FM Radio"},
      {"Control", BYTES(1), RES_LOOKUP, false, ",0=Set,128=Ack", ""},
      {"Item", BYTES(1), RES_LOOKUP, false, ",1=Seeking up,2=Tuned,3=Seeking down", ""},
      {"Frequency", BYTES(4), 0.001, false, "kHz", ""},
      {"Noise level", 2, 1, false, 0, ""} // Not sure about this
      ,
      {"Signal level", 4, 1, false, 0, ""} // ... and this, doesn't make complete sense compared to display
      ,
      {"Reserved", 2, RES_BINARY, false, 0, ""},
      {"Text", BYTES(32), RES_STRINGLZ, false, 0, ""},
      {0}}}

    ,
    {"SonicHub: Playlist",
     130816,
     PACKET_INCOMPLETE,
     PACKET_FAST,
     0x40,
     0,
     {{"Manufacturer Code", 11, RES_MANUFACTURER, false, "=275", "Navico"},
      {"Reserved", 2, RES_NOTUSED, false, 0, ""},
      {"Industry Code", 3, RES_LOOKUP, false, "=4", "Marine Industry"},
      {"Reserved", BYTES(1), RES_NOTUSED, false, 0, ""},
      {"Proprietary ID", BYTES(1), RES_LOOKUP, false, "=13", "Playlist"},
      {"Control", BYTES(1), RES_LOOKUP, false, ",0=Set,128=Ack", ""},
      {"Item", BYTES(1), RES_LOOKUP, false, ",1=Report,4=Next Song,6=Previous Song", ""},
      {"A", BYTES(1), RES_INTEGER, false, 0, ""},
      {"Current Track", BYTES(4), RES_INTEGER, false, 0, ""},
      {"Tracks", BYTES(4), RES_INTEGER, false, 0, ""},
      {"Length", BYTES(4), 0.001, false, "s", ""},
      {"Position in track", BYTES(4), 0.001, false, "s", ""},
      {0}}}

    ,
    {"SonicHub: Track",
     130816,
     PACKET_INCOMPLETE,
     PACKET_FAST,
     0x40,
     0,
     {{"Manufacturer Code", 11, RES_MANUFACTURER, false, "=275", "Navico"},
      {"Reserved", 2, RES_NOTUSED, false, 0, ""},
      {"Industry Code", 3, RES_LOOKUP, false, "=4", "Marine Industry"},
      {"Reserved", BYTES(1), RES_NOTUSED, false, 0, ""},
      {"Proprietary ID", BYTES(1), RES_LOOKUP, false, "=14", "Track"},
      {"Control", BYTES(1), RES_LOOKUP, false, ",0=Set,128=Ack", ""},
      {"Item", BYTES(4), RES_INTEGER, false, 0, ""},
      {"Text", BYTES(32), RES_STRINGLZ, false, 0, ""},
      {0}}}

    ,
    {"SonicHub: Artist",
     130816,
     PACKET_INCOMPLETE,
     PACKET_FAST,
     0x40,
     0,
     {{"Manufacturer Code", 11, RES_MANUFACTURER, false, "=275", "Navico"},
      {"Reserved", 2, RES_NOTUSED, false, 0, ""},
      {"Industry Code", 3, RES_LOOKUP, false, "=4", "Marine Industry"},
      {"Reserved", BYTES(1), RES_NOTUSED, false, 0, ""},
      {"Proprietary ID", BYTES(1), RES_LOOKUP, false, "=15", "Artist"},
      {"Control", BYTES(1), RES_LOOKUP, false, ",0=Set,128=Ack", ""},
      {"Item", BYTES(4), RES_INTEGER, false, 0, ""},
      {"Text", BYTES(32), RES_STRINGLZ, false, 0, ""},
      {0}}}

    ,
    {"SonicHub: Album",
     130816,
     PACKET_INCOMPLETE,
     PACKET_FAST,
     0x40,
     0,
     {{"Manufacturer Code", 11, RES_MANUFACTURER, false, "=275", "Navico"},
      {"Reserved", 2, RES_NOTUSED, false, 0, ""},
      {"Industry Code", 3, RES_LOOKUP, false, "=4", "Marine Industry"},
      {"Reserved", BYTES(1), RES_NOTUSED, false, 0, ""},
      {"Proprietary ID", BYTES(1), RES_LOOKUP, false, "=16", "Album"},
      {"Control", BYTES(1), RES_LOOKUP, false, ",0=Set,128=Ack", ""},
      {"Item", BYTES(4), RES_INTEGER, false, 0, ""},
      {"Text", BYTES(32), RES_STRINGLZ, false, 0, ""},
      {0}}}

    ,
    {"SonicHub: Menu Item",
     130816,
     PACKET_INCOMPLETE,
     PACKET_FAST,
     0x40,
     0,
     {{"Manufacturer Code", 11, RES_MANUFACTURER, false, "=275", "Navico"},
      {"Reserved", 2, RES_NOTUSED, false, 0, ""},
      {"Industry Code", 3, RES_LOOKUP, false, "=4", "Marine Industry"},
      {"Reserved", BYTES(1), RES_NOTUSED, false, 0, ""},
      {"Proprietary ID", BYTES(1), RES_LOOKUP, false, "=19", "Menu Item"},
      {"Control", BYTES(1), RES_LOOKUP, false, ",0=Set,128=Ack", ""},
      {"Item", BYTES(4), RES_INTEGER, false, 0, ""},
      {"C", BYTES(1), 1, false, 0, ""},
      {"D", BYTES(1), 1, false, 0, ""},
      {"E", BYTES(1), 1, false, 0, ""},
      {"Text", BYTES(32), RES_STRINGLZ, false, 0, ""},
      {0}}}

    ,
    {"SonicHub: Zones",
     130816,
     PACKET_INCOMPLETE,
     PACKET_FAST,
     0x40,
     0,
     {{"Manufacturer Code", 11, RES_MANUFACTURER, false, "=275", "Navico"},
      {"Reserved", 2, RES_NOTUSED, false, 0, ""},
      {"Industry Code", 3, RES_LOOKUP, false, "=4", "Marine Industry"},
      {"Reserved", BYTES(1), RES_NOTUSED, false, 0, ""},
      {"Proprietary ID", BYTES(1), RES_LOOKUP, false, "=20", "Zones"},
      {"Control", BYTES(1), RES_LOOKUP, false, ",0=Set,128=Ack", ""},
      {"Zones", BYTES(1), RES_INTEGER, false, 0, ""},
      {0}}}

    ,
    {"SonicHub: Max Volume",
     130816,
     PACKET_INCOMPLETE,
     PACKET_FAST,
     0x40,
     0,
     {{"Manufacturer Code", 11, RES_MANUFACTURER, false, "=275", "Navico"},
      {"Reserved", 2, RES_NOTUSED, false, 0, ""},
      {"Industry Code", 3, RES_LOOKUP, false, "=4", "Marine Industry"},
      {"Reserved", BYTES(1), RES_NOTUSED, false, 0, ""},
      {"Proprietary ID", BYTES(1), RES_LOOKUP, false, "=23", "Max Volume"},
      {"Control", BYTES(1), RES_LOOKUP, false, ",0=Set,128=Ack", ""},
      {"Zone", BYTES(1), RES_LOOKUP, false, ",0=Zone 1,1=Zone 2,2=Zone 3", ""},
      {"Level", BYTES(1), RES_INTEGER, false, 0, ""},
      {0}}}

    ,
    {"SonicHub: Volume",
     130816,
     PACKET_COMPLETE,
     PACKET_FAST,
     8,
     0,
     {{"Manufacturer Code", 11, RES_MANUFACTURER, false, "=275", "Navico"},
      {"Reserved", 2, RES_NOTUSED, false, 0, ""},
      {"Industry Code", 3, RES_LOOKUP, false, "=4", "Marine Industry"},
      {"Reserved", BYTES(1), RES_NOTUSED, false, 0, ""},
      {"Proprietary ID", BYTES(1), RES_LOOKUP, false, "=24", "Volume"},
      {"Control", BYTES(1), RES_LOOKUP, false, ",0=Set,128=Ack", ""},
      {"Zone", BYTES(1), RES_LOOKUP, false, ",0=Zone 1,1=Zone 2,2=Zone 3", ""},
      {"Level", BYTES(1), RES_INTEGER, false, 0, ""},
      {0}}}

    ,
    {"SonicHub: Init #1",
     130816,
     PACKET_INCOMPLETE,
     PACKET_FAST,
     0x40,
     0,
     {{"Manufacturer Code", 11, RES_MANUFACTURER, false, "=275", "Navico"},
      {"Reserved", 2, RES_NOTUSED, false, 0, ""},
      {"Industry Code", 3, RES_LOOKUP, false, "=4", "Marine Industry"},
      {"Reserved", BYTES(1), RES_NOTUSED, false, 0, ""},
      {"Proprietary ID", BYTES(1), RES_LOOKUP, false, "=25", "Init #1"},
      {"Control", BYTES(1), RES_LOOKUP, false, ",0=Set,128=Ack", ""},
      {0}}}

    ,
    {"SonicHub: Position",
     130816,
     PACKET_INCOMPLETE,
     PACKET_FAST,
     0x40,
     0,
     {{"Manufacturer Code", 11, RES_MANUFACTURER, false, "=275", "Navico"},
      {"Reserved", 2, RES_NOTUSED, false, 0, ""},
      {"Industry Code", 3, RES_LOOKUP, false, "=4", "Marine Industry"},
      {"Reserved", BYTES(1), RES_NOTUSED, false, 0, ""},
      {"Proprietary ID", BYTES(1), RES_LOOKUP, false, "=48", "Position"},
      {"Control", BYTES(1), RES_LOOKUP, false, ",0=Set,128=Ack", ""},
      {"Position", BYTES(4), 0.001, false, "s", ""},
      {0}}}

    ,
    {"SonicHub: Init #3",
     130816,
     PACKET_INCOMPLETE | PACKET_NOT_SEEN,
     PACKET_FAST,
     9,
     0,
     {{"Manufacturer Code", 11, RES_MANUFACTURER, false, "=275", "Navico"},
      {"Reserved", 2, RES_NOTUSED, false, 0, ""},
      {"Industry Code", 3, RES_LOOKUP, false, "=4", "Marine Industry"},
      {"Reserved", BYTES(1), RES_NOTUSED, false, 0, ""},
      {"Proprietary ID", BYTES(1), RES_LOOKUP, false, "=50", "Init #3"},
      {"Control", BYTES(1), RES_LOOKUP, false, ",0=Set,128=Ack", ""},
      {"A", BYTES(1), RES_INTEGER, false, 0, ""},
      {"B", BYTES(1), RES_INTEGER, false, 0, ""},
      {0}}}

    ,
    {"Simrad: Text Message",
     130816,
     PACKET_INCOMPLETE,
     PACKET_FAST,
     0x40,
     0,
     {{"Manufacturer Code", 11, RES_MANUFACTURER, false, "=1857", "Simrad"},
      {"Reserved", 2, RES_NOTUSED, false, 0, ""},
      {"Industry Code", 3, RES_LOOKUP, false, "=4", "Marine Industry"},
      {"Reserved", BYTES(1), RES_NOTUSED, false, 0, ""},
      {"Proprietary ID", BYTES(1), RES_LOOKUP, false, "=50", "Init #3"} // FIXME
      ,
      {"A", BYTES(1), 1, false, 0, ""},
      {"B", BYTES(1), 1, false, 0, ""},
      {"C", BYTES(1), 1, false, 0, ""},
      {"SID", BYTES(1), 1, false, 0, ""},
      {"Prio", BYTES(1), 1, false, 0, ""},
      {"Text", BYTES(32), RES_ASCII, false, 0, ""},
      {0}}}

    ,
    {"Manufacturer Proprietary fast-packet non-addressed",
     130816,
     PACKET_INCOMPLETE,
     PACKET_FAST,
     223,
     0,
     {{"Manufacturer Code", 11, RES_MANUFACTURER, false, 0, ""},
      {"Reserved", 2, RES_NOTUSED, false, 0, ""},
      {"Industry Code", 3, RES_LOOKUP, false, LOOKUP_INDUSTRY_CODE, ""},
      {"Data", BYTES(221), RES_BINARY, false, 0, ""},
      {0}},
     0,
     0,
     true}

    ,
    {"Navico: Product Information",
     130817,
     PACKET_INCOMPLETE,
     PACKET_FAST,
     0x0e,
     0,
     {{"Manufacturer Code", 11, RES_MANUFACTURER, false, "=275", "Navico"},
      {"Reserved", 2, RES_NOTUSED, false, 0, ""},
      {"Industry Code", 3, RES_LOOKUP, false, "=4", "Marine Industry"},
      {"Product Code", BYTES(2), RES_INTEGER, false, 0, ""},
      {"Model", BYTES(32), RES_ASCII, false, 0, ""},
      {"A", BYTES(1), 1, false, 0, ""},
      {"B", BYTES(1), 1, false, 0, ""},
      {"C", BYTES(1), 1, false, 0, ""},
      {"Firmware version", BYTES(10), RES_ASCII, false, 0, ""},
      {"Firmware date", BYTES(32), RES_ASCII, false, 0, ""},
      {"Firmware time", BYTES(32), RES_ASCII, false, 0, ""},
      {0}}}

    ,
    {"Simnet: Reprogram Data",
     130818,
     PACKET_INCOMPLETE,
     PACKET_FAST,
     223,
     0,
     {{"Manufacturer Code", 11, RES_MANUFACTURER, false, "=1857", "Simrad"},
      {"Reserved", 2, RES_NOTUSED, false, 0, ""},
      {"Industry Code", 3, RES_LOOKUP, false, "=4", "Marine Industry"},
      {"Version", BYTES(2), RES_INTEGER, false, 0, ""},
      {"Sequence", BYTES(2), RES_INTEGER, false, 0, ""},
      {"Data", BYTES(217), RES_BINARY, false, 0, ""},
      {0}}}

    ,
    {"Simnet: Request Reprogram",
     130819,
     PACKET_FIELD_LENGTHS_UNKNOWN | PACKET_NOT_SEEN,
     PACKET_FAST,
     0x08,
     0,
     {{"Manufacturer Code", 11, RES_MANUFACTURER, false, "=1857", "Simrad"},
      {"Reserved", 2, RES_NOTUSED, false, 0, ""},
      {"Industry Code", 3, RES_LOOKUP, false, "=4", "Marine Industry"},
      {0}}}

    ,
    {"Simnet: Reprogram Status",
     130820,
     PACKET_FIELD_LENGTHS_UNKNOWN | PACKET_NOT_SEEN,
     PACKET_FAST,
     0x08,
     0,
     {{"Manufacturer Code", 11, RES_MANUFACTURER, false, "=1857", "Simrad"},
      {"Reserved", 2, RES_NOTUSED, false, 0, ""},
      {"Industry Code", 3, RES_LOOKUP, false, "=4", "Marine Industry"},
      {"Reserved", BYTES(1), RES_BINARY, false, 0, ""},
      {"Status", BYTES(1), 1, false, 0, ""},
      {"Reserved", BYTES(3), RES_BINARY, false, 0, ""},
      {0}}}

    /* M/V Dirona */
    ,
    {"Furuno: Unknown",
     130820,
     PACKET_INCOMPLETE,
     PACKET_FAST,
     0x08,
     0,
     {{"Manufacturer Code", 11, RES_MANUFACTURER, false, "=1855", "Furuno"},
      {"Reserved", 2, RES_NOTUSED, false, 0, ""},
      {"Industry Code", 3, RES_LOOKUP, false, "=4", "Marine Industry"},
      {"A", BYTES(1), 1, false, 0, ""},
      {"B", BYTES(1), 1, false, 0, ""},
      {"C", BYTES(1), 1, false, 0, ""},
      {"D", BYTES(1), 1, false, 0, ""},
      {"E", BYTES(1), 1, false, 0, ""},
      {0}}}

    /* Fusion */
    ,
    {"Fusion: Source Name",
     130820,
     PACKET_INCOMPLETE,
     PACKET_FAST,
     13,
     0,
     {{"Manufacturer Code", 11, RES_MANUFACTURER, false, "=419", "Fusion"},
      {"Reserved", 2, RES_NOTUSED, false, 0, ""},
      {"Industry Code", 3, RES_LOOKUP, false, "=4", "Marine Industry"},
      {"Message ID", BYTES(1), 1, false, "=2", "Source"},
      {"A", BYTES(1), 1, false, 0, ""},
      {"Source ID", BYTES(1), 1, false, 0, ""},
      {"Current Source ID", BYTES(1), 1, false, 0, ""},
      {"D", BYTES(1), 1, false, 0, ""},
      {"E", BYTES(1), 1, false, 0, ""},
      {"Source", BYTES(5), RES_STRINGLZ, false, 0, ""},
      {0}}}

    ,
    {"Fusion: Track Info",
     130820,
     PACKET_INCOMPLETE,
     PACKET_FAST,
     23,
     0,
     {{"Manufacturer Code", 11, RES_MANUFACTURER, false, "=419", "Fusion"},
      {"Reserved", 2, RES_NOTUSED, false, 0, ""},
      {"Industry Code", 3, RES_LOOKUP, false, "=4", "Marine Industry"},
      {"Message ID", BYTES(1), 1, false, "=4", "Track Info"},
      {"A", BYTES(2), 1, false, 0, ""},
      {"Transport", 4, RES_LOOKUP, false, ",1=Playing,2=Paused", ""},
      {"X", 4, 1, false, 0, ""},
      {"B", BYTES(1), 1, false, 0, ""},
      {"Track #", BYTES(2), 1, false, 0, ""},
      {"C", BYTES(2), 1, false, 0, ""},
      {"Track Count", BYTES(2), 1, false, 0, ""},
      {"E", BYTES(2), 1, false, 0, ""},
      {"Track Length", BYTES(3), 0.001, false, 0, ""},
      {"G", BYTES(3), 0.001, false, 0, ""},
      {"H", BYTES(2), 1, false, 0, ""}}},
    {"Fusion: Track",
     130820,
     PACKET_INCOMPLETE,
     PACKET_FAST,
     0x20,
     0,
     {{"Manufacturer Code", 11, RES_MANUFACTURER, false, "=419", "Fusion"},
      {"Reserved", 2, RES_NOTUSED, false, 0, ""},
      {"Industry Code", 3, RES_LOOKUP, false, "=4", "Marine Industry"},
      {"Message ID", BYTES(1), 1, false, "=5", "Track Title"},
      {"A", BYTES(1), 1, false, 0, ""},
      {"B", BYTES(5), 1, false, 0, ""},
      {"Track", BYTES(10), RES_STRINGLZ, false, 0, ""},
      {0}}}

    ,
    {"Fusion: Artist",
     130820,
     PACKET_INCOMPLETE,
     PACKET_FAST,
     0x20,
     0,
     {{"Manufacturer Code", 11, RES_MANUFACTURER, false, "=419", "Fusion"},
      {"Reserved", 2, RES_NOTUSED, false, 0, ""},
      {"Industry Code", 3, RES_LOOKUP, false, "=4", "Marine Industry"},
      {"Message ID", BYTES(1), 1, false, "=6", "Track Artist"},
      {"A", BYTES(1), 1, false, 0, ""},
      {"B", BYTES(5), 1, false, 0, ""},
      {"Artist", BYTES(10), RES_STRINGLZ, false, 0, ""},
      {0}}}

    ,
    {"Fusion: Album",
     130820,
     PACKET_INCOMPLETE,
     PACKET_FAST,
     0x20,
     0,
     {{"Manufacturer Code", 11, RES_MANUFACTURER, false, "=419", "Fusion"},
      {"Reserved", 2, RES_NOTUSED, false, 0, ""},
      {"Industry Code", 3, RES_LOOKUP, false, "=4", "Marine Industry"},
      {"Message ID", BYTES(1), 1, false, "=7", "Track Album"},
      {"A", BYTES(1), 1, false, 0, ""},
      {"B", BYTES(5), 1, false, 0, ""},
      {"Album", BYTES(10), RES_STRINGLZ, false, 0, ""},
      {0}}}

    ,
    {"Fusion: Unit Name",
     130820,
     PACKET_INCOMPLETE,
     PACKET_FAST,
     0x20,
     0,
     {{"Manufacturer Code", 11, RES_MANUFACTURER, false, "=419", "Fusion"},
      {"Reserved", 2, RES_NOTUSED, false, 0, ""},
      {"Industry Code", 3, RES_LOOKUP, false, "=4", "Marine Industry"},
      {"Message ID", BYTES(1), 1, false, "=33", "Unit Name"},
      {"A", BYTES(1), 1, false, 0, ""},
      {"Name", BYTES(14), RES_STRINGLZ, false, 0, ""},
      {0}}}

    ,
    {"Fusion: Zone Name",
     130820,
     PACKET_INCOMPLETE,
     PACKET_FAST,
     0x20,
     0,
     {{"Manufacturer Code", 11, RES_MANUFACTURER, false, "=419", "Fusion"},
      {"Reserved", 2, RES_NOTUSED, false, 0, ""},
      {"Industry Code", 3, RES_LOOKUP, false, "=4", "Marine Industry"},
      {"Message ID", BYTES(1), 1, false, "=45", "Zone Name"},
      {"A", BYTES(1), 1, false, 0, ""},
      {"Number", BYTES(1), 1, false, 0, ""},
      {"Name", BYTES(13), RES_STRINGLZ, false, 0, ""},
      {0}}}

    ,
    {"Fusion: Play Progress",
     130820,
     PACKET_INCOMPLETE,
     PACKET_FAST,
     9,
     0,
     {{"Manufacturer Code", 11, RES_MANUFACTURER, false, "=419", "Fusion"},
      {"Reserved", 2, RES_NOTUSED, false, 0, ""},
      {"Industry Code", 3, RES_LOOKUP, false, "=4", "Marine Industry"},
      {"Message ID", BYTES(1), 1, false, "=9", "Track Progress"},
      {"A", BYTES(1), 1, false, 0, ""},
      {"B", BYTES(1), 1, false, 0, ""},
      {"Progress", BYTES(3), 0.001, false, "s", ""},
      {0}}}

    ,
    {"Fusion: AM/FM Station",
     130820,
     PACKET_INCOMPLETE,
     PACKET_FAST,
     0x0A,
     0,
     {{"Manufacturer Code", 11, RES_MANUFACTURER, false, "=419", "Fusion"},
      {"Reserved", 2, RES_NOTUSED, false, 0, ""},
      {"Industry Code", 3, RES_LOOKUP, false, "=4", "Marine Industry"},
      {"Message ID", BYTES(1), 1, false, "=11", "AM/FM Station"},
      {"A", BYTES(1), 1, false, 0, ""},
      {"AM/FM", BYTES(1), RES_LOOKUP, false, ",0=AM,1=FM", ""},
      {"B", BYTES(1), 1, false, 0, ""},
      {"Frequency", BYTES(4), 0.000001, false, "Hz", ""},
      {"C", BYTES(1), 1, false, 0, ""},
      {"Track", BYTES(10), RES_STRINGLZ, false, 0, ""},
      {0}}}

    ,
    {"Fusion: VHF",
     130820,
     PACKET_INCOMPLETE,
     PACKET_FAST,
     9,
     0,
     {{"Manufacturer Code", 11, RES_MANUFACTURER, false, "=419", "Fusion"},
      {"Reserved", 2, RES_NOTUSED, false, 0, ""},
      {"Industry Code", 3, RES_LOOKUP, false, "=4", "Marine Industry"},
      {"Message ID", BYTES(1), 1, false, "=12", "VHF"},
      {"A", BYTES(1), 1, false, 0, ""},
      {"B", BYTES(1), 1, false, 0, ""},
      {"Channel", BYTES(1), 1, false, 0, ""},
      {"D", BYTES(3), 1, false, 0, ""},
      {0}}}

    ,
    {"Fusion: Squelch",
     130820,
     PACKET_INCOMPLETE,
     PACKET_FAST,
     6,
     0,
     {{"Manufacturer Code", 11, RES_MANUFACTURER, false, "=419", "Fusion"},
      {"Reserved", 2, RES_NOTUSED, false, 0, ""},
      {"Industry Code", 3, RES_LOOKUP, false, "=4", "Marine Industry"},
      {"Message ID", BYTES(1), 1, false, "=13", "Squelch"},
      {"A", BYTES(1), 1, false, 0, ""},
      {"B", BYTES(1), 1, false, 0, ""},
      {"Squelch", BYTES(1), 1, false, 0, ""},
      {0}}}

    ,
    {"Fusion: Scan",
     130820,
     PACKET_INCOMPLETE,
     PACKET_FAST,
     6,
     0,
     {{"Manufacturer Code", 11, RES_MANUFACTURER, false, "=419", "Fusion"},
      {"Reserved", 2, RES_NOTUSED, false, 0, ""},
      {"Industry Code", 3, RES_LOOKUP, false, "=4", "Marine Industry"},
      {"Message ID", BYTES(1), 1, false, "=14", "Scan"},
      {"A", BYTES(1), 1, false, 0, ""},
      {"B", BYTES(1), 1, false, 0, ""},
      {"Scan", BYTES(1), RES_LOOKUP, false, ",0=Off,1=Scan", ""},
      {0}}}

    ,
    {"Fusion: Menu Item",
     130820,
     PACKET_INCOMPLETE,
     PACKET_FAST,
     23,
     0,
     {{"Manufacturer Code", 11, RES_MANUFACTURER, false, "=419", "Fusion"},
      {"Reserved", 2, RES_NOTUSED, false, 0, ""},
      {"Industry Code", 3, RES_LOOKUP, false, "=4", "Marine Industry"},
      {"Message ID", BYTES(1), 1, false, "=17", "Menu Item"},
      {"A", BYTES(1), 1, false, 0, ""},
      {"B", BYTES(1), 1, false, 0, ""},
      {"Line", BYTES(1), 1, false, 0, ""},
      {"E", BYTES(1), 1, false, 0, ""},
      {"F", BYTES(1), 1, false, 0, ""},
      {"G", BYTES(1), 1, false, 0, ""},
      {"H", BYTES(1), 1, false, 0, ""},
      {"I", BYTES(1), 1, false, 0, ""},
      {"Text", BYTES(5), RES_STRINGLZ, false, 0, ""},
      {0}}}

    ,
    {"Fusion: Replay",
     130820,
     PACKET_INCOMPLETE,
     PACKET_FAST,
     23,
     0,
     {{"Manufacturer Code", 11, RES_MANUFACTURER, false, "=419", "Fusion"},
      {"Reserved", 2, RES_NOTUSED, false, 0, ""},
      {"Industry Code", 3, RES_LOOKUP, false, "=4", "Marine Industry"},
      {"Message ID", BYTES(1), 1, false, "=20", "Replay"},
      {"A", BYTES(1), 1, false, 0, ""},
      {"Mode", BYTES(1), RES_LOOKUP, false, ",9=USB Repeat,10=USB Shuffle,12=iPod Repeat,13=iPod Shuffle", ""},
      {"C", BYTES(3), 1, false, 0, ""},
      {"D", BYTES(1), 1, false, 0, ""},
      {"E", BYTES(1), 1, false, 0, ""},
      {"Status", BYTES(1), RES_LOOKUP, false, ",0=Off,1=One/Track,2=All/Album", ""},
      {"H", BYTES(1), 1, false, 0, ""},
      {"I", BYTES(1), 1, false, 0, ""},
      {"J", BYTES(1), 1, false, 0, ""},
      {0}}}

    ,
    {"Fusion: Mute",
     130820,
     PACKET_INCOMPLETE,
     PACKET_FAST,
     5,
     0,
     {{"Manufacturer Code", 11, RES_MANUFACTURER, false, "=419", "Fusion"},
      {"Reserved", 2, RES_NOTUSED, false, 0, ""},
      {"Industry Code", 3, RES_LOOKUP, false, "=4", "Marine Industry"},
      {"Message ID", BYTES(1), 1, false, "=23", "Mute"},
      {"A", BYTES(1), 1, false, 0, ""},
      {"Mute", BYTES(1), RES_LOOKUP, false, ",1=Muted,2=Not Muted", ""},
      {0}}}

    ,
    // Range: 0 to +24
    {"Fusion: Sub Volume",
     130820,
     PACKET_INCOMPLETE,
     PACKET_FAST,
     8,
     0,
     {{"Manufacturer Code", 11, RES_MANUFACTURER, false, "=419", "Fusion"},
      {"Reserved", 2, RES_NOTUSED, false, 0, ""},
      {"Industry Code", 3, RES_LOOKUP, false, "=4", "Marine Industry"},
      {"Message ID", BYTES(1), 1, false, "=26", "Sub Volume"},
      {"A", BYTES(1), 1, false, 0, ""},
      {"Zone 1", BYTES(1), 1, false, "vol", ""},
      {"Zone 2", BYTES(1), 1, false, "vol", ""},
      {"Zone 3", BYTES(1), 1, false, "vol", ""},
      {"Zone 4", BYTES(1), 1, false, "vol", ""},
      {0}}}

    ,
    // Range: -15 to +15
    {"Fusion: Tone",
     130820,
     PACKET_INCOMPLETE | PACKET_NOT_SEEN,
     PACKET_FAST,
     8,
     0,
     {{"Manufacturer Code", 11, RES_MANUFACTURER, false, "=419", "Fusion"},
      {"Reserved", 2, RES_NOTUSED, false, 0, ""},
      {"Industry Code", 3, RES_LOOKUP, false, "=4", "Marine Industry"},
      {"Message ID", BYTES(1), 1, false, "=27", "Tone"},
      {"A", BYTES(1), 1, false, 0, ""},
      {"B", BYTES(1), 1, false, 0, ""},
      {"Bass", BYTES(1), 1, true, "vol", ""},
      {"Mid", BYTES(1), 1, true, "vol", ""},
      {"Treble", BYTES(1), 1, true, "vol", ""},
      {0}}}

    ,
    {"Fusion: Volume",
     130820,
     PACKET_INCOMPLETE | PACKET_NOT_SEEN,
     PACKET_FAST,
     0x0A,
     0,
     {{"Manufacturer Code", 11, RES_MANUFACTURER, false, "=419", "Fusion"},
      {"Reserved", 2, RES_NOTUSED, false, 0, ""},
      {"Industry Code", 3, RES_LOOKUP, false, "=4", "Marine Industry"},
      {"Message ID", BYTES(1), 1, false, "=29", "Volume"},
      {"A", BYTES(1), 1, false, 0, ""},
      {"Zone 1", BYTES(1), 1, false, "vol", ""},
      {"Zone 2", BYTES(1), 1, false, "vol", ""},
      {"Zone 3", BYTES(1), 1, false, "vol", ""},
      {"Zone 4", BYTES(1), 1, false, "vol", ""},
      {0}}}

    ,
    {"Fusion: Power State",
     130820,
     PACKET_INCOMPLETE | PACKET_NOT_SEEN,
     PACKET_FAST,
     5,
     0,
     {{"Manufacturer Code", 11, RES_MANUFACTURER, false, "=419", "Fusion"},
      {"Reserved", 2, RES_NOTUSED, false, 0, ""},
      {"Industry Code", 3, RES_LOOKUP, false, "=4", "Marine Industry"},
      {"Message ID", BYTES(1), 1, false, "=32", "Power"},
      {"A", BYTES(1), 1, false, 0, ""},
      {"State", BYTES(1), RES_LOOKUP, false, ",1=On,2=Off", ""},
      {0}}}

    ,
    {"Fusion: SiriusXM Channel",
     130820,
     PACKET_INCOMPLETE | PACKET_NOT_SEEN,
     PACKET_FAST,
     0x20,
     0,
     {{"Manufacturer Code", 11, RES_MANUFACTURER, false, "=419", "Fusion"},
      {"Reserved", 2, RES_NOTUSED, false, 0, ""},
      {"Industry Code", 3, RES_LOOKUP, false, "=4", "Marine Industry"},
      {"Message ID", BYTES(1), 1, false, "=36", "SiriusXM Channel"},
      {"A", BYTES(4), 1, false, 0, ""},
      {"Channel", BYTES(12), RES_STRINGLZ, false, 0, ""},
      {0}}}

    ,
    {"Fusion: SiriusXM Title",
     130820,
     PACKET_INCOMPLETE | PACKET_NOT_SEEN,
     PACKET_FAST,
     0x20,
     0,
     {{"Manufacturer Code", 11, RES_MANUFACTURER, false, "=419", "Fusion"},
      {"Reserved", 2, RES_NOTUSED, false, 0, ""},
      {"Industry Code", 3, RES_LOOKUP, false, "=4", "Marine Industry"},
      {"Message ID", BYTES(1), 1, false, "=37", "SiriusXM Title"},
      {"A", BYTES(4), 1, false, 0, ""},
      {"Title", BYTES(12), RES_STRINGLZ, false, 0, ""},
      {0}}}

    ,
    {"Fusion: SiriusXM Artist",
     130820,
     PACKET_INCOMPLETE | PACKET_NOT_SEEN,
     PACKET_FAST,
     0x20,
     0,
     {{"Manufacturer Code", 11, RES_MANUFACTURER, false, "=419", "Fusion"},
      {"Reserved", 2, RES_NOTUSED, false, 0, ""},
      {"Industry Code", 3, RES_LOOKUP, false, "=4", "Marine Industry"},
      {"Message ID", BYTES(1), 1, false, "=38", "SiriusXM Artist"},
      {"A", BYTES(4), 1, false, 0, ""},
      {"Artist", BYTES(12), RES_STRINGLZ, false, 0, ""},
      {0}}}

    ,
    {"Fusion: SiriusXM Genre",
     130820,
     PACKET_INCOMPLETE | PACKET_NOT_SEEN,
     PACKET_FAST,
     0x20,
     0,
     {{"Manufacturer Code", 11, RES_MANUFACTURER, false, "=419", "Fusion"},
      {"Reserved", 2, RES_NOTUSED, false, 0, ""},
      {"Industry Code", 3, RES_LOOKUP, false, "=4", "Marine Industry"},
      {"Message ID", BYTES(1), 1, false, "=40", "SiriusXM Genre"},
      {"A", BYTES(4), 1, false, 0, ""},
      {"Genre", BYTES(12), RES_STRINGLZ, false, 0, ""},
      {0}}}

    /* M/V Dirona */
    ,
    {"Furuno: Unknown",
     130821,
     PACKET_INCOMPLETE,
     PACKET_FAST,
     0x0c,
     0,
     {{"Manufacturer Code", 11, RES_MANUFACTURER, false, "=1855", "Furuno"},
      {"Reserved", 2, RES_NOTUSED, false, 0, ""},
      {"Industry Code", 3, RES_LOOKUP, false, "=4", "Marine Industry"},
      {"SID", BYTES(1), 1, false, 0, ""},
      {"A", BYTES(1), 1, false, 0, ""},
      {"B", BYTES(1), 1, false, 0, ""},
      {"C", BYTES(1), 1, false, 0, ""},
      {"D", BYTES(1), 1, false, 0, ""},
      {"E", BYTES(1), 1, false, 0, ""},
      {"F", BYTES(1), 1, false, 0, ""},
      {"G", BYTES(1), 1, false, 0, ""},
      {"H", BYTES(1), 1, false, 0, ""},
      {"I", BYTES(1), 1, false, 0, ""},
      {0}}}

    ,
    {"Maretron: Proprietary Temperature High Range",
     130823,
     PACKET_COMPLETE,
     PACKET_FAST,
     9,
     0,
     {{"Manufacturer Code", 11, RES_MANUFACTURER, false, "=137", "Maretron"},
      {"Reserved", 2, RES_NOTUSED, false, 0, ""},
      {"Industry Code", 3, RES_LOOKUP, false, "=4", "Marine Industry"},
      {"SID", BYTES(1), 1, false, 0, ""},
      {"Instance", BYTES(1), 1, false, 0, ""},
      {"Source", BYTES(1), RES_LOOKUP, false, LOOKUP_TEMPERATURE_SOURCE, ""},
      {"Actual Temperature", BYTES(2), RES_TEMPERATURE_HIGH, false, "K", ""},
      {"Set Temperature", BYTES(2), RES_TEMPERATURE_HIGH, false, "K", ""},
      {0}}}

    ,
    {"B&G: Wind data",
     130824,
     PACKET_INCOMPLETE,
     PACKET_SINGLE,
     0x08,
     0,
     {{"Manufacturer Code", 11, RES_MANUFACTURER, false, "=381", "B&G"},
      {"Reserved", 2, RES_NOTUSED, false, 0, ""},
      {"Industry Code", 3, RES_LOOKUP, false, "=4", "Marine Industry"},
      {"Field 4", BYTES(1), 1, false, 0, ""},
      {"Field 5", BYTES(1), 1, false, 0, ""},
      {"Timestamp", BYTES(4), 1, false, 0, "Increasing field, what else can it be?"},
      {0}}}

    /* M/V Dirona */
    ,
    {"Maretron: Annunciator",
     130824,
     PACKET_INCOMPLETE,
     PACKET_FAST,
     9,
     0,
     {{"Manufacturer Code", 11, RES_MANUFACTURER, false, "=137", "Maretron"},
      {"Reserved", 2, RES_NOTUSED, false, 0, ""},
      {"Industry Code", 3, RES_LOOKUP, false, "=4", "Marine Industry"},
      {"Field 4", BYTES(1), 1, false, 0, ""},
      {"Field 5", BYTES(1), 1, false, 0, ""},
      {"Field 6", BYTES(2), 1, false, 0, ""},
      {"Field 7", BYTES(1), 1, false, 0, ""},
      {"Field 8", BYTES(2), 1, false, 0, ""},
      {0}}}

    /* Uwe Lovas has seen this from EP-70R */
    ,
    {"Lowrance: unknown",
     130827,
     PACKET_INCOMPLETE | PACKET_NOT_SEEN,
     PACKET_FAST,
     10,
     0,
     {{"Manufacturer Code", 11, RES_MANUFACTURER, false, "=140", "Lowrance"},
      {"Reserved", 2, RES_NOTUSED, false, 0, ""},
      {"Industry Code", 3, RES_LOOKUP, false, "=4", "Marine Industry"},
      {"A", BYTES(1), 1, false, 0, ""},
      {"B", BYTES(1), 1, false, 0, ""},
      {"C", BYTES(1), 1, false, 0, ""},
      {"D", BYTES(1), 1, false, 0, ""},
      {"E", BYTES(2), 1, false, 0, ""},
      {"F", BYTES(2), 1, false, 0, ""},
      {0}}}

    ,
    {"Simnet: Set Serial Number",
     130828,
     PACKET_INCOMPLETE | PACKET_NOT_SEEN,
     PACKET_FAST,
     0x08,
     0,
     {{"Manufacturer Code", 11, RES_MANUFACTURER, false, "=1857", "Simrad"},
      {"Reserved", 2, RES_NOTUSED, false, 0, ""},
      {"Industry Code", 3, RES_LOOKUP, false, "=4", "Marine Industry"},
      {0}}}

    ,
    {"Suzuki: Engine and Storage Device Config",
     130831,
     PACKET_INCOMPLETE,
     PACKET_FAST,
     0x08,
     0,
     {{"Manufacturer Code", 11, RES_MANUFACTURER, false, 0, ""} // FIXME
      ,
      {"Reserved", 2, RES_NOTUSED, false, 0, ""},
      {"Industry Code", 3, RES_LOOKUP, false, "=4", "Marine Industry"},
      {0}}}

    ,
    {"Simnet: Fuel Used - High Resolution",
     130832,
     PACKET_INCOMPLETE | PACKET_NOT_SEEN,
     PACKET_FAST,
     0x08,
     0,
     {{"Manufacturer Code", 11, RES_MANUFACTURER, false, "=1857", "Simrad"},
      {"Reserved", 2, RES_NOTUSED, false, 0, ""},
      {"Industry Code", 3, RES_LOOKUP, false, "=4", "Marine Industry"},
      {0}}}

    ,
    {"Simnet: Engine and Tank Configuration",
     130834,
     PACKET_INCOMPLETE | PACKET_NOT_SEEN,
     PACKET_FAST,
     0x08,
     0,
     {{"Manufacturer Code", 11, RES_MANUFACTURER, false, "=1857", "Simrad"},
      {"Reserved", 2, RES_NOTUSED, false, 0, ""},
      {"Industry Code", 3, RES_LOOKUP, false, "=4", "Marine Industry"},
      {0}}}

    ,
    {"Simnet: Set Engine and Tank Configuration",
     130835,
     PACKET_INCOMPLETE | PACKET_NOT_SEEN,
     PACKET_FAST,
     0x08,
     0,
     {{"Manufacturer Code", 11, RES_MANUFACTURER, false, "=1857", "Simrad"},
      {"Reserved", 2, RES_NOTUSED, false, 0, ""},
      {"Industry Code", 3, RES_LOOKUP, false, "=4", "Marine Industry"},
      {0}}}

    /* Seen when HDS8 configures EP65R */
    ,
    {"Simnet: Fluid Level Sensor Configuration",
     130836,
     PACKET_INCOMPLETE,
     PACKET_FAST,
     0x0e,
     0,
     {{"Manufacturer Code", 11, RES_MANUFACTURER, false, "=1857", "Simrad"},
      {"Reserved", 2, RES_NOTUSED, false, 0, ""},
      {"Industry Code", 3, RES_LOOKUP, false, "=4", "Marine Industry"},
      {"C", BYTES(1), 1, false, 0, ""},
      {"Device", BYTES(1), RES_INTEGER, false, 0, ""},
      {"Instance", BYTES(1), 1, false, 0, ""},
      {"F", 1 * 4, 1, false, 0, ""},
      {"Tank type", 1 * 4, RES_LOOKUP, false, ",0=Fuel,1=Water,2=Gray water,3=Live well,4=Oil,5=Black water", ""},
      {"Capacity", BYTES(4), 0.1, false, 0, ""},
      {"G", BYTES(1), 1, false, 0, ""},
      {"H", BYTES(2), 1, true, 0, ""},
      {"I", BYTES(1), 1, true, 0, ""},
      {0}}}

    ,
    {"Maretron Proprietary Switch Status Counter",
     130836,
     PACKET_COMPLETE,
     PACKET_FAST,
     16,
     0,
     {{"Manufacturer Code", 11, RES_MANUFACTURER, false, "=137", "Maretron"},
      {"Reserved", 2, RES_NOTUSED, false, 0, ""},
      {"Industry Code", 3, RES_LOOKUP, false, "=4", "Marine Industry"},
      {"Instance", BYTES(1), 1, false, 0, ""},
      {"Indicator Number", BYTES(1), 1, false, 0, ""},
      {"Start Date", BYTES(2), RES_DATE, false, "days", "Timestamp of last reset in Days since January 1, 1970"},
      {"Start Time", BYTES(4), RES_TIME, false, "s", "Timestamp of last reset Seconds since midnight"},
      {"OFF Counter", BYTES(1), RES_INTEGER, false, 0, ""},
      {"ON Counter", BYTES(1), RES_INTEGER, false, 0, ""},
      {"ERROR Counter", BYTES(1), RES_INTEGER, false, 0, ""},
      {"Switch Status", 2, RES_LOOKUP, false, LOOKUP_OFF_ON, ""},
      {"Reserved", BYTES(2), 1, false, 0, ""},
      {0}}}

    ,
    {"Simnet: Fuel Flow Turbine Configuration",
     130837,
     PACKET_INCOMPLETE | PACKET_NOT_SEEN,
     PACKET_FAST,
     0x08,
     0,
     {{"Manufacturer Code", 11, RES_MANUFACTURER, false, "=1857", "Simrad"},
      {"Reserved", 2, RES_NOTUSED, false, 0, ""},
      {"Industry Code", 3, RES_LOOKUP, false, "=4", "Marine Industry"},
      {0}}}

    ,
    {"Maretron Proprietary Switch Status Timer",
     130837,
     PACKET_COMPLETE,
     PACKET_FAST,
     23,
     0,
     {{"Manufacturer Code", 11, RES_MANUFACTURER, false, "=137", "Maretron"},
      {"Reserved", 2, RES_NOTUSED, false, 0, ""},
      {"Industry Code", 3, RES_LOOKUP, false, "=4", "Marine Industry"},
      {"Instance", BYTES(1), 1, false, 0, ""},
      {"Indicator Number", BYTES(1), 1, false, 0, ""},
      {"Start Date", BYTES(2), RES_DATE, false, "days", "Timestamp of last reset in Days since January 1, 1970"},
      {"Start Time", BYTES(4), RES_TIME, false, "s", "Timestamp of last reset Seconds since midnight"},
      {"Accumulated OFF Period", BYTES(4), RES_DECIMAL, false, "seconds", ""},
      {"Accumulated ON Period", BYTES(4), RES_DECIMAL, false, "seconds", ""},
      {"Accumulated ERROR Period", BYTES(4), RES_DECIMAL, false, "seconds", ""},
      {"Switch Status", 2, RES_LOOKUP, false, LOOKUP_OFF_ON, ""},
      {"Reserved", 6, 1, false, 0, ""},
      {0}}}

    ,
    {"Simnet: Fluid Level Warning",
     130838,
     PACKET_INCOMPLETE | PACKET_NOT_SEEN,
     PACKET_FAST,
     0x08,
     0,
     {{"Manufacturer Code", 11, RES_MANUFACTURER, false, "=1857", "Simrad"},
      {"Reserved", 2, RES_NOTUSED, false, 0, ""},
      {"Industry Code", 3, RES_LOOKUP, false, "=4", "Marine Industry"},
      {0}}}

    ,
    {"Simnet: Pressure Sensor Configuration",
     130839,
     PACKET_INCOMPLETE | PACKET_NOT_SEEN,
     PACKET_FAST,
     0x08,
     0,
     {{"Manufacturer Code", 11, RES_MANUFACTURER, false, "=1857", "Simrad"},
      {"Reserved", 2, RES_NOTUSED, false, 0, ""},
      {"Industry Code", 3, RES_LOOKUP, false, "=4", "Marine Industry"},
      {0}}}

    ,
    {"Simnet: Data User Group Configuration",
     130840,
     PACKET_INCOMPLETE | PACKET_NOT_SEEN,
     PACKET_FAST,
     0x08,
     0,
     {{"Manufacturer Code", 11, RES_MANUFACTURER, false, "=1857", "Simrad"},
      {"Reserved", 2, RES_NOTUSED, false, 0, ""},
      {"Industry Code", 3, RES_LOOKUP, false, "=4", "Marine Industry"},
      {0}}}

    /* Where did this come from ?
    ,
    { "Simnet: DSC Message", 130842,
     PACKET_INCOMPLETE | PACKET_NOT_SEEN, PACKET_FAST,
    0x08, 0,
      { { "Manufacturer Code", 11, RES_MANUFACTURER, false, "=1857", "Simrad" }
      , { "Reserved", 2, RES_NOTUSED, false, 0, "" }
      , { "Industry Code", 3, RES_LOOKUP, false, "=4", "Marine Industry" }
      , { 0 }
      }
    }
    */

    ,
    {"Simnet: AIS Class B static data (msg 24 Part A)",
     130842,
     PACKET_INCOMPLETE | PACKET_NOT_SEEN,
     PACKET_FAST,
     0x1d,
     0,
     {{"Manufacturer Code", 11, RES_MANUFACTURER, false, "=1857", "Simrad"},
      {"Reserved", 2, RES_NOTUSED, false, 0, ""},
      {"Industry Code", 3, RES_LOOKUP, false, "=4", "Marine Industry"},
      {"Message ID", 6, 1, false, "=0", "Msg 24 Part A"},
      {"Repeat indicator", 2, RES_LOOKUP, false, LOOKUP_REPEAT_INDICATOR, ""},
      {"D", BYTES(1), 1, false, 0, ""},
      {"E", BYTES(1), 1, false, 0, ""},
      {"User ID", BYTES(4), RES_INTEGER, false, "MMSI", ""},
      {"Name", BYTES(20), RES_ASCII, false, 0, ""},
      {0}}}

    ,
    {"Furuno: Six Degrees Of Freedom Movement",
     130842,
     PACKET_INCOMPLETE,
     PACKET_FAST,
     29,
     0,
     {{"Manufacturer Code", 11, RES_MANUFACTURER, false, "=1855", "Furuno"},
      {"Reserved", 2, RES_NOTUSED, false, 0, ""},
      {"Industry Code", 3, RES_LOOKUP, false, "=4", "Marine Industry"},
      {"A", BYTES(4), 1, true, 0, ""},
      {"B", BYTES(4), 1, true, 0, ""},
      {"C", BYTES(4), 1, true, 0, ""},
      {"D", BYTES(1), 1, true, 0, ""},
      {"E", BYTES(4), 1, true, 0, ""},
      {"F", BYTES(4), 1, true, 0, ""},
      {"G", BYTES(2), 1, true, 0, ""},
      {"H", BYTES(2), 1, true, 0, ""},
      {"I", BYTES(2), 1, true, 0, ""},
      {0}}}

    ,
    {"Simnet: AIS Class B static data (msg 24 Part B)",
     130842,
     PACKET_INCOMPLETE,
     PACKET_FAST,
     0x25,
     0,
     {{"Manufacturer Code", 11, RES_MANUFACTURER, false, "=1857", "Simrad"},
      {"Reserved", 2, RES_NOTUSED, false, 0, ""},
      {"Industry Code", 3, RES_LOOKUP, false, "=4", "Marine Industry"},
      {"Message ID", 6, 1, false, "=1", "Msg 24 Part B"},
      {"Repeat indicator", 2, RES_LOOKUP, false, LOOKUP_REPEAT_INDICATOR, ""},
      {"D", BYTES(1), 1, false, 0, ""},
      {"E", BYTES(1), 1, false, 0, ""},
      {"User ID", BYTES(4), RES_INTEGER, false, "MMSI", ""},
      {"Type of ship", BYTES(1), RES_LOOKUP, false, LOOKUP_SHIP_TYPE, ""},
      {"Vendor ID", BYTES(7), RES_ASCII, false, 0, ""},
      {"Callsign", BYTES(7), RES_ASCII, false, 0, ",0=unavailable"},
      {"Length", BYTES(2), 0.1, false, "m", ""},
      {"Beam", BYTES(2), 0.1, false, "m", ""},
      {"Position reference from Starboard", BYTES(2), 0.1, false, "m", ""},
      {"Position reference from Bow", BYTES(2), 0.1, false, "m", ""},
      {"Mothership User ID", BYTES(4), RES_INTEGER, false, "MMSI", "Id of mother ship sent by daughter vessels"},
      {"Reserved", 2, RES_BINARY, false, 0, "reserved"},
      {"Spare", 6, RES_INTEGER, false, 0, ",0=unavailable"},
      {0}}}

    ,
    {"Furuno: Heel Angle, Roll Information",
     130843,
     PACKET_INCOMPLETE,
     PACKET_FAST,
     0x08,
     0,
     {{"Manufacturer Code", 11, RES_MANUFACTURER, false, "=1855", "Furuno"},
      {"Reserved", 2, RES_NOTUSED, false, 0, ""},
      {"Industry Code", 3, RES_LOOKUP, false, "=4", "Marine Industry"},
      {"A", BYTES(1), 1, false, 0, ""},
      {"B", BYTES(1), 1, false, 0, ""},
      {"Yaw", BYTES(2), RES_RADIANS, true, "rad", ""},
      {"Pitch", BYTES(2), RES_RADIANS, true, "rad", ""},
      {"Roll", BYTES(2), RES_RADIANS, true, "rad", ""},
      {0}}}

    ,
    {"Simnet: Sonar Status, Frequency and DSP Voltage",
     130843,
     PACKET_INCOMPLETE | PACKET_NOT_SEEN,
     PACKET_FAST,
     0x08,
     0,
     {{"Manufacturer Code", 11, RES_MANUFACTURER, false, "=1857", "Simrad"},
      {"Reserved", 2, RES_NOTUSED, false, 0, ""},
      {"Industry Code", 3, RES_LOOKUP, false, "=4", "Marine Industry"},
      {0}}}

    ,
    {"Simnet: Compass Heading Offset",
     130845,
     PACKET_INCOMPLETE,
     PACKET_FAST,
     0x0e,
     0,
     {{"Manufacturer Code", 11, RES_MANUFACTURER, false, "=1857", "Simrad"},
      {"Reserved", 2, RES_NOTUSED, false, 0, ""},
      {"Industry Code", 3, RES_LOOKUP, false, "=4", "Marine Industry"},
      {"Message ID", 6, 1, false, 0, ""},
      {"Repeat indicator", 2, RES_LOOKUP, false, LOOKUP_REPEAT_INDICATOR, ""},
      {"Unused", BYTES(3), 1, false, 0, ""},
      {"Type", BYTES(2), 1, false, "=0", "Heading Offset"},
      {"A", BYTES(2), RES_NOTUSED, false, 0, ""},
      {"Angle", BYTES(2), RES_RADIANS, true, "rad", ""},
      {"Unused", BYTES(2), RES_NOTUSED, false, 0, ""},
      {0}}}

    ,
    {"Furuno: Multi Sats In View Extended",
     130845,
     PACKET_INCOMPLETE,
     PACKET_FAST,
     0x08,
     0,
     {{"Manufacturer Code", 11, RES_MANUFACTURER, false, "=1855", "Furuno"},
      {"Reserved", 2, RES_NOTUSED, false, 0, ""},
      {"Industry Code", 3, RES_LOOKUP, false, "=4", "Marine Industry"},
      {0}}}

    ,
    {"Simnet: Compass Local Field",
     130845,
     PACKET_INCOMPLETE | PACKET_NOT_SEEN,
     PACKET_FAST,
     0x0e,
     0,
     {{"Manufacturer Code", 11, RES_MANUFACTURER, false, "=1857", "Simrad"},
      {"Reserved", 2, RES_NOTUSED, false, 0, ""},
      {"Industry Code", 3, RES_LOOKUP, false, "=4", "Marine Industry"},
      {"Message ID", 6, 1, false, 0, ""},
      {"Repeat indicator", 2, RES_LOOKUP, false, LOOKUP_REPEAT_INDICATOR, ""},
      {"Unused", BYTES(3), 1, false, 0, ""},
      {"Type", BYTES(2), 1, false, "=768", "Local field"},
      {"A", BYTES(2), RES_NOTUSED, false, 0, ""},
      {"Local field", BYTES(2), RES_PERCENTAGE, false, "%", ""},
      {"Unused", BYTES(2), RES_NOTUSED, false, 0, ""},
      {0}}}

    ,
    {"Simnet: Compass Field Angle",
     130845,
     PACKET_INCOMPLETE | PACKET_NOT_SEEN,
     PACKET_FAST,
     0x0e,
     0,
     {{"Manufacturer Code", 11, RES_MANUFACTURER, false, "=1857", "Simrad"},
      {"Reserved", 2, RES_NOTUSED, false, 0, ""},
      {"Industry Code", 3, RES_LOOKUP, false, "=4", "Marine Industry"},
      {"Message ID", 6, 1, false, 0, ""},
      {"Repeat indicator", 2, RES_LOOKUP, false, LOOKUP_REPEAT_INDICATOR, ""},
      {"Unused", BYTES(3), 1, false, 0, ""},
      {"Type", BYTES(2), 1, false, "=1024", "Local field"},
      {"A", BYTES(2), 1, false, 0, ""},
      {"Field angle", BYTES(2), RES_RADIANS, true, "rad", ""},
      {"Unused", BYTES(2), RES_NOTUSED, false, 0, ""},
      {0}}}

    ,
    {"Simnet: Parameter Handle",
     130845,
     PACKET_INCOMPLETE | PACKET_NOT_SEEN,
     PACKET_FAST,
     0x0e,
     0,
     {{"Manufacturer Code", 11, RES_MANUFACTURER, false, "=1857", "Simrad"},
      {"Reserved", 2, RES_NOTUSED, false, 0, ""},
      {"Industry Code", 3, RES_LOOKUP, false, "=4", "Marine Industry"},
      {"Message ID", 6, 1, false, 0, ""},
      {"Repeat indicator", 2, RES_LOOKUP, false, LOOKUP_REPEAT_INDICATOR, ""},
      {"D", BYTES(1), 1, false, 0, ""},
      {"Group", BYTES(1), 1, false, 0, ""},
      {"F", BYTES(1), 1, false, 0, ""},
      {"G", BYTES(1), 1, false, 0, ""},
      {"H", BYTES(1), 1, false, 0, ""},
      {"I", BYTES(1), 1, false, 0, ""},
      {"J", BYTES(1), 1, false, 0, ""},
      {"Backlight",
       BYTES(1),
       RES_LOOKUP,
       false,
       ",1=Day Mode,4=Night Mode,11=Level 1,22=Level 2,33=Level 3,44=Level 4,55=Level 5,66=Level 6,77=Level 7,88=Level 8,99=Level "
       "9",
       ""},
      {"L", BYTES(2), 1, false, 0, ""},
      {0}}}

    ,
    {"Furuno: Motion Sensor Status Extended",
     130846,
     PACKET_INCOMPLETE,
     PACKET_FAST,
     0x08,
     0,
     {{"Manufacturer Code", 11, RES_MANUFACTURER, false, "=1855", "Furuno"},
      {"Reserved", 2, RES_NOTUSED, false, 0, ""},
      {"Industry Code", 3, RES_LOOKUP, false, "=4", "Marine Industry"},
      {0}}}

    ,
    {"SeaTalk: Node Statistics",
     130847,
     PACKET_INCOMPLETE | PACKET_NOT_SEEN,
     PACKET_FAST,
     0,
     0,
     {{"Manufacturer Code", 11, RES_MANUFACTURER, false, "=1851", "Raymarine"},
      {"Software Release", BYTES(2), 1, false, 0, ""},
      {"Development Version", BYTES(1), 1, false, 0, ""},
      {"Product Code", BYTES(2), 1, false, 0, ""},
      {"Year", BYTES(1), 1, false, 0, ""},
      {"Month", BYTES(1), 1, false, 0, ""},
      {"Device Number", BYTES(2), 1, false, 0, ""},
      {"Node Voltage", BYTES(2), 0.01, false, "V", ""},
      {0}}}

#define LOOKUP_SIMNET_AP_EVENTS \
  (",6=Standby"                 \
   ",9=Auto mode"               \
   ",10=Nav mode"               \
   ",13=Non Follow Up mode"     \
   ",15=Wind mode"              \
   ",18=Square (Turn)"          \
   ",19=C-Turn"                 \
   ",20=U-Turn"                 \
   ",21=Spiral (Turn)"          \
   ",22=Zig Zag (Turn)"         \
   ",23=Lazy-S (Turn)"          \
   ",24=Depth (Turn)"           \
   ",26=Change Course")

#define LOOKUP_SIMNET_DIRECTION \
  (",2=Port"                    \
   ",3=Starboard"               \
   ",4=Left rudder (port)"      \
   ",5=Right rudder (starboard)")

    ,
    {"Simnet: Event Command: AP command",
     130850,
     PACKET_INCOMPLETE,
     PACKET_FAST,
     12,
     0,
     {{"Manufacturer Code", 11, RES_MANUFACTURER, false, "=1857", "Simrad"},
      {"Reserved", 2, RES_NOTUSED, false, 0, ""},
      {"Industry Code", 3, RES_LOOKUP, false, "=4", "Marine Industry"},
      {"Proprietary ID", BYTES(1), RES_LOOKUP, false, ",2,4", "AP command"},
      {"B", BYTES(2), RES_NOTUSED, false, 0, ""},
      {"Controlling Device", BYTES(1), 1, false, 0, ""},
      {"Event", BYTES(2), RES_LOOKUP, false, LOOKUP_SIMNET_AP_EVENTS, ""},
      {"Direction", BYTES(1), RES_LOOKUP, false, LOOKUP_SIMNET_DIRECTION, ""},
      {"Angle", BYTES(2), RES_RADIANS, false, "rad", ""},
      {"G", BYTES(1), RES_NOTUSED, false, 0, ""},
      {0}}}

    ,
    {"Simnet: Event Command: Alarm?",
     130850,
     PACKET_INCOMPLETE,
     PACKET_FAST,
     12,
     0,
     {{"Manufacturer Code", 11, RES_MANUFACTURER, false, "=1857", "Simrad"},
      {"Reserved", 2, RES_NOTUSED, false, 0, ""},
      {"Industry Code", 3, RES_LOOKUP, false, "=4", "Marine Industry"},
      {"A", BYTES(2), 1, false, 0, ""},
      {"Proprietary ID", BYTES(1), RES_LOOKUP, false, "=1", "Alarm command"},
      {"C", BYTES(1), 1, false, 0, ""},
      {"Alarm", BYTES(2), RES_LOOKUP, false, ",57=Raise,56=Clear", ""},
      {"Message ID", BYTES(2), RES_INTEGER, false, 0, ""},
      {"F", BYTES(1), 1, false, 0, ""},
      {"G", BYTES(1), 1, false, 0, ""},
      {0}}}

    ,
    {"Simnet: Event Command: Unknown",
     130850,
     PACKET_INCOMPLETE,
     PACKET_FAST,
     12,
     0,
     {{"Manufacturer Code", 11, RES_MANUFACTURER, false, "=1857", "Simrad"},
      {"Reserved", 2, RES_NOTUSED, false, 0, ""},
      {"Industry Code", 3, RES_LOOKUP, false, "=4", "Marine Industry"},
      {"A", BYTES(2), 1, false, 0, ""},
      {"Proprietary ID", BYTES(1), RES_LOOKUP, false, "=1", "Alarm command"},
      {"B", BYTES(1), 1, false, 0, ""},
      {"C", BYTES(2), 1, false, 0, ""},
      {"D", BYTES(2), 1, false, 0, ""},
      {"E", BYTES(2), 1, false, 0, ""},
      {0}}}

    ,
    {"Simnet: Event Reply: AP command",
     130851,
     PACKET_INCOMPLETE,
     PACKET_FAST,
     12,
     0,
     {{"Manufacturer Code", 11, RES_MANUFACTURER, false, "=1857", "Simrad"},
      {"Reserved", 2, RES_NOTUSED, false, 0, ""},
      {"Industry Code", 3, RES_LOOKUP, false, "=4", "Marine Industry"},
      {"Proprietary ID", BYTES(1), RES_LOOKUP, false, "=2", "AP command"},
      {"B", BYTES(2), 1, false, 0, ""},
      {"Controlling Device", BYTES(1), 1, false, 0, ""},
      {"Event", BYTES(2), RES_LOOKUP, false, LOOKUP_SIMNET_AP_EVENTS, ""},
      {"Direction", BYTES(1), RES_LOOKUP, false, LOOKUP_SIMNET_DIRECTION, ""},
      {"Angle", BYTES(2), RES_RADIANS, false, "rad", ""},
      {"G", BYTES(1), 1, false, 0, ""},
      {0}}}

    ,
    {"Simnet: Alarm Message",
     130856,
     PACKET_INCOMPLETE | PACKET_NOT_SEEN,
     PACKET_FAST,
     0x08,
     0,
     {{"Manufacturer Code", 11, RES_MANUFACTURER, false, "=1857", "Simrad"},
      {"Reserved", 2, RES_NOTUSED, false, 0, ""},
      {"Industry Code", 3, RES_LOOKUP, false, "=4", "Marine Industry"},
      {"Message ID", BYTES(2), 1, false, 0, ""},
      {"B", BYTES(1), 1, false, 0, ""},
      {"C", BYTES(1), 1, false, 0, ""},
      {"Text", BYTES(255), RES_ASCII, false, 0, ""},
      {0}}}

    ,
    {"Airmar: Additional Weather Data",
     130880,
     PACKET_INCOMPLETE | PACKET_NOT_SEEN,
     PACKET_FAST,
     0x1e,
     0,
     {{"Manufacturer Code", 11, RES_MANUFACTURER, false, "=135", "Airmar"},
      {"Reserved", 2, RES_NOTUSED, false, 0, ""},
      {"Industry Code", 3, RES_LOOKUP, false, "=4", "Marine Industry"},
      {"C", BYTES(1), 1, false, 0, ""},
      {"Apparent Windchill Temperature", BYTES(2), RES_TEMPERATURE, false, "K", ""},
      {"True Windchill Temperature", BYTES(2), RES_TEMPERATURE, false, "K", ""},
      {"Dewpoint", BYTES(2), RES_TEMPERATURE, false, "K", ""},
      {0}}}

    ,
    {"Airmar: Heater Control",
     130881,
     PACKET_INCOMPLETE | PACKET_NOT_SEEN,
     PACKET_FAST,
     0x9,
     0,
     {{"Manufacturer Code", 11, RES_MANUFACTURER, false, "=135", "Airmar"},
      {"Reserved", 2, RES_NOTUSED, false, 0, ""},
      {"Industry Code", 3, RES_LOOKUP, false, "=4", "Marine Industry"},
      {"C", BYTES(1), 1, false, 0, ""},
      {"Plate Temperature", BYTES(2), RES_TEMPERATURE, false, "K", ""},
      {"Air Temperature", BYTES(2), RES_TEMPERATURE, false, "K", ""},
      {"Dewpoint", BYTES(2), RES_TEMPERATURE, false, "K", ""},
      {0}}}

    ,
    {"Airmar: POST",
     130944,
     PACKET_INCOMPLETE | PACKET_NOT_SEEN,
     PACKET_FAST,
     0x8,
     0,
     {{"Manufacturer Code", 11, RES_MANUFACTURER, false, "=135", "Airmar"},
      {"Reserved", 2, RES_NOTUSED, false, 0, ""},
      {"Industry Code", 3, RES_LOOKUP, false, "=4", "Marine Industry"},
      {"Control", 4, RES_LOOKUP, false, ",0=Report previous values,1=Generate new values", ""},
      {"Reserved", 7, RES_BINARY, false, 0, ""},
      {"Number of ID/test result pairs to follow", BYTES(1), RES_INTEGER, false, 0, ""}

      ,
      {"Test ID",
       BYTES(1),
       RES_LOOKUP,
       false,
       ",1=Format Code,2=Factory EEPROM,3=User EEPROM,4=Water Temp Sensor,5=Sonar Transceiver,6=Speed sensor,7=Internal "
       "temperature sensor,8=Battery voltage sensor",
       "See Airmar docs for table of IDs and failure codes; these lookup values are for DST200"},
      {"Test result", BYTES(1), RES_LOOKUP, false, ",0=Pass", "Values other than 0 are failure codes"},
      {0}}}

    ,
    {"Actisense: Operating mode",
     ACTISENSE_BEM + 0x11,
     PACKET_COMPLETE,
     PACKET_SINGLE,
     0x0e,
     0,
     {{"SID", BYTES(1), 1, false, 0, ""},
      {"Model ID", BYTES(2), RES_INTEGER, false, 0, ""},
      {"Serial ID", BYTES(4), RES_INTEGER, false, 0, ""},
      {"Error ID", BYTES(4), RES_INTEGER, false, 0, ""},
      {"Operating Mode", BYTES(2), 1, false, 0, ""},
      {0}}}

    ,
    {"Actisense: Startup status",
     ACTISENSE_BEM + 0xf0,
     PACKET_INCOMPLETE,
     PACKET_FAST,
     0x0f,
     0,
     {{"SID", BYTES(1), 1, false, 0, ""},
      {"Model ID", BYTES(2), RES_INTEGER, false, 0, ""},
      {"Serial ID", BYTES(4), RES_INTEGER, false, 0, ""},
      {"Error ID", BYTES(4), RES_INTEGER, false, 0, ""},
      {"Firmware version", BYTES(2), 0.001, false, 0, ""},
      {"Reset status", BYTES(1), 1, false, 0, ""},
      {"A", BYTES(1), 1, false, 0, ""},
      {0}}}

    ,
    {"Actisense: System status",
     ACTISENSE_BEM + 0xf2,
     PACKET_INCOMPLETE,
     PACKET_FAST,
     0x22,
     0,
     {{"SID", BYTES(1), 1, false, 0, ""},
      {"Model ID", BYTES(2), RES_INTEGER, false, 0, ""},
      {"Serial ID", BYTES(4), RES_INTEGER, false, 0, ""},
      {"Error ID", BYTES(4), RES_INTEGER, false, 0, ""},
      {"Indi channel count", BYTES(1), 1, false, 0, ""},
      {"Ch1 Rx Bandwidth", BYTES(1), 1, false, 0, ""},
      {"Ch1 Rx Load", BYTES(1), 1, false, 0, ""},
      {"Ch1 Rx Filtered", BYTES(1), 1, false, 0, ""},
      {"Ch1 Rx Dropped", BYTES(1), 1, false, 0, ""},
      {"Ch1 Tx Bandwidth", BYTES(1), 1, false, 0, ""},
      {"Ch1 Tx Load", BYTES(1), 1, false, 0, ""},
      {"Ch2 Rx Bandwidth", BYTES(1), 1, false, 0, ""},
      {"Ch2 Rx Load", BYTES(1), 1, false, 0, ""},
      {"Ch2 Rx Filtered", BYTES(1), 1, false, 0, ""},
      {"Ch2 Rx Dropped", BYTES(1), 1, false, 0, ""},
      {"Ch2 Tx Bandwidth", BYTES(1), 1, false, 0, ""},
      {"Ch2 Tx Load", BYTES(1), 1, false, 0, ""},
      {"Uni channel count", BYTES(1), 1, false, 0, ""},
      {"Ch1 Bandwidth", BYTES(1), 1, false, 0, ""},
      {"Ch1 Deleted", BYTES(1), 1, false, 0, ""},
      {"Ch1 BufferLoading", BYTES(1), 1, false, 0, ""},
      {"Ch1 PointerLoading", BYTES(1), 1, false, 0, ""},
      {"Ch2 Bandwidth", BYTES(1), 1, false, 0, ""},
      {"Ch2 Deleted", BYTES(1), 1, false, 0, ""},
      {"Ch2 BufferLoading", BYTES(1), 1, false, 0, ""},
      {"Ch2 PointerLoading", BYTES(1), 1, false, 0, ""},
      {0}}}

    ,
    {"Actisense: ?",
     ACTISENSE_BEM + 0xf4,
     PACKET_INCOMPLETE,
     PACKET_FAST,
     17,
     0,
     {{"SID", BYTES(1), 1, false, 0, ""},
      {"Model ID", BYTES(2), RES_INTEGER, false, 0, ""},
      {"Serial ID", BYTES(4), RES_INTEGER, false, 0, ""},
      {0}}}

    ,
    {"iKonvert: Network status",
     IKONVERT_BEM,
     PACKET_COMPLETE,
     PACKET_FAST,
     15,
     0,
     {{"CAN network load", BYTES(1), RES_INTEGER, false, 0, ""},
      {"Errors", BYTES(4), RES_INTEGER, false, 0, ""},
      {"Device count", BYTES(1), RES_INTEGER, false, 0, ""},
      {"Uptime", BYTES(4), RES_INTEGER, false, 0, "s"},
      {"Gateway address", BYTES(1), RES_INTEGER, false, 0, ""},
      {"Rejected TX requests", BYTES(4), RES_INTEGER, false, 0, ""},
      {0}}}

};

size_t pgnListSize = ARRAY_SIZE(pgnList);

#else
extern Pgn    pgnList[];
extern size_t pgnListSize;
#endif

typedef struct
{
  char *name;
  int   id;
} Company;

#ifdef GLOBAL_COMPANYLIST
/* https://www.nmea.org/Assets/20190623%20manu%20code%20and%20product%20code.pdf */
static Company companyList[] = {{"Actia", 199},
                                {"Actisense", 273},
                                {"Advansea", 578},
                                {"AEM Power", 735},
                                {"Aetna Engineering/Fireboy/Xintex", 215},
                                {"Airmar", 135},
                                {"Alltek Marine Electronics Corp", 459},
                                {"Amphenol LTW Technology", 274},
                                {"Aquatic AV", 600},
                                {"ARKS Enterprises, Inc.", 69},
                                {"Arlt Tecnologies", 614},
                                {"ASA Electronics", 905},
                                {"Attwood Marine", 502},
                                {"Au Electronics Group", 735},
                                {"Autonnic", 715},
                                {"Aventics GmbH", 605},
                                {"Bavaria Yacts", 637},
                                {"Beede Instruments", 185},
                                {"BEP Marine", 116},
                                {"BEP Marine", 295},
                                {"Beyond Measure", 396},
                                {"B & G", 381},
                                {"BJ Technologies (Beneteau)", 802},
                                {"Blue Seas", 969},
                                {"Blue Water Data", 148},
                                {"Blue Water Desalination", 811},
                                {"Böning Automationstechnologie GmbH & Co. KG", 341},
                                {"Broyda Industries", 795},
                                {"Camano Light", 384},
                                {"Canadian Automotive", 796},
                                {"Capi 2", 394},
                                {"Carling Technologies Inc. (Moritz Aerospace)", 176},
                                {"Chetco Digitial Instruments", 481},
                                {"Clarion US", 773},
                                {"Coelmo SRL Italy", 286},
                                {"Comar Systems Limited", 438},
                                {"ComNav", 404},
                                {"Cox Powertrain", 968},
                                {"CPAC Systems AB", 165},
                                {"Cummins", 440},
                                {"DaeMyung", 743},
                                {"Data Panel Corp", 868},
                                {"Dief", 329},
                                {"Digital Switching Systems", 211},
                                {"Digital Yacht", 437},
                                {"Disenos Y Technologia", 201},
                                {"Diverse Yacht Services", 641},
                                {"Ecotronix", 930},
                                {"Egersund Marine Electronics AS", 426},
                                {"Electronic Design", 373},
                                {"EMMI NETWORK S.L.", 224},
                                {"Empir Bus", 304},
                                {"em-trak Marine Electronics", 427},
                                {"Eride", 243},
                                {"Evinrude/BRP", 163},
                                {"Faria Instruments", 1863},
                                {"Fell Marine", 844},
                                {"Fischer Panda", 311},
                                {"Fischer Panda DE", 785},
                                {"Fischer Panda Generators", 356},
                                {"FLIR", 815},
                                {"Floscan Instrument Co. Inc.", 192},
                                {"Furuno", 1855},
                                {"Fusion Electronics", 419},
                                {"FW Murphy/Enovation Controls", 78},
                                {"Garmin", 229},
                                {"Garmin", 645},
                                {"GeoNav", 385},
                                {"Gill Sensors", 803},
                                {"Glendinning", 378},
                                {"GME aka Standard Communications Pty LTD", 475},
                                {"Groco", 272},
                                {"Hamilton Jet", 283},
                                {"Hemisphere GPS Inc", 88},
                                {"HMI Systems", 776},
                                {"Honda Marine", 175},
                                {"Honda Marine", 200},
                                {"Honda Marine", 225},
                                {"Honda Marine", 250},
                                {"Honda Motor Company LTD", 140},
                                {"Honda Motor Company LTD", 257},
                                {"Humminbird Marine Electronics", 467},
                                {"Humminbird Marine Electronics", 476},
                                {"ICOM", 315},
                                {"Intellian", 606},
                                {"Japan Radio Co", 1853},
                                {"JL Audio", 704},
                                {"Johnson Outdoors Marine Electronics Inc Geonav", 385},
                                {"Kohler Power Systems", 85},
                                {"Korean Maritime University", 345},
                                {"Kvasar", 743},
                                {"Kvasar AB", 1859},
                                {"KVH", 579},
                                {"L3 Technologies", 890},
                                {"Lcj Capteurs", 499},
                                {"Litton", 1858},
                                {"Livorsi Marine", 400},
                                {"Lowrance", 140},
                                {"Lumishore", 798},
                                {"LxNav", 739},
                                {"Maretron", 137},
                                {"Marinecraft (South Korea)", 571},
                                {"Marines Co (South Korea)", 909},
                                {"Marinesoft Co. LTD", 510},
                                {"Mastervolt", 355},
                                {"MBW Technologies", 307},
                                {"McMurdo Group aka Orolia LTD", 573},
                                {"Mercury Marine", 144},
                                {"MMP", 1860},
                                {"Moritz Aerospace", 176},
                                {"Mystic Valley Communications", 198},
                                {"National Instruments Korea", 529},
                                {"Nautibus Electronic GmbH", 147},
                                {"Nautic-on", 911},
                                {"Navico", 275},
                                {"Navionics", 1852},
                                {"Naviop S.R.L.", 503},
                                {"Nexfour Solutions", 896},
                                {"Nobletec", 193},
                                {"NoLand Engineering", 517},
                                {"Northern Lights", 374},
                                {"Northstar Technologies", 1854},
                                {"NovAtel", 305},
                                {"Ocean Sat BV", 478},
                                {"Ocean Signal", 777},
                                {"Oceanvolt", 847},
                                {"Offshore Systems (UK) Ltd.", 161},
                                {"Onwa Marine", 532},
                                {"Parker Hannifin aka Village Marine Tech", 451},
                                {"Poly Planar", 781},
                                {"Prospec", 862},
                                {"Qwerty", 328},
                                {"Raymarine", 1851},
                                {"REAP Systems", 734},
                                {"Rhodan Marine Systems", 894},
                                {"Rockford Corp", 688},
                                {"Rolls Royce Marine", 370},
                                {"Rose Point Navigation Systems", 384},
                                {"Sailormade Marine Telemetry/Tetra Technology LTD", 235},
                                {"SamwonIT", 612},
                                {"SAN GIORGIO S.E.I.N", 460},
                                {"San Jose Technology", 580},
                                {"Sea Cross Marine AB", 471},
                                {"Sea Recovery", 285},
                                {"Seekeeper", 778},
                                {"Shenzhen Jiuzhou Himunication", 658},
                                {"Ship Module aka Customware", 595},
                                {"Simrad", 1857},
                                {"SI-TEX Marine Electronics", 470},
                                {"Sleipner Motor AS", 306},
                                {"Standard Horizon", 421},
                                {"Still Water Designs and Audio", 799},
                                {"Suzuki Motor Corporation", 586},
                                {"TeamSurv", 838},
                                {"Teleflex Marine (SeaStar Solutions)", 1850},
                                {"Thrane and Thrane", 351},
                                {"Tides Marine", 797},
                                {"Timbolier Industries", 962},
                                {"Tohatsu Co, JP", 431},
                                {"Transas USA", 518},
                                {"Trimble", 1856},
                                {"True Heading AB", 422},
                                {"Twin Disc", 80},
                                {"Undheim Systems", 824},
                                {"US Coast Guard", 591},
                                {"VDO (aka Continental-Corporation)", 443},
                                {"Vector Cantech", 1861},
                                {"Veethree Electronics & Marine", 466},
                                {"Vesper Marine Ltd", 504},
                                {"Victron Energy", 358},
                                {"Volvo Penta", 174},
                                {"Watcheye", 493},
                                {"Wema U.S.A dba KUS", 644},
                                {"Westerbeke", 154},
                                {"Woosung", 744},
                                {"Xantrex Technology Inc.", 168},
                                {"Xintex/Atena", 215},
                                {"Yacht Control", 583},
                                {"Yacht Devices", 717},
                                {"Yacht Monitoring Solutions", 233},
                                {"Yamaha Marine", 1862},
                                {"Yanmar Marine", 172},
                                {"ZF", 228}};
#endif
