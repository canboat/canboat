# CANBOAT

A small but effective set of command-line utilities to work with CAN networks on BOATs. Guess you now know where the name comes from.

The most common version of CAN networks on board, and in fact at the moment the only ones that this suite can analyse, are NMEA 2000 PGNs.

The NMEA 2000 database and implementation is copyrighted by the NMEA (National Marine Electronics Association). Access is restricted to members and parties that pay for it. If they do so they are not able to divulge the content of the database, thus making it impossible for open source developers to get access to it.

For this reason we have reverse engineered the NMEA 2000 database by network observation and assembling data from public sources.

To use the programs included in this project you may need a supported CAN interface. This can be a marketed-as-such NMEA 2000 Gateway or a non NMEA specific CAN interface. 

For more information go to the [CANBoat Wiki](http://github.com/canboat/canboat/wiki).

where you can find instructions on how to build the programs on your own computer and how to start extending the PGN database.

## Version history

See [Changelog](CHANGELOG.md).

---

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
