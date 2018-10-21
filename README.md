# CANBOAT

A small but effective set of command-line utilities to work with CAN networks on BOATs. Guess you now know where the name comes from.

The most common version of CAN networks on board, and in fact at the moment the only ones that this suite can analyse, are NMEA 2000 PGNs.

The NMEA 2000 database and implementation is copyrighted by the NMEA (National Marine Electronics Association). Access is restricted to members and parties that pay for it. If they do so they are not able to divulge the content of the database, thus making it impossible for open source developers to get access to it.

For this reason we have reverse engineered the NMEA 2000 database by network observation and assembling data from public sources.

To use the programs included in this project you may need a supported CAN interface. This can be a marketed-as-such NMEA 2000 Gateway or a non NMEA specific CAN interface. 

For more information go to the CANBoat Wiki at [http://github.com/canboat/canboat/wiki].

where you can find instructions on how to build the programs on your own computer and how to start extending the PGN database.

## Version history

See [CHANGELOG.md].

---

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
