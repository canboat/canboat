/*

Analyzes NMEA 2000 PGNs.

(C) 2009-2023, Kees Verruijt, Harlingen, The Netherlands.

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

#ifndef DUP_H_INCLUDED
#define DUP_H_INCLUDED

#define DUP(n, c) DUP##n(c)

#define DUP39(c) DUP30(c) DUP9(c)
#define DUP38(c) DUP30(c) DUP8(c)
#define DUP37(c) DUP30(c) DUP7(c)
#define DUP36(c) DUP30(c) DUP6(c)
#define DUP35(c) DUP30(c) DUP5(c)
#define DUP34(c) DUP30(c) DUP4(c)
#define DUP33(c) DUP30(c) DUP3(c)
#define DUP32(c) DUP30(c) DUP2(c)
#define DUP31(c) DUP30(c) DUP1(c)
#define DUP30(c) DUP20(c) DUP10(c)

#define DUP29(c) DUP20(c) DUP9(c)
#define DUP28(c) DUP20(c) DUP8(c)
#define DUP27(c) DUP20(c) DUP7(c)
#define DUP26(c) DUP20(c) DUP6(c)
#define DUP25(c) DUP20(c) DUP5(c)
#define DUP24(c) DUP20(c) DUP4(c)
#define DUP23(c) DUP20(c) DUP3(c)
#define DUP22(c) DUP20(c) DUP2(c)
#define DUP21(c) DUP20(c) DUP1(c)
#define DUP20(c) DUP10(c) DUP10(c)

#define DUP19(c) DUP10(c) DUP9(c)
#define DUP18(c) DUP10(c) DUP8(c)
#define DUP17(c) DUP10(c) DUP7(c)
#define DUP16(c) DUP10(c) DUP6(c)
#define DUP15(c) DUP10(c) DUP5(c)
#define DUP14(c) DUP10(c) DUP4(c)
#define DUP13(c) DUP10(c) DUP3(c)
#define DUP12(c) DUP10(c) DUP2(c)
#define DUP11(c) DUP10(c) DUP1(c)

#define DUP10(c) c c c c c c c c c c
#define DUP9(c) c c c c c c c c c
#define DUP8(c) c c c c c c c c
#define DUP7(c) c c c c c c c
#define DUP6(c) c c c c c c
#define DUP5(c) c c c c c
#define DUP4(c) c c c c
#define DUP3(c) c c c
#define DUP2(c) c c
#define DUP1(c) c
#define DUP0(c)

#define DUP100(c) DUP10(DUP10(c))
#define DUP1000(c) DUP10(DUP100(c))

#endif
