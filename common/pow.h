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

#ifndef POW_H_INCLUDED
#define POW_H_INCLUDED

#include "dup.h"

/**
 * POW2(n) evaluates to the floating point value
 * of 2^n.
 *
 * Can be used for values 0..39
 */

#define POW2(n) 1. DUP(n, *2)

/**
 * POW2NEG(n) evaluates to the floating point value
 * of 2^-n.
 *
 * Can be used for values 0..39
 */

#define POW2NEG(n) 1. DUP(n, / 2)

#endif
