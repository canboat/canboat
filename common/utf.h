/*

(C) 2009-2026, Kees Verruijt, Harlingen, The Netherlands.
(C) 2019, Davidp

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

#ifndef UTF_H_INCLUDED
#define UTF_H_INCLUDED

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

typedef uint8_t  utf8_t;  // The type of a single UTF-8 character
typedef uint16_t utf16_t; // The type of a single UTF-16 character

/*
 * Converts a UTF-16 string to a UTF-8 string.
 *
 * utf16:
 * The UTF-16 string, not null-terminated.
 *
 * utf16_len:
 * The length of the UTF-16 string, in 16-bit characters.
 *
 * utf8:
 * The buffer where the resulting UTF-8 string will be stored.
 * If set to NULL, indicates that the function should just calculate
 * the required buffer size and not actually perform any conversions.
 *
 * utf8_len:
 * The length of the UTF-8 buffer, in 8-bit characters.
 * Ignored if utf8 is NULL.
 *
 * return:
 * If utf8 is NULL, the size of the required UTF-8 buffer.
 * Otherwise, the number of characters written to the utf8 buffer.
 *
 */
size_t utf16_to_utf8(utf16_t const *utf16, size_t utf16_len, utf8_t *utf8, size_t utf8_len);

/*
 * Converts a UTF-8 string to a UTF-16 string.
 *
 * utf8:
 * The UTF-8 string, not null-terminated.
 *
 * utf8_len:
 * The length of the UTF-8 string, in 8-bit characters.
 *
 * utf16:
 * The buffer where the resulting UTF-16 string will be stored.
 * If set to NULL, indicates that the function should just calculate
 * the required buffer size and not actually perform any conversions.
 *
 * utf16_len:
 * The length of the UTF-16 buffer, in 16-bit characters.
 * Ignored if utf16 is NULL.
 *
 * return:
 * If utf16 is NULL, the size of the required UTF-16 buffer,
 * in 16-bit characters.
 * Otherwise, the number of characters written to the utf8 buffer, in
 * 16-bit characters.
 *
 */
size_t utf8_to_utf16(utf8_t const *utf8, size_t utf8_len, utf16_t *utf16, size_t utf16_len);

/*
 * Reports whether a byte run is well-formed UTF-8.
 *
 * Strict: overlong encodings, surrogates and codepoints beyond the Unicode
 * maximum all count as invalid, since decode_utf8() rejects them.
 *
 * utf8:
 *     The byte run to inspect. Not NUL-terminated; utf8_len bytes are read.
 *
 * utf8_len:
 *     The number of bytes to inspect.
 *
 * Returns:
 *     true if every byte is part of a well-formed sequence.
 */
bool utf8_is_valid(utf8_t const *utf8, size_t utf8_len);

#endif // UTF8_H_INCLUDED
