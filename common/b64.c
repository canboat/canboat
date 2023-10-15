/*

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

#include "common.h"

#define B64_CHUNKSIZE 6

static const char B64EncodeTable[65]
    = {'A', 'B', 'C', 'D', 'E', 'F', 'G', 'H', 'I', 'J', 'K', 'L', 'M', 'N', 'O', 'P', 'Q', 'R', 'S', 'T', 'U', 'V',
       'W', 'X', 'Y', 'Z', 'a', 'b', 'c', 'd', 'e', 'f', 'g', 'h', 'i', 'j', 'k', 'l', 'm', 'n', 'o', 'p', 'q', 'r',
       's', 't', 'u', 'v', 'w', 'x', 'y', 'z', '0', '1', '2', '3', '4', '5', '6', '7', '8', '9', '+', '/', '='};

static int B64DecodeTable[256] = {
    -1, -1, -1, -1, -1, -1, -1, -1, -1, -2, -2, -1, -1, -2, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,
    -2, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, 62, -1, -1, -1, 63, 52, 53, 54, 55, 56, 57, 58, 59, 60, 61, -1, -1, -1, 64, -1, -1,
    -1, 0,  1,  2,  3,  4,  5,  6,  7,  8,  9,  10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24, 25, -1, -1, -1, -1, -1,
    -1, 26, 27, 28, 29, 30, 31, 32, 33, 34, 35, 36, 37, 38, 39, 40, 41, 42, 43, 44, 45, 46, 47, 48, 49, 50, 51, -1, -1, -1, -1, -1,
    -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,
    -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,
    -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,
    -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1};

static const char AISEncodeTable[65]
    = {'0', '1', '2', '3', '4', '5', '6', '7', '8', '9', ':', ';', '<', '=', '>', '?', '@', 'A', 'B', 'C', 'D', 'E',
       'F', 'G', 'H', 'I', 'J', 'K', 'L', 'M', 'N', 'O', 'P', 'Q', 'R', 'S', 'T', 'U', 'V', 'W', '`', 'a', 'b', 'c',
       'd', 'e', 'f', 'g', 'h', 'i', 'j', 'k', 'l', 'm', 'n', 'o', 'p', 'q', 'r', 's', 't', 'u', 'v', 'w'};

static int AISDecodeTable[256] = {
    -1, -1, -1, -1, -1, -1, -1, -1, -1, -2, -2, -1, -1, -2, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,
    -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, 0,  1,  2,  3,  4,  5,  6,  7,  8,  9,  10, 11, 12, 13, 14, 15,
    16, 17, 18, 19, 20, 21, 22, 23, 24, 25, 26, 27, 28, 29, 30, 31, 32, 33, 34, 35, 36, 37, 38, 39, -1, -1, -1, -1, -1, -1, -1, -1,
    40, 41, 42, 43, 44, 45, 46, 47, 48, 49, 50, 51, 52, 53, 54, 55, 56, 57, 58, 59, 60, 61, 62, 63, -1, -1, -1, -1, -1, -1, -1, -1,
    -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,
    -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,
    -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,
    -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1};

/**
 * B64 encode data (StringBuffer version, recommended)
 *
 * @param sb       Base64 encoded output
 * @param data     binary input data
 * @param len      length of 'data'
 */
void sbAppendEncodeBase64(StringBuffer *sb, const uint8_t *data, size_t len, enum Base64Encoding encoding)
{
  /*
   * Since every 3 input characters are translated into 4 output characters
   * following buffer is big enough to translate B64_CHUNKSIZE
   * input characters (including boundary effects)
   */
  char           otemp[((B64_CHUNKSIZE + 2) / 3) * 4 + 3];
  char          *optr;
  size_t         count;
  const uint8_t *in = data;
  unsigned char  u1, u2, u3;
  const char    *encodeTable;

  switch (encoding)
  {
    case BASE64_AIS:
      encodeTable = AISEncodeTable;
      break;
    case BASE64_RFC:
    default:
      encodeTable = B64EncodeTable;
      break;
  }

  sbEnsureCapacity(sb, len * 4 / 3 + 8);

  while (len > 0)
  {
    count = CB_MIN(B64_CHUNKSIZE, len);

    len -= count;

    optr = otemp;
    while (count >= 3)
    {
      u1      = *(in++);
      u2      = *(in++);
      u3      = *(in++);
      *optr++ = encodeTable[u1 >> 2];
      *optr++ = encodeTable[((u1 & 0x03) << 4) | (u2 >> 4)];
      *optr++ = encodeTable[((u2 & 0x0f) << 2) | (u3 >> 6)];
      *optr++ = encodeTable[u3 & 0x3f];
      count -= 3;
    }

    if (count)
    {
      if (count == 2)
      {
        u1      = *(in++);
        u2      = *(in++);
        *optr++ = encodeTable[u1 >> 2];
        *optr++ = encodeTable[(unsigned) (((u1 & 0x03) << 4) | ((u2 & 0xf0) >> 4))];
        *optr++ = encodeTable[(unsigned) ((u2 & 0x0f) << 2)];
        if (encoding == BASE64_AIS)
        {
          *optr++ = encodeTable[64];
        }
      }
      else
      {
        u1      = *(in++);
        *optr++ = encodeTable[u1 >> 2];
        *optr++ = encodeTable[(unsigned) ((u1 & 0x03) << 4)];
        if (encoding == BASE64_AIS)
        {
          *optr++ = encodeTable[64];
          *optr++ = encodeTable[64];
        }
      }
    }
    *optr = 0;

    sbAppendString(sb, otemp);
  }
}

void sbAppendDecodeBase64(StringBuffer *sb, const char *data, size_t len, enum Base64Encoding encoding)
{
  const uint8_t *s    = (const uint8_t *) data;
  const uint8_t *end  = s + len;
  uint8_t       *d    = 0;
  uint32_t       n    = 0;
  int            iter = 0;
  const int     *decodeTable;

  switch (encoding)
  {
    case BASE64_AIS:
      decodeTable = AISDecodeTable;
      break;
    case BASE64_RFC:
    default:
      decodeTable = B64DecodeTable;
      break;
  }

  sbEnsureCapacity(sb, len * 3 / 4 + 8 + sbGetLength(sb));
  d = (uint8_t *) sbGet(sb) + sbGetLength(sb);

  while (s < end)
  {
    int c = decodeTable[*s++];

    switch (c)
    {
      case -2: // Whitespace
        continue;
      case -1: // Invalid
        break;
      case 64: // = pad character, end of data
        s = end;
        continue; // instead of double break
      default:
        n = n << 6 | c; // Add 6 more bits to n

        iter++;
        if (iter == 4) // Every 4 characters we have 24 bits of output data
        {
          *d++ = (uint8_t) (n >> 16);
          *d++ = (uint8_t) (n >> 8);
          *d++ = (uint8_t) (n);
          n    = 0;
          iter = 0;
        }
    }
  }

  // Handle any remainder
  if (iter == 3)
  {
    *d++ = (uint8_t) (n >> 10);
    *d++ = (uint8_t) (n >> 2);
  }
  else if (iter == 2)
  {
    *d++ = (uint8_t) (n >> 4);
  }

  sb->len = d - (uint8_t *) sbGet(sb);
}
