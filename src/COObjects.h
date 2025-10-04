/*
 * Copyright (c) 2025 by Andreas Wagener (AW)
 * CANopen central device library for Arduino UNO R4.
 *
 * This file is free software; you can redistribute it and/or modify
 * it under the terms of either the GNU General Public License version 2
 * or the GNU Lesser General Public License version 2.1, both as
 * published by the Free Software Foundation.
 
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.

 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.

 */

#ifndef COObjects_H
#define COObjects_H

/*----------------------------------------------------------------------
 * some sample types for Objects of a CANopen OD
 *
 * 2025-01-08 AW
 *---------------------------------------------------------------------*/
 
#include <stdint.h>

typedef struct ODEntry {
  uint16_t Idx;
  uint8_t SubIdx;
  void *Value;
  uint32_t len;
} ODEntry;

typedef struct ODEntry08 {
  uint16_t Idx;
  uint8_t SubIdx;
  uint8_t *Value;
  uint32_t len;
} ODEntry08;

typedef struct ODEntry16 {
  uint16_t Idx;
  uint8_t SubIdx;
  uint16_t *Value;
  uint32_t len;
} ODEntry16;

typedef struct ODEntry32 {
  uint16_t Idx;
  uint8_t SubIdx;
  uint32_t *Value;
  uint32_t len;
} ODEntry32;

typedef struct ODEntryString {
  uint16_t Idx;
  uint8_t SubIdx;
  char *Value;
  uint32_t len;
} ODEntryString;

#endif