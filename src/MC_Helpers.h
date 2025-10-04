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
 
#ifndef MC_HELPERS_H
#define MC_HELPERS_H

/*-----------------------------------------
 * define a type for a function pointer either having
 * no parameter in the call or having a pointer object as an
 * argument which can be used to hand over a received message
 * 
 * -------------------------------------------------------*/

#include "Arduino.h"
#include <stdint.h>

//define a void function pointer which takes an argument
typedef void* (*ifunction_pointer_t)(void *op, int index);

struct ifunction_holder {
    ifunction_pointer_t callback;
    void *op;
};

typedef void* (*vfunction_pointer_t)(void *op);

struct vfunction_holder {
    vfunction_pointer_t callback;
    void *op;
};

typedef void* (*pfunction_pointer_t)(void *op, void *p);

struct pfunction_holder {
    pfunction_pointer_t callback;
    void *op;
};

//a function pointer suitable to hand over the information of a MQTT topic
typedef void* (*tfunction_pointer_t)(void *op, void *t, uint32_t l);

struct tfunction_holder {
    tfunction_pointer_t callback;
    void *op;
};

#endif
