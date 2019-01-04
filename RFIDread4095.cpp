/****************************************************************************
 *
 *   Copyright (c) 2017 Jay Wilhelm. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name ETAGRFIDArduino nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/
/*
 * ManchesterDecoder.h
 *
 * Created: 11/14/2017 7:26:06 PM
 *  Author: Jay Wilhelm jwilhelm@ohio.edu
 *  
 *  This is a header file
 *  This header file establishes variables, definitions, and data structures
 *  that are used in the functions contained in an associated .cpp file.
 *  Together, the header file and cpp file comprise a library, or a collection
 *  of functions that can be called in a sketch.
 */ 

/* Obtaining data output from an RFID tag.
 * 
 * Tags that use the EM4100 communication protocol send out a stream of data that 
 * begins with nine 1s in a row. Then you get a sequence of data chucks that consist 
 * four bits of data followed by an odd/even parity bit. If the data bits sum to an
 * odd number then the parity bit is 1, otherwise it is 0. There are ten of these
 * five-bit chunks followed by a four more partiy bits that allow for an odd/even
 * parity check on each "column" of the data stream. Then there is a stop bit 
 * (always 0) For example you could arrange a data stream as follows:
 * 
 *    1 1 1 1 1 1 1 1 1 1    (nine 1s to indicate the beginning of a data stream)
 *                           (these 1s are not stored)
 *    
 *    0 1 0 0   1            (four data bits and a parity bit)
 *    0 0 1 1   0            (four data bits and a parity bit)
 *    0 1 0 1   0            (four data bits and a parity bit)
 *    0 0 0 0   0            (four data bits and a parity bit)
 *    0 0 1 0   1            (four data bits and a parity bit)
 *    1 0 1 0   0            (four data bits and a parity bit)
 *    1 1 1 1   0            (four data bits and a parity bit)
 *    0 1 1 1   1            (four data bits and a parity bit)
 *    0 1 0 1   0            (four data bits and a parity bit)
 *    1 1 0 1   1            (four data bits and a parity bit)
 *    
 *    1 0 1 0   0            (four parity bits and a stop bit)
 * 
 * This data scheme allows for error checking within the data stream. If the parity bits
 * don't match up with the data bits, there there has been a transmission error
 * 
 * The code below creates a data structure that allows the RFID data stream to be stored
 * and checked efficiently. It consists of nibbles (partial bytes) of five bits (four data
 * bits and a parity bit) arranged one after another.
 */

#include <Arduino.h>              //Enable all the basic arduino functions
#include "RFIDread4095.h"    //Reference back to the header file that defines variable for this set of functions 

