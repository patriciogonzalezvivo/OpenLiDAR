/*
* Header File for the Telescope Control protocols for the Meade LX200
* Author:  John Kielkopf (kielkopf@louisville.edu)
* 
* Extracted from 
*   - https://github.com/jochym/indi-base/blob/master/libindi/obsolete/celestronprotocol.h
*   - https://github.com/jochym/indi-base/blob/master/libindi/obsolete/celestronprotocol.c
*
* This file contains header information used in common with xmtel.

    This library is free software; you can redistribute it and/or
    modify it under the terms of the GNU Lesser General Public
    License as published by the Free Software Foundation; either
    version 2.1 of the License, or (at your option) any later version.

    This library is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
    Lesser General Public License for more details.

    You should have received a copy of the GNU Lesser General Public
    License along with this library; if not, write to the Free Software
    Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301  USA
*
* 15 May 2003 -- Version 2.00
*
*/

#pragma once

/* Set this if a slew to the north sends the telescope south. */

#define REVERSE_NS 0 /* 1 for reverse; 0 for normal. */

/* The following parameters are used internally to set speed and direction. */
/* Do not change these values. */

#define SLEW   0
#define FIND   1
#define CENTER 2
#define GUIDE  3

#if REVERSE_NS > 0
#define NORTH 3
#define SOUTH 0
#else
#define NORTH 0
#define SOUTH 3
#endif

#define EAST 2
#define WEST 1

/* Slew speed defines */
#define SLEWRATE8 8 /* should be 8 degrees per second (not 16-inch) */
#define SLEWRATE4 4 /* should be 4 degrees per second */
#define SLEWRATE3 3 /* should be 3 degrees per second */
#define SLEWRATE2 2 /* should be 2 degrees per second */


#ifdef __cplusplus
extern "C" {
#endif

int ConnectTel(const char *port);
void DisconnectTel();
int CheckConnectTel();  // 0 if connection is OK, -1 otherwis

void SetRate(int newRate);
int StartSlew(int direction);
int StopSlew(int direction);
double GetRA();
double GetDec();
int SlewToCoords(double newRA, double newDec);
int SyncToCoords(double newRA, double newDec);
int CheckCoords(double desRA, double desDec, double tolRA, double tolDEC);
int isScopeSlewing();
int updateLocation(double lng, double lat);

void StopNSEW();

#ifdef __cplusplus
}
#endif
