/****************************************************************************
 *
 *   Copyright (c) 2014 MAV GEO Library (MAVGEO). All rights reserved.
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
 * 3. Neither the name MAVGEO nor the names of its contributors may be
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

/**
* @file geo_mag_declination.c
*
* Calculation / lookup table for earth magnetic field declination.
*
* Lookup table from Scott Ferguson <scottfromscott@gmail.com>
*
* XXX Lookup table currently too coarse in resolution (only full degrees)
* and lat/lon res - needs extension medium term.
*
*/

#include <cstdint>
#include <cmath>
#include "geo_mag_declination.h"

/** set this always to the sampling in degrees for the table below */
#define SAMPLING_RES		10.0f
#define SAMPLING_MIN_LAT	-60.0f
#define SAMPLING_MAX_LAT	60.0f
#define SAMPLING_MIN_LON	-180.0f
#define SAMPLING_MAX_LON	180.0f

static const int8_t declination_table[13][37] = \
{
	{ 47,46,45,43,42,41,39,37,33,29,23,16,10,4,-1,-6,-10,-15,-20,-27,-34,-42,-49,-56,-62,-67,-72,-74,-75,-73,-61,-22,26,42,47,48,47 },
	{ 31,31,31,30,30,30,30,29,27,24,18,11,3,-4,-9,-13,-15,-18,-21,-27,-33,-40,-47,-52,-56,-57,-56,-52,-44,-30,-14,2,14,22,27,30,31 },
	{ 22,23,23,23,22,22,22,23,22,19,13,5,-4,-12,-17,-20,-22,-22,-23,-25,-30,-36,-41,-45,-46,-44,-39,-31,-21,-11,-3,4,10,15,19,21,22 },
	{ 17,17,17,18,17,17,17,17,16,13,8,-1,-10,-18,-22,-25,-26,-25,-22,-20,-21,-25,-29,-32,-31,-28,-23,-16,-9,-3,0,4,7,11,14,16,17 },
	{ 13,13,14,14,14,13,13,12,11,9,3,-5,-14,-20,-24,-25,-24,-21,-17,-12,-9,-11,-14,-17,-18,-16,-12,-8,-3,-0,1,3,6,8,11,12,13 },
	{ 11,11,11,11,11,10,10,10,9,6,-0,-8,-15,-21,-23,-22,-19,-15,-10,-5,-2,-2,-4,-7,-9,-8,-7,-4,-1,1,1,2,4,7,9,10,11 },
	{ 10,9,9,9,9,9,9,8,7,3,-3,-10,-16,-20,-20,-18,-14,-9,-5,-2,1,2,0,-2,-4,-4,-3,-2,-0,0,0,1,3,5,7,9,10 },
	{ 9,9,9,9,9,9,9,8,6,1,-4,-11,-16,-18,-17,-14,-10,-5,-2,-0,2,3,2,0,-1,-2,-2,-1,-0,-1,-1,-1,1,3,6,8,9 },
	{ 8,9,9,10,10,10,10,8,5,0,-6,-12,-15,-16,-15,-11,-7,-4,-1,1,3,4,3,2,1,0,-0,-0,-1,-2,-3,-4,-2,0,3,6,8 },
	{ 7,9,10,11,12,12,12,9,5,-1,-7,-13,-15,-15,-13,-10,-6,-3,0,2,3,4,4,4,3,2,1,0,-1,-3,-5,-6,-6,-3,0,4,7 },
	{ 5,8,11,13,14,15,14,11,5,-2,-9,-15,-17,-16,-13,-10,-6,-3,0,3,4,5,6,6,6,5,4,2,-1,-5,-8,-9,-9,-6,-3,1,5 },
	{ 3,8,11,15,17,17,16,12,5,-4,-12,-18,-19,-18,-16,-12,-8,-4,-0,3,5,7,9,10,10,9,7,4,-1,-6,-10,-12,-12,-9,-5,-1,3 },
	{ 3,8,12,16,19,20,18,13,4,-8,-18,-24,-25,-23,-20,-16,-11,-6,-1,3,7,11,14,16,17,17,14,8,-0,-8,-13,-15,-14,-11,-7,-2,3 },	
};

static float get_lookup_table_val(unsigned lat, unsigned lon);

float get_mag_declination(float lat_rad, float lon_rad)
{
	float lat = lat_rad / M_PI * 180.0f;
	float lon = lon_rad / M_PI * 180.0f;

	/*
	 * If the values exceed valid ranges, return zero as default
	 * as we have no way of knowing what the closest real value
	 * would be.
	 */
	if (lat < -90.0f || lat > 90.0f ||
	    lon < -180.0f || lon > 180.0f) {
		return 0.0f;
	}

	/* round down to nearest sampling resolution */
	int min_lat = (int)(lat / SAMPLING_RES) * SAMPLING_RES;
	int min_lon = (int)(lon / SAMPLING_RES) * SAMPLING_RES;

	/* for the rare case of hitting the bounds exactly
	 * the rounding logic wouldn't fit, so enforce it.
	 */

	/* limit to table bounds - required for maxima even when table spans full globe range */
	if (lat <= SAMPLING_MIN_LAT) {
		min_lat = SAMPLING_MIN_LAT;
	}

	if (lat >= SAMPLING_MAX_LAT) {
		min_lat = (int)(lat / SAMPLING_RES) * SAMPLING_RES - SAMPLING_RES;
	}

	if (lon <= SAMPLING_MIN_LON) {
		min_lon = SAMPLING_MIN_LON;
	}

	if (lon >= SAMPLING_MAX_LON) {
		min_lon = (int)(lon / SAMPLING_RES) * SAMPLING_RES - SAMPLING_RES;
	}

	/* find index of nearest low sampling point */
	unsigned min_lat_index = (-(SAMPLING_MIN_LAT) + min_lat)  / SAMPLING_RES;
	unsigned min_lon_index = (-(SAMPLING_MIN_LON) + min_lon) / SAMPLING_RES;

	float declination_sw = get_lookup_table_val(min_lat_index, min_lon_index);
	float declination_se = get_lookup_table_val(min_lat_index, min_lon_index + 1);
	float declination_ne = get_lookup_table_val(min_lat_index + 1, min_lon_index + 1);
	float declination_nw = get_lookup_table_val(min_lat_index + 1, min_lon_index);

	/* perform bilinear interpolation on the four grid corners */

	float declination_min = ((lon - min_lon) / SAMPLING_RES) * (declination_se - declination_sw) + declination_sw;
	float declination_max = ((lon - min_lon) / SAMPLING_RES) * (declination_ne - declination_nw) + declination_nw;

	float declination_ret = ((lat - min_lat) / SAMPLING_RES) * (declination_max - declination_min) + declination_min;

	return declination_ret / 180.0f * M_PI;
}

float get_lookup_table_val(unsigned lat_index, unsigned lon_index)
{
	return declination_table[lat_index][lon_index];
}