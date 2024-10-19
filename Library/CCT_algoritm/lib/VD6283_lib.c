/*
* Copyright (c) 2024, STMicroelectronics - All Rights Reserved
*
* This file is part "VD6283 API" and is licensed under the terms of
* 'BSD 3-clause "New" or "Revised" License'.
*
********************************************************************************
*
* License terms BSD 3-clause "New" or "Revised" License.
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions are met
*
* 1. Redistributions of source code must retain the above copyright notice, this
* list of conditions and the following disclaimer.
*
* 2. Redistributions in binary form must reproduce the above copyright notice,
* this list of conditions and the following disclaimer in the documentation
* and/or other materials provided with the distribution.
*
* 3. Neither the name of the copyright holder nor the names of its contributors
* may be used to endorse or promote products derived from this software
* without specific prior written permission.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
* AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
* IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
* DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
* FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
* DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
* SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
* CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
* OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
* OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*
*
********************************************************************************
*
*/

#include "STALS_lib.h"
#include "STALS_lib_platform.h"
#include <math.h>

#ifndef __KERNEL__
#include <assert.h>
#else
#include <linux/bug.h>
#endif

#include "STALS_compat.h"

#ifndef ARRAY_SIZE
#define ARRAY_SIZE(a)				(sizeof(a) / sizeof(a[0]))
#endif

#ifndef MIN
#define MIN(a, b)	((a) < (b) ? (a) : (b))
#endif
#ifndef MAX
#define MAX(a, b)	((a) > (b) ? (a) : (b))
#endif
#ifndef M_PI
#define M_PI					3.14159265358979323846
#endif


#define CHECK_NULL_PTR(a) \
do { \
	if (!a) \
		return STALS_ERROR_INVALID_PARAMS; \
} while (0)

#define CHECK_NEGATIVE_AND_ZERO(a) \
do { \
	if (a <= 0) \
		return STALS_ERROR_INVALID_PARAMS; \
} while (0)




STALS_ErrCode_t STALS_LIB_normalize_als_for_xyz(struct STALS_Als_t *als_meas, uint32_t expo_us, double meas_norm[STALS_ALS_MAX_CHANNELS]){
	STALS_ErrCode_t res = STALS_NO_ERROR;
	uint8_t c;
	double expo_scale = 100800.0 / expo_us;
	CHECK_NULL_PTR(als_meas);
	CHECK_NULL_PTR(meas_norm);

	for (c = 0; c < STALS_ALS_MAX_CHANNELS; c++) {
 		if (!(als_meas->Channels & (1 << c))){
			meas_norm[c] = 0;
		}
		else{
			meas_norm[c] = expo_scale * (als_meas->CountValue[c]/256) / (als_meas->Gains[c] /256);
		}
	}
	return res;
}


STALS_ErrCode_t STALS_LIB_convert_to_xyz(double *matrix,
										double *parametres,
                                        uint16_t N,
										double *X,
										double *Y,
										double *Z)
{
	STALS_ErrCode_t res = STALS_NO_ERROR;
	CHECK_NULL_PTR(matrix);
	CHECK_NULL_PTR(parametres);
	CHECK_NULL_PTR(X);
	CHECK_NULL_PTR(Y);
	CHECK_NULL_PTR(Z);
	*X = 0;
	*Y = 0;
	*Z = 0;
	for (int i = 0; i < N; i++) {
        *X += matrix[0*N + i] * parametres[i];
        *Y += matrix[1*N + i] * parametres[i];
        *Z += matrix[2*N + i] * parametres[i];
	}

	return res;
}


STALS_ErrCode_t STALS_LIB_compute_cct(double X,
									  double Y,
									  double Z,
									  double *cct
									 )
{
	STALS_ErrCode_t res = STALS_NO_ERROR;
	CHECK_NULL_PTR(cct);

	double xyNormFactor;
	double m_xNormCoeff, m_yNormCoeff;
	double nCoeff;

	xyNormFactor = (X + Y + Z);
	// avoid zero division error
	if(xyNormFactor == 0){
		*cct = 0;
	}
	else{
		m_xNormCoeff = X / xyNormFactor;
		m_yNormCoeff = Y / xyNormFactor;
		nCoeff = (m_xNormCoeff - 0.3320) / (0.1858 - m_yNormCoeff);
		*cct = (449 * pow(nCoeff, 3) + 3525 * pow(nCoeff, 2) + 6823.3 * nCoeff + 5520.33);
	}
	return res;
}


STALS_ErrCode_t STALS_LIB_compute_lux(double X, double Y, double Z, double *lux)
{
	STALS_ErrCode_t res = STALS_NO_ERROR;
	CHECK_NULL_PTR(lux);
	(void)X;
	(void)Z;
	*lux = Y;
	return res;
}


STALS_ErrCode_t STALS_LIB_flk_autogain(void *pHandle, enum STALS_Channel_Id_t ChannelId, uint32_t timeoutMs, uint16_t *pAppliedGain)
{
	/* duplicate 0x42AB to avoid 100x and keep multiples of 2 for array size */
	const uint16_t Gains[] = {
		0x42AB, 0x42AB, 0x3200, 0x2154, 0x1900, 0x10AB, 0x0A00, 0x0723,
		0x0500, 0x0354, 0x0280, 0x01AB, 0x0140, 0x0100, 0x00D4, 0x00B5
	};
	const unsigned int sat_limit = 2;
	int idx = 7;
	STALS_ErrCode_t res;
	uint32_t saturation;
	int i;
	unsigned int j;

	CHECK_NULL_PTR(pAppliedGain);
	/* limit timeout */
	timeoutMs = timeoutMs == 0 ? 1 : timeoutMs;
	timeoutMs = timeoutMs >= 100 ? 100 : timeoutMs;

	for (i = 3; i >= 0; i--) {
		res = STALS_SetGain(pHandle, ChannelId, Gains[idx], pAppliedGain);
		if (res)
			return res;
		res = STALS_Start(pHandle, STALS_MODE_FLICKER, ChannelId);
		if (res)
			return res;

		/* read saturation value each ms so we can exit early if saturation detected */
		for (j = 0; j < timeoutMs; j++) {
			res = STALS_lib_delay(1);
			if (res)
				return res;
			res = STALS_GetControl(pHandle, STALS_SATURATION_VALUE, &saturation);
			if (res)
				return res;
			if (saturation > sat_limit)
				break;
		}

		res = STALS_Stop(pHandle, STALS_MODE_FLICKER);
		if (res)
			return res;
		/* update index to next value */
		if (i)
			idx += saturation > sat_limit ? 1 << (i - 1) : -(1 << (i - 1));
		else if (saturation > sat_limit)
			idx++;
	}
	if (idx > 15)
		idx = 15;

	return STALS_SetGain(pHandle, ChannelId, Gains[idx], pAppliedGain);
}

