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
#ifndef __STALS_LIB__
#define __STALS_LIB__ 1

#include "STALS.h"


/**
 * This is a platform specific function that will be called by the library to wait for a given amount of time.
 * This function is mandatory and must be implemented by the user.
 *
 * @param delayMs                          Delay in ms
 *
 * @note This function is mandatory and must be implemented by the user.
 */
STALS_ErrCode_t STALS_lib_delay(uint32_t delay_ms);



/**
 * @brief Normalize ALS measurements for XYZ coordinates
 *
 * This function normalizes the ALS measurements for the XYZ coordinates based on the given exposure time.
 *
 *
 * @param als_meas Pointer to an STALS_Als_t structure containing the ALS measurements
 * @param expo_us Unsigned 32-bit integer representing the exposure time in microseconds
 * @param meas_norm Double array of size STALS_ALS_MAX_CHANNELS to store the normalized measurements
 *
 * @note The gain used is encoded in 8.8 fixed-point format and the data is still encoded in 16.8 format.
 *
 * @return STALS_ErrCode_t error code indicating the success or failure of the function
 * \retval  STALS_NO_ERROR                 Success
 * \retval  STALS_ERROR_INVALID_PARAMS     At least one of the provided parameters to the function is invalid
 */
STALS_ErrCode_t STALS_LIB_normalize_als_for_xyz(struct STALS_Als_t *als_meas, uint32_t expo_us, double meas_norm[STALS_ALS_MAX_CHANNELS]);


/**
 * This function will convert the input parameters to XYZ from
 * @param cct_matrix                       The array that contains the Nx3 matrix in row inline
 * @param parametres                       The array of input parameters to multiply the matrix by.
 * @param N                                The number of columns in the matrix and the length of the parametres array.
 * @param X                                Pointer to the X value
 * @param Y                                Pointer to the Y value
 * @param Z                                Pointer to the Z value
 *
 * @note This function doesn't normalize measure
 *
 * @return STALS_ErrCode_t error code indicating the success or failure of the function
 * \retval  STALS_NO_ERROR                 Success
 * \retval  STALS_ERROR_INVALID_PARAMS     At least one of the provided parameters to the function is invalid
*/

STALS_ErrCode_t STALS_LIB_convert_to_xyz(double *matrix,
										double *parametres,
                                        uint16_t N,
										double *X,
										double *Y,
										double *Z);


/**
 * @brief Compute the correlated color temperature (CCT) using the McCamy method
 *
 * This function computes the correlated color temperature (CCT) using the McCamy method based on the given XYZ coordinates.
 *
 * @param X Double representing the X coordinate
 * @param Y Double representing the Y coordinate
 * @param Z Double representing the Z coordinate
 * @param cct Pointer to a double to store the computed CCT

 * @note This function uses the McCamy method to compute the CCT.
 *
 * @return STALS_ErrCode_t error code indicating the success or failure of the function
 */
STALS_ErrCode_t STALS_LIB_compute_cct(double X, double Y, double Z, double *cct);



/**
 * @brief Compute the illuminance in lux based on the XYZ coordinates
 *
 * This function computes the illuminance in lux based on the given XYZ coordinates.
 *
 * @param X Double representing the X coordinate
 * @param Y Double representing the Y coordinate
 * @param Z Double representing the Z coordinate
 * @param lux Pointer to a double to store the computed illuminance in lux
 *
 * @note The XYZ coordinates must be obtained from normalized ALS measurements before calling this function.
 *
 * @return STALS_ErrCode_t error code indicating the success or failure of the function
 */
STALS_ErrCode_t STALS_LIB_compute_lux(double X, double Y, double Z, double *lux);






/**
 * This function will find and applied best gain setting for flicker detection. For that it will start some flicker
 * measures with different gains. In case of success compute gain will be applied.
 *
 * @param pHandle                          Rainbow instance as fill in by STALS_Init()
 * @param ChannelId                        Channel Id to use.
 * @param timeoutMs                        Max time to spent for one measure. A good value to use here is the period of the
 *                                         minimal expected frequency. Minimal value is 1ms and maximal is 100ms. If you provided
 *                                         value out of this range it will be silently clipped.
 * @param pAppliedGain                     Pointer in which the value of the actual gain applied in the device is returned. Value in 8.8 fixed point unit
 *
 * @note This function required STALS_lib_delay() platform support.
 *
 * \retval  STALS_NO_ERROR                 Success
 * \retval  STALS_ERROR_INVALID_PARAMS     At least one of the provided parameters to the function is invalid
 */
STALS_ErrCode_t STALS_LIB_flk_autogain(void *pHandle, enum STALS_Channel_Id_t ChannelId, uint32_t timeoutMs, uint16_t *pAppliedGain);

#endif
