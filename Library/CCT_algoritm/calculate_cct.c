#include <stdint.h>
#include <inttypes.h>
#include <unistd.h>
#include <assert.h>
#include <stdio.h>

#include "sensible_sensors.h"
#include "STALS.h"
#include "STALS_lib.h"

#define MAX_N 3 // or any other maximum value for N

/**
* @brief  Calculate cct
* @param  cct - pointer to write cct result
* @param  red, green, blue - values RGB
* @retval STALS_ErrCode_t value.
*/
static STALS_ErrCode_t calculate_cct(uint16_t *cct, uint32_t red, uint32_t green, uint32_t blue)
{
	STALS_ErrCode_t res = STALS_NO_ERROR;

	const double matrix[3][3] = {
		{-0.067849073, 0.708704342, -0.50282272},
		{-0.151549294, 0.714781775, -0.340903875},
		{-0.306018955, 0.338182367, 0.476947762}
	};
	double X = 0;
	double Y = 0;
	double Z = 0;
	double cctValue = 0;
	double parameters[3];
	uint16_t N=3;
	double matrix_to_use[3*MAX_N];

	for (int i = 0; i < N; i++) {
        matrix_to_use[0*N + i] = matrix[0][i];
        matrix_to_use[1*N + i] = matrix[1][i];
        matrix_to_use[2*N + i] = matrix[2][i];
	}

	parameters[0] = red;
	parameters[1] = green;
	parameters[2] = blue;

	res = STALS_LIB_convert_to_xyz(matrix_to_use,
								   parameters,
								   N,
								   &X,&Y,&Z);
	if(res != STALS_NO_ERROR) return res;

	res = STALS_LIB_compute_cct(X,Y,Z, &cctValue);
	if(res != STALS_NO_ERROR) return res;

	*cct = (uint16_t)cctValue;

	return res;
}

/**
* @brief  Get value cct
* @retval cct value.
*/
uint16_t get_cct(void)
{
	uint32_t red = 0;
	uint32_t green = 0;
	uint32_t blue = 0;
	uint16_t cct = 0;

	SensorsReadRGB(&red, &green, &blue);

	if (STALS_NO_ERROR != calculate_cct(&cct, red, green, blue)) {
		cct = 0;
	}

	return cct;
}
