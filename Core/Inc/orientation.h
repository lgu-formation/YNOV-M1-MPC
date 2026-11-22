/*
 * orientation.h
 *
 * Orientation interfaces
 *
 */

#ifndef INC_ORIENTATION_H_
#define INC_ORIENTATION_H_

#include <stdint.h>
#include <stdbool.h>
#include "imu.h"

/* Angles structure */
typedef struct {
	float roll_deg;
	float pitch_deg;
	float yaw_deg; // relative yaw only as no magnetometer
} OrientationAngles;

/* Orientation structure */
typedef struct {
	OrientationAngles angles;

	int32_t sum_gx;
	int32_t sum_gy;
	int32_t sum_gz;

	float bias_gx_dps;
	float bias_gy_dps;
	float bias_gz_dps;

	uint32_t calibration_count;
	uint32_t last_ms;
	bool tracking;
	bool calibrated;
} Orientation;

void Orientation_Reset(Orientation *ctx);

void Orientation_Resync(Orientation *ctx);

bool Orientation_AddCalibration(Orientation *ctx, const ImuSample *sample);

bool Orientation_Update(Orientation *ctx, const ImuSample *sample, uint32_t now_ms);

const OrientationAngles *Orientation_GetAngles(const Orientation *ctx);


#endif /* INC_ORIENTATION_H_ */
