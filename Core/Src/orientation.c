/*
 * orientation.c
 *
 * Orientation implementation
 *
 */

#include "orientation.h"
#include "app_config.h"
#include <math.h>
/*
* Toutes ces constantes sont de type float.
* Les operations restent dans le domaine float avec atan2f et sqrtf.
* GYRO_LSB_PER_DPS_F = 131.0f : sensibilite pour +/-250 deg/s.
* 0.98 + 0.02 = 1.00 : fusion gyro/accel du filtre complementaire.
*/
#define RAD_TO_DEG_F 		(57.295779513f)
#define GYRO_LSB_PER_DPS_F 	(131.0f)
#define FILTER_GYRO_F 		(0.98f)
#define FILTER_ACCEL_F 		(0.02f)
#define MAX_DELTA_MS 		(100U)

static bool accel_angles(const ImuSample *sample, float *roll_deg, float *pitch_deg)
{
	float x;
	float y;
	float z;
	/*
	* float : x, y, z et norme au carre vont dans le meme domaine
	* mathematique que sqrtf/atan2f. Le controle norm2<1 evite
	* le vecteur nul et donc un angle mal defini.
	*/
	float norm2;
	if ((sample == NULL) || (roll_deg == NULL) || (pitch_deg == NULL))
	{
		return false;
	}
	x = (float)sample->ax;
	y = (float)sample->ay;
	z = (float)sample->az;

	norm2 = (x * x) + (y * y) + (z * z);
	if (norm2 < 1.0f)
	{
		return false; /* Protect atan2(0,0). */
	}

	/*
	* atan2f renvoie un angle en radians ; multiplication par
	* RAD_TO_DEG_F pour fournir des degrés.
	*/
	*roll_deg = atan2f(y, z) * RAD_TO_DEG_F;
	*pitch_deg = atan2f(-x, sqrtf((y * y) + (z * z))) * RAD_TO_DEG_F;
	/*
	* isfinite : refuse NaN et infinis, problemes qui se propagent
	* facilement dans les calculs flottants ; To Be Done : verifier le traitement
	* au niveau application si false persiste.
	*/
	return isfinite(*roll_deg) && isfinite(*pitch_deg);
}

void Orientation_Reset(Orientation *ctx)
{
	if (ctx != NULL)
	{
		*ctx = (Orientation){0};
	}
}

void Orientation_Resync(Orientation *ctx)
{
	if (ctx != NULL)
	{
		ctx->tracking = false;
	}
}

bool Orientation_AddCalibration(Orientation *ctx, const ImuSample *sample)
{
	if ((ctx == NULL) || (sample == NULL)) {
		return false;
	}
	if (ctx->calibrated) {
		return true;
	}
	ctx->sum_gx += sample->gx;
	ctx->sum_gy += sample->gy;
	ctx->sum_gz += sample->gz;
	ctx->calibration_count++;

	if (ctx->calibration_count >= APP_CALIBRATION_SAMPLES)
	{
		const float denominator = (float)APP_CALIBRATION_SAMPLES * GYRO_LSB_PER_DPS_F;
		ctx->bias_gx_dps = (float)ctx->sum_gx / denominator;
		ctx->bias_gy_dps = (float)ctx->sum_gy / denominator;
		ctx->bias_gz_dps = (float)ctx->sum_gz / denominator;
		ctx->calibrated = true;
		ctx->tracking = false;
	}
	return ctx->calibrated;
}

bool Orientation_Update(Orientation *ctx, const ImuSample *sample, uint32_t now_ms)
{

	float roll_acc = 0.0f;
	float pitch_acc = 0.0f;
	float dt;

	uint32_t delta_ms;
	bool accel_ok;
	float gx_dps;
	float gy_dps;
	float gz_dps;
	if ((ctx == NULL) || (sample == NULL) || (!ctx->calibrated))
	{
		return false;
	}
	accel_ok = accel_angles(sample, &roll_acc, &pitch_acc);
	if (!ctx->tracking)
	{
		if (!accel_ok) {
			return false;
		}
		ctx->angles.roll_deg = roll_acc;
		ctx->angles.pitch_deg = pitch_acc;
		/* Do not reset relative yaw on RESUME. */
		ctx->last_ms = now_ms;
		ctx->tracking = true;
		return true;
	}

	delta_ms = (now_ms - ctx->last_ms);
	if (delta_ms == 0U) {
		return false;
	}
	ctx->last_ms = now_ms;
	if (delta_ms > MAX_DELTA_MS)
	{
		ctx->tracking = false;
		return false;
	}

	dt = (float)delta_ms / 1000.0f;

	gx_dps = ((float)sample->gx / GYRO_LSB_PER_DPS_F) - ctx->bias_gx_dps;
	gy_dps = ((float)sample->gy / GYRO_LSB_PER_DPS_F) - ctx->bias_gy_dps;
	gz_dps = ((float)sample->gz / GYRO_LSB_PER_DPS_F) - ctx->bias_gz_dps;
	if (accel_ok)
	{
		ctx->angles.roll_deg =
		FILTER_GYRO_F * (ctx->angles.roll_deg + (gx_dps * dt)) +
		FILTER_ACCEL_F * roll_acc;
		ctx->angles.pitch_deg =
		FILTER_GYRO_F * (ctx->angles.pitch_deg + (gy_dps * dt)) +
		FILTER_ACCEL_F * pitch_acc;
	}
	else
	{
		ctx->angles.roll_deg += gx_dps * dt;
		ctx->angles.pitch_deg += gy_dps * dt;
	}
	/*
	* Integration du yaw relatif sans magnetometre. La normalisation
	* sur [-180,180) suppose que les variations par echantillon sont
	* moderees (delta_ms limite et gyro +/-250 deg/s).
	*/
	ctx->angles.yaw_deg += gz_dps * dt;
	if (ctx->angles.yaw_deg >= 180.0f) { ctx->angles.yaw_deg -= 360.0f; }
	if (ctx->angles.yaw_deg < -180.0f) { ctx->angles.yaw_deg += 360.0f; }

	return isfinite(ctx->angles.roll_deg) &&
		isfinite(ctx->angles.pitch_deg) &&
		isfinite(ctx->angles.yaw_deg);
}

const OrientationAngles *Orientation_GetAngles(const Orientation *ctx)
{
	return (ctx == NULL) ? NULL : &ctx->angles;
}
