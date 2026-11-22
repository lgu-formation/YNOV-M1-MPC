#ifndef TELEMETRY_H
#define TELEMETRY_H

#include <stdint.h>
#include "imu.h"
#include "orientation.h"
#include "rotation.h"

void Telemetry_Tick(uint32_t now_ms, const ImuSample *sample,
                    const OrientationAngles *angles,
                    const RotationVector3 *a_reference);


#endif
