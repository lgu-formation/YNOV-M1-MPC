#include "telemetry.h"
#include "app_config.h"

#if TP_USB_CDC_ENABLED
#include "usb_device.h"
#include "usbd_cdc_if.h"
#include "usbd_cdc.h"
#include <stdio.h>
#include <stddef.h>
extern USBD_HandleTypeDef hUsbDeviceFS;
static uint32_t s_last_tx_ms = 0U;
static uint8_t s_tx[192] = {0};

void Telemetry_Tick(uint32_t now_ms, const ImuSample *sample,
                    const OrientationAngles *angles,
                    const RotationVector3 *a_reference)
{
    int n;
    USBD_CDC_HandleTypeDef *cdc;
    if ((sample == NULL) || (angles == NULL) || (a_reference == NULL)) {return;}
    if ((now_ms - s_last_tx_ms) < TP_TELEMETRY_PERIOD_MS) {return;}
    if ((hUsbDeviceFS.dev_state != USBD_STATE_CONFIGURED) ||
        (hUsbDeviceFS.pClassData == NULL)) {return;}
    cdc = (USBD_CDC_HandleTypeDef *)hUsbDeviceFS.pClassData;
    if (cdc->TxState != 0U) {return;}

    /* Protocole 12 champs, compatible avec imu_viewer.py */
    n = snprintf((char *)s_tx, sizeof(s_tx),
       "%d;%d;%d;%d;%d;%d;%.2f;%.2f;%.2f;%.2f;%.2f;%.2f\r\n",
       (int)sample->ax, (int)sample->ay, (int)sample->az,
       (int)sample->gx, (int)sample->gy, (int)sample->gz,
       (double)angles->roll_deg, (double)angles->pitch_deg,
       (double)angles->yaw_deg,
       (double)a_reference->x, (double)a_reference->y,
       (double)a_reference->z);
    if ((n > 0) && ((size_t)n < sizeof(s_tx)))
    {
        if (CDC_Transmit_FS(s_tx, (uint16_t)n) == USBD_OK)
        {
            s_last_tx_ms = now_ms;
        }
    }
}
#else
void Telemetry_Tick(uint32_t now_ms, const ImuSample *sample,
                    const OrientationAngles *angles,
                    const RotationVector3 *a_reference)
{
    (void)now_ms;
    (void)sample;
    (void)angles;
    (void)a_reference;
}
#endif
