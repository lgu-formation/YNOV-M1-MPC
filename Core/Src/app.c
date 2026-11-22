/*
 * app.c implementation
 *
 * Application management
 *
 */

#include "app.h"
#include "app_config.h"
#include "imu.h"
#include "orientation.h"
#include "oled.h"
#include "rotation.h"
#include "telemetry.h"

#include <stddef.h>
#include <stdbool.h>
#include <stdio.h>  /* snprintf() pour affichage des mesures */

#define LED_GREEN_PORT 			GPIOD
#define LED_GREEN_PIN 			GPIO_PIN_12
#define LED_RED_PORT 			GPIOD
#define LED_RED_PIN 			GPIO_PIN_14
#define BUTTON_PIN 				GPIO_PIN_0
#define MPU_INT_PIN 			GPIO_PIN_2
#define APP_LINE_CAPACITY 		(22U)

/* PD13 : LED orange LD3, utilisable comme marqueur de temps pour EXTI2. */
#define TP_TRACE_PORT           GPIOD
#define TP_TRACE_PIN            GPIO_PIN_13
#define TP_DEG_TO_RAD_F         (0.017453292519943295f)


typedef enum
{
	APP_ERR_NONE = 0,
	APP_ERR_IMU_INIT,
	APP_ERR_OLED,
	APP_ERR_I2C,
	APP_ERR_NO_IRQ,
	APP_ERR_ANGLES
} AppError;

/* Application context structure */
typedef struct {
	/*
	* TYPE : Pointeur HAL vers bus I2C.
	*/
	I2C_HandleTypeDef *i2c;
	/*
	* TYPE : Structure angle, biais et calibration.
	*/
	Orientation orientation;

	/* Dernière mesure */
	ImuSample last_sample;

	/* Acceleration transformée dans le repere fixe. */
	RotationVector3 a_reference;
	/*
	* TYPE : Etat du système : INIT/CALIBRATION/RUN/PAUSE/ERROR (enum).
	*/
	AppState state;

	/*
	* TYPE : Enum : code de diagnostic erreur.
	*/
	AppError error;
	/*
	* TYPE : uint32_t : horodatages interruptions.
	*/
	uint32_t last_irq_ms;
	/*
	* TYPE : uint32_t : affichage OLED.
	*/
	uint32_t last_oled_ms;
	/*
	* TYPE : uint32_t : clignotement LED.
	*/
	uint32_t last_led_ms;
	/*
	* TYPE : uint32_t : horodatage pour l'anti-rebond.
	*/
	uint32_t last_button_ms;
	/*
	* TYPE : uint32_t : instant de la dernière mesure acceptèe.
	*/
	uint32_t last_imu_ms;
	/*
	* TYPE : uint32_t : compteur sans signe, pour affiche potentiel.
	*/
	uint32_t n_samples;
	/*
	* TYPE : uint32_t : compte des événements consommés (pas toutes les requests).
	*/
	uint32_t n_irqs;
	/*
	* TYPE : uint8_t : erreurs consécutives plafonnées à UINT8_MAX.
	*/
	uint8_t failure_count;
	/*
	* TYPE : uint8_t : index de page OLED (0..7).
	*/
	uint8_t next_page;
	/*
	* TYPE : bool : etat binaire de l'anti-rebond initial.
	*/
	bool button_ever_pressed;
	/*
	* TYPE : bool : disponibilite de l afficheur
	*/
	bool oled_ok;

} AppContext;

static AppContext s_app = {0};

static volatile uint32_t s_imu_ready = 0U;
static volatile uint32_t s_button_requested = 0U;

static bool take_event(volatile uint32_t *flag)
{
	const uint32_t previous_primask = __get_PRIMASK();
	uint32_t value;
	__disable_irq();
	/*
	* Interruption globale désactivée : une nouvelle IRQ ne
	* peut pas s'inserer entre la lecture et le nettoyage du drapeau.
	*/
	value = *flag;
	*flag = 0U;
	if ((previous_primask & 1U) == 0U)
	{
		__enable_irq();
	}

	return value != 0U;
}


void App_OnExti(uint16_t pin)
{
	if (pin == MPU_INT_PIN)
	{
		/* Fenêtre temporelle mesurable pour debug : PD13 = 1 pendant le callback. */
		HAL_GPIO_WritePin(TP_TRACE_PORT, TP_TRACE_PIN, GPIO_PIN_SET);
		s_imu_ready = 1U;
#if TP_ISR_LONG_WAIT

		const uint32_t begin_cycles = DWT->CYCCNT;
		const uint32_t cycles = (SystemCoreClock / 1000U) * TP_ISR_WAIT_MS;
		while ((DWT->CYCCNT - begin_cycles) < cycles)
		{
			__NOP();
		}
#endif
		HAL_GPIO_WritePin(TP_TRACE_PORT, TP_TRACE_PIN, GPIO_PIN_RESET);
	}
	else if (pin == BUTTON_PIN)
	{
		s_button_requested = 1U;
	}
	else
	{
		/* No action */
	}
}

/*
* Mode erreur (error entry point)
* Mise à jour état et activation de la led rouge
*/
static void enter_error(AppError error)
{
	s_app.error = error;
	s_app.state = APP_ERROR;
	HAL_GPIO_WritePin(LED_RED_PORT, LED_RED_PIN, GPIO_PIN_SET);
}

/*
* Fonction d'initialisation matérielle
*/
static void begin_hardware(void)
{
	ImuStatus result;

	uint8_t status = 0U;
	s_imu_ready = 0U;
	s_app.failure_count = 0U;
	s_app.n_irqs = 0U;
	s_app.n_samples = 0U;
	s_app.error = APP_ERR_NONE;
	s_app.oled_ok = false;
	s_app.next_page = 0U;
	Orientation_Reset(&s_app.orientation);
	HAL_GPIO_WritePin(LED_RED_PORT, LED_RED_PIN, GPIO_PIN_RESET);
	result = Imu_Init(s_app.i2c);

	if (result != IMU_OK)
	{
		/* Ecran ok si IMU absente */
		s_app.oled_ok = Oled_Init(s_app.i2c);
		enter_error(APP_ERR_IMU_INIT);
		return;
	}
	s_app.oled_ok = Oled_Init(s_app.i2c);
	if (!s_app.oled_ok)
	{
		enter_error(APP_ERR_OLED);
		return;
	}
	/* Suppression latched interrupt */
	result = Imu_ReadInterruptStatus(&status);
	if (result != IMU_OK)
	{
		enter_error(APP_ERR_I2C);
		return;
	}
	result = Imu_EnableDataReady(true);
	if (result != IMU_OK)
	{
		enter_error(APP_ERR_I2C);
		return;
	}

	s_app.last_irq_ms = HAL_GetTick();
	s_app.last_imu_ms = s_app.last_irq_ms;
	s_app.last_oled_ms = s_app.last_irq_ms;
	s_app.state = APP_CALIBRATION;
}

/*
* Comptabilisation des erreurs IMU
*/
static void note_imu_failure(void)
{
	if (s_app.failure_count < UINT8_MAX)
	{
		++s_app.failure_count;
	}

	if (s_app.failure_count >= APP_MAX_IMU_FAILURES)
	{
		enter_error(APP_ERR_I2C);
	}
}

static void handle_imu_event(uint32_t now)
{
	/*
	* Struct echantillon initialisée, copie uniquement apres HAL_OK.
	*
	*/
	ImuSample sample = {0};
	uint8_t status = 0U;
	ImuStatus result;

	/*
	* Ce compteur augmente lors de la consommation d'un drapeau
	*/
	++s_app.n_irqs;
	s_app.last_irq_ms = now;
	result = Imu_ReadInterruptStatus(&status);
	if (result != IMU_OK)
	{
		note_imu_failure();
		return;
	}

	if ((status & 0x01U) == 0U)
	{
		return; /* Spurious ou traitement déjà clearé. */
	}
	result = Imu_ReadSample(&sample);

	if (result != IMU_OK)
	{
		note_imu_failure();
		return;
	}
	s_app.failure_count = 0U;
	s_app.last_imu_ms = now;
	++s_app.n_samples;
	s_app.last_sample = sample; /* Uniquement après lecture I2C valide. */
	if (s_app.state == APP_CALIBRATION)
	{
		if (Orientation_AddCalibration(&s_app.orientation, &sample))
		{
			s_app.state = APP_RUN;
		}

	}
	else if (s_app.state == APP_RUN)
	{
		if (Orientation_Update(&s_app.orientation, &sample, now))
		{
			const OrientationAngles *a = Orientation_GetAngles(&s_app.orientation);
			if (a != NULL)
			{
			/* C : axes de la carte ; F : repere fixe.
			 * a_F = Rz(psi) Ry(theta) Rx(phi) a_C.
			 * Les angles de Orientation_GetAngles() sont en degrés.
			 */
			const RotationVector3 a_C = {(float)sample.ax, (float)sample.ay, (float)sample.az};

			s_app.a_reference = Rotation_ExprimerDansF(
				a->roll_deg * TP_DEG_TO_RAD_F,
				a->pitch_deg * TP_DEG_TO_RAD_F,
				a->yaw_deg * TP_DEG_TO_RAD_F,
				a_C);
			}
		}
	}
	else
	{
		/* PAUSE */
	}
}

/*
* Button debouncing
*/
static bool button_accepted(uint32_t now)
{
	const bool event = take_event(&s_button_requested);
	if (!event) {
		return false;
	}
	if ((!s_app.button_ever_pressed) || ((now - s_app.last_button_ms) >= APP_BUTTON_DEBOUNCE_MS))
	{
		s_app.last_button_ms = now;
		s_app.button_ever_pressed = true;
		return true;
	}
	return false;
}

static void append_char(char *line, size_t *used, char c)
{
	if (*used < (APP_LINE_CAPACITY - 1U))
	{
		line[*used] = c;
		++(*used);
		line[*used] = '\0';
	}
}

static void append_text(char *line, size_t *used, const char *str)
 {
	 size_t idx = 0U;
	 if (str == NULL) {
		 return;
	 }
	 while (str[idx] != '\0')
	 {
		 append_char(line, used, str[idx]);
		 ++idx;
	 }
 }

 static void append_uint(char *line, size_t *used, uint32_t value)
 {
	 static const char decimal_digits[] = "0123456789";
	 char temp[10] = {0};
	 size_t digits = 0U;

	 do
	 {
		 temp[digits] = decimal_digits[value % 10U];
		 value /= 10U;
		 ++digits;
	 } while ((value != 0U) && (digits < sizeof(temp)));

	 while (digits > 0U)
	 {
		 --digits;
		 append_char(line, used, temp[digits]);
	 }
 }

static void format_angle(char *line, const char *label, float degrees)
 {
	 size_t used = 0U;
	 /* Le signe initial ne change pas apres mise en forme. */
	 const bool negative = degrees < 0.0f;
	 /*
	 * float : conserve fraction d angle. Si NaN, le contrôle ultérieur
	 * retombe sur 0 ; les infinis sont bornés à 999.9.
	 */
	 float magnitude = negative ? -degrees : degrees;
	 uint32_t tenths;
	 line[0] = '\0';

	 if (!(magnitude >= 0.0f)) {
		 magnitude = 0.0f;
	 }
	 if (magnitude > 999.9f) {
		 magnitude = 999.9f;
	 }

	 tenths = (uint32_t)((magnitude * 10.0f) + 0.5f);
	 append_text(line, &used, label);
	 append_text(line, &used, ": ");
	 //if (negative) {
	 append_char(line, &used, negative ? '-' : '+');
	 //}
	 append_uint(line, &used, tenths / 10U);
	 append_char(line, &used, '.');
	 append_uint(line, &used, tenths % 10U);
}

static const char *state_label(void)
 {
	 switch (s_app.state)
	 {
		 case APP_INIT: return "MODE: INIT";
		 case APP_CALIBRATION: return "MODE: CALIB";
		 case APP_RUN: return "MODE: RUN";
		 case APP_PAUSE: return "MODE: PAUSE";
		 case APP_ERROR: return "MODE: ERROR";
		 default: return "MODE: UNKNOWN";
	 }
 }

static const char *error_label(void)
 {
	 switch (s_app.error)
	 {
		 case APP_ERR_NONE: return "STATUS OK";
		 case APP_ERR_IMU_INIT: return "ERR IMU INIT";
		 case APP_ERR_OLED: return "ERR OLED";
		 case APP_ERR_I2C: return "ERR I2C";
		 case APP_ERR_NO_IRQ: return "ERR NO IRQ";
		 case APP_ERR_ANGLES: return "ERR ANGLES";
		 default: return "ERR UNKNOWN";
	 }
 }


/* Une ligne de 21 caracteres maximum pour le SSD1306 (font 5x7).
 * Les données de s_app sont en comptes bruts +/-2 g : 16384 LSB/g.
 * Exemple : X C:+0.00 F:+0.00 ; Z C:+0.71 F:+1.00.
 */
static void format_axis_cf(char *line, char axis, float raw_c, float raw_f)
{
    const float lsb_per_g = 16384.0f;
    (void)snprintf(line, APP_LINE_CAPACITY,
                   "%c C:%+5.2f F:%+5.2f", axis,
                   (double)(raw_c / lsb_per_g),
                   (double)(raw_f / lsb_per_g));
}


static bool redraw_one_page(void)
 {
	 /*
	 * Buffer local 22 octets, init à zéro ; 21 caractères au maximum
	 * plus NUL. size_t used suit la taille effectivement occupée.
	 */
	 char line[APP_LINE_CAPACITY] = {0};
	 size_t used = 0U;

	 const OrientationAngles *angles = Orientation_GetAngles(&s_app.orientation);
	 const bool angles_visible = (s_app.state == APP_RUN) || (s_app.state == APP_PAUSE);

	 switch (s_app.next_page)
    {
        case 0U:
            append_text(line, &used, "@ IMU - AXES C/F");
            break;

        case 1U:
            append_text(line, &used, state_label());
            break;

        case 2U:
            if (s_app.state == APP_ERROR)
            {
                append_text(line, &used, error_label());
            }
            else if (s_app.state == APP_CALIBRATION)
            {
                append_text(line, &used, "CALIBRATION");
            }
            else if (s_app.state == APP_RUN)
            {
                format_axis_cf(line, 'X',
                    (float)s_app.last_sample.ax, s_app.a_reference.x);
            }
            else if (s_app.state == APP_PAUSE)
            {
                append_text(line, &used, "PAUSE - MESURES FIGEES");
            }
            break;

        case 3U:
            if (s_app.state == APP_RUN)
            {
                format_axis_cf(line, 'Y',
                    (float)s_app.last_sample.ay, s_app.a_reference.y);
            }
            break;

        case 4U:
            if (s_app.state == APP_RUN)
            {
                format_axis_cf(line, 'Z',
                    (float)s_app.last_sample.az, s_app.a_reference.z);
            }
            break;

        case 5U:
            if (angles_visible && (angles != NULL))
            {
                format_angle(line, "ROLL", angles->roll_deg);
            }
            break;

        case 6U:
            if (angles_visible && (angles != NULL))
            {
                format_angle(line, "PITCH", angles->pitch_deg);
            }
            break;

        case 7U:
            if (angles_visible && (angles != NULL))
            {
                format_angle(line, "YAW", angles->yaw_deg);
            }
            break;

        default:
            return false;
    }
	 if (!Oled_WritePage(s_app.next_page, line)) {
		 return false;
	 }

	 ++s_app.next_page;

	 if (s_app.next_page >= 8U) {
		 s_app.next_page = 0U;
	 }

	 return true;
 }



void App_Init(I2C_HandleTypeDef *i2c) {

	s_app = (AppContext){0};
	s_app.i2c = i2c;
	s_app.state = APP_INIT;
	s_imu_ready = 0U;
	s_button_requested = 0U;
#if TP_ISR_LONG_WAIT
	/* Init DWT CYCCNT (Cortex-M4) avant le premier DATA_READY. */
	CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
	DWT->CYCCNT = 0U;
	DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
#endif
	HAL_GPIO_WritePin(TP_TRACE_PORT, TP_TRACE_PIN, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(LED_GREEN_PORT, LED_GREEN_PIN, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(LED_RED_PORT, LED_RED_PIN, GPIO_PIN_RESET);
}

AppState App_GetState(void)
{
	return s_app.state;
}

void App_Run(void)
{
	/*
	* Calcul après begin_hardware (qui peut prendre
	* plus de 100 ms), sinon le watchdog peut declarer une panne.
	*/
	uint32_t now = HAL_GetTick();
	/* L'événement de ce tour ne change pas avant le switch. */
	const bool button = button_accepted(now);

	if ((now - s_app.last_led_ms) >= APP_LED_PERIOD_MS)
	{
	    s_app.last_led_ms = now;
	    HAL_GPIO_TogglePin(LED_GREEN_PORT, LED_GREEN_PIN);
	}

	/* ================================================================
	 * TODO - FSM : COMPLETER UNIQUEMENT LES CASES DU SWITCH.
	 * Le code existant dans begin_hardware(), handle_imu_event(),
	 * button_accepted() et enter_error() EST FOURNI : ne pas le modifier.
	 *
	 * ================================================================ */
	switch (s_app.state)
	{
		case APP_INIT:
			/* TODO FSM-1 : initialiser IMU + OLED. */
			break;

		case APP_CALIBRATION:
			/* TODO FSM-2 : si DATA_READY present, traiter l'echantillon.
			 * handle_imu_event() declenche RUN apres 200 mesures. */
			break;

		case APP_RUN:
			/* TODO FSM-3 : bouton -> PAUSE ; consommer DATA_READY. */
			break;

		case APP_PAUSE:
			/* TODO FSM-4 : bouton -> RUN + resynchroniser le filtre.
			 * Continuer a consommer les evenements IMU en PAUSE. */
			break;

		case APP_ERROR:
			/* TODO FSM-5 : bouton -> INIT pour retenter. */
			break;

		default:
			enter_error(APP_ERR_IMU_INIT);
			break;
	}


	/* On relit le tick */
	now = HAL_GetTick();
	if ((s_app.state == APP_CALIBRATION) || (s_app.state == APP_RUN) || (s_app.state == APP_PAUSE))
	{
		if ((now - s_app.last_irq_ms) >= APP_IMU_WATCHDOG_MS)
		{
			enter_error(APP_ERR_NO_IRQ);
		}
	}

	/* Au plus une page OLED de 128-byte toutes les 20 ms. */
	if ((s_app.oled_ok) && ((now - s_app.last_oled_ms) >= APP_OLED_PAGE_PERIOD_MS))
	{
		s_app.last_oled_ms = now;
		if (!redraw_one_page())
		{
			s_app.oled_ok = false;
			enter_error(APP_ERR_OLED);
		}
	}

	/* USB : no-op tant que TP_USB_CDC_ENABLED vaut 0. */
	if (s_app.n_samples > 0U)
	{
		Telemetry_Tick(now, &s_app.last_sample,
			Orientation_GetAngles(&s_app.orientation), &s_app.a_reference);
	}
}

