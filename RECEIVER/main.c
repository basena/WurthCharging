#include <DAVE.h>                 //Declarations from DAVE Code Generation (includes SFR declaration)
#include "xmc_vadc.h"

volatile static float g_acomp_mV = 0;
volatile static float g_acomp_mV_average = 0;
volatile static float g_temperature = 0;
volatile static float g_temperature_average = 0;
volatile static float g_voltage = 0;
volatile static float g_voltage_average = 0;
volatile static float g_voltage_out_max = 21000;
volatile static float g_current = 0;
volatile static float g_current_average = 0;
volatile static float g_current_max = 4000;
volatile static float g_P_out;
volatile static uint32_t g_ADC_voltage, g_ADC_current, g_ADC_temperature, g_ADC_acomp, g_ADC_vref;
volatile static float g_ADC_vref_average=880;
volatile uint8_t g_clamp = 1, led_r, led_b, led_up, led_down;
volatile static uint32_t counter_20ms = 0;

volatile static uint32_t g_overvoltage_detected=0;
volatile static uint32_t g_overcurrent_detected=0;
volatile static uint32_t g_overtemperature_detected=0;
volatile static uint32_t g_critical_detected=0;

#define VOLTAGE_MAX			28000
#define VOLTAGE_HYSTERESIS	1000
#define CURRENT_MAX			11000
#define CURRENT_HYSTERESIS	1000
#define TEMP_MAX			110
#define TEMP_HYSTERESIS		20
#define ACOMP_MAX           2000
#define ACOMP_HYSTERESIS    50
#define VREF_VALUE_MV       2480

static uint8_t up_hex = ~0x00; //0x44; //0b01000100;
static uint8_t dn_hex = ~0xFF; //0x77; //0b01110111;


void check_threshold (uint32_t value, uint32_t threshold, uint32_t hysteresis_offset, volatile uint32_t* status)
{
	if ( ( value > threshold + hysteresis_offset) && ( *status == 0 ) )
	{
		*status = 1;
	}
	if ( ( value < threshold - hysteresis_offset) && ( *status == 1 ) )
	{
		*status = 0;
	}
	return;
}

int main(void)
{
  DAVE_STATUS_t status;
  uint32_t send_redundant;

  status = DAVE_Init();           /* Initialization of DAVE APPs  */

  if(status != DAVE_STATUS_SUCCESS)
  {
    /* Placeholder for error handler code. The while loop below can be replaced with an user error handler. */
    XMC_DEBUG("DAVE APPs initialization failed\n");

    while(1U)
    {

    }
  }

  /* Placeholder for user application code. The while loop below can be replaced with user application code. */
  while(1U)
  {
		/* Check for overvoltage, -current, temperature situations */
		check_threshold (g_voltage_average, VOLTAGE_MAX, VOLTAGE_HYSTERESIS, &g_overvoltage_detected);
		check_threshold (g_current_average, CURRENT_MAX, CURRENT_HYSTERESIS, &g_overcurrent_detected);
		check_threshold (g_temperature_average, TEMP_MAX, TEMP_HYSTERESIS, &g_overtemperature_detected);

		g_critical_detected = g_overvoltage_detected | g_overcurrent_detected | g_overtemperature_detected | g_clamp;

		if ( g_critical_detected )
		{
//			DIGITAL_IO_SetOutputHigh(&CLAMP);
			DIGITAL_IO_SetOutputHigh(&G_LED); /*Micrium:*/ led_b=1;
			DIGITAL_IO_SetOutputLow(&R_LED);  /*Micrium:*/ led_r=0;
		}
		else
		{
			DIGITAL_IO_SetOutputLow(&CLAMP);
			DIGITAL_IO_SetOutputLow(&G_LED);  /*Micrium:*/ led_b=0;
			DIGITAL_IO_SetOutputHigh(&R_LED); /*Micrium:*/ led_r=1;
		}

		if (counter_20ms >= 5)
		{
			counter_20ms = 0;
			if (g_voltage < g_voltage_out_max && g_current < g_current_max)
			{
				/* Power Up */
				DIGITAL_IO_SetOutputLow(&UP_LED); led_up = 0;
				DIGITAL_IO_SetOutputHigh(&DOWN_LED); led_down = 1;
				send_redundant = 5;
				while (send_redundant-- > 0)
					while(UART_Transmit(&COM, &up_hex, 1));
			}
			else
			{
				/* Power Down */
				DIGITAL_IO_SetOutputHigh(&UP_LED); led_up = 1;
				DIGITAL_IO_SetOutputLow(&DOWN_LED); led_down = 0;
				send_redundant = 5;
				while (send_redundant-- > 0)
					while(UART_Transmit(&COM, &dn_hex, 1));
			}
		}
  }
}

float average_calc(float new_value, float current_average, uint16_t filter)
{
	return (current_average*((float)filter-1) + new_value)/filter;
}

void task_20ms(void)
{
	const int16_t ads_max_plus1 = 1024;
	const float Ntc_teiler = 3.74;
	const float Ntc_offset = 121;
	const float R1_  = 1000;
	const float R2_  = 43;
	const float Ref_ = 5000;
	float T1, T2;
	volatile static uint16_t ADC_voltage, ADC_current, ADC_temperature;
	float temperature_s;

	counter_20ms++;

	/* Read analog to digital converters */
	g_ADC_voltage     = ADC_MEASUREMENT_ADV_GetResult(&VOLT_CURR_TEMP_Voltage_handle) / 4; // 4-samples filter inside ADC_MEASUREMENT_ADV enabled
	g_ADC_current     = ADC_MEASUREMENT_ADV_GetResult(&VOLT_CURR_TEMP_Current_handle) / 4; // 4-samples filter inside ADC_MEASUREMENT_ADV enabled
	g_ADC_temperature = ADC_MEASUREMENT_ADV_GetResult(&VOLT_CURR_TEMP_Temperature_handle) / 4; // 4-samples filter inside ADC_MEASUREMENT_ADV enabled

	/* Temperature processing
	 *               (Adc_max_plus1 - Adc_wert)
	 *  Temperatur = -------------------------- - Ntc_Offset
	 *                    Ntc_Teiler
	 */
	ADC_temperature = g_ADC_temperature / 4;	// 12 bit --> 10bit
	temperature_s = ads_max_plus1 - (float)ADC_temperature;
	temperature_s = temperature_s / Ntc_teiler;
	temperature_s = temperature_s - Ntc_offset;

	// TODO: Remove fixed temperature
	g_temperature = 40;
	if (g_temperature_average == 0)
		g_temperature_average = g_temperature;
	g_temperature_average = average_calc(g_temperature, g_temperature_average, 16);


	/* Voltage processing
	 *          (R1 + R2) * Ref * Adc_Wert
	 *   U_out = --------------------------
	 *           R2 * ADC_max_plus1
	 */
	ADC_voltage = g_ADC_voltage / 4;	// 12 bit --> 10bit

	T1 = (R1_ + R2_) * Ref_;
	T2 = R2_ * ads_max_plus1;
	g_voltage = ((float)ADC_voltage * T1) / T2;

	if (g_voltage_average == 0)
		g_voltage_average = g_voltage;
	g_voltage_average = average_calc(g_voltage, g_voltage_average, 16);

	/* Current processing
	 *
	 */
	ADC_current = g_ADC_current / 4;	// 12 bit --> 10bit
	g_current =  4.38*ADC_current * 10000 / 826;
	if (g_current_average == 0)
		g_current_average = g_current;
	g_current_average = average_calc(g_current, g_current_average, 16);

	/* Power processing
	 */
	g_P_out = g_voltage * g_current * 1E-6;

	return;
}

void ACOMP_ISR(void)
{
	static uint8_t startup_blank = 0;

	/* For the first 160ms (32*5ms) after startup ignore clamp */
	if (startup_blank < 32)
		startup_blank++;

	/* Read analog to digital converters */
	g_ADC_acomp = ADC_MEASUREMENT_ADV_GetResult(&ACOMP_ACOMP_handle)/4; // 4-samples filter inside ADC_MEASUREMENT_ADV enabled
	g_ADC_vref  =ADC_MEASUREMENT_ADV_GetResult(&ACOMP_VRef_handle)/4; // 4-samples filter inside ADC_MEASUREMENT_ADV enabled
	g_ADC_vref_average = average_calc((float)g_ADC_vref, g_ADC_vref_average, 256);

	g_acomp_mV = (float)g_ADC_acomp * VREF_VALUE_MV / g_ADC_vref_average;;
	g_acomp_mV_average = average_calc(g_acomp_mV, g_acomp_mV_average, 32);

	if (((g_acomp_mV_average > ACOMP_MAX + ACOMP_HYSTERESIS ) && (g_clamp == 0) && (startup_blank>=32)))
	{
		// Upper boundary transition
		g_clamp = 1;
	}
	if ((g_acomp_mV_average < ACOMP_MAX - ACOMP_HYSTERESIS ) && (g_clamp == 1))
	{
		// Lower boundary transition
		g_clamp = 0;
	}
	return;
}
