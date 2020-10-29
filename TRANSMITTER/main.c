#include <DAVE.h>                 //Declarations from DAVE Code Generation (includes SFR declaration)
#include <stdio.h>
/**

 * @brief main() - Application entry point
 *
 * <b>Details of function</b><br>
 * This routine is the application entry point. It is invoked by the device startup code. It is responsible for
 * invoking the APP initialization dispatcher routine - DAVE_Init() and hosting the place-holder for user application
 * code.
 */

#define JMP1_AUTOMATIC_CONTROL 1
//#define POWER_UP	0x77
//#define POWER_DOWN	0x88

static const float g_hysterese_voltage_V = 0.050;
volatile static float g_power_up = 0, g_power_down = 0;
volatile static float g_acomp_mV = 0;
volatile static float g_acomp_mV_average = 0;
volatile static float g_temperature = 0;
volatile static float g_temperature_average = 0;
volatile static float g_voltage = 0;
volatile static float g_voltage_average = 0;
volatile static float g_current = 0;
volatile static float g_current_average = 0;
volatile static float g_P_out;
volatile static uint8_t led_r, led_g, led_b;

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


volatile uint8_t g_clamp = 0;
volatile static uint8_t g_pwm_en = 0;
volatile static uint32_t g_ADC_voltage, g_ADC_current, g_ADC_temperature, g_ADC_acomp, g_ADC_vref;
volatile static float g_ADC_vref_average=880;
volatile static float g_PWM_current_frequency=175E3;
volatile static uint32_t counter_20ms = 0;
uint8_t debug_output_buffer[255];
const float PWM_start_frequency = 175E3;
const float PWM_max_frequency = 205E3;
const float PWM_min_frequency = 100E3;
const float PWM_delta_frequency  = 1E3;
uint32_t histogram_received_patterns[256];
const uint8_t hamm_margin = 4;
const uint8_t ones_thresh = 4;

void PWM_enable_disable_toggle()
{
	/* Toggle PWM enable */
	g_pwm_en ^=0x1;

	/* Overwrite g_pwm_en to avoid enabling PWM in any critical situation */
	if ( g_critical_detected )
		g_pwm_en=0;

	if (g_pwm_en==1)
	{
	    PWM_CCU8_Start(&PWM);
	}
	else
	{
		PWM_CCU8_Stop(&PWM);
	}
}

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

uint8_t count_ones (uint8_t a)
{
	uint8_t count = 0;
	while (a) {
		count += a & 1;
		a >>= 1;
	}
	return count;
}

uint8_t hamm_diff (uint8_t a, uint8_t b)
{
	uint8_t diff = a ^ b;
	return count_ones(diff);
}

float average_calc(float new_value, float current_average, uint16_t filter)
{
	return (current_average*((float)filter-1) + new_value)/filter;
}

int main(void)
{
  DAVE_STATUS_t status;
  uint8_t received_byte,index=0;
  uint8_t receive_buffer[5],power_up,power_down;
  uint8_t debounce_enc_switch_cnt=0,debounce_enc_switch_state=0,debounce_enc_switch_state_old=1;
  uint32_t watchdog_inField_communication;
  uint32_t ENC_A_old;
  uint32_t ENC_A_new, ENC_B_new;


  //static uint32_t samples=0;

  status = DAVE_Init();           /* Initialization of DAVE APPs  */

  for (uint32_t index=0; index<256;index++)
	  histogram_received_patterns[index]=0;

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
	// TODO: Temperature is overwritten here with 50. For debugging only until sensor is calibrated
	g_temperature_average=50;

	/* Check for overvoltage, -current, temperature situations */
	check_threshold (g_voltage_average, VOLTAGE_MAX, VOLTAGE_HYSTERESIS, &g_overvoltage_detected);
	check_threshold (g_current_average, CURRENT_MAX, CURRENT_HYSTERESIS, &g_overcurrent_detected);
	check_threshold (g_temperature_average, TEMP_MAX, TEMP_HYSTERESIS, &g_overtemperature_detected);

	g_critical_detected = g_overvoltage_detected | g_overcurrent_detected | g_overtemperature_detected | g_clamp;

	/* Check for critical overvoltage, -current, temperature situations and set red/green LEDs*/
	if ( g_critical_detected )
	{
		g_pwm_en = 0;
		PWM_CCU8_Stop(&PWM);
		DIGITAL_IO_SetOutputHigh(&G_LED); /*Micrium:*/ led_g=1;
		DIGITAL_IO_SetOutputLow(&R_LED); /*Micrium:*/ led_r=0;
	}
	else
	{
		DIGITAL_IO_SetOutputLow(&G_LED); /*Micrium:*/ led_g=0;
		DIGITAL_IO_SetOutputHigh(&R_LED); /*Micrium:*/ led_r=1;
	}

	/* Process incoming patterns from inField UART communication */
	g_power_up=0;
	g_power_down=0;

	while (!UART_IsRXFIFOEmpty (&COM))
	{
		/* Store last 5 bytes inside array received_byte*/
		UART_Receive(&COM, &received_byte, 1);
		if (index<5)
		{
			receive_buffer[index] = received_byte & 0b00011111;
			histogram_received_patterns[received_byte]++;
			index++;
		}
		/* 5 bytes received?
		 * Process and check for power-up/down commands
		 */
		if (index==5)
		{
			power_up=0;
			power_down=0;
			for (index=0; index<5; index++)
			{
				uint8_t ones = count_ones(receive_buffer[index]);
				if (ones < ones_thresh) //(hamm_diff(receive_buffer[index], POWER_UP   ) < hamm_margin)
					power_up++;
				else //if (hamm_diff(receive_buffer[index], POWER_DOWN ) < hamm_margin)
					power_down++;
			}
			if (power_up > 3)
			{
				g_power_up=1;
			}
			else if (power_down > 3)
			{
				g_power_down=1;
			}
			index = 0;
		}
		/* Reset watchdog of InField-communication */
		watchdog_inField_communication = 0;
	}

	/* Debug output on UART every 200 millisecond */
	if (counter_20ms>10)
	{
		counter_20ms=0;
		sprintf ((char *)debug_output_buffer,"Volt[mV]:%07.1f   Curr[mA]:%07.1f   Pow[W]:%05.1f   Temp[°C]: %05.1f   Frequ[kHz]:%07.3f OV:%1i OC:%1i OT:%1i CL:%1i %02x %02x %02x %02x %02x %02x \n\r",
				g_voltage_average,
				g_current_average,
				g_P_out,
				g_temperature_average,
				g_PWM_current_frequency,
				(int)g_overvoltage_detected,
				(int)g_overcurrent_detected,
				(int)g_overtemperature_detected,
				(int)g_clamp,
				(int)receive_buffer[0],
				(int)receive_buffer[1],
				(int)receive_buffer[2],
				(int)receive_buffer[3],
				(int)receive_buffer[4],
				count_ones(receive_buffer[4])
									);

		UART_Transmit(&RS232, debug_output_buffer, (uint32_t)strlen((char *)debug_output_buffer));
		/* Use this timer to increment watchdog on inField communication */
		if (watchdog_inField_communication < 5)
			watchdog_inField_communication++;
	}

	/* Check if automatic control is enabled.
	 * If yes, process power up/down commands from inField communication
	 */
	if (DIGITAL_IO_GetInput(&JMP_1)==JMP1_AUTOMATIC_CONTROL)
	{
		/* automatic control */
		if (g_power_up==1)
		{
			/* Decrease PWM frequency for power up */
			if (g_PWM_current_frequency > PWM_min_frequency)
			{
				g_PWM_current_frequency = g_PWM_current_frequency - PWM_delta_frequency;
				PWM_CCU8_SetFreqSymmetric(&PWM, (uint32_t)g_PWM_current_frequency);
			}
			g_power_up=0;
		}
		if (g_power_down==1)
		{
			/* Increase PWM frequency for power down */
			if (g_PWM_current_frequency < PWM_max_frequency)
			{
				g_PWM_current_frequency = g_PWM_current_frequency + PWM_delta_frequency;
				PWM_CCU8_SetFreqSymmetric(&PWM, (uint32_t)g_PWM_current_frequency);
			}
			g_power_down=0;
		}
	}

	/* Check for automatic/manual control */
	if (DIGITAL_IO_GetInput(&JMP_1)!=JMP1_AUTOMATIC_CONTROL)
	{
		ENC_A_new = DIGITAL_IO_GetInput(&ENC_A);
		ENC_B_new = DIGITAL_IO_GetInput(&ENC_B);

		/* manual control */
		if ((ENC_A_new==0)&&(ENC_A_old==1))
		{
			if (ENC_B_new==1)
			{
				/* Decrease PWM frequency for power up */
				if (g_PWM_current_frequency > PWM_min_frequency)
				{
					g_PWM_current_frequency = g_PWM_current_frequency - PWM_delta_frequency;
					PWM_CCU8_SetFreqSymmetric(&PWM, (uint32_t)g_PWM_current_frequency);
				}

			}
			if (ENC_B_new==0)
			{
				/* Increase PWM frequency for power down */
				if (g_PWM_current_frequency < PWM_max_frequency)
				{
					g_PWM_current_frequency = g_PWM_current_frequency + PWM_delta_frequency;
					PWM_CCU8_SetFreqSymmetric(&PWM, (uint32_t)g_PWM_current_frequency);
				}
			}
		}
		ENC_A_old=ENC_A_new;
	}

	/* Check encoder switch and toggle on/off state of PLL */
	if (DIGITAL_IO_GetInput(&ENC_Switch)==debounce_enc_switch_state)
	{
		debounce_enc_switch_cnt++;
		if (debounce_enc_switch_cnt>=30)
		{
			if ((debounce_enc_switch_state==0)&&(debounce_enc_switch_state_old!=debounce_enc_switch_state))
			{
				PWM_enable_disable_toggle();
			}
			debounce_enc_switch_cnt=0;
			debounce_enc_switch_state_old=debounce_enc_switch_state;
		}
	}
	else
	{
		debounce_enc_switch_cnt=0;
	}
	debounce_enc_switch_state=DIGITAL_IO_GetInput(&ENC_Switch);

	/* Check if sudden current drop is detected. If yes, reset PWM frequency to start frequency */
	if (g_current_average/2 > (float)g_current)
	{
		if (g_PWM_current_frequency > PWM_start_frequency)
		{
			g_PWM_current_frequency = PWM_start_frequency;
			PWM_CCU8_SetFreqSymmetric(&PWM, (uint32_t)g_PWM_current_frequency);
		}
	}

	/* Set blue LED */
	if (DIGITAL_IO_GetInput(&JMP_1)==JMP1_AUTOMATIC_CONTROL)
	{
		/* automatic mode */
		/* In automatic mode blue LED is turned on as long as command patterns are received
		 * from receiver and PWM is still enabled
		 * */
		if ( (watchdog_inField_communication < 3) && ( g_pwm_en == 1) )
		{
			DIGITAL_IO_SetOutputLow(&B_LED);   /*Micrium:*/ led_b=0;
		}
		else
		{
			DIGITAL_IO_SetOutputHigh(&B_LED);  /*Micrium:*/ led_b=1;
		}
	}
	else
	{
		/* manual mode */
		/* In manual mode blue LED is turned on as long as PWM is still enabled */
		if ( g_pwm_en == 1 )
		{
			DIGITAL_IO_SetOutputLow(&B_LED); /*Micrium:*/ led_b=0;
		}
		else
		{
			DIGITAL_IO_SetOutputHigh(&B_LED); /*Micrium:*/ led_b=1;
		}
	}
  }
}

void task_20ms(void)
{
	const float R1_  = 1000;
	const float R2_  = 47;

	const uint32_t ads_max_plus1=1024;

	const float Ntc_teiler = 3.74;
	const float Ntc_offset = 121;

	uint16_t ADC_voltage, ADC_current, ADC_temperature;
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
	g_temperature = (int)(temperature_s + 0.5);

	if (g_temperature_average == 0)
		g_temperature_average = g_temperature;
	g_temperature_average = average_calc(g_temperature, g_temperature_average, 16);

	/* Voltage processing
	 *          (R1 + R2) * Adc_Wert
	 *   U_out = --------------------------
	 *           R2
	 */
	ADC_voltage = VREF_VALUE_MV * g_ADC_voltage / g_ADC_vref_average;
	g_voltage = ((float)ADC_voltage * (R1_ + R2_) ) / R2_;

	if (g_voltage_average == 0)
		g_voltage_average = g_voltage;
	g_voltage_average = average_calc(g_voltage, g_voltage_average, 16);

	/* Current processing
	 *
	 */
	ADC_current = VREF_VALUE_MV * g_ADC_current / g_ADC_vref_average;

	g_current =  ADC_current * 4.5;
	if (g_current_average == 0)
		g_current_average = g_current;
	g_current_average = average_calc(g_current, g_current_average, 16);

	/* Power processing
	 */
	g_P_out = g_voltage_average * g_current_average * 1E-6;

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
