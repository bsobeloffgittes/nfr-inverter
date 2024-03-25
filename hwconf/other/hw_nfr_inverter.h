#ifndef HW_NFR_H_
#define HW_NFR_H_

#define HW_NAME					"NFR_INVERTER"


// This doesn't seem to be referenced anywhere so probably not needed
#define HW_MAJOR				1
#define HW_MINOR				0


// HW PROPERTIES
// #define HW_HAS_DRV8301 // Not sure about this - might be good for fault stuff but don't want SPI?
#define HW_HAS_3_SHUNTS
#define HW_HAS_PHASE_SHUNTS
// got rid of property "permanent NRF"
#define HW_HAS_PHASE_FILTERS

// Macros
#define ENABLE_GATE()			palSetPad(GPIOB, 5)
#define DISABLE_GATE()			palClearPad(GPIOB, 5)
#define DCCAL_ON()    // Literally no clue what this means ------------------------------------------------------------------------
#define DCCAL_OFF()
#define IS_DRV_FAULT()			(!palReadPad(GPIOB, 7))

#define PHASE_FILTER_GPIO		GPIOC
#define PHASE_FILTER_PIN		13
#define PHASE_FILTER_ON()		palSetPad(PHASE_FILTER_GPIO, PHASE_FILTER_PIN)
#define PHASE_FILTER_OFF()		palClearPad(PHASE_FILTER_GPIO, PHASE_FILTER_PIN)

#define CURRENT_FILTER_ON()		palSetPad(GPIOD, 2)
#define CURRENT_FILTER_OFF()	palClearPad(GPIOD, 2)

#define LED_GREEN_GPIO			GPIOB
#define LED_GREEN_PIN			0
#define LED_RED_GPIO			GPIOB
#define LED_RED_PIN				1
#define LED_GREEN_ON()			palSetPad(LED_GREEN_GPIO, LED_GREEN_PIN)
#define LED_GREEN_OFF()			palClearPad(LED_GREEN_GPIO, LED_GREEN_PIN)
#define LED_RED_ON()			palSetPad(LED_RED_GPIO, LED_RED_PIN)
#define LED_RED_OFF()			palClearPad(LED_RED_GPIO, LED_RED_PIN)

// Ignoring sensor port voltage control

// // Not sure if shutdown pin is needed because it's left floating on hardware ---------------------------------------------------------
// // Shutdown pin
// #define HW_SHUTDOWN_GPIO		GPIOC
// #define HW_SHUTDOWN_PIN			5
// #define HW_SHUTDOWN_HOLD_ON()	palSetPad(HW_SHUTDOWN_GPIO, HW_SHUTDOWN_PIN)
// #define HW_SHUTDOWN_HOLD_OFF()	palClearPad(HW_SHUTDOWN_GPIO, HW_SHUTDOWN_PIN)
// #define HW_SAMPLE_SHUTDOWN()	hw_sample_shutdown_button()

/* // Hold shutdown pin early to wake up on short pulses
#define HW_EARLY_INIT()			palSetPadMode(HW_SHUTDOWN_GPIO, HW_SHUTDOWN_PIN, PAL_MODE_OUTPUT_PUSHPULL); \
								HW_SHUTDOWN_HOLD_ON(); \
								palSetPadMode(GPIOD, 2, \
								PAL_MODE_OUTPUT_PUSHPULL | \
								PAL_STM32_OSPEED_HIGHEST); \
								CURRENT_FILTER_ON()
*/




// Using ADC from hw_60_core.h, not from example
/*
 * ADC Vector
 *
 * 0:	IN0		SENS1
 * 1:	IN1		SENS2
 * 2:	IN2		SENS3
 * 3:	IN10	CURR1
 * 4:	IN11	CURR2
 * 5:	IN12	CURR3
 * 6:	IN5		ADC_EXT1
 * 7:	IN6		ADC_EXT2
 * 8:	IN3		TEMP_PCB
 * 9:	IN14	TEMP_MOTOR
 * 10:	IN15	ADC_EXT3, Shutdown on MK3
 * 11:	IN13	AN_IN
 * 12:	Vrefint
 * 13:	IN0		SENS1
 * 14:	IN1		SENS2
 */

#define HW_ADC_INJ_CHANNELS		3
#define HW_ADC_NBR_CONV			5
#define HW_ADC_CHANNELS			(HW_ADC_NBR_CONV * 3)

// ADC Indexes
#define ADC_IND_SENS1			3
#define ADC_IND_SENS2			4
#define ADC_IND_SENS3			5
#define ADC_IND_CURR1			0
#define ADC_IND_CURR2			1
#define ADC_IND_CURR3			2
#define ADC_IND_VIN_SENS		11
#define ADC_IND_EXT				6
#define ADC_IND_EXT2			7
#define ADC_IND_TEMP_MOS		8
#define ADC_IND_TEMP_MOTOR		9
#define ADC_IND_VREFINT			12
#define ADC_IND_SHUTDOWN		10

// Component parameters
#ifndef V_REG
#define V_REG					3.3
#endif
#ifndef VIN_R1
#define VIN_R1					195.4583
#endif
#ifndef VIN_R2
#define VIN_R2					1
#endif
// Defined in V/V
// #ifndef VOLT_AMP_GAIN
// #define VOLT_AMP_GAIN           0.00509014
// #endif
// // Gain for voltage across shunt
#ifndef CURRENT_AMP_GAIN
#define CURRENT_AMP_GAIN		45.1
#endif
#ifndef CURRENT_SHUNT_RES
#define CURRENT_SHUNT_RES		0.0005
#endif

// Input voltage 
#define GET_INPUT_VOLTAGE()		((V_REG / 4095.0) * (float)ADC_Value[ADC_IND_VIN_SENS] * ((VIN_R1 + VIN_R2) / VIN_R2))

// NTC Thermistors
// #define NTC_RES(adc_val)		((4095.0 * 10000.0) / adc_val - 10000.0)
#define NTC_RES(adc_val)		(-110000.0 * (adc_val / 4095.0)) / ((adc_val / 4095.0) - 15.0)
// #define NTC_TEMP(adc_ind)		(1.0 / ((logf(NTC_RES(ADC_Value[adc_ind]) / 10000.0) / 3380.0) + (1.0 / 298.15)) - 273.15)
#define NTC_TEMP(adc_ind)		(1.0 / ((logf(NTC_RES(ADC_Value[adc_ind]) / 5000.0) / 3375.0) + (1.0 / 298.15)) - 273.15)

// CHANGE THIS ONCE THERMISTOR, ETC IS DECIDED ON -----------------------------------------------------------------------------------------
#define NTC_RES_MOTOR(adc_val)	(10000.0 / ((4095.0 / (float)adc_val) - 1.0)) // Motor temp sensor on low side
#define NTC_TEMP_MOTOR(beta)	(1.0 / ((logf(NTC_RES_MOTOR(ADC_Value[ADC_IND_TEMP_MOTOR]) / 10000.0) / beta) + (1.0 / 298.15)) - 273.15)

// Voltage on ADC channel
#define ADC_VOLTS(ch)			((float)ADC_Value[ch] / 4096.0 * V_REG)

// Double samples in beginning and end for positive current measurement.
// Useful when the shunt sense traces have noise that causes offset.
#ifndef CURR1_DOUBLE_SAMPLE
#define CURR1_DOUBLE_SAMPLE		0
#endif
#ifndef CURR2_DOUBLE_SAMPLE
#define CURR2_DOUBLE_SAMPLE		0
#endif
#ifndef CURR3_DOUBLE_SAMPLE
#define CURR3_DOUBLE_SAMPLE		0
#endif

// COMM-port ADC GPIOs
#define HW_ADC_EXT_GPIO			GPIOA
#define HW_ADC_EXT_PIN			5
#define HW_ADC_EXT2_GPIO		GPIOA
#define HW_ADC_EXT2_PIN			6


// UART Peripheral
#define HW_UART_DEV				SD3
#define HW_UART_GPIO_AF			GPIO_AF_USART3
#define HW_UART_TX_PORT			GPIOB
#define HW_UART_TX_PIN			10
#define HW_UART_RX_PORT			GPIOB
#define HW_UART_RX_PIN			11


// Permanent UART Peripheral (for NRF51)
#define HW_UART_P_BAUD			115200
#define HW_UART_P_DEV			SD4
#define HW_UART_P_DEV_TX		SD5 // UART for TX, due to mistake below
#define HW_UART_P_GPIO_AF		GPIO_AF_UART4
#define HW_UART_P_TX_PORT		GPIOC
#define HW_UART_P_TX_PIN		12 // This is a mistake in the HW. We have to use a hack to use UART5.
#define HW_UART_P_RX_PORT		GPIOC
#define HW_UART_P_RX_PIN		11

// ICU Peripheral for servo decoding - Can probably get rid of this
#define HW_USE_SERVO_TIM4
#define HW_ICU_TIMER			TIM4
#define HW_ICU_TIM_CLK_EN()		RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE)
#define HW_ICU_DEV				ICUD4
#define HW_ICU_CHANNEL			ICU_CHANNEL_1
#define HW_ICU_GPIO_AF			GPIO_AF_TIM4
#define HW_ICU_GPIO				GPIOB
#define HW_ICU_PIN				6

// I2C Peripheral
#define HW_I2C_DEV				I2CD2
#define HW_I2C_GPIO_AF			GPIO_AF_I2C2
#define HW_I2C_SCL_PORT			GPIOB
#define HW_I2C_SCL_PIN			10
#define HW_I2C_SDA_PORT			GPIOB
#define HW_I2C_SDA_PIN			11

// Hall/encoder pins
#define HW_HALL_ENC_GPIO1		GPIOC
#define HW_HALL_ENC_PIN1		6
#define HW_HALL_ENC_GPIO2		GPIOC
#define HW_HALL_ENC_PIN2		7
#define HW_HALL_ENC_GPIO3		GPIOC
#define HW_HALL_ENC_PIN3		8
#define HW_ENC_TIM				TIM3
#define HW_ENC_TIM_AF			GPIO_AF_TIM3
#define HW_ENC_TIM_CLK_EN()		RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE)
#define HW_ENC_EXTI_PORTSRC		EXTI_PortSourceGPIOC
#define HW_ENC_EXTI_PINSRC		EXTI_PinSource8
#define HW_ENC_EXTI_CH			EXTI9_5_IRQn
#define HW_ENC_EXTI_LINE		EXTI_Line8
#define HW_ENC_EXTI_ISR_VEC		EXTI9_5_IRQHandler
#define HW_ENC_TIM_ISR_CH		TIM3_IRQn
#define HW_ENC_TIM_ISR_VEC		TIM3_IRQHandler

// SPI pins
#define HW_SPI_DEV				SPID1
#define HW_SPI_GPIO_AF			GPIO_AF_SPI1
#define HW_SPI_PORT_NSS			GPIOB
#define HW_SPI_PIN_NSS			11
#define HW_SPI_PORT_SCK			GPIOA
#define HW_SPI_PIN_SCK			5
#define HW_SPI_PORT_MOSI		GPIOA
#define HW_SPI_PIN_MOSI			7
#define HW_SPI_PORT_MISO		GPIOA
#define HW_SPI_PIN_MISO			6

// // SPI for DRV8301 - probably get rid of this????
// #define DRV8301_MOSI_GPIO		GPIOB
// #define DRV8301_MOSI_PIN		4
// #define DRV8301_MISO_GPIO		GPIOB
// #define DRV8301_MISO_PIN		3
// #define DRV8301_SCK_GPIO		GPIOC
// #define DRV8301_SCK_PIN			10
// #define DRV8301_CS_GPIO			GPIOC
// #define DRV8301_CS_PIN			9

// Ignoring MPU950

// Ignoring NRF SWD

// Measurement macros - double check what these do ----------------------------------------------------------------------------------------------------------------------
#define ADC_V_L1				ADC_Value[ADC_IND_SENS1]
#define ADC_V_L2				ADC_Value[ADC_IND_SENS2]
#define ADC_V_L3				ADC_Value[ADC_IND_SENS3]
#define ADC_V_ZERO				(ADC_Value[ADC_IND_VIN_SENS] / 2)

// Macros
#define READ_HALL1()			palReadPad(HW_HALL_ENC_GPIO1, HW_HALL_ENC_PIN1)
#define READ_HALL2()			palReadPad(HW_HALL_ENC_GPIO2, HW_HALL_ENC_PIN2)
#define READ_HALL3()			palReadPad(HW_HALL_ENC_GPIO3, HW_HALL_ENC_PIN3)

// Override dead time. See the stm32f4 reference manual for calculating this value.
#define HW_DEAD_TIME_NSEC					1400.0

// Default setting overrides
#ifndef MCCONF_DEFAULT_MOTOR_TYPE
#define MCCONF_DEFAULT_MOTOR_TYPE		MOTOR_TYPE_FOC
#endif
#ifndef MCCONF_FOC_F_ZV
#define MCCONF_FOC_F_ZV					30000.0
#endif
#ifndef MCCONF_L_MAX_ABS_CURRENT
#define MCCONF_L_MAX_ABS_CURRENT		150.0	// The maximum absolute current above which a fault is generated
#endif
#ifndef MCCONF_FOC_SAMPLE_V0_V7
#define MCCONF_FOC_SAMPLE_V0_V7			false	// Run control loop in both v0 and v7 (requires phase shunts) - check if this should actually be true
#endif

// Setting limits
#define HW_LIM_CURRENT			-120.0, 120.0
#define HW_LIM_CURRENT_IN		-120.0, 120.0
#define HW_LIM_CURRENT_ABS		0.0, 160.0
// default lim_vin is 6, 57
#define HW_LIM_VIN				20.0, 600.0
#define HW_LIM_ERPM				-200e3, 200e3
#define HW_LIM_DUTY_MIN			0.0, 0.1
#define HW_LIM_DUTY_MAX			0.0, 0.99
#define HW_LIM_TEMP_FET			-40.0, 110.0


#endif /*HW_NFR_H_*/