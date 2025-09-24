/* ***************************************************************************************
 *  Project:    USB oscilloscope
 *  File:       app.c
 *
 *  Language:   C
 ************************************************************************************** */

/* #INCLUDES */
#include "app.h"
#include "main.h"
#include "fsm.h"
#include "usbd_cdc_if.h"
#include <string.h>


/* #DEFINES */
// usb transmission
#define USB_BUFLEN 64
#define CMD_RX_BUFFER_SIZE 128
#define CMD_TX_BUFFER_SIZE 128

// data transmission
#define MAX_TRANSFER_BYTELENGTH 1000

// sampling frequency
#define FREQ_TIMER 100000000
#define MAX_FREQ_ADC 10000000
#define MIN_FREQ_ADC 20000
#define INIT_FREQ_ADC MAX_FREQ_ADC

// moving average filter
#define MOV_AVG_MODE_CUMULATIVE 1
#define MOV_AVG_MODE_RECURSIVE 2
#define CUR_MOV_AVG_MODE MOV_AVG_MODE_RECURSIVE

#define MASK_D0_D6	0x7F
#define MASK_D7_D11 0xF80

/* PRIVATE FUNCTION PROTOTYPES */
static void applyMovingAverageFilter();
static void CDC_Send(uint8_t* data, uint16_t len);
static void receiveCommand();
static void processCommand(const char* cmd);
static void GPIO_Init();
static void EXTI_Init();
static void TIM3_Init();
static void DMA_Init(uint32_t numOfTransfers);
static int determineBufferAddresses(uint16_t buf[], uint32_t bufSize, uint32_t* addressArray[][NUMBER_OF_COLUMNS]);
static void TIM1_Init();

/* STATIC GLOBAL VARIABLES */
static char cmdTxBuffer[CMD_TX_BUFFER_SIZE];

static volatile uint8_t usbRxBuf[USB_BUFLEN];
static volatile uint16_t usbRxBufLen;
static volatile uint8_t usbRxFlag = 0;

static volatile uint8_t cmdRxBuffer[CMD_RX_BUFFER_SIZE];
static volatile uint16_t cmdRxIndex = 0;

/* configuration variables (controllable by PC application) */
static volatile uint32_t newSamplingFrequency = 10000000;
static volatile uint32_t currentSamplingFrequency = 10000000;

static volatile uint16_t newReferenceVoltage = 4095;
static volatile uint16_t currentReferenceVoltage = 4095;

static volatile TRIGGERMODE_T currentTriggerMode = TRIGGER_RISING_EDGE;

// number of points used by moving average filter
static volatile uint16_t newNumOfPoints = 51;
static volatile uint16_t currentNumOfPoints = 51;

/* status variables */
static volatile uint8_t overflowHappened = 0;

/* GLOBAL VARIABLES */

// array, that holds the base addresses for each DMA transaction
// <NUM_OF_TRANSFERS_PER_TRANSACTION> sample values are referenced per array element
volatile uint32_t* addressArray[NUMBER_OF_ROWS][NUMBER_OF_COLUMNS] = {0};

/* Global array variable for storing the 12bit sampling values from the ADC[D11...D0] */
volatile int16_t sampleBuffer[NUM_OF_SAMPLES] = {0};

// moving average filter output and status variable
volatile int16_t filteredSampleBuffer[NUM_OF_SAMPLES] = {0};
// 0 = inactive (default) ; 1 = active
static volatile uint8_t movAvgFilterActive = 0;

// overflowBuffer is needed to bridge the time until the DMA is disabled
// size is dependent on the number of possible transfers, before DMA is disabled in the ISR
volatile uint16_t overflowBuffer[1000] = {0};



/* FUNCTION DEFINITIONS */

/* app_Init()
 *
 * run all application initializations
 *	 													*/
void app_Init()
{
	__disable_irq();

	// peripheral initializations
	GPIO_Init();
	EXTI_Init();
	TIM3_Init();								// debounces button B1

	// determine all values for the addressArray
	uint32_t bufferSize = NUMBER_OF_ROWS*NUMBER_OF_COLUMNS*NUM_OF_TRANSFERS_PER_TRANSACTION;
	determineBufferAddresses(sampleBuffer, bufferSize, addressArray);
	DMA_Init(NUM_OF_TRANSFERS_PER_TRANSACTION);	// init value of NDTR: NUM_OF_TRANSFERS_PER_TRANSACTION

	TIM1_Init();								// creates clock-signal (default: freq=10MHz)

	__enable_irq();
}

void reInit_DMA()
{
	__disable_irq();
	DMA_Init(NUM_OF_TRANSFERS_PER_TRANSACTION);
	__enable_irq();
}

/* void setOnboardStatusLEDs(uint8_t status)
 *
 * uses onboard LEDs LD1...3 of NUCLEO-F767 for status display
 *
 * possible inputs: 0...7
 *
 * 0 is not recommended -> LEDs are off
 * (not distinguishable from error or invalid input)
 *	 													*/
void setOnboardStatusLEDs(uint8_t status)
{
	int ledMask = 0;
	// turn LD1..3 off
	GPIOB->ODR &= ~(LD1_Pin|LD2_Pin|LD3_Pin);
	switch(status)
	{
	case 0:
		break;
	case 1:
		ledMask = LD1_Pin;
		break;
	case 2:
		ledMask = LD2_Pin;
		break;
	case 3:
		ledMask = LD2_Pin|LD1_Pin;
		break;
	case 4:
		ledMask = LD3_Pin;
		break;
	case 5:
		ledMask = LD3_Pin|LD1_Pin;
		break;
	case 6:
		ledMask = LD3_Pin|LD2_Pin;
		break;
	case 7:
		ledMask = LD3_Pin|LD2_Pin|LD1_Pin;
		break;
	default:
		break;
	}
	GPIOB->ODR |= ledMask;
}

/* void configureTrigger(TRIGGERMODE_T triggerMode)
 *
 * configures the peripheral to set the sampling frequency
 * according to the variable newSamplingFrequency (value sent over from PC)
 *	 													*/
void configureSamplingFrequency()
{
	// disable TIM1
	TIM1->CR1 &= ~TIM_CR1_CEN;

	// ARR-value
	uint16_t TIM1_ticks = 0;

	// Timer-Clk: 100 MHz
	// Timer-Clk is set in TIM1_Init())

	// max. frequency: 10MHz (ADC DS p.4 - timing characteristics)
	if(newSamplingFrequency > MAX_FREQ_ADC)
	{
		newSamplingFrequency = MAX_FREQ_ADC;
	}
	// min. frequency: 20kHz (ADC DS p.4 - timing characteristics)
	else if(newSamplingFrequency < MIN_FREQ_ADC)
	{
		newSamplingFrequency = MIN_FREQ_ADC;
	}

	TIM1_ticks = (FREQ_TIMER / newSamplingFrequency) - 1;

	// set auto-reload register (ARR)
	TIM1->ARR &= ~(0xFFFF);
	TIM1->ARR |= TIM1_ticks;
	// set capture conmpare register
	TIM1->CCR1 &= ~(0xFFFF);
	TIM1->CCR1 |= (TIM1_ticks + 1) / 2;

	currentSamplingFrequency = FREQ_TIMER / (TIM1_ticks + 1);

	// enable TIM1
	TIM1->CR1 |= TIM_CR1_CEN;
}

/* void configureReferenceVoltage()
 *
 * configures the external DAC (LTC1450) to set the ADC reference voltage in mV
 * according to the variable newReferenceVoltage (value sent over from PC)
 *
 * µC[PE14:PE10, PE8:PE2] --> DAC[D11:D0]
 * PE0: /CLEAR
 * PE1: /LOAD
 * 														*/
void configureReferenceVoltage()
{
	// set LOAD=1 -> deactivate DAC-Latch (don't update DAC with value from data pins)
	GPIOE->ODR |= GPIO_ODR_OD1;

	// Vref = 4096mV - VDAC
	uint16_t DAC_val = 4095 - newReferenceVoltage;

	uint16_t DAC_ODR_val = 0, temp;

	// set bits DAC[D0:D6] ( µC[PE2:PE8] )
	temp = DAC_val & MASK_D0_D6;				// extract DAC[D0:D6]
	DAC_ODR_val |= (temp << 2);		// prepare assignment to µC[PE2:PE8]
	// set bits DAC[D7:D11] ( µC[PE10:PE14] )
	temp = (DAC_val & MASK_D7_D11) >> 7;			// extract DAC[D7:D11] and flush right
	DAC_ODR_val |= (temp << 10);		// prepare assignment to µC[PE10:PE14]

	// clear µC[PE2:PE8, PE10:PE14]
	GPIOE->ODR &= ~(GPIO_ODR_OD2|GPIO_ODR_OD3|GPIO_ODR_OD4|GPIO_ODR_OD5|GPIO_ODR_OD6|GPIO_ODR_OD7|GPIO_ODR_OD8|
				    GPIO_ODR_OD10|GPIO_ODR_OD11|GPIO_ODR_OD12|GPIO_ODR_OD13|GPIO_ODR_OD14);
	GPIOE->ODR |= DAC_ODR_val;					// assign DAC[D0:D11]

	currentReferenceVoltage = newReferenceVoltage;

	// set LOAD=0 -> activate DAC-Latch (update DAC with value from data pins)
	GPIOE->ODR &= ~GPIO_ODR_OD1;
}

/* void configureTrigger(TRIGGERMODE_T triggerMode)
 *
 * sets the variable for trigger mode (value sent over from PC)
 *	 													*/
void configureTrigger(TRIGGERMODE_T triggerMode)
{
	currentTriggerMode = triggerMode;

	if(currentTriggerMode == TRIGGER_RISING_EDGE)
	{
		EXTI->RTSR |= EXTI_RTSR_TR2;
		EXTI->FTSR &= ~EXTI_FTSR_TR2;
	}
	if(currentTriggerMode == TRIGGER_FALLING_EDGE)
	{
		EXTI->FTSR |= EXTI_FTSR_TR2;
		EXTI->RTSR &= ~EXTI_RTSR_TR2;
	}
}

/* void enableTrigger()
 *
 * configures the peripheral to enable the trigger
 *	 													*/
void enableTrigger()
{
	// unmask EXTI13 (trigger activated)
	EXTI->IMR |= EXTI_IMR_MR2;
}

/* void disableTrigger()
 *
 * configures the peripheral to disable the trigger
 *	 													*/
void disableTrigger()
{
	// unmask EXTI13 (trigger deactivated)
	EXTI->IMR &= ~(EXTI_IMR_MR2);
}

/* void manualTrigger()
 *
 * manually enable the DMA (doesn't wait for the trigger condition to be met)
 *	 													*/
void manualTrigger()
{
	/* enable DMA transfer from ADC parallel interface */
	// enable DMA2Stream5
	if(!(DMA2_Stream5->CR & DMA_SxCR_EN))	// only start, if DMA is currently disabled
			DMA2_Stream5->CR |= DMA_SxCR_EN;
}

/* void configureMovAvgFilter()
 *
 * sets the "number of points" variable for the moving average filter (value sent over from PC)
 *	 													*/
void configureMovAvgFilter()
{
	/* deactivate filter, if:
	* 1. normal deactivation (numOfPoints == 0)
	* 2. filter points > number of samples
	* 3. number of points are an even number
	* 											*/
	if( (newNumOfPoints == 0) || (newNumOfPoints > NUM_OF_SAMPLES) || ((newNumOfPoints % 2) != 1) )
	{
		movAvgFilterActive = 0;
		currentNumOfPoints = 0;
	}
	/* activate filter */
	else
	{
		movAvgFilterActive = 1;
		currentNumOfPoints = newNumOfPoints;
	}
}

/* void preprocessData()
 *
 * 1. takes raw ADC input data (see pin arrangement in GPIO_Init()) and calculates
 * 16-bit signed integer
 *
 * 2. extracts OF (overflow) information (if OF-bit was set at any moment, overflow happened)
 *	 													*/
void preprocessData()
{
	overflowHappened = 0x00;
	for(uint32_t i = 0; i < NUM_OF_SAMPLES; i++)
	{
		uint16_t tempDatapoint = sampleBuffer[i];
		// shift (see ADC->µC pin layout)
		tempDatapoint >>= 3;
		// check OF-bit (12th bit in data)
		if(tempDatapoint & (1 << 12))
			overflowHappened = 0xFF;
		// mask out OF-bit
		tempDatapoint &= 0xFFF;
		// if datapoint is negative reverse 2's complement
		if(((tempDatapoint >> 11) & 1) == 1)
		{
			// 1. minus 1
			tempDatapoint -= 1;
			// 2. invert all data-bits (12bit)
			tempDatapoint ^= 0xFFF;
			// 3. add negative sign
			sampleBuffer[i] = -tempDatapoint;
		}
		else
		{
			sampleBuffer[i] = tempDatapoint;
		}
	}

	// further preprocessing
	if(movAvgFilterActive == 1)
	{
		applyMovingAverageFilter();
	}
}

/* static void applyMovingAverageFilter()
 *
 * applies a moving average filter to the sampled data
 * input: int16_t sampleBuffer
 * output: int16_t filteredSampleBuffer
 *
 * from https://www.analog.com/media/en/technical-documentation/dsp-book/dsp_book_Ch15.pdf
 * p.2 TABLE 15-1
 *	 													*/
static void applyMovingAverageFilter()
{
	int32_t halfPointWidth = (int32_t)((currentNumOfPoints-1)/2);

	if(CUR_MOV_AVG_MODE == MOV_AVG_MODE_CUMULATIVE)
	{
		// numOfPoints must be an odd number (checked in configureMovAvgFilter())
		// odd number -> filter is applied
		for(uint32_t i = halfPointWidth; i <= NUM_OF_SAMPLES-(halfPointWidth+1); i++)
		{
			int32_t sum = 0;  // 32bit accumulator (16bit-int might overflow while accumulating)
			for(int32_t j = -halfPointWidth; j <= halfPointWidth; j++)	// signed integer (negative values)
			{
				sum += sampleBuffer[i+j];
			}
			// be aware of integer conversion (conversion rank of integer type is important for correct result)
			filteredSampleBuffer[i] = (int16_t)(sum / (int32_t)currentNumOfPoints);
		}
	}
	else if(CUR_MOV_AVG_MODE == MOV_AVG_MODE_RECURSIVE)
	{
		int32_t accumulator = 0;
		// first accumulation
		for(uint32_t i = 0; i <= currentNumOfPoints-1; i++)
		{
			accumulator += sampleBuffer[i];
		}
		filteredSampleBuffer[halfPointWidth] = (int16_t)(accumulator / (int32_t)currentNumOfPoints);

		// recursive calculation of rest values
		for(uint32_t i = (halfPointWidth+1); i <= (NUM_OF_SAMPLES - (halfPointWidth+1)); i++)
		{
			accumulator += (sampleBuffer[i+halfPointWidth] - sampleBuffer[i-(halfPointWidth+1)]);
			filteredSampleBuffer[i] = (int16_t)(accumulator / (int32_t)currentNumOfPoints);
		}
	}
}

/* --- COMMUNICATION --- */

/* void usbRxCallback(uint8_t* Buf, uint32_t *Len)
 *
 * callback function called by CDC_Receive_FS(...) in "usbd_cdc_if.h"
 *
 * receives a newline-terminated command
 *	 													*/
void usbRxCallback(uint8_t* Buf, uint32_t *Len)
{
	memcpy(usbRxBuf, Buf, *Len);
	usbRxBufLen = *Len;
	usbRxFlag = 1;
	receiveCommand();
}

/* void sendCommand(const char* command)
 *
 * takes the command string as an argument
 *
 * sends a newline-terminated command
 *	 													*/
void sendCommand(const char* command)
{
    // format command into newline-terminated string
    int len = snprintf(cmdTxBuffer, CMD_TX_BUFFER_SIZE, "%s\n", command);

    // only send if length is valid (safety)
    if (len > 0 && len < CMD_TX_BUFFER_SIZE) {
        CDC_Send((uint8_t*)cmdTxBuffer, len);
    }
}

/* void sendCurrentSamplingFrequency()
 *
 * sends a confirmation string, that shows the real sampling frequency
 * currently active
 *	 													*/
void sendCurrentSamplingFrequency()
{
	char tempStr1[] = "SAMPLING_FREQUENCY: ";
	char tempStr2[20];
	sprintf(tempStr2, "%u", currentSamplingFrequency);
	sendCommand(strncat(tempStr1, tempStr2, 20));
}

/* void sendCurrentReferenceVoltage()
 *
 * sends a confirmation string, that shows the real ADC reference voltage
 * currently active
 *	 													*/
void sendCurrentReferenceVoltage()
{
	char tempStr1[] = "REFERENCE_VOLTAGE: ";
	char tempStr2[20];
	sprintf(tempStr2, "%u", currentReferenceVoltage);
	sendCommand(strncat(tempStr1, tempStr2, 20));
}

/* void sendCurrentTriggerMode()
 *
 * sends a confirmation string, that shows the real trigger mode
 * currently active
 *	 													*/
void sendCurrentTriggerMode()
{
	if(currentTriggerMode == TRIGGER_RISING_EDGE)
		sendCommand("TRIGGER:RISING_EDGE");
	if(currentTriggerMode == TRIGGER_FALLING_EDGE)
		sendCommand("TRIGGER:FALLING_EDGE");
}

/* void sendCurrentNumOfPoints()
 *
 * sends a confirmation string, that shows the real number of points
 * currently used for the moving average filter
 *
 * if number of points is 0, filter is not active
 *	 													*/
void sendCurrentNumOfPoints()
{
	char tempStr1[] = "MOVING_AVERAGE: ";
	char tempStr2[20];
	sprintf(tempStr2, "%u", currentNumOfPoints);
	sendCommand(strncat(tempStr1, tempStr2, 20));
}

/* void sendData()
 *
 * 1. sends the sampling data as a series of bytes (data is 2 bytes long, LSB is sent first)
 *
 * 2. after the sampling data, the overflow (OF) data is sent as a single byte:
 * 	- 0xFF: overflow
 * 	- 0x00: no overflow
 *	 													*/
void sendData()
{
	int remainingBytes = NUM_OF_SAMPLES*2;
	int numberOfBytes = MAX_TRANSFER_BYTELENGTH;
	int idxSampleNumber = 0;

	int16_t(*dataSource)[NUM_OF_SAMPLES];

	// check if filter is active
	if(movAvgFilterActive == 1)
		dataSource = &filteredSampleBuffer;
	else
		dataSource = &sampleBuffer;

	// send data in portions of <MAX_TRANSFER_BYTELENGTH> bytes
	while(remainingBytes > 0)
	{
		if(remainingBytes < MAX_TRANSFER_BYTELENGTH)
			numberOfBytes = remainingBytes;
		else
			numberOfBytes = MAX_TRANSFER_BYTELENGTH;

		remainingBytes -= numberOfBytes;

		CDC_Send((uint8_t*)(&((*dataSource)[idxSampleNumber])), numberOfBytes);
		// prepare index for next transfer
		// (next <MAX_TRANSFER_BYTELENGTH> bytes = <MAX_TRANSFER_BYTELENGTH / 2> sample values)
		idxSampleNumber += MAX_TRANSFER_BYTELENGTH/2;
	}
	// send overflow information byte
	CDC_Send((uint8_t*)(&overflowHappened), 1);
}

/* void CDC_Send(uint8_t* data, uint16_t len)
 *
 * safely uses CDC_Transmit_FS(...)
 *	 													*/
static void CDC_Send(uint8_t* data, uint16_t len)
{
	while (CDC_Transmit_FS(data, len) == USBD_BUSY)
	{
		// wait until USB stack is ready
	}
}

static void receiveCommand()
{
	// wait for command reception
	while(usbRxFlag == 0) {}

	for (uint32_t i = 0; i < usbRxBufLen; i++)
	{
		uint8_t ch = usbRxBuf[i];

		// check for termination character
		if (ch == '\n' || ch == '\r') {
			if (cmdRxIndex > 0) {
			cmdRxBuffer[cmdRxIndex] = '\0'; // NULL-terminate

			// process the received command:
			processCommand((char*)cmdRxBuffer);

			cmdRxIndex = 0; // reset buffer
			}
		}
		else
		{
			if (cmdRxIndex < CMD_RX_BUFFER_SIZE - 1)
			{
				// insert character into buffer
				cmdRxBuffer[cmdRxIndex++] = ch;
			}
			else
			{
				// buffer overflow -> reset
				cmdRxIndex = 0;
			}
		}
	}

	usbRxFlag = 0;
	usbRxBufLen = 0;
}

/* static void processCommand(const char* cmd)
 *
 * receives NULL-terminated (\0) string with the
 * corresponding command sent to the µC from host (PC)
 *	 													*/
static void processCommand(const char* cmd)
{
    if (strcmp(cmd, "RESET") == 0) {
    	fsm_add_event(EV_RESET);
    }
	else if (strcmp(cmd, "APP") == 0) {
    	fsm_add_event(EV_START_APP);
    }
    else if (strncmp(cmd, "CONFIGURE_FREQ <FREQ_IN_HZ>", 14) == 0) {
    	sscanf (cmd,"%*s %u", &newSamplingFrequency);
    	fsm_add_event(EV_CONFIGURE_FREQ);
    }
	else if (strncmp(cmd, "CONFIGURE_REF_VOLT <VOLTAGE_IN_MV>", 18) == 0) {
		sscanf (cmd,"%*s %u", &newReferenceVoltage);
		fsm_add_event(EV_CONFIGURE_REF_VOLT);
    }
    else if (strcmp(cmd, "CONFIGURE_TRIG_RISE_EDGE") == 0) {
    	fsm_add_event(EV_CONFIGURE_TRIG_RISE_EDGE);
    }
    else if (strcmp(cmd, "CONFIGURE_TRIG_FALL_EDGE") == 0) {
    	fsm_add_event(EV_CONFIGURE_TRIG_FALL_EDGE);
    }
	else if (strncmp(cmd, "CONFIGURE_MOV_AVG <NUM_OF_POINTS>", 17) == 0) {
		sscanf (cmd,"%*s %u", &newNumOfPoints);
		fsm_add_event(EV_CONFIGURE_MOV_AVG);
    }
    else if (strcmp(cmd, "RUN_START") == 0) {
    	fsm_add_event(EV_SAMPL_RUN_START);
    }
    else if (strcmp(cmd, "READY_FOR_DATA") == 0) {
        	fsm_add_event(EV_SAMPL_RDY_FOR_DATA);
    }
    else if (strcmp(cmd, "RECEIVED") == 0) {
    	fsm_add_event(EV_SAMPL_RECEIVED);
    }
    else if (strcmp(cmd, "CONTINUE") == 0) {
    	fsm_add_event(EV_SAMPL_ACK_CONTINUE);
    }
    else if (strcmp(cmd, "STOP") == 0) {
    	fsm_add_event(EV_SAMPL_ACK_STOP);
    }
    else {
        const char* error = "ERR:UNKNOWN_CMD\n";
        CDC_Transmit_FS((uint8_t*)error, strlen(error));
    }
}

/* --- SYSTEM INITIALIZATION --- */

/* static void GPIO_Init()
 *
 * inititalization of the used GPIOs
 *	 													*/
static void GPIO_Init()
{
	/* GPIO for button to manually start sampling
	 * PC13: USR_Btn B1 (NUCLEO-F767)*/
	/* enable clock for GPIOC */
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOCEN;
	// USR_Btn->input (MODE: 0b00)
	GPIOC->MODER &= ~GPIO_MODER_MODER13_Msk;


	/* GPIO for onboard status LEDs (LD1..3 of NUCLEO-F767)
	 * PB0:  LD1 (green)
	 * PB7:  LD2 (blue)
	 * PB14: LD3 (red) */
	/* enable clock for GPIOB */
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN;
	// turn LD1..3 off
	GPIOB->ODR &= ~(LD1_Pin|LD2_Pin|LD3_Pin);
	// LD1..3->output (MODE: 0b01)
	GPIOB->MODER &= ~(GPIO_MODER_MODER0_Msk|GPIO_MODER_MODER7_Msk|GPIO_MODER_MODER14_Msk);
	GPIOB->MODER |= (GPIO_MODER_MODER0_0|GPIO_MODER_MODER7_0|GPIO_MODER_MODER14_0);


	/* --- ADC-GPIOs --- */

	/* GPIOs for ADC parallel interface
	 * ADC[D11:D0] --> µC[PF14:PF3]	*/
	/* enable clock for GPIOF */
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOFEN;
	// PF3...14->input (MODE: 0b00)
	GPIOF->MODER &= ~(GPIO_MODER_MODER3_Msk|GPIO_MODER_MODER4_Msk|GPIO_MODER_MODER5_Msk|GPIO_MODER_MODER6_Msk|
					  GPIO_MODER_MODER7_Msk|GPIO_MODER_MODER8_Msk|GPIO_MODER_MODER9_Msk|GPIO_MODER_MODER10_Msk|
					  GPIO_MODER_MODER11_Msk|GPIO_MODER_MODER12_Msk|GPIO_MODER_MODER13_Msk|GPIO_MODER_MODER14_Msk);

	// assign pull-downs for defined value of pins
	GPIOF->PUPDR &= ~(0xFFFFFFFF);
	GPIOF->PUPDR |= GPIO_PUPDR_PUPDR14_0|GPIO_PUPDR_PUPDR13_1|GPIO_PUPDR_PUPDR12_0|
					GPIO_PUPDR_PUPDR11_1|GPIO_PUPDR_PUPDR10_0|GPIO_PUPDR_PUPDR9_1|
					GPIO_PUPDR_PUPDR8_0|GPIO_PUPDR_PUPDR7_1|GPIO_PUPDR_PUPDR6_0|
					GPIO_PUPDR_PUPDR5_1|GPIO_PUPDR_PUPDR4_0|GPIO_PUPDR_PUPDR3_1;

	GPIOF->PUPDR |= GPIO_PUPDR_PUPDR15_1|GPIO_PUPDR_PUPDR2_1|GPIO_PUPDR_PUPDR1_1|GPIO_PUPDR_PUPDR0_1;

	/* GPIO for ADC Overflow Flag
	 * ADC[OF_BIT] --> µC[PF15]	 	*/
	/* enable clock for GPIOF */
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOFEN;
	// PF15->input (MODE: 0b00)
	GPIOF->MODER &= ~(GPIO_MODER_MODER15_Msk);

	/* GPIO for Trigger
	 * TRIGGERED_SIG_OUT --> µC[PF2]*/
	/* enable clock for GPIOF */
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOFEN;
	// PF2->input (MODE: 0b00)
	GPIOF->MODER &= ~(GPIO_MODER_MODER2_Msk);


	/* GPIO for TIMer output compare event
	 * PE9: TIM1_CH1 (AF1)
	 * clock for ADC, triggers DMA2 channel 6 stream 5 transfers*/
	/* enable clock for GPIOE */
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOEEN;
	// PE9->alternate function (MODE: 0b10)
	GPIOE->MODER &= ~GPIO_MODER_MODER9_Msk;
	GPIOE->MODER |= GPIO_MODER_MODER9_1;
	// output speed (very high speed: 0b11)
	GPIOE->OSPEEDR |= (GPIO_OSPEEDR_OSPEEDR9_1|GPIO_OSPEEDR_OSPEEDR9_0);
	// select alternate function AF1 of PE9 (TIM1_CH1)
	GPIOE->AFR[1] &= ~(GPIO_AFRH_AFRH1);
	GPIOE->AFR[1] |= GPIO_AFRH_AFRH1_0;


	/* --- DAC-GPIOs --- */

	/* GPIOs for DAC parallel interface
	 * µC[PE14:PE10, PE8:PE2] --> DAC[D11:D0]	*/
	/* enable clock for GPIOE */
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOEEN;
	// PE2...8 & PE10...14->output (MODE: 0b01)
	GPIOE->MODER &= ~(GPIO_MODER_MODER2_Msk|GPIO_MODER_MODER3_Msk|GPIO_MODER_MODER4_Msk|GPIO_MODER_MODER5_Msk|
					  GPIO_MODER_MODER6_Msk|GPIO_MODER_MODER7_Msk|GPIO_MODER_MODER8_Msk|GPIO_MODER_MODER10_Msk|
					  GPIO_MODER_MODER11_Msk|GPIO_MODER_MODER12_Msk|GPIO_MODER_MODER13_Msk|GPIO_MODER_MODER14_Msk);
	GPIOE->MODER |=  (GPIO_MODER_MODER2_0|GPIO_MODER_MODER3_0|GPIO_MODER_MODER4_0|GPIO_MODER_MODER5_0|
					  GPIO_MODER_MODER6_0|GPIO_MODER_MODER7_0|GPIO_MODER_MODER8_0|GPIO_MODER_MODER10_0|
					  GPIO_MODER_MODER11_0|GPIO_MODER_MODER12_0|GPIO_MODER_MODER13_0|GPIO_MODER_MODER14_0);
	// open-drain outputs (OT=1), external pull-up are used (3V3 -> 5V)
	GPIOE->OTYPER |= (GPIO_OTYPER_OT2|GPIO_OTYPER_OT3|GPIO_OTYPER_OT4|GPIO_OTYPER_OT5|GPIO_OTYPER_OT6|GPIO_OTYPER_OT7|
					  GPIO_OTYPER_OT8|GPIO_OTYPER_OT10|GPIO_OTYPER_OT11|GPIO_OTYPER_OT12|GPIO_OTYPER_OT13|GPIO_OTYPER_OT14);

	/* GPIOs for DAC CLEAR and LOAD
	 * PE0: /CLEAR
	 * PE1: /LOAD 		*/
	/* enable clock for GPIOE */
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOEEN;
	// PE0...1->output (MODE: 0b01)
	GPIOE->MODER &= ~(GPIO_MODER_MODER0_Msk|GPIO_MODER_MODER1_Msk);
	GPIOE->MODER |= (GPIO_MODER_MODER0_0|GPIO_MODER_MODER1_0);
	// open-drain outputs (OT=1), external pull-up are used (3V3 -> 5V)
	GPIOE->OTYPER |= (GPIO_OTYPER_OT0|GPIO_OTYPER_OT1);
	// set /LOAD=0 and /CLEAR=1 -> DAC-Output=0V -> Vref(ADC)=4.096V
	GPIOE->ODR &= ~(GPIO_ODR_OD1);	// /LOAD=0
	GPIOE->ODR |= GPIO_ODR_OD0;		// /CLEAR=1

}

/* static void EXTI_Init()
 *
 * initialization and configuration of EXTI-modules
 *	 													*/
static void EXTI_Init()
{
	// enable SYSCFG-clock
	RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN;

	/* EXTI13 Configuration for USR_Btn B1 (PC13) */
	// SysCfg configuration EXTI13->PC13
	SYSCFG->EXTICR[3] |= SYSCFG_EXTICR4_EXTI13_PC;
	// unmask EXTI13
	EXTI->IMR |= EXTI_IMR_MR13;
	// enable rising edge detection for EXTI13 (USR_Btn is high-active)
	EXTI->RTSR |= EXTI_RTSR_TR13;
	// NVIC IRQ config for EXTI 10-15
	NVIC_SetPriority(EXTI15_10_IRQn, 15);
	NVIC_ClearPendingIRQ(EXTI15_10_IRQn);
	NVIC_EnableIRQ(EXTI15_10_IRQn);

	/* EXTI2 Configuration for Trigger Signal */
	// SysCfg configuration EXTI2->PF2
	SYSCFG->EXTICR[0] |= SYSCFG_EXTICR1_EXTI2_PF;
	// mask EXTI13 (deacivated - trigger is activated in sampling state)
	EXTI->IMR &= ~(EXTI_IMR_MR2);
	// edge detection for EXTI2 is configured in sampling state)
	EXTI->RTSR |= EXTI_RTSR_TR2;
	// EXTI->FTSR |= EXTI_FTSR_TR2;
	// NVIC IRQ config for EXTI2
	NVIC_SetPriority(EXTI2_IRQn, 15);
	NVIC_ClearPendingIRQ(EXTI2_IRQn);
	NVIC_EnableIRQ(EXTI2_IRQn);


}

/* static void TIM3_Init()
 *
 * initalization of TIM3
 * TIM3 used for debouncing USR-Btn
 *	 													*/
static void TIM3_Init()
{
	// enable TIM3-clock
	RCC->APB1ENR |= RCC_APB1ENR_TIM3EN;

	// set clock with prescaler to clock_cnt=clock_in/(PSC+1)
	uint16_t prescaleValueTIM3 = 49999;
	TIM3->PSC |= prescaleValueTIM3;
	// APB1-TimerClk is 100MHz -> 100MHz/(49999+1)=2kHz-> Timer-Clock is 2kHz

	// set auto-reload value
	TIM3->ARR &= ~(0xFFFFFFFF);
	uint16_t TIM3_ticks = 10;
	TIM3->ARR |= TIM3_ticks;      // count time: 5ms

	// enable update interrupt-flag
	TIM3->DIER |= (TIM_DIER_UIE);

	// activate upward counter (default) and one-pulse mode
	TIM3->CR1 |= (TIM_CR1_OPM);

	// NVIC IRQ config
	NVIC_SetPriority(TIM3_IRQn, 1);
	NVIC_ClearPendingIRQ(TIM3_IRQn);
	NVIC_EnableIRQ(TIM3_IRQn);
}

/* static void DMA_Init(uint32_t numOfTransfers)
 *
 * initalization of DMA2 channel 6, stream 5
 *	 													*/
static void DMA_Init(uint32_t numOfTransfers)
{
	/* DMA for transferring data between GPIOF_IDR (input data register) and SRAM
	 * also triggered by TIMer
	 * used DAM-Controller: DMA2
	 * channel: 6
	 * stream: 5 (TIM1_UP)*/

	// enable DMA2 clock
	RCC->AHB1ENR |= RCC_AHB1ENR_DMA2EN;

	/* following procedure in RM p.267 */

	// 1. disable stream2 of DMA2 & clear all stream-dedicated status-bits
	DMA2_Stream5->CR &= ~(DMA_SxCR_EN);
	while(DMA2_Stream5->CR & DMA_SxCR_EN) {}
	DMA2->HIFCR |= (DMA_HIFCR_CTCIF5|DMA_HIFCR_CHTIF5|DMA_HIFCR_CTEIF5|DMA_HIFCR_CDMEIF5|DMA_HIFCR_CFEIF5);

	// 2. set peripheral port (periph2mem: source) address
	// source address is GPIOF_IDR
	DMA2_Stream5->PAR = (uint32_t)&GPIOF->IDR;

	// 3. set memory (periph2mem: destination) addresses
	// initialized with entries of the first row of addressArray
	uint32_t initialM0AR = addressArray[0][0];
	uint32_t initialM1AR = addressArray[0][1];
	DMA2_Stream5->M0AR = initialM0AR;
	DMA2_Stream5->M1AR = initialM1AR;

	// 4. configure total number of transferred items
	DMA2_Stream5->NDTR &= ~(0xFFFF);
	DMA2_Stream5->NDTR |= numOfTransfers;

	// 5. select DMA channel 6
	DMA2_Stream5->CR &= ~(DMA_SxCR_CHSEL);
	DMA2_Stream5->CR |= DMA_SxCR_CHSEL_2|DMA_SxCR_CHSEL_1;

	// 6. (only if peripheral=flow-controller)

	// 7. configure stream priority (0b11:highest - 0b00:lowest)
	DMA2_Stream5->CR &= ~(DMA_SxCR_PL);
	DMA2_Stream5->CR |= (DMA_SxCR_PL_1|DMA_SxCR_PL_0);	// highest priority

	// 8. configure FIFO usage (default: direct mode (DMDIS=0))
	DMA2_Stream5->FCR &= ~(DMA_SxFCR_DMDIS);

	// 9.
	// data transfer direction (0b00: periph2mem)
	DMA2_Stream5->CR &= ~(DMA_SxCR_DIR);
	//DMA2_Stream5->CR |= 0;

	// peripheral (source) increment mode (PINC=0: no increment - fixed address)
	DMA2_Stream5->CR &= ~(DMA_SxCR_PINC);

	// memory (destination) increment mode (MINC=1: according to MSIZE)
	DMA2_Stream5->CR |= DMA_SxCR_MINC;

	// XSIZE[1:0]: (0b00: byte, 0b01: half-word, 0b10: word)
	// peripheral (source) data size
	DMA2_Stream5->CR &= ~(DMA_SxCR_PSIZE);
	DMA2_Stream5->CR |= (DMA_SxCR_PSIZE_0);	// half-word

	// memory (destination) data size
	DMA2_Stream5->CR &= ~(DMA_SxCR_MSIZE);
	DMA2_Stream5->CR |= (DMA_SxCR_MSIZE_0);	// half-word

	/* enable double-buffer mode (circular mode is automatically enabled)
	* after one transaction M0AR and M1AR are switched as the destination address
	* while M0AR is active, M1AR can be changed (and vice versa)
	* CR_CT=0 -> M0AR used;	CR_CT=1 -_>M1AR used	(CT: current target) */
	DMA2_Stream5->CR |= (DMA_SxCR_DBM);

	// enable transfer error interrupt
	DMA2_Stream5->CR |= DMA_SxCR_TCIE;
	// enable transfer complete interrupt
	DMA2_Stream5->CR |= DMA_SxCR_TEIE;

	// configure NVIC for DMA2 stream5 interrupts
	NVIC_SetPriority(DMA2_Stream5_IRQn, 0);
	NVIC_ClearPendingIRQ(DMA2_Stream5_IRQn);
	NVIC_EnableIRQ(DMA2_Stream5_IRQn);

	// DMA2 stream 5 is enabled, when trigger condition is met
	// in file "stm32f7xx_it.c" function: EXTI2_IRQHandler()
}
/* static int determineBufferAddresses(uint16_t buf[], uint32_t bufSize, uint32_t* addressArray[][NUMBER_OF_COLUMNS])
 * determines all addresses needed in the DMA transfers for the complete buffer
 * returns the addresses in a <numOfElements>x2-Array (address is zero, if invalid)
 * used for the memory pointers M0AR (first column) and M1AR (second column) of DMA2
 */
static int determineBufferAddresses(uint16_t buf[], uint32_t bufSize, uint32_t* addressArray[][NUMBER_OF_COLUMNS])
{
	uint32_t index = 0;
	uint32_t addressArraySize = NUMBER_OF_ROWS*NUMBER_OF_COLUMNS;
	uint32_t partSize = bufSize/(addressArraySize);
	// if the buffer can't be separated into equal even parts -> error
	if(bufSize%(addressArraySize) != 0)
		return 1;
	for(int i = 0; i < NUMBER_OF_ROWS; i++)
	{
		for(int j = 0; j < NUMBER_OF_COLUMNS; j++)
		{
			addressArray[i][j] = (uint32_t*)&buf[index];
			index += partSize;
		}
	}
	return 0;
}

/* static void TIM1_Init()
 *
 * initialization of TIM1
 * TIM1 used for triggering DMA transfers
 * and creating a clock signal on PE9 (both are synchronous)
 * transmission is done on falling edge of clock signal (avoid setup-hold-violation)
 *
 *	 													*/
static void TIM1_Init()
{
	/* create ADC-Clock (10MHz) at OC-Pin PE9 (TIM1_CH1)
	 * used timer: TIM1 (APB2)
	 * clock: APB2 Timer Clock: 100MHz */
	// enable TIM1-clock
	RCC->APB2ENR |= RCC_APB2ENR_TIM1EN;

	// set clock with prescaler to clock_cnt=clock_in/(PSC+1)
	uint16_t prescaleValueTIM1 = 0;
	TIM1->PSC = prescaleValueTIM1;
	// APB1-TimerClk is 100MHz -> 100MHz/(0+1)=100MHz-> Timer-Clock is 100MHz

	// set auto-reload value
	TIM1->ARR &= ~(0xFFFF);
	uint16_t TIM1_ticks = (FREQ_TIMER / INIT_FREQ_ADC) - 1;
	TIM1->ARR |= TIM1_ticks; 	// period: (9+1)ticks = 100ns
	// set Capture/Compare register value
	TIM1->CCR1 &= ~(0xFFFF);
	TIM1->CCR1 |= (TIM1_ticks + 1) / 2; 		// HIGH for 50ns (duty-cycle=50%)
	// capture mode enabled  and configure channel as output (polarity: active low !!!)
	TIM1->CCER |= TIM_CCER_CC1E|TIM_CCER_CC1P;
	// PWM mode 1 (active: CNT<CCR1, else inactive) (mode: 0b0110)
	TIM1->CCMR1 &= ~(TIM_CCMR1_OC1M);
	TIM1->CCMR1 |= TIM_CCMR1_OC1M_2|TIM_CCMR1_OC1M_1;
	// CC1 channel configured as output (0b00)
	TIM1->CCMR1 &= ~(TIM_CCMR1_CC1S);

	// Main output enable (enables OC1)
	TIM1->BDTR |= TIM_BDTR_MOE;

	// enable update dma request (used for transferring GPIOG_IDR->SRAM)
	TIM1->DIER |= (TIM_DIER_UDE);

	// enable TIM1
	TIM1->CR1 |= TIM_CR1_CEN;
}
