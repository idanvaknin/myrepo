/*
 * Copyright (c) 2015, Freescale Semiconductor, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * o Redistributions of source code must retain the above copyright notice, this list
 *   of conditions and the following disclaimer.
 *
 * o Redistributions in binary form must reproduce the above copyright notice, this
 *   list of conditions and the following disclaimer in the documentation and/or
 *   other materials provided with the distribution.
 *
 * o Neither the name of Freescale Semiconductor, Inc. nor the names of its
 *   contributors may be used to endorse or promote products derived from this
 *   software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 * ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

//***************************************************************************
//	RHP_FULL_TASKS_2.8
//***************************************************************************
// Idan Vaknin
//************************
// 2.8V 16/04/18
//************************

/* FreeRTOS kernel includes. */
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "timers.h"
#include "MK64F12.h"
#include "semphr.h"

#include "fsl_port.h"
#include "fsl_gpio.h"
#include "fsl_common.h"

/* Freescale includes. */
#include "fsl_device_registers.h"
#include "fsl_debug_console.h"
#include "fsl_dspi.h"
#include "board.h"

#include "fsl_edma.h"
#include "fsl_dmamux.h"

#include "fsl_rnga.h"
#include "fsl_pit.h"

#include "fsl_uart_freertos.h"
#include "fsl_uart.h"

#include "pin_mux.h"
#include "clock_config.h"

#include "stdbool.h"
#include "time.h"
#include "stdio.h"
#include "stdlib.h"
#include "math.h"

//#include "Functions.c"
/*******************************************************************************
 * Definitions
 ******************************************************************************/
/* UART instance and clock */
#define DEMO_UART 							UART0
#define DEMO_UART_CLKSRC 					UART0_CLK_SRC
#define DEMO_UART_RX_TX_IRQn 				UART0_RX_TX_IRQn
#define MAX_MESSAGE_LENGTH					234

#define PIT_100USEC_HANDLER 				PIT0_IRQHandler
#define PIT_100_USEC_IRQ_ID 				PIT0_IRQn
#define PIT_10SEC_HANDLER 					PIT1_IRQHandler
#define PIT_10_SEC_IRQ_ID 					PIT1_IRQn
#define PIT_60SEC_HANDLER 					PIT2_IRQHandler
#define PIT_60_SEC_IRQ_ID 					PIT2_IRQn
/* Get source clock for PIT driver */
#define PIT_SOURCE_CLOCK 					CLOCK_GetFreq(kCLOCK_BusClk)

#define LD_GPIO 							LD_SYNTH_GPIO						
#define LD_PORT								LD_SYNTH_PORT						
#define LD_GPIO_PIN 						LD_SYNTH_GPIO_PIN
#define LD_IRQ 								LD_SYNTH_IRQ
#define LD_IRQ_HANDLER 						LD_SYNTH_IRQ_HANDLER

#define EXAMPLE_DSPI_MASTER_BASEADDR		SPI0
#define EXAMPLE_DSPI_MASTER_CLK_SRC 		DSPI0_CLK_SRC
#define EXAMPLE_DSPI_MASTER_IRQ 			SPI0_IRQn
#define EXAMPLE_DSPI_MASTER_PCS 			kDSPI_Pcs0
#define EXAMPLE_DSPI_MASTER_IRQHandler 		SPI0_IRQHandler

#define TRANSFER_BAUDRATE 					500000U 							//! Transfer baud rate - 500k (SPI).

#define DEMO_ADC16_IRQn 					ADC0_IRQn
#define DEMO_ADC16_BASE 					ADC0
#define DEMO_ADC16_CHANNEL_GROUP 			0U
#define DEMO_ADC16_IRQ_HANDLER_FUNC 		ADC0_IRQHandler

#define A2D0_16BIT_470KHZ 					0									
#define A2D0_16BIT_470KHZ_CONTINUOUS 		1									
#define A2D0_16BIT_470HZ_AVG32	 			2									
#define A2D0_16BIT_470KHZ_AVG32_CONTINUOUS	3									
#define A2D1_16BIT_470KHZ_AVG32_CONTINUOUS	4								

#define SPI0_8BIT	  						0									
#define SPI0_16BIT	  						1									

#define DCA1								1
#define DCA2								2
#define DCA3								3

#define NUM_OF_DMA_TRANS					40
#define DMA_BUFFER_SIZE						20
#define NUM_OF_PARAM						6
#define NUM_OF_CYCLES_TO_UPDATE				5									
#define NUM_OF_BARS_INPUT_SPECTRUM			100									
#define NUM_OF_BARS_OUTPUT_SPECTRUM			300									
#define INPUT_SPECTRUM_THRESHOLD   			10000								
#define NUM_OF_GD_CYCLES_TO_INPUT_SWEEP     10									
#define NUM_OF_ERROR_POINT_TO_SWEEP			20									

#define D_1_MV								1									
#define D_10_MV								8								
#define D_100_MV							81									
#define D_1000_MV							819									

#define VVA									0									// Parameter 1.
#define AMP_EQ_1							1									// Parameter 2.
#define AMP_EQ_2							2									// Parameter 3.
#define RIPPLE_EQ_1							3									// Parameter 4.
#define RIPPLE_EQ_2							4									// Parameter 5.
#define PHASE_SHIFTER						5									// Parameter 6.

#define VVA_B								6									// Parameter 7.
#define AMP_EQ_1_B							7									// Parameter 8.
#define AMP_EQ_2_B							8									// Parameter 9.
#define RIPPLE_EQ_1_B						9									// Parameter 10.
#define RIPPLE_EQ_2_B						10									// Parameter 11.
#define PHASE_SHIFTER_B						11									// Parameter 12.

#define SEND_1000_RANDOM    				10
#define SIMULATED_ANNEALING 				4
#define GENETIC_ALGORITHM					5
#define PSO									6
#define START_MODE							7
#define GRADIENT_DESCENT 					0
#define WAIT_TO_START						3
#define DAC_WORK							8

//#define MAKE_1000_RANDOM														
#define OPERATION_MODE
#define IF_SAW_FILTER 870
#define RF_IN 1
#define RF_OUT 0

#define CYCLE_TO_TEMP_DECCAY 				100
#define GENETIC_ALG_POP 					100									// Number of elements in algorithm population.
#define PSO_NUM_OF_PARTICLES				100
#define GA_POP_POOL							4200								// The size of the array from which we draw a parent.
#define MUTATION_PROBABILITY				0.00								// Determine the chance of mutation in a "gene".

/* Task priorities. */
#define uart_task_PRIORITY 					(configMAX_PRIORITIES - 1)			// Max priority declared in FreeRTOSConfig.h as 32.

#define NUM_OF_CALIB_FREQ					22
#define F_2_50_GHZ							0
#define F_2_52_GHZ							1
#define F_2_54_GHZ							2
#define F_2_56_GHZ							3
#define F_2_58_GHZ							4
#define F_2_60_GHZ							5
#define F_2_62_GHZ							6
#define F_2_64_GHZ							7
#define F_2_66_GHZ							8
#define F_2_68_GHZ							9
#define F_2_70_GHZ							10
#define OF_2_50_GHZ							11
#define OF_2_52_GHZ							12
#define OF_2_54_GHZ							13
#define OF_2_56_GHZ							14
#define OF_2_58_GHZ							15
#define OF_2_60_GHZ							16
#define OF_2_62_GHZ							17
#define OF_2_64_GHZ							18
#define OF_2_66_GHZ							19
#define OF_2_68_GHZ							20
#define OF_2_70_GHZ							21

/*******************************************************************************
 * Prototypes
 ******************************************************************************/

static void sample_task(void *pvParameters);									// This TASK responsible of setting up the system and take measurements.
//static void analyze_and_set(void *pvParameters);								// This TASK responsible of deciding what is the best step to make and setting it up.
static void uart_send_task(void *pvParameters);										// This TASK responsible of handle the uart send communication.
static void uart_recv_task(void *pvParameters);										// This TASK responsible of handle the uart receive communication.

void Config_A2D_moudule(int configSet);											
uint16_t make_average_from_adc0(void);											
uint16_t make_average_from_adc1(void);											
void set_spi_to_dac(int configSet);												
uint16_t initate_adc0_sample(void);												//Sample the power detector - adc0 (error)
uint16_t initate_adc1_sample(void);												
void update_dac(uint16_t DacVoltage, dspi_command_data_config_t *command, uint8_t DacChannel);	
void initiate_dac(dspi_command_data_config_t *command);
void init_GPIO();
void check_and_save_last_100(void);												
void sample_gradient_descent(void);

void scan_and_save(void);
void set_and_save_random_solution(void);										
void gradient_descent_analyze_and_set(void);								//Analyze and set the selected value of the DAC - gradient decent first loop
void alg1_analyze_and_set(void); 												
void alg2_analyze_and_set(void);												
void alg3_analyze_and_set(void);												
void simulated_annealing_analyze_and_set(void);									
void translate_GUI_message(uint8_t GuiMessage[]);								
void main_spi_config(dspi_master_config_t *masterConfig);						
void dac_initialization(void);													
void edma_initialization(void);												
void set_up_random_genetic_algorithm(void);										
void genetic_algorithm_analyze_and_set(void);									
void sample_genetic_algorithm(void);											
void generate_pso_random_solution(void);										
void sample_pso(void);															
void pso_analyze_and_set(void);													
void enqueue(uint16_t InData);													
void dequeue(void);																
void synthesizer_initialization(void);											
void udate_synthesizer(uint16_t SynthValue, dspi_command_data_config_t *command, uint16_t ControlBit);	
void swep_input_spectrum(void);
void swep_output_spectrum(void);
void sample_out_spectrum(void);
void locate_output_spectrum_error_points(void);
void analyze_input_spectrum(void);
void update_dca(uint8_t DcaValue, dspi_command_data_config_t *command, uint8_t DcaNumber);
void gd_second_loop_sample(void);
uint32_t clac_second_loop_price_func(void);
void gd_second_loop_aas(void);
void spi_Sync_send_data(uint16_t DataToSend1,uint16_t DataToSend2);
void send_calib_data(uint16_t data, uint8_t freq , bool IsRawvalue1);
void swep_output_full_spectrum(void);
void sample_power_output_spectrum(void);
void send_input_block_locations(void);

void NEW_second_loop_price_func(void);
void sample_NEW_gd_second_loop(void);

void send_to_pc(uint8_t message_to_send[], uint8_t message_type);
void sw1_RF(uint8_t mode);


/*******************************************************************************
 * Variables
 ******************************************************************************/

uint8_t send_buffer[MAX_MESSAGE_LENGTH] = {0};													// UART send buffer.
uint8_t* pToSend_buffer = send_buffer;											// Pointer to UART send buffer.

/* STATES */
enum START_STATE {MODE1, MODE2, MODE3, MODE4, CALIB};
enum GB_STATE {RUN2, RUN1, RUN3, RUN4};

/* Message_Type */
enum Message_Type {INSPEC, OUTSPC, INPOWR, CCCCCC, GRADEN, CALIBR};
uint8_t message_type = 0;

/*******************************************************************************
* Queue
******************************************************************************/
/* messageQ handle*/
static QueueHandle_t message_queue = NULL;


uint8_t recv_buffer[4];															// UART receive buffer.
uint8_t AlgorithmNumber = 0;													
uint16_t dma0Val[DMA_BUFFER_SIZE] = {0};										
uint16_t dma1Val[DMA_BUFFER_SIZE] = {0};										
uint16_t tempDacVal = 0;														// Temporary variable to hold parameter voltage value that is send for update from the GUI.
edma_handle_t g_EDMA_Handle0;													// Handler for the DMA0 callback.
edma_handle_t g_EDMA_Handle1;													// Handler for the DMA1 callback.
status_t status;																// A variable to indicate system status to the GUI.
//float CloseLimitFactor = 1.0f;													// Created mainly for 'simulated anealing'
//long double Temperature = 30.0;
//long double Alpha = 0.90;
//uint16_t TemperatureDecayCounter = 0;											// A counter of number of cycles between two Temperature decays.
bool IsGeneralHalt = false;													
bool FirstTimeInSA = true;														// A variable to indicate if we are stepping for the first time in simulated annealing algorithm.
bool FirstTimeInGA = true;														// A variable to indicate if we are stepping for the first time in genetic algorithm.
bool FirstTimeInPSO = true;						 								
uint8_t NumOfCycleToUpdate = NUM_OF_CYCLES_TO_UPDATE;							// The number of cycle to GUI update.
//uint8_t PopulatinPool[GA_POP_POOL] = {0};										// In 'genetic' algorithm, Array from which we draw a parent.
extern int error;																// A variable to hold return values from different RTOS and SDK function.

uint8_t DCA1_RF_IN_value = 0;
uint8_t DCA2_RF_Fisrt_Loop_value = 0;

uint8_t DCA3_RF_IN_swep_value = 20;
uint8_t DCA3_RF_OUT_Error_value = 0;
uint8_t DCA3_RF_OUT_swep_value = 30;

struct DacChannel																
{																				
	uint8_t  ParamNumber;														
	uint16_t DacValue;															
	uint64_t AnalogReadForEpsilonPlus;
	uint64_t AnalogReadForEpsilonMinus;
	uint16_t DacMaxLimit;														
	uint16_t DacMinLimit;														
	uint16_t DacJumpValue;														
	bool DacIsHalt;																
};

struct DacChannel DacChannels[NUM_OF_PARAM*2]; 									// structure array declaration.
struct DacChannel *pToDacChannels[NUM_OF_PARAM*2];								// Array of pointers to point each of the 8 parameters instant structures.

struct BestResult																
{
	uint16_t VvaValue;															// A variable to hold VVA voltage.
	uint16_t AmpEq1Value;														// A variable to hold AMP_EQ_1 voltage.
	uint16_t AmpEq2Value;														// A variable to hold AMP_EQ_2 voltage.
	uint16_t RippleEq1Value;													// A variable to hold RIPPLE_EQ_1 voltage.
	uint16_t RippleEq2Value;													// A variable to hold RIPPLE_EQ_2 voltage.
	uint16_t PhaseShifterValue;													// A variable to hold PHASE_SHIFTER voltage.
	uint16_t A2dResult;															// A variable to hold system output voltage.
};

struct UartFifo																	// Structure for the UART buffer FIFO that is implemented as linked list.
{
	uint16_t Data;
	struct UartFifo* next;
};

struct UartFifo* Front = NULL;													// A UART FIFO structure instant for the beginning of the buffer FIFO.
struct UartFifo* Rear = NULL;													// A UART FIFO structure instant for the end of the buffer FIFO.

struct InputSpectrum															
{
	uint16_t Data;
	bool IsValid;																
};

struct SpectrumBlock															
{
	uint16_t FreqLow;															
	uint16_t FreqHigh;															
	uint16_t FreqMid;															
	uint8_t SizeOfBlock;														
	uint8_t BarLow;																//	The starting position of the block
	uint8_t BarHigh;															
	uint8_t SizeOfBlockBar;														
	struct SpectrumBlock *Next;													
	struct SpectrumBlock *Previous;												
};

struct SpectrumBlock *FirstBlock = NULL;										
struct SpectrumBlock *CurrentBlock = NULL;										
struct SpectrumBlock *LastBlock = NULL;											

struct InputSpectrum InputSpectrumStructArray[NUM_OF_BARS_INPUT_SPECTRUM];		
bool InputSpectrumDummyArray[NUM_OF_BARS_OUTPUT_SPECTRUM] = {false};			
uint16_t InputSpectrumValue = 0;												

uint16_t OutputSpectrumArray[NUM_OF_BARS_OUTPUT_SPECTRUM];						// An array to hold the ADC values from the output sweep.
bool OutputSpectrumErrorPointLocationArray[NUM_OF_BARS_OUTPUT_SPECTRUM];		// A boolean array to represent the location of the relevant error point to be checked.

bool IsScanFin = false;															// A variable to indicate that a scan has finished.
uint64_t CurrentAdcValue;														// A variable to hold the system output voltage every interaction.

volatile uint32_t masterCommand;												// A variable for initiate DSPI.
uint32_t masterFifoSize;														// A variable to hold master DSPI FIFO size;
dspi_command_data_config_t commandDataCS0;										// DSPI configuration structure instant for chip select 0 - Pin D0.
dspi_command_data_config_t *pToChipSelectCMD;									// A pointer to DSPI configuration structure.

dspi_command_data_config_t commandDataCS1;										// DSPI configuration structure instant for chip select 0 - Pin D0.
dspi_command_data_config_t *pToChipSelectCMD1;									// A pointer to DSPI configuration structure.


SemaphoreHandle_t SemaphoreFromSampleTask = NULL;								// A handler for the semaphore 
SemaphoreHandle_t SemaphoreFromDMA0 = NULL;										// A handler for the semaphore 
SemaphoreHandle_t SemaphoreFromDMA1 = NULL;										// A handler for the semaphore 
SemaphoreHandle_t SemaphoreFromUartTx = NULL;									// A handler for the semaphore
SemaphoreHandle_t SemaphoreFromUartRxCallBack = NULL;							// A handler for the semaphore 
SemaphoreHandle_t SemaphoreFromSPI0TxIrq = NULL;								// A handler for the semaphore 
SemaphoreHandle_t SemaphoreFromLockDetectIrq = NULL;							// A handler for the semaphore
SemaphoreHandle_t SemaphoreFromTimer0Irq = NULL;								// A handler for the semaphore 
SemaphoreHandle_t SemaphoreFromTimer1Irq = NULL;								// A handler for the semaphore 
SemaphoreHandle_t SemaphoreFromTimer2Irq = NULL;								// A handler for the semaphore 
SemaphoreHandle_t SemaphoreHalt = NULL;

edma_transfer_config_t transferConfig;
edma_config_t userConfig;

uart_rtos_handle_t handle;
struct _uart_handle t_handle;

uart_rtos_config_t uart_config = {												// A structure to configure the UART.
    .baudrate = 230400,
    .parity = kUART_ParityDisabled,
    .stopbits = kUART_OneStopBit,
    .buffer = recv_buffer,
    .buffer_size = sizeof(recv_buffer),
};

volatile bool SynthLockDetect = false;											// Indicate lock detect signal is high. 
uint16_t freq = 2499 + IF_SAW_FILTER;
bool UartFin = false;															
bool IsInputSweepFinished = false;												
uint8_t StartSubMode = 0;														
bool OneMinHasPassed = false;													
bool HalfSecHasPassed = false;													
bool TenSecHasPassed = false;													
bool FirstTimeInStartSubMode = true;											
bool FirstTimeInGDSubMode = true;												
uint8_t GDSubMode = 0;															
uint8_t GDCyclesCounter = 0;
uint8_t UpdateGuiCounter = 0;

struct CalibFreq
{
	uint16_t RawValue1;
	uint16_t RawValue2;
	uint8_t IsRawValue1;
	uint8_t Freq;
};

struct CalibFreq CalibFreqStractArray[NUM_OF_CALIB_FREQ]; 						// structure array declaration.
struct CalibFreq *pToCalibFreqStractArray[NUM_OF_CALIB_FREQ]; 					// Pointers array to structure array.

uint8_t TheCalibFreq = 0;

uint16_t ErrorPointIndexArray[NUM_OF_ERROR_POINT_TO_SWEEP];						//=> The location of the relevant error(20)

uint16_t DataToSendSpi1 = 0;
uint16_t DataToSendSpi2 = 0;

uint8_t StartOfBlockArray[NUM_OF_BARS_INPUT_SPECTRUM];
uint8_t SizeOfBlockArray[NUM_OF_BARS_INPUT_SPECTRUM];

uint8_t MultyPurposeIndex = 0;
uint8_t MultyPurposeIndex1 = 0;

//******************************************************************************
/*******************************************************************************
 * CallBacks
 ******************************************************************************/
//******************************************************************************

//******************************************************************************
//	void PIT_1SEC_HANDLER(void).
//	****************************************************************************
//******************************************************************************
void PIT_100USEC_HANDLER(void)
{
	static BaseType_t xHigherPriorityTaskWoken;
	xHigherPriorityTaskWoken = pdFALSE;
    /* Clear interrupt flag.*/
    PIT_ClearStatusFlags(PIT, kPIT_Chnl_0, kPIT_TimerFlag);
	xSemaphoreGiveFromISR(SemaphoreFromTimer0Irq,&xHigherPriorityTaskWoken);
	portYIELD_FROM_ISR( xHigherPriorityTaskWoken );
}

//******************************************************************************
//	void PIT_10SEC_HANDLER(void).
//	****************************************************************************
//******************************************************************************
void PIT_10SEC_HANDLER(void)
{
	static BaseType_t xHigherPriorityTaskWoken;
	xHigherPriorityTaskWoken = pdFALSE;
    /* Clear interrupt flag.*/
    PIT_ClearStatusFlags(PIT, kPIT_Chnl_1, kPIT_TimerFlag);
    TenSecHasPassed = true;														
	xSemaphoreGiveFromISR(SemaphoreFromTimer1Irq,&xHigherPriorityTaskWoken);
	portYIELD_FROM_ISR( xHigherPriorityTaskWoken );
}

//******************************************************************************
//	void PIT_60SEC_HANDLER(void).
//	****************************************************************************
//******************************************************************************
void PIT_60SEC_HANDLER(void)
{
	static BaseType_t xHigherPriorityTaskWoken;
	xHigherPriorityTaskWoken = pdFALSE;
    /* Clear interrupt flag.*/
    PIT_ClearStatusFlags(PIT, kPIT_Chnl_2, kPIT_TimerFlag);
    OneMinHasPassed = true;														
    HalfSecHasPassed = true;													
	xSemaphoreGiveFromISR(SemaphoreFromTimer2Irq,&xHigherPriorityTaskWoken);
	portYIELD_FROM_ISR( xHigherPriorityTaskWoken );
}

//******************************************************************************
//	void LD_IRQ_HANDLER(void).
//	****************************************************************************
//******************************************************************************
void LD_IRQ_HANDLER(void)
{
	static BaseType_t xHigherPriorityTaskWoken;
	xHigherPriorityTaskWoken = pdFALSE;
    /* Clear external interrupt flag. */
    GPIO_ClearPinsInterruptFlags(LD_GPIO, 1U << LD_GPIO_PIN);
	xSemaphoreGiveFromISR(SemaphoreFromLockDetectIrq,&xHigherPriorityTaskWoken);
	portYIELD_FROM_ISR( xHigherPriorityTaskWoken );
}

//******************************************************************************
//	void EXAMPLE_DSPI_MASTER_IRQHandler(void).
//	****************************************************************************
//******************************************************************************
void EXAMPLE_DSPI_MASTER_IRQHandler(void)
{

	static BaseType_t xHigherPriorityTaskWoken;
	xHigherPriorityTaskWoken = pdFALSE;
	DSPI_ClearStatusFlags(EXAMPLE_DSPI_MASTER_BASEADDR, kDSPI_TxCompleteFlag);
	xSemaphoreGiveFromISR(SemaphoreFromSPI0TxIrq,&xHigherPriorityTaskWoken);
	portYIELD_FROM_ISR( xHigherPriorityTaskWoken );

}

//******************************************************************************
//	void EDMA0_Callback(edma_handle_t *handle, void *param, bool transferDone, uint32_t tcds).
//	****************************************************************************
//******************************************************************************
void EDMA0_Callback(edma_handle_t *handle, void *param, bool transferDone, uint32_t tcds)
{
    if (transferDone)
    {
    	ADC0->SC1[0] |= ADC_SC1_ADCH(31);									
    	static BaseType_t xHigherPriorityTaskWoken;
    	xHigherPriorityTaskWoken = pdFALSE;
    	xSemaphoreGiveFromISR(SemaphoreFromDMA0,&xHigherPriorityTaskWoken);
    	portYIELD_FROM_ISR( xHigherPriorityTaskWoken );
    }
}

//******************************************************************************
//	void EDMA1_Callback(edma_handle_t *handle, void *param, bool transferDone, uint32_t tcds).
//	****************************************************************************
//******************************************************************************
void EDMA1_Callback(edma_handle_t *handle, void *param, bool transferDone, uint32_t tcds)
{
    if (transferDone)
    {
    	ADC1->SC1[0] |= ADC_SC1_ADCH(31);										
    	static BaseType_t xHigherPriorityTaskWoken;
    	xHigherPriorityTaskWoken = pdFALSE;
    	xSemaphoreGiveFromISR(SemaphoreFromDMA1,&xHigherPriorityTaskWoken);
    	portYIELD_FROM_ISR( xHigherPriorityTaskWoken );
    }
}

/*******************************************************************************
 * Code
 ******************************************************************************/
/*!
 * @brief Application entry point.
 */

int main(void)
{

    /* Define the init structure for the input switch pin */
    gpio_pin_config_t ld_config = {
        kGPIO_DigitalInput, 0,
    };


    /* Init board hardware. */
	BOARD_InitPins();															
    BOARD_BootClockRUN();														
    NVIC_SetPriority(UART0_RX_TX_IRQn, 5);										
    Config_A2D_moudule(A2D0_16BIT_470KHZ_AVG32_CONTINUOUS);						
    Config_A2D_moudule(A2D1_16BIT_470KHZ_AVG32_CONTINUOUS);						

    
    PORT_SetPinInterruptConfig(LD_PORT, LD_GPIO_PIN, kPORT_InterruptRisingEdge);
    NVIC_SetPriority(LD_IRQ, 5);												
    GPIO_PinInit(LD_GPIO, LD_GPIO_PIN, &ld_config);								
    EnableIRQ(LD_IRQ);															

    dspi_master_config_t masterConfig;
    dspi_master_config_t *pTomasterConfig = &masterConfig;
    main_spi_config(pTomasterConfig);

    /* Structure of initialize PIT */
    pit_config_t pitConfig;

    /*
     * pitConfig.enableRunInDebug = false;
     */
    PIT_GetDefaultConfig(&pitConfig);

    /* Init pit module */
    PIT_Init(PIT, &pitConfig);

    /* Set timer period for channel 0 */
    PIT_SetTimerPeriod(PIT, kPIT_Chnl_0, USEC_TO_COUNT(1900U, PIT_SOURCE_CLOCK));

    /* Enable timer interrupts for channel 0 */
    PIT_EnableInterrupts(PIT, kPIT_Chnl_0, kPIT_TimerInterruptEnable);

    /* Enable at the NVIC */
    NVIC_SetPriority(PIT_100_USEC_IRQ_ID, 5);									// set up the interrupt priority.
    EnableIRQ(PIT_100_USEC_IRQ_ID);

    /* Set timer period for channel 1 */
    PIT_SetTimerPeriod(PIT, kPIT_Chnl_1, MSEC_TO_COUNT(10000U, PIT_SOURCE_CLOCK));

    /* Enable timer interrupts for channel 1 */
    PIT_EnableInterrupts(PIT, kPIT_Chnl_1, kPIT_TimerInterruptEnable);

    /* Enable at the NVIC */
    NVIC_SetPriority(PIT_10_SEC_IRQ_ID, 5);										// set up the nterrupt priority.
    EnableIRQ(PIT_10_SEC_IRQ_ID);

    /* Set timer period for channel 2 */

    PIT_SetTimerPeriod(PIT, kPIT_Chnl_2, MSEC_TO_COUNT(60000U, PIT_SOURCE_CLOCK));

    /* Enable timer interrupts for channel 2 */
    PIT_EnableInterrupts(PIT, kPIT_Chnl_2, kPIT_TimerInterruptEnable);

    /* Enable at the NVIC */
    NVIC_SetPriority(PIT_60_SEC_IRQ_ID, 5);										// set up the interrupt priority.
    EnableIRQ(PIT_60_SEC_IRQ_ID);

    init_GPIO();

    SemaphoreFromSampleTask = xSemaphoreCreateBinary();							// Create A Binary semaphore 
    SemaphoreFromDMA0 = xSemaphoreCreateBinary();								// Create A Binary semaphore 
    SemaphoreFromDMA1 = xSemaphoreCreateBinary();								// Create A Binary semaphore
    SemaphoreFromUartTx = xSemaphoreCreateBinary();					// Create A Binary semaphore
    //SemaphoreFromUartRxCallBack = xSemaphoreCreateBinary();					// Create A Binary semaphore 
    SemaphoreFromSPI0TxIrq = xSemaphoreCreateBinary();							// Create A Binary semaphore 
    SemaphoreFromLockDetectIrq = xSemaphoreCreateBinary();						// Create A Binary semaphore 
    SemaphoreFromTimer0Irq = xSemaphoreCreateBinary();							// Create A Binary semaphore 
    SemaphoreFromTimer1Irq = xSemaphoreCreateBinary();							// Create A Binary semaphore 
    SemaphoreFromTimer2Irq = xSemaphoreCreateBinary();							// Create A Binary semaphore

    SemaphoreHalt = xSemaphoreCreateBinary();



    for(int i = 0; i<NUM_OF_PARAM ; i++)
    	pToDacChannels[i] = &DacChannels[i];									// Initialize array of pointers to parameter structure.
/*
    for(int i = 0; i<10 ; i++)
    	pToBestResults[i] = &BestResults[i];									// Initialize array of pointers to BestResults structure.
*/


    /* Initialize messageQ for 10 logs with maximum lenght of one message 234 B */
    //messageQ_init( 10, MAX_MESSAGE_LENGTH );
    message_queue = xQueueCreate( 10, MAX_MESSAGE_LENGTH * sizeof( uint8_t ));

    if( (SemaphoreFromSampleTask != NULL)&&(SemaphoreFromTimer2Irq != NULL)&&(SemaphoreFromTimer0Irq != NULL)&&(SemaphoreFromTimer1Irq != NULL)&&(SemaphoreFromLockDetectIrq != NULL)&&(SemaphoreFromDMA0 != NULL)&&(SemaphoreFromDMA1 != NULL)&&(SemaphoreFromUartTx != NULL)/*&&(SemaphoreFromUartRxCallBack != NULL)*/&&(SemaphoreFromSPI0TxIrq != NULL) && (SemaphoreHalt != NULL))
    {
    	xTaskCreate(sample_task, "sample_task", /*configMINIMAL_STACK_SIZE*/(unsigned short)1000, NULL, uart_task_PRIORITY-17, NULL);
    	//xTaskCreate(analyze_and_set, "analyze_and_set", /*configMINIMAL_STACK_SIZE*/(unsigned short)1000, NULL, uart_task_PRIORITY-5, NULL);
    	xTaskCreate(uart_send_task, "uart_send_task", /*configMINIMAL_STACK_SIZE*/ (unsigned short)1500, NULL, uart_task_PRIORITY-12, NULL);
    	xTaskCreate(uart_recv_task, "uart_recv_task", /*configMINIMAL_STACK_SIZE*/ (unsigned short)1000, NULL, uart_task_PRIORITY-10, NULL);

    	vTaskStartScheduler();													// start the scheduler.
    }
    for (;;);
}



static void uart_send_task(void *pvParameters)
{
	//size_t n;
	//int error;

	uint8_t message[MAX_MESSAGE_LENGTH] = {0};

    uart_config.srcclk = CLOCK_GetFreq(DEMO_UART_CLKSRC);
    uart_config.base = DEMO_UART;

    if (0 > UART_RTOS_Init(&handle, &t_handle, &uart_config))
    {
        vTaskSuspend(NULL);
    }


     while(1)
    {
    	// xSemaphoreTake(SemaphoreFromUartTx,portMAX_DELAY);
    	//if(UpdateGuiCounter == NumOfCycleToUpdate)
    	//		{
    				UpdateGuiCounter=0;

    				UartFin = false;

    				if(xQueueReceive(message_queue, message, portMAX_DELAY))
    				{
    					UART_RTOS_Send(&handle, (uint8_t *)message, MAX_MESSAGE_LENGTH);
    				}

    				//UART_RTOS_Send(&handle, (uint8_t *)pToSend_buffer, MAX_MESSAGE_LENGTH);

    				UartFin = true;

    				vTaskDelay(pdMS_TO_TICKS( 1UL ));
    		//	}

    }


    UART_RTOS_Deinit(&handle);

    vTaskSuspend(NULL);
}




static void uart_recv_task(void *pvParameters)
{
	size_t n;
	size_t error;


     while(1)
    {

    	 error = UART_RTOS_Receive(&handle, recv_buffer, 4, &n);
    	 error = error;

    	 translate_GUI_message(recv_buffer);

    	 vTaskDelay(pdMS_TO_TICKS( 5UL ));
    }


    UART_RTOS_Deinit(&handle);

    vTaskSuspend(NULL);
}




static void sample_task(void *pvParameters)
{
	uart_config.srcclk = CLOCK_GetFreq(DEMO_UART_CLKSRC);

	uart_config.base = DEMO_UART;

	if (0 > UART_RTOS_Init(&handle, &t_handle, &uart_config))
	{
		vTaskSuspend(NULL);
	}

	edma_initialization();
	dac_initialization(); //default value

	/* Start master transfer*/
	masterCommand = DSPI_MasterGetFormattedCommand(&commandDataCS0);
	masterFifoSize = FSL_FEATURE_DSPI_FIFO_SIZEn(EXAMPLE_DSPI_MASTER_BASEADDR);
	SPI0->MCR |= SPI_MCR_PCSIS(1);												
	SPI0->MCR |= SPI_MCR_PCSIS(2);												
	DSPI_StopTransfer(EXAMPLE_DSPI_MASTER_BASEADDR);
	DSPI_FlushFifo(EXAMPLE_DSPI_MASTER_BASEADDR, true, true);
	DSPI_ClearStatusFlags(EXAMPLE_DSPI_MASTER_BASEADDR, kDSPI_AllStatusFlag);

	//Sample the power detector - adc0 (error)
	CurrentAdcValue = initate_adc0_sample();

	set_spi_to_dac(SPI0_16BIT);													
	initiate_dac(pToChipSelectCMD);

	synthesizer_initialization();												

	// initialization DCA1(RF_IN) and DCA2(First Loop)
	update_dca(DCA1_RF_IN_value, pToChipSelectCMD, DCA1);
	update_dca(DCA2_RF_Fisrt_Loop_value, pToChipSelectCMD, DCA2);

    AlgorithmNumber = START_MODE;			//WAIT_TO_START;

    message_type = INSPEC;

    NumOfCycleToUpdate = 1;

	for(int i = 0; i<NUM_OF_PARAM*2; i++)
	{
		update_dac(DacChannels[i].DacValue,pToChipSelectCMD,i);
	}

	while(1)
	{
		//Sample the power detector - adc0 (error)
		CurrentAdcValue = initate_adc0_sample();

		switch(AlgorithmNumber)													
		{
			// Wait until get START command from the GHI
		/*	case WAIT_TO_START:
				for(int i = 0; i<100;i++);
				//vTaskDelay(pdMS_TO_TICKS( 1UL ));
			break;
*/

		//
					case DAC_WORK:
						swep_input_spectrum();								// Sweep the next 5 frequencies.
						for(int i=0;i<1000000000;i++) ;
						//analyze_input_spectrum();

						//locate_output_spectrum_error_points();
					break;

			case START_MODE:

				switch(StartSubMode)											
				{
					case 0:														
						if(FirstTimeInStartSubMode == true)						
						{
							//freq = 2499 + IF_SAW_FILTER;

							swep_input_spectrum();								// Sweep the next 5 frequencies.

							analyze_input_spectrum();

							locate_output_spectrum_error_points();

							FirstTimeInStartSubMode = false;
						}

						if(IsInputSweepFinished)
						{

							IsInputSweepFinished = false;						
							//xSemaphoreGive(SemaphoreFromSampleTask);
							freq = 2499 + IF_SAW_FILTER;
							StartSubMode = 20;
							MultyPurposeIndex = 0;
							FirstTimeInStartSubMode = true;
						}
						break;
						
					case 20 :

						if(UartFin==true)
						{
							UartFin = false;

							//Sampling the power of the blocks that detected at the input scan
							send_input_block_locations();

							StartSubMode = 21;
							FirstTimeInStartSubMode = true;
						}
						else
							vTaskDelay(pdMS_TO_TICKS( 1UL ));
						break;

					case 21:

						if(UartFin==true)
						{
							UartFin = false;

							sample_power_output_spectrum();

							StartSubMode = 22;
							FirstTimeInStartSubMode = true;
						}
						else
							vTaskDelay(pdMS_TO_TICKS( 1UL ));

						break;

					case 22 :

						if(UartFin==true)
						{
							swep_output_spectrum();

							StartSubMode = 1;
							FirstTimeInStartSubMode = true;
						}
						else
							vTaskDelay(pdMS_TO_TICKS( 1UL ));

						break;

					case 1:
						if(FirstTimeInStartSubMode == true)
							{
							//	PIT_StartTimer(PIT, kPIT_Chnl_2);
								FirstTimeInStartSubMode = false;

								message_type = GRADEN;

							}
							//if(coutGD1++ == 1200)			//(OneMinHasPassed)
							//{
							//	coutGD1 = 0;
								StartSubMode = 2;
								FirstTimeInStartSubMode = true;
								OneMinHasPassed = false;
							//	PIT_StopTimer(PIT, kPIT_Chnl_2);
							//}

							//Sample the power detector - adc0 (error)
							CurrentAdcValue = initate_adc0_sample();

							sample_gradient_descent();

							//Analyze and set the selected value of the DAC - gradient decent first loop
							gradient_descent_analyze_and_set();

						break;

					case 2:
						if(FirstTimeInStartSubMode == true)
						{
							PIT_StartTimer(PIT, kPIT_Chnl_2);
							FirstTimeInStartSubMode = false;
						}
						if(OneMinHasPassed)
						{
							AlgorithmNumber = GRADIENT_DESCENT;
							freq = 2299 + IF_SAW_FILTER;
							FirstTimeInGDSubMode = true;
							OneMinHasPassed = false;
							HalfSecHasPassed = false;
							PIT_StopTimer(PIT, kPIT_Chnl_2);
						}


						if(UartFin==true)
						{

							freq = 2299 + IF_SAW_FILTER;


							UartFin = false;
							gd_second_loop_sample();
							//NEW_second_loop_price_func();


							gd_second_loop_aas();

							sample_out_spectrum();

							//xSemaphoreGive(SemaphoreFromSampleTask);
						}
						else
							vTaskDelay(pdMS_TO_TICKS( 1UL ));
						break;



					case 11:
						if(UartFin==true)
						{
							//xSemaphoreGive(SemaphoreFromSampleTask);
							message_type = CALIBR;

							//F_2_50_GHZ -> OF_2_70_GHZ
							send_buffer[6] = TheCalibFreq;
							send_buffer[7] = CalibFreqStractArray[TheCalibFreq].IsRawValue1;

							if(CalibFreqStractArray[TheCalibFreq].IsRawValue1 == 1)
							{
								send_buffer[8] = (uint8_t)(CalibFreqStractArray[TheCalibFreq].RawValue1 & 0xff);
								send_buffer[9] = (uint8_t)(CalibFreqStractArray[TheCalibFreq].RawValue1 >> 8);
							}
							else
							{
								send_buffer[8] = (uint8_t)(CalibFreqStractArray[TheCalibFreq].RawValue2 & 0xff);
								send_buffer[9] = (uint8_t)(CalibFreqStractArray[TheCalibFreq].RawValue2 >> 8);
							}

							StartSubMode = 1;
							FirstTimeInStartSubMode = true;

						}
						else
							vTaskDelay(pdMS_TO_TICKS( 1UL ));

						break;
				}

				break;


			case GRADIENT_DESCENT : case 1 : case 2:							
				switch(GDSubMode)
				{
					case RUN2:
						if(FirstTimeInGDSubMode == true)						
						{
							//Start 10 second timer
							PIT_StartTimer(PIT, kPIT_Chnl_1);

							FirstTimeInGDSubMode = false;						
						}

						//Check if 10 sec have passed,If passed jump to RUN1(If passed 10 times of ping pong jumps to RUN3)
						if(TenSecHasPassed)										
						{
							GDSubMode = RUN1;
							FirstTimeInGDSubMode = true;						
							TenSecHasPassed = false;							
							PIT_StopTimer(PIT, kPIT_Chnl_1);					

							//If passed 10 times of ping pong jumps to RUN2
							if((++GDCyclesCounter)==NUM_OF_GD_CYCLES_TO_INPUT_SWEEP)	
							{
								GDCyclesCounter = 0;							
								GDSubMode = RUN3;
								FirstTimeInGDSubMode = true;					
								//freq = 2499 + IF_SAW_FILTER;
							}
						}
						
						if(UartFin==true)										
						{
							//freq = 2299 + IF_SAW_FILTER;
							UartFin = false;

							//Selects 20 error points and samples them
							swep_output_spectrum();

							//Sample 20 error points
							//sample_out_spectrum();


							//Sample error value for GD Second loop ,DET2 - adc1 (error)
							CurrentAdcValue = clac_second_loop_price_func(); //initate_adc0_sample();

							gd_second_loop_sample();

							//Analyze and set the selected value of the DAC - gradient decent second loop
							gd_second_loop_aas();

							//Sample 20 error points
							sample_out_spectrum();

							//xSemaphoreGive(SemaphoreFromSampleTask);
						}
						else
							vTaskDelay(pdMS_TO_TICKS( 1UL ));
						break;

					case RUN1 :
						if(FirstTimeInGDSubMode == true)						
						{
							//timer 2 initialization, for half a second.
							PIT_SetTimerPeriod(PIT, kPIT_Chnl_2, MSEC_TO_COUNT(500U, PIT_SOURCE_CLOCK));

							//Start ten second timer
					//		PIT_StartTimer(PIT, kPIT_Chnl_2);

							FirstTimeInGDSubMode = false;

							message_type = GRADEN;
						}
					//	if(coutGD1++ == 1200)			//(OneMinHasPassed)
					//	{
					//		coutGD1 = 0;
							GDSubMode = RUN2;
							FirstTimeInGDSubMode = true;						
							HalfSecHasPassed = false;							
							//freq = 2299 + IF_SAW_FILTER;
					//		PIT_StopTimer(PIT, kPIT_Chnl_2);
					//	}

						//Sample the power detector - adc0 (error)
						CurrentAdcValue = initate_adc0_sample();

						sample_gradient_descent();

						//xSemaphoreGive(SemaphoreFromSampleTask);

						//Analyze and set the selected value of the DAC - gradient decent first loop
						gradient_descent_analyze_and_set();

						break;

					case RUN3:
						if(UartFin==true)										
						{

							//freq = 2499 + IF_SAW_FILTER;
							FirstTimeInGDSubMode = true;						
							UartFin = false;

							swep_input_spectrum();

							analyze_input_spectrum();

							locate_output_spectrum_error_points();

							GDSubMode = 21;
							//xSemaphoreGive(SemaphoreFromSampleTask);
						}
						else
							vTaskDelay(pdMS_TO_TICKS( 1UL ));
						break;

					case 21:

						if(UartFin==true)
						{
							UartFin = false;

							//Send the detected blocks entrance Locations
							send_input_block_locations();
							/*
							MultyPurposeIndex = 0;

							message_type = INPOWR;

							while((StartOfBlockArray[MultyPurposeIndex] != 0) || (MultyPurposeIndex == 0))
							{
								send_buffer[6+2*MultyPurposeIndex] = StartOfBlockArray[MultyPurposeIndex];
								send_buffer[7+2*MultyPurposeIndex] = SizeOfBlockArray[MultyPurposeIndex];
								MultyPurposeIndex++;
							}

							if((MultyPurposeIndex1++)==5)
							{
								GDSubMode = 22;
								FirstTimeInGDSubMode = true;
								MultyPurposeIndex1 = 0;
							}

							send_buffer[6+2*MultyPurposeIndex] = 0;
							send_buffer[7+2*MultyPurposeIndex] = 0;
							send_buffer[8+2*MultyPurposeIndex] = 0;
							*/

							//xSemaphoreGive(SemaphoreFromSampleTask);
							GDSubMode = 22;
						}
						else
							vTaskDelay(pdMS_TO_TICKS( 1UL ));

						break;


					case 22:
						if(UartFin==true)
						{
							UartFin = false;

							//	Sampling the power of the blocks that detected at the input scan
							sample_power_output_spectrum();
							/*
							message_type = CCCCCC;

							freq = 2499 + IF_SAW_FILTER;

							for(int i = 0 ; i < 100 ; i++)
							{
								freq +=2;
								udate_synthesizer(freq, pToChipSelectCMD, 0);
								//xSemaphoreTake(SemaphoreFromLockDetectIrq,portMAX_DELAY);

							    PIT_StartTimer(PIT, kPIT_Chnl_0);
							    xSemaphoreTake(SemaphoreFromTimer0Irq,portMAX_DELAY);
							    PIT_StopTimer(PIT, kPIT_Chnl_0);

								InputSpectrumValue = initate_adc1_sample();

								send_buffer[6+2*i] = (uint8_t)(InputSpectrumValue & 0xff);
								send_buffer[7+2*i] = (uint8_t)(InputSpectrumValue >> 8);
							}
							*/


							GDSubMode = RUN2;
							FirstTimeInGDSubMode = true;

							//xSemaphoreGive(SemaphoreFromSampleTask);
						}
						else
							vTaskDelay(pdMS_TO_TICKS( 1UL ));

						break;

				}

				break;

		}


		//xSemaphoreGive(SemaphoreFromUartTx);

		UpdateGuiCounter++;
		if(UpdateGuiCounter == NumOfCycleToUpdate)
		{
			UpdateGuiCounter=0;

			UartFin = false;

			send_to_pc(send_buffer, message_type);

			UartFin = true;

		//vTaskDelay(pdMS_TO_TICKS( 1UL ));

		}

	}
}














