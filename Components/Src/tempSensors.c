/**
  ******************************************************************************
  * @file    tempSensors.c
  * @author  Nikita lazarev <nikitaterm@gmail.com>
  * @version V1.01
  * @date    14-Feb-2016
  * @brief   Source file of temperature sensors driver.
  ******************************************************************************
  */

#include "stm32f4xx_hal.h"

#include "tempSensors.h"
#include "thermocoupleLookUp.h"

#define ADC_M ADC1
#define DMA_M DMA2_Stream0
#define DMA_CHANEL DMA_CHANNEL_0

/* ADC parameters */
#define ADC_V_REF 2.94  /// ADC reference voltage
#define ADC_MAV_V 4095  /// ADC max output value (12-bit)

/** Possible statuses */
typedef enum {
  NotInitialized,
  Stopped,
  Run

} Status;

/** Temperature sensors channel definitions */
typedef struct
{
  uint32_t ADC_channel;  /// ADC channel
  
  double GAIN_k;        /// Channel gain coefficient

} Channel_DataTypeDef;

/** Add new items to support more channels */
#define CH_AMOUNT 2
static Channel_DataTypeDef channels[CH_AMOUNT] = 
                          { {ADC_CHANNEL_11,         371},  /// H_END1 channel

                            /* Insert a new channel here */      /// NEW channel

                            {ADC_CHANNEL_TEMPSENSOR, 0}  /**< Ref temperature sensor channel.
                                                      Must be the last channel in the list */
                          };

#define REF_TEMP_SENSOR CH_AMOUNT - 1

/* Private functions */
/**
  * @brief Get the value of the reference temperature sensor
  */
static double getRefTemp(void);

/* Private members */
static ADC_HandleTypeDef hadc; /// ADC instance to be used
static DMA_HandleTypeDef hdma; /// DMA instance to be used

static uint32_t temp[CH_AMOUNT];   /**< Temperature values:
                                        [0] - channel 0,
                                        [1] - cold junction */

static Status status = NotInitialized;

void tempSensorsInit(void)
{
  if (status == NotInitialized) {
    __ADC1_CLK_ENABLE();
    __DMA2_CLK_ENABLE();
    ADC_ChannelConfTypeDef sConfig;
    
    /**Configure the global features of the ADC (Clock, Resolution, Data Alignment and
       number of conversion) */
    hadc.Instance = ADC_M;
    hadc.Init.ClockPrescaler = ADC_CLOCKPRESCALER_PCLK_DIV8;
    hadc.Init.Resolution = ADC_RESOLUTION12b;
    hadc.Init.ScanConvMode = ENABLE;
    hadc.Init.ContinuousConvMode = ENABLE;
    hadc.Init.DiscontinuousConvMode = DISABLE;
    hadc.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
    hadc.Init.DataAlign = ADC_DATAALIGN_RIGHT;
    hadc.Init.NbrOfConversion = CH_AMOUNT;
    hadc.Init.DMAContinuousRequests = ENABLE;
    hadc.Init.EOCSelection = EOC_SINGLE_CONV;
    HAL_ADC_Init(&hadc);

    /**Configure for the selected ADC regular channel its corresponding rank in the
       sequencer and its sample time. 
       Configure the channel 0*/
    uint32_t rank = 1;
    sConfig.Channel = channels[rank - 1].ADC_channel;
    sConfig.Rank = rank++;
    sConfig.SamplingTime = ADC_SAMPLETIME_56CYCLES;
    HAL_ADC_ConfigChannel(&hadc, &sConfig);

    /* Insert new channels here. Keep in mide the ranks! */

    /** Configure the embedded temperature sensor channel (cold junction) */
    sConfig.Channel = channels[rank - 1].ADC_channel;
    sConfig.Rank = rank++;
    sConfig.SamplingTime = ADC_SAMPLETIME_56CYCLES;
    HAL_ADC_ConfigChannel(&hadc, &sConfig);

    /** Configure the DMA module */
    hdma.Instance = DMA_M;
    hdma.Init.Channel = DMA_CHANEL;
    hdma.Init.Direction = DMA_PERIPH_TO_MEMORY;
    hdma.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma.Init.MemInc = DMA_MINC_ENABLE;
    hdma.Init.PeriphDataAlignment = DMA_PDATAALIGN_WORD;
    hdma.Init.MemDataAlignment = DMA_MDATAALIGN_WORD;
    hdma.Init.Mode = DMA_CIRCULAR;
    hdma.Init.Priority = DMA_PRIORITY_HIGH;
    hdma.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
    HAL_DMA_Init(&hdma);
    __HAL_LINKDMA(&hadc,DMA_Handle,hdma);

    status = Stopped;
  }
}

void tempSensorsDeInit(void) {
  if (status == Stopped) {
    __ADC1_CLK_DISABLE();
    HAL_DMA_DeInit(hadc.DMA_Handle);  

    status = NotInitialized;
  }
}

void tempSensorsRun(void)
{
  if (status == Stopped) {
    HAL_ADC_Start(&hadc);
    HAL_ADC_Start_DMA(&hadc, (uint32_t*)temp, CH_AMOUNT);
    status = Run;
  }
}

void tempSensorsStop(void)
{
  if (status == Run) {
    HAL_ADC_Stop_DMA(&hadc);
    HAL_ADC_Stop(&hadc);
    status = Stopped;
  }
}

/* Reference temperature sensor parameters */
#define V25 0.76
#define AVG_SLOPE 0.0025
#define CONST_T 25

double getRefTemp(void) {
  return (ADC_V_REF*temp[REF_TEMP_SENSOR]/ADC_MAV_V - V25)/AVG_SLOPE + CONST_T;
}

double tempSensorsGet(uint8_t ch) {
  if (status == Run) {
    double hot_end_v = (ADC_V_REF*temp[ch]/ADC_MAV_V/channels[ch].GAIN_k)*1000;
    double alpha, betta;
    int i = 0;
    while (k_type[i] <= hot_end_v)
      i++;
    alpha = kTYPE_DELTA/(k_type[i]-k_type[i-1]);
    betta = (i-1)*kTYPE_DELTA - alpha*k_type[i-1];
    return alpha*hot_end_v+betta+getRefTemp();
  }
  return 0;
}
