/* USER CODE BEGIN Header */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/

#include <main.h>
#include <string.h>
#include <ff.h>
#include <stm32f769i_discovery_audio.h>
#include "GifDecoder.h"
#include "wavdecoder.h"
#include <stdio.h>
#include "GUI.h"
#include <WM.h>
#include <stm32f769i_discovery_ts.h>
#include <MainTask.h>
#include "video_player_app.h"
#include "GUI_AVI.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/


SAI_HandleTypeDef SaiHandle;
DMA_HandleTypeDef            hSaiDma;

/*JPEG Related variable
 */





/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/



RTC_HandleTypeDef hrtc;
UART_HandleTypeDef UartHandle;
DMA_HandleTypeDef DMAUartHandle;
FIL JPEG_File;  /* File object */
Wavheader Wheader;
extern uint8_t volume;
/* USER CODE BEGIN PV */
/*GIF Decode Varible part*/

/* template parameters are maxGifWidth, maxGifHeight, lzwMaxBits
 *
 * The lzwMaxBits value of 12 supports all GIFs, but uses 16kB RAM
 * lzwMaxBits can be set to 10 or 11 for smaller displays to save RAM, but use 12 for large displays
 * All 32x32-pixel GIFs tested so far work with 11, most work with 10
 */
const uint16_t kMatrixWidth = 800;        // known working: 32, 64, 96, 128
const uint16_t kMatrixHeight = 480;       // known working: 16, 32, 48, 64
//GifDecoder<kMatrixWidth, kMatrixHeight, 12> decoder((uint8_t *)GIFIMAGE_ADDRESS, (uint8_t *)GIFIMAGEBU_ADDRESS);

/* USER CODE END PV */



/* Private function prototypes -----------------------------------------------*/
 void SystemClock_Config(void);
static void MX_RTC_Init(void);


/* USER CODE BEGIN PFP */
/* USER CODE END PFP */
void vApplicationStackOverflowHook(xTaskHandle *pxTask, signed char *pcTaskName )
{
	xTaskHandle *bad_task_handle;
	signed char *bad_task_name;

    bad_task_handle = pxTask;     // this seems to give me the crashed task handle
    bad_task_name = pcTaskName;     // this seems to give me a pointer to the name of the crashed task
    //mLED_3_On();   // a LED is lit when the task has crashed
    taskDISABLE_INTERRUPTS();
    for( ;; );
    taskEXIT_CRITICAL();
}





AUDIO_DrvTypeDef *audio_drv;
static void Playback_Init(uint32_t SampleRate)
{
  RCC_PeriphCLKInitTypeDef RCC_PeriphCLKInitStruct;
  
  /* Configure PLLSAI prescalers */
  /* PLLSAI_VCO: VCO_429M 
     SAI_CLK(first level) = PLLSAI_VCO/PLLSAIQ = 429/2 = 214.5 Mhz
     SAI_CLK_x = SAI_CLK(first level)/PLLSAIDIVQ = 214.5/19 = 11.289 Mhz */ 
  RCC_PeriphCLKInitStruct.PeriphClockSelection = RCC_PERIPHCLK_SAI2;
  RCC_PeriphCLKInitStruct.Sai2ClockSelection = RCC_SAI2CLKSOURCE_PLLSAI;
  RCC_PeriphCLKInitStruct.PLLSAI.PLLSAIN = 429; 
  RCC_PeriphCLKInitStruct.PLLSAI.PLLSAIQ = 2; 
  RCC_PeriphCLKInitStruct.PLLSAIDivQ = 19;     
  
  if(HAL_RCCEx_PeriphCLKConfig(&RCC_PeriphCLKInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /* Initialize SAI */
  __HAL_SAI_RESET_HANDLE_STATE(&SaiHandle);

  SaiHandle.Instance = AUDIO_SAIx;

  __HAL_SAI_DISABLE(&SaiHandle);

  SaiHandle.Init.AudioMode      = SAI_MODEMASTER_TX;
  SaiHandle.Init.Synchro        = SAI_ASYNCHRONOUS;
  SaiHandle.Init.OutputDrive    = SAI_OUTPUTDRIVE_ENABLE;
  SaiHandle.Init.NoDivider      = SAI_MASTERDIVIDER_ENABLE;
  SaiHandle.Init.FIFOThreshold  = SAI_FIFOTHRESHOLD_1QF;
  SaiHandle.Init.AudioFrequency = SampleRate;
  SaiHandle.Init.Protocol       = SAI_FREE_PROTOCOL;
  SaiHandle.Init.DataSize       = SAI_DATASIZE_16;
  SaiHandle.Init.FirstBit       = SAI_FIRSTBIT_MSB;
  SaiHandle.Init.ClockStrobing  = SAI_CLOCKSTROBING_FALLINGEDGE;

  SaiHandle.FrameInit.FrameLength       = 32;
  SaiHandle.FrameInit.ActiveFrameLength = 16;
  SaiHandle.FrameInit.FSDefinition      = SAI_FS_CHANNEL_IDENTIFICATION;
  SaiHandle.FrameInit.FSPolarity        = SAI_FS_ACTIVE_LOW;
  SaiHandle.FrameInit.FSOffset          = SAI_FS_BEFOREFIRSTBIT;

  SaiHandle.SlotInit.FirstBitOffset = 0;
  SaiHandle.SlotInit.SlotSize       = SAI_SLOTSIZE_DATASIZE;
  SaiHandle.SlotInit.SlotNumber     = 2; 
  SaiHandle.SlotInit.SlotActive     = (SAI_SLOTACTIVE_0 | SAI_SLOTACTIVE_1);

  if(HAL_OK != HAL_SAI_Init(&SaiHandle))
  {
    Error_Handler();
  }
  
  /* Enable SAI to generate clock used by audio driver */
  __HAL_SAI_ENABLE(&SaiHandle);
  
  /* Initialize audio driver */
  if(WM8994_ID != wm8994_drv.ReadID(AUDIO_I2C_ADDRESS))
  {
    Error_Handler();
  }
  
  audio_drv = &wm8994_drv;
  audio_drv->Reset(AUDIO_I2C_ADDRESS);  
  if(0 != audio_drv->Init(AUDIO_I2C_ADDRESS, OUTPUT_DEVICE_HEADPHONE, 20, SampleRate ))
  {
    Error_Handler();
  }
}

FIL fi;

/*gif call back functions start */
void screenClearCallback(void)
{
	BSP_LCD_Clear(LCD_COLOR_BLACK);
}
#if 0
void updateScreenCallback(void)
{
	uint32_t xPos = (BSP_LCD_GetXSize() - decoder.GetGifWidth())/2;
	uint32_t yPos = (BSP_LCD_GetYSize() - decoder.GetGifHeight())/2;
	CopyPicture((uint32_t *)IMAGEBUFF_ADDRESS, (uint32_t *)LAYER1_ADDRESS, xPos, yPos, decoder.GetGifWidth(), decoder.GetGifHeight());
}
void drawPixelCallback(int16_t x, int16_t y, uint8_t red, uint8_t green, uint8_t blue)
{
	*((uint32_t*)(IMAGEBUFF_ADDRESS+(x+y* (decoder.GetGifWidth()))*4)) = ((uint16_t)red << 16) | ((uint16_t)green << 8) | (blue);
}
#endif

/*gif call back functions end */
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
uint32_t FrameOffset=0;
const uint16_t block_size = 4096*6;
uint16_t buff[block_size];
uint8_t buff_audio[block_size*2];
#define PLAY_HEADER          0x2C
bool flag = 0;
uint8_t temp_data[8];
uint8_t waitfor_move=0;
TaskHandle_t xTaskVolume = NULL;
TaskHandle_t xTaskMusic = NULL;
static TaskHandle_t xTaskGUI = NULL;
static TaskHandle_t xTaskTouchEx = NULL;
TaskHandle_t xTaskJpeg = NULL;
static void vTaskVolume(void *pvParameters)
{
  UBaseType_t  priority;
  uint32_t ulNotifiedValue;
  while(1)
  {

	  ulNotifiedValue = xTaskNotifyWait(0, 0xffffffff, &ulNotifiedValue, 400 );
	  if (ulNotifiedValue & 0x01)
	  {

			priority = uxTaskPriorityGet( NULL );
			vTaskPrioritySet(NULL, 0);
			wm8994_SetVolume(AUDIO_I2C_ADDRESS, volume);
			vTaskPrioritySet(NULL, priority);

	  }
	  BSP_LED_Toggle(LED2);
    //vTaskDelay(400);
  }
}


static void vTaskTouchEx(void *pvParameters)
{
	volatile uint32_t ulNotificationValue;
	TS_StateTypeDef  ts;
	static volatile GUI_PID_STATE TS_State = {0, 0, 0, 0};
	while(1)
	{
		ulNotificationValue = ulTaskNotifyTake( pdTRUE, 50 );


		if (ulNotificationValue==1)
		{
			BSP_TS_GetState((TS_StateTypeDef *)&ts);
			TS_State.Pressed = 1;
			TS_State.x = ts.touchX[0];
			TS_State.y = ts.touchY[0];
			GUI_PID_StoreState((const GUI_PID_STATE*)&TS_State);
		}
		else
		{
			if (TS_State.Pressed == 1)
			{
				TS_State.Pressed = 0;
				TS_State.x = -1;
				TS_State.y = -1;
				GUI_PID_StoreState((const GUI_PID_STATE*)&TS_State);;
			}
		}

	}
}
unsigned int RetrieveMP3Data(void * pMP3CompressedData,
unsigned int nMP3DataSizeInChars,
void * token)
{
	UINT nData_Read;

	f_read((FIL*)token, pMP3CompressedData, nMP3DataSizeInChars, &nData_Read);
	return nData_Read;
}

AudioTime Total_AudioTime;
extern char time_char[18];
WM_HWIN xhWin;
FATFS fs;
char filename[256];
extern uint8_t Play_Status;
extern uint8_t Repeat_Status;
#define Audio_BUFFER_EMPTY (0)
#define Audio_BUFFER_FULL  (1)

enum JPEG_DATA_STATE{JPEG_DATA_IDEL, JPEG_DATA_READY, JPEG_DATA_PROCESSING};
uint8_t state_audio[2];
SemaphoreHandle_t xSem_audio;

static void vTaskMusic(void *pvParameters)
{

  uint32_t ulNotifiedValue;
  volatile uint32_t realx = 0;
  xSem_audio = xSemaphoreCreateBinary();
  while(1)
  {
	  xTaskNotifyWait(0, 0xffffffff, &ulNotifiedValue, osWaitForever );
	  if (ulNotifiedValue)
	  {
		if (ulNotifiedValue  & 0x01)
		{
			if (state_audio[0] == Audio_BUFFER_FULL)
			{
				memcpy((U8*)buff, (U8 *)buff_audio, block_size);
			}
			else
			{
				realx++;
			}
			state_audio[0] = Audio_BUFFER_EMPTY;
		}
		if (ulNotifiedValue  & 0x02)
		{
			if (state_audio[1] == Audio_BUFFER_FULL)
			{
				memcpy((U8*)&buff[block_size/2], (U8 *)(buff_audio+block_size), block_size);
			}
			else
			{
				realx++;
			}

			state_audio[1] = Audio_BUFFER_EMPTY;
		}
		if (waitfor_move)
		{
			xSemaphoreGive(xSem_audio);
		}
	  }
  }
}

uint8_t    FrameBuffer[AVI_VIDEO_BUF_SIZE], FrameBuffer2[AVI_VIDEO_BUF_SIZE];;
uint32_t JPEG_Size[2];
SemaphoreHandle_t xSem_jpeg;
JPEG_DATA_STATE Data_State[2]={JPEG_DATA_IDEL, JPEG_DATA_IDEL};
extern JPEG_HandleTypeDef    JPEG_Handle;
static void vTaskJpeg(void *pvParameters)
{

	xSem_jpeg = xSemaphoreCreateBinary();
	while(1)
	{
		if (JPEG_Handle.State == HAL_JPEG_STATE_READY)
		{
			if (Data_State[0] == JPEG_DATA_READY)
			{
				Data_State[0] = JPEG_DATA_PROCESSING;
				HW_JPEG_Draw2(FrameBuffer, JPEG_Size[0]);
				Data_State[0] = JPEG_DATA_IDEL;
			}
			else if (Data_State[1] == JPEG_DATA_READY)
			{
				Data_State[1] = JPEG_DATA_PROCESSING;
				HW_JPEG_Draw2(FrameBuffer2, JPEG_Size[1]);
				Data_State[1] = JPEG_DATA_IDEL;
			}
			else
			{
				xSemaphoreTake(xSem_jpeg, 1000);
			}
		}
		else
		{
			xSemaphoreTake(xSem_jpeg, 1000);
		}
		BSP_LED_Toggle(LED_RED);
	}
}
int Get_data(uint8_t *buff, uint32_t Datalength)
{
	unsigned int length;

	f_read(&fi, buff, Datalength, &length);
	return length;
}
AVI_INFO Avix;

void  vTaskGUI(void *pvParameters)
{
	unsigned int length;
	FRESULT res;
	FILINFO fno;
	DIR dj;
	volatile AVISTATUS avi_res;
	HW_JPEG_Init();
	uint32_t offset, audio_pr;
	uint8_t *buf_pr = (uint8_t*)buff_audio;


	BSP_SD_Init();
	Playback_Init(44100);
	HAL_SAI_Transmit_DMA(&SaiHandle, (uint8_t *)buff, block_size);
	if (f_mount(&fs,(char*)"",1) == FR_OK)
	{

		f_open(&fi, "/Avi/happy.avi", FA_READ);

		while(1)
		{
			f_read(&fi, (U8 *)FrameBuffer, AVI_VIDEO_BUF_SIZE, &length);
			avi_res = _AVI_Init(&Avix, FrameBuffer, AVI_VIDEO_BUF_SIZE);
			offset = _AVI_SearchID(FrameBuffer, AVI_VIDEO_BUF_SIZE, (U8*)"movi");
			_Avi_Get_Streaminfo(&Avix, FrameBuffer + offset + 4);

			f_lseek(&fi, offset+12);
			audio_pr = 0;
			state_audio[0] = Audio_BUFFER_EMPTY;
			state_audio[1] = Audio_BUFFER_EMPTY;
			wm8994_SetFrequency(AUDIO_I2C_ADDRESS, Avix.SampleRate);
			while(f_size(&fi)!=f_tell(&fi) && Avix.StreamSize<=AVI_VIDEO_BUF_SIZE)
			{

				if (AVI_AUDS_FLAG==Avix.StreamID)
				{
					if (((state_audio[1] == Audio_BUFFER_FULL) && (state_audio[0]==Audio_BUFFER_FULL))||\
							((audio_pr+Avix.StreamSize>=(block_size<<1)) && state_audio[0] == Audio_BUFFER_FULL)||\
							(((audio_pr <= block_size) && (audio_pr+Avix.StreamSize) >= block_size) && state_audio[1] == Audio_BUFFER_FULL))
					{
						waitfor_move = 1;
						if (JPEG_Handle.State == HAL_JPEG_STATE_READY)
						{
							length++;
						}
						xSemaphoreTake(xSem_audio, osWaitForever);
						waitfor_move = 0;
					}


					if (audio_pr+Avix.StreamSize>=(block_size<<1))
					{
	//					if (state_audio[0] == Audio_BUFFER_FULL)
	//					{
	//						waitfor_move = 1;
	//						xSemaphoreTake(xSem_audio, osWaitForever);
	//						waitfor_move = 0;
	//					}
						f_read(&fi, (U8 *)buf_pr+audio_pr, (block_size<<1)-audio_pr, &length);
						f_read(&fi, (U8 *)buf_pr, audio_pr+Avix.StreamSize-(block_size<<1), &length);

						audio_pr = audio_pr+Avix.StreamSize-(block_size<<1);
						state_audio[1] = Audio_BUFFER_FULL;
					}
					else if ((audio_pr <= block_size) && (audio_pr+Avix.StreamSize) >= block_size)
					{
	//					if (state_audio[1] == Audio_BUFFER_FULL)
	//					{
	//						waitfor_move = 1;
	//						xSemaphoreTake(xSem_audio, osWaitForever);
	//						waitfor_move = 0;
	//
	//					}
						f_read(&fi, (U8 *)buf_pr+audio_pr, Avix.StreamSize, &length);
						state_audio[0] = Audio_BUFFER_FULL;
						audio_pr+=Avix.StreamSize;
					}
					else
					{
						f_read(&fi, (U8 *)buf_pr+audio_pr, Avix.StreamSize, &length);
						audio_pr+=Avix.StreamSize;
					}
					f_read(&fi, (U8 *)temp_data, 8, &length);
					_Avi_Get_Streaminfo(&Avix,temp_data);

				}
				else if (Avix.StreamID == AVI_VIDS_FLAG)
				{

					if (Avix.StreamSize >0)
					{
						if (Data_State[0]==JPEG_DATA_IDEL)
						{
							f_read(&fi, (U8 *)FrameBuffer, Avix.StreamSize, &length);
							JPEG_Size[0] = Avix.StreamSize;
							Data_State[0] = JPEG_DATA_READY;

						}

						else if (Data_State[1]==JPEG_DATA_IDEL)
						{
							f_read(&fi, (U8 *)FrameBuffer2, Avix.StreamSize, &length);
							JPEG_Size[1] = Avix.StreamSize;
							Data_State[1] = JPEG_DATA_READY;
						}
						else
						{
							f_lseek(&fi, f_tell(&fi)+ Avix.StreamSize);
						}
					}
					f_read(&fi, (U8 *)temp_data, 8, &length);
					_Avi_Get_Streaminfo(&Avix, temp_data);
				}

			}
		f_lseek(&fi, 0);
		}
		f_close(&fi);

	}
	while(1)
	{
		BSP_LED_Toggle(LED_RED);
		vTaskDelay(500);
	}

}
//higher number, higher priority
static void AppTaskCreate(void)
{
  //xTaskCreate(vTaskVolume, "TaskVolume", 512, NULL, 3, &xTaskVolume);
  xTaskCreate(vTaskMusic, "TaskMusic", 2048, NULL, 4, &xTaskMusic);
  xTaskCreate(vTaskGUI, "TaskGUI", 20240, NULL, 3, &xTaskGUI);
  xTaskCreate(vTaskTouchEx, "TaskTouchEx", 512, NULL, 1, &xTaskTouchEx);
  xTaskCreate(vTaskJpeg, "TaskJPEG", 2048, NULL, 4, &xTaskJpeg);

}
static void MPU_Config(void)
{
	MPU_Region_InitTypeDef MPU_InitStruct;

	  /* Disable the MPU */
	  HAL_MPU_Disable();

	  /* Configure the MPU as Normal Non Cacheable for the SRAM1 and SRAM2 */
	  MPU_InitStruct.Enable = MPU_REGION_ENABLE;
	  MPU_InitStruct.BaseAddress = 0x20020000;
	  MPU_InitStruct.Size = MPU_REGION_SIZE_512KB;
	  MPU_InitStruct.AccessPermission = MPU_REGION_FULL_ACCESS;
	  MPU_InitStruct.IsBufferable = MPU_ACCESS_NOT_BUFFERABLE;
	  MPU_InitStruct.IsCacheable = MPU_ACCESS_NOT_CACHEABLE;
	  MPU_InitStruct.IsShareable = MPU_ACCESS_SHAREABLE;
	  MPU_InitStruct.Number = MPU_REGION_NUMBER1;
	  MPU_InitStruct.TypeExtField = MPU_TEX_LEVEL1;
	  MPU_InitStruct.SubRegionDisable = 0x00;
	  MPU_InitStruct.DisableExec = MPU_INSTRUCTION_ACCESS_ENABLE;

	  HAL_MPU_ConfigRegion(&MPU_InitStruct);

	  /* Configure the MPU attributes as Non Cacheable and Non Bufferable for SDRAM */
	  MPU_InitStruct.Enable = MPU_REGION_ENABLE;
	  MPU_InitStruct.BaseAddress = 0xC0000000;
	  MPU_InitStruct.Size = MPU_REGION_SIZE_256MB;
	  MPU_InitStruct.AccessPermission = MPU_REGION_FULL_ACCESS;
	  MPU_InitStruct.IsBufferable = MPU_ACCESS_NOT_BUFFERABLE;
	  MPU_InitStruct.IsCacheable = MPU_ACCESS_NOT_CACHEABLE;
	  MPU_InitStruct.IsShareable = MPU_ACCESS_NOT_SHAREABLE;
	  MPU_InitStruct.Number = MPU_REGION_NUMBER2;
	  MPU_InitStruct.TypeExtField = MPU_TEX_LEVEL0;
	  MPU_InitStruct.SubRegionDisable = 0x00;
	  MPU_InitStruct.DisableExec = MPU_INSTRUCTION_ACCESS_ENABLE;

	  HAL_MPU_ConfigRegion(&MPU_InitStruct);

	  /* Configure the MPU attributes as WT for SDRAM */
	  MPU_InitStruct.Enable = MPU_REGION_ENABLE;
	  MPU_InitStruct.BaseAddress = 0xC0000000;
	  MPU_InitStruct.Size = MPU_REGION_SIZE_8MB;
	  MPU_InitStruct.AccessPermission = MPU_REGION_FULL_ACCESS;
	  MPU_InitStruct.IsBufferable = MPU_ACCESS_NOT_BUFFERABLE;
	  MPU_InitStruct.IsCacheable = MPU_ACCESS_CACHEABLE;
	  MPU_InitStruct.IsShareable = MPU_ACCESS_NOT_SHAREABLE;
	  MPU_InitStruct.Number = MPU_REGION_NUMBER3;
	  MPU_InitStruct.TypeExtField = MPU_TEX_LEVEL0;
	  MPU_InitStruct.SubRegionDisable = 0x00;
	  MPU_InitStruct.DisableExec = MPU_INSTRUCTION_ACCESS_ENABLE;

	  HAL_MPU_ConfigRegion(&MPU_InitStruct);

	  /* Configure the MPU attributes as strongly ordred for QSPI (unused zone) */
	  MPU_InitStruct.Enable = MPU_REGION_ENABLE;
	  MPU_InitStruct.BaseAddress = 0x90000000;
	  MPU_InitStruct.Size = MPU_REGION_SIZE_256MB;
	  MPU_InitStruct.AccessPermission = MPU_REGION_PRIV_RO;
	  MPU_InitStruct.IsBufferable = MPU_ACCESS_NOT_BUFFERABLE;
	  MPU_InitStruct.IsCacheable = MPU_ACCESS_NOT_CACHEABLE;
	  MPU_InitStruct.IsShareable = MPU_ACCESS_NOT_SHAREABLE;
	  MPU_InitStruct.Number = MPU_REGION_NUMBER4;
	  MPU_InitStruct.TypeExtField = MPU_TEX_LEVEL0;
	  MPU_InitStruct.SubRegionDisable = 0x00;
	  MPU_InitStruct.DisableExec = MPU_INSTRUCTION_ACCESS_ENABLE;

	  HAL_MPU_ConfigRegion(&MPU_InitStruct);

	  /* Configure the MPU attributes as WT for QSPI (used 16Mbytes) */
	  MPU_InitStruct.Enable = MPU_REGION_ENABLE;
	  MPU_InitStruct.BaseAddress = 0x90000000;
	  MPU_InitStruct.Size = MPU_REGION_SIZE_16MB;
	  MPU_InitStruct.AccessPermission = MPU_REGION_PRIV_RO;
	  MPU_InitStruct.IsBufferable = MPU_ACCESS_NOT_BUFFERABLE;
	  MPU_InitStruct.IsCacheable = MPU_ACCESS_CACHEABLE;
	  MPU_InitStruct.IsShareable = MPU_ACCESS_NOT_SHAREABLE;
	  MPU_InitStruct.Number = MPU_REGION_NUMBER5;
	  MPU_InitStruct.TypeExtField = MPU_TEX_LEVEL0;
	  MPU_InitStruct.SubRegionDisable = 0x00;
	  MPU_InitStruct.DisableExec = MPU_INSTRUCTION_ACCESS_ENABLE;

	  HAL_MPU_ConfigRegion(&MPU_InitStruct);

	  /* Enable the MPU */
	  HAL_MPU_Enable(MPU_PRIVILEGED_DEFAULT);
}
int main(void)
{
	/* USER CODE BEGIN 1 */
	/* USER CODE END 1 */

	
	uint8_t SD_state = MSD_OK, res;
	



	MPU_Config();
	SCB_InvalidateICache();

	/* Enable branch prediction */
	SCB->CCR |= (1 <<18);
	__DSB();

	/* Invalidate I-Cache : ICIALLU register*/
	SCB_InvalidateICache();

	/* Enable I-Cache */
	SCB_EnableICache();

	SCB_InvalidateDCache();
	SCB_EnableDCache();

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
	HAL_Init();

	/* USER CODE BEGIN Init */
	/* USER CODE END Init */

	/* Configure the system clock */
	SystemClock_Config();

	/* USER CODE BEGIN SysInit */
	/* USER CODE END SysInit */

	/* Initialize all configured peripherals */
	/* Enable the CRC Module */
	__HAL_RCC_CRC_CLK_ENABLE();
	MX_RTC_Init();
	//MPU_Config();


	/*##-1- JPEG Initialization ################################################*/
	/* Init The JPEG Color Look Up Tables used for YCbCr to RGB conversion   */

	/* Init the HAL JPEG driver */
	//JPEG_Handle.Instance = JPEG;


	DMAUartHandle.Instance				  = DMA2_Stream7;
	DMAUartHandle.Init.Channel             = DMA_CHANNEL_4;
	DMAUartHandle.Init.Direction           = DMA_MEMORY_TO_PERIPH;
	DMAUartHandle.Init.PeriphInc           = DMA_PINC_DISABLE;
	DMAUartHandle.Init.MemInc              = DMA_MINC_ENABLE;
	DMAUartHandle.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
	DMAUartHandle.Init.MemDataAlignment    = DMA_MDATAALIGN_BYTE;
	DMAUartHandle.Init.Mode                = DMA_NORMAL;
	DMAUartHandle.Init.Priority            = DMA_PRIORITY_LOW;
	DMAUartHandle.Init.FIFOMode            = DMA_FIFOMODE_DISABLE;
	HAL_DMA_Init(&DMAUartHandle);

	HAL_NVIC_SetPriority(DMA2_Stream7_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(DMA2_Stream7_IRQn);


	UartHandle.Instance        = USARTx;
	UartHandle.Init.BaudRate   = 9600;
	UartHandle.Init.WordLength = UART_WORDLENGTH_8B;
	UartHandle.Init.StopBits   = UART_STOPBITS_1;
	UartHandle.Init.Parity     = UART_PARITY_ODD;
	UartHandle.Init.HwFlowCtl  = UART_HWCONTROL_NONE;
	UartHandle.Init.Mode       = UART_MODE_TX_RX;
	UartHandle.Init.OverSampling = UART_OVERSAMPLING_16;
	if (HAL_UART_Init(&UartHandle) != HAL_OK)
	{
	  /* Initialization Error */
	  Error_Handler();
	}
	__HAL_LINKDMA(&UartHandle, hdmatx, DMAUartHandle);
	/* Peripheral interrupt init*/
	HAL_NVIC_SetPriority(USART1_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(USART1_IRQn);

	/* USER CODE BEGIN 2 */
	// gif decode callback function set
	/* Initializes the SDRAM device */
	BSP_SDRAM_Init();
	BSP_LED_Init(LED1);
	BSP_LED_Init(LED2);
	BSP_LED_Init(LED3);
	BSP_TS_Init (800, 480);
	BSP_TS_ITConfig();
	GUI_Init();
	GUI_SetBkColor(GUI_WHITE);
	GUI_Clear();

	GUI_SetLayerVisEx (1, 0);
	GUI_SelectLayer(0);
	WM_SetCreateFlags(WM_CF_MEMDEV);
	WM_MULTIBUF_Enable(1);

	AppTaskCreate();


	vTaskStartScheduler();

	while(1);



    /* USER CODE BEGIN 3 */
  /* USER CODE END 3 */
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  BaseType_t xHigherPriorityTaskWoken = pdFALSE;

	if (TS_INT_PIN == GPIO_Pin)
	{
		vTaskNotifyGiveFromISR( xTaskTouchEx, &xHigherPriorityTaskWoken );
		portYIELD_FROM_ISR( xHigherPriorityTaskWoken);
	}
}
/**
  * @brief System Clock Configuration
  * @retval None
  */
 void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

  /**Configure LSE Drive Capability
  */
  HAL_PWR_EnableBkUpAccess();
  __HAL_RCC_LSEDRIVE_CONFIG(RCC_LSEDRIVE_LOW);
  /**Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /**Initializes the CPU, AHB and APB busses clocks
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE|RCC_OSCILLATORTYPE_LSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 25;
  RCC_OscInitStruct.PLL.PLLN = 432;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /**Activate the Over-Drive mode
  */
  if (HAL_PWREx_EnableOverDrive() != HAL_OK)
  {
    Error_Handler();
  }
  /**Initializes the CPU, AHB and APB busses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_7) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_LTDC|RCC_PERIPHCLK_RTC;
  PeriphClkInitStruct.PLLSAI.PLLSAIN = 384;
  PeriphClkInitStruct.PLLSAI.PLLSAIR = 7;
  PeriphClkInitStruct.PLLSAI.PLLSAIQ = 2;
  PeriphClkInitStruct.PLLSAI.PLLSAIP = RCC_PLLSAIP_DIV2;
  PeriphClkInitStruct.PLLSAIDivQ = 1;
  PeriphClkInitStruct.PLLSAIDivR = RCC_PLLSAIDIVR_2;
  PeriphClkInitStruct.RTCClockSelection = RCC_RTCCLKSOURCE_LSE;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
}




/**
  * @brief RTC Initialization Function
  * @param None
  * @retval None
  */
static void MX_RTC_Init(void)
{

  /* USER CODE BEGIN RTC_Init 0 */
  /* USER CODE END RTC_Init 0 */

  /* USER CODE BEGIN RTC_Init 1 */
  /* USER CODE END RTC_Init 1 */
  /**Initialize RTC Only 
  */
  hrtc.Instance = RTC;
  hrtc.Init.HourFormat = RTC_HOURFORMAT_24;
  hrtc.Init.AsynchPrediv = 127;
  hrtc.Init.SynchPrediv = 255;
  hrtc.Init.OutPut = RTC_OUTPUT_DISABLE;
  hrtc.Init.OutPutPolarity = RTC_OUTPUT_POLARITY_HIGH;
  hrtc.Init.OutPutType = RTC_OUTPUT_TYPE_OPENDRAIN;
  if (HAL_RTC_Init(&hrtc) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN RTC_Init 2 */
  /* USER CODE END RTC_Init 2 */

}





/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* USER CODE END Error_Handler_Debug */
}

/**
  * @brief  SAI MSP Init.
  * @param  hsai : pointer to a SAI_HandleTypeDef structure that contains
  *                the configuration information for SAI module.
  * @retval None
  */
void HAL_SAI_MspInit(SAI_HandleTypeDef *hsai)
{
  GPIO_InitTypeDef  GPIO_Init;
  
  /* Enable SAI1 clock */
  __HAL_RCC_SAI1_CLK_ENABLE();
  
  /* Configure GPIOs used for SAI2 */
  AUDIO_SAIx_MCLK_ENABLE();
  AUDIO_SAIx_SCK_ENABLE();
  AUDIO_SAIx_FS_ENABLE();
  AUDIO_SAIx_SD_ENABLE();
  
  GPIO_Init.Mode      = GPIO_MODE_AF_PP;
  GPIO_Init.Pull      = GPIO_NOPULL;
  GPIO_Init.Speed     = GPIO_SPEED_FREQ_VERY_HIGH;
  
  GPIO_Init.Alternate = AUDIO_SAIx_FS_AF;
  GPIO_Init.Pin       = AUDIO_SAIx_FS_PIN;
  HAL_GPIO_Init(AUDIO_SAIx_FS_GPIO_PORT, &GPIO_Init);
  GPIO_Init.Alternate = AUDIO_SAIx_SCK_AF;
  GPIO_Init.Pin       = AUDIO_SAIx_SCK_PIN;
  HAL_GPIO_Init(AUDIO_SAIx_SCK_GPIO_PORT, &GPIO_Init);
  GPIO_Init.Alternate = AUDIO_SAIx_SD_AF;
  GPIO_Init.Pin       = AUDIO_SAIx_SD_PIN;
  HAL_GPIO_Init(AUDIO_SAIx_SD_GPIO_PORT, &GPIO_Init);
  GPIO_Init.Alternate = AUDIO_SAIx_MCLK_AF;
  GPIO_Init.Pin       = AUDIO_SAIx_MCLK_PIN;
  HAL_GPIO_Init(AUDIO_SAIx_MCLK_GPIO_PORT, &GPIO_Init);
  
  /* Configure DMA used for SAI2 */
  __HAL_RCC_DMA2_CLK_ENABLE();

  if(hsai->Instance == AUDIO_SAIx)
  {
    hSaiDma.Init.Channel             = DMA_CHANNEL_10;
    hSaiDma.Init.Direction           = DMA_MEMORY_TO_PERIPH;
    hSaiDma.Init.PeriphInc           = DMA_PINC_DISABLE;
    hSaiDma.Init.MemInc              = DMA_MINC_ENABLE;
    hSaiDma.Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD;
    hSaiDma.Init.MemDataAlignment    = DMA_MDATAALIGN_HALFWORD;
    hSaiDma.Init.Mode                = DMA_CIRCULAR;
    hSaiDma.Init.Priority            = DMA_PRIORITY_HIGH;
    hSaiDma.Init.FIFOMode            = DMA_FIFOMODE_ENABLE;      
    hSaiDma.Init.FIFOThreshold       = DMA_FIFO_THRESHOLD_FULL;
    hSaiDma.Init.MemBurst            = DMA_MBURST_SINGLE;         
    hSaiDma.Init.PeriphBurst         = DMA_PBURST_SINGLE;         

    /* Select the DMA instance to be used for the transfer : DMA2_Stream6 */
    hSaiDma.Instance                 = DMA2_Stream6;
  
    /* Associate the DMA handle */
    __HAL_LINKDMA(hsai, hdmatx, hSaiDma);

    /* Deinitialize the Stream for new transfer */
    HAL_DMA_DeInit(&hSaiDma);

    /* Configure the DMA Stream */
    if (HAL_OK != HAL_DMA_Init(&hSaiDma))
    {
      Error_Handler();
    }
  }
	
  HAL_NVIC_SetPriority(DMA2_Stream6_IRQn, 0x01, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream6_IRQn);
}

/**
  * @brief Tx Transfer completed callbacks.
  * @param  hsai : pointer to a SAI_HandleTypeDef structure that contains
  *                the configuration information for SAI module.
  * @retval None
  */

void BSP_AUDIO_OUT_TransferComplete_CallBack()
{
	//unsigned int len;
  /* Manage the remaining file size and new address offset: This function
     should be coded by user (its prototype is already declared in stm32f769i_discovery_audio.h) */

  BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    /* Notify the task that the transmission is complete. */
    xTaskNotifyFromISR( xTaskMusic, 1, eSetBits, &xHigherPriorityTaskWoken);


    /* If xHigherPriorityTaskWoken is now set to pdTRUE then a context switch
    should be performed to ensure the interrupt returns directly to the highest
    priority task.  The macro used for this purpose is dependent on the port in
    use and may be called portEND_SWITCHING_ISR(). */
    portYIELD_FROM_ISR( xHigherPriorityTaskWoken );
}

void HAL_SAI_TxCpltCallback(SAI_HandleTypeDef *hsai)
{
	BSP_AUDIO_OUT_TransferComplete_CallBack();
}
/**
  * @brief Tx Transfer Half completed callbacks
  * @param  hsai : pointer to a SAI_HandleTypeDef structure that contains
  *                the configuration information for SAI module.
  * @retval None
  */
void BSP_AUDIO_OUT_HalfTransfer_CallBack()
{
  //unsigned int len;
  /* Manage the remaining file size and new address offset: This function
     should be coded by user (its prototype is already declared in stm32f769i_discovery_audio.h) */
	//f_read(&fi, &buff[0], block_size, &len);
  BaseType_t xHigherPriorityTaskWoken = pdFALSE;

  /* Notify the task that the transmission is complete. */
  xTaskNotifyFromISR( xTaskMusic, 2, eSetBits, &xHigherPriorityTaskWoken );


  /* If xHigherPriorityTaskWoken is now set to pdTRUE then a context switch
  should be performed to ensure the interrupt returns directly to the highest
  priority task.  The macro used for this purpose is dependent on the port in
  use and may be called portEND_SWITCHING_ISR(). */
  portYIELD_FROM_ISR( xHigherPriorityTaskWoken );
}
void HAL_SAI_TxHalfCpltCallback(SAI_HandleTypeDef *hsai)
{
	BSP_AUDIO_OUT_HalfTransfer_CallBack();
}
#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
