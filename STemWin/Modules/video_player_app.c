/**
  ******************************************************************************
  * @file    videoplayer_app.c
  * @author  MCD Application Team
  * @brief   This file provides routines for JPEG decoding with DMA method.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2016 STMicroelectronics International N.V. 
  * All rights reserved.</center></h2>
  *
  * Redistribution and use in source and binary forms, with or without 
  * modification, are permitted, provided that the following conditions are met:
  *
  * 1. Redistribution of source code must retain the above copyright notice, 
  *    this list of conditions and the following disclaimer.
  * 2. Redistributions in binary form must reproduce the above copyright notice,
  *    this list of conditions and the following disclaimer in the documentation
  *    and/or other materials provided with the distribution.
  * 3. Neither the name of STMicroelectronics nor the names of other 
  *    contributors to this software may be used to endorse or promote products 
  *    derived from this software without specific written permission.
  * 4. This software, including modifications and/or derivative works of this 
  *    software, must execute solely and exclusively on microcontroller or
  *    microprocessor devices manufactured by or for STMicroelectronics.
  * 5. Redistribution and use of this software other than as permitted under 
  *    this license is void and will automatically terminate your rights under 
  *    this license. 
  *
  * THIS SOFTWARE IS PROVIDED BY STMICROELECTRONICS AND CONTRIBUTORS "AS IS" 
  * AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT 
  * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A 
  * PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
  * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT 
  * SHALL STMICROELECTRONICS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
  * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, 
  * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF 
  * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING 
  * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
  * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "video_player_app.h"
#include "jpeg_utils.h"
#include "LCDConf.h"
#include "GUI_AVI.h"

/** @addtogroup STM32F7xx_HAL_Examples
  * @{
  */

/** @addtogroup MJPEG_VideoDecoding
  * @{
  */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
#define DECODE_CONVERT_OUTPUT_BUFF    0x11
#define JPEG_DATA_INPUT				  0x20
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
JPEG_YCbCrToRGB_Convert_Function pConvert_Function;
JPEG_HandleTypeDef    JPEG_Handle;

#if defined ( __ICCARM__ ) /* IAR Compiler */
#pragma location=0x2000A000
uint32_t MCU_Data_OutBuffer[CHUNK_SIZE_OUT/4];

#elif defined ( __CC_ARM ) /* Keil Compiler */
uint32_t MCU_Data_OutBuffer[CHUNK_SIZE_OUT/4] __attribute__((at(0x2000A000)));

#elif defined ( __GNUC__ ) /* GNU Compiler */
uint32_t MCU_Data_OutBuffer[CHUNK_SIZE_OUT/4] __attribute__((section(".MCU_Data_section")));
#endif
extern uint8_t    FrameBuffer[AVI_VIDEO_BUF_SIZE];
extern GUI_AVI_HANDLE havi;
static uint32_t MCU_BlockIndex = 0;
osThreadId hOutputThread;
static osMessageQId OutputEvent = 0;
uint32_t MCU_TotalNb = 0, IsFirstTime = 0;
static uint32_t OutputLength=0;
osSemaphoreId osVidSemph;
/* Private function prototypes -----------------------------------------------*/
static void OutputThread(void const *argument);
extern void LCD_LL_DrawBitmap16bpp(int LayerIndex, int x, int y, U16 const * p, int xSize, int ySize, int BytesPerLine);
void TransferComplete_CallBack(void);
void HalfTransfer_CallBack(void);

/* Private functions ---------------------------------------------------------*/

/**
  * @brief  Init HArdware Decoder
  * @param  None
  * @retval None
  */
void HW_JPEG_Init(void)
{
  static DMA_HandleTypeDef   hdmaIn;
  static DMA_HandleTypeDef   hdmaOut;
  
  JPEG_InitColorTables();
  
    /* Enable JPEG clock */
  __HAL_RCC_JPEG_CLK_ENABLE();
    /* Enable DMA clock */
  __DMA2_CLK_ENABLE();  
  
  HAL_NVIC_SetPriority(JPEG_IRQn, 0x06, 0x0F);
  HAL_NVIC_EnableIRQ(JPEG_IRQn);
  
  /* Input DMA */    
  /* Set the parameters to be configured */
  hdmaIn.Init.Channel = DMA_CHANNEL_9;
  hdmaIn.Init.Direction = DMA_MEMORY_TO_PERIPH;
  hdmaIn.Init.PeriphInc = DMA_PINC_DISABLE;
  hdmaIn.Init.MemInc = DMA_MINC_ENABLE;
  hdmaIn.Init.PeriphDataAlignment = DMA_PDATAALIGN_WORD;
  hdmaIn.Init.MemDataAlignment = DMA_MDATAALIGN_WORD;
  hdmaIn.Init.Mode = DMA_NORMAL;
  hdmaIn.Init.Priority = DMA_PRIORITY_HIGH;
  hdmaIn.Init.FIFOMode = DMA_FIFOMODE_ENABLE;         
  hdmaIn.Init.FIFOThreshold = DMA_FIFO_THRESHOLD_FULL;
  hdmaIn.Init.MemBurst = DMA_MBURST_INC4;
  hdmaIn.Init.PeriphBurst = DMA_PBURST_INC4;      
  
  hdmaIn.Instance = DMA2_Stream3;
  
  /* Associate the DMA handle */
  __HAL_LINKDMA(&JPEG_Handle, hdmain, hdmaIn);
  
  HAL_NVIC_SetPriority(DMA2_Stream3_IRQn, 0x06, 0x0F);
  HAL_NVIC_EnableIRQ(DMA2_Stream3_IRQn);    
  
  /* DeInitialize the DMA Stream */
  HAL_DMA_DeInit(&hdmaIn);  
  /* Initialize the DMA stream */
  HAL_DMA_Init(&hdmaIn);
  
  
  /* Output DMA */
  /* Set the parameters to be configured */ 
  hdmaOut.Init.Channel = DMA_CHANNEL_9;
  hdmaOut.Init.Direction = DMA_PERIPH_TO_MEMORY;
  hdmaOut.Init.PeriphInc = DMA_PINC_DISABLE;
  hdmaOut.Init.MemInc = DMA_MINC_ENABLE;
  hdmaOut.Init.PeriphDataAlignment = DMA_PDATAALIGN_WORD;
  hdmaOut.Init.MemDataAlignment = DMA_MDATAALIGN_WORD;
  hdmaOut.Init.Mode = DMA_NORMAL;
  hdmaOut.Init.Priority = DMA_PRIORITY_VERY_HIGH;
  hdmaOut.Init.FIFOMode = DMA_FIFOMODE_ENABLE;         
  hdmaOut.Init.FIFOThreshold = DMA_FIFO_THRESHOLD_FULL;
  hdmaOut.Init.MemBurst = DMA_MBURST_INC4;
 
  hdmaOut.Init.PeriphBurst = DMA_PBURST_INC4;

  hdmaOut.Instance = DMA2_Stream4;
  /* DeInitialize the DMA Stream */
  HAL_DMA_DeInit(&hdmaOut);  
  /* Initialize the DMA stream */
  HAL_DMA_Init(&hdmaOut);

  /* Associate the DMA handle */
  __HAL_LINKDMA(&JPEG_Handle, hdmaout, hdmaOut);
  
  HAL_NVIC_SetPriority(DMA2_Stream4_IRQn, 0x07, 0x0F);
  HAL_NVIC_EnableIRQ(DMA2_Stream4_IRQn);  
  
   /* Init the HAL JPEG driver */
  JPEG_Handle.Instance = JPEG;
  HAL_JPEG_Init(&JPEG_Handle);

    /* Output FRAME Queue */
  osMessageQDef(OUTPUT_Queue, 3, uint16_t);
  OutputEvent = osMessageCreate (osMessageQ(OUTPUT_Queue), NULL); 
  
  /* Output Thread  definition */
  osThreadDef(OUTPUT_THREAD, OutputThread, osPriorityRealtime, 0, configMINIMAL_STACK_SIZE);
  hOutputThread = osThreadCreate(osThread(OUTPUT_THREAD), NULL);
  
    /* Create the Semaphore used by the two threads */
    /* Create Semaphore lock */
  osSemaphoreDef(Semaphore);
  osVidSemph= osSemaphoreCreate(osSemaphore(Semaphore), 1);  
  osSemaphoreWait(osVidSemph , osWaitForever); 
}


/**
  * @brief  Stop HArdware Decoder
  * @param  None
  * @retval None
  */
void HW_JPEG_DeInit(void)
{ 
  HAL_JPEG_DeInit(&JPEG_Handle);
  HAL_DMA_DeInit(JPEG_Handle.hdmaout);  
  __HAL_RCC_JPEG_CLK_DISABLE();
  
  if(hOutputThread)
  {
    osThreadTerminate(hOutputThread);
    hOutputThread = 0;
  }
  
  if(osVidSemph)
  {
    osSemaphoreDelete(osVidSemph);
    osVidSemph = 0;
  }
  
  if(OutputEvent)
  {
    vQueueDelete(OutputEvent); 
    OutputEvent = 0;  
  }
}

/**
  * @brief  Output Jpeg task
  * @param  None
  * @retval None
  */
static void OutputThread(void const *argument)
{
  uint32_t ConvertedDataCount;
  osEvent event;
  volatile unsigned int length;
  for(;;)
  {
    event = osMessageGet(OutputEvent, osWaitForever );
    
    if( event.status == osEventMessage )
    {
      switch(event.value.v)
      {
		  case DECODE_CONVERT_OUTPUT_BUFF:
			MCU_BlockIndex += pConvert_Function( (uint8_t *)MCU_Data_OutBuffer, (uint8_t *)0xC0000000, MCU_BlockIndex, OutputLength, &ConvertedDataCount);

			if((MCU_BlockIndex == MCU_TotalNb) && (MCU_TotalNb != 0))
			{
				LCD_LL_DrawBitmap16bpp(0, (800 - JPEG_Handle.Conf.ImageWidth)/2, (480 - JPEG_Handle.Conf.ImageHeight)/2 \
						, (uint16_t *)0xC0000000, JPEG_Handle.Conf.ImageWidth, JPEG_Handle.Conf.ImageHeight, JPEG_Handle.Conf.ImageWidth * 2);
			}
			else
			{
			  HAL_JPEG_Resume(&JPEG_Handle, JPEG_PAUSE_RESUME_OUTPUT);
			}
			break;
		  case JPEG_DATA_INPUT:
			  length = Get_data();
			  HAL_JPEG_ConfigInputBuffer(&JPEG_Handle, FrameBuffer, length);
			  HAL_JPEG_Resume(&JPEG_Handle, JPEG_PAUSE_RESUME_INPUT);

      }
    } 
  }
}

/**
  * @brief  Decode_DMA
  * @param hjpeg: JPEG handle pointer
  * @param  FileName    : jpg file path for decode.
  * @param  DestAddress : ARGB destination Frame Buffer Address.
  * @retval None
  */
U32 HW_JPEG_Draw (const void * pFileData, U32 DataSize, U32 x0, U32 y0)
{  
  MCU_BlockIndex = 0;
  MCU_TotalNb = 0;
  HAL_JPEG_Decode_DMA(&JPEG_Handle , (uint8_t *)pFileData ,DataSize ,(uint8_t *)MCU_Data_OutBuffer ,CHUNK_SIZE_OUT);
  if(osSemaphoreWait(osVidSemph , 1000) == osErrorOS)
  {
    return 1;
  }
  return 0;
}

/**
  * @brief  JPEG Data Ready callback
  * @param hjpeg: JPEG handle pointer
  * @param pDataOut: pointer to the output data buffer
  * @param OutDataLength: length of output buffer in bytes
  * @retval None
  */
void HAL_JPEG_DataReadyCallback (JPEG_HandleTypeDef *hjpeg, uint8_t *pDataOut, uint32_t OutDataLength)
{
  HAL_JPEG_Pause(hjpeg, JPEG_PAUSE_RESUME_OUTPUT);
  HAL_JPEG_ConfigOutputBuffer(hjpeg, (uint8_t *)MCU_Data_OutBuffer, CHUNK_SIZE_OUT); 
  OutputLength = OutDataLength;
  osMessagePut ( OutputEvent, DECODE_CONVERT_OUTPUT_BUFF, 0);
}

/**
  * @brief  JPEG Decode complete callback
  * @param hjpeg: JPEG handle pointer
  * @retval None
  */
void HAL_JPEG_DecodeCpltCallback(JPEG_HandleTypeDef *hjpeg)
{
  osSemaphoreRelease(osVidSemph);
}

/**
  * @brief  JPEG Get Data callback
  * @param hjpeg: JPEG handle pointer
  * @param NbDecodedData: Number of decoded (consummed) bytes from input buffer
  * @retval None
  */
void HAL_JPEG_GetDataCallback(JPEG_HandleTypeDef *hjpeg, uint32_t NbDecodedData)
{
	if (NbDecodedData == AVI_VIDEO_BUF_SIZE)
	{
		HAL_JPEG_Pause(hjpeg, JPEG_PAUSE_RESUME_INPUT);
		osMessagePut ( OutputEvent, JPEG_DATA_INPUT, 0);
	}
	else
	{
		HAL_JPEG_ConfigInputBuffer(hjpeg, FrameBuffer+NbDecodedData, AVI_VIDEO_BUF_SIZE-NbDecodedData);
	}
}
/**
  * @brief  JPEG Info ready callback
  * @param hjpeg: JPEG handle pointer
  * @param pInfo: JPEG Info Struct pointer
  * @retval None
  */
void HAL_JPEG_InfoReadyCallback(JPEG_HandleTypeDef *hjpeg, JPEG_ConfTypeDef *pInfo)
{
	if(pInfo->ChromaSubsampling == JPEG_420_SUBSAMPLING)
	  {
	    if((pInfo->ImageWidth % 16) != 0)
	    pInfo->ImageWidth += (16 - (pInfo->ImageWidth % 16));

	    if((pInfo->ImageHeight % 16) != 0)
	    pInfo->ImageHeight += (16 - (pInfo->ImageHeight % 16));
	  }

	  if(pInfo->ChromaSubsampling == JPEG_422_SUBSAMPLING)
	  {
	    if((pInfo->ImageWidth % 16) != 0)
	    pInfo->ImageWidth += (16 - (pInfo->ImageWidth % 16));

	    if((pInfo->ImageHeight % 8) != 0)
	    pInfo->ImageHeight += (8 - (pInfo->ImageHeight % 8));
	  }

	  if(pInfo->ChromaSubsampling == JPEG_444_SUBSAMPLING)
	  {
	    if((pInfo->ImageWidth % 8) != 0)
	    pInfo->ImageWidth += (8 - (pInfo->ImageWidth % 8));

	    if((pInfo->ImageHeight % 8) != 0)
	    pInfo->ImageHeight += (8 - (pInfo->ImageHeight % 8));
	  }
  if(JPEG_GetDecodeColorConvertFunc(pInfo, &pConvert_Function, &MCU_TotalNb) == HAL_OK)
  {
  }   
}
/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
