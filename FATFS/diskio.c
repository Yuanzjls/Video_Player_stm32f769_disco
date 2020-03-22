/*-----------------------------------------------------------------------*/
/* Low level disk I/O module skeleton for FatFs     (C)ChaN, 2016        */
/*-----------------------------------------------------------------------*/
/* If a working storage control module is available, it should be        */
/* attached to the FatFs via a glue function rather than modifying it.   */
/* This is an example of glue functions to attach various exsisting      */
/* storage control modules to the FatFs module with a defined API.       */
/*-----------------------------------------------------------------------*/

#include "ff.h"			/* Obtains integer types */
#include "diskio.h"		/* Declarations of disk functions */
#include "stm32f769i_discovery_sd.h"
#include "cmsis_os.h"
/* Definitions of physical drive number for each drive */
#define DEV_RAM		1	/* Example: Map Ramdisk to physical drive 0 */
#define DEV_MMC		0	/* Example: Map MMC/SD card to physical drive 1 */
#define DEV_USB		2	/* Example: Map USB MSD to physical drive 2 */

#define SD_TIMEOUT 1000
#define SD_DEFAULT_BLOCK_SIZE 512

#define QUEUE_SIZE         (uint32_t) 10
#define READ_CPLT_MSG      (uint32_t) 1
#define WRITE_CPLT_MSG     (uint32_t) 2
static osMessageQId SDQueueID;
/*-----------------------------------------------------------------------*/
/* Get Drive Status                                                      */
/*-----------------------------------------------------------------------*/

DSTATUS disk_status (
	BYTE pdrv		/* Physical drive nmuber to identify the drive */
)
{
	DSTATUS stat;
	int result;

	switch (pdrv) {
	case DEV_RAM :
		//result = RAM_disk_status();

		// translate the reslut code here

		return stat;

	case DEV_MMC :
		 stat = STA_NOINIT;

		  if(BSP_SD_GetCardState() == MSD_OK)
		  {
		    stat &= ~STA_NOINIT;
		  }

		// translate the reslut code here

		return stat;

	case DEV_USB :
		//result = USB_disk_status();

		// translate the reslut code here

		return stat;
	}
	return STA_NOINIT;
}


/*-----------------------------------------------------------------------*/
/* Inidialize a Drive                                                    */
/*-----------------------------------------------------------------------*/

DSTATUS disk_initialize (
	BYTE pdrv				/* Physical drive nmuber to identify the drive */
)
{
	DSTATUS stat;
	int result;

	switch (pdrv) {
	case DEV_RAM :
		//result = RAM_disk_initialize();

		// translate the reslut code here

		return stat;

	case DEV_MMC :
		stat = BSP_SD_Init();

		osMessageQDef(SD_Queue, QUEUE_SIZE, uint16_t);
		SDQueueID = osMessageCreate (osMessageQ(SD_Queue), NULL);
		// translate the reslut code here

		return stat;

	case DEV_USB :
		//result = USB_disk_initialize();

		// translate the reslut code here

		return stat;
	}
	return STA_NOINIT;
}



/*-----------------------------------------------------------------------*/
/* Read Sector(s)                                                        */
/*-----------------------------------------------------------------------*/
// To do, may add usb drive in the future, it needs another variable likeulNotificationValue.


DRESULT disk_read (
	BYTE pdrv,		/* Physical drive nmuber to identify the drive */
	BYTE *buff,		/* Data buffer to store read data */
	DWORD sector,	/* Start sector in LBA */
	UINT count		/* Number of sectors to read */
)
{
	DRESULT res;
	osEvent event;
	uint32_t timer;
	uint32_t alignedAddr;
	switch (pdrv) {
	case DEV_RAM :
		// translate the arguments here

		//result = RAM_disk_read(buff, sector, count);

		// translate the reslut code here

		return res;

	case DEV_MMC :
		// translate the arguments here

		res = RES_ERROR;

		  /*
		    the SCB_CleanDCache_by_Addr() requires a 32-Byte aligned address
		    adjust the address and the D-Cache size to clean accordingly.
		  */
		if(BSP_SD_ReadBlocks_DMA((uint32_t*)buff,
		                           (uint32_t) (sector),
		                           count) == MSD_OK)
		  {
		    /* wait for a message from the queue or a timeout */
		    event = osMessageGet(SDQueueID, SD_TIMEOUT);

		    if (event.status == osEventMessage)
		    {
		      if (event.value.v == READ_CPLT_MSG)
		      {
		        timer = osKernelSysTick() + SD_TIMEOUT;
		        /* block until SDIO IP is ready or a timeout occur */
		        while(timer > osKernelSysTick())
		        {
		          if (BSP_SD_GetCardState() == SD_TRANSFER_OK)
		          {
		            res = RES_OK;
//		            alignedAddr = (uint32_t)buff & ~0x1F;
//		            SCB_InvalidateDCache_by_Addr((uint32_t*)alignedAddr, count*BLOCKSIZE + ((uint32_t)buff - alignedAddr));
		            break;
				  }
			    }
			  }
		    }
		  }


		  return res;

		// translate the reslut code here



	case DEV_USB :
		// translate the arguments here

		//result = USB_disk_read(buff, sector, count);

		// translate the reslut code here

		return res;
	}

	return RES_PARERR;
}



/*-----------------------------------------------------------------------*/
/* Write Sector(s)                                                       */
/*-----------------------------------------------------------------------*/

#if FF_FS_READONLY == 0

DRESULT disk_write (
	BYTE pdrv,			/* Physical drive nmuber to identify the drive */
	const BYTE *buff,	/* Data to be written */
	DWORD sector,		/* Start sector in LBA */
	UINT count			/* Number of sectors to write */
)
{
	DRESULT res;
	osEvent event;


	switch (pdrv) {
	case DEV_RAM :
		// translate the arguments here

		//result = RAM_disk_write(buff, sector, count);

		// translate the reslut code here

		return res;

	case DEV_MMC :
		// translate the arguments here

		res = RES_ERROR;

		if(BSP_SD_WriteBlocks_DMA((uint32_t*)buff,
		                           (uint32_t) (sector),
		                           count) == MSD_OK)
		  {
		    /* Get the message from the queue */
		    event = osMessageGet(SDQueueID, SD_TIMEOUT);

		    if (event.status == osEventMessage)
		    {
		      if (event.value.v == WRITE_CPLT_MSG)
		      {
		        timer = osKernelSysTick() + SD_TIMEOUT;
		        /* block until SDIO IP is ready or a timeout occur */
		        while(timer > osKernelSysTick())
		        {
		          if (BSP_SD_GetCardState() == SD_TRANSFER_OK)
		          {
		            res = RES_OK;
		            break;
		          }
		        }
		      }
		    }
		  }

		  return res;

		// translate the reslut code here



	case DEV_USB :
		// translate the arguments here

		//result = USB_disk_write(buff, sector, count);

		// translate the reslut code here

		return res;
	}

	return RES_PARERR;
}

#endif


/*-----------------------------------------------------------------------*/
/* Miscellaneous Functions                                               */
/*-----------------------------------------------------------------------*/

DRESULT disk_ioctl (
	BYTE pdrv,		/* Physical drive nmuber (0..) */
	BYTE cmd,		/* Control code */
	void *buff		/* Buffer to send/receive control data */
)
{
	DRESULT res;
	int result;

	switch (pdrv) {
	case DEV_RAM :

		// Process of the command for the RAM drive

		return res;

	case DEV_MMC :

		 res = RES_ERROR;
		 HAL_SD_CardInfoTypeDef CardInfo;


		BSP_SD_GetCardInfo(&CardInfo);
		  switch (cmd)
		  {
		  /* Make sure that no pending write process */
		  case CTRL_SYNC :
		    res = RES_OK;
		    break;

		  /* Get number of sectors on the disk (DWORD) */
		  case GET_SECTOR_COUNT :

		    *(DWORD*)buff = CardInfo.LogBlockNbr;
		    res = RES_OK;
		    break;

		  /* Get R/W sector size (WORD) */
		  case GET_SECTOR_SIZE :
		    //BSP_SD_GetCardInfo(&CardInfo);
		    *(WORD*)buff = CardInfo.LogBlockSize;
		    res = RES_OK;
		    break;

		  /* Get erase block size in unit of sector (DWORD) */
		  case GET_BLOCK_SIZE :
		    //BSP_SD_GetCardInfo(&CardInfo);
		    *(DWORD*)buff = CardInfo.LogBlockSize / SD_DEFAULT_BLOCK_SIZE;
			res = RES_OK;
		    break;

		  default:
		    res = RES_PARERR;
		  }

		  return res;


	case DEV_USB :

		// Process of the command the USB drive

		return res;
	}

	return RES_PARERR;
}

DWORD get_fattime(void)

{

return 0;

}
/**
  * @brief Tx Transfer completed callbacks
  * @param SdCard: SD_CARD1 or SD_CARD2
  * @retval None
  */
void BSP_SD_WriteCpltCallback()
{
  /*
   * No need to add an "osKernelRunning()" check here, as the SD_initialize()
   * is always called before any SD_Read()/SD_Write() call
   */
   osMessagePut(SDQueueID, WRITE_CPLT_MSG, osWaitForever);
}

/**
  * @brief Rx Transfer completed callbacks
  * @param SdCard: SD_CARD1 or SD_CARD2
  * @retval None
  */
void BSP_SD_ReadCpltCallback()
{
  /*
   * No need to add an "osKernelRunning()" check here, as the SD_initialize()
   * is always called before any SD_Read()/SD_Write() call
   */
   osMessagePut(SDQueueID, READ_CPLT_MSG, osWaitForever);
}
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
