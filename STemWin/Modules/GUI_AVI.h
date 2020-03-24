/**
  ******************************************************************************
  * @file    GUI_AVI.h
  * @author  MCD Application Team
  * @brief   Header for GUI_AVI.c file
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

/* Define to prevent recursive inclusion -------------------------------------*/

#ifndef  __GUI_AVI_H
#define  __GUI_AVI_H

#include "GUI.h"

#if defined(__cplusplus)
extern "C" {     /* Make sure we have C-declarations in C++ programs */
#endif

/* Exported types ------------------------------------------------------------*/  
typedef enum {
	AVI_OK   =0,			
	AVI_RIFF_ERR,			
	AVI_AVI_ERR,			
	AVI_LIST_ERR,			
	AVI_HDRL_ERR,			
	AVI_AVIH_ERR,			
	AVI_STRL_ERR,			
	AVI_STRH_ERR,			
	AVI_STRF_ERR,			
	AVI_MOVI_ERR,			
	AVI_FORMAT_ERR,			
	AVI_STREAM_ERR,			
}AVISTATUS;



#define AVI_RIFF_ID			0x46464952  
#define AVI_AVI_ID			0x20495641
#define AVI_LIST_ID			0x5453494C  
#define AVI_HDRL_ID			0x6C726468	
#define AVI_MOVI_ID			0x69766F6D 	
#define AVI_STRL_ID			0x6C727473	

#define AVI_AVIH_ID			0x68697661 	
#define AVI_STRH_ID			0x68727473	
#define AVI_STRF_ID			0x66727473 	
#define AVI_STRD_ID			0x64727473 	

#define AVI_VIDS_STREAM		        0x73646976		
#define AVI_AUDS_STREAM		        0x73647561 		
                                        
                                        
#define AVI_VIDS_FLAG		        0x6463			
#define AVI_AUDS_FLAG		        0x7762 


#define AVI_FORMAT_MJPG		        0x47504A4D 





#if defined(__GNUC__)
#define PACKED_STRUCT struct __packed
#else
#define PACKED_STRUCT __packed struct
#endif

typedef PACKED_STRUCT
{	  
	U32 SecPerFrame;		
	U32 TotalFrame;			
	U32 Width;			
	U32 Height;			
	U32 SampleRate; 		
	U16 Channels;	  		
	U16 AudioBufSize;		
	U16 AudioType;	  		
	U16 StreamID;			
	U32 StreamSize;			
	U8* VideoFLAG;			
	U8* AudioFLAG;			
}AVI_INFO;

typedef struct  
{	
	U32 RiffID;			
	U32 FileSize;			
	U32 AviID;			
}AVI_HEADER;

typedef struct
{	
	U32 FrameID;			
	U32 FrameSize;			
}FRAME_HEADER;

typedef struct
{	
	U32 ListID;			
	U32 BlockSize;			
	U32 ListType;			
}LIST_HEADER;

typedef struct
{	
	U32 BlockID;		
	U32 BlockSize;		
	U32 SecPerFrame;	
	U32 MaxByteSec; 	
	U32 PaddingGranularity; 
	U32 Flags;		
	U32 TotalFrame;		
	U32 InitFrames;  	
	U32 Streams;		
	U32 RefBufSize;		
	U32 Width;		
	U32 Height;		
	U32 Reserved[4];	
}AVIH_HEADER;


typedef struct
{	
	U32 BlockID;			
	U32 BlockSize;			
	U32 StreamType;			
	U32 Handler;			
	U32 Flags;  			
	U16 Priority;			
	U16 Language;			
	U32 InitFrames;  		
	U32 Scale;			
	U32 Rate; 			
	U32 Start;			
	U32 Length;			
 	U32 RefBufSize;  		
        U32 Quality;			
	U32 SampleSize;			
	struct				
	{				
	   	U16 Left;
		U16 Top;
		U16 Right;
		U16 Bottom;
	}Frame;				
}STRH_HEADER;


typedef struct
{
	U32	     BmpSize;
 	U32      Width;			
	U32      Height;			
	U16      Planes;			
	U16      BitCount;			
	U32      Compression;		
	U32      SizeImage;			
	U32      XpixPerMeter;		
	U32      YpixPerMeter;		
	U32      ClrUsed;			
	U32      ClrImportant;		
}BMP_HEADER;


typedef struct 
{
	U8  rgbBlue;			
	U8  rgbGreen; 			
	U8  rgbRed; 			
	U8  rgbReserved;		
}AVIRGBQUAD;

typedef struct 
{
	U32 BlockID;		
	U32 BlockSize;		
	BMP_HEADER bmiHeader;  	
	AVIRGBQUAD bmColors[1];	
}STRF_BMPHEADER;  


typedef struct 
{
	U32 BlockID;			
	U32 BlockSize;			
   	U16 FormatTag;			
	U16 Channels;	  		
	U32 SampleRate; 		
	U32 BaudRate;   		
	U16 BlockAlign; 		
	U16 Size;			
}STRF_WAVHEADER;




#define	 MAKEWORD(ptr)	(U16)(((U16)*((U8*)(ptr))<<8)|(U16)*(U8*)((ptr)+1))
#define  MAKEDWORD(ptr)	(U32)(((U16)*(U8*)(ptr)|(((U16)*(U8*)(ptr+1))<<8)|\
						(((U16)*(U8*)(ptr+2))<<16)|(((U16)*(U8*)(ptr+3))<<24))) 

extern AVI_INFO Avix;

AVISTATUS _AVI_Init(AVI_INFO*, U8 *buf, U16 size);
U32 _AVI_SearchID(U8* buf,U32 size,U8 *id);
AVISTATUS _Avi_Get_Streaminfo(AVI_INFO*, U8* buf);





#if defined(__cplusplus)
}
#endif

#endif   /* ifdef __GUI_AVI_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
