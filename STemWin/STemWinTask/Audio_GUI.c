/*********************************************************************
*                                                                    *
*                SEGGER Microcontroller GmbH & Co. KG                *
*        Solutions for real time microcontroller applications        *
*                                                                    *
**********************************************************************
*                                                                    *
* C-file generated by:                                               *
*                                                                    *
*        GUI_Builder for emWin version 5.44                          *
*        Compiled Nov 10 2017, 08:53:57                              *
*        (c) 2017 Segger Microcontroller GmbH & Co. KG               *
*                                                                    *
**********************************************************************
*                                                                    *
*        Internet: www.segger.com  Support: support@segger.com       *
*                                                                    *
**********************************************************************
*/

// USER START (Optionally insert additional includes)
#include "stm32f769i_discovery.h"
#include "stdio.h"
#include "FreeRTOS.h"
#include "task.h"
#include "string.h"
#include "ff.h"
// USER END

#include "DIALOG.h"
#include <stm32f769i_discovery_audio.h>
#include "MainTask.h"
GUI_XBF_DATA XBF_Data;
GUI_FONT     Font;
/*********************************************************************
*
*       Defines
*
**********************************************************************
*/
#define ID_WINDOW_0 (GUI_ID_USER + 0x00)
#define ID_BUTTON_0 (GUI_ID_USER + 0x01)
#define ID_SLIDER_0 (GUI_ID_USER + 0x02)
#define ID_TEXT_0 (GUI_ID_USER + 0x03)
#define ID_TEXT_1 (GUI_ID_USER + 0x04)
#define ID_TEXT_2 (GUI_ID_USER + 0x08)
#define ID_TEXT_3 (GUI_ID_USER + 0x09)
#define ID_SLIDER_1 (GUI_ID_USER + 0x0a)

// USER START (Optionally insert additional defines)
#define Initial_volume (20)
// USER END

/*********************************************************************
*
*       Static data
*
**********************************************************************
*/

// USER START (Optionally insert additional static data)
uint8_t volume = Initial_volume;
static char volume_number[12];
char time_char[15];
extern TaskHandle_t xTaskVolume;
extern TaskHandle_t xTaskMusic;
extern AudioTime Total_AudioTime;
uint8_t Play_Status=0;
static FIL fi_xbf;
// USER END

/*********************************************************************
*
*       _aDialogCreate
*/
static const GUI_WIDGET_CREATE_INFO _aDialogCreate[] = {
  { WINDOW_CreateIndirect, "Window", ID_WINDOW_0, 0, 0, 800, 480, 0, 0x0, 0 },
  { BUTTON_CreateIndirect, "PAUSE", ID_BUTTON_0, 365, 340, 80, 70, 0, 0x0, 0 },
  { SLIDER_CreateIndirect, "Slider", ID_SLIDER_0, 251, 129, 420, 40, 0, 0x0, 0 },
  { TEXT_CreateIndirect, "Audio Player", ID_TEXT_0, 312, 20, 156, 39, 0, 0x0, 0 },
  { TEXT_CreateIndirect, "Volume", ID_TEXT_1, 105, 133, 115, 20, 0, 0x0, 0 },
  { TEXT_CreateIndirect, "Filename", ID_TEXT_2, 0, 70, 799, 32, 0, 0x64, 0 },
  { TEXT_CreateIndirect, "Text", ID_TEXT_3, 91, 208, 127, 20, 0, 0x64, 0 },
  { SLIDER_CreateIndirect, "Slider", ID_SLIDER_1, 248, 190, 425, 47, 0, 0x0, 0 },
  // USER START (Optionally insert additional widgets)
  // USER END
};



static int _cbGetData(U32 Off, U16 NumBytes, void * pVoid, void * pBuffer) {
  //FIL hFile;
  DWORD  NumBytesRead;

  //hFile = *(FIL *)pVoid;
  //
  // Set file pointer to the requested position
  //
  if (f_lseek(&fi_xbf, Off) != FR_OK) {
    return 1; // Error
  }
  //
  // Read font data
  //
  if (f_read(&fi_xbf, pBuffer, NumBytes, &NumBytesRead) != FR_OK) {
    return 1; // Error
  }
  if (NumBytesRead != NumBytes) {
    return 1; // Error
  }
  return 0;   // Ok
}
/*********************************************************************
*
*       Static code
*
**********************************************************************
*/

// USER START (Optionally insert additional static code)
// USER END

/*********************************************************************
*
*       _cbDialog
*/
static void _cbDialog(WM_MESSAGE * pMsg) {
  WM_HWIN hItem;
  int     NCode;
  int     Id;
  // USER START (Optionally insert additional variables)
  static  WM_HTIMER hTimerProcess;
  // USER END

  switch (pMsg->MsgId) {
  case WM_INIT_DIALOG:
    //
    // Initialization of 'Framewin'
    //


	sprintf(volume_number, "Volume:%3d", volume);
	strcpy(time_char, "00:00 / 03:00");

    hItem = pMsg->hWin;
    WINDOW_SetBkColor(hItem, GUI_MAKE_COLOR(GUI_WHITE));
    //
    // Initialization of 'PLAY'
    //
    hItem = WM_GetDialogItem(pMsg->hWin, ID_BUTTON_0);
    BUTTON_SetFont(hItem, GUI_FONT_24_ASCII);
    BUTTON_SetText(hItem, "PAUSE");
    //
    // Initialization of 'Audio Player'
    //
    hItem = WM_GetDialogItem(pMsg->hWin, ID_TEXT_0);
    TEXT_SetFont(hItem, GUI_FONT_32_ASCII);
    TEXT_SetTextAlign(hItem, GUI_TA_HCENTER | GUI_TA_VCENTER);
    //
    // Initialization of 'Volume'
    //
    hItem = WM_GetDialogItem(pMsg->hWin, ID_TEXT_1);
    TEXT_SetTextAlign(hItem, GUI_TA_HCENTER | GUI_TA_VCENTER);
    TEXT_SetFont(hItem, GUI_FONT_24_ASCII);
    //
    // Initialization of 'Text'
    //


    hItem = WM_GetDialogItem(pMsg->hWin, ID_TEXT_2);
    TEXT_SetFont(hItem, &Font);
    TEXT_SetTextAlign(hItem, GUI_TA_HCENTER | GUI_TA_VCENTER);

    // USER START (Optionally insert additional code for further widget initialization)

    hItem = WM_GetDialogItem(pMsg->hWin, ID_SLIDER_0);
    SLIDER_SetRange(hItem, 0, 100);
    SLIDER_SetValue(hItem, volume);
    //
    // Initialization of 'Text'
    //
    hItem = WM_GetDialogItem(pMsg->hWin, ID_TEXT_3);
    TEXT_SetFont(hItem, GUI_FONT_24_ASCII);
    TEXT_SetTextAlign(hItem, GUI_TA_HCENTER | GUI_TA_VCENTER);
    TEXT_SetText(hItem, time_char);

    hItem = WM_GetDialogItem(pMsg->hWin, ID_SLIDER_1);
	SLIDER_SetRange(hItem, 0, 100);
	SLIDER_SetValue(hItem, 0);

	hTimerProcess = WM_CreateTimer(pMsg->hWin, 0, 100, 0);
    // USER END
    break;

  case WM_NOTIFY_PARENT:
    Id    = WM_GetId(pMsg->hWinSrc);
    NCode = pMsg->Data.v;
    switch(Id) {
    case ID_BUTTON_0: // Notifications sent by 'PLAY'
      switch(NCode) {
      case WM_NOTIFICATION_CLICKED:
        // USER START (Optionally insert code for reacting on notification message)
    	  hItem = WM_GetDialogItem(pMsg->hWin, ID_BUTTON_0);
    	  //BUTTON_SetPressed(hItem, 0);
		 if (BUTTON_IsPressed(hItem) == 1)
		 {
			 if (Play_Status == 0)
			 {
				 Play_Status = 1;
				 BUTTON_SetText(hItem, "PAUSE");
			 }
			 else
			 {
				 Play_Status = 0;
				 BUTTON_SetText(hItem, "PLAY");
			 }
			 xTaskNotify(xTaskMusic, 0x08, eSetBits);
		 }

        // USER END
        break;
      case WM_NOTIFICATION_RELEASED:
        // USER START (Optionally insert code for reacting on notification message)



        // USER END
        break;

      case WM_NOTIFICATION_MOVED_OUT:

    	  break;
      // USER START (Optionally insert additional code for further notification handling)
      // USER END
      }
      break;
    case ID_SLIDER_0: // Notifications sent by 'Slider'
      switch(NCode) {
      case WM_NOTIFICATION_CLICKED:
        // USER START (Optionally insert code for reacting on notification message)
        // USER END
        break;
      case WM_NOTIFICATION_RELEASED:
        // USER START (Optionally insert code for reacting on notification message)

        // USER END
        break;
      case WM_NOTIFICATION_VALUE_CHANGED:
        // USER START (Optionally insert code for reacting on notification message)
    	  hItem = WM_GetDialogItem(pMsg->hWin, ID_SLIDER_0);
    	  volume = SLIDER_GetValue(hItem);
    	  hItem = WM_GetDialogItem(pMsg->hWin, ID_TEXT_1);
    	  sprintf(&volume_number[7], "%3d", volume);
    	  TEXT_SetText(hItem, volume_number);
    	  //wm8994_SetVolume(AUDIO_I2C_ADDRESS, volume);
    	  xTaskNotify( xTaskVolume, 0x01, eSetBits );
        // USER END
        break;
      // USER START (Optionally insert additional code for further notification handling)
      // USER END
      }
      break;
    // USER START (Optionally insert additional code for further Ids)
      case ID_SLIDER_1: // Notifications sent by 'Slider'
        switch(NCode) {
        case WM_NOTIFICATION_CLICKED:
          // USER START (Optionally insert code for reacting on notification message)
        	WM_DeleteTimer(hTimerProcess);
          // USER END
          break;
        case WM_NOTIFICATION_RELEASED:
        	hItem = WM_GetDialogItem(pMsg->hWin, ID_SLIDER_1);
		  Total_AudioTime.current_progress_insecond = SLIDER_GetValue(hItem)*Total_AudioTime.total_second/100;
		  hTimerProcess = WM_CreateTimer(pMsg->hWin, 0, 100, 0);
		  xTaskNotify(xTaskMusic, 0x04, eSetBits);

          break;
        case WM_NOTIFICATION_VALUE_CHANGED:
          // USER START (Optionally insert code for reacting on notification message)

          // USER END
          break;
        // USER START (Optionally insert additional code for further notification handling)
        // USER END
        }
    // USER END
    }
    break;
  // USER START (Optionally insert additional message handling)
	case WM_TIMER:
		Id = WM_GetTimerId(pMsg->Data.v);
		hItem = WM_GetDialogItem(pMsg->hWin, ID_TEXT_3);
		TEXT_SetText(hItem, time_char);
		hItem = WM_GetDialogItem(pMsg->hWin, ID_SLIDER_1);
		SLIDER_SetValue(hItem, 100 * Total_AudioTime.current_progress_insecond/ Total_AudioTime.total_second);
		WM_RestartTimer(pMsg->Data.v, 100);
		break;
	case WM_USER_UPDATEFILENAME:
		hItem = WM_GetDialogItem(pMsg->hWin, ID_TEXT_2);
		TEXT_SetText(hItem, pMsg->Data.p);
		break;
  // USER END
  default:
    WM_DefaultProc(pMsg);
    break;
  }
}

/*********************************************************************
*
*       Public code
*
**********************************************************************
*/
/*********************************************************************
*
*       CreateFramewin
*/
WM_HWIN CreateFramewin(void);
WM_HWIN CreateFramewin(void) {
  WM_HWIN hWin;

  hWin = GUI_CreateDialogBox(_aDialogCreate, GUI_COUNTOF(_aDialogCreate), _cbDialog, WM_HBKWIN, 0, 0);
  return hWin;
}
WM_HWIN xhWin;
// USER START (Optionally insert additional public code)

void YaHeiFont_Init(void)
{
	f_open(&fi_xbf, "YaHei.xbf", FA_READ);
	GUI_XBF_CreateFont(&Font,              // Pointer to GUI_FONT structure in RAM
						 &XBF_Data,          // Pointer to GUI_XBF_DATA structure in RAM
						 GUI_XBF_TYPE_PROP,  // Font type to be created
						 _cbGetData,         // Pointer to callback function
						 (void *)&fi_xbf);    // Pointer to be passed to GetData function
}
__weak void MainTask(void)
{

	uint16_t fr, re_font;


	GUI_Clear();

	YaHeiFont_Init();

	xhWin = CreateFramewin();

	GUI_UC_SetEncodeUTF8();
	xTaskNotify(xTaskMusic, 0x08, eSetBits);
	while(1)
	{
		GUI_Delay(50);

	}
}
// USER END

/*************************** End of file ****************************/
