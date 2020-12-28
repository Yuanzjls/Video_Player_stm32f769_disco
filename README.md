# STM32F769-Disco MJPEG player with wav audio

This is a STM32 MJPEG player, with a wav sound decoder. The project uses FreeRTOS, hardware jpeg decode, SD card reader. In order to make the movie files playable in this platform, you can use any video format transform software to make MJPEG file with wav sound, and using avi to package them together. Because the STM32F7 is not powerful enough to decode high resolution MJPEG files, you will see player loss some frames during playing. In order to sync screen and sound, the project guarantee that it play sounds without losing frames.

![](video_player_flow.jpg)

## SD card reading and JPEG decode

Both SD card reading and JPEG decode have a little amount of delay, so you don't want to those tasks block each other, otherwise, the time of delay will be sum of individual delays. In this program, I have make sure that JPEG decode task is not waiting for data to come, especially wait for reading data from SD card. I make two buffer to store original JPEG data, So SD card and JPEG and work at the same time without interfering each other.

## Demo

https://www.youtube.com/watch?v=mcuelFrxrag

## Todo

* Add Control Panel to the screen
* Add file explore
* Add shuttle, loop play function
