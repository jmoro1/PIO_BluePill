# PIO_BluePill
Basic PlatformIO project imported from STM32CubeMX/IDE for programming a [CLONED] BluePill (stm32f103c8)

This repo is only for deploying base funcionality STM32CubeIDE-like code using PlatformIO in VSCode, probably it'll not be updated much but I'll get it public because maybe it helps someone.

To import a STM32CubeIDE project to PIO:
1-Generate the code for the board/MCU of yours (remember to select Serial Wire as debug method) and generate code.
2-Generate a PIO project for your board and select stm32cube framework.
3-Once created go to the STM32CubeIDE project folder and copy all the files from the /Src folder into /src (PIO) project folder and /Inc folder to /include (PIO) folder.
4-Restart VSCode and build up your firmware, its ready to be uploaded.
5-My platformio.ini file its modified so that the debugger doesnt give an error because my bluepill is a clone even though I use a genuine STLink V2 to interface with it. If you have a genuine one, maybe using the default platformio.ini file works better.
