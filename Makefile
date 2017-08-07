TARGET=main.hex
EXECUTABLE=main.elf
BINARY=main.bin

CC=arm-none-eabi-gcc
LD=arm-none-eabi-gcc
AR=arm-none-eabi-ar
AS=arm-none-eabi-as
CP=arm-none-eabi-objcopy
OD=arm-none-eabi-objdump

BIN=$(CP) -O ihex 

# Select the appropriate option for your device, the available options are listed below
# with a description copied from stm32f10x.h
# Make sure to set the startup code file to the right device family, too!
#
# STM32F10X_LD 		STM32F10X_LD: STM32 Low density devices
# STM32F10X_LD_VL	STM32F10X_LD_VL: STM32 Low density Value Line devices
# STM32F10X_MD		STM32F10X_MD: STM32 Medium density devices
# STM32F10X_MD_VL	STM32F10X_MD_VL: STM32 Medium density Value Line devices 
# STM32F10X_HD		STM32F10X_HD: STM32 High density devices
# STM32F10X_HD_VL	STM32F10X_HD_VL: STM32 High density value line devices
# STM32F10X_XL		STM32F10X_XL: STM32 XL-density devices
# STM32F10X_CL		STM32F10X_CL: STM32 Connectivity line devices 
#
# - Low-density devices are STM32F101xx, STM32F102xx and STM32F103xx microcontrollers
#   where the Flash memory density ranges between 16 and 32 Kbytes.
# 
# - Low-density value line devices are STM32F100xx microcontrollers where the Flash
#   memory density ranges between 16 and 32 Kbytes.
# 
# - Medium-density devices are STM32F101xx, STM32F102xx and STM32F103xx microcontrollers
#   where the Flash memory density ranges between 64 and 128 Kbytes.
# 
# - Medium-density value line devices are STM32F100xx microcontrollers where the 
#   Flash memory density ranges between 64 and 128 Kbytes.   
# 
# - High-density devices are STM32F101xx and STM32F103xx microcontrollers where
#   the Flash memory density ranges between 256 and 512 Kbytes.
# 
# - High-density value line devices are STM32F100xx microcontrollers where the 
#   Flash memory density ranges between 256 and 512 Kbytes.   
# 
# - XL-density devices are STM32F101xx and STM32F103xx microcontrollers where
#   the Flash memory density ranges between 512 and 1024 Kbytes.
# 
# - Connectivity line devices are STM32F105xx and STM32F107xx microcontrollers.
#
# HSE_VALUE sets the value of the HSE clock, 8MHz in this case 

DEFS = -DUSE_STDPERIPH_DRIVER -DSTM32F10X_CL -DHSE_VALUE=8000000 -DSTM32F107xC -DMKS_TFT -DILI9325
#STARTUP = 	../Libraries/CMSIS/CM3/DeviceSupport/ST/STM32F10x/startup/gcc_ride7/startup_stm32f10x_cl.s 
STARTUP = Drivers/CMSIS/Device/ST/STM32F1xx/Source/Templates/gcc/startup_stm32f107xc.s

MCU = cortex-m3
MCFLAGS = -mcpu=$(MCU) -mthumb -mlittle-endian -mthumb-interwork

STM32_INCLUDES = -IDrivers/STM32F1xx_HAL_Driver/Inc/ \
		 -IDrivers/CMSIS/Device/ST/STM32F1xx/Include/ \
		 -IDrivers/CMSIS/Include/ \
		 -IIcons \
		 -IInc \
		 -IBootloader \
		 -IMiddlewares/Third_Party/FreeRTOS/Source/include \
		 -IMiddlewares/Third_Party/FatFs/src/ \
		 -IMiddlewares/ST/STM32_USB_Host_Library/Class/MSC/Inc \
		 -IMiddlewares/ST/STM32_USB_Host_Library/Core/Inc \
		 -IMiddlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM3 \
		 -IMiddlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS

OPTIMIZE       = -Os

CFLAGS	= $(MCFLAGS)  $(OPTIMIZE)  $(DEFS) -I. -I./ $(STM32_INCLUDES)  -Wl,-T,STM32F107VC_FLASH.ld
AFLAGS	= $(MCFLAGS) 

SRC =   Src/Buzzer.cpp \
	Src/Display.cpp \
	Src/eeprom.c \
	Src/fatfs.c \
	Src/FileManager.cpp \
	Src/main.c \
	Src/Mem.cpp \
	Src/MessageLog.cpp \
	Src/Misc.cpp \
	Src/PanelDue.cpp \
	Src/Print.cpp \
	Src/RequestTimer.cpp \
#	Src/sd_diskio.c \
	Src/SerialIo.cpp \
	Src/spiflash_w25q16dv.c \
	Src/spisd_diskio.c \
	Src/stm32f1xx_hal_msp.c \
	Src/stm32f1xx_hal_timebase_TIM.c \
	Src/stm32f1xx_it.c \
	Src/system_stm32f1xx_4boot.c \
	Src/usb_host.c \
	Src/usbh_conf.c \
	Src/usbh_diskio.c \
	Src/UserInterface.cpp \
	Src/UTFT.cpp \
	Src/UTouch.cpp \
	Src/bsp_driver_sd.c

SRC += 	Bootloader/flash.c \
Bootloader/main.c \
Bootloader/stm32f1xx_it.c \

SRC += 	Drivers/CMSIS/Device/ST/STM32F1xx/Source/Templates/system_stm32f1xx.c \
Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal.c \
Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_cortex.c \
Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_dma.c \
Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_flash.c \
Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_flash_ex.c \
Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_gpio.c \
Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_gpio_ex.c \
Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_hcd.c \
Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_i2c.c \
Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_pwr.c \
Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_rcc.c \
Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_rcc_ex.c \
Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_sd.c \
Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_spi.c \
Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_spi_ex.c \
Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_sram.c \
Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_tim.c \
Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_tim_ex.c \
Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_uart.c \
Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_ll_fsmc.c \
Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_ll_sdmmc.c \
Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_ll_usb.c \

SRC += 	Fonts/glcd17x22.cpp \
Fonts/glcd28x32.cpp \
Icons/HomeIcons.cpp \
Icons/KeyIcons.cpp \
Icons/MiscIcons.cpp \
Icons/NozzleIcons.cpp \

SRC += 	Middlewares/ST/STM32_USB_Host_Library/Class/MSC/Src/usbh_msc.c \
Middlewares/ST/STM32_USB_Host_Library/Class/MSC/Src/usbh_msc_bot.c \
Middlewares/ST/STM32_USB_Host_Library/Class/MSC/Src/usbh_msc_scsi.c \
Middlewares/ST/STM32_USB_Host_Library/Core/Src/usbh_core.c \
Middlewares/ST/STM32_USB_Host_Library/Core/Src/usbh_ctlreq.c \
Middlewares/ST/STM32_USB_Host_Library/Core/Src/usbh_ioreq.c \
Middlewares/ST/STM32_USB_Host_Library/Core/Src/usbh_pipes.c \
Middlewares/Third_Party/FatFs/src/diskio.c \
Middlewares/Third_Party/FatFs/src/ff.c \
Middlewares/Third_Party/FatFs/src/ff_gen_drv.c \
Middlewares/Third_Party/FatFs/src/option/ccsbcs.c \
Middlewares/Third_Party/FatFs/src/option/syscall.c \
Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS/cmsis_os.c \
Middlewares/Third_Party/FreeRTOS/Source/croutine.c \
Middlewares/Third_Party/FreeRTOS/Source/event_groups.c \
Middlewares/Third_Party/FreeRTOS/Source/list.c \
Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM3/port.c \
Middlewares/Third_Party/FreeRTOS/Source/portable/MemMang/heap_4.c \
Middlewares/Third_Party/FreeRTOS/Source/queue.c \
Middlewares/Third_Party/FreeRTOS/Source/tasks.c \
Middlewares/Third_Party/FreeRTOS/Source/timers.c \

OBJDIR = build
OBJ = $(SRC:%.c=$(OBJDIR)/%.o) 
OBJ += Startup.o

all: $(BINARY)

$(BINARY): $(EXECUTABLE)
	$(CP) -O binary $(EXECUTABLE) $(BINARY)

flash: $(BINARY)
#	st-flash write $(BINARY) 0x08000000
	stm32flash -w $(BINARY) -v -g 0x0 /dev/ttyUSB0
	
$(EXECUTABLE): $(SRC) $(STARTUP)
	$(CC) $(CFLAGS) $^ -lm -lc -lnosys -o $@

clean:
	rm -f Startup.lst  $(TARGET)  $(TARGET).lst $(OBJ) $(AUTOGEN)  $(TARGET).out  $(TARGET).hex  $(TARGET).map \
	$(TARGET).dmp  $(EXECUTABLE)

# Последнюю строчку лучше добавить в make, в секцию executable:
# $(EXECUTABLE): $(SRC) $(STARTUP)
#        $(CC) $(CFLAGS) $^ -lm -lc -lnosys -o $@
#        $(CP) -O binary $(EXECUTABLE).elf $(EXECUTABLE).bin
