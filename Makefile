# This file was automagically generated by mbed.org. For more information, 
# see http://mbed.org/handbook/Exporting-to-GCC-ARM-Embedded

###############################################################################
# Boiler-plate

# cross-platform directory manipulation
ifeq ($(shell echo $$OS),$$OS)
    MAKEDIR = if not exist "$(1)" mkdir "$(1)"
    RM = rmdir /S /Q "$(1)"
else
    MAKEDIR = '$(SHELL)' -c "mkdir -p \"$(1)\""
    RM = '$(SHELL)' -c "rm -rf \"$(1)\""
endif

OBJDIR := BUILD
# Move to the build directory
ifeq (,$(filter $(OBJDIR),$(notdir $(CURDIR))))
.SUFFIXES:
mkfile_path := $(abspath $(lastword $(MAKEFILE_LIST)))
MAKETARGET = '$(MAKE)' --no-print-directory -C $(OBJDIR) -f '$(mkfile_path)' \
		'SRCDIR=$(CURDIR)' $(MAKECMDGOALS)
.PHONY: $(OBJDIR) clean
all:
	+@$(call MAKEDIR,$(OBJDIR))
	+@$(MAKETARGET)
$(OBJDIR): all
Makefile : ;
% :: $(OBJDIR) ; :
clean :
	$(call RM,$(OBJDIR))

else

# trick rules into thinking we are in the root, when we are in the bulid dir
VPATH = ..

# Boiler-plate
###############################################################################
# Project settings

PROJECT := MAX32630FTHR_Neurosky


# Project settings
###############################################################################
# Objects and Paths

OBJECTS += Brain.o
OBJECTS += USBDevice/USBAudio/USBAudio.o
OBJECTS += USBDevice/USBDevice/USBDevice.o
OBJECTS += USBDevice/USBDevice/USBHAL_EFM32.o
OBJECTS += USBDevice/USBDevice/USBHAL_KL25Z.o
OBJECTS += USBDevice/USBDevice/USBHAL_LPC11U.o
OBJECTS += USBDevice/USBDevice/USBHAL_LPC17.o
OBJECTS += USBDevice/USBDevice/USBHAL_LPC40.o
OBJECTS += USBDevice/USBDevice/USBHAL_Maxim.o
OBJECTS += USBDevice/USBDevice/USBHAL_RZ_A1H.o
OBJECTS += USBDevice/USBDevice/USBHAL_STM32F4.o
OBJECTS += USBDevice/USBHID/USBHID.o
OBJECTS += USBDevice/USBHID/USBKeyboard.o
OBJECTS += USBDevice/USBHID/USBMouse.o
OBJECTS += USBDevice/USBHID/USBMouseKeyboard.o
OBJECTS += USBDevice/USBMIDI/USBMIDI.o
OBJECTS += USBDevice/USBMSD/USBMSD.o
OBJECTS += USBDevice/USBSerial/USBCDC.o
OBJECTS += USBDevice/USBSerial/USBSerial.o
OBJECTS += main.o
OBJECTS += max32630fthr/MAX14690/MAX14690.o
OBJECTS += max32630fthr/max32630fthr.o

 SYS_OBJECTS += mbed/TARGET_MAX32630FTHR/TOOLCHAIN_GCC_ARM/PeripheralPins.o
 SYS_OBJECTS += mbed/TARGET_MAX32630FTHR/TOOLCHAIN_GCC_ARM/adc.o
 SYS_OBJECTS += mbed/TARGET_MAX32630FTHR/TOOLCHAIN_GCC_ARM/analogin_api.o
 SYS_OBJECTS += mbed/TARGET_MAX32630FTHR/TOOLCHAIN_GCC_ARM/clkman.o
 SYS_OBJECTS += mbed/TARGET_MAX32630FTHR/TOOLCHAIN_GCC_ARM/crc.o
 SYS_OBJECTS += mbed/TARGET_MAX32630FTHR/TOOLCHAIN_GCC_ARM/device_nvic.o
 SYS_OBJECTS += mbed/TARGET_MAX32630FTHR/TOOLCHAIN_GCC_ARM/except.o
 SYS_OBJECTS += mbed/TARGET_MAX32630FTHR/TOOLCHAIN_GCC_ARM/flc.o
 SYS_OBJECTS += mbed/TARGET_MAX32630FTHR/TOOLCHAIN_GCC_ARM/gpio.o
 SYS_OBJECTS += mbed/TARGET_MAX32630FTHR/TOOLCHAIN_GCC_ARM/gpio_api.o
 SYS_OBJECTS += mbed/TARGET_MAX32630FTHR/TOOLCHAIN_GCC_ARM/gpio_irq_api.o
 SYS_OBJECTS += mbed/TARGET_MAX32630FTHR/TOOLCHAIN_GCC_ARM/i2c_api.o
 SYS_OBJECTS += mbed/TARGET_MAX32630FTHR/TOOLCHAIN_GCC_ARM/i2cm.o
 SYS_OBJECTS += mbed/TARGET_MAX32630FTHR/TOOLCHAIN_GCC_ARM/i2cs.o
 SYS_OBJECTS += mbed/TARGET_MAX32630FTHR/TOOLCHAIN_GCC_ARM/icc.o
 SYS_OBJECTS += mbed/TARGET_MAX32630FTHR/TOOLCHAIN_GCC_ARM/ioman.o
 SYS_OBJECTS += mbed/TARGET_MAX32630FTHR/TOOLCHAIN_GCC_ARM/low_level_init.o
 SYS_OBJECTS += mbed/TARGET_MAX32630FTHR/TOOLCHAIN_GCC_ARM/lp.o
 SYS_OBJECTS += mbed/TARGET_MAX32630FTHR/TOOLCHAIN_GCC_ARM/maa.o
 SYS_OBJECTS += mbed/TARGET_MAX32630FTHR/TOOLCHAIN_GCC_ARM/mbed_board.o
 SYS_OBJECTS += mbed/TARGET_MAX32630FTHR/TOOLCHAIN_GCC_ARM/mbed_fault_handler.o
 SYS_OBJECTS += mbed/TARGET_MAX32630FTHR/TOOLCHAIN_GCC_ARM/mbed_retarget.o
 SYS_OBJECTS += mbed/TARGET_MAX32630FTHR/TOOLCHAIN_GCC_ARM/mbed_sdk_boot.o
 SYS_OBJECTS += mbed/TARGET_MAX32630FTHR/TOOLCHAIN_GCC_ARM/mbed_tz_context.o
 SYS_OBJECTS += mbed/TARGET_MAX32630FTHR/TOOLCHAIN_GCC_ARM/mxc_aes.o
 SYS_OBJECTS += mbed/TARGET_MAX32630FTHR/TOOLCHAIN_GCC_ARM/mxc_assert.o
 SYS_OBJECTS += mbed/TARGET_MAX32630FTHR/TOOLCHAIN_GCC_ARM/mxc_sys.o
 SYS_OBJECTS += mbed/TARGET_MAX32630FTHR/TOOLCHAIN_GCC_ARM/owm.o
 SYS_OBJECTS += mbed/TARGET_MAX32630FTHR/TOOLCHAIN_GCC_ARM/pinmap.o
 SYS_OBJECTS += mbed/TARGET_MAX32630FTHR/TOOLCHAIN_GCC_ARM/pmu.o
 SYS_OBJECTS += mbed/TARGET_MAX32630FTHR/TOOLCHAIN_GCC_ARM/port_api.o
 SYS_OBJECTS += mbed/TARGET_MAX32630FTHR/TOOLCHAIN_GCC_ARM/prng.o
 SYS_OBJECTS += mbed/TARGET_MAX32630FTHR/TOOLCHAIN_GCC_ARM/pt.o
 SYS_OBJECTS += mbed/TARGET_MAX32630FTHR/TOOLCHAIN_GCC_ARM/pwmout_api.o
 SYS_OBJECTS += mbed/TARGET_MAX32630FTHR/TOOLCHAIN_GCC_ARM/rtc.o
 SYS_OBJECTS += mbed/TARGET_MAX32630FTHR/TOOLCHAIN_GCC_ARM/rtc_api.o
 SYS_OBJECTS += mbed/TARGET_MAX32630FTHR/TOOLCHAIN_GCC_ARM/serial_api.o
 SYS_OBJECTS += mbed/TARGET_MAX32630FTHR/TOOLCHAIN_GCC_ARM/sleep.o
 SYS_OBJECTS += mbed/TARGET_MAX32630FTHR/TOOLCHAIN_GCC_ARM/spi_api.o
 SYS_OBJECTS += mbed/TARGET_MAX32630FTHR/TOOLCHAIN_GCC_ARM/spim.o
 SYS_OBJECTS += mbed/TARGET_MAX32630FTHR/TOOLCHAIN_GCC_ARM/spix.o
 SYS_OBJECTS += mbed/TARGET_MAX32630FTHR/TOOLCHAIN_GCC_ARM/startup_max3263x.o
 SYS_OBJECTS += mbed/TARGET_MAX32630FTHR/TOOLCHAIN_GCC_ARM/system_max3263x.o
 SYS_OBJECTS += mbed/TARGET_MAX32630FTHR/TOOLCHAIN_GCC_ARM/tmr.o
 SYS_OBJECTS += mbed/TARGET_MAX32630FTHR/TOOLCHAIN_GCC_ARM/tmr_utils.o
 SYS_OBJECTS += mbed/TARGET_MAX32630FTHR/TOOLCHAIN_GCC_ARM/uart.o
 SYS_OBJECTS += mbed/TARGET_MAX32630FTHR/TOOLCHAIN_GCC_ARM/us_ticker.o
 SYS_OBJECTS += mbed/TARGET_MAX32630FTHR/TOOLCHAIN_GCC_ARM/wdt.o
 SYS_OBJECTS += mbed/TARGET_MAX32630FTHR/TOOLCHAIN_GCC_ARM/wdt2.o

INCLUDE_PATHS += -I../
INCLUDE_PATHS += -I../.
INCLUDE_PATHS += -I..//usr/src/mbed-sdk
INCLUDE_PATHS += -I../USBDevice
INCLUDE_PATHS += -I../USBDevice/USBAudio
INCLUDE_PATHS += -I../USBDevice/USBDevice
INCLUDE_PATHS += -I../USBDevice/USBHID
INCLUDE_PATHS += -I../USBDevice/USBMIDI
INCLUDE_PATHS += -I../USBDevice/USBMSD
INCLUDE_PATHS += -I../USBDevice/USBSerial
INCLUDE_PATHS += -I../max32630fthr
INCLUDE_PATHS += -I../max32630fthr/MAX14690
INCLUDE_PATHS += -I../mbed
INCLUDE_PATHS += -I../mbed/TARGET_MAX32630FTHR/TOOLCHAIN_GCC_ARM
INCLUDE_PATHS += -I../mbed/drivers
INCLUDE_PATHS += -I../mbed/hal
INCLUDE_PATHS += -I../mbed/platform

LIBRARY_PATHS := -L../mbed/TARGET_MAX32630FTHR/TOOLCHAIN_GCC_ARM 
LIBRARIES := -lexactLE  -lmbed 
LINKER_SCRIPT ?= ../mbed/TARGET_MAX32630FTHR/TOOLCHAIN_GCC_ARM/max3263x.ld

# Objects and Paths
###############################################################################
# Tools and Flags

AS      = arm-none-eabi-gcc
CC      = arm-none-eabi-gcc
CPP     = arm-none-eabi-g++
LD      = arm-none-eabi-gcc
ELF2BIN = arm-none-eabi-objcopy
PREPROC = arm-none-eabi-cpp -E -P -Wl,--gc-sections -Wl,--wrap,main -Wl,--wrap,_malloc_r -Wl,--wrap,_free_r -Wl,--wrap,_realloc_r -Wl,--wrap,_memalign_r -Wl,--wrap,_calloc_r -Wl,--wrap,exit -Wl,--wrap,atexit -Wl,-n -Wl,--wrap,printf -Wl,--wrap,sprintf -Wl,--wrap,snprintf -Wl,--wrap,vprintf -Wl,--wrap,vsprintf -Wl,--wrap,vsnprintf -Wl,--wrap,fprintf -Wl,--wrap,vfprintf -mcpu=cortex-m4 -mthumb -mfpu=fpv4-sp-d16 -mfloat-abi=softfp -Wall -Wextra -Wno-unused-parameter -Wno-missing-field-initializers -fmessage-length=0 -fno-exceptions -ffunction-sections -fdata-sections -funsigned-char -MMD -fomit-frame-pointer -Os -g -DMBED_TRAP_ERRORS_ENABLED=1 -DMBED_MINIMAL_PRINTF -mcpu=cortex-m4 -mthumb -mfpu=fpv4-sp-d16 -mfloat-abi=softfp -DMBED_BOOT_STACK_SIZE=4096 -DXIP_ENABLE=0


C_FLAGS += -c
C_FLAGS += -std=gnu11
C_FLAGS += -include
C_FLAGS += mbed_config.h
C_FLAGS += -DTARGET_LIKE_CORTEX_M4
C_FLAGS += -DTOOLCHAIN_GCC
C_FLAGS += -DDEVICE_STDIO_MESSAGES=1
C_FLAGS += -DOPEN_DRAIN_LEDS
C_FLAGS += -DDEVICE_PORTINOUT=1
C_FLAGS += -D__FPU_PRESENT=1
C_FLAGS += -DMBED_TICKLESS
C_FLAGS += -DDEVICE_LPTICKER=1
C_FLAGS += -DDEVICE_MPU=1
C_FLAGS += -DDEVICE_I2C=1
C_FLAGS += -DDEVICE_INTERRUPTIN=1
C_FLAGS += -DTARGET=MAX32630
C_FLAGS += -DDEVICE_SPI=1
C_FLAGS += -DTARGET_LIKE_MBED
C_FLAGS += -DDEVICE_PWMOUT=1
C_FLAGS += -DTARGET_MAX32630FTHR
C_FLAGS += -D__SYSTEM_HFX=96000000
C_FLAGS += -DBLE_HCI_UART
C_FLAGS += -DDEVICE_PORTIN=1
C_FLAGS += -D__CMSIS_RTOS
C_FLAGS += -DDEVICE_USTICKER=1
C_FLAGS += -DTARGET_CORTEX
C_FLAGS += -DTARGET_RTOS_M4_M7
C_FLAGS += -DTARGET_CORTEX_M
C_FLAGS += -D__MBED_CMSIS_RTOS_CM
C_FLAGS += -DDEVICE_PORTOUT=1
C_FLAGS += -D__CORTEX_M4
C_FLAGS += -DTARGET_REV=0x4132
C_FLAGS += -DTARGET_RELEASE
C_FLAGS += -DDEVICE_SERIAL_FC=1
C_FLAGS += -DDEVICE_ANALOGIN=1
C_FLAGS += -DTARGET_M4
C_FLAGS += -DTARGET_Maxim
C_FLAGS += -DTOOLCHAIN_GCC_ARM
C_FLAGS += -DTARGET_NAME=MAX32630FTHR
C_FLAGS += -D__MBED__=1
C_FLAGS += -DMBED_BUILD_TIMESTAMP=1621198569.986343
C_FLAGS += -DTARGET_MAX32630
C_FLAGS += -DARM_MATH_CM4
C_FLAGS += -DDEVICE_SERIAL=1
C_FLAGS += -include
C_FLAGS += mbed_config.h
C_FLAGS += -c
C_FLAGS += -std=gnu11
C_FLAGS += -Wall
C_FLAGS += -Wextra
C_FLAGS += -Wno-unused-parameter
C_FLAGS += -Wno-missing-field-initializers
C_FLAGS += -fmessage-length=0
C_FLAGS += -fno-exceptions
C_FLAGS += -ffunction-sections
C_FLAGS += -fdata-sections
C_FLAGS += -funsigned-char
C_FLAGS += -MMD
C_FLAGS += -fomit-frame-pointer
C_FLAGS += -Os
C_FLAGS += -g
C_FLAGS += -DMBED_TRAP_ERRORS_ENABLED=1
C_FLAGS += -DMBED_MINIMAL_PRINTF
C_FLAGS += -mcpu=cortex-m4
C_FLAGS += -mthumb
C_FLAGS += -mfpu=fpv4-sp-d16
C_FLAGS += -mfloat-abi=softfp

CXX_FLAGS += -c
CXX_FLAGS += -std=gnu++14
CXX_FLAGS += -fno-rtti
CXX_FLAGS += -Wvla
CXX_FLAGS += -include
CXX_FLAGS += mbed_config.h
CXX_FLAGS += -DTARGET_LIKE_CORTEX_M4
CXX_FLAGS += -DTOOLCHAIN_GCC
CXX_FLAGS += -DDEVICE_STDIO_MESSAGES=1
CXX_FLAGS += -DOPEN_DRAIN_LEDS
CXX_FLAGS += -DDEVICE_PORTINOUT=1
CXX_FLAGS += -D__FPU_PRESENT=1
CXX_FLAGS += -DMBED_TICKLESS
CXX_FLAGS += -DDEVICE_LPTICKER=1
CXX_FLAGS += -DDEVICE_MPU=1
CXX_FLAGS += -DDEVICE_I2C=1
CXX_FLAGS += -DDEVICE_INTERRUPTIN=1
CXX_FLAGS += -DTARGET=MAX32630
CXX_FLAGS += -DDEVICE_SPI=1
CXX_FLAGS += -DTARGET_LIKE_MBED
CXX_FLAGS += -DDEVICE_PWMOUT=1
CXX_FLAGS += -DTARGET_MAX32630FTHR
CXX_FLAGS += -D__SYSTEM_HFX=96000000
CXX_FLAGS += -DBLE_HCI_UART
CXX_FLAGS += -DDEVICE_PORTIN=1
CXX_FLAGS += -D__CMSIS_RTOS
CXX_FLAGS += -DDEVICE_USTICKER=1
CXX_FLAGS += -DTARGET_CORTEX
CXX_FLAGS += -DTARGET_RTOS_M4_M7
CXX_FLAGS += -DTARGET_CORTEX_M
CXX_FLAGS += -D__MBED_CMSIS_RTOS_CM
CXX_FLAGS += -DDEVICE_PORTOUT=1
CXX_FLAGS += -D__CORTEX_M4
CXX_FLAGS += -DTARGET_REV=0x4132
CXX_FLAGS += -DTARGET_RELEASE
CXX_FLAGS += -DDEVICE_SERIAL_FC=1
CXX_FLAGS += -DDEVICE_ANALOGIN=1
CXX_FLAGS += -DTARGET_M4
CXX_FLAGS += -DTARGET_Maxim
CXX_FLAGS += -DTOOLCHAIN_GCC_ARM
CXX_FLAGS += -DTARGET_NAME=MAX32630FTHR
CXX_FLAGS += -D__MBED__=1
CXX_FLAGS += -DMBED_BUILD_TIMESTAMP=1621198569.986343
CXX_FLAGS += -DTARGET_MAX32630
CXX_FLAGS += -DARM_MATH_CM4
CXX_FLAGS += -DDEVICE_SERIAL=1
CXX_FLAGS += -include
CXX_FLAGS += mbed_config.h
CXX_FLAGS += -c
CXX_FLAGS += -std=gnu++14
CXX_FLAGS += -fno-rtti
CXX_FLAGS += -Wvla
CXX_FLAGS += -Wall
CXX_FLAGS += -Wextra
CXX_FLAGS += -Wno-unused-parameter
CXX_FLAGS += -Wno-missing-field-initializers
CXX_FLAGS += -fmessage-length=0
CXX_FLAGS += -fno-exceptions
CXX_FLAGS += -ffunction-sections
CXX_FLAGS += -fdata-sections
CXX_FLAGS += -funsigned-char
CXX_FLAGS += -MMD
CXX_FLAGS += -fomit-frame-pointer
CXX_FLAGS += -Os
CXX_FLAGS += -g
CXX_FLAGS += -DMBED_TRAP_ERRORS_ENABLED=1
CXX_FLAGS += -DMBED_MINIMAL_PRINTF
CXX_FLAGS += -mcpu=cortex-m4
CXX_FLAGS += -mthumb
CXX_FLAGS += -mfpu=fpv4-sp-d16
CXX_FLAGS += -mfloat-abi=softfp

ASM_FLAGS += -c
ASM_FLAGS += -x
ASM_FLAGS += assembler-with-cpp
ASM_FLAGS += -DTARGET=MAX32630
ASM_FLAGS += -DOPEN_DRAIN_LEDS
ASM_FLAGS += -D__MBED_CMSIS_RTOS_CM
ASM_FLAGS += -D__CORTEX_M4
ASM_FLAGS += -D__SYSTEM_HFX=96000000
ASM_FLAGS += -DBLE_HCI_UART
ASM_FLAGS += -D__FPU_PRESENT=1
ASM_FLAGS += -DMBED_TICKLESS
ASM_FLAGS += -D__CMSIS_RTOS
ASM_FLAGS += -DTARGET_REV=0x4132
ASM_FLAGS += -DARM_MATH_CM4
ASM_FLAGS += -I/usr/src/mbed-sdk
ASM_FLAGS += -I../USBDevice
ASM_FLAGS += -I../USBDevice/USBAudio
ASM_FLAGS += -I../USBDevice/USBDevice
ASM_FLAGS += -I../USBDevice/USBHID
ASM_FLAGS += -I../USBDevice/USBMIDI
ASM_FLAGS += -I../USBDevice/USBMSD
ASM_FLAGS += -I../USBDevice/USBSerial
ASM_FLAGS += -I../max32630fthr
ASM_FLAGS += -I../max32630fthr/MAX14690
ASM_FLAGS += -I../mbed
ASM_FLAGS += -I../mbed/TARGET_MAX32630FTHR/TOOLCHAIN_GCC_ARM
ASM_FLAGS += -I../mbed/drivers
ASM_FLAGS += -I../mbed/hal
ASM_FLAGS += -I../mbed/platform
ASM_FLAGS += -include
ASM_FLAGS += /filer/workspace_data/exports/f/f3d3396b8e82395879f8772ff82d313b/MAX32630FTHR_Neurosky/mbed_config.h
ASM_FLAGS += -c
ASM_FLAGS += -x
ASM_FLAGS += assembler-with-cpp
ASM_FLAGS += -Wall
ASM_FLAGS += -Wextra
ASM_FLAGS += -Wno-unused-parameter
ASM_FLAGS += -Wno-missing-field-initializers
ASM_FLAGS += -fmessage-length=0
ASM_FLAGS += -fno-exceptions
ASM_FLAGS += -ffunction-sections
ASM_FLAGS += -fdata-sections
ASM_FLAGS += -funsigned-char
ASM_FLAGS += -MMD
ASM_FLAGS += -fomit-frame-pointer
ASM_FLAGS += -Os
ASM_FLAGS += -g
ASM_FLAGS += -DMBED_TRAP_ERRORS_ENABLED=1
ASM_FLAGS += -DMBED_MINIMAL_PRINTF
ASM_FLAGS += -mcpu=cortex-m4
ASM_FLAGS += -mthumb
ASM_FLAGS += -mfpu=fpv4-sp-d16
ASM_FLAGS += -mfloat-abi=softfp


LD_FLAGS :=-Wl,--gc-sections -Wl,--wrap,main -Wl,--wrap,_malloc_r -Wl,--wrap,_free_r -Wl,--wrap,_realloc_r -Wl,--wrap,_memalign_r -Wl,--wrap,_calloc_r -Wl,--wrap,exit -Wl,--wrap,atexit -Wl,-n -Wl,--wrap,printf -Wl,--wrap,sprintf -Wl,--wrap,snprintf -Wl,--wrap,vprintf -Wl,--wrap,vsprintf -Wl,--wrap,vsnprintf -Wl,--wrap,fprintf -Wl,--wrap,vfprintf -mcpu=cortex-m4 -mthumb -mfpu=fpv4-sp-d16 -mfloat-abi=softfp -DMBED_BOOT_STACK_SIZE=4096 -DXIP_ENABLE=0 
LD_SYS_LIBS :=-Wl,--start-group -lstdc++ -lsupc++ -lm -lc -lgcc -lnosys -lexactLE -lmbed -Wl,--end-group

# Tools and Flags
###############################################################################
# Rules

.PHONY: all lst size


all: $(PROJECT).bin $(PROJECT).hex size


.s.o:
	+@$(call MAKEDIR,$(dir $@))
	+@echo "Assemble: $(notdir $<)"
  
	@$(AS) -c $(ASM_FLAGS) -o $@ $<
  


.S.o:
	+@$(call MAKEDIR,$(dir $@))
	+@echo "Assemble: $(notdir $<)"
  
	@$(AS) -c $(ASM_FLAGS) -o $@ $<
  

.c.o:
	+@$(call MAKEDIR,$(dir $@))
	+@echo "Compile: $(notdir $<)"
	@$(CC) $(C_FLAGS) $(INCLUDE_PATHS) -o $@ $<

.cpp.o:
	+@$(call MAKEDIR,$(dir $@))
	+@echo "Compile: $(notdir $<)"
	@$(CPP) $(CXX_FLAGS) $(INCLUDE_PATHS) -o $@ $<


$(PROJECT).link_script.ld: $(LINKER_SCRIPT)
	@$(PREPROC) $< -o $@



$(PROJECT).elf: $(OBJECTS) $(SYS_OBJECTS) $(PROJECT).link_script.ld 
	+@echo "$(filter %.o, $^)" > .link_options.txt
	+@echo "link: $(notdir $@)"
	@$(LD) $(LD_FLAGS) -T $(filter-out %.o, $^) $(LIBRARY_PATHS) --output $@ @.link_options.txt $(LIBRARIES) $(LD_SYS_LIBS)


$(PROJECT).bin: $(PROJECT).elf
	$(ELF2BIN) -O binary $< $@
	+@echo "===== bin file ready to flash: $(OBJDIR)/$@ =====" 

$(PROJECT).hex: $(PROJECT).elf
	$(ELF2BIN) -O ihex $< $@


# Rules
###############################################################################
# Dependencies

DEPS = $(OBJECTS:.o=.d) $(SYS_OBJECTS:.o=.d)
-include $(DEPS)
endif

# Dependencies
###############################################################################
# Catch-all

%: ;

# Catch-all
###############################################################################