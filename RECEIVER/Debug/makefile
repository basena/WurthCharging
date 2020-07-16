################################################################################
# Automatically-generated file. Do not edit!
################################################################################

-include ../makefile.init

RM := rm -rf

# All of the sources participating in the build are defined here
-include sources.mk
-include Startup/subdir.mk
-include Libraries/XMCLib/src/subdir.mk
-include Libraries/Newlib/subdir.mk
-include Dave/Generated/UCPROBE/subdir.mk
-include Dave/Generated/UART/subdir.mk
-include Dave/Generated/PWM_CCU4/subdir.mk
-include Dave/Generated/INTERRUPT/subdir.mk
-include Dave/Generated/GLOBAL_CCU4/subdir.mk
-include Dave/Generated/GLOBAL_ADC/subdir.mk
-include Dave/Generated/DIGITAL_IO/subdir.mk
-include Dave/Generated/CPU_CTRL_XMC1/subdir.mk
-include Dave/Generated/CLOCK_XMC1/subdir.mk
-include Dave/Generated/ADC_MEASUREMENT_ADV/subdir.mk
-include Dave/Generated/subdir.mk
-include subdir.mk
-include objects.mk

ifneq ($(MAKECMDGOALS),clean)
ifneq ($(strip $(S_DEPS)),)
-include $(S_DEPS)
endif
ifneq ($(strip $(S_UPPER_DEPS)),)
-include $(S_UPPER_DEPS)
endif
ifneq ($(strip $(C_DEPS)),)
-include $(C_DEPS)
endif
endif

-include ../makefile.defs

# Add inputs and outputs from these tool invocations to the build variables 
SECONDARY_FLASH += \
RECEIVER.hex \

SECONDARY_LIST += \
RECEIVER.lst \

SECONDARY_SIZE += \
RECEIVER.siz \


LIST_FILES = $(addsuffix .o.lst, $(basename $(OBJS))) 
MAP_FILE = "RECEIVER.map"

TOOLCHAIN_ROOT=C:/DAVE-IDE-4.4.2-32Bit/eclipse/ARM-GCC-49
PROJECT_LOC=C:/FW/WIRELESS_CHARGER_RECEIVER

# All Target
all: RECEIVER.elf secondary-outputs

# Tool invocations
RECEIVER.elf: $(OBJS) ../linker_script.ld $(USER_OBJS)
	@echo 'Building target: $@'
	@echo 'Invoking: ARM-GCC C Linker'
	$(file >objects.rsp,$(OBJS))
	"$(TOOLCHAIN_ROOT)/bin/arm-none-eabi-gcc" -T"../linker_script.ld" -nostartfiles -Xlinker --gc-sections -specs=nano.specs -specs=nosys.specs -Wl,-Map,$(MAP_FILE) -mcpu=cortex-m0 -mthumb -g -gdwarf-2 -o "RECEIVER.elf" "@objects.rsp" $(USER_OBJS) $(LIBS)
	@echo 'Finished building target: $@'
	@echo.
RECEIVER.hex: RECEIVER.elf
	@echo 'Invoking: ARM-GCC Create Flash Image'
	"$(TOOLCHAIN_ROOT)/bin/arm-none-eabi-objcopy" -O ihex "RECEIVER.elf" "RECEIVER.hex"
	@echo 'Finished building: $@'
	@echo.
RECEIVER.lst: RECEIVER.elf
	@echo 'Invoking: ARM-GCC Create Listing'
	"$(TOOLCHAIN_ROOT)/bin/arm-none-eabi-objdump" -h -S "RECEIVER.elf" > "RECEIVER.lst"
	@echo 'Finished building: $@'
	@echo.
RECEIVER.siz: RECEIVER.elf
	@echo 'Invoking: ARM-GCC Print Size'
	"$(TOOLCHAIN_ROOT)/bin/arm-none-eabi-size" --format=berkeley "RECEIVER.elf"
	@echo 'Finished building: $@'
	@echo.
# Other Targets
clean:
	-$(RM) $(SECONDARY_FLASH)$(SECONDARY_LIST)$(SECONDARY_SIZE)$(ARM_GCC_FOR_XMC_LINKER_OUTPUT_TYPE_OUTPUTS)$(S_DEPS)$(S_UPPER_DEPS)$(C_DEPS)
	-$(RM) Dave/Generated/ADC_MEASUREMENT_ADV/adc_measurement_adv.o Dave/Generated/ADC_MEASUREMENT_ADV/adc_measurement_adv_conf.o Dave/Generated/CLOCK_XMC1/clock_xmc1.o Dave/Generated/CLOCK_XMC1/clock_xmc1_conf.o Dave/Generated/CPU_CTRL_XMC1/cpu_ctrl_xmc1.o Dave/Generated/CPU_CTRL_XMC1/cpu_ctrl_xmc1_conf.o Dave/Generated/DAVE.o Dave/Generated/DIGITAL_IO/digital_io.o Dave/Generated/DIGITAL_IO/digital_io_conf.o Dave/Generated/GLOBAL_ADC/global_adc.o Dave/Generated/GLOBAL_ADC/global_adc_conf.o Dave/Generated/GLOBAL_CCU4/global_ccu4.o Dave/Generated/GLOBAL_CCU4/global_ccu4_conf.o Dave/Generated/INTERRUPT/interrupt.o Dave/Generated/INTERRUPT/interrupt_conf.o Dave/Generated/PWM_CCU4/pwm_ccu4.o Dave/Generated/PWM_CCU4/pwm_ccu4_conf.o Dave/Generated/UART/uart.o Dave/Generated/UART/uart_conf.o Dave/Generated/UCPROBE/cpu_a.o Dave/Generated/UCPROBE/cpu_bsp.o Dave/Generated/UCPROBE/cpu_c.o Dave/Generated/UCPROBE/cpu_core.o Dave/Generated/UCPROBE/lib_ascii.o Dave/Generated/UCPROBE/lib_math.o Dave/Generated/UCPROBE/lib_mem.o Dave/Generated/UCPROBE/lib_mem_a.o Dave/Generated/UCPROBE/lib_str.o Dave/Generated/UCPROBE/probe_com.o Dave/Generated/UCPROBE/probe_com_os.o Dave/Generated/UCPROBE/probe_rs232.o Dave/Generated/UCPROBE/probe_rs232_os.o Dave/Generated/UCPROBE/probe_rs232c.o Dave/Generated/UCPROBE/probe_scope.o Dave/Generated/UCPROBE/ucprobe.o Dave/Generated/UCPROBE/ucprobe_conf.o Libraries/Newlib/syscalls.o Libraries/XMCLib/src/xmc1_eru.o Libraries/XMCLib/src/xmc1_flash.o Libraries/XMCLib/src/xmc1_gpio.o Libraries/XMCLib/src/xmc1_rtc.o Libraries/XMCLib/src/xmc1_scu.o Libraries/XMCLib/src/xmc_acmp.o Libraries/XMCLib/src/xmc_bccu.o Libraries/XMCLib/src/xmc_can.o Libraries/XMCLib/src/xmc_ccu4.o Libraries/XMCLib/src/xmc_ccu8.o Libraries/XMCLib/src/xmc_common.o Libraries/XMCLib/src/xmc_eru.o Libraries/XMCLib/src/xmc_gpio.o Libraries/XMCLib/src/xmc_i2c.o Libraries/XMCLib/src/xmc_i2s.o Libraries/XMCLib/src/xmc_ledts.o 
	-$(RM) Libraries/XMCLib/src/xmc_math.o Libraries/XMCLib/src/xmc_pau.o Libraries/XMCLib/src/xmc_posif.o Libraries/XMCLib/src/xmc_prng.o Libraries/XMCLib/src/xmc_rtc.o Libraries/XMCLib/src/xmc_spi.o Libraries/XMCLib/src/xmc_uart.o Libraries/XMCLib/src/xmc_usic.o Libraries/XMCLib/src/xmc_vadc.o Libraries/XMCLib/src/xmc_wdt.o Startup/startup_XMC1300.o Startup/system_XMC1300.o main.o  RECEIVER.elf $(LIST_FILES) $(MAP_FILE) objects.rsp
	-@echo.
secondary-outputs: $(SECONDARY_FLASH) $(SECONDARY_LIST) $(SECONDARY_SIZE)

.PHONY: all clean dependents
.SECONDARY:

-include ../makefile.targets
