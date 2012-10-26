################################################################################
# Automatically-generated file. Do not edit!
################################################################################

-include ../makefile.init

RM := rm -rf

# All of the sources participating in the build are defined here
-include sources.mk
-include subdir.mk
-include src/subdir.mk
-include objects.mk

ifneq ($(MAKECMDGOALS),clean)
ifneq ($(strip $(C_DEPS)),)
-include $(C_DEPS)
endif
endif

-include ../makefile.defs

# Add inputs and outputs from these tool invocations to the build variables 

# All Target
all: 1_Blinky_SysTick.axf

# Tool invocations
1_Blinky_SysTick.axf: $(OBJS) $(USER_OBJS)
	@echo 'Building target: $@'
	@echo 'Invoking: MCU Linker'
	arm-none-eabi-gcc -nostdlib -L"C:\Users\Alan\Dropbox\Tesis\workspace\Lib_CMSIS2p0_and_Drivers_17XX\Debug" -Xlinker -Map=1_Blinky_SysTick.map -Xlinker --gc-sections -mcpu=cortex-m3 -mthumb -T "Blinky_basic_Debug.ld" -o"1_Blinky_SysTick.axf" $(OBJS) $(USER_OBJS) $(LIBS)
	@echo 'Finished building target: $@'
	@echo ' '
	$(MAKE) --no-print-directory post-build

# Other Targets
clean:
	-$(RM) $(OBJS)$(C_DEPS)$(EXECUTABLES) 1_Blinky_SysTick.axf
	-@echo ' '

post-build:
	-@echo 'Performing post-build steps'
	-arm-none-eabi-size 1_Blinky_SysTick.axf; # arm-none-eabi-objcopy -O binary 1_Blinky_SysTick.axf 1_Blinky_SysTick.bin ; checksum -p LPC1769 -d 1_Blinky_SysTick.bin;
	-@echo ' '

.PHONY: all clean dependents
.SECONDARY: post-build

-include ../makefile.targets
