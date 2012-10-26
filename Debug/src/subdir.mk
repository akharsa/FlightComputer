################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../src/cr_startup_lpc176x.c \
../src/main.c 

OBJS += \
./src/cr_startup_lpc176x.o \
./src/main.o 

C_DEPS += \
./src/cr_startup_lpc176x.d \
./src/main.d 


# Each subdirectory must supply rules for building sources it contributes
src/cr_startup_lpc176x.o: ../src/cr_startup_lpc176x.c
	@echo 'Building file: $<'
	@echo 'Invoking: MCU C Compiler'
	arm-none-eabi-gcc -D__USE_CMSIS -D__REDLIB__ -DDEBUG -D__CODE_RED -I"C:\Users\Alan\Dropbox\Tesis\workspace\Lib_CMSIS2p0_and_Drivers_17XX\Core\CM3\CoreSupport" -I"C:\Users\Alan\Dropbox\Tesis\workspace\Lib_CMSIS2p0_and_Drivers_17XX\Core\CM3\DeviceSupport\NXP\LPC17xx" -I"C:\Users\Alan\Dropbox\Tesis\workspace\Lib_CMSIS2p0_and_Drivers_17XX\Drivers\include" -O0 -Os -g3 -Wall -c -fmessage-length=0 -fno-builtin -ffunction-sections -fdata-sections -mcpu=cortex-m3 -mthumb -MMD -MP -MF"$(@:%.o=%.d)" -MT"src/cr_startup_lpc176x.d" -o"$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '

src/%.o: ../src/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: MCU C Compiler'
	arm-none-eabi-gcc -D__USE_CMSIS -D__REDLIB__ -DDEBUG -D__CODE_RED -I"C:\Users\Alan\Dropbox\Tesis\workspace\Lib_CMSIS2p0_and_Drivers_17XX\Core\CM3\CoreSupport" -I"C:\Users\Alan\Dropbox\Tesis\workspace\Lib_CMSIS2p0_and_Drivers_17XX\Core\CM3\DeviceSupport\NXP\LPC17xx" -I"C:\Users\Alan\Dropbox\Tesis\workspace\Lib_CMSIS2p0_and_Drivers_17XX\Drivers\include" -O0 -g3 -Wall -c -fmessage-length=0 -fno-builtin -ffunction-sections -fdata-sections -mcpu=cortex-m3 -mthumb -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o"$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


