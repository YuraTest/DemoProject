################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Drivers/CMSIS/src/system_stm32f4xx.c 

S_UPPER_SRCS += \
../Drivers/CMSIS/src/startup_stm32f40_41xxx.S 

OBJS += \
./Drivers/CMSIS/src/startup_stm32f40_41xxx.o \
./Drivers/CMSIS/src/system_stm32f4xx.o 

S_UPPER_DEPS += \
./Drivers/CMSIS/src/startup_stm32f40_41xxx.d 

C_DEPS += \
./Drivers/CMSIS/src/system_stm32f4xx.d 


# Each subdirectory must supply rules for building sources it contributes
Drivers/CMSIS/src/%.o: ../Drivers/CMSIS/src/%.S
	@echo 'Building file: $<'
	@echo 'Invoking: Cross ARM GNU Assembler'
	arm-none-eabi-gcc -mcpu=cortex-m4 -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16 -Os -fmessage-length=0 -fsigned-char -ffunction-sections -fdata-sections  -g -x assembler-with-cpp -I"/home/yurock/workspace/DemoProject/freeRtos_SPL/Drivers/CMSIS/inc" -I"/home/yurock/workspace/DemoProject/freeRtos_SPL/Drivers/SLP/inc" -I"/home/yurock/workspace/DemoProject/freeRtos_SPL/Inc" -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@)" -c -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '

Drivers/CMSIS/src/%.o: ../Drivers/CMSIS/src/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: Cross ARM C Compiler'
	arm-none-eabi-gcc -mcpu=cortex-m4 -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16 -Os -fmessage-length=0 -fsigned-char -ffunction-sections -fdata-sections  -g -DSTM32F40_41xxx -DUSE_STDPERIPH_DRIVER -I"/home/yurock/workspace/DemoProject/freeRtos_SPL" -I"/home/yurock/workspace/DemoProject/freeRtos_SPL/Drivers/CMSIS/inc" -I"/home/yurock/workspace/DemoProject/freeRtos_SPL/Drivers/SLP/inc" -I"/home/yurock/workspace/DemoProject/freeRtos_SPL/Inc" -I"/home/yurock/workspace/DemoProject/freeRtos_SPL/FreeRTOS" -I"/home/yurock/workspace/DemoProject/freeRtos_SPL/FreeRTOS/include" -I"/home/yurock/workspace/DemoProject/freeRtos_SPL/FATFS_SDIO" -I"/home/yurock/workspace/DemoProject/freeRtos_SPL/FATFS_SDIO/fatfs" -I"/home/yurock/workspace/DemoProject/freeRtos_SPL/FATFS_SDIO/fatfs/lo_level_ub" -std=gnu11 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@)" -c -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


