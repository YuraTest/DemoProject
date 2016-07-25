################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../CMSIS_BOOT/system_stm32f4xx.c 

S_UPPER_SRCS += \
../CMSIS_BOOT/startup_stm32f407xx.S 

OBJS += \
./CMSIS_BOOT/startup_stm32f407xx.o \
./CMSIS_BOOT/system_stm32f4xx.o 

S_UPPER_DEPS += \
./CMSIS_BOOT/startup_stm32f407xx.d 

C_DEPS += \
./CMSIS_BOOT/system_stm32f4xx.d 


# Each subdirectory must supply rules for building sources it contributes
CMSIS_BOOT/%.o: ../CMSIS_BOOT/%.S
	@echo 'Building file: $<'
	@echo 'Invoking: Cross ARM GNU Assembler'
	arm-none-eabi-gcc -mcpu=cortex-m4 -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16 -Os -fmessage-length=0 -fsigned-char -ffunction-sections -fdata-sections -fno-builtin -Werror -Wall  -g -x assembler-with-cpp -DUSE_STDPERIPH_DRIVER -DSTM32F40XX -I"/home/yurock/workspace/DemoProject/test_hal/CMSIS_BOOT" -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@)" -c -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '

CMSIS_BOOT/%.o: ../CMSIS_BOOT/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: Cross ARM C Compiler'
	arm-none-eabi-gcc -mcpu=cortex-m4 -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16 -Os -fmessage-length=0 -fsigned-char -ffunction-sections -fdata-sections -fno-builtin -Werror -Wall  -g -DUSE_STDPERIPH_DRIVER -DSTM32F40XX -DSTM32F407xx -I"/home/yurock/workspace/DemoProject/test_hal/Inc" -I"/home/yurock/workspace/DemoProject/test_hal/HAL/Inc/Legacy" -I"/home/yurock/workspace/DemoProject/test_hal/HAL/Inc" -I"/home/yurock/workspace/DemoProject/test_hal/CMSIS_BOOT" -I"/home/yurock/workspace/DemoProject/test_hal/CMSIS" -std=gnu11 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@)" -c -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


