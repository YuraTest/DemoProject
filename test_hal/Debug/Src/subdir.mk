################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Src/gpio.c \
../Src/main.c \
../Src/new.c \
../Src/stm32f4xx_hal_msp.c \
../Src/stm32f4xx_it.c 

OBJS += \
./Src/gpio.o \
./Src/main.o \
./Src/new.o \
./Src/stm32f4xx_hal_msp.o \
./Src/stm32f4xx_it.o 

C_DEPS += \
./Src/gpio.d \
./Src/main.d \
./Src/new.d \
./Src/stm32f4xx_hal_msp.d \
./Src/stm32f4xx_it.d 


# Each subdirectory must supply rules for building sources it contributes
Src/%.o: ../Src/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: Cross ARM C Compiler'
	arm-none-eabi-gcc -mcpu=cortex-m4 -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16 -Os -fmessage-length=0 -fsigned-char -ffunction-sections -fdata-sections -fno-builtin -Werror -Wall  -g -DUSE_STDPERIPH_DRIVER -DSTM32F40XX -DSTM32F407xx -I"/home/yurock/workspace/DemoProject/test_hal/Inc" -I"/home/yurock/workspace/DemoProject/test_hal/HAL/Inc/Legacy" -I"/home/yurock/workspace/DemoProject/test_hal/HAL/Inc" -I"/home/yurock/workspace/DemoProject/test_hal/CMSIS_BOOT" -I"/home/yurock/workspace/DemoProject/test_hal/CMSIS" -std=gnu11 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@)" -c -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


