################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../HAL/Src/stm32f4xx_hal.c \
../HAL/Src/stm32f4xx_hal_cortex.c \
../HAL/Src/stm32f4xx_hal_dma.c \
../HAL/Src/stm32f4xx_hal_dma_ex.c \
../HAL/Src/stm32f4xx_hal_flash.c \
../HAL/Src/stm32f4xx_hal_flash_ex.c \
../HAL/Src/stm32f4xx_hal_flash_ramfunc.c \
../HAL/Src/stm32f4xx_hal_gpio.c \
../HAL/Src/stm32f4xx_hal_pwr.c \
../HAL/Src/stm32f4xx_hal_pwr_ex.c \
../HAL/Src/stm32f4xx_hal_rcc.c \
../HAL/Src/stm32f4xx_hal_rcc_ex.c \
../HAL/Src/stm32f4xx_hal_tim.c \
../HAL/Src/stm32f4xx_hal_tim_ex.c 

OBJS += \
./HAL/Src/stm32f4xx_hal.o \
./HAL/Src/stm32f4xx_hal_cortex.o \
./HAL/Src/stm32f4xx_hal_dma.o \
./HAL/Src/stm32f4xx_hal_dma_ex.o \
./HAL/Src/stm32f4xx_hal_flash.o \
./HAL/Src/stm32f4xx_hal_flash_ex.o \
./HAL/Src/stm32f4xx_hal_flash_ramfunc.o \
./HAL/Src/stm32f4xx_hal_gpio.o \
./HAL/Src/stm32f4xx_hal_pwr.o \
./HAL/Src/stm32f4xx_hal_pwr_ex.o \
./HAL/Src/stm32f4xx_hal_rcc.o \
./HAL/Src/stm32f4xx_hal_rcc_ex.o \
./HAL/Src/stm32f4xx_hal_tim.o \
./HAL/Src/stm32f4xx_hal_tim_ex.o 

C_DEPS += \
./HAL/Src/stm32f4xx_hal.d \
./HAL/Src/stm32f4xx_hal_cortex.d \
./HAL/Src/stm32f4xx_hal_dma.d \
./HAL/Src/stm32f4xx_hal_dma_ex.d \
./HAL/Src/stm32f4xx_hal_flash.d \
./HAL/Src/stm32f4xx_hal_flash_ex.d \
./HAL/Src/stm32f4xx_hal_flash_ramfunc.d \
./HAL/Src/stm32f4xx_hal_gpio.d \
./HAL/Src/stm32f4xx_hal_pwr.d \
./HAL/Src/stm32f4xx_hal_pwr_ex.d \
./HAL/Src/stm32f4xx_hal_rcc.d \
./HAL/Src/stm32f4xx_hal_rcc_ex.d \
./HAL/Src/stm32f4xx_hal_tim.d \
./HAL/Src/stm32f4xx_hal_tim_ex.d 


# Each subdirectory must supply rules for building sources it contributes
HAL/Src/%.o: ../HAL/Src/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: Cross ARM C Compiler'
	arm-none-eabi-gcc -mcpu=cortex-m4 -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16 -Os -fmessage-length=0 -fsigned-char -ffunction-sections -fdata-sections -fno-builtin -Werror -Wall  -g -DUSE_STDPERIPH_DRIVER -DSTM32F40XX -DSTM32F407xx -I"/home/yurock/workspace/DemoProject/test_hal/Inc" -I"/home/yurock/workspace/DemoProject/test_hal/HAL/Inc/Legacy" -I"/home/yurock/workspace/DemoProject/test_hal/HAL/Inc" -I"/home/yurock/workspace/DemoProject/test_hal/CMSIS_BOOT" -I"/home/yurock/workspace/DemoProject/test_hal/CMSIS" -std=gnu11 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@)" -c -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


