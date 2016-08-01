################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Src/freertos.c \
../Src/main.c \
../Src/perirh_init.c \
../Src/printf.c \
../Src/stm32f4xx_it.c \
../Src/tim6.c \
../Src/uart.c \
../Src/uart1Task.c 

OBJS += \
./Src/freertos.o \
./Src/main.o \
./Src/perirh_init.o \
./Src/printf.o \
./Src/stm32f4xx_it.o \
./Src/tim6.o \
./Src/uart.o \
./Src/uart1Task.o 

C_DEPS += \
./Src/freertos.d \
./Src/main.d \
./Src/perirh_init.d \
./Src/printf.d \
./Src/stm32f4xx_it.d \
./Src/tim6.d \
./Src/uart.d \
./Src/uart1Task.d 


# Each subdirectory must supply rules for building sources it contributes
Src/%.o: ../Src/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: Cross ARM C Compiler'
	arm-none-eabi-gcc -mcpu=cortex-m4 -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16 -Os -fmessage-length=0 -fsigned-char -ffunction-sections -fdata-sections  -g -DSTM32F40_41xxx -DUSE_STDPERIPH_DRIVER -I"/home/yurock/workspace/stm32f4/freeRtos_SPL" -I"/home/yurock/workspace/stm32f4/freeRtos_SPL/Drivers/CMSIS/inc" -I"/home/yurock/workspace/stm32f4/freeRtos_SPL/Drivers/SLP/inc" -I"/home/yurock/workspace/stm32f4/freeRtos_SPL/Inc" -I"/home/yurock/workspace/stm32f4/freeRtos_SPL/FreeRTOS" -I"/home/yurock/workspace/stm32f4/freeRtos_SPL/FreeRTOS/include" -std=gnu11 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@)" -c -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


