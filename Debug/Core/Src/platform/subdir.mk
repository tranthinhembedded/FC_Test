################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (14.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Src/platform/delay.c \
../Core/Src/platform/dma.c \
../Core/Src/platform/gpio.c \
../Core/Src/platform/i2c.c \
../Core/Src/platform/spi.c \
../Core/Src/platform/stm32f4xx_hal_msp.c \
../Core/Src/platform/stm32f4xx_it.c \
../Core/Src/platform/syscalls.c \
../Core/Src/platform/sysmem.c \
../Core/Src/platform/system_stm32f4xx.c \
../Core/Src/platform/tim.c \
../Core/Src/platform/usart.c 

OBJS += \
./Core/Src/platform/delay.o \
./Core/Src/platform/dma.o \
./Core/Src/platform/gpio.o \
./Core/Src/platform/i2c.o \
./Core/Src/platform/spi.o \
./Core/Src/platform/stm32f4xx_hal_msp.o \
./Core/Src/platform/stm32f4xx_it.o \
./Core/Src/platform/syscalls.o \
./Core/Src/platform/sysmem.o \
./Core/Src/platform/system_stm32f4xx.o \
./Core/Src/platform/tim.o \
./Core/Src/platform/usart.o 

C_DEPS += \
./Core/Src/platform/delay.d \
./Core/Src/platform/dma.d \
./Core/Src/platform/gpio.d \
./Core/Src/platform/i2c.d \
./Core/Src/platform/spi.d \
./Core/Src/platform/stm32f4xx_hal_msp.d \
./Core/Src/platform/stm32f4xx_it.d \
./Core/Src/platform/syscalls.d \
./Core/Src/platform/sysmem.d \
./Core/Src/platform/system_stm32f4xx.d \
./Core/Src/platform/tim.d \
./Core/Src/platform/usart.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Src/platform/%.o Core/Src/platform/%.su Core/Src/platform/%.cyclo: ../Core/Src/platform/%.c Core/Src/platform/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F411xE -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I../Middlewares/ST/ARM/DSP/Inc -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-Src-2f-platform

clean-Core-2f-Src-2f-platform:
	-$(RM) ./Core/Src/platform/delay.cyclo ./Core/Src/platform/delay.d ./Core/Src/platform/delay.o ./Core/Src/platform/delay.su ./Core/Src/platform/dma.cyclo ./Core/Src/platform/dma.d ./Core/Src/platform/dma.o ./Core/Src/platform/dma.su ./Core/Src/platform/gpio.cyclo ./Core/Src/platform/gpio.d ./Core/Src/platform/gpio.o ./Core/Src/platform/gpio.su ./Core/Src/platform/i2c.cyclo ./Core/Src/platform/i2c.d ./Core/Src/platform/i2c.o ./Core/Src/platform/i2c.su ./Core/Src/platform/spi.cyclo ./Core/Src/platform/spi.d ./Core/Src/platform/spi.o ./Core/Src/platform/spi.su ./Core/Src/platform/stm32f4xx_hal_msp.cyclo ./Core/Src/platform/stm32f4xx_hal_msp.d ./Core/Src/platform/stm32f4xx_hal_msp.o ./Core/Src/platform/stm32f4xx_hal_msp.su ./Core/Src/platform/stm32f4xx_it.cyclo ./Core/Src/platform/stm32f4xx_it.d ./Core/Src/platform/stm32f4xx_it.o ./Core/Src/platform/stm32f4xx_it.su ./Core/Src/platform/syscalls.cyclo ./Core/Src/platform/syscalls.d ./Core/Src/platform/syscalls.o ./Core/Src/platform/syscalls.su ./Core/Src/platform/sysmem.cyclo ./Core/Src/platform/sysmem.d ./Core/Src/platform/sysmem.o ./Core/Src/platform/sysmem.su ./Core/Src/platform/system_stm32f4xx.cyclo ./Core/Src/platform/system_stm32f4xx.d ./Core/Src/platform/system_stm32f4xx.o ./Core/Src/platform/system_stm32f4xx.su ./Core/Src/platform/tim.cyclo ./Core/Src/platform/tim.d ./Core/Src/platform/tim.o ./Core/Src/platform/tim.su ./Core/Src/platform/usart.cyclo ./Core/Src/platform/usart.d ./Core/Src/platform/usart.o ./Core/Src/platform/usart.su

.PHONY: clean-Core-2f-Src-2f-platform

