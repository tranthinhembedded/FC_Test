################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (14.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Src/platform/delay.c \
../Core/Src/platform/syscalls.c \
../Core/Src/platform/sysmem.c 

OBJS += \
./Core/Src/platform/delay.o \
./Core/Src/platform/syscalls.o \
./Core/Src/platform/sysmem.o 

C_DEPS += \
./Core/Src/platform/delay.d \
./Core/Src/platform/syscalls.d \
./Core/Src/platform/sysmem.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Src/platform/%.o Core/Src/platform/%.su Core/Src/platform/%.cyclo: ../Core/Src/platform/%.c Core/Src/platform/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F411xE -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I../Middlewares/ST/ARM/DSP/Inc -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-Src-2f-platform

clean-Core-2f-Src-2f-platform:
	-$(RM) ./Core/Src/platform/delay.cyclo ./Core/Src/platform/delay.d ./Core/Src/platform/delay.o ./Core/Src/platform/delay.su ./Core/Src/platform/syscalls.cyclo ./Core/Src/platform/syscalls.d ./Core/Src/platform/syscalls.o ./Core/Src/platform/syscalls.su ./Core/Src/platform/sysmem.cyclo ./Core/Src/platform/sysmem.d ./Core/Src/platform/sysmem.o ./Core/Src/platform/sysmem.su

.PHONY: clean-Core-2f-Src-2f-platform

