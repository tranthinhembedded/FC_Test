################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (14.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Src/comm/pid_tuning.c \
../Core/Src/comm/telemetry.c 

OBJS += \
./Core/Src/comm/pid_tuning.o \
./Core/Src/comm/telemetry.o 

C_DEPS += \
./Core/Src/comm/pid_tuning.d \
./Core/Src/comm/telemetry.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Src/comm/%.o Core/Src/comm/%.su Core/Src/comm/%.cyclo: ../Core/Src/comm/%.c Core/Src/comm/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F411xE -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I../Middlewares/ST/ARM/DSP/Inc -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-Src-2f-comm

clean-Core-2f-Src-2f-comm:
	-$(RM) ./Core/Src/comm/pid_tuning.cyclo ./Core/Src/comm/pid_tuning.d ./Core/Src/comm/pid_tuning.o ./Core/Src/comm/pid_tuning.su ./Core/Src/comm/telemetry.cyclo ./Core/Src/comm/telemetry.d ./Core/Src/comm/telemetry.o ./Core/Src/comm/telemetry.su

.PHONY: clean-Core-2f-Src-2f-comm

