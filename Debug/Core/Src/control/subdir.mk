################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (14.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Src/control/flight_control.c \
../Core/Src/control/pid.c 

OBJS += \
./Core/Src/control/flight_control.o \
./Core/Src/control/pid.o 

C_DEPS += \
./Core/Src/control/flight_control.d \
./Core/Src/control/pid.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Src/control/%.o Core/Src/control/%.su Core/Src/control/%.cyclo: ../Core/Src/control/%.c Core/Src/control/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F411xE -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I../Middlewares/ST/ARM/DSP/Inc -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-Src-2f-control

clean-Core-2f-Src-2f-control:
	-$(RM) ./Core/Src/control/flight_control.cyclo ./Core/Src/control/flight_control.d ./Core/Src/control/flight_control.o ./Core/Src/control/flight_control.su ./Core/Src/control/pid.cyclo ./Core/Src/control/pid.d ./Core/Src/control/pid.o ./Core/Src/control/pid.su

.PHONY: clean-Core-2f-Src-2f-control

