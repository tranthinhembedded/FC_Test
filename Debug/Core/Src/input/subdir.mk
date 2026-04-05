################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (14.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Src/input/rc_input.c 

OBJS += \
./Core/Src/input/rc_input.o 

C_DEPS += \
./Core/Src/input/rc_input.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Src/input/%.o Core/Src/input/%.su Core/Src/input/%.cyclo: ../Core/Src/input/%.c Core/Src/input/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F411xE -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I../Middlewares/ST/ARM/DSP/Inc -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-Src-2f-input

clean-Core-2f-Src-2f-input:
	-$(RM) ./Core/Src/input/rc_input.cyclo ./Core/Src/input/rc_input.d ./Core/Src/input/rc_input.o ./Core/Src/input/rc_input.su

.PHONY: clean-Core-2f-Src-2f-input

