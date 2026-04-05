################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (14.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Src/sensor/complementary_filter.c \
../Core/Src/sensor/imu_config.c \
../Core/Src/sensor/mag_calibration.c \
../Core/Src/sensor/magnetometer_sensor.c \
../Core/Src/sensor/sensor_common.c 

OBJS += \
./Core/Src/sensor/complementary_filter.o \
./Core/Src/sensor/imu_config.o \
./Core/Src/sensor/mag_calibration.o \
./Core/Src/sensor/magnetometer_sensor.o \
./Core/Src/sensor/sensor_common.o 

C_DEPS += \
./Core/Src/sensor/complementary_filter.d \
./Core/Src/sensor/imu_config.d \
./Core/Src/sensor/mag_calibration.d \
./Core/Src/sensor/magnetometer_sensor.d \
./Core/Src/sensor/sensor_common.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Src/sensor/%.o Core/Src/sensor/%.su Core/Src/sensor/%.cyclo: ../Core/Src/sensor/%.c Core/Src/sensor/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F411xE -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I../Middlewares/ST/ARM/DSP/Inc -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-Src-2f-sensor

clean-Core-2f-Src-2f-sensor:
	-$(RM) ./Core/Src/sensor/complementary_filter.cyclo ./Core/Src/sensor/complementary_filter.d ./Core/Src/sensor/complementary_filter.o ./Core/Src/sensor/complementary_filter.su ./Core/Src/sensor/imu_config.cyclo ./Core/Src/sensor/imu_config.d ./Core/Src/sensor/imu_config.o ./Core/Src/sensor/imu_config.su ./Core/Src/sensor/mag_calibration.cyclo ./Core/Src/sensor/mag_calibration.d ./Core/Src/sensor/mag_calibration.o ./Core/Src/sensor/mag_calibration.su ./Core/Src/sensor/magnetometer_sensor.cyclo ./Core/Src/sensor/magnetometer_sensor.d ./Core/Src/sensor/magnetometer_sensor.o ./Core/Src/sensor/magnetometer_sensor.su ./Core/Src/sensor/sensor_common.cyclo ./Core/Src/sensor/sensor_common.d ./Core/Src/sensor/sensor_common.o ./Core/Src/sensor/sensor_common.su

.PHONY: clean-Core-2f-Src-2f-sensor

