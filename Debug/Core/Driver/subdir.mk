################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Driver/bmp280.c \
../Core/Driver/ibus.c \
../Core/Driver/mpu6500.c \
../Core/Driver/ms5611.c \
../Core/Driver/nrf24l01p.c \
../Core/Driver/qmc5883.c 

OBJS += \
./Core/Driver/bmp280.o \
./Core/Driver/ibus.o \
./Core/Driver/mpu6500.o \
./Core/Driver/ms5611.o \
./Core/Driver/nrf24l01p.o \
./Core/Driver/qmc5883.o 

C_DEPS += \
./Core/Driver/bmp280.d \
./Core/Driver/ibus.d \
./Core/Driver/mpu6500.d \
./Core/Driver/ms5611.d \
./Core/Driver/nrf24l01p.d \
./Core/Driver/qmc5883.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Driver/%.o Core/Driver/%.su: ../Core/Driver/%.c Core/Driver/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m3 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F103xB -c -I../Core/Inc -I../Drivers/STM32F1xx_HAL_Driver/Inc/Legacy -I../Drivers/STM32F1xx_HAL_Driver/Inc -I../Drivers/CMSIS/Device/ST/STM32F1xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-Core-2f-Driver

clean-Core-2f-Driver:
	-$(RM) ./Core/Driver/bmp280.d ./Core/Driver/bmp280.o ./Core/Driver/bmp280.su ./Core/Driver/ibus.d ./Core/Driver/ibus.o ./Core/Driver/ibus.su ./Core/Driver/mpu6500.d ./Core/Driver/mpu6500.o ./Core/Driver/mpu6500.su ./Core/Driver/ms5611.d ./Core/Driver/ms5611.o ./Core/Driver/ms5611.su ./Core/Driver/nrf24l01p.d ./Core/Driver/nrf24l01p.o ./Core/Driver/nrf24l01p.su ./Core/Driver/qmc5883.d ./Core/Driver/qmc5883.o ./Core/Driver/qmc5883.su

.PHONY: clean-Core-2f-Driver

