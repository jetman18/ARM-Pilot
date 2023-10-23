################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Lib/gps.c \
../Core/Lib/imu.c \
../Core/Lib/log.c \
../Core/Lib/maths.c \
../Core/Lib/pid.c \
../Core/Lib/ppmreceiver.c \
../Core/Lib/pwmwrite.c \
../Core/Lib/timer.c 

OBJS += \
./Core/Lib/gps.o \
./Core/Lib/imu.o \
./Core/Lib/log.o \
./Core/Lib/maths.o \
./Core/Lib/pid.o \
./Core/Lib/ppmreceiver.o \
./Core/Lib/pwmwrite.o \
./Core/Lib/timer.o 

C_DEPS += \
./Core/Lib/gps.d \
./Core/Lib/imu.d \
./Core/Lib/log.d \
./Core/Lib/maths.d \
./Core/Lib/pid.d \
./Core/Lib/ppmreceiver.d \
./Core/Lib/pwmwrite.d \
./Core/Lib/timer.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Lib/%.o Core/Lib/%.su: ../Core/Lib/%.c Core/Lib/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m3 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F103xB -c -I../Core/Inc -I../Drivers/STM32F1xx_HAL_Driver/Inc/Legacy -I../Drivers/STM32F1xx_HAL_Driver/Inc -I../Drivers/CMSIS/Device/ST/STM32F1xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-Core-2f-Lib

clean-Core-2f-Lib:
	-$(RM) ./Core/Lib/gps.d ./Core/Lib/gps.o ./Core/Lib/gps.su ./Core/Lib/imu.d ./Core/Lib/imu.o ./Core/Lib/imu.su ./Core/Lib/log.d ./Core/Lib/log.o ./Core/Lib/log.su ./Core/Lib/maths.d ./Core/Lib/maths.o ./Core/Lib/maths.su ./Core/Lib/pid.d ./Core/Lib/pid.o ./Core/Lib/pid.su ./Core/Lib/ppmreceiver.d ./Core/Lib/ppmreceiver.o ./Core/Lib/ppmreceiver.su ./Core/Lib/pwmwrite.d ./Core/Lib/pwmwrite.o ./Core/Lib/pwmwrite.su ./Core/Lib/timer.d ./Core/Lib/timer.o ./Core/Lib/timer.su

.PHONY: clean-Core-2f-Lib

