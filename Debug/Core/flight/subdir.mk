################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/flight/controller.c \
../Core/flight/estimate.c \
../Core/flight/fixedwing.c \
../Core/flight/mavlink_handler.c \
../Core/flight/navigation.c \
../Core/flight/scheduler.c 

OBJS += \
./Core/flight/controller.o \
./Core/flight/estimate.o \
./Core/flight/fixedwing.o \
./Core/flight/mavlink_handler.o \
./Core/flight/navigation.o \
./Core/flight/scheduler.o 

C_DEPS += \
./Core/flight/controller.d \
./Core/flight/estimate.d \
./Core/flight/fixedwing.d \
./Core/flight/mavlink_handler.d \
./Core/flight/navigation.d \
./Core/flight/scheduler.d 


# Each subdirectory must supply rules for building sources it contributes
Core/flight/%.o Core/flight/%.su: ../Core/flight/%.c Core/flight/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m3 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F103xB -c -I../Core/Inc -I../Drivers/STM32F1xx_HAL_Driver/Inc/Legacy -I../Drivers/STM32F1xx_HAL_Driver/Inc -I../Drivers/CMSIS/Device/ST/STM32F1xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-Core-2f-flight

clean-Core-2f-flight:
	-$(RM) ./Core/flight/controller.d ./Core/flight/controller.o ./Core/flight/controller.su ./Core/flight/estimate.d ./Core/flight/estimate.o ./Core/flight/estimate.su ./Core/flight/fixedwing.d ./Core/flight/fixedwing.o ./Core/flight/fixedwing.su ./Core/flight/mavlink_handler.d ./Core/flight/mavlink_handler.o ./Core/flight/mavlink_handler.su ./Core/flight/navigation.d ./Core/flight/navigation.o ./Core/flight/navigation.su ./Core/flight/scheduler.d ./Core/flight/scheduler.o ./Core/flight/scheduler.su

.PHONY: clean-Core-2f-flight

