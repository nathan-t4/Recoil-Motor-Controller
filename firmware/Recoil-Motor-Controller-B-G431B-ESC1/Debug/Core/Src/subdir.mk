################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Src/app.c \
../Core/Src/can.c \
../Core/Src/current_controller.c \
../Core/Src/encoder.c \
../Core/Src/foc_math.c \
../Core/Src/main.c \
../Core/Src/math_ops.c \
../Core/Src/motor.c \
../Core/Src/motor_controller.c \
../Core/Src/position_controller.c \
../Core/Src/powerstage.c \
../Core/Src/stm32g4xx_hal_msp.c \
../Core/Src/stm32g4xx_it.c \
../Core/Src/syscalls.c \
../Core/Src/sysmem.c \
../Core/Src/system_stm32g4xx.c 

OBJS += \
./Core/Src/app.o \
./Core/Src/can.o \
./Core/Src/current_controller.o \
./Core/Src/encoder.o \
./Core/Src/foc_math.o \
./Core/Src/main.o \
./Core/Src/math_ops.o \
./Core/Src/motor.o \
./Core/Src/motor_controller.o \
./Core/Src/position_controller.o \
./Core/Src/powerstage.o \
./Core/Src/stm32g4xx_hal_msp.o \
./Core/Src/stm32g4xx_it.o \
./Core/Src/syscalls.o \
./Core/Src/sysmem.o \
./Core/Src/system_stm32g4xx.o 

C_DEPS += \
./Core/Src/app.d \
./Core/Src/can.d \
./Core/Src/current_controller.d \
./Core/Src/encoder.d \
./Core/Src/foc_math.d \
./Core/Src/main.d \
./Core/Src/math_ops.d \
./Core/Src/motor.d \
./Core/Src/motor_controller.d \
./Core/Src/position_controller.d \
./Core/Src/powerstage.d \
./Core/Src/stm32g4xx_hal_msp.d \
./Core/Src/stm32g4xx_it.d \
./Core/Src/syscalls.d \
./Core/Src/sysmem.d \
./Core/Src/system_stm32g4xx.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Src/%.o Core/Src/%.su: ../Core/Src/%.c Core/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32G431xx -c -I../Core/Inc -I../Drivers/STM32G4xx_HAL_Driver/Inc -I../Drivers/STM32G4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32G4xx/Include -I../Drivers/CMSIS/Include -I../Lib -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-Src

clean-Core-2f-Src:
	-$(RM) ./Core/Src/app.d ./Core/Src/app.o ./Core/Src/app.su ./Core/Src/can.d ./Core/Src/can.o ./Core/Src/can.su ./Core/Src/current_controller.d ./Core/Src/current_controller.o ./Core/Src/current_controller.su ./Core/Src/encoder.d ./Core/Src/encoder.o ./Core/Src/encoder.su ./Core/Src/foc_math.d ./Core/Src/foc_math.o ./Core/Src/foc_math.su ./Core/Src/main.d ./Core/Src/main.o ./Core/Src/main.su ./Core/Src/math_ops.d ./Core/Src/math_ops.o ./Core/Src/math_ops.su ./Core/Src/motor.d ./Core/Src/motor.o ./Core/Src/motor.su ./Core/Src/motor_controller.d ./Core/Src/motor_controller.o ./Core/Src/motor_controller.su ./Core/Src/position_controller.d ./Core/Src/position_controller.o ./Core/Src/position_controller.su ./Core/Src/powerstage.d ./Core/Src/powerstage.o ./Core/Src/powerstage.su ./Core/Src/stm32g4xx_hal_msp.d ./Core/Src/stm32g4xx_hal_msp.o ./Core/Src/stm32g4xx_hal_msp.su ./Core/Src/stm32g4xx_it.d ./Core/Src/stm32g4xx_it.o ./Core/Src/stm32g4xx_it.su ./Core/Src/syscalls.d ./Core/Src/syscalls.o ./Core/Src/syscalls.su ./Core/Src/sysmem.d ./Core/Src/sysmem.o ./Core/Src/sysmem.su ./Core/Src/system_stm32g4xx.d ./Core/Src/system_stm32g4xx.o ./Core/Src/system_stm32g4xx.su

.PHONY: clean-Core-2f-Src

