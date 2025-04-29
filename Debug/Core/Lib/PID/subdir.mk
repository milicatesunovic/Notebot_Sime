################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Lib/PID/pid.c 

OBJS += \
./Core/Lib/PID/pid.o 

C_DEPS += \
./Core/Lib/PID/pid.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Lib/PID/%.o Core/Lib/PID/%.su Core/Lib/PID/%.cyclo: ../Core/Lib/PID/%.c Core/Lib/PID/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DSTM32L432xx -DUSE_FULL_LL_DRIVER -DUSE_HAL_DRIVER -c -I../Core/Inc -I../Drivers/STM32L4xx_HAL_Driver/Inc -I../Drivers/CMSIS/Device/ST/STM32L4xx/Include -I../Drivers/CMSIS/Include -I../Drivers/STM32L4xx_HAL_Driver/Inc/Legacy -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-Lib-2f-PID

clean-Core-2f-Lib-2f-PID:
	-$(RM) ./Core/Lib/PID/pid.cyclo ./Core/Lib/PID/pid.d ./Core/Lib/PID/pid.o ./Core/Lib/PID/pid.su

.PHONY: clean-Core-2f-Lib-2f-PID

