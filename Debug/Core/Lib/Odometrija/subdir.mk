################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Lib/Odometrija/odometrija.c 

OBJS += \
./Core/Lib/Odometrija/odometrija.o 

C_DEPS += \
./Core/Lib/Odometrija/odometrija.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Lib/Odometrija/%.o Core/Lib/Odometrija/%.su Core/Lib/Odometrija/%.cyclo: ../Core/Lib/Odometrija/%.c Core/Lib/Odometrija/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DSTM32L432xx -DUSE_FULL_LL_DRIVER -DUSE_HAL_DRIVER -c -I../Core/Inc -I../Drivers/STM32L4xx_HAL_Driver/Inc -I../Drivers/CMSIS/Device/ST/STM32L4xx/Include -I../Drivers/CMSIS/Include -I../Drivers/STM32L4xx_HAL_Driver/Inc/Legacy -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-Lib-2f-Odometrija

clean-Core-2f-Lib-2f-Odometrija:
	-$(RM) ./Core/Lib/Odometrija/odometrija.cyclo ./Core/Lib/Odometrija/odometrija.d ./Core/Lib/Odometrija/odometrija.o ./Core/Lib/Odometrija/odometrija.su

.PHONY: clean-Core-2f-Lib-2f-Odometrija

