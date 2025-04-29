################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Lib/Motori/motori.c 

OBJS += \
./Core/Lib/Motori/motori.o 

C_DEPS += \
./Core/Lib/Motori/motori.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Lib/Motori/%.o Core/Lib/Motori/%.su Core/Lib/Motori/%.cyclo: ../Core/Lib/Motori/%.c Core/Lib/Motori/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DSTM32L432xx -DUSE_FULL_LL_DRIVER -DUSE_HAL_DRIVER -c -I../Core/Inc -I../Drivers/STM32L4xx_HAL_Driver/Inc -I../Drivers/CMSIS/Device/ST/STM32L4xx/Include -I../Drivers/CMSIS/Include -I../Drivers/STM32L4xx_HAL_Driver/Inc/Legacy -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-Lib-2f-Motori

clean-Core-2f-Lib-2f-Motori:
	-$(RM) ./Core/Lib/Motori/motori.cyclo ./Core/Lib/Motori/motori.d ./Core/Lib/Motori/motori.o ./Core/Lib/Motori/motori.su

.PHONY: clean-Core-2f-Lib-2f-Motori

