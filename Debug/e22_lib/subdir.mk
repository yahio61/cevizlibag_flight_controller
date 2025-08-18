################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
/home/yahya/STM32CubeIDE/workspace_1.19.0/E22_library/e22_lib/e22_lib.c 

OBJS += \
./e22_lib/e22_lib.o 

C_DEPS += \
./e22_lib/e22_lib.d 


# Each subdirectory must supply rules for building sources it contributes
e22_lib/e22_lib.o: /home/yahya/STM32CubeIDE/workspace_1.19.0/E22_library/e22_lib/e22_lib.c e22_lib/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F446xx -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I/home/yahya/STM32CubeIDE/workspace_1.19.0/bmi_088_library/bmi088 -I/home/yahya/STM32CubeIDE/workspace_1.19.0/bme_280_library/bme_280 -I/home/yahya/STM32CubeIDE/workspace_1.19.0/E22_library/e22_lib -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-e22_lib

clean-e22_lib:
	-$(RM) ./e22_lib/e22_lib.cyclo ./e22_lib/e22_lib.d ./e22_lib/e22_lib.o ./e22_lib/e22_lib.su

.PHONY: clean-e22_lib

