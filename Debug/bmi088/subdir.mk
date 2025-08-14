################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
D:/st_kutuphaneler/bmi_088_library/bmi088/bmi088.c 

OBJS += \
./bmi088/bmi088.o 

C_DEPS += \
./bmi088/bmi088.d 


# Each subdirectory must supply rules for building sources it contributes
bmi088/bmi088.o: D:/st_kutuphaneler/bmi_088_library/bmi088/bmi088.c bmi088/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F446xx -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I"D:/st_kutuphaneler/bmi_088_library/bmi088" -I"D:/st_kutuphaneler/bme_280_library/bme_280" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-bmi088

clean-bmi088:
	-$(RM) ./bmi088/bmi088.cyclo ./bmi088/bmi088.d ./bmi088/bmi088.o ./bmi088/bmi088.su

.PHONY: clean-bmi088

