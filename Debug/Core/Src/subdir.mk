################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Src/adc.c \
../Core/Src/algorithms.c \
../Core/Src/dataPacking.c \
../Core/Src/dma.c \
../Core/Src/e22_lib.c \
../Core/Src/gpio.c \
../Core/Src/i2c.c \
../Core/Src/main.c \
../Core/Src/pin_timing.c \
../Core/Src/queternion.c \
../Core/Src/rtc.c \
../Core/Src/spi.c \
../Core/Src/stm32f4xx_hal_msp.c \
../Core/Src/stm32f4xx_it.c \
../Core/Src/syscalls.c \
../Core/Src/sysmem.c \
../Core/Src/system_stm32f4xx.c \
../Core/Src/tim.c \
../Core/Src/ukb_test.c \
../Core/Src/usart.c \
../Core/Src/usr_gnss_l86_parser.c 

OBJS += \
./Core/Src/adc.o \
./Core/Src/algorithms.o \
./Core/Src/dataPacking.o \
./Core/Src/dma.o \
./Core/Src/e22_lib.o \
./Core/Src/gpio.o \
./Core/Src/i2c.o \
./Core/Src/main.o \
./Core/Src/pin_timing.o \
./Core/Src/queternion.o \
./Core/Src/rtc.o \
./Core/Src/spi.o \
./Core/Src/stm32f4xx_hal_msp.o \
./Core/Src/stm32f4xx_it.o \
./Core/Src/syscalls.o \
./Core/Src/sysmem.o \
./Core/Src/system_stm32f4xx.o \
./Core/Src/tim.o \
./Core/Src/ukb_test.o \
./Core/Src/usart.o \
./Core/Src/usr_gnss_l86_parser.o 

C_DEPS += \
./Core/Src/adc.d \
./Core/Src/algorithms.d \
./Core/Src/dataPacking.d \
./Core/Src/dma.d \
./Core/Src/e22_lib.d \
./Core/Src/gpio.d \
./Core/Src/i2c.d \
./Core/Src/main.d \
./Core/Src/pin_timing.d \
./Core/Src/queternion.d \
./Core/Src/rtc.d \
./Core/Src/spi.d \
./Core/Src/stm32f4xx_hal_msp.d \
./Core/Src/stm32f4xx_it.d \
./Core/Src/syscalls.d \
./Core/Src/sysmem.d \
./Core/Src/system_stm32f4xx.d \
./Core/Src/tim.d \
./Core/Src/ukb_test.d \
./Core/Src/usart.d \
./Core/Src/usr_gnss_l86_parser.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Src/%.o Core/Src/%.su Core/Src/%.cyclo: ../Core/Src/%.c Core/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F446xx -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I"D:/st_kutuphaneler/bmi_088_library/bmi088" -I"D:/st_kutuphaneler/bme_280_library/bme_280" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-Src

clean-Core-2f-Src:
	-$(RM) ./Core/Src/adc.cyclo ./Core/Src/adc.d ./Core/Src/adc.o ./Core/Src/adc.su ./Core/Src/algorithms.cyclo ./Core/Src/algorithms.d ./Core/Src/algorithms.o ./Core/Src/algorithms.su ./Core/Src/dataPacking.cyclo ./Core/Src/dataPacking.d ./Core/Src/dataPacking.o ./Core/Src/dataPacking.su ./Core/Src/dma.cyclo ./Core/Src/dma.d ./Core/Src/dma.o ./Core/Src/dma.su ./Core/Src/e22_lib.cyclo ./Core/Src/e22_lib.d ./Core/Src/e22_lib.o ./Core/Src/e22_lib.su ./Core/Src/gpio.cyclo ./Core/Src/gpio.d ./Core/Src/gpio.o ./Core/Src/gpio.su ./Core/Src/i2c.cyclo ./Core/Src/i2c.d ./Core/Src/i2c.o ./Core/Src/i2c.su ./Core/Src/main.cyclo ./Core/Src/main.d ./Core/Src/main.o ./Core/Src/main.su ./Core/Src/pin_timing.cyclo ./Core/Src/pin_timing.d ./Core/Src/pin_timing.o ./Core/Src/pin_timing.su ./Core/Src/queternion.cyclo ./Core/Src/queternion.d ./Core/Src/queternion.o ./Core/Src/queternion.su ./Core/Src/rtc.cyclo ./Core/Src/rtc.d ./Core/Src/rtc.o ./Core/Src/rtc.su ./Core/Src/spi.cyclo ./Core/Src/spi.d ./Core/Src/spi.o ./Core/Src/spi.su ./Core/Src/stm32f4xx_hal_msp.cyclo ./Core/Src/stm32f4xx_hal_msp.d ./Core/Src/stm32f4xx_hal_msp.o ./Core/Src/stm32f4xx_hal_msp.su ./Core/Src/stm32f4xx_it.cyclo ./Core/Src/stm32f4xx_it.d ./Core/Src/stm32f4xx_it.o ./Core/Src/stm32f4xx_it.su ./Core/Src/syscalls.cyclo ./Core/Src/syscalls.d ./Core/Src/syscalls.o ./Core/Src/syscalls.su ./Core/Src/sysmem.cyclo ./Core/Src/sysmem.d ./Core/Src/sysmem.o ./Core/Src/sysmem.su ./Core/Src/system_stm32f4xx.cyclo ./Core/Src/system_stm32f4xx.d ./Core/Src/system_stm32f4xx.o ./Core/Src/system_stm32f4xx.su ./Core/Src/tim.cyclo ./Core/Src/tim.d ./Core/Src/tim.o ./Core/Src/tim.su ./Core/Src/ukb_test.cyclo ./Core/Src/ukb_test.d ./Core/Src/ukb_test.o ./Core/Src/ukb_test.su ./Core/Src/usart.cyclo ./Core/Src/usart.d ./Core/Src/usart.o ./Core/Src/usart.su ./Core/Src/usr_gnss_l86_parser.cyclo ./Core/Src/usr_gnss_l86_parser.d ./Core/Src/usr_gnss_l86_parser.o ./Core/Src/usr_gnss_l86_parser.su

.PHONY: clean-Core-2f-Src

