################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: local
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../drivers/Src/Legacy/stm32l4xx_hal_can.c 

OBJS += \
./drivers/Src/Legacy/stm32l4xx_hal_can.o 

C_DEPS += \
./drivers/Src/Legacy/stm32l4xx_hal_can.d 


# Each subdirectory must supply rules for building sources it contributes
drivers/Src/Legacy/%.o drivers/Src/Legacy/%.su drivers/Src/Legacy/%.cyclo: ../drivers/Src/Legacy/%.c drivers/Src/Legacy/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DSTM32L4 -DSTM32 -DB_L475E_IOT01A1 -DSTM32L475VGTx -c -I../Inc -I"/home/athul-muralidhar/Mastering-Microcontroller-and-Embedded-Driver-Development/stm32lxx_spi_slave/drivers/Inc" -I"/home/athul-muralidhar/Mastering-Microcontroller-and-Embedded-Driver-Development/stm32lxx_spi_slave/drivers/Src" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-drivers-2f-Src-2f-Legacy

clean-drivers-2f-Src-2f-Legacy:
	-$(RM) ./drivers/Src/Legacy/stm32l4xx_hal_can.cyclo ./drivers/Src/Legacy/stm32l4xx_hal_can.d ./drivers/Src/Legacy/stm32l4xx_hal_can.o ./drivers/Src/Legacy/stm32l4xx_hal_can.su

.PHONY: clean-drivers-2f-Src-2f-Legacy

