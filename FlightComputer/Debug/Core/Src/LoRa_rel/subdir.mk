################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (11.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Src/LoRa_rel/sx127x_io.c 

OBJS += \
./Core/Src/LoRa_rel/sx127x_io.o 

C_DEPS += \
./Core/Src/LoRa_rel/sx127x_io.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Src/LoRa_rel/%.o Core/Src/LoRa_rel/%.su Core/Src/LoRa_rel/%.cyclo: ../Core/Src/LoRa_rel/%.c Core/Src/LoRa_rel/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F429xx -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I"C:/Users/liuWilly/source/repos/Asfoloth/FlightComputer/Core/Inc/IMU_rel" -I../Lora -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-Src-2f-LoRa_rel

clean-Core-2f-Src-2f-LoRa_rel:
	-$(RM) ./Core/Src/LoRa_rel/sx127x_io.cyclo ./Core/Src/LoRa_rel/sx127x_io.d ./Core/Src/LoRa_rel/sx127x_io.o ./Core/Src/LoRa_rel/sx127x_io.su

.PHONY: clean-Core-2f-Src-2f-LoRa_rel

