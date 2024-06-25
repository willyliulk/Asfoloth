################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (11.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Src/IMU_rel/IMU_ALL.c \
../Core/Src/IMU_rel/float16Tool.c \
../Core/Src/IMU_rel/gnssPvtDataParser.c \
../Core/Src/IMU_rel/xsens_mdata2.c \
../Core/Src/IMU_rel/xsens_mti.c \
../Core/Src/IMU_rel/xsens_utility.c 

OBJS += \
./Core/Src/IMU_rel/IMU_ALL.o \
./Core/Src/IMU_rel/float16Tool.o \
./Core/Src/IMU_rel/gnssPvtDataParser.o \
./Core/Src/IMU_rel/xsens_mdata2.o \
./Core/Src/IMU_rel/xsens_mti.o \
./Core/Src/IMU_rel/xsens_utility.o 

C_DEPS += \
./Core/Src/IMU_rel/IMU_ALL.d \
./Core/Src/IMU_rel/float16Tool.d \
./Core/Src/IMU_rel/gnssPvtDataParser.d \
./Core/Src/IMU_rel/xsens_mdata2.d \
./Core/Src/IMU_rel/xsens_mti.d \
./Core/Src/IMU_rel/xsens_utility.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Src/IMU_rel/%.o Core/Src/IMU_rel/%.su Core/Src/IMU_rel/%.cyclo: ../Core/Src/IMU_rel/%.c Core/Src/IMU_rel/subdir.mk
<<<<<<< HEAD
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F429xx -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I"C:/Users/liuWilly/source/repos/Asfoloth/FlightComputer/Core/Inc/IMU_rel" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
=======
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F429xx -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I"C:/Users/liuWilly/source/repos/Asfoloth/FlightComputer/Core/Inc/IMU_rel" -I"C:/Users/liuWilly/source/repos/Asfoloth/FlightComputer/LoRa" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
>>>>>>> 5980df17532ad40811aa1e6ea6121fdbdf3ba488

clean: clean-Core-2f-Src-2f-IMU_rel

clean-Core-2f-Src-2f-IMU_rel:
	-$(RM) ./Core/Src/IMU_rel/IMU_ALL.cyclo ./Core/Src/IMU_rel/IMU_ALL.d ./Core/Src/IMU_rel/IMU_ALL.o ./Core/Src/IMU_rel/IMU_ALL.su ./Core/Src/IMU_rel/float16Tool.cyclo ./Core/Src/IMU_rel/float16Tool.d ./Core/Src/IMU_rel/float16Tool.o ./Core/Src/IMU_rel/float16Tool.su ./Core/Src/IMU_rel/gnssPvtDataParser.cyclo ./Core/Src/IMU_rel/gnssPvtDataParser.d ./Core/Src/IMU_rel/gnssPvtDataParser.o ./Core/Src/IMU_rel/gnssPvtDataParser.su ./Core/Src/IMU_rel/xsens_mdata2.cyclo ./Core/Src/IMU_rel/xsens_mdata2.d ./Core/Src/IMU_rel/xsens_mdata2.o ./Core/Src/IMU_rel/xsens_mdata2.su ./Core/Src/IMU_rel/xsens_mti.cyclo ./Core/Src/IMU_rel/xsens_mti.d ./Core/Src/IMU_rel/xsens_mti.o ./Core/Src/IMU_rel/xsens_mti.su ./Core/Src/IMU_rel/xsens_utility.cyclo ./Core/Src/IMU_rel/xsens_utility.d ./Core/Src/IMU_rel/xsens_utility.o ./Core/Src/IMU_rel/xsens_utility.su

.PHONY: clean-Core-2f-Src-2f-IMU_rel

