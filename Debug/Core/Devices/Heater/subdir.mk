################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Devices/Heater/heater.c 

OBJS += \
./Core/Devices/Heater/heater.o 

C_DEPS += \
./Core/Devices/Heater/heater.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Devices/Heater/%.o Core/Devices/Heater/%.su Core/Devices/Heater/%.cyclo: ../Core/Devices/Heater/%.c Core/Devices/Heater/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DSTM32F407xx -DUSE_FULL_LL_DRIVER -DHSE_VALUE=8000000 -DHSE_STARTUP_TIMEOUT=100 -DLSE_STARTUP_TIMEOUT=5000 -DLSE_VALUE=32768 -DEXTERNAL_CLOCK_VALUE=12288000 -DHSI_VALUE=16000000 -DLSI_VALUE=32000 -DVDD_VALUE=3300 -DPREFETCH_ENABLE=1 -DINSTRUCTION_CACHE_ENABLE=1 -DDATA_CACHE_ENABLE=1 -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I"D:/STworkspace/exp_test/Scheduler" -I"D:/STworkspace/exp_test/BSP" -I"D:/STworkspace/exp_test/BSP/SysTick" -I"D:/STworkspace/exp_test/Core/Common" -I"D:/STworkspace/exp_test/Core/WDog" -I"D:/STworkspace/exp_test/Core/LED" -I"D:/STworkspace/exp_test/BSP/UART" -I"D:/STworkspace/exp_test/Core/CMDLine" -I"D:/STworkspace/exp_test/Core/Board" -I"D:/STworkspace/exp_test/BSP/Delay" -I"D:/STworkspace/exp_test/Core/Devices" -I"D:/STworkspace/exp_test/Core/Devices/LT8722" -I"D:/STworkspace/exp_test/BSP/I2C" -I"D:/STworkspace/exp_test/Core/Devices/NTC" -I"D:/STworkspace/exp_test/Core/Temperature" -I"D:/STworkspace/exp_test/Core/Devices/BMP390" -I"D:/STworkspace/exp_test/Core/Devices/Heater" -I"D:/STworkspace/exp_test/Core/COPC" -I"D:/STworkspace/exp_test/ThirdParty/libfsp" -I"D:/STworkspace/exp_test/ThirdParty" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-Devices-2f-Heater

clean-Core-2f-Devices-2f-Heater:
	-$(RM) ./Core/Devices/Heater/heater.cyclo ./Core/Devices/Heater/heater.d ./Core/Devices/Heater/heater.o ./Core/Devices/Heater/heater.su

.PHONY: clean-Core-2f-Devices-2f-Heater
