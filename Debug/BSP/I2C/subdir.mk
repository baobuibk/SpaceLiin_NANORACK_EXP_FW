################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../BSP/I2C/i2c.c 

OBJS += \
./BSP/I2C/i2c.o 

C_DEPS += \
./BSP/I2C/i2c.d 


# Each subdirectory must supply rules for building sources it contributes
BSP/I2C/%.o BSP/I2C/%.su BSP/I2C/%.cyclo: ../BSP/I2C/%.c BSP/I2C/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DSTM32F407xx -DUSE_FULL_LL_DRIVER -DHSE_VALUE=8000000 -DHSE_STARTUP_TIMEOUT=100 -DLSE_STARTUP_TIMEOUT=5000 -DLSE_VALUE=32768 -DEXTERNAL_CLOCK_VALUE=12288000 -DHSI_VALUE=16000000 -DLSI_VALUE=32000 -DVDD_VALUE=3300 -DPREFETCH_ENABLE=1 -DINSTRUCTION_CACHE_ENABLE=1 -DDATA_CACHE_ENABLE=1 -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I"D:/STworkspace/exp_test/Scheduler" -I"D:/STworkspace/exp_test/BSP" -I"D:/STworkspace/exp_test/BSP/SysTick" -I"D:/STworkspace/exp_test/Core/Common" -I"D:/STworkspace/exp_test/Core/WDog" -I"D:/STworkspace/exp_test/Core/LED" -I"D:/STworkspace/exp_test/BSP/UART" -I"D:/STworkspace/exp_test/Core/CMDLine" -I"D:/STworkspace/exp_test/Core/Board" -I"D:/STworkspace/exp_test/BSP/Delay" -I"D:/STworkspace/exp_test/Core/Devices" -I"D:/STworkspace/exp_test/Core/Devices/LT8722" -I"D:/STworkspace/exp_test/BSP/I2C" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-BSP-2f-I2C

clean-BSP-2f-I2C:
	-$(RM) ./BSP/I2C/i2c.cyclo ./BSP/I2C/i2c.d ./BSP/I2C/i2c.o ./BSP/I2C/i2c.su

.PHONY: clean-BSP-2f-I2C
