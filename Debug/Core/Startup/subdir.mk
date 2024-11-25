################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
S_SRCS += \
../Core/Startup/startup_stm32f407vgtx.s 

OBJS += \
./Core/Startup/startup_stm32f407vgtx.o 

S_DEPS += \
./Core/Startup/startup_stm32f407vgtx.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Startup/%.o: ../Core/Startup/%.s Core/Startup/subdir.mk
	arm-none-eabi-gcc -mcpu=cortex-m4 -g3 -DDEBUG -c -I"D:/STworkspace/exp_test/Scheduler" -I"D:/STworkspace/exp_test/BSP" -I"D:/STworkspace/exp_test/BSP/SysTick" -I"D:/STworkspace/exp_test/Core/Common" -I"D:/STworkspace/exp_test/Core/WDog" -I"D:/STworkspace/exp_test/Core/LED" -I"D:/STworkspace/exp_test/BSP/UART" -I"D:/STworkspace/exp_test/Core/CMDLine" -I"D:/STworkspace/exp_test/Core/Board" -I"D:/STworkspace/exp_test/Core/Temperature" -I"D:/STworkspace/exp_test/BSP/Delay" -x assembler-with-cpp -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@" "$<"

clean: clean-Core-2f-Startup

clean-Core-2f-Startup:
	-$(RM) ./Core/Startup/startup_stm32f407vgtx.d ./Core/Startup/startup_stm32f407vgtx.o

.PHONY: clean-Core-2f-Startup

