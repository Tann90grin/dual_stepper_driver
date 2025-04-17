################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../Core/TMC2209_Driver/TMC2209.cpp 

OBJS += \
./Core/TMC2209_Driver/TMC2209.o 

CPP_DEPS += \
./Core/TMC2209_Driver/TMC2209.d 


# Each subdirectory must supply rules for building sources it contributes
Core/TMC2209_Driver/%.o Core/TMC2209_Driver/%.su Core/TMC2209_Driver/%.cyclo: ../Core/TMC2209_Driver/%.cpp Core/TMC2209_Driver/subdir.mk
	arm-none-eabi-g++ "$<" -mcpu=cortex-m4 -std=gnu++14 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F411xE -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I../USB_DEVICE/App -I../USB_DEVICE/Target -I../Middlewares/ST/STM32_USB_Device_Library/Core/Inc -I../Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Inc -O0 -ffunction-sections -fdata-sections -fno-exceptions -fno-rtti -fno-use-cxa-atexit -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-TMC2209_Driver

clean-Core-2f-TMC2209_Driver:
	-$(RM) ./Core/TMC2209_Driver/TMC2209.cyclo ./Core/TMC2209_Driver/TMC2209.d ./Core/TMC2209_Driver/TMC2209.o ./Core/TMC2209_Driver/TMC2209.su

.PHONY: clean-Core-2f-TMC2209_Driver

