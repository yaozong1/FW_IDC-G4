################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../TM1638/TM1638.c \
../TM1638/TM1638_platform.c 

OBJS += \
./TM1638/TM1638.o \
./TM1638/TM1638_platform.o 

C_DEPS += \
./TM1638/TM1638.d \
./TM1638/TM1638_platform.d 


# Each subdirectory must supply rules for building sources it contributes
TM1638/%.o TM1638/%.su TM1638/%.cyclo: ../TM1638/%.c TM1638/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m0 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F030x8 -c -I../Core/Inc -I../Drivers/STM32F0xx_HAL_Driver/Inc -I../Drivers/STM32F0xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F0xx/Include -I../Drivers/CMSIS/Include -I"C:/Users/h1576/Desktop/Burn/IDC-G4_FW/FW_IDC-G4/IDC-G4-1/RTT" -I"C:/Users/h1576/Desktop/Burn/IDC-G4_FW/FW_IDC-G4/IDC-G4-1/TM1638" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-TM1638

clean-TM1638:
	-$(RM) ./TM1638/TM1638.cyclo ./TM1638/TM1638.d ./TM1638/TM1638.o ./TM1638/TM1638.su ./TM1638/TM1638_platform.cyclo ./TM1638/TM1638_platform.d ./TM1638/TM1638_platform.o ./TM1638/TM1638_platform.su

.PHONY: clean-TM1638

