################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Drivers/BSP/custom/custom.c 

OBJS += \
./Drivers/BSP/custom/custom.o 

C_DEPS += \
./Drivers/BSP/custom/custom.d 


# Each subdirectory must supply rules for building sources it contributes
Drivers/BSP/custom/%.o Drivers/BSP/custom/%.su Drivers/BSP/custom/%.cyclo: ../Drivers/BSP/custom/%.c Drivers/BSP/custom/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m33 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32U575xx -c -I../Core/Inc -I../Drivers/STM32U5xx_HAL_Driver/Inc -I../Drivers/STM32U5xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32U5xx/Include -I../Drivers/CMSIS/Include -I../Middlewares/ST/BlueNRG-2/hci/hci_tl_patterns/Basic -I../Middlewares/ST/BlueNRG-2/utils -I../Middlewares/ST/BlueNRG-2/includes -I../BlueNRG_2/Target -I../BlueNRG_2/App -I../Drivers/BSP/custom -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Drivers-2f-BSP-2f-custom

clean-Drivers-2f-BSP-2f-custom:
	-$(RM) ./Drivers/BSP/custom/custom.cyclo ./Drivers/BSP/custom/custom.d ./Drivers/BSP/custom/custom.o ./Drivers/BSP/custom/custom.su

.PHONY: clean-Drivers-2f-BSP-2f-custom

