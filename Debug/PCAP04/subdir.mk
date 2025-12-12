################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../PCAP04/user_spi_interface.c 

OBJS += \
./PCAP04/user_spi_interface.o 

C_DEPS += \
./PCAP04/user_spi_interface.d 


# Each subdirectory must supply rules for building sources it contributes
PCAP04/%.o PCAP04/%.su PCAP04/%.cyclo: ../PCAP04/%.c PCAP04/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m33 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32U575xx -c -I../Core/Inc -I../Drivers/STM32U5xx_HAL_Driver/Inc -I../Drivers/STM32U5xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32U5xx/Include -I../Drivers/CMSIS/Include -I../Middlewares/ST/BlueNRG-2/hci/hci_tl_patterns/Basic -I../Middlewares/ST/BlueNRG-2/utils -I../Middlewares/ST/BlueNRG-2/includes -I../BlueNRG_2/Target -I../BlueNRG_2/Custom_App -I../PCAP04 -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-PCAP04

clean-PCAP04:
	-$(RM) ./PCAP04/user_spi_interface.cyclo ./PCAP04/user_spi_interface.d ./PCAP04/user_spi_interface.o ./PCAP04/user_spi_interface.su

.PHONY: clean-PCAP04

