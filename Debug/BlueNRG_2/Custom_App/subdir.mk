################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../BlueNRG_2/Custom_App/app_bluenrg_2.c \
../BlueNRG_2/Custom_App/gatt_db.c \
../BlueNRG_2/Custom_App/sensor.c 

OBJS += \
./BlueNRG_2/Custom_App/app_bluenrg_2.o \
./BlueNRG_2/Custom_App/gatt_db.o \
./BlueNRG_2/Custom_App/sensor.o 

C_DEPS += \
./BlueNRG_2/Custom_App/app_bluenrg_2.d \
./BlueNRG_2/Custom_App/gatt_db.d \
./BlueNRG_2/Custom_App/sensor.d 


# Each subdirectory must supply rules for building sources it contributes
BlueNRG_2/Custom_App/%.o BlueNRG_2/Custom_App/%.su BlueNRG_2/Custom_App/%.cyclo: ../BlueNRG_2/Custom_App/%.c BlueNRG_2/Custom_App/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m33 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32U575xx -c -I../Core/Inc -I../Drivers/STM32U5xx_HAL_Driver/Inc -I../Drivers/STM32U5xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32U5xx/Include -I../Drivers/CMSIS/Include -I../Middlewares/ST/BlueNRG-2/hci/hci_tl_patterns/Basic -I../Middlewares/ST/BlueNRG-2/utils -I../Middlewares/ST/BlueNRG-2/includes -I../BlueNRG_2/Target -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-BlueNRG_2-2f-Custom_App

clean-BlueNRG_2-2f-Custom_App:
	-$(RM) ./BlueNRG_2/Custom_App/app_bluenrg_2.cyclo ./BlueNRG_2/Custom_App/app_bluenrg_2.d ./BlueNRG_2/Custom_App/app_bluenrg_2.o ./BlueNRG_2/Custom_App/app_bluenrg_2.su ./BlueNRG_2/Custom_App/gatt_db.cyclo ./BlueNRG_2/Custom_App/gatt_db.d ./BlueNRG_2/Custom_App/gatt_db.o ./BlueNRG_2/Custom_App/gatt_db.su ./BlueNRG_2/Custom_App/sensor.cyclo ./BlueNRG_2/Custom_App/sensor.d ./BlueNRG_2/Custom_App/sensor.o ./BlueNRG_2/Custom_App/sensor.su

.PHONY: clean-BlueNRG_2-2f-Custom_App

