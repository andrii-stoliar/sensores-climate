################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Sensors_i2c/HTS221.c \
../Sensors_i2c/LPS25HB.c 

OBJS += \
./Sensors_i2c/HTS221.o \
./Sensors_i2c/LPS25HB.o 

C_DEPS += \
./Sensors_i2c/HTS221.d \
./Sensors_i2c/LPS25HB.d 


# Each subdirectory must supply rules for building sources it contributes
Sensors_i2c/%.o Sensors_i2c/%.su Sensors_i2c/%.cyclo: ../Sensors_i2c/%.c Sensors_i2c/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DSTM32F303x8 -DUSE_FULL_LL_DRIVER -DHSE_VALUE=8000000 -DHSE_STARTUP_TIMEOUT=100 -DLSE_STARTUP_TIMEOUT=5000 -DLSE_VALUE=32768 -DEXTERNAL_CLOCK_VALUE=8000000 -DHSI_VALUE=8000000 -DLSI_VALUE=40000 -DVDD_VALUE=3300 -DPREFETCH_ENABLE=1 -c -I../Core/Inc -I../Drivers/STM32F3xx_HAL_Driver/Inc -I../Drivers/CMSIS/Device/ST/STM32F3xx/Include -I../Drivers/CMSIS/Include -I"C:/MyFiles/Study/7th_sem/VRS/Workspace/sensores-climate/Sensors_i2c" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Sensors_i2c

clean-Sensors_i2c:
	-$(RM) ./Sensors_i2c/HTS221.cyclo ./Sensors_i2c/HTS221.d ./Sensors_i2c/HTS221.o ./Sensors_i2c/HTS221.su ./Sensors_i2c/LPS25HB.cyclo ./Sensors_i2c/LPS25HB.d ./Sensors_i2c/LPS25HB.o ./Sensors_i2c/LPS25HB.su

.PHONY: clean-Sensors_i2c

