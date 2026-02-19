################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Drivers/BSP/STM32H7xx_Nucleo/stm32h7xx_nucleo.c 

OBJS += \
./Drivers/BSP/STM32H7xx_Nucleo/stm32h7xx_nucleo.o 

C_DEPS += \
./Drivers/BSP/STM32H7xx_Nucleo/stm32h7xx_nucleo.d 


# Each subdirectory must supply rules for building sources it contributes
Drivers/BSP/STM32H7xx_Nucleo/%.o Drivers/BSP/STM32H7xx_Nucleo/%.su Drivers/BSP/STM32H7xx_Nucleo/%.cyclo: ../Drivers/BSP/STM32H7xx_Nucleo/%.c Drivers/BSP/STM32H7xx_Nucleo/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m7 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32H723xx -DSTM32 -DSTM32H7SINGLE -DNUCLEO_H723ZG -DSTM32H723ZGTx -DSTM32H7 -c -I../Inc -I"D:/STM32CubeIDE/workspace_2.0.0/elec391-robo-maestro/Core/Inc" -I"D:/STM32CubeIDE/workspace_2.0.0/elec391-robo-maestro/Drivers/CMSIS/Device/ST/STM32H7xx/Include" -I"D:/STM32CubeIDE/workspace_2.0.0/elec391-robo-maestro/Drivers/STM32H7xx_HAL_Driver/Inc" -I"D:/STM32CubeIDE/workspace_2.0.0/elec391-robo-maestro/Drivers/BSP/STM32H7xx_Nucleo" -I"D:/STM32CubeIDE/workspace_2.0.0/elec391-robo-maestro/Drivers/CMSIS/Core/Include" -I"D:/STM32CubeIDE/workspace_2.0.0/elec391-robo-maestro/Drivers/CMSIS/Core_A/Include" -I"D:/STM32CubeIDE/workspace_2.0.0/elec391-robo-maestro/Drivers/CMSIS/Include" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Drivers-2f-BSP-2f-STM32H7xx_Nucleo

clean-Drivers-2f-BSP-2f-STM32H7xx_Nucleo:
	-$(RM) ./Drivers/BSP/STM32H7xx_Nucleo/stm32h7xx_nucleo.cyclo ./Drivers/BSP/STM32H7xx_Nucleo/stm32h7xx_nucleo.d ./Drivers/BSP/STM32H7xx_Nucleo/stm32h7xx_nucleo.o ./Drivers/BSP/STM32H7xx_Nucleo/stm32h7xx_nucleo.su

.PHONY: clean-Drivers-2f-BSP-2f-STM32H7xx_Nucleo

