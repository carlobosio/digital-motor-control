################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Src/ethernetif.c \
../Src/freertos.c \
../Src/lwip.c \
../Src/main.c \
../Src/stm32f7xx_hal_msp.c \
../Src/stm32f7xx_hal_timebase_tim.c \
../Src/stm32f7xx_it.c \
../Src/syscalls.c \
../Src/system_stm32f7xx.c 

OBJS += \
./Src/ethernetif.o \
./Src/freertos.o \
./Src/lwip.o \
./Src/main.o \
./Src/stm32f7xx_hal_msp.o \
./Src/stm32f7xx_hal_timebase_tim.o \
./Src/stm32f7xx_it.o \
./Src/syscalls.o \
./Src/system_stm32f7xx.o 

C_DEPS += \
./Src/ethernetif.d \
./Src/freertos.d \
./Src/lwip.d \
./Src/main.d \
./Src/stm32f7xx_hal_msp.d \
./Src/stm32f7xx_hal_timebase_tim.d \
./Src/stm32f7xx_it.d \
./Src/syscalls.d \
./Src/system_stm32f7xx.d 


# Each subdirectory must supply rules for building sources it contributes
Src/%.o: ../Src/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: MCU GCC Compiler'
	@echo $(PWD)
	arm-none-eabi-gcc -mcpu=cortex-m7 -mthumb -mfloat-abi=hard -mfpu=fpv5-sp-d16 -DUSE_HAL_DRIVER -DSTM32F746xx -I"C:/Users/carlo/workspace/F746ZG_someexperiments/Inc" -I"C:/Users/carlo/workspace/F746ZG_someexperiments/Middlewares/Third_Party/LwIP/src/include" -I"C:/Users/carlo/workspace/F746ZG_someexperiments/Middlewares/Third_Party/LwIP/system" -I"C:/Users/carlo/workspace/F746ZG_someexperiments/Drivers/STM32F7xx_HAL_Driver/Inc" -I"C:/Users/carlo/workspace/F746ZG_someexperiments/Drivers/STM32F7xx_HAL_Driver/Inc/Legacy" -I"C:/Users/carlo/workspace/F746ZG_someexperiments/Middlewares/Third_Party/FreeRTOS/Source/include" -I"C:/Users/carlo/workspace/F746ZG_someexperiments/Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2" -I"C:/Users/carlo/workspace/F746ZG_someexperiments/Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM7/r0p1" -I"C:/Users/carlo/workspace/F746ZG_someexperiments/Middlewares/Third_Party/LwIP/src/include/netif/ppp" -I"C:/Users/carlo/workspace/F746ZG_someexperiments/Drivers/CMSIS/Device/ST/STM32F7xx/Include" -I"C:/Users/carlo/workspace/F746ZG_someexperiments/Middlewares/Third_Party/LwIP/src/include/lwip" -I"C:/Users/carlo/workspace/F746ZG_someexperiments/Middlewares/Third_Party/LwIP/src/include/lwip/apps" -I"C:/Users/carlo/workspace/F746ZG_someexperiments/Middlewares/Third_Party/LwIP/src/include/lwip/priv" -I"C:/Users/carlo/workspace/F746ZG_someexperiments/Middlewares/Third_Party/LwIP/src/include/lwip/prot" -I"C:/Users/carlo/workspace/F746ZG_someexperiments/Middlewares/Third_Party/LwIP/src/include/netif" -I"C:/Users/carlo/workspace/F746ZG_someexperiments/Middlewares/Third_Party/LwIP/src/include/compat/posix" -I"C:/Users/carlo/workspace/F746ZG_someexperiments/Middlewares/Third_Party/LwIP/src/include/compat/posix/arpa" -I"C:/Users/carlo/workspace/F746ZG_someexperiments/Middlewares/Third_Party/LwIP/src/include/compat/posix/net" -I"C:/Users/carlo/workspace/F746ZG_someexperiments/Middlewares/Third_Party/LwIP/src/include/compat/posix/sys" -I"C:/Users/carlo/workspace/F746ZG_someexperiments/Middlewares/Third_Party/LwIP/src/include/compat/stdc" -I"C:/Users/carlo/workspace/F746ZG_someexperiments/Middlewares/Third_Party/LwIP/system/arch" -I"C:/Users/carlo/workspace/F746ZG_someexperiments/Drivers/CMSIS/Include"  -Og -g3 -Wall -fmessage-length=0 -ffunction-sections -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


