################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Middlewares/Third_Party/FreeRTOS/Source/croutine.c \
../Middlewares/Third_Party/FreeRTOS/Source/event_groups.c \
../Middlewares/Third_Party/FreeRTOS/Source/list.c \
../Middlewares/Third_Party/FreeRTOS/Source/queue.c \
../Middlewares/Third_Party/FreeRTOS/Source/stream_buffer.c \
../Middlewares/Third_Party/FreeRTOS/Source/tasks.c \
../Middlewares/Third_Party/FreeRTOS/Source/timers.c 

OBJS += \
./Middlewares/Third_Party/FreeRTOS/Source/croutine.o \
./Middlewares/Third_Party/FreeRTOS/Source/event_groups.o \
./Middlewares/Third_Party/FreeRTOS/Source/list.o \
./Middlewares/Third_Party/FreeRTOS/Source/queue.o \
./Middlewares/Third_Party/FreeRTOS/Source/stream_buffer.o \
./Middlewares/Third_Party/FreeRTOS/Source/tasks.o \
./Middlewares/Third_Party/FreeRTOS/Source/timers.o 

C_DEPS += \
./Middlewares/Third_Party/FreeRTOS/Source/croutine.d \
./Middlewares/Third_Party/FreeRTOS/Source/event_groups.d \
./Middlewares/Third_Party/FreeRTOS/Source/list.d \
./Middlewares/Third_Party/FreeRTOS/Source/queue.d \
./Middlewares/Third_Party/FreeRTOS/Source/stream_buffer.d \
./Middlewares/Third_Party/FreeRTOS/Source/tasks.d \
./Middlewares/Third_Party/FreeRTOS/Source/timers.d 


# Each subdirectory must supply rules for building sources it contributes
Middlewares/Third_Party/FreeRTOS/Source/%.o: ../Middlewares/Third_Party/FreeRTOS/Source/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: MCU GCC Compiler'
	@echo $(PWD)
	arm-none-eabi-gcc -mcpu=cortex-m7 -mthumb -mfloat-abi=hard -mfpu=fpv5-sp-d16 -DUSE_HAL_DRIVER -DSTM32F746xx -I"C:/Users/carlo/workspace/F746ZG_someexperiments/Inc" -I"C:/Users/carlo/workspace/F746ZG_someexperiments/Middlewares/Third_Party/LwIP/src/include" -I"C:/Users/carlo/workspace/F746ZG_someexperiments/Middlewares/Third_Party/LwIP/system" -I"C:/Users/carlo/workspace/F746ZG_someexperiments/Drivers/STM32F7xx_HAL_Driver/Inc" -I"C:/Users/carlo/workspace/F746ZG_someexperiments/Drivers/STM32F7xx_HAL_Driver/Inc/Legacy" -I"C:/Users/carlo/workspace/F746ZG_someexperiments/Middlewares/Third_Party/FreeRTOS/Source/include" -I"C:/Users/carlo/workspace/F746ZG_someexperiments/Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2" -I"C:/Users/carlo/workspace/F746ZG_someexperiments/Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM7/r0p1" -I"C:/Users/carlo/workspace/F746ZG_someexperiments/Middlewares/Third_Party/LwIP/src/include/netif/ppp" -I"C:/Users/carlo/workspace/F746ZG_someexperiments/Drivers/CMSIS/Device/ST/STM32F7xx/Include" -I"C:/Users/carlo/workspace/F746ZG_someexperiments/Middlewares/Third_Party/LwIP/src/include/lwip" -I"C:/Users/carlo/workspace/F746ZG_someexperiments/Middlewares/Third_Party/LwIP/src/include/lwip/apps" -I"C:/Users/carlo/workspace/F746ZG_someexperiments/Middlewares/Third_Party/LwIP/src/include/lwip/priv" -I"C:/Users/carlo/workspace/F746ZG_someexperiments/Middlewares/Third_Party/LwIP/src/include/lwip/prot" -I"C:/Users/carlo/workspace/F746ZG_someexperiments/Middlewares/Third_Party/LwIP/src/include/netif" -I"C:/Users/carlo/workspace/F746ZG_someexperiments/Middlewares/Third_Party/LwIP/src/include/compat/posix" -I"C:/Users/carlo/workspace/F746ZG_someexperiments/Middlewares/Third_Party/LwIP/src/include/compat/posix/arpa" -I"C:/Users/carlo/workspace/F746ZG_someexperiments/Middlewares/Third_Party/LwIP/src/include/compat/posix/net" -I"C:/Users/carlo/workspace/F746ZG_someexperiments/Middlewares/Third_Party/LwIP/src/include/compat/posix/sys" -I"C:/Users/carlo/workspace/F746ZG_someexperiments/Middlewares/Third_Party/LwIP/src/include/compat/stdc" -I"C:/Users/carlo/workspace/F746ZG_someexperiments/Middlewares/Third_Party/LwIP/system/arch" -I"C:/Users/carlo/workspace/F746ZG_someexperiments/Drivers/CMSIS/Include"  -Og -g3 -Wall -fmessage-length=0 -ffunction-sections -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


