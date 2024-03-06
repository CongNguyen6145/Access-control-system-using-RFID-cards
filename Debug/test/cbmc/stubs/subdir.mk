################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (11.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../test/cbmc/stubs/HTTPClient_ReadHeader_llhttp_execute.c \
../test/cbmc/stubs/HTTPClient_Send_llhttp_execute.c \
../test/cbmc/stubs/callback_stubs.c \
../test/cbmc/stubs/get_time_stub.c \
../test/cbmc/stubs/httpHeaderStrncpy.c \
../test/cbmc/stubs/memmove.c \
../test/cbmc/stubs/strncpy.c \
../test/cbmc/stubs/transport_interface_stubs.c 

OBJS += \
./test/cbmc/stubs/HTTPClient_ReadHeader_llhttp_execute.o \
./test/cbmc/stubs/HTTPClient_Send_llhttp_execute.o \
./test/cbmc/stubs/callback_stubs.o \
./test/cbmc/stubs/get_time_stub.o \
./test/cbmc/stubs/httpHeaderStrncpy.o \
./test/cbmc/stubs/memmove.o \
./test/cbmc/stubs/strncpy.o \
./test/cbmc/stubs/transport_interface_stubs.o 

C_DEPS += \
./test/cbmc/stubs/HTTPClient_ReadHeader_llhttp_execute.d \
./test/cbmc/stubs/HTTPClient_Send_llhttp_execute.d \
./test/cbmc/stubs/callback_stubs.d \
./test/cbmc/stubs/get_time_stub.d \
./test/cbmc/stubs/httpHeaderStrncpy.d \
./test/cbmc/stubs/memmove.d \
./test/cbmc/stubs/strncpy.d \
./test/cbmc/stubs/transport_interface_stubs.d 


# Each subdirectory must supply rules for building sources it contributes
test/cbmc/stubs/%.o test/cbmc/stubs/%.su test/cbmc/stubs/%.cyclo: ../test/cbmc/stubs/%.c test/cbmc/stubs/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F429xx -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I../Utils/ -I../Drivers/Device -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2 -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../LWIP/App -I../LWIP/Target -I"D:/myWorkspace/imic_stm32_demo final/imic_stm32_demo/Middlewares/Third_Party/LwIP/src/include" -I../Middlewares/Third_Party/LwIP/system -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -I"D:/myWorkspace/imic_stm32_demo final/imic_stm32_demo/coreHTTP/include" -I"D:/myWorkspace/imic_stm32_demo final/imic_stm32_demo/coreHTTP/dependency/3rdparty/llhttp/include" -I"D:/myWorkspace/imic_stm32_demo final/imic_stm32_demo/coreHTTP/interface" -I"D:/myWorkspace/imic_stm32_demo final/imic_stm32_demo/test/cbmc/include" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-test-2f-cbmc-2f-stubs

clean-test-2f-cbmc-2f-stubs:
	-$(RM) ./test/cbmc/stubs/HTTPClient_ReadHeader_llhttp_execute.cyclo ./test/cbmc/stubs/HTTPClient_ReadHeader_llhttp_execute.d ./test/cbmc/stubs/HTTPClient_ReadHeader_llhttp_execute.o ./test/cbmc/stubs/HTTPClient_ReadHeader_llhttp_execute.su ./test/cbmc/stubs/HTTPClient_Send_llhttp_execute.cyclo ./test/cbmc/stubs/HTTPClient_Send_llhttp_execute.d ./test/cbmc/stubs/HTTPClient_Send_llhttp_execute.o ./test/cbmc/stubs/HTTPClient_Send_llhttp_execute.su ./test/cbmc/stubs/callback_stubs.cyclo ./test/cbmc/stubs/callback_stubs.d ./test/cbmc/stubs/callback_stubs.o ./test/cbmc/stubs/callback_stubs.su ./test/cbmc/stubs/get_time_stub.cyclo ./test/cbmc/stubs/get_time_stub.d ./test/cbmc/stubs/get_time_stub.o ./test/cbmc/stubs/get_time_stub.su ./test/cbmc/stubs/httpHeaderStrncpy.cyclo ./test/cbmc/stubs/httpHeaderStrncpy.d ./test/cbmc/stubs/httpHeaderStrncpy.o ./test/cbmc/stubs/httpHeaderStrncpy.su ./test/cbmc/stubs/memmove.cyclo ./test/cbmc/stubs/memmove.d ./test/cbmc/stubs/memmove.o ./test/cbmc/stubs/memmove.su ./test/cbmc/stubs/strncpy.cyclo ./test/cbmc/stubs/strncpy.d ./test/cbmc/stubs/strncpy.o ./test/cbmc/stubs/strncpy.su ./test/cbmc/stubs/transport_interface_stubs.cyclo ./test/cbmc/stubs/transport_interface_stubs.d ./test/cbmc/stubs/transport_interface_stubs.o ./test/cbmc/stubs/transport_interface_stubs.su

.PHONY: clean-test-2f-cbmc-2f-stubs

