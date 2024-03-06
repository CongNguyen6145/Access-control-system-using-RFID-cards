################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (11.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../test/unit-test/core_http_send_utest.c \
../test/unit-test/core_http_utest.c 

OBJS += \
./test/unit-test/core_http_send_utest.o \
./test/unit-test/core_http_utest.o 

C_DEPS += \
./test/unit-test/core_http_send_utest.d \
./test/unit-test/core_http_utest.d 


# Each subdirectory must supply rules for building sources it contributes
test/unit-test/%.o test/unit-test/%.su test/unit-test/%.cyclo: ../test/unit-test/%.c test/unit-test/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F429xx -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I../Utils/ -I../Drivers/Device -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2 -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../LWIP/App -I../LWIP/Target -I"D:/myWorkspace/imic_stm32_demo final/imic_stm32_demo/Middlewares/Third_Party/LwIP/src/include" -I../Middlewares/Third_Party/LwIP/system -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -I"D:/myWorkspace/imic_stm32_demo final/imic_stm32_demo/coreHTTP/include" -I"D:/myWorkspace/imic_stm32_demo final/imic_stm32_demo/coreHTTP/dependency/3rdparty/llhttp/include" -I"D:/myWorkspace/imic_stm32_demo final/imic_stm32_demo/coreHTTP/interface" -I"D:/myWorkspace/imic_stm32_demo final/imic_stm32_demo/test/cbmc/include" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-test-2f-unit-2d-test

clean-test-2f-unit-2d-test:
	-$(RM) ./test/unit-test/core_http_send_utest.cyclo ./test/unit-test/core_http_send_utest.d ./test/unit-test/core_http_send_utest.o ./test/unit-test/core_http_send_utest.su ./test/unit-test/core_http_utest.cyclo ./test/unit-test/core_http_utest.d ./test/unit-test/core_http_utest.o ./test/unit-test/core_http_utest.su

.PHONY: clean-test-2f-unit-2d-test

