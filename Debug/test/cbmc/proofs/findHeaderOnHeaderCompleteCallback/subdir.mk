################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (11.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../test/cbmc/proofs/findHeaderOnHeaderCompleteCallback/findHeaderOnHeaderCompleteCallback_harness.c 

OBJS += \
./test/cbmc/proofs/findHeaderOnHeaderCompleteCallback/findHeaderOnHeaderCompleteCallback_harness.o 

C_DEPS += \
./test/cbmc/proofs/findHeaderOnHeaderCompleteCallback/findHeaderOnHeaderCompleteCallback_harness.d 


# Each subdirectory must supply rules for building sources it contributes
test/cbmc/proofs/findHeaderOnHeaderCompleteCallback/%.o test/cbmc/proofs/findHeaderOnHeaderCompleteCallback/%.su test/cbmc/proofs/findHeaderOnHeaderCompleteCallback/%.cyclo: ../test/cbmc/proofs/findHeaderOnHeaderCompleteCallback/%.c test/cbmc/proofs/findHeaderOnHeaderCompleteCallback/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F429xx -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I../Utils/ -I../Drivers/Device -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2 -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../LWIP/App -I../LWIP/Target -I"D:/myWorkspace/imic_stm32_demo final/imic_stm32_demo/Middlewares/Third_Party/LwIP/src/include" -I../Middlewares/Third_Party/LwIP/system -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -I"D:/myWorkspace/imic_stm32_demo final/imic_stm32_demo/coreHTTP/include" -I"D:/myWorkspace/imic_stm32_demo final/imic_stm32_demo/coreHTTP/dependency/3rdparty/llhttp/include" -I"D:/myWorkspace/imic_stm32_demo final/imic_stm32_demo/coreHTTP/interface" -I"D:/myWorkspace/imic_stm32_demo final/imic_stm32_demo/test/cbmc/include" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-test-2f-cbmc-2f-proofs-2f-findHeaderOnHeaderCompleteCallback

clean-test-2f-cbmc-2f-proofs-2f-findHeaderOnHeaderCompleteCallback:
	-$(RM) ./test/cbmc/proofs/findHeaderOnHeaderCompleteCallback/findHeaderOnHeaderCompleteCallback_harness.cyclo ./test/cbmc/proofs/findHeaderOnHeaderCompleteCallback/findHeaderOnHeaderCompleteCallback_harness.d ./test/cbmc/proofs/findHeaderOnHeaderCompleteCallback/findHeaderOnHeaderCompleteCallback_harness.o ./test/cbmc/proofs/findHeaderOnHeaderCompleteCallback/findHeaderOnHeaderCompleteCallback_harness.su

.PHONY: clean-test-2f-cbmc-2f-proofs-2f-findHeaderOnHeaderCompleteCallback

