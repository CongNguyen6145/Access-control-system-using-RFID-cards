################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (11.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Drivers/BSP/Components/lan8742/lan8742.c 

OBJS += \
./Drivers/BSP/Components/lan8742/lan8742.o 

C_DEPS += \
./Drivers/BSP/Components/lan8742/lan8742.d 


# Each subdirectory must supply rules for building sources it contributes
Drivers/BSP/Components/lan8742/%.o Drivers/BSP/Components/lan8742/%.su Drivers/BSP/Components/lan8742/%.cyclo: ../Drivers/BSP/Components/lan8742/%.c Drivers/BSP/Components/lan8742/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F429xx -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I../Utils/ -I../Drivers/Device -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2 -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../LWIP/App -I../LWIP/Target -I"C:/Users/T14/Documents/laptrinh/stm32/imic_stm32_demo_http_bug_fixed/imic_stm32_demo final 3/imic_stm32_demo final 2/imic_stm32_demo/Middlewares/Third_Party/LwIP/src/include" -I../Middlewares/Third_Party/LwIP/system -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -I"C:/Users/T14/Documents/laptrinh/stm32/imic_stm32_demo_http_bug_fixed/imic_stm32_demo final 3/imic_stm32_demo final 2/imic_stm32_demo/coreHTTP/include" -I"C:/Users/T14/Documents/laptrinh/stm32/imic_stm32_demo_http_bug_fixed/imic_stm32_demo final 3/imic_stm32_demo final 2/imic_stm32_demo/coreHTTP/dependency/3rdparty/llhttp/include" -I"C:/Users/T14/Documents/laptrinh/stm32/imic_stm32_demo_http_bug_fixed/imic_stm32_demo final 3/imic_stm32_demo final 2/imic_stm32_demo/coreHTTP/interface" -I"C:/Users/T14/Documents/laptrinh/stm32/imic_stm32_demo_http_bug_fixed/imic_stm32_demo final 3/imic_stm32_demo final 2/imic_stm32_demo/test/cbmc/include" -I"C:/Users/T14/Documents/laptrinh/stm32/imic_stm32_demo_http_bug_fixed/imic_stm32_demo final 3/imic_stm32_demo final 2/imic_stm32_demo/Drivers/BSP/Components/lan8742" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Drivers-2f-BSP-2f-Components-2f-lan8742

clean-Drivers-2f-BSP-2f-Components-2f-lan8742:
	-$(RM) ./Drivers/BSP/Components/lan8742/lan8742.cyclo ./Drivers/BSP/Components/lan8742/lan8742.d ./Drivers/BSP/Components/lan8742/lan8742.o ./Drivers/BSP/Components/lan8742/lan8742.su

.PHONY: clean-Drivers-2f-BSP-2f-Components-2f-lan8742

