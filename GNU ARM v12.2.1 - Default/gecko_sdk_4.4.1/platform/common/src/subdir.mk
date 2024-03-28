################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../gecko_sdk_4.4.1/platform/common/src/sl_assert.c \
../gecko_sdk_4.4.1/platform/common/src/sl_slist.c \
../gecko_sdk_4.4.1/platform/common/src/sl_string.c \
../gecko_sdk_4.4.1/platform/common/src/sl_syscalls.c 

OBJS += \
./gecko_sdk_4.4.1/platform/common/src/sl_assert.o \
./gecko_sdk_4.4.1/platform/common/src/sl_slist.o \
./gecko_sdk_4.4.1/platform/common/src/sl_string.o \
./gecko_sdk_4.4.1/platform/common/src/sl_syscalls.o 

C_DEPS += \
./gecko_sdk_4.4.1/platform/common/src/sl_assert.d \
./gecko_sdk_4.4.1/platform/common/src/sl_slist.d \
./gecko_sdk_4.4.1/platform/common/src/sl_string.d \
./gecko_sdk_4.4.1/platform/common/src/sl_syscalls.d 


# Each subdirectory must supply rules for building sources it contributes
gecko_sdk_4.4.1/platform/common/src/sl_assert.o: ../gecko_sdk_4.4.1/platform/common/src/sl_assert.c gecko_sdk_4.4.1/platform/common/src/subdir.mk
	@echo 'Building file: $<'
	@echo 'Invoking: GNU ARM C Compiler'
	arm-none-eabi-gcc -g -gdwarf-2 -mcpu=cortex-m33 -mthumb -std=c99 '-DDEBUG_EFM=1' '-DEFR32MG24B210F1536IM48=1' '-DHARDWARE_BOARD_DEFAULT_RF_BAND_2400=1' '-DHARDWARE_BOARD_SUPPORTS_1_RF_BAND=1' '-DHARDWARE_BOARD_SUPPORTS_RF_BAND_2400=1' '-DHFXO_FREQ=39000000' '-DSL_BOARD_NAME="BRD4186C"' '-DSL_BOARD_REV="A01"' '-DSL_COMPONENT_CATALOG_PRESENT=1' -I"D:\CUSP24\ECEN5833\SimplicityStudioProjects\LPHR_Software\config" -I"D:\CUSP24\ECEN5833\SimplicityStudioProjects\LPHR_Software\autogen" -I"D:\CUSP24\ECEN5833\SimplicityStudioProjects\LPHR_Software" -I"D:\CUSP24\ECEN5833\SimplicityStudioProjects\LPHR_Software\gecko_sdk_4.4.1\platform\Device\SiliconLabs\EFR32MG24\Include" -I"D:\CUSP24\ECEN5833\SimplicityStudioProjects\LPHR_Software\gecko_sdk_4.4.1\platform\common\inc" -I"D:\CUSP24\ECEN5833\SimplicityStudioProjects\LPHR_Software\gecko_sdk_4.4.1\hardware\board\inc" -I"D:\CUSP24\ECEN5833\SimplicityStudioProjects\LPHR_Software\gecko_sdk_4.4.1\platform\CMSIS\Core\Include" -I"D:\CUSP24\ECEN5833\SimplicityStudioProjects\LPHR_Software\gecko_sdk_4.4.1\hardware\driver\configuration_over_swo\inc" -I"D:\CUSP24\ECEN5833\SimplicityStudioProjects\LPHR_Software\gecko_sdk_4.4.1\platform\driver\debug\inc" -I"D:\CUSP24\ECEN5833\SimplicityStudioProjects\LPHR_Software\gecko_sdk_4.4.1\platform\service\device_init\inc" -I"D:\CUSP24\ECEN5833\SimplicityStudioProjects\LPHR_Software\gecko_sdk_4.4.1\platform\emdrv\dmadrv\inc" -I"D:\CUSP24\ECEN5833\SimplicityStudioProjects\LPHR_Software\gecko_sdk_4.4.1\platform\emdrv\common\inc" -I"D:\CUSP24\ECEN5833\SimplicityStudioProjects\LPHR_Software\gecko_sdk_4.4.1\platform\emlib\inc" -I"D:\CUSP24\ECEN5833\SimplicityStudioProjects\LPHR_Software\gecko_sdk_4.4.1\platform\driver\i2cspm\inc" -I"D:\CUSP24\ECEN5833\SimplicityStudioProjects\LPHR_Software\gecko_sdk_4.4.1\platform\service\iostream\inc" -I"D:\CUSP24\ECEN5833\SimplicityStudioProjects\LPHR_Software\gecko_sdk_4.4.1\platform\driver\leddrv\inc" -I"D:\CUSP24\ECEN5833\SimplicityStudioProjects\LPHR_Software\gecko_sdk_4.4.1\hardware\driver\mx25_flash_shutdown\inc\sl_mx25_flash_shutdown_usart" -I"D:\CUSP24\ECEN5833\SimplicityStudioProjects\LPHR_Software\gecko_sdk_4.4.1\platform\peripheral\inc" -I"D:\CUSP24\ECEN5833\SimplicityStudioProjects\LPHR_Software\gecko_sdk_4.4.1\platform\common\toolchain\inc" -I"D:\CUSP24\ECEN5833\SimplicityStudioProjects\LPHR_Software\gecko_sdk_4.4.1\platform\service\system\inc" -I"D:\CUSP24\ECEN5833\SimplicityStudioProjects\LPHR_Software\gecko_sdk_4.4.1\platform\service\sleeptimer\inc" -I"D:\CUSP24\ECEN5833\SimplicityStudioProjects\LPHR_Software\gecko_sdk_4.4.1\platform\service\udelay\inc" -Os -Wall -Wextra -ffunction-sections -fdata-sections -imacrossl_gcc_preinclude.h -mfpu=fpv5-sp-d16 -mfloat-abi=hard -mcmse --specs=nano.specs -c -fmessage-length=0 -MMD -MP -MF"gecko_sdk_4.4.1/platform/common/src/sl_assert.d" -MT"$@" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '

gecko_sdk_4.4.1/platform/common/src/sl_slist.o: ../gecko_sdk_4.4.1/platform/common/src/sl_slist.c gecko_sdk_4.4.1/platform/common/src/subdir.mk
	@echo 'Building file: $<'
	@echo 'Invoking: GNU ARM C Compiler'
	arm-none-eabi-gcc -g -gdwarf-2 -mcpu=cortex-m33 -mthumb -std=c99 '-DDEBUG_EFM=1' '-DEFR32MG24B210F1536IM48=1' '-DHARDWARE_BOARD_DEFAULT_RF_BAND_2400=1' '-DHARDWARE_BOARD_SUPPORTS_1_RF_BAND=1' '-DHARDWARE_BOARD_SUPPORTS_RF_BAND_2400=1' '-DHFXO_FREQ=39000000' '-DSL_BOARD_NAME="BRD4186C"' '-DSL_BOARD_REV="A01"' '-DSL_COMPONENT_CATALOG_PRESENT=1' -I"D:\CUSP24\ECEN5833\SimplicityStudioProjects\LPHR_Software\config" -I"D:\CUSP24\ECEN5833\SimplicityStudioProjects\LPHR_Software\autogen" -I"D:\CUSP24\ECEN5833\SimplicityStudioProjects\LPHR_Software" -I"D:\CUSP24\ECEN5833\SimplicityStudioProjects\LPHR_Software\gecko_sdk_4.4.1\platform\Device\SiliconLabs\EFR32MG24\Include" -I"D:\CUSP24\ECEN5833\SimplicityStudioProjects\LPHR_Software\gecko_sdk_4.4.1\platform\common\inc" -I"D:\CUSP24\ECEN5833\SimplicityStudioProjects\LPHR_Software\gecko_sdk_4.4.1\hardware\board\inc" -I"D:\CUSP24\ECEN5833\SimplicityStudioProjects\LPHR_Software\gecko_sdk_4.4.1\platform\CMSIS\Core\Include" -I"D:\CUSP24\ECEN5833\SimplicityStudioProjects\LPHR_Software\gecko_sdk_4.4.1\hardware\driver\configuration_over_swo\inc" -I"D:\CUSP24\ECEN5833\SimplicityStudioProjects\LPHR_Software\gecko_sdk_4.4.1\platform\driver\debug\inc" -I"D:\CUSP24\ECEN5833\SimplicityStudioProjects\LPHR_Software\gecko_sdk_4.4.1\platform\service\device_init\inc" -I"D:\CUSP24\ECEN5833\SimplicityStudioProjects\LPHR_Software\gecko_sdk_4.4.1\platform\emdrv\dmadrv\inc" -I"D:\CUSP24\ECEN5833\SimplicityStudioProjects\LPHR_Software\gecko_sdk_4.4.1\platform\emdrv\common\inc" -I"D:\CUSP24\ECEN5833\SimplicityStudioProjects\LPHR_Software\gecko_sdk_4.4.1\platform\emlib\inc" -I"D:\CUSP24\ECEN5833\SimplicityStudioProjects\LPHR_Software\gecko_sdk_4.4.1\platform\driver\i2cspm\inc" -I"D:\CUSP24\ECEN5833\SimplicityStudioProjects\LPHR_Software\gecko_sdk_4.4.1\platform\service\iostream\inc" -I"D:\CUSP24\ECEN5833\SimplicityStudioProjects\LPHR_Software\gecko_sdk_4.4.1\platform\driver\leddrv\inc" -I"D:\CUSP24\ECEN5833\SimplicityStudioProjects\LPHR_Software\gecko_sdk_4.4.1\hardware\driver\mx25_flash_shutdown\inc\sl_mx25_flash_shutdown_usart" -I"D:\CUSP24\ECEN5833\SimplicityStudioProjects\LPHR_Software\gecko_sdk_4.4.1\platform\peripheral\inc" -I"D:\CUSP24\ECEN5833\SimplicityStudioProjects\LPHR_Software\gecko_sdk_4.4.1\platform\common\toolchain\inc" -I"D:\CUSP24\ECEN5833\SimplicityStudioProjects\LPHR_Software\gecko_sdk_4.4.1\platform\service\system\inc" -I"D:\CUSP24\ECEN5833\SimplicityStudioProjects\LPHR_Software\gecko_sdk_4.4.1\platform\service\sleeptimer\inc" -I"D:\CUSP24\ECEN5833\SimplicityStudioProjects\LPHR_Software\gecko_sdk_4.4.1\platform\service\udelay\inc" -Os -Wall -Wextra -ffunction-sections -fdata-sections -imacrossl_gcc_preinclude.h -mfpu=fpv5-sp-d16 -mfloat-abi=hard -mcmse --specs=nano.specs -c -fmessage-length=0 -MMD -MP -MF"gecko_sdk_4.4.1/platform/common/src/sl_slist.d" -MT"$@" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '

gecko_sdk_4.4.1/platform/common/src/sl_string.o: ../gecko_sdk_4.4.1/platform/common/src/sl_string.c gecko_sdk_4.4.1/platform/common/src/subdir.mk
	@echo 'Building file: $<'
	@echo 'Invoking: GNU ARM C Compiler'
	arm-none-eabi-gcc -g -gdwarf-2 -mcpu=cortex-m33 -mthumb -std=c99 '-DDEBUG_EFM=1' '-DEFR32MG24B210F1536IM48=1' '-DHARDWARE_BOARD_DEFAULT_RF_BAND_2400=1' '-DHARDWARE_BOARD_SUPPORTS_1_RF_BAND=1' '-DHARDWARE_BOARD_SUPPORTS_RF_BAND_2400=1' '-DHFXO_FREQ=39000000' '-DSL_BOARD_NAME="BRD4186C"' '-DSL_BOARD_REV="A01"' '-DSL_COMPONENT_CATALOG_PRESENT=1' -I"D:\CUSP24\ECEN5833\SimplicityStudioProjects\LPHR_Software\config" -I"D:\CUSP24\ECEN5833\SimplicityStudioProjects\LPHR_Software\autogen" -I"D:\CUSP24\ECEN5833\SimplicityStudioProjects\LPHR_Software" -I"D:\CUSP24\ECEN5833\SimplicityStudioProjects\LPHR_Software\gecko_sdk_4.4.1\platform\Device\SiliconLabs\EFR32MG24\Include" -I"D:\CUSP24\ECEN5833\SimplicityStudioProjects\LPHR_Software\gecko_sdk_4.4.1\platform\common\inc" -I"D:\CUSP24\ECEN5833\SimplicityStudioProjects\LPHR_Software\gecko_sdk_4.4.1\hardware\board\inc" -I"D:\CUSP24\ECEN5833\SimplicityStudioProjects\LPHR_Software\gecko_sdk_4.4.1\platform\CMSIS\Core\Include" -I"D:\CUSP24\ECEN5833\SimplicityStudioProjects\LPHR_Software\gecko_sdk_4.4.1\hardware\driver\configuration_over_swo\inc" -I"D:\CUSP24\ECEN5833\SimplicityStudioProjects\LPHR_Software\gecko_sdk_4.4.1\platform\driver\debug\inc" -I"D:\CUSP24\ECEN5833\SimplicityStudioProjects\LPHR_Software\gecko_sdk_4.4.1\platform\service\device_init\inc" -I"D:\CUSP24\ECEN5833\SimplicityStudioProjects\LPHR_Software\gecko_sdk_4.4.1\platform\emdrv\dmadrv\inc" -I"D:\CUSP24\ECEN5833\SimplicityStudioProjects\LPHR_Software\gecko_sdk_4.4.1\platform\emdrv\common\inc" -I"D:\CUSP24\ECEN5833\SimplicityStudioProjects\LPHR_Software\gecko_sdk_4.4.1\platform\emlib\inc" -I"D:\CUSP24\ECEN5833\SimplicityStudioProjects\LPHR_Software\gecko_sdk_4.4.1\platform\driver\i2cspm\inc" -I"D:\CUSP24\ECEN5833\SimplicityStudioProjects\LPHR_Software\gecko_sdk_4.4.1\platform\service\iostream\inc" -I"D:\CUSP24\ECEN5833\SimplicityStudioProjects\LPHR_Software\gecko_sdk_4.4.1\platform\driver\leddrv\inc" -I"D:\CUSP24\ECEN5833\SimplicityStudioProjects\LPHR_Software\gecko_sdk_4.4.1\hardware\driver\mx25_flash_shutdown\inc\sl_mx25_flash_shutdown_usart" -I"D:\CUSP24\ECEN5833\SimplicityStudioProjects\LPHR_Software\gecko_sdk_4.4.1\platform\peripheral\inc" -I"D:\CUSP24\ECEN5833\SimplicityStudioProjects\LPHR_Software\gecko_sdk_4.4.1\platform\common\toolchain\inc" -I"D:\CUSP24\ECEN5833\SimplicityStudioProjects\LPHR_Software\gecko_sdk_4.4.1\platform\service\system\inc" -I"D:\CUSP24\ECEN5833\SimplicityStudioProjects\LPHR_Software\gecko_sdk_4.4.1\platform\service\sleeptimer\inc" -I"D:\CUSP24\ECEN5833\SimplicityStudioProjects\LPHR_Software\gecko_sdk_4.4.1\platform\service\udelay\inc" -Os -Wall -Wextra -ffunction-sections -fdata-sections -imacrossl_gcc_preinclude.h -mfpu=fpv5-sp-d16 -mfloat-abi=hard -mcmse --specs=nano.specs -c -fmessage-length=0 -MMD -MP -MF"gecko_sdk_4.4.1/platform/common/src/sl_string.d" -MT"$@" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '

gecko_sdk_4.4.1/platform/common/src/sl_syscalls.o: ../gecko_sdk_4.4.1/platform/common/src/sl_syscalls.c gecko_sdk_4.4.1/platform/common/src/subdir.mk
	@echo 'Building file: $<'
	@echo 'Invoking: GNU ARM C Compiler'
	arm-none-eabi-gcc -g -gdwarf-2 -mcpu=cortex-m33 -mthumb -std=c99 '-DDEBUG_EFM=1' '-DEFR32MG24B210F1536IM48=1' '-DHARDWARE_BOARD_DEFAULT_RF_BAND_2400=1' '-DHARDWARE_BOARD_SUPPORTS_1_RF_BAND=1' '-DHARDWARE_BOARD_SUPPORTS_RF_BAND_2400=1' '-DHFXO_FREQ=39000000' '-DSL_BOARD_NAME="BRD4186C"' '-DSL_BOARD_REV="A01"' '-DSL_COMPONENT_CATALOG_PRESENT=1' -I"D:\CUSP24\ECEN5833\SimplicityStudioProjects\LPHR_Software\config" -I"D:\CUSP24\ECEN5833\SimplicityStudioProjects\LPHR_Software\autogen" -I"D:\CUSP24\ECEN5833\SimplicityStudioProjects\LPHR_Software" -I"D:\CUSP24\ECEN5833\SimplicityStudioProjects\LPHR_Software\gecko_sdk_4.4.1\platform\Device\SiliconLabs\EFR32MG24\Include" -I"D:\CUSP24\ECEN5833\SimplicityStudioProjects\LPHR_Software\gecko_sdk_4.4.1\platform\common\inc" -I"D:\CUSP24\ECEN5833\SimplicityStudioProjects\LPHR_Software\gecko_sdk_4.4.1\hardware\board\inc" -I"D:\CUSP24\ECEN5833\SimplicityStudioProjects\LPHR_Software\gecko_sdk_4.4.1\platform\CMSIS\Core\Include" -I"D:\CUSP24\ECEN5833\SimplicityStudioProjects\LPHR_Software\gecko_sdk_4.4.1\hardware\driver\configuration_over_swo\inc" -I"D:\CUSP24\ECEN5833\SimplicityStudioProjects\LPHR_Software\gecko_sdk_4.4.1\platform\driver\debug\inc" -I"D:\CUSP24\ECEN5833\SimplicityStudioProjects\LPHR_Software\gecko_sdk_4.4.1\platform\service\device_init\inc" -I"D:\CUSP24\ECEN5833\SimplicityStudioProjects\LPHR_Software\gecko_sdk_4.4.1\platform\emdrv\dmadrv\inc" -I"D:\CUSP24\ECEN5833\SimplicityStudioProjects\LPHR_Software\gecko_sdk_4.4.1\platform\emdrv\common\inc" -I"D:\CUSP24\ECEN5833\SimplicityStudioProjects\LPHR_Software\gecko_sdk_4.4.1\platform\emlib\inc" -I"D:\CUSP24\ECEN5833\SimplicityStudioProjects\LPHR_Software\gecko_sdk_4.4.1\platform\driver\i2cspm\inc" -I"D:\CUSP24\ECEN5833\SimplicityStudioProjects\LPHR_Software\gecko_sdk_4.4.1\platform\service\iostream\inc" -I"D:\CUSP24\ECEN5833\SimplicityStudioProjects\LPHR_Software\gecko_sdk_4.4.1\platform\driver\leddrv\inc" -I"D:\CUSP24\ECEN5833\SimplicityStudioProjects\LPHR_Software\gecko_sdk_4.4.1\hardware\driver\mx25_flash_shutdown\inc\sl_mx25_flash_shutdown_usart" -I"D:\CUSP24\ECEN5833\SimplicityStudioProjects\LPHR_Software\gecko_sdk_4.4.1\platform\peripheral\inc" -I"D:\CUSP24\ECEN5833\SimplicityStudioProjects\LPHR_Software\gecko_sdk_4.4.1\platform\common\toolchain\inc" -I"D:\CUSP24\ECEN5833\SimplicityStudioProjects\LPHR_Software\gecko_sdk_4.4.1\platform\service\system\inc" -I"D:\CUSP24\ECEN5833\SimplicityStudioProjects\LPHR_Software\gecko_sdk_4.4.1\platform\service\sleeptimer\inc" -I"D:\CUSP24\ECEN5833\SimplicityStudioProjects\LPHR_Software\gecko_sdk_4.4.1\platform\service\udelay\inc" -Os -Wall -Wextra -ffunction-sections -fdata-sections -imacrossl_gcc_preinclude.h -mfpu=fpv5-sp-d16 -mfloat-abi=hard -mcmse --specs=nano.specs -c -fmessage-length=0 -MMD -MP -MF"gecko_sdk_4.4.1/platform/common/src/sl_syscalls.d" -MT"$@" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '

