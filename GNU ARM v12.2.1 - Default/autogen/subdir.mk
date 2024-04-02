################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../autogen/sl_board_default_init.c \
../autogen/sl_device_init_clocks.c \
../autogen/sl_event_handler.c \
../autogen/sl_i2cspm_init.c \
../autogen/sl_iostream_handles.c \
../autogen/sl_iostream_init_eusart_instances.c \
../autogen/sl_simple_led_instances.c 

OBJS += \
./autogen/sl_board_default_init.o \
./autogen/sl_device_init_clocks.o \
./autogen/sl_event_handler.o \
./autogen/sl_i2cspm_init.o \
./autogen/sl_iostream_handles.o \
./autogen/sl_iostream_init_eusart_instances.o \
./autogen/sl_simple_led_instances.o 

C_DEPS += \
./autogen/sl_board_default_init.d \
./autogen/sl_device_init_clocks.d \
./autogen/sl_event_handler.d \
./autogen/sl_i2cspm_init.d \
./autogen/sl_iostream_handles.d \
./autogen/sl_iostream_init_eusart_instances.d \
./autogen/sl_simple_led_instances.d 


# Each subdirectory must supply rules for building sources it contributes
autogen/sl_board_default_init.o: ../autogen/sl_board_default_init.c autogen/subdir.mk
	@echo 'Building file: $<'
	@echo 'Invoking: GNU ARM C Compiler'
	arm-none-eabi-gcc -g -gdwarf-2 -mcpu=cortex-m33 -mthumb -std=c99 '-DDEBUG_EFM=1' '-DEFR32MG24B210F1536IM48=1' '-DHARDWARE_BOARD_DEFAULT_RF_BAND_2400=1' '-DHARDWARE_BOARD_SUPPORTS_1_RF_BAND=1' '-DHARDWARE_BOARD_SUPPORTS_RF_BAND_2400=1' '-DHFXO_FREQ=39000000' '-DSL_BOARD_NAME="BRD4186C"' '-DSL_BOARD_REV="A01"' '-DSL_COMPONENT_CATALOG_PRESENT=1' -I"C:\Users\Halcyon\Desktop\CUSP24\ECEN5833\LPEDT_Software\config" -I"C:\Users\Halcyon\Desktop\CUSP24\ECEN5833\LPEDT_Software\autogen" -I"C:\Users\Halcyon\Desktop\CUSP24\ECEN5833\LPEDT_Software" -I"C:\Users\Halcyon\Desktop\CUSP24\ECEN5833\LPEDT_Software\gecko_sdk_4.4.1\platform\Device\SiliconLabs\EFR32MG24\Include" -I"C:\Users\Halcyon\Desktop\CUSP24\ECEN5833\LPEDT_Software\gecko_sdk_4.4.1\platform\common\inc" -I"C:\Users\Halcyon\Desktop\CUSP24\ECEN5833\LPEDT_Software\gecko_sdk_4.4.1\hardware\board\inc" -I"C:\Users\Halcyon\Desktop\CUSP24\ECEN5833\LPEDT_Software\gecko_sdk_4.4.1\platform\CMSIS\Core\Include" -I"C:\Users\Halcyon\Desktop\CUSP24\ECEN5833\LPEDT_Software\gecko_sdk_4.4.1\hardware\driver\configuration_over_swo\inc" -I"C:\Users\Halcyon\Desktop\CUSP24\ECEN5833\LPEDT_Software\gecko_sdk_4.4.1\platform\driver\debug\inc" -I"C:\Users\Halcyon\Desktop\CUSP24\ECEN5833\LPEDT_Software\gecko_sdk_4.4.1\platform\service\device_init\inc" -I"C:\Users\Halcyon\Desktop\CUSP24\ECEN5833\LPEDT_Software\gecko_sdk_4.4.1\platform\emdrv\dmadrv\inc" -I"C:\Users\Halcyon\Desktop\CUSP24\ECEN5833\LPEDT_Software\gecko_sdk_4.4.1\platform\emdrv\common\inc" -I"C:\Users\Halcyon\Desktop\CUSP24\ECEN5833\LPEDT_Software\gecko_sdk_4.4.1\platform\emlib\inc" -I"C:\Users\Halcyon\Desktop\CUSP24\ECEN5833\LPEDT_Software\gecko_sdk_4.4.1\platform\driver\i2cspm\inc" -I"C:\Users\Halcyon\Desktop\CUSP24\ECEN5833\LPEDT_Software\gecko_sdk_4.4.1\platform\service\iostream\inc" -I"C:\Users\Halcyon\Desktop\CUSP24\ECEN5833\LPEDT_Software\gecko_sdk_4.4.1\platform\driver\leddrv\inc" -I"C:\Users\Halcyon\Desktop\CUSP24\ECEN5833\LPEDT_Software\gecko_sdk_4.4.1\hardware\driver\mx25_flash_shutdown\inc\sl_mx25_flash_shutdown_usart" -I"C:\Users\Halcyon\Desktop\CUSP24\ECEN5833\LPEDT_Software\gecko_sdk_4.4.1\platform\peripheral\inc" -I"C:\Users\Halcyon\Desktop\CUSP24\ECEN5833\LPEDT_Software\gecko_sdk_4.4.1\platform\common\toolchain\inc" -I"C:\Users\Halcyon\Desktop\CUSP24\ECEN5833\LPEDT_Software\gecko_sdk_4.4.1\platform\service\system\inc" -I"C:\Users\Halcyon\Desktop\CUSP24\ECEN5833\LPEDT_Software\gecko_sdk_4.4.1\platform\service\sleeptimer\inc" -I"C:\Users\Halcyon\Desktop\CUSP24\ECEN5833\LPEDT_Software\gecko_sdk_4.4.1\platform\service\udelay\inc" -Os -Wall -Wextra -ffunction-sections -fdata-sections -imacrossl_gcc_preinclude.h -mfpu=fpv5-sp-d16 -mfloat-abi=hard -mcmse --specs=nano.specs -c -fmessage-length=0 -MMD -MP -MF"autogen/sl_board_default_init.d" -MT"$@" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '

autogen/sl_device_init_clocks.o: ../autogen/sl_device_init_clocks.c autogen/subdir.mk
	@echo 'Building file: $<'
	@echo 'Invoking: GNU ARM C Compiler'
	arm-none-eabi-gcc -g -gdwarf-2 -mcpu=cortex-m33 -mthumb -std=c99 '-DDEBUG_EFM=1' '-DEFR32MG24B210F1536IM48=1' '-DHARDWARE_BOARD_DEFAULT_RF_BAND_2400=1' '-DHARDWARE_BOARD_SUPPORTS_1_RF_BAND=1' '-DHARDWARE_BOARD_SUPPORTS_RF_BAND_2400=1' '-DHFXO_FREQ=39000000' '-DSL_BOARD_NAME="BRD4186C"' '-DSL_BOARD_REV="A01"' '-DSL_COMPONENT_CATALOG_PRESENT=1' -I"C:\Users\Halcyon\Desktop\CUSP24\ECEN5833\LPEDT_Software\config" -I"C:\Users\Halcyon\Desktop\CUSP24\ECEN5833\LPEDT_Software\autogen" -I"C:\Users\Halcyon\Desktop\CUSP24\ECEN5833\LPEDT_Software" -I"C:\Users\Halcyon\Desktop\CUSP24\ECEN5833\LPEDT_Software\gecko_sdk_4.4.1\platform\Device\SiliconLabs\EFR32MG24\Include" -I"C:\Users\Halcyon\Desktop\CUSP24\ECEN5833\LPEDT_Software\gecko_sdk_4.4.1\platform\common\inc" -I"C:\Users\Halcyon\Desktop\CUSP24\ECEN5833\LPEDT_Software\gecko_sdk_4.4.1\hardware\board\inc" -I"C:\Users\Halcyon\Desktop\CUSP24\ECEN5833\LPEDT_Software\gecko_sdk_4.4.1\platform\CMSIS\Core\Include" -I"C:\Users\Halcyon\Desktop\CUSP24\ECEN5833\LPEDT_Software\gecko_sdk_4.4.1\hardware\driver\configuration_over_swo\inc" -I"C:\Users\Halcyon\Desktop\CUSP24\ECEN5833\LPEDT_Software\gecko_sdk_4.4.1\platform\driver\debug\inc" -I"C:\Users\Halcyon\Desktop\CUSP24\ECEN5833\LPEDT_Software\gecko_sdk_4.4.1\platform\service\device_init\inc" -I"C:\Users\Halcyon\Desktop\CUSP24\ECEN5833\LPEDT_Software\gecko_sdk_4.4.1\platform\emdrv\dmadrv\inc" -I"C:\Users\Halcyon\Desktop\CUSP24\ECEN5833\LPEDT_Software\gecko_sdk_4.4.1\platform\emdrv\common\inc" -I"C:\Users\Halcyon\Desktop\CUSP24\ECEN5833\LPEDT_Software\gecko_sdk_4.4.1\platform\emlib\inc" -I"C:\Users\Halcyon\Desktop\CUSP24\ECEN5833\LPEDT_Software\gecko_sdk_4.4.1\platform\driver\i2cspm\inc" -I"C:\Users\Halcyon\Desktop\CUSP24\ECEN5833\LPEDT_Software\gecko_sdk_4.4.1\platform\service\iostream\inc" -I"C:\Users\Halcyon\Desktop\CUSP24\ECEN5833\LPEDT_Software\gecko_sdk_4.4.1\platform\driver\leddrv\inc" -I"C:\Users\Halcyon\Desktop\CUSP24\ECEN5833\LPEDT_Software\gecko_sdk_4.4.1\hardware\driver\mx25_flash_shutdown\inc\sl_mx25_flash_shutdown_usart" -I"C:\Users\Halcyon\Desktop\CUSP24\ECEN5833\LPEDT_Software\gecko_sdk_4.4.1\platform\peripheral\inc" -I"C:\Users\Halcyon\Desktop\CUSP24\ECEN5833\LPEDT_Software\gecko_sdk_4.4.1\platform\common\toolchain\inc" -I"C:\Users\Halcyon\Desktop\CUSP24\ECEN5833\LPEDT_Software\gecko_sdk_4.4.1\platform\service\system\inc" -I"C:\Users\Halcyon\Desktop\CUSP24\ECEN5833\LPEDT_Software\gecko_sdk_4.4.1\platform\service\sleeptimer\inc" -I"C:\Users\Halcyon\Desktop\CUSP24\ECEN5833\LPEDT_Software\gecko_sdk_4.4.1\platform\service\udelay\inc" -Os -Wall -Wextra -ffunction-sections -fdata-sections -imacrossl_gcc_preinclude.h -mfpu=fpv5-sp-d16 -mfloat-abi=hard -mcmse --specs=nano.specs -c -fmessage-length=0 -MMD -MP -MF"autogen/sl_device_init_clocks.d" -MT"$@" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '

autogen/sl_event_handler.o: ../autogen/sl_event_handler.c autogen/subdir.mk
	@echo 'Building file: $<'
	@echo 'Invoking: GNU ARM C Compiler'
	arm-none-eabi-gcc -g -gdwarf-2 -mcpu=cortex-m33 -mthumb -std=c99 '-DDEBUG_EFM=1' '-DEFR32MG24B210F1536IM48=1' '-DHARDWARE_BOARD_DEFAULT_RF_BAND_2400=1' '-DHARDWARE_BOARD_SUPPORTS_1_RF_BAND=1' '-DHARDWARE_BOARD_SUPPORTS_RF_BAND_2400=1' '-DHFXO_FREQ=39000000' '-DSL_BOARD_NAME="BRD4186C"' '-DSL_BOARD_REV="A01"' '-DSL_COMPONENT_CATALOG_PRESENT=1' -I"C:\Users\Halcyon\Desktop\CUSP24\ECEN5833\LPEDT_Software\config" -I"C:\Users\Halcyon\Desktop\CUSP24\ECEN5833\LPEDT_Software\autogen" -I"C:\Users\Halcyon\Desktop\CUSP24\ECEN5833\LPEDT_Software" -I"C:\Users\Halcyon\Desktop\CUSP24\ECEN5833\LPEDT_Software\gecko_sdk_4.4.1\platform\Device\SiliconLabs\EFR32MG24\Include" -I"C:\Users\Halcyon\Desktop\CUSP24\ECEN5833\LPEDT_Software\gecko_sdk_4.4.1\platform\common\inc" -I"C:\Users\Halcyon\Desktop\CUSP24\ECEN5833\LPEDT_Software\gecko_sdk_4.4.1\hardware\board\inc" -I"C:\Users\Halcyon\Desktop\CUSP24\ECEN5833\LPEDT_Software\gecko_sdk_4.4.1\platform\CMSIS\Core\Include" -I"C:\Users\Halcyon\Desktop\CUSP24\ECEN5833\LPEDT_Software\gecko_sdk_4.4.1\hardware\driver\configuration_over_swo\inc" -I"C:\Users\Halcyon\Desktop\CUSP24\ECEN5833\LPEDT_Software\gecko_sdk_4.4.1\platform\driver\debug\inc" -I"C:\Users\Halcyon\Desktop\CUSP24\ECEN5833\LPEDT_Software\gecko_sdk_4.4.1\platform\service\device_init\inc" -I"C:\Users\Halcyon\Desktop\CUSP24\ECEN5833\LPEDT_Software\gecko_sdk_4.4.1\platform\emdrv\dmadrv\inc" -I"C:\Users\Halcyon\Desktop\CUSP24\ECEN5833\LPEDT_Software\gecko_sdk_4.4.1\platform\emdrv\common\inc" -I"C:\Users\Halcyon\Desktop\CUSP24\ECEN5833\LPEDT_Software\gecko_sdk_4.4.1\platform\emlib\inc" -I"C:\Users\Halcyon\Desktop\CUSP24\ECEN5833\LPEDT_Software\gecko_sdk_4.4.1\platform\driver\i2cspm\inc" -I"C:\Users\Halcyon\Desktop\CUSP24\ECEN5833\LPEDT_Software\gecko_sdk_4.4.1\platform\service\iostream\inc" -I"C:\Users\Halcyon\Desktop\CUSP24\ECEN5833\LPEDT_Software\gecko_sdk_4.4.1\platform\driver\leddrv\inc" -I"C:\Users\Halcyon\Desktop\CUSP24\ECEN5833\LPEDT_Software\gecko_sdk_4.4.1\hardware\driver\mx25_flash_shutdown\inc\sl_mx25_flash_shutdown_usart" -I"C:\Users\Halcyon\Desktop\CUSP24\ECEN5833\LPEDT_Software\gecko_sdk_4.4.1\platform\peripheral\inc" -I"C:\Users\Halcyon\Desktop\CUSP24\ECEN5833\LPEDT_Software\gecko_sdk_4.4.1\platform\common\toolchain\inc" -I"C:\Users\Halcyon\Desktop\CUSP24\ECEN5833\LPEDT_Software\gecko_sdk_4.4.1\platform\service\system\inc" -I"C:\Users\Halcyon\Desktop\CUSP24\ECEN5833\LPEDT_Software\gecko_sdk_4.4.1\platform\service\sleeptimer\inc" -I"C:\Users\Halcyon\Desktop\CUSP24\ECEN5833\LPEDT_Software\gecko_sdk_4.4.1\platform\service\udelay\inc" -Os -Wall -Wextra -ffunction-sections -fdata-sections -imacrossl_gcc_preinclude.h -mfpu=fpv5-sp-d16 -mfloat-abi=hard -mcmse --specs=nano.specs -c -fmessage-length=0 -MMD -MP -MF"autogen/sl_event_handler.d" -MT"$@" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '

autogen/sl_i2cspm_init.o: ../autogen/sl_i2cspm_init.c autogen/subdir.mk
	@echo 'Building file: $<'
	@echo 'Invoking: GNU ARM C Compiler'
	arm-none-eabi-gcc -g -gdwarf-2 -mcpu=cortex-m33 -mthumb -std=c99 '-DDEBUG_EFM=1' '-DEFR32MG24B210F1536IM48=1' '-DHARDWARE_BOARD_DEFAULT_RF_BAND_2400=1' '-DHARDWARE_BOARD_SUPPORTS_1_RF_BAND=1' '-DHARDWARE_BOARD_SUPPORTS_RF_BAND_2400=1' '-DHFXO_FREQ=39000000' '-DSL_BOARD_NAME="BRD4186C"' '-DSL_BOARD_REV="A01"' '-DSL_COMPONENT_CATALOG_PRESENT=1' -I"C:\Users\Halcyon\Desktop\CUSP24\ECEN5833\LPEDT_Software\config" -I"C:\Users\Halcyon\Desktop\CUSP24\ECEN5833\LPEDT_Software\autogen" -I"C:\Users\Halcyon\Desktop\CUSP24\ECEN5833\LPEDT_Software" -I"C:\Users\Halcyon\Desktop\CUSP24\ECEN5833\LPEDT_Software\gecko_sdk_4.4.1\platform\Device\SiliconLabs\EFR32MG24\Include" -I"C:\Users\Halcyon\Desktop\CUSP24\ECEN5833\LPEDT_Software\gecko_sdk_4.4.1\platform\common\inc" -I"C:\Users\Halcyon\Desktop\CUSP24\ECEN5833\LPEDT_Software\gecko_sdk_4.4.1\hardware\board\inc" -I"C:\Users\Halcyon\Desktop\CUSP24\ECEN5833\LPEDT_Software\gecko_sdk_4.4.1\platform\CMSIS\Core\Include" -I"C:\Users\Halcyon\Desktop\CUSP24\ECEN5833\LPEDT_Software\gecko_sdk_4.4.1\hardware\driver\configuration_over_swo\inc" -I"C:\Users\Halcyon\Desktop\CUSP24\ECEN5833\LPEDT_Software\gecko_sdk_4.4.1\platform\driver\debug\inc" -I"C:\Users\Halcyon\Desktop\CUSP24\ECEN5833\LPEDT_Software\gecko_sdk_4.4.1\platform\service\device_init\inc" -I"C:\Users\Halcyon\Desktop\CUSP24\ECEN5833\LPEDT_Software\gecko_sdk_4.4.1\platform\emdrv\dmadrv\inc" -I"C:\Users\Halcyon\Desktop\CUSP24\ECEN5833\LPEDT_Software\gecko_sdk_4.4.1\platform\emdrv\common\inc" -I"C:\Users\Halcyon\Desktop\CUSP24\ECEN5833\LPEDT_Software\gecko_sdk_4.4.1\platform\emlib\inc" -I"C:\Users\Halcyon\Desktop\CUSP24\ECEN5833\LPEDT_Software\gecko_sdk_4.4.1\platform\driver\i2cspm\inc" -I"C:\Users\Halcyon\Desktop\CUSP24\ECEN5833\LPEDT_Software\gecko_sdk_4.4.1\platform\service\iostream\inc" -I"C:\Users\Halcyon\Desktop\CUSP24\ECEN5833\LPEDT_Software\gecko_sdk_4.4.1\platform\driver\leddrv\inc" -I"C:\Users\Halcyon\Desktop\CUSP24\ECEN5833\LPEDT_Software\gecko_sdk_4.4.1\hardware\driver\mx25_flash_shutdown\inc\sl_mx25_flash_shutdown_usart" -I"C:\Users\Halcyon\Desktop\CUSP24\ECEN5833\LPEDT_Software\gecko_sdk_4.4.1\platform\peripheral\inc" -I"C:\Users\Halcyon\Desktop\CUSP24\ECEN5833\LPEDT_Software\gecko_sdk_4.4.1\platform\common\toolchain\inc" -I"C:\Users\Halcyon\Desktop\CUSP24\ECEN5833\LPEDT_Software\gecko_sdk_4.4.1\platform\service\system\inc" -I"C:\Users\Halcyon\Desktop\CUSP24\ECEN5833\LPEDT_Software\gecko_sdk_4.4.1\platform\service\sleeptimer\inc" -I"C:\Users\Halcyon\Desktop\CUSP24\ECEN5833\LPEDT_Software\gecko_sdk_4.4.1\platform\service\udelay\inc" -Os -Wall -Wextra -ffunction-sections -fdata-sections -imacrossl_gcc_preinclude.h -mfpu=fpv5-sp-d16 -mfloat-abi=hard -mcmse --specs=nano.specs -c -fmessage-length=0 -MMD -MP -MF"autogen/sl_i2cspm_init.d" -MT"$@" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '

autogen/sl_iostream_handles.o: ../autogen/sl_iostream_handles.c autogen/subdir.mk
	@echo 'Building file: $<'
	@echo 'Invoking: GNU ARM C Compiler'
	arm-none-eabi-gcc -g -gdwarf-2 -mcpu=cortex-m33 -mthumb -std=c99 '-DDEBUG_EFM=1' '-DEFR32MG24B210F1536IM48=1' '-DHARDWARE_BOARD_DEFAULT_RF_BAND_2400=1' '-DHARDWARE_BOARD_SUPPORTS_1_RF_BAND=1' '-DHARDWARE_BOARD_SUPPORTS_RF_BAND_2400=1' '-DHFXO_FREQ=39000000' '-DSL_BOARD_NAME="BRD4186C"' '-DSL_BOARD_REV="A01"' '-DSL_COMPONENT_CATALOG_PRESENT=1' -I"C:\Users\Halcyon\Desktop\CUSP24\ECEN5833\LPEDT_Software\config" -I"C:\Users\Halcyon\Desktop\CUSP24\ECEN5833\LPEDT_Software\autogen" -I"C:\Users\Halcyon\Desktop\CUSP24\ECEN5833\LPEDT_Software" -I"C:\Users\Halcyon\Desktop\CUSP24\ECEN5833\LPEDT_Software\gecko_sdk_4.4.1\platform\Device\SiliconLabs\EFR32MG24\Include" -I"C:\Users\Halcyon\Desktop\CUSP24\ECEN5833\LPEDT_Software\gecko_sdk_4.4.1\platform\common\inc" -I"C:\Users\Halcyon\Desktop\CUSP24\ECEN5833\LPEDT_Software\gecko_sdk_4.4.1\hardware\board\inc" -I"C:\Users\Halcyon\Desktop\CUSP24\ECEN5833\LPEDT_Software\gecko_sdk_4.4.1\platform\CMSIS\Core\Include" -I"C:\Users\Halcyon\Desktop\CUSP24\ECEN5833\LPEDT_Software\gecko_sdk_4.4.1\hardware\driver\configuration_over_swo\inc" -I"C:\Users\Halcyon\Desktop\CUSP24\ECEN5833\LPEDT_Software\gecko_sdk_4.4.1\platform\driver\debug\inc" -I"C:\Users\Halcyon\Desktop\CUSP24\ECEN5833\LPEDT_Software\gecko_sdk_4.4.1\platform\service\device_init\inc" -I"C:\Users\Halcyon\Desktop\CUSP24\ECEN5833\LPEDT_Software\gecko_sdk_4.4.1\platform\emdrv\dmadrv\inc" -I"C:\Users\Halcyon\Desktop\CUSP24\ECEN5833\LPEDT_Software\gecko_sdk_4.4.1\platform\emdrv\common\inc" -I"C:\Users\Halcyon\Desktop\CUSP24\ECEN5833\LPEDT_Software\gecko_sdk_4.4.1\platform\emlib\inc" -I"C:\Users\Halcyon\Desktop\CUSP24\ECEN5833\LPEDT_Software\gecko_sdk_4.4.1\platform\driver\i2cspm\inc" -I"C:\Users\Halcyon\Desktop\CUSP24\ECEN5833\LPEDT_Software\gecko_sdk_4.4.1\platform\service\iostream\inc" -I"C:\Users\Halcyon\Desktop\CUSP24\ECEN5833\LPEDT_Software\gecko_sdk_4.4.1\platform\driver\leddrv\inc" -I"C:\Users\Halcyon\Desktop\CUSP24\ECEN5833\LPEDT_Software\gecko_sdk_4.4.1\hardware\driver\mx25_flash_shutdown\inc\sl_mx25_flash_shutdown_usart" -I"C:\Users\Halcyon\Desktop\CUSP24\ECEN5833\LPEDT_Software\gecko_sdk_4.4.1\platform\peripheral\inc" -I"C:\Users\Halcyon\Desktop\CUSP24\ECEN5833\LPEDT_Software\gecko_sdk_4.4.1\platform\common\toolchain\inc" -I"C:\Users\Halcyon\Desktop\CUSP24\ECEN5833\LPEDT_Software\gecko_sdk_4.4.1\platform\service\system\inc" -I"C:\Users\Halcyon\Desktop\CUSP24\ECEN5833\LPEDT_Software\gecko_sdk_4.4.1\platform\service\sleeptimer\inc" -I"C:\Users\Halcyon\Desktop\CUSP24\ECEN5833\LPEDT_Software\gecko_sdk_4.4.1\platform\service\udelay\inc" -Os -Wall -Wextra -ffunction-sections -fdata-sections -imacrossl_gcc_preinclude.h -mfpu=fpv5-sp-d16 -mfloat-abi=hard -mcmse --specs=nano.specs -c -fmessage-length=0 -MMD -MP -MF"autogen/sl_iostream_handles.d" -MT"$@" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '

autogen/sl_iostream_init_eusart_instances.o: ../autogen/sl_iostream_init_eusart_instances.c autogen/subdir.mk
	@echo 'Building file: $<'
	@echo 'Invoking: GNU ARM C Compiler'
	arm-none-eabi-gcc -g -gdwarf-2 -mcpu=cortex-m33 -mthumb -std=c99 '-DDEBUG_EFM=1' '-DEFR32MG24B210F1536IM48=1' '-DHARDWARE_BOARD_DEFAULT_RF_BAND_2400=1' '-DHARDWARE_BOARD_SUPPORTS_1_RF_BAND=1' '-DHARDWARE_BOARD_SUPPORTS_RF_BAND_2400=1' '-DHFXO_FREQ=39000000' '-DSL_BOARD_NAME="BRD4186C"' '-DSL_BOARD_REV="A01"' '-DSL_COMPONENT_CATALOG_PRESENT=1' -I"C:\Users\Halcyon\Desktop\CUSP24\ECEN5833\LPEDT_Software\config" -I"C:\Users\Halcyon\Desktop\CUSP24\ECEN5833\LPEDT_Software\autogen" -I"C:\Users\Halcyon\Desktop\CUSP24\ECEN5833\LPEDT_Software" -I"C:\Users\Halcyon\Desktop\CUSP24\ECEN5833\LPEDT_Software\gecko_sdk_4.4.1\platform\Device\SiliconLabs\EFR32MG24\Include" -I"C:\Users\Halcyon\Desktop\CUSP24\ECEN5833\LPEDT_Software\gecko_sdk_4.4.1\platform\common\inc" -I"C:\Users\Halcyon\Desktop\CUSP24\ECEN5833\LPEDT_Software\gecko_sdk_4.4.1\hardware\board\inc" -I"C:\Users\Halcyon\Desktop\CUSP24\ECEN5833\LPEDT_Software\gecko_sdk_4.4.1\platform\CMSIS\Core\Include" -I"C:\Users\Halcyon\Desktop\CUSP24\ECEN5833\LPEDT_Software\gecko_sdk_4.4.1\hardware\driver\configuration_over_swo\inc" -I"C:\Users\Halcyon\Desktop\CUSP24\ECEN5833\LPEDT_Software\gecko_sdk_4.4.1\platform\driver\debug\inc" -I"C:\Users\Halcyon\Desktop\CUSP24\ECEN5833\LPEDT_Software\gecko_sdk_4.4.1\platform\service\device_init\inc" -I"C:\Users\Halcyon\Desktop\CUSP24\ECEN5833\LPEDT_Software\gecko_sdk_4.4.1\platform\emdrv\dmadrv\inc" -I"C:\Users\Halcyon\Desktop\CUSP24\ECEN5833\LPEDT_Software\gecko_sdk_4.4.1\platform\emdrv\common\inc" -I"C:\Users\Halcyon\Desktop\CUSP24\ECEN5833\LPEDT_Software\gecko_sdk_4.4.1\platform\emlib\inc" -I"C:\Users\Halcyon\Desktop\CUSP24\ECEN5833\LPEDT_Software\gecko_sdk_4.4.1\platform\driver\i2cspm\inc" -I"C:\Users\Halcyon\Desktop\CUSP24\ECEN5833\LPEDT_Software\gecko_sdk_4.4.1\platform\service\iostream\inc" -I"C:\Users\Halcyon\Desktop\CUSP24\ECEN5833\LPEDT_Software\gecko_sdk_4.4.1\platform\driver\leddrv\inc" -I"C:\Users\Halcyon\Desktop\CUSP24\ECEN5833\LPEDT_Software\gecko_sdk_4.4.1\hardware\driver\mx25_flash_shutdown\inc\sl_mx25_flash_shutdown_usart" -I"C:\Users\Halcyon\Desktop\CUSP24\ECEN5833\LPEDT_Software\gecko_sdk_4.4.1\platform\peripheral\inc" -I"C:\Users\Halcyon\Desktop\CUSP24\ECEN5833\LPEDT_Software\gecko_sdk_4.4.1\platform\common\toolchain\inc" -I"C:\Users\Halcyon\Desktop\CUSP24\ECEN5833\LPEDT_Software\gecko_sdk_4.4.1\platform\service\system\inc" -I"C:\Users\Halcyon\Desktop\CUSP24\ECEN5833\LPEDT_Software\gecko_sdk_4.4.1\platform\service\sleeptimer\inc" -I"C:\Users\Halcyon\Desktop\CUSP24\ECEN5833\LPEDT_Software\gecko_sdk_4.4.1\platform\service\udelay\inc" -Os -Wall -Wextra -ffunction-sections -fdata-sections -imacrossl_gcc_preinclude.h -mfpu=fpv5-sp-d16 -mfloat-abi=hard -mcmse --specs=nano.specs -c -fmessage-length=0 -MMD -MP -MF"autogen/sl_iostream_init_eusart_instances.d" -MT"$@" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '

autogen/sl_simple_led_instances.o: ../autogen/sl_simple_led_instances.c autogen/subdir.mk
	@echo 'Building file: $<'
	@echo 'Invoking: GNU ARM C Compiler'
	arm-none-eabi-gcc -g -gdwarf-2 -mcpu=cortex-m33 -mthumb -std=c99 '-DDEBUG_EFM=1' '-DEFR32MG24B210F1536IM48=1' '-DHARDWARE_BOARD_DEFAULT_RF_BAND_2400=1' '-DHARDWARE_BOARD_SUPPORTS_1_RF_BAND=1' '-DHARDWARE_BOARD_SUPPORTS_RF_BAND_2400=1' '-DHFXO_FREQ=39000000' '-DSL_BOARD_NAME="BRD4186C"' '-DSL_BOARD_REV="A01"' '-DSL_COMPONENT_CATALOG_PRESENT=1' -I"C:\Users\Halcyon\Desktop\CUSP24\ECEN5833\LPEDT_Software\config" -I"C:\Users\Halcyon\Desktop\CUSP24\ECEN5833\LPEDT_Software\autogen" -I"C:\Users\Halcyon\Desktop\CUSP24\ECEN5833\LPEDT_Software" -I"C:\Users\Halcyon\Desktop\CUSP24\ECEN5833\LPEDT_Software\gecko_sdk_4.4.1\platform\Device\SiliconLabs\EFR32MG24\Include" -I"C:\Users\Halcyon\Desktop\CUSP24\ECEN5833\LPEDT_Software\gecko_sdk_4.4.1\platform\common\inc" -I"C:\Users\Halcyon\Desktop\CUSP24\ECEN5833\LPEDT_Software\gecko_sdk_4.4.1\hardware\board\inc" -I"C:\Users\Halcyon\Desktop\CUSP24\ECEN5833\LPEDT_Software\gecko_sdk_4.4.1\platform\CMSIS\Core\Include" -I"C:\Users\Halcyon\Desktop\CUSP24\ECEN5833\LPEDT_Software\gecko_sdk_4.4.1\hardware\driver\configuration_over_swo\inc" -I"C:\Users\Halcyon\Desktop\CUSP24\ECEN5833\LPEDT_Software\gecko_sdk_4.4.1\platform\driver\debug\inc" -I"C:\Users\Halcyon\Desktop\CUSP24\ECEN5833\LPEDT_Software\gecko_sdk_4.4.1\platform\service\device_init\inc" -I"C:\Users\Halcyon\Desktop\CUSP24\ECEN5833\LPEDT_Software\gecko_sdk_4.4.1\platform\emdrv\dmadrv\inc" -I"C:\Users\Halcyon\Desktop\CUSP24\ECEN5833\LPEDT_Software\gecko_sdk_4.4.1\platform\emdrv\common\inc" -I"C:\Users\Halcyon\Desktop\CUSP24\ECEN5833\LPEDT_Software\gecko_sdk_4.4.1\platform\emlib\inc" -I"C:\Users\Halcyon\Desktop\CUSP24\ECEN5833\LPEDT_Software\gecko_sdk_4.4.1\platform\driver\i2cspm\inc" -I"C:\Users\Halcyon\Desktop\CUSP24\ECEN5833\LPEDT_Software\gecko_sdk_4.4.1\platform\service\iostream\inc" -I"C:\Users\Halcyon\Desktop\CUSP24\ECEN5833\LPEDT_Software\gecko_sdk_4.4.1\platform\driver\leddrv\inc" -I"C:\Users\Halcyon\Desktop\CUSP24\ECEN5833\LPEDT_Software\gecko_sdk_4.4.1\hardware\driver\mx25_flash_shutdown\inc\sl_mx25_flash_shutdown_usart" -I"C:\Users\Halcyon\Desktop\CUSP24\ECEN5833\LPEDT_Software\gecko_sdk_4.4.1\platform\peripheral\inc" -I"C:\Users\Halcyon\Desktop\CUSP24\ECEN5833\LPEDT_Software\gecko_sdk_4.4.1\platform\common\toolchain\inc" -I"C:\Users\Halcyon\Desktop\CUSP24\ECEN5833\LPEDT_Software\gecko_sdk_4.4.1\platform\service\system\inc" -I"C:\Users\Halcyon\Desktop\CUSP24\ECEN5833\LPEDT_Software\gecko_sdk_4.4.1\platform\service\sleeptimer\inc" -I"C:\Users\Halcyon\Desktop\CUSP24\ECEN5833\LPEDT_Software\gecko_sdk_4.4.1\platform\service\udelay\inc" -Os -Wall -Wextra -ffunction-sections -fdata-sections -imacrossl_gcc_preinclude.h -mfpu=fpv5-sp-d16 -mfloat-abi=hard -mcmse --specs=nano.specs -c -fmessage-length=0 -MMD -MP -MF"autogen/sl_simple_led_instances.d" -MT"$@" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


