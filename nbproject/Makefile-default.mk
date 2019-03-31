#
# Generated Makefile - do not edit!
#
# Edit the Makefile in the project folder instead (../Makefile). Each target
# has a -pre and a -post target defined where you can add customized code.
#
# This makefile implements configuration specific macros and targets.


# Include project Makefile
ifeq "${IGNORE_LOCAL}" "TRUE"
# do not include local makefile. User is passing all local related variables already
else
include Makefile
# Include makefile containing local settings
ifeq "$(wildcard nbproject/Makefile-local-default.mk)" "nbproject/Makefile-local-default.mk"
include nbproject/Makefile-local-default.mk
endif
endif

# Environment
MKDIR=gnumkdir -p
RM=rm -f 
MV=mv 
CP=cp 

# Macros
CND_CONF=default
ifeq ($(TYPE_IMAGE), DEBUG_RUN)
IMAGE_TYPE=debug
OUTPUT_SUFFIX=elf
DEBUGGABLE_SUFFIX=elf
FINAL_IMAGE=dist/${CND_CONF}/${IMAGE_TYPE}/avr-iot-workshop.X.${IMAGE_TYPE}.${OUTPUT_SUFFIX}
else
IMAGE_TYPE=production
OUTPUT_SUFFIX=hex
DEBUGGABLE_SUFFIX=elf
FINAL_IMAGE=dist/${CND_CONF}/${IMAGE_TYPE}/avr-iot-workshop.X.${IMAGE_TYPE}.${OUTPUT_SUFFIX}
endif

ifeq ($(COMPARE_BUILD), true)
COMPARISON_BUILD=
else
COMPARISON_BUILD=
endif

ifdef SUB_IMAGE_ADDRESS

else
SUB_IMAGE_ADDRESS_COMMAND=
endif

# Object Directory
OBJECTDIR=build/${CND_CONF}/${IMAGE_TYPE}

# Distribution Directory
DISTDIR=dist/${CND_CONF}/${IMAGE_TYPE}

# Source Files Quoted if spaced
SOURCEFILES_QUOTED_IF_SPACED=mcc_generated_files/cli/cli.c mcc_generated_files/cloud/bsd_adapter/bsdWINC.c mcc_generated_files/cloud/crypto_client/crypto_client.c mcc_generated_files/cloud/crypto_client/cryptoauthlib_main.c mcc_generated_files/cloud/mqtt_packetPopulation/mqtt_packetPopulate.c mcc_generated_files/cloud/wifi_service.c mcc_generated_files/cloud/cloud_service.c mcc_generated_files/credentials_storage/credentials_storage.c mcc_generated_files/cryptoauthlib/lib/atcacert/atcacert_client.c mcc_generated_files/cryptoauthlib/lib/atcacert/atcacert_host_sw.c mcc_generated_files/cryptoauthlib/lib/atcacert/atcacert_def.c mcc_generated_files/cryptoauthlib/lib/atcacert/atcacert_date.c mcc_generated_files/cryptoauthlib/lib/atcacert/atcacert_host_hw.c mcc_generated_files/cryptoauthlib/lib/atcacert/atcacert_der.c mcc_generated_files/cryptoauthlib/lib/atcacert/atcacert_pem.c mcc_generated_files/cryptoauthlib/lib/basic/atca_basic_checkmac.c mcc_generated_files/cryptoauthlib/lib/basic/atca_basic_secureboot.c mcc_generated_files/cryptoauthlib/lib/basic/atca_basic_selftest.c mcc_generated_files/cryptoauthlib/lib/basic/atca_basic_ecdh.c mcc_generated_files/cryptoauthlib/lib/basic/atca_basic_updateextra.c mcc_generated_files/cryptoauthlib/lib/basic/atca_basic_privwrite.c mcc_generated_files/cryptoauthlib/lib/basic/atca_basic_info.c mcc_generated_files/cryptoauthlib/lib/basic/atca_basic.c mcc_generated_files/cryptoauthlib/lib/basic/atca_basic_read.c mcc_generated_files/cryptoauthlib/lib/basic/atca_basic_aes.c mcc_generated_files/cryptoauthlib/lib/basic/atca_basic_verify.c mcc_generated_files/cryptoauthlib/lib/basic/atca_basic_sha.c mcc_generated_files/cryptoauthlib/lib/basic/atca_basic_genkey.c mcc_generated_files/cryptoauthlib/lib/basic/atca_basic_nonce.c mcc_generated_files/cryptoauthlib/lib/basic/atca_basic_sign.c mcc_generated_files/cryptoauthlib/lib/basic/atca_basic_kdf.c mcc_generated_files/cryptoauthlib/lib/basic/atca_basic_hmac.c mcc_generated_files/cryptoauthlib/lib/basic/atca_basic_counter.c mcc_generated_files/cryptoauthlib/lib/basic/atca_basic_derivekey.c mcc_generated_files/cryptoauthlib/lib/basic/atca_helpers.c mcc_generated_files/cryptoauthlib/lib/basic/atca_basic_lock.c mcc_generated_files/cryptoauthlib/lib/basic/atca_basic_mac.c mcc_generated_files/cryptoauthlib/lib/basic/atca_basic_random.c mcc_generated_files/cryptoauthlib/lib/basic/atca_basic_gendig.c mcc_generated_files/cryptoauthlib/lib/basic/atca_basic_write.c mcc_generated_files/cryptoauthlib/lib/crypto/hashes/sha1_routines.c mcc_generated_files/cryptoauthlib/lib/crypto/hashes/sha2_routines.c mcc_generated_files/cryptoauthlib/lib/crypto/atca_crypto_sw_sha1.c mcc_generated_files/cryptoauthlib/lib/crypto/atca_crypto_sw_ecdsa.c mcc_generated_files/cryptoauthlib/lib/crypto/atca_crypto_sw_sha2.c mcc_generated_files/cryptoauthlib/lib/crypto/atca_crypto_sw_rand.c mcc_generated_files/cryptoauthlib/lib/hal/hal_timer.c mcc_generated_files/cryptoauthlib/lib/hal/hal_i2c.c mcc_generated_files/cryptoauthlib/lib/hal/atca_hal.c mcc_generated_files/cryptoauthlib/lib/host/atca_host.c mcc_generated_files/cryptoauthlib/lib/jwt/atca_jwt.c mcc_generated_files/cryptoauthlib/lib/tls/atcatls_cfg.c mcc_generated_files/cryptoauthlib/lib/tls/atcatls.c mcc_generated_files/cryptoauthlib/lib/atca_device.c mcc_generated_files/cryptoauthlib/lib/atca_execution.c mcc_generated_files/cryptoauthlib/lib/atca_command.c mcc_generated_files/cryptoauthlib/lib/atca_cfgs.c mcc_generated_files/cryptoauthlib/lib/atca_iface.c mcc_generated_files/drivers/i2c_simple_master.c mcc_generated_files/mqtt/mqtt_comm_bsd/mqtt_comm_layer.c mcc_generated_files/mqtt/mqtt_core/mqtt_core.c mcc_generated_files/mqtt/mqtt_exchange_buffer/mqtt_exchange_buffer.c mcc_generated_files/mqtt/mqtt_packetTransfer_interface.c mcc_generated_files/src/clkctrl.c mcc_generated_files/src/twi0.c mcc_generated_files/src/wdt.c mcc_generated_files/src/system.c mcc_generated_files/src/i2c_types.c mcc_generated_files/src/usart2.c mcc_generated_files/src/rtc.c mcc_generated_files/src/pin_manager.c mcc_generated_files/src/cpuint.c mcc_generated_files/src/spi0.c mcc_generated_files/src/adc0.c mcc_generated_files/src/protected_io.S mcc_generated_files/src/slpctrl.c mcc_generated_files/src/bod.c mcc_generated_files/winc/bsp/source/nm_bsp_mega.c mcc_generated_files/winc/bus_wrapper/source/nm_bus_wrapper_mega.c mcc_generated_files/winc/common/source/nm_common.c mcc_generated_files/winc/driver/source/m2m_ate_mode.c mcc_generated_files/winc/driver/source/m2m_ssl.c mcc_generated_files/winc/driver/source/m2m_ota.c mcc_generated_files/winc/driver/source/nmspi.c mcc_generated_files/winc/driver/source/m2m_hif.c mcc_generated_files/winc/driver/source/m2m_periph.c mcc_generated_files/winc/driver/source/nmasic.c mcc_generated_files/winc/driver/source/nmdrv.c mcc_generated_files/winc/driver/source/m2m_wifi.c mcc_generated_files/winc/driver/source/m2m_crypto.c mcc_generated_files/winc/driver/source/nmi2c.c mcc_generated_files/winc/driver/source/nmuart.c mcc_generated_files/winc/driver/source/nmbus.c mcc_generated_files/winc/socket/source/socket.c mcc_generated_files/winc/spi_flash/source/spi_flash.c mcc_generated_files/mcc.c mcc_generated_files/application_manager.c mcc_generated_files/driver_isr.c mcc_generated_files/sensors_handling.c mcc_generated_files/device_config.c mcc_generated_files/led.c mcc_generated_files/debug_print.c main.c

# Object Files Quoted if spaced
OBJECTFILES_QUOTED_IF_SPACED=${OBJECTDIR}/mcc_generated_files/cli/cli.o ${OBJECTDIR}/mcc_generated_files/cloud/bsd_adapter/bsdWINC.o ${OBJECTDIR}/mcc_generated_files/cloud/crypto_client/crypto_client.o ${OBJECTDIR}/mcc_generated_files/cloud/crypto_client/cryptoauthlib_main.o ${OBJECTDIR}/mcc_generated_files/cloud/mqtt_packetPopulation/mqtt_packetPopulate.o ${OBJECTDIR}/mcc_generated_files/cloud/wifi_service.o ${OBJECTDIR}/mcc_generated_files/cloud/cloud_service.o ${OBJECTDIR}/mcc_generated_files/credentials_storage/credentials_storage.o ${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/atcacert/atcacert_client.o ${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/atcacert/atcacert_host_sw.o ${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/atcacert/atcacert_def.o ${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/atcacert/atcacert_date.o ${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/atcacert/atcacert_host_hw.o ${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/atcacert/atcacert_der.o ${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/atcacert/atcacert_pem.o ${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/basic/atca_basic_checkmac.o ${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/basic/atca_basic_secureboot.o ${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/basic/atca_basic_selftest.o ${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/basic/atca_basic_ecdh.o ${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/basic/atca_basic_updateextra.o ${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/basic/atca_basic_privwrite.o ${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/basic/atca_basic_info.o ${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/basic/atca_basic.o ${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/basic/atca_basic_read.o ${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/basic/atca_basic_aes.o ${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/basic/atca_basic_verify.o ${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/basic/atca_basic_sha.o ${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/basic/atca_basic_genkey.o ${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/basic/atca_basic_nonce.o ${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/basic/atca_basic_sign.o ${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/basic/atca_basic_kdf.o ${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/basic/atca_basic_hmac.o ${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/basic/atca_basic_counter.o ${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/basic/atca_basic_derivekey.o ${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/basic/atca_helpers.o ${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/basic/atca_basic_lock.o ${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/basic/atca_basic_mac.o ${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/basic/atca_basic_random.o ${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/basic/atca_basic_gendig.o ${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/basic/atca_basic_write.o ${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/crypto/hashes/sha1_routines.o ${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/crypto/hashes/sha2_routines.o ${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/crypto/atca_crypto_sw_sha1.o ${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/crypto/atca_crypto_sw_ecdsa.o ${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/crypto/atca_crypto_sw_sha2.o ${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/crypto/atca_crypto_sw_rand.o ${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/hal/hal_timer.o ${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/hal/hal_i2c.o ${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/hal/atca_hal.o ${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/host/atca_host.o ${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/jwt/atca_jwt.o ${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/tls/atcatls_cfg.o ${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/tls/atcatls.o ${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/atca_device.o ${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/atca_execution.o ${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/atca_command.o ${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/atca_cfgs.o ${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/atca_iface.o ${OBJECTDIR}/mcc_generated_files/drivers/i2c_simple_master.o ${OBJECTDIR}/mcc_generated_files/mqtt/mqtt_comm_bsd/mqtt_comm_layer.o ${OBJECTDIR}/mcc_generated_files/mqtt/mqtt_core/mqtt_core.o ${OBJECTDIR}/mcc_generated_files/mqtt/mqtt_exchange_buffer/mqtt_exchange_buffer.o ${OBJECTDIR}/mcc_generated_files/mqtt/mqtt_packetTransfer_interface.o ${OBJECTDIR}/mcc_generated_files/src/clkctrl.o ${OBJECTDIR}/mcc_generated_files/src/twi0.o ${OBJECTDIR}/mcc_generated_files/src/wdt.o ${OBJECTDIR}/mcc_generated_files/src/system.o ${OBJECTDIR}/mcc_generated_files/src/i2c_types.o ${OBJECTDIR}/mcc_generated_files/src/usart2.o ${OBJECTDIR}/mcc_generated_files/src/rtc.o ${OBJECTDIR}/mcc_generated_files/src/pin_manager.o ${OBJECTDIR}/mcc_generated_files/src/cpuint.o ${OBJECTDIR}/mcc_generated_files/src/spi0.o ${OBJECTDIR}/mcc_generated_files/src/adc0.o ${OBJECTDIR}/mcc_generated_files/src/protected_io.o ${OBJECTDIR}/mcc_generated_files/src/slpctrl.o ${OBJECTDIR}/mcc_generated_files/src/bod.o ${OBJECTDIR}/mcc_generated_files/winc/bsp/source/nm_bsp_mega.o ${OBJECTDIR}/mcc_generated_files/winc/bus_wrapper/source/nm_bus_wrapper_mega.o ${OBJECTDIR}/mcc_generated_files/winc/common/source/nm_common.o ${OBJECTDIR}/mcc_generated_files/winc/driver/source/m2m_ate_mode.o ${OBJECTDIR}/mcc_generated_files/winc/driver/source/m2m_ssl.o ${OBJECTDIR}/mcc_generated_files/winc/driver/source/m2m_ota.o ${OBJECTDIR}/mcc_generated_files/winc/driver/source/nmspi.o ${OBJECTDIR}/mcc_generated_files/winc/driver/source/m2m_hif.o ${OBJECTDIR}/mcc_generated_files/winc/driver/source/m2m_periph.o ${OBJECTDIR}/mcc_generated_files/winc/driver/source/nmasic.o ${OBJECTDIR}/mcc_generated_files/winc/driver/source/nmdrv.o ${OBJECTDIR}/mcc_generated_files/winc/driver/source/m2m_wifi.o ${OBJECTDIR}/mcc_generated_files/winc/driver/source/m2m_crypto.o ${OBJECTDIR}/mcc_generated_files/winc/driver/source/nmi2c.o ${OBJECTDIR}/mcc_generated_files/winc/driver/source/nmuart.o ${OBJECTDIR}/mcc_generated_files/winc/driver/source/nmbus.o ${OBJECTDIR}/mcc_generated_files/winc/socket/source/socket.o ${OBJECTDIR}/mcc_generated_files/winc/spi_flash/source/spi_flash.o ${OBJECTDIR}/mcc_generated_files/mcc.o ${OBJECTDIR}/mcc_generated_files/application_manager.o ${OBJECTDIR}/mcc_generated_files/driver_isr.o ${OBJECTDIR}/mcc_generated_files/sensors_handling.o ${OBJECTDIR}/mcc_generated_files/device_config.o ${OBJECTDIR}/mcc_generated_files/led.o ${OBJECTDIR}/mcc_generated_files/debug_print.o ${OBJECTDIR}/main.o
POSSIBLE_DEPFILES=${OBJECTDIR}/mcc_generated_files/cli/cli.o.d ${OBJECTDIR}/mcc_generated_files/cloud/bsd_adapter/bsdWINC.o.d ${OBJECTDIR}/mcc_generated_files/cloud/crypto_client/crypto_client.o.d ${OBJECTDIR}/mcc_generated_files/cloud/crypto_client/cryptoauthlib_main.o.d ${OBJECTDIR}/mcc_generated_files/cloud/mqtt_packetPopulation/mqtt_packetPopulate.o.d ${OBJECTDIR}/mcc_generated_files/cloud/wifi_service.o.d ${OBJECTDIR}/mcc_generated_files/cloud/cloud_service.o.d ${OBJECTDIR}/mcc_generated_files/credentials_storage/credentials_storage.o.d ${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/atcacert/atcacert_client.o.d ${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/atcacert/atcacert_host_sw.o.d ${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/atcacert/atcacert_def.o.d ${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/atcacert/atcacert_date.o.d ${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/atcacert/atcacert_host_hw.o.d ${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/atcacert/atcacert_der.o.d ${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/atcacert/atcacert_pem.o.d ${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/basic/atca_basic_checkmac.o.d ${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/basic/atca_basic_secureboot.o.d ${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/basic/atca_basic_selftest.o.d ${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/basic/atca_basic_ecdh.o.d ${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/basic/atca_basic_updateextra.o.d ${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/basic/atca_basic_privwrite.o.d ${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/basic/atca_basic_info.o.d ${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/basic/atca_basic.o.d ${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/basic/atca_basic_read.o.d ${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/basic/atca_basic_aes.o.d ${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/basic/atca_basic_verify.o.d ${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/basic/atca_basic_sha.o.d ${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/basic/atca_basic_genkey.o.d ${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/basic/atca_basic_nonce.o.d ${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/basic/atca_basic_sign.o.d ${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/basic/atca_basic_kdf.o.d ${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/basic/atca_basic_hmac.o.d ${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/basic/atca_basic_counter.o.d ${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/basic/atca_basic_derivekey.o.d ${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/basic/atca_helpers.o.d ${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/basic/atca_basic_lock.o.d ${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/basic/atca_basic_mac.o.d ${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/basic/atca_basic_random.o.d ${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/basic/atca_basic_gendig.o.d ${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/basic/atca_basic_write.o.d ${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/crypto/hashes/sha1_routines.o.d ${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/crypto/hashes/sha2_routines.o.d ${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/crypto/atca_crypto_sw_sha1.o.d ${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/crypto/atca_crypto_sw_ecdsa.o.d ${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/crypto/atca_crypto_sw_sha2.o.d ${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/crypto/atca_crypto_sw_rand.o.d ${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/hal/hal_timer.o.d ${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/hal/hal_i2c.o.d ${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/hal/atca_hal.o.d ${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/host/atca_host.o.d ${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/jwt/atca_jwt.o.d ${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/tls/atcatls_cfg.o.d ${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/tls/atcatls.o.d ${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/atca_device.o.d ${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/atca_execution.o.d ${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/atca_command.o.d ${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/atca_cfgs.o.d ${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/atca_iface.o.d ${OBJECTDIR}/mcc_generated_files/drivers/i2c_simple_master.o.d ${OBJECTDIR}/mcc_generated_files/mqtt/mqtt_comm_bsd/mqtt_comm_layer.o.d ${OBJECTDIR}/mcc_generated_files/mqtt/mqtt_core/mqtt_core.o.d ${OBJECTDIR}/mcc_generated_files/mqtt/mqtt_exchange_buffer/mqtt_exchange_buffer.o.d ${OBJECTDIR}/mcc_generated_files/mqtt/mqtt_packetTransfer_interface.o.d ${OBJECTDIR}/mcc_generated_files/src/clkctrl.o.d ${OBJECTDIR}/mcc_generated_files/src/twi0.o.d ${OBJECTDIR}/mcc_generated_files/src/wdt.o.d ${OBJECTDIR}/mcc_generated_files/src/system.o.d ${OBJECTDIR}/mcc_generated_files/src/i2c_types.o.d ${OBJECTDIR}/mcc_generated_files/src/usart2.o.d ${OBJECTDIR}/mcc_generated_files/src/rtc.o.d ${OBJECTDIR}/mcc_generated_files/src/pin_manager.o.d ${OBJECTDIR}/mcc_generated_files/src/cpuint.o.d ${OBJECTDIR}/mcc_generated_files/src/spi0.o.d ${OBJECTDIR}/mcc_generated_files/src/adc0.o.d ${OBJECTDIR}/mcc_generated_files/src/protected_io.o.d ${OBJECTDIR}/mcc_generated_files/src/slpctrl.o.d ${OBJECTDIR}/mcc_generated_files/src/bod.o.d ${OBJECTDIR}/mcc_generated_files/winc/bsp/source/nm_bsp_mega.o.d ${OBJECTDIR}/mcc_generated_files/winc/bus_wrapper/source/nm_bus_wrapper_mega.o.d ${OBJECTDIR}/mcc_generated_files/winc/common/source/nm_common.o.d ${OBJECTDIR}/mcc_generated_files/winc/driver/source/m2m_ate_mode.o.d ${OBJECTDIR}/mcc_generated_files/winc/driver/source/m2m_ssl.o.d ${OBJECTDIR}/mcc_generated_files/winc/driver/source/m2m_ota.o.d ${OBJECTDIR}/mcc_generated_files/winc/driver/source/nmspi.o.d ${OBJECTDIR}/mcc_generated_files/winc/driver/source/m2m_hif.o.d ${OBJECTDIR}/mcc_generated_files/winc/driver/source/m2m_periph.o.d ${OBJECTDIR}/mcc_generated_files/winc/driver/source/nmasic.o.d ${OBJECTDIR}/mcc_generated_files/winc/driver/source/nmdrv.o.d ${OBJECTDIR}/mcc_generated_files/winc/driver/source/m2m_wifi.o.d ${OBJECTDIR}/mcc_generated_files/winc/driver/source/m2m_crypto.o.d ${OBJECTDIR}/mcc_generated_files/winc/driver/source/nmi2c.o.d ${OBJECTDIR}/mcc_generated_files/winc/driver/source/nmuart.o.d ${OBJECTDIR}/mcc_generated_files/winc/driver/source/nmbus.o.d ${OBJECTDIR}/mcc_generated_files/winc/socket/source/socket.o.d ${OBJECTDIR}/mcc_generated_files/winc/spi_flash/source/spi_flash.o.d ${OBJECTDIR}/mcc_generated_files/mcc.o.d ${OBJECTDIR}/mcc_generated_files/application_manager.o.d ${OBJECTDIR}/mcc_generated_files/driver_isr.o.d ${OBJECTDIR}/mcc_generated_files/sensors_handling.o.d ${OBJECTDIR}/mcc_generated_files/device_config.o.d ${OBJECTDIR}/mcc_generated_files/led.o.d ${OBJECTDIR}/mcc_generated_files/debug_print.o.d ${OBJECTDIR}/main.o.d

# Object Files
OBJECTFILES=${OBJECTDIR}/mcc_generated_files/cli/cli.o ${OBJECTDIR}/mcc_generated_files/cloud/bsd_adapter/bsdWINC.o ${OBJECTDIR}/mcc_generated_files/cloud/crypto_client/crypto_client.o ${OBJECTDIR}/mcc_generated_files/cloud/crypto_client/cryptoauthlib_main.o ${OBJECTDIR}/mcc_generated_files/cloud/mqtt_packetPopulation/mqtt_packetPopulate.o ${OBJECTDIR}/mcc_generated_files/cloud/wifi_service.o ${OBJECTDIR}/mcc_generated_files/cloud/cloud_service.o ${OBJECTDIR}/mcc_generated_files/credentials_storage/credentials_storage.o ${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/atcacert/atcacert_client.o ${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/atcacert/atcacert_host_sw.o ${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/atcacert/atcacert_def.o ${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/atcacert/atcacert_date.o ${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/atcacert/atcacert_host_hw.o ${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/atcacert/atcacert_der.o ${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/atcacert/atcacert_pem.o ${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/basic/atca_basic_checkmac.o ${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/basic/atca_basic_secureboot.o ${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/basic/atca_basic_selftest.o ${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/basic/atca_basic_ecdh.o ${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/basic/atca_basic_updateextra.o ${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/basic/atca_basic_privwrite.o ${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/basic/atca_basic_info.o ${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/basic/atca_basic.o ${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/basic/atca_basic_read.o ${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/basic/atca_basic_aes.o ${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/basic/atca_basic_verify.o ${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/basic/atca_basic_sha.o ${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/basic/atca_basic_genkey.o ${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/basic/atca_basic_nonce.o ${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/basic/atca_basic_sign.o ${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/basic/atca_basic_kdf.o ${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/basic/atca_basic_hmac.o ${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/basic/atca_basic_counter.o ${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/basic/atca_basic_derivekey.o ${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/basic/atca_helpers.o ${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/basic/atca_basic_lock.o ${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/basic/atca_basic_mac.o ${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/basic/atca_basic_random.o ${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/basic/atca_basic_gendig.o ${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/basic/atca_basic_write.o ${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/crypto/hashes/sha1_routines.o ${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/crypto/hashes/sha2_routines.o ${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/crypto/atca_crypto_sw_sha1.o ${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/crypto/atca_crypto_sw_ecdsa.o ${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/crypto/atca_crypto_sw_sha2.o ${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/crypto/atca_crypto_sw_rand.o ${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/hal/hal_timer.o ${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/hal/hal_i2c.o ${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/hal/atca_hal.o ${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/host/atca_host.o ${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/jwt/atca_jwt.o ${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/tls/atcatls_cfg.o ${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/tls/atcatls.o ${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/atca_device.o ${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/atca_execution.o ${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/atca_command.o ${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/atca_cfgs.o ${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/atca_iface.o ${OBJECTDIR}/mcc_generated_files/drivers/i2c_simple_master.o ${OBJECTDIR}/mcc_generated_files/mqtt/mqtt_comm_bsd/mqtt_comm_layer.o ${OBJECTDIR}/mcc_generated_files/mqtt/mqtt_core/mqtt_core.o ${OBJECTDIR}/mcc_generated_files/mqtt/mqtt_exchange_buffer/mqtt_exchange_buffer.o ${OBJECTDIR}/mcc_generated_files/mqtt/mqtt_packetTransfer_interface.o ${OBJECTDIR}/mcc_generated_files/src/clkctrl.o ${OBJECTDIR}/mcc_generated_files/src/twi0.o ${OBJECTDIR}/mcc_generated_files/src/wdt.o ${OBJECTDIR}/mcc_generated_files/src/system.o ${OBJECTDIR}/mcc_generated_files/src/i2c_types.o ${OBJECTDIR}/mcc_generated_files/src/usart2.o ${OBJECTDIR}/mcc_generated_files/src/rtc.o ${OBJECTDIR}/mcc_generated_files/src/pin_manager.o ${OBJECTDIR}/mcc_generated_files/src/cpuint.o ${OBJECTDIR}/mcc_generated_files/src/spi0.o ${OBJECTDIR}/mcc_generated_files/src/adc0.o ${OBJECTDIR}/mcc_generated_files/src/protected_io.o ${OBJECTDIR}/mcc_generated_files/src/slpctrl.o ${OBJECTDIR}/mcc_generated_files/src/bod.o ${OBJECTDIR}/mcc_generated_files/winc/bsp/source/nm_bsp_mega.o ${OBJECTDIR}/mcc_generated_files/winc/bus_wrapper/source/nm_bus_wrapper_mega.o ${OBJECTDIR}/mcc_generated_files/winc/common/source/nm_common.o ${OBJECTDIR}/mcc_generated_files/winc/driver/source/m2m_ate_mode.o ${OBJECTDIR}/mcc_generated_files/winc/driver/source/m2m_ssl.o ${OBJECTDIR}/mcc_generated_files/winc/driver/source/m2m_ota.o ${OBJECTDIR}/mcc_generated_files/winc/driver/source/nmspi.o ${OBJECTDIR}/mcc_generated_files/winc/driver/source/m2m_hif.o ${OBJECTDIR}/mcc_generated_files/winc/driver/source/m2m_periph.o ${OBJECTDIR}/mcc_generated_files/winc/driver/source/nmasic.o ${OBJECTDIR}/mcc_generated_files/winc/driver/source/nmdrv.o ${OBJECTDIR}/mcc_generated_files/winc/driver/source/m2m_wifi.o ${OBJECTDIR}/mcc_generated_files/winc/driver/source/m2m_crypto.o ${OBJECTDIR}/mcc_generated_files/winc/driver/source/nmi2c.o ${OBJECTDIR}/mcc_generated_files/winc/driver/source/nmuart.o ${OBJECTDIR}/mcc_generated_files/winc/driver/source/nmbus.o ${OBJECTDIR}/mcc_generated_files/winc/socket/source/socket.o ${OBJECTDIR}/mcc_generated_files/winc/spi_flash/source/spi_flash.o ${OBJECTDIR}/mcc_generated_files/mcc.o ${OBJECTDIR}/mcc_generated_files/application_manager.o ${OBJECTDIR}/mcc_generated_files/driver_isr.o ${OBJECTDIR}/mcc_generated_files/sensors_handling.o ${OBJECTDIR}/mcc_generated_files/device_config.o ${OBJECTDIR}/mcc_generated_files/led.o ${OBJECTDIR}/mcc_generated_files/debug_print.o ${OBJECTDIR}/main.o

# Source Files
SOURCEFILES=mcc_generated_files/cli/cli.c mcc_generated_files/cloud/bsd_adapter/bsdWINC.c mcc_generated_files/cloud/crypto_client/crypto_client.c mcc_generated_files/cloud/crypto_client/cryptoauthlib_main.c mcc_generated_files/cloud/mqtt_packetPopulation/mqtt_packetPopulate.c mcc_generated_files/cloud/wifi_service.c mcc_generated_files/cloud/cloud_service.c mcc_generated_files/credentials_storage/credentials_storage.c mcc_generated_files/cryptoauthlib/lib/atcacert/atcacert_client.c mcc_generated_files/cryptoauthlib/lib/atcacert/atcacert_host_sw.c mcc_generated_files/cryptoauthlib/lib/atcacert/atcacert_def.c mcc_generated_files/cryptoauthlib/lib/atcacert/atcacert_date.c mcc_generated_files/cryptoauthlib/lib/atcacert/atcacert_host_hw.c mcc_generated_files/cryptoauthlib/lib/atcacert/atcacert_der.c mcc_generated_files/cryptoauthlib/lib/atcacert/atcacert_pem.c mcc_generated_files/cryptoauthlib/lib/basic/atca_basic_checkmac.c mcc_generated_files/cryptoauthlib/lib/basic/atca_basic_secureboot.c mcc_generated_files/cryptoauthlib/lib/basic/atca_basic_selftest.c mcc_generated_files/cryptoauthlib/lib/basic/atca_basic_ecdh.c mcc_generated_files/cryptoauthlib/lib/basic/atca_basic_updateextra.c mcc_generated_files/cryptoauthlib/lib/basic/atca_basic_privwrite.c mcc_generated_files/cryptoauthlib/lib/basic/atca_basic_info.c mcc_generated_files/cryptoauthlib/lib/basic/atca_basic.c mcc_generated_files/cryptoauthlib/lib/basic/atca_basic_read.c mcc_generated_files/cryptoauthlib/lib/basic/atca_basic_aes.c mcc_generated_files/cryptoauthlib/lib/basic/atca_basic_verify.c mcc_generated_files/cryptoauthlib/lib/basic/atca_basic_sha.c mcc_generated_files/cryptoauthlib/lib/basic/atca_basic_genkey.c mcc_generated_files/cryptoauthlib/lib/basic/atca_basic_nonce.c mcc_generated_files/cryptoauthlib/lib/basic/atca_basic_sign.c mcc_generated_files/cryptoauthlib/lib/basic/atca_basic_kdf.c mcc_generated_files/cryptoauthlib/lib/basic/atca_basic_hmac.c mcc_generated_files/cryptoauthlib/lib/basic/atca_basic_counter.c mcc_generated_files/cryptoauthlib/lib/basic/atca_basic_derivekey.c mcc_generated_files/cryptoauthlib/lib/basic/atca_helpers.c mcc_generated_files/cryptoauthlib/lib/basic/atca_basic_lock.c mcc_generated_files/cryptoauthlib/lib/basic/atca_basic_mac.c mcc_generated_files/cryptoauthlib/lib/basic/atca_basic_random.c mcc_generated_files/cryptoauthlib/lib/basic/atca_basic_gendig.c mcc_generated_files/cryptoauthlib/lib/basic/atca_basic_write.c mcc_generated_files/cryptoauthlib/lib/crypto/hashes/sha1_routines.c mcc_generated_files/cryptoauthlib/lib/crypto/hashes/sha2_routines.c mcc_generated_files/cryptoauthlib/lib/crypto/atca_crypto_sw_sha1.c mcc_generated_files/cryptoauthlib/lib/crypto/atca_crypto_sw_ecdsa.c mcc_generated_files/cryptoauthlib/lib/crypto/atca_crypto_sw_sha2.c mcc_generated_files/cryptoauthlib/lib/crypto/atca_crypto_sw_rand.c mcc_generated_files/cryptoauthlib/lib/hal/hal_timer.c mcc_generated_files/cryptoauthlib/lib/hal/hal_i2c.c mcc_generated_files/cryptoauthlib/lib/hal/atca_hal.c mcc_generated_files/cryptoauthlib/lib/host/atca_host.c mcc_generated_files/cryptoauthlib/lib/jwt/atca_jwt.c mcc_generated_files/cryptoauthlib/lib/tls/atcatls_cfg.c mcc_generated_files/cryptoauthlib/lib/tls/atcatls.c mcc_generated_files/cryptoauthlib/lib/atca_device.c mcc_generated_files/cryptoauthlib/lib/atca_execution.c mcc_generated_files/cryptoauthlib/lib/atca_command.c mcc_generated_files/cryptoauthlib/lib/atca_cfgs.c mcc_generated_files/cryptoauthlib/lib/atca_iface.c mcc_generated_files/drivers/i2c_simple_master.c mcc_generated_files/mqtt/mqtt_comm_bsd/mqtt_comm_layer.c mcc_generated_files/mqtt/mqtt_core/mqtt_core.c mcc_generated_files/mqtt/mqtt_exchange_buffer/mqtt_exchange_buffer.c mcc_generated_files/mqtt/mqtt_packetTransfer_interface.c mcc_generated_files/src/clkctrl.c mcc_generated_files/src/twi0.c mcc_generated_files/src/wdt.c mcc_generated_files/src/system.c mcc_generated_files/src/i2c_types.c mcc_generated_files/src/usart2.c mcc_generated_files/src/rtc.c mcc_generated_files/src/pin_manager.c mcc_generated_files/src/cpuint.c mcc_generated_files/src/spi0.c mcc_generated_files/src/adc0.c mcc_generated_files/src/protected_io.S mcc_generated_files/src/slpctrl.c mcc_generated_files/src/bod.c mcc_generated_files/winc/bsp/source/nm_bsp_mega.c mcc_generated_files/winc/bus_wrapper/source/nm_bus_wrapper_mega.c mcc_generated_files/winc/common/source/nm_common.c mcc_generated_files/winc/driver/source/m2m_ate_mode.c mcc_generated_files/winc/driver/source/m2m_ssl.c mcc_generated_files/winc/driver/source/m2m_ota.c mcc_generated_files/winc/driver/source/nmspi.c mcc_generated_files/winc/driver/source/m2m_hif.c mcc_generated_files/winc/driver/source/m2m_periph.c mcc_generated_files/winc/driver/source/nmasic.c mcc_generated_files/winc/driver/source/nmdrv.c mcc_generated_files/winc/driver/source/m2m_wifi.c mcc_generated_files/winc/driver/source/m2m_crypto.c mcc_generated_files/winc/driver/source/nmi2c.c mcc_generated_files/winc/driver/source/nmuart.c mcc_generated_files/winc/driver/source/nmbus.c mcc_generated_files/winc/socket/source/socket.c mcc_generated_files/winc/spi_flash/source/spi_flash.c mcc_generated_files/mcc.c mcc_generated_files/application_manager.c mcc_generated_files/driver_isr.c mcc_generated_files/sensors_handling.c mcc_generated_files/device_config.c mcc_generated_files/led.c mcc_generated_files/debug_print.c main.c


CFLAGS=
ASFLAGS=
LDLIBSOPTIONS=

############# Tool locations ##########################################
# If you copy a project from one host to another, the path where the  #
# compiler is installed may be different.                             #
# If you open this project with MPLAB X in the new host, this         #
# makefile will be regenerated and the paths will be corrected.       #
#######################################################################
# fixDeps replaces a bunch of sed/cat/printf statements that slow down the build
FIXDEPS=fixDeps

.build-conf:  ${BUILD_SUBPROJECTS}
ifneq ($(INFORMATION_MESSAGE), )
	@echo $(INFORMATION_MESSAGE)
endif
	${MAKE}  -f nbproject/Makefile-default.mk dist/${CND_CONF}/${IMAGE_TYPE}/avr-iot-workshop.X.${IMAGE_TYPE}.${OUTPUT_SUFFIX}

MP_PROCESSOR_OPTION=ATmega4808
# ------------------------------------------------------------------------------------
# Rules for buildStep: assemble
ifeq ($(TYPE_IMAGE), DEBUG_RUN)
else
endif

# ------------------------------------------------------------------------------------
# Rules for buildStep: assembleWithPreprocess
ifeq ($(TYPE_IMAGE), DEBUG_RUN)
${OBJECTDIR}/mcc_generated_files/src/protected_io.o: mcc_generated_files/src/protected_io.S  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/mcc_generated_files/src" 
	@${RM} ${OBJECTDIR}/mcc_generated_files/src/protected_io.o.d 
	@${RM} ${OBJECTDIR}/mcc_generated_files/src/protected_io.o 
	@${RM} ${OBJECTDIR}/mcc_generated_files/src/protected_io.o.ok ${OBJECTDIR}/mcc_generated_files/src/protected_io.o.err 
	 ${MP_CC} $(MP_EXTRA_AS_PRE) -mmcu=atmega4808  -I "C:\Program Files (x86)\Microchip\MPLABX\v5.15\packs\Microchip\ATmega_DFP\2.0.2\include"  -B "C:\Program Files (x86)\Microchip\MPLABX\v5.15\packs\Microchip\ATmega_DFP\2.0.2\gcc\dev\atmega4808"  -DDEBUG  -x assembler-with-cpp -c -D__$(MP_PROCESSOR_OPTION)__  -MD -MP -MF "${OBJECTDIR}/mcc_generated_files/src/protected_io.o.d" -MT "${OBJECTDIR}/mcc_generated_files/src/protected_io.o.d" -MT ${OBJECTDIR}/mcc_generated_files/src/protected_io.o -o ${OBJECTDIR}/mcc_generated_files/src/protected_io.o mcc_generated_files/src/protected_io.S  -DXPRJ_default=$(CND_CONF)  -Wa,--defsym=__MPLAB_BUILD=1$(MP_EXTRA_AS_POST),-MD="${OBJECTDIR}/mcc_generated_files/src/protected_io.o.asm.d",--defsym=__ICD2RAM=1,--defsym=__MPLAB_DEBUG=1,--gdwarf-2,--defsym=__DEBUG=1
	
else
${OBJECTDIR}/mcc_generated_files/src/protected_io.o: mcc_generated_files/src/protected_io.S  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/mcc_generated_files/src" 
	@${RM} ${OBJECTDIR}/mcc_generated_files/src/protected_io.o.d 
	@${RM} ${OBJECTDIR}/mcc_generated_files/src/protected_io.o 
	@${RM} ${OBJECTDIR}/mcc_generated_files/src/protected_io.o.ok ${OBJECTDIR}/mcc_generated_files/src/protected_io.o.err 
	 ${MP_CC} $(MP_EXTRA_AS_PRE) -mmcu=atmega4808  -I "C:\Program Files (x86)\Microchip\MPLABX\v5.15\packs\Microchip\ATmega_DFP\2.0.2\include"  -B "C:\Program Files (x86)\Microchip\MPLABX\v5.15\packs\Microchip\ATmega_DFP\2.0.2\gcc\dev\atmega4808"  -x assembler-with-cpp -c -D__$(MP_PROCESSOR_OPTION)__  -MD -MP -MF "${OBJECTDIR}/mcc_generated_files/src/protected_io.o.d" -MT "${OBJECTDIR}/mcc_generated_files/src/protected_io.o.d" -MT ${OBJECTDIR}/mcc_generated_files/src/protected_io.o -o ${OBJECTDIR}/mcc_generated_files/src/protected_io.o mcc_generated_files/src/protected_io.S  -DXPRJ_default=$(CND_CONF)  -Wa,--defsym=__MPLAB_BUILD=1$(MP_EXTRA_AS_POST),-MD="${OBJECTDIR}/mcc_generated_files/src/protected_io.o.asm.d"
	
endif

# ------------------------------------------------------------------------------------
# Rules for buildStep: compile
ifeq ($(TYPE_IMAGE), DEBUG_RUN)
${OBJECTDIR}/mcc_generated_files/cli/cli.o: mcc_generated_files/cli/cli.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/mcc_generated_files/cli" 
	@${RM} ${OBJECTDIR}/mcc_generated_files/cli/cli.o.d 
	@${RM} ${OBJECTDIR}/mcc_generated_files/cli/cli.o 
	 ${MP_CC}  $(MP_EXTRA_CC_PRE) -mmcu=atmega4808  -I "C:\Program Files (x86)\Microchip\MPLABX\v5.15\packs\Microchip\ATmega_DFP\2.0.2\include"  -B "C:\Program Files (x86)\Microchip\MPLABX\v5.15\packs\Microchip\ATmega_DFP\2.0.2\gcc\dev\atmega4808" -g -DDEBUG  -gdwarf-2  -x c -c -D__$(MP_PROCESSOR_OPTION)__  -funsigned-char -funsigned-bitfields -Os -ffunction-sections -fdata-sections -fpack-struct -fshort-enums -Wall -MD -MP -MF "${OBJECTDIR}/mcc_generated_files/cli/cli.o.d" -MT "${OBJECTDIR}/mcc_generated_files/cli/cli.o.d" -MT ${OBJECTDIR}/mcc_generated_files/cli/cli.o  -o ${OBJECTDIR}/mcc_generated_files/cli/cli.o mcc_generated_files/cli/cli.c  -DXPRJ_default=$(CND_CONF)  $(COMPARISON_BUILD) 
	
${OBJECTDIR}/mcc_generated_files/cloud/bsd_adapter/bsdWINC.o: mcc_generated_files/cloud/bsd_adapter/bsdWINC.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/mcc_generated_files/cloud/bsd_adapter" 
	@${RM} ${OBJECTDIR}/mcc_generated_files/cloud/bsd_adapter/bsdWINC.o.d 
	@${RM} ${OBJECTDIR}/mcc_generated_files/cloud/bsd_adapter/bsdWINC.o 
	 ${MP_CC}  $(MP_EXTRA_CC_PRE) -mmcu=atmega4808  -I "C:\Program Files (x86)\Microchip\MPLABX\v5.15\packs\Microchip\ATmega_DFP\2.0.2\include"  -B "C:\Program Files (x86)\Microchip\MPLABX\v5.15\packs\Microchip\ATmega_DFP\2.0.2\gcc\dev\atmega4808" -g -DDEBUG  -gdwarf-2  -x c -c -D__$(MP_PROCESSOR_OPTION)__  -funsigned-char -funsigned-bitfields -Os -ffunction-sections -fdata-sections -fpack-struct -fshort-enums -Wall -MD -MP -MF "${OBJECTDIR}/mcc_generated_files/cloud/bsd_adapter/bsdWINC.o.d" -MT "${OBJECTDIR}/mcc_generated_files/cloud/bsd_adapter/bsdWINC.o.d" -MT ${OBJECTDIR}/mcc_generated_files/cloud/bsd_adapter/bsdWINC.o  -o ${OBJECTDIR}/mcc_generated_files/cloud/bsd_adapter/bsdWINC.o mcc_generated_files/cloud/bsd_adapter/bsdWINC.c  -DXPRJ_default=$(CND_CONF)  $(COMPARISON_BUILD) 
	
${OBJECTDIR}/mcc_generated_files/cloud/crypto_client/crypto_client.o: mcc_generated_files/cloud/crypto_client/crypto_client.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/mcc_generated_files/cloud/crypto_client" 
	@${RM} ${OBJECTDIR}/mcc_generated_files/cloud/crypto_client/crypto_client.o.d 
	@${RM} ${OBJECTDIR}/mcc_generated_files/cloud/crypto_client/crypto_client.o 
	 ${MP_CC}  $(MP_EXTRA_CC_PRE) -mmcu=atmega4808  -I "C:\Program Files (x86)\Microchip\MPLABX\v5.15\packs\Microchip\ATmega_DFP\2.0.2\include"  -B "C:\Program Files (x86)\Microchip\MPLABX\v5.15\packs\Microchip\ATmega_DFP\2.0.2\gcc\dev\atmega4808" -g -DDEBUG  -gdwarf-2  -x c -c -D__$(MP_PROCESSOR_OPTION)__  -funsigned-char -funsigned-bitfields -Os -ffunction-sections -fdata-sections -fpack-struct -fshort-enums -Wall -MD -MP -MF "${OBJECTDIR}/mcc_generated_files/cloud/crypto_client/crypto_client.o.d" -MT "${OBJECTDIR}/mcc_generated_files/cloud/crypto_client/crypto_client.o.d" -MT ${OBJECTDIR}/mcc_generated_files/cloud/crypto_client/crypto_client.o  -o ${OBJECTDIR}/mcc_generated_files/cloud/crypto_client/crypto_client.o mcc_generated_files/cloud/crypto_client/crypto_client.c  -DXPRJ_default=$(CND_CONF)  $(COMPARISON_BUILD) 
	
${OBJECTDIR}/mcc_generated_files/cloud/crypto_client/cryptoauthlib_main.o: mcc_generated_files/cloud/crypto_client/cryptoauthlib_main.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/mcc_generated_files/cloud/crypto_client" 
	@${RM} ${OBJECTDIR}/mcc_generated_files/cloud/crypto_client/cryptoauthlib_main.o.d 
	@${RM} ${OBJECTDIR}/mcc_generated_files/cloud/crypto_client/cryptoauthlib_main.o 
	 ${MP_CC}  $(MP_EXTRA_CC_PRE) -mmcu=atmega4808  -I "C:\Program Files (x86)\Microchip\MPLABX\v5.15\packs\Microchip\ATmega_DFP\2.0.2\include"  -B "C:\Program Files (x86)\Microchip\MPLABX\v5.15\packs\Microchip\ATmega_DFP\2.0.2\gcc\dev\atmega4808" -g -DDEBUG  -gdwarf-2  -x c -c -D__$(MP_PROCESSOR_OPTION)__  -funsigned-char -funsigned-bitfields -Os -ffunction-sections -fdata-sections -fpack-struct -fshort-enums -Wall -MD -MP -MF "${OBJECTDIR}/mcc_generated_files/cloud/crypto_client/cryptoauthlib_main.o.d" -MT "${OBJECTDIR}/mcc_generated_files/cloud/crypto_client/cryptoauthlib_main.o.d" -MT ${OBJECTDIR}/mcc_generated_files/cloud/crypto_client/cryptoauthlib_main.o  -o ${OBJECTDIR}/mcc_generated_files/cloud/crypto_client/cryptoauthlib_main.o mcc_generated_files/cloud/crypto_client/cryptoauthlib_main.c  -DXPRJ_default=$(CND_CONF)  $(COMPARISON_BUILD) 
	
${OBJECTDIR}/mcc_generated_files/cloud/mqtt_packetPopulation/mqtt_packetPopulate.o: mcc_generated_files/cloud/mqtt_packetPopulation/mqtt_packetPopulate.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/mcc_generated_files/cloud/mqtt_packetPopulation" 
	@${RM} ${OBJECTDIR}/mcc_generated_files/cloud/mqtt_packetPopulation/mqtt_packetPopulate.o.d 
	@${RM} ${OBJECTDIR}/mcc_generated_files/cloud/mqtt_packetPopulation/mqtt_packetPopulate.o 
	 ${MP_CC}  $(MP_EXTRA_CC_PRE) -mmcu=atmega4808  -I "C:\Program Files (x86)\Microchip\MPLABX\v5.15\packs\Microchip\ATmega_DFP\2.0.2\include"  -B "C:\Program Files (x86)\Microchip\MPLABX\v5.15\packs\Microchip\ATmega_DFP\2.0.2\gcc\dev\atmega4808" -g -DDEBUG  -gdwarf-2  -x c -c -D__$(MP_PROCESSOR_OPTION)__  -funsigned-char -funsigned-bitfields -Os -ffunction-sections -fdata-sections -fpack-struct -fshort-enums -Wall -MD -MP -MF "${OBJECTDIR}/mcc_generated_files/cloud/mqtt_packetPopulation/mqtt_packetPopulate.o.d" -MT "${OBJECTDIR}/mcc_generated_files/cloud/mqtt_packetPopulation/mqtt_packetPopulate.o.d" -MT ${OBJECTDIR}/mcc_generated_files/cloud/mqtt_packetPopulation/mqtt_packetPopulate.o  -o ${OBJECTDIR}/mcc_generated_files/cloud/mqtt_packetPopulation/mqtt_packetPopulate.o mcc_generated_files/cloud/mqtt_packetPopulation/mqtt_packetPopulate.c  -DXPRJ_default=$(CND_CONF)  $(COMPARISON_BUILD) 
	
${OBJECTDIR}/mcc_generated_files/cloud/wifi_service.o: mcc_generated_files/cloud/wifi_service.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/mcc_generated_files/cloud" 
	@${RM} ${OBJECTDIR}/mcc_generated_files/cloud/wifi_service.o.d 
	@${RM} ${OBJECTDIR}/mcc_generated_files/cloud/wifi_service.o 
	 ${MP_CC}  $(MP_EXTRA_CC_PRE) -mmcu=atmega4808  -I "C:\Program Files (x86)\Microchip\MPLABX\v5.15\packs\Microchip\ATmega_DFP\2.0.2\include"  -B "C:\Program Files (x86)\Microchip\MPLABX\v5.15\packs\Microchip\ATmega_DFP\2.0.2\gcc\dev\atmega4808" -g -DDEBUG  -gdwarf-2  -x c -c -D__$(MP_PROCESSOR_OPTION)__  -funsigned-char -funsigned-bitfields -Os -ffunction-sections -fdata-sections -fpack-struct -fshort-enums -Wall -MD -MP -MF "${OBJECTDIR}/mcc_generated_files/cloud/wifi_service.o.d" -MT "${OBJECTDIR}/mcc_generated_files/cloud/wifi_service.o.d" -MT ${OBJECTDIR}/mcc_generated_files/cloud/wifi_service.o  -o ${OBJECTDIR}/mcc_generated_files/cloud/wifi_service.o mcc_generated_files/cloud/wifi_service.c  -DXPRJ_default=$(CND_CONF)  $(COMPARISON_BUILD) 
	
${OBJECTDIR}/mcc_generated_files/cloud/cloud_service.o: mcc_generated_files/cloud/cloud_service.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/mcc_generated_files/cloud" 
	@${RM} ${OBJECTDIR}/mcc_generated_files/cloud/cloud_service.o.d 
	@${RM} ${OBJECTDIR}/mcc_generated_files/cloud/cloud_service.o 
	 ${MP_CC}  $(MP_EXTRA_CC_PRE) -mmcu=atmega4808  -I "C:\Program Files (x86)\Microchip\MPLABX\v5.15\packs\Microchip\ATmega_DFP\2.0.2\include"  -B "C:\Program Files (x86)\Microchip\MPLABX\v5.15\packs\Microchip\ATmega_DFP\2.0.2\gcc\dev\atmega4808" -g -DDEBUG  -gdwarf-2  -x c -c -D__$(MP_PROCESSOR_OPTION)__  -funsigned-char -funsigned-bitfields -Os -ffunction-sections -fdata-sections -fpack-struct -fshort-enums -Wall -MD -MP -MF "${OBJECTDIR}/mcc_generated_files/cloud/cloud_service.o.d" -MT "${OBJECTDIR}/mcc_generated_files/cloud/cloud_service.o.d" -MT ${OBJECTDIR}/mcc_generated_files/cloud/cloud_service.o  -o ${OBJECTDIR}/mcc_generated_files/cloud/cloud_service.o mcc_generated_files/cloud/cloud_service.c  -DXPRJ_default=$(CND_CONF)  $(COMPARISON_BUILD) 
	
${OBJECTDIR}/mcc_generated_files/credentials_storage/credentials_storage.o: mcc_generated_files/credentials_storage/credentials_storage.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/mcc_generated_files/credentials_storage" 
	@${RM} ${OBJECTDIR}/mcc_generated_files/credentials_storage/credentials_storage.o.d 
	@${RM} ${OBJECTDIR}/mcc_generated_files/credentials_storage/credentials_storage.o 
	 ${MP_CC}  $(MP_EXTRA_CC_PRE) -mmcu=atmega4808  -I "C:\Program Files (x86)\Microchip\MPLABX\v5.15\packs\Microchip\ATmega_DFP\2.0.2\include"  -B "C:\Program Files (x86)\Microchip\MPLABX\v5.15\packs\Microchip\ATmega_DFP\2.0.2\gcc\dev\atmega4808" -g -DDEBUG  -gdwarf-2  -x c -c -D__$(MP_PROCESSOR_OPTION)__  -funsigned-char -funsigned-bitfields -Os -ffunction-sections -fdata-sections -fpack-struct -fshort-enums -Wall -MD -MP -MF "${OBJECTDIR}/mcc_generated_files/credentials_storage/credentials_storage.o.d" -MT "${OBJECTDIR}/mcc_generated_files/credentials_storage/credentials_storage.o.d" -MT ${OBJECTDIR}/mcc_generated_files/credentials_storage/credentials_storage.o  -o ${OBJECTDIR}/mcc_generated_files/credentials_storage/credentials_storage.o mcc_generated_files/credentials_storage/credentials_storage.c  -DXPRJ_default=$(CND_CONF)  $(COMPARISON_BUILD) 
	
${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/atcacert/atcacert_client.o: mcc_generated_files/cryptoauthlib/lib/atcacert/atcacert_client.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/atcacert" 
	@${RM} ${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/atcacert/atcacert_client.o.d 
	@${RM} ${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/atcacert/atcacert_client.o 
	 ${MP_CC}  $(MP_EXTRA_CC_PRE) -mmcu=atmega4808  -I "C:\Program Files (x86)\Microchip\MPLABX\v5.15\packs\Microchip\ATmega_DFP\2.0.2\include"  -B "C:\Program Files (x86)\Microchip\MPLABX\v5.15\packs\Microchip\ATmega_DFP\2.0.2\gcc\dev\atmega4808" -g -DDEBUG  -gdwarf-2  -x c -c -D__$(MP_PROCESSOR_OPTION)__  -funsigned-char -funsigned-bitfields -Os -ffunction-sections -fdata-sections -fpack-struct -fshort-enums -Wall -MD -MP -MF "${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/atcacert/atcacert_client.o.d" -MT "${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/atcacert/atcacert_client.o.d" -MT ${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/atcacert/atcacert_client.o  -o ${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/atcacert/atcacert_client.o mcc_generated_files/cryptoauthlib/lib/atcacert/atcacert_client.c  -DXPRJ_default=$(CND_CONF)  $(COMPARISON_BUILD) 
	
${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/atcacert/atcacert_host_sw.o: mcc_generated_files/cryptoauthlib/lib/atcacert/atcacert_host_sw.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/atcacert" 
	@${RM} ${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/atcacert/atcacert_host_sw.o.d 
	@${RM} ${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/atcacert/atcacert_host_sw.o 
	 ${MP_CC}  $(MP_EXTRA_CC_PRE) -mmcu=atmega4808  -I "C:\Program Files (x86)\Microchip\MPLABX\v5.15\packs\Microchip\ATmega_DFP\2.0.2\include"  -B "C:\Program Files (x86)\Microchip\MPLABX\v5.15\packs\Microchip\ATmega_DFP\2.0.2\gcc\dev\atmega4808" -g -DDEBUG  -gdwarf-2  -x c -c -D__$(MP_PROCESSOR_OPTION)__  -funsigned-char -funsigned-bitfields -Os -ffunction-sections -fdata-sections -fpack-struct -fshort-enums -Wall -MD -MP -MF "${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/atcacert/atcacert_host_sw.o.d" -MT "${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/atcacert/atcacert_host_sw.o.d" -MT ${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/atcacert/atcacert_host_sw.o  -o ${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/atcacert/atcacert_host_sw.o mcc_generated_files/cryptoauthlib/lib/atcacert/atcacert_host_sw.c  -DXPRJ_default=$(CND_CONF)  $(COMPARISON_BUILD) 
	
${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/atcacert/atcacert_def.o: mcc_generated_files/cryptoauthlib/lib/atcacert/atcacert_def.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/atcacert" 
	@${RM} ${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/atcacert/atcacert_def.o.d 
	@${RM} ${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/atcacert/atcacert_def.o 
	 ${MP_CC}  $(MP_EXTRA_CC_PRE) -mmcu=atmega4808  -I "C:\Program Files (x86)\Microchip\MPLABX\v5.15\packs\Microchip\ATmega_DFP\2.0.2\include"  -B "C:\Program Files (x86)\Microchip\MPLABX\v5.15\packs\Microchip\ATmega_DFP\2.0.2\gcc\dev\atmega4808" -g -DDEBUG  -gdwarf-2  -x c -c -D__$(MP_PROCESSOR_OPTION)__  -funsigned-char -funsigned-bitfields -Os -ffunction-sections -fdata-sections -fpack-struct -fshort-enums -Wall -MD -MP -MF "${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/atcacert/atcacert_def.o.d" -MT "${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/atcacert/atcacert_def.o.d" -MT ${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/atcacert/atcacert_def.o  -o ${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/atcacert/atcacert_def.o mcc_generated_files/cryptoauthlib/lib/atcacert/atcacert_def.c  -DXPRJ_default=$(CND_CONF)  $(COMPARISON_BUILD) 
	
${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/atcacert/atcacert_date.o: mcc_generated_files/cryptoauthlib/lib/atcacert/atcacert_date.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/atcacert" 
	@${RM} ${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/atcacert/atcacert_date.o.d 
	@${RM} ${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/atcacert/atcacert_date.o 
	 ${MP_CC}  $(MP_EXTRA_CC_PRE) -mmcu=atmega4808  -I "C:\Program Files (x86)\Microchip\MPLABX\v5.15\packs\Microchip\ATmega_DFP\2.0.2\include"  -B "C:\Program Files (x86)\Microchip\MPLABX\v5.15\packs\Microchip\ATmega_DFP\2.0.2\gcc\dev\atmega4808" -g -DDEBUG  -gdwarf-2  -x c -c -D__$(MP_PROCESSOR_OPTION)__  -funsigned-char -funsigned-bitfields -Os -ffunction-sections -fdata-sections -fpack-struct -fshort-enums -Wall -MD -MP -MF "${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/atcacert/atcacert_date.o.d" -MT "${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/atcacert/atcacert_date.o.d" -MT ${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/atcacert/atcacert_date.o  -o ${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/atcacert/atcacert_date.o mcc_generated_files/cryptoauthlib/lib/atcacert/atcacert_date.c  -DXPRJ_default=$(CND_CONF)  $(COMPARISON_BUILD) 
	
${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/atcacert/atcacert_host_hw.o: mcc_generated_files/cryptoauthlib/lib/atcacert/atcacert_host_hw.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/atcacert" 
	@${RM} ${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/atcacert/atcacert_host_hw.o.d 
	@${RM} ${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/atcacert/atcacert_host_hw.o 
	 ${MP_CC}  $(MP_EXTRA_CC_PRE) -mmcu=atmega4808  -I "C:\Program Files (x86)\Microchip\MPLABX\v5.15\packs\Microchip\ATmega_DFP\2.0.2\include"  -B "C:\Program Files (x86)\Microchip\MPLABX\v5.15\packs\Microchip\ATmega_DFP\2.0.2\gcc\dev\atmega4808" -g -DDEBUG  -gdwarf-2  -x c -c -D__$(MP_PROCESSOR_OPTION)__  -funsigned-char -funsigned-bitfields -Os -ffunction-sections -fdata-sections -fpack-struct -fshort-enums -Wall -MD -MP -MF "${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/atcacert/atcacert_host_hw.o.d" -MT "${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/atcacert/atcacert_host_hw.o.d" -MT ${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/atcacert/atcacert_host_hw.o  -o ${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/atcacert/atcacert_host_hw.o mcc_generated_files/cryptoauthlib/lib/atcacert/atcacert_host_hw.c  -DXPRJ_default=$(CND_CONF)  $(COMPARISON_BUILD) 
	
${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/atcacert/atcacert_der.o: mcc_generated_files/cryptoauthlib/lib/atcacert/atcacert_der.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/atcacert" 
	@${RM} ${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/atcacert/atcacert_der.o.d 
	@${RM} ${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/atcacert/atcacert_der.o 
	 ${MP_CC}  $(MP_EXTRA_CC_PRE) -mmcu=atmega4808  -I "C:\Program Files (x86)\Microchip\MPLABX\v5.15\packs\Microchip\ATmega_DFP\2.0.2\include"  -B "C:\Program Files (x86)\Microchip\MPLABX\v5.15\packs\Microchip\ATmega_DFP\2.0.2\gcc\dev\atmega4808" -g -DDEBUG  -gdwarf-2  -x c -c -D__$(MP_PROCESSOR_OPTION)__  -funsigned-char -funsigned-bitfields -Os -ffunction-sections -fdata-sections -fpack-struct -fshort-enums -Wall -MD -MP -MF "${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/atcacert/atcacert_der.o.d" -MT "${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/atcacert/atcacert_der.o.d" -MT ${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/atcacert/atcacert_der.o  -o ${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/atcacert/atcacert_der.o mcc_generated_files/cryptoauthlib/lib/atcacert/atcacert_der.c  -DXPRJ_default=$(CND_CONF)  $(COMPARISON_BUILD) 
	
${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/atcacert/atcacert_pem.o: mcc_generated_files/cryptoauthlib/lib/atcacert/atcacert_pem.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/atcacert" 
	@${RM} ${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/atcacert/atcacert_pem.o.d 
	@${RM} ${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/atcacert/atcacert_pem.o 
	 ${MP_CC}  $(MP_EXTRA_CC_PRE) -mmcu=atmega4808  -I "C:\Program Files (x86)\Microchip\MPLABX\v5.15\packs\Microchip\ATmega_DFP\2.0.2\include"  -B "C:\Program Files (x86)\Microchip\MPLABX\v5.15\packs\Microchip\ATmega_DFP\2.0.2\gcc\dev\atmega4808" -g -DDEBUG  -gdwarf-2  -x c -c -D__$(MP_PROCESSOR_OPTION)__  -funsigned-char -funsigned-bitfields -Os -ffunction-sections -fdata-sections -fpack-struct -fshort-enums -Wall -MD -MP -MF "${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/atcacert/atcacert_pem.o.d" -MT "${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/atcacert/atcacert_pem.o.d" -MT ${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/atcacert/atcacert_pem.o  -o ${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/atcacert/atcacert_pem.o mcc_generated_files/cryptoauthlib/lib/atcacert/atcacert_pem.c  -DXPRJ_default=$(CND_CONF)  $(COMPARISON_BUILD) 
	
${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/basic/atca_basic_checkmac.o: mcc_generated_files/cryptoauthlib/lib/basic/atca_basic_checkmac.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/basic" 
	@${RM} ${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/basic/atca_basic_checkmac.o.d 
	@${RM} ${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/basic/atca_basic_checkmac.o 
	 ${MP_CC}  $(MP_EXTRA_CC_PRE) -mmcu=atmega4808  -I "C:\Program Files (x86)\Microchip\MPLABX\v5.15\packs\Microchip\ATmega_DFP\2.0.2\include"  -B "C:\Program Files (x86)\Microchip\MPLABX\v5.15\packs\Microchip\ATmega_DFP\2.0.2\gcc\dev\atmega4808" -g -DDEBUG  -gdwarf-2  -x c -c -D__$(MP_PROCESSOR_OPTION)__  -funsigned-char -funsigned-bitfields -Os -ffunction-sections -fdata-sections -fpack-struct -fshort-enums -Wall -MD -MP -MF "${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/basic/atca_basic_checkmac.o.d" -MT "${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/basic/atca_basic_checkmac.o.d" -MT ${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/basic/atca_basic_checkmac.o  -o ${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/basic/atca_basic_checkmac.o mcc_generated_files/cryptoauthlib/lib/basic/atca_basic_checkmac.c  -DXPRJ_default=$(CND_CONF)  $(COMPARISON_BUILD) 
	
${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/basic/atca_basic_secureboot.o: mcc_generated_files/cryptoauthlib/lib/basic/atca_basic_secureboot.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/basic" 
	@${RM} ${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/basic/atca_basic_secureboot.o.d 
	@${RM} ${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/basic/atca_basic_secureboot.o 
	 ${MP_CC}  $(MP_EXTRA_CC_PRE) -mmcu=atmega4808  -I "C:\Program Files (x86)\Microchip\MPLABX\v5.15\packs\Microchip\ATmega_DFP\2.0.2\include"  -B "C:\Program Files (x86)\Microchip\MPLABX\v5.15\packs\Microchip\ATmega_DFP\2.0.2\gcc\dev\atmega4808" -g -DDEBUG  -gdwarf-2  -x c -c -D__$(MP_PROCESSOR_OPTION)__  -funsigned-char -funsigned-bitfields -Os -ffunction-sections -fdata-sections -fpack-struct -fshort-enums -Wall -MD -MP -MF "${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/basic/atca_basic_secureboot.o.d" -MT "${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/basic/atca_basic_secureboot.o.d" -MT ${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/basic/atca_basic_secureboot.o  -o ${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/basic/atca_basic_secureboot.o mcc_generated_files/cryptoauthlib/lib/basic/atca_basic_secureboot.c  -DXPRJ_default=$(CND_CONF)  $(COMPARISON_BUILD) 
	
${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/basic/atca_basic_selftest.o: mcc_generated_files/cryptoauthlib/lib/basic/atca_basic_selftest.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/basic" 
	@${RM} ${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/basic/atca_basic_selftest.o.d 
	@${RM} ${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/basic/atca_basic_selftest.o 
	 ${MP_CC}  $(MP_EXTRA_CC_PRE) -mmcu=atmega4808  -I "C:\Program Files (x86)\Microchip\MPLABX\v5.15\packs\Microchip\ATmega_DFP\2.0.2\include"  -B "C:\Program Files (x86)\Microchip\MPLABX\v5.15\packs\Microchip\ATmega_DFP\2.0.2\gcc\dev\atmega4808" -g -DDEBUG  -gdwarf-2  -x c -c -D__$(MP_PROCESSOR_OPTION)__  -funsigned-char -funsigned-bitfields -Os -ffunction-sections -fdata-sections -fpack-struct -fshort-enums -Wall -MD -MP -MF "${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/basic/atca_basic_selftest.o.d" -MT "${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/basic/atca_basic_selftest.o.d" -MT ${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/basic/atca_basic_selftest.o  -o ${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/basic/atca_basic_selftest.o mcc_generated_files/cryptoauthlib/lib/basic/atca_basic_selftest.c  -DXPRJ_default=$(CND_CONF)  $(COMPARISON_BUILD) 
	
${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/basic/atca_basic_ecdh.o: mcc_generated_files/cryptoauthlib/lib/basic/atca_basic_ecdh.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/basic" 
	@${RM} ${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/basic/atca_basic_ecdh.o.d 
	@${RM} ${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/basic/atca_basic_ecdh.o 
	 ${MP_CC}  $(MP_EXTRA_CC_PRE) -mmcu=atmega4808  -I "C:\Program Files (x86)\Microchip\MPLABX\v5.15\packs\Microchip\ATmega_DFP\2.0.2\include"  -B "C:\Program Files (x86)\Microchip\MPLABX\v5.15\packs\Microchip\ATmega_DFP\2.0.2\gcc\dev\atmega4808" -g -DDEBUG  -gdwarf-2  -x c -c -D__$(MP_PROCESSOR_OPTION)__  -funsigned-char -funsigned-bitfields -Os -ffunction-sections -fdata-sections -fpack-struct -fshort-enums -Wall -MD -MP -MF "${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/basic/atca_basic_ecdh.o.d" -MT "${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/basic/atca_basic_ecdh.o.d" -MT ${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/basic/atca_basic_ecdh.o  -o ${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/basic/atca_basic_ecdh.o mcc_generated_files/cryptoauthlib/lib/basic/atca_basic_ecdh.c  -DXPRJ_default=$(CND_CONF)  $(COMPARISON_BUILD) 
	
${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/basic/atca_basic_updateextra.o: mcc_generated_files/cryptoauthlib/lib/basic/atca_basic_updateextra.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/basic" 
	@${RM} ${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/basic/atca_basic_updateextra.o.d 
	@${RM} ${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/basic/atca_basic_updateextra.o 
	 ${MP_CC}  $(MP_EXTRA_CC_PRE) -mmcu=atmega4808  -I "C:\Program Files (x86)\Microchip\MPLABX\v5.15\packs\Microchip\ATmega_DFP\2.0.2\include"  -B "C:\Program Files (x86)\Microchip\MPLABX\v5.15\packs\Microchip\ATmega_DFP\2.0.2\gcc\dev\atmega4808" -g -DDEBUG  -gdwarf-2  -x c -c -D__$(MP_PROCESSOR_OPTION)__  -funsigned-char -funsigned-bitfields -Os -ffunction-sections -fdata-sections -fpack-struct -fshort-enums -Wall -MD -MP -MF "${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/basic/atca_basic_updateextra.o.d" -MT "${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/basic/atca_basic_updateextra.o.d" -MT ${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/basic/atca_basic_updateextra.o  -o ${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/basic/atca_basic_updateextra.o mcc_generated_files/cryptoauthlib/lib/basic/atca_basic_updateextra.c  -DXPRJ_default=$(CND_CONF)  $(COMPARISON_BUILD) 
	
${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/basic/atca_basic_privwrite.o: mcc_generated_files/cryptoauthlib/lib/basic/atca_basic_privwrite.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/basic" 
	@${RM} ${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/basic/atca_basic_privwrite.o.d 
	@${RM} ${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/basic/atca_basic_privwrite.o 
	 ${MP_CC}  $(MP_EXTRA_CC_PRE) -mmcu=atmega4808  -I "C:\Program Files (x86)\Microchip\MPLABX\v5.15\packs\Microchip\ATmega_DFP\2.0.2\include"  -B "C:\Program Files (x86)\Microchip\MPLABX\v5.15\packs\Microchip\ATmega_DFP\2.0.2\gcc\dev\atmega4808" -g -DDEBUG  -gdwarf-2  -x c -c -D__$(MP_PROCESSOR_OPTION)__  -funsigned-char -funsigned-bitfields -Os -ffunction-sections -fdata-sections -fpack-struct -fshort-enums -Wall -MD -MP -MF "${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/basic/atca_basic_privwrite.o.d" -MT "${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/basic/atca_basic_privwrite.o.d" -MT ${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/basic/atca_basic_privwrite.o  -o ${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/basic/atca_basic_privwrite.o mcc_generated_files/cryptoauthlib/lib/basic/atca_basic_privwrite.c  -DXPRJ_default=$(CND_CONF)  $(COMPARISON_BUILD) 
	
${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/basic/atca_basic_info.o: mcc_generated_files/cryptoauthlib/lib/basic/atca_basic_info.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/basic" 
	@${RM} ${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/basic/atca_basic_info.o.d 
	@${RM} ${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/basic/atca_basic_info.o 
	 ${MP_CC}  $(MP_EXTRA_CC_PRE) -mmcu=atmega4808  -I "C:\Program Files (x86)\Microchip\MPLABX\v5.15\packs\Microchip\ATmega_DFP\2.0.2\include"  -B "C:\Program Files (x86)\Microchip\MPLABX\v5.15\packs\Microchip\ATmega_DFP\2.0.2\gcc\dev\atmega4808" -g -DDEBUG  -gdwarf-2  -x c -c -D__$(MP_PROCESSOR_OPTION)__  -funsigned-char -funsigned-bitfields -Os -ffunction-sections -fdata-sections -fpack-struct -fshort-enums -Wall -MD -MP -MF "${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/basic/atca_basic_info.o.d" -MT "${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/basic/atca_basic_info.o.d" -MT ${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/basic/atca_basic_info.o  -o ${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/basic/atca_basic_info.o mcc_generated_files/cryptoauthlib/lib/basic/atca_basic_info.c  -DXPRJ_default=$(CND_CONF)  $(COMPARISON_BUILD) 
	
${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/basic/atca_basic.o: mcc_generated_files/cryptoauthlib/lib/basic/atca_basic.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/basic" 
	@${RM} ${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/basic/atca_basic.o.d 
	@${RM} ${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/basic/atca_basic.o 
	 ${MP_CC}  $(MP_EXTRA_CC_PRE) -mmcu=atmega4808  -I "C:\Program Files (x86)\Microchip\MPLABX\v5.15\packs\Microchip\ATmega_DFP\2.0.2\include"  -B "C:\Program Files (x86)\Microchip\MPLABX\v5.15\packs\Microchip\ATmega_DFP\2.0.2\gcc\dev\atmega4808" -g -DDEBUG  -gdwarf-2  -x c -c -D__$(MP_PROCESSOR_OPTION)__  -funsigned-char -funsigned-bitfields -Os -ffunction-sections -fdata-sections -fpack-struct -fshort-enums -Wall -MD -MP -MF "${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/basic/atca_basic.o.d" -MT "${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/basic/atca_basic.o.d" -MT ${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/basic/atca_basic.o  -o ${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/basic/atca_basic.o mcc_generated_files/cryptoauthlib/lib/basic/atca_basic.c  -DXPRJ_default=$(CND_CONF)  $(COMPARISON_BUILD) 
	
${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/basic/atca_basic_read.o: mcc_generated_files/cryptoauthlib/lib/basic/atca_basic_read.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/basic" 
	@${RM} ${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/basic/atca_basic_read.o.d 
	@${RM} ${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/basic/atca_basic_read.o 
	 ${MP_CC}  $(MP_EXTRA_CC_PRE) -mmcu=atmega4808  -I "C:\Program Files (x86)\Microchip\MPLABX\v5.15\packs\Microchip\ATmega_DFP\2.0.2\include"  -B "C:\Program Files (x86)\Microchip\MPLABX\v5.15\packs\Microchip\ATmega_DFP\2.0.2\gcc\dev\atmega4808" -g -DDEBUG  -gdwarf-2  -x c -c -D__$(MP_PROCESSOR_OPTION)__  -funsigned-char -funsigned-bitfields -Os -ffunction-sections -fdata-sections -fpack-struct -fshort-enums -Wall -MD -MP -MF "${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/basic/atca_basic_read.o.d" -MT "${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/basic/atca_basic_read.o.d" -MT ${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/basic/atca_basic_read.o  -o ${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/basic/atca_basic_read.o mcc_generated_files/cryptoauthlib/lib/basic/atca_basic_read.c  -DXPRJ_default=$(CND_CONF)  $(COMPARISON_BUILD) 
	
${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/basic/atca_basic_aes.o: mcc_generated_files/cryptoauthlib/lib/basic/atca_basic_aes.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/basic" 
	@${RM} ${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/basic/atca_basic_aes.o.d 
	@${RM} ${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/basic/atca_basic_aes.o 
	 ${MP_CC}  $(MP_EXTRA_CC_PRE) -mmcu=atmega4808  -I "C:\Program Files (x86)\Microchip\MPLABX\v5.15\packs\Microchip\ATmega_DFP\2.0.2\include"  -B "C:\Program Files (x86)\Microchip\MPLABX\v5.15\packs\Microchip\ATmega_DFP\2.0.2\gcc\dev\atmega4808" -g -DDEBUG  -gdwarf-2  -x c -c -D__$(MP_PROCESSOR_OPTION)__  -funsigned-char -funsigned-bitfields -Os -ffunction-sections -fdata-sections -fpack-struct -fshort-enums -Wall -MD -MP -MF "${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/basic/atca_basic_aes.o.d" -MT "${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/basic/atca_basic_aes.o.d" -MT ${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/basic/atca_basic_aes.o  -o ${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/basic/atca_basic_aes.o mcc_generated_files/cryptoauthlib/lib/basic/atca_basic_aes.c  -DXPRJ_default=$(CND_CONF)  $(COMPARISON_BUILD) 
	
${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/basic/atca_basic_verify.o: mcc_generated_files/cryptoauthlib/lib/basic/atca_basic_verify.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/basic" 
	@${RM} ${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/basic/atca_basic_verify.o.d 
	@${RM} ${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/basic/atca_basic_verify.o 
	 ${MP_CC}  $(MP_EXTRA_CC_PRE) -mmcu=atmega4808  -I "C:\Program Files (x86)\Microchip\MPLABX\v5.15\packs\Microchip\ATmega_DFP\2.0.2\include"  -B "C:\Program Files (x86)\Microchip\MPLABX\v5.15\packs\Microchip\ATmega_DFP\2.0.2\gcc\dev\atmega4808" -g -DDEBUG  -gdwarf-2  -x c -c -D__$(MP_PROCESSOR_OPTION)__  -funsigned-char -funsigned-bitfields -Os -ffunction-sections -fdata-sections -fpack-struct -fshort-enums -Wall -MD -MP -MF "${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/basic/atca_basic_verify.o.d" -MT "${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/basic/atca_basic_verify.o.d" -MT ${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/basic/atca_basic_verify.o  -o ${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/basic/atca_basic_verify.o mcc_generated_files/cryptoauthlib/lib/basic/atca_basic_verify.c  -DXPRJ_default=$(CND_CONF)  $(COMPARISON_BUILD) 
	
${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/basic/atca_basic_sha.o: mcc_generated_files/cryptoauthlib/lib/basic/atca_basic_sha.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/basic" 
	@${RM} ${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/basic/atca_basic_sha.o.d 
	@${RM} ${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/basic/atca_basic_sha.o 
	 ${MP_CC}  $(MP_EXTRA_CC_PRE) -mmcu=atmega4808  -I "C:\Program Files (x86)\Microchip\MPLABX\v5.15\packs\Microchip\ATmega_DFP\2.0.2\include"  -B "C:\Program Files (x86)\Microchip\MPLABX\v5.15\packs\Microchip\ATmega_DFP\2.0.2\gcc\dev\atmega4808" -g -DDEBUG  -gdwarf-2  -x c -c -D__$(MP_PROCESSOR_OPTION)__  -funsigned-char -funsigned-bitfields -Os -ffunction-sections -fdata-sections -fpack-struct -fshort-enums -Wall -MD -MP -MF "${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/basic/atca_basic_sha.o.d" -MT "${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/basic/atca_basic_sha.o.d" -MT ${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/basic/atca_basic_sha.o  -o ${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/basic/atca_basic_sha.o mcc_generated_files/cryptoauthlib/lib/basic/atca_basic_sha.c  -DXPRJ_default=$(CND_CONF)  $(COMPARISON_BUILD) 
	
${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/basic/atca_basic_genkey.o: mcc_generated_files/cryptoauthlib/lib/basic/atca_basic_genkey.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/basic" 
	@${RM} ${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/basic/atca_basic_genkey.o.d 
	@${RM} ${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/basic/atca_basic_genkey.o 
	 ${MP_CC}  $(MP_EXTRA_CC_PRE) -mmcu=atmega4808  -I "C:\Program Files (x86)\Microchip\MPLABX\v5.15\packs\Microchip\ATmega_DFP\2.0.2\include"  -B "C:\Program Files (x86)\Microchip\MPLABX\v5.15\packs\Microchip\ATmega_DFP\2.0.2\gcc\dev\atmega4808" -g -DDEBUG  -gdwarf-2  -x c -c -D__$(MP_PROCESSOR_OPTION)__  -funsigned-char -funsigned-bitfields -Os -ffunction-sections -fdata-sections -fpack-struct -fshort-enums -Wall -MD -MP -MF "${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/basic/atca_basic_genkey.o.d" -MT "${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/basic/atca_basic_genkey.o.d" -MT ${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/basic/atca_basic_genkey.o  -o ${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/basic/atca_basic_genkey.o mcc_generated_files/cryptoauthlib/lib/basic/atca_basic_genkey.c  -DXPRJ_default=$(CND_CONF)  $(COMPARISON_BUILD) 
	
${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/basic/atca_basic_nonce.o: mcc_generated_files/cryptoauthlib/lib/basic/atca_basic_nonce.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/basic" 
	@${RM} ${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/basic/atca_basic_nonce.o.d 
	@${RM} ${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/basic/atca_basic_nonce.o 
	 ${MP_CC}  $(MP_EXTRA_CC_PRE) -mmcu=atmega4808  -I "C:\Program Files (x86)\Microchip\MPLABX\v5.15\packs\Microchip\ATmega_DFP\2.0.2\include"  -B "C:\Program Files (x86)\Microchip\MPLABX\v5.15\packs\Microchip\ATmega_DFP\2.0.2\gcc\dev\atmega4808" -g -DDEBUG  -gdwarf-2  -x c -c -D__$(MP_PROCESSOR_OPTION)__  -funsigned-char -funsigned-bitfields -Os -ffunction-sections -fdata-sections -fpack-struct -fshort-enums -Wall -MD -MP -MF "${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/basic/atca_basic_nonce.o.d" -MT "${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/basic/atca_basic_nonce.o.d" -MT ${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/basic/atca_basic_nonce.o  -o ${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/basic/atca_basic_nonce.o mcc_generated_files/cryptoauthlib/lib/basic/atca_basic_nonce.c  -DXPRJ_default=$(CND_CONF)  $(COMPARISON_BUILD) 
	
${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/basic/atca_basic_sign.o: mcc_generated_files/cryptoauthlib/lib/basic/atca_basic_sign.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/basic" 
	@${RM} ${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/basic/atca_basic_sign.o.d 
	@${RM} ${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/basic/atca_basic_sign.o 
	 ${MP_CC}  $(MP_EXTRA_CC_PRE) -mmcu=atmega4808  -I "C:\Program Files (x86)\Microchip\MPLABX\v5.15\packs\Microchip\ATmega_DFP\2.0.2\include"  -B "C:\Program Files (x86)\Microchip\MPLABX\v5.15\packs\Microchip\ATmega_DFP\2.0.2\gcc\dev\atmega4808" -g -DDEBUG  -gdwarf-2  -x c -c -D__$(MP_PROCESSOR_OPTION)__  -funsigned-char -funsigned-bitfields -Os -ffunction-sections -fdata-sections -fpack-struct -fshort-enums -Wall -MD -MP -MF "${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/basic/atca_basic_sign.o.d" -MT "${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/basic/atca_basic_sign.o.d" -MT ${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/basic/atca_basic_sign.o  -o ${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/basic/atca_basic_sign.o mcc_generated_files/cryptoauthlib/lib/basic/atca_basic_sign.c  -DXPRJ_default=$(CND_CONF)  $(COMPARISON_BUILD) 
	
${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/basic/atca_basic_kdf.o: mcc_generated_files/cryptoauthlib/lib/basic/atca_basic_kdf.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/basic" 
	@${RM} ${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/basic/atca_basic_kdf.o.d 
	@${RM} ${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/basic/atca_basic_kdf.o 
	 ${MP_CC}  $(MP_EXTRA_CC_PRE) -mmcu=atmega4808  -I "C:\Program Files (x86)\Microchip\MPLABX\v5.15\packs\Microchip\ATmega_DFP\2.0.2\include"  -B "C:\Program Files (x86)\Microchip\MPLABX\v5.15\packs\Microchip\ATmega_DFP\2.0.2\gcc\dev\atmega4808" -g -DDEBUG  -gdwarf-2  -x c -c -D__$(MP_PROCESSOR_OPTION)__  -funsigned-char -funsigned-bitfields -Os -ffunction-sections -fdata-sections -fpack-struct -fshort-enums -Wall -MD -MP -MF "${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/basic/atca_basic_kdf.o.d" -MT "${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/basic/atca_basic_kdf.o.d" -MT ${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/basic/atca_basic_kdf.o  -o ${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/basic/atca_basic_kdf.o mcc_generated_files/cryptoauthlib/lib/basic/atca_basic_kdf.c  -DXPRJ_default=$(CND_CONF)  $(COMPARISON_BUILD) 
	
${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/basic/atca_basic_hmac.o: mcc_generated_files/cryptoauthlib/lib/basic/atca_basic_hmac.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/basic" 
	@${RM} ${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/basic/atca_basic_hmac.o.d 
	@${RM} ${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/basic/atca_basic_hmac.o 
	 ${MP_CC}  $(MP_EXTRA_CC_PRE) -mmcu=atmega4808  -I "C:\Program Files (x86)\Microchip\MPLABX\v5.15\packs\Microchip\ATmega_DFP\2.0.2\include"  -B "C:\Program Files (x86)\Microchip\MPLABX\v5.15\packs\Microchip\ATmega_DFP\2.0.2\gcc\dev\atmega4808" -g -DDEBUG  -gdwarf-2  -x c -c -D__$(MP_PROCESSOR_OPTION)__  -funsigned-char -funsigned-bitfields -Os -ffunction-sections -fdata-sections -fpack-struct -fshort-enums -Wall -MD -MP -MF "${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/basic/atca_basic_hmac.o.d" -MT "${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/basic/atca_basic_hmac.o.d" -MT ${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/basic/atca_basic_hmac.o  -o ${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/basic/atca_basic_hmac.o mcc_generated_files/cryptoauthlib/lib/basic/atca_basic_hmac.c  -DXPRJ_default=$(CND_CONF)  $(COMPARISON_BUILD) 
	
${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/basic/atca_basic_counter.o: mcc_generated_files/cryptoauthlib/lib/basic/atca_basic_counter.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/basic" 
	@${RM} ${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/basic/atca_basic_counter.o.d 
	@${RM} ${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/basic/atca_basic_counter.o 
	 ${MP_CC}  $(MP_EXTRA_CC_PRE) -mmcu=atmega4808  -I "C:\Program Files (x86)\Microchip\MPLABX\v5.15\packs\Microchip\ATmega_DFP\2.0.2\include"  -B "C:\Program Files (x86)\Microchip\MPLABX\v5.15\packs\Microchip\ATmega_DFP\2.0.2\gcc\dev\atmega4808" -g -DDEBUG  -gdwarf-2  -x c -c -D__$(MP_PROCESSOR_OPTION)__  -funsigned-char -funsigned-bitfields -Os -ffunction-sections -fdata-sections -fpack-struct -fshort-enums -Wall -MD -MP -MF "${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/basic/atca_basic_counter.o.d" -MT "${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/basic/atca_basic_counter.o.d" -MT ${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/basic/atca_basic_counter.o  -o ${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/basic/atca_basic_counter.o mcc_generated_files/cryptoauthlib/lib/basic/atca_basic_counter.c  -DXPRJ_default=$(CND_CONF)  $(COMPARISON_BUILD) 
	
${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/basic/atca_basic_derivekey.o: mcc_generated_files/cryptoauthlib/lib/basic/atca_basic_derivekey.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/basic" 
	@${RM} ${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/basic/atca_basic_derivekey.o.d 
	@${RM} ${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/basic/atca_basic_derivekey.o 
	 ${MP_CC}  $(MP_EXTRA_CC_PRE) -mmcu=atmega4808  -I "C:\Program Files (x86)\Microchip\MPLABX\v5.15\packs\Microchip\ATmega_DFP\2.0.2\include"  -B "C:\Program Files (x86)\Microchip\MPLABX\v5.15\packs\Microchip\ATmega_DFP\2.0.2\gcc\dev\atmega4808" -g -DDEBUG  -gdwarf-2  -x c -c -D__$(MP_PROCESSOR_OPTION)__  -funsigned-char -funsigned-bitfields -Os -ffunction-sections -fdata-sections -fpack-struct -fshort-enums -Wall -MD -MP -MF "${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/basic/atca_basic_derivekey.o.d" -MT "${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/basic/atca_basic_derivekey.o.d" -MT ${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/basic/atca_basic_derivekey.o  -o ${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/basic/atca_basic_derivekey.o mcc_generated_files/cryptoauthlib/lib/basic/atca_basic_derivekey.c  -DXPRJ_default=$(CND_CONF)  $(COMPARISON_BUILD) 
	
${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/basic/atca_helpers.o: mcc_generated_files/cryptoauthlib/lib/basic/atca_helpers.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/basic" 
	@${RM} ${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/basic/atca_helpers.o.d 
	@${RM} ${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/basic/atca_helpers.o 
	 ${MP_CC}  $(MP_EXTRA_CC_PRE) -mmcu=atmega4808  -I "C:\Program Files (x86)\Microchip\MPLABX\v5.15\packs\Microchip\ATmega_DFP\2.0.2\include"  -B "C:\Program Files (x86)\Microchip\MPLABX\v5.15\packs\Microchip\ATmega_DFP\2.0.2\gcc\dev\atmega4808" -g -DDEBUG  -gdwarf-2  -x c -c -D__$(MP_PROCESSOR_OPTION)__  -funsigned-char -funsigned-bitfields -Os -ffunction-sections -fdata-sections -fpack-struct -fshort-enums -Wall -MD -MP -MF "${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/basic/atca_helpers.o.d" -MT "${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/basic/atca_helpers.o.d" -MT ${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/basic/atca_helpers.o  -o ${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/basic/atca_helpers.o mcc_generated_files/cryptoauthlib/lib/basic/atca_helpers.c  -DXPRJ_default=$(CND_CONF)  $(COMPARISON_BUILD) 
	
${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/basic/atca_basic_lock.o: mcc_generated_files/cryptoauthlib/lib/basic/atca_basic_lock.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/basic" 
	@${RM} ${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/basic/atca_basic_lock.o.d 
	@${RM} ${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/basic/atca_basic_lock.o 
	 ${MP_CC}  $(MP_EXTRA_CC_PRE) -mmcu=atmega4808  -I "C:\Program Files (x86)\Microchip\MPLABX\v5.15\packs\Microchip\ATmega_DFP\2.0.2\include"  -B "C:\Program Files (x86)\Microchip\MPLABX\v5.15\packs\Microchip\ATmega_DFP\2.0.2\gcc\dev\atmega4808" -g -DDEBUG  -gdwarf-2  -x c -c -D__$(MP_PROCESSOR_OPTION)__  -funsigned-char -funsigned-bitfields -Os -ffunction-sections -fdata-sections -fpack-struct -fshort-enums -Wall -MD -MP -MF "${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/basic/atca_basic_lock.o.d" -MT "${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/basic/atca_basic_lock.o.d" -MT ${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/basic/atca_basic_lock.o  -o ${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/basic/atca_basic_lock.o mcc_generated_files/cryptoauthlib/lib/basic/atca_basic_lock.c  -DXPRJ_default=$(CND_CONF)  $(COMPARISON_BUILD) 
	
${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/basic/atca_basic_mac.o: mcc_generated_files/cryptoauthlib/lib/basic/atca_basic_mac.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/basic" 
	@${RM} ${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/basic/atca_basic_mac.o.d 
	@${RM} ${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/basic/atca_basic_mac.o 
	 ${MP_CC}  $(MP_EXTRA_CC_PRE) -mmcu=atmega4808  -I "C:\Program Files (x86)\Microchip\MPLABX\v5.15\packs\Microchip\ATmega_DFP\2.0.2\include"  -B "C:\Program Files (x86)\Microchip\MPLABX\v5.15\packs\Microchip\ATmega_DFP\2.0.2\gcc\dev\atmega4808" -g -DDEBUG  -gdwarf-2  -x c -c -D__$(MP_PROCESSOR_OPTION)__  -funsigned-char -funsigned-bitfields -Os -ffunction-sections -fdata-sections -fpack-struct -fshort-enums -Wall -MD -MP -MF "${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/basic/atca_basic_mac.o.d" -MT "${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/basic/atca_basic_mac.o.d" -MT ${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/basic/atca_basic_mac.o  -o ${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/basic/atca_basic_mac.o mcc_generated_files/cryptoauthlib/lib/basic/atca_basic_mac.c  -DXPRJ_default=$(CND_CONF)  $(COMPARISON_BUILD) 
	
${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/basic/atca_basic_random.o: mcc_generated_files/cryptoauthlib/lib/basic/atca_basic_random.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/basic" 
	@${RM} ${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/basic/atca_basic_random.o.d 
	@${RM} ${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/basic/atca_basic_random.o 
	 ${MP_CC}  $(MP_EXTRA_CC_PRE) -mmcu=atmega4808  -I "C:\Program Files (x86)\Microchip\MPLABX\v5.15\packs\Microchip\ATmega_DFP\2.0.2\include"  -B "C:\Program Files (x86)\Microchip\MPLABX\v5.15\packs\Microchip\ATmega_DFP\2.0.2\gcc\dev\atmega4808" -g -DDEBUG  -gdwarf-2  -x c -c -D__$(MP_PROCESSOR_OPTION)__  -funsigned-char -funsigned-bitfields -Os -ffunction-sections -fdata-sections -fpack-struct -fshort-enums -Wall -MD -MP -MF "${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/basic/atca_basic_random.o.d" -MT "${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/basic/atca_basic_random.o.d" -MT ${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/basic/atca_basic_random.o  -o ${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/basic/atca_basic_random.o mcc_generated_files/cryptoauthlib/lib/basic/atca_basic_random.c  -DXPRJ_default=$(CND_CONF)  $(COMPARISON_BUILD) 
	
${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/basic/atca_basic_gendig.o: mcc_generated_files/cryptoauthlib/lib/basic/atca_basic_gendig.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/basic" 
	@${RM} ${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/basic/atca_basic_gendig.o.d 
	@${RM} ${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/basic/atca_basic_gendig.o 
	 ${MP_CC}  $(MP_EXTRA_CC_PRE) -mmcu=atmega4808  -I "C:\Program Files (x86)\Microchip\MPLABX\v5.15\packs\Microchip\ATmega_DFP\2.0.2\include"  -B "C:\Program Files (x86)\Microchip\MPLABX\v5.15\packs\Microchip\ATmega_DFP\2.0.2\gcc\dev\atmega4808" -g -DDEBUG  -gdwarf-2  -x c -c -D__$(MP_PROCESSOR_OPTION)__  -funsigned-char -funsigned-bitfields -Os -ffunction-sections -fdata-sections -fpack-struct -fshort-enums -Wall -MD -MP -MF "${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/basic/atca_basic_gendig.o.d" -MT "${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/basic/atca_basic_gendig.o.d" -MT ${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/basic/atca_basic_gendig.o  -o ${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/basic/atca_basic_gendig.o mcc_generated_files/cryptoauthlib/lib/basic/atca_basic_gendig.c  -DXPRJ_default=$(CND_CONF)  $(COMPARISON_BUILD) 
	
${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/basic/atca_basic_write.o: mcc_generated_files/cryptoauthlib/lib/basic/atca_basic_write.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/basic" 
	@${RM} ${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/basic/atca_basic_write.o.d 
	@${RM} ${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/basic/atca_basic_write.o 
	 ${MP_CC}  $(MP_EXTRA_CC_PRE) -mmcu=atmega4808  -I "C:\Program Files (x86)\Microchip\MPLABX\v5.15\packs\Microchip\ATmega_DFP\2.0.2\include"  -B "C:\Program Files (x86)\Microchip\MPLABX\v5.15\packs\Microchip\ATmega_DFP\2.0.2\gcc\dev\atmega4808" -g -DDEBUG  -gdwarf-2  -x c -c -D__$(MP_PROCESSOR_OPTION)__  -funsigned-char -funsigned-bitfields -Os -ffunction-sections -fdata-sections -fpack-struct -fshort-enums -Wall -MD -MP -MF "${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/basic/atca_basic_write.o.d" -MT "${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/basic/atca_basic_write.o.d" -MT ${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/basic/atca_basic_write.o  -o ${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/basic/atca_basic_write.o mcc_generated_files/cryptoauthlib/lib/basic/atca_basic_write.c  -DXPRJ_default=$(CND_CONF)  $(COMPARISON_BUILD) 
	
${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/crypto/hashes/sha1_routines.o: mcc_generated_files/cryptoauthlib/lib/crypto/hashes/sha1_routines.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/crypto/hashes" 
	@${RM} ${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/crypto/hashes/sha1_routines.o.d 
	@${RM} ${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/crypto/hashes/sha1_routines.o 
	 ${MP_CC}  $(MP_EXTRA_CC_PRE) -mmcu=atmega4808  -I "C:\Program Files (x86)\Microchip\MPLABX\v5.15\packs\Microchip\ATmega_DFP\2.0.2\include"  -B "C:\Program Files (x86)\Microchip\MPLABX\v5.15\packs\Microchip\ATmega_DFP\2.0.2\gcc\dev\atmega4808" -g -DDEBUG  -gdwarf-2  -x c -c -D__$(MP_PROCESSOR_OPTION)__  -funsigned-char -funsigned-bitfields -Os -ffunction-sections -fdata-sections -fpack-struct -fshort-enums -Wall -MD -MP -MF "${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/crypto/hashes/sha1_routines.o.d" -MT "${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/crypto/hashes/sha1_routines.o.d" -MT ${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/crypto/hashes/sha1_routines.o  -o ${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/crypto/hashes/sha1_routines.o mcc_generated_files/cryptoauthlib/lib/crypto/hashes/sha1_routines.c  -DXPRJ_default=$(CND_CONF)  $(COMPARISON_BUILD) 
	
${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/crypto/hashes/sha2_routines.o: mcc_generated_files/cryptoauthlib/lib/crypto/hashes/sha2_routines.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/crypto/hashes" 
	@${RM} ${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/crypto/hashes/sha2_routines.o.d 
	@${RM} ${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/crypto/hashes/sha2_routines.o 
	 ${MP_CC}  $(MP_EXTRA_CC_PRE) -mmcu=atmega4808  -I "C:\Program Files (x86)\Microchip\MPLABX\v5.15\packs\Microchip\ATmega_DFP\2.0.2\include"  -B "C:\Program Files (x86)\Microchip\MPLABX\v5.15\packs\Microchip\ATmega_DFP\2.0.2\gcc\dev\atmega4808" -g -DDEBUG  -gdwarf-2  -x c -c -D__$(MP_PROCESSOR_OPTION)__  -funsigned-char -funsigned-bitfields -Os -ffunction-sections -fdata-sections -fpack-struct -fshort-enums -Wall -MD -MP -MF "${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/crypto/hashes/sha2_routines.o.d" -MT "${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/crypto/hashes/sha2_routines.o.d" -MT ${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/crypto/hashes/sha2_routines.o  -o ${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/crypto/hashes/sha2_routines.o mcc_generated_files/cryptoauthlib/lib/crypto/hashes/sha2_routines.c  -DXPRJ_default=$(CND_CONF)  $(COMPARISON_BUILD) 
	
${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/crypto/atca_crypto_sw_sha1.o: mcc_generated_files/cryptoauthlib/lib/crypto/atca_crypto_sw_sha1.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/crypto" 
	@${RM} ${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/crypto/atca_crypto_sw_sha1.o.d 
	@${RM} ${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/crypto/atca_crypto_sw_sha1.o 
	 ${MP_CC}  $(MP_EXTRA_CC_PRE) -mmcu=atmega4808  -I "C:\Program Files (x86)\Microchip\MPLABX\v5.15\packs\Microchip\ATmega_DFP\2.0.2\include"  -B "C:\Program Files (x86)\Microchip\MPLABX\v5.15\packs\Microchip\ATmega_DFP\2.0.2\gcc\dev\atmega4808" -g -DDEBUG  -gdwarf-2  -x c -c -D__$(MP_PROCESSOR_OPTION)__  -funsigned-char -funsigned-bitfields -Os -ffunction-sections -fdata-sections -fpack-struct -fshort-enums -Wall -MD -MP -MF "${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/crypto/atca_crypto_sw_sha1.o.d" -MT "${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/crypto/atca_crypto_sw_sha1.o.d" -MT ${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/crypto/atca_crypto_sw_sha1.o  -o ${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/crypto/atca_crypto_sw_sha1.o mcc_generated_files/cryptoauthlib/lib/crypto/atca_crypto_sw_sha1.c  -DXPRJ_default=$(CND_CONF)  $(COMPARISON_BUILD) 
	
${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/crypto/atca_crypto_sw_ecdsa.o: mcc_generated_files/cryptoauthlib/lib/crypto/atca_crypto_sw_ecdsa.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/crypto" 
	@${RM} ${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/crypto/atca_crypto_sw_ecdsa.o.d 
	@${RM} ${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/crypto/atca_crypto_sw_ecdsa.o 
	 ${MP_CC}  $(MP_EXTRA_CC_PRE) -mmcu=atmega4808  -I "C:\Program Files (x86)\Microchip\MPLABX\v5.15\packs\Microchip\ATmega_DFP\2.0.2\include"  -B "C:\Program Files (x86)\Microchip\MPLABX\v5.15\packs\Microchip\ATmega_DFP\2.0.2\gcc\dev\atmega4808" -g -DDEBUG  -gdwarf-2  -x c -c -D__$(MP_PROCESSOR_OPTION)__  -funsigned-char -funsigned-bitfields -Os -ffunction-sections -fdata-sections -fpack-struct -fshort-enums -Wall -MD -MP -MF "${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/crypto/atca_crypto_sw_ecdsa.o.d" -MT "${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/crypto/atca_crypto_sw_ecdsa.o.d" -MT ${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/crypto/atca_crypto_sw_ecdsa.o  -o ${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/crypto/atca_crypto_sw_ecdsa.o mcc_generated_files/cryptoauthlib/lib/crypto/atca_crypto_sw_ecdsa.c  -DXPRJ_default=$(CND_CONF)  $(COMPARISON_BUILD) 
	
${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/crypto/atca_crypto_sw_sha2.o: mcc_generated_files/cryptoauthlib/lib/crypto/atca_crypto_sw_sha2.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/crypto" 
	@${RM} ${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/crypto/atca_crypto_sw_sha2.o.d 
	@${RM} ${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/crypto/atca_crypto_sw_sha2.o 
	 ${MP_CC}  $(MP_EXTRA_CC_PRE) -mmcu=atmega4808  -I "C:\Program Files (x86)\Microchip\MPLABX\v5.15\packs\Microchip\ATmega_DFP\2.0.2\include"  -B "C:\Program Files (x86)\Microchip\MPLABX\v5.15\packs\Microchip\ATmega_DFP\2.0.2\gcc\dev\atmega4808" -g -DDEBUG  -gdwarf-2  -x c -c -D__$(MP_PROCESSOR_OPTION)__  -funsigned-char -funsigned-bitfields -Os -ffunction-sections -fdata-sections -fpack-struct -fshort-enums -Wall -MD -MP -MF "${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/crypto/atca_crypto_sw_sha2.o.d" -MT "${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/crypto/atca_crypto_sw_sha2.o.d" -MT ${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/crypto/atca_crypto_sw_sha2.o  -o ${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/crypto/atca_crypto_sw_sha2.o mcc_generated_files/cryptoauthlib/lib/crypto/atca_crypto_sw_sha2.c  -DXPRJ_default=$(CND_CONF)  $(COMPARISON_BUILD) 
	
${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/crypto/atca_crypto_sw_rand.o: mcc_generated_files/cryptoauthlib/lib/crypto/atca_crypto_sw_rand.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/crypto" 
	@${RM} ${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/crypto/atca_crypto_sw_rand.o.d 
	@${RM} ${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/crypto/atca_crypto_sw_rand.o 
	 ${MP_CC}  $(MP_EXTRA_CC_PRE) -mmcu=atmega4808  -I "C:\Program Files (x86)\Microchip\MPLABX\v5.15\packs\Microchip\ATmega_DFP\2.0.2\include"  -B "C:\Program Files (x86)\Microchip\MPLABX\v5.15\packs\Microchip\ATmega_DFP\2.0.2\gcc\dev\atmega4808" -g -DDEBUG  -gdwarf-2  -x c -c -D__$(MP_PROCESSOR_OPTION)__  -funsigned-char -funsigned-bitfields -Os -ffunction-sections -fdata-sections -fpack-struct -fshort-enums -Wall -MD -MP -MF "${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/crypto/atca_crypto_sw_rand.o.d" -MT "${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/crypto/atca_crypto_sw_rand.o.d" -MT ${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/crypto/atca_crypto_sw_rand.o  -o ${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/crypto/atca_crypto_sw_rand.o mcc_generated_files/cryptoauthlib/lib/crypto/atca_crypto_sw_rand.c  -DXPRJ_default=$(CND_CONF)  $(COMPARISON_BUILD) 
	
${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/hal/hal_timer.o: mcc_generated_files/cryptoauthlib/lib/hal/hal_timer.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/hal" 
	@${RM} ${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/hal/hal_timer.o.d 
	@${RM} ${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/hal/hal_timer.o 
	 ${MP_CC}  $(MP_EXTRA_CC_PRE) -mmcu=atmega4808  -I "C:\Program Files (x86)\Microchip\MPLABX\v5.15\packs\Microchip\ATmega_DFP\2.0.2\include"  -B "C:\Program Files (x86)\Microchip\MPLABX\v5.15\packs\Microchip\ATmega_DFP\2.0.2\gcc\dev\atmega4808" -g -DDEBUG  -gdwarf-2  -x c -c -D__$(MP_PROCESSOR_OPTION)__  -funsigned-char -funsigned-bitfields -Os -ffunction-sections -fdata-sections -fpack-struct -fshort-enums -Wall -MD -MP -MF "${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/hal/hal_timer.o.d" -MT "${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/hal/hal_timer.o.d" -MT ${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/hal/hal_timer.o  -o ${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/hal/hal_timer.o mcc_generated_files/cryptoauthlib/lib/hal/hal_timer.c  -DXPRJ_default=$(CND_CONF)  $(COMPARISON_BUILD) 
	
${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/hal/hal_i2c.o: mcc_generated_files/cryptoauthlib/lib/hal/hal_i2c.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/hal" 
	@${RM} ${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/hal/hal_i2c.o.d 
	@${RM} ${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/hal/hal_i2c.o 
	 ${MP_CC}  $(MP_EXTRA_CC_PRE) -mmcu=atmega4808  -I "C:\Program Files (x86)\Microchip\MPLABX\v5.15\packs\Microchip\ATmega_DFP\2.0.2\include"  -B "C:\Program Files (x86)\Microchip\MPLABX\v5.15\packs\Microchip\ATmega_DFP\2.0.2\gcc\dev\atmega4808" -g -DDEBUG  -gdwarf-2  -x c -c -D__$(MP_PROCESSOR_OPTION)__  -funsigned-char -funsigned-bitfields -Os -ffunction-sections -fdata-sections -fpack-struct -fshort-enums -Wall -MD -MP -MF "${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/hal/hal_i2c.o.d" -MT "${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/hal/hal_i2c.o.d" -MT ${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/hal/hal_i2c.o  -o ${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/hal/hal_i2c.o mcc_generated_files/cryptoauthlib/lib/hal/hal_i2c.c  -DXPRJ_default=$(CND_CONF)  $(COMPARISON_BUILD) 
	
${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/hal/atca_hal.o: mcc_generated_files/cryptoauthlib/lib/hal/atca_hal.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/hal" 
	@${RM} ${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/hal/atca_hal.o.d 
	@${RM} ${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/hal/atca_hal.o 
	 ${MP_CC}  $(MP_EXTRA_CC_PRE) -mmcu=atmega4808  -I "C:\Program Files (x86)\Microchip\MPLABX\v5.15\packs\Microchip\ATmega_DFP\2.0.2\include"  -B "C:\Program Files (x86)\Microchip\MPLABX\v5.15\packs\Microchip\ATmega_DFP\2.0.2\gcc\dev\atmega4808" -g -DDEBUG  -gdwarf-2  -x c -c -D__$(MP_PROCESSOR_OPTION)__  -funsigned-char -funsigned-bitfields -Os -ffunction-sections -fdata-sections -fpack-struct -fshort-enums -Wall -MD -MP -MF "${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/hal/atca_hal.o.d" -MT "${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/hal/atca_hal.o.d" -MT ${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/hal/atca_hal.o  -o ${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/hal/atca_hal.o mcc_generated_files/cryptoauthlib/lib/hal/atca_hal.c  -DXPRJ_default=$(CND_CONF)  $(COMPARISON_BUILD) 
	
${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/host/atca_host.o: mcc_generated_files/cryptoauthlib/lib/host/atca_host.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/host" 
	@${RM} ${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/host/atca_host.o.d 
	@${RM} ${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/host/atca_host.o 
	 ${MP_CC}  $(MP_EXTRA_CC_PRE) -mmcu=atmega4808  -I "C:\Program Files (x86)\Microchip\MPLABX\v5.15\packs\Microchip\ATmega_DFP\2.0.2\include"  -B "C:\Program Files (x86)\Microchip\MPLABX\v5.15\packs\Microchip\ATmega_DFP\2.0.2\gcc\dev\atmega4808" -g -DDEBUG  -gdwarf-2  -x c -c -D__$(MP_PROCESSOR_OPTION)__  -funsigned-char -funsigned-bitfields -Os -ffunction-sections -fdata-sections -fpack-struct -fshort-enums -Wall -MD -MP -MF "${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/host/atca_host.o.d" -MT "${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/host/atca_host.o.d" -MT ${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/host/atca_host.o  -o ${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/host/atca_host.o mcc_generated_files/cryptoauthlib/lib/host/atca_host.c  -DXPRJ_default=$(CND_CONF)  $(COMPARISON_BUILD) 
	
${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/jwt/atca_jwt.o: mcc_generated_files/cryptoauthlib/lib/jwt/atca_jwt.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/jwt" 
	@${RM} ${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/jwt/atca_jwt.o.d 
	@${RM} ${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/jwt/atca_jwt.o 
	 ${MP_CC}  $(MP_EXTRA_CC_PRE) -mmcu=atmega4808  -I "C:\Program Files (x86)\Microchip\MPLABX\v5.15\packs\Microchip\ATmega_DFP\2.0.2\include"  -B "C:\Program Files (x86)\Microchip\MPLABX\v5.15\packs\Microchip\ATmega_DFP\2.0.2\gcc\dev\atmega4808" -g -DDEBUG  -gdwarf-2  -x c -c -D__$(MP_PROCESSOR_OPTION)__  -funsigned-char -funsigned-bitfields -Os -ffunction-sections -fdata-sections -fpack-struct -fshort-enums -Wall -MD -MP -MF "${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/jwt/atca_jwt.o.d" -MT "${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/jwt/atca_jwt.o.d" -MT ${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/jwt/atca_jwt.o  -o ${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/jwt/atca_jwt.o mcc_generated_files/cryptoauthlib/lib/jwt/atca_jwt.c  -DXPRJ_default=$(CND_CONF)  $(COMPARISON_BUILD) 
	
${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/tls/atcatls_cfg.o: mcc_generated_files/cryptoauthlib/lib/tls/atcatls_cfg.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/tls" 
	@${RM} ${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/tls/atcatls_cfg.o.d 
	@${RM} ${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/tls/atcatls_cfg.o 
	 ${MP_CC}  $(MP_EXTRA_CC_PRE) -mmcu=atmega4808  -I "C:\Program Files (x86)\Microchip\MPLABX\v5.15\packs\Microchip\ATmega_DFP\2.0.2\include"  -B "C:\Program Files (x86)\Microchip\MPLABX\v5.15\packs\Microchip\ATmega_DFP\2.0.2\gcc\dev\atmega4808" -g -DDEBUG  -gdwarf-2  -x c -c -D__$(MP_PROCESSOR_OPTION)__  -funsigned-char -funsigned-bitfields -Os -ffunction-sections -fdata-sections -fpack-struct -fshort-enums -Wall -MD -MP -MF "${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/tls/atcatls_cfg.o.d" -MT "${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/tls/atcatls_cfg.o.d" -MT ${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/tls/atcatls_cfg.o  -o ${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/tls/atcatls_cfg.o mcc_generated_files/cryptoauthlib/lib/tls/atcatls_cfg.c  -DXPRJ_default=$(CND_CONF)  $(COMPARISON_BUILD) 
	
${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/tls/atcatls.o: mcc_generated_files/cryptoauthlib/lib/tls/atcatls.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/tls" 
	@${RM} ${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/tls/atcatls.o.d 
	@${RM} ${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/tls/atcatls.o 
	 ${MP_CC}  $(MP_EXTRA_CC_PRE) -mmcu=atmega4808  -I "C:\Program Files (x86)\Microchip\MPLABX\v5.15\packs\Microchip\ATmega_DFP\2.0.2\include"  -B "C:\Program Files (x86)\Microchip\MPLABX\v5.15\packs\Microchip\ATmega_DFP\2.0.2\gcc\dev\atmega4808" -g -DDEBUG  -gdwarf-2  -x c -c -D__$(MP_PROCESSOR_OPTION)__  -funsigned-char -funsigned-bitfields -Os -ffunction-sections -fdata-sections -fpack-struct -fshort-enums -Wall -MD -MP -MF "${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/tls/atcatls.o.d" -MT "${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/tls/atcatls.o.d" -MT ${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/tls/atcatls.o  -o ${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/tls/atcatls.o mcc_generated_files/cryptoauthlib/lib/tls/atcatls.c  -DXPRJ_default=$(CND_CONF)  $(COMPARISON_BUILD) 
	
${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/atca_device.o: mcc_generated_files/cryptoauthlib/lib/atca_device.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib" 
	@${RM} ${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/atca_device.o.d 
	@${RM} ${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/atca_device.o 
	 ${MP_CC}  $(MP_EXTRA_CC_PRE) -mmcu=atmega4808  -I "C:\Program Files (x86)\Microchip\MPLABX\v5.15\packs\Microchip\ATmega_DFP\2.0.2\include"  -B "C:\Program Files (x86)\Microchip\MPLABX\v5.15\packs\Microchip\ATmega_DFP\2.0.2\gcc\dev\atmega4808" -g -DDEBUG  -gdwarf-2  -x c -c -D__$(MP_PROCESSOR_OPTION)__  -funsigned-char -funsigned-bitfields -Os -ffunction-sections -fdata-sections -fpack-struct -fshort-enums -Wall -MD -MP -MF "${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/atca_device.o.d" -MT "${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/atca_device.o.d" -MT ${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/atca_device.o  -o ${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/atca_device.o mcc_generated_files/cryptoauthlib/lib/atca_device.c  -DXPRJ_default=$(CND_CONF)  $(COMPARISON_BUILD) 
	
${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/atca_execution.o: mcc_generated_files/cryptoauthlib/lib/atca_execution.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib" 
	@${RM} ${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/atca_execution.o.d 
	@${RM} ${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/atca_execution.o 
	 ${MP_CC}  $(MP_EXTRA_CC_PRE) -mmcu=atmega4808  -I "C:\Program Files (x86)\Microchip\MPLABX\v5.15\packs\Microchip\ATmega_DFP\2.0.2\include"  -B "C:\Program Files (x86)\Microchip\MPLABX\v5.15\packs\Microchip\ATmega_DFP\2.0.2\gcc\dev\atmega4808" -g -DDEBUG  -gdwarf-2  -x c -c -D__$(MP_PROCESSOR_OPTION)__  -funsigned-char -funsigned-bitfields -Os -ffunction-sections -fdata-sections -fpack-struct -fshort-enums -Wall -MD -MP -MF "${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/atca_execution.o.d" -MT "${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/atca_execution.o.d" -MT ${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/atca_execution.o  -o ${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/atca_execution.o mcc_generated_files/cryptoauthlib/lib/atca_execution.c  -DXPRJ_default=$(CND_CONF)  $(COMPARISON_BUILD) 
	
${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/atca_command.o: mcc_generated_files/cryptoauthlib/lib/atca_command.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib" 
	@${RM} ${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/atca_command.o.d 
	@${RM} ${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/atca_command.o 
	 ${MP_CC}  $(MP_EXTRA_CC_PRE) -mmcu=atmega4808  -I "C:\Program Files (x86)\Microchip\MPLABX\v5.15\packs\Microchip\ATmega_DFP\2.0.2\include"  -B "C:\Program Files (x86)\Microchip\MPLABX\v5.15\packs\Microchip\ATmega_DFP\2.0.2\gcc\dev\atmega4808" -g -DDEBUG  -gdwarf-2  -x c -c -D__$(MP_PROCESSOR_OPTION)__  -funsigned-char -funsigned-bitfields -Os -ffunction-sections -fdata-sections -fpack-struct -fshort-enums -Wall -MD -MP -MF "${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/atca_command.o.d" -MT "${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/atca_command.o.d" -MT ${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/atca_command.o  -o ${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/atca_command.o mcc_generated_files/cryptoauthlib/lib/atca_command.c  -DXPRJ_default=$(CND_CONF)  $(COMPARISON_BUILD) 
	
${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/atca_cfgs.o: mcc_generated_files/cryptoauthlib/lib/atca_cfgs.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib" 
	@${RM} ${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/atca_cfgs.o.d 
	@${RM} ${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/atca_cfgs.o 
	 ${MP_CC}  $(MP_EXTRA_CC_PRE) -mmcu=atmega4808  -I "C:\Program Files (x86)\Microchip\MPLABX\v5.15\packs\Microchip\ATmega_DFP\2.0.2\include"  -B "C:\Program Files (x86)\Microchip\MPLABX\v5.15\packs\Microchip\ATmega_DFP\2.0.2\gcc\dev\atmega4808" -g -DDEBUG  -gdwarf-2  -x c -c -D__$(MP_PROCESSOR_OPTION)__  -funsigned-char -funsigned-bitfields -Os -ffunction-sections -fdata-sections -fpack-struct -fshort-enums -Wall -MD -MP -MF "${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/atca_cfgs.o.d" -MT "${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/atca_cfgs.o.d" -MT ${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/atca_cfgs.o  -o ${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/atca_cfgs.o mcc_generated_files/cryptoauthlib/lib/atca_cfgs.c  -DXPRJ_default=$(CND_CONF)  $(COMPARISON_BUILD) 
	
${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/atca_iface.o: mcc_generated_files/cryptoauthlib/lib/atca_iface.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib" 
	@${RM} ${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/atca_iface.o.d 
	@${RM} ${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/atca_iface.o 
	 ${MP_CC}  $(MP_EXTRA_CC_PRE) -mmcu=atmega4808  -I "C:\Program Files (x86)\Microchip\MPLABX\v5.15\packs\Microchip\ATmega_DFP\2.0.2\include"  -B "C:\Program Files (x86)\Microchip\MPLABX\v5.15\packs\Microchip\ATmega_DFP\2.0.2\gcc\dev\atmega4808" -g -DDEBUG  -gdwarf-2  -x c -c -D__$(MP_PROCESSOR_OPTION)__  -funsigned-char -funsigned-bitfields -Os -ffunction-sections -fdata-sections -fpack-struct -fshort-enums -Wall -MD -MP -MF "${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/atca_iface.o.d" -MT "${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/atca_iface.o.d" -MT ${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/atca_iface.o  -o ${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/atca_iface.o mcc_generated_files/cryptoauthlib/lib/atca_iface.c  -DXPRJ_default=$(CND_CONF)  $(COMPARISON_BUILD) 
	
${OBJECTDIR}/mcc_generated_files/drivers/i2c_simple_master.o: mcc_generated_files/drivers/i2c_simple_master.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/mcc_generated_files/drivers" 
	@${RM} ${OBJECTDIR}/mcc_generated_files/drivers/i2c_simple_master.o.d 
	@${RM} ${OBJECTDIR}/mcc_generated_files/drivers/i2c_simple_master.o 
	 ${MP_CC}  $(MP_EXTRA_CC_PRE) -mmcu=atmega4808  -I "C:\Program Files (x86)\Microchip\MPLABX\v5.15\packs\Microchip\ATmega_DFP\2.0.2\include"  -B "C:\Program Files (x86)\Microchip\MPLABX\v5.15\packs\Microchip\ATmega_DFP\2.0.2\gcc\dev\atmega4808" -g -DDEBUG  -gdwarf-2  -x c -c -D__$(MP_PROCESSOR_OPTION)__  -funsigned-char -funsigned-bitfields -Os -ffunction-sections -fdata-sections -fpack-struct -fshort-enums -Wall -MD -MP -MF "${OBJECTDIR}/mcc_generated_files/drivers/i2c_simple_master.o.d" -MT "${OBJECTDIR}/mcc_generated_files/drivers/i2c_simple_master.o.d" -MT ${OBJECTDIR}/mcc_generated_files/drivers/i2c_simple_master.o  -o ${OBJECTDIR}/mcc_generated_files/drivers/i2c_simple_master.o mcc_generated_files/drivers/i2c_simple_master.c  -DXPRJ_default=$(CND_CONF)  $(COMPARISON_BUILD) 
	
${OBJECTDIR}/mcc_generated_files/mqtt/mqtt_comm_bsd/mqtt_comm_layer.o: mcc_generated_files/mqtt/mqtt_comm_bsd/mqtt_comm_layer.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/mcc_generated_files/mqtt/mqtt_comm_bsd" 
	@${RM} ${OBJECTDIR}/mcc_generated_files/mqtt/mqtt_comm_bsd/mqtt_comm_layer.o.d 
	@${RM} ${OBJECTDIR}/mcc_generated_files/mqtt/mqtt_comm_bsd/mqtt_comm_layer.o 
	 ${MP_CC}  $(MP_EXTRA_CC_PRE) -mmcu=atmega4808  -I "C:\Program Files (x86)\Microchip\MPLABX\v5.15\packs\Microchip\ATmega_DFP\2.0.2\include"  -B "C:\Program Files (x86)\Microchip\MPLABX\v5.15\packs\Microchip\ATmega_DFP\2.0.2\gcc\dev\atmega4808" -g -DDEBUG  -gdwarf-2  -x c -c -D__$(MP_PROCESSOR_OPTION)__  -funsigned-char -funsigned-bitfields -Os -ffunction-sections -fdata-sections -fpack-struct -fshort-enums -Wall -MD -MP -MF "${OBJECTDIR}/mcc_generated_files/mqtt/mqtt_comm_bsd/mqtt_comm_layer.o.d" -MT "${OBJECTDIR}/mcc_generated_files/mqtt/mqtt_comm_bsd/mqtt_comm_layer.o.d" -MT ${OBJECTDIR}/mcc_generated_files/mqtt/mqtt_comm_bsd/mqtt_comm_layer.o  -o ${OBJECTDIR}/mcc_generated_files/mqtt/mqtt_comm_bsd/mqtt_comm_layer.o mcc_generated_files/mqtt/mqtt_comm_bsd/mqtt_comm_layer.c  -DXPRJ_default=$(CND_CONF)  $(COMPARISON_BUILD) 
	
${OBJECTDIR}/mcc_generated_files/mqtt/mqtt_core/mqtt_core.o: mcc_generated_files/mqtt/mqtt_core/mqtt_core.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/mcc_generated_files/mqtt/mqtt_core" 
	@${RM} ${OBJECTDIR}/mcc_generated_files/mqtt/mqtt_core/mqtt_core.o.d 
	@${RM} ${OBJECTDIR}/mcc_generated_files/mqtt/mqtt_core/mqtt_core.o 
	 ${MP_CC}  $(MP_EXTRA_CC_PRE) -mmcu=atmega4808  -I "C:\Program Files (x86)\Microchip\MPLABX\v5.15\packs\Microchip\ATmega_DFP\2.0.2\include"  -B "C:\Program Files (x86)\Microchip\MPLABX\v5.15\packs\Microchip\ATmega_DFP\2.0.2\gcc\dev\atmega4808" -g -DDEBUG  -gdwarf-2  -x c -c -D__$(MP_PROCESSOR_OPTION)__  -funsigned-char -funsigned-bitfields -Os -ffunction-sections -fdata-sections -fpack-struct -fshort-enums -Wall -MD -MP -MF "${OBJECTDIR}/mcc_generated_files/mqtt/mqtt_core/mqtt_core.o.d" -MT "${OBJECTDIR}/mcc_generated_files/mqtt/mqtt_core/mqtt_core.o.d" -MT ${OBJECTDIR}/mcc_generated_files/mqtt/mqtt_core/mqtt_core.o  -o ${OBJECTDIR}/mcc_generated_files/mqtt/mqtt_core/mqtt_core.o mcc_generated_files/mqtt/mqtt_core/mqtt_core.c  -DXPRJ_default=$(CND_CONF)  $(COMPARISON_BUILD) 
	
${OBJECTDIR}/mcc_generated_files/mqtt/mqtt_exchange_buffer/mqtt_exchange_buffer.o: mcc_generated_files/mqtt/mqtt_exchange_buffer/mqtt_exchange_buffer.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/mcc_generated_files/mqtt/mqtt_exchange_buffer" 
	@${RM} ${OBJECTDIR}/mcc_generated_files/mqtt/mqtt_exchange_buffer/mqtt_exchange_buffer.o.d 
	@${RM} ${OBJECTDIR}/mcc_generated_files/mqtt/mqtt_exchange_buffer/mqtt_exchange_buffer.o 
	 ${MP_CC}  $(MP_EXTRA_CC_PRE) -mmcu=atmega4808  -I "C:\Program Files (x86)\Microchip\MPLABX\v5.15\packs\Microchip\ATmega_DFP\2.0.2\include"  -B "C:\Program Files (x86)\Microchip\MPLABX\v5.15\packs\Microchip\ATmega_DFP\2.0.2\gcc\dev\atmega4808" -g -DDEBUG  -gdwarf-2  -x c -c -D__$(MP_PROCESSOR_OPTION)__  -funsigned-char -funsigned-bitfields -Os -ffunction-sections -fdata-sections -fpack-struct -fshort-enums -Wall -MD -MP -MF "${OBJECTDIR}/mcc_generated_files/mqtt/mqtt_exchange_buffer/mqtt_exchange_buffer.o.d" -MT "${OBJECTDIR}/mcc_generated_files/mqtt/mqtt_exchange_buffer/mqtt_exchange_buffer.o.d" -MT ${OBJECTDIR}/mcc_generated_files/mqtt/mqtt_exchange_buffer/mqtt_exchange_buffer.o  -o ${OBJECTDIR}/mcc_generated_files/mqtt/mqtt_exchange_buffer/mqtt_exchange_buffer.o mcc_generated_files/mqtt/mqtt_exchange_buffer/mqtt_exchange_buffer.c  -DXPRJ_default=$(CND_CONF)  $(COMPARISON_BUILD) 
	
${OBJECTDIR}/mcc_generated_files/mqtt/mqtt_packetTransfer_interface.o: mcc_generated_files/mqtt/mqtt_packetTransfer_interface.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/mcc_generated_files/mqtt" 
	@${RM} ${OBJECTDIR}/mcc_generated_files/mqtt/mqtt_packetTransfer_interface.o.d 
	@${RM} ${OBJECTDIR}/mcc_generated_files/mqtt/mqtt_packetTransfer_interface.o 
	 ${MP_CC}  $(MP_EXTRA_CC_PRE) -mmcu=atmega4808  -I "C:\Program Files (x86)\Microchip\MPLABX\v5.15\packs\Microchip\ATmega_DFP\2.0.2\include"  -B "C:\Program Files (x86)\Microchip\MPLABX\v5.15\packs\Microchip\ATmega_DFP\2.0.2\gcc\dev\atmega4808" -g -DDEBUG  -gdwarf-2  -x c -c -D__$(MP_PROCESSOR_OPTION)__  -funsigned-char -funsigned-bitfields -Os -ffunction-sections -fdata-sections -fpack-struct -fshort-enums -Wall -MD -MP -MF "${OBJECTDIR}/mcc_generated_files/mqtt/mqtt_packetTransfer_interface.o.d" -MT "${OBJECTDIR}/mcc_generated_files/mqtt/mqtt_packetTransfer_interface.o.d" -MT ${OBJECTDIR}/mcc_generated_files/mqtt/mqtt_packetTransfer_interface.o  -o ${OBJECTDIR}/mcc_generated_files/mqtt/mqtt_packetTransfer_interface.o mcc_generated_files/mqtt/mqtt_packetTransfer_interface.c  -DXPRJ_default=$(CND_CONF)  $(COMPARISON_BUILD) 
	
${OBJECTDIR}/mcc_generated_files/src/clkctrl.o: mcc_generated_files/src/clkctrl.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/mcc_generated_files/src" 
	@${RM} ${OBJECTDIR}/mcc_generated_files/src/clkctrl.o.d 
	@${RM} ${OBJECTDIR}/mcc_generated_files/src/clkctrl.o 
	 ${MP_CC}  $(MP_EXTRA_CC_PRE) -mmcu=atmega4808  -I "C:\Program Files (x86)\Microchip\MPLABX\v5.15\packs\Microchip\ATmega_DFP\2.0.2\include"  -B "C:\Program Files (x86)\Microchip\MPLABX\v5.15\packs\Microchip\ATmega_DFP\2.0.2\gcc\dev\atmega4808" -g -DDEBUG  -gdwarf-2  -x c -c -D__$(MP_PROCESSOR_OPTION)__  -funsigned-char -funsigned-bitfields -Os -ffunction-sections -fdata-sections -fpack-struct -fshort-enums -Wall -MD -MP -MF "${OBJECTDIR}/mcc_generated_files/src/clkctrl.o.d" -MT "${OBJECTDIR}/mcc_generated_files/src/clkctrl.o.d" -MT ${OBJECTDIR}/mcc_generated_files/src/clkctrl.o  -o ${OBJECTDIR}/mcc_generated_files/src/clkctrl.o mcc_generated_files/src/clkctrl.c  -DXPRJ_default=$(CND_CONF)  $(COMPARISON_BUILD) 
	
${OBJECTDIR}/mcc_generated_files/src/twi0.o: mcc_generated_files/src/twi0.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/mcc_generated_files/src" 
	@${RM} ${OBJECTDIR}/mcc_generated_files/src/twi0.o.d 
	@${RM} ${OBJECTDIR}/mcc_generated_files/src/twi0.o 
	 ${MP_CC}  $(MP_EXTRA_CC_PRE) -mmcu=atmega4808  -I "C:\Program Files (x86)\Microchip\MPLABX\v5.15\packs\Microchip\ATmega_DFP\2.0.2\include"  -B "C:\Program Files (x86)\Microchip\MPLABX\v5.15\packs\Microchip\ATmega_DFP\2.0.2\gcc\dev\atmega4808" -g -DDEBUG  -gdwarf-2  -x c -c -D__$(MP_PROCESSOR_OPTION)__  -funsigned-char -funsigned-bitfields -Os -ffunction-sections -fdata-sections -fpack-struct -fshort-enums -Wall -MD -MP -MF "${OBJECTDIR}/mcc_generated_files/src/twi0.o.d" -MT "${OBJECTDIR}/mcc_generated_files/src/twi0.o.d" -MT ${OBJECTDIR}/mcc_generated_files/src/twi0.o  -o ${OBJECTDIR}/mcc_generated_files/src/twi0.o mcc_generated_files/src/twi0.c  -DXPRJ_default=$(CND_CONF)  $(COMPARISON_BUILD) 
	
${OBJECTDIR}/mcc_generated_files/src/wdt.o: mcc_generated_files/src/wdt.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/mcc_generated_files/src" 
	@${RM} ${OBJECTDIR}/mcc_generated_files/src/wdt.o.d 
	@${RM} ${OBJECTDIR}/mcc_generated_files/src/wdt.o 
	 ${MP_CC}  $(MP_EXTRA_CC_PRE) -mmcu=atmega4808  -I "C:\Program Files (x86)\Microchip\MPLABX\v5.15\packs\Microchip\ATmega_DFP\2.0.2\include"  -B "C:\Program Files (x86)\Microchip\MPLABX\v5.15\packs\Microchip\ATmega_DFP\2.0.2\gcc\dev\atmega4808" -g -DDEBUG  -gdwarf-2  -x c -c -D__$(MP_PROCESSOR_OPTION)__  -funsigned-char -funsigned-bitfields -Os -ffunction-sections -fdata-sections -fpack-struct -fshort-enums -Wall -MD -MP -MF "${OBJECTDIR}/mcc_generated_files/src/wdt.o.d" -MT "${OBJECTDIR}/mcc_generated_files/src/wdt.o.d" -MT ${OBJECTDIR}/mcc_generated_files/src/wdt.o  -o ${OBJECTDIR}/mcc_generated_files/src/wdt.o mcc_generated_files/src/wdt.c  -DXPRJ_default=$(CND_CONF)  $(COMPARISON_BUILD) 
	
${OBJECTDIR}/mcc_generated_files/src/system.o: mcc_generated_files/src/system.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/mcc_generated_files/src" 
	@${RM} ${OBJECTDIR}/mcc_generated_files/src/system.o.d 
	@${RM} ${OBJECTDIR}/mcc_generated_files/src/system.o 
	 ${MP_CC}  $(MP_EXTRA_CC_PRE) -mmcu=atmega4808  -I "C:\Program Files (x86)\Microchip\MPLABX\v5.15\packs\Microchip\ATmega_DFP\2.0.2\include"  -B "C:\Program Files (x86)\Microchip\MPLABX\v5.15\packs\Microchip\ATmega_DFP\2.0.2\gcc\dev\atmega4808" -g -DDEBUG  -gdwarf-2  -x c -c -D__$(MP_PROCESSOR_OPTION)__  -funsigned-char -funsigned-bitfields -Os -ffunction-sections -fdata-sections -fpack-struct -fshort-enums -Wall -MD -MP -MF "${OBJECTDIR}/mcc_generated_files/src/system.o.d" -MT "${OBJECTDIR}/mcc_generated_files/src/system.o.d" -MT ${OBJECTDIR}/mcc_generated_files/src/system.o  -o ${OBJECTDIR}/mcc_generated_files/src/system.o mcc_generated_files/src/system.c  -DXPRJ_default=$(CND_CONF)  $(COMPARISON_BUILD) 
	
${OBJECTDIR}/mcc_generated_files/src/i2c_types.o: mcc_generated_files/src/i2c_types.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/mcc_generated_files/src" 
	@${RM} ${OBJECTDIR}/mcc_generated_files/src/i2c_types.o.d 
	@${RM} ${OBJECTDIR}/mcc_generated_files/src/i2c_types.o 
	 ${MP_CC}  $(MP_EXTRA_CC_PRE) -mmcu=atmega4808  -I "C:\Program Files (x86)\Microchip\MPLABX\v5.15\packs\Microchip\ATmega_DFP\2.0.2\include"  -B "C:\Program Files (x86)\Microchip\MPLABX\v5.15\packs\Microchip\ATmega_DFP\2.0.2\gcc\dev\atmega4808" -g -DDEBUG  -gdwarf-2  -x c -c -D__$(MP_PROCESSOR_OPTION)__  -funsigned-char -funsigned-bitfields -Os -ffunction-sections -fdata-sections -fpack-struct -fshort-enums -Wall -MD -MP -MF "${OBJECTDIR}/mcc_generated_files/src/i2c_types.o.d" -MT "${OBJECTDIR}/mcc_generated_files/src/i2c_types.o.d" -MT ${OBJECTDIR}/mcc_generated_files/src/i2c_types.o  -o ${OBJECTDIR}/mcc_generated_files/src/i2c_types.o mcc_generated_files/src/i2c_types.c  -DXPRJ_default=$(CND_CONF)  $(COMPARISON_BUILD) 
	
${OBJECTDIR}/mcc_generated_files/src/usart2.o: mcc_generated_files/src/usart2.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/mcc_generated_files/src" 
	@${RM} ${OBJECTDIR}/mcc_generated_files/src/usart2.o.d 
	@${RM} ${OBJECTDIR}/mcc_generated_files/src/usart2.o 
	 ${MP_CC}  $(MP_EXTRA_CC_PRE) -mmcu=atmega4808  -I "C:\Program Files (x86)\Microchip\MPLABX\v5.15\packs\Microchip\ATmega_DFP\2.0.2\include"  -B "C:\Program Files (x86)\Microchip\MPLABX\v5.15\packs\Microchip\ATmega_DFP\2.0.2\gcc\dev\atmega4808" -g -DDEBUG  -gdwarf-2  -x c -c -D__$(MP_PROCESSOR_OPTION)__  -funsigned-char -funsigned-bitfields -Os -ffunction-sections -fdata-sections -fpack-struct -fshort-enums -Wall -MD -MP -MF "${OBJECTDIR}/mcc_generated_files/src/usart2.o.d" -MT "${OBJECTDIR}/mcc_generated_files/src/usart2.o.d" -MT ${OBJECTDIR}/mcc_generated_files/src/usart2.o  -o ${OBJECTDIR}/mcc_generated_files/src/usart2.o mcc_generated_files/src/usart2.c  -DXPRJ_default=$(CND_CONF)  $(COMPARISON_BUILD) 
	
${OBJECTDIR}/mcc_generated_files/src/rtc.o: mcc_generated_files/src/rtc.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/mcc_generated_files/src" 
	@${RM} ${OBJECTDIR}/mcc_generated_files/src/rtc.o.d 
	@${RM} ${OBJECTDIR}/mcc_generated_files/src/rtc.o 
	 ${MP_CC}  $(MP_EXTRA_CC_PRE) -mmcu=atmega4808  -I "C:\Program Files (x86)\Microchip\MPLABX\v5.15\packs\Microchip\ATmega_DFP\2.0.2\include"  -B "C:\Program Files (x86)\Microchip\MPLABX\v5.15\packs\Microchip\ATmega_DFP\2.0.2\gcc\dev\atmega4808" -g -DDEBUG  -gdwarf-2  -x c -c -D__$(MP_PROCESSOR_OPTION)__  -funsigned-char -funsigned-bitfields -Os -ffunction-sections -fdata-sections -fpack-struct -fshort-enums -Wall -MD -MP -MF "${OBJECTDIR}/mcc_generated_files/src/rtc.o.d" -MT "${OBJECTDIR}/mcc_generated_files/src/rtc.o.d" -MT ${OBJECTDIR}/mcc_generated_files/src/rtc.o  -o ${OBJECTDIR}/mcc_generated_files/src/rtc.o mcc_generated_files/src/rtc.c  -DXPRJ_default=$(CND_CONF)  $(COMPARISON_BUILD) 
	
${OBJECTDIR}/mcc_generated_files/src/pin_manager.o: mcc_generated_files/src/pin_manager.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/mcc_generated_files/src" 
	@${RM} ${OBJECTDIR}/mcc_generated_files/src/pin_manager.o.d 
	@${RM} ${OBJECTDIR}/mcc_generated_files/src/pin_manager.o 
	 ${MP_CC}  $(MP_EXTRA_CC_PRE) -mmcu=atmega4808  -I "C:\Program Files (x86)\Microchip\MPLABX\v5.15\packs\Microchip\ATmega_DFP\2.0.2\include"  -B "C:\Program Files (x86)\Microchip\MPLABX\v5.15\packs\Microchip\ATmega_DFP\2.0.2\gcc\dev\atmega4808" -g -DDEBUG  -gdwarf-2  -x c -c -D__$(MP_PROCESSOR_OPTION)__  -funsigned-char -funsigned-bitfields -Os -ffunction-sections -fdata-sections -fpack-struct -fshort-enums -Wall -MD -MP -MF "${OBJECTDIR}/mcc_generated_files/src/pin_manager.o.d" -MT "${OBJECTDIR}/mcc_generated_files/src/pin_manager.o.d" -MT ${OBJECTDIR}/mcc_generated_files/src/pin_manager.o  -o ${OBJECTDIR}/mcc_generated_files/src/pin_manager.o mcc_generated_files/src/pin_manager.c  -DXPRJ_default=$(CND_CONF)  $(COMPARISON_BUILD) 
	
${OBJECTDIR}/mcc_generated_files/src/cpuint.o: mcc_generated_files/src/cpuint.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/mcc_generated_files/src" 
	@${RM} ${OBJECTDIR}/mcc_generated_files/src/cpuint.o.d 
	@${RM} ${OBJECTDIR}/mcc_generated_files/src/cpuint.o 
	 ${MP_CC}  $(MP_EXTRA_CC_PRE) -mmcu=atmega4808  -I "C:\Program Files (x86)\Microchip\MPLABX\v5.15\packs\Microchip\ATmega_DFP\2.0.2\include"  -B "C:\Program Files (x86)\Microchip\MPLABX\v5.15\packs\Microchip\ATmega_DFP\2.0.2\gcc\dev\atmega4808" -g -DDEBUG  -gdwarf-2  -x c -c -D__$(MP_PROCESSOR_OPTION)__  -funsigned-char -funsigned-bitfields -Os -ffunction-sections -fdata-sections -fpack-struct -fshort-enums -Wall -MD -MP -MF "${OBJECTDIR}/mcc_generated_files/src/cpuint.o.d" -MT "${OBJECTDIR}/mcc_generated_files/src/cpuint.o.d" -MT ${OBJECTDIR}/mcc_generated_files/src/cpuint.o  -o ${OBJECTDIR}/mcc_generated_files/src/cpuint.o mcc_generated_files/src/cpuint.c  -DXPRJ_default=$(CND_CONF)  $(COMPARISON_BUILD) 
	
${OBJECTDIR}/mcc_generated_files/src/spi0.o: mcc_generated_files/src/spi0.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/mcc_generated_files/src" 
	@${RM} ${OBJECTDIR}/mcc_generated_files/src/spi0.o.d 
	@${RM} ${OBJECTDIR}/mcc_generated_files/src/spi0.o 
	 ${MP_CC}  $(MP_EXTRA_CC_PRE) -mmcu=atmega4808  -I "C:\Program Files (x86)\Microchip\MPLABX\v5.15\packs\Microchip\ATmega_DFP\2.0.2\include"  -B "C:\Program Files (x86)\Microchip\MPLABX\v5.15\packs\Microchip\ATmega_DFP\2.0.2\gcc\dev\atmega4808" -g -DDEBUG  -gdwarf-2  -x c -c -D__$(MP_PROCESSOR_OPTION)__  -funsigned-char -funsigned-bitfields -Os -ffunction-sections -fdata-sections -fpack-struct -fshort-enums -Wall -MD -MP -MF "${OBJECTDIR}/mcc_generated_files/src/spi0.o.d" -MT "${OBJECTDIR}/mcc_generated_files/src/spi0.o.d" -MT ${OBJECTDIR}/mcc_generated_files/src/spi0.o  -o ${OBJECTDIR}/mcc_generated_files/src/spi0.o mcc_generated_files/src/spi0.c  -DXPRJ_default=$(CND_CONF)  $(COMPARISON_BUILD) 
	
${OBJECTDIR}/mcc_generated_files/src/adc0.o: mcc_generated_files/src/adc0.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/mcc_generated_files/src" 
	@${RM} ${OBJECTDIR}/mcc_generated_files/src/adc0.o.d 
	@${RM} ${OBJECTDIR}/mcc_generated_files/src/adc0.o 
	 ${MP_CC}  $(MP_EXTRA_CC_PRE) -mmcu=atmega4808  -I "C:\Program Files (x86)\Microchip\MPLABX\v5.15\packs\Microchip\ATmega_DFP\2.0.2\include"  -B "C:\Program Files (x86)\Microchip\MPLABX\v5.15\packs\Microchip\ATmega_DFP\2.0.2\gcc\dev\atmega4808" -g -DDEBUG  -gdwarf-2  -x c -c -D__$(MP_PROCESSOR_OPTION)__  -funsigned-char -funsigned-bitfields -Os -ffunction-sections -fdata-sections -fpack-struct -fshort-enums -Wall -MD -MP -MF "${OBJECTDIR}/mcc_generated_files/src/adc0.o.d" -MT "${OBJECTDIR}/mcc_generated_files/src/adc0.o.d" -MT ${OBJECTDIR}/mcc_generated_files/src/adc0.o  -o ${OBJECTDIR}/mcc_generated_files/src/adc0.o mcc_generated_files/src/adc0.c  -DXPRJ_default=$(CND_CONF)  $(COMPARISON_BUILD) 
	
${OBJECTDIR}/mcc_generated_files/src/slpctrl.o: mcc_generated_files/src/slpctrl.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/mcc_generated_files/src" 
	@${RM} ${OBJECTDIR}/mcc_generated_files/src/slpctrl.o.d 
	@${RM} ${OBJECTDIR}/mcc_generated_files/src/slpctrl.o 
	 ${MP_CC}  $(MP_EXTRA_CC_PRE) -mmcu=atmega4808  -I "C:\Program Files (x86)\Microchip\MPLABX\v5.15\packs\Microchip\ATmega_DFP\2.0.2\include"  -B "C:\Program Files (x86)\Microchip\MPLABX\v5.15\packs\Microchip\ATmega_DFP\2.0.2\gcc\dev\atmega4808" -g -DDEBUG  -gdwarf-2  -x c -c -D__$(MP_PROCESSOR_OPTION)__  -funsigned-char -funsigned-bitfields -Os -ffunction-sections -fdata-sections -fpack-struct -fshort-enums -Wall -MD -MP -MF "${OBJECTDIR}/mcc_generated_files/src/slpctrl.o.d" -MT "${OBJECTDIR}/mcc_generated_files/src/slpctrl.o.d" -MT ${OBJECTDIR}/mcc_generated_files/src/slpctrl.o  -o ${OBJECTDIR}/mcc_generated_files/src/slpctrl.o mcc_generated_files/src/slpctrl.c  -DXPRJ_default=$(CND_CONF)  $(COMPARISON_BUILD) 
	
${OBJECTDIR}/mcc_generated_files/src/bod.o: mcc_generated_files/src/bod.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/mcc_generated_files/src" 
	@${RM} ${OBJECTDIR}/mcc_generated_files/src/bod.o.d 
	@${RM} ${OBJECTDIR}/mcc_generated_files/src/bod.o 
	 ${MP_CC}  $(MP_EXTRA_CC_PRE) -mmcu=atmega4808  -I "C:\Program Files (x86)\Microchip\MPLABX\v5.15\packs\Microchip\ATmega_DFP\2.0.2\include"  -B "C:\Program Files (x86)\Microchip\MPLABX\v5.15\packs\Microchip\ATmega_DFP\2.0.2\gcc\dev\atmega4808" -g -DDEBUG  -gdwarf-2  -x c -c -D__$(MP_PROCESSOR_OPTION)__  -funsigned-char -funsigned-bitfields -Os -ffunction-sections -fdata-sections -fpack-struct -fshort-enums -Wall -MD -MP -MF "${OBJECTDIR}/mcc_generated_files/src/bod.o.d" -MT "${OBJECTDIR}/mcc_generated_files/src/bod.o.d" -MT ${OBJECTDIR}/mcc_generated_files/src/bod.o  -o ${OBJECTDIR}/mcc_generated_files/src/bod.o mcc_generated_files/src/bod.c  -DXPRJ_default=$(CND_CONF)  $(COMPARISON_BUILD) 
	
${OBJECTDIR}/mcc_generated_files/winc/bsp/source/nm_bsp_mega.o: mcc_generated_files/winc/bsp/source/nm_bsp_mega.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/mcc_generated_files/winc/bsp/source" 
	@${RM} ${OBJECTDIR}/mcc_generated_files/winc/bsp/source/nm_bsp_mega.o.d 
	@${RM} ${OBJECTDIR}/mcc_generated_files/winc/bsp/source/nm_bsp_mega.o 
	 ${MP_CC}  $(MP_EXTRA_CC_PRE) -mmcu=atmega4808  -I "C:\Program Files (x86)\Microchip\MPLABX\v5.15\packs\Microchip\ATmega_DFP\2.0.2\include"  -B "C:\Program Files (x86)\Microchip\MPLABX\v5.15\packs\Microchip\ATmega_DFP\2.0.2\gcc\dev\atmega4808" -g -DDEBUG  -gdwarf-2  -x c -c -D__$(MP_PROCESSOR_OPTION)__  -funsigned-char -funsigned-bitfields -Os -ffunction-sections -fdata-sections -fpack-struct -fshort-enums -Wall -MD -MP -MF "${OBJECTDIR}/mcc_generated_files/winc/bsp/source/nm_bsp_mega.o.d" -MT "${OBJECTDIR}/mcc_generated_files/winc/bsp/source/nm_bsp_mega.o.d" -MT ${OBJECTDIR}/mcc_generated_files/winc/bsp/source/nm_bsp_mega.o  -o ${OBJECTDIR}/mcc_generated_files/winc/bsp/source/nm_bsp_mega.o mcc_generated_files/winc/bsp/source/nm_bsp_mega.c  -DXPRJ_default=$(CND_CONF)  $(COMPARISON_BUILD) 
	
${OBJECTDIR}/mcc_generated_files/winc/bus_wrapper/source/nm_bus_wrapper_mega.o: mcc_generated_files/winc/bus_wrapper/source/nm_bus_wrapper_mega.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/mcc_generated_files/winc/bus_wrapper/source" 
	@${RM} ${OBJECTDIR}/mcc_generated_files/winc/bus_wrapper/source/nm_bus_wrapper_mega.o.d 
	@${RM} ${OBJECTDIR}/mcc_generated_files/winc/bus_wrapper/source/nm_bus_wrapper_mega.o 
	 ${MP_CC}  $(MP_EXTRA_CC_PRE) -mmcu=atmega4808  -I "C:\Program Files (x86)\Microchip\MPLABX\v5.15\packs\Microchip\ATmega_DFP\2.0.2\include"  -B "C:\Program Files (x86)\Microchip\MPLABX\v5.15\packs\Microchip\ATmega_DFP\2.0.2\gcc\dev\atmega4808" -g -DDEBUG  -gdwarf-2  -x c -c -D__$(MP_PROCESSOR_OPTION)__  -funsigned-char -funsigned-bitfields -Os -ffunction-sections -fdata-sections -fpack-struct -fshort-enums -Wall -MD -MP -MF "${OBJECTDIR}/mcc_generated_files/winc/bus_wrapper/source/nm_bus_wrapper_mega.o.d" -MT "${OBJECTDIR}/mcc_generated_files/winc/bus_wrapper/source/nm_bus_wrapper_mega.o.d" -MT ${OBJECTDIR}/mcc_generated_files/winc/bus_wrapper/source/nm_bus_wrapper_mega.o  -o ${OBJECTDIR}/mcc_generated_files/winc/bus_wrapper/source/nm_bus_wrapper_mega.o mcc_generated_files/winc/bus_wrapper/source/nm_bus_wrapper_mega.c  -DXPRJ_default=$(CND_CONF)  $(COMPARISON_BUILD) 
	
${OBJECTDIR}/mcc_generated_files/winc/common/source/nm_common.o: mcc_generated_files/winc/common/source/nm_common.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/mcc_generated_files/winc/common/source" 
	@${RM} ${OBJECTDIR}/mcc_generated_files/winc/common/source/nm_common.o.d 
	@${RM} ${OBJECTDIR}/mcc_generated_files/winc/common/source/nm_common.o 
	 ${MP_CC}  $(MP_EXTRA_CC_PRE) -mmcu=atmega4808  -I "C:\Program Files (x86)\Microchip\MPLABX\v5.15\packs\Microchip\ATmega_DFP\2.0.2\include"  -B "C:\Program Files (x86)\Microchip\MPLABX\v5.15\packs\Microchip\ATmega_DFP\2.0.2\gcc\dev\atmega4808" -g -DDEBUG  -gdwarf-2  -x c -c -D__$(MP_PROCESSOR_OPTION)__  -funsigned-char -funsigned-bitfields -Os -ffunction-sections -fdata-sections -fpack-struct -fshort-enums -Wall -MD -MP -MF "${OBJECTDIR}/mcc_generated_files/winc/common/source/nm_common.o.d" -MT "${OBJECTDIR}/mcc_generated_files/winc/common/source/nm_common.o.d" -MT ${OBJECTDIR}/mcc_generated_files/winc/common/source/nm_common.o  -o ${OBJECTDIR}/mcc_generated_files/winc/common/source/nm_common.o mcc_generated_files/winc/common/source/nm_common.c  -DXPRJ_default=$(CND_CONF)  $(COMPARISON_BUILD) 
	
${OBJECTDIR}/mcc_generated_files/winc/driver/source/m2m_ate_mode.o: mcc_generated_files/winc/driver/source/m2m_ate_mode.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/mcc_generated_files/winc/driver/source" 
	@${RM} ${OBJECTDIR}/mcc_generated_files/winc/driver/source/m2m_ate_mode.o.d 
	@${RM} ${OBJECTDIR}/mcc_generated_files/winc/driver/source/m2m_ate_mode.o 
	 ${MP_CC}  $(MP_EXTRA_CC_PRE) -mmcu=atmega4808  -I "C:\Program Files (x86)\Microchip\MPLABX\v5.15\packs\Microchip\ATmega_DFP\2.0.2\include"  -B "C:\Program Files (x86)\Microchip\MPLABX\v5.15\packs\Microchip\ATmega_DFP\2.0.2\gcc\dev\atmega4808" -g -DDEBUG  -gdwarf-2  -x c -c -D__$(MP_PROCESSOR_OPTION)__  -funsigned-char -funsigned-bitfields -Os -ffunction-sections -fdata-sections -fpack-struct -fshort-enums -Wall -MD -MP -MF "${OBJECTDIR}/mcc_generated_files/winc/driver/source/m2m_ate_mode.o.d" -MT "${OBJECTDIR}/mcc_generated_files/winc/driver/source/m2m_ate_mode.o.d" -MT ${OBJECTDIR}/mcc_generated_files/winc/driver/source/m2m_ate_mode.o  -o ${OBJECTDIR}/mcc_generated_files/winc/driver/source/m2m_ate_mode.o mcc_generated_files/winc/driver/source/m2m_ate_mode.c  -DXPRJ_default=$(CND_CONF)  $(COMPARISON_BUILD) 
	
${OBJECTDIR}/mcc_generated_files/winc/driver/source/m2m_ssl.o: mcc_generated_files/winc/driver/source/m2m_ssl.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/mcc_generated_files/winc/driver/source" 
	@${RM} ${OBJECTDIR}/mcc_generated_files/winc/driver/source/m2m_ssl.o.d 
	@${RM} ${OBJECTDIR}/mcc_generated_files/winc/driver/source/m2m_ssl.o 
	 ${MP_CC}  $(MP_EXTRA_CC_PRE) -mmcu=atmega4808  -I "C:\Program Files (x86)\Microchip\MPLABX\v5.15\packs\Microchip\ATmega_DFP\2.0.2\include"  -B "C:\Program Files (x86)\Microchip\MPLABX\v5.15\packs\Microchip\ATmega_DFP\2.0.2\gcc\dev\atmega4808" -g -DDEBUG  -gdwarf-2  -x c -c -D__$(MP_PROCESSOR_OPTION)__  -funsigned-char -funsigned-bitfields -Os -ffunction-sections -fdata-sections -fpack-struct -fshort-enums -Wall -MD -MP -MF "${OBJECTDIR}/mcc_generated_files/winc/driver/source/m2m_ssl.o.d" -MT "${OBJECTDIR}/mcc_generated_files/winc/driver/source/m2m_ssl.o.d" -MT ${OBJECTDIR}/mcc_generated_files/winc/driver/source/m2m_ssl.o  -o ${OBJECTDIR}/mcc_generated_files/winc/driver/source/m2m_ssl.o mcc_generated_files/winc/driver/source/m2m_ssl.c  -DXPRJ_default=$(CND_CONF)  $(COMPARISON_BUILD) 
	
${OBJECTDIR}/mcc_generated_files/winc/driver/source/m2m_ota.o: mcc_generated_files/winc/driver/source/m2m_ota.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/mcc_generated_files/winc/driver/source" 
	@${RM} ${OBJECTDIR}/mcc_generated_files/winc/driver/source/m2m_ota.o.d 
	@${RM} ${OBJECTDIR}/mcc_generated_files/winc/driver/source/m2m_ota.o 
	 ${MP_CC}  $(MP_EXTRA_CC_PRE) -mmcu=atmega4808  -I "C:\Program Files (x86)\Microchip\MPLABX\v5.15\packs\Microchip\ATmega_DFP\2.0.2\include"  -B "C:\Program Files (x86)\Microchip\MPLABX\v5.15\packs\Microchip\ATmega_DFP\2.0.2\gcc\dev\atmega4808" -g -DDEBUG  -gdwarf-2  -x c -c -D__$(MP_PROCESSOR_OPTION)__  -funsigned-char -funsigned-bitfields -Os -ffunction-sections -fdata-sections -fpack-struct -fshort-enums -Wall -MD -MP -MF "${OBJECTDIR}/mcc_generated_files/winc/driver/source/m2m_ota.o.d" -MT "${OBJECTDIR}/mcc_generated_files/winc/driver/source/m2m_ota.o.d" -MT ${OBJECTDIR}/mcc_generated_files/winc/driver/source/m2m_ota.o  -o ${OBJECTDIR}/mcc_generated_files/winc/driver/source/m2m_ota.o mcc_generated_files/winc/driver/source/m2m_ota.c  -DXPRJ_default=$(CND_CONF)  $(COMPARISON_BUILD) 
	
${OBJECTDIR}/mcc_generated_files/winc/driver/source/nmspi.o: mcc_generated_files/winc/driver/source/nmspi.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/mcc_generated_files/winc/driver/source" 
	@${RM} ${OBJECTDIR}/mcc_generated_files/winc/driver/source/nmspi.o.d 
	@${RM} ${OBJECTDIR}/mcc_generated_files/winc/driver/source/nmspi.o 
	 ${MP_CC}  $(MP_EXTRA_CC_PRE) -mmcu=atmega4808  -I "C:\Program Files (x86)\Microchip\MPLABX\v5.15\packs\Microchip\ATmega_DFP\2.0.2\include"  -B "C:\Program Files (x86)\Microchip\MPLABX\v5.15\packs\Microchip\ATmega_DFP\2.0.2\gcc\dev\atmega4808" -g -DDEBUG  -gdwarf-2  -x c -c -D__$(MP_PROCESSOR_OPTION)__  -funsigned-char -funsigned-bitfields -Os -ffunction-sections -fdata-sections -fpack-struct -fshort-enums -Wall -MD -MP -MF "${OBJECTDIR}/mcc_generated_files/winc/driver/source/nmspi.o.d" -MT "${OBJECTDIR}/mcc_generated_files/winc/driver/source/nmspi.o.d" -MT ${OBJECTDIR}/mcc_generated_files/winc/driver/source/nmspi.o  -o ${OBJECTDIR}/mcc_generated_files/winc/driver/source/nmspi.o mcc_generated_files/winc/driver/source/nmspi.c  -DXPRJ_default=$(CND_CONF)  $(COMPARISON_BUILD) 
	
${OBJECTDIR}/mcc_generated_files/winc/driver/source/m2m_hif.o: mcc_generated_files/winc/driver/source/m2m_hif.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/mcc_generated_files/winc/driver/source" 
	@${RM} ${OBJECTDIR}/mcc_generated_files/winc/driver/source/m2m_hif.o.d 
	@${RM} ${OBJECTDIR}/mcc_generated_files/winc/driver/source/m2m_hif.o 
	 ${MP_CC}  $(MP_EXTRA_CC_PRE) -mmcu=atmega4808  -I "C:\Program Files (x86)\Microchip\MPLABX\v5.15\packs\Microchip\ATmega_DFP\2.0.2\include"  -B "C:\Program Files (x86)\Microchip\MPLABX\v5.15\packs\Microchip\ATmega_DFP\2.0.2\gcc\dev\atmega4808" -g -DDEBUG  -gdwarf-2  -x c -c -D__$(MP_PROCESSOR_OPTION)__  -funsigned-char -funsigned-bitfields -Os -ffunction-sections -fdata-sections -fpack-struct -fshort-enums -Wall -MD -MP -MF "${OBJECTDIR}/mcc_generated_files/winc/driver/source/m2m_hif.o.d" -MT "${OBJECTDIR}/mcc_generated_files/winc/driver/source/m2m_hif.o.d" -MT ${OBJECTDIR}/mcc_generated_files/winc/driver/source/m2m_hif.o  -o ${OBJECTDIR}/mcc_generated_files/winc/driver/source/m2m_hif.o mcc_generated_files/winc/driver/source/m2m_hif.c  -DXPRJ_default=$(CND_CONF)  $(COMPARISON_BUILD) 
	
${OBJECTDIR}/mcc_generated_files/winc/driver/source/m2m_periph.o: mcc_generated_files/winc/driver/source/m2m_periph.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/mcc_generated_files/winc/driver/source" 
	@${RM} ${OBJECTDIR}/mcc_generated_files/winc/driver/source/m2m_periph.o.d 
	@${RM} ${OBJECTDIR}/mcc_generated_files/winc/driver/source/m2m_periph.o 
	 ${MP_CC}  $(MP_EXTRA_CC_PRE) -mmcu=atmega4808  -I "C:\Program Files (x86)\Microchip\MPLABX\v5.15\packs\Microchip\ATmega_DFP\2.0.2\include"  -B "C:\Program Files (x86)\Microchip\MPLABX\v5.15\packs\Microchip\ATmega_DFP\2.0.2\gcc\dev\atmega4808" -g -DDEBUG  -gdwarf-2  -x c -c -D__$(MP_PROCESSOR_OPTION)__  -funsigned-char -funsigned-bitfields -Os -ffunction-sections -fdata-sections -fpack-struct -fshort-enums -Wall -MD -MP -MF "${OBJECTDIR}/mcc_generated_files/winc/driver/source/m2m_periph.o.d" -MT "${OBJECTDIR}/mcc_generated_files/winc/driver/source/m2m_periph.o.d" -MT ${OBJECTDIR}/mcc_generated_files/winc/driver/source/m2m_periph.o  -o ${OBJECTDIR}/mcc_generated_files/winc/driver/source/m2m_periph.o mcc_generated_files/winc/driver/source/m2m_periph.c  -DXPRJ_default=$(CND_CONF)  $(COMPARISON_BUILD) 
	
${OBJECTDIR}/mcc_generated_files/winc/driver/source/nmasic.o: mcc_generated_files/winc/driver/source/nmasic.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/mcc_generated_files/winc/driver/source" 
	@${RM} ${OBJECTDIR}/mcc_generated_files/winc/driver/source/nmasic.o.d 
	@${RM} ${OBJECTDIR}/mcc_generated_files/winc/driver/source/nmasic.o 
	 ${MP_CC}  $(MP_EXTRA_CC_PRE) -mmcu=atmega4808  -I "C:\Program Files (x86)\Microchip\MPLABX\v5.15\packs\Microchip\ATmega_DFP\2.0.2\include"  -B "C:\Program Files (x86)\Microchip\MPLABX\v5.15\packs\Microchip\ATmega_DFP\2.0.2\gcc\dev\atmega4808" -g -DDEBUG  -gdwarf-2  -x c -c -D__$(MP_PROCESSOR_OPTION)__  -funsigned-char -funsigned-bitfields -Os -ffunction-sections -fdata-sections -fpack-struct -fshort-enums -Wall -MD -MP -MF "${OBJECTDIR}/mcc_generated_files/winc/driver/source/nmasic.o.d" -MT "${OBJECTDIR}/mcc_generated_files/winc/driver/source/nmasic.o.d" -MT ${OBJECTDIR}/mcc_generated_files/winc/driver/source/nmasic.o  -o ${OBJECTDIR}/mcc_generated_files/winc/driver/source/nmasic.o mcc_generated_files/winc/driver/source/nmasic.c  -DXPRJ_default=$(CND_CONF)  $(COMPARISON_BUILD) 
	
${OBJECTDIR}/mcc_generated_files/winc/driver/source/nmdrv.o: mcc_generated_files/winc/driver/source/nmdrv.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/mcc_generated_files/winc/driver/source" 
	@${RM} ${OBJECTDIR}/mcc_generated_files/winc/driver/source/nmdrv.o.d 
	@${RM} ${OBJECTDIR}/mcc_generated_files/winc/driver/source/nmdrv.o 
	 ${MP_CC}  $(MP_EXTRA_CC_PRE) -mmcu=atmega4808  -I "C:\Program Files (x86)\Microchip\MPLABX\v5.15\packs\Microchip\ATmega_DFP\2.0.2\include"  -B "C:\Program Files (x86)\Microchip\MPLABX\v5.15\packs\Microchip\ATmega_DFP\2.0.2\gcc\dev\atmega4808" -g -DDEBUG  -gdwarf-2  -x c -c -D__$(MP_PROCESSOR_OPTION)__  -funsigned-char -funsigned-bitfields -Os -ffunction-sections -fdata-sections -fpack-struct -fshort-enums -Wall -MD -MP -MF "${OBJECTDIR}/mcc_generated_files/winc/driver/source/nmdrv.o.d" -MT "${OBJECTDIR}/mcc_generated_files/winc/driver/source/nmdrv.o.d" -MT ${OBJECTDIR}/mcc_generated_files/winc/driver/source/nmdrv.o  -o ${OBJECTDIR}/mcc_generated_files/winc/driver/source/nmdrv.o mcc_generated_files/winc/driver/source/nmdrv.c  -DXPRJ_default=$(CND_CONF)  $(COMPARISON_BUILD) 
	
${OBJECTDIR}/mcc_generated_files/winc/driver/source/m2m_wifi.o: mcc_generated_files/winc/driver/source/m2m_wifi.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/mcc_generated_files/winc/driver/source" 
	@${RM} ${OBJECTDIR}/mcc_generated_files/winc/driver/source/m2m_wifi.o.d 
	@${RM} ${OBJECTDIR}/mcc_generated_files/winc/driver/source/m2m_wifi.o 
	 ${MP_CC}  $(MP_EXTRA_CC_PRE) -mmcu=atmega4808  -I "C:\Program Files (x86)\Microchip\MPLABX\v5.15\packs\Microchip\ATmega_DFP\2.0.2\include"  -B "C:\Program Files (x86)\Microchip\MPLABX\v5.15\packs\Microchip\ATmega_DFP\2.0.2\gcc\dev\atmega4808" -g -DDEBUG  -gdwarf-2  -x c -c -D__$(MP_PROCESSOR_OPTION)__  -funsigned-char -funsigned-bitfields -Os -ffunction-sections -fdata-sections -fpack-struct -fshort-enums -Wall -MD -MP -MF "${OBJECTDIR}/mcc_generated_files/winc/driver/source/m2m_wifi.o.d" -MT "${OBJECTDIR}/mcc_generated_files/winc/driver/source/m2m_wifi.o.d" -MT ${OBJECTDIR}/mcc_generated_files/winc/driver/source/m2m_wifi.o  -o ${OBJECTDIR}/mcc_generated_files/winc/driver/source/m2m_wifi.o mcc_generated_files/winc/driver/source/m2m_wifi.c  -DXPRJ_default=$(CND_CONF)  $(COMPARISON_BUILD) 
	
${OBJECTDIR}/mcc_generated_files/winc/driver/source/m2m_crypto.o: mcc_generated_files/winc/driver/source/m2m_crypto.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/mcc_generated_files/winc/driver/source" 
	@${RM} ${OBJECTDIR}/mcc_generated_files/winc/driver/source/m2m_crypto.o.d 
	@${RM} ${OBJECTDIR}/mcc_generated_files/winc/driver/source/m2m_crypto.o 
	 ${MP_CC}  $(MP_EXTRA_CC_PRE) -mmcu=atmega4808  -I "C:\Program Files (x86)\Microchip\MPLABX\v5.15\packs\Microchip\ATmega_DFP\2.0.2\include"  -B "C:\Program Files (x86)\Microchip\MPLABX\v5.15\packs\Microchip\ATmega_DFP\2.0.2\gcc\dev\atmega4808" -g -DDEBUG  -gdwarf-2  -x c -c -D__$(MP_PROCESSOR_OPTION)__  -funsigned-char -funsigned-bitfields -Os -ffunction-sections -fdata-sections -fpack-struct -fshort-enums -Wall -MD -MP -MF "${OBJECTDIR}/mcc_generated_files/winc/driver/source/m2m_crypto.o.d" -MT "${OBJECTDIR}/mcc_generated_files/winc/driver/source/m2m_crypto.o.d" -MT ${OBJECTDIR}/mcc_generated_files/winc/driver/source/m2m_crypto.o  -o ${OBJECTDIR}/mcc_generated_files/winc/driver/source/m2m_crypto.o mcc_generated_files/winc/driver/source/m2m_crypto.c  -DXPRJ_default=$(CND_CONF)  $(COMPARISON_BUILD) 
	
${OBJECTDIR}/mcc_generated_files/winc/driver/source/nmi2c.o: mcc_generated_files/winc/driver/source/nmi2c.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/mcc_generated_files/winc/driver/source" 
	@${RM} ${OBJECTDIR}/mcc_generated_files/winc/driver/source/nmi2c.o.d 
	@${RM} ${OBJECTDIR}/mcc_generated_files/winc/driver/source/nmi2c.o 
	 ${MP_CC}  $(MP_EXTRA_CC_PRE) -mmcu=atmega4808  -I "C:\Program Files (x86)\Microchip\MPLABX\v5.15\packs\Microchip\ATmega_DFP\2.0.2\include"  -B "C:\Program Files (x86)\Microchip\MPLABX\v5.15\packs\Microchip\ATmega_DFP\2.0.2\gcc\dev\atmega4808" -g -DDEBUG  -gdwarf-2  -x c -c -D__$(MP_PROCESSOR_OPTION)__  -funsigned-char -funsigned-bitfields -Os -ffunction-sections -fdata-sections -fpack-struct -fshort-enums -Wall -MD -MP -MF "${OBJECTDIR}/mcc_generated_files/winc/driver/source/nmi2c.o.d" -MT "${OBJECTDIR}/mcc_generated_files/winc/driver/source/nmi2c.o.d" -MT ${OBJECTDIR}/mcc_generated_files/winc/driver/source/nmi2c.o  -o ${OBJECTDIR}/mcc_generated_files/winc/driver/source/nmi2c.o mcc_generated_files/winc/driver/source/nmi2c.c  -DXPRJ_default=$(CND_CONF)  $(COMPARISON_BUILD) 
	
${OBJECTDIR}/mcc_generated_files/winc/driver/source/nmuart.o: mcc_generated_files/winc/driver/source/nmuart.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/mcc_generated_files/winc/driver/source" 
	@${RM} ${OBJECTDIR}/mcc_generated_files/winc/driver/source/nmuart.o.d 
	@${RM} ${OBJECTDIR}/mcc_generated_files/winc/driver/source/nmuart.o 
	 ${MP_CC}  $(MP_EXTRA_CC_PRE) -mmcu=atmega4808  -I "C:\Program Files (x86)\Microchip\MPLABX\v5.15\packs\Microchip\ATmega_DFP\2.0.2\include"  -B "C:\Program Files (x86)\Microchip\MPLABX\v5.15\packs\Microchip\ATmega_DFP\2.0.2\gcc\dev\atmega4808" -g -DDEBUG  -gdwarf-2  -x c -c -D__$(MP_PROCESSOR_OPTION)__  -funsigned-char -funsigned-bitfields -Os -ffunction-sections -fdata-sections -fpack-struct -fshort-enums -Wall -MD -MP -MF "${OBJECTDIR}/mcc_generated_files/winc/driver/source/nmuart.o.d" -MT "${OBJECTDIR}/mcc_generated_files/winc/driver/source/nmuart.o.d" -MT ${OBJECTDIR}/mcc_generated_files/winc/driver/source/nmuart.o  -o ${OBJECTDIR}/mcc_generated_files/winc/driver/source/nmuart.o mcc_generated_files/winc/driver/source/nmuart.c  -DXPRJ_default=$(CND_CONF)  $(COMPARISON_BUILD) 
	
${OBJECTDIR}/mcc_generated_files/winc/driver/source/nmbus.o: mcc_generated_files/winc/driver/source/nmbus.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/mcc_generated_files/winc/driver/source" 
	@${RM} ${OBJECTDIR}/mcc_generated_files/winc/driver/source/nmbus.o.d 
	@${RM} ${OBJECTDIR}/mcc_generated_files/winc/driver/source/nmbus.o 
	 ${MP_CC}  $(MP_EXTRA_CC_PRE) -mmcu=atmega4808  -I "C:\Program Files (x86)\Microchip\MPLABX\v5.15\packs\Microchip\ATmega_DFP\2.0.2\include"  -B "C:\Program Files (x86)\Microchip\MPLABX\v5.15\packs\Microchip\ATmega_DFP\2.0.2\gcc\dev\atmega4808" -g -DDEBUG  -gdwarf-2  -x c -c -D__$(MP_PROCESSOR_OPTION)__  -funsigned-char -funsigned-bitfields -Os -ffunction-sections -fdata-sections -fpack-struct -fshort-enums -Wall -MD -MP -MF "${OBJECTDIR}/mcc_generated_files/winc/driver/source/nmbus.o.d" -MT "${OBJECTDIR}/mcc_generated_files/winc/driver/source/nmbus.o.d" -MT ${OBJECTDIR}/mcc_generated_files/winc/driver/source/nmbus.o  -o ${OBJECTDIR}/mcc_generated_files/winc/driver/source/nmbus.o mcc_generated_files/winc/driver/source/nmbus.c  -DXPRJ_default=$(CND_CONF)  $(COMPARISON_BUILD) 
	
${OBJECTDIR}/mcc_generated_files/winc/socket/source/socket.o: mcc_generated_files/winc/socket/source/socket.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/mcc_generated_files/winc/socket/source" 
	@${RM} ${OBJECTDIR}/mcc_generated_files/winc/socket/source/socket.o.d 
	@${RM} ${OBJECTDIR}/mcc_generated_files/winc/socket/source/socket.o 
	 ${MP_CC}  $(MP_EXTRA_CC_PRE) -mmcu=atmega4808  -I "C:\Program Files (x86)\Microchip\MPLABX\v5.15\packs\Microchip\ATmega_DFP\2.0.2\include"  -B "C:\Program Files (x86)\Microchip\MPLABX\v5.15\packs\Microchip\ATmega_DFP\2.0.2\gcc\dev\atmega4808" -g -DDEBUG  -gdwarf-2  -x c -c -D__$(MP_PROCESSOR_OPTION)__  -funsigned-char -funsigned-bitfields -Os -ffunction-sections -fdata-sections -fpack-struct -fshort-enums -Wall -MD -MP -MF "${OBJECTDIR}/mcc_generated_files/winc/socket/source/socket.o.d" -MT "${OBJECTDIR}/mcc_generated_files/winc/socket/source/socket.o.d" -MT ${OBJECTDIR}/mcc_generated_files/winc/socket/source/socket.o  -o ${OBJECTDIR}/mcc_generated_files/winc/socket/source/socket.o mcc_generated_files/winc/socket/source/socket.c  -DXPRJ_default=$(CND_CONF)  $(COMPARISON_BUILD) 
	
${OBJECTDIR}/mcc_generated_files/winc/spi_flash/source/spi_flash.o: mcc_generated_files/winc/spi_flash/source/spi_flash.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/mcc_generated_files/winc/spi_flash/source" 
	@${RM} ${OBJECTDIR}/mcc_generated_files/winc/spi_flash/source/spi_flash.o.d 
	@${RM} ${OBJECTDIR}/mcc_generated_files/winc/spi_flash/source/spi_flash.o 
	 ${MP_CC}  $(MP_EXTRA_CC_PRE) -mmcu=atmega4808  -I "C:\Program Files (x86)\Microchip\MPLABX\v5.15\packs\Microchip\ATmega_DFP\2.0.2\include"  -B "C:\Program Files (x86)\Microchip\MPLABX\v5.15\packs\Microchip\ATmega_DFP\2.0.2\gcc\dev\atmega4808" -g -DDEBUG  -gdwarf-2  -x c -c -D__$(MP_PROCESSOR_OPTION)__  -funsigned-char -funsigned-bitfields -Os -ffunction-sections -fdata-sections -fpack-struct -fshort-enums -Wall -MD -MP -MF "${OBJECTDIR}/mcc_generated_files/winc/spi_flash/source/spi_flash.o.d" -MT "${OBJECTDIR}/mcc_generated_files/winc/spi_flash/source/spi_flash.o.d" -MT ${OBJECTDIR}/mcc_generated_files/winc/spi_flash/source/spi_flash.o  -o ${OBJECTDIR}/mcc_generated_files/winc/spi_flash/source/spi_flash.o mcc_generated_files/winc/spi_flash/source/spi_flash.c  -DXPRJ_default=$(CND_CONF)  $(COMPARISON_BUILD) 
	
${OBJECTDIR}/mcc_generated_files/mcc.o: mcc_generated_files/mcc.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/mcc_generated_files" 
	@${RM} ${OBJECTDIR}/mcc_generated_files/mcc.o.d 
	@${RM} ${OBJECTDIR}/mcc_generated_files/mcc.o 
	 ${MP_CC}  $(MP_EXTRA_CC_PRE) -mmcu=atmega4808  -I "C:\Program Files (x86)\Microchip\MPLABX\v5.15\packs\Microchip\ATmega_DFP\2.0.2\include"  -B "C:\Program Files (x86)\Microchip\MPLABX\v5.15\packs\Microchip\ATmega_DFP\2.0.2\gcc\dev\atmega4808" -g -DDEBUG  -gdwarf-2  -x c -c -D__$(MP_PROCESSOR_OPTION)__  -funsigned-char -funsigned-bitfields -Os -ffunction-sections -fdata-sections -fpack-struct -fshort-enums -Wall -MD -MP -MF "${OBJECTDIR}/mcc_generated_files/mcc.o.d" -MT "${OBJECTDIR}/mcc_generated_files/mcc.o.d" -MT ${OBJECTDIR}/mcc_generated_files/mcc.o  -o ${OBJECTDIR}/mcc_generated_files/mcc.o mcc_generated_files/mcc.c  -DXPRJ_default=$(CND_CONF)  $(COMPARISON_BUILD) 
	
${OBJECTDIR}/mcc_generated_files/application_manager.o: mcc_generated_files/application_manager.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/mcc_generated_files" 
	@${RM} ${OBJECTDIR}/mcc_generated_files/application_manager.o.d 
	@${RM} ${OBJECTDIR}/mcc_generated_files/application_manager.o 
	 ${MP_CC}  $(MP_EXTRA_CC_PRE) -mmcu=atmega4808  -I "C:\Program Files (x86)\Microchip\MPLABX\v5.15\packs\Microchip\ATmega_DFP\2.0.2\include"  -B "C:\Program Files (x86)\Microchip\MPLABX\v5.15\packs\Microchip\ATmega_DFP\2.0.2\gcc\dev\atmega4808" -g -DDEBUG  -gdwarf-2  -x c -c -D__$(MP_PROCESSOR_OPTION)__  -funsigned-char -funsigned-bitfields -Os -ffunction-sections -fdata-sections -fpack-struct -fshort-enums -Wall -MD -MP -MF "${OBJECTDIR}/mcc_generated_files/application_manager.o.d" -MT "${OBJECTDIR}/mcc_generated_files/application_manager.o.d" -MT ${OBJECTDIR}/mcc_generated_files/application_manager.o  -o ${OBJECTDIR}/mcc_generated_files/application_manager.o mcc_generated_files/application_manager.c  -DXPRJ_default=$(CND_CONF)  $(COMPARISON_BUILD) 
	
${OBJECTDIR}/mcc_generated_files/driver_isr.o: mcc_generated_files/driver_isr.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/mcc_generated_files" 
	@${RM} ${OBJECTDIR}/mcc_generated_files/driver_isr.o.d 
	@${RM} ${OBJECTDIR}/mcc_generated_files/driver_isr.o 
	 ${MP_CC}  $(MP_EXTRA_CC_PRE) -mmcu=atmega4808  -I "C:\Program Files (x86)\Microchip\MPLABX\v5.15\packs\Microchip\ATmega_DFP\2.0.2\include"  -B "C:\Program Files (x86)\Microchip\MPLABX\v5.15\packs\Microchip\ATmega_DFP\2.0.2\gcc\dev\atmega4808" -g -DDEBUG  -gdwarf-2  -x c -c -D__$(MP_PROCESSOR_OPTION)__  -funsigned-char -funsigned-bitfields -Os -ffunction-sections -fdata-sections -fpack-struct -fshort-enums -Wall -MD -MP -MF "${OBJECTDIR}/mcc_generated_files/driver_isr.o.d" -MT "${OBJECTDIR}/mcc_generated_files/driver_isr.o.d" -MT ${OBJECTDIR}/mcc_generated_files/driver_isr.o  -o ${OBJECTDIR}/mcc_generated_files/driver_isr.o mcc_generated_files/driver_isr.c  -DXPRJ_default=$(CND_CONF)  $(COMPARISON_BUILD) 
	
${OBJECTDIR}/mcc_generated_files/sensors_handling.o: mcc_generated_files/sensors_handling.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/mcc_generated_files" 
	@${RM} ${OBJECTDIR}/mcc_generated_files/sensors_handling.o.d 
	@${RM} ${OBJECTDIR}/mcc_generated_files/sensors_handling.o 
	 ${MP_CC}  $(MP_EXTRA_CC_PRE) -mmcu=atmega4808  -I "C:\Program Files (x86)\Microchip\MPLABX\v5.15\packs\Microchip\ATmega_DFP\2.0.2\include"  -B "C:\Program Files (x86)\Microchip\MPLABX\v5.15\packs\Microchip\ATmega_DFP\2.0.2\gcc\dev\atmega4808" -g -DDEBUG  -gdwarf-2  -x c -c -D__$(MP_PROCESSOR_OPTION)__  -funsigned-char -funsigned-bitfields -Os -ffunction-sections -fdata-sections -fpack-struct -fshort-enums -Wall -MD -MP -MF "${OBJECTDIR}/mcc_generated_files/sensors_handling.o.d" -MT "${OBJECTDIR}/mcc_generated_files/sensors_handling.o.d" -MT ${OBJECTDIR}/mcc_generated_files/sensors_handling.o  -o ${OBJECTDIR}/mcc_generated_files/sensors_handling.o mcc_generated_files/sensors_handling.c  -DXPRJ_default=$(CND_CONF)  $(COMPARISON_BUILD) 
	
${OBJECTDIR}/mcc_generated_files/device_config.o: mcc_generated_files/device_config.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/mcc_generated_files" 
	@${RM} ${OBJECTDIR}/mcc_generated_files/device_config.o.d 
	@${RM} ${OBJECTDIR}/mcc_generated_files/device_config.o 
	 ${MP_CC}  $(MP_EXTRA_CC_PRE) -mmcu=atmega4808  -I "C:\Program Files (x86)\Microchip\MPLABX\v5.15\packs\Microchip\ATmega_DFP\2.0.2\include"  -B "C:\Program Files (x86)\Microchip\MPLABX\v5.15\packs\Microchip\ATmega_DFP\2.0.2\gcc\dev\atmega4808" -g -DDEBUG  -gdwarf-2  -x c -c -D__$(MP_PROCESSOR_OPTION)__  -funsigned-char -funsigned-bitfields -Os -ffunction-sections -fdata-sections -fpack-struct -fshort-enums -Wall -MD -MP -MF "${OBJECTDIR}/mcc_generated_files/device_config.o.d" -MT "${OBJECTDIR}/mcc_generated_files/device_config.o.d" -MT ${OBJECTDIR}/mcc_generated_files/device_config.o  -o ${OBJECTDIR}/mcc_generated_files/device_config.o mcc_generated_files/device_config.c  -DXPRJ_default=$(CND_CONF)  $(COMPARISON_BUILD) 
	
${OBJECTDIR}/mcc_generated_files/led.o: mcc_generated_files/led.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/mcc_generated_files" 
	@${RM} ${OBJECTDIR}/mcc_generated_files/led.o.d 
	@${RM} ${OBJECTDIR}/mcc_generated_files/led.o 
	 ${MP_CC}  $(MP_EXTRA_CC_PRE) -mmcu=atmega4808  -I "C:\Program Files (x86)\Microchip\MPLABX\v5.15\packs\Microchip\ATmega_DFP\2.0.2\include"  -B "C:\Program Files (x86)\Microchip\MPLABX\v5.15\packs\Microchip\ATmega_DFP\2.0.2\gcc\dev\atmega4808" -g -DDEBUG  -gdwarf-2  -x c -c -D__$(MP_PROCESSOR_OPTION)__  -funsigned-char -funsigned-bitfields -Os -ffunction-sections -fdata-sections -fpack-struct -fshort-enums -Wall -MD -MP -MF "${OBJECTDIR}/mcc_generated_files/led.o.d" -MT "${OBJECTDIR}/mcc_generated_files/led.o.d" -MT ${OBJECTDIR}/mcc_generated_files/led.o  -o ${OBJECTDIR}/mcc_generated_files/led.o mcc_generated_files/led.c  -DXPRJ_default=$(CND_CONF)  $(COMPARISON_BUILD) 
	
${OBJECTDIR}/mcc_generated_files/debug_print.o: mcc_generated_files/debug_print.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/mcc_generated_files" 
	@${RM} ${OBJECTDIR}/mcc_generated_files/debug_print.o.d 
	@${RM} ${OBJECTDIR}/mcc_generated_files/debug_print.o 
	 ${MP_CC}  $(MP_EXTRA_CC_PRE) -mmcu=atmega4808  -I "C:\Program Files (x86)\Microchip\MPLABX\v5.15\packs\Microchip\ATmega_DFP\2.0.2\include"  -B "C:\Program Files (x86)\Microchip\MPLABX\v5.15\packs\Microchip\ATmega_DFP\2.0.2\gcc\dev\atmega4808" -g -DDEBUG  -gdwarf-2  -x c -c -D__$(MP_PROCESSOR_OPTION)__  -funsigned-char -funsigned-bitfields -Os -ffunction-sections -fdata-sections -fpack-struct -fshort-enums -Wall -MD -MP -MF "${OBJECTDIR}/mcc_generated_files/debug_print.o.d" -MT "${OBJECTDIR}/mcc_generated_files/debug_print.o.d" -MT ${OBJECTDIR}/mcc_generated_files/debug_print.o  -o ${OBJECTDIR}/mcc_generated_files/debug_print.o mcc_generated_files/debug_print.c  -DXPRJ_default=$(CND_CONF)  $(COMPARISON_BUILD) 
	
${OBJECTDIR}/main.o: main.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}" 
	@${RM} ${OBJECTDIR}/main.o.d 
	@${RM} ${OBJECTDIR}/main.o 
	 ${MP_CC}  $(MP_EXTRA_CC_PRE) -mmcu=atmega4808  -I "C:\Program Files (x86)\Microchip\MPLABX\v5.15\packs\Microchip\ATmega_DFP\2.0.2\include"  -B "C:\Program Files (x86)\Microchip\MPLABX\v5.15\packs\Microchip\ATmega_DFP\2.0.2\gcc\dev\atmega4808" -g -DDEBUG  -gdwarf-2  -x c -c -D__$(MP_PROCESSOR_OPTION)__  -funsigned-char -funsigned-bitfields -Os -ffunction-sections -fdata-sections -fpack-struct -fshort-enums -Wall -MD -MP -MF "${OBJECTDIR}/main.o.d" -MT "${OBJECTDIR}/main.o.d" -MT ${OBJECTDIR}/main.o  -o ${OBJECTDIR}/main.o main.c  -DXPRJ_default=$(CND_CONF)  $(COMPARISON_BUILD) 
	
else
${OBJECTDIR}/mcc_generated_files/cli/cli.o: mcc_generated_files/cli/cli.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/mcc_generated_files/cli" 
	@${RM} ${OBJECTDIR}/mcc_generated_files/cli/cli.o.d 
	@${RM} ${OBJECTDIR}/mcc_generated_files/cli/cli.o 
	 ${MP_CC}  $(MP_EXTRA_CC_PRE) -mmcu=atmega4808  -I "C:\Program Files (x86)\Microchip\MPLABX\v5.15\packs\Microchip\ATmega_DFP\2.0.2\include"  -B "C:\Program Files (x86)\Microchip\MPLABX\v5.15\packs\Microchip\ATmega_DFP\2.0.2\gcc\dev\atmega4808"  -x c -c -D__$(MP_PROCESSOR_OPTION)__  -funsigned-char -funsigned-bitfields -Os -ffunction-sections -fdata-sections -fpack-struct -fshort-enums -Wall -MD -MP -MF "${OBJECTDIR}/mcc_generated_files/cli/cli.o.d" -MT "${OBJECTDIR}/mcc_generated_files/cli/cli.o.d" -MT ${OBJECTDIR}/mcc_generated_files/cli/cli.o  -o ${OBJECTDIR}/mcc_generated_files/cli/cli.o mcc_generated_files/cli/cli.c  -DXPRJ_default=$(CND_CONF)  $(COMPARISON_BUILD) 
	
${OBJECTDIR}/mcc_generated_files/cloud/bsd_adapter/bsdWINC.o: mcc_generated_files/cloud/bsd_adapter/bsdWINC.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/mcc_generated_files/cloud/bsd_adapter" 
	@${RM} ${OBJECTDIR}/mcc_generated_files/cloud/bsd_adapter/bsdWINC.o.d 
	@${RM} ${OBJECTDIR}/mcc_generated_files/cloud/bsd_adapter/bsdWINC.o 
	 ${MP_CC}  $(MP_EXTRA_CC_PRE) -mmcu=atmega4808  -I "C:\Program Files (x86)\Microchip\MPLABX\v5.15\packs\Microchip\ATmega_DFP\2.0.2\include"  -B "C:\Program Files (x86)\Microchip\MPLABX\v5.15\packs\Microchip\ATmega_DFP\2.0.2\gcc\dev\atmega4808"  -x c -c -D__$(MP_PROCESSOR_OPTION)__  -funsigned-char -funsigned-bitfields -Os -ffunction-sections -fdata-sections -fpack-struct -fshort-enums -Wall -MD -MP -MF "${OBJECTDIR}/mcc_generated_files/cloud/bsd_adapter/bsdWINC.o.d" -MT "${OBJECTDIR}/mcc_generated_files/cloud/bsd_adapter/bsdWINC.o.d" -MT ${OBJECTDIR}/mcc_generated_files/cloud/bsd_adapter/bsdWINC.o  -o ${OBJECTDIR}/mcc_generated_files/cloud/bsd_adapter/bsdWINC.o mcc_generated_files/cloud/bsd_adapter/bsdWINC.c  -DXPRJ_default=$(CND_CONF)  $(COMPARISON_BUILD) 
	
${OBJECTDIR}/mcc_generated_files/cloud/crypto_client/crypto_client.o: mcc_generated_files/cloud/crypto_client/crypto_client.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/mcc_generated_files/cloud/crypto_client" 
	@${RM} ${OBJECTDIR}/mcc_generated_files/cloud/crypto_client/crypto_client.o.d 
	@${RM} ${OBJECTDIR}/mcc_generated_files/cloud/crypto_client/crypto_client.o 
	 ${MP_CC}  $(MP_EXTRA_CC_PRE) -mmcu=atmega4808  -I "C:\Program Files (x86)\Microchip\MPLABX\v5.15\packs\Microchip\ATmega_DFP\2.0.2\include"  -B "C:\Program Files (x86)\Microchip\MPLABX\v5.15\packs\Microchip\ATmega_DFP\2.0.2\gcc\dev\atmega4808"  -x c -c -D__$(MP_PROCESSOR_OPTION)__  -funsigned-char -funsigned-bitfields -Os -ffunction-sections -fdata-sections -fpack-struct -fshort-enums -Wall -MD -MP -MF "${OBJECTDIR}/mcc_generated_files/cloud/crypto_client/crypto_client.o.d" -MT "${OBJECTDIR}/mcc_generated_files/cloud/crypto_client/crypto_client.o.d" -MT ${OBJECTDIR}/mcc_generated_files/cloud/crypto_client/crypto_client.o  -o ${OBJECTDIR}/mcc_generated_files/cloud/crypto_client/crypto_client.o mcc_generated_files/cloud/crypto_client/crypto_client.c  -DXPRJ_default=$(CND_CONF)  $(COMPARISON_BUILD) 
	
${OBJECTDIR}/mcc_generated_files/cloud/crypto_client/cryptoauthlib_main.o: mcc_generated_files/cloud/crypto_client/cryptoauthlib_main.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/mcc_generated_files/cloud/crypto_client" 
	@${RM} ${OBJECTDIR}/mcc_generated_files/cloud/crypto_client/cryptoauthlib_main.o.d 
	@${RM} ${OBJECTDIR}/mcc_generated_files/cloud/crypto_client/cryptoauthlib_main.o 
	 ${MP_CC}  $(MP_EXTRA_CC_PRE) -mmcu=atmega4808  -I "C:\Program Files (x86)\Microchip\MPLABX\v5.15\packs\Microchip\ATmega_DFP\2.0.2\include"  -B "C:\Program Files (x86)\Microchip\MPLABX\v5.15\packs\Microchip\ATmega_DFP\2.0.2\gcc\dev\atmega4808"  -x c -c -D__$(MP_PROCESSOR_OPTION)__  -funsigned-char -funsigned-bitfields -Os -ffunction-sections -fdata-sections -fpack-struct -fshort-enums -Wall -MD -MP -MF "${OBJECTDIR}/mcc_generated_files/cloud/crypto_client/cryptoauthlib_main.o.d" -MT "${OBJECTDIR}/mcc_generated_files/cloud/crypto_client/cryptoauthlib_main.o.d" -MT ${OBJECTDIR}/mcc_generated_files/cloud/crypto_client/cryptoauthlib_main.o  -o ${OBJECTDIR}/mcc_generated_files/cloud/crypto_client/cryptoauthlib_main.o mcc_generated_files/cloud/crypto_client/cryptoauthlib_main.c  -DXPRJ_default=$(CND_CONF)  $(COMPARISON_BUILD) 
	
${OBJECTDIR}/mcc_generated_files/cloud/mqtt_packetPopulation/mqtt_packetPopulate.o: mcc_generated_files/cloud/mqtt_packetPopulation/mqtt_packetPopulate.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/mcc_generated_files/cloud/mqtt_packetPopulation" 
	@${RM} ${OBJECTDIR}/mcc_generated_files/cloud/mqtt_packetPopulation/mqtt_packetPopulate.o.d 
	@${RM} ${OBJECTDIR}/mcc_generated_files/cloud/mqtt_packetPopulation/mqtt_packetPopulate.o 
	 ${MP_CC}  $(MP_EXTRA_CC_PRE) -mmcu=atmega4808  -I "C:\Program Files (x86)\Microchip\MPLABX\v5.15\packs\Microchip\ATmega_DFP\2.0.2\include"  -B "C:\Program Files (x86)\Microchip\MPLABX\v5.15\packs\Microchip\ATmega_DFP\2.0.2\gcc\dev\atmega4808"  -x c -c -D__$(MP_PROCESSOR_OPTION)__  -funsigned-char -funsigned-bitfields -Os -ffunction-sections -fdata-sections -fpack-struct -fshort-enums -Wall -MD -MP -MF "${OBJECTDIR}/mcc_generated_files/cloud/mqtt_packetPopulation/mqtt_packetPopulate.o.d" -MT "${OBJECTDIR}/mcc_generated_files/cloud/mqtt_packetPopulation/mqtt_packetPopulate.o.d" -MT ${OBJECTDIR}/mcc_generated_files/cloud/mqtt_packetPopulation/mqtt_packetPopulate.o  -o ${OBJECTDIR}/mcc_generated_files/cloud/mqtt_packetPopulation/mqtt_packetPopulate.o mcc_generated_files/cloud/mqtt_packetPopulation/mqtt_packetPopulate.c  -DXPRJ_default=$(CND_CONF)  $(COMPARISON_BUILD) 
	
${OBJECTDIR}/mcc_generated_files/cloud/wifi_service.o: mcc_generated_files/cloud/wifi_service.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/mcc_generated_files/cloud" 
	@${RM} ${OBJECTDIR}/mcc_generated_files/cloud/wifi_service.o.d 
	@${RM} ${OBJECTDIR}/mcc_generated_files/cloud/wifi_service.o 
	 ${MP_CC}  $(MP_EXTRA_CC_PRE) -mmcu=atmega4808  -I "C:\Program Files (x86)\Microchip\MPLABX\v5.15\packs\Microchip\ATmega_DFP\2.0.2\include"  -B "C:\Program Files (x86)\Microchip\MPLABX\v5.15\packs\Microchip\ATmega_DFP\2.0.2\gcc\dev\atmega4808"  -x c -c -D__$(MP_PROCESSOR_OPTION)__  -funsigned-char -funsigned-bitfields -Os -ffunction-sections -fdata-sections -fpack-struct -fshort-enums -Wall -MD -MP -MF "${OBJECTDIR}/mcc_generated_files/cloud/wifi_service.o.d" -MT "${OBJECTDIR}/mcc_generated_files/cloud/wifi_service.o.d" -MT ${OBJECTDIR}/mcc_generated_files/cloud/wifi_service.o  -o ${OBJECTDIR}/mcc_generated_files/cloud/wifi_service.o mcc_generated_files/cloud/wifi_service.c  -DXPRJ_default=$(CND_CONF)  $(COMPARISON_BUILD) 
	
${OBJECTDIR}/mcc_generated_files/cloud/cloud_service.o: mcc_generated_files/cloud/cloud_service.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/mcc_generated_files/cloud" 
	@${RM} ${OBJECTDIR}/mcc_generated_files/cloud/cloud_service.o.d 
	@${RM} ${OBJECTDIR}/mcc_generated_files/cloud/cloud_service.o 
	 ${MP_CC}  $(MP_EXTRA_CC_PRE) -mmcu=atmega4808  -I "C:\Program Files (x86)\Microchip\MPLABX\v5.15\packs\Microchip\ATmega_DFP\2.0.2\include"  -B "C:\Program Files (x86)\Microchip\MPLABX\v5.15\packs\Microchip\ATmega_DFP\2.0.2\gcc\dev\atmega4808"  -x c -c -D__$(MP_PROCESSOR_OPTION)__  -funsigned-char -funsigned-bitfields -Os -ffunction-sections -fdata-sections -fpack-struct -fshort-enums -Wall -MD -MP -MF "${OBJECTDIR}/mcc_generated_files/cloud/cloud_service.o.d" -MT "${OBJECTDIR}/mcc_generated_files/cloud/cloud_service.o.d" -MT ${OBJECTDIR}/mcc_generated_files/cloud/cloud_service.o  -o ${OBJECTDIR}/mcc_generated_files/cloud/cloud_service.o mcc_generated_files/cloud/cloud_service.c  -DXPRJ_default=$(CND_CONF)  $(COMPARISON_BUILD) 
	
${OBJECTDIR}/mcc_generated_files/credentials_storage/credentials_storage.o: mcc_generated_files/credentials_storage/credentials_storage.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/mcc_generated_files/credentials_storage" 
	@${RM} ${OBJECTDIR}/mcc_generated_files/credentials_storage/credentials_storage.o.d 
	@${RM} ${OBJECTDIR}/mcc_generated_files/credentials_storage/credentials_storage.o 
	 ${MP_CC}  $(MP_EXTRA_CC_PRE) -mmcu=atmega4808  -I "C:\Program Files (x86)\Microchip\MPLABX\v5.15\packs\Microchip\ATmega_DFP\2.0.2\include"  -B "C:\Program Files (x86)\Microchip\MPLABX\v5.15\packs\Microchip\ATmega_DFP\2.0.2\gcc\dev\atmega4808"  -x c -c -D__$(MP_PROCESSOR_OPTION)__  -funsigned-char -funsigned-bitfields -Os -ffunction-sections -fdata-sections -fpack-struct -fshort-enums -Wall -MD -MP -MF "${OBJECTDIR}/mcc_generated_files/credentials_storage/credentials_storage.o.d" -MT "${OBJECTDIR}/mcc_generated_files/credentials_storage/credentials_storage.o.d" -MT ${OBJECTDIR}/mcc_generated_files/credentials_storage/credentials_storage.o  -o ${OBJECTDIR}/mcc_generated_files/credentials_storage/credentials_storage.o mcc_generated_files/credentials_storage/credentials_storage.c  -DXPRJ_default=$(CND_CONF)  $(COMPARISON_BUILD) 
	
${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/atcacert/atcacert_client.o: mcc_generated_files/cryptoauthlib/lib/atcacert/atcacert_client.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/atcacert" 
	@${RM} ${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/atcacert/atcacert_client.o.d 
	@${RM} ${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/atcacert/atcacert_client.o 
	 ${MP_CC}  $(MP_EXTRA_CC_PRE) -mmcu=atmega4808  -I "C:\Program Files (x86)\Microchip\MPLABX\v5.15\packs\Microchip\ATmega_DFP\2.0.2\include"  -B "C:\Program Files (x86)\Microchip\MPLABX\v5.15\packs\Microchip\ATmega_DFP\2.0.2\gcc\dev\atmega4808"  -x c -c -D__$(MP_PROCESSOR_OPTION)__  -funsigned-char -funsigned-bitfields -Os -ffunction-sections -fdata-sections -fpack-struct -fshort-enums -Wall -MD -MP -MF "${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/atcacert/atcacert_client.o.d" -MT "${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/atcacert/atcacert_client.o.d" -MT ${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/atcacert/atcacert_client.o  -o ${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/atcacert/atcacert_client.o mcc_generated_files/cryptoauthlib/lib/atcacert/atcacert_client.c  -DXPRJ_default=$(CND_CONF)  $(COMPARISON_BUILD) 
	
${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/atcacert/atcacert_host_sw.o: mcc_generated_files/cryptoauthlib/lib/atcacert/atcacert_host_sw.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/atcacert" 
	@${RM} ${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/atcacert/atcacert_host_sw.o.d 
	@${RM} ${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/atcacert/atcacert_host_sw.o 
	 ${MP_CC}  $(MP_EXTRA_CC_PRE) -mmcu=atmega4808  -I "C:\Program Files (x86)\Microchip\MPLABX\v5.15\packs\Microchip\ATmega_DFP\2.0.2\include"  -B "C:\Program Files (x86)\Microchip\MPLABX\v5.15\packs\Microchip\ATmega_DFP\2.0.2\gcc\dev\atmega4808"  -x c -c -D__$(MP_PROCESSOR_OPTION)__  -funsigned-char -funsigned-bitfields -Os -ffunction-sections -fdata-sections -fpack-struct -fshort-enums -Wall -MD -MP -MF "${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/atcacert/atcacert_host_sw.o.d" -MT "${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/atcacert/atcacert_host_sw.o.d" -MT ${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/atcacert/atcacert_host_sw.o  -o ${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/atcacert/atcacert_host_sw.o mcc_generated_files/cryptoauthlib/lib/atcacert/atcacert_host_sw.c  -DXPRJ_default=$(CND_CONF)  $(COMPARISON_BUILD) 
	
${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/atcacert/atcacert_def.o: mcc_generated_files/cryptoauthlib/lib/atcacert/atcacert_def.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/atcacert" 
	@${RM} ${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/atcacert/atcacert_def.o.d 
	@${RM} ${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/atcacert/atcacert_def.o 
	 ${MP_CC}  $(MP_EXTRA_CC_PRE) -mmcu=atmega4808  -I "C:\Program Files (x86)\Microchip\MPLABX\v5.15\packs\Microchip\ATmega_DFP\2.0.2\include"  -B "C:\Program Files (x86)\Microchip\MPLABX\v5.15\packs\Microchip\ATmega_DFP\2.0.2\gcc\dev\atmega4808"  -x c -c -D__$(MP_PROCESSOR_OPTION)__  -funsigned-char -funsigned-bitfields -Os -ffunction-sections -fdata-sections -fpack-struct -fshort-enums -Wall -MD -MP -MF "${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/atcacert/atcacert_def.o.d" -MT "${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/atcacert/atcacert_def.o.d" -MT ${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/atcacert/atcacert_def.o  -o ${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/atcacert/atcacert_def.o mcc_generated_files/cryptoauthlib/lib/atcacert/atcacert_def.c  -DXPRJ_default=$(CND_CONF)  $(COMPARISON_BUILD) 
	
${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/atcacert/atcacert_date.o: mcc_generated_files/cryptoauthlib/lib/atcacert/atcacert_date.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/atcacert" 
	@${RM} ${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/atcacert/atcacert_date.o.d 
	@${RM} ${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/atcacert/atcacert_date.o 
	 ${MP_CC}  $(MP_EXTRA_CC_PRE) -mmcu=atmega4808  -I "C:\Program Files (x86)\Microchip\MPLABX\v5.15\packs\Microchip\ATmega_DFP\2.0.2\include"  -B "C:\Program Files (x86)\Microchip\MPLABX\v5.15\packs\Microchip\ATmega_DFP\2.0.2\gcc\dev\atmega4808"  -x c -c -D__$(MP_PROCESSOR_OPTION)__  -funsigned-char -funsigned-bitfields -Os -ffunction-sections -fdata-sections -fpack-struct -fshort-enums -Wall -MD -MP -MF "${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/atcacert/atcacert_date.o.d" -MT "${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/atcacert/atcacert_date.o.d" -MT ${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/atcacert/atcacert_date.o  -o ${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/atcacert/atcacert_date.o mcc_generated_files/cryptoauthlib/lib/atcacert/atcacert_date.c  -DXPRJ_default=$(CND_CONF)  $(COMPARISON_BUILD) 
	
${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/atcacert/atcacert_host_hw.o: mcc_generated_files/cryptoauthlib/lib/atcacert/atcacert_host_hw.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/atcacert" 
	@${RM} ${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/atcacert/atcacert_host_hw.o.d 
	@${RM} ${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/atcacert/atcacert_host_hw.o 
	 ${MP_CC}  $(MP_EXTRA_CC_PRE) -mmcu=atmega4808  -I "C:\Program Files (x86)\Microchip\MPLABX\v5.15\packs\Microchip\ATmega_DFP\2.0.2\include"  -B "C:\Program Files (x86)\Microchip\MPLABX\v5.15\packs\Microchip\ATmega_DFP\2.0.2\gcc\dev\atmega4808"  -x c -c -D__$(MP_PROCESSOR_OPTION)__  -funsigned-char -funsigned-bitfields -Os -ffunction-sections -fdata-sections -fpack-struct -fshort-enums -Wall -MD -MP -MF "${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/atcacert/atcacert_host_hw.o.d" -MT "${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/atcacert/atcacert_host_hw.o.d" -MT ${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/atcacert/atcacert_host_hw.o  -o ${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/atcacert/atcacert_host_hw.o mcc_generated_files/cryptoauthlib/lib/atcacert/atcacert_host_hw.c  -DXPRJ_default=$(CND_CONF)  $(COMPARISON_BUILD) 
	
${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/atcacert/atcacert_der.o: mcc_generated_files/cryptoauthlib/lib/atcacert/atcacert_der.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/atcacert" 
	@${RM} ${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/atcacert/atcacert_der.o.d 
	@${RM} ${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/atcacert/atcacert_der.o 
	 ${MP_CC}  $(MP_EXTRA_CC_PRE) -mmcu=atmega4808  -I "C:\Program Files (x86)\Microchip\MPLABX\v5.15\packs\Microchip\ATmega_DFP\2.0.2\include"  -B "C:\Program Files (x86)\Microchip\MPLABX\v5.15\packs\Microchip\ATmega_DFP\2.0.2\gcc\dev\atmega4808"  -x c -c -D__$(MP_PROCESSOR_OPTION)__  -funsigned-char -funsigned-bitfields -Os -ffunction-sections -fdata-sections -fpack-struct -fshort-enums -Wall -MD -MP -MF "${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/atcacert/atcacert_der.o.d" -MT "${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/atcacert/atcacert_der.o.d" -MT ${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/atcacert/atcacert_der.o  -o ${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/atcacert/atcacert_der.o mcc_generated_files/cryptoauthlib/lib/atcacert/atcacert_der.c  -DXPRJ_default=$(CND_CONF)  $(COMPARISON_BUILD) 
	
${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/atcacert/atcacert_pem.o: mcc_generated_files/cryptoauthlib/lib/atcacert/atcacert_pem.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/atcacert" 
	@${RM} ${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/atcacert/atcacert_pem.o.d 
	@${RM} ${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/atcacert/atcacert_pem.o 
	 ${MP_CC}  $(MP_EXTRA_CC_PRE) -mmcu=atmega4808  -I "C:\Program Files (x86)\Microchip\MPLABX\v5.15\packs\Microchip\ATmega_DFP\2.0.2\include"  -B "C:\Program Files (x86)\Microchip\MPLABX\v5.15\packs\Microchip\ATmega_DFP\2.0.2\gcc\dev\atmega4808"  -x c -c -D__$(MP_PROCESSOR_OPTION)__  -funsigned-char -funsigned-bitfields -Os -ffunction-sections -fdata-sections -fpack-struct -fshort-enums -Wall -MD -MP -MF "${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/atcacert/atcacert_pem.o.d" -MT "${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/atcacert/atcacert_pem.o.d" -MT ${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/atcacert/atcacert_pem.o  -o ${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/atcacert/atcacert_pem.o mcc_generated_files/cryptoauthlib/lib/atcacert/atcacert_pem.c  -DXPRJ_default=$(CND_CONF)  $(COMPARISON_BUILD) 
	
${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/basic/atca_basic_checkmac.o: mcc_generated_files/cryptoauthlib/lib/basic/atca_basic_checkmac.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/basic" 
	@${RM} ${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/basic/atca_basic_checkmac.o.d 
	@${RM} ${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/basic/atca_basic_checkmac.o 
	 ${MP_CC}  $(MP_EXTRA_CC_PRE) -mmcu=atmega4808  -I "C:\Program Files (x86)\Microchip\MPLABX\v5.15\packs\Microchip\ATmega_DFP\2.0.2\include"  -B "C:\Program Files (x86)\Microchip\MPLABX\v5.15\packs\Microchip\ATmega_DFP\2.0.2\gcc\dev\atmega4808"  -x c -c -D__$(MP_PROCESSOR_OPTION)__  -funsigned-char -funsigned-bitfields -Os -ffunction-sections -fdata-sections -fpack-struct -fshort-enums -Wall -MD -MP -MF "${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/basic/atca_basic_checkmac.o.d" -MT "${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/basic/atca_basic_checkmac.o.d" -MT ${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/basic/atca_basic_checkmac.o  -o ${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/basic/atca_basic_checkmac.o mcc_generated_files/cryptoauthlib/lib/basic/atca_basic_checkmac.c  -DXPRJ_default=$(CND_CONF)  $(COMPARISON_BUILD) 
	
${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/basic/atca_basic_secureboot.o: mcc_generated_files/cryptoauthlib/lib/basic/atca_basic_secureboot.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/basic" 
	@${RM} ${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/basic/atca_basic_secureboot.o.d 
	@${RM} ${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/basic/atca_basic_secureboot.o 
	 ${MP_CC}  $(MP_EXTRA_CC_PRE) -mmcu=atmega4808  -I "C:\Program Files (x86)\Microchip\MPLABX\v5.15\packs\Microchip\ATmega_DFP\2.0.2\include"  -B "C:\Program Files (x86)\Microchip\MPLABX\v5.15\packs\Microchip\ATmega_DFP\2.0.2\gcc\dev\atmega4808"  -x c -c -D__$(MP_PROCESSOR_OPTION)__  -funsigned-char -funsigned-bitfields -Os -ffunction-sections -fdata-sections -fpack-struct -fshort-enums -Wall -MD -MP -MF "${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/basic/atca_basic_secureboot.o.d" -MT "${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/basic/atca_basic_secureboot.o.d" -MT ${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/basic/atca_basic_secureboot.o  -o ${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/basic/atca_basic_secureboot.o mcc_generated_files/cryptoauthlib/lib/basic/atca_basic_secureboot.c  -DXPRJ_default=$(CND_CONF)  $(COMPARISON_BUILD) 
	
${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/basic/atca_basic_selftest.o: mcc_generated_files/cryptoauthlib/lib/basic/atca_basic_selftest.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/basic" 
	@${RM} ${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/basic/atca_basic_selftest.o.d 
	@${RM} ${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/basic/atca_basic_selftest.o 
	 ${MP_CC}  $(MP_EXTRA_CC_PRE) -mmcu=atmega4808  -I "C:\Program Files (x86)\Microchip\MPLABX\v5.15\packs\Microchip\ATmega_DFP\2.0.2\include"  -B "C:\Program Files (x86)\Microchip\MPLABX\v5.15\packs\Microchip\ATmega_DFP\2.0.2\gcc\dev\atmega4808"  -x c -c -D__$(MP_PROCESSOR_OPTION)__  -funsigned-char -funsigned-bitfields -Os -ffunction-sections -fdata-sections -fpack-struct -fshort-enums -Wall -MD -MP -MF "${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/basic/atca_basic_selftest.o.d" -MT "${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/basic/atca_basic_selftest.o.d" -MT ${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/basic/atca_basic_selftest.o  -o ${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/basic/atca_basic_selftest.o mcc_generated_files/cryptoauthlib/lib/basic/atca_basic_selftest.c  -DXPRJ_default=$(CND_CONF)  $(COMPARISON_BUILD) 
	
${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/basic/atca_basic_ecdh.o: mcc_generated_files/cryptoauthlib/lib/basic/atca_basic_ecdh.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/basic" 
	@${RM} ${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/basic/atca_basic_ecdh.o.d 
	@${RM} ${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/basic/atca_basic_ecdh.o 
	 ${MP_CC}  $(MP_EXTRA_CC_PRE) -mmcu=atmega4808  -I "C:\Program Files (x86)\Microchip\MPLABX\v5.15\packs\Microchip\ATmega_DFP\2.0.2\include"  -B "C:\Program Files (x86)\Microchip\MPLABX\v5.15\packs\Microchip\ATmega_DFP\2.0.2\gcc\dev\atmega4808"  -x c -c -D__$(MP_PROCESSOR_OPTION)__  -funsigned-char -funsigned-bitfields -Os -ffunction-sections -fdata-sections -fpack-struct -fshort-enums -Wall -MD -MP -MF "${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/basic/atca_basic_ecdh.o.d" -MT "${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/basic/atca_basic_ecdh.o.d" -MT ${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/basic/atca_basic_ecdh.o  -o ${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/basic/atca_basic_ecdh.o mcc_generated_files/cryptoauthlib/lib/basic/atca_basic_ecdh.c  -DXPRJ_default=$(CND_CONF)  $(COMPARISON_BUILD) 
	
${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/basic/atca_basic_updateextra.o: mcc_generated_files/cryptoauthlib/lib/basic/atca_basic_updateextra.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/basic" 
	@${RM} ${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/basic/atca_basic_updateextra.o.d 
	@${RM} ${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/basic/atca_basic_updateextra.o 
	 ${MP_CC}  $(MP_EXTRA_CC_PRE) -mmcu=atmega4808  -I "C:\Program Files (x86)\Microchip\MPLABX\v5.15\packs\Microchip\ATmega_DFP\2.0.2\include"  -B "C:\Program Files (x86)\Microchip\MPLABX\v5.15\packs\Microchip\ATmega_DFP\2.0.2\gcc\dev\atmega4808"  -x c -c -D__$(MP_PROCESSOR_OPTION)__  -funsigned-char -funsigned-bitfields -Os -ffunction-sections -fdata-sections -fpack-struct -fshort-enums -Wall -MD -MP -MF "${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/basic/atca_basic_updateextra.o.d" -MT "${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/basic/atca_basic_updateextra.o.d" -MT ${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/basic/atca_basic_updateextra.o  -o ${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/basic/atca_basic_updateextra.o mcc_generated_files/cryptoauthlib/lib/basic/atca_basic_updateextra.c  -DXPRJ_default=$(CND_CONF)  $(COMPARISON_BUILD) 
	
${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/basic/atca_basic_privwrite.o: mcc_generated_files/cryptoauthlib/lib/basic/atca_basic_privwrite.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/basic" 
	@${RM} ${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/basic/atca_basic_privwrite.o.d 
	@${RM} ${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/basic/atca_basic_privwrite.o 
	 ${MP_CC}  $(MP_EXTRA_CC_PRE) -mmcu=atmega4808  -I "C:\Program Files (x86)\Microchip\MPLABX\v5.15\packs\Microchip\ATmega_DFP\2.0.2\include"  -B "C:\Program Files (x86)\Microchip\MPLABX\v5.15\packs\Microchip\ATmega_DFP\2.0.2\gcc\dev\atmega4808"  -x c -c -D__$(MP_PROCESSOR_OPTION)__  -funsigned-char -funsigned-bitfields -Os -ffunction-sections -fdata-sections -fpack-struct -fshort-enums -Wall -MD -MP -MF "${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/basic/atca_basic_privwrite.o.d" -MT "${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/basic/atca_basic_privwrite.o.d" -MT ${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/basic/atca_basic_privwrite.o  -o ${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/basic/atca_basic_privwrite.o mcc_generated_files/cryptoauthlib/lib/basic/atca_basic_privwrite.c  -DXPRJ_default=$(CND_CONF)  $(COMPARISON_BUILD) 
	
${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/basic/atca_basic_info.o: mcc_generated_files/cryptoauthlib/lib/basic/atca_basic_info.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/basic" 
	@${RM} ${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/basic/atca_basic_info.o.d 
	@${RM} ${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/basic/atca_basic_info.o 
	 ${MP_CC}  $(MP_EXTRA_CC_PRE) -mmcu=atmega4808  -I "C:\Program Files (x86)\Microchip\MPLABX\v5.15\packs\Microchip\ATmega_DFP\2.0.2\include"  -B "C:\Program Files (x86)\Microchip\MPLABX\v5.15\packs\Microchip\ATmega_DFP\2.0.2\gcc\dev\atmega4808"  -x c -c -D__$(MP_PROCESSOR_OPTION)__  -funsigned-char -funsigned-bitfields -Os -ffunction-sections -fdata-sections -fpack-struct -fshort-enums -Wall -MD -MP -MF "${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/basic/atca_basic_info.o.d" -MT "${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/basic/atca_basic_info.o.d" -MT ${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/basic/atca_basic_info.o  -o ${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/basic/atca_basic_info.o mcc_generated_files/cryptoauthlib/lib/basic/atca_basic_info.c  -DXPRJ_default=$(CND_CONF)  $(COMPARISON_BUILD) 
	
${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/basic/atca_basic.o: mcc_generated_files/cryptoauthlib/lib/basic/atca_basic.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/basic" 
	@${RM} ${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/basic/atca_basic.o.d 
	@${RM} ${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/basic/atca_basic.o 
	 ${MP_CC}  $(MP_EXTRA_CC_PRE) -mmcu=atmega4808  -I "C:\Program Files (x86)\Microchip\MPLABX\v5.15\packs\Microchip\ATmega_DFP\2.0.2\include"  -B "C:\Program Files (x86)\Microchip\MPLABX\v5.15\packs\Microchip\ATmega_DFP\2.0.2\gcc\dev\atmega4808"  -x c -c -D__$(MP_PROCESSOR_OPTION)__  -funsigned-char -funsigned-bitfields -Os -ffunction-sections -fdata-sections -fpack-struct -fshort-enums -Wall -MD -MP -MF "${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/basic/atca_basic.o.d" -MT "${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/basic/atca_basic.o.d" -MT ${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/basic/atca_basic.o  -o ${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/basic/atca_basic.o mcc_generated_files/cryptoauthlib/lib/basic/atca_basic.c  -DXPRJ_default=$(CND_CONF)  $(COMPARISON_BUILD) 
	
${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/basic/atca_basic_read.o: mcc_generated_files/cryptoauthlib/lib/basic/atca_basic_read.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/basic" 
	@${RM} ${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/basic/atca_basic_read.o.d 
	@${RM} ${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/basic/atca_basic_read.o 
	 ${MP_CC}  $(MP_EXTRA_CC_PRE) -mmcu=atmega4808  -I "C:\Program Files (x86)\Microchip\MPLABX\v5.15\packs\Microchip\ATmega_DFP\2.0.2\include"  -B "C:\Program Files (x86)\Microchip\MPLABX\v5.15\packs\Microchip\ATmega_DFP\2.0.2\gcc\dev\atmega4808"  -x c -c -D__$(MP_PROCESSOR_OPTION)__  -funsigned-char -funsigned-bitfields -Os -ffunction-sections -fdata-sections -fpack-struct -fshort-enums -Wall -MD -MP -MF "${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/basic/atca_basic_read.o.d" -MT "${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/basic/atca_basic_read.o.d" -MT ${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/basic/atca_basic_read.o  -o ${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/basic/atca_basic_read.o mcc_generated_files/cryptoauthlib/lib/basic/atca_basic_read.c  -DXPRJ_default=$(CND_CONF)  $(COMPARISON_BUILD) 
	
${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/basic/atca_basic_aes.o: mcc_generated_files/cryptoauthlib/lib/basic/atca_basic_aes.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/basic" 
	@${RM} ${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/basic/atca_basic_aes.o.d 
	@${RM} ${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/basic/atca_basic_aes.o 
	 ${MP_CC}  $(MP_EXTRA_CC_PRE) -mmcu=atmega4808  -I "C:\Program Files (x86)\Microchip\MPLABX\v5.15\packs\Microchip\ATmega_DFP\2.0.2\include"  -B "C:\Program Files (x86)\Microchip\MPLABX\v5.15\packs\Microchip\ATmega_DFP\2.0.2\gcc\dev\atmega4808"  -x c -c -D__$(MP_PROCESSOR_OPTION)__  -funsigned-char -funsigned-bitfields -Os -ffunction-sections -fdata-sections -fpack-struct -fshort-enums -Wall -MD -MP -MF "${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/basic/atca_basic_aes.o.d" -MT "${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/basic/atca_basic_aes.o.d" -MT ${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/basic/atca_basic_aes.o  -o ${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/basic/atca_basic_aes.o mcc_generated_files/cryptoauthlib/lib/basic/atca_basic_aes.c  -DXPRJ_default=$(CND_CONF)  $(COMPARISON_BUILD) 
	
${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/basic/atca_basic_verify.o: mcc_generated_files/cryptoauthlib/lib/basic/atca_basic_verify.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/basic" 
	@${RM} ${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/basic/atca_basic_verify.o.d 
	@${RM} ${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/basic/atca_basic_verify.o 
	 ${MP_CC}  $(MP_EXTRA_CC_PRE) -mmcu=atmega4808  -I "C:\Program Files (x86)\Microchip\MPLABX\v5.15\packs\Microchip\ATmega_DFP\2.0.2\include"  -B "C:\Program Files (x86)\Microchip\MPLABX\v5.15\packs\Microchip\ATmega_DFP\2.0.2\gcc\dev\atmega4808"  -x c -c -D__$(MP_PROCESSOR_OPTION)__  -funsigned-char -funsigned-bitfields -Os -ffunction-sections -fdata-sections -fpack-struct -fshort-enums -Wall -MD -MP -MF "${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/basic/atca_basic_verify.o.d" -MT "${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/basic/atca_basic_verify.o.d" -MT ${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/basic/atca_basic_verify.o  -o ${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/basic/atca_basic_verify.o mcc_generated_files/cryptoauthlib/lib/basic/atca_basic_verify.c  -DXPRJ_default=$(CND_CONF)  $(COMPARISON_BUILD) 
	
${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/basic/atca_basic_sha.o: mcc_generated_files/cryptoauthlib/lib/basic/atca_basic_sha.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/basic" 
	@${RM} ${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/basic/atca_basic_sha.o.d 
	@${RM} ${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/basic/atca_basic_sha.o 
	 ${MP_CC}  $(MP_EXTRA_CC_PRE) -mmcu=atmega4808  -I "C:\Program Files (x86)\Microchip\MPLABX\v5.15\packs\Microchip\ATmega_DFP\2.0.2\include"  -B "C:\Program Files (x86)\Microchip\MPLABX\v5.15\packs\Microchip\ATmega_DFP\2.0.2\gcc\dev\atmega4808"  -x c -c -D__$(MP_PROCESSOR_OPTION)__  -funsigned-char -funsigned-bitfields -Os -ffunction-sections -fdata-sections -fpack-struct -fshort-enums -Wall -MD -MP -MF "${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/basic/atca_basic_sha.o.d" -MT "${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/basic/atca_basic_sha.o.d" -MT ${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/basic/atca_basic_sha.o  -o ${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/basic/atca_basic_sha.o mcc_generated_files/cryptoauthlib/lib/basic/atca_basic_sha.c  -DXPRJ_default=$(CND_CONF)  $(COMPARISON_BUILD) 
	
${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/basic/atca_basic_genkey.o: mcc_generated_files/cryptoauthlib/lib/basic/atca_basic_genkey.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/basic" 
	@${RM} ${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/basic/atca_basic_genkey.o.d 
	@${RM} ${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/basic/atca_basic_genkey.o 
	 ${MP_CC}  $(MP_EXTRA_CC_PRE) -mmcu=atmega4808  -I "C:\Program Files (x86)\Microchip\MPLABX\v5.15\packs\Microchip\ATmega_DFP\2.0.2\include"  -B "C:\Program Files (x86)\Microchip\MPLABX\v5.15\packs\Microchip\ATmega_DFP\2.0.2\gcc\dev\atmega4808"  -x c -c -D__$(MP_PROCESSOR_OPTION)__  -funsigned-char -funsigned-bitfields -Os -ffunction-sections -fdata-sections -fpack-struct -fshort-enums -Wall -MD -MP -MF "${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/basic/atca_basic_genkey.o.d" -MT "${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/basic/atca_basic_genkey.o.d" -MT ${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/basic/atca_basic_genkey.o  -o ${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/basic/atca_basic_genkey.o mcc_generated_files/cryptoauthlib/lib/basic/atca_basic_genkey.c  -DXPRJ_default=$(CND_CONF)  $(COMPARISON_BUILD) 
	
${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/basic/atca_basic_nonce.o: mcc_generated_files/cryptoauthlib/lib/basic/atca_basic_nonce.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/basic" 
	@${RM} ${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/basic/atca_basic_nonce.o.d 
	@${RM} ${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/basic/atca_basic_nonce.o 
	 ${MP_CC}  $(MP_EXTRA_CC_PRE) -mmcu=atmega4808  -I "C:\Program Files (x86)\Microchip\MPLABX\v5.15\packs\Microchip\ATmega_DFP\2.0.2\include"  -B "C:\Program Files (x86)\Microchip\MPLABX\v5.15\packs\Microchip\ATmega_DFP\2.0.2\gcc\dev\atmega4808"  -x c -c -D__$(MP_PROCESSOR_OPTION)__  -funsigned-char -funsigned-bitfields -Os -ffunction-sections -fdata-sections -fpack-struct -fshort-enums -Wall -MD -MP -MF "${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/basic/atca_basic_nonce.o.d" -MT "${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/basic/atca_basic_nonce.o.d" -MT ${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/basic/atca_basic_nonce.o  -o ${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/basic/atca_basic_nonce.o mcc_generated_files/cryptoauthlib/lib/basic/atca_basic_nonce.c  -DXPRJ_default=$(CND_CONF)  $(COMPARISON_BUILD) 
	
${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/basic/atca_basic_sign.o: mcc_generated_files/cryptoauthlib/lib/basic/atca_basic_sign.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/basic" 
	@${RM} ${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/basic/atca_basic_sign.o.d 
	@${RM} ${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/basic/atca_basic_sign.o 
	 ${MP_CC}  $(MP_EXTRA_CC_PRE) -mmcu=atmega4808  -I "C:\Program Files (x86)\Microchip\MPLABX\v5.15\packs\Microchip\ATmega_DFP\2.0.2\include"  -B "C:\Program Files (x86)\Microchip\MPLABX\v5.15\packs\Microchip\ATmega_DFP\2.0.2\gcc\dev\atmega4808"  -x c -c -D__$(MP_PROCESSOR_OPTION)__  -funsigned-char -funsigned-bitfields -Os -ffunction-sections -fdata-sections -fpack-struct -fshort-enums -Wall -MD -MP -MF "${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/basic/atca_basic_sign.o.d" -MT "${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/basic/atca_basic_sign.o.d" -MT ${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/basic/atca_basic_sign.o  -o ${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/basic/atca_basic_sign.o mcc_generated_files/cryptoauthlib/lib/basic/atca_basic_sign.c  -DXPRJ_default=$(CND_CONF)  $(COMPARISON_BUILD) 
	
${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/basic/atca_basic_kdf.o: mcc_generated_files/cryptoauthlib/lib/basic/atca_basic_kdf.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/basic" 
	@${RM} ${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/basic/atca_basic_kdf.o.d 
	@${RM} ${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/basic/atca_basic_kdf.o 
	 ${MP_CC}  $(MP_EXTRA_CC_PRE) -mmcu=atmega4808  -I "C:\Program Files (x86)\Microchip\MPLABX\v5.15\packs\Microchip\ATmega_DFP\2.0.2\include"  -B "C:\Program Files (x86)\Microchip\MPLABX\v5.15\packs\Microchip\ATmega_DFP\2.0.2\gcc\dev\atmega4808"  -x c -c -D__$(MP_PROCESSOR_OPTION)__  -funsigned-char -funsigned-bitfields -Os -ffunction-sections -fdata-sections -fpack-struct -fshort-enums -Wall -MD -MP -MF "${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/basic/atca_basic_kdf.o.d" -MT "${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/basic/atca_basic_kdf.o.d" -MT ${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/basic/atca_basic_kdf.o  -o ${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/basic/atca_basic_kdf.o mcc_generated_files/cryptoauthlib/lib/basic/atca_basic_kdf.c  -DXPRJ_default=$(CND_CONF)  $(COMPARISON_BUILD) 
	
${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/basic/atca_basic_hmac.o: mcc_generated_files/cryptoauthlib/lib/basic/atca_basic_hmac.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/basic" 
	@${RM} ${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/basic/atca_basic_hmac.o.d 
	@${RM} ${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/basic/atca_basic_hmac.o 
	 ${MP_CC}  $(MP_EXTRA_CC_PRE) -mmcu=atmega4808  -I "C:\Program Files (x86)\Microchip\MPLABX\v5.15\packs\Microchip\ATmega_DFP\2.0.2\include"  -B "C:\Program Files (x86)\Microchip\MPLABX\v5.15\packs\Microchip\ATmega_DFP\2.0.2\gcc\dev\atmega4808"  -x c -c -D__$(MP_PROCESSOR_OPTION)__  -funsigned-char -funsigned-bitfields -Os -ffunction-sections -fdata-sections -fpack-struct -fshort-enums -Wall -MD -MP -MF "${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/basic/atca_basic_hmac.o.d" -MT "${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/basic/atca_basic_hmac.o.d" -MT ${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/basic/atca_basic_hmac.o  -o ${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/basic/atca_basic_hmac.o mcc_generated_files/cryptoauthlib/lib/basic/atca_basic_hmac.c  -DXPRJ_default=$(CND_CONF)  $(COMPARISON_BUILD) 
	
${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/basic/atca_basic_counter.o: mcc_generated_files/cryptoauthlib/lib/basic/atca_basic_counter.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/basic" 
	@${RM} ${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/basic/atca_basic_counter.o.d 
	@${RM} ${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/basic/atca_basic_counter.o 
	 ${MP_CC}  $(MP_EXTRA_CC_PRE) -mmcu=atmega4808  -I "C:\Program Files (x86)\Microchip\MPLABX\v5.15\packs\Microchip\ATmega_DFP\2.0.2\include"  -B "C:\Program Files (x86)\Microchip\MPLABX\v5.15\packs\Microchip\ATmega_DFP\2.0.2\gcc\dev\atmega4808"  -x c -c -D__$(MP_PROCESSOR_OPTION)__  -funsigned-char -funsigned-bitfields -Os -ffunction-sections -fdata-sections -fpack-struct -fshort-enums -Wall -MD -MP -MF "${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/basic/atca_basic_counter.o.d" -MT "${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/basic/atca_basic_counter.o.d" -MT ${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/basic/atca_basic_counter.o  -o ${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/basic/atca_basic_counter.o mcc_generated_files/cryptoauthlib/lib/basic/atca_basic_counter.c  -DXPRJ_default=$(CND_CONF)  $(COMPARISON_BUILD) 
	
${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/basic/atca_basic_derivekey.o: mcc_generated_files/cryptoauthlib/lib/basic/atca_basic_derivekey.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/basic" 
	@${RM} ${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/basic/atca_basic_derivekey.o.d 
	@${RM} ${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/basic/atca_basic_derivekey.o 
	 ${MP_CC}  $(MP_EXTRA_CC_PRE) -mmcu=atmega4808  -I "C:\Program Files (x86)\Microchip\MPLABX\v5.15\packs\Microchip\ATmega_DFP\2.0.2\include"  -B "C:\Program Files (x86)\Microchip\MPLABX\v5.15\packs\Microchip\ATmega_DFP\2.0.2\gcc\dev\atmega4808"  -x c -c -D__$(MP_PROCESSOR_OPTION)__  -funsigned-char -funsigned-bitfields -Os -ffunction-sections -fdata-sections -fpack-struct -fshort-enums -Wall -MD -MP -MF "${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/basic/atca_basic_derivekey.o.d" -MT "${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/basic/atca_basic_derivekey.o.d" -MT ${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/basic/atca_basic_derivekey.o  -o ${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/basic/atca_basic_derivekey.o mcc_generated_files/cryptoauthlib/lib/basic/atca_basic_derivekey.c  -DXPRJ_default=$(CND_CONF)  $(COMPARISON_BUILD) 
	
${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/basic/atca_helpers.o: mcc_generated_files/cryptoauthlib/lib/basic/atca_helpers.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/basic" 
	@${RM} ${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/basic/atca_helpers.o.d 
	@${RM} ${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/basic/atca_helpers.o 
	 ${MP_CC}  $(MP_EXTRA_CC_PRE) -mmcu=atmega4808  -I "C:\Program Files (x86)\Microchip\MPLABX\v5.15\packs\Microchip\ATmega_DFP\2.0.2\include"  -B "C:\Program Files (x86)\Microchip\MPLABX\v5.15\packs\Microchip\ATmega_DFP\2.0.2\gcc\dev\atmega4808"  -x c -c -D__$(MP_PROCESSOR_OPTION)__  -funsigned-char -funsigned-bitfields -Os -ffunction-sections -fdata-sections -fpack-struct -fshort-enums -Wall -MD -MP -MF "${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/basic/atca_helpers.o.d" -MT "${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/basic/atca_helpers.o.d" -MT ${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/basic/atca_helpers.o  -o ${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/basic/atca_helpers.o mcc_generated_files/cryptoauthlib/lib/basic/atca_helpers.c  -DXPRJ_default=$(CND_CONF)  $(COMPARISON_BUILD) 
	
${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/basic/atca_basic_lock.o: mcc_generated_files/cryptoauthlib/lib/basic/atca_basic_lock.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/basic" 
	@${RM} ${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/basic/atca_basic_lock.o.d 
	@${RM} ${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/basic/atca_basic_lock.o 
	 ${MP_CC}  $(MP_EXTRA_CC_PRE) -mmcu=atmega4808  -I "C:\Program Files (x86)\Microchip\MPLABX\v5.15\packs\Microchip\ATmega_DFP\2.0.2\include"  -B "C:\Program Files (x86)\Microchip\MPLABX\v5.15\packs\Microchip\ATmega_DFP\2.0.2\gcc\dev\atmega4808"  -x c -c -D__$(MP_PROCESSOR_OPTION)__  -funsigned-char -funsigned-bitfields -Os -ffunction-sections -fdata-sections -fpack-struct -fshort-enums -Wall -MD -MP -MF "${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/basic/atca_basic_lock.o.d" -MT "${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/basic/atca_basic_lock.o.d" -MT ${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/basic/atca_basic_lock.o  -o ${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/basic/atca_basic_lock.o mcc_generated_files/cryptoauthlib/lib/basic/atca_basic_lock.c  -DXPRJ_default=$(CND_CONF)  $(COMPARISON_BUILD) 
	
${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/basic/atca_basic_mac.o: mcc_generated_files/cryptoauthlib/lib/basic/atca_basic_mac.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/basic" 
	@${RM} ${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/basic/atca_basic_mac.o.d 
	@${RM} ${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/basic/atca_basic_mac.o 
	 ${MP_CC}  $(MP_EXTRA_CC_PRE) -mmcu=atmega4808  -I "C:\Program Files (x86)\Microchip\MPLABX\v5.15\packs\Microchip\ATmega_DFP\2.0.2\include"  -B "C:\Program Files (x86)\Microchip\MPLABX\v5.15\packs\Microchip\ATmega_DFP\2.0.2\gcc\dev\atmega4808"  -x c -c -D__$(MP_PROCESSOR_OPTION)__  -funsigned-char -funsigned-bitfields -Os -ffunction-sections -fdata-sections -fpack-struct -fshort-enums -Wall -MD -MP -MF "${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/basic/atca_basic_mac.o.d" -MT "${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/basic/atca_basic_mac.o.d" -MT ${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/basic/atca_basic_mac.o  -o ${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/basic/atca_basic_mac.o mcc_generated_files/cryptoauthlib/lib/basic/atca_basic_mac.c  -DXPRJ_default=$(CND_CONF)  $(COMPARISON_BUILD) 
	
${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/basic/atca_basic_random.o: mcc_generated_files/cryptoauthlib/lib/basic/atca_basic_random.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/basic" 
	@${RM} ${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/basic/atca_basic_random.o.d 
	@${RM} ${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/basic/atca_basic_random.o 
	 ${MP_CC}  $(MP_EXTRA_CC_PRE) -mmcu=atmega4808  -I "C:\Program Files (x86)\Microchip\MPLABX\v5.15\packs\Microchip\ATmega_DFP\2.0.2\include"  -B "C:\Program Files (x86)\Microchip\MPLABX\v5.15\packs\Microchip\ATmega_DFP\2.0.2\gcc\dev\atmega4808"  -x c -c -D__$(MP_PROCESSOR_OPTION)__  -funsigned-char -funsigned-bitfields -Os -ffunction-sections -fdata-sections -fpack-struct -fshort-enums -Wall -MD -MP -MF "${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/basic/atca_basic_random.o.d" -MT "${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/basic/atca_basic_random.o.d" -MT ${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/basic/atca_basic_random.o  -o ${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/basic/atca_basic_random.o mcc_generated_files/cryptoauthlib/lib/basic/atca_basic_random.c  -DXPRJ_default=$(CND_CONF)  $(COMPARISON_BUILD) 
	
${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/basic/atca_basic_gendig.o: mcc_generated_files/cryptoauthlib/lib/basic/atca_basic_gendig.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/basic" 
	@${RM} ${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/basic/atca_basic_gendig.o.d 
	@${RM} ${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/basic/atca_basic_gendig.o 
	 ${MP_CC}  $(MP_EXTRA_CC_PRE) -mmcu=atmega4808  -I "C:\Program Files (x86)\Microchip\MPLABX\v5.15\packs\Microchip\ATmega_DFP\2.0.2\include"  -B "C:\Program Files (x86)\Microchip\MPLABX\v5.15\packs\Microchip\ATmega_DFP\2.0.2\gcc\dev\atmega4808"  -x c -c -D__$(MP_PROCESSOR_OPTION)__  -funsigned-char -funsigned-bitfields -Os -ffunction-sections -fdata-sections -fpack-struct -fshort-enums -Wall -MD -MP -MF "${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/basic/atca_basic_gendig.o.d" -MT "${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/basic/atca_basic_gendig.o.d" -MT ${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/basic/atca_basic_gendig.o  -o ${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/basic/atca_basic_gendig.o mcc_generated_files/cryptoauthlib/lib/basic/atca_basic_gendig.c  -DXPRJ_default=$(CND_CONF)  $(COMPARISON_BUILD) 
	
${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/basic/atca_basic_write.o: mcc_generated_files/cryptoauthlib/lib/basic/atca_basic_write.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/basic" 
	@${RM} ${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/basic/atca_basic_write.o.d 
	@${RM} ${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/basic/atca_basic_write.o 
	 ${MP_CC}  $(MP_EXTRA_CC_PRE) -mmcu=atmega4808  -I "C:\Program Files (x86)\Microchip\MPLABX\v5.15\packs\Microchip\ATmega_DFP\2.0.2\include"  -B "C:\Program Files (x86)\Microchip\MPLABX\v5.15\packs\Microchip\ATmega_DFP\2.0.2\gcc\dev\atmega4808"  -x c -c -D__$(MP_PROCESSOR_OPTION)__  -funsigned-char -funsigned-bitfields -Os -ffunction-sections -fdata-sections -fpack-struct -fshort-enums -Wall -MD -MP -MF "${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/basic/atca_basic_write.o.d" -MT "${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/basic/atca_basic_write.o.d" -MT ${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/basic/atca_basic_write.o  -o ${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/basic/atca_basic_write.o mcc_generated_files/cryptoauthlib/lib/basic/atca_basic_write.c  -DXPRJ_default=$(CND_CONF)  $(COMPARISON_BUILD) 
	
${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/crypto/hashes/sha1_routines.o: mcc_generated_files/cryptoauthlib/lib/crypto/hashes/sha1_routines.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/crypto/hashes" 
	@${RM} ${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/crypto/hashes/sha1_routines.o.d 
	@${RM} ${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/crypto/hashes/sha1_routines.o 
	 ${MP_CC}  $(MP_EXTRA_CC_PRE) -mmcu=atmega4808  -I "C:\Program Files (x86)\Microchip\MPLABX\v5.15\packs\Microchip\ATmega_DFP\2.0.2\include"  -B "C:\Program Files (x86)\Microchip\MPLABX\v5.15\packs\Microchip\ATmega_DFP\2.0.2\gcc\dev\atmega4808"  -x c -c -D__$(MP_PROCESSOR_OPTION)__  -funsigned-char -funsigned-bitfields -Os -ffunction-sections -fdata-sections -fpack-struct -fshort-enums -Wall -MD -MP -MF "${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/crypto/hashes/sha1_routines.o.d" -MT "${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/crypto/hashes/sha1_routines.o.d" -MT ${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/crypto/hashes/sha1_routines.o  -o ${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/crypto/hashes/sha1_routines.o mcc_generated_files/cryptoauthlib/lib/crypto/hashes/sha1_routines.c  -DXPRJ_default=$(CND_CONF)  $(COMPARISON_BUILD) 
	
${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/crypto/hashes/sha2_routines.o: mcc_generated_files/cryptoauthlib/lib/crypto/hashes/sha2_routines.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/crypto/hashes" 
	@${RM} ${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/crypto/hashes/sha2_routines.o.d 
	@${RM} ${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/crypto/hashes/sha2_routines.o 
	 ${MP_CC}  $(MP_EXTRA_CC_PRE) -mmcu=atmega4808  -I "C:\Program Files (x86)\Microchip\MPLABX\v5.15\packs\Microchip\ATmega_DFP\2.0.2\include"  -B "C:\Program Files (x86)\Microchip\MPLABX\v5.15\packs\Microchip\ATmega_DFP\2.0.2\gcc\dev\atmega4808"  -x c -c -D__$(MP_PROCESSOR_OPTION)__  -funsigned-char -funsigned-bitfields -Os -ffunction-sections -fdata-sections -fpack-struct -fshort-enums -Wall -MD -MP -MF "${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/crypto/hashes/sha2_routines.o.d" -MT "${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/crypto/hashes/sha2_routines.o.d" -MT ${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/crypto/hashes/sha2_routines.o  -o ${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/crypto/hashes/sha2_routines.o mcc_generated_files/cryptoauthlib/lib/crypto/hashes/sha2_routines.c  -DXPRJ_default=$(CND_CONF)  $(COMPARISON_BUILD) 
	
${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/crypto/atca_crypto_sw_sha1.o: mcc_generated_files/cryptoauthlib/lib/crypto/atca_crypto_sw_sha1.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/crypto" 
	@${RM} ${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/crypto/atca_crypto_sw_sha1.o.d 
	@${RM} ${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/crypto/atca_crypto_sw_sha1.o 
	 ${MP_CC}  $(MP_EXTRA_CC_PRE) -mmcu=atmega4808  -I "C:\Program Files (x86)\Microchip\MPLABX\v5.15\packs\Microchip\ATmega_DFP\2.0.2\include"  -B "C:\Program Files (x86)\Microchip\MPLABX\v5.15\packs\Microchip\ATmega_DFP\2.0.2\gcc\dev\atmega4808"  -x c -c -D__$(MP_PROCESSOR_OPTION)__  -funsigned-char -funsigned-bitfields -Os -ffunction-sections -fdata-sections -fpack-struct -fshort-enums -Wall -MD -MP -MF "${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/crypto/atca_crypto_sw_sha1.o.d" -MT "${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/crypto/atca_crypto_sw_sha1.o.d" -MT ${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/crypto/atca_crypto_sw_sha1.o  -o ${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/crypto/atca_crypto_sw_sha1.o mcc_generated_files/cryptoauthlib/lib/crypto/atca_crypto_sw_sha1.c  -DXPRJ_default=$(CND_CONF)  $(COMPARISON_BUILD) 
	
${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/crypto/atca_crypto_sw_ecdsa.o: mcc_generated_files/cryptoauthlib/lib/crypto/atca_crypto_sw_ecdsa.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/crypto" 
	@${RM} ${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/crypto/atca_crypto_sw_ecdsa.o.d 
	@${RM} ${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/crypto/atca_crypto_sw_ecdsa.o 
	 ${MP_CC}  $(MP_EXTRA_CC_PRE) -mmcu=atmega4808  -I "C:\Program Files (x86)\Microchip\MPLABX\v5.15\packs\Microchip\ATmega_DFP\2.0.2\include"  -B "C:\Program Files (x86)\Microchip\MPLABX\v5.15\packs\Microchip\ATmega_DFP\2.0.2\gcc\dev\atmega4808"  -x c -c -D__$(MP_PROCESSOR_OPTION)__  -funsigned-char -funsigned-bitfields -Os -ffunction-sections -fdata-sections -fpack-struct -fshort-enums -Wall -MD -MP -MF "${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/crypto/atca_crypto_sw_ecdsa.o.d" -MT "${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/crypto/atca_crypto_sw_ecdsa.o.d" -MT ${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/crypto/atca_crypto_sw_ecdsa.o  -o ${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/crypto/atca_crypto_sw_ecdsa.o mcc_generated_files/cryptoauthlib/lib/crypto/atca_crypto_sw_ecdsa.c  -DXPRJ_default=$(CND_CONF)  $(COMPARISON_BUILD) 
	
${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/crypto/atca_crypto_sw_sha2.o: mcc_generated_files/cryptoauthlib/lib/crypto/atca_crypto_sw_sha2.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/crypto" 
	@${RM} ${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/crypto/atca_crypto_sw_sha2.o.d 
	@${RM} ${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/crypto/atca_crypto_sw_sha2.o 
	 ${MP_CC}  $(MP_EXTRA_CC_PRE) -mmcu=atmega4808  -I "C:\Program Files (x86)\Microchip\MPLABX\v5.15\packs\Microchip\ATmega_DFP\2.0.2\include"  -B "C:\Program Files (x86)\Microchip\MPLABX\v5.15\packs\Microchip\ATmega_DFP\2.0.2\gcc\dev\atmega4808"  -x c -c -D__$(MP_PROCESSOR_OPTION)__  -funsigned-char -funsigned-bitfields -Os -ffunction-sections -fdata-sections -fpack-struct -fshort-enums -Wall -MD -MP -MF "${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/crypto/atca_crypto_sw_sha2.o.d" -MT "${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/crypto/atca_crypto_sw_sha2.o.d" -MT ${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/crypto/atca_crypto_sw_sha2.o  -o ${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/crypto/atca_crypto_sw_sha2.o mcc_generated_files/cryptoauthlib/lib/crypto/atca_crypto_sw_sha2.c  -DXPRJ_default=$(CND_CONF)  $(COMPARISON_BUILD) 
	
${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/crypto/atca_crypto_sw_rand.o: mcc_generated_files/cryptoauthlib/lib/crypto/atca_crypto_sw_rand.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/crypto" 
	@${RM} ${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/crypto/atca_crypto_sw_rand.o.d 
	@${RM} ${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/crypto/atca_crypto_sw_rand.o 
	 ${MP_CC}  $(MP_EXTRA_CC_PRE) -mmcu=atmega4808  -I "C:\Program Files (x86)\Microchip\MPLABX\v5.15\packs\Microchip\ATmega_DFP\2.0.2\include"  -B "C:\Program Files (x86)\Microchip\MPLABX\v5.15\packs\Microchip\ATmega_DFP\2.0.2\gcc\dev\atmega4808"  -x c -c -D__$(MP_PROCESSOR_OPTION)__  -funsigned-char -funsigned-bitfields -Os -ffunction-sections -fdata-sections -fpack-struct -fshort-enums -Wall -MD -MP -MF "${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/crypto/atca_crypto_sw_rand.o.d" -MT "${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/crypto/atca_crypto_sw_rand.o.d" -MT ${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/crypto/atca_crypto_sw_rand.o  -o ${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/crypto/atca_crypto_sw_rand.o mcc_generated_files/cryptoauthlib/lib/crypto/atca_crypto_sw_rand.c  -DXPRJ_default=$(CND_CONF)  $(COMPARISON_BUILD) 
	
${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/hal/hal_timer.o: mcc_generated_files/cryptoauthlib/lib/hal/hal_timer.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/hal" 
	@${RM} ${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/hal/hal_timer.o.d 
	@${RM} ${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/hal/hal_timer.o 
	 ${MP_CC}  $(MP_EXTRA_CC_PRE) -mmcu=atmega4808  -I "C:\Program Files (x86)\Microchip\MPLABX\v5.15\packs\Microchip\ATmega_DFP\2.0.2\include"  -B "C:\Program Files (x86)\Microchip\MPLABX\v5.15\packs\Microchip\ATmega_DFP\2.0.2\gcc\dev\atmega4808"  -x c -c -D__$(MP_PROCESSOR_OPTION)__  -funsigned-char -funsigned-bitfields -Os -ffunction-sections -fdata-sections -fpack-struct -fshort-enums -Wall -MD -MP -MF "${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/hal/hal_timer.o.d" -MT "${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/hal/hal_timer.o.d" -MT ${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/hal/hal_timer.o  -o ${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/hal/hal_timer.o mcc_generated_files/cryptoauthlib/lib/hal/hal_timer.c  -DXPRJ_default=$(CND_CONF)  $(COMPARISON_BUILD) 
	
${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/hal/hal_i2c.o: mcc_generated_files/cryptoauthlib/lib/hal/hal_i2c.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/hal" 
	@${RM} ${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/hal/hal_i2c.o.d 
	@${RM} ${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/hal/hal_i2c.o 
	 ${MP_CC}  $(MP_EXTRA_CC_PRE) -mmcu=atmega4808  -I "C:\Program Files (x86)\Microchip\MPLABX\v5.15\packs\Microchip\ATmega_DFP\2.0.2\include"  -B "C:\Program Files (x86)\Microchip\MPLABX\v5.15\packs\Microchip\ATmega_DFP\2.0.2\gcc\dev\atmega4808"  -x c -c -D__$(MP_PROCESSOR_OPTION)__  -funsigned-char -funsigned-bitfields -Os -ffunction-sections -fdata-sections -fpack-struct -fshort-enums -Wall -MD -MP -MF "${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/hal/hal_i2c.o.d" -MT "${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/hal/hal_i2c.o.d" -MT ${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/hal/hal_i2c.o  -o ${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/hal/hal_i2c.o mcc_generated_files/cryptoauthlib/lib/hal/hal_i2c.c  -DXPRJ_default=$(CND_CONF)  $(COMPARISON_BUILD) 
	
${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/hal/atca_hal.o: mcc_generated_files/cryptoauthlib/lib/hal/atca_hal.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/hal" 
	@${RM} ${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/hal/atca_hal.o.d 
	@${RM} ${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/hal/atca_hal.o 
	 ${MP_CC}  $(MP_EXTRA_CC_PRE) -mmcu=atmega4808  -I "C:\Program Files (x86)\Microchip\MPLABX\v5.15\packs\Microchip\ATmega_DFP\2.0.2\include"  -B "C:\Program Files (x86)\Microchip\MPLABX\v5.15\packs\Microchip\ATmega_DFP\2.0.2\gcc\dev\atmega4808"  -x c -c -D__$(MP_PROCESSOR_OPTION)__  -funsigned-char -funsigned-bitfields -Os -ffunction-sections -fdata-sections -fpack-struct -fshort-enums -Wall -MD -MP -MF "${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/hal/atca_hal.o.d" -MT "${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/hal/atca_hal.o.d" -MT ${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/hal/atca_hal.o  -o ${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/hal/atca_hal.o mcc_generated_files/cryptoauthlib/lib/hal/atca_hal.c  -DXPRJ_default=$(CND_CONF)  $(COMPARISON_BUILD) 
	
${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/host/atca_host.o: mcc_generated_files/cryptoauthlib/lib/host/atca_host.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/host" 
	@${RM} ${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/host/atca_host.o.d 
	@${RM} ${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/host/atca_host.o 
	 ${MP_CC}  $(MP_EXTRA_CC_PRE) -mmcu=atmega4808  -I "C:\Program Files (x86)\Microchip\MPLABX\v5.15\packs\Microchip\ATmega_DFP\2.0.2\include"  -B "C:\Program Files (x86)\Microchip\MPLABX\v5.15\packs\Microchip\ATmega_DFP\2.0.2\gcc\dev\atmega4808"  -x c -c -D__$(MP_PROCESSOR_OPTION)__  -funsigned-char -funsigned-bitfields -Os -ffunction-sections -fdata-sections -fpack-struct -fshort-enums -Wall -MD -MP -MF "${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/host/atca_host.o.d" -MT "${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/host/atca_host.o.d" -MT ${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/host/atca_host.o  -o ${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/host/atca_host.o mcc_generated_files/cryptoauthlib/lib/host/atca_host.c  -DXPRJ_default=$(CND_CONF)  $(COMPARISON_BUILD) 
	
${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/jwt/atca_jwt.o: mcc_generated_files/cryptoauthlib/lib/jwt/atca_jwt.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/jwt" 
	@${RM} ${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/jwt/atca_jwt.o.d 
	@${RM} ${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/jwt/atca_jwt.o 
	 ${MP_CC}  $(MP_EXTRA_CC_PRE) -mmcu=atmega4808  -I "C:\Program Files (x86)\Microchip\MPLABX\v5.15\packs\Microchip\ATmega_DFP\2.0.2\include"  -B "C:\Program Files (x86)\Microchip\MPLABX\v5.15\packs\Microchip\ATmega_DFP\2.0.2\gcc\dev\atmega4808"  -x c -c -D__$(MP_PROCESSOR_OPTION)__  -funsigned-char -funsigned-bitfields -Os -ffunction-sections -fdata-sections -fpack-struct -fshort-enums -Wall -MD -MP -MF "${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/jwt/atca_jwt.o.d" -MT "${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/jwt/atca_jwt.o.d" -MT ${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/jwt/atca_jwt.o  -o ${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/jwt/atca_jwt.o mcc_generated_files/cryptoauthlib/lib/jwt/atca_jwt.c  -DXPRJ_default=$(CND_CONF)  $(COMPARISON_BUILD) 
	
${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/tls/atcatls_cfg.o: mcc_generated_files/cryptoauthlib/lib/tls/atcatls_cfg.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/tls" 
	@${RM} ${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/tls/atcatls_cfg.o.d 
	@${RM} ${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/tls/atcatls_cfg.o 
	 ${MP_CC}  $(MP_EXTRA_CC_PRE) -mmcu=atmega4808  -I "C:\Program Files (x86)\Microchip\MPLABX\v5.15\packs\Microchip\ATmega_DFP\2.0.2\include"  -B "C:\Program Files (x86)\Microchip\MPLABX\v5.15\packs\Microchip\ATmega_DFP\2.0.2\gcc\dev\atmega4808"  -x c -c -D__$(MP_PROCESSOR_OPTION)__  -funsigned-char -funsigned-bitfields -Os -ffunction-sections -fdata-sections -fpack-struct -fshort-enums -Wall -MD -MP -MF "${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/tls/atcatls_cfg.o.d" -MT "${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/tls/atcatls_cfg.o.d" -MT ${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/tls/atcatls_cfg.o  -o ${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/tls/atcatls_cfg.o mcc_generated_files/cryptoauthlib/lib/tls/atcatls_cfg.c  -DXPRJ_default=$(CND_CONF)  $(COMPARISON_BUILD) 
	
${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/tls/atcatls.o: mcc_generated_files/cryptoauthlib/lib/tls/atcatls.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/tls" 
	@${RM} ${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/tls/atcatls.o.d 
	@${RM} ${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/tls/atcatls.o 
	 ${MP_CC}  $(MP_EXTRA_CC_PRE) -mmcu=atmega4808  -I "C:\Program Files (x86)\Microchip\MPLABX\v5.15\packs\Microchip\ATmega_DFP\2.0.2\include"  -B "C:\Program Files (x86)\Microchip\MPLABX\v5.15\packs\Microchip\ATmega_DFP\2.0.2\gcc\dev\atmega4808"  -x c -c -D__$(MP_PROCESSOR_OPTION)__  -funsigned-char -funsigned-bitfields -Os -ffunction-sections -fdata-sections -fpack-struct -fshort-enums -Wall -MD -MP -MF "${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/tls/atcatls.o.d" -MT "${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/tls/atcatls.o.d" -MT ${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/tls/atcatls.o  -o ${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/tls/atcatls.o mcc_generated_files/cryptoauthlib/lib/tls/atcatls.c  -DXPRJ_default=$(CND_CONF)  $(COMPARISON_BUILD) 
	
${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/atca_device.o: mcc_generated_files/cryptoauthlib/lib/atca_device.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib" 
	@${RM} ${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/atca_device.o.d 
	@${RM} ${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/atca_device.o 
	 ${MP_CC}  $(MP_EXTRA_CC_PRE) -mmcu=atmega4808  -I "C:\Program Files (x86)\Microchip\MPLABX\v5.15\packs\Microchip\ATmega_DFP\2.0.2\include"  -B "C:\Program Files (x86)\Microchip\MPLABX\v5.15\packs\Microchip\ATmega_DFP\2.0.2\gcc\dev\atmega4808"  -x c -c -D__$(MP_PROCESSOR_OPTION)__  -funsigned-char -funsigned-bitfields -Os -ffunction-sections -fdata-sections -fpack-struct -fshort-enums -Wall -MD -MP -MF "${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/atca_device.o.d" -MT "${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/atca_device.o.d" -MT ${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/atca_device.o  -o ${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/atca_device.o mcc_generated_files/cryptoauthlib/lib/atca_device.c  -DXPRJ_default=$(CND_CONF)  $(COMPARISON_BUILD) 
	
${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/atca_execution.o: mcc_generated_files/cryptoauthlib/lib/atca_execution.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib" 
	@${RM} ${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/atca_execution.o.d 
	@${RM} ${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/atca_execution.o 
	 ${MP_CC}  $(MP_EXTRA_CC_PRE) -mmcu=atmega4808  -I "C:\Program Files (x86)\Microchip\MPLABX\v5.15\packs\Microchip\ATmega_DFP\2.0.2\include"  -B "C:\Program Files (x86)\Microchip\MPLABX\v5.15\packs\Microchip\ATmega_DFP\2.0.2\gcc\dev\atmega4808"  -x c -c -D__$(MP_PROCESSOR_OPTION)__  -funsigned-char -funsigned-bitfields -Os -ffunction-sections -fdata-sections -fpack-struct -fshort-enums -Wall -MD -MP -MF "${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/atca_execution.o.d" -MT "${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/atca_execution.o.d" -MT ${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/atca_execution.o  -o ${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/atca_execution.o mcc_generated_files/cryptoauthlib/lib/atca_execution.c  -DXPRJ_default=$(CND_CONF)  $(COMPARISON_BUILD) 
	
${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/atca_command.o: mcc_generated_files/cryptoauthlib/lib/atca_command.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib" 
	@${RM} ${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/atca_command.o.d 
	@${RM} ${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/atca_command.o 
	 ${MP_CC}  $(MP_EXTRA_CC_PRE) -mmcu=atmega4808  -I "C:\Program Files (x86)\Microchip\MPLABX\v5.15\packs\Microchip\ATmega_DFP\2.0.2\include"  -B "C:\Program Files (x86)\Microchip\MPLABX\v5.15\packs\Microchip\ATmega_DFP\2.0.2\gcc\dev\atmega4808"  -x c -c -D__$(MP_PROCESSOR_OPTION)__  -funsigned-char -funsigned-bitfields -Os -ffunction-sections -fdata-sections -fpack-struct -fshort-enums -Wall -MD -MP -MF "${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/atca_command.o.d" -MT "${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/atca_command.o.d" -MT ${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/atca_command.o  -o ${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/atca_command.o mcc_generated_files/cryptoauthlib/lib/atca_command.c  -DXPRJ_default=$(CND_CONF)  $(COMPARISON_BUILD) 
	
${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/atca_cfgs.o: mcc_generated_files/cryptoauthlib/lib/atca_cfgs.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib" 
	@${RM} ${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/atca_cfgs.o.d 
	@${RM} ${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/atca_cfgs.o 
	 ${MP_CC}  $(MP_EXTRA_CC_PRE) -mmcu=atmega4808  -I "C:\Program Files (x86)\Microchip\MPLABX\v5.15\packs\Microchip\ATmega_DFP\2.0.2\include"  -B "C:\Program Files (x86)\Microchip\MPLABX\v5.15\packs\Microchip\ATmega_DFP\2.0.2\gcc\dev\atmega4808"  -x c -c -D__$(MP_PROCESSOR_OPTION)__  -funsigned-char -funsigned-bitfields -Os -ffunction-sections -fdata-sections -fpack-struct -fshort-enums -Wall -MD -MP -MF "${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/atca_cfgs.o.d" -MT "${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/atca_cfgs.o.d" -MT ${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/atca_cfgs.o  -o ${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/atca_cfgs.o mcc_generated_files/cryptoauthlib/lib/atca_cfgs.c  -DXPRJ_default=$(CND_CONF)  $(COMPARISON_BUILD) 
	
${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/atca_iface.o: mcc_generated_files/cryptoauthlib/lib/atca_iface.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib" 
	@${RM} ${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/atca_iface.o.d 
	@${RM} ${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/atca_iface.o 
	 ${MP_CC}  $(MP_EXTRA_CC_PRE) -mmcu=atmega4808  -I "C:\Program Files (x86)\Microchip\MPLABX\v5.15\packs\Microchip\ATmega_DFP\2.0.2\include"  -B "C:\Program Files (x86)\Microchip\MPLABX\v5.15\packs\Microchip\ATmega_DFP\2.0.2\gcc\dev\atmega4808"  -x c -c -D__$(MP_PROCESSOR_OPTION)__  -funsigned-char -funsigned-bitfields -Os -ffunction-sections -fdata-sections -fpack-struct -fshort-enums -Wall -MD -MP -MF "${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/atca_iface.o.d" -MT "${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/atca_iface.o.d" -MT ${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/atca_iface.o  -o ${OBJECTDIR}/mcc_generated_files/cryptoauthlib/lib/atca_iface.o mcc_generated_files/cryptoauthlib/lib/atca_iface.c  -DXPRJ_default=$(CND_CONF)  $(COMPARISON_BUILD) 
	
${OBJECTDIR}/mcc_generated_files/drivers/i2c_simple_master.o: mcc_generated_files/drivers/i2c_simple_master.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/mcc_generated_files/drivers" 
	@${RM} ${OBJECTDIR}/mcc_generated_files/drivers/i2c_simple_master.o.d 
	@${RM} ${OBJECTDIR}/mcc_generated_files/drivers/i2c_simple_master.o 
	 ${MP_CC}  $(MP_EXTRA_CC_PRE) -mmcu=atmega4808  -I "C:\Program Files (x86)\Microchip\MPLABX\v5.15\packs\Microchip\ATmega_DFP\2.0.2\include"  -B "C:\Program Files (x86)\Microchip\MPLABX\v5.15\packs\Microchip\ATmega_DFP\2.0.2\gcc\dev\atmega4808"  -x c -c -D__$(MP_PROCESSOR_OPTION)__  -funsigned-char -funsigned-bitfields -Os -ffunction-sections -fdata-sections -fpack-struct -fshort-enums -Wall -MD -MP -MF "${OBJECTDIR}/mcc_generated_files/drivers/i2c_simple_master.o.d" -MT "${OBJECTDIR}/mcc_generated_files/drivers/i2c_simple_master.o.d" -MT ${OBJECTDIR}/mcc_generated_files/drivers/i2c_simple_master.o  -o ${OBJECTDIR}/mcc_generated_files/drivers/i2c_simple_master.o mcc_generated_files/drivers/i2c_simple_master.c  -DXPRJ_default=$(CND_CONF)  $(COMPARISON_BUILD) 
	
${OBJECTDIR}/mcc_generated_files/mqtt/mqtt_comm_bsd/mqtt_comm_layer.o: mcc_generated_files/mqtt/mqtt_comm_bsd/mqtt_comm_layer.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/mcc_generated_files/mqtt/mqtt_comm_bsd" 
	@${RM} ${OBJECTDIR}/mcc_generated_files/mqtt/mqtt_comm_bsd/mqtt_comm_layer.o.d 
	@${RM} ${OBJECTDIR}/mcc_generated_files/mqtt/mqtt_comm_bsd/mqtt_comm_layer.o 
	 ${MP_CC}  $(MP_EXTRA_CC_PRE) -mmcu=atmega4808  -I "C:\Program Files (x86)\Microchip\MPLABX\v5.15\packs\Microchip\ATmega_DFP\2.0.2\include"  -B "C:\Program Files (x86)\Microchip\MPLABX\v5.15\packs\Microchip\ATmega_DFP\2.0.2\gcc\dev\atmega4808"  -x c -c -D__$(MP_PROCESSOR_OPTION)__  -funsigned-char -funsigned-bitfields -Os -ffunction-sections -fdata-sections -fpack-struct -fshort-enums -Wall -MD -MP -MF "${OBJECTDIR}/mcc_generated_files/mqtt/mqtt_comm_bsd/mqtt_comm_layer.o.d" -MT "${OBJECTDIR}/mcc_generated_files/mqtt/mqtt_comm_bsd/mqtt_comm_layer.o.d" -MT ${OBJECTDIR}/mcc_generated_files/mqtt/mqtt_comm_bsd/mqtt_comm_layer.o  -o ${OBJECTDIR}/mcc_generated_files/mqtt/mqtt_comm_bsd/mqtt_comm_layer.o mcc_generated_files/mqtt/mqtt_comm_bsd/mqtt_comm_layer.c  -DXPRJ_default=$(CND_CONF)  $(COMPARISON_BUILD) 
	
${OBJECTDIR}/mcc_generated_files/mqtt/mqtt_core/mqtt_core.o: mcc_generated_files/mqtt/mqtt_core/mqtt_core.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/mcc_generated_files/mqtt/mqtt_core" 
	@${RM} ${OBJECTDIR}/mcc_generated_files/mqtt/mqtt_core/mqtt_core.o.d 
	@${RM} ${OBJECTDIR}/mcc_generated_files/mqtt/mqtt_core/mqtt_core.o 
	 ${MP_CC}  $(MP_EXTRA_CC_PRE) -mmcu=atmega4808  -I "C:\Program Files (x86)\Microchip\MPLABX\v5.15\packs\Microchip\ATmega_DFP\2.0.2\include"  -B "C:\Program Files (x86)\Microchip\MPLABX\v5.15\packs\Microchip\ATmega_DFP\2.0.2\gcc\dev\atmega4808"  -x c -c -D__$(MP_PROCESSOR_OPTION)__  -funsigned-char -funsigned-bitfields -Os -ffunction-sections -fdata-sections -fpack-struct -fshort-enums -Wall -MD -MP -MF "${OBJECTDIR}/mcc_generated_files/mqtt/mqtt_core/mqtt_core.o.d" -MT "${OBJECTDIR}/mcc_generated_files/mqtt/mqtt_core/mqtt_core.o.d" -MT ${OBJECTDIR}/mcc_generated_files/mqtt/mqtt_core/mqtt_core.o  -o ${OBJECTDIR}/mcc_generated_files/mqtt/mqtt_core/mqtt_core.o mcc_generated_files/mqtt/mqtt_core/mqtt_core.c  -DXPRJ_default=$(CND_CONF)  $(COMPARISON_BUILD) 
	
${OBJECTDIR}/mcc_generated_files/mqtt/mqtt_exchange_buffer/mqtt_exchange_buffer.o: mcc_generated_files/mqtt/mqtt_exchange_buffer/mqtt_exchange_buffer.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/mcc_generated_files/mqtt/mqtt_exchange_buffer" 
	@${RM} ${OBJECTDIR}/mcc_generated_files/mqtt/mqtt_exchange_buffer/mqtt_exchange_buffer.o.d 
	@${RM} ${OBJECTDIR}/mcc_generated_files/mqtt/mqtt_exchange_buffer/mqtt_exchange_buffer.o 
	 ${MP_CC}  $(MP_EXTRA_CC_PRE) -mmcu=atmega4808  -I "C:\Program Files (x86)\Microchip\MPLABX\v5.15\packs\Microchip\ATmega_DFP\2.0.2\include"  -B "C:\Program Files (x86)\Microchip\MPLABX\v5.15\packs\Microchip\ATmega_DFP\2.0.2\gcc\dev\atmega4808"  -x c -c -D__$(MP_PROCESSOR_OPTION)__  -funsigned-char -funsigned-bitfields -Os -ffunction-sections -fdata-sections -fpack-struct -fshort-enums -Wall -MD -MP -MF "${OBJECTDIR}/mcc_generated_files/mqtt/mqtt_exchange_buffer/mqtt_exchange_buffer.o.d" -MT "${OBJECTDIR}/mcc_generated_files/mqtt/mqtt_exchange_buffer/mqtt_exchange_buffer.o.d" -MT ${OBJECTDIR}/mcc_generated_files/mqtt/mqtt_exchange_buffer/mqtt_exchange_buffer.o  -o ${OBJECTDIR}/mcc_generated_files/mqtt/mqtt_exchange_buffer/mqtt_exchange_buffer.o mcc_generated_files/mqtt/mqtt_exchange_buffer/mqtt_exchange_buffer.c  -DXPRJ_default=$(CND_CONF)  $(COMPARISON_BUILD) 
	
${OBJECTDIR}/mcc_generated_files/mqtt/mqtt_packetTransfer_interface.o: mcc_generated_files/mqtt/mqtt_packetTransfer_interface.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/mcc_generated_files/mqtt" 
	@${RM} ${OBJECTDIR}/mcc_generated_files/mqtt/mqtt_packetTransfer_interface.o.d 
	@${RM} ${OBJECTDIR}/mcc_generated_files/mqtt/mqtt_packetTransfer_interface.o 
	 ${MP_CC}  $(MP_EXTRA_CC_PRE) -mmcu=atmega4808  -I "C:\Program Files (x86)\Microchip\MPLABX\v5.15\packs\Microchip\ATmega_DFP\2.0.2\include"  -B "C:\Program Files (x86)\Microchip\MPLABX\v5.15\packs\Microchip\ATmega_DFP\2.0.2\gcc\dev\atmega4808"  -x c -c -D__$(MP_PROCESSOR_OPTION)__  -funsigned-char -funsigned-bitfields -Os -ffunction-sections -fdata-sections -fpack-struct -fshort-enums -Wall -MD -MP -MF "${OBJECTDIR}/mcc_generated_files/mqtt/mqtt_packetTransfer_interface.o.d" -MT "${OBJECTDIR}/mcc_generated_files/mqtt/mqtt_packetTransfer_interface.o.d" -MT ${OBJECTDIR}/mcc_generated_files/mqtt/mqtt_packetTransfer_interface.o  -o ${OBJECTDIR}/mcc_generated_files/mqtt/mqtt_packetTransfer_interface.o mcc_generated_files/mqtt/mqtt_packetTransfer_interface.c  -DXPRJ_default=$(CND_CONF)  $(COMPARISON_BUILD) 
	
${OBJECTDIR}/mcc_generated_files/src/clkctrl.o: mcc_generated_files/src/clkctrl.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/mcc_generated_files/src" 
	@${RM} ${OBJECTDIR}/mcc_generated_files/src/clkctrl.o.d 
	@${RM} ${OBJECTDIR}/mcc_generated_files/src/clkctrl.o 
	 ${MP_CC}  $(MP_EXTRA_CC_PRE) -mmcu=atmega4808  -I "C:\Program Files (x86)\Microchip\MPLABX\v5.15\packs\Microchip\ATmega_DFP\2.0.2\include"  -B "C:\Program Files (x86)\Microchip\MPLABX\v5.15\packs\Microchip\ATmega_DFP\2.0.2\gcc\dev\atmega4808"  -x c -c -D__$(MP_PROCESSOR_OPTION)__  -funsigned-char -funsigned-bitfields -Os -ffunction-sections -fdata-sections -fpack-struct -fshort-enums -Wall -MD -MP -MF "${OBJECTDIR}/mcc_generated_files/src/clkctrl.o.d" -MT "${OBJECTDIR}/mcc_generated_files/src/clkctrl.o.d" -MT ${OBJECTDIR}/mcc_generated_files/src/clkctrl.o  -o ${OBJECTDIR}/mcc_generated_files/src/clkctrl.o mcc_generated_files/src/clkctrl.c  -DXPRJ_default=$(CND_CONF)  $(COMPARISON_BUILD) 
	
${OBJECTDIR}/mcc_generated_files/src/twi0.o: mcc_generated_files/src/twi0.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/mcc_generated_files/src" 
	@${RM} ${OBJECTDIR}/mcc_generated_files/src/twi0.o.d 
	@${RM} ${OBJECTDIR}/mcc_generated_files/src/twi0.o 
	 ${MP_CC}  $(MP_EXTRA_CC_PRE) -mmcu=atmega4808  -I "C:\Program Files (x86)\Microchip\MPLABX\v5.15\packs\Microchip\ATmega_DFP\2.0.2\include"  -B "C:\Program Files (x86)\Microchip\MPLABX\v5.15\packs\Microchip\ATmega_DFP\2.0.2\gcc\dev\atmega4808"  -x c -c -D__$(MP_PROCESSOR_OPTION)__  -funsigned-char -funsigned-bitfields -Os -ffunction-sections -fdata-sections -fpack-struct -fshort-enums -Wall -MD -MP -MF "${OBJECTDIR}/mcc_generated_files/src/twi0.o.d" -MT "${OBJECTDIR}/mcc_generated_files/src/twi0.o.d" -MT ${OBJECTDIR}/mcc_generated_files/src/twi0.o  -o ${OBJECTDIR}/mcc_generated_files/src/twi0.o mcc_generated_files/src/twi0.c  -DXPRJ_default=$(CND_CONF)  $(COMPARISON_BUILD) 
	
${OBJECTDIR}/mcc_generated_files/src/wdt.o: mcc_generated_files/src/wdt.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/mcc_generated_files/src" 
	@${RM} ${OBJECTDIR}/mcc_generated_files/src/wdt.o.d 
	@${RM} ${OBJECTDIR}/mcc_generated_files/src/wdt.o 
	 ${MP_CC}  $(MP_EXTRA_CC_PRE) -mmcu=atmega4808  -I "C:\Program Files (x86)\Microchip\MPLABX\v5.15\packs\Microchip\ATmega_DFP\2.0.2\include"  -B "C:\Program Files (x86)\Microchip\MPLABX\v5.15\packs\Microchip\ATmega_DFP\2.0.2\gcc\dev\atmega4808"  -x c -c -D__$(MP_PROCESSOR_OPTION)__  -funsigned-char -funsigned-bitfields -Os -ffunction-sections -fdata-sections -fpack-struct -fshort-enums -Wall -MD -MP -MF "${OBJECTDIR}/mcc_generated_files/src/wdt.o.d" -MT "${OBJECTDIR}/mcc_generated_files/src/wdt.o.d" -MT ${OBJECTDIR}/mcc_generated_files/src/wdt.o  -o ${OBJECTDIR}/mcc_generated_files/src/wdt.o mcc_generated_files/src/wdt.c  -DXPRJ_default=$(CND_CONF)  $(COMPARISON_BUILD) 
	
${OBJECTDIR}/mcc_generated_files/src/system.o: mcc_generated_files/src/system.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/mcc_generated_files/src" 
	@${RM} ${OBJECTDIR}/mcc_generated_files/src/system.o.d 
	@${RM} ${OBJECTDIR}/mcc_generated_files/src/system.o 
	 ${MP_CC}  $(MP_EXTRA_CC_PRE) -mmcu=atmega4808  -I "C:\Program Files (x86)\Microchip\MPLABX\v5.15\packs\Microchip\ATmega_DFP\2.0.2\include"  -B "C:\Program Files (x86)\Microchip\MPLABX\v5.15\packs\Microchip\ATmega_DFP\2.0.2\gcc\dev\atmega4808"  -x c -c -D__$(MP_PROCESSOR_OPTION)__  -funsigned-char -funsigned-bitfields -Os -ffunction-sections -fdata-sections -fpack-struct -fshort-enums -Wall -MD -MP -MF "${OBJECTDIR}/mcc_generated_files/src/system.o.d" -MT "${OBJECTDIR}/mcc_generated_files/src/system.o.d" -MT ${OBJECTDIR}/mcc_generated_files/src/system.o  -o ${OBJECTDIR}/mcc_generated_files/src/system.o mcc_generated_files/src/system.c  -DXPRJ_default=$(CND_CONF)  $(COMPARISON_BUILD) 
	
${OBJECTDIR}/mcc_generated_files/src/i2c_types.o: mcc_generated_files/src/i2c_types.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/mcc_generated_files/src" 
	@${RM} ${OBJECTDIR}/mcc_generated_files/src/i2c_types.o.d 
	@${RM} ${OBJECTDIR}/mcc_generated_files/src/i2c_types.o 
	 ${MP_CC}  $(MP_EXTRA_CC_PRE) -mmcu=atmega4808  -I "C:\Program Files (x86)\Microchip\MPLABX\v5.15\packs\Microchip\ATmega_DFP\2.0.2\include"  -B "C:\Program Files (x86)\Microchip\MPLABX\v5.15\packs\Microchip\ATmega_DFP\2.0.2\gcc\dev\atmega4808"  -x c -c -D__$(MP_PROCESSOR_OPTION)__  -funsigned-char -funsigned-bitfields -Os -ffunction-sections -fdata-sections -fpack-struct -fshort-enums -Wall -MD -MP -MF "${OBJECTDIR}/mcc_generated_files/src/i2c_types.o.d" -MT "${OBJECTDIR}/mcc_generated_files/src/i2c_types.o.d" -MT ${OBJECTDIR}/mcc_generated_files/src/i2c_types.o  -o ${OBJECTDIR}/mcc_generated_files/src/i2c_types.o mcc_generated_files/src/i2c_types.c  -DXPRJ_default=$(CND_CONF)  $(COMPARISON_BUILD) 
	
${OBJECTDIR}/mcc_generated_files/src/usart2.o: mcc_generated_files/src/usart2.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/mcc_generated_files/src" 
	@${RM} ${OBJECTDIR}/mcc_generated_files/src/usart2.o.d 
	@${RM} ${OBJECTDIR}/mcc_generated_files/src/usart2.o 
	 ${MP_CC}  $(MP_EXTRA_CC_PRE) -mmcu=atmega4808  -I "C:\Program Files (x86)\Microchip\MPLABX\v5.15\packs\Microchip\ATmega_DFP\2.0.2\include"  -B "C:\Program Files (x86)\Microchip\MPLABX\v5.15\packs\Microchip\ATmega_DFP\2.0.2\gcc\dev\atmega4808"  -x c -c -D__$(MP_PROCESSOR_OPTION)__  -funsigned-char -funsigned-bitfields -Os -ffunction-sections -fdata-sections -fpack-struct -fshort-enums -Wall -MD -MP -MF "${OBJECTDIR}/mcc_generated_files/src/usart2.o.d" -MT "${OBJECTDIR}/mcc_generated_files/src/usart2.o.d" -MT ${OBJECTDIR}/mcc_generated_files/src/usart2.o  -o ${OBJECTDIR}/mcc_generated_files/src/usart2.o mcc_generated_files/src/usart2.c  -DXPRJ_default=$(CND_CONF)  $(COMPARISON_BUILD) 
	
${OBJECTDIR}/mcc_generated_files/src/rtc.o: mcc_generated_files/src/rtc.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/mcc_generated_files/src" 
	@${RM} ${OBJECTDIR}/mcc_generated_files/src/rtc.o.d 
	@${RM} ${OBJECTDIR}/mcc_generated_files/src/rtc.o 
	 ${MP_CC}  $(MP_EXTRA_CC_PRE) -mmcu=atmega4808  -I "C:\Program Files (x86)\Microchip\MPLABX\v5.15\packs\Microchip\ATmega_DFP\2.0.2\include"  -B "C:\Program Files (x86)\Microchip\MPLABX\v5.15\packs\Microchip\ATmega_DFP\2.0.2\gcc\dev\atmega4808"  -x c -c -D__$(MP_PROCESSOR_OPTION)__  -funsigned-char -funsigned-bitfields -Os -ffunction-sections -fdata-sections -fpack-struct -fshort-enums -Wall -MD -MP -MF "${OBJECTDIR}/mcc_generated_files/src/rtc.o.d" -MT "${OBJECTDIR}/mcc_generated_files/src/rtc.o.d" -MT ${OBJECTDIR}/mcc_generated_files/src/rtc.o  -o ${OBJECTDIR}/mcc_generated_files/src/rtc.o mcc_generated_files/src/rtc.c  -DXPRJ_default=$(CND_CONF)  $(COMPARISON_BUILD) 
	
${OBJECTDIR}/mcc_generated_files/src/pin_manager.o: mcc_generated_files/src/pin_manager.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/mcc_generated_files/src" 
	@${RM} ${OBJECTDIR}/mcc_generated_files/src/pin_manager.o.d 
	@${RM} ${OBJECTDIR}/mcc_generated_files/src/pin_manager.o 
	 ${MP_CC}  $(MP_EXTRA_CC_PRE) -mmcu=atmega4808  -I "C:\Program Files (x86)\Microchip\MPLABX\v5.15\packs\Microchip\ATmega_DFP\2.0.2\include"  -B "C:\Program Files (x86)\Microchip\MPLABX\v5.15\packs\Microchip\ATmega_DFP\2.0.2\gcc\dev\atmega4808"  -x c -c -D__$(MP_PROCESSOR_OPTION)__  -funsigned-char -funsigned-bitfields -Os -ffunction-sections -fdata-sections -fpack-struct -fshort-enums -Wall -MD -MP -MF "${OBJECTDIR}/mcc_generated_files/src/pin_manager.o.d" -MT "${OBJECTDIR}/mcc_generated_files/src/pin_manager.o.d" -MT ${OBJECTDIR}/mcc_generated_files/src/pin_manager.o  -o ${OBJECTDIR}/mcc_generated_files/src/pin_manager.o mcc_generated_files/src/pin_manager.c  -DXPRJ_default=$(CND_CONF)  $(COMPARISON_BUILD) 
	
${OBJECTDIR}/mcc_generated_files/src/cpuint.o: mcc_generated_files/src/cpuint.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/mcc_generated_files/src" 
	@${RM} ${OBJECTDIR}/mcc_generated_files/src/cpuint.o.d 
	@${RM} ${OBJECTDIR}/mcc_generated_files/src/cpuint.o 
	 ${MP_CC}  $(MP_EXTRA_CC_PRE) -mmcu=atmega4808  -I "C:\Program Files (x86)\Microchip\MPLABX\v5.15\packs\Microchip\ATmega_DFP\2.0.2\include"  -B "C:\Program Files (x86)\Microchip\MPLABX\v5.15\packs\Microchip\ATmega_DFP\2.0.2\gcc\dev\atmega4808"  -x c -c -D__$(MP_PROCESSOR_OPTION)__  -funsigned-char -funsigned-bitfields -Os -ffunction-sections -fdata-sections -fpack-struct -fshort-enums -Wall -MD -MP -MF "${OBJECTDIR}/mcc_generated_files/src/cpuint.o.d" -MT "${OBJECTDIR}/mcc_generated_files/src/cpuint.o.d" -MT ${OBJECTDIR}/mcc_generated_files/src/cpuint.o  -o ${OBJECTDIR}/mcc_generated_files/src/cpuint.o mcc_generated_files/src/cpuint.c  -DXPRJ_default=$(CND_CONF)  $(COMPARISON_BUILD) 
	
${OBJECTDIR}/mcc_generated_files/src/spi0.o: mcc_generated_files/src/spi0.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/mcc_generated_files/src" 
	@${RM} ${OBJECTDIR}/mcc_generated_files/src/spi0.o.d 
	@${RM} ${OBJECTDIR}/mcc_generated_files/src/spi0.o 
	 ${MP_CC}  $(MP_EXTRA_CC_PRE) -mmcu=atmega4808  -I "C:\Program Files (x86)\Microchip\MPLABX\v5.15\packs\Microchip\ATmega_DFP\2.0.2\include"  -B "C:\Program Files (x86)\Microchip\MPLABX\v5.15\packs\Microchip\ATmega_DFP\2.0.2\gcc\dev\atmega4808"  -x c -c -D__$(MP_PROCESSOR_OPTION)__  -funsigned-char -funsigned-bitfields -Os -ffunction-sections -fdata-sections -fpack-struct -fshort-enums -Wall -MD -MP -MF "${OBJECTDIR}/mcc_generated_files/src/spi0.o.d" -MT "${OBJECTDIR}/mcc_generated_files/src/spi0.o.d" -MT ${OBJECTDIR}/mcc_generated_files/src/spi0.o  -o ${OBJECTDIR}/mcc_generated_files/src/spi0.o mcc_generated_files/src/spi0.c  -DXPRJ_default=$(CND_CONF)  $(COMPARISON_BUILD) 
	
${OBJECTDIR}/mcc_generated_files/src/adc0.o: mcc_generated_files/src/adc0.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/mcc_generated_files/src" 
	@${RM} ${OBJECTDIR}/mcc_generated_files/src/adc0.o.d 
	@${RM} ${OBJECTDIR}/mcc_generated_files/src/adc0.o 
	 ${MP_CC}  $(MP_EXTRA_CC_PRE) -mmcu=atmega4808  -I "C:\Program Files (x86)\Microchip\MPLABX\v5.15\packs\Microchip\ATmega_DFP\2.0.2\include"  -B "C:\Program Files (x86)\Microchip\MPLABX\v5.15\packs\Microchip\ATmega_DFP\2.0.2\gcc\dev\atmega4808"  -x c -c -D__$(MP_PROCESSOR_OPTION)__  -funsigned-char -funsigned-bitfields -Os -ffunction-sections -fdata-sections -fpack-struct -fshort-enums -Wall -MD -MP -MF "${OBJECTDIR}/mcc_generated_files/src/adc0.o.d" -MT "${OBJECTDIR}/mcc_generated_files/src/adc0.o.d" -MT ${OBJECTDIR}/mcc_generated_files/src/adc0.o  -o ${OBJECTDIR}/mcc_generated_files/src/adc0.o mcc_generated_files/src/adc0.c  -DXPRJ_default=$(CND_CONF)  $(COMPARISON_BUILD) 
	
${OBJECTDIR}/mcc_generated_files/src/slpctrl.o: mcc_generated_files/src/slpctrl.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/mcc_generated_files/src" 
	@${RM} ${OBJECTDIR}/mcc_generated_files/src/slpctrl.o.d 
	@${RM} ${OBJECTDIR}/mcc_generated_files/src/slpctrl.o 
	 ${MP_CC}  $(MP_EXTRA_CC_PRE) -mmcu=atmega4808  -I "C:\Program Files (x86)\Microchip\MPLABX\v5.15\packs\Microchip\ATmega_DFP\2.0.2\include"  -B "C:\Program Files (x86)\Microchip\MPLABX\v5.15\packs\Microchip\ATmega_DFP\2.0.2\gcc\dev\atmega4808"  -x c -c -D__$(MP_PROCESSOR_OPTION)__  -funsigned-char -funsigned-bitfields -Os -ffunction-sections -fdata-sections -fpack-struct -fshort-enums -Wall -MD -MP -MF "${OBJECTDIR}/mcc_generated_files/src/slpctrl.o.d" -MT "${OBJECTDIR}/mcc_generated_files/src/slpctrl.o.d" -MT ${OBJECTDIR}/mcc_generated_files/src/slpctrl.o  -o ${OBJECTDIR}/mcc_generated_files/src/slpctrl.o mcc_generated_files/src/slpctrl.c  -DXPRJ_default=$(CND_CONF)  $(COMPARISON_BUILD) 
	
${OBJECTDIR}/mcc_generated_files/src/bod.o: mcc_generated_files/src/bod.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/mcc_generated_files/src" 
	@${RM} ${OBJECTDIR}/mcc_generated_files/src/bod.o.d 
	@${RM} ${OBJECTDIR}/mcc_generated_files/src/bod.o 
	 ${MP_CC}  $(MP_EXTRA_CC_PRE) -mmcu=atmega4808  -I "C:\Program Files (x86)\Microchip\MPLABX\v5.15\packs\Microchip\ATmega_DFP\2.0.2\include"  -B "C:\Program Files (x86)\Microchip\MPLABX\v5.15\packs\Microchip\ATmega_DFP\2.0.2\gcc\dev\atmega4808"  -x c -c -D__$(MP_PROCESSOR_OPTION)__  -funsigned-char -funsigned-bitfields -Os -ffunction-sections -fdata-sections -fpack-struct -fshort-enums -Wall -MD -MP -MF "${OBJECTDIR}/mcc_generated_files/src/bod.o.d" -MT "${OBJECTDIR}/mcc_generated_files/src/bod.o.d" -MT ${OBJECTDIR}/mcc_generated_files/src/bod.o  -o ${OBJECTDIR}/mcc_generated_files/src/bod.o mcc_generated_files/src/bod.c  -DXPRJ_default=$(CND_CONF)  $(COMPARISON_BUILD) 
	
${OBJECTDIR}/mcc_generated_files/winc/bsp/source/nm_bsp_mega.o: mcc_generated_files/winc/bsp/source/nm_bsp_mega.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/mcc_generated_files/winc/bsp/source" 
	@${RM} ${OBJECTDIR}/mcc_generated_files/winc/bsp/source/nm_bsp_mega.o.d 
	@${RM} ${OBJECTDIR}/mcc_generated_files/winc/bsp/source/nm_bsp_mega.o 
	 ${MP_CC}  $(MP_EXTRA_CC_PRE) -mmcu=atmega4808  -I "C:\Program Files (x86)\Microchip\MPLABX\v5.15\packs\Microchip\ATmega_DFP\2.0.2\include"  -B "C:\Program Files (x86)\Microchip\MPLABX\v5.15\packs\Microchip\ATmega_DFP\2.0.2\gcc\dev\atmega4808"  -x c -c -D__$(MP_PROCESSOR_OPTION)__  -funsigned-char -funsigned-bitfields -Os -ffunction-sections -fdata-sections -fpack-struct -fshort-enums -Wall -MD -MP -MF "${OBJECTDIR}/mcc_generated_files/winc/bsp/source/nm_bsp_mega.o.d" -MT "${OBJECTDIR}/mcc_generated_files/winc/bsp/source/nm_bsp_mega.o.d" -MT ${OBJECTDIR}/mcc_generated_files/winc/bsp/source/nm_bsp_mega.o  -o ${OBJECTDIR}/mcc_generated_files/winc/bsp/source/nm_bsp_mega.o mcc_generated_files/winc/bsp/source/nm_bsp_mega.c  -DXPRJ_default=$(CND_CONF)  $(COMPARISON_BUILD) 
	
${OBJECTDIR}/mcc_generated_files/winc/bus_wrapper/source/nm_bus_wrapper_mega.o: mcc_generated_files/winc/bus_wrapper/source/nm_bus_wrapper_mega.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/mcc_generated_files/winc/bus_wrapper/source" 
	@${RM} ${OBJECTDIR}/mcc_generated_files/winc/bus_wrapper/source/nm_bus_wrapper_mega.o.d 
	@${RM} ${OBJECTDIR}/mcc_generated_files/winc/bus_wrapper/source/nm_bus_wrapper_mega.o 
	 ${MP_CC}  $(MP_EXTRA_CC_PRE) -mmcu=atmega4808  -I "C:\Program Files (x86)\Microchip\MPLABX\v5.15\packs\Microchip\ATmega_DFP\2.0.2\include"  -B "C:\Program Files (x86)\Microchip\MPLABX\v5.15\packs\Microchip\ATmega_DFP\2.0.2\gcc\dev\atmega4808"  -x c -c -D__$(MP_PROCESSOR_OPTION)__  -funsigned-char -funsigned-bitfields -Os -ffunction-sections -fdata-sections -fpack-struct -fshort-enums -Wall -MD -MP -MF "${OBJECTDIR}/mcc_generated_files/winc/bus_wrapper/source/nm_bus_wrapper_mega.o.d" -MT "${OBJECTDIR}/mcc_generated_files/winc/bus_wrapper/source/nm_bus_wrapper_mega.o.d" -MT ${OBJECTDIR}/mcc_generated_files/winc/bus_wrapper/source/nm_bus_wrapper_mega.o  -o ${OBJECTDIR}/mcc_generated_files/winc/bus_wrapper/source/nm_bus_wrapper_mega.o mcc_generated_files/winc/bus_wrapper/source/nm_bus_wrapper_mega.c  -DXPRJ_default=$(CND_CONF)  $(COMPARISON_BUILD) 
	
${OBJECTDIR}/mcc_generated_files/winc/common/source/nm_common.o: mcc_generated_files/winc/common/source/nm_common.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/mcc_generated_files/winc/common/source" 
	@${RM} ${OBJECTDIR}/mcc_generated_files/winc/common/source/nm_common.o.d 
	@${RM} ${OBJECTDIR}/mcc_generated_files/winc/common/source/nm_common.o 
	 ${MP_CC}  $(MP_EXTRA_CC_PRE) -mmcu=atmega4808  -I "C:\Program Files (x86)\Microchip\MPLABX\v5.15\packs\Microchip\ATmega_DFP\2.0.2\include"  -B "C:\Program Files (x86)\Microchip\MPLABX\v5.15\packs\Microchip\ATmega_DFP\2.0.2\gcc\dev\atmega4808"  -x c -c -D__$(MP_PROCESSOR_OPTION)__  -funsigned-char -funsigned-bitfields -Os -ffunction-sections -fdata-sections -fpack-struct -fshort-enums -Wall -MD -MP -MF "${OBJECTDIR}/mcc_generated_files/winc/common/source/nm_common.o.d" -MT "${OBJECTDIR}/mcc_generated_files/winc/common/source/nm_common.o.d" -MT ${OBJECTDIR}/mcc_generated_files/winc/common/source/nm_common.o  -o ${OBJECTDIR}/mcc_generated_files/winc/common/source/nm_common.o mcc_generated_files/winc/common/source/nm_common.c  -DXPRJ_default=$(CND_CONF)  $(COMPARISON_BUILD) 
	
${OBJECTDIR}/mcc_generated_files/winc/driver/source/m2m_ate_mode.o: mcc_generated_files/winc/driver/source/m2m_ate_mode.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/mcc_generated_files/winc/driver/source" 
	@${RM} ${OBJECTDIR}/mcc_generated_files/winc/driver/source/m2m_ate_mode.o.d 
	@${RM} ${OBJECTDIR}/mcc_generated_files/winc/driver/source/m2m_ate_mode.o 
	 ${MP_CC}  $(MP_EXTRA_CC_PRE) -mmcu=atmega4808  -I "C:\Program Files (x86)\Microchip\MPLABX\v5.15\packs\Microchip\ATmega_DFP\2.0.2\include"  -B "C:\Program Files (x86)\Microchip\MPLABX\v5.15\packs\Microchip\ATmega_DFP\2.0.2\gcc\dev\atmega4808"  -x c -c -D__$(MP_PROCESSOR_OPTION)__  -funsigned-char -funsigned-bitfields -Os -ffunction-sections -fdata-sections -fpack-struct -fshort-enums -Wall -MD -MP -MF "${OBJECTDIR}/mcc_generated_files/winc/driver/source/m2m_ate_mode.o.d" -MT "${OBJECTDIR}/mcc_generated_files/winc/driver/source/m2m_ate_mode.o.d" -MT ${OBJECTDIR}/mcc_generated_files/winc/driver/source/m2m_ate_mode.o  -o ${OBJECTDIR}/mcc_generated_files/winc/driver/source/m2m_ate_mode.o mcc_generated_files/winc/driver/source/m2m_ate_mode.c  -DXPRJ_default=$(CND_CONF)  $(COMPARISON_BUILD) 
	
${OBJECTDIR}/mcc_generated_files/winc/driver/source/m2m_ssl.o: mcc_generated_files/winc/driver/source/m2m_ssl.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/mcc_generated_files/winc/driver/source" 
	@${RM} ${OBJECTDIR}/mcc_generated_files/winc/driver/source/m2m_ssl.o.d 
	@${RM} ${OBJECTDIR}/mcc_generated_files/winc/driver/source/m2m_ssl.o 
	 ${MP_CC}  $(MP_EXTRA_CC_PRE) -mmcu=atmega4808  -I "C:\Program Files (x86)\Microchip\MPLABX\v5.15\packs\Microchip\ATmega_DFP\2.0.2\include"  -B "C:\Program Files (x86)\Microchip\MPLABX\v5.15\packs\Microchip\ATmega_DFP\2.0.2\gcc\dev\atmega4808"  -x c -c -D__$(MP_PROCESSOR_OPTION)__  -funsigned-char -funsigned-bitfields -Os -ffunction-sections -fdata-sections -fpack-struct -fshort-enums -Wall -MD -MP -MF "${OBJECTDIR}/mcc_generated_files/winc/driver/source/m2m_ssl.o.d" -MT "${OBJECTDIR}/mcc_generated_files/winc/driver/source/m2m_ssl.o.d" -MT ${OBJECTDIR}/mcc_generated_files/winc/driver/source/m2m_ssl.o  -o ${OBJECTDIR}/mcc_generated_files/winc/driver/source/m2m_ssl.o mcc_generated_files/winc/driver/source/m2m_ssl.c  -DXPRJ_default=$(CND_CONF)  $(COMPARISON_BUILD) 
	
${OBJECTDIR}/mcc_generated_files/winc/driver/source/m2m_ota.o: mcc_generated_files/winc/driver/source/m2m_ota.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/mcc_generated_files/winc/driver/source" 
	@${RM} ${OBJECTDIR}/mcc_generated_files/winc/driver/source/m2m_ota.o.d 
	@${RM} ${OBJECTDIR}/mcc_generated_files/winc/driver/source/m2m_ota.o 
	 ${MP_CC}  $(MP_EXTRA_CC_PRE) -mmcu=atmega4808  -I "C:\Program Files (x86)\Microchip\MPLABX\v5.15\packs\Microchip\ATmega_DFP\2.0.2\include"  -B "C:\Program Files (x86)\Microchip\MPLABX\v5.15\packs\Microchip\ATmega_DFP\2.0.2\gcc\dev\atmega4808"  -x c -c -D__$(MP_PROCESSOR_OPTION)__  -funsigned-char -funsigned-bitfields -Os -ffunction-sections -fdata-sections -fpack-struct -fshort-enums -Wall -MD -MP -MF "${OBJECTDIR}/mcc_generated_files/winc/driver/source/m2m_ota.o.d" -MT "${OBJECTDIR}/mcc_generated_files/winc/driver/source/m2m_ota.o.d" -MT ${OBJECTDIR}/mcc_generated_files/winc/driver/source/m2m_ota.o  -o ${OBJECTDIR}/mcc_generated_files/winc/driver/source/m2m_ota.o mcc_generated_files/winc/driver/source/m2m_ota.c  -DXPRJ_default=$(CND_CONF)  $(COMPARISON_BUILD) 
	
${OBJECTDIR}/mcc_generated_files/winc/driver/source/nmspi.o: mcc_generated_files/winc/driver/source/nmspi.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/mcc_generated_files/winc/driver/source" 
	@${RM} ${OBJECTDIR}/mcc_generated_files/winc/driver/source/nmspi.o.d 
	@${RM} ${OBJECTDIR}/mcc_generated_files/winc/driver/source/nmspi.o 
	 ${MP_CC}  $(MP_EXTRA_CC_PRE) -mmcu=atmega4808  -I "C:\Program Files (x86)\Microchip\MPLABX\v5.15\packs\Microchip\ATmega_DFP\2.0.2\include"  -B "C:\Program Files (x86)\Microchip\MPLABX\v5.15\packs\Microchip\ATmega_DFP\2.0.2\gcc\dev\atmega4808"  -x c -c -D__$(MP_PROCESSOR_OPTION)__  -funsigned-char -funsigned-bitfields -Os -ffunction-sections -fdata-sections -fpack-struct -fshort-enums -Wall -MD -MP -MF "${OBJECTDIR}/mcc_generated_files/winc/driver/source/nmspi.o.d" -MT "${OBJECTDIR}/mcc_generated_files/winc/driver/source/nmspi.o.d" -MT ${OBJECTDIR}/mcc_generated_files/winc/driver/source/nmspi.o  -o ${OBJECTDIR}/mcc_generated_files/winc/driver/source/nmspi.o mcc_generated_files/winc/driver/source/nmspi.c  -DXPRJ_default=$(CND_CONF)  $(COMPARISON_BUILD) 
	
${OBJECTDIR}/mcc_generated_files/winc/driver/source/m2m_hif.o: mcc_generated_files/winc/driver/source/m2m_hif.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/mcc_generated_files/winc/driver/source" 
	@${RM} ${OBJECTDIR}/mcc_generated_files/winc/driver/source/m2m_hif.o.d 
	@${RM} ${OBJECTDIR}/mcc_generated_files/winc/driver/source/m2m_hif.o 
	 ${MP_CC}  $(MP_EXTRA_CC_PRE) -mmcu=atmega4808  -I "C:\Program Files (x86)\Microchip\MPLABX\v5.15\packs\Microchip\ATmega_DFP\2.0.2\include"  -B "C:\Program Files (x86)\Microchip\MPLABX\v5.15\packs\Microchip\ATmega_DFP\2.0.2\gcc\dev\atmega4808"  -x c -c -D__$(MP_PROCESSOR_OPTION)__  -funsigned-char -funsigned-bitfields -Os -ffunction-sections -fdata-sections -fpack-struct -fshort-enums -Wall -MD -MP -MF "${OBJECTDIR}/mcc_generated_files/winc/driver/source/m2m_hif.o.d" -MT "${OBJECTDIR}/mcc_generated_files/winc/driver/source/m2m_hif.o.d" -MT ${OBJECTDIR}/mcc_generated_files/winc/driver/source/m2m_hif.o  -o ${OBJECTDIR}/mcc_generated_files/winc/driver/source/m2m_hif.o mcc_generated_files/winc/driver/source/m2m_hif.c  -DXPRJ_default=$(CND_CONF)  $(COMPARISON_BUILD) 
	
${OBJECTDIR}/mcc_generated_files/winc/driver/source/m2m_periph.o: mcc_generated_files/winc/driver/source/m2m_periph.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/mcc_generated_files/winc/driver/source" 
	@${RM} ${OBJECTDIR}/mcc_generated_files/winc/driver/source/m2m_periph.o.d 
	@${RM} ${OBJECTDIR}/mcc_generated_files/winc/driver/source/m2m_periph.o 
	 ${MP_CC}  $(MP_EXTRA_CC_PRE) -mmcu=atmega4808  -I "C:\Program Files (x86)\Microchip\MPLABX\v5.15\packs\Microchip\ATmega_DFP\2.0.2\include"  -B "C:\Program Files (x86)\Microchip\MPLABX\v5.15\packs\Microchip\ATmega_DFP\2.0.2\gcc\dev\atmega4808"  -x c -c -D__$(MP_PROCESSOR_OPTION)__  -funsigned-char -funsigned-bitfields -Os -ffunction-sections -fdata-sections -fpack-struct -fshort-enums -Wall -MD -MP -MF "${OBJECTDIR}/mcc_generated_files/winc/driver/source/m2m_periph.o.d" -MT "${OBJECTDIR}/mcc_generated_files/winc/driver/source/m2m_periph.o.d" -MT ${OBJECTDIR}/mcc_generated_files/winc/driver/source/m2m_periph.o  -o ${OBJECTDIR}/mcc_generated_files/winc/driver/source/m2m_periph.o mcc_generated_files/winc/driver/source/m2m_periph.c  -DXPRJ_default=$(CND_CONF)  $(COMPARISON_BUILD) 
	
${OBJECTDIR}/mcc_generated_files/winc/driver/source/nmasic.o: mcc_generated_files/winc/driver/source/nmasic.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/mcc_generated_files/winc/driver/source" 
	@${RM} ${OBJECTDIR}/mcc_generated_files/winc/driver/source/nmasic.o.d 
	@${RM} ${OBJECTDIR}/mcc_generated_files/winc/driver/source/nmasic.o 
	 ${MP_CC}  $(MP_EXTRA_CC_PRE) -mmcu=atmega4808  -I "C:\Program Files (x86)\Microchip\MPLABX\v5.15\packs\Microchip\ATmega_DFP\2.0.2\include"  -B "C:\Program Files (x86)\Microchip\MPLABX\v5.15\packs\Microchip\ATmega_DFP\2.0.2\gcc\dev\atmega4808"  -x c -c -D__$(MP_PROCESSOR_OPTION)__  -funsigned-char -funsigned-bitfields -Os -ffunction-sections -fdata-sections -fpack-struct -fshort-enums -Wall -MD -MP -MF "${OBJECTDIR}/mcc_generated_files/winc/driver/source/nmasic.o.d" -MT "${OBJECTDIR}/mcc_generated_files/winc/driver/source/nmasic.o.d" -MT ${OBJECTDIR}/mcc_generated_files/winc/driver/source/nmasic.o  -o ${OBJECTDIR}/mcc_generated_files/winc/driver/source/nmasic.o mcc_generated_files/winc/driver/source/nmasic.c  -DXPRJ_default=$(CND_CONF)  $(COMPARISON_BUILD) 
	
${OBJECTDIR}/mcc_generated_files/winc/driver/source/nmdrv.o: mcc_generated_files/winc/driver/source/nmdrv.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/mcc_generated_files/winc/driver/source" 
	@${RM} ${OBJECTDIR}/mcc_generated_files/winc/driver/source/nmdrv.o.d 
	@${RM} ${OBJECTDIR}/mcc_generated_files/winc/driver/source/nmdrv.o 
	 ${MP_CC}  $(MP_EXTRA_CC_PRE) -mmcu=atmega4808  -I "C:\Program Files (x86)\Microchip\MPLABX\v5.15\packs\Microchip\ATmega_DFP\2.0.2\include"  -B "C:\Program Files (x86)\Microchip\MPLABX\v5.15\packs\Microchip\ATmega_DFP\2.0.2\gcc\dev\atmega4808"  -x c -c -D__$(MP_PROCESSOR_OPTION)__  -funsigned-char -funsigned-bitfields -Os -ffunction-sections -fdata-sections -fpack-struct -fshort-enums -Wall -MD -MP -MF "${OBJECTDIR}/mcc_generated_files/winc/driver/source/nmdrv.o.d" -MT "${OBJECTDIR}/mcc_generated_files/winc/driver/source/nmdrv.o.d" -MT ${OBJECTDIR}/mcc_generated_files/winc/driver/source/nmdrv.o  -o ${OBJECTDIR}/mcc_generated_files/winc/driver/source/nmdrv.o mcc_generated_files/winc/driver/source/nmdrv.c  -DXPRJ_default=$(CND_CONF)  $(COMPARISON_BUILD) 
	
${OBJECTDIR}/mcc_generated_files/winc/driver/source/m2m_wifi.o: mcc_generated_files/winc/driver/source/m2m_wifi.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/mcc_generated_files/winc/driver/source" 
	@${RM} ${OBJECTDIR}/mcc_generated_files/winc/driver/source/m2m_wifi.o.d 
	@${RM} ${OBJECTDIR}/mcc_generated_files/winc/driver/source/m2m_wifi.o 
	 ${MP_CC}  $(MP_EXTRA_CC_PRE) -mmcu=atmega4808  -I "C:\Program Files (x86)\Microchip\MPLABX\v5.15\packs\Microchip\ATmega_DFP\2.0.2\include"  -B "C:\Program Files (x86)\Microchip\MPLABX\v5.15\packs\Microchip\ATmega_DFP\2.0.2\gcc\dev\atmega4808"  -x c -c -D__$(MP_PROCESSOR_OPTION)__  -funsigned-char -funsigned-bitfields -Os -ffunction-sections -fdata-sections -fpack-struct -fshort-enums -Wall -MD -MP -MF "${OBJECTDIR}/mcc_generated_files/winc/driver/source/m2m_wifi.o.d" -MT "${OBJECTDIR}/mcc_generated_files/winc/driver/source/m2m_wifi.o.d" -MT ${OBJECTDIR}/mcc_generated_files/winc/driver/source/m2m_wifi.o  -o ${OBJECTDIR}/mcc_generated_files/winc/driver/source/m2m_wifi.o mcc_generated_files/winc/driver/source/m2m_wifi.c  -DXPRJ_default=$(CND_CONF)  $(COMPARISON_BUILD) 
	
${OBJECTDIR}/mcc_generated_files/winc/driver/source/m2m_crypto.o: mcc_generated_files/winc/driver/source/m2m_crypto.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/mcc_generated_files/winc/driver/source" 
	@${RM} ${OBJECTDIR}/mcc_generated_files/winc/driver/source/m2m_crypto.o.d 
	@${RM} ${OBJECTDIR}/mcc_generated_files/winc/driver/source/m2m_crypto.o 
	 ${MP_CC}  $(MP_EXTRA_CC_PRE) -mmcu=atmega4808  -I "C:\Program Files (x86)\Microchip\MPLABX\v5.15\packs\Microchip\ATmega_DFP\2.0.2\include"  -B "C:\Program Files (x86)\Microchip\MPLABX\v5.15\packs\Microchip\ATmega_DFP\2.0.2\gcc\dev\atmega4808"  -x c -c -D__$(MP_PROCESSOR_OPTION)__  -funsigned-char -funsigned-bitfields -Os -ffunction-sections -fdata-sections -fpack-struct -fshort-enums -Wall -MD -MP -MF "${OBJECTDIR}/mcc_generated_files/winc/driver/source/m2m_crypto.o.d" -MT "${OBJECTDIR}/mcc_generated_files/winc/driver/source/m2m_crypto.o.d" -MT ${OBJECTDIR}/mcc_generated_files/winc/driver/source/m2m_crypto.o  -o ${OBJECTDIR}/mcc_generated_files/winc/driver/source/m2m_crypto.o mcc_generated_files/winc/driver/source/m2m_crypto.c  -DXPRJ_default=$(CND_CONF)  $(COMPARISON_BUILD) 
	
${OBJECTDIR}/mcc_generated_files/winc/driver/source/nmi2c.o: mcc_generated_files/winc/driver/source/nmi2c.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/mcc_generated_files/winc/driver/source" 
	@${RM} ${OBJECTDIR}/mcc_generated_files/winc/driver/source/nmi2c.o.d 
	@${RM} ${OBJECTDIR}/mcc_generated_files/winc/driver/source/nmi2c.o 
	 ${MP_CC}  $(MP_EXTRA_CC_PRE) -mmcu=atmega4808  -I "C:\Program Files (x86)\Microchip\MPLABX\v5.15\packs\Microchip\ATmega_DFP\2.0.2\include"  -B "C:\Program Files (x86)\Microchip\MPLABX\v5.15\packs\Microchip\ATmega_DFP\2.0.2\gcc\dev\atmega4808"  -x c -c -D__$(MP_PROCESSOR_OPTION)__  -funsigned-char -funsigned-bitfields -Os -ffunction-sections -fdata-sections -fpack-struct -fshort-enums -Wall -MD -MP -MF "${OBJECTDIR}/mcc_generated_files/winc/driver/source/nmi2c.o.d" -MT "${OBJECTDIR}/mcc_generated_files/winc/driver/source/nmi2c.o.d" -MT ${OBJECTDIR}/mcc_generated_files/winc/driver/source/nmi2c.o  -o ${OBJECTDIR}/mcc_generated_files/winc/driver/source/nmi2c.o mcc_generated_files/winc/driver/source/nmi2c.c  -DXPRJ_default=$(CND_CONF)  $(COMPARISON_BUILD) 
	
${OBJECTDIR}/mcc_generated_files/winc/driver/source/nmuart.o: mcc_generated_files/winc/driver/source/nmuart.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/mcc_generated_files/winc/driver/source" 
	@${RM} ${OBJECTDIR}/mcc_generated_files/winc/driver/source/nmuart.o.d 
	@${RM} ${OBJECTDIR}/mcc_generated_files/winc/driver/source/nmuart.o 
	 ${MP_CC}  $(MP_EXTRA_CC_PRE) -mmcu=atmega4808  -I "C:\Program Files (x86)\Microchip\MPLABX\v5.15\packs\Microchip\ATmega_DFP\2.0.2\include"  -B "C:\Program Files (x86)\Microchip\MPLABX\v5.15\packs\Microchip\ATmega_DFP\2.0.2\gcc\dev\atmega4808"  -x c -c -D__$(MP_PROCESSOR_OPTION)__  -funsigned-char -funsigned-bitfields -Os -ffunction-sections -fdata-sections -fpack-struct -fshort-enums -Wall -MD -MP -MF "${OBJECTDIR}/mcc_generated_files/winc/driver/source/nmuart.o.d" -MT "${OBJECTDIR}/mcc_generated_files/winc/driver/source/nmuart.o.d" -MT ${OBJECTDIR}/mcc_generated_files/winc/driver/source/nmuart.o  -o ${OBJECTDIR}/mcc_generated_files/winc/driver/source/nmuart.o mcc_generated_files/winc/driver/source/nmuart.c  -DXPRJ_default=$(CND_CONF)  $(COMPARISON_BUILD) 
	
${OBJECTDIR}/mcc_generated_files/winc/driver/source/nmbus.o: mcc_generated_files/winc/driver/source/nmbus.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/mcc_generated_files/winc/driver/source" 
	@${RM} ${OBJECTDIR}/mcc_generated_files/winc/driver/source/nmbus.o.d 
	@${RM} ${OBJECTDIR}/mcc_generated_files/winc/driver/source/nmbus.o 
	 ${MP_CC}  $(MP_EXTRA_CC_PRE) -mmcu=atmega4808  -I "C:\Program Files (x86)\Microchip\MPLABX\v5.15\packs\Microchip\ATmega_DFP\2.0.2\include"  -B "C:\Program Files (x86)\Microchip\MPLABX\v5.15\packs\Microchip\ATmega_DFP\2.0.2\gcc\dev\atmega4808"  -x c -c -D__$(MP_PROCESSOR_OPTION)__  -funsigned-char -funsigned-bitfields -Os -ffunction-sections -fdata-sections -fpack-struct -fshort-enums -Wall -MD -MP -MF "${OBJECTDIR}/mcc_generated_files/winc/driver/source/nmbus.o.d" -MT "${OBJECTDIR}/mcc_generated_files/winc/driver/source/nmbus.o.d" -MT ${OBJECTDIR}/mcc_generated_files/winc/driver/source/nmbus.o  -o ${OBJECTDIR}/mcc_generated_files/winc/driver/source/nmbus.o mcc_generated_files/winc/driver/source/nmbus.c  -DXPRJ_default=$(CND_CONF)  $(COMPARISON_BUILD) 
	
${OBJECTDIR}/mcc_generated_files/winc/socket/source/socket.o: mcc_generated_files/winc/socket/source/socket.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/mcc_generated_files/winc/socket/source" 
	@${RM} ${OBJECTDIR}/mcc_generated_files/winc/socket/source/socket.o.d 
	@${RM} ${OBJECTDIR}/mcc_generated_files/winc/socket/source/socket.o 
	 ${MP_CC}  $(MP_EXTRA_CC_PRE) -mmcu=atmega4808  -I "C:\Program Files (x86)\Microchip\MPLABX\v5.15\packs\Microchip\ATmega_DFP\2.0.2\include"  -B "C:\Program Files (x86)\Microchip\MPLABX\v5.15\packs\Microchip\ATmega_DFP\2.0.2\gcc\dev\atmega4808"  -x c -c -D__$(MP_PROCESSOR_OPTION)__  -funsigned-char -funsigned-bitfields -Os -ffunction-sections -fdata-sections -fpack-struct -fshort-enums -Wall -MD -MP -MF "${OBJECTDIR}/mcc_generated_files/winc/socket/source/socket.o.d" -MT "${OBJECTDIR}/mcc_generated_files/winc/socket/source/socket.o.d" -MT ${OBJECTDIR}/mcc_generated_files/winc/socket/source/socket.o  -o ${OBJECTDIR}/mcc_generated_files/winc/socket/source/socket.o mcc_generated_files/winc/socket/source/socket.c  -DXPRJ_default=$(CND_CONF)  $(COMPARISON_BUILD) 
	
${OBJECTDIR}/mcc_generated_files/winc/spi_flash/source/spi_flash.o: mcc_generated_files/winc/spi_flash/source/spi_flash.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/mcc_generated_files/winc/spi_flash/source" 
	@${RM} ${OBJECTDIR}/mcc_generated_files/winc/spi_flash/source/spi_flash.o.d 
	@${RM} ${OBJECTDIR}/mcc_generated_files/winc/spi_flash/source/spi_flash.o 
	 ${MP_CC}  $(MP_EXTRA_CC_PRE) -mmcu=atmega4808  -I "C:\Program Files (x86)\Microchip\MPLABX\v5.15\packs\Microchip\ATmega_DFP\2.0.2\include"  -B "C:\Program Files (x86)\Microchip\MPLABX\v5.15\packs\Microchip\ATmega_DFP\2.0.2\gcc\dev\atmega4808"  -x c -c -D__$(MP_PROCESSOR_OPTION)__  -funsigned-char -funsigned-bitfields -Os -ffunction-sections -fdata-sections -fpack-struct -fshort-enums -Wall -MD -MP -MF "${OBJECTDIR}/mcc_generated_files/winc/spi_flash/source/spi_flash.o.d" -MT "${OBJECTDIR}/mcc_generated_files/winc/spi_flash/source/spi_flash.o.d" -MT ${OBJECTDIR}/mcc_generated_files/winc/spi_flash/source/spi_flash.o  -o ${OBJECTDIR}/mcc_generated_files/winc/spi_flash/source/spi_flash.o mcc_generated_files/winc/spi_flash/source/spi_flash.c  -DXPRJ_default=$(CND_CONF)  $(COMPARISON_BUILD) 
	
${OBJECTDIR}/mcc_generated_files/mcc.o: mcc_generated_files/mcc.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/mcc_generated_files" 
	@${RM} ${OBJECTDIR}/mcc_generated_files/mcc.o.d 
	@${RM} ${OBJECTDIR}/mcc_generated_files/mcc.o 
	 ${MP_CC}  $(MP_EXTRA_CC_PRE) -mmcu=atmega4808  -I "C:\Program Files (x86)\Microchip\MPLABX\v5.15\packs\Microchip\ATmega_DFP\2.0.2\include"  -B "C:\Program Files (x86)\Microchip\MPLABX\v5.15\packs\Microchip\ATmega_DFP\2.0.2\gcc\dev\atmega4808"  -x c -c -D__$(MP_PROCESSOR_OPTION)__  -funsigned-char -funsigned-bitfields -Os -ffunction-sections -fdata-sections -fpack-struct -fshort-enums -Wall -MD -MP -MF "${OBJECTDIR}/mcc_generated_files/mcc.o.d" -MT "${OBJECTDIR}/mcc_generated_files/mcc.o.d" -MT ${OBJECTDIR}/mcc_generated_files/mcc.o  -o ${OBJECTDIR}/mcc_generated_files/mcc.o mcc_generated_files/mcc.c  -DXPRJ_default=$(CND_CONF)  $(COMPARISON_BUILD) 
	
${OBJECTDIR}/mcc_generated_files/application_manager.o: mcc_generated_files/application_manager.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/mcc_generated_files" 
	@${RM} ${OBJECTDIR}/mcc_generated_files/application_manager.o.d 
	@${RM} ${OBJECTDIR}/mcc_generated_files/application_manager.o 
	 ${MP_CC}  $(MP_EXTRA_CC_PRE) -mmcu=atmega4808  -I "C:\Program Files (x86)\Microchip\MPLABX\v5.15\packs\Microchip\ATmega_DFP\2.0.2\include"  -B "C:\Program Files (x86)\Microchip\MPLABX\v5.15\packs\Microchip\ATmega_DFP\2.0.2\gcc\dev\atmega4808"  -x c -c -D__$(MP_PROCESSOR_OPTION)__  -funsigned-char -funsigned-bitfields -Os -ffunction-sections -fdata-sections -fpack-struct -fshort-enums -Wall -MD -MP -MF "${OBJECTDIR}/mcc_generated_files/application_manager.o.d" -MT "${OBJECTDIR}/mcc_generated_files/application_manager.o.d" -MT ${OBJECTDIR}/mcc_generated_files/application_manager.o  -o ${OBJECTDIR}/mcc_generated_files/application_manager.o mcc_generated_files/application_manager.c  -DXPRJ_default=$(CND_CONF)  $(COMPARISON_BUILD) 
	
${OBJECTDIR}/mcc_generated_files/driver_isr.o: mcc_generated_files/driver_isr.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/mcc_generated_files" 
	@${RM} ${OBJECTDIR}/mcc_generated_files/driver_isr.o.d 
	@${RM} ${OBJECTDIR}/mcc_generated_files/driver_isr.o 
	 ${MP_CC}  $(MP_EXTRA_CC_PRE) -mmcu=atmega4808  -I "C:\Program Files (x86)\Microchip\MPLABX\v5.15\packs\Microchip\ATmega_DFP\2.0.2\include"  -B "C:\Program Files (x86)\Microchip\MPLABX\v5.15\packs\Microchip\ATmega_DFP\2.0.2\gcc\dev\atmega4808"  -x c -c -D__$(MP_PROCESSOR_OPTION)__  -funsigned-char -funsigned-bitfields -Os -ffunction-sections -fdata-sections -fpack-struct -fshort-enums -Wall -MD -MP -MF "${OBJECTDIR}/mcc_generated_files/driver_isr.o.d" -MT "${OBJECTDIR}/mcc_generated_files/driver_isr.o.d" -MT ${OBJECTDIR}/mcc_generated_files/driver_isr.o  -o ${OBJECTDIR}/mcc_generated_files/driver_isr.o mcc_generated_files/driver_isr.c  -DXPRJ_default=$(CND_CONF)  $(COMPARISON_BUILD) 
	
${OBJECTDIR}/mcc_generated_files/sensors_handling.o: mcc_generated_files/sensors_handling.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/mcc_generated_files" 
	@${RM} ${OBJECTDIR}/mcc_generated_files/sensors_handling.o.d 
	@${RM} ${OBJECTDIR}/mcc_generated_files/sensors_handling.o 
	 ${MP_CC}  $(MP_EXTRA_CC_PRE) -mmcu=atmega4808  -I "C:\Program Files (x86)\Microchip\MPLABX\v5.15\packs\Microchip\ATmega_DFP\2.0.2\include"  -B "C:\Program Files (x86)\Microchip\MPLABX\v5.15\packs\Microchip\ATmega_DFP\2.0.2\gcc\dev\atmega4808"  -x c -c -D__$(MP_PROCESSOR_OPTION)__  -funsigned-char -funsigned-bitfields -Os -ffunction-sections -fdata-sections -fpack-struct -fshort-enums -Wall -MD -MP -MF "${OBJECTDIR}/mcc_generated_files/sensors_handling.o.d" -MT "${OBJECTDIR}/mcc_generated_files/sensors_handling.o.d" -MT ${OBJECTDIR}/mcc_generated_files/sensors_handling.o  -o ${OBJECTDIR}/mcc_generated_files/sensors_handling.o mcc_generated_files/sensors_handling.c  -DXPRJ_default=$(CND_CONF)  $(COMPARISON_BUILD) 
	
${OBJECTDIR}/mcc_generated_files/device_config.o: mcc_generated_files/device_config.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/mcc_generated_files" 
	@${RM} ${OBJECTDIR}/mcc_generated_files/device_config.o.d 
	@${RM} ${OBJECTDIR}/mcc_generated_files/device_config.o 
	 ${MP_CC}  $(MP_EXTRA_CC_PRE) -mmcu=atmega4808  -I "C:\Program Files (x86)\Microchip\MPLABX\v5.15\packs\Microchip\ATmega_DFP\2.0.2\include"  -B "C:\Program Files (x86)\Microchip\MPLABX\v5.15\packs\Microchip\ATmega_DFP\2.0.2\gcc\dev\atmega4808"  -x c -c -D__$(MP_PROCESSOR_OPTION)__  -funsigned-char -funsigned-bitfields -Os -ffunction-sections -fdata-sections -fpack-struct -fshort-enums -Wall -MD -MP -MF "${OBJECTDIR}/mcc_generated_files/device_config.o.d" -MT "${OBJECTDIR}/mcc_generated_files/device_config.o.d" -MT ${OBJECTDIR}/mcc_generated_files/device_config.o  -o ${OBJECTDIR}/mcc_generated_files/device_config.o mcc_generated_files/device_config.c  -DXPRJ_default=$(CND_CONF)  $(COMPARISON_BUILD) 
	
${OBJECTDIR}/mcc_generated_files/led.o: mcc_generated_files/led.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/mcc_generated_files" 
	@${RM} ${OBJECTDIR}/mcc_generated_files/led.o.d 
	@${RM} ${OBJECTDIR}/mcc_generated_files/led.o 
	 ${MP_CC}  $(MP_EXTRA_CC_PRE) -mmcu=atmega4808  -I "C:\Program Files (x86)\Microchip\MPLABX\v5.15\packs\Microchip\ATmega_DFP\2.0.2\include"  -B "C:\Program Files (x86)\Microchip\MPLABX\v5.15\packs\Microchip\ATmega_DFP\2.0.2\gcc\dev\atmega4808"  -x c -c -D__$(MP_PROCESSOR_OPTION)__  -funsigned-char -funsigned-bitfields -Os -ffunction-sections -fdata-sections -fpack-struct -fshort-enums -Wall -MD -MP -MF "${OBJECTDIR}/mcc_generated_files/led.o.d" -MT "${OBJECTDIR}/mcc_generated_files/led.o.d" -MT ${OBJECTDIR}/mcc_generated_files/led.o  -o ${OBJECTDIR}/mcc_generated_files/led.o mcc_generated_files/led.c  -DXPRJ_default=$(CND_CONF)  $(COMPARISON_BUILD) 
	
${OBJECTDIR}/mcc_generated_files/debug_print.o: mcc_generated_files/debug_print.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/mcc_generated_files" 
	@${RM} ${OBJECTDIR}/mcc_generated_files/debug_print.o.d 
	@${RM} ${OBJECTDIR}/mcc_generated_files/debug_print.o 
	 ${MP_CC}  $(MP_EXTRA_CC_PRE) -mmcu=atmega4808  -I "C:\Program Files (x86)\Microchip\MPLABX\v5.15\packs\Microchip\ATmega_DFP\2.0.2\include"  -B "C:\Program Files (x86)\Microchip\MPLABX\v5.15\packs\Microchip\ATmega_DFP\2.0.2\gcc\dev\atmega4808"  -x c -c -D__$(MP_PROCESSOR_OPTION)__  -funsigned-char -funsigned-bitfields -Os -ffunction-sections -fdata-sections -fpack-struct -fshort-enums -Wall -MD -MP -MF "${OBJECTDIR}/mcc_generated_files/debug_print.o.d" -MT "${OBJECTDIR}/mcc_generated_files/debug_print.o.d" -MT ${OBJECTDIR}/mcc_generated_files/debug_print.o  -o ${OBJECTDIR}/mcc_generated_files/debug_print.o mcc_generated_files/debug_print.c  -DXPRJ_default=$(CND_CONF)  $(COMPARISON_BUILD) 
	
${OBJECTDIR}/main.o: main.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}" 
	@${RM} ${OBJECTDIR}/main.o.d 
	@${RM} ${OBJECTDIR}/main.o 
	 ${MP_CC}  $(MP_EXTRA_CC_PRE) -mmcu=atmega4808  -I "C:\Program Files (x86)\Microchip\MPLABX\v5.15\packs\Microchip\ATmega_DFP\2.0.2\include"  -B "C:\Program Files (x86)\Microchip\MPLABX\v5.15\packs\Microchip\ATmega_DFP\2.0.2\gcc\dev\atmega4808"  -x c -c -D__$(MP_PROCESSOR_OPTION)__  -funsigned-char -funsigned-bitfields -Os -ffunction-sections -fdata-sections -fpack-struct -fshort-enums -Wall -MD -MP -MF "${OBJECTDIR}/main.o.d" -MT "${OBJECTDIR}/main.o.d" -MT ${OBJECTDIR}/main.o  -o ${OBJECTDIR}/main.o main.c  -DXPRJ_default=$(CND_CONF)  $(COMPARISON_BUILD) 
	
endif

# ------------------------------------------------------------------------------------
# Rules for buildStep: compileCPP
ifeq ($(TYPE_IMAGE), DEBUG_RUN)
else
endif

# ------------------------------------------------------------------------------------
# Rules for buildStep: link
ifeq ($(TYPE_IMAGE), DEBUG_RUN)
dist/${CND_CONF}/${IMAGE_TYPE}/avr-iot-workshop.X.${IMAGE_TYPE}.${OUTPUT_SUFFIX}: ${OBJECTFILES}  nbproject/Makefile-${CND_CONF}.mk    
	@${MKDIR} dist/${CND_CONF}/${IMAGE_TYPE} 
	${MP_CC} $(MP_EXTRA_LD_PRE) -mmcu=atmega4808  -B "C:\Program Files (x86)\Microchip\MPLABX\v5.15\packs\Microchip\ATmega_DFP\2.0.2\gcc\dev\atmega4808"   -gdwarf-2 -D__$(MP_PROCESSOR_OPTION)__  -Wl,-Map="dist\${CND_CONF}\${IMAGE_TYPE}\avr-iot-workshop.X.${IMAGE_TYPE}.map"    -o dist/${CND_CONF}/${IMAGE_TYPE}/avr-iot-workshop.X.${IMAGE_TYPE}.${OUTPUT_SUFFIX} ${OBJECTFILES_QUOTED_IF_SPACED}      -DXPRJ_default=$(CND_CONF)  $(COMPARISON_BUILD)  -Wl,--defsym=__MPLAB_BUILD=1$(MP_EXTRA_LD_POST)$(MP_LINKER_FILE_OPTION),--defsym=__ICD2RAM=1,--defsym=__MPLAB_DEBUG=1,--defsym=__DEBUG=1 -Wl,--gc-sections -Wl,--start-group  -Wl,-lm -Wl,--end-group 
	
	
	
	
	
	
else
dist/${CND_CONF}/${IMAGE_TYPE}/avr-iot-workshop.X.${IMAGE_TYPE}.${OUTPUT_SUFFIX}: ${OBJECTFILES}  nbproject/Makefile-${CND_CONF}.mk   
	@${MKDIR} dist/${CND_CONF}/${IMAGE_TYPE} 
	${MP_CC} $(MP_EXTRA_LD_PRE) -mmcu=atmega4808  -B "C:\Program Files (x86)\Microchip\MPLABX\v5.15\packs\Microchip\ATmega_DFP\2.0.2\gcc\dev\atmega4808"  -D__$(MP_PROCESSOR_OPTION)__  -Wl,-Map="dist\${CND_CONF}\${IMAGE_TYPE}\avr-iot-workshop.X.${IMAGE_TYPE}.map"    -o dist/${CND_CONF}/${IMAGE_TYPE}/avr-iot-workshop.X.${IMAGE_TYPE}.${DEBUGGABLE_SUFFIX} ${OBJECTFILES_QUOTED_IF_SPACED}      -DXPRJ_default=$(CND_CONF)  $(COMPARISON_BUILD)  -Wl,--defsym=__MPLAB_BUILD=1$(MP_EXTRA_LD_POST)$(MP_LINKER_FILE_OPTION) -Wl,--gc-sections -Wl,--start-group  -Wl,-lm -Wl,--end-group 
	${MP_CC_DIR}\\avr-objcopy -O ihex "dist/${CND_CONF}/${IMAGE_TYPE}/avr-iot-workshop.X.${IMAGE_TYPE}.${DEBUGGABLE_SUFFIX}" "dist/${CND_CONF}/${IMAGE_TYPE}/avr-iot-workshop.X.${IMAGE_TYPE}.hex"
	
	
	
	
	
endif


# Subprojects
.build-subprojects:


# Subprojects
.clean-subprojects:

# Clean Targets
.clean-conf: ${CLEAN_SUBPROJECTS}
	${RM} -r build/default
	${RM} -r dist/default

# Enable dependency checking
.dep.inc: .depcheck-impl

DEPFILES=$(shell mplabwildcard ${POSSIBLE_DEPFILES})
ifneq (${DEPFILES},)
include ${DEPFILES}
endif
