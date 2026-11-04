# hw definition file for processing by chibios_hwdef.py
# Arduino UNO Q (ABX00162): lines shared by the bootloader and the firmware for its STM32U585.
# The board's QRB2210 (Linux) is the companion computer; besides SWD, its only link to the
# STM32 is LPUART1 (/dev/ttyHS1 on Linux).

# MCU class and specific type
MCU STM32U5xx STM32U585xx

# 16 MHz crystal
OSCILLATOR_HZ 16000000

# board ID: development value, not yet allocated in Tools/AP_Bootloader/board_types.txt
APJ_BOARD_ID 5999

FLASH_SIZE_KB 2048

# LPUART1 to the QRB2210: bootloader protocol, then MAVLink as SERIAL0. The STM32 has no USB connection
SERIAL_ORDER LPUART1
PG7 LPUART1_TX LPUART1
PG8 LPUART1_RX LPUART1

# RTS to the QRB2210, held asserted so the Linux side can always send
PG6 QRB_RTS OUTPUT LOW

# SWD
PA13 JTMS-SWDIO SWD
PA14 JTCK-SWCLK SWD

# no DMA until ArduPilot has a GPDMA backend
define DMA_RESERVE_SIZE 0
define HAL_UART_NODMA
define AP_HAL_SHARED_DMA_ENABLED 0
