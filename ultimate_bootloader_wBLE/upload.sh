sudo openocd -f interface/stlink.cfg -f target/nrf52.cfg -c init -c "reset halt" -c "flash write_image erase build/uboot_ble.hex" -c "reset" -c exit

