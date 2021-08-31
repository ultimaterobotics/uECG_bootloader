## uECG device bootloader with BLE firmware update capability

This bootloader version by default starts in BLE mode and allows to update uECG firmware using an app. It can also be switched into base station mode for firmware uploading using nodejs-based PC tool (requires base station).

Bootloader relies on BLE functionality provided by https://github.com/ultimaterobotics/urf_lib , by default it's expected to be in the same level folder (so that you have folder urf_lib next to folder ultimate_bootloader_wBLE). It is not heavily linked to uECG PCB design - you can easily change button/leds pin numbers in its code, remove PCB version detection part in the beginning of main(), and it will work with any other device.

Other project parts can be found here: https://github.com/ultimaterobotics/uECG
