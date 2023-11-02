# hidbridge
## USB dongle for a Bluetooth keyboard

- Bluetooth keyboards require a GATT stack running on a PC and most BIOS don't include that
- USB dongle that looks like a keyboard (and most BIOS do support USB keyboards)
- Pairs and connects automatically to your Bluetooth keyboard
- Forwards "Human Interface Device" (HID) messages in both directions

## Build instructions

0. Install west and nrfprog if not already done so:
  - https://pypi.org/project/west/
  - https://www.nordicsemi.com/Products/Development-tools/nRF-Command-Line-Tools
1. Create an empty directory <br>`mkdir hidbridge`
2. Enter the directory <br>`cd hidbridge`
3. Initialise **west** project <br>`west init -m https://github.com/gadgetproject/hidbridge`
4. Fetch subordinate repos <br>`west update`
5. Build the software <br>`west build -b nrf52840dk_nrf52840 gp/soft`
6. Flash the software <br>`west flash`
