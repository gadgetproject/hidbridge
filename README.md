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

## Test Platform

1. Using a second Nordin board (I have PCA10040) <br>`west build -b nrf52840dk_nrf52840 nrf/samples/bluetooth/peripheral_hids_keyboard`
2. Flash the software <br>`west flash`
3. Button 4 starts advertising; LED 1 flashes.
4. On **hidbridge** board, `downstream scan` and `downstream connect`.
5. Connects, displays passkey and bonds; LED 1 lit.
6. Button 1 accepts bond; LED 1 off, LED 2 lit.
7. Button 1 sends one keystroke in sequence `hello\\n`
