# esp32 vl53l0x distance sensor to Bluetooth Low Energy notifications

This is a slightly modified version of the `embassy_ble` from esp-wifi example.
Tested on the QTPY-ESP32C3 board.
It connects over I2C to a VL53L0X distance sensor and reads the input every 500ms.
Depending on the distance it sends notifications every different timings.
It also lights the NeoPixel LED on the QTPY board depending on distance.

The board has a LiPoly BFF connected to it and the A2 analog PIN is used to read the battery.
On the QTPY ESP32C3 borad this maps it to the GPIO 1 PIN.

The device advertises as `SEB-PATCH-C3`.
The messages sent ae 4 bytes containitng two 16 bit values:
- 0x00: Distance in mm Big Endian.
- 0x02: Battery level Big Endian.

These values are to be assembled back by the client and then presented to the user.

## Building

For this project, I compile in a machine that has the NFS mounted to the ESP-Wifi repository and use a dfiferent machine for flashing.

The build command needs to be ported to this specific directory.
```bash
$ cargo watch -x 'besp32c3 --example embassy_ble --release --features "async,ble,phy-enable-usb"'
```

## Flashing
```bash
$ espflash flash --monitor target/riscv32imc-unknown-none-elf/release/examples/embassy_ble
```

## Status
Currently only works on the esp-wifi repository.
To replicate, copy the `src/main.rs` to `esp-wifi/examples/embassy_ble.rs` and build it there.

## TODO:
- Read temperature and send it as well.
- Send a message when the battery is low.
- Send the version information to the client on request.
- Allow the client to set the frequency of the notifications.
- Allow the client to set the color of the LED.
- Allow the client to set the brightness of the LED.
- Allow the client to set the timeout of the LED.
- Make it work standalone.
- Make the battery module configurable.
- Allow turning off the LED to save power.
- Allow turning off the VL53L0X over the I2C to save power.
