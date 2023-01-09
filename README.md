## ESP32-c3 Fan Temperature Controller

This is a simple repo that uses a AHT10 i2c temperature sensor to control the speed of two 4-pin fans via their PWM control pins. It checks the measured temperature on the AHT10 against a range specified in the code and linearly maps the fans from off to full speed based on where the temperature falls within (or outside) of that range.

### Flashing / Setup Instructions:

The project uses the [espflash](https://github.com/esp-rs/espflash) utility to upload the code to the board over USB. This can be installed easily using the `cargo install espflash` command.

Note that in order to use the `flash --port COM7` argument with espflash, you will need to be using the version 2.x release candidate of espflash, which may not be installed automatically via cargo. If this is the case, you'll need to clone the [espflash repo](https://github.com/esp-rs/espflash) and then `cargo install --path .` inside the cloned repo's `/espflash/` directory to get the latest version. The older version in cargo (1.7.x last time I checked) will work, but if you have more than one serial port you'll need to select it manually every time you upload code.

You will also need to install the `riscv32imc-unknown-none-elf` target for using the `rustup target add riscv32imc-unknown-none-elf` command in order to compile for the board.

From there, `cargo run --release` should result in the code being compiled and uploaded to the serial port that your board is connected to, and remaining connected in monitor mode. If multiple serial ports are found, espflash should ask you to select a port at upload time.
