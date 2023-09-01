# Rust_Embedded_MAX31865
Board STM32F401RET6 reading temperature using sensor MAX31865 with SPI1

## Install:
```sh
sudo apt-get install gdb-multiarch openocd
```

## Execute:
In a linux terminal execute:
```sh
openocd -f interface/stlink-v2-1.cfg -f target/stm32f4x.cfg
```

In a other linux terminal execute:
cargo update
cargo build
cargo run

