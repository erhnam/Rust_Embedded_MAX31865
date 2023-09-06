# Rust_Embedded_MAX31865
Board STM32F401RET6 reading temperature using sensor MAX31865 with SPI1 and W5500 Ethernet controller in SPI2

## Connection with 3-Wired

![image1](https://github.com/erhnam/Rust_Embedded_MAX31865/assets/9365733/8b2555d1-f79a-41db-8998-8e51537ffeee)

![image0](https://github.com/erhnam/Rust_Embedded_MAX31865/assets/9365733/893c91e8-5ed5-46af-8e08-c2008489cc5e)

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

![imagen](https://github.com/erhnam/Rust_Embedded_MAX31865/assets/9365733/81256931-2165-41ef-8f13-fc184c126a57)

![imagen](https://github.com/erhnam/Rust_Embedded_MAX31865/assets/9365733/4f239804-cb13-417a-acf7-1776ed5b1f15)
