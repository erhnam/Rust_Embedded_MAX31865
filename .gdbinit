target extended-remote localhost:3333

# print demangled symbols
set print asm-demangle on

# detect unhandled exceptions, hard faults and panics
break DefaultHandler
break HardFault
break rust_begin_unwind

monitor arm semihosting enable
monitor reset init
monitor reset halt

load

# start the process but immediately halt the processor
stepi
