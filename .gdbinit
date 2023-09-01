file target/thumbv7em-none-eabihf/debug/emb-test
target extended-remote :3333
monitor arm semihosting enable
break main
load
