#!/bin/sh

for GDB in arm-none-eabi-gdb gdb-multiarch $(uname -m | grep -q ^arm && echo gdb)
do
command -v $GDB >/dev/null && break
done

exec "$GDB" -iex 'add-auto-load-safe-path .' "$1"
