#!/bin/sh
for i in arm-none-eabi-gdb gdb-multiarch; do command -v $i >/dev/null && GDB=$i; done
[ -n "$GDB" ] || echo "Falling back to standard GDB..."
exec "${GDB:-gdb}" -iex 'add-auto-load-safe-path .' "$1"
