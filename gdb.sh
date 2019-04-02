#!/bin/bash
exec arm-none-eabi-gdb -iex 'add-auto-load-safe-path .' $1
