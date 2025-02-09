/build: folder used to hold all intermittent files during the creation of the .uf2 file

BM_Pico.c: Main program that uses Cortex M0+ Systick module to generate timed interrupts and blink on board LED

BM_Pico.h Header file for c main program

boot2.s: assembly file defining boot stage 2 process for RP2040

elf2uf2.exe: executable used to convert elf file to uf2 during build process

memmap_boo2.ld: Linker script defining the mapping of the boot2 script

memmap.ld: Linker script defining mapping of rest of the program. Taken and modified from RP2040 sdk

pad_checksum.py: python script to add CRC32 checksum and padding to create 256 byte second stage boot loader for flashing from boot2.s. Taken from RP2040 SDK
