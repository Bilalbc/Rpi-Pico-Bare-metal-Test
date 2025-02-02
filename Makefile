ARMGNU = arm-none-eabi
CC = $(ARMGNU)-gcc
MACH = cortex-m0plus
NAME = Blinker-Systick
AFLAGS = --warn --fatal-warnings -mcpu=$(MACH) -g
CFLAGS = -c -mcpu=$(MACH) -mthumb -std=gnu11 -Wall -O0 
LFLAGS = -nostdlib -T memmap.ld -Wl,-Map=$(BUILD_DIR)/$(NAME).map
LFLAGSB2 = -nostdlib -T memmap_boot2.ld
BUILD_DIR = build

all : $(BUILD_DIR)/$(NAME).uf2

$(BUILD_DIR)/$(NAME).o : $(NAME).c
	$(CC) $(CFLAGS) -o $(BUILD_DIR)/$(NAME).o $(NAME).c

$(BUILD_DIR)/boot2.bin : boot2.s memmap_boot2.ld
	$(ARMGNU)-as $(AFLAGS) boot2.s -o $(BUILD_DIR)/boot2.o
	$(ARMGNU)-ld $(LFLAGSB2) $(BUILD_DIR)/boot2.o -o $(BUILD_DIR)/boot2.elf
	$(ARMGNU)-objcopy -O binary $(BUILD_DIR)/boot2.elf $(BUILD_DIR)/boot2.bin
	
$(BUILD_DIR)/boot2_padded.o : $(BUILD_DIR)/boot2.bin
	python3 ./pad_checksum.py -p 256 -s 0xFFFFFFFF $(BUILD_DIR)/boot2.bin $(BUILD_DIR)/boot2_padded.S
	$(ARMGNU)-as $(AFLAGS) $(BUILD_DIR)/boot2_padded.S -o $(BUILD_DIR)/boot2_padded.o
		
$(BUILD_DIR)/$(NAME).elf : $(BUILD_DIR)/boot2_padded.o $(BUILD_DIR)/$(NAME).o 
	$(CC) $(LFLAGS) -o $(BUILD_DIR)/$(NAME).elf $(BUILD_DIR)/boot2_padded.o $(BUILD_DIR)/$(NAME).o 
	
clean:
	del /s /q $(BUILD_DIR)\*

$(BUILD_DIR)/$(NAME).uf2 : $(BUILD_DIR)/$(NAME).elf
	elf2uf2 $(BUILD_DIR)/$(NAME).elf $(BUILD_DIR)/$(NAME).uf2