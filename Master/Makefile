mmcu = atmega328p
gcc = avr-gcc -mmcu=$(mmcu)

all: main.hex

main.o: main.c
	$(gcc) -Os -c -o main.o main.c

main.elf: main.o
	$(gcc) -o main.elf main.o

main.hex: main.elf
	avr-objcopy -O ihex main.elf main.hex

clean: 
	rm -rf *.o *.hex *.elf 

flash: main.hex
	avrdude -c usbasp -p m328p -U flash:w:main.hex
