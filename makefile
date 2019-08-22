makeCompile: Sigmux.c
	avr-gcc -O3 -mmcu=atmega8 -c Sigmux.c
	avr-gcc -O3 -mmcu=atmega8 -o Sigmux.elf ./Sigmux.o
	avr-objcopy -j .text -j .data -O ihex Sigmux.elf Sigmux.hex

program: Sigmux.hex
	sudo avrdude -c USBasp -p atmega8 -U flash:w:Sigmux.hex:i

fuseProgram: Sigmux.c
	sudo avrdude -c USBasp -p atmega8 -U lfuse:w:0xef:m -U hfuse:w:0xc9:m
