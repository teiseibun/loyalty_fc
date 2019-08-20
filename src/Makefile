EXECUTABLE=loyalty_fc.elf
BIN_IMAGE=loyalty_fc.bin

CC=arm-none-eabi-gcc
OBJCOPY=arm-none-eabi-objcopy
GDB=arm-none-eabi-gdb

CFLAGS=-g -O2 -mlittle-endian -mthumb
CFLAGS+=-mcpu=cortex-m4
CFLAGS+=-mfpu=fpv4-sp-d16 -mfloat-abi=hard
CFLAGS+=--specs=nano.specs --specs=nosys.specs -fno-builtin-printf
CFLAGS+=-Wall
CFLAGS+=-D USE_STDPERIPH_DRIVER \
	-D STM32F40XX \
	-D __FPU_PRESENT=1 \
	-D __FPU_USED=1 \
	-D ARM_MATH_CM4

CFLAGS+=-I./

CFLAGS+=-Wl,-T,stm32_flash.ld

ARCH=CM4F

LDLIBS+=-Wl,--start-group -lm -Wl,--end-group

ST_LIB=./lib/STM32F4xx_StdPeriph_Driver
CFLAGS+=-I./lib/CMSIS/ST/STM32F4xx/Include

CFLAGS+=-I./lib/CMSIS/Include

CFLAGS+=-I$(ST_LIB)/inc

COMMON=./common
DRIVER=./driver
CORE=./core

SRC=./lib/CMSIS/system_stm32f4xx.c

CFLAGS+=-I$(ST_LIB)/inc
SRC+=$(ST_LIB)/src/misc.c \
	$(ST_LIB)/src/stm32f4xx_rcc.c \
	$(ST_LIB)/src/stm32f4xx_dma.c \
	$(ST_LIB)/src/stm32f4xx_flash.c \
	$(ST_LIB)/src/stm32f4xx_gpio.c \
	$(ST_LIB)/src/stm32f4xx_usart.c \
	$(ST_LIB)/src/stm32f4xx_tim.c\
	$(ST_LIB)/src/stm32f4xx_spi.c\
	$(ST_LIB)/src/stm32f4xx_i2c.c \
	$(ST_LIB)/src/stm32f4xx_adc.c

CFLAGS+=-I$(COMMON)
SRC+=$(COMMON)/stdlib.c \
	$(COMMON)/string.c \
	$(COMMON)/delay.c

CFLAGS+=-I$(DRIVER)/periph/
SRC+=$(DRIVER)/periph/uart.c \
	$(DRIVER)/periph/i2c.c \
	$(DRIVER)/periph/spi.c \
	$(DRIVER)/periph/led.c \
	$(DRIVER)/periph/timer.c

CFLAGS+=-I$(DRIVER)/device/
SRC+=$(DRIVER)/device/mpu6050.c

CFLAGS+=-I$(CORE)
SRC+=$(CORE)/main.c \
	$(CORE)/link.c

CFLAGS+=-I./

STARTUP=./startup_stm32f4xx.s

all:$(BIN_IMAGE)

$(BIN_IMAGE):$(EXECUTABLE)
	$(OBJCOPY) -O binary $^ $@

STARTUP_OBJ = startup_stm32f4xx.o

$(STARTUP_OBJ): $(STARTUP) 
	$(CC) $(CFLAGS) $^ $(LDLIBS) -c $(STARTUP)

$(EXECUTABLE):$(SRC) $(STARTUP_OBJ)
	$(CC) $(CFLAGS) $^ $(LDLIBS) -o $@

#Make clean
clean:
	rm -rf $(EXECUTABLE)
	rm -rf $(BIN_IMAGE)
#Make flash
flash:
	st-flash write $(BIN_IMAGE) 0x8000000

#Make openocd
openocd:
	openocd -s /opt/openocd/share/openocd/scripts/ -f ./debug/openocd.cfg

#Make cgdb
gdbauto:
	cgdb -d $(GDB) -x ./debug/openocd_gdb.gdb

.PHONY:all clean flash
