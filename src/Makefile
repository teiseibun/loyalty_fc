EXECUTABLE=loyalty_fc.elf

CC=arm-none-eabi-gcc
OBJCOPY=arm-none-eabi-objcopy
GDB=arm-none-eabi-gdb

ARCH=CM4F

CFLAGS=-g -O2 -mlittle-endian -mthumb
CFLAGS+=-mcpu=cortex-m4
CFLAGS+=-mfpu=fpv4-sp-d16 -mfloat-abi=hard
CFLAGS+=--specs=nano.specs --specs=nosys.specs -fno-builtin-printf
CFLAGS+=-Wall
CFLAGS+=-fno-strict-aliasing
CFLAGS+=-D USE_STDPERIPH_DRIVER \
	-D STM32F40XX \
	-D __FPU_PRESENT=1 \
	-D __FPU_USED=1 \
	-D ARM_MATH_CM4
CFLAGS+=-Wl,-T,stm32_flash.ld

LDLIBS+=-Wl,--start-group -lm -Wl,--end-group

ST_LIB=./lib/STM32F4xx_StdPeriph_Driver
COMMON=./common
DRIVER=./driver
CORE=./core

ARM_DSP=./lib/CMSIS/DSP_Lib

SRC=

SRC+=$(ARM_DSP)/Source/FastMathFunctions/arm_cos_f32.c \
	$(ARM_DSP)/Source/FastMathFunctions/arm_sin_f32.c \
	$(ARM_DSP)/Source/MatrixFunctions/arm_mat_init_f32.c \
	$(ARM_DSP)/Source/MatrixFunctions/arm_mat_scale_f32.c \
	$(ARM_DSP)/Source/MatrixFunctions/arm_mat_add_f32.c \
	$(ARM_DSP)/Source/MatrixFunctions/arm_mat_sub_f32.c \
	$(ARM_DSP)/Source/MatrixFunctions/arm_mat_mult_f32.c \
	$(ARM_DSP)/Source/MatrixFunctions/arm_mat_trans_f32.c \
	$(ARM_DSP)/Source/MatrixFunctions/arm_mat_inverse_f32.c

SRC+=./lib/CMSIS/system_stm32f4xx.c

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

SRC+=$(COMMON)/stdlib.c \
	$(COMMON)/string.c \
	$(COMMON)/delay.c

SRC+=$(DRIVER)/periph/uart.c \
	$(DRIVER)/periph/i2c.c \
	$(DRIVER)/periph/spi.c \
	$(DRIVER)/periph/led.c \
	$(DRIVER)/periph/timer.c \
	$(DRIVER)/device/mpu6050.c

SRC+=$(CORE)/main.c \
	$(CORE)/link.c \
	$(CORE)/ahrs.c \
	$(CORE)/lpf.c \
	$(CORE)/matrix.c \
	$(CORE)/vector.c

CFLAGS+=-I./
CFLAGS+=-I./lib/CMSIS/Include
CFLAGS+=-I./lib/CMSIS/ST/STM32F4xx/Include
CFLAGS+=-I$(ST_LIB)/inc
CFLAGS+=-I$(ST_LIB)/inc
CFLAGS+=-I$(COMMON)
CFLAGS+=-I$(DRIVER)/periph/
CFLAGS+=-I$(DRIVER)/device/
CFLAGS+=-I$(CORE)

OBJS=$(SRC:.c=.o)

STARTUP=./startup_stm32f4xx.s
STARTUP_OBJ = startup_stm32f4xx.o

all:$(EXECUTABLE)

$(EXECUTABLE): $(STARTUP_OBJ) $(OBJS)
	@echo "LD" $@
	@$(CC) $(CFLAGS) $(OBJS) $(STARTUP_OBJ) $(LDLIBS) -o $@

%.o: %.s 
	@echo "CC" $@
	@$(CC) $(CFLAGS) $^ $(LDLIBS) -c $(STARTUP)

%.o: %.c
	@echo "CC" $@
	@$(CC) $(CFLAGS) -c $< $(LDLIBS) -o $@

clean:
	rm -rf $(EXECUTABLE)
	rm -rf $(OBJS)

flash:
	openocd -f interface/stlink-v2.cfg \
	-f target/stm32f4x_stlink.cfg \
	-c "init" \
	-c "reset init" \
	-c "halt" \
	-c "flash write_image erase $(EXECUTABLE)" \
	-c "verify_image $(EXECUTABLE)" \
	-c "reset run" -c shutdown

openocd:
	openocd -s /opt/openocd/share/openocd/scripts/ -f ./debug/openocd.cfg

gdbauto:
	cgdb -d $(GDB) -x ./debug/openocd_gdb.gdb

.PHONY:all clean flash
