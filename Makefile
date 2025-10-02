## Makefile

# Prefix for older riscv gcc is  risv-none-embed
# Prefix for newer riscv gcc is  risv-none-elf
# TOOLCHAIN_PREFIX := riscv-none-embed
TOOLCHAIN_PREFIX := ../../MRS_Toolchain_Linux_x64_V210/RISC-V_Embedded_GCC12/bin/riscv-wch-elf


APP_C_SRCS += \
  ./src/main.c


PICC_M1_COMMON_C_SRCS := \
  ./NFCA_LIB/ISO14443-3A.c \
  ./PICC_COMMON/wch_nfca_picc_bsp.c \
  ./M1_COMMON/wch_nfca_picc_m1.c


SDK_STDPERIPHDRIVER_C_SRCS += \
  ./sdk/StdPeriphDriver/CH58x_adc.c \
  ./sdk/StdPeriphDriver/CH58x_clk.c \
  ./sdk/StdPeriphDriver/CH58x_flash.c \
  ./sdk/StdPeriphDriver/CH58x_gpio.c \
  ./sdk/StdPeriphDriver/CH58x_i2c.c \
  ./sdk/StdPeriphDriver/CH58x_lcd.c \
  ./sdk/StdPeriphDriver/CH58x_pwm.c \
  ./sdk/StdPeriphDriver/CH58x_pwr.c \
  ./sdk/StdPeriphDriver/CH58x_spi0.c \
  ./sdk/StdPeriphDriver/CH58x_sys.c \
  ./sdk/StdPeriphDriver/CH58x_timer0.c \
  ./sdk/StdPeriphDriver/CH58x_timer1.c \
  ./sdk/StdPeriphDriver/CH58x_timer2.c \
  ./sdk/StdPeriphDriver/CH58x_timer3.c \
  ./sdk/StdPeriphDriver/CH58x_uart0.c \
  ./sdk/StdPeriphDriver/CH58x_uart1.c \
  ./sdk/StdPeriphDriver/CH58x_uart2.c \
  ./sdk/StdPeriphDriver/CH58x_uart3.c \
  ./sdk/StdPeriphDriver/CH58x_usbdev.c \
  ./sdk/StdPeriphDriver/CH58x_usbhostBase.c \
  ./sdk/StdPeriphDriver/CH58x_usbhostClass.c

SDK_STARTUP_S_UPPER_SRCS += \
  ./sdk/Startup/startup_CH585.S

C_SRCS := \
  $(APP_C_SRCS) \
  $(PICC_M1_COMMON_C_SRCS) \
  $(SDK_STDPERIPHDRIVER_C_SRCS)

S_UPPER_SRCS := \
  $(SDK_STARTUP_S_UPPER_SRCS)

OBJS := \
  $(foreach src,$(C_SRCS),$(subst ./,obj/,$(patsubst %.c,%.o,$(src)))) \
  $(foreach src,$(S_UPPER_SRCS),$(subst ./,obj/,$(patsubst %.S,%.o,$(src))))

MAKEFILE_DEPS := \
  $(foreach obj,$(OBJS),$(patsubst %.o,%.d,$(obj)))


STDPERIPHDRIVER_LIBS := -L"./sdk/StdPeriphDriver" -lISP585
NFC_LIBS := -L"./NFCA_LIB" -lCH58x_NFCA
LIBS := $(STDPERIPHDRIVER_LIBS) $(NFC_LIBS)

SECONDARY_FLASH := main.hex
SECONDARY_LIST := main.lst
SECONDARY_SIZE := main.siz
SECONDARY_BIN := main.bin

# ARCH := rv32imac_zicsr_zifencei
ARCH := rv32imc_zba_zbb_zbc_zbs_xw

CFLAGS_COMMON := \
  -march=$(ARCH) \
  -mabi=ilp32 \
  -mcmodel=medany \
  -msmall-data-limit=8 \
  -mno-save-restore \
  -Os \
  -fmessage-length=0 \
  -fsigned-char \
  -ffunction-sections \
  -fdata-sections \
  -fno-common \
  --param=highcode-gen-section-name=1
  #-g

.PHONY: all
all: main.elf secondary-outputs

.PHONY: clean
clean:
	-rm $(OBJS)
	-rm $(MAKEFILE_DEPS)
	-rm $(SECONDARY_FLASH)
	-rm $(SECONDARY_LIST)
	-rm $(SECONDARY_BIN)
	-rm main.elf
	-rm main.map
	-rm -r ./obj

.PHONY: secondary-outputs
secondary-outputs: $(SECONDARY_FLASH) $(SECONDARY_LIST) $(SECONDARY_SIZE) $(SECONDARY_BIN)

main.elf: $(OBJS)
	${TOOLCHAIN_PREFIX}-gcc \
	    $(CFLAGS_COMMON) \
	    -T "sdk/Ld/Link.ld" \
	    -nostartfiles \
	    -Xlinker \
	    --gc-sections \
	    -Xlinker \
	    --print-memory-usage \
	    -Wl,-Map,"main.map" \
	    -Lobj \
	    --specs=nano.specs \
	    --specs=nosys.specs \
	    -o "main.elf" \
	    $(OBJS) \
	    $(LIBS)

%.hex: %.elf
	@ ${TOOLCHAIN_PREFIX}-objcopy -O ihex "$<"  "$@"

%.bin: %.elf
	$(TOOLCHAIN_PREFIX)-objcopy -O binary $< "$@"

%.lst: %.elf
	@ ${TOOLCHAIN_PREFIX}-objdump \
	    --source \
	    --all-headers \
	    --demangle \
	    --line-numbers \
	    --wide "$<" > "$@"

%.siz: %.elf
	@ ${TOOLCHAIN_PREFIX}-size --format=berkeley "$<"

obj/%.o: ./%.c
	@ mkdir --parents $(dir $@)
	@ ${TOOLCHAIN_PREFIX}-gcc \
	    $(CFLAGS_COMMON) \
	    -DDEBUG=0 \
	    -I"src/include" \
	    -I"sdk/StdPeriphDriver/inc" \
	    -I"sdk/RVMSIS" \
	    -I"NFCA_LIB" \
	    -I"PICC_COMMON" \
	    -I"M1_COMMON" \
	    -std=gnu99 \
	    -MMD \
	    -MP \
	    -MF"$(@:%.o=%.d)" \
	    -MT"$(@)" \
	    -c \
	    -o "$@" "$<"

obj/%.o: ./%.S
	@ mkdir --parents $(dir $@)
	@ ${TOOLCHAIN_PREFIX}-gcc \
	    $(CFLAGS_COMMON) \
	    -x assembler \
	    -MMD \
	    -MP \
	    -MF"$(@:%.o=%.d)" \
	    -MT"$(@)" \
	    -c \
	    -o "$@" "$<"

f: clean all flash

flash: 
	chprog main.bin
