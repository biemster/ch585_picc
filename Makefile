all : flash

TARGET:=ch585_picc
TARGET_MCU:=CH585
TARGET_MCU_PACKAGE:=CH585M

include ../../CH570/ch32fun/ch32fun/ch32fun.mk
LDFLAGS+=-L. -lCH58x_NFCA

flash : cv_flash
clean : cv_clean
