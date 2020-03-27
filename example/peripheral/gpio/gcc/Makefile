##############################################################################
BUILD = build
BIN = gpio_gcc
SDK_PATH = /mnt/c/PhyPlusInc/PHY62XX_SDK_1.1.8
CMSIS_PATH = /mnt/c/Users/theom/AppData/Local/Arm/Packs/ARM/CMSIS/5.5.1
##############################################################################
.PHONY: all directory clean size

CC = arm-none-eabi-gcc
OBJCOPY = arm-none-eabi-objcopy
SIZE = arm-none-eabi-size

ifeq ($(OS), Windows_NT)
  MKDIR = gmkdir
else
  MKDIR = mkdir
endif

CFLAGS += -W -Wall --std=gnu99
CFLAGS += -fno-diagnostics-show-caret
CFLAGS += -fdata-sections -ffunction-sections
CFLAGS += -funsigned-char -funsigned-bitfields
CFLAGS += -mcpu=cortex-m0 -mthumb
CFLAGS += -MD -MP -MT $(BUILD)/$(*F).o -MF $(BUILD)/$(@F).d

LDFLAGS += -mcpu=cortex-m0 -mthumb
LDFLAGS += -Wl,--gc-sections
LDFLAGS += -Wl,--script=phy6202.ld
LDFLAGS += -Wl,--just-symbols=rom.sym
LIBS += $(SDK_PATH)/lib/rf.lib

INCLUDES += \
  -I$(SDK_PATH)/components/inc \
  -I$(SDK_PATH)/components/common \
  -I$(SDK_PATH)/components/osal/include \
  -I$(SDK_PATH)/components/ble/include \
  -I$(SDK_PATH)/components/ble/hci \
  -I$(SDK_PATH)/components/ble/host \
  -I$(SDK_PATH)/components/ble/controller/include \
  -I$(SDK_PATH)/components/profiles/DevInfo \
  -I$(SDK_PATH)/components/profiles/GATT \
  -I$(SDK_PATH)/components/profiles/SimpleProfile \
  -I$(SDK_PATH)/components/profiles/Roles \
  -I$(SDK_PATH)/components/profiles/hrs \
  -I$(SDK_PATH)/components/driver/common \
  -I$(SDK_PATH)/components/driver/log \
  -I$(SDK_PATH)/components/driver/gpio \
  -I$(SDK_PATH)/components/driver/pwrmgr \
  -I$(SDK_PATH)/components/driver/uart \
  -I$(SDK_PATH)/components/driver/adc \
  -I$(SDK_PATH)/components/driver/clock \
  -I$(SDK_PATH)/components/driver/spi \
  -I$(SDK_PATH)/components/driver/key \
  -I$(SDK_PATH)/components/driver/i2c \
  -I$(SDK_PATH)/components/driver/watchdog \
  -I$(SDK_PATH)/components/driver/timer \
  -I$(SDK_PATH)/components/driver/kscan \
  -I$(SDK_PATH)/lib \
  -I$(CMSIS_PATH)/Device/ARM/ARMCM0/Include \
  -I$(CMSIS_PATH)/CMSIS/Core/Include \
  -I../Sources

SRCS += \
  $(SDK_PATH)/components/driver/gpio/gpio.c \
  $(SDK_PATH)/components/driver/uart/uart.c \
  $(SDK_PATH)/components/driver/pwrmgr/pwrmgr.c \
  $(SDK_PATH)/components/driver/clock/clock.c \
  $(SDK_PATH)/components/driver/adc/adc.c \
  $(SDK_PATH)/components/driver/common/common.c \
  $(SDK_PATH)/components/driver/key/key.c \
  $(SDK_PATH)/misc/jump_table.c \
  ../Source/main.c \
  ../Source/gpio_demo.c \
  ../Source/gpio_Main.c \
  ../Source/OSAL_gpio.c \
  ./system_ARMCM0.c \
  ./startup_ARMCM0.c
#  $(SDK_PATH)/components/driver/log/my_printf.c is included in rf.lib

DEFINES += \
  -DCFG_CP \
  -DCFG_QFN32 \
  -DCFG_SLEEP_MODE=PWR_MODE_SLEEP \
  -DDEBUG_INFO=1 \
  -DADV_NCONN_CFG=0x01 \
  -DADV_CONN_CFG=0x02 \
  -DSCAN_CFG=0x04 \
  -DINIT_CFG=0x08 \
  -DBROADCASTER_CFG=0x01 \
  -DOBSERVER_CFG=0x02 \
  -DPERIPHERAL_CFG=0x04 \
  -DCENTRAL_CFG=0x08 \
  -DHOST_CONFIG=0x4  

CFLAGS += $(INCLUDES) $(DEFINES)

OBJS = $(addprefix $(BUILD)/, $(notdir %/$(subst .c,.o, $(SRCS))))
# $(info List of files : [${OBJS}])

all: directory $(BUILD)/$(BIN).elf $(BUILD)/$(BIN).hex $(BUILD)/$(BIN).bin size

$(BUILD)/$(BIN).elf: $(OBJS)
	@echo LD $@
	@$(CC) $(LDFLAGS) $(OBJS) $(LIBS) -o $@

$(BUILD)/$(BIN).hex: $(BUILD)/$(BIN).elf
	@echo OBJCOPY $@
	@$(OBJCOPY) -O ihex $^ $@

$(BUILD)/$(BIN).bin: $(BUILD)/$(BIN).elf
	@echo OBJCOPY $@
	@$(OBJCOPY) -O binary $^ $@

%.o:
	@echo CC $@
	@$(CC) $(CFLAGS) $(filter %/$(subst .o,.c,$(notdir $@)), $(SRCS)) -c -o $@

directory:
	@$(MKDIR) -p $(BUILD)

size: $(BUILD)/$(BIN).elf
	@echo size:
	@$(SIZE) -t $^

clean:
	@echo clean
	@-rm -rf $(BUILD)

-include $(wildcard $(BUILD)/*.d)
