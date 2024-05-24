SIMPLELINK_MSP432P4_SDK := $(SIMPLELINK_MSP432P4_SDK)

CC  = arm-none-eabi-gcc # compiler
AS  = arm-none-eabi-gcc # assembler
LNK = arm-none-eabi-gcc # linker

OBJCOPY     = arm-none-eabi-objcopy # copy and translate object files into machine-readable opcodes
NAME	    = topographic-robot
SRC_DIR     = src
LIB_DIR     = ../../lib
LNK_DIR     = ../../lnk
BUILD_DIR   = build
DEBUG_DIR   = $(BUILD_DIR)/debug
RELEASE_DIR = $(BUILD_DIR)/release
OPENOCD     = openocd # open on-chip debugger (debugging)
OPENOCD_DIR = /usr/share/openocd/scripts/board
OPENOCD_CFG = "$(OPENOCD_DIR)/ti_msp432_launchpad.cfg"
QEMU	   = qemu-system-arm

ifeq (, $(shell which arm-none-eabi-gdb))
  GDB = gdb-multiarch
else
  GDB = arm-none-eabi-gdb
endif

CFLAGS_COMMON   = -I"$(SIMPLELINK_MSP432P4_SDK)/source" \
		  -I"$(SIMPLELINK_MSP432P4_SDK)/source/third_party/CMSIS/Include" \
		  -D__MSP432P401R__ \
		  -DDeviceFamily_MSP432P401x \
		  -mcpu=cortex-m4 \
		  -mthumb \
		  -mfloat-abi=hard \
		  -mfpu=fpv4-sp-d16
CFLAGS_DEBUG    = -g -O0 -DDEBUG $(CFLAGS_COMMON)
CFLAGS_RELEASE  = -s -O3 -DNDEBUG $(CLFAGS_COMMON)
ASFLAGS_DEBUG   = $(CFLAGS_COMMON)
ASFLAGS_RELEASE = $(CFLAGS_COMMON)
LFLAGS	        = -T "$(LNK_DIR)/msp432p401r.lds" --specs=nosys.specs \
		  -L"$(SIMPLELINK_MSP432P4_SDK)/source" \
		  -l:ti/devices/msp432p4xx/driverlib/gcc/msp432p4xx_driverlib.a \
		  -static \
		  -Wl,--gc-sections \
		  -lgcc \
		  -lc \
		  -lm \
		  -lnosys \
		  --specs=nano.specs \
		  -mcpu=cortex-m4 \
		  -mthumb \
		  -mfloat-abi=hard \
		  -mfpu=fpv4-sp-d16

C_SOURCES           = $(wildcard $(SRC_DIR)/*.c) $(wildcard $(LIB_DIR)/*.c)
ASM_SOURCES         = $(wildcard $(SRC_DIR)/*.s)
C_OBJECTS_DEBUG     = $(patsubst $(SRC_DIR)/%.c,$(DEBUG_DIR)/%.o,$(wildcard $(SRC_DIR)/*.c)) \
		      $(patsubst $(LIB_DIR)/%.c,$(DEBUG_DIR)/%.o,$(wildcard $(LIB_DIR)/*.c))
C_OBJECTS_RELEASE   = $(patsubst $(SRC_DIR)/%.c,$(RELEASE_DIR)/%.o,$(wildcard $(SRC_DIR)/*.c)) \
		      $(patsubst $(LIB_DIR)/%.c,$(RELEASE_DIR)/%.o,$(wildcard $(LIB_DIR)/*.c))
ASM_OBJECTS_DEBUG   = $(patsubst $(SRC_DIR)/%.s,$(DEBUG_DIR)/%.o,$(ASM_SOURCES))
ASM_OBJECTS_RELEASE = $(patsubst $(SRC_DIR)/%.s,$(RELEASE_DIR)/%.o,$(ASM_SOURCES))
OBJECTS_DEBUG       = $(C_OBJECTS_DEBUG) $(ASM_OBJECTS_DEBUG)
OBJECTS_RELEASE     = $(C_OBJECTS_RELEASE) $(ASM_OBJECTS_RELEASE)

all: release

debug: CFLAGS = $(CFLAGS_DEBUG)
debug: $(DEBUG_DIR)/$(NAME).elf

release: CFLAGS = $(CFLAGS_RELEASE)
release: $(RELEASE_DIR)/$(NAME).elf

$(DEBUG_DIR)/$(NAME).elf: $(OBJECTS_DEBUG)
	@mkdir -p $(DEBUG_DIR)
	@echo "Linking $@"
	$(LNK) $(OBJECTS_DEBUG) $(LFLAGS) -o $@

$(RELEASE_DIR)/$(NAME).elf: $(OBJECTS_RELEASE)
	@mkdir -p $(RELEASE_DIR)
	@echo "Linking $@"
	$(LNK) $(OBJECTS_RELEASE) $(LFLAGS) -o $@

$(DEBUG_DIR)/%.o: $(SRC_DIR)/%.c
	@mkdir -p $(DEBUG_DIR)
	@echo "Compiling $<"
	$(CC) $(CFLAGS_COMMON) -c $< -o $@

$(DEBUG_DIR)/%.o: $(SRC_DIR)/%.s
	@mkdir -p $(DEBUG_DIR)
	@echo "Assembling $<"
	$(AS) $(ASFLAGS_DEBUG) -c $< -o $@

$(DEBUG_DIR)/%.o: $(LIB_DIR)/%.c
	@mkdir -p $(DEBUG_DIR)
	@echo "Compiling $<"
	$(CC) $(CFLAGS_COMMON) -c $< -o $@

$(RELEASE_DIR)/%.o: $(SRC_DIR)/%.c
	@mkdir -p $(RELEASE_DIR)
	@echo "Compiling $<"
	$(CC) $(CFLAGS_COMMON) -c $< -o $@

$(RELEASE_DIR)/%.o: $(SRC_DIR)/%.s
	@mkdir -p $(RELEASE_DIR)
	@echo "Assembling $<"
	$(AS) $(ASFLAGS_RELEASE) -c $< -o $@

$(RELEASE_DIR)/%.o: $(LIB_DIR)/%.c
	@mkdir -p $(RELEASE_DIR)
	@echo "Compiling $<"
	$(CC) $(CFLAGS_COMMON) -c $< -o $@

clean:
	@echo "Cleaning..."
	rm -rf $(BUILD_DIR)

upload: release
	@echo "Uploading..."
	$(OPENOCD) -f $(OPENOCD_CFG) -c "program $(RELEASE_DIR)/$(NAME).elf verify reset; shutdown"

debug-run: debug
	@echo "Starting debugger..."
	@$(OPENOCD) -f $(OPENOCD_CFG) & \
	$(GDB) $(DEBUG_DIR)/$(NAME).elf \
	-ex "target remote localhost:3333" \
	-ex "monitor reset halt"

simulate: debug
	@echo "Simulating..."
	pkill qemu-system-arm || true
	@$(QEMU) -machine mps2-an385 -cpu cortex-m3 -kernel $(DEBUG_DIR)/$(NAME).elf -nographic -s -S &
	@$(GDB) $(DEBUG_DIR)/$(NAME).elf -ex "target remote localhost:1234"
	@pkill qemu-system-arm || true

.PHONY: all debug release clean upload debug-run simulate

