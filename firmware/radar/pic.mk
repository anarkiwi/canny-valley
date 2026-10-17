# XC16 image for the QM4004 controller (U4, PIC24FJ256GB106); included from
# firmware/Makefile after core.mk.
XC16 ?= /opt/xc16/bin
RADAR_MCU = 24FJ256GB106
PIC_DIR = $(RADAR_DIR)/pic
PIC_SRC = $(wildcard $(PIC_DIR)/*.c)
PIC_HDR = $(wildcard $(PIC_DIR)/*.h)
TUSB_SRC = $(addprefix $(TUSB_DIR)/,tusb.c common/tusb_fifo.c device/usbd.c \
	class/usbtmc/usbtmc_device.c portable/microchip/pic/dcd_pic.c)
RADAR_OUT = $(BUILD)/radar
RADAR_OBJ = $(patsubst $(RADAR_DIR)/%.c,$(RADAR_OUT)/%.o,$(PIC_SRC) $(RADAR_CORE_SRC)) \
	$(patsubst $(SCPI_DIR)/src/%.c,$(RADAR_OUT)/scpi/%.o,$(SCPI_SRC)) \
	$(patsubst $(TUSB_DIR)/%.c,$(RADAR_OUT)/tusb/%.o,$(TUSB_SRC))
RADAR_FORMAT += $(PIC_SRC) $(PIC_HDR)

PIC_OPT = -O1
PIC_CFLAGS = -mcpu=$(RADAR_MCU) -std=gnu99 $(PIC_OPT) -fno-short-double \
	-ffunction-sections -fdata-sections -MMD -MP -DFCY=16000000UL \
	'-D__has_attribute(x)=0' \
	$(RADAR_DEFS) $(RADAR_INC) -I$(PIC_DIR) -I$(TUSB_DIR)
PIC_LDFLAGS = -mcpu=$(RADAR_MCU) -Wl,--script=p$(RADAR_MCU).gld,--heap=0 \
	-Wl,--gc-sections,--report-mem,-Map=$(RADAR_OUT)/radar.map

# -O2 (available in the free edition) inlines the scheduler into the ISR.
$(RADAR_OUT)/pic/hal.o: PIC_OPT = -O2

$(RADAR_OUT)/pic/%.o: $(PIC_DIR)/%.c
	@mkdir -p $(dir $@)
	$(XC16)/xc16-gcc $(PIC_CFLAGS) -Wall -Wextra -Werror -c -o $@ $<

$(RADAR_OUT)/core/%.o: $(RADAR_DIR)/core/%.c
	@mkdir -p $(dir $@)
	$(XC16)/xc16-gcc $(PIC_CFLAGS) -Wall -Wextra -c -o $@ $<

$(RADAR_OUT)/scpi/%.o: $(SCPI_DIR)/src/%.c
	@mkdir -p $(dir $@)
	$(XC16)/xc16-gcc $(PIC_CFLAGS) -w -c -o $@ $<

$(RADAR_OUT)/tusb/%.o: $(TUSB_DIR)/%.c
	@mkdir -p $(dir $@)
	$(XC16)/xc16-gcc $(PIC_CFLAGS) -DNDEBUG -w -c -o $@ $<

$(RADAR_OUT)/radar.elf: $(RADAR_OBJ)
	$(XC16)/xc16-gcc $(PIC_LDFLAGS) -o $@ $^

$(RADAR_OUT)/radar.hex: $(RADAR_OUT)/radar.elf
	$(XC16)/xc16-bin2hex $<

$(RADAR_OUT)/isr.lst: $(RADAR_OUT)/radar.elf
	$(XC16)/xc16-objdump -d $< | \
		awk '/^[0-9a-f]+ <[^.]/{p=/<__INT1Interrupt>:/} p' > $@
	@test -s $@

radar-compile: $(RADAR_OUT)/radar.hex $(RADAR_OUT)/isr.lst

-include $(RADAR_OBJ:.o=.d)

.PHONY: radar-compile
