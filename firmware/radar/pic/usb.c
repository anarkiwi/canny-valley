/* USB identity of the stock QM4004 firmware and the USBTMC/USB488 glue
   between TinyUSB and the core. All callbacks run from tud_task(). */
#include <xc.h>

#include "board.h"
#include "device/usbd_pvt.h"
#include "hal.h"
#include "radar.h"
#include "tusb.h"

enum { ITF_TMC, ITF_TOTAL };
enum { STR_MANUFACTURER = 1, STR_PRODUCT, STR_SERIAL };

#define EP_OUT 0x01
#define EP_IN 0x81
#define EP_INT 0x82
#define CONFIG_LEN                                                             \
  (TUD_CONFIG_DESC_LEN + TUD_USBTMC_IF_DESCRIPTOR_LEN +                        \
   TUD_USBTMC_BULK_DESCRIPTORS_LEN + TUD_USBTMC_INT_DESCRIPTOR_LEN)

static const tusb_desc_device_t desc_device = {
    .bLength = sizeof(tusb_desc_device_t),
    .bDescriptorType = TUSB_DESC_DEVICE,
    .bcdUSB = 0x0200,
    .bDeviceClass = TUSB_CLASS_UNSPECIFIED,
    .bMaxPacketSize0 = CFG_TUD_ENDPOINT0_SIZE,
    .idVendor = 0x2012,
    .idProduct = 0x0013,
    .bcdDevice = 0x0100,
    .iManufacturer = STR_MANUFACTURER,
    .iProduct = STR_PRODUCT,
    .iSerialNumber = STR_SERIAL,
    .bNumConfigurations = 1,
};

static const uint8_t desc_config[] = {
    TUD_CONFIG_DESCRIPTOR(1, ITF_TOTAL, 0, CONFIG_LEN,
                          TUSB_DESC_CONFIG_ATT_SELF_POWERED, 100),
    TUD_USBTMC_IF_DESCRIPTOR(ITF_TMC, 3, 0, TUD_USBTMC_PROTOCOL_USB488),
    TUD_USBTMC_BULK_DESCRIPTORS(EP_OUT, EP_IN, 64),
    TUD_USBTMC_INT_DESCRIPTOR(EP_INT, 2, 16),
};

static const usbtmc_response_capabilities_488_t capabilities = {
    .USBTMC_status = USBTMC_STATUS_SUCCESS,
    .bcdUSBTMC = USBTMC_VERSION,
    .bmIntfcCapabilities = {.supportsIndicatorPulse = 1},
    .bcdUSB488 = USBTMC_488_VERSION,
    .bmIntfcCapabilities488 = {.is488_2 = 1},
    .bmDevCapabilities488 = {.SCPI = 1},
};

uint8_t const *tud_descriptor_device_cb(void) {
  return (uint8_t const *)&desc_device;
}

uint8_t const *tud_descriptor_configuration_cb(uint8_t index) {
  (void)index;
  return desc_config;
}

uint16_t const *tud_descriptor_string_cb(uint8_t index, uint16_t langid) {
  static uint16_t d[16];
  static const char *const fixed[] = {"Quonset Micro", "QM4004"};
  const char *s;
  uint8_t n = 0;
  (void)langid;
  if (index == 0) {
    d[++n] = 0x0409;
  } else {
    if (index > STR_SERIAL)
      return NULL;
    s = index == STR_SERIAL ? radar_serial() : fixed[index - 1];
    while (s[n] && n < 15) {
      d[n + 1] = (uint8_t)s[n];
      n++;
    }
  }
  d[0] = (uint16_t)(TUSB_DESC_STRING << 8 | (2u * n + 2u));
  return d;
}

uint32_t tusb_time_millis_api(void) { return hal_ms(); }

void __attribute__((interrupt, auto_psv)) _USB1Interrupt(void) {
  tud_int_handler(0);
}

/* TransferSize of a bulk-IN request the core has not answered yet. */
static uint32_t in_want;
/* Bytes of the core's response handed to TinyUSB, released on completion. */
static size_t in_len;
static bool out_eom;

static void tmc_reset(void) {
  in_want = 0;
  in_len = 0;
  radar_clear();
}

void usb_poll(void) {
  const uint8_t *p;
  bool eom;
  size_t n;
  if (!in_want)
    return;
  n = radar_tx_peek(&p, in_want, &eom);
  if (!n)
    return;
  in_want = 0;
  if (tud_usbtmc_transmit_dev_msg_data(p, n, eom, false))
    in_len = n;
}

/* USB488 READ_STATUS_BYTE answered over the interrupt endpoint from a static
   buffer (the class driver queues it from the stack); everything else goes
   to the TinyUSB USBTMC driver. */
static bool tmc_control(uint8_t rhport, uint8_t stage,
                        tusb_control_request_t const *req) {
  static usbtmc_read_stb_interrupt_488_t note;
  usbtmc_read_stb_rsp_488_t rsp;
  uint8_t tag = (uint8_t)(req->wValue & 0x7Fu);
  if (stage != CONTROL_STAGE_SETUP ||
      req->bmRequestType_bit.type != TUSB_REQ_TYPE_CLASS ||
      req->bRequest != USB488_bREQUEST_READ_STATUS_BYTE)
    return usbtmcd_control_xfer_cb(rhport, stage, req);
  TU_VERIFY(req->bmRequestType == 0xA1 && req->wLength == sizeof(rsp) &&
            req->wValue == tag && tag >= 2 && req->wIndex == ITF_TMC);
  rsp.bTag = tag;
  rsp.statusByte = 0;
  if (usbd_edpt_busy(rhport, EP_INT)) {
    rsp.USBTMC_status = USB488_STATUS_INTERRUPT_IN_BUSY;
  } else {
    rsp.USBTMC_status = USBTMC_STATUS_SUCCESS;
    note.bNotify1.one = 1;
    note.bNotify1.bTag = tag;
    note.StatusByte = radar_stb();
    TU_VERIFY(
        usbd_edpt_xfer(rhport, EP_INT, (uint8_t *)&note, sizeof(note), false));
  }
  return tud_control_xfer(rhport, req, &rsp, sizeof(rsp));
}

static const usbd_class_driver_t tmc_driver = {
    .name = NULL,
    .init = usbtmcd_init_cb,
    .deinit = usbtmcd_deinit,
    .reset = usbtmcd_reset_cb,
    .open = usbtmcd_open_cb,
    .control_xfer_cb = tmc_control,
    .xfer_cb = usbtmcd_xfer_cb,
};

usbd_class_driver_t const *usbd_app_driver_get_cb(uint8_t *driver_count) {
  *driver_count = 1;
  return &tmc_driver;
}

usbtmc_response_capabilities_488_t const *tud_usbtmc_get_capabilities_cb(void) {
  return &capabilities;
}

void tud_usbtmc_open_cb(uint8_t interface_id) {
  (void)interface_id;
  tmc_reset();
  tud_usbtmc_start_bus_read();
}

bool tud_usbtmc_msgBulkOut_start_cb(usbtmc_msg_request_dev_dep_out const *h) {
  out_eom = h->bmTransferAttributes.EOM;
  return true;
}

bool tud_usbtmc_msg_data_cb(void *data, size_t len, bool transfer_complete) {
  radar_rx((const uint8_t *)data, len, transfer_complete && out_eom);
  tud_usbtmc_start_bus_read();
  return true;
}

bool tud_usbtmc_msgBulkIn_request_cb(usbtmc_msg_request_dev_dep_in const *r) {
  in_want = r->TransferSize;
  in_len = 0;
  usb_poll();
  return true;
}

bool tud_usbtmc_msgBulkIn_complete_cb(void) {
  radar_tx_done(in_len);
  in_len = 0;
  tud_usbtmc_start_bus_read();
  return true;
}

bool tud_usbtmc_initiate_clear_cb(uint8_t *tmcResult) {
  tmc_reset();
  *tmcResult = USBTMC_STATUS_SUCCESS;
  return true;
}

bool tud_usbtmc_check_clear_cb(usbtmc_get_clear_status_rsp_t *rsp) {
  rsp->USBTMC_status = USBTMC_STATUS_SUCCESS;
  rsp->bmClear.BulkInFifoBytes = 0;
  return true;
}

bool tud_usbtmc_initiate_abort_bulk_in_cb(uint8_t *tmcResult) {
  tmc_reset();
  *tmcResult = USBTMC_STATUS_SUCCESS;
  return true;
}

/* Runs before the driver maps its state to the response; re-arming the
   bulk-OUT endpoint here ends the abort, so report it complete. */
bool tud_usbtmc_check_abort_bulk_in_cb(usbtmc_check_abort_bulk_rsp_t *rsp) {
  if (tud_usbtmc_start_bus_read())
    rsp->USBTMC_status = USBTMC_STATUS_SUCCESS;
  return true;
}

bool tud_usbtmc_initiate_abort_bulk_out_cb(uint8_t *tmcResult) {
  tmc_reset();
  *tmcResult = USBTMC_STATUS_SUCCESS;
  return true;
}

bool tud_usbtmc_check_abort_bulk_out_cb(usbtmc_check_abort_bulk_rsp_t *rsp) {
  (void)rsp;
  return true;
}

void tud_usbtmc_bulkIn_clearFeature_cb(void) {}

void tud_usbtmc_bulkOut_clearFeature_cb(void) { tud_usbtmc_start_bus_read(); }

bool tud_usbtmc_indicator_pulse_cb(tusb_control_request_t const *msg,
                                   uint8_t *tmcResult) {
  (void)msg;
  *tmcResult = USBTMC_STATUS_SUCCESS;
  return true;
}

uint8_t tud_usbtmc_get_stb_cb(uint8_t *tmcResult) {
  *tmcResult = USBTMC_STATUS_SUCCESS;
  return radar_stb();
}
