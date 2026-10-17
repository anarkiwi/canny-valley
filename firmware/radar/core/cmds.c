#include <string.h>

#include "core.h"
#include "fmt.h"
#include "hal.h"

#define F_MIN 2.25
#define F_MAX 2.5
#define NS_PER_PD 50
#define MODEL "QM4004"

static scpi_error_t errors[10];
static char input[256];

static scpi_result_t range_error(scpi_t *c) {
  SCPI_ErrorPush(c, RADAR_ERR_RANGE);
  return SCPI_RES_ERR;
}

static void fixed(scpi_t *c, int64_t v, uint8_t decimals) {
  char s[24];
  SCPI_ResultCharacters(c, s, fmt_fixed(s, v, decimals));
}

static synth_plan_t plan(void) {
  synth_plan_t p;
  core_plan(&p);
  return p;
}

static scpi_result_t real(scpi_t *c, double lo, double hi, double *dst) {
  double v;
  if (!SCPI_ParamDouble(c, &v, TRUE))
    return SCPI_RES_ERR;
  if (!(v >= lo && v <= hi))
    return range_error(c);
  *dst = v;
  core_reapply();
  return SCPI_RES_OK;
}

static scpi_result_t uint_param(scpi_t *c, uint32_t lo, uint32_t hi,
                                uint32_t *v) {
  if (!SCPI_ParamUInt32(c, v, TRUE))
    return SCPI_RES_ERR;
  return *v >= lo && *v <= hi ? SCPI_RES_OK : range_error(c);
}

static scpi_result_t freq_star(scpi_t *c) {
  return real(c, F_MIN, F_MAX, &core.f0);
}

static scpi_result_t freq_stop(scpi_t *c) {
  return real(c, F_MIN, F_MAX, &core.f1);
}

static scpi_result_t freq_star_q(scpi_t *c) {
  fixed(c, synth_hz(plan().n0), 9);
  return SCPI_RES_OK;
}

static scpi_result_t freq_stop_q(scpi_t *c) {
  synth_plan_t p = plan();
  fixed(c, synth_hz(p.step ? p.n1 : synth_nq(core.f1)), 9);
  return SCPI_RES_OK;
}

static scpi_result_t duration(scpi_t *c, double *dst) {
  double v;
  if (!SCPI_ParamDouble(c, &v, TRUE))
    return SCPI_RES_ERR;
  if (!(v > 0.0 && v <= SYNTH_RAMP_MS_MAX) ||
      synth_samples(v, core.rdiv) < SYNTH_GAP_MIN)
    return range_error(c);
  *dst = v;
  core_reapply();
  return SCPI_RES_OK;
}

static scpi_result_t ramp(scpi_t *c) { return duration(c, &core.ramp_ms); }

static scpi_result_t dwell(scpi_t *c) { return duration(c, &core.dwell_ms); }

static scpi_result_t ramp_q(scpi_t *c) {
  fixed(c, (int64_t)plan().nr * core.rdiv * NS_PER_PD, 6);
  return SCPI_RES_OK;
}

static scpi_result_t dwell_q(scpi_t *c) {
  fixed(c, (int64_t)plan().nd * core.rdiv * NS_PER_PD, 6);
  return SCPI_RES_OK;
}

static scpi_result_t type(scpi_t *c) {
  uint32_t v;
  if (uint_param(c, 0, TYPE_SAW, &v) != SCPI_RES_OK)
    return SCPI_RES_ERR;
  core.type = (uint8_t)v;
  if (core.running)
    core_stop(false);
  return SCPI_RES_OK;
}

static scpi_result_t type_q(scpi_t *c) {
  SCPI_ResultUInt32(c, core.type);
  return SCPI_RES_OK;
}

static scpi_result_t start(scpi_t *c) {
  return core_start() ? SCPI_RES_OK : (c->cmd_error = TRUE, SCPI_RES_ERR);
}

static scpi_result_t stop(scpi_t *c) {
  (void)c;
  core_stop(true);
  return SCPI_RES_OK;
}

static scpi_result_t origin_q(scpi_t *c) {
  synth_plan_t p = plan();
  bool cw = core.type == TYPE_CW;
  fixed(
      c,
      cw ? 0
         : synth_div_round((5 * SYNTH_DELAY_TCY - 2 * SYNTH_MARGIN) * 1000000LL,
                           4 * core.rdiv),
      6);
  SCPI_ResultUInt32(c, cw || (p.step ? p.step > 0 : core.f1 > core.f0));
  return SCPI_RES_OK;
}

static scpi_result_t limits_q(scpi_t *c) {
  const int64_t slope = (int64_t)SYNTH_FREF * SYNTH_FREF;
  fixed(c, synth_hz(SYNTH_NQ_MIN), 9);
  fixed(c, synth_hz(SYNTH_NQ_MAX), 9);
  fixed(c,
        (slope >> 24) * 1000000LL +
            synth_div_round((slope & 0xFFFFFF) * 1000000LL, 1LL << 24),
        6);
  fixed(c, (int64_t)core.rdiv * NS_PER_PD, 6);
  fixed(c, (int64_t)core.rdiv * NS_PER_PD * SYNTH_GAP_MIN, 6);
  return SCPI_RES_OK;
}

static scpi_result_t rf(scpi_t *c) {
  scpi_bool_t on;
  if (!SCPI_ParamBool(c, &on, TRUE))
    return SCPI_RES_ERR;
  core.rf = on;
  hal_rf(on);
  return SCPI_RES_OK;
}

static scpi_result_t rf_q(scpi_t *c) {
  SCPI_ResultUInt32(c, core.rf);
  return SCPI_RES_OK;
}

static scpi_result_t timeout(scpi_t *c) {
  uint32_t v;
  if (uint_param(c, 0, CORE_TIMEOUT_MAX, &v) != SCPI_RES_OK)
    return SCPI_RES_ERR;
  core.timeout_s = (uint16_t)v;
  return SCPI_RES_OK;
}

static scpi_result_t timeout_q(scpi_t *c) {
  SCPI_ResultUInt32(c, core.timeout_s);
  return SCPI_RES_OK;
}

static scpi_result_t lock_q(scpi_t *c) {
  SCPI_ResultUInt32(c, core_lock());
  return SCPI_RES_OK;
}

static scpi_result_t refdiv(scpi_t *c) {
  uint32_t v;
  if (uint_param(c, 1, 256, &v) != SCPI_RES_OK)
    return SCPI_RES_ERR;
  core.refdiv = (uint16_t)v;
  return SCPI_RES_OK;
}

static scpi_result_t refdiv_q(scpi_t *c) {
  SCPI_ResultUInt32(c, core.refdiv);
  return SCPI_RES_OK;
}

static scpi_result_t acquire(scpi_t *c, uint32_t lo, uint32_t hi) {
  uint32_t v;
  if (uint_param(c, lo, hi, &v) != SCPI_RES_OK)
    return SCPI_RES_ERR;
  return core_acquire(v) ? SCPI_RES_OK : (c->cmd_error = TRUE, SCPI_RES_ERR);
}

/* Bring-up diagnostics: raw synthesiser registers, the LD_SDO level and its
   edge count. They use SPI2 and Timer2, so only while the clock is stopped. */
static bool clock_idle(scpi_t *c) {
  if (core.clock)
    SCPI_ErrorPush(c, RADAR_ERR_CONFLICT);
  return !core.clock;
}

static scpi_result_t reg(scpi_t *c) {
  uint32_t a, v;
  if (!clock_idle(c) || uint_param(c, 0, 0x3F, &a) != SCPI_RES_OK ||
      uint_param(c, 0, 0xFFFFFF, &v) != SCPI_RES_OK)
    return SCPI_RES_ERR;
  hal_synth_write((uint8_t)a, v);
  return SCPI_RES_OK;
}

static scpi_result_t reg_q(scpi_t *c) {
  uint32_t a;
  if (!clock_idle(c) || uint_param(c, 0, 0x3F, &a) != SCPI_RES_OK)
    return SCPI_RES_ERR;
  SCPI_ResultUInt32(c, hal_synth_read((uint8_t)a));
  return SCPI_RES_OK;
}

static scpi_result_t pin_q(scpi_t *c) {
  if (!clock_idle(c))
    return SCPI_RES_ERR;
  SCPI_ResultUInt32(c, hal_gpo_level());
  return SCPI_RES_OK;
}

static scpi_result_t adc_q(scpi_t *c) {
  if (!clock_idle(c))
    return SCPI_RES_ERR;
  SCPI_ResultUInt32(c, hal_adc_slow());
  return SCPI_RES_OK;
}

static scpi_result_t edge_q(scpi_t *c) {
  uint32_t ms, ps = 0;
  if (!clock_idle(c) || uint_param(c, 1, 1000, &ms) != SCPI_RES_OK)
    return SCPI_RES_ERR;
  if (SCPI_ParamUInt32(c, &ps, FALSE) && ps > 3)
    return range_error(c);
  SCPI_ResultUInt32(c, hal_edges((uint16_t)ms, (uint8_t)ps));
  return SCPI_RES_OK;
}

static scpi_result_t frame(scpi_t *c) { return acquire(c, 1, CORE_FRAME_MAX); }

static scpi_result_t cont(scpi_t *c) { return acquire(c, 0, 0x7FFFFFFFu); }

static scpi_result_t frame_q(scpi_t *c) {
  (void)c;
  core.out = OUT_HEX;
  core.text_len = core.sent = 0;
  return SCPI_RES_OK;
}

static scpi_result_t block_q(scpi_t *c) {
  (void)c;
  core.out = OUT_BLOCK;
  core.text_len = core.sent = 0;
  return SCPI_RES_OK;
}

static scpi_result_t abort_acq(scpi_t *c) {
  (void)c;
  core_abort();
  return SCPI_RES_OK;
}

static scpi_result_t rate(scpi_t *c) {
  double hz;
  if (!SCPI_ParamDouble(c, &hz, TRUE))
    return SCPI_RES_ERR;
  if (!(hz >= (double)SYNTH_FREF / (SYNTH_R_MAX + SYNTH_R_STEP / 2.0) &&
        hz <= (double)SYNTH_FREF / (SYNTH_R_MIN - SYNTH_R_STEP / 2.0)))
    return range_error(c);
  core_set_rate(
      (uint16_t)(SYNTH_R_STEP *
                 (uint16_t)((double)SYNTH_FREF / SYNTH_R_STEP / hz + 0.5)));
  return SCPI_RES_OK;
}

static scpi_result_t rate_q(scpi_t *c) {
  fixed(c, synth_div_round(SYNTH_FREF * 1000000LL, core.rdiv), 6);
  return SCPI_RES_OK;
}

static scpi_result_t temp_q(scpi_t *c) {
  int32_t code = core_temp() & 0x3FFF;
  if (code & 0x2000)
    code -= 0x4000;
  fixed(c, synth_div_round((int64_t)code * 25, 8), 2);
  return SCPI_RES_OK;
}

static scpi_result_t stat_q(scpi_t *c) {
  SCPI_ResultInt32(c, 0);
  SCPI_ResultText(c, "Operational");
  return SCPI_RES_OK;
}

static scpi_result_t model_q(scpi_t *c) {
  SCPI_ResultMnemonic(c, MODEL);
  return SCPI_RES_OK;
}

static scpi_result_t serial_q(scpi_t *c) {
  SCPI_ResultMnemonic(c, core.serial);
  return SCPI_RES_OK;
}

static scpi_result_t firm_q(scpi_t *c) {
  SCPI_ResultMnemonic(c, RADAR_FIRMWARE);
  return SCPI_RES_OK;
}

static const scpi_command_t commands[] = {
    {"*CLS", SCPI_CoreCls},
    {"*ESE", SCPI_CoreEse},
    {"*ESE?", SCPI_CoreEseQ},
    {"*ESR?", SCPI_CoreEsrQ},
    {"*IDN?", SCPI_CoreIdnQ},
    {"*OPC", SCPI_CoreOpc},
    {"*OPC?", SCPI_CoreOpcQ},
    {"*RST", SCPI_CoreRst},
    {"*SRE", SCPI_CoreSre},
    {"*SRE?", SCPI_CoreSreQ},
    {"*STB?", SCPI_CoreStbQ},
    {"*TST?", SCPI_CoreTstQ},
    {"*WAI", SCPI_CoreWai},
    {"SYSTem:ERRor[:NEXT]?", SCPI_SystemErrorNextQ},
    {"SYSTem:VERSion?", SCPI_SystemVersionQ},
    {"SYSTem:STATus?", stat_q},
    {"SYSTem:TEMPerature?", temp_q},
    {"SYSTem:IDENtify?", model_q},
    {"SYSTem:MODNUM?", model_q},
    {"SYSTem:SERNUM?", serial_q},
    {"SYSTem:FIRMware?", firm_q},
    {"SWEEP:FREQSTAR", freq_star},
    {"SWEEP:FREQSTAR?", freq_star_q},
    {"SWEEP:FREQSTOP", freq_stop},
    {"SWEEP:FREQSTOP?", freq_stop_q},
    {"SWEEP:RAMPTIME", ramp},
    {"SWEEP:RAMPTIME?", ramp_q},
    {"SWEEP:DWELl", dwell},
    {"SWEEP:DWELl?", dwell_q},
    {"SWEEP:TYPE", type},
    {"SWEEP:TYPE?", type_q},
    {"SWEEP:START", start},
    {"SWEEP:STOP", stop},
    {"SWEEP:ORIGin?", origin_q},
    {"SWEEP:LIMits?", limits_q},
    {"POWEr:RF", rf},
    {"POWEr:RF?", rf_q},
    {"POWEr:RF:TIMEout", timeout},
    {"POWEr:RF:TIMEout?", timeout_q},
    {"FREQuency:LOCK?", lock_q},
    {"FREQuency:REFerence:DIVider", refdiv},
    {"FREQuency:REFerence:DIVider?", refdiv_q},
    {"CAPTure:FRAMe", frame},
    {"CAPTure:FRAMe?", frame_q},
    {"CAPTure:CONTinuous", cont},
    {"CAPTure:ABORt", abort_acq},
    {"CAPTure:BLOCk?", block_q},
    {"CAPTure:RATE", rate},
    {"CAPTure:RATE?", rate_q},
    {"SYSTem:REGister", reg},
    {"SYSTem:REGister?", reg_q},
    {"SYSTem:PIN?", pin_q},
    {"SYSTem:EDGE?", edge_q},
    {"SYSTem:ADC?", adc_q},
    SCPI_CMD_LIST_END,
};

static size_t scpi_write(scpi_t *c, const char *data, size_t len) {
  (void)c;
  return core_text(data, len);
}

static scpi_result_t reset(scpi_t *c) {
  (void)c;
  hal_reset();
  return SCPI_RES_OK;
}

static scpi_interface_t iface = {NULL, scpi_write, NULL, NULL, reset};

void core_scpi_init(void) {
  SCPI_Init(&core.scpi, commands, &iface, scpi_units_def, "Quonset Microwave",
            MODEL, core.serial, RADAR_FIRMWARE, input, sizeof input, errors,
            sizeof errors / sizeof errors[0]);
}
