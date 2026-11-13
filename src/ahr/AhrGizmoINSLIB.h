#pragma once

#include "ahr.h"

#include <Arduino.h>
#include <madflight_modules.h> // modules only - madflight.h must be included exactly once (in main.cpp)
#include <math.h>

#include "non-MIT-INSLIB/nav_suite.h" // INSLIB (has its own extern "C" guards)

//---------------------------------------------------------------------------------
// Configuration
//---------------------------------------------------------------------------------
struct InsFusionConfig {
  uint32_t samples_per_epoch = 8; // IMU samples per INS epoch. 1 = every IMU sample goes straight to the filter (no averaging).
  //uint32_t period_ms = 10;        // only for the optional INS task (start_task): how often it drains the IMU topic
  float gnss_rx_latency_ms = 100; // receiver internal latency before the PVT message is sent. VERIFY/tune
  uint8_t gnss_min_sats = 6;
  float gnss_max_vel_acc_mps = 2.0f;
  float mag_year = 2026.8f;       // decimal year for the World Magnetic Model
  bool use_mag = true;
  bool estimate_mag_bias = false;  // true: estimate the magnetometer hard-iron bias online (18-state mode). Does NOT replace
                                  // a proper calibration (soft-iron is not modelled), but reduces yaw error from a rough one.
                                  // Costs CPU: 18 instead of 15 filter states (worst-case epoch grows roughly with n^3).
};


class AhrGizmoINSLIB : public AhrGizmo {
private:
  AhrConfig *config;
  AhrState *state;

  bool feed_;
public:
  AhrGizmoINSLIB(Ahr* ahr, bool feed_samples = false) {
    this->config = &(ahr->config);
    this->state = (AhrState*)ahr;
    feed_ = feed_samples;

    setup();
  }

  void setInitalOrientation(float *qnew) {
    (void)qnew; // INSLIB initialises itself (auto_init)
    //ahrs.q0 = qnew[0];
    //ahrs.q1 = qnew[1];
    //ahrs.q2 = qnew[2];
    //ahrs.q3 = qnew[3];
  }

//---------------------------------------------------------------------------------
// madflight AHR gizmo: lets INSLIB act as madflight's attitude estimator.
//
// Ahr::update() (called from imu_loop() at the IMU rate) calls AhrGizmoIns::update(); afterwards madflight derives
// ahr.roll/pitch/yaw from ahr.q as usual, so CLI 'pahr', logs and everything reading ahr.* see the INSLIB attitude.
//
//   feed_samples = false (default, run mode A): the INS runs in its own task (insf.start_task); the gizmo only mirrors
//                  the latest INS quaternion into ahr.q. No INS work happens inside imu_loop().
//   feed_samples = true  (run mode B): the gizmo feeds every raw IMU sample to the INS right here, in the IMU task.
//
// Create it AFTER madflight_setup() and AFTER insf.setup():   ahr.gizmo = new AhrGizmoIns(&ahr, &insf [, true]);
// The old gizmo is deliberately not deleted: the IMU task may be inside it right now.
//---------------------------------------------------------------------------------

  bool update() {
    //if (feed_) {
      Imu* i = config->pimu; // raw (calibrated, unfiltered) IMU sample
      return feed(i->ax, i->ay, i->az, i->gx, i->gy, i->gz, i->ts);
      //if (ins_->state->att_valid) for (int k = 0; k < 4; k++) ahr_->q[k] = ins_->state->q[k];
    //} else {
      //InsFusionState s; // consistent copy: the INS task publishes concurrently
      //if (ins_->topic.pull_latest(&s) && s.att_valid) for (int k = 0; k < 4; k++) ahr_->q[k] = s.q[k];
    //}
    //return true;
  }






//---------------------------------------------------------------------------------
// Module
//---------------------------------------------------------------------------------

public:
  InsFusionConfig insconfig;

  // diagnostics
  uint32_t n_epochs = 0;
  uint32_t n_imu_samples = 0;
  uint32_t n_gnss = 0;
  uint32_t last_step_us = 0, max_step_us = 0;
  uint32_t max_dt_us = 0, n_gap = 0;   // largest epoch spacing seen, and number of epochs later than 1.5x the running mean
  float dbg_acc_ins[3] = {0}; // last averaged accel in INSLIB convention (expect {0,0,-9.8} when level and still)

  // Returns 0 on success
  int setup() {
    if (!imu.installed()) return -1;

    // nav_suite_t is large (history ring buffers + several Kalman filters): keep it static, zero-initialised.
    memset(&ns, 0, sizeof(ns));

    ins_init_t init;
    memset(&init, 0, sizeof(init));
    init.pos_init_stddev_m = 50.0f;     // coarse: auto_init takes the first GNSS fix
    init.vel_init_stddev_mps = 5.0f;
    for (int i = 0; i < 3; i++) init.rpy_init_stddev_rad[i] = 0.17453293f; // 10 deg

    ins_options_t opt;
    memset(&opt, 0, sizeof(opt));
    opt.auto_init = true;               // level + position from the first IMU/GNSS data
    opt.estimate_mag_bias = insconfig.estimate_mag_bias;
    // VERIFY: later tuning candidates (see ins.h ins_options_t): kalman_update_dt_sec (default 0.05 s),
    // gnss_max_*_stddev, imu_*_misalignment, imu_acc_fixed_bias (use your accel calibration), etc.

    if (nav_suite_init(&ns, &init, &opt) != 0) return -2;

    started = true;
    MF_printf("AHR: nav_suite_t = %u bytes\n", (unsigned)sizeof(nav_suite_t));
    return 0;
  }

  // Drain the IMU topic and feed every sample to the filter. Use this from the optional INS task (start_task).
  // Returns true if at least one epoch was processed.
  bool step() {
    if (!started) return false;
    if (!imu_sub) imu_sub = new MsgSubscription<ImuState>("ins_imu", &imu.topic);
    ImuState s;
    bool any = false;
    while (imu_sub->pull_next(&s)) any |= feed(s.ax, s.ay, s.az, s.gx, s.gy, s.gz, s.ts);
    return any;
  }

  // Feed ONE raw IMU sample (madflight units/frame: [g], [deg/s], ts in [us]).
  // With insconfig.samples_per_epoch == 1 every sample is one filter epoch (no averaging); with N > 1 the samples are
  // averaged over N. Call from exactly one context (the IMU task via the AHR gizmo / imu_loop(), OR the INS task).
  // Returns true if an epoch was processed.
  bool feed(float ax, float ay, float az, float gx, float gy, float gz, uint32_t ts) {
    if (!started) return false;
    acc_sum[0] += ax; acc_sum[1] += ay; acc_sum[2] += az;
    gyr_sum[0] += gx; gyr_sum[1] += gy; gyr_sum[2] += gz;
    n_acc++;
    n_imu_samples++;
    if (n_acc < insconfig.samples_per_epoch) return false;

    uint32_t t0 = micros();
    float sum_a[3] = {acc_sum[0], acc_sum[1], acc_sum[2]};
    float sum_g[3] = {gyr_sum[0], gyr_sum[1], gyr_sum[2]};
    const int n = (int)n_acc;
    const uint32_t ts_end = ts;
    for (int i = 0; i < 3; i++) acc_sum[i] = gyr_sum[i] = 0;
    n_acc = 0;

    int64_t t_end64 = unwrap(ts_end);
    if (!have_t_prev) { // first epoch: just establish time base
      t_prev64 = t_end64;
      have_t_prev = true;
      return false;
    }
    float dt = (float)(t_end64 - t_prev64) * 1e-6f;
    uint32_t dt_us = (uint32_t)(t_end64 - t_prev64);
    t_prev64 = t_end64;
    if (dt <= 0.0f || dt > 0.5f) return false;

    // spacing diagnostics: a late epoch means the IMU task was blocked (e.g. a long covariance update)
    if (dt_us > max_dt_us) max_dt_us = dt_us;
    if (dt_mean_us > 0 && dt_us > 1.5f * dt_mean_us) n_gap++;
    dt_mean_us = (dt_mean_us == 0) ? (float)dt_us : dt_mean_us + 0.01f * ((float)dt_us - dt_mean_us);

    const float inv_n = 1.0f / (float)n;
    const float G = 9.80665f;
    const float D2R = 0.017453292519943295f;

    ins_measurements_t m;
    memset(&m, 0, sizeof(m));
    m.timestamp = t_end64;
    m.strapdown_dt_sec = dt;

    // madflight: acceleration in [g], +1 g on the Down axis when level (it is -specific force).
    // INSLIB: specific force in [m/s^2], FRD, -9.8 on z when level. => negate. VERIFY with dbg_acc_ins.
    m.acc.is_valid = true;
    for (int i = 0; i < 3; i++) {
      m.acc.data[i] = -sum_a[i] * inv_n * G;
      dbg_acc_ins[i] = m.acc.data[i];
    }
    m.gyr.is_valid = true;
    for (int i = 0; i < 3; i++) m.gyr.data[i] = sum_g[i] * inv_n * D2R; // deg/s -> rad/s

    // ---- 2. magnetometer (new sample only) ----
    if (insconfig.use_mag) {
      MagState ms;
      if (mag.topic.pull_latest(&ms) && ms.ts != last_mag_ts && !(ms.mx == 0 && ms.my == 0 && ms.mz == 0)) {
        last_mag_ts = ms.ts;
        m.mag.is_valid = true; // VERIFY: sign/axes of mag vs. IMU (nose north, level: mx large positive, mz positive in the northern hemisphere)
        m.mag.data[0] = ms.mx; m.mag.data[1] = ms.my; m.mag.data[2] = ms.mz;
      }
    }

    // ---- 3. barometer (new sample only) ----
    {
      BarState bs;
      if (bar.topic.pull_latest(&bs) && bs.ts != last_bar_ts && bs.press > 30000.0f && bs.press < 110000.0f) {
        last_bar_ts = bs.ts;
        m.baro.is_valid = true;
        m.baro.pressure_pa = bs.press;
        m.baro.stddev_m = 0.0f; // consumer default
      }
    }

    // ---- 4. GNSS (new fix only) ----
    state->gnss_used = false;
    {
      GpsState g;
      if (gps.installed() && gps.topic.pull_latest(&g) && g.time != last_gps_itow && g.fix >= GPS_OK_FIX_3D) {
        last_gps_itow = g.time;
        float vacc = g.have_vel_acc ? g.vel_acc * 1e-3f : 1.0f; // mm/s -> m/s
        if (g.sat >= insconfig.gnss_min_sats && vacc > 0 && vacc < insconfig.gnss_max_vel_acc_mps) {
          float hacc = g.have_hacc ? g.hacc * 1e-3f : 5.0f;
          float vac2 = g.have_vacc ? g.vacc * 1e-3f : 8.0f;
          float undulation_mm = g.have_undulation ? (float)g.undulation : 0.0f;
          undulation_m = undulation_mm * 1e-3f;

          m.gnss_pos.is_valid = true;
          m.gnss_pos.llh[0] = (double)g.lat * 1e-7 * (M_PI / 180.0);
          m.gnss_pos.llh[1] = (double)g.lon * 1e-7 * (M_PI / 180.0);
          // madflight stores MSL altitude and undulation = MSL - ellipsoid  =>  ellipsoid = MSL - undulation
          m.gnss_pos.llh[2] = ((double)g.alt - (double)undulation_mm) * 1e-3;
          memset(m.gnss_pos.Qll_ned, 0, sizeof(m.gnss_pos.Qll_ned)); // 3x3 column-major, diagonal only
          m.gnss_pos.Qll_ned[0] = hacc * hacc;
          m.gnss_pos.Qll_ned[4] = hacc * hacc;
          m.gnss_pos.Qll_ned[8] = vac2 * vac2;

          if (g.have_veld) {
            m.gnss_vel.is_valid = true;
            m.gnss_vel.vel_ned[0] = g.veln * 1e-3f;
            m.gnss_vel.vel_ned[1] = g.vele * 1e-3f;
            m.gnss_vel.vel_ned[2] = g.veld * 1e-3f;
            memset(m.gnss_vel.Qll_ned, 0, sizeof(m.gnss_vel.Qll_ned));
            m.gnss_vel.Qll_ned[0] = vacc * vacc;
            m.gnss_vel.Qll_ned[4] = vacc * vacc;
            m.gnss_vel.Qll_ned[8] = vacc * vacc; // one 3D speed accuracy for all axes (conservative for the vertical axis)
          }

          // How old is the fix relative to this epoch? Message arrival is last_gps_time_ms (system ms, VERIFY it is
          // stamped when the PVT message is parsed), plus receiver latency before it was sent.
          uint32_t arrival_us = g.last_gps_time_ms * 1000u;
          int32_t age_us = (int32_t)(ts_end - arrival_us) + (int32_t)(insconfig.gnss_rx_latency_ms * 1000.0f);
          if (age_us < 0) age_us = 0;
          m.gnss_delay_ms = age_us / 1000;
          m.gnss_delay_us = age_us % 1000;
          m.gps_week = g.time_week;
          m.gps_itow_ms = (int)g.time;

          if (!mag_model_set) {
            nav_suite_set_magnetic_model_position(&ns, m.gnss_pos.llh[0], m.gnss_pos.llh[1], insconfig.mag_year);
            mag_model_set = true;
          }
          state->gnss_used = true;
          n_gnss++;
        }
      }
    }

    // ---- 5. run the filters ----
    nav_suite_update(&ns, &m);
    n_epochs++;

    // ---- 6. publish ----
    state->ts = ts_end;
    state->mode = (uint8_t)nav_suite_get_mode(&ns);
    state->ready = ins_is_ready(&ns.ins);

    float v[3];
    state->vel_up_valid = state->ready && ins_get_velocity_ned(&ns.ins, v);
    if (state->vel_up_valid) {
      for (int i = 0; i < 3; i++) state->vel_ned[i] = v[i];
      state->vel_up = -v[2];
    }
    float hb, vb;
    state->baro_v_up_valid = nav_suite_get_baro_alt(&ns, &hb, &vb);
    if (state->baro_v_up_valid) { state->baro_v_up = vb; state->baro_h = hb; }
    state->baro_isa_valid = baro_alt_get_isa_altitude(&ns.baro_alt, &state->baro_h_isa);

    float p[3];
    if (state->ready && ins_get_position_local(&ns.ins, p)) for (int i = 0; i < 3; i++) state->pos_ned[i] = p[i];
    state->height_valid = nav_suite_get_height(&ns, &state->height);

    float r, pt, y;
    state->att_valid = nav_suite_get_rpy(&ns, &r, &pt, &y);
    if (state->att_valid) {
      state->roll = r * 57.2957795f; state->pitch = pt * 57.2957795f; state->yaw = y * 57.2957795f;
    }
    float q[4];
    if (state->ready && ins_get_quaternion(&ns.ins, q)) {
      for (int i = 0; i < 4; i++) state->q[i] = q[i];
    } else if (state->att_valid) { // attitude-only mode (no GNSS yet): build q from roll/pitch/yaw (ZYX, body->NED)
      float cr = cosf(r * 0.5f), sr = sinf(r * 0.5f), cp = cosf(pt * 0.5f), sp = sinf(pt * 0.5f), cy = cosf(y * 0.5f), sy = sinf(y * 0.5f);
      state->q[0] = cr * cp * cy + sr * sp * sy;
      state->q[1] = sr * cp * cy - cr * sp * sy;
      state->q[2] = cr * sp * cy + sr * cp * sy;
      state->q[3] = cr * cp * sy - sr * sp * cy;
    }
    state->pos_valid = false;
    if (state->ready && state->mode == 3) {
      double llh[3]; // INSLIB keeps lat/lon natively: no ECEF round trip (expensive without a double FPU, as on the RP2350)
      if (ins_get_latlonh(&ns.ins, llh)) {
        state->lat_deg = llh[0] * 57.29577951308232;
        state->lon_deg = llh[1] * 57.29577951308232;
        state->pos_valid = true;
      }
    }
    state->h_ell_valid = nav_suite_get_height_ellipsoid(&ns, &state->h_ell);
    state->h_msl = state->h_ell + undulation_m;
    float ab[3];
    if (state->ready && ins_get_bias_acc(&ns.ins, ab)) for (int i = 0; i < 3; i++) state->acc_bias[i] = ab[i];

    last_step_us = micros() - t0;
    if (last_step_us > max_step_us) max_step_us = last_step_us;
    return true;
  }

/*  
  // FreeRTOS task: 16 KB+ of stack needed (INSLIB reports ~13 KB worst case on x86-64, VERIFY with uxTaskGetStackHighWaterMark)
  static void task(void* pv) {
    InsFusion* self = (InsFusion*)pv;
    TickType_t last = xTaskGetTickCount();
    const TickType_t period = pdMS_TO_TICKS(self->insconfig.period_ms);
    for (;;) {
      self->step();
      vTaskDelayUntil(&last, period);
    }
  }
*/

  // Best available climb rate [m/s]: GNSS-aided INS when it is ready and in FULL mode, else the baro/accel filter.
  float getV() {
     return (state->vel_up_valid && state->mode == 3) ? state->vel_up : state->baro_v_up;
  }

  // Best available height relative to start [m]
  float getH() { 
    return state->height_valid ? state->height : state->baro_h;
  }

  // Absolute position, valid only while GNSS-aided (FULL mode). Returns false otherwise: do not report INS position to
  // anything while this is false - without GNSS the position drifts away within seconds.
  bool getLatLon(double* lat_deg, double* lon_deg) {
    if (!state->pos_valid) return false;
    *lat_deg = state->lat_deg; *lon_deg = state->lon_deg; return true;
  }

  // WGS84 ellipsoid height [m]
  bool getHeightEllipsoid(float* h) {
     if (!state->h_ell_valid) return false;
     *h = state->h_ell;
     return true;
  }

  // Height above mean sea level [m] = ellipsoid height + undulation (undulation is 0 until the GNSS driver supplies it)
  bool getHeightMsl(float* h) {
    if (!state->h_ell_valid) return false;
    *h = state->h_msl;
    return true; 
  }

  // Height above the start point [m] and above a point marked later (e.g. at launch). This is "above home", NOT above
  // the terrain below you (no terrain data).
  void markHome() {
    home_h = getH();
  }

  float getAgh() {
    return getH() - home_h;
  }

  void resetDiagnostics() {
    max_step_us = 0;
    max_dt_us = 0;
    n_gap = 0;
  }

  // true if getV() is backed by a running filter (otherwise it would just return 0)
  bool hasV() {
    return (state->vel_up_valid && state->mode == 3) || state->baro_v_up_valid;
  }

  // Static pressure [Pa] consistent with the filtered (baro + accelerometer) height, for reporting instead of the raw
  // pressure. raw_pa: the current raw barometer pressure. The weather/QNH offset stays in raw_pa; only the filtered-minus-raw
  // height difference is applied, via the ISA formula in ratio form, so a formula mismatch cancels and filtered == raw gives
  // exactly raw_pa. Uses the baro-only ISA altitude - never the GNSS-corrected height, which would put GNSS height error
  // into the "pressure". Returns false (use raw_pa) if the filter is not valid or disagrees with raw by more than 10 m.
  bool getStaticPressure(float raw_pa, float* out_pa) {
    if (!state->baro_isa_valid || !(raw_pa > 30000.0f && raw_pa < 110000.0f)) return false;
    const float h_raw = baro_alt_pressure_to_altitude(raw_pa);
    if (fabsf(state->baro_h_isa - h_raw) > 10.0f) return false;
    *out_pa = raw_pa * powf((1.0f - state->baro_h_isa / 44330.0f) / (1.0f - h_raw / 44330.0f), 5.255f);
    return true;
  }

  const char* vSource() {
    return (state->vel_up_valid && state->mode == 3) ? "GNSS-INS" : (state->baro_v_up_valid ? "BARO-INS" : "none");
  }

  // One-line orientation self-test string for printing (use your safe printer, e.g. mfSerial.printf("%s\n", buf)).
  // Compares the INS roll/pitch/yaw with the angles recomputed from the published quaternion (they must agree - this
  // validates the quaternion convention), and shows the accelerometer as INSLIB sees it (level + still: {0,0,-9.8}).
  int orientationString(char* buf, size_t n) {
    const float r2d = 57.2957795f;
    const float* q = state->q;
    float qr = atan2f(2 * (q[0] * q[1] + q[2] * q[3]), 1 - 2 * (q[1] * q[1] + q[2] * q[2])) * r2d;
    float sp = 2 * (q[0] * q[2] - q[3] * q[1]); if (sp > 1) sp = 1; if (sp < -1) sp = -1;
    float qp = asinf(sp) * r2d;
    float qy = atan2f(2 * (q[0] * q[3] + q[1] * q[2]), 1 - 2 * (q[2] * q[2] + q[3] * q[3])) * r2d;
    return snprintf(buf, n,
      "INS rpy %7.2f %7.2f %7.2f | from q %7.2f %7.2f %7.2f | acc(INS frame) %6.2f %6.2f %6.2f | mode=%d ready=%d att=%d | "
      "bias %5.2f %5.2f %5.2f | step max %uus, epoch dt max %uus, gaps %u",
      state->roll, state->pitch, state->yaw, qr, qp, qy,
      dbg_acc_ins[0], dbg_acc_ins[1], dbg_acc_ins[2], state->mode, state->ready, state->att_valid,
      state->acc_bias[0], state->acc_bias[1], state->acc_bias[2], (unsigned)max_step_us, (unsigned)max_dt_us, (unsigned)n_gap);
  }

/*
  // Start the INS task. core: 0 or 1 - check which core mf_IMU runs on (FreeRTOS task table, "Core" column) and
  // prefer the other one if it has idle time. priority: keep it BELOW the IMU task so the IMU is never starved.
  void start_task(UBaseType_t priority, int core = 0, uint32_t stack_words = 5120) {
    hal_xTaskCreate(task, "mf_INS", stack_words, this, priority, &task_handle, core);
  }

  UBaseType_t stack_high_water_words() {
    return task_handle ? uxTaskGetStackHighWaterMark(task_handle) : 0;
  }
*/

private:
  nav_suite_t ns; // lives inside the object: declare the InsFusion instance as a global, never on a task stack
  MsgSubscription<ImuState>* imu_sub = nullptr;
  TaskHandle_t task_handle = nullptr;
  bool started = false, have_t_prev = false, mag_model_set = false;
  int64_t t_prev64 = 0, t_unwrapped = 0;
  uint32_t t_last32 = 0, last_mag_ts = 0, last_bar_ts = 0, last_gps_itow = 0xFFFFFFFFu;
  bool unwrap_init = false;
  float acc_sum[3] = {0}, gyr_sum[3] = {0};
  uint32_t n_acc = 0;
  float dt_mean_us = 0;
  float undulation_m = 0;   // geoid undulation (MSL - ellipsoid) from the last accepted GNSS fix
  float home_h = 0;

  int64_t unwrap(uint32_t t) {
    if (!unwrap_init) { t_unwrapped = t; t_last32 = t; unwrap_init = true; return t_unwrapped; }
    t_unwrapped += (int32_t)(t - t_last32);
    t_last32 = t;
    return t_unwrapped;
  }
};
