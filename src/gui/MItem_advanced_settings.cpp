/**
 * @file MItem_advanced_settings.cpp
 */

#include "MItem_advanced_settings.hpp"

#include "ScreenHandler.hpp"
#include <WindowMenuSpin.hpp>
#include <module/planner.h>
#include <module/prusa/homing_corexy.hpp>
#include <module/stepper/indirection.h>
#include <module/stepper/trinamic.h>
#include <config_store/constants.hpp>
#include <config_store/store_instance.hpp>
#include <persistent_stores/store_instances/config_store/defaults.hpp>
#include <persistent_stores/store_instances/config_store/store_c_api.h>
#include <printers.h>
#include <stdint.h>

static constexpr NumericInputConfig steps_per_unit_spin_config = {
    .min_value = 1,
    .max_value = 1000,
    .step = 0.01F,
    .max_decimal_places = 2,
};

static constexpr NumericInputConfig motor_current_spin_config = {
    .min_value = 200,
    .max_value = 958,
    .step = 1,
    .unit = Unit::milliamper,
};

static constexpr NumericInputConfig homing_sensitivity_spin_config = {
    .min_value = -10,
    .max_value = 10,
    .step = 1,
};

static constexpr NumericInputConfig chopper_toff_spin_config = {
    .min_value = 1,
    .max_value = 15,
    .step = 1,
};

static constexpr NumericInputConfig chopper_hend_spin_config = {
    .min_value = -3,
    .max_value = 12,
    .step = 1,
};

static constexpr NumericInputConfig chopper_hstrt_spin_config = {
    .min_value = 1,
    .max_value = 8,
    .step = 1,
};

static constexpr NumericInputConfig lcd_timeout_spin_config = {
    .min_value = 1,
    .max_value = 30,
    .step = 1,
    .unit = Unit::minute,
};

static void store_and_apply_steps_per_unit(const AxisEnum axis, const float steps) {
    if (steps < steps_per_unit_spin_config.min_value || steps > steps_per_unit_spin_config.max_value) {
        return;
    }

    planner.synchronize();
    if (planner.draining()) {
        return;
    }

    switch (axis) {
    case X_AXIS:
        set_steps_per_unit_x(steps);
        break;
    case Y_AXIS:
        set_steps_per_unit_y(steps);
        break;
    case Z_AXIS:
        set_steps_per_unit_z(steps);
        break;
    case E_AXIS:
        set_steps_per_unit_e(steps);
        break;
    default:
        return;
    }

    auto settings = planner.user_settings;
    settings.axis_steps_per_mm[axis] = steps;
    settings.axis_msteps_per_mm[axis] = steps * PLANNER_STEPS_MULTIPLIER;
    planner.apply_settings(settings);
    planner.refresh_positioning();
}

static uint16_t get_current_ma_x() {
#if PRINTER_IS_PRUSA_COREONE() && AXIS_IS_TMC(X)
    return stepperX.getMilliamps();
#else
    return get_rms_current_ma_x();
#endif
}

static uint16_t get_current_ma_y() {
#if PRINTER_IS_PRUSA_COREONE() && AXIS_IS_TMC(Y)
    return stepperY.getMilliamps();
#else
    return get_rms_current_ma_y();
#endif
}

static uint16_t get_current_ma_z() {
#if PRINTER_IS_PRUSA_COREONE() && AXIS_IS_TMC(Z)
    return stepperZ.getMilliamps();
#else
    return get_rms_current_ma_z();
#endif
}

static uint16_t get_current_ma_e() {
#if PRINTER_IS_PRUSA_COREONE() && AXIS_IS_TMC(E0)
    return stepperE0.getMilliamps();
#else
    return get_rms_current_ma_e();
#endif
}

static bool motor_current_is_valid(const uint16_t current) {
    return current >= motor_current_spin_config.min_value && current <= motor_current_spin_config.max_value;
}

static bool current_change_invalidates_corexy_homing(const AxisEnum axis, const uint16_t current) {
#if HAS_PRECISE_HOMING_COREXY()
    switch (axis) {
    case X_AXIS:
        return current != get_rms_current_ma_x();
    case Y_AXIS:
        return current != get_rms_current_ma_y();
    default:
        return false;
    }
#else
    (void)axis;
    (void)current;
    return false;
#endif
}

static void store_motor_current(const AxisEnum axis, const uint16_t current) {
    if (!motor_current_is_valid(current)) {
        return;
    }

    const bool invalidate_corexy_homing = current_change_invalidates_corexy_homing(axis, current);

    switch (axis) {
    case X_AXIS:
        set_rms_current_ma_x(current);
        break;
    case Y_AXIS:
        set_rms_current_ma_y(current);
        break;
    case Z_AXIS:
        set_rms_current_ma_z(current);
        break;
    case E_AXIS:
        set_rms_current_ma_e(current);
        break;
    default:
        return;
    }

#if HAS_PRECISE_HOMING_COREXY()
    if (invalidate_corexy_homing) {
        corexy_clear_homing_calibration();
    }
#endif
}

static void apply_motor_current(const AxisEnum axis, const uint16_t current) {
    if (!motor_current_is_valid(current)) {
        return;
    }

#if PRINTER_IS_PRUSA_COREONE()
    switch (axis) {
    case X_AXIS:
        #if AXIS_IS_TMC(X)
        stepperX.rms_current(current);
        #endif
        break;
    case Y_AXIS:
        #if AXIS_IS_TMC(Y)
        stepperY.rms_current(current);
        #endif
        break;
    case Z_AXIS:
        #if AXIS_IS_TMC(Z)
        stepperZ.rms_current(current);
        #endif
        break;
    case E_AXIS:
        #if AXIS_IS_TMC(E0)
        stepperE0.rms_current(current);
        #endif
        break;
    default:
        return;
    }
#else
    (void)axis;
#endif
}

static void store_and_apply_motor_current(const AxisEnum axis, const uint16_t current) {
    store_motor_current(axis, current);
    apply_motor_current(axis, current);
}

static int16_t get_default_homing_sensitivity(const AxisEnum axis) {
    return axis == X_AXIS ? X_STALL_SENSITIVITY : Y_STALL_SENSITIVITY;
}

static int16_t get_homing_sensitivity(const AxisEnum axis) {
    const int16_t sensitivity = (axis == X_AXIS) ? config_store().homing_sens_x.get() : config_store().homing_sens_y.get();
    return sensitivity == config_store_ns::stallguard_sensitivity_unset ? get_default_homing_sensitivity(axis) : sensitivity;
}

static bool homing_sensitivity_is_valid(const int16_t sensitivity) {
    return sensitivity >= homing_sensitivity_spin_config.min_value && sensitivity <= homing_sensitivity_spin_config.max_value;
}

static void store_homing_sensitivity(const AxisEnum axis, const int16_t sensitivity) {
    if (!homing_sensitivity_is_valid(sensitivity)) {
        return;
    }

    switch (axis) {
    case X_AXIS:
        config_store().homing_sens_x.set(sensitivity);
        break;
    case Y_AXIS:
        config_store().homing_sens_y.set(sensitivity);
        break;
    default:
        return;
    }

#if HAS_PRECISE_HOMING_COREXY()
    corexy_clear_homing_calibration();
#endif
}

static void apply_homing_sensitivity(const AxisEnum axis, const int16_t sensitivity) {
    if (!homing_sensitivity_is_valid(sensitivity)) {
        return;
    }

#if PRINTER_IS_PRUSA_COREONE()
    switch (axis) {
    case X_AXIS:
        #if X_SENSORLESS
        stepperX.stall_sensitivity(sensitivity);
        #endif
        break;
    case Y_AXIS:
        #if Y_SENSORLESS
        stepperY.stall_sensitivity(sensitivity);
        #endif
        break;
    default:
        return;
    }
#else
    (void)axis;
    (void)sensitivity;
#endif
}

static void store_and_apply_homing_sensitivity(const AxisEnum axis, const int16_t sensitivity) {
    store_homing_sensitivity(axis, sensitivity);
    apply_homing_sensitivity(axis, sensitivity);
}

static bool chopper_timing_is_valid(const uint8_t toff, const int8_t hend, const uint8_t hstrt) {
    return toff >= chopper_toff_spin_config.min_value && toff <= chopper_toff_spin_config.max_value
        && hend >= chopper_hend_spin_config.min_value && hend <= chopper_hend_spin_config.max_value
        && hstrt >= chopper_hstrt_spin_config.min_value && hstrt <= chopper_hstrt_spin_config.max_value;
}

static void apply_chopper_timing(const AxisEnum axis) {
#if PRINTER_IS_PRUSA_COREONE()
    uint8_t toff = config_store_ns::defaults::tmc_chopper_toff;
    int8_t hend = config_store_ns::defaults::tmc_chopper_hend;
    uint8_t hstrt = config_store_ns::defaults::tmc_chopper_hstrt;

    switch (axis) {
    case X_AXIS:
        toff = config_store().tmc_chopper_toff_x.get();
        hend = config_store().tmc_chopper_hend_x.get();
        hstrt = config_store().tmc_chopper_hstrt_x.get();
        break;
    case Y_AXIS:
        toff = config_store().tmc_chopper_toff_y.get();
        hend = config_store().tmc_chopper_hend_y.get();
        hstrt = config_store().tmc_chopper_hstrt_y.get();
        break;
    default:
        return;
    }

    if (!chopper_timing_is_valid(toff, hend, hstrt)) {
        return;
    }

    planner.synchronize();
    coreone_apply_xy_chopper_timing(axis, toff, hend, hstrt);
#else
    (void)axis;
#endif
}

static void store_chopper_timing_toff(const AxisEnum axis, const uint8_t toff) {
    if (toff < chopper_toff_spin_config.min_value || toff > chopper_toff_spin_config.max_value) {
        return;
    }

    switch (axis) {
    case X_AXIS:
        config_store().tmc_chopper_toff_x.set(toff);
        break;
    case Y_AXIS:
        config_store().tmc_chopper_toff_y.set(toff);
        break;
    default:
        return;
    }

    apply_chopper_timing(axis);
}

static void store_chopper_timing_hend(const AxisEnum axis, const int8_t hend) {
    if (hend < chopper_hend_spin_config.min_value || hend > chopper_hend_spin_config.max_value) {
        return;
    }

    switch (axis) {
    case X_AXIS:
        config_store().tmc_chopper_hend_x.set(hend);
        break;
    case Y_AXIS:
        config_store().tmc_chopper_hend_y.set(hend);
        break;
    default:
        return;
    }

    apply_chopper_timing(axis);
}

static void store_chopper_timing_hstrt(const AxisEnum axis, const uint8_t hstrt) {
    if (hstrt < chopper_hstrt_spin_config.min_value || hstrt > chopper_hstrt_spin_config.max_value) {
        return;
    }

    switch (axis) {
    case X_AXIS:
        config_store().tmc_chopper_hstrt_x.set(hstrt);
        break;
    case Y_AXIS:
        config_store().tmc_chopper_hstrt_y.set(hstrt);
        break;
    default:
        return;
    }

    apply_chopper_timing(axis);
}

MI_ADV_STEPS_PER_UNIT_X::MI_ADV_STEPS_PER_UNIT_X()
    : WiSpin(get_steps_per_unit_x(), steps_per_unit_spin_config, _("X-axis steps/mm")) {}

void MI_ADV_STEPS_PER_UNIT_X::OnClick() {
    store_and_apply_steps_per_unit(X_AXIS, GetVal());
}

MI_ADV_STEPS_PER_UNIT_Y::MI_ADV_STEPS_PER_UNIT_Y()
    : WiSpin(get_steps_per_unit_y(), steps_per_unit_spin_config, _("Y-axis steps/mm")) {}

void MI_ADV_STEPS_PER_UNIT_Y::OnClick() {
    store_and_apply_steps_per_unit(Y_AXIS, GetVal());
}

MI_ADV_STEPS_PER_UNIT_Z::MI_ADV_STEPS_PER_UNIT_Z()
    : WiSpin(get_steps_per_unit_z(), steps_per_unit_spin_config, _("Z-axis steps/mm")) {}

void MI_ADV_STEPS_PER_UNIT_Z::OnClick() {
    store_and_apply_steps_per_unit(Z_AXIS, GetVal());
}

MI_ADV_STEPS_PER_UNIT_E::MI_ADV_STEPS_PER_UNIT_E()
    : WiSpin(get_steps_per_unit_e(), steps_per_unit_spin_config, _("E-axis steps/mm")) {}

void MI_ADV_STEPS_PER_UNIT_E::OnClick() {
    store_and_apply_steps_per_unit(E_AXIS, GetVal());
}

MI_ADV_CURRENT_X::MI_ADV_CURRENT_X()
    : WiSpin(get_current_ma_x(), motor_current_spin_config, _("X-axis current")) {}

void MI_ADV_CURRENT_X::Store() {
    store_and_apply_motor_current(X_AXIS, static_cast<uint16_t>(GetVal()));
}

void MI_ADV_CURRENT_X::OnClick() {
    Store();
}

MI_ADV_CURRENT_Y::MI_ADV_CURRENT_Y()
    : WiSpin(get_current_ma_y(), motor_current_spin_config, _("Y-axis current")) {}

void MI_ADV_CURRENT_Y::Store() {
    store_and_apply_motor_current(Y_AXIS, static_cast<uint16_t>(GetVal()));
}

void MI_ADV_CURRENT_Y::OnClick() {
    Store();
}

MI_ADV_CURRENT_Z::MI_ADV_CURRENT_Z()
    : WiSpin(get_current_ma_z(), motor_current_spin_config, _("Z-axis current")) {}

void MI_ADV_CURRENT_Z::Store() {
    store_and_apply_motor_current(Z_AXIS, static_cast<uint16_t>(GetVal()));
}

void MI_ADV_CURRENT_Z::OnClick() {
    Store();
}

MI_ADV_CURRENT_E::MI_ADV_CURRENT_E()
    : WiSpin(get_current_ma_e(), motor_current_spin_config, _("E-axis current")) {}

void MI_ADV_CURRENT_E::Store() {
    store_and_apply_motor_current(E_AXIS, static_cast<uint16_t>(GetVal()));
}

void MI_ADV_CURRENT_E::OnClick() {
    Store();
}

MI_ADV_CURRENT_RESET_DEFAULTS::MI_ADV_CURRENT_RESET_DEFAULTS()
    : IWindowMenuItem(_("Reset to defaults")) {}

void MI_ADV_CURRENT_RESET_DEFAULTS::click([[maybe_unused]] IWindowMenu &window_menu) {
    Screens::Access()->Get()->WindowEvent(nullptr, GUI_event_t::CHILD_CLICK, reinterpret_cast<void *>(static_cast<intptr_t>(AdvancedSettingsClickCommand::Reset_motor_currents)));
}

MI_ADV_HOMING_SENS_X::MI_ADV_HOMING_SENS_X()
    : WiSpin(get_homing_sensitivity(X_AXIS), homing_sensitivity_spin_config, _("X-axis sensitivity")) {}

void MI_ADV_HOMING_SENS_X::Store() {
    store_and_apply_homing_sensitivity(X_AXIS, static_cast<int16_t>(GetVal()));
}

void MI_ADV_HOMING_SENS_X::OnClick() {
    Store();
}

MI_ADV_HOMING_SENS_Y::MI_ADV_HOMING_SENS_Y()
    : WiSpin(get_homing_sensitivity(Y_AXIS), homing_sensitivity_spin_config, _("Y-axis sensitivity")) {}

void MI_ADV_HOMING_SENS_Y::Store() {
    store_and_apply_homing_sensitivity(Y_AXIS, static_cast<int16_t>(GetVal()));
}

void MI_ADV_HOMING_SENS_Y::OnClick() {
    Store();
}

MI_ADV_HOMING_SENS_RESET_DEFAULTS::MI_ADV_HOMING_SENS_RESET_DEFAULTS()
    : IWindowMenuItem(_("Reset to defaults")) {}

void MI_ADV_HOMING_SENS_RESET_DEFAULTS::click([[maybe_unused]] IWindowMenu &window_menu) {
    Screens::Access()->Get()->WindowEvent(nullptr, GUI_event_t::CHILD_CLICK, reinterpret_cast<void *>(static_cast<intptr_t>(AdvancedSettingsClickCommand::Reset_homing_sensitivity)));
}

MI_ADV_LCD_TIMEOUT::MI_ADV_LCD_TIMEOUT()
    : WiSpin(config_store().lcd_backlight_timeout_min.get(), lcd_timeout_spin_config, _("LCD timeout")) {}

void MI_ADV_LCD_TIMEOUT::OnClick() {
    config_store().lcd_backlight_timeout_min.set(static_cast<uint8_t>(GetVal()));
}

MI_ADV_CHOPPER_TOFF_X::MI_ADV_CHOPPER_TOFF_X()
    : WiSpin(config_store().tmc_chopper_toff_x.get(), chopper_toff_spin_config, _("X TOFF")) {}

void MI_ADV_CHOPPER_TOFF_X::Store() {
    store_chopper_timing_toff(X_AXIS, static_cast<uint8_t>(GetVal()));
}

void MI_ADV_CHOPPER_TOFF_X::OnClick() {
    Store();
}

MI_ADV_CHOPPER_HEND_X::MI_ADV_CHOPPER_HEND_X()
    : WiSpin(config_store().tmc_chopper_hend_x.get(), chopper_hend_spin_config, _("X HEND")) {}

void MI_ADV_CHOPPER_HEND_X::Store() {
    store_chopper_timing_hend(X_AXIS, static_cast<int8_t>(GetVal()));
}

void MI_ADV_CHOPPER_HEND_X::OnClick() {
    Store();
}

MI_ADV_CHOPPER_HSTRT_X::MI_ADV_CHOPPER_HSTRT_X()
    : WiSpin(config_store().tmc_chopper_hstrt_x.get(), chopper_hstrt_spin_config, _("X HSTRT")) {}

void MI_ADV_CHOPPER_HSTRT_X::Store() {
    store_chopper_timing_hstrt(X_AXIS, static_cast<uint8_t>(GetVal()));
}

void MI_ADV_CHOPPER_HSTRT_X::OnClick() {
    Store();
}

MI_ADV_CHOPPER_TOFF_Y::MI_ADV_CHOPPER_TOFF_Y()
    : WiSpin(config_store().tmc_chopper_toff_y.get(), chopper_toff_spin_config, _("Y TOFF")) {}

void MI_ADV_CHOPPER_TOFF_Y::Store() {
    store_chopper_timing_toff(Y_AXIS, static_cast<uint8_t>(GetVal()));
}

void MI_ADV_CHOPPER_TOFF_Y::OnClick() {
    Store();
}

MI_ADV_CHOPPER_HEND_Y::MI_ADV_CHOPPER_HEND_Y()
    : WiSpin(config_store().tmc_chopper_hend_y.get(), chopper_hend_spin_config, _("Y HEND")) {}

void MI_ADV_CHOPPER_HEND_Y::Store() {
    store_chopper_timing_hend(Y_AXIS, static_cast<int8_t>(GetVal()));
}

void MI_ADV_CHOPPER_HEND_Y::OnClick() {
    Store();
}

MI_ADV_CHOPPER_HSTRT_Y::MI_ADV_CHOPPER_HSTRT_Y()
    : WiSpin(config_store().tmc_chopper_hstrt_y.get(), chopper_hstrt_spin_config, _("Y HSTRT")) {}

void MI_ADV_CHOPPER_HSTRT_Y::Store() {
    store_chopper_timing_hstrt(Y_AXIS, static_cast<uint8_t>(GetVal()));
}

void MI_ADV_CHOPPER_HSTRT_Y::OnClick() {
    Store();
}

MI_ADV_CHOPPER_RESET_DEFAULTS::MI_ADV_CHOPPER_RESET_DEFAULTS()
    : IWindowMenuItem(_("Reset to defaults")) {}

void MI_ADV_CHOPPER_RESET_DEFAULTS::click([[maybe_unused]] IWindowMenu &window_menu) {
    Screens::Access()->Get()->WindowEvent(nullptr, GUI_event_t::CHILD_CLICK, reinterpret_cast<void *>(static_cast<intptr_t>(AdvancedSettingsClickCommand::Reset_chopper_timing)));
}
