// gui.cpp
#include <stdlib.h>

#include "display.hpp"
#include "gui.hpp"
#include "gui_time.hpp" //gui::GetTick
#include "ScreenHandler.hpp"
#include "sound.hpp"
#include "IDialog.hpp"
#include "Jogwheel.hpp"
#include "ScreenShot.hpp"
#include "gui_media_events.hpp"
#include "gui_invalidate.hpp"
#include "knob_event.hpp"
#include "marlin_client.hpp"
#include "marlin_vars.hpp"
#include <utils/timing/rate_limiter.hpp>
#include <logging/log.hpp>
#include "display_hw_checks.hpp"
#include <option/has_leds.h>
#if HAS_LEDS()
    #include <leds/led_manager.hpp>
#endif

#include <option/has_touch.h>

#if HAS_TOUCH()
    #include <hw/touchscreen/touchscreen.hpp>
#endif

#include <config_store/store_instance.hpp>
#include <guiconfig/guiconfig.h>

#if HAS_MINI_DISPLAY()
    #include "st7789v.hpp"
#endif

#if HAS_SELFTEST()
    #include <gui/screen_menu_selftest_snake.hpp>
#endif

LOG_COMPONENT_REF(GUI);
LOG_COMPONENT_REF(Touch);

static bool gui_invalid = false;

constexpr padding_ui8_t GuiDefaults::Padding;
constexpr Rect16 GuiDefaults::RectHeader;
constexpr Rect16 GuiDefaults::RectScreenBody;
constexpr Rect16 GuiDefaults::RectScreen;
constexpr Rect16 GuiDefaults::RectScreenNoFoot;
constexpr Rect16 GuiDefaults::RectScreenNoHeader;
constexpr Rect16 GuiDefaults::RectFooter;

static const constexpr uint32_t GUI_DELAY_MIN = 1;
static const constexpr uint32_t GUI_DELAY_MAX = 10;
static const constexpr uint8_t GUI_DELAY_LOOP = 100;
static const constexpr uint32_t GUI_DELAY_REDRAW = 40; // 40 ms => 25 fps

static RateLimiter<uint32_t> gui_roll_timer(txtroll_t::GetBaseTick());
static RateLimiter<uint32_t> gui_loop_timer(GUI_DELAY_LOOP);
static RateLimiter<uint32_t> gui_redraw_timer(GUI_DELAY_REDRAW);

#if HAS_LEDS() && (PRINTER_IS_PRUSA_COREONE() || PRINTER_IS_PRUSA_COREONEL())
static constexpr int32_t LCD_IDLE_BACKLIGHT_TIMEOUT_MS = 5 * 60 * 1000;
static constexpr uint8_t LCD_BACKLIGHT_ACTIVE_BRIGHTNESS = 100;
static constexpr uint8_t LCD_BACKLIGHT_OFF_BRIGHTNESS = 0;

static uint32_t lcd_backlight_last_activity_ms = 0;
static bool lcd_backlight_active = true;
static marlin_server::State lcd_backlight_last_print_state = marlin_server::State::Idle;
static uint16_t lcd_backlight_last_job_id = 0;
static uint8_t lcd_backlight_last_media_inserted = 0;

static void lcd_backlight_apply_brightness() {
    leds::LEDManager::instance().set_lcd_brightness(lcd_backlight_active ? LCD_BACKLIGHT_ACTIVE_BRIGHTNESS : LCD_BACKLIGHT_OFF_BRIGHTNESS);
}

static void lcd_backlight_set_active(const bool active) {
    if (lcd_backlight_active == active) {
        lcd_backlight_apply_brightness();
        return;
    }

    lcd_backlight_active = active;
    lcd_backlight_apply_brightness();
}

static void lcd_backlight_activity_ping() {
    lcd_backlight_last_activity_ms = ticks_ms();
    lcd_backlight_set_active(true);
}

static bool lcd_backlight_is_active() {
    return lcd_backlight_active;
}

static bool lcd_backlight_printer_busy() {
    return marlin_client::is_printing();
}

static void lcd_backlight_init_activity() {
    lcd_backlight_last_activity_ms = ticks_ms();
    lcd_backlight_last_print_state = marlin_vars().print_state.get();
    lcd_backlight_last_job_id = marlin_vars().job_id.get();
    lcd_backlight_last_media_inserted = marlin_vars().media_inserted.get();
    lcd_backlight_set_active(true);
}

static void lcd_backlight_handle_idle_timeout() {
    const uint32_t now = ticks_ms();
    const auto print_state = marlin_vars().print_state.get();
    const uint16_t job_id = marlin_vars().job_id.get();
    const uint8_t media_inserted = marlin_vars().media_inserted.get();

    if (print_state != lcd_backlight_last_print_state || job_id != lcd_backlight_last_job_id || media_inserted != lcd_backlight_last_media_inserted) {
        lcd_backlight_last_print_state = print_state;
        lcd_backlight_last_job_id = job_id;
        lcd_backlight_last_media_inserted = media_inserted;
        lcd_backlight_activity_ping();
        return;
    }

    if (lcd_backlight_printer_busy()) {
        lcd_backlight_activity_ping();
        return;
    }

    if (lcd_backlight_active && ticks_diff(now, lcd_backlight_last_activity_ms) >= LCD_IDLE_BACKLIGHT_TIMEOUT_MS) {
        lcd_backlight_set_active(false);
    }
}
#endif

static void gui_handle_leds_and_lcd_backlight() {
#if HAS_LEDS()
    leds::LEDManager::instance().update();
    #if PRINTER_IS_PRUSA_COREONE() || PRINTER_IS_PRUSA_COREONEL()
    lcd_backlight_handle_idle_timeout();
    #endif
#endif
}

void gui_init(void) {
    display::init();

// select jogwheel type by measured 'reset delay'
// original displays with 15 position encoder returns values 1-2 (short delay - no capacitor)
// new displays with MK3 encoder returns values around 16000 (long delay - 100nF capacitor)
#if HAS_MINI_DISPLAY()
    // run-time jogwheel type detection decides which type of jogwheel device has (each type has different encoder behaviour)
    jogwheel.SetJogwheelType(st7789v_reset_delay);
#else
    jogwheel.SetJogwheelType(0);
#endif

#if HAS_LEDS() && (PRINTER_IS_PRUSA_COREONE() || PRINTER_IS_PRUSA_COREONEL())
    lcd_backlight_init_activity();
#endif
}

void gui_handle_jogwheel() {
    BtnState_t btn_ev;
    bool is_btn = jogwheel.ConsumeButtonEvent(btn_ev);
    int32_t encoder_diff = jogwheel.ConsumeEncoderDiff();

    if (encoder_diff != 0 || is_btn) {
#if HAS_LEDS() && (PRINTER_IS_PRUSA_COREONE() || PRINTER_IS_PRUSA_COREONEL())
        const bool backlight_was_active = lcd_backlight_is_active();
        lcd_backlight_activity_ping();
        if (!backlight_was_active) {
            return;
        }
#endif
        gui::knob::EventEncoder(encoder_diff);

        if (is_btn) {
            gui::knob::EventClick(btn_ev);
        }
    }
}

#if HAS_TOUCH()
void gui_handle_touch() {
    if (!touchscreen.is_enabled()) {
        return;
    }

    const auto touch_event = touchscreen.get_event();
    if (!touch_event) {
        return;
    }

#if HAS_LEDS() && (PRINTER_IS_PRUSA_COREONE() || PRINTER_IS_PRUSA_COREONEL())
    const bool backlight_was_active = lcd_backlight_is_active();
    lcd_backlight_activity_ping();
    if (!backlight_was_active) {
        return;
    }
#endif

    // we clicked on something, does not really matter on what we clicked
    // we must notify serve to so it knows user is doing something and resets menu timeout, heater timeout ...
    Screens::Access()->ResetTimeout();

    if (touch_event.type == GUI_event_t::TOUCH_CLICK) {
        Sound_Play(eSOUND_TYPE::ButtonEcho);
        marlin_client::notify_server_about_knob_click();
    }

    event_conversion_union event_data {
        .point = {
            .x = touch_event.pos_x,
            .y = touch_event.pos_y,
        }
    };

    // Determine if we should propagate the event only to the captured window or globally as a screen event
    const bool propagate_as_screen_event = (touch_event.type != GUI_event_t::TOUCH_CLICK);

    if (propagate_as_screen_event) {
        Screens::Access()->ScreenEvent(nullptr, touch_event.type, event_data.pvoid);
    }

    else if (window_t *captured_window = Screens::Access()->Get()->GetCapturedWindow(); captured_window && captured_window->get_rect_for_touch().Contain(event_data.point)) {
        captured_window->WindowEvent(captured_window, touch_event.type, event_data.pvoid);
    }
}
#endif

void gui_redraw(void) {
    const uint32_t now = ticks_ms();

    if (gui_loop_timer.check(now)) {
        Screens::Access()->ScreenEvent(nullptr, GUI_event_t::LOOP, 0);
    }

    if (txtroll_t::HasInstance() && gui_roll_timer.check(now)) {
        Screens::Access()->ScreenEvent(nullptr, GUI_event_t::TEXT_ROLL, nullptr);
    }

    bool should_sleep = true;
    if (gui_invalid) {
        if (gui_redraw_timer.check(now)) {
            Screens::Access()->Draw();
            gui_invalid = false;
            should_sleep = false;
        }
    }

    if (should_sleep) {
        uint32_t sleep = std::clamp(gui_redraw_timer.remaining_cooldown(now), GUI_DELAY_MIN, GUI_DELAY_MAX);
        osDelay(sleep);
    }
}

// at least one window is invalid
void gui_invalidate(void) {
    gui_invalid = true;
}

static uint8_t guiloop_nesting = 0;
uint8_t gui_get_nesting(void) { return guiloop_nesting; }

void gui_bare_loop() {
    ++guiloop_nesting;

    gui_handle_jogwheel();

    gui_handle_leds_and_lcd_backlight();

    gui_redraw();

    --guiloop_nesting;
}

void gui_loop(void) {
    ++guiloop_nesting;
    lcd::communication_check();
    gui_handle_jogwheel();

    gui_handle_leds_and_lcd_backlight();

#if HAS_TOUCH()
    gui_handle_touch();
#endif

    MediaState_t media_state = MediaState_t::unknown;
    if (GuiMediaEventsHandler::ConsumeSent(media_state)) {
        switch (media_state) {
        case MediaState_t::inserted:
        case MediaState_t::removed:
        case MediaState_t::error:
            Screens::Access()->ScreenEvent(nullptr, GUI_event_t::MEDIA, (void *)int(media_state));
            break;
        default:
            break;
        }
    }

    gui_redraw();
    marlin_client::loop();
    GuiMediaEventsHandler::Tick();
#if HAS_SELFTEST()
    if (marlin_client::event_clr(marlin_server::Event::RequestCalibrationsScreen)) {
        Screens::Access()->Open<ScreenMenuSTSCalibrations>();
    }
#endif
    --guiloop_nesting;
}
