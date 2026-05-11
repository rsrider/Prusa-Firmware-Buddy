#pragma once

#include <utils/timing/rate_limiter.hpp>

namespace leds {

/**
 * @brief A class encapsulating handling of LEDs and other peripherals
 * connected to LED interfaces, like LCD backlight and XL enclosure fan.
 *
 * Takes care of updating the LED states and animations and passing the data
 * over to the Neopixel/ws2812 interfaces.
 */
class LEDManager {
public:
    static LEDManager &instance();

    void init();

    void update();

    /**
     * @brief Called from the power panic module to quickly turn off leds from the AC fault task.
     */
    void enter_power_panic();

    /**
     * @param brighthess Brightness in percents (1-100)
     */
    void set_lcd_brightness(uint8_t brightness);

private:
    void apply_lcd_brightness();

    static constexpr uint32_t gui_delay_redraw = 40; // 40 ms => 25 fps
    RateLimiter<uint32_t> rate_limiter { gui_delay_redraw };
    freertos::Mutex power_panic_mutex;
    bool power_panic { false };
    uint8_t lcd_brightness { 100 };
};

}; // namespace leds
