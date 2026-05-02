#pragma once

#include <app/clusters/window-covering-server/window-covering-delegate.h>
#include <led_strip.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

// Called when a new target position is commanded.
// position_percent100ths: 0 = fully open, 10000 = fully closed
typedef void (*valve_position_cb_t)(uint16_t position_percent100ths, void *user_data);

namespace chip {
namespace app {
namespace Clusters {
namespace WindowCovering {

class ValveDelegate : public Delegate {
public:
    ValveDelegate() = default;

    void init(led_strip_handle_t led);
    void set_position_callback(valve_position_cb_t cb, void *user_data);

    // Called from motor task when movement finishes or is stopped
    void on_move_done(uint16_t final_pos);

    CHIP_ERROR HandleMovement(WindowCoveringType type) override;
    CHIP_ERROR HandleStopMotion() override;

private:
    struct MotorArgs {
        ValveDelegate *self;
        uint16_t      start_pos;
        uint16_t      target_pos;
        uint32_t      travel_ms;
        bool          opening;  // true = moving toward open (0), false = closing
    };

    static void motor_task_fn(void *arg);
    void        cancel_motor_task();
    void        set_led(uint8_t r, uint8_t g, uint8_t b);

    led_strip_handle_t m_led         = nullptr;
    TaskHandle_t       m_motor_task  = nullptr;
    bool               m_stop_req    = false;
    uint16_t           m_current_pos = 0;    // tracks physical position
    TickType_t         m_move_start  = 0;
    uint32_t           m_move_ms     = 0;
    int32_t            m_move_delta  = 0;

    valve_position_cb_t m_cb        = nullptr;
    void               *m_user_data = nullptr;

    static constexpr uint32_t FULL_STROKE_MS = 10000;
    static const char        *TAG;
};

} // namespace WindowCovering
} // namespace Clusters
} // namespace app
} // namespace chip
