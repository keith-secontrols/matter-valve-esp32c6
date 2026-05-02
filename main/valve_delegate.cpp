#include "valve_delegate.h"

#include <app-common/zap-generated/attributes/Accessors.h>
#include <app/clusters/window-covering-server/window-covering-server.h>
#include <platform/CHIPDeviceLayer.h>
#include <system/SystemLayer.h>
#include <esp_log.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

namespace chip {
namespace app {
namespace Clusters {
namespace WindowCovering {

const char *ValveDelegate::TAG = "valve";

// ---------------------------------------------------------------------------
// Public
// ---------------------------------------------------------------------------

void ValveDelegate::init(led_strip_handle_t led)
{
    m_led = led;
    set_led(0, 0, 0);
}

void ValveDelegate::set_position_callback(valve_position_cb_t cb, void *user_data)
{
    m_cb        = cb;
    m_user_data = user_data;
}

CHIP_ERROR ValveDelegate::HandleMovement(WindowCoveringType type)
{
    if (type != WindowCoveringType::Lift) {
        return CHIP_NO_ERROR;
    }

    app::DataModel::Nullable<uint16_t> target_attr;
    Attributes::TargetPositionLiftPercent100ths::Get(mEndpoint, target_attr);
    if (target_attr.IsNull()) {
        return CHIP_NO_ERROR;
    }

    uint16_t target = target_attr.Value();
    if (target == m_current_pos) {
        return CHIP_NO_ERROR;
    }

    cancel_motor_task();

    int32_t delta      = (int32_t)target - (int32_t)m_current_pos;
    uint32_t travel_ms = (uint32_t)(abs(delta) * FULL_STROKE_MS / 10000);

    ESP_LOGI(TAG, "\033[1;37mMove %u%% -> %u%% open  (%lums)\033[0m",
             100 - m_current_pos / 100, 100 - target / 100, (unsigned long)travel_ms);

    MotorArgs *args = new MotorArgs{
        .self      = this,
        .start_pos = m_current_pos,
        .target_pos = target,
        .travel_ms = travel_ms,
        .opening   = (delta < 0),
    };

    m_stop_req   = false;
    m_move_start = xTaskGetTickCount();
    m_move_ms    = travel_ms;
    m_move_delta = delta;

    if (m_cb) {
        m_cb(target, m_user_data);  // TODO: will become H-bridge direction + enable
    }

    xTaskCreate(motor_task_fn, "motor", 4096, args, 5, &m_motor_task);
    return CHIP_NO_ERROR;
}

CHIP_ERROR ValveDelegate::HandleStopMotion()
{
    if (m_motor_task == nullptr) {
        return CHIP_NO_ERROR;
    }

    // Work out where the motor stopped based on elapsed time
    TickType_t elapsed_ticks = xTaskGetTickCount() - m_move_start;
    uint32_t   elapsed_ms    = elapsed_ticks * portTICK_PERIOD_MS;
    float      frac          = (m_move_ms > 0) ? ((float)elapsed_ms / m_move_ms) : 1.0f;
    if (frac > 1.0f) frac = 1.0f;

    uint16_t stopped_pos = (uint16_t)(m_current_pos + m_move_delta * frac);
    ESP_LOGI(TAG, "\033[1;37mStop at ~%u%% open\033[0m", 100 - stopped_pos / 100);

    m_stop_req = true;
    xTaskAbortDelay(m_motor_task);  // wake the sleeping task early

    return CHIP_NO_ERROR;
}

// Called by motor_task_fn when movement finishes (normally or stopped)
void ValveDelegate::on_move_done(uint16_t final_pos)
{
    m_current_pos   = final_pos;
    m_motor_task    = nullptr;
    EndpointId ep   = mEndpoint;

    ESP_LOGI(TAG, "\033[1;37mPosition: %u%% open\033[0m", 100 - final_pos / 100);

    // Update the Matter attribute on the Matter thread
    chip::DeviceLayer::SystemLayer().ScheduleLambda([ep, final_pos]() {
        LiftPositionSet(ep, app::DataModel::MakeNullable<uint16_t>(final_pos));
    });
}

// ---------------------------------------------------------------------------
// Private
// ---------------------------------------------------------------------------

void ValveDelegate::motor_task_fn(void *arg)
{
    MotorArgs *args = static_cast<MotorArgs *>(arg);
    ValveDelegate *self = args->self;

    // LED: red = opening, green = closing
    if (args->opening) {
        self->set_led(255, 0, 0);   // red
    } else {
        self->set_led(0, 255, 0);   // green
    }

    // Wait for the proportional travel time (abortable by HandleStopMotion)
    vTaskDelay(pdMS_TO_TICKS(args->travel_ms));

    self->set_led(0, 0, 0);  // LED off

    // TODO: disable H-bridge here

    // Determine final position
    uint16_t final_pos;
    if (self->m_stop_req) {
        // Interpolate based on elapsed time
        TickType_t elapsed = xTaskGetTickCount() - self->m_move_start;
        uint32_t   elapsed_ms = elapsed * portTICK_PERIOD_MS;
        float frac = (args->travel_ms > 0) ? ((float)elapsed_ms / args->travel_ms) : 1.0f;
        if (frac > 1.0f) frac = 1.0f;
        final_pos = (uint16_t)(args->start_pos + (int32_t)(args->target_pos - args->start_pos) * frac);
    } else {
        final_pos = args->target_pos;
    }

    delete args;
    self->on_move_done(final_pos);
    vTaskDelete(nullptr);
}

void ValveDelegate::cancel_motor_task()
{
    if (m_motor_task == nullptr) return;
    m_stop_req = true;
    xTaskAbortDelay(m_motor_task);
    // Give the task a tick to clean up and delete itself
    vTaskDelay(pdMS_TO_TICKS(10));
    m_motor_task = nullptr;
}

void ValveDelegate::set_led(uint8_t r, uint8_t g, uint8_t b)
{
    if (m_led == nullptr) return;
    led_strip_set_pixel(m_led, 0, r, g, b);
    led_strip_refresh(m_led);
}

} // namespace WindowCovering
} // namespace Clusters
} // namespace app
} // namespace chip
