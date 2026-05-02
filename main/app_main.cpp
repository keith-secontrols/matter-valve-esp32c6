/*
 * Matter water valve actuator with temperature sensor
 * Device types:
 *   Endpoint 1: Window Covering (used for valve position 0-100%)
 *   Endpoint 2: Temperature Sensor
 *
 * Window Covering position convention (Matter spec):
 *   0     = fully open  (valve fully open,   100% flow)
 *   10000 = fully closed (valve fully closed, 0% flow)
 *
 * Alexa: "open 50%" -> target position 5000
 */

#include <app/server/CommissioningWindowManager.h>
#include <app/server/Server.h>
#include <esp_err.h>
#include <esp_log.h>
#include <esp_matter.h>
#include <esp_matter_ota.h>
#include <nvs_flash.h>

#include <app/clusters/window-covering-server/window-covering-server.h>
#include <bsp/esp-bsp.h>
#include <led_strip.h>
#include <led_strip_rmt.h>

#include <app_openthread_config.h>
#include <app_reset.h>
#include <common_macros.h>

#include "valve_delegate.h"

static const char *TAG = "app_main";

using namespace esp_matter;
using namespace esp_matter::attribute;
using namespace esp_matter::endpoint;
using namespace chip::app::Clusters;

// ---------------------------------------------------------------------------
// Hardware -- LED strip init (WS2812 on GPIO8)
// ---------------------------------------------------------------------------

static led_strip_handle_t s_led_strip = nullptr;

static void led_strip_init()
{
    led_strip_config_t cfg = {
        .strip_gpio_num   = 8,
        .max_leds         = 1,
        .led_model        = LED_MODEL_WS2812,
        .color_component_format = LED_STRIP_COLOR_COMPONENT_FMT_GRB,
        .flags = { .invert_out = false },
    };
    led_strip_rmt_config_t rmt_cfg = {
        .clk_src       = RMT_CLK_SRC_DEFAULT,
        .resolution_hz = 10 * 1000 * 1000,
        .mem_block_symbols = 64,
        .flags = { .with_dma = false },
    };
    ESP_ERROR_CHECK(led_strip_new_rmt_device(&cfg, &rmt_cfg, &s_led_strip));
    led_strip_clear(s_led_strip);
}

// ---------------------------------------------------------------------------
// Hardware stub -- H-bridge direction (TODO: add GPIO pins later)
// ---------------------------------------------------------------------------

static void valve_set_position(uint16_t position_percent100ths, void *user_data)
{
    // TODO: set H-bridge direction and enable here
    // Direction: position_percent100ths < current means opening (reverse)
    //            position_percent100ths > current means closing (forward)
}

// ---------------------------------------------------------------------------
// Temperature readback -- call this from your sensor task/timer
// ---------------------------------------------------------------------------

static uint16_t s_temp_endpoint_id = 0;

void app_update_temperature(float temp_celsius)
{
    if (s_temp_endpoint_id == 0) return;

    chip::DeviceLayer::SystemLayer().ScheduleLambda([temp_celsius]() {
        attribute_t *attr = attribute::get(s_temp_endpoint_id,
                                           TemperatureMeasurement::Id,
                                           TemperatureMeasurement::Attributes::MeasuredValue::Id);
        esp_matter_attr_val_t val = esp_matter_invalid(NULL);
        attribute::get_val(attr, &val);
        val.val.i16 = static_cast<int16_t>(temp_celsius * 100);
        attribute::update(s_temp_endpoint_id,
                          TemperatureMeasurement::Id,
                          TemperatureMeasurement::Attributes::MeasuredValue::Id,
                          &val);
    });
}

// ---------------------------------------------------------------------------
// Matter callbacks
// ---------------------------------------------------------------------------

static esp_err_t factory_reset_button_register()
{
    button_handle_t push_button;
    esp_err_t err = bsp_iot_button_create(&push_button, NULL, BSP_BUTTON_NUM);
    VerifyOrReturnError(err == ESP_OK, err);
    return app_reset_button_register(push_button);
}

static void open_commissioning_window_if_necessary()
{
    VerifyOrReturn(chip::Server::GetInstance().GetFabricTable().FabricCount() == 0);
    chip::CommissioningWindowManager &mgr = chip::Server::GetInstance().GetCommissioningWindowManager();
    VerifyOrReturn(mgr.IsCommissioningWindowOpen() == false);
    CHIP_ERROR err = mgr.OpenBasicCommissioningWindow(chip::System::Clock::Seconds16(300),
                                                      chip::CommissioningWindowAdvertisement::kDnssdOnly);
    if (err != CHIP_NO_ERROR) {
        ESP_LOGE(TAG, "Failed to open commissioning window: %" CHIP_ERROR_FORMAT, err.Format());
    }
}

static void app_event_cb(const ChipDeviceEvent *event, intptr_t arg)
{
    switch (event->Type) {
    case chip::DeviceLayer::DeviceEventType::kCommissioningComplete:
        ESP_LOGI(TAG, "Commissioning complete");
        break;
    case chip::DeviceLayer::DeviceEventType::kFailSafeTimerExpired:
        ESP_LOGI(TAG, "Commissioning failed, fail safe timer expired");
        break;
    case chip::DeviceLayer::DeviceEventType::kFabricRemoved:
        ESP_LOGI(TAG, "Fabric removed");
        open_commissioning_window_if_necessary();
        break;
    case chip::DeviceLayer::DeviceEventType::kBLEDeinitialized:
        ESP_LOGI(TAG, "BLE deinitialized");
        break;
    default:
        break;
    }
}

static esp_err_t app_identification_cb(identification::callback_type_t type, uint16_t endpoint_id,
                                       uint8_t effect_id, uint8_t effect_variant, void *priv_data)
{
    ESP_LOGI(TAG, "Identification callback: type:%u effect:%u variant:%u", type, effect_id, effect_variant);
    return ESP_OK;
}

static esp_err_t app_attribute_update_cb(attribute::callback_type_t type, uint16_t endpoint_id,
                                         uint32_t cluster_id, uint32_t attribute_id,
                                         esp_matter_attr_val_t *val, void *priv_data)
{
    // Window covering position is handled via the delegate (HandleMovement).
    // Temperature is read-only from the network side.
    return ESP_OK;
}

// ---------------------------------------------------------------------------
// Main
// ---------------------------------------------------------------------------

static chip::app::Clusters::WindowCovering::ValveDelegate s_valve_delegate;

extern "C" void app_main()
{
    nvs_flash_init();
    led_strip_init();

    esp_err_t err = factory_reset_button_register();
    ABORT_APP_ON_FAILURE(err == ESP_OK, ESP_LOGE(TAG, "Failed to init reset button: %d", err));

    // Create Matter node
    node::config_t node_config;
    node_t *node = node::create(&node_config, app_attribute_update_cb, app_identification_cb);
    ABORT_APP_ON_FAILURE(node != nullptr, ESP_LOGE(TAG, "Failed to create Matter node"));

    // --- Endpoint 1: Window Covering (valve position) ---
    window_covering::config_t wc_config;
    // Enable position-aware lift feature (required for percentage control)
    wc_config.window_covering.feature_flags =
        chip::to_underlying(chip::app::Clusters::WindowCovering::Feature::kLift) |
        chip::to_underlying(chip::app::Clusters::WindowCovering::Feature::kPositionAwareLift);
    wc_config.window_covering.delegate = &s_valve_delegate;

    endpoint_t *wc_ep = window_covering::create(node, &wc_config, ENDPOINT_FLAG_NONE, NULL);
    ABORT_APP_ON_FAILURE(wc_ep != nullptr, ESP_LOGE(TAG, "Failed to create window_covering endpoint"));

    // Init delegate. SetDefaultDelegate is called by the framework during esp_matter::start()
    // but does NOT call SetEndpoint, so we must do it explicitly here.
    s_valve_delegate.init(s_led_strip);
    s_valve_delegate.set_position_callback(valve_set_position, NULL);
    s_valve_delegate.SetEndpoint(endpoint::get_id(wc_ep));

    // --- Endpoint 2: Temperature Sensor ---
    temperature_sensor::config_t temp_config;
    endpoint_t *temp_ep = temperature_sensor::create(node, &temp_config, ENDPOINT_FLAG_NONE, NULL);
    ABORT_APP_ON_FAILURE(temp_ep != nullptr, ESP_LOGE(TAG, "Failed to create temperature_sensor endpoint"));
    s_temp_endpoint_id = endpoint::get_id(temp_ep);

#if CHIP_DEVICE_CONFIG_ENABLE_THREAD
    esp_openthread_platform_config_t config = {
        .radio_config = ESP_OPENTHREAD_DEFAULT_RADIO_CONFIG(),
        .host_config = ESP_OPENTHREAD_DEFAULT_HOST_CONFIG(),
        .port_config = ESP_OPENTHREAD_DEFAULT_PORT_CONFIG(),
    };
    set_openthread_platform_config(&config);
#endif

    err = esp_matter::start(app_event_cb);
    ABORT_APP_ON_FAILURE(err == ESP_OK, ESP_LOGE(TAG, "Failed to start Matter: %d", err));
}
