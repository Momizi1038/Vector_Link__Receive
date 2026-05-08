/**
 * Copyright (c) 2023 Raspberry Pi (Trading) Ltd.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 *
 * [修正版 v2] BLE GATTクライアント - Picow Controller受信側
 *
 * 修正内容 (v2追加):
 *  7. Connection Update完了後、サービス探索開始まで200ms待機
 *     → ATT 0x7F (UNLIKELY_ERROR) の回避
 *     → 送信側heartbeatの1msタイマー圧迫による応答遅延を吸収
 *  8. Connection Updateを行わないオプションを追加
 *     → まずUpdate無しで接続・サービス探索を確認してから有効化する
 */

#include <stdio.h>
#include "btstack.h"
#include "pico/cyw43_arch.h"
#include "pico/stdlib.h"

#include "type.h"

// -------------------------------------------------------
// UUID定義
// Peripheral側 ds4_data_gatt.h の定義に合わせること
// -------------------------------------------------------

// Picow Controller カスタムサービス UUID (128bit, Little Endian)
// "6f8b2c10-9d3a-4b1f-8c7e-2a4d5e9f1a3b"
static const uint8_t picow_service_uuid[16] = {
    0x6f, 0x8b, 0x2c, 0x10, 0x9d, 0x3a, 0x4b, 0x1f,
    0x8c, 0x7e, 0x2a, 0x4d, 0x5e, 0x9f, 0x1a, 0x3b
};

// ControllerData Characteristic UUID (128bit, Little Endian)
// Peripheral側のGATTファイルと必ず一致させること
// "6f8b2c10-9d3a-4b1f-8c7e-2a4d5e9f1a3b" ← Peripheral側と同じUUIDの場合はこちら
static const uint8_t controller_data_uuid[16] = {
    0x6f, 0x8b, 0x2c, 0x10, 0x9d, 0x3a, 0x4b, 0x1f,
    0x8c, 0x7e, 0x2a, 0x4d, 0x5e, 0x9f, 0x1a, 0x3b
};

// -------------------------------------------------------
// デバッグログ制御
// -------------------------------------------------------
#if 1
#define DEBUG_LOG(...) printf(__VA_ARGS__)
#else
#define DEBUG_LOG(...)
#endif

// -------------------------------------------------------
// タイミング設定
// -------------------------------------------------------
#define LED_QUICK_FLASH_DELAY_MS    100
#define LED_SLOW_FLASH_DELAY_MS    1000

// 接続後Connection Updateを送るまでの待機時間
#define CONNECTION_UPDATE_DELAY_MS  500

// Connection Update完了後、サービス探索を始めるまでの待機時間
// 送信側のランループ負荷が落ち着くまで待つ
// [修正7] 0ms → 200ms に変更（ATT 0x7F対策）
#define SERVICE_DISCOVERY_DELAY_MS  500

// RSSIサンプリング間隔（N受信パケットに1回）
#define RSSI_SAMPLE_INTERVAL         30

// -------------------------------------------------------
// ステートマシン定義
// -------------------------------------------------------
typedef enum {
    TC_OFF,
    TC_IDLE,
    TC_W4_SCAN_RESULT,
    TC_W4_CONNECT,
    TC_W4_CONNECTION_UPDATE_COMPLETE,
    TC_W4_SERVICE_DISCOVERY_DELAY,   // [修正7] 探索前待機ステート追加
    TC_W4_SERVICE_RESULT,
    TC_W4_CHARACTERISTIC_RESULT,
    TC_W4_ENABLE_NOTIFICATIONS_COMPLETE,
    TC_W4_READY
} gc_state_t;

// -------------------------------------------------------
// グローバル変数
// -------------------------------------------------------
static btstack_packet_callback_registration_t hci_event_callback_registration;
static gc_state_t state = TC_OFF;

static bd_addr_t        server_addr;
static bd_addr_type_t   server_addr_type;
static hci_con_handle_t connection_handle;

static gatt_client_service_t        server_service;
static gatt_client_characteristic_t server_characteristic;

static bool listener_registered = false;
static gatt_client_notification_t notification_listener;

static btstack_timer_source_t heartbeat;
static btstack_timer_source_t connection_update_timer;   // Connection Update遅延用
static btstack_timer_source_t service_discovery_timer;  // [修正7] 探索開始遅延用

static gatt_client_service_t Controller_Service;
static bool controller_service_found    = false;
static bool notify_characteristic_found = false;

static int Rssi_server  = 0;
static int rssi_counter = 0;

static void handle_gatt_client_event(uint8_t packet_type, uint16_t channel,uint8_t *packet, uint16_t size);

// -------------------------------------------------------
// スキャン開始
// -------------------------------------------------------
static void client_start(void) {
    DEBUG_LOG("[BLE] Start scanning...\n");
    state = TC_W4_SCAN_RESULT;
    gap_set_scan_parameters(0, 0x0030, 0x0030);
    gap_start_scan();
}

// -------------------------------------------------------
// アドバタイズパケット内の名前チェック
// -------------------------------------------------------
static bool advertisement_report_contains_name(const char *name, uint8_t *advertisement_report) {
    const uint8_t *adv_data = gap_event_advertising_report_get_data(advertisement_report);
    uint8_t        adv_len  = gap_event_advertising_report_get_data_length(advertisement_report);

    ad_context_t context;
    for (ad_iterator_init(&context, adv_len, adv_data);
         ad_iterator_has_more(&context);
         ad_iterator_next(&context)) {

        uint8_t        data_type = ad_iterator_get_data_type(&context);
        uint8_t        data_size = ad_iterator_get_data_len(&context);
        const uint8_t *data      = ad_iterator_get_data(&context);

        if (data_type == BLUETOOTH_DATA_TYPE_COMPLETE_LOCAL_NAME ||
            data_type == BLUETOOTH_DATA_TYPE_SHORTENED_LOCAL_NAME) {

            printf("[ADV] Name (len=%d): ", data_size);
            for (int i = 0; i < data_size; i++) printf("%c", data[i]);
            printf("\n");

            if (data_size == (uint8_t)strlen(name) &&
                memcmp(data, name, data_size) == 0) {
                printf("[ADV] MATCH: '%s'\n", name);
                return true;
            }
        }
    }
    return false;
}

// -------------------------------------------------------
// [修正7] サービス探索開始を遅延させるタイマーコールバック
// Connection Update完了直後はATTサーバーが応答できない場合があるため
// 200ms待ってから探索を開始する
// -------------------------------------------------------
static void service_discovery_timer_handler(btstack_timer_source_t *ts) {
    UNUSED(ts);
    printf("[GATT] Starting primary service discovery (after delay)...\n");
    printf("[STATE] TC_W4_SERVICE_DISCOVERY_DELAY -> TC_W4_SERVICE_RESULT\n");
    state = TC_W4_SERVICE_RESULT;
    controller_service_found    = false;
    notify_characteristic_found = false;
    gatt_client_discover_primary_services(handle_gatt_client_event, connection_handle);
}

// -------------------------------------------------------
// Connection Updateを遅延送信するタイマーコールバック
// -------------------------------------------------------
// [修正8] CONNECTION_UPDATE_DISABLE が 1 の場合はスキップ
#define CONNECTION_UPDATE_DISABLE  1  // 0=有効, 1=無効(テスト用)

static void connection_update_timer_handler(btstack_timer_source_t *ts) {
    UNUSED(ts);
    if (CONNECTION_UPDATE_DISABLE) {
        printf("[BLE] Connection Update is DISABLED (skipping)\n");
        printf("[GATT] Transitioning to service discovery delay...\n");
        state = TC_W4_SERVICE_DISCOVERY_DELAY;
        btstack_run_loop_set_timer(&service_discovery_timer, SERVICE_DISCOVERY_DELAY_MS);
        btstack_run_loop_add_timer(&service_discovery_timer);
        return;
    }
    printf("[BLE] Sending connection parameter update (interval=15ms)\n");
    hci_send_cmd(&hci_le_connection_update,
                 connection_handle,
                 8, 8,    // interval_min / interval_max (12 × 1.25ms = 15ms)
                 0,       // slave_latency
                 100,     // supervision_timeout (100 × 10ms = 1000ms)
                 0, 0xFFFF);
}

// -------------------------------------------------------
// GATT クライアントイベントハンドラ
// -------------------------------------------------------
static void handle_gatt_client_event(uint8_t packet_type, uint16_t channel,
                                     uint8_t *packet, uint16_t size) {
    UNUSED(packet_type);
    UNUSED(channel);
    UNUSED(size);

    uint8_t att_status;

    switch (state) {

        // --------------------------------------------------
        case TC_W4_SERVICE_RESULT:
        // --------------------------------------------------
            switch (hci_event_packet_get_type(packet)) {

                case GATT_EVENT_SERVICE_QUERY_RESULT: {
                    gatt_client_service_t service;
                    gatt_event_service_query_result_get_service(packet, &service);

                    printf("[GATT] Service found: handle=0x%04x-0x%04x uuid16=0x%04x\n",
                           service.start_group_handle,
                           service.end_group_handle,
                           service.uuid16);

                    if (service.uuid16 == 0 &&
                        memcmp(service.uuid128, picow_service_uuid, 16) == 0) {
                        Controller_Service       = service;
                        controller_service_found = true;
                        printf("[GATT] ✓ Found Controller service!\n");
                    }
                    break;
                }

                case GATT_EVENT_QUERY_COMPLETE:
                    att_status = gatt_event_query_complete_get_att_status(packet);
                    if (att_status != ATT_ERROR_SUCCESS) {
                        printf("[GATT] Service query error: 0x%02x\n", att_status);
                        // [修正7] エラー時は少し待ってから再試行（即切断しない）
                        printf("[GATT] Retrying service discovery in 500ms...\n");
                        state = TC_W4_SERVICE_DISCOVERY_DELAY;
                        btstack_run_loop_set_timer(&service_discovery_timer, 500);
                        btstack_run_loop_add_timer(&service_discovery_timer);
                        break;
                    }

                    if (!controller_service_found) {
                        printf("[GATT] Controller service not found, disconnecting\n");
                        gap_disconnect(connection_handle);
                        break;
                    }

                    state = TC_W4_CHARACTERISTIC_RESULT;
                    notify_characteristic_found = false;
                    printf("[GATT] Discovering characteristics...\n");
                    gatt_client_discover_characteristics_for_service(
                        handle_gatt_client_event,
                        connection_handle,
                        &Controller_Service);
                    break;

                default:
                    break;
            }
            break;

        // --------------------------------------------------
        case TC_W4_CHARACTERISTIC_RESULT:
        // --------------------------------------------------
            switch (hci_event_packet_get_type(packet)) {

                case GATT_EVENT_CHARACTERISTIC_QUERY_RESULT: {
                    gatt_client_characteristic_t ch;
                    gatt_event_characteristic_query_result_get_characteristic(packet, &ch);

                    printf("[GATT] Characteristic: handle=0x%04x props=0x%02x uuid16=0x%04x\n",
                           ch.value_handle, ch.properties, ch.uuid16);

                    bool has_notify = (ch.properties & ATT_PROPERTY_NOTIFY) != 0;

                    // UUID128比較（uuid16==0 の場合）またはNotify対応の最初のCharacteristicを採用
                    bool is_target_uuid = false;
                    if (ch.uuid16 == 0) {
                        is_target_uuid = (memcmp(ch.uuid128, controller_data_uuid, 16) == 0);
                    }

                    if (has_notify && (is_target_uuid || !notify_characteristic_found)) {
                        server_characteristic       = ch;
                        notify_characteristic_found = true;
                        printf("[GATT] Using characteristic handle=0x%04x (notify capable)\n",
                               ch.value_handle);
                    }
                    break;
                }

                case GATT_EVENT_QUERY_COMPLETE:
                    att_status = gatt_event_query_complete_get_att_status(packet);
                    if (att_status != ATT_ERROR_SUCCESS) {
                        printf("[GATT] Characteristic query error: 0x%02x\n", att_status);
                        gap_disconnect(connection_handle);
                        break;
                    }

                    if (!notify_characteristic_found) {
                        printf("[GATT] Notify characteristic not found, disconnecting\n");
                        gap_disconnect(connection_handle);
                        break;
                    }

                    listener_registered = true;
                    gatt_client_listen_for_characteristic_value_updates(
                        &notification_listener,
                        handle_gatt_client_event,
                        connection_handle,
                        &server_characteristic);

                    state = TC_W4_ENABLE_NOTIFICATIONS_COMPLETE;
                    printf("[GATT] Enabling notifications on handle=0x%04x...\n",
                           server_characteristic.value_handle);
                    gatt_client_write_client_characteristic_configuration(
                        handle_gatt_client_event,
                        connection_handle,
                        &server_characteristic,
                        GATT_CLIENT_CHARACTERISTICS_CONFIGURATION_NOTIFICATION);
                    break;

                default:
                    break;
            }
            break;

        // --------------------------------------------------
        case TC_W4_ENABLE_NOTIFICATIONS_COMPLETE:
        // --------------------------------------------------
            switch (hci_event_packet_get_type(packet)) {
                case GATT_EVENT_QUERY_COMPLETE:
                    att_status = gatt_event_query_complete_get_att_status(packet);
                    printf("[GATT] Notifications enable result: 0x%02x\n", att_status);
                    if (att_status != ATT_ERROR_SUCCESS) {
                        gap_disconnect(connection_handle);
                        break;
                    }
                    state = TC_W4_READY;
                    printf("[BLE] ===== READY - Receiving controller data =====\n");
                    break;
                default:
                    break;
            }
            break;

        // --------------------------------------------------
        case TC_W4_READY:
        // --------------------------------------------------
            switch (hci_event_packet_get_type(packet)) {

                case GATT_EVENT_NOTIFICATION: {
                    uint16_t       value_length = gatt_event_notification_get_value_length(packet);
                    const uint8_t *value        = gatt_event_notification_get_value(packet);

                    if (value_length != sizeof(ds4_data)) {
                        printf("[RX] Unexpected length: %d (expected %d)\n",
                               value_length, (int)sizeof(ds4_data));
                        break;
                    }

                    ds4_data controller;
                    memcpy(&controller, value, sizeof(ds4_data));

                    // チェックサム検証
                    uint8_t sum = (uint8_t)(1 +
                        controller.jyoutai + controller.L_x + controller.L_y +
                        controller.R_x    + controller.R_y  + controller.L2   +
                        controller.R2     + controller.key  + controller.boton);
                    bool valid = ((sum % 255) == controller.checsam);

                    // RSSIをカウンタ方式で定期取得
                    if (++rssi_counter >= RSSI_SAMPLE_INTERVAL) {
                        rssi_counter = 0;
                        gap_read_rssi(connection_handle);
                    }

                    printf("[RX] %s LX=%4d LY=%4d RX=%4d RY=%4d "
                           "L2=%3d R2=%3d key=%02x btn=%02x RSSI=%ddBm\n",
                           valid ? "OK" : "NG",
                           controller.L_x, controller.L_y,
                           controller.R_x, controller.R_y,
                           controller.L2,  controller.R2,
                           controller.key, controller.boton,
                           Rssi_server);
                    break;
                }

                default:
                    printf("[GATT] Unknown packet: 0x%02x\n",
                           hci_event_packet_get_type(packet));
                    break;
            }
            break;

        // --------------------------------------------------
        case TC_W4_SERVICE_DISCOVERY_DELAY:
        // --------------------------------------------------
            // タイマー待機中はGATTイベントを無視
            break;

        default:
            DEBUG_LOG("[GATT] Unhandled state: %d\n", state);
            break;
    }
}

// -------------------------------------------------------
// HCI イベントハンドラ
// -------------------------------------------------------
static void hci_event_handler(uint8_t packet_type, uint16_t channel,
                               uint8_t *packet, uint16_t size) {
    UNUSED(size);
    UNUSED(channel);
    bd_addr_t local_addr;

    if (packet_type != HCI_EVENT_PACKET) return;

    uint8_t event_type = hci_event_packet_get_type(packet);
    switch (event_type) {

        case BTSTACK_EVENT_STATE:
            if (btstack_event_state_get_state(packet) == HCI_STATE_WORKING) {
                gap_local_bd_addr(local_addr);
                printf("[BLE] BTstack up on %s\n", bd_addr_to_str(local_addr));
                client_start();
            } else {
                state = TC_OFF;
            }
            break;

        case GAP_EVENT_ADVERTISING_REPORT:
            if (state != TC_W4_SCAN_RESULT) return;
            if (!advertisement_report_contains_name("PicoW Controller", packet)) return;

            gap_event_advertising_report_get_address(packet, server_addr);
            server_addr_type = gap_event_advertising_report_get_address_type(packet);
            state = TC_W4_CONNECT;
            gap_stop_scan();
            printf("[BLE] Connecting to %s...\n", bd_addr_to_str(server_addr));
            gap_connect(server_addr, server_addr_type);
            break;

        case HCI_EVENT_LE_META:
            switch (hci_event_le_meta_get_subevent_code(packet)) {

                case HCI_SUBEVENT_LE_CONNECTION_COMPLETE:
                    if (state != TC_W4_CONNECT) return;
                    connection_handle = hci_subevent_le_connection_complete_get_connection_handle(packet);

                    {
                        // 接続時のInterval（Peripheral側が設定した値）をログ出力
                        uint16_t init_interval = hci_subevent_le_connection_complete_get_conn_interval(packet);
                        printf("[BLE] Connected! handle=0x%04x initial_interval=%.2fms\n",
                               connection_handle, init_interval * 1.25f);
                    }

                    printf("[STATE] TC_W4_CONNECT -> TC_W4_CONNECTION_UPDATE_COMPLETE\n");
                    // Connection Updateを500ms後に送信
                    state = TC_W4_CONNECTION_UPDATE_COMPLETE;
                    btstack_run_loop_set_timer(&connection_update_timer, CONNECTION_UPDATE_DELAY_MS);
                    btstack_run_loop_add_timer(&connection_update_timer);
                    break;

                case HCI_SUBEVENT_LE_CONNECTION_UPDATE_COMPLETE: {
                    uint8_t  status   = hci_subevent_le_connection_update_complete_get_status(packet);
                    uint16_t interval = hci_subevent_le_connection_update_complete_get_conn_interval(packet);
                    printf("[BLE] Connection update: status=0x%02x interval=%d (%.2fms)\n",
                           status, interval, interval * 1.25f);
                    printf("[STATE] TC_W4_CONNECTION_UPDATE_COMPLETE -> TC_W4_SERVICE_DISCOVERY_DELAY\n");

                    // [修正7] Update完了後すぐではなく200ms後にサービス探索を開始
                    // ATTサーバーの準備完了を待つ
                    state = TC_W4_SERVICE_DISCOVERY_DELAY;
                    printf("[GATT] Waiting %dms before service discovery...\n",
                           SERVICE_DISCOVERY_DELAY_MS);
                    btstack_run_loop_set_timer(&service_discovery_timer, SERVICE_DISCOVERY_DELAY_MS);
                    btstack_run_loop_add_timer(&service_discovery_timer);
                    break;
                }

                default:
                    break;
            }
            break;

        case HCI_EVENT_COMMAND_COMPLETE: {
            uint16_t opcode = hci_event_command_complete_get_command_opcode(packet);
            if (opcode == HCI_OPCODE_HCI_READ_RSSI) {
                // return_parameters: [status(1), handle(2), rssi(1)]
                Rssi_server = (int8_t)hci_event_command_complete_get_return_parameters(packet)[3];
                printf("[RSSI] %d dBm\n", Rssi_server);
            }
            break;
        }

        case HCI_EVENT_DISCONNECTION_COMPLETE:
            // 残っているタイマーをすべて止める
            btstack_run_loop_remove_timer(&connection_update_timer);
            btstack_run_loop_remove_timer(&service_discovery_timer);

            connection_handle = HCI_CON_HANDLE_INVALID;
            if (listener_registered) {
                listener_registered = false;
                gatt_client_stop_listening_for_characteristic_value_updates(
                    &notification_listener);
            }
            printf("[BLE] Disconnected from %s\n", bd_addr_to_str(server_addr));
            if (state == TC_OFF) break;
            client_start();
            break;

        default:
            break;
    }
}

// -------------------------------------------------------
// LEDハートビートタイマー
// -------------------------------------------------------
static void heartbeat_handler(struct btstack_timer_source *ts) {
    static bool quick_flash = false;
    static bool led_on      = true;

    led_on = !led_on;
    cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, led_on);

    if (listener_registered && led_on) {
        quick_flash = !quick_flash;
    } else if (!listener_registered) {
        quick_flash = false;
    }

    btstack_run_loop_set_timer(ts,
        (led_on || quick_flash) ? LED_QUICK_FLASH_DELAY_MS : LED_SLOW_FLASH_DELAY_MS);
    btstack_run_loop_add_timer(ts);
}

// -------------------------------------------------------
// main
// -------------------------------------------------------
int main(void) {
    stdio_init_all();

    if (cyw43_arch_init()) {
        printf("failed to initialise cyw43_arch\n");
        return -1;
    }

    l2cap_init();
    sm_init();
    sm_set_io_capabilities(IO_CAPABILITY_NO_INPUT_NO_OUTPUT);

    // タイマーコールバックを登録
    connection_update_timer.process  = &connection_update_timer_handler;
    service_discovery_timer.process  = &service_discovery_timer_handler;  // [修正7]

    // LE Peripheral がATTクエリを出す場合（Android/iOS対応）のため空のATTサーバを設定
    att_server_init(NULL, NULL, NULL);

    gatt_client_init();

    hci_event_callback_registration.callback = &hci_event_handler;
    hci_add_event_handler(&hci_event_callback_registration);

    // LEDハートビートタイマー開始
    heartbeat.process = &heartbeat_handler;
    btstack_run_loop_set_timer(&heartbeat, LED_SLOW_FLASH_DELAY_MS);
    btstack_run_loop_add_timer(&heartbeat);

    hci_power_control(HCI_POWER_ON);
    btstack_run_loop_execute();

    return 0;
}