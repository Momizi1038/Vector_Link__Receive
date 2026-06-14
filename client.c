/**
 * Bluetooth Classic SPP クライアント - Picow Controller受信側
 * 
 * [修正版] SDP Query 開始メカニズムを修正
 * - Inquiry 完了時に直接 SDP Query を開始
 * - SDP イベントをメインハンドラに統合
 *
 * 接続フロー:
 *   1. Inquiry → サーバーのBDアドレス取得
 *   2. SDP Query → RFCOMMチャンネル番号取得
 *   3. rfcomm_create_channel → データ受信
 */

#include <stdio.h>
#include <string.h>
#include "btstack.h"
#include "pico/cyw43_arch.h"
#include "pico/stdlib.h"

#include "type.h"

// -------------------------------------------------------
// 設定
// -------------------------------------------------------
#define LED_FLASH_CONNECTED_MS   100
#define LED_FLASH_IDLE_MS       1000
#define INQUIRY_DURATION         5       // Inquiry時間 × 1.28秒
#define TARGET_DEVICE_NAME      "PicoW Controller"

// -------------------------------------------------------
// ステートマシン
// -------------------------------------------------------
typedef enum {
    TC_OFF,
    TC_W4_INQUIRY_RESULT,       // Inquiry中
    TC_W4_SDP_RESULT,           // SDPクエリ中
    TC_W4_RFCOMM_CONNECT,       // RFCOMM接続待ち
    TC_W4_READY                 // 接続完了・受信中
} tc_state_t;

// -------------------------------------------------------
// グローバル変数
// -------------------------------------------------------
static btstack_packet_callback_registration_t hci_event_callback_registration;
static tc_state_t state = TC_OFF;

static bd_addr_t  server_addr;
static bool       server_found = false;
static uint8_t    rfcomm_channel = 0;   // SDPで取得するチャンネル番号
static uint16_t   rfcomm_cid    = 0;

static int        rssi_server  = 0;
static int        rssi_counter = 0;
#define RSSI_SAMPLE_INTERVAL 30

static btstack_timer_source_t heartbeat;

// -------------------------------------------------------
// 前方宣言
// -------------------------------------------------------
static void spp_client_packet_handler(uint8_t packet_type, uint16_t channel,
                                      uint8_t *packet, uint16_t size);
static void client_start_inquiry(void);
static void client_start_sdp_query(void);

// -------------------------------------------------------
// Inquiry 開始（リセット付き）
// -------------------------------------------------------
static void client_start_inquiry(void) {
    printf("[INQ] Starting Inquiry...\n");
    state        = TC_W4_INQUIRY_RESULT;
    server_found = false;
    rfcomm_channel = 0;
    memset(server_addr, 0, sizeof(server_addr));
    gap_inquiry_start(INQUIRY_DURATION);
}

// -------------------------------------------------------
// SDPクエリ開始（Inquiry完了後に呼ばれる）
// [修正] コンテキストパラメータを削除、直接呼び出し対応
// -------------------------------------------------------
static void client_start_sdp_query(void) {
    if (rfcomm_channel != 0) {
        printf("[SDP] Channel already obtained: %d\n", rfcomm_channel);
        return;
    }
    
    printf("[SDP] Querying RFCOMM channel from %s...\n", bd_addr_to_str(server_addr));
    state = TC_W4_SDP_RESULT;
    
    // SPP UUID (0x1101) でRFCOMMチャンネルと名前を取得
    // ※ BTStack内部でSDP_EVENT_QUERY_RFCOMM_SERVICE とSDP_EVENT_QUERY_COMPLETE が
    //    メインハンドラに配信される
    sdp_client_query_rfcomm_channel_and_name_for_uuid(
        &spp_client_packet_handler,  // [修正] コールバックを直接指定
        server_addr,
        BLUETOOTH_SERVICE_CLASS_SERIAL_PORT   // 0x1101
    );
}

// -------------------------------------------------------
// LED タイマー
// -------------------------------------------------------
static void heartbeat_handler(struct btstack_timer_source *ts) {
    static bool led_on = true;
    led_on = !led_on;
    cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, led_on);
    btstack_run_loop_set_timer(ts,
        (state == TC_W4_READY) ? LED_FLASH_CONNECTED_MS : LED_FLASH_IDLE_MS);
    btstack_run_loop_add_timer(ts);
}

// -------------------------------------------------------
// メインパケットハンドラ
// [重要修正] Inquiry 完了後、ACL 接続を明示的に確立
// -------------------------------------------------------
static void spp_client_packet_handler(uint8_t packet_type, uint16_t channel,
                                      uint8_t *packet, uint16_t size) {
    UNUSED(channel);

    bd_addr_t event_addr;

    switch (packet_type) {

        case HCI_EVENT_PACKET:
            switch (hci_event_packet_get_type(packet)) {

                case BTSTACK_EVENT_STATE:
                    if (btstack_event_state_get_state(packet) == HCI_STATE_WORKING) {
                        gap_local_bd_addr(event_addr);
                        printf("[SPP] BTstack up on %s\n", bd_addr_to_str(event_addr));
                        client_start_inquiry();
                    } else {
                        state = TC_OFF;
                    }
                    break;

                case GAP_EVENT_INQUIRY_RESULT: {
                    if (state != TC_W4_INQUIRY_RESULT) break;

                    bd_addr_t found_addr;
                    gap_event_inquiry_result_get_bd_addr(packet, found_addr);

                    char name_buf[240] = {0};
                    bool has_name = (bool)gap_event_inquiry_result_get_name_available(packet);

                    if (has_name) {
                        uint8_t name_len = gap_event_inquiry_result_get_name_len(packet);
                        const uint8_t *name_data = gap_event_inquiry_result_get_name(packet);
                        if (name_len >= sizeof(name_buf)) name_len = sizeof(name_buf) - 1;
                        memcpy(name_buf, name_data, name_len);
                        printf("[INQ] Found: %s -> name: %s\n",
                               bd_addr_to_str(found_addr), name_buf);

                        if (strcmp(name_buf, TARGET_DEVICE_NAME) == 0) {
                            memcpy(server_addr, found_addr, sizeof(bd_addr_t));
                            server_found = true;
                            printf("[INQ] Target found: %s\n", bd_addr_to_str(server_addr));
                            gap_inquiry_stop();
                        }
                    } else {
                        printf("[INQ] Found (no name): %s\n", bd_addr_to_str(found_addr));
                    }
                    break;
                }

                // --- Inquiry 完了 → ACL 接続確立 ---
                case GAP_EVENT_INQUIRY_COMPLETE:
                    if (state != TC_W4_INQUIRY_RESULT) break;
                    if (!server_found) {
                        printf("[INQ] Target not found, retrying...\n");
                        client_start_inquiry();
                        break;
                    }
                    
                    printf("[INQ] Inquiry complete, starting SDP query for %s...\n",
                           bd_addr_to_str(server_addr));
                    state = TC_W4_SDP_RESULT;
                                       
                    int sdp_err = sdp_client_query_rfcomm_channel_and_name_for_uuid(
                        &spp_client_packet_handler,
                        server_addr,
                        BLUETOOTH_SERVICE_CLASS_SERIAL_PORT
                    );
                    if (sdp_err != 0) {
                        printf("[SDP] ERROR: SDP query failed: %d\n", sdp_err);
                        state = TC_W4_INQUIRY_RESULT;
                        client_start_inquiry();
                    }
                    break;

                // --- HCI ACL 接続完了 → SDP Query 開始 ---
                case HCI_EVENT_CONNECTION_COMPLETE: {
                    uint8_t status = hci_event_connection_complete_get_status(packet);
                    if (status != ERROR_CODE_SUCCESS) {
                        printf("[HCI] ACL Connection failed: 0x%02x\n", status);
                        client_start_inquiry();
                        break;
                    }
                    hci_event_connection_complete_get_bd_addr(packet, event_addr);
                    printf("[HCI] ===== ACL Connection established to %s =====\n",
                           bd_addr_to_str(event_addr));
                    
                    // // [重要] ACL 接続成功後に SDP Query を実行
                    // printf("[SDP] Starting SDP query for channel discovery...\n");
                    // state = TC_W4_SDP_RESULT;
                    
                    // int sdp_err = sdp_client_query_rfcomm_channel_and_name_for_uuid(
                    //     &spp_client_packet_handler,
                    //     server_addr,
                    //     BLUETOOTH_SERVICE_CLASS_SERIAL_PORT
                    // );
                    // if (sdp_err != 0) {
                    //     printf("[SDP] ERROR: SDP query failed: %d\n", sdp_err);
                    //     client_start_inquiry();
                    // }
                    break;
                }

                // --- SDP Query 結果: RFCOMM サービス情報 ---
                case SDP_EVENT_QUERY_RFCOMM_SERVICE: {
                    if (state != TC_W4_SDP_RESULT) break;
                    
                    uint8_t ch = sdp_event_query_rfcomm_service_get_rfcomm_channel(packet);
                    const char *name = sdp_event_query_rfcomm_service_get_name(packet);
                    printf("[SDP] Found service: \"%s\" channel=%d\n", name, ch);
                    rfcomm_channel = ch;
                    break;
                }

                // --- SDP Query 完了 → RFCOMM 接続 ---
                case SDP_EVENT_QUERY_COMPLETE: {
                    if (state != TC_W4_SDP_RESULT) break;
                    
                    uint8_t status = sdp_event_query_complete_get_status(packet);
                    if (status != ERROR_CODE_SUCCESS || rfcomm_channel == 0) {
                        printf("[SDP] Query failed (status=0x%02x channel=%d), retrying...\n",
                               status, rfcomm_channel);
                        rfcomm_channel = 0;
                        client_start_inquiry();
                        break;
                    }
                    
                    printf("[SDP] Query complete! channel=%d, creating RFCOMM channel...\n", 
                           rfcomm_channel);
                    state = TC_W4_RFCOMM_CONNECT;
                    
                    int rfcomm_err = rfcomm_create_channel(spp_client_packet_handler,
                                                           server_addr,
                                                           rfcomm_channel,
                                                           &rfcomm_cid);
                    if (rfcomm_err != 0) {
                        printf("[RFCOMM] ERROR: Channel create failed: %d\n", rfcomm_err);
                        rfcomm_channel = 0;
                        client_start_inquiry();
                    } else {
                        printf("[RFCOMM] Channel create request sent, waiting for RFCOMM_EVENT_CHANNEL_OPENED...\n");
                    }
                    break;
                }

                // --- ペアリング PIN ---
                case HCI_EVENT_PIN_CODE_REQUEST:
                    printf("[SPP] PIN code request\n");
                    hci_event_pin_code_request_get_bd_addr(packet, event_addr);
                    gap_pin_code_response(event_addr, "0000");
                    break;

                // --- SSP 自動承認 ---
                case HCI_EVENT_USER_CONFIRMATION_REQUEST:
                    hci_event_user_confirmation_request_get_bd_addr(packet, event_addr);
                    gap_ssp_confirmation_response(event_addr);
                    break;

                // --- RFCOMM 接続完了 ---
                case RFCOMM_EVENT_CHANNEL_OPENED:
                    if (rfcomm_event_channel_opened_get_status(packet) != ERROR_CODE_SUCCESS) {
                        printf("[RFCOMM] Channel open failed: 0x%02x\n",
                               rfcomm_event_channel_opened_get_status(packet));
                        rfcomm_cid = 0;
                        rfcomm_channel = 0;
                        client_start_inquiry();
                        break;
                    }
                    rfcomm_cid = rfcomm_event_channel_opened_get_rfcomm_cid(packet);
                    state = TC_W4_READY;
                    printf("[SPP] ===== Connected! cid=0x%04x mtu=%d =====\n",
                           rfcomm_cid,
                           rfcomm_event_channel_opened_get_max_frame_size(packet));
                    break;

                // --- 切断イベント ---
                case HCI_EVENT_DISCONNECTION_COMPLETE:
                    printf("[HCI] Disconnection complete\n");
                    rfcomm_cid = 0;
                    rfcomm_channel = 0;
                    client_start_inquiry();
                    //if (state != TC_OFF) client_start_inquiry();
                    break;

                case RFCOMM_EVENT_CHANNEL_CLOSED:
                    printf("[RFCOMM] Channel closed\n");
                    rfcomm_cid = 0;
                    rfcomm_channel = 0;
                    state = TC_OFF;
                    //if (state != TC_OFF) client_start_inquiry();
                    break;

                // --- RSSI読み取り結果 ---
                case HCI_EVENT_COMMAND_COMPLETE: {
                    uint16_t opcode = hci_event_command_complete_get_command_opcode(packet);
                    if (opcode == HCI_OPCODE_HCI_READ_RSSI) {
                        rssi_server = (int8_t)
                            hci_event_command_complete_get_return_parameters(packet)[3];
                        printf("[RSSI] %d dBm\n", rssi_server);
                    }
                    break;
                }

                default:
                    break;
            }
            break;

        // --- RFCOMMデータ受信 ---
        case RFCOMM_DATA_PACKET: {
            if (size != sizeof(ds4_data)) {
                printf("[RX] Unexpected size: %d (expected %d)\n",
                       size, (int)sizeof(ds4_data));
                rfcomm_grant_credits(rfcomm_cid, 1);
                break;
            }

            ds4_data controller;
            memcpy(&controller, packet, sizeof(ds4_data));

            uint8_t sum = (uint8_t)(1 +
                controller.jyoutai + controller.L_x + controller.L_y +
                controller.R_x    + controller.R_y  + controller.L2  +
                controller.R2     + controller.key  + controller.boton);
            bool valid = ((sum % 256) == controller.checsam);

            // RSSI 定期取得
            if (++rssi_counter >= RSSI_SAMPLE_INTERVAL) {
                rssi_counter = 0;
                hci_connection_t *con = hci_connection_for_bd_addr_and_type(server_addr, BD_ADDR_TYPE_ACL);
                if (con != NULL) {
                    gap_read_rssi(con->con_handle);
                }
            }

            printf("[RX] %s LX=%3d LY=%3d RX=%3d RY=%3d "
                   "L2=%3d R2=%3d key=%02x btn=%02x RSSI=%ddBm\n",
                   valid ? "OK" : "NG",
                   controller.L_x, controller.L_y,
                   controller.R_x, controller.R_y,
                   controller.L2,  controller.R2,
                   controller.key, controller.boton,
                   rssi_server);

            rfcomm_grant_credits(rfcomm_cid, 1);
            break;
        }

        default:
            break;
    }
}

// -------------------------------------------------------
// その他の関数（変更なし）
// -------------------------------------------------------
// static void client_start_inquiry(void) {
//     printf("[INQ] Starting Inquiry...\n");
//     state        = TC_W4_INQUIRY_RESULT;
//     server_found = false;
//     rfcomm_channel = 0;
//     memset(server_addr, 0, sizeof(server_addr));
//     gap_inquiry_start(INQUIRY_DURATION);
// }

// static void heartbeat_handler(struct btstack_timer_source *ts) {
//     static bool led_on = true;
//     led_on = !led_on;
//     cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, led_on);
//     btstack_run_loop_set_timer(ts,
//         (state == TC_W4_READY) ? LED_FLASH_CONNECTED_MS : LED_FLASH_IDLE_MS);
//     btstack_run_loop_add_timer(ts);
// }

// -------------------------------------------------------
// main（変更なし）
// -------------------------------------------------------
int main(void) {
    stdio_init_all();

    if (cyw43_arch_init()) {
        printf("failed to initialise cyw43_arch\n");
        return -1;
    }

    l2cap_init();
    rfcomm_init();
    sdp_client_init();

    hci_set_inquiry_mode(INQUIRY_MODE_RSSI_AND_EIR);
    gap_ssp_set_authentication_requirement(SSP_IO_AUTHREQ_MITM_PROTECTION_NOT_REQUIRED_NO_BONDING);
    gap_ssp_set_io_capability(SSP_IO_CAPABILITY_NO_INPUT_NO_OUTPUT);

    hci_event_callback_registration.callback = &spp_client_packet_handler;
    hci_add_event_handler(&hci_event_callback_registration);

    heartbeat.process = &heartbeat_handler;
    btstack_run_loop_set_timer(&heartbeat, LED_FLASH_IDLE_MS);
    btstack_run_loop_add_timer(&heartbeat);

    hci_power_control(HCI_POWER_ON);
    btstack_run_loop_execute();

    return 0;
}