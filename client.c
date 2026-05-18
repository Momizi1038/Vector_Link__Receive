/**
 * Bluetooth Classic SPP クライアント - Picow Controller受信側
 *
 * BLE (GATTクライアント/Notify) から Bluetooth Classic (SPP/RFCOMM) への移行版
 *
 * 変更点:
 *  - BLEスキャン (gap_start_scan) → Classic Inquiry (gap_inquiry_start)
 *  - GAP_EVENT_ADVERTISING_REPORT → GAP_EVENT_INQUIRY_RESULT でデバイス名比較
 *  - GATTクライアント (gatt_client_discover_*) → rfcomm_create_channel で直接接続
 *  - GATT Notify受信 → RFCOMM_DATA_PACKET で受信
 *  - Connection Update / Service Discovery 遅延 タイマー → 削除
 *  - ステートマシンを Classic 用に整理
 */

#include <stdio.h>
#include <string.h>
#include "btstack.h"
#include "pico/cyw43_arch.h"
#include "pico/stdlib.h"

#include "type.h"   // ds4_data 構造体

// -------------------------------------------------------
// デバッグログ制御
// -------------------------------------------------------
#if 1
#define DEBUG_LOG(...) printf(__VA_ARGS__)
#else
#define DEBUG_LOG(...)
#endif

// -------------------------------------------------------
// 設定
// -------------------------------------------------------
#define LED_FLASH_CONNECTED_MS   100
#define LED_FLASH_IDLE_MS       1000

// Inquiry 時間（N × 1.28秒、最大48）
#define INQUIRY_DURATION         5   // 約6.4秒

// SPP チャンネル番号（サーバー側と一致させること）
#define SPP_RFCOMM_CHANNEL       1

// 接続対象デバイス名（サーバー側 gap_set_local_name と一致させること）
#define TARGET_DEVICE_NAME      "PicoW Controller"

// -------------------------------------------------------
// ステートマシン定義（Classic 用に簡素化）
// -------------------------------------------------------
typedef enum {
    TC_OFF,
    TC_IDLE,
    TC_W4_INQUIRY_RESULT,    // Inquiry 実行中 → サーバーを探索
    TC_W4_CONNECT,           // RFCOMM 接続待ち
    TC_W4_READY              // 接続完了・データ受信中
} gc_state_t;

// -------------------------------------------------------
// グローバル変数
// -------------------------------------------------------
static btstack_packet_callback_registration_t hci_event_callback_registration;
static gc_state_t state = TC_OFF;

static bd_addr_t  server_addr;          // 見つけたサーバーの BD アドレス
static bool       server_found = false;

static uint16_t   rfcomm_cid = 0;      // 0 = 未接続

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

// -------------------------------------------------------
// Inquiry 開始
// -------------------------------------------------------
static void client_start_inquiry(void) {
    DEBUG_LOG("[SPP] Starting Inquiry...\n");
    state        = TC_W4_INQUIRY_RESULT;
    server_found = false;
    // General/Unlimited Inquiry Access Code, INQUIRY_DURATION × 1.28s, 最大応答数=0(無制限)
    gap_inquiry_start(INQUIRY_DURATION);
}

// -------------------------------------------------------
// LED ハートビートタイマー
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
// HCI イベント + RFCOMM イベント + RFCOMM データを一元処理
// -------------------------------------------------------
static void spp_client_packet_handler(uint8_t packet_type, uint16_t channel,
                                      uint8_t *packet, uint16_t size) {
    UNUSED(channel);

    bd_addr_t event_addr;

    switch (packet_type) {

        // --------------------------------------------------
        // HCI イベント
        // --------------------------------------------------
        case HCI_EVENT_PACKET:
            switch (hci_event_packet_get_type(packet)) {

                // --- スタック起動 ---
                case BTSTACK_EVENT_STATE:
                    if (btstack_event_state_get_state(packet) == HCI_STATE_WORKING) {
                        gap_local_bd_addr(event_addr);
                        printf("[SPP] BTstack up on %s\n", bd_addr_to_str(event_addr));
                        client_start_inquiry();
                    } else {
                        state = TC_OFF;
                    }
                    break;

                // --- Inquiry 結果（1デバイスごとに届く）---
                case GAP_EVENT_INQUIRY_RESULT: {
                    if (state != TC_W4_INQUIRY_RESULT) break;

                    // BD アドレス取得（出力引数形式）
                    bd_addr_t found_addr;
                    gap_event_inquiry_result_get_bd_addr(packet, found_addr);

                    // EIR にデバイス名が含まれているか確認
                    // 正しい API: gap_event_inquiry_result_get_name_available / _len / (data)
                    char name_buffer[240];
                    bool has_name = (bool)gap_event_inquiry_result_get_name_available(packet);
                    if (has_name) {
                        uint8_t name_len = gap_event_inquiry_result_get_name_len(packet);
                        const uint8_t *name_data = gap_event_inquiry_result_get_name(packet);
                        if (name_len >= sizeof(name_buffer)) name_len = sizeof(name_buffer) - 1;
                        memcpy(name_buffer, name_data, name_len);
                        name_buffer[name_len] = '\0';
                        printf("[INQ] Found: %s -> name: %s\n",
                               bd_addr_to_str(found_addr), name_buffer);

                        if (strncmp(name_buffer, TARGET_DEVICE_NAME,
                                    strlen(TARGET_DEVICE_NAME)) == 0) {
                            memcpy(server_addr, found_addr, sizeof(bd_addr_t));
                            server_found = true;
                            printf("[INQ] Target found: %s\n", bd_addr_to_str(server_addr));
                        }
                    } else {
                        // 名前なし: Remote Name Request フォールバック用にアドレスを保存
                        printf("[INQ] Found (no name): %s\n", bd_addr_to_str(found_addr));
                        // Inquiry完了後に gap_remote_name_request で名前を取得する
                        // （複数台あれば最後の1台のみ保存する簡易実装）
                        if (!server_found) {
                            memcpy(server_addr, found_addr, sizeof(bd_addr_t));
                            // pageScanRepetitionMode と clockOffset も保存が理想だが
                            // 簡易実装のため固定値を使用（動作はする）
                        }
                    }
                    break;
                }

                // --- Inquiry 完了 ---
                case GAP_EVENT_INQUIRY_COMPLETE:
                    if (state != TC_W4_INQUIRY_RESULT) break;
                    if (server_found) {
                        // EIR で名前取得済み → 即接続
                        state = TC_W4_CONNECT;
                        printf("[SPP] Connecting to %s channel=%d...\n",
                               bd_addr_to_str(server_addr), SPP_RFCOMM_CHANNEL);
                        rfcomm_create_channel(spp_client_packet_handler,
                                             server_addr,
                                             SPP_RFCOMM_CHANNEL,
                                             &rfcomm_cid);
                    } else {
                        // EIR に名前なし & 候補アドレスあり → Remote Name Request
                        static const bd_addr_t zero_addr = {0};
                        if (memcmp(server_addr, zero_addr, sizeof(bd_addr_t)) != 0) {
                            printf("[INQ] Requesting remote name from %s...\n",
                                   bd_addr_to_str(server_addr));
                            gap_remote_name_request(server_addr, 0, 0);
                        } else {
                            printf("[INQ] No devices found, retrying...\n");
                            client_start_inquiry();
                        }
                    }
                    break;

                // --- Remote Name Request 完了（EIR に名前がなかった場合のフォールバック）---
                case HCI_EVENT_REMOTE_NAME_REQUEST_COMPLETE: {
                    if (state != TC_W4_INQUIRY_RESULT) break;
                    bd_addr_t name_addr;
                    // hci_event_remote_name_request_complete_get_bd_addr は出力引数
                    reverse_bd_addr(&packet[3], name_addr);
                    uint8_t status = packet[2];
                    if (status != ERROR_CODE_SUCCESS) {
                        printf("[INQ] Remote name request failed for %s\n",
                               bd_addr_to_str(name_addr));
                        break;
                    }
                    const char *remote_name = (const char *)&packet[9];
                    printf("[INQ] Remote name: %s -> %s\n",
                           bd_addr_to_str(name_addr), remote_name);
                    if (strncmp(remote_name, TARGET_DEVICE_NAME,
                                strlen(TARGET_DEVICE_NAME)) == 0) {
                        memcpy(server_addr, name_addr, sizeof(bd_addr_t));
                        server_found = true;
                        printf("[INQ] Target found via name request: %s\n",
                               bd_addr_to_str(server_addr));
                        // 即接続開始
                        state = TC_W4_CONNECT;
                        printf("[SPP] Connecting to %s channel=%d...\n",
                               bd_addr_to_str(server_addr), SPP_RFCOMM_CHANNEL);
                        rfcomm_create_channel(spp_client_packet_handler,
                                             server_addr,
                                             SPP_RFCOMM_CHANNEL,
                                             &rfcomm_cid);
                    }
                    break;
                }

                // --- ペアリング PIN コード要求 ---
                case HCI_EVENT_PIN_CODE_REQUEST:
                    printf("[SPP] PIN code request\n");
                    hci_event_pin_code_request_get_bd_addr(packet, event_addr);
                    gap_pin_code_response(event_addr, "0000");
                    break;

                // --- SSP 数値比較 ---
                case HCI_EVENT_USER_CONFIRMATION_REQUEST:
                    hci_event_user_confirmation_request_get_bd_addr(packet, event_addr);
                    gap_ssp_confirmation_response(event_addr);
                    break;

                // --- RFCOMM チャンネル開通 ---
                case RFCOMM_EVENT_CHANNEL_OPENED:
                    if (rfcomm_event_channel_opened_get_status(packet) != ERROR_CODE_SUCCESS) {
                        printf("[SPP] Connection failed: 0x%02x\n",
                               rfcomm_event_channel_opened_get_status(packet));
                        rfcomm_cid = 0;
                        // 失敗したら Inquiry からやり直し
                        client_start_inquiry();
                        break;
                    }
                    rfcomm_cid = rfcomm_event_channel_opened_get_rfcomm_cid(packet);
                    state      = TC_W4_READY;
                    printf("[SPP] ===== READY cid=0x%04x mtu=%d =====\n",
                           rfcomm_cid,
                           rfcomm_event_channel_opened_get_max_frame_size(packet));
                    break;

                // --- RFCOMM チャンネル切断 ---
                case RFCOMM_EVENT_CHANNEL_CLOSED:
                    printf("[SPP] Disconnected\n");
                    rfcomm_cid = 0;
                    if (state == TC_OFF) break;
                    // 自動再接続
                    client_start_inquiry();
                    break;

                // --- RSSI 読み取り結果 ---
                case HCI_EVENT_COMMAND_COMPLETE: {
                    uint16_t opcode = hci_event_command_complete_get_command_opcode(packet);
                    if (opcode == HCI_OPCODE_HCI_READ_RSSI) {
                        rssi_server = (int8_t)hci_event_command_complete_get_return_parameters(packet)[3];
                        printf("[RSSI] %d dBm\n", rssi_server);
                    }
                    break;
                }

                default:
                    break;
            }
            break;

        // --------------------------------------------------
        // RFCOMM データ受信
        // BLE の GATT Notify に相当
        // --------------------------------------------------
        case RFCOMM_DATA_PACKET: {
            if (size != sizeof(ds4_data)) {
                printf("[RX] Unexpected length: %d (expected %d)\n",
                       size, (int)sizeof(ds4_data));
                // RFCOMM フロー制御: 受信処理後にクレジットを返す
                rfcomm_grant_credits(rfcomm_cid, 1);
                break;
            }

            ds4_data controller;
            memcpy(&controller, packet, sizeof(ds4_data));

            // チェックサム検証
            uint8_t sum = (uint8_t)(1 +
                controller.jyoutai + controller.L_x + controller.L_y +
                controller.R_x    + controller.R_y  + controller.L2   +
                controller.R2     + controller.key  + controller.boton);
            bool valid = ((sum % 255) == controller.checsam);

            // RSSI 定期取得（Classic は hci_connection_handle_for_bd_addr で handle を取得）
            if (++rssi_counter >= RSSI_SAMPLE_INTERVAL) {
                rssi_counter = 0;
                hci_connection_t *con = hci_connection_for_bd_addr_and_type(
                    server_addr, BD_ADDR_TYPE_ACL);
                if (con != NULL) {
                    gap_read_rssi(con->con_handle);
                }
            }

            printf("[RX] %s LX=%4d LY=%4d RX=%4d RY=%4d "
                   "L2=%3d R2=%3d key=%02x btn=%02x RSSI=%ddBm\n",
                   valid ? "OK" : "NG",
                   controller.L_x, controller.L_y,
                   controller.R_x, controller.R_y,
                   controller.L2,  controller.R2,
                   controller.key, controller.boton,
                   rssi_server);

            // RFCOMM クレジットを返して次パケットを受信可能にする
            rfcomm_grant_credits(rfcomm_cid, 1);
            break;
        }

        default:
            break;
    }
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

    // --- BTstack プロトコルスタック初期化 ---
    l2cap_init();
    rfcomm_init();

    // SSP 設定（サーバー側と合わせる）
    gap_ssp_set_authentication_requirement(SSP_IO_CAPABILITY_DISPLAY_YES_NO);
    gap_ssp_set_io_capability(SSP_IO_CAPABILITY_DISPLAY_YES_NO);

    // HCI イベントハンドラ登録
    hci_event_callback_registration.callback = &spp_client_packet_handler;
    hci_add_event_handler(&hci_event_callback_registration);

    // LED ハートビートタイマー開始
    heartbeat.process = &heartbeat_handler;
    btstack_run_loop_set_timer(&heartbeat, LED_FLASH_IDLE_MS);
    btstack_run_loop_add_timer(&heartbeat);

    // EIR（Extended Inquiry Result）を有効化してInquiry結果に名前を含める
    // INQUIRY_MODE_RSSI_AND_EIR = 2
    hci_set_inquiry_mode(INQUIRY_MODE_RSSI_AND_EIR);

    hci_power_control(HCI_POWER_ON);
    btstack_run_loop_execute();

    return 0;
}