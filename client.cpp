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
#include "pico/stdlib.h"
#include "btstack.h"
#include "pico/cyw43_arch.h"
#include "pico/stdlib.h"
#include "pico/multicore.h"

#include "lib/E220Connect/e220.h"
#include "hardware/uart.h"
#include "hardware/gpio.h"

#include "type.h"
#include "send_data.h"
#include "uart_output.h"

// -------------------------------------------------------
// 設定
// -------------------------------------------------------
#define LED_FLASH_CONNECTED_MS   100
#define LED_FLASH_IDLE_MS       1000
#define INQUIRY_DURATION         5       // Inquiry時間 × 1.28秒
#define TARGET_DEVICE_NAME      "PicoW Controller"

#define DEBUG_LOG_BT    1
#define DEBUG_LOG_LORA  0

// 外部マイコン出力用UART
#define EXTERNAL_UART_ID uart0
#define EXTERNAL_UART_TX_PIN 0
#define EXTERNAL_UART_RX_PIN 1
#define EXTERNAL_UART_BAUD_RATE 115200
UartOutput output_uart(EXTERNAL_UART_ID, EXTERNAL_UART_TX_PIN, EXTERNAL_UART_RX_PIN, EXTERNAL_UART_BAUD_RATE);
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

// Bluetooth Classic ACL Connection Handle
static hci_con_handle_t connection_handle = HCI_CON_HANDLE_INVALID;

static int        rssi_server  = 0;
static int        rssi_counter = 0;
#define RSSI_SAMPLE_INTERVAL 30

#define ConectLED_D1 6
#define BlueLED_D2 3
#define Yellow_D3 2

critical_section_t cs_bt_data;
bool share_update;
ds4_data share_data;

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
    state              = TC_W4_INQUIRY_RESULT;
    server_found       = false;
    rfcomm_channel     = 0;
    rfcomm_cid         = 0;
    connection_handle  = HCI_CON_HANDLE_INVALID;

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
                case GAP_EVENT_INQUIRY_COMPLETE:{
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
                }

                // --- HCI ACL 接続完了 → SDP Query 開始 ---
                case HCI_EVENT_CONNECTION_COMPLETE: {
                    uint8_t status = hci_event_connection_complete_get_status(packet);

                    if (status != ERROR_CODE_SUCCESS) {
                        printf("[HCI] ACL Connection failed: 0x%02x\n", status);
                        connection_handle = HCI_CON_HANDLE_INVALID;

                        client_start_inquiry();
                        break;
                    }

                    hci_event_connection_complete_get_bd_addr(packet, event_addr);

                     // ACL Connection Handleを取得
                    connection_handle = hci_event_connection_complete_get_connection_handle(packet);

                    printf("[HCI] ===== ACL Connection established to %s =====\n",
                        bd_addr_to_str(event_addr));

                    printf("[HCI] connection_handle = 0x%04x\n", connection_handle);
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

                case HCI_EVENT_MODE_CHANGE: {

                    uint8_t status = hci_event_mode_change_get_status(packet);
                    hci_con_handle_t handle = hci_event_mode_change_get_handle(packet);
                    uint8_t mode = hci_event_mode_change_get_mode(packet);
                    uint16_t interval = hci_event_mode_change_get_interval(packet);

                    printf("[SNIFF] MODE_CHANGE: status=0x%02x handle=0x%04x mode=%u interval=%u (%.3f ms)\n",
                        status, handle, mode, interval, interval * 0.625f);

                    if(status != ERROR_CODE_SUCCESS){
                        printf("[SNIFF] Mode change failed\n");
                    }else if(mode == ACL_CONNECTION_MODE_SNIFF){
                        printf("[SNIFF] ===== Sniff Mode ACTIVE =====\n");
                    }else if(mode == ACL_CONNECTION_MODE_ACTIVE){
                        printf("[SNIFF] ===== Active Mode =====\n");
                    }else{
                        printf("[SNIFF] Unknown connection mode: %u\n", mode);
                    }

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
                case HCI_EVENT_PIN_CODE_REQUEST:{
                    printf("[SPP] PIN code request\n");
                    hci_event_pin_code_request_get_bd_addr(packet, event_addr);
                    gap_pin_code_response(event_addr, "0000");
                    break;
                }

                // --- SSP 自動承認 ---
                case HCI_EVENT_USER_CONFIRMATION_REQUEST:{
                    hci_event_user_confirmation_request_get_bd_addr(packet, event_addr);
                    gap_ssp_confirmation_response(event_addr);
                    break;
                }

                // --- RFCOMM 接続完了 ---
                case RFCOMM_EVENT_CHANNEL_OPENED:{
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
                           rfcomm_cid, rfcomm_event_channel_opened_get_max_frame_size(packet));

                    // ---------------------------------------------------
                    // Sniff Mode開始
                    // ---------------------------------------------------
                    if (connection_handle != HCI_CON_HANDLE_INVALID) {

                        printf("[SNIFF] Requesting Sniff Mode...\n");

                        uint8_t sniff_status = gap_sniff_mode_enter(connection_handle, 
                            16,            16,              4, 1 );
                      // minimum interval maximum interval
                        printf("[SNIFF] enter request status = 0x%02x\n", sniff_status);
                    }else{
                        printf("[SNIFF] ERROR: invalid connection handle\n");
                    }

                    break;
                }

                // --- 切断イベント ---
                case HCI_EVENT_DISCONNECTION_COMPLETE:{
                    printf("[HCI] Disconnection complete\n");
                    rfcomm_cid = 0;
                    rfcomm_channel = 0;
                    connection_handle = HCI_CON_HANDLE_INVALID;

                    client_start_inquiry();
                    //if (state != TC_OFF) client_start_inquiry();
                    break;
                }

                case RFCOMM_EVENT_CHANNEL_CLOSED:{
                    printf("[RFCOMM] Channel closed\n");
                    rfcomm_cid = 0;
                    rfcomm_channel = 0;
                    state = TC_OFF;
                    //if (state != TC_OFF) client_start_inquiry();
                    break;
                }

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

                default:{
                    break;
                }
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

            

            critical_section_enter_blocking(&cs_bt_data);
            share_data = controller;
            share_update = true;
            critical_section_exit(&cs_bt_data);

            #if DEBUG_LOG_BT

                int sum = 
                    controller.jyoutai + controller.L_x + controller.L_y +
                    controller.R_x    + controller.R_y  + controller.L2  +
                    controller.R2     + controller.key  + controller.boton;
                bool valid = ((sum % 255) + 1 == controller.checsam);
                static uint32_t callback_count = 0;
                static uint32_t byte_count = 0;
                static absolute_time_t last_log;

                callback_count++;
                byte_count += size;
                absolute_time_t now = get_absolute_time();

                if (absolute_time_diff_us(last_log, now) >= 1000000) {
                    printf("RFCOMM callbacks=%lu bytes=%lu last_size=%u\n",
                        callback_count,byte_count,size);
                    printf("[LastData] %s Lx:%3d,Rx:%3d,ste:%3d\n", valid ? "OK" : "NG",
                        controller.L_x,controller.R_x,controller.jyoutai);
                    callback_count = 0;
                    byte_count = 0;
                    last_log = now;
                }
            
                // printf("[RX] %s Data:%02x-%02x-%02x-%02x-%02x-%02x-%02x-%02x-%02x cek=%3d\n",
                //        valid ? "OK" : "NG",
                //        controller.L_x, controller.L_y,
                //        controller.R_x, controller.R_y,
                //        controller.L2,  controller.R2,
                //        controller.key, controller.boton, controller.jyoutai,
                //        controller.checsam);
                // printf("[RX] %s Lx:%3d,Rx:%3d,ste:%3d\n", valid ? "OK" : "NG",
                    // controller.L_y,controller.R_x,controller.jyoutai);
            #endif

            rfcomm_grant_credits(rfcomm_cid, 1);
            break;
        }

        default:
            break;
    }
}

void core1_entry(){
    bool aux_state = true;
    bool bt_get_state = false;
    bool e220_get_state = false;
    int e220_result = 0;
    bool get_new_data = false;
    ds4_data bt_get_data,e220_get_data,send_data;


    absolute_time_t t_now;
    while(true){
        t_now = get_absolute_time();

        //Bluetooth受信処理
        critical_section_enter_blocking(&cs_bt_data);
        if(share_update){
            bt_get_data = share_data;
            share_update = false;
            bt_get_state = true;
        }else{
            bt_get_state = false;
        }
        critical_section_exit(&cs_bt_data);

        int sum = bt_get_data.jyoutai + bt_get_data.L_x + bt_get_data.L_y +
            bt_get_data.R_x    + bt_get_data.R_y  + bt_get_data.L2  +
            bt_get_data.R2     + bt_get_data.key  + bt_get_data.boton +
            bt_get_data.seq_H  + bt_get_data.seq_L;
        bool valid_bt = ((sum % 255) + 1 == bt_get_data.checsam);

        #if DEBUG_LOG_BT
            if(bt_get_state){
                printf("[BTbr]Get:OK,Data:%d,%d,%d,%d,State:%d\n",
                    bt_get_data.L_x,bt_get_data.R_x,bt_get_data.L2,
                    bt_get_data.R2, bt_get_data.jyoutai);
            }

        #endif

        //e220 受信処理
        if(Lara1_readable()){
            e220_result = Lora1_get_data(&e220_get_data,500);
            if(e220_result == true){
                e220_get_state = true;
                #if DEBUG_LOG_LORA
                    printf("[LoRa]Get:OK,Data:%d,%d,%d,%d,State:%d\n",
                        e220_get_data.L_x,e220_get_data.R_x,e220_get_data.L2,
                        e220_get_data.R2, e220_get_data.jyoutai);
                #endif
            }else{
                e220_get_state = false;
                #if DEBUG_LOG_LOR
                    printf("[LoRa]undefined err code:%d",e220_result);
                #endif
            }
        }else{
            e220_get_state = false;
        }

        //統合処理
        if(bt_get_state){
          output_uart.sendLatest(bt_get_data);
        }
        
        if(e220_get_state){
            output_uart.sendLatest(e220_get_data);
        }

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

    gpio_init(ConectLED_D1);
    gpio_init(BlueLED_D2);
    gpio_init(Yellow_D3);
    gpio_set_dir(ConectLED_D1,GPIO_OUT);
    gpio_set_dir(BlueLED_D2,GPIO_OUT);
    gpio_set_dir(Yellow_D3,GPIO_OUT);
    gpio_put(Yellow_D3,true);
    Lora1_init();
    sleep_ms(500);

    output_uart.begin();

    critical_section_init(&cs_bt_data);
    multicore_launch_core1(core1_entry);

    if (cyw43_arch_init()) {
        printf("failed to initialise cyw43_arch\n");
        return -1;
    }

    l2cap_init();
    rfcomm_init();
    sdp_client_init();

    //bluetooth 設定
    hci_set_inquiry_mode(INQUIRY_MODE_RSSI_AND_EIR);
    gap_set_default_link_policy_settings(LM_LINK_POLICY_ENABLE_SNIFF_MODE); // Bluetooth Classic Sniff Modeを許可
    gap_ssp_set_authentication_requirement(SSP_IO_AUTHREQ_MITM_PROTECTION_NOT_REQUIRED_NO_BONDING);
    gap_ssp_set_io_capability(SSP_IO_CAPABILITY_NO_INPUT_NO_OUTPUT);

    hci_event_callback_registration.callback = &spp_client_packet_handler;
    hci_add_event_handler(&hci_event_callback_registration);

    hci_power_control(HCI_POWER_ON);
    btstack_run_loop_execute();

    return 0;
}