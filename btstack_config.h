#ifndef _PICO_BTSTACK_CONFIG_H
#define _PICO_BTSTACK_CONFIG_H

#include "btstack_config_common.h"

// ========================================================
// [追加] Classic (SPP/RFCOMM/SDP) 用メモリプール設定
// ========================================================

// RFCOMM関連
#define MAX_NR_RFCOMM_MULTIPLEXERS   1
#define MAX_NR_RFCOMM_SERVICES       1
#define MAX_NR_RFCOMM_CHANNELS       1

// L2CAP関連（RFCOMM下層 + SDP用）
#define MAX_NR_L2CAP_SERVICES        2   // RFCOMM + SDP Lookup
#define MAX_NR_L2CAP_CHANNELS        4   // 複数同時接続用

// SDP関連（クライアント側が SDP Query を実行するため必須）
#define MAX_NR_SDAP_CONNECTIONS      1
#define MAX_NR_SERVICE_RECORD_ITEMS  4

#endif
