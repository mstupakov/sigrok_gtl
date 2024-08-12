msg_id_db = {
    0x0B00: "GATTM_ADD_SVC_REQ"              ,
    0x0B01: "GATTM_ADD_SVC_RSP"              ,
    0x0B02: "GATTM_SVC_GET_PERMISSION_REQ"   ,
    0x0B03: "GATTM_SVC_GET_PERMISSION_RSP"   ,
    0x0B04: "GATTM_SVC_SET_PERMISSION_REQ"   ,
    0x0B05: "GATTM_SVC_SET_PERMISSION_RSP"   ,
    0x0B06: "GATTM_ATT_GET_PERMISSION_REQ"   ,
    0x0B07: "GATTM_ATT_GET_PERMISSION_RSP"   ,
    0x0B08: "GATTM_ATT_SET_PERMISSION_REQ"   ,
    0x0B09: "GATTM_ATT_SET_PERMISSION_RSP"   ,
    0x0B0A: "GATTM_ATT_GET_VALUE_REQ"        ,
    0x0B0B: "GATTM_ATT_GET_VALUE_RSP"        ,
    0x0B0C: "GATTM_ATT_SET_VALUE_REQ"        ,
    0x0B0D: "GATTM_ATT_SET_VALUE_RSP"        ,
    0x0B0E: "GATTM_DESTROY_DB_REQ"           ,
    0x0B0F: "GATTM_DESTROY_DB_RSP"           ,
    0x0B10: "GATTM_SVC_GET_LIST_REQ"         ,
    0x0B11: "GATTM_SVC_GET_LIST_RSP"         ,
    0x0B12: "GATTM_ATT_GET_INFO_REQ"         ,
    0x0B13: "GATTM_ATT_GET_INFO_RSP"         ,
    0x0C00: "GATTC_CMP_EVT"                  ,
    0x0C01: "GATTC_EXC_MTU_CMD"              ,
    0x0C02: "GATTC_MTU_CHANGED_IND"          ,
    0x0C03: "GATTC_DISC_CMD"                 ,
    0x0C04: "GATTC_DISC_SVC_IND"             ,
    0x0C05: "GATTC_DISC_SVC_INCL_IND"        ,
    0x0C06: "GATTC_DISC_CHAR_IND"            ,
    0x0C07: "GATTC_DISC_CHAR_DESC_IND"       ,
    0x0C08: "GATTC_READ_CMD"                 ,
    0x0C09: "GATTC_READ_IND"                 ,
    0x0C0A: "GATTC_WRITE_CMD"                ,
    0x0C0B: "GATTC_EXECUTE_WRITE_CMD"        ,
    0x0C0C: "GATTC_EVENT_IND"                ,
    0x0C0D: "GATTC_EVENT_REQ_IND"            ,
    0x0C0E: "GATTC_EVENT_CFM"                ,
    0x0C0F: "GATTC_REG_TO_PEER_EVT_CMD"      ,
    0x0C10: "GATTC_SEND_EVT_CMD"             ,
    0x0C11: "GATTC_SEND_SVC_CHANGED_CMD"     ,
    0x0C12: "GATTC_SVC_CHANGED_CFG_IND"      ,
    0x0C13: "GATTC_READ_REQ_IND"             ,
    0x0C14: "GATTC_READ_CFM"                 ,
    0x0C15: "GATTC_WRITE_REQ_IND"            ,
    0x0C16: "GATTC_WRITE_CFM"                ,
    0x0C17: "GATTC_ATT_INFO_REQ_IND"         ,
    0x0C18: "GATTC_ATT_INFO_CFM"             ,
    0x0C19: "GATTC_SDP_SVC_DISC_CMD"         ,
    0x0C1A: "GATTC_SDP_SVC_IND"              ,
    0x0C1B: "GATTC_TRANSACTION_TO_ERROR_IND" ,
    0x0C1C: "GATTC_CLIENT_RTX_IND"           ,
    0x0C1D: "GATTC_SERVER_RTX_IND"           ,
    0x0D00: "GAPM_CMP_EVT"                   ,
    0x0D01: "GAPM_DEVICE_READY_IND"          ,
    0x0D02: "GAPM_RESET_CMD"                 ,
    0x0D03: "GAPM_CANCEL_CMD"                ,
    0x0D04: "GAPM_SET_DEV_CONFIG_CMD"        ,
    0x0D05: "GAPM_SET_CHANNEL_MAP_CMD"       ,
    0x0D06: "GAPM_GET_DEV_INFO_CMD"          ,
    0x0D07: "GAPM_DEV_VERSION_IND"           ,
    0x0D08: "GAPM_DEV_BDADDR_IND"            ,
    0x0D09: "GAPM_DEV_ADV_TX_POWER_IND"      ,
    0x0D0A: "GAPM_DBG_MEM_INFO_IND"          ,
    0x0D0B: "GAPM_WHITE_LIST_MGT_CMD"        ,
    0x0D0C: "GAPM_WHITE_LIST_SIZE_IND"       ,
    0x0D0D: "GAPM_START_ADVERTISE_CMD"       ,
    0x0D0E: "GAPM_UPDATE_ADVERTISE_DATA_CMD" ,
    0x0D0F: "GAPM_START_SCAN_CMD"            ,
    0x0D10: "GAPM_ADV_REPORT_IND"            ,
    0x0D11: "GAPM_START_CONNECTION_CMD"      ,
    0x0D12: "GAPM_PEER_NAME_IND"             ,
    0x0D13: "GAPM_CONNECTION_CFM"            ,
    0x0D14: "GAPM_RESOLV_ADDR_CMD"           ,
    0x0D15: "GAPM_ADDR_SOLVED_IND"           ,
    0x0D16: "GAPM_GEN_RAND_ADDR_CMD"         ,
    0x0D17: "GAPM_USE_ENC_BLOCK_CMD"         ,
    0x0D18: "GAPM_USE_ENC_BLOCK_IND"         ,
    0x0D19: "GAPM_GEN_RAND_NB_CMD"           ,
    0x0D1A: "GAPM_GEN_RAND_NB_IND"           ,
    0x0D1B: "GAPM_PROFILE_TASK_ADD_CMD"      ,
    0x0D1C: "GAPM_PROFILE_ADDED_IND"         ,
    0x0D1D: "GAPM_UNKNOWN_TASK_IND"          ,
    0x0D1E: "GAPM_SUGG_DFLT_DATA_LEN_IND"    ,
    0x0D1F: "GAPM_MAX_DATA_LEN_IND"          ,
    0x0D20: "GAPM_RAL_MGT_CMD"               ,
    0x0D21: "GAPM_RAL_SIZE_IND"              ,
    0x0D22: "GAPM_RAL_ADDR_IND"              ,
    0x0D23: "GAPM_LIM_DISC_TO_IND"           ,
    0x0D24: "GAPM_SCAN_TO_IND"               ,
    0x0D25: "GAPM_ADDR_RENEW_TO_IND"         ,
    0x0D26: "GAPM_UNKNOWN_TASK_MSG"          ,
    0x0D27: "GAPM_USE_P256_BLOCK_CMD"        ,
    0x0D28: "GAPM_USE_P256_BLOCK_IND"        ,
    0x0E00: "GAPC_CMP_EVT"                   ,
    0x0E01: "GAPC_CONNECTION_REQ_IND"        ,
    0x0E02: "GAPC_CONNECTION_CFM"            ,
    0x0E03: "GAPC_DISCONNECT_IND"            ,
    0x0E04: "GAPC_DISCONNECT_CMD"            ,
    0x0E05: "GAPC_GET_INFO_CMD"              ,
    0x0E06: "GAPC_PEER_ATT_INFO_IND"         ,
    0x0E07: "GAPC_PEER_VERSION_IND"          ,
    0x0E08: "GAPC_PEER_FEATURES_IND"         ,
    0x0E09: "GAPC_CON_RSSI_IND"              ,
    0x0E0A: "GAPC_GET_DEV_INFO_REQ_IND"      ,
    0x0E0B: "GAPC_GET_DEV_INFO_CFM"          ,
    0x0E0C: "GAPC_SET_DEV_INFO_REQ_IND"      ,
    0x0E0D: "GAPC_SET_DEV_INFO_CFM"          ,
    0x0E0E: "GAPC_PARAM_UPDATE_CMD"          ,
    0x0E0F: "GAPC_PARAM_UPDATE_REQ_IND"      ,
    0x0E10: "GAPC_PARAM_UPDATE_CFM"          ,
    0x0E11: "GAPC_PARAM_UPDATED_IND"         ,
    0x0E12: "GAPC_BOND_CMD"                  ,
    0x0E13: "GAPC_BOND_REQ_IND"              ,
    0x0E14: "GAPC_BOND_CFM"                  ,
    0x0E15: "GAPC_BOND_IND"                  ,
    0x0E16: "GAPC_ENCRYPT_CMD"               ,
    0x0E17: "GAPC_ENCRYPT_REQ_IND"           ,
    0x0E18: "GAPC_ENCRYPT_CFM"               ,
    0x0E19: "GAPC_ENCRYPT_IND"               ,
    0x0E1A: "GAPC_SECURITY_CMD"              ,
    0x0E1B: "GAPC_SECURITY_IND"              ,
    0x0E1C: "GAPC_SIGN_COUNTER_IND"          ,
    0x0E1D: "GAPC_CON_CHANNEL_MAP_IND"       ,
    0x0E1E: "GAPC_LECB_CREATE_CMD"           ,
    0x0E1F: "GAPC_LECB_DESTROY_CMD"          ,
    0x0E20: "GAPC_LECB_CONNECT_CMD"          ,
    0x0E21: "GAPC_LECB_CONNECT_REQ_IND"      ,
    0x0E22: "GAPC_LECB_CONNECT_IND"          ,
    0x0E23: "GAPC_LECB_CONNECT_CFM"          ,
    0x0E24: "GAPC_LECB_ADD_CMD"              ,
    0x0E25: "GAPC_LECB_ADD_IND"              ,
    0x0E26: "GAPC_LECB_DISCONNECT_CMD"       ,
    0x0E27: "GAPC_LECB_DISCONNECT_IND"       ,
    0x0E28: "GAPC_SET_LE_PING_TO_CMD"        ,
    0x0E29: "GAPC_LE_PING_TO_VAL_IND"        ,
    0x0E2A: "GAPC_LE_PING_TO_IND"            ,
    0x0E2B: "GAPC_SET_LE_PKT_SIZE_CMD"       ,
    0x0E2C: "GAPC_LE_PKT_SIZE_IND"           ,
    0x0E2D: "GAPC_SIGN_CMD"                  ,
    0x0E2E: "GAPC_SIGN_IND"                  ,
    0x0E2F: "GAPC_PARAM_UPDATE_TO_IND"       ,
    0x0E30: "GAPC_SMP_TIMEOUT_TIMER_IND"     ,
    0x0E31: "GAPC_SMP_REP_ATTEMPTS_TIMER_IND",
    0x0E32: "GAPC_LECB_CONN_TO_IND"          ,
    0x0E33: "GAPC_LECB_DISCONN_TO_IND"       ,
    0x0E34: "GAPC_KEYPRESS_NOTIFICATION"     ,
    0x0E35: "GAPC_KEYPRESS_NOTIFICATION_CMD" ,
    0x0E36: "GAPC_KEYPRESS_NOTIFICATION_IND" ,
    0x0F01: "SYS_APP_WRITE_CMD_REQ_IND"      ,
    0xA001: "APP_GEN_RAND_REQ"               ,
    0xA002: "APP_GEN_RAND_RSP"               ,
    0xA003: "APP_GET_FW_VERSION"             ,
    0xA004: "APP_FW_VERSION_IND"             ,
    0xA005: "APP_BOOT_FROM_EXTERNAL_HOST_CMD",
    0xA006: "APP_BOOT_FROM_EXTERNAL_HOST_IND",
}

err_code_db = {
    0x00: "NO_ERROR"                           ,
    0x40: "GAP_ERR_INVALID_PARAM"              ,
    0x41: "GAP_ERR_PROTOCOL_PROBLEM"           ,
    0x42: "GAP_ERR_NOT_SUPPORTED"              ,
    0x43: "GAP_ERR_COMMAND_DISALLOWED"         ,
    0x44: "GAP_ERR_CANCELED"                   ,
    0x45: "GAP_ERR_TIMEOUT"                    ,
    0x46: "GAP_ERR_DISCONNECTED"               ,
    0x47: "GAP_ERR_NOT_FOUND"                  ,
    0x48: "GAP_ERR_REJECTED"                   ,
    0x49: "GAP_ERR_PRIVACY_CFG_PB"             ,
    0x4A: "GAP_ERR_ADV_DATA_INVALID"           ,
    0x50: "GATT_INVALID_PARAM_ERR"             ,
    0x51: "GATT_ERR_INVALID_ATT_LEN"           ,
    0x52: "GATT_ERR_INSUF_RESOURCE"            ,
    0x53: "GATT_ERR_INVALID_TYPE_IN_SVC_SEARCH",
    0x54: "GATT_ERR_WRITE"                     ,
    0x55: "GATT_ERR_SIGNED_WRITE"              ,
    0x56: "GATT_ERR_PROTOCOL_PROBLEM"          ,
    0x57: "GATT_ERR_TIMEOUT"                   ,
    0x58: "GATT_ERR_ATT_CLIENT_MISSING"        ,
    0x59: "GATT_ERR_ATT_SERVER_MISSING"        ,
    0x61: "SMP_ERROR_LOC_PASSKEY_ENTRY_FAILED" ,
    0x62: "SMP_ERROR_LOC_OOB_NOT_AVAILABLE"    ,
    0x63: "SMP_ERROR_LOC_AUTH_REQ"             ,
    0x64: "SMP_ERROR_LOC_CONF_VAL_FAILED"      ,
    0x65: "SMP_ERROR_LOC_PAIRING_NOT_SUPP"     ,
    0x66: "SMP_ERROR_LOC_ENC_KEY_SIZE"         ,
    0x67: "SMP_ERROR_LOC_CMD_NOT_SUPPORTED"    ,
    0x68: "SMP_ERROR_LOC_UNSPECIFIED_REASON"   ,
    0x69: "SMP_ERROR_LOC_REPEATED_ATTEMPTS"    ,
    0x6A: "SMP_ERROR_LOC_INVALID_PARAM"        ,
    0x6B: "SMP_ERROR_LOC_REQ_DISALLOWED"       ,
    0x6C: "SMP_ERROR_LOC_LL_ERROR"             ,
    0x6D: "SMP_ERROR_LOC_ADDR_RESOLV_FAIL"     ,
    0x6E: "SMP_ERROR_LOC_SIGN_VERIF_FAIL"      ,
    0x6F: "SMP_ERROR_LOC_TIMEOUT"              ,
    0x71: "SMP_ERROR_REM_PASSKEY_ENTRY_FAILED" ,
    0x72: "SMP_ERROR_REM_OOB_NOT_AVAILABLE"    ,
    0x73: "SMP_ERROR_REM_AUTH_REQ"             ,
    0x74: "SMP_ERROR_REM_CONF_VAL_FAILED"      ,
    0x75: "SMP_ERROR_REM_PAIRING_NOT_SUPP"     ,
    0x76: "SMP_ERROR_REM_ENC_KEY_SIZE"         ,
    0x77: "SMP_ERROR_REM_CMD_NOT_SUPPORTED"    ,
    0x78: "SMP_ERROR_REM_UNSPECIFIED_REASON"   ,
    0x79: "SMP_ERROR_REM_REPEATED_ATTEMPTS"    ,
    0x7A: "SMP_ERROR_REM_INVALID_PARAM"        ,
    0x7B: "SMP_ERROR_REM_ENC_KEY_MISSING"      ,
    0x7C: "SMP_ERROR_REM_ENC_NOT_SUPPORTED"    ,
    0x7D: "SMP_ERROR_REM_ENC_TIMEOUT"          ,
    0x01: "ATT_ERR_INVALID_HANDLE"             ,
    0x02: "ATT_ERR_READ_NOT_PERMITTED"         ,
    0x03: "ATT_ERR_WRITE_NOT_PERMITTED"        ,
    0x04: "ATT_ERR_INVALID_PDU"                ,
    0x05: "ATT_ERR_INSUFF_AUTHEN"              ,
    0x06: "ATT_ERR_REQUEST_NOT_SUPPORTED"      ,
    0x07: "ATT_ERR_INVALID_OFFSET"             ,
    0x08: "ATT_ERR_INSUFF_AUTHOR"              ,
    0x09: "ATT_ERR_PREPARE_QUEUE_FULL"         ,
    0x0A: "ATT_ERR_ATTRIBUTE_NOT_FOUND"        ,
    0x0B: "ATT_ERR_ATTRIBUTE_NOT_LONG"         ,
    0x0C: "ATT_ERR_INSUFF_ENC_KEY_SIZE"        ,
    0x0D: "ATT_ERR_INVALID_ATTRIBUTE_VAL_LEN"  ,
    0x0E: "ATT_ERR_UNLIKELY_ERR"               ,
    0x0F: "ATT_ERR_INSUFF_ENC"                 ,
    0x10: "ATT_UNSUPP_GRP_TYPE"                ,
    0x11: "ATT_INSUFF_RESOURCE"                ,
    0x80: "ATT_ERR_APP_ERROR"                  ,
    0x80: "PRF_APP_ERROR"                      ,
    0x81: "PRF_ERR_INVALID_PARAM"              ,
    0x82: "PRF_ERR_INEXISTENT_HDL"             ,
    0x83: "PRF_ERR_STOP_DISC_CHAR_MISSING"     ,
    0x84: "PRF_ERR_MULTIPLE_SVC"               ,
    0x85: "PRF_ERR_STOP_DISC_WRONG_CHAR_PROP"  ,
    0x86: "PRF_ERR_MULTIPLE_CHAR"              ,
    0x87: "PRF_ERR_NOT_WRITABLE"               ,
    0x88: "PRF_ERR_NOT_READABLE"               ,
    0x89: "PRF_ERR_REQ_DISALLOWED"             ,
    0x8A: "PRF_ERR_NTF_DISABLED"               ,
    0x8B: "PRF_ERR_IND_DISABLED"               ,
    0x8C: "PRF_ERR_FEATURE_NOT_SUPPORTED"      ,
    0x8D: "PRF_ERR_UNEXPECTED_LEN"             ,
    0x8E: "PRF_ERR_DISCONNECTED"               ,
    0xFD: "PRF_CCCD_IMPR_CONFIGURED"           ,
    0xFE: "PRF_PROC_IN_PROGRESS"               ,
    0xFF: "PRF_OUT_OF_RANGE"                   ,
}

gapm_operation_db = {
    0x00: "GAPM_NO_OP"                         ,
    0x01: "GAPM_RESET"                         ,
    0x02: "GAPM_CANCEL"                        ,
    0x03: "GAPM_SET_DEV_CONFIG"                ,
    0x04: "GAPM_SET_CHANNEL_MAP"               ,
    0x05: "GAPM_GET_DEV_VERSION"               ,
    0x06: "GAPM_GET_DEV_BDADDR"                ,
    0x07: "GAPM_GET_DEV_ADV_TX_POWER"          ,
    0x08: "GAPM_GET_WLIST_SIZE"                ,
    0x09: "GAPM_ADD_DEV_IN_WLIST"              ,
    0x0A: "GAPM_RMV_DEV_FRM_WLIST"             ,
    0x0B: "GAPM_CLEAR_WLIST"                   ,
    0x0C: "GAPM_ADV_NON_CONN"                  ,
    0x0D: "GAPM_ADV_UNDIRECT"                  ,
    0x0E: "GAPM_ADV_DIRECT"                    ,
    0x0F: "GAPM_ADV_DIRECT_LDC"                ,
    0x10: "GAPM_UPDATE_ADVERTISE_DATA"         ,
    0x11: "GAPM_SCAN_ACTIVE"                   ,
    0x12: "GAPM_SCAN_PASSIVE"                  ,
    0x13: "GAPM_CONNECTION_DIRECT"             ,
    0x14: "GAPM_CONNECTION_AUTO"               ,
    0x15: "GAPM_CONNECTION_SELECTIVE"          ,
    0x16: "GAPM_CONNECTION_NAME_REQUEST"       ,
    0x17: "GAPM_RESOLV_ADDR"                   ,
    0x18: "GAPM_GEN_RAND_ADDR"                 ,
    0x19: "GAPM_USE_ENC_BLOCK"                 ,
    0x1A: "GAPM_GEN_RAND_NB"                   ,
    0x27: "GAPM_USE_P256_BLOCK"                ,
    0x1B: "GAPM_PROFILE_TASK_ADD"              ,
    0x1C: "GAP_DBG_GET_MEM_INFO"               ,
    0x1D: "GAPM_PLF_RESET"                     ,
    0x1E: "GAPM_SET_SUGGESTED_DFLT_LE_DATA_LEN",
    0x1F: "GAPM_GET_SUGGESTED_DFLT_LE_DATA_LEN",
    0x20: "GAPM_GET_MAX_LE_DATA_LEN"           ,
    0x21: "GAPM_GET_RAL_SIZE"                  ,
    0x22: "GAPM_GET_RAL_LOC_ADDR"              ,
    0x23: "GAPM_GET_RAL_PEER_ADDR"             ,
    0x24: "GAPM_ADD_DEV_IN_RAL"                ,
    0x25: "GAPM_RMV_DEV_FRM_RAL"               ,
    0x26: "GAPM_CLEAR_RAL"                     ,
}
