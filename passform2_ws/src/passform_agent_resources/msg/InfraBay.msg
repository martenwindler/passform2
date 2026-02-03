uint8 CONNECTED = 0
uint8 DISCONNECTED = 1
uint8 TIMEOUT = 2
uint8 UNKNOWN_ERROR = 3

uint8 bay_id  # internal ID of the bay connector. Starts at 1
uint8 connection_status # 0 connected, 1 disconnected, 2 timeout, 3 unknown error
string mac  # UNUSED. MAC address of the currently present module
string nfc_tag  # UNUSED. Identified NFC
string uuid  # bay uuid
string module_uuid  # UUID of the module currently present in the bay
InfraSupply supply  # currently active supply (power, vacuum, ...)
