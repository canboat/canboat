/*
 * Defines to interface with a Digital Yacht iKonvert
 */

#define IKONVERT_ASCII_PREFIX "$PDGY,"
#define IKONVERT_BINARY_PREFIX "!PDGY,"

#define PRIVATE_MSG IKONVERT_ASCII_PREFIX ",000000,"

#define ALIVE_MSG PRIVATE_MSG ",,,,,,"

#define RX_PGN_MSG_PREFIX IKONVERT_BINARY_PREFIX "%u,%u,%u,%u,%u.%u," // pgn, prio, src, dst, time followed by Base64 data
#define TX_PGN_MSG_PREFIX IKONVERT_BINARY_PREFIX "%u,%u,"             // pgn, dst followed by Base64 data
#define RX_NETWORK_STATUS_MSG PRIVATE_MSG "%u,%u,%u,%u,%u,%u"         // load, errors, count, uptime, addr, rejected tx pgn requests
#define TX_RESET_MSG IKONVERT_ASCII_PREFIX "N2NET_RESET"              // reset device incl tx/rx lists
#define TX_SET_RX_LIST_MSG IKONVERT_ASCII_PREFIX "RX_LIST"            // followed by [, <pgn>] ...
#define TX_SET_TX_LIST_MSG IKONVERT_ASCII_PREFIX "TX_LIST"            // followed by [, <pgn>] ...
#define TX_ONLINE_MSG IKONVERT_ASCII_PREFIX "N2NET_INIT"              // Setup RX+TX list first
#define TX_OFFLINE_MSG IKONVERT_ASCII_PREFIX "N2NET_OFFLINE"          // Setup RX+TX list first
#define TX_SETUP_MSG IKONVERT_ASCII_PREFIX "N2NET_SETUP,%u,%u,%u,%u,," // Instance, Address, Baud, Checksum (A/V/nil
#define RX_NAK_MSG IKONVERT_ASCII_PREFIX "NAK,%u,"                     // Error # followed by string
#define RX_ACK_MSG IKONVERT_ASCII_PREFIX "ACK,%s"                      // Acknowledgement of command

#define BAUD_RATE 230400
