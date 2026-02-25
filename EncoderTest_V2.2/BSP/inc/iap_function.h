#ifndef IAP_FUNCTION_H
#define IAP_FUNCTION_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Bootloader version string */
#define IAP_BL_VERSION_STR      "KV30_BL_1.0"

/* Application start address (skip bootloader space) */
#define IAP_APP_START_ADDR      (0x00005000u)

/* Flash memory configuration */
#define IAP_PFLASH_BASE         (0x00000000u)      /* Flash base address */
#define IAP_PFLASH_SIZE         (0x00020000u)      /* 128KB total flash */
#define IAP_PFLASH_SECTOR_SIZE  (2048u)            /* 2KB sector size */
#define IAP_FLASH_WRITE_UNIT    (8u)               /* 8-byte alignment */

/* Bootloader flag in SRAM (32KB SRAM: 0x20000000~0x20007FFF) */
#define IAP_FLAG_ADDR           (0x20000000u)
#define IAP_MAGIC_VALUE         (0x5AA55AA5u)

/* Maximum packet size for firmware transfer */
#define IAP_PACKET_MAX          (2048u)

/* Function declarations */
void IAP_RequestUpdate(void);
bool IAP_CheckAndClearRequest(void);
void IAP_JumpToApp(uint32_t appAddr);
void IAP_RunBootloader(void);
bool IAP_IsAppValid(uint32_t appAddr);
void IAP_ProtocolFeedByte(uint8_t b);
#ifdef __cplusplus
}
#endif

#endif /* IAP_FUNCTION_H */