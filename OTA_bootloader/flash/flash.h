#ifndef __FLASH_H
#define __FLASH_H

#include "gd32e23x.h"
#include "gd32e23x_fmc.h"

#define ERASE_FAILED 9

//FMC地址掩码
#define FMC_ADDR_MASK		  ((uint32_t)0x0800fc00)
//FMC起始地址
#define FMC_ADDR_BASE		  ADDR_FMC_SECTOR_0			/* FMC的起始地址 */
//FMC大小
#define FMC_ALL_SIZE		  ((uint8_t)64)				/* FMC的大小,单位:Kbytes */
/* FMC扇区大小 */
#define FMC_PAGE_SIZE         ((uint16_t)0x400)			/* 每个扇区大小, 1 Kbytes */

/* FLASH 扇区的起始地址 */
#define ADDR_FMC_SECTOR_0     ((uint32_t)0x08000000)	/* 扇区0起始地址, 1 Kbytes */
#define ADDR_FMC_SECTOR_1     ((uint32_t)0x08000400)	/* 扇区1起始地址, 1 Kbytes */
#define ADDR_FMC_SECTOR_2     ((uint32_t)0x08000800)	/* 扇区2起始地址, 1 Kbytes */
#define ADDR_FMC_SECTOR_3     ((uint32_t)0x08000c00)	/* 扇区3起始地址, 1 Kbytes */
#define ADDR_FMC_SECTOR_4     ((uint32_t)0x08001000)	/* 扇区4起始地址, 1 Kbytes */
#define ADDR_FMC_SECTOR_5     ((uint32_t)0x08001400)	/* 扇区5起始地址, 1 Kbytes */
#define ADDR_FMC_SECTOR_6     ((uint32_t)0x08001800)	/* 扇区6起始地址, 1 Kbytes */
#define ADDR_FMC_SECTOR_7     ((uint32_t)0x08001c00)	/* 扇区7起始地址, 1 Kbytes */
#define ADDR_FMC_SECTOR_8     ((uint32_t)0x08002000)	/* 扇区8起始地址, 1 Kbytes */
#define ADDR_FMC_SECTOR_9     ((uint32_t)0x08002400)	/* 扇区9起始地址, 1 Kbytes */
#define ADDR_FMC_SECTOR_10    ((uint32_t)0x08002800)	/* 扇区10起始地址, 1 Kbytes */
#define ADDR_FMC_SECTOR_11    ((uint32_t)0x08002c00)	/* 扇区11起始地址, 1 Kbytes */
#define ADDR_FMC_SECTOR_12    ((uint32_t)0x08003000)	/* 扇区12起始地址, 1 Kbytes */
#define ADDR_FMC_SECTOR_13    ((uint32_t)0x08003400)	/* 扇区13起始地址, 1 Kbytes */
#define ADDR_FMC_SECTOR_14    ((uint32_t)0x08003800)	/* 扇区14起始地址, 1 Kbytes */
#define ADDR_FMC_SECTOR_15    ((uint32_t)0x08003c00)	/* 扇区15起始地址, 1 Kbytes */
#define ADDR_FMC_SECTOR_16    ((uint32_t)0x08004000)	/* 扇区16起始地址, 1 Kbytes */
#define ADDR_FMC_SECTOR_17    ((uint32_t)0x08004400)	/* 扇区17起始地址, 1 Kbytes */
#define ADDR_FMC_SECTOR_18    ((uint32_t)0x08004800)	/* 扇区18起始地址, 1 Kbytes */
#define ADDR_FMC_SECTOR_19    ((uint32_t)0x08004c00)	/* 扇区19起始地址, 1 Kbytes */
#define ADDR_FMC_SECTOR_20    ((uint32_t)0x08005000)	/* 扇区20起始地址, 1 Kbytes */
#define ADDR_FMC_SECTOR_21    ((uint32_t)0x08005400)	/* 扇区21起始地址, 1 Kbytes */
#define ADDR_FMC_SECTOR_22    ((uint32_t)0x08005800)	/* 扇区22起始地址, 1 Kbytes */
#define ADDR_FMC_SECTOR_23    ((uint32_t)0x08005c00)	/* 扇区23起始地址, 1 Kbytes */
#define ADDR_FMC_SECTOR_24    ((uint32_t)0x08006000)	/* 扇区24起始地址, 1 Kbytes */
#define ADDR_FMC_SECTOR_25    ((uint32_t)0x08006400)	/* 扇区25起始地址, 1 Kbytes */
#define ADDR_FMC_SECTOR_26    ((uint32_t)0x08006800)	/* 扇区26起始地址, 1 Kbytes */
#define ADDR_FMC_SECTOR_27    ((uint32_t)0x08006c00)	/* 扇区27起始地址, 1 Kbytes */
#define ADDR_FMC_SECTOR_28    ((uint32_t)0x08007000)	/* 扇区28起始地址, 1 Kbytes */
#define ADDR_FMC_SECTOR_29    ((uint32_t)0x08007400)	/* 扇区29起始地址, 1 Kbytes */
#define ADDR_FMC_SECTOR_30    ((uint32_t)0x08007800)	/* 扇区30起始地址, 1 Kbytes */
#define ADDR_FMC_SECTOR_31    ((uint32_t)0x08007c00)	/* 扇区31起始地址, 1 Kbytes */
#define ADDR_FMC_SECTOR_32    ((uint32_t)0x08008000)	/* 扇区32起始地址, 1 Kbytes */
#define ADDR_FMC_SECTOR_33    ((uint32_t)0x08008400)	/* 扇区33起始地址, 1 Kbytes */
#define ADDR_FMC_SECTOR_34    ((uint32_t)0x08008800)	/* 扇区34起始地址, 1 Kbytes */
#define ADDR_FMC_SECTOR_35    ((uint32_t)0x08008c00)	/* 扇区35起始地址, 1 Kbytes */
#define ADDR_FMC_SECTOR_36    ((uint32_t)0x08009000)	/* 扇区36起始地址, 1 Kbytes */
#define ADDR_FMC_SECTOR_37    ((uint32_t)0x08009400)	/* 扇区37起始地址, 1 Kbytes */
#define ADDR_FMC_SECTOR_38    ((uint32_t)0x08009800)	/* 扇区38起始地址, 1 Kbytes */
#define ADDR_FMC_SECTOR_39    ((uint32_t)0x08009c00)	/* 扇区39起始地址, 1 Kbytes */
#define ADDR_FMC_SECTOR_40    ((uint32_t)0x0800a000)	/* 扇区40起始地址, 1 Kbytes */
#define ADDR_FMC_SECTOR_41    ((uint32_t)0x0800a400)	/* 扇区41起始地址, 1 Kbytes */
#define ADDR_FMC_SECTOR_42    ((uint32_t)0x0800a800)	/* 扇区42起始地址, 1 Kbytes */
#define ADDR_FMC_SECTOR_43    ((uint32_t)0x0800ac00)	/* 扇区43起始地址, 1 Kbytes */
#define ADDR_FMC_SECTOR_44    ((uint32_t)0x0800b000)	/* 扇区44起始地址, 1 Kbytes */
#define ADDR_FMC_SECTOR_45    ((uint32_t)0x0800b400)	/* 扇区45起始地址, 1 Kbytes */
#define ADDR_FMC_SECTOR_46    ((uint32_t)0x0800b800)	/* 扇区46起始地址, 1 Kbytes */
#define ADDR_FMC_SECTOR_47    ((uint32_t)0x0800bc00)	/* 扇区47起始地址, 1 Kbytes */
#define ADDR_FMC_SECTOR_48    ((uint32_t)0x0800c000)	/* 扇区48起始地址, 1 Kbytes */
#define ADDR_FMC_SECTOR_49    ((uint32_t)0x0800c400)	/* 扇区49起始地址, 1 Kbytes */
#define ADDR_FMC_SECTOR_50    ((uint32_t)0x0800c800)	/* 扇区50起始地址, 1 Kbytes */
#define ADDR_FMC_SECTOR_51    ((uint32_t)0x0800cc00)	/* 扇区51起始地址, 1 Kbytes */
#define ADDR_FMC_SECTOR_52    ((uint32_t)0x0800d000)	/* 扇区52起始地址, 1 Kbytes */
#define ADDR_FMC_SECTOR_53    ((uint32_t)0x0800d400)	/* 扇区53起始地址, 1 Kbytes */
#define ADDR_FMC_SECTOR_54    ((uint32_t)0x0800d800)	/* 扇区54起始地址, 1 Kbytes */
#define ADDR_FMC_SECTOR_55    ((uint32_t)0x0800dc00)	/* 扇区55起始地址, 1 Kbytes */
#define ADDR_FMC_SECTOR_56    ((uint32_t)0x0800e000)	/* 扇区56起始地址, 1 Kbytes */
#define ADDR_FMC_SECTOR_57    ((uint32_t)0x0800e400)	/* 扇区57起始地址, 1 Kbytes */
#define ADDR_FMC_SECTOR_58    ((uint32_t)0x0800e800)	/* 扇区58起始地址, 1 Kbytes */
#define ADDR_FMC_SECTOR_59    ((uint32_t)0x0800ec00)	/* 扇区59起始地址, 1 Kbytes */
#define ADDR_FMC_SECTOR_60    ((uint32_t)0x0800f000)	/* 扇区60起始地址, 1 Kbytes */
#define ADDR_FMC_SECTOR_61    ((uint32_t)0x0800f400)	/* 扇区61起始地址, 1 Kbytes */
#define ADDR_FMC_SECTOR_62    ((uint32_t)0x0800f800)	/* 扇区62起始地址, 1 Kbytes */
#define ADDR_FMC_SECTOR_63    ((uint32_t)0x0800fc00)	/* 扇区63起始地址, 1 Kbytes */

extern uint32_t fmc_sector_get(uint32_t address);
extern uint8_t fmc_erase_sector(uint32_t fmc_sector);
extern uint8_t fmc_write_word(uint32_t address, uint8_t *rxbuf, uint32_t rxsize);
extern void fmc_read_word(uint32_t address, uint16_t length, uint32_t* data_addr);
extern uint8_t fmc_erase(uint32_t address, uint8_t erase_page_num);		
extern uint8_t fmc_write_single(uint32_t address, uint32_t *rxbuf);

#endif