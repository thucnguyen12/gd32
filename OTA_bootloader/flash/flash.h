#ifndef __FLASH_H
#define __FLASH_H

#include "gd32e23x.h"
#include "gd32e23x_fmc.h"

#define ERASE_FAILED 9

//FMC��ַ����
#define FMC_ADDR_MASK		  ((uint32_t)0x0800fc00)
//FMC��ʼ��ַ
#define FMC_ADDR_BASE		  ADDR_FMC_SECTOR_0			/* FMC����ʼ��ַ */
//FMC��С
#define FMC_ALL_SIZE		  ((uint8_t)64)				/* FMC�Ĵ�С,��λ:Kbytes */
/* FMC������С */
#define FMC_PAGE_SIZE         ((uint16_t)0x400)			/* ÿ��������С, 1 Kbytes */

/* FLASH ��������ʼ��ַ */
#define ADDR_FMC_SECTOR_0     ((uint32_t)0x08000000)	/* ����0��ʼ��ַ, 1 Kbytes */
#define ADDR_FMC_SECTOR_1     ((uint32_t)0x08000400)	/* ����1��ʼ��ַ, 1 Kbytes */
#define ADDR_FMC_SECTOR_2     ((uint32_t)0x08000800)	/* ����2��ʼ��ַ, 1 Kbytes */
#define ADDR_FMC_SECTOR_3     ((uint32_t)0x08000c00)	/* ����3��ʼ��ַ, 1 Kbytes */
#define ADDR_FMC_SECTOR_4     ((uint32_t)0x08001000)	/* ����4��ʼ��ַ, 1 Kbytes */
#define ADDR_FMC_SECTOR_5     ((uint32_t)0x08001400)	/* ����5��ʼ��ַ, 1 Kbytes */
#define ADDR_FMC_SECTOR_6     ((uint32_t)0x08001800)	/* ����6��ʼ��ַ, 1 Kbytes */
#define ADDR_FMC_SECTOR_7     ((uint32_t)0x08001c00)	/* ����7��ʼ��ַ, 1 Kbytes */
#define ADDR_FMC_SECTOR_8     ((uint32_t)0x08002000)	/* ����8��ʼ��ַ, 1 Kbytes */
#define ADDR_FMC_SECTOR_9     ((uint32_t)0x08002400)	/* ����9��ʼ��ַ, 1 Kbytes */
#define ADDR_FMC_SECTOR_10    ((uint32_t)0x08002800)	/* ����10��ʼ��ַ, 1 Kbytes */
#define ADDR_FMC_SECTOR_11    ((uint32_t)0x08002c00)	/* ����11��ʼ��ַ, 1 Kbytes */
#define ADDR_FMC_SECTOR_12    ((uint32_t)0x08003000)	/* ����12��ʼ��ַ, 1 Kbytes */
#define ADDR_FMC_SECTOR_13    ((uint32_t)0x08003400)	/* ����13��ʼ��ַ, 1 Kbytes */
#define ADDR_FMC_SECTOR_14    ((uint32_t)0x08003800)	/* ����14��ʼ��ַ, 1 Kbytes */
#define ADDR_FMC_SECTOR_15    ((uint32_t)0x08003c00)	/* ����15��ʼ��ַ, 1 Kbytes */
#define ADDR_FMC_SECTOR_16    ((uint32_t)0x08004000)	/* ����16��ʼ��ַ, 1 Kbytes */
#define ADDR_FMC_SECTOR_17    ((uint32_t)0x08004400)	/* ����17��ʼ��ַ, 1 Kbytes */
#define ADDR_FMC_SECTOR_18    ((uint32_t)0x08004800)	/* ����18��ʼ��ַ, 1 Kbytes */
#define ADDR_FMC_SECTOR_19    ((uint32_t)0x08004c00)	/* ����19��ʼ��ַ, 1 Kbytes */
#define ADDR_FMC_SECTOR_20    ((uint32_t)0x08005000)	/* ����20��ʼ��ַ, 1 Kbytes */
#define ADDR_FMC_SECTOR_21    ((uint32_t)0x08005400)	/* ����21��ʼ��ַ, 1 Kbytes */
#define ADDR_FMC_SECTOR_22    ((uint32_t)0x08005800)	/* ����22��ʼ��ַ, 1 Kbytes */
#define ADDR_FMC_SECTOR_23    ((uint32_t)0x08005c00)	/* ����23��ʼ��ַ, 1 Kbytes */
#define ADDR_FMC_SECTOR_24    ((uint32_t)0x08006000)	/* ����24��ʼ��ַ, 1 Kbytes */
#define ADDR_FMC_SECTOR_25    ((uint32_t)0x08006400)	/* ����25��ʼ��ַ, 1 Kbytes */
#define ADDR_FMC_SECTOR_26    ((uint32_t)0x08006800)	/* ����26��ʼ��ַ, 1 Kbytes */
#define ADDR_FMC_SECTOR_27    ((uint32_t)0x08006c00)	/* ����27��ʼ��ַ, 1 Kbytes */
#define ADDR_FMC_SECTOR_28    ((uint32_t)0x08007000)	/* ����28��ʼ��ַ, 1 Kbytes */
#define ADDR_FMC_SECTOR_29    ((uint32_t)0x08007400)	/* ����29��ʼ��ַ, 1 Kbytes */
#define ADDR_FMC_SECTOR_30    ((uint32_t)0x08007800)	/* ����30��ʼ��ַ, 1 Kbytes */
#define ADDR_FMC_SECTOR_31    ((uint32_t)0x08007c00)	/* ����31��ʼ��ַ, 1 Kbytes */
#define ADDR_FMC_SECTOR_32    ((uint32_t)0x08008000)	/* ����32��ʼ��ַ, 1 Kbytes */
#define ADDR_FMC_SECTOR_33    ((uint32_t)0x08008400)	/* ����33��ʼ��ַ, 1 Kbytes */
#define ADDR_FMC_SECTOR_34    ((uint32_t)0x08008800)	/* ����34��ʼ��ַ, 1 Kbytes */
#define ADDR_FMC_SECTOR_35    ((uint32_t)0x08008c00)	/* ����35��ʼ��ַ, 1 Kbytes */
#define ADDR_FMC_SECTOR_36    ((uint32_t)0x08009000)	/* ����36��ʼ��ַ, 1 Kbytes */
#define ADDR_FMC_SECTOR_37    ((uint32_t)0x08009400)	/* ����37��ʼ��ַ, 1 Kbytes */
#define ADDR_FMC_SECTOR_38    ((uint32_t)0x08009800)	/* ����38��ʼ��ַ, 1 Kbytes */
#define ADDR_FMC_SECTOR_39    ((uint32_t)0x08009c00)	/* ����39��ʼ��ַ, 1 Kbytes */
#define ADDR_FMC_SECTOR_40    ((uint32_t)0x0800a000)	/* ����40��ʼ��ַ, 1 Kbytes */
#define ADDR_FMC_SECTOR_41    ((uint32_t)0x0800a400)	/* ����41��ʼ��ַ, 1 Kbytes */
#define ADDR_FMC_SECTOR_42    ((uint32_t)0x0800a800)	/* ����42��ʼ��ַ, 1 Kbytes */
#define ADDR_FMC_SECTOR_43    ((uint32_t)0x0800ac00)	/* ����43��ʼ��ַ, 1 Kbytes */
#define ADDR_FMC_SECTOR_44    ((uint32_t)0x0800b000)	/* ����44��ʼ��ַ, 1 Kbytes */
#define ADDR_FMC_SECTOR_45    ((uint32_t)0x0800b400)	/* ����45��ʼ��ַ, 1 Kbytes */
#define ADDR_FMC_SECTOR_46    ((uint32_t)0x0800b800)	/* ����46��ʼ��ַ, 1 Kbytes */
#define ADDR_FMC_SECTOR_47    ((uint32_t)0x0800bc00)	/* ����47��ʼ��ַ, 1 Kbytes */
#define ADDR_FMC_SECTOR_48    ((uint32_t)0x0800c000)	/* ����48��ʼ��ַ, 1 Kbytes */
#define ADDR_FMC_SECTOR_49    ((uint32_t)0x0800c400)	/* ����49��ʼ��ַ, 1 Kbytes */
#define ADDR_FMC_SECTOR_50    ((uint32_t)0x0800c800)	/* ����50��ʼ��ַ, 1 Kbytes */
#define ADDR_FMC_SECTOR_51    ((uint32_t)0x0800cc00)	/* ����51��ʼ��ַ, 1 Kbytes */
#define ADDR_FMC_SECTOR_52    ((uint32_t)0x0800d000)	/* ����52��ʼ��ַ, 1 Kbytes */
#define ADDR_FMC_SECTOR_53    ((uint32_t)0x0800d400)	/* ����53��ʼ��ַ, 1 Kbytes */
#define ADDR_FMC_SECTOR_54    ((uint32_t)0x0800d800)	/* ����54��ʼ��ַ, 1 Kbytes */
#define ADDR_FMC_SECTOR_55    ((uint32_t)0x0800dc00)	/* ����55��ʼ��ַ, 1 Kbytes */
#define ADDR_FMC_SECTOR_56    ((uint32_t)0x0800e000)	/* ����56��ʼ��ַ, 1 Kbytes */
#define ADDR_FMC_SECTOR_57    ((uint32_t)0x0800e400)	/* ����57��ʼ��ַ, 1 Kbytes */
#define ADDR_FMC_SECTOR_58    ((uint32_t)0x0800e800)	/* ����58��ʼ��ַ, 1 Kbytes */
#define ADDR_FMC_SECTOR_59    ((uint32_t)0x0800ec00)	/* ����59��ʼ��ַ, 1 Kbytes */
#define ADDR_FMC_SECTOR_60    ((uint32_t)0x0800f000)	/* ����60��ʼ��ַ, 1 Kbytes */
#define ADDR_FMC_SECTOR_61    ((uint32_t)0x0800f400)	/* ����61��ʼ��ַ, 1 Kbytes */
#define ADDR_FMC_SECTOR_62    ((uint32_t)0x0800f800)	/* ����62��ʼ��ַ, 1 Kbytes */
#define ADDR_FMC_SECTOR_63    ((uint32_t)0x0800fc00)	/* ����63��ʼ��ַ, 1 Kbytes */

extern uint32_t fmc_sector_get(uint32_t address);
extern uint8_t fmc_erase_sector(uint32_t fmc_sector);
extern uint8_t fmc_write_word(uint32_t address, uint8_t *rxbuf, uint32_t rxsize);
extern void fmc_read_word(uint32_t address, uint16_t length, uint32_t* data_addr);
extern uint8_t fmc_erase(uint32_t address, uint8_t erase_page_num);		
extern uint8_t fmc_write_single(uint32_t address, uint32_t *rxbuf);

#endif