
///
/// @file	flash.c
///
/// Flash-related data structures and functions, including the application
/// signature for the bootloader.
///
#include <stdint.h>
#include "flash_layout.h"
#include "em_msc.h"
#include "flash.h"

// The application signature block.
//
// The presence of this block, which is the first thing to be erased and the
// last thing to be programmed during an update, tells the bootloader that
// a valid application is installed.
// TODO place this section in the linker file
//  /* placing my named section at given address: */
//  .IDDATA 0x0001EFFE :
//  {
//    KEEP(*(.IDDATA)) /* keep my variable even if not referenced */
//  } > FLASH

//static const uint8_t __attribute__((section (".IDDATA"))) app_signature[2] = { FLASH_SIG0, FLASH_SIG1 };

static void CheckInit(void)
{
    static bool Init = false;
    if(!Init)
    {
        MSC_Init();
        Init = true;
        //(void)app_signature;
    }
}

void flash_erase_scratch(void)
{
    CheckInit();
    MSC_ErasePage((uint32_t *)FLASH_SCRATCH);
}

uint8_t flash_read_scratch(uint16_t address)
{
    uint8_t	d=0;
    d = *(uint8_t *)(FLASH_SCRATCH | address);
    return d;
}

typedef union {
    uint32_t Long;
    uint8_t Byte[4];
} Longin_t;

void flash_write_scratch(uint16_t address, uint8_t c)
{
    // only allows 4 byte writes, so must read other four bytes update required byte and write back
    flash_write_byte(FLASH_SCRATCH | address,c);
}
void flash_write_byte(uint32_t address, uint8_t c)
{
    CheckInit();
    uint32_t Adr;
    Longin_t Val;
    Adr = (address&~0x00000003);
    Val.Long = *((uint32_t *)Adr);
    Val.Byte[address&0x03] = c;
    MSC_WriteWord((uint32_t*)Adr,&(Val.Long),4);
}

void FlashLockBlock(uint8_t * Address)
{
    uint32_t page = ((uint32_t) Address)/FLASH_PAGE_SIZE; // find the page number
    uint32_t PLWIdx,PLWMask;
    uint32_t *PLW;
    uint32_t  val;
    // PLW[0] has pages 0-31, PLW[1] has page 32-63...
    if(page<=31)
    {
        PLWIdx = 0;
    }
    else
    {
        PLWIdx = 1;
        page >>= 1;
    }
    PLWMask = 1UL<<(page&0x1F);		// Clear bits above 31 and shift to get bit mask
    PLW = (uint32_t *)LOCKBITS_BASE;					// set base address of registers
    val = (PLW[PLWIdx]&~PLWMask);			// clear bit to lock flash block
    CheckInit();
    MSC_WriteWord(&PLW[PLWIdx],(void*)&val,sizeof(uint32_t));
}

