//Copyright 2016 <>< Charles Lohr, See LICENSE file.

#ifndef _I2SDUPLEX_TEST
#define _I2SDUPLEX_TEST


//Stuff that should be for the header:


#define DMABUFFERDEPTH 2
#define I2SDMABUFLEN (41)
#define LINE32LEN I2SDMABUFLEN
#define RX_NUM (I2SDMABUFLEN)

extern uint32_t i2sBDRX[I2SDMABUFLEN*DMABUFFERDEPTH];
extern uint32_t i2sBDTX[I2SDMABUFLEN*DMABUFFERDEPTH];

extern int fxcycle;
extern int erx, etx;

void ICACHE_FLASH_ATTR testi2s_init();

#endif

