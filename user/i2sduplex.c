//ESP8266 I2S Input+Output

#include <esp8266.h>
#include "driver/slc_register.h"
#include "user_interface.h"
#include "driver/dmastuff.h"
#include "driver/pin_mux_register.h"
#include "i2sduplex.h"
#include "user.h"

//These contol the speed at which the bus comms.
#define WS_I2S_BCK 3  //Can't be less than 1.
#define WS_I2S_DIV 4

//I2S DMA buffer descriptors
static struct sdio_queue i2sBufDescRX[DMABUFFERDEPTH];
static struct sdio_queue i2sBufDescTX[1];
uint32_t i2sBDRX[I2SDMABUFLEN*DMABUFFERDEPTH];
//uint32_t i2sBDTX[I2SDMABUFLEN*DMABUFFERDEPTH];
int fxcycle;
int erx, etx;
uint32_t * curdma;
uint32_t * prvdma;
//uint8_t packet[140];
//jnlm: We don't know the segment number till packet 20.
uint8_t f20[20*140];
uint8_t camErr;
LOCAL void slc_isr(void) {
    //portBASE_TYPE HPTaskAwoken=0;
    struct sdio_queue *finishedDesc;
    uint32 slc_intr_status;
    int x;
    fxcycle++;
    static uint8_t bcnt = 0;
    static uint8_t seg = 0;
    static uint8_t pkt = 0;
    static uint8_t* packet;
    uint32_t header;
    slc_intr_status = READ_PERI_REG(SLC_INT_STATUS);
    //clear all intr flags
    WRITE_PERI_REG(SLC_INT_CLR, 0xffffffff);//slc_intr_status);

    //printf( "%08x\n", slc_intr_status );
    if ( (slc_intr_status & SLC_RX_EOF_INT_ST))
    {
        finishedDesc=(struct sdio_queue*)READ_PERI_REG(SLC_RX_EOF_DES_ADDR);
        prvdma = (uint8_t*)finishedDesc->buf_ptr;
        etx++;	//I know it's wacky, but the nomeclature is backwards, this is for TX packets in here.
    }
    if ( (slc_intr_status & SLC_TX_EOF_INT_ST))
    {
        finishedDesc=(struct sdio_queue*)READ_PERI_REG(SLC_TX_EOF_DES_ADDR);
        //finishedDesc=finishedDesc->next_link_ptr;

        //Don't know why - but this MUST be done, otherwise everything comes to a screeching halt.
        finishedDesc->owner=1;
        erx ++;
        curdma = (uint32_t*)finishedDesc->buf_ptr;
        if((((curdma[0]>>24) & 0xF) == 0x00)){
            pkt = (curdma[0] >> 16) & 0xFF;
            gpio_output_set((1 << 2),0, 0, 0);
            header = curdma[0];
            packet = curdma;
            for (int i=0; i< 20;++i){
                static uint16_t data[4];
                data[0] = (curdma[i*2+1] >> 16) & 0x3FFF;
                data[1] = (curdma[i*2+1] ) & 0x3FFF;
                data[2] = (curdma[i*2+2] >> 16) & 0x3FFF;
                data[3] = (curdma[i*2+2] ) & 0x3FFF;
                packet[i*7] = data[0] & 0xFF;
                packet[i*7+1] = (data[0] >> 8) | (data[1] << 6);
                packet[i*7+2] = (data[1] >> 2);
                packet[i*7+3] = (data[1] >> 10) | (data[2] << 4);
                packet[i*7+4] = (data[2] >> 4);
                packet[i*7+5] = (data[2] >> 12) | (data[3] << 2);
                packet[i*7+6] = (data[3] >> 6);
            }
            if( pkt == 20){
                uint8_t pseg = seg;
                seg = (header>>28) & 0xF;
                if (seg >2 && seg < 5){
                    if(erx < 1000 || (pseg + 1 !=seg))
                        camErr = 0;
                    erx = 0;
                }
                //os_printf("%02x\n",seg);
            }
            if(seg > 0 && seg < 5 && pkt >= 20 && pkt < 60){
                if(seg == 1 && pkt == 20 && capture_state == CAPTURE_START){
                    capture_state = CAPTURE_PROGRESS;
                }
                if(capture_state == CAPTURE_PROGRESS){
                    // Distribut the copying of the f20 into main buff over 20 packets
                    if(pkt < 40)
                        memcpy(buff+(seg-1)*280*30+140*(pkt-20),f20+140*(pkt-20),140);
                    memcpy(buff+pkt*140+(seg-1)*280*30,packet,140);
                }
                if (pkt == 59 && seg == 4){
                    capture_state = CAPTURE_DONE;
                }
            }else if(pkt < 20){
                //jnlm: Save all packets before packet 20 when we get segment number
                memcpy(f20+pkt*140,packet,140);
            }
        }

        //Nomaclature weird. this is actually RX packets.
        gpio_output_set(0,(1 << 2), 0, 0);
    }

}

//Initialize I2S subsystem for DMA circular buffer use
void ICACHE_FLASH_ATTR testi2s_init() {

    //Stop transmission
    CLEAR_PERI_REG_MASK(SLC_TX_LINK, SLC_TXLINK_START);
    CLEAR_PERI_REG_MASK(SLC_RX_LINK, SLC_RXLINK_START);
    CLEAR_PERI_REG_MASK(I2SCONF,I2S_I2S_TX_START|I2S_I2S_RX_START);

    WRITE_PERI_REG(PERIPHS_IO_MUX, 0x105);

    PIN_FUNC_SELECT(PERIPHS_IO_MUX_GPIO2_U, 0);
    gpio_output_set(0, 0, (1 << 2), 0);

    PIN_FUNC_SELECT(PERIPHS_IO_MUX_MTCK_U, 3);
    gpio_output_set(0, 0, (1 << 13), 0);
    gpio_output_set((1 << 13),0, 0, 0);

    int x, y;
    //Bits are shifted out

    //Initialize DMA buffer descriptors in such a way that they will form a circular
    //buffer.

    for (x=0; x<DMABUFFERDEPTH; x++) {
        i2sBufDescRX[x].owner=1;
        i2sBufDescRX[x].eof=1;
        i2sBufDescRX[x].sub_sof=0;
        i2sBufDescRX[x].datalen=I2SDMABUFLEN*4;
        i2sBufDescRX[x].blocksize=I2SDMABUFLEN*4;
        i2sBufDescRX[x].buf_ptr=(uint32_t)&i2sBDRX[x*I2SDMABUFLEN];
        i2sBufDescRX[x].unused=0;
        i2sBufDescRX[x].next_link_ptr=(int)((x<(DMABUFFERDEPTH-1))?(&i2sBufDescRX[x+1]):(&i2sBufDescRX[0]));
        for( y = 0; y < I2SDMABUFLEN; y++ )
        {
            i2sBDRX[y+x*I2SDMABUFLEN] = 0x00000000;
        }
    }

    i2sBufDescTX[0].owner=1;
    i2sBufDescTX[0].eof=1;
    i2sBufDescTX[0].sub_sof=0;
    i2sBufDescTX[0].datalen=0;
    i2sBufDescTX[0].blocksize=0;
    i2sBufDescTX[0].buf_ptr=0;
    i2sBufDescTX[0].unused=0;
    i2sBufDescTX[0].next_link_ptr=i2sBufDescTX;

    //Reset DMA )
    //SLC_TX_LOOP_TEST = IF this isn't set, SO will occasionally get unrecoverable errors when you underflow.
    //Originally this little tidbit was found at https://github.com/pvvx/esp8266web/blob/master/info/libs/bios/sip_slc.c
    //
    //I have not tried without SLC_AHBM_RST | SLC_AHBM_FIFO_RST.  I just assume they are useful?
    SET_PERI_REG_MASK(SLC_CONF0, SLC_TX_LOOP_TEST |SLC_RXLINK_RST|SLC_TXLINK_RST|SLC_AHBM_RST | SLC_AHBM_FIFO_RST);
    CLEAR_PERI_REG_MASK(SLC_CONF0, SLC_RXLINK_RST|SLC_TXLINK_RST|SLC_AHBM_RST | SLC_AHBM_FIFO_RST);

    //Clear DMA int flags
    SET_PERI_REG_MASK(SLC_INT_CLR,  0xffffffff);
    CLEAR_PERI_REG_MASK(SLC_INT_CLR,  0xffffffff);

    //Enable and configure DMA
    CLEAR_PERI_REG_MASK(SLC_CONF0, (SLC_MODE<<SLC_MODE_S));
    SET_PERI_REG_MASK(SLC_CONF0,(1<<SLC_MODE_S));

    CLEAR_PERI_REG_MASK(SLC_TX_LINK,SLC_TXLINK_DESCADDR_MASK);
    SET_PERI_REG_MASK(SLC_TX_LINK, ((uint32)&i2sBufDescRX[0]) & SLC_TXLINK_DESCADDR_MASK); //any random desc is OK, we don't use TX but it needs something valid
    CLEAR_PERI_REG_MASK(SLC_RX_LINK,SLC_RXLINK_DESCADDR_MASK);
    SET_PERI_REG_MASK(SLC_RX_LINK, ((uint32)&i2sBufDescTX[0]) & SLC_RXLINK_DESCADDR_MASK);

    //Attach the DMA interrupt
    ets_isr_attach(ETS_SLC_INUM, slc_isr,NULL);
    WRITE_PERI_REG(SLC_INT_ENA,  SLC_INTEREST_EVENT);

    //clear any interrupt flags that are set
    WRITE_PERI_REG(SLC_INT_CLR, 0xffffffff);
    ///enable DMA intr in cpu
    ets_isr_unmask(1<<ETS_SLC_INUM);

    //Init pins to i2s functions
    PIN_FUNC_SELECT(PERIPHS_IO_MUX_MTDI_U, FUNC_I2SI_DATA);
    //PIN_FUNC_SELECT(PERIPHS_IO_MUX_MTMS_U, FUNC_I2SI_WS);  //Dunno why - this is needed.  If it's not enabled, nothing will be read.

    PIN_FUNC_SELECT(PERIPHS_IO_MUX_U0RXD_U, FUNC_I2SO_DATA);
    //PIN_FUNC_SELECT(PERIPHS_IO_MUX_GPIO2_U, FUNC_I2SO_WS);
    PIN_FUNC_SELECT(PERIPHS_IO_MUX_MTDO_U, FUNC_I2SO_BCK);

    //Enable clock to i2s subsystem
    i2c_writeReg_Mask_def(i2c_bbpll, i2c_bbpll_en_audio_clock_out, 1);

    //Reset I2S subsystem
    CLEAR_PERI_REG_MASK(I2SCONF,I2S_I2S_RESET_MASK);
    SET_PERI_REG_MASK(I2SCONF,I2S_I2S_RESET_MASK);
    CLEAR_PERI_REG_MASK(I2SCONF,I2S_I2S_RESET_MASK);

    CLEAR_PERI_REG_MASK(I2S_FIFO_CONF, I2S_I2S_DSCR_EN|(I2S_I2S_RX_FIFO_MOD<<I2S_I2S_RX_FIFO_MOD_S)|(I2S_I2S_TX_FIFO_MOD<<I2S_I2S_TX_FIFO_MOD_S));
    SET_PERI_REG_MASK(I2S_FIFO_CONF, I2S_I2S_DSCR_EN);
    WRITE_PERI_REG(I2SRXEOF_NUM, RX_NUM);

    CLEAR_PERI_REG_MASK(I2SCONF_CHAN, (I2S_TX_CHAN_MOD<<I2S_TX_CHAN_MOD_S)|(I2S_RX_CHAN_MOD<<I2S_RX_CHAN_MOD_S));

    //some magic, don't ask, cause I donno
    SET_PERI_REG_MASK(I2STIMING, 0x1CFFFFF);

    //Clear int
    SET_PERI_REG_MASK(I2SINT_CLR,   I2S_I2S_TX_REMPTY_INT_CLR|I2S_I2S_TX_WFULL_INT_CLR|
            I2S_I2S_RX_WFULL_INT_CLR|I2S_I2S_PUT_DATA_INT_CLR|I2S_I2S_TAKE_DATA_INT_CLR);
    CLEAR_PERI_REG_MASK(I2SINT_CLR, I2S_I2S_TX_REMPTY_INT_CLR|I2S_I2S_TX_WFULL_INT_CLR|
            I2S_I2S_RX_WFULL_INT_CLR|I2S_I2S_PUT_DATA_INT_CLR|I2S_I2S_TAKE_DATA_INT_CLR);


    CLEAR_PERI_REG_MASK(I2SCONF, I2S_TRANS_SLAVE_MOD|I2S_RECE_SLAVE_MOD|
            (I2S_BITS_MOD<<I2S_BITS_MOD_S)|
            (I2S_BCK_DIV_NUM <<I2S_BCK_DIV_NUM_S)|
            (I2S_CLKM_DIV_NUM<<I2S_CLKM_DIV_NUM_S));

    SET_PERI_REG_MASK(I2SCONF, I2S_RIGHT_FIRST|I2S_MSB_RIGHT|
            I2S_RECE_MSB_SHIFT|I2S_TRANS_MSB_SHIFT|
            ((WS_I2S_BCK&I2S_BCK_DIV_NUM )<<I2S_BCK_DIV_NUM_S)|
            ((WS_I2S_DIV&I2S_CLKM_DIV_NUM)<<I2S_CLKM_DIV_NUM_S) );


    //enable int
    SET_PERI_REG_MASK(I2SINT_ENA,   I2S_I2S_TX_REMPTY_INT_ENA|I2S_I2S_TX_WFULL_INT_ENA|
            I2S_I2S_RX_REMPTY_INT_ENA|I2S_I2S_TX_PUT_DATA_INT_ENA|I2S_I2S_RX_TAKE_DATA_INT_ENA);


    ets_delay_us(50000);

    //WRITE_PERI_REG(PERIPHS_IO_MUX, 0x105);
    //PIN_FUNC_SELECT(PERIPHS_IO_MUX_MTCK_U, 3);
    //gpio_output_set(0, 0, (1 << 13), 0);
    //gpio_output_set(0,(1 << 13), 0, 0);
    gpio_output_set(0,(1 << 13), 0, 0);

    //ets_delay_us(1000);


    //Start transmission
    SET_PERI_REG_MASK(SLC_TX_LINK, SLC_TXLINK_START);
    SET_PERI_REG_MASK(SLC_RX_LINK, SLC_RXLINK_START);
    SET_PERI_REG_MASK(I2SCONF,I2S_I2S_TX_START|I2S_I2S_RX_START);

}


