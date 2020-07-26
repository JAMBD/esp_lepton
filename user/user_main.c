/*
 * ----------------------------------------------------------------------------
 * "THE BEER-WARE LICENSE" (Revision 42):
 * Jeroen Domburg <jeroen@spritesmods.com> wrote this file. As long as you retain 
 * this notice you can do whatever you want with this stuff. If we meet some day, 
 * and you think this stuff is worth it, you can buy me a beer in return. 
 * ----------------------------------------------------------------------------
 */

/*
   This is example code for the esphttpd library. It's a small-ish demo showing off 
   the server, including WiFi connection management capabilities, some IO and
   some pictures of cats.
 */

#include <esp8266.h>
#include "httpd.h"
#include "io.h"
#include "httpdespfs.h"
#include "cgi.h"
#include "cgiwifi.h"
#include "cgiflash.h"
#include "user_interface.h"
#include "stdout.h"
#include "auth.h"
#include "espfs.h"
#include "captdns.h"
#include "i2sduplex.h"
#include "webpages-espfs.h"
#include "cgiwebsocket.h"
#include "cgi-test.h"
#include "osapi.h"
#include "gpio.h"
#include "driver/slc_register.h"
#include "driver/spi.h"
#include "driver/i2c.h"
#include "driver/sdio_slv.h"
#include "driver/spi_interface.h"
#include "user.h"
//The example can print out the heap use every 3 seconds. You can use this to catch memory leaks.
#define SHOW_HEAP_USE


#define ADDRESS  (0x2A)

#define AGC (0x01)
#define SYS (0x02)
#define VID (0x03)
#define OEM (0x08)
#define RAD (0x0E)

#define GET (0x00)
#define SET (0x01)
#define RUN (0x02)

#define PACKET_SIZE 3360
#define NUM_PARTS 10

//Function that tells the authentication system what users/passwords live on the system.
//This is disabled in the default build; if you want to try it, enable the authBasic line in
//the builtInUrls below.
int ICACHE_FLASH_ATTR myPassFn(HttpdConnData *connData, int no, char *user, int userLen, char *pass, int passLen) {
    if (no==0) {
        os_strcpy(user, "admin");
        os_strcpy(pass, "admin");
        return 1;
        //Add more users this way. Check against incrementing no for each user added.
        //	} else if (no==1) {
        //		os_strcpy(user, "user1");
        //		os_strcpy(pass, "something");
        //		return 1;
}
return 0;
}

static ETSTimer websockTimer;

uint32_t hdr=0;
int8_t part = 0;
uint8_t web_buff[280*120+4+14];
uint8_t* obuff = (uint8_t*)&(web_buff[14]);
uint8_t* buff = (uint8_t*)&(web_buff[4 + 14]);
uint8_t lock = 0;
uint8_t go_flag = 0;
Websock* local_ws = NULL;

uint8_t tmp_data[14];
uint16_t frame_cnt = 0;

uint8_t volatile capture_state = CAPTURE_START;

int read_reg(unsigned int reg);

//Broadcast the uptime in seconds every second over connected websockets
static void  websockTimerCb(void *arg) {
    go_flag = 1;
}


// websocket hack

#define FLAG_FIN (1 << 7)
#define OPCODE_CONTINUE 0x0
#define OPCODE_TEXT 0x1
#define OPCODE_BINARY 0x2
#define OPCODE_CLOSE 0x8
#define OPCODE_PING 0x9
#define OPCODE_PONG 0xA
typedef struct {
	char head[HTTPD_MAX_HEAD_LEN];
	int headPos;
	char *sendBuff;
	int sendBuffLen;
	char *chunkHdr;
	void *sendBacklog;
	int sendBacklogSize;
	int flags;
} httpdpriv_hack_t;
int ICACHE_FLASH_ATTR genHeader(int opcode, int len, char* buf){
	int i=0;
	buf[i++]=opcode;
	if (len>65535) {
		buf[i++]=127;
		buf[i++]=0; buf[i++]=0; buf[i++]=0; buf[i++]=0; 
		buf[i++]=len>>24;
		buf[i++]=len>>16;
		buf[i++]=len>>8;
		buf[i++]=len;
	} else if (len>125) {
		buf[i++]=126;
		buf[i++]=len>>8;
		buf[i++]=len;
	} else {
		buf[i++]=len;
	}
        return i;
}

int ICACHE_FLASH_ATTR genFlag(int flags){
	int fl=0;
	if (flags&WEBSOCK_FLAG_BIN) fl=OPCODE_BINARY; else fl=OPCODE_TEXT;
	if (!(flags&WEBSOCK_FLAG_CONT)) fl|=FLAG_FIN;
        return fl;
}

//end websocket hack

static void send_image() {
    if(capture_state == CAPTURE_DONE && lock == 0){
        lock = 1;
        if (local_ws != NULL){
            memcpy(tmp_data, web_buff + hdr, 18);
            obuff[hdr+3] = (hdr >> 24) & 0xFF;
            obuff[hdr+2] = (hdr >> 16) & 0xFF;
            obuff[hdr+1] = (hdr >> 8) & 0xFF;
            obuff[hdr] = hdr & 0xFF;

            char buf[14];
            int i = genHeader(genFlag(WEBSOCK_FLAG_BIN), PACKET_SIZE + 4, buf);
            memcpy(obuff+hdr-i, buf, i);
            httdSetTransferMode(local_ws->conn, HTTPD_TRANSFER_NONE);
            void* tmp = ((httpdpriv_hack_t *)(local_ws->conn->priv))->sendBuff;
            ((httpdpriv_hack_t *)(local_ws->conn->priv))->sendBuff = obuff-i+hdr;
            ((httpdpriv_hack_t *)(local_ws->conn->priv))->sendBuffLen = i + PACKET_SIZE + 4;
            httpdFlushSendBuffer(local_ws->conn);
            ((httpdpriv_hack_t *)(local_ws->conn->priv))->sendBuff = tmp;

            memcpy(web_buff + hdr, tmp_data, 18);
            //cgiWebsockBroadcast("/websocket/ws.cgi", (char*)&(obuff[hdr]), PACKET_SIZE + 4, WEBSOCK_FLAG_BIN);
        }
        // (jnlm): got to roll this thing backwards because otherwise the send might not have finished before we change the headers.
        part -= 1;
        if (part < 0){
            part = NUM_PARTS-1;
            frame_cnt ++;
            capture_state = CAPTURE_START;
        }
        hdr = (part * PACKET_SIZE);

        lock = 0;
    }
}
void myWebsocketClose(Websock *ws){
    if (local_ws == ws)
        local_ws = NULL;
}
//On reception of a message, send "You sent: " plus whatever the other side sent
void myWebsocketRecv(Websock *ws, char *data, int len, int flags) {
    int i;
    char mbuff[128];
    //os_sprintf(mbuff, "You sent: ");
    for (i=0; i<len; i++) mbuff[i+10]=data[i];
    mbuff[i+10]=0;
    cgiWebsocketSend(ws, mbuff, os_strlen(mbuff), WEBSOCK_FLAG_NONE);
}

//Websocket connected. Install reception handler and send welcome message.
void myWebsocketConnect(Websock *ws) {
    ws->recvCb=myWebsocketRecv;
    ws->closeCb=myWebsocketClose;
    local_ws = ws;
    cgiWebsocketSend(ws, "Hi, Websocket!", 14, WEBSOCK_FLAG_NONE);
    //testi2s_init();
}

//On reception of a message, echo it back verbatim
void myEchoWebsocketRecv(Websock *ws, char *data, int len, int flags) {
    //os_printf("EchoWs: echo, len=%d\n", len);
    cgiWebsocketSend(ws, data, len, flags);
}

//Echo websocket connected. Install reception handler.
void myEchoWebsocketConnect(Websock *ws) {
    //os_printf("EchoWs: connect\n");
    ws->recvCb=myEchoWebsocketRecv;
}


#ifdef ESPFS_POS
CgiUploadFlashDef uploadParams={
    .type=CGIFLASH_TYPE_ESPFS,
    .fw1Pos=ESPFS_POS,
    .fw2Pos=0,
    .fwSize=ESPFS_SIZE,
};
#define INCLUDE_FLASH_FNS
#endif
#ifdef OTA_FLASH_SIZE_K
CgiUploadFlashDef uploadParams={
    .type=CGIFLASH_TYPE_FW,
    .fw1Pos=0x1000,
    .fw2Pos=((OTA_FLASH_SIZE_K*1024)/2)+0x1000,
    .fwSize=((OTA_FLASH_SIZE_K*1024)/2)-0x1000,
    .tagName=OTA_TAGNAME
};
#define INCLUDE_FLASH_FNS
#endif

/*
   This is the main url->function dispatching data struct.
   In short, it's a struct with various URLs plus their handlers. The handlers can
   be 'standard' CGI functions you wrote, or 'special' CGIs requiring an argument.
   They can also be auth-functions. An asterisk will match any url starting with
   everything before the asterisks; "*" matches everything. The list will be
   handled top-down, so make sure to put more specific rules above the more
   general ones. Authorization things (like authBasic) act as a 'barrier' and
   should be placed above the URLs they protect.
 */
HttpdBuiltInUrl builtInUrls[]={
    {"*", authBasic, myPassFn},
    {"*", cgiRedirectApClientToHostname, "esp8266.nonet"},
    {"/", cgiRedirect, "/index.tpl"},
    {"/led.tpl", cgiEspFsTemplate, tplLed},
    {"/index.tpl", cgiEspFsTemplate, tplCounter},
    {"/led.cgi", cgiLed, NULL},
    {"/websocket/ws.cgi", cgiWebsocket, myWebsocketConnect},
    {"/websocket/echo.cgi", cgiWebsocket, myEchoWebsocketConnect},

    {"*", cgiEspFsHook, NULL}, //Catch-all cgi function for the filesystem
    {NULL, NULL, NULL}
};


#ifdef SHOW_HEAP_USE
static ETSTimer prHeapTimer;

static void ICACHE_FLASH_ATTR prHeapTimerCb(void *arg) {
    os_printf("Heap: %ld\n", (unsigned long)system_get_free_heap_size());
    os_printf("camera error %d : %d : %d\n",camErr,capture_state,frame_cnt);
    frame_cnt = 0;
    if(camErr == 1 ){
        os_printf("camera reset\n");
        testi2s_init();
    }
    camErr = 1;

    os_printf("I2C:%02x\n",read_reg(0x02));
}
#endif


    void ICACHE_FLASH_ATTR
user_set_softap_config(void)
{
    struct softap_config config;

    wifi_softap_get_config(&config); // Get config first.

    os_memset(config.ssid, 0, 32);
    os_memset(config.password, 0, 64);
    os_memcpy(config.ssid, "espLepton", 9);
    os_memcpy(config.password, "wififlir", 8);
    config.authmode = AUTH_WPA_WPA2_PSK;
    config.ssid_len = 0;// or its actual length
    config.beacon_interval = 100;
    config.max_connection = 4; // how many stations can connect to ESP8266 softAP at most.

    wifi_softap_set_config(&config);// Set ESP8266 softap config .

}

#define user_procTaskPrio        0
#define user_procTaskQueueLen    1
os_event_t    user_procTaskQueue[user_procTaskQueueLen];
static void loop(os_event_t *events){
    if (go_flag){
        send_image();
        go_flag = 0;
    }
    system_os_post(user_procTaskPrio, 0, 0 );
}



void lepton_command(unsigned int moduleID, unsigned int commandID, unsigned int command)
{
    i2c_beginTransmission(ADDRESS);

    // Command Register is a 16-bit register located at Register Address 0x0004
    i2c_write(0x00);
    i2c_write(0x04);

    if (moduleID == 0x08) //OEM module ID
    {
        i2c_write(moduleID | 0x40);
    }
    else
    {
        i2c_write(moduleID & 0x0f);
    }
    i2c_write( ((commandID << 2 ) & 0xfc) | (command & 0x3));
    i2c_stop();
}

void agc_enable()
{
    i2c_beginTransmission(ADDRESS); // transmit to device #4
    i2c_write(0x01);
    i2c_write(0x05);
    i2c_write(0x00);
    i2c_write(0x01);
    i2c_stop();
}

void set_reg(unsigned int reg)
{
    i2c_beginTransmission(ADDRESS); // transmit to device #4
    i2c_write(reg >> 8 & 0xff);
    i2c_write(reg & 0xff);            // sends one uint8_t
    i2c_stop();
}

//Status reg 15:8 Error Code  7:3 Reserved 2:Boot Status 1:Boot Mode 0:busy

int read_reg(unsigned int reg)
{
    int reading = 0;
    set_reg(reg);

    i2c_requestFrom(ADDRESS);

    reading = i2c_read();  // receive high uint8_t (overwrites previous reading)
    reading = reading << 8;    // shift high uint8_t to be high 8 bits

    reading |= i2c_read(); // receive low uint8_t as lower 8 bits
    os_printf("REG %d: %x\n",reg,reading);

    return reading;
}

void read_data()
{
    int i;
    int data;
    int payload_length;
    i = 0;
    while (read_reg(0x2) & 0x01)
    {
        if (++i > 10) break;
        os_printf("busy\n");
    }

    payload_length = read_reg(0x6);
    os_printf("LEN: %d\n",payload_length);

    i2c_requestFrom(ADDRESS);
    //set_reg(0x08);
    for (i = 0; i < (payload_length / 2); i++)
    {
        data = i2c_read() << 8;
        data |= i2c_read();
        os_printf("%x\n",data);
    }

}

//Main routine. Initialize stdout, the I/O, filesystem and the webserver and we're done.
void  ICACHE_FLASH_ATTR user_init(void) {
    //does this even work?
    ets_update_cpu_frequency(160);

    gpio_init();
    //uart 115200
    ioInit();


    char ssid[32] = USER_MAIN_SSID;
    char password[64] = USER_MAIN_WPA2PW;
    struct station_config stationConf;

    //Set station mode
    wifi_set_opmode( 0x1 );

    //Set ap settings
    os_memcpy(&stationConf.ssid, ssid, 32);
    os_memcpy(&stationConf.password, password, 64    );
    wifi_station_set_config(&stationConf);

    /*    
          wifi_set_opmode(SOFTAP_MODE);

          user_set_softap_config();
     */     

    stdoutInit();
    captdnsInit();


    // 0x40200000 is the base address for spi flash memory mapping, ESPFS_POS is the position
    // where image is written in flash that is defined in Makefile.
#ifdef ESPFS_POS
    espFsInit((void*)(0x40200000 + ESPFS_POS));
#else
    espFsInit((void*)(webpages_espfs_start));
#endif
    httpdInit(builtInUrls, 80);
#ifdef SHOW_HEAP_USE
    os_timer_disarm(&prHeapTimer);
    os_timer_setfn(&prHeapTimer, prHeapTimerCb, NULL);
    os_timer_arm(&prHeapTimer, 3000, 1);
#endif

    os_timer_disarm(&websockTimer);
    os_timer_setfn(&websockTimer, websockTimerCb, NULL);
    os_timer_arm(&websockTimer, 15, 1);
    os_printf("\nReady\n");

    i2c_init();

    os_printf("SYS Flir Serial Number\r\n");
    lepton_command(SYS, 0x2 , GET);
    //read_data();

    testi2s_init();
    system_os_task(loop, user_procTaskPrio,user_procTaskQueue, user_procTaskQueueLen);
    system_os_post(user_procTaskPrio, 0, 0 );

}

void user_rf_pre_init() {
    //Not needed, but some SDK versions want this defined.
}
/*
//Sdk 2.0.0 needs extra sector to store rf cal stuff. Place that at the end of the flash.
uint32 ICACHE_FLASH_ATTR
user_rf_cal_sector_set(void)
{
enum flash_size_map size_map = system_get_flash_size_map();
uint32 rf_cal_sec = 0;

switch (size_map) {
case FLASH_SIZE_4M_MAP_256_256:
rf_cal_sec = 128 - 8;
break;

case FLASH_SIZE_8M_MAP_512_512:
rf_cal_sec = 256 - 5;
break;

case FLASH_SIZE_16M_MAP_512_512:
case FLASH_SIZE_16M_MAP_1024_1024:
rf_cal_sec = 512 - 5;
break;

case FLASH_SIZE_32M_MAP_512_512:
case FLASH_SIZE_32M_MAP_1024_1024:
rf_cal_sec = 1024 - 5;
break;

default:
rf_cal_sec = 0;
break;
}

return rf_cal_sec;
}*/
