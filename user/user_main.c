#include <esp8266.h>
#include "cgiflash.h"
#include "httpd.h"
#include "httpdespfs.h"
#include "auth.h"
#include "cgi.h"

int ICACHE_FLASH_ATTR password(HttpdConnData *connData, int no, char *user, int userLen, char *pass, int passLen) {
    if (no==0) {
        os_strcpy(user, "admin");
        os_strcpy(pass, "admin");
        return 1;
    }
    return 0;
}

HttpdBuiltInUrl builtInUrls[]={
    {"*", authBasic, password},
    {"/", cgiRedirect, "/index.tpl"},
    {"/index.tpl", cgiEspFsTemplate, tplIndex},
    {"*", cgiEspFsHook, NULL}, //Catch-all cgi function for the filesystem
    {NULL, NULL, NULL}
};

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

void  ICACHE_FLASH_ATTR user_init(void) {
    ets_update_cpu_frequency(160);

    char ssid[32] = USER_MAIN_SSID;
    char password[64] = USER_MAIN_WPA2PW;
    struct station_config stationConf;

    //Set station mode
    wifi_set_opmode( 0x1 );

    //Set ap settings
    os_memcpy(&stationConf.ssid, ssid, 32);
    os_memcpy(&stationConf.password, password, 64);
    wifi_station_set_config(&stationConf);

    captdnsInit();


    // 0x40200000 is the base address for spi flash memory mapping, ESPFS_POS is the position
    // where image is written in flash that is defined in Makefile.
#ifdef ESPFS_POS
    espFsInit((void*)(0x40200000 + ESPFS_POS));
#else
    espFsInit((void*)(webpages_espfs_start));
#endif
    httpdInit(builtInUrls, 80);

    os_printf("\nReady\n");
}
