#ifndef CGI_H
#define CGI_H

#include "httpd.h"

int ICACHE_FLASH_ATTR tplIndex(HttpdConnData *connData, char *token, void **arg);

#endif

