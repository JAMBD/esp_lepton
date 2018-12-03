#include <esp8266.h>
#include "cgi.h"

//Template code for the counter on the index page.
int ICACHE_FLASH_ATTR tplIndex(HttpdConnData *connData, char *token, void **arg) {
	char buff[128];
	if (token==NULL) return HTTPD_CGI_DONE;
	httpdSend(connData, buff, -1);
	return HTTPD_CGI_DONE;
}
