#ifndef __USER_H_
#define _USER_H_
#define CAPTURE_START 1
#define CAPTURE_PROGRESS 2
#define CAPTURE_DONE 0

extern uint8_t volatile  capture_state;
extern uint8_t* buff;
extern uint8_t camErr;
extern uint8_t lock;
#endif
