#ifndef H_SERIAL_WRAPPER
#define H_SERIAL_WRAPPER

#ifdef __cplusplus
extern "C" {
#endif

int initSerial();
void closeSerial();
void ledON();
void ledOFF();

#ifdef __cplusplus
}
#endif

#endif /* H_SERIAL_WRAPPER */

