#ifndef __APP_DEFS_H__
#define __APP_DEFS_H__

#ifdef __cplusplus
extern "C" {
#endif

/*
	Jace. 191107. For CTN rev0.1.
*/
#define CTN_REV01

/*
	Jace. 191107. Add I2C slave function for CTN.
*/
#define CTN_REV01_I2C

/*
	Jace. 191115. Add binary tone player function.
*/
#define CTN_TONE_PLAY

/*
	Jace. 191107. amazon alexa skill kit json parser.
*/
#define CTN_ASK_PARSER

/*
	Jace. 191107. communicate external device using UART.
*/
#define CTN_REV01_UART_COMMx

/*
	Jace. 200214. Add blynk's App. function.
*/
#define BLYNK_APPS

/*
	Jace. 200225. Add blynk's i2c slave function.
*/
#define BLYNK_I2C

/*
	Jace. 200309. Add Factory reset after 20th reconnect timeout.
*/
#define FACTORY_RESET


#if defined(BLYNK_I2C)
void blynk_notify_i2c_i94124(uint8_t cmd);
#endif

#ifdef __cplusplus
}
#endif

#endif
