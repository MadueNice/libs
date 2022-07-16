#ifndef __CANOPENSIMPLE_H__
#define __CANOPENSIMPLE_H__

#include "main.h"

#include "CANopen.h"
#include "OD.h"
#include "print.h"


#define log_printf(macropar_message, ...) \
        printf(macropar_message, ##__VA_ARGS__)


/* default values for CO_CANopenInit() */
#define NMT_CONTROL \
            CO_NMT_STARTUP_TO_OPERATIONAL \
          | CO_NMT_ERR_ON_ERR_REG \
          | CO_ERR_REG_GENERIC_ERR \
          | CO_ERR_REG_COMMUNICATION
#define FIRST_HB_TIME 500
#define SDO_SRV_TIMEOUT_TIME 1000
#define SDO_CLI_TIMEOUT_TIME 500
#define SDO_CLI_BLOCK false
#define OD_STATUS_BITS NULL


typedef struct
{
	uint32_t time_old, time_current;
	CO_ReturnError_t err;
	CO_NMT_reset_cmd_t reset;
	void *CANptr;
	uint8_t pendingNodeId, activeNodeId;
	uint16_t pendingBitRate;
} CO_State_t;

typedef struct
{
	uint8_t NodeIdReq;
	uint16_t index;
	uint8_t subIndex;
	bool_t flagResponse;
	uint8_t msgBuf[32];
	uint8_t msgBufSize;
	CO_SDO_abortCode_t SDOabortCode;
} CO_SDOMsg_t;

uint8_t CO_Init(CO_t **CO, CO_State_t *state, uint8_t NodeId, uint16_t BitRate, CAN_HandleTypeDef *hcan);

/* In non-blocking loop*/
uint8_t CO_ProccesApp(CO_t *CO, CO_State_t *CO_State);
/* In non-blocking loop, no more than 1ms*/
uint8_t CO_ProccesComm(CO_t *CO, CO_State_t *CO_State);

/* Interrupt every 1ms*/
void CO_Task1ms(CO_t *CO);

CO_SDO_return_t CO_SDO_read(CO_t *CO,CO_SDOMsg_t *msg);
CO_SDO_return_t CO_SDO_write(CO_t *CO,CO_SDOMsg_t *msg);

ODR_t CO_OD_read(OD_t* OD, uint16_t index, uint8_t subIndex, void *buf, uint8_t sizeofbuf, bool_t odOrig);
ODR_t CO_OD_write(OD_t* OD, uint16_t index, uint8_t subIndex, void *buf, uint8_t sizeofbuf, bool_t odOrig);

#endif // #ifndef __CO_H__
