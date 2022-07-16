/*
 * Device and application specific definitions for CANopenNode.
 *
 * @file        CO_driver_target.h
 * @author      Janez Paternoster
 * @copyright   2021 Janez Paternoster
 *
 * This file is part of CANopenNode, an opensource CANopen Stack.
 * Project home page is <https://github.com/CANopenNode/CANopenNode>.
 * For more information on CANopen see <http://www.can-cia.org/>.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */


#ifndef CO_DRIVER_TARGET_H
#define CO_DRIVER_TARGET_H

/* This file contains device and application specific definitions.
 * It is included from CO_driver.h, which contains documentation
 * for common definitions below. */

#include <stddef.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>



#ifdef CO_DRIVER_CUSTOM
#include "CO_driver_custom.h"
#endif

#ifdef __cplusplus
extern "C" {
#endif

/* Stack configuration override default values.
 * For more information see file CO_config.h. */

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

/* Basic definitions. If big endian, CO_SWAP_xx macros must swap bytes. */
#define CO_LITTLE_ENDIAN
#define CO_SWAP_16(x) (uint16_t)(((uint16_t)(x) >> 8) | ((uint16_t)(x) << 8))
#define CO_SWAP_32(x) CO_SWAP_16((x) >> 16) | CO_SWAP_16((x)) << 16
#define CO_SWAP_64(x) x
/* NULL is defined in stddef.h */
/* true and false are defined in stdbool.h */
/* int8_t to uint64_t are defined in stdint.h */
typedef uint_fast8_t            bool_t;
typedef float                   float32_t;
typedef double                  float64_t;


/* Access to received CAN message */
#define CO_CANrxMsg_readIdent(msg) ((((uint16_t)(((CO_CANrxMsg_t *)(msg))->ident))>>2)&0x7FF)
#define CO_CANrxMsg_readDLC(msg)   ((uint8_t)(((CO_CANrxMsg_t *)(msg))->DLC))
#define CO_CANrxMsg_readData(msg)  ((uint8_t *)(((CO_CANrxMsg_t *)(msg))->data))

/* Received message object */
typedef struct {
    uint16_t ident;
    uint16_t mask;
    void *object;
    void (*CANrx_callback)(void *object, void *message);
} CO_CANrx_t;

typedef struct {
    uint32_t ident;
    uint8_t DLC;
    uint8_t data[8];
} CO_CANrxMsg_t;

/* Transmit message object */
typedef struct {
    uint32_t ident;
    uint8_t DLC;
    uint8_t data[8];
    volatile bool_t bufferFull;
    volatile bool_t syncFlag;
} CO_CANtx_t;

/* CAN module object */
typedef struct {
    void *CANptr;
    CO_CANrx_t *rxArray;
    uint16_t rxSize;
    CO_CANtx_t *txArray;
    uint16_t txSize;
    uint16_t CANerrorStatus;
    volatile bool_t CANnormal;
    volatile bool_t useCANrxFilters;
    volatile bool_t bufferInhibitFlag;
    volatile bool_t firstCANtxMessage;
    volatile uint16_t CANtxCount;
    uint32_t errOld;
} CO_CANmodule_t;


/* Data storage object for one entry */
typedef struct {
    void *addr;
    size_t len;
    uint8_t subIndexOD;
    uint8_t attr;
    /* Additional variables (target specific) */
    void *addrNV;
} CO_storage_entry_t;


/* (un)lock critical section in CO_CANsend() */
#define CO_LOCK_CAN_SEND(CAN_MODULE)
#define CO_UNLOCK_CAN_SEND(CAN_MODULE)

/* (un)lock critical section in CO_errorReport() or CO_errorReset() */
#define CO_LOCK_EMCY(CAN_MODULE)
#define CO_UNLOCK_EMCY(CAN_MODULE)

/* (un)lock critical section when accessing Object Dictionary */
#define CO_LOCK_OD(CAN_MODULE)
#define CO_UNLOCK_OD(CAN_MODULE)

/* Synchronization between CAN receive and message processing threads. */
#define CO_MemoryBarrier()
#define CO_FLAG_READ(rxNew) ((rxNew) != NULL)

#define CO_FLAG_SET(rxNew) {CO_MemoryBarrier(); rxNew = (void*)1L;}
#define CO_FLAG_CLEAR(rxNew) {CO_MemoryBarrier(); rxNew = NULL;}



//typedef struct
//{
//	uint32_t time_old, time_current;
//	CO_ReturnError_t err;
//	CO_NMT_reset_cmd_t reset;
//	void *CANptr;
//	uint8_t pendingNodeId, activeNodeId;
//	uint16_t pendingBitRate;
//} CO_State_t;
//
//typedef struct
//{
//	uint8_t NodeIdReq;
//	uint16_t index;
//	uint8_t subIndex;
//	bool_t flagResponse;
//	uint8_t msgBuf[32];
//	uint8_t msgBufSize;
//	CO_SDO_abortCode_t SDOabortCode;
//} CO_SDOMsg_t;
//
//uint8_t CO_Init(CO_t **CO, CO_State_t *state, uint8_t NodeId, uint16_t BitRate, CAN_HandleTypeDef *hcan);
//uint8_t CO_ProccesApp(CO_t *CO, CO_State_t *CO_State);
//uint8_t CO_ProccesComm(CO_t *CO, CO_State_t *CO_State);
//
//CO_SDO_return_t CO_SDO_read(CO_t *CO,CO_SDOMsg_t *msg);
//CO_SDO_return_t CO_SDO_write(CO_t *CO,CO_SDOMsg_t *msg);
//
//ODR_t CO_OD_read(OD_t* OD, uint16_t index, uint8_t subIndex, void *buf, uint8_t sizeofbuf, bool_t odOrig);
//ODR_t CO_OD_write(OD_t* OD, uint16_t index, uint8_t subIndex, void *buf, uint8_t sizeofbuf, bool_t odOrig);



#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* CO_DRIVER_TARGET_H */
