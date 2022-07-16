/*
 * CAN module object for STM32F3xx CAN peripheral IP.
 *
 * This file is a template for other microcontrollers.
 *
 * @file        CO_driver.c
 * @ingroup     CO_driver
 * @author      Hamed Jafarzadeh 	2022
 * 				Tilen Marjerle		2021
 * 				Janez Paternoster	2020
 * @copyright   2004 - 2022 Janez Paternoster
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
 *
 * Implementation Author:               Tilen Majerle <tilen@majerle.eu>
 */
#include "301/CO_driver.h"
#include "stm32f4xx_hal.h"
#include "main.h"
#include "can.h"
#include "print.h"

#define CO_CAN_PRINT 0x1u

/* Local CAN module object */
static CO_CANmodule_t *CANModule_local = NULL; /* Local instance of global CAN module */

/* CAN masks for identifiers */
#define CANID_MASK                              0x07FF  /*!< CAN standard ID mask */
#define FLAG_RTR                                0x8000  /*!< RTR flag, part of identifier */

#if defined(USE_OS)
/* Mutex for atomic access */
static osMutexId_t co_mutex;

/* Semaphore for main app thread synchronization */
osSemaphoreId_t co_drv_app_thread_sync_semaphore;

/* Semaphore for periodic thread synchronization */
osSemaphoreId_t co_drv_periodic_thread_sync_semaphore;
#endif /* defined(USE_OS) */

/* CAN handle object */
extern CAN_HandleTypeDef hcan;

/******************************************************************************/
void CO_CANsetConfigurationMode(void *CANptr) {
	/* Put CAN module in configuration mode */
	if (CANptr != NULL) {
		HAL_CAN_Stop(&hcan1);
	}
}

/******************************************************************************/
void CO_CANsetNormalMode(CO_CANmodule_t *CANmodule) {
	/* Put CAN module in normal mode */
	if (CANmodule->CANptr != NULL
			&& HAL_CAN_Start(CANmodule->CANptr) == HAL_OK) {
		CANmodule->CANnormal = true;
	}
}

/******************************************************************************/
CO_ReturnError_t CO_CANmodule_init(CO_CANmodule_t *CANmodule,
		void *HALCanObject, CO_CANrx_t rxArray[], uint16_t rxSize,
		CO_CANtx_t txArray[], uint16_t txSize, uint16_t CANbitRate) {

	/* verify arguments */
	if (CANmodule == NULL || rxArray == NULL || txArray == NULL) {
		return CO_ERROR_ILLEGAL_ARGUMENT;
	}


	/* Hold CANModule variable */
	CANModule_local = CANmodule;

	CANmodule->CANptr = (CAN_HandleTypeDef*) HALCanObject;

	/* Configure object variables */
	CANmodule->rxArray = rxArray;
	CANmodule->rxSize = rxSize;
	CANmodule->txArray = txArray;
	CANmodule->txSize = txSize;
	CANmodule->CANerrorStatus = 0;
	CANmodule->CANnormal = false;
	CANmodule->useCANrxFilters = false; /* Do not use HW filters */
	CANmodule->bufferInhibitFlag = false;
	CANmodule->firstCANtxMessage = true;
	CANmodule->CANtxCount = 0U;
	CANmodule->errOld = 0U;

	/* Reset all variables */
	for (uint16_t i = 0U; i < rxSize; i++) {
		rxArray[i].ident = 0U;
		rxArray[i].mask = 0xFFFFU;
		rxArray[i].object = NULL;
		rxArray[i].CANrx_callback = NULL;
	}
	for (uint16_t i = 0U; i < txSize; i++) {
		txArray[i].bufferFull = false;
	}

	/***************************************/
	/* STM32F3 CAN related configuration */
	/***************************************/

	CO_CANmodule_disable(CANmodule);

//	HAL_CAN_MspDeInit(CANmodule->CANptr); // DeInit CAN MSP

	/* Set instance at the beginning */
	MX_CAN1_Init(); // Setup CAN based on CubeMX Configuration

	/*
	 * Configure global filter that is used as last check if message did not pass any of other filters:
	 *
	 * We do not rely on hardware filters in this example
	 * and are performing software filters instead
	 *
	 * Accept non-matching standard ID messages
	 * Reject non-matching extended ID messages
	 */
	/* Configure mask 0 so, that all messages with standard identifier are accepted */
	CAN_FilterTypeDef FilterConfig;
	FilterConfig.FilterBank = 0;
	FilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
	FilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;
	FilterConfig.FilterIdHigh = 0x0;
	FilterConfig.FilterIdLow = 0x0;
	FilterConfig.FilterMaskIdHigh = 0x0;
	FilterConfig.FilterMaskIdLow = 0x0;
	FilterConfig.FilterFIFOAssignment = CAN_RX_FIFO0;
	FilterConfig.FilterActivation = ENABLE;
	FilterConfig.SlaveStartFilterBank = 14;

	if (HAL_CAN_ConfigFilter(CANmodule->CANptr, &FilterConfig)
					!= HAL_OK) {
		return CO_ERROR_ILLEGAL_ARGUMENT;
	}

	/* Enable notifications */
	/* Activate the CAN notification interrupts */
	if (HAL_CAN_ActivateNotification(CANmodule->CANptr,
	CAN_IT_RX_FIFO0_MSG_PENDING |
	CAN_IT_RX_FIFO1_MSG_PENDING |
	CAN_IT_TX_MAILBOX_EMPTY) != HAL_OK) {
		return CO_ERROR_ILLEGAL_ARGUMENT;
	}
	return CO_ERROR_NO;
}

#if defined(USE_OS)

/**
 * \brief           Create all OS objects for CANopen
 * \return          `1` on success, `0` otherwise
 */
uint8_t
co_drv_create_os_objects(void) {
    /* Create new mutex for OS context */
    if (co_mutex == NULL) {
        const osMutexAttr_t attr = {
            .attr_bits = osMutexRecursive,
            .name = "co"
        };
        co_mutex = osMutexNew(&attr);
    }

    /* Semaphore for main app thread synchronization */
    if (co_drv_app_thread_sync_semaphore == NULL) {
        const osSemaphoreAttr_t attr = {
                .name = "co_app_thread_sync"
        };
        co_drv_app_thread_sync_semaphore = osSemaphoreNew(1, 1, &attr);
    }

    /* Semaphore for periodic thread synchronization */
    if (co_drv_periodic_thread_sync_semaphore == NULL) {
        const osSemaphoreAttr_t attr = {
                .name = "co_periodic_thread_sync"
        };
        co_drv_periodic_thread_sync_semaphore = osSemaphoreNew(1, 1, &attr);
    }

    return 1;
}

/**
 * \brief           Lock mutex or wait to be available
 * \return          `1` on success, `0` otherwise
 */
uint8_t
co_drv_mutex_lock(void) {
    return osMutexAcquire(co_mutex, osWaitForever) == osOK;
}

/**
 * \brief           Release previously locked mutex
 * \return          `1` on success, `0` otherwise
 */
uint8_t
co_drv_mutex_unlock(void) {
    return osMutexRelease(co_mutex) == osOK;
}

#endif /* defined(USE_OS) */

/******************************************************************************/
void CO_CANmodule_disable(CO_CANmodule_t *CANmodule) {
	if (CANmodule != NULL && CANmodule->CANptr != NULL) {
		HAL_CAN_Stop(CANmodule->CANptr);
	}
}

/******************************************************************************/
CO_ReturnError_t CO_CANrxBufferInit(CO_CANmodule_t *CANmodule, uint16_t index,
		uint16_t ident, uint16_t mask, bool_t rtr, void *object,
		void (*CANrx_callback)(void *object, void *message)) {
	CO_ReturnError_t ret = CO_ERROR_NO;

	if (CANmodule != NULL && object != NULL && CANrx_callback != NULL
			&& index < CANmodule->rxSize) {
		CO_CANrx_t *buffer = &CANmodule->rxArray[index];

		/* Configure object variables */
		buffer->object = object;
		buffer->CANrx_callback = CANrx_callback;

		/*
		 * Configure global identifier, including RTR bit
		 *
		 * This is later used for RX operation match case
		 */
		buffer->ident = (ident & CANID_MASK) | (rtr ? FLAG_RTR : 0x00);
		buffer->mask = (mask & CANID_MASK) | FLAG_RTR;

		/* Set CAN hardware module filter and mask. */
		if (CANmodule->useCANrxFilters) {
			__NOP();
		}
	} else {
		ret = CO_ERROR_ILLEGAL_ARGUMENT;
	}

	return ret;
}

/******************************************************************************/
CO_CANtx_t*
CO_CANtxBufferInit(CO_CANmodule_t *CANmodule, uint16_t index, uint16_t ident,
		bool_t rtr, uint8_t noOfBytes, bool_t syncFlag) {
	CO_CANtx_t *buffer = NULL;

	if (CANmodule != NULL && index < CANmodule->txSize) {
		buffer = &CANmodule->txArray[index];

		/* CAN identifier, DLC and rtr, bit aligned with CAN module transmit buffer */
		buffer->ident = ((uint32_t) ident & CANID_MASK)
				| ((uint32_t) (rtr ? FLAG_RTR : 0x00));
		buffer->DLC = noOfBytes;
		buffer->bufferFull = false;
		buffer->syncFlag = syncFlag;
	}
	return buffer;
}

/**
 * \brief           Send CAN message to network
 * This function must be called with atomic access.
 *
 * \param[in]       CANmodule: CAN module instance
 * \param[in]       buffer: Pointer to buffer to transmit
 */
static uint8_t prv_send_can_message(CO_CANmodule_t *CANmodule,
		CO_CANtx_t *buffer) {
	static CAN_TxHeaderTypeDef tx_hdr;
	uint8_t success = 0;

	/* Check if TX FIFO is ready to accept more messages */
	if (HAL_CAN_GetTxMailboxesFreeLevel(CANModule_local->CANptr) > 0) {
		/*
		 * RTR flag is part of identifier value
		 * hence it needs to be properly decoded
		 */
		tx_hdr.ExtId = 0u;
		tx_hdr.IDE = CAN_ID_STD;
		tx_hdr.DLC = buffer->DLC;
		tx_hdr.StdId = buffer->ident & CANID_MASK;
		tx_hdr.RTR =
				(buffer->ident & FLAG_RTR) ? CAN_RTR_REMOTE : CAN_RTR_DATA;

		uint32_t TxMailboxNum; // Transmission MailBox number

		/* Now add message to FIFO. Should not fail */
		success = HAL_CAN_AddTxMessage(CANmodule->CANptr, &tx_hdr, buffer->data,
				&TxMailboxNum) == HAL_OK;
	}
	return success;
}

/******************************************************************************/
CO_ReturnError_t CO_CANsend(CO_CANmodule_t *CANmodule, CO_CANtx_t *buffer) {
	CO_ReturnError_t err = CO_ERROR_NO;

	/* Verify overflow */
	if (buffer->bufferFull) {
		if (!CANmodule->firstCANtxMessage) {
			/* don't set error, if bootup message is still on buffers */
			CANmodule->CANerrorStatus |= CO_CAN_ERRTX_OVERFLOW;
		}
		err = CO_ERROR_TX_OVERFLOW;
	}

	/*
	 * Send message to CAN network
	 *
	 * Lock interrupts for atomic operation
	 */
	CO_LOCK_CAN_SEND(CANmodule);
	if (prv_send_can_message(CANmodule, buffer)) {
		CANmodule->bufferInhibitFlag = buffer->syncFlag;
	} else {
		buffer->bufferFull = true;
		CANmodule->CANtxCount++;
	}
	CO_UNLOCK_CAN_SEND(CANmodule);

	return err;
}

/******************************************************************************/
void CO_CANclearPendingSyncPDOs(CO_CANmodule_t *CANmodule) {
	uint32_t tpdoDeleted = 0U;

	CO_LOCK_CAN_SEND(CANmodule);
	/* Abort message from CAN module, if there is synchronous TPDO.
	 * Take special care with this functionality. */
	if (/*messageIsOnCanBuffer && */CANmodule->bufferInhibitFlag) {
		/* clear TXREQ */
		CANmodule->bufferInhibitFlag = false;
		tpdoDeleted = 1U;
	}
	/* delete also pending synchronous TPDOs in TX buffers */
	if (CANmodule->CANtxCount > 0) {
		for (uint16_t i = CANmodule->txSize; i > 0U; --i) {
			if (CANmodule->txArray[i].bufferFull) {
				if (CANmodule->txArray[i].syncFlag) {
					CANmodule->txArray[i].bufferFull = false;
					CANmodule->CANtxCount--;
					tpdoDeleted = 2U;
				}
			}
		}
	}
	CO_UNLOCK_CAN_SEND(CANmodule);
	if (tpdoDeleted) {
		CANmodule->CANerrorStatus |= CO_CAN_ERRTX_PDO_LATE;
	}
}

/******************************************************************************/
/* Get error counters from the module. If necessary, function may use
 * different way to determine errors. */

void CO_CANmodule_process(CO_CANmodule_t *CANmodule) {
	uint32_t err = 0xFFFFFFFF;

	/* TODO: There might be some imporovement needed here */

	if (CANmodule->errOld != err) {
		uint16_t status = CANmodule->CANerrorStatus;

		uint32_t CO_HAL_CAN_ERROR = ((CAN_HandleTypeDef*) CANmodule->CANptr)->ErrorCode;

		CANmodule->errOld = CO_HAL_CAN_ERROR;

		if (CO_HAL_CAN_ERROR & HAL_CAN_ERROR_BOF) {
			/* bus off */
			status |= CO_CAN_ERRTX_BUS_OFF;
		} else {
			/* recalculate CANerrorStatus, first clear some flags */
			status &= 0xFFFF
					^ (CO_CAN_ERRTX_BUS_OFF | CO_CAN_ERRRX_WARNING
							| CO_CAN_ERRRX_PASSIVE | CO_CAN_ERRTX_WARNING
							| CO_CAN_ERRTX_PASSIVE);

			if (CO_HAL_CAN_ERROR & HAL_CAN_ERROR_EWG) { /* bus warning */
				status |= CO_CAN_ERRTX_WARNING | CO_CAN_ERRRX_WARNING;
			}

			//TODO : [HJ] Might need to check if it is the first message or not (Bootup message)
			if (CO_HAL_CAN_ERROR & HAL_CAN_ERROR_RX_FOV0) {
				status |= CO_CAN_ERRTX_PASSIVE | CO_CAN_ERRRX_PASSIVE;
			}

			if ((CO_HAL_CAN_ERROR & HAL_CAN_ERROR_RX_FOV0)
					|| (CO_HAL_CAN_ERROR & HAL_CAN_ERROR_RX_FOV1)) {
				status |= CO_CAN_ERRRX_OVERFLOW;
			}
		}
		CANmodule->CANerrorStatus = status;
	}
}




/**
 * \brief           Read message from RX FIFO
 * \param           hcan: pointer to an CAN_HandleTypeDef structure that contains
 *                      the configuration information for the specified CAN.
 * \param[in]       fifo: Fifo number to use for read
 * \param[in]       fifo_isrs: List of interrupts for respected FIFO
 */
static void prv_read_can_received_msg(CAN_HandleTypeDef *hcan,
		uint32_t fifo, uint32_t fifo_isrs) {
	static CAN_RxHeaderTypeDef rx_hdr;
	CO_CANrxMsg_t rcvMsg;
	CO_CANrx_t *buffer = NULL; /* receive message buffer from CO_CANmodule_t object. */
	uint16_t index; /* index of received message */
	uint32_t rcvMsgIdent; /* identifier of the received message */
	uint8_t messageFound = 0;

	/* Read received message from FIFO */
	if (HAL_CAN_GetRxMessage(hcan, fifo, &rx_hdr, rcvMsg.data) != HAL_OK) {
		return;
	}


#if CO_CAN_PRINT
	printf("%.4X%10c", (unsigned int)rx_hdr.StdId, ' ');
	for(uint8_t i=0; i<8; i++)
	{
		if(i < rx_hdr.DLC) printf("%.2X ", rcvMsg.data[i]);
		else printf("-- ");
	}
	printf("\n");
#endif

//	if(hcan == &hcan2)
//	{
//		return;
//	}

	/* Setup identifier (with RTR) and length */
	rcvMsg.ident = rx_hdr.StdId
			| (rx_hdr.RTR == CAN_RTR_REMOTE ? FLAG_RTR : 0x00);
	rcvMsg.DLC = rx_hdr.DLC;

	rcvMsgIdent = rcvMsg.ident;

	/*
	 * Hardware filters are not used for the moment
	 * \todo: Implement hardware filters...
	 */
	if (CANModule_local->useCANrxFilters) {
		__BKPT(0);
	} else {
		/*
		 * We are not using hardware filters, hence it is necessary
		 * to manually match received message ID with all buffers
		 */
		buffer = CANModule_local->rxArray;
		for (index = CANModule_local->rxSize; index > 0U; --index, ++buffer) {
			if (((rcvMsgIdent ^ buffer->ident) & buffer->mask) == 0U) {
				messageFound = 1;
				break;
			}
		}
	}
	/* Call specific function, which will process the message */
	if (messageFound && buffer != NULL && buffer->CANrx_callback != NULL) {
		buffer->CANrx_callback(buffer->object, (void*) &rcvMsg);
	}
}

/**
 * \brief           Rx FIFO 0 callback.
 * \param[in]       hcan: pointer to an CAN_HandleTypeDef structure that contains
 *                      the configuration information for the specified CAN.
 */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
	prv_read_can_received_msg(hcan, CAN_RX_FIFO0,0);
}


/**
 * \brief           Rx FIFO 1 callback.
 * \param[in]       hcan: pointer to an CAN_HandleTypeDef structure that contains
 *                      the configuration information for the specified CAN.
 */
void HAL_CAN_RxFifo1MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
	prv_read_can_received_msg(hcan, CAN_RX_FIFO1,0);
}


/**
 * \brief           TX buffer has been well transmitted callback
 * \param[in]       hcan: pointer to an CAN_HandleTypeDef structure that contains
 *                      the configuration information for the specified CAN.
 * \param[in]       MailboxNumber: the mailbox number that has been transmitted
 */
void CO_CANinterrupt_TX(CO_CANmodule_t *CANmodule, uint32_t MailboxNumber) {

	CANModule_local->firstCANtxMessage = false; /* First CAN message (bootup) was sent successfully */
	CANModule_local->bufferInhibitFlag = false; /* Clear flag from previous message */
	if (CANModule_local->CANtxCount > 0U) { /* Are there any new messages waiting to be send */
		CO_CANtx_t *buffer = &CANModule_local->txArray[0]; /* Start with first buffer handle */
		uint16_t i;

		/*
		 * Try to send more buffers, process all empty ones
		 *
		 * This function is always called from interrupt,
		 * however to make sure no preemption can happen, interrupts are anyway locked
		 * (unless you can guarantee no higher priority interrupt will try to access to CAN instance and send data,
		 *  then no need to lock interrupts..)
		 */
		CO_LOCK_CAN_SEND(CANModule_local);
		for (i = CANModule_local->txSize; i > 0U; --i, ++buffer) {
			/* Try to send message */
			if (buffer->bufferFull) {
				if (prv_send_can_message(CANModule_local, buffer)) {
					buffer->bufferFull = false;
					CANModule_local->CANtxCount--;
					CANModule_local->bufferInhibitFlag = buffer->syncFlag;
				}
			}
		}
		/* Clear counter if no more messages */
		if (i == 0U) {
			CANModule_local->CANtxCount = 0U;
		}
		CO_UNLOCK_CAN_SEND(CANModule_local);
	}

}



void HAL_CAN_TxMailbox0CompleteCallback(CAN_HandleTypeDef *hcan){
	CO_CANinterrupt_TX(CANModule_local,CAN_TX_MAILBOX0);
}
void HAL_CAN_TxMailbox1CompleteCallback(CAN_HandleTypeDef *hcan){
	CO_CANinterrupt_TX(CANModule_local,CAN_TX_MAILBOX0);
}
void HAL_CAN_TxMailbox2CompleteCallback(CAN_HandleTypeDef *hcan){
	CO_CANinterrupt_TX(CANModule_local,CAN_TX_MAILBOX0);
}

//
//uint8_t CO_Init(CO_t **CO, CO_State_t *state, uint8_t NodeId, uint16_t BitRate, CAN_HandleTypeDef *hcan)
//{
//	uint32_t heapMemoryUsed;
//	state->reset = -1;
//	state->CANptr = hcan; /* CAN module address */
//	state->pendingNodeId = NodeId; /* read from dip switches or nonvolatile memory, configurable by LSS slave */
//	state->activeNodeId = NodeId; /* Copied from CO_pendingNodeId in the communication reset section */
//	state->pendingBitRate = BitRate;  /* read from dip switches or nonvolatile memory, configurable by LSS slave */
//
//	#if (CO_CONFIG_STORAGE) & CO_CONFIG_STORAGE_ENABLE
//	  CO_storage_t storage;
//	  CO_storage_entry_t storageEntries[] = {
//	      {
//	          .addr = &OD_PERSIST_COMM,
//	          .len = sizeof(OD_PERSIST_COMM),
//	          .subIndexOD = 2,
//	          .attr = CO_storage_cmd | CO_storage_restore,
//	          .addrNV = NULL
//	      }
//	  };
//	  uint8_t storageEntriesCount = sizeof(storageEntries) / sizeof(storageEntries[0]);
//	  uint32_t storageInitError = 0;
//	#endif
//
//	  /* Configure microcontroller. */
//
//
//	  /* Allocate memory */
//	  CO_config_t *config_ptr = NULL;
//
//	#ifdef CO_MULTIPLE_OD
//	  /* example usage of CO_MULTIPLE_OD (but still single OD here) */
//	  CO_config_t co_config = {0};
//	  OD_INIT_CONFIG(co_config); /* helper macro from OD.h */
//	  co_config.CNT_LEDS = 0;
//	  co_config.CNT_LSS_SLV = 1;
//	  config_ptr = &co_config;
//	#endif /* CO_MULTIPLE_OD */
//	  *CO = CO_new(config_ptr, &heapMemoryUsed);
//	  if (CO == NULL) {
//	      printf("Error: Can't allocate memory\n");
//	      return 0;
//	  }
//	  else {
//	      printf("Allocated %u bytes for CANopen objects\n", heapMemoryUsed);
//	  }
//
//
//	#if (CO_CONFIG_STORAGE) & CO_CONFIG_STORAGE_ENABLE
//	  err = CO_storageBlank_init(&storage,
//	                             CO->CANmodule,
//	                             OD_ENTRY_H1010_storeParameters,
//	                             OD_ENTRY_H1011_restoreDefaultParameters,
//	                             storageEntries,
//	                             storageEntriesCount,
//	                             &storageInitError);
//
//	  if (err != CO_ERROR_NO && err != CO_ERROR_DATA_CORRUPT) {
//	      printf("Error: Storage %d\n", storageInitError);
//	      return 0;
//	  }
//	#endif
//	  return 1;
//}
//
//uint8_t CO_ProccesApp(CO_t *CO, CO_State_t *CO_State)
//{
//	if(CO_State->reset == CO_RESET_APP)
//	{
//		  /* delete objects from memory */
//		  CO_CANsetConfigurationMode(CO_State->CANptr);
//		  CO_delete(CO);
//
//		  printf("CANopenNode finished\n");
//		  //HAL_NVIC_SystemReset();
//
//		  /* reset */
//		  return 0;
//	}
//	if(CO_State->reset != CO_RESET_NOT)
//	{
//		/* CANopen communication reset - initialize CANopen objects *******************/
//			  printf("CANopenNode - Reset communication...\n");
//
//			  /* Wait rt_thread. */
//			  CO->CANmodule->CANnormal = false;
//
//			  /* Enter CAN configuration. */
//			  CO_CANsetConfigurationMode(CO_State->CANptr);
//			  //CO_CANmodule_disable(&(CO.CANmodule));
//
//			  /* initialize CANopen */
//
//			  CO_State->err = CO_CANinit(CO, CO_State->CANptr, CO_State->pendingBitRate);
//			  if (CO_State->err != CO_ERROR_NO) {
//				  printf("Error: CAN initialization failed: %d\n", CO_State->err);
//				  return 0;
//			  }
//
//			  CO_LSS_address_t lssAddress = {.identity = {
//				  .vendorID = OD_PERSIST_COMM.x1018_identity.vendor_ID,
//				  .productCode = OD_PERSIST_COMM.x1018_identity.productCode,
//				  .revisionNumber = OD_PERSIST_COMM.x1018_identity.revisionNumber,
//				  .serialNumber = OD_PERSIST_COMM.x1018_identity.serialNumber
//			  }};
//			  CO_State->err = CO_LSSinit(CO, &lssAddress, &(CO_State->pendingNodeId), &(CO_State->pendingBitRate));
//			  if(CO_State->err != CO_ERROR_NO) {
//				  printf("Error: LSS slave initialization failed: %d\n", CO_State->err);
//				  return 0;
//			  }
//
//			  CO_State->activeNodeId = CO_State->pendingNodeId;
//			  uint32_t errInfo = 0;
//
//			  CO_State->err = CO_CANopenInit(CO,                /* CANopen object */
//								   NULL,              /* alternate NMT */
//								   NULL,              /* alternate em */
//								   OD,                /* Object dictionary */
//								   OD_STATUS_BITS,    /* Optional OD_statusBits */
//								   NMT_CONTROL,       /* CO_NMT_control_t */
//								   FIRST_HB_TIME,     /* firstHBTime_ms */
//								   SDO_SRV_TIMEOUT_TIME, /* SDOserverTimeoutTime_ms */
//								   SDO_CLI_TIMEOUT_TIME, /* SDOclientTimeoutTime_ms */
//								   SDO_CLI_BLOCK,     /* SDOclientBlockTransfer */
//								   CO_State->activeNodeId,
//								   &errInfo);
//			  if(CO_State->err != CO_ERROR_NO && CO_State->err != CO_ERROR_NODE_ID_UNCONFIGURED_LSS) {
//				  if (CO_State->err == CO_ERROR_OD_PARAMETERS) {
//					  printf("Error: Object Dictionary entry 0x%X\n", errInfo);
//				  }
//				  else {
//					  printf("Error: CANopen initialization failed: %d\n", CO_State->err);
//				  }
//				  return 0;
//			  }
//
//			  CO_State->err = CO_CANopenInitPDO(CO, CO->em, OD, CO_State->activeNodeId, &errInfo);
//			  if(CO_State->err != CO_ERROR_NO) {
//				  if (CO_State->err == CO_ERROR_OD_PARAMETERS) {
//					  printf("Error: Object Dictionary entry 0x%X\n", errInfo);
//				  }
//				  else {
//					  printf("Error: PDO initialization failed: %d\n", CO_State->err);
//				  }
//				  //return 0;
//			  }
//
//			  /* Configure Timer interrupt function for execution every 1 millisecond */
//
//
//			  /* Configure CAN transmit and receive interrupt */
//
//
//			  /* Configure CANopen callbacks, etc */
//			  if(!CO->nodeIdUnconfigured) {
//
//		#if (CO_CONFIG_STORAGE) & CO_CONFIG_STORAGE_ENABLE
//				  if(storageInitError != 0) {
//					  CO_errorReport(CO->em, CO_EM_NON_VOLATILE_MEMORY,
//									 CO_EMC_HARDWARE, storageInitError);
//				  }
//		#endif
//			  }
//			  else {
//				  printf("CANopenNode - Node-id not initialized\n");
//			  }
//
//
//			  /* start CAN */
//			  CO_CANsetNormalMode(CO->CANmodule);
//
//			  CO_State->reset = CO_RESET_NOT;
//
//			  printf("CANopenNode - Running...\n");
//			  //fflush(stdout);
//			  HAL_TIM_Base_Start_IT(pHTIM); //1ms interrupt
//			  CO_State->time_old = CO_State->time_current = HAL_GetTick();
//	}
//	return 1;
//}
//
//
//uint8_t CO_ProccesComm(CO_t *CO, CO_State_t *CO_State)
//{
//	/* loop for normal program execution ******************************************/
//	/* get time difference since last function call */
//	uint32_t timeDifference_us = (CO_State->time_current - CO_State->time_old) * 1000;
//	/* CANopen process */
//	CO_State->time_old = CO_State->time_current;
//	CO_State->reset = CO_process(CO, false, timeDifference_us, NULL);
//	/* optional sleep for short time */
//	return 1;
//}
//
//
//
//CO_SDO_return_t CO_SDO_read(CO_t *CO,CO_SDOMsg_t *msg)
//{
//	static bool_t SDOUploadInit = false;
//	static bool_t SDOWaitResponse = false;
//	static CO_SDO_return_t SDOStateResponse;
//
//    size_t sizeIndicated;
//    size_t sizeTransferred;
//    uint32_t timerNext_us;
//
//	uint16_t SDOtimeoutTime_ms = 10;
//	uint16_t SDOtimeDifference_us = 1000;
//	bool_t SDOblockEnable = false;
//	bool_t SDOclientAbort = false;
//
//	if(!SDOUploadInit){
//		msg->SDOabortCode = CO_SDO_AB_NONE;
//		SDOStateResponse = CO_SDOclient_setup(CO->SDOclient,
//						  CO_CAN_ID_SDO_CLI + msg->NodeIdReq,
//						  CO_CAN_ID_SDO_SRV + msg->NodeIdReq,
//						  msg->NodeIdReq);
//		if(SDOStateResponse != CO_SDO_RT_ok_communicationEnd) return SDOStateResponse;
//		SDOStateResponse = CO_SDOclientUploadInitiate(CO->SDOclient,
//													msg->index,
//													msg->subIndex,
//													SDOtimeoutTime_ms,
//													SDOblockEnable);
//		if(SDOStateResponse != CO_SDO_RT_ok_communicationEnd) return SDOStateResponse;
//		SDOUploadInit = true;
//		SDOWaitResponse = true;
//	}
//
//	if(SDOWaitResponse){
//		SDOStateResponse = CO_SDOclientUpload(CO->SDOclient,
//											SDOtimeDifference_us,
//											SDOclientAbort,
//											&(msg->SDOabortCode),
//											&sizeIndicated,
//											&sizeTransferred,
//											&timerNext_us);
//
//		if(SDOStateResponse < 0)
//		{
//			SDOWaitResponse = SDOUploadInit = false;
//			msg->flagResponse = true;
//			return CO_SDO_RT_wrongArguments;
//		}else if(SDOStateResponse == 0)
//		{
//			SDOWaitResponse = SDOUploadInit = false;
//			msg->flagResponse = true;
//			msg->msgBufSize = CO_SDOclientUploadBufRead(CO->SDOclient, msg->msgBuf, 32);
//			return CO_SDO_RT_ok_communicationEnd;
//		}
//
//	}
//
//	return CO_SDO_RT_waitingResponse;
//}
//
//CO_SDO_return_t CO_SDO_write(CO_t *CO,CO_SDOMsg_t *msg){
//
//	static bool_t SDODownloadInit = false;
//	static bool_t SDODownload = false;
//	static CO_SDO_return_t SDOStateResponse;
//
//	uint16_t dataSize;
//	size_t sizeTransferred;
//	uint32_t timerNext_us;
//
//	uint16_t SDOtimeoutTime_ms = 10;
//	bool_t SDOblockEnable = false;
//	uint32_t timeDifference_us = 1000;
//	bool_t SDOclientAbort = false;
//
//	if(!SDODownloadInit)
//	{
//		msg->SDOabortCode = CO_SDO_AB_NONE;
//		SDOStateResponse = CO_SDOclient_setup(CO->SDOclient,
//											  CO_CAN_ID_SDO_CLI + msg->NodeIdReq,
//											  CO_CAN_ID_SDO_SRV + msg->NodeIdReq,
//											  msg->NodeIdReq);
//		if(SDOStateResponse != CO_SDO_RT_ok_communicationEnd) return SDOStateResponse;
//
//		SDOStateResponse = CO_SDOclientDownloadInitiate(	CO->SDOclient,
//												msg->index,
//												msg->subIndex,
//												msg->msgBufSize,
//												SDOtimeoutTime_ms,
//												SDOblockEnable);
//	    if(SDOStateResponse != CO_SDO_RT_ok_communicationEnd) return SDOStateResponse;
//
//	    dataSize = CO_SDOclientDownloadBufWrite(CO->SDOclient, msg->msgBuf, msg->msgBufSize);
//
//	    if(dataSize < msg->msgBufSize)
//	    {
//	    	SDOblockEnable = true;
//	    }
//
//	    SDODownloadInit = true;
//	    SDODownload = true;
//	}
//
//
//	if(SDODownload)
//	{
//		SDOStateResponse = CO_SDOclientDownload(	CO->SDOclient,
//													timeDifference_us,
//													SDOclientAbort,
//													SDOblockEnable,
//													&(msg->SDOabortCode),
//													&sizeTransferred,
//													&timerNext_us);
//		if(SDOStateResponse < 0)
//		{
//			SDODownload = SDODownloadInit = false;
//			msg->flagResponse = true;
//			return CO_SDO_RT_wrongArguments;
//		}else if(SDOStateResponse == 0)
//		{
//			SDODownloadInit = SDODownload = false;
//			msg->flagResponse = true;
//			return CO_SDO_RT_ok_communicationEnd;
//		}
//
//	}
//
//	return CO_SDO_RT_waitingResponse;
//}
//
//ODR_t CO_OD_read(OD_t* OD, uint16_t index, uint8_t subIndex, void *buf, uint8_t sizeofbuf, bool_t odOrig)
//{
//	OD_entry_t* objOD;
//	ODR_t ODR;
//	OD_IO_t io;
//	OD_size_t readcount;
//
//	objOD = OD_find(OD, index);
//	ODR = OD_getSub(objOD, subIndex, &io, odOrig);
//
//	if(sizeofbuf < io.stream.dataLength) ODR = ODR_DATA_SHORT;
//	if(sizeofbuf > io.stream.dataLength) ODR = ODR_DATA_LONG;
//
//	if(ODR == ODR_OK)
//	{
//		ODR = io.read(&(io.stream), buf,
//				  	  io.stream.dataLength, &readcount);
//		if(ODR == ODR_OK && readcount != sizeofbuf) ODR = ODR_GENERAL;
//	}
//	return ODR;
//}
//
//ODR_t CO_OD_write(OD_t* OD, uint16_t index, uint8_t subIndex, void *buf, uint8_t sizeofbuf, bool_t odOrig)
//{
//	OD_entry_t* objOD;
//	ODR_t ODR;
//	OD_IO_t io;
//	OD_size_t writecount;
//
//	objOD = OD_find(OD, index);
//	ODR = OD_getSub(objOD, subIndex, &io, odOrig);
//
//	if(sizeofbuf < io.stream.dataLength) ODR = ODR_DATA_SHORT;
//	if(sizeofbuf > io.stream.dataLength) ODR = ODR_DATA_LONG;
//
//	if(ODR == ODR_OK)
//	{
//		ODR = io.write(&(io.stream), buf,
//				  	  io.stream.dataLength, &writecount);
//		if(ODR == ODR_OK && writecount != sizeofbuf) ODR = ODR_GENERAL;
//	}
//	return ODR;
//}
