#include "CANOpenSimple.h"

#define pHTIM htim14
extern TIM_HandleTypeDef *pHTIM;

uint8_t CO_Init(CO_t **CO, CO_State_t *state, uint8_t NodeId, uint16_t BitRate, CAN_HandleTypeDef *hcan)
{
	uint32_t heapMemoryUsed;
	state->reset = -1;
	state->CANptr = hcan; /* CAN module address */
	state->pendingNodeId = NodeId; /* read from dip switches or nonvolatile memory, configurable by LSS slave */
	state->activeNodeId = NodeId; /* Copied from CO_pendingNodeId in the communication reset section */
	state->pendingBitRate = BitRate;  /* read from dip switches or nonvolatile memory, configurable by LSS slave */

	#if (CO_CONFIG_STORAGE) & CO_CONFIG_STORAGE_ENABLE
	  CO_storage_t storage;
	  CO_storage_entry_t storageEntries[] = {
	      {
	          .addr = &OD_PERSIST_COMM,
	          .len = sizeof(OD_PERSIST_COMM),
	          .subIndexOD = 2,
	          .attr = CO_storage_cmd | CO_storage_restore,
	          .addrNV = NULL
	      }
	  };
	  uint8_t storageEntriesCount = sizeof(storageEntries) / sizeof(storageEntries[0]);
	  uint32_t storageInitError = 0;
	#endif

	  /* Configure microcontroller. */


	  /* Allocate memory */
	  CO_config_t *config_ptr = NULL;
	  
	#ifdef CO_MULTIPLE_OD
	  /* example usage of CO_MULTIPLE_OD (but still single OD here) */
	  CO_config_t co_config = {0};
	  OD_INIT_CONFIG(co_config); /* helper macro from OD.h */
	  co_config.CNT_LEDS = 0;
	  co_config.CNT_LSS_SLV = 1;
	  config_ptr = &co_config;
	#endif /* CO_MULTIPLE_OD */
	  *CO = CO_new(config_ptr, &heapMemoryUsed);
	  if (CO == NULL) {
	      printf("Error: Can't allocate memory\n");
	      return 0;
	  }
	  else {
	      printf("Allocated %u bytes for CANopen objects\n", (unsigned int)heapMemoryUsed);
	  }


	#if (CO_CONFIG_STORAGE) & CO_CONFIG_STORAGE_ENABLE
	  err = CO_storageBlank_init(&storage,
	                             CO->CANmodule,
	                             OD_ENTRY_H1010_storeParameters,
	                             OD_ENTRY_H1011_restoreDefaultParameters,
	                             storageEntries,
	                             storageEntriesCount,
	                             &storageInitError);

	  if (err != CO_ERROR_NO && err != CO_ERROR_DATA_CORRUPT) {
	      printf("Error: Storage %d\n", storageInitError);
	      return 0;
	  }
	#endif
	  return 1;
}

uint8_t CO_ProccesApp(CO_t *CO, CO_State_t *CO_State)
{
	if(CO_State->reset == CO_RESET_APP)
	{
		  /* delete objects from memory */
		  CO_CANsetConfigurationMode(CO_State->CANptr);
		  CO_delete(CO);

		  printf("CANopenNode finished\n");
		  //HAL_NVIC_SystemReset();

		  /* reset */
		  return 0;
	}
	if(CO_State->reset != CO_RESET_NOT)
	{
		/* CANopen communication reset - initialize CANopen objects *******************/
			  printf("CANopenNode - Reset communication...\n");

			  /* Wait rt_thread. */
			  CO->CANmodule->CANnormal = false;

			  /* Enter CAN configuration. */
			  CO_CANsetConfigurationMode(CO_State->CANptr);
			  //CO_CANmodule_disable(&(CO.CANmodule));

			  /* initialize CANopen */

			  CO_State->err = CO_CANinit(CO, CO_State->CANptr, CO_State->pendingBitRate);
			  if (CO_State->err != CO_ERROR_NO) {
				  printf("Error: CAN initialization failed: %d\n", CO_State->err);
				  return 0;
			  }

			  CO_LSS_address_t lssAddress = {.identity = {
				  .vendorID = OD_PERSIST_COMM.x1018_identity.vendor_ID,
				  .productCode = OD_PERSIST_COMM.x1018_identity.productCode,
				  .revisionNumber = OD_PERSIST_COMM.x1018_identity.revisionNumber,
				  .serialNumber = OD_PERSIST_COMM.x1018_identity.serialNumber
			  }};
			  CO_State->err = CO_LSSinit(CO, &lssAddress, &(CO_State->pendingNodeId), &(CO_State->pendingBitRate));
			  if(CO_State->err != CO_ERROR_NO) {
				  printf("Error: LSS slave initialization failed: %d\n", CO_State->err);
				  return 0;
			  }

			  CO_State->activeNodeId = CO_State->pendingNodeId;
			  uint32_t errInfo = 0;

			  CO_State->err = CO_CANopenInit(CO,                /* CANopen object */
								   NULL,              /* alternate NMT */
								   NULL,              /* alternate em */
								   OD,                /* Object dictionary */
								   OD_STATUS_BITS,    /* Optional OD_statusBits */
								   NMT_CONTROL,       /* CO_NMT_control_t */
								   FIRST_HB_TIME,     /* firstHBTime_ms */
								   SDO_SRV_TIMEOUT_TIME, /* SDOserverTimeoutTime_ms */
								   SDO_CLI_TIMEOUT_TIME, /* SDOclientTimeoutTime_ms */
								   SDO_CLI_BLOCK,     /* SDOclientBlockTransfer */
								   CO_State->activeNodeId,
								   &errInfo);
			  if(CO_State->err != CO_ERROR_NO && CO_State->err != CO_ERROR_NODE_ID_UNCONFIGURED_LSS) {
				  if (CO_State->err == CO_ERROR_OD_PARAMETERS) {
					  printf("Error: Object Dictionary entry 0x%X\n", (unsigned int)errInfo);
				  }
				  else {
					  printf("Error: CANopen initialization failed: %d\n", CO_State->err);
				  }
				  return 0;
			  }

			  CO_State->err = CO_CANopenInitPDO(CO, CO->em, OD, CO_State->activeNodeId, &errInfo);
			  if(CO_State->err != CO_ERROR_NO) {
				  if (CO_State->err == CO_ERROR_OD_PARAMETERS) {
					  printf("Error: Object Dictionary entry 0x%X\n", (unsigned int)errInfo);
				  }
				  else {
					  printf("Error: PDO initialization failed: %d\n", CO_State->err);
				  }
				  //return 0;
			  }

			  /* Configure Timer interrupt function for execution every 1 millisecond */


			  /* Configure CAN transmit and receive interrupt */


			  /* Configure CANopen callbacks, etc */
			  if(!CO->nodeIdUnconfigured) {

		#if (CO_CONFIG_STORAGE) & CO_CONFIG_STORAGE_ENABLE
				  if(storageInitError != 0) {
					  CO_errorReport(CO->em, CO_EM_NON_VOLATILE_MEMORY,
									 CO_EMC_HARDWARE, storageInitError);
				  }
		#endif
			  }
			  else {
				  printf("CANopenNode - Node-id not initialized\n");
			  }


			  /* start CAN */
			  CO_CANsetNormalMode(CO->CANmodule);

			  CO_State->reset = CO_RESET_NOT;

			  printf("CANopenNode - Running...\n");
			  //fflush(stdout);
			  HAL_TIM_Base_Start_IT(pHTIM); //1ms interrupt
			  CO_State->time_old = CO_State->time_current = HAL_GetTick();
	}
	return 1;
}


uint8_t CO_ProccesComm(CO_t *CO, CO_State_t *CO_State)
{
	/* loop for normal program execution ******************************************/
	/* get time difference since last function call */
	uint32_t timeDifference_us = (CO_State->time_current - CO_State->time_old) * 1000;
	/* CANopen process */
	CO_State->time_old = CO_State->time_current;
	CO_State->reset = CO_process(CO, false, timeDifference_us, NULL);
	/* optional sleep for short time */
	return 1;
}



CO_SDO_return_t CO_SDO_read(CO_t *CO,CO_SDOMsg_t *msg)
{
	static bool_t SDOUploadInit = false;
	static bool_t SDOWaitResponse = false;
	static CO_SDO_return_t SDOStateResponse;

    size_t sizeIndicated;
    size_t sizeTransferred;
    uint32_t timerNext_us;

	uint16_t SDOtimeoutTime_ms = 10;
	uint16_t SDOtimeDifference_us = 1000;
	bool_t SDOblockEnable = false;
	bool_t SDOclientAbort = false;

	if(!SDOUploadInit){
		msg->SDOabortCode = CO_SDO_AB_NONE;
		SDOStateResponse = CO_SDOclient_setup(CO->SDOclient,
						  CO_CAN_ID_SDO_CLI + msg->NodeIdReq,
						  CO_CAN_ID_SDO_SRV + msg->NodeIdReq,
						  msg->NodeIdReq);
		if(SDOStateResponse != CO_SDO_RT_ok_communicationEnd) return SDOStateResponse;
		SDOStateResponse = CO_SDOclientUploadInitiate(CO->SDOclient,
													msg->index,
													msg->subIndex,
													SDOtimeoutTime_ms,
													SDOblockEnable);
		if(SDOStateResponse != CO_SDO_RT_ok_communicationEnd) return SDOStateResponse;
		SDOUploadInit = true;
		SDOWaitResponse = true;
	}

	if(SDOWaitResponse){
		SDOStateResponse = CO_SDOclientUpload(CO->SDOclient,
											SDOtimeDifference_us,
											SDOclientAbort,
											&(msg->SDOabortCode),
											&sizeIndicated,
											&sizeTransferred,
											&timerNext_us);

		if(SDOStateResponse < 0)
		{
			SDOWaitResponse = SDOUploadInit = false;
			msg->flagResponse = true;
			return CO_SDO_RT_wrongArguments;
		}else if(SDOStateResponse == 0)
		{
			SDOWaitResponse = SDOUploadInit = false;
			msg->flagResponse = true;
			msg->msgBufSize = CO_SDOclientUploadBufRead(CO->SDOclient, msg->msgBuf, 32);
			return CO_SDO_RT_ok_communicationEnd;
		}

	}

	return CO_SDO_RT_waitingResponse;
}

CO_SDO_return_t CO_SDO_write(CO_t *CO,CO_SDOMsg_t *msg){

	static bool_t SDODownloadInit = false;
	static bool_t SDODownload = false;
	static CO_SDO_return_t SDOStateResponse;

	uint16_t dataSize;
	size_t sizeTransferred;
	uint32_t timerNext_us;

	uint16_t SDOtimeoutTime_ms = 10;
	bool_t SDOblockEnable = false;
	uint32_t timeDifference_us = 1000;
	bool_t SDOclientAbort = false;

	if(!SDODownloadInit)
	{
		msg->SDOabortCode = CO_SDO_AB_NONE;
		SDOStateResponse = CO_SDOclient_setup(CO->SDOclient,
											  CO_CAN_ID_SDO_CLI + msg->NodeIdReq,
											  CO_CAN_ID_SDO_SRV + msg->NodeIdReq,
											  msg->NodeIdReq);
		if(SDOStateResponse != CO_SDO_RT_ok_communicationEnd) return SDOStateResponse;

		SDOStateResponse = CO_SDOclientDownloadInitiate(	CO->SDOclient,
												msg->index,
												msg->subIndex,
												msg->msgBufSize,
												SDOtimeoutTime_ms,
												SDOblockEnable);
	    if(SDOStateResponse != CO_SDO_RT_ok_communicationEnd) return SDOStateResponse;

	    dataSize = CO_SDOclientDownloadBufWrite(CO->SDOclient, msg->msgBuf, msg->msgBufSize);

	    if(dataSize < msg->msgBufSize)
	    {
	    	SDOblockEnable = true;
	    }

	    SDODownloadInit = true;
	    SDODownload = true;
	}


	if(SDODownload)
	{
		SDOStateResponse = CO_SDOclientDownload(	CO->SDOclient,
													timeDifference_us,
													SDOclientAbort,
													SDOblockEnable,
													&(msg->SDOabortCode),
													&sizeTransferred,
													&timerNext_us);
		if(SDOStateResponse < 0)
		{
			SDODownload = SDODownloadInit = false;
			msg->flagResponse = true;
			return CO_SDO_RT_wrongArguments;
		}else if(SDOStateResponse == 0)
		{
			SDODownloadInit = SDODownload = false;
			msg->flagResponse = true;
			return CO_SDO_RT_ok_communicationEnd;
		}

	}

	return CO_SDO_RT_waitingResponse;
}

ODR_t CO_OD_read(OD_t* OD, uint16_t index, uint8_t subIndex, void *buf, uint8_t sizeofbuf, bool_t odOrig)
{
	OD_entry_t* objOD;
	ODR_t ODR;
	OD_IO_t io;
	OD_size_t readcount;

	objOD = OD_find(OD, index);
	ODR = OD_getSub(objOD, subIndex, &io, odOrig);

	if(sizeofbuf < io.stream.dataLength) ODR = ODR_DATA_SHORT;
	if(sizeofbuf > io.stream.dataLength) ODR = ODR_DATA_LONG;

	if(ODR == ODR_OK)
	{
		ODR = io.read(&(io.stream), buf,
				  	  io.stream.dataLength, &readcount);
		if(ODR == ODR_OK && readcount != sizeofbuf) ODR = ODR_GENERAL;
	}
	return ODR;
}

ODR_t CO_OD_write(OD_t* OD, uint16_t index, uint8_t subIndex, void *buf, uint8_t sizeofbuf, bool_t odOrig)
{
	OD_entry_t* objOD;
	ODR_t ODR;
	OD_IO_t io;
	OD_size_t writecount;

	objOD = OD_find(OD, index);
	ODR = OD_getSub(objOD, subIndex, &io, odOrig);

	if(sizeofbuf < io.stream.dataLength) ODR = ODR_DATA_SHORT;
	if(sizeofbuf > io.stream.dataLength) ODR = ODR_DATA_LONG;

	if(ODR == ODR_OK)
	{
		ODR = io.write(&(io.stream), buf,
				  	  io.stream.dataLength, &writecount);
		if(ODR == ODR_OK && writecount != sizeofbuf) ODR = ODR_GENERAL;
	}
	return ODR;
}

void CO_Task1ms(CO_t *CO)
{
	CO_LOCK_OD(CO->CANmodule);
	if (!CO->nodeIdUnconfigured && CO->CANmodule->CANnormal) {
		bool_t syncWas = false;
		/* get time difference since last function call */
		uint32_t timeDifference_us = 1000;

#if (CO_CONFIG_SYNC) & CO_CONFIG_SYNC_ENABLE
		syncWas = CO_process_SYNC(CO, timeDifference_us, NULL);
#endif
#if (CO_CONFIG_PDO) & CO_CONFIG_RPDO_ENABLE
		CO_process_RPDO(CO, syncWas, timeDifference_us, NULL);
#endif
#if (CO_CONFIG_PDO) & CO_CONFIG_TPDO_ENABLE
		CO_process_TPDO(CO, syncWas, timeDifference_us, NULL);
#endif

		/* Further I/O or nonblocking application code may go here. */
	}
	CO_UNLOCK_OD(CO->CANmodule);

}
