/*
 * skp_can.c
 *
 *  Created on: Jan 14, 2020
 *      Author: sukkinpang
 */
#include "main.h"
#include "skp_can.h"
#include <stdio.h>

extern FDCAN_HandleTypeDef hfdcan1;
extern FDCAN_TxHeaderTypeDef TxHeader;
extern FDCAN_RxHeaderTypeDef RxHeader;
extern uint8_t RxData[64];


/**
  * @brief  Rx FIFO 0 callback.
  * @param  hfdcan pointer to an FDCAN_HandleTypeDef structure that contains
  *         the configuration information for the specified FDCAN.
  * @param  RxFifo0ITs indicates which Rx FIFO 0 interrupts are signaled.
  *         This parameter can be any combination of @arg FDCAN_Rx_Fifo0_Interrupts.
  * @retval None
  */

static const uint8_t DLCtoBytes[] = {0, 1, 2, 3, 4, 5, 6, 7, 8, 12, 16, 20, 24, 32, 48, 64};
/*
void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{

	uint8_t i, dlc;

	if((RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) != 0)
	{
		// Retrieve Rx messages from RX FIFO0
		if (HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &RxHeader, RxData) != HAL_OK)
		{
		  Error_Handler();
		}else
		{
			dlc = DLCtoBytes[RxHeader.DataLength >> 16];

			printf("Len:%x ID:%x Data:",dlc,RxHeader.Identifier);

			for(i=0;i< dlc;i++)
			{
				printf("%x ",RxData[i]);
			}

			printf(" \n");

		}
	}
}

*/
void skp_can_init(void)
{

	  hfdcan1.Instance = FDCAN1;
	  hfdcan1.Init.ClockDivider = FDCAN_CLOCK_DIV1;
	  hfdcan1.Init.FrameFormat = FDCAN_FRAME_FD_BRS;
	  hfdcan1.Init.Mode = FDCAN_MODE_NORMAL;
	  hfdcan1.Init.AutoRetransmission = ENABLE;
	  hfdcan1.Init.TransmitPause = ENABLE;
	  hfdcan1.Init.ProtocolException = DISABLE;

      // These values are based on a 80MHz clock to the CAN module

	  /*
	  hfdcan1.Init.NominalPrescaler = 1;			//1000kbps
	  hfdcan1.Init.NominalSyncJumpWidth = 1;
	  hfdcan1.Init.NominalTimeSeg1 = 63;
	  hfdcan1.Init.NominalTimeSeg2 = 16;
       */

	  hfdcan1.Init.NominalPrescaler = 1;			//500kbps
	  hfdcan1.Init.NominalSyncJumpWidth = 16;
	  hfdcan1.Init.NominalTimeSeg1 = 119;
	  hfdcan1.Init.NominalTimeSeg2 = 40;

	  /*

	  hfdcan1.Init.DataPrescaler = 1;
	  hfdcan1.Init.DataSyncJumpWidth = 1;
	  hfdcan1.Init.DataTimeSeg1 = 7;				//8000kbps
	  hfdcan1.Init.DataTimeSeg2 = 2;

	  hfdcan1.Init.DataPrescaler = 2;
	  hfdcan1.Init.DataSyncJumpWidth = 1;
	  hfdcan1.Init.DataTimeSeg1 = 7;				//4000kbps
	  hfdcan1.Init.DataTimeSeg2 = 2;
*/

	  hfdcan1.Init.DataPrescaler = 1;
	  hfdcan1.Init.DataSyncJumpWidth = 1;
	  hfdcan1.Init.DataTimeSeg1 = 30;				//2000kbps
	  hfdcan1.Init.DataTimeSeg2 = 9;

	  hfdcan1.Init.StdFiltersNbr = 1;
	  hfdcan1.Init.ExtFiltersNbr = 0;

	  hfdcan1.Init.TxFifoQueueMode = FDCAN_TX_FIFO_OPERATION;

	  if (HAL_FDCAN_Init(&hfdcan1) != HAL_OK)
	  {

		printf("##########  HAL_FDCAN_Init ERROR 1 ##########\n");
	    Error_Handler();
	  }
}

void FDCAN_Config(void)
{
  FDCAN_FilterTypeDef sFilterConfig;

  /* Configure Rx filter */
  sFilterConfig.IdType = FDCAN_STANDARD_ID;
  sFilterConfig.FilterIndex = 0;
  sFilterConfig.FilterType = FDCAN_FILTER_MASK;
  sFilterConfig.FilterType = 0;
  sFilterConfig.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
  sFilterConfig.FilterID1 = 0x321;
  sFilterConfig.FilterID2 = 0x7FF;

  //sFilterConfig.FilterID1 = 0;
  //sFilterConfig.FilterID2 = 0;


  if (HAL_FDCAN_ConfigFilter(&hfdcan1, &sFilterConfig) != HAL_OK)

  {
    printf("##########  HAL_FDCAN_ConfigFilter ERROR 2 ##########\n");
    Error_Handler();
  }

  /* Configure global filter:
     Filter all remote frames with STD and EXT ID
     Reject non matching frames with STD ID and EXT ID */
  //if (HAL_FDCAN_ConfigGlobalFilter(&hfdcan1, FDCAN_REJECT, FDCAN_REJECT, FDCAN_FILTER_REMOTE, FDCAN_FILTER_REMOTE) != HAL_OK)
  if (HAL_FDCAN_ConfigGlobalFilter(&hfdcan1, FDCAN_ACCEPT_IN_RX_FIFO0, FDCAN_ACCEPT_IN_RX_FIFO0, FDCAN_FILTER_REMOTE, FDCAN_FILTER_REMOTE) != HAL_OK)
  {
	  printf("##########  HAL_FDCAN_ConfigFilter ERROR 3 ##########\n");
    Error_Handler();
  }

// 6,2 working for a bit
  if (HAL_FDCAN_ConfigTxDelayCompensation(&hfdcan1, 6, 2) != HAL_OK)
  {
	  printf("##########  HAL_FDCAN_ConfigFilter ERROR 6 ##########\n");
    Error_Handler();
  }

  if (HAL_FDCAN_EnableTxDelayCompensation(&hfdcan1) != HAL_OK)
  {
	  printf("##########  HAL_FDCAN_ConfigFilter ERROR 7 ##########\n");
    Error_Handler();
  }


  /* Start the FDCAN module */
  if (HAL_FDCAN_Start(&hfdcan1) != HAL_OK)
  {
	  printf("##########  HAL_FDCAN_ConfigFilter ERROR 4 ##########\n");
    Error_Handler();
  }

  if (HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0) != HAL_OK)
  {
	  printf("##########  HAL_FDCAN_ConfigFilter ERROR 5 ##########\n");
    Error_Handler();
  }

  /* Prepare Tx Header */
  TxHeader.Identifier = 0x111;
  TxHeader.IdType = FDCAN_STANDARD_ID;
  TxHeader.TxFrameType = FDCAN_DATA_FRAME;
  TxHeader.DataLength = FDCAN_DLC_BYTES_64;
  TxHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
  TxHeader.BitRateSwitch = FDCAN_BRS_ON;
  TxHeader.FDFormat = FDCAN_FD_CAN;
  TxHeader.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
  TxHeader.MessageMarker = 0;
  //if (HAL_FDCAN_ConfigTxDelayCompensation(&hfdcan1, 5, 0) != HAL_OK)

}
