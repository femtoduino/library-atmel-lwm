#include "../../../config.h"
/**
 * \file phy.c
 *
 * \brief AT86RF233 PHY implementation
 *
 * Copyright (C) 2012-2014, Atmel Corporation. All rights reserved.
 *
 * \asf_license_start
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * 3. The name of Atmel may not be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * 4. This software may only be redistributed and used in connection with an
 *    Atmel microcontroller product.
 *
 * THIS SOFTWARE IS PROVIDED BY ATMEL "AS IS" AND ANY EXPRESS OR IMPLIED
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT ARE
 * EXPRESSLY AND SPECIFICALLY DISCLAIMED. IN NO EVENT SHALL ATMEL BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 * STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 * \asf_license_stop
 *
 * Modification and other use of this code is subject to Atmel's Limited
 * License Agreement (license.txt).
 *
 * $Id: phy.c 9267 2014-03-18 21:46:19Z ataradov $
 *
 */

#ifdef PHY_AT86RF233

/*- Includes ---------------------------------------------------------------*/
#include "../sys/sysTypes.h"
#include "../hal/hal.h"
#include "../hal/halPhy.h"
#include "../phy/phy.h"

#include "at86rf233.h"

/*- Definitions ------------------------------------------------------------*/
#define PHY_CRC_SIZE          2
#define TRX_RPC_REG_VALUE     0xeb
#define IRQ_CLEAR_VALUE       0xff

/*- Types ------------------------------------------------------------------*/
typedef enum
{
  PHY_STATE_INITIAL,
  PHY_STATE_IDLE,
  PHY_STATE_SLEEP,
  PHY_STATE_TX_WAIT_END,
} PhyState_t;

/*- Prototypes -------------------------------------------------------------*/
static void phyWriteRegister(uint8_t reg, uint8_t value);
static uint8_t phyReadRegister(uint8_t reg);
static void phyWaitState(uint8_t state);
static void phyTrxSetState(uint8_t state);
static void phySetChannel(void);
static void phySetRxState(void);

/*- Variables --------------------------------------------------------------*/
static PhyState_t phyState = PHY_STATE_INITIAL;
static uint8_t phyRxBuffer[128];
static bool phyRxState;
static uint8_t phyChannel;
static uint8_t phyBand;

/*- Implementations --------------------------------------------------------*/

/*************************************************************************//**
*****************************************************************************/
void PHY_Init(void)
{
//   HAL_PhyReset();
//   TRXPR_REG_s.trxrst = 1;

//   phyRxState = false;
//   phyBand = 0;
//   phyState = PHY_STATE_IDLE;

//   phyTrxSetState(TRX_CMD_TRX_OFF);

//   TRX_RPC_REG = TRX_RPC_REG_VALUE;

//   TRX_CTRL_2_REG_s.rxSafeMode = 1;

// #ifdef PHY_ENABLE_RANDOM_NUMBER_GENERATOR
//   CSMA_SEED_0_REG = (uint8_t)PHY_RandomReq();
// #endif

  HAL_PhyReset();
  phyRxState = false;
  phyState = PHY_STATE_IDLE;

  phyWriteRegister(TRX_STATE_REG, TRX_CMD_TRX_OFF);
  phyWaitState(TRX_STATUS_TRX_OFF);

  phyWriteRegister(TRX_CTRL_1_REG, (1<<TX_AUTO_CRC_ON) | (3<<SPI_CMD_MODE) |
      (1<<IRQ_MASK_MODE));

  phyWriteRegister(TRX_CTRL_2_REG, (1<<RX_SAFE_MODE) | (1<<OQPSK_SCRAM_EN));
}

/*************************************************************************//**
*****************************************************************************/
void PHY_SetRxState(bool rx)
{
  phyRxState = rx;
  phySetRxState();
}

/*************************************************************************//**
*****************************************************************************/
void PHY_SetChannel(uint8_t channel)
{
  // phyChannel = channel;
  // phySetChannel();
  uint8_t reg;

  reg = phyReadRegister(PHY_CC_CCA_REG) & ~0x1f;
  phyWriteRegister(PHY_CC_CCA_REG, reg | channel);
}

/*************************************************************************//**
*****************************************************************************/
// void PHY_SetBand(uint8_t band)
// {
//   phyBand = band;
//   phySetChannel();
// }

/*************************************************************************//**
*****************************************************************************/
void PHY_SetPanId(uint16_t panId)
{
  // uint8_t *d = (uint8_t *)&panId;

  // PAN_ID_0_REG = d[0];
  // PAN_ID_1_REG = d[1];

  uint8_t *d = (uint8_t *)&panId;

  phyWriteRegister(PAN_ID_0_REG, d[0]);
  phyWriteRegister(PAN_ID_1_REG, d[1]);
}

/*************************************************************************//**
*****************************************************************************/
void PHY_SetShortAddr(uint16_t addr)
{
//   uint8_t *d = (uint8_t *)&addr;

//   SHORT_ADDR_0_REG = d[0];
//   SHORT_ADDR_1_REG = d[1];

// #ifndef PHY_ENABLE_RANDOM_NUMBER_GENERATOR
//   CSMA_SEED_0_REG = d[0] + d[1];
// #endif

  uint8_t *d = (uint8_t *)&addr;

  phyWriteRegister(SHORT_ADDR_0_REG, d[0]);
  phyWriteRegister(SHORT_ADDR_1_REG, d[1]);
  phyWriteRegister(CSMA_SEED_0_REG, d[0] + d[1]);
}

/*************************************************************************//**
*****************************************************************************/
void PHY_SetTxPower(uint8_t txPower)
{
  // PHY_TX_PWR_REG_s.txPwr = txPower;
  uint8_t reg;

  reg = phyReadRegister(PHY_TX_PWR_REG) & ~0x0f;
  phyWriteRegister(PHY_TX_PWR_REG, reg | txPower);
}

/*************************************************************************//**
*****************************************************************************/
void PHY_Sleep(void)
{
  // phyTrxSetState(TRX_CMD_TRX_OFF);
  // TRXPR_REG_s.slptr = 1;
  // phyState = PHY_STATE_SLEEP;

  phyTrxSetState(TRX_CMD_TRX_OFF);
  HAL_PhySlpTrSet();
  phyState = PHY_STATE_SLEEP;
}

/*************************************************************************//**
*****************************************************************************/
void PHY_Wakeup(void)
{
  // TRXPR_REG_s.slptr = 0;
  // phySetRxState();
  // phyState = PHY_STATE_IDLE;
  HAL_PhySlpTrClear();
  phySetRxState();
  phyState = PHY_STATE_IDLE;
}

/*************************************************************************//**
*****************************************************************************/
void PHY_DataReq(uint8_t *data, uint8_t size)
{
  // phyTrxSetState(TRX_CMD_TX_ARET_ON);

  // IRQ_STATUS_REG = IRQ_CLEAR_VALUE;

  // TRX_FRAME_BUFFER(0) = size + PHY_CRC_SIZE;
  // for (uint8_t i = 0; i < size; i++)
  //   TRX_FRAME_BUFFER(i+1) = data[i];

  // phyState = PHY_STATE_TX_WAIT_END;
  // TRX_STATE_REG = TRX_CMD_TX_START;
  phyTrxSetState(TRX_CMD_TX_ARET_ON);

  phyReadRegister(IRQ_STATUS_REG);

  HAL_PhySpiSelect();
  HAL_PhySpiWriteByte(RF_CMD_FRAME_W);
  HAL_PhySpiWriteByte(size + PHY_CRC_SIZE);
  for (uint8_t i = 0; i < size; i++)
    HAL_PhySpiWriteByte(data[i]);
  HAL_PhySpiDeselect();

  phyState = PHY_STATE_TX_WAIT_END;
  HAL_PhySlpTrSet();
  HAL_PhySlpTrClear();
}

#ifdef PHY_ENABLE_RANDOM_NUMBER_GENERATOR
/*************************************************************************//**
*****************************************************************************/
uint16_t PHY_RandomReq(void)
{
  // uint16_t rnd = 0;

  // TRX_RPC_REG = 0x00;

  // phyTrxSetState(TRX_CMD_RX_ON);

  // for (uint8_t i = 0; i < 16; i += 2)
  // {
  //   HAL_Delay(RANDOM_NUMBER_UPDATE_INTERVAL);
  //   rnd |= PHY_RSSI_REG_s.rndValue << i;
  // }

  // phyTrxSetState(TRX_CMD_TRX_OFF);

  // TRX_RPC_REG = TRX_RPC_REG_VALUE;

  // phySetRxState();

  // return rnd;

  uint16_t rnd = 0;
  uint8_t rndValue;

  phyTrxSetState(TRX_CMD_RX_ON);

  for (uint8_t i = 0; i < 16; i += 2)
  {
    HAL_Delay(RANDOM_NUMBER_UPDATE_INTERVAL);
    rndValue = (phyReadRegister(PHY_RSSI_REG) >> RND_VALUE) & 3;
    rnd |= rndValue << i;
  }

  phySetRxState();

  return rnd;
}
#endif

#ifdef PHY_ENABLE_AES_MODULE
/*************************************************************************//**
*****************************************************************************/
void PHY_EncryptReq(uint8_t *text, uint8_t *key)
{
  // for (uint8_t i = 0; i < AES_BLOCK_SIZE; i++)
  //   AES_KEY = key[i];

  // AES_CTRL = (0<<AES_CTRL_DIR) | (0<<AES_CTRL_MODE);

  // for (uint8_t i = 0; i < AES_BLOCK_SIZE; i++)
  //   AES_STATE = text[i];

  // AES_CTRL |= (1<<AES_CTRL_REQUEST);

  // while (0 == (AES_STATUS & (1<<AES_STATUS_RY)));

  // for (uint8_t i = 0; i < AES_BLOCK_SIZE; i++)
  //   text[i] = AES_STATE;

  HAL_PhySpiSelect();
  HAL_PhySpiWriteByte(RF_CMD_SRAM_W);
  HAL_PhySpiWriteByte(AES_CTRL_REG);
  HAL_PhySpiWriteByte((1<<AES_CTRL_MODE) | (0<<AES_CTRL_DIR));
  for (uint8_t i = 0; i < AES_BLOCK_SIZE; i++)
    HAL_PhySpiWriteByte(key[i]);
  HAL_PhySpiDeselect();

  HAL_PhySpiSelect();
  HAL_PhySpiWriteByte(RF_CMD_SRAM_W);
  HAL_PhySpiWriteByte(AES_CTRL_REG);
  HAL_PhySpiWriteByte((0<<AES_CTRL_MODE) | (0<<AES_CTRL_DIR));
  for (uint8_t i = 0; i < AES_BLOCK_SIZE; i++)
    HAL_PhySpiWriteByte(text[i]);
  HAL_PhySpiWriteByte((1<<AES_CTRL_REQUEST) | (0<<AES_CTRL_MODE) | (0<<AES_CTRL_DIR));
  HAL_PhySpiDeselect();

  HAL_Delay(AES_CORE_CYCLE_TIME);

  HAL_PhySpiSelect();
  HAL_PhySpiWriteByte(RF_CMD_SRAM_R);
  HAL_PhySpiWriteByte(AES_STATE_REG);
  for (uint8_t i = 0; i < AES_BLOCK_SIZE; i++)
    text[i] = HAL_PhySpiWriteByte(0);
  HAL_PhySpiDeselect();
}
#endif

#ifdef PHY_ENABLE_ENERGY_DETECTION
/*************************************************************************//**
*****************************************************************************/
int8_t PHY_EdReq(void)
{
  // int8_t ed;

  // phyTrxSetState(TRX_CMD_RX_ON);

  // IRQ_STATUS_REG_s.ccaEdDone = 1;
  // PHY_ED_LEVEL_REG = 0;
  // while (0 == IRQ_STATUS_REG_s.ccaEdDone);

  // ed = (int8_t)PHY_ED_LEVEL_REG + PHY_RSSI_BASE_VAL;

  // phySetRxState();

  // return ed;

  uint8_t ed;

  phyTrxSetState(TRX_CMD_RX_ON);
  phyWriteRegister(PHY_ED_LEVEL_REG, 0);

  while (0 == (phyReadRegister(IRQ_STATUS_REG) & (1<<CCA_ED_DONE)));

  ed = (int8_t)phyReadRegister(PHY_ED_LEVEL_REG);

  phySetRxState();

  return ed + PHY_RSSI_BASE_VAL;
}
#endif

/*************************************************************************//**
*****************************************************************************/
static void phyWriteRegister(uint8_t reg, uint8_t value)
{
  HAL_PhySpiSelect();
  HAL_PhySpiWriteByteInline(RF_CMD_REG_W | reg);
  HAL_PhySpiWriteByteInline(value);
  HAL_PhySpiDeselect();
}

/*************************************************************************//**
*****************************************************************************/
static uint8_t phyReadRegister(uint8_t reg)
{
  uint8_t value;

  HAL_PhySpiSelect();
  HAL_PhySpiWriteByteInline(RF_CMD_REG_R | reg);
  value = HAL_PhySpiWriteByteInline(0);
  HAL_PhySpiDeselect();

  return value;
}

/*************************************************************************//**
*****************************************************************************/
static void phyWaitState(uint8_t state)
{
  while (state != (phyReadRegister(TRX_STATUS_REG) & TRX_STATUS_MASK));
}

/*************************************************************************//**
*****************************************************************************/
// static void phySetChannel(void)
// {
//   CC_CTRL_1_REG_s.ccBand = phyBand;

//   if (phyBand)
//     CC_CTRL_0_REG = phyChannel;
//   else
//     PHY_CC_CCA_REG_s.channel = phyChannel;
// }

/*************************************************************************//**
*****************************************************************************/
static void phySetRxState(void)
{
  // phyTrxSetState(TRX_CMD_TRX_OFF);

  // IRQ_STATUS_REG = IRQ_CLEAR_VALUE;

  // if (phyRxState)
  //   phyTrxSetState(TRX_CMD_RX_AACK_ON);
  phyTrxSetState(TRX_CMD_TRX_OFF);

  phyReadRegister(IRQ_STATUS_REG);

  if (phyRxState)
    phyTrxSetState(TRX_CMD_RX_AACK_ON);
}

/*************************************************************************//**
*****************************************************************************/
static void phyTrxSetState(uint8_t state)
{
  // TRX_STATE_REG = TRX_CMD_FORCE_TRX_OFF;
  // while (TRX_STATUS_TRX_OFF != TRX_STATUS_REG_s.trxStatus);

  // TRX_STATE_REG = state;
  // while (state != TRX_STATUS_REG_s.trxStatus);
  phyWriteRegister(TRX_STATE_REG, TRX_CMD_FORCE_TRX_OFF);
  phyWaitState(TRX_STATUS_TRX_OFF);

  phyWriteRegister(TRX_STATE_REG, state);
  phyWaitState(state);
}

/*************************************************************************//**
*****************************************************************************/
void PHY_TaskHandler(void)
{
  // if (PHY_STATE_SLEEP == phyState)
  //   return;

  // if (IRQ_STATUS_REG_s.rxEnd)
  // {
  //   PHY_DataInd_t ind;
  //   uint8_t size = TST_RX_LENGTH_REG;

  //   for (uint8_t i = 0; i < size + 1/*lqi*/; i++)
  //     phyRxBuffer[i] = TRX_FRAME_BUFFER(i);

  //   ind.data = phyRxBuffer;
  //   ind.size = size - PHY_CRC_SIZE;
  //   ind.lqi  = phyRxBuffer[size];
  //   ind.rssi = (int8_t)PHY_ED_LEVEL_REG + PHY_RSSI_BASE_VAL;
  //   PHY_DataInd(&ind);

  //   while (TRX_STATUS_RX_AACK_ON != TRX_STATUS_REG_s.trxStatus);

  //   IRQ_STATUS_REG_s.rxEnd = 1;
  //   TRX_CTRL_2_REG_s.rxSafeMode = 0;
  //   TRX_CTRL_2_REG_s.rxSafeMode = 1;
  // }

  // else if (IRQ_STATUS_REG_s.txEnd)
  // {
  //   if (TRX_STATUS_TX_ARET_ON == TRX_STATUS_REG_s.trxStatus)
  //   {
  //     uint8_t status = TRX_STATE_REG_s.tracStatus;

  //     if (TRAC_STATUS_SUCCESS == status)
  //       status = PHY_STATUS_SUCCESS;
  //     else if (TRAC_STATUS_CHANNEL_ACCESS_FAILURE == status)
  //       status = PHY_STATUS_CHANNEL_ACCESS_FAILURE;
  //     else if (TRAC_STATUS_NO_ACK == status)
  //       status = PHY_STATUS_NO_ACK;
  //     else
  //       status = PHY_STATUS_ERROR;

  //     phySetRxState();
  //     phyState = PHY_STATE_IDLE;

  //     PHY_DataConf(status);
  //   }

  //   IRQ_STATUS_REG_s.txEnd = 1;
  // }

  if (PHY_STATE_SLEEP == phyState)
    return;

  if (phyReadRegister(IRQ_STATUS_REG) & (1<<TRX_END))
  {
    if (PHY_STATE_IDLE == phyState)
    {
      PHY_DataInd_t ind;
      uint8_t size;
      int8_t rssi;

      rssi = (int8_t)phyReadRegister(PHY_ED_LEVEL_REG);

      HAL_PhySpiSelect();
      HAL_PhySpiWriteByte(RF_CMD_FRAME_R);
      size = HAL_PhySpiWriteByte(0);
      for (uint8_t i = 0; i < size + 1/*lqi*/; i++)
        phyRxBuffer[i] = HAL_PhySpiWriteByte(0);
      HAL_PhySpiDeselect();

      ind.data = phyRxBuffer;
      ind.size = size - PHY_CRC_SIZE;
      ind.lqi  = phyRxBuffer[size];
      ind.rssi = rssi + PHY_RSSI_BASE_VAL;
      PHY_DataInd(&ind);

      phyWaitState(TRX_STATUS_RX_AACK_ON);
    }

    else if (PHY_STATE_TX_WAIT_END == phyState)
    {
      uint8_t status = (phyReadRegister(TRX_STATE_REG) >> TRAC_STATUS) & 7;

      if (TRAC_STATUS_SUCCESS == status)
        status = PHY_STATUS_SUCCESS;
      else if (TRAC_STATUS_CHANNEL_ACCESS_FAILURE == status)
        status = PHY_STATUS_CHANNEL_ACCESS_FAILURE;
      else if (TRAC_STATUS_NO_ACK == status)
        status = PHY_STATUS_NO_ACK;
      else
        status = PHY_STATUS_ERROR;

      phySetRxState();
      phyState = PHY_STATE_IDLE;

      PHY_DataConf(status);
    }
  }
}

#endif // PHY_AT86RF233
