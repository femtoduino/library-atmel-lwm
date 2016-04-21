#include "halPhy.h"

uint8_t HAL_PhySpiWriteByte(uint8_t value)
{
  return HAL_PhySpiWriteByteInline(value);
}

void HAL_PhyReset(void)
{
  // HAL_GPIO_PHY_RST_clr();
  digitalWrite(PIN_SPI_RESET, LOW);
  HAL_Delay(10);
  // HAL_GPIO_PHY_RST_set();
  digitalWrite(PIN_SPI_RESET, HIGH);
}

uint8_t HAL_PhySpiWriteByteInline(uint8_t value)
{
  SERCOM4->SPI.DATA.reg = value;
  while (!SERCOM4->SPI.INTFLAG.bit.RXC);
  return SERCOM4->SPI.DATA.reg;
  // return SPI.transfer(value);

}

/*************************************************************************//**
*****************************************************************************/
void HAL_PhySpiSelect(void)
{
  // HAL_GPIO_PHY_CS_clr();
  digitalWrite(PIN_SPI_SS, LOW);
}

/*************************************************************************//**
*****************************************************************************/
void HAL_PhySpiDeselect(void)
{
  // HAL_GPIO_PHY_CS_set();
  digitalWrite(PIN_SPI_SS, HIGH);
}

/*************************************************************************//**
*****************************************************************************/
void HAL_PhySlpTrSet(void)
{
  // HAL_GPIO_PHY_SLP_TR_set();
  digitalWrite(PIN_SPI_SLEEP_TR, HIGH);
}

/*************************************************************************//**
*****************************************************************************/
void HAL_PhySlpTrClear(void)
{
  // HAL_GPIO_PHY_SLP_TR_clr();
  digitalWrite(PIN_SPI_SLEEP_TR, LOW);
}