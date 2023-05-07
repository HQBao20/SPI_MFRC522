/******************************************************************************/
/*                              INCLUDE FILES                                 */
/******************************************************************************/
#include "stm32f10x.h"
#include "spi.h"

/******************************************************************************/
/*                     EXPORTED TYPES and DEFINITIONS                         */
/******************************************************************************/

/******************************************************************************/
/*                              PRIVATE DATA                                  */
/******************************************************************************/

/******************************************************************************/
/*                              EXPORTED DATA                                 */
/******************************************************************************/

/******************************************************************************/
/*                            PRIVATE FUNCTIONS                               */
/******************************************************************************/

/******************************************************************************/
/*                            EXPORTED FUNCTIONS                              */
/******************************************************************************/

/******************************************************************************/
/**
 * @brief Initialize SPI
 * 
 */
void spiInit(void)
{
    GPIO_InitTypeDef gpioInit;
    SPI_InitTypeDef spi2Init;

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_SPI2, ENABLE);

    /* PB12 - CS */
    gpioInit.GPIO_Mode = GPIO_Mode_Out_PP;
    gpioInit.GPIO_Pin = GPIO_Pin_12;
    gpioInit.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOB, &gpioInit);

    /* PB13 - SCK */
    gpioInit.GPIO_Mode = GPIO_Mode_AF_PP;
    gpioInit.GPIO_Pin = GPIO_Pin_13;
    gpioInit.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOB, &gpioInit);

    /* PB14 - MISO */
    gpioInit.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    gpioInit.GPIO_Pin = GPIO_Pin_14;
    gpioInit.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOB, &gpioInit);

    /* PB15 - MOSI */
    gpioInit.GPIO_Mode = GPIO_Mode_AF_PP;
    gpioInit.GPIO_Pin = GPIO_Pin_15;
    gpioInit.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOB, &gpioInit);

    /* SPI2 */
    spi2Init.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_32;
    spi2Init.SPI_CPHA = SPI_CPHA_1Edge;
    spi2Init.SPI_CPOL = SPI_CPOL_Low;
    spi2Init.SPI_DataSize = SPI_DataSize_8b;
    spi2Init.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
    spi2Init.SPI_FirstBit = SPI_FirstBit_MSB;
    spi2Init.SPI_Mode = SPI_Mode_Master;
    spi2Init.SPI_NSS = SPI_NSS_Soft;
    SPI_Init(SPI2, &spi2Init);
    SPI_Cmd(SPI2, ENABLE);
}

/**
 * @brief  Send data
 * @param  [uint8_t] : byData
 * @retval None
 */
uint8_t spiSenData(uint8_t byData)
{
    SPI_I2S_SendData(SPI2, byData);
    while (SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_BSY) == SET)
    {

    }

    return SPI_I2S_ReceiveData(SPI2);
}
