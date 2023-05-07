/******************************************************************************/
/*                              INCLUDE FILES                                 */
/******************************************************************************/
#include "stm32f10x.h"
#include "spi.h"
#include "mfrc522.h"

/******************************************************************************/
/*                     EXPORTED TYPES and DEFINITIONS                         */
/******************************************************************************/
/* MFRC522 Commands */
#define PCD_IDLE                        0x00   //NO action; Cancel the current command
#define PCD_AUTHENT                     0x0E   //Authentication Key
#define PCD_RECEIVE                     0x08   //Receive Data
#define PCD_TRANSMIT                    0x04   //Transmit data
#define PCD_TRANSCEIVE                  0x0C   //Transmit and receive data,
#define PCD_RESETPHASE                  0x0F   //Reset
#define PCD_CALCCRC                     0x03   //CRC Calculate

/* Mifare_One card command word */
#define PICC_REQIDL                     0x26   // find the antenna area does not enter hibernation
#define PICC_REQALL                     0x52   // find all the cards antenna area
#define PICC_ANTICOLL                    0x93   // anti-collision
#define PICC_SElECTTAG                  0x93   // election card
#define PICC_AUTHENT1A                  0x60   // authentication key A
#define PICC_AUTHENT1B                  0x61   // authentication key B
#define PICC_READ                       0x30   // Read Block
#define PICC_WRITE                      0xA0   // write block
#define PICC_DECREMENT                  0xC0   // debit
#define PICC_INCREMENT                  0xC1   // recharge
#define PICC_RESTORE                    0xC2   // transfer block data to the buffer
#define PICC_TRANSFER                   0xB0   // save the data in the buffer
#define PICC_HALT                       0x50   // Sleep

/* MFRC522 Registers */
//Page 0: Command and Status
#define MFRC522_REG_RESERVED00          0x00
#define MFRC522_REG_COMMAND             0x01
#define MFRC522_REG_COMM_IE_N           0x02
#define MFRC522_REG_DIV1_EN             0x03
#define MFRC522_REG_COMM_IRQ            0x04
#define MFRC522_REG_DIV_IRQ             0x05
#define MFRC522_REG_ERROR               0x06
#define MFRC522_REG_STATUS1             0x07
#define MFRC522_REG_STATUS2             0x08
#define MFRC522_REG_FIFO_DATA           0x09
#define MFRC522_REG_FIFO_LEVEL          0x0A
#define MFRC522_REG_WATER_LEVEL         0x0B
#define MFRC522_REG_CONTROL             0x0C
#define MFRC522_REG_BIT_FRAMING         0x0D
#define MFRC522_REG_COLL                0x0E
#define MFRC522_REG_RESERVED01          0x0F
//Page 1: Command 
#define MFRC522_REG_RESERVED10          0x10
#define MFRC522_REG_MODE                0x11
#define MFRC522_REG_TX_MODE             0x12
#define MFRC522_REG_RX_MODE             0x13
#define MFRC522_REG_TX_CONTROL          0x14
#define MFRC522_REG_TX_AUTO             0x15
#define MFRC522_REG_TX_SELL             0x16
#define MFRC522_REG_RX_SELL             0x17
#define MFRC522_REG_RX_THRESHOLD        0x18
#define MFRC522_REG_DEMOD               0x19
#define MFRC522_REG_RESERVED11          0x1A
#define MFRC522_REG_RESERVED12          0x1B
#define MFRC522_REG_MIFARE              0x1C
#define MFRC522_REG_RESERVED13          0x1D
#define MFRC522_REG_RESERVED14          0x1E
#define MFRC522_REG_SERIALSPEED         0x1F
//Page 2: CFG
#define MFRC522_REG_RESERVED20          0x20
#define MFRC522_REG_CRC_RESULT_M        0x21
#define MFRC522_REG_CRC_RESULT_L        0x22
#define MFRC522_REG_RESERVED21          0x23
#define MFRC522_REG_MOD_WIDTH           0x24
#define MFRC522_REG_RESERVED22          0x25
#define MFRC522_REG_RF_CFG              0x26
#define MFRC522_REG_GS_N                0x27
#define MFRC522_REG_CWGS_PREG           0x28
#define MFRC522_REG__MODGS_PREG         0x29
#define MFRC522_REG_T_MODE              0x2A
#define MFRC522_REG_T_PRESCALER         0x2B
#define MFRC522_REG_T_RELOAD_H          0x2C
#define MFRC522_REG_T_RELOAD_L          0x2D
#define MFRC522_REG_T_COUNTER_VALUE_H   0x2E
#define MFRC522_REG_T_COUNTER_VALUE_L   0x2F
//Page 3:TestRegister
#define MFRC522_REG_RESERVED30          0x30
#define MFRC522_REG_TEST_SEL1           0x31
#define MFRC522_REG_TEST_SEL2           0x32
#define MFRC522_REG_TEST_PIN_EN         0x33
#define MFRC522_REG_TEST_PIN_VALUE      0x34
#define MFRC522_REG_TEST_BUS            0x35
#define MFRC522_REG_AUTO_TEST           0x36
#define MFRC522_REG_VERSION             0x37
#define MFRC522_REG_ANALOG_TEST         0x38
#define MFRC522_REG_TEST_ADC1           0x39
#define MFRC522_REG_TEST_ADC2           0x3A
#define MFRC522_REG_TEST_ADC0           0x3B
#define MFRC522_REG_RESERVED31          0x3C
#define MFRC522_REG_RESERVED32          0x3D
#define MFRC522_REG_RESERVED33          0x3E
#define MFRC522_REG_RESERVED34          0x3F
//Dummy byte
#define MFRC522_DUMMY                   0x00

#define MFRC522_MAX_LEN                 16

#define MFRC522_CS_ENB                  GPIO_ResetBits(GPIOB, GPIO_Pin_12)
#define MFRC522_CS_DIS                  GPIO_SetBits(GPIOB, GPIO_Pin_12)
#define RESET_ADDRESS                   0x7E
#define SET_ADDRESS                     0x80
/******************************************************************************/
/*                              PRIVATE DATA                                  */
/******************************************************************************/

/******************************************************************************/
/*                              EXPORTED DATA                                 */
/******************************************************************************/

/******************************************************************************/
/*                            PRIVATE FUNCTIONS                               */
/******************************************************************************/
static void mfrc522WriteRegister(uint8_t byAddr, uint8_t byVal);
static uint8_t mfrc522ReadRegister(uint8_t byAddr);
static void mfrc522SetBitMask(uint8_t byReg, uint8_t byMask);
static void mfrc522ClearBitMask(uint8_t byReg, uint8_t byMask);
static void mfrc522AntennaOn(void);
static void mfrc522AntennaOff(void);
static void mfrc522Reset(void);
static MFRC522Status mfrc522Request(uint8_t byReqMode, uint8_t* pbyTagType);
static MFRC522Status mfrc522ToCard(uint8_t byCommand, uint8_t* pbySendData, uint8_t bySendLen, uint8_t* pbyBackData, uint16_t* pwBackLen);
static MFRC522Status mfrc522Anticoll(uint8_t* pbySerNum);
static void mfrc522CalculateCRC(uint8_t* pbyIndata, uint8_t byLen, uint8_t* pbyOutData);
static uint8_t mfrc522SelectTag(uint8_t* pbySerNum);
static MFRC522Status mfrc522Auth(uint8_t byAuthMode, uint8_t byBlockAddr, uint8_t* pbySectorkey, uint8_t* pbySerNum);
static MFRC522Status mfrc522Read(uint8_t byBlockAddr, uint8_t* pbyRecvData);
static MFRC522Status mfrc522Write(uint8_t byBlockAddr, uint8_t* pbyWriteData);
static void mfrc522Halt(void);

/******************************************************************************/
/*                            EXPORTED FUNCTIONS                              */
/******************************************************************************/

/******************************************************************************/
/**
 * @brief 
 * 
 * @param byAddr 
 * @param byVal 
 */
static void mfrc522WriteRegister(uint8_t byAddr, uint8_t byVal)
{
    MFRC522_CS_ENB;
    /* Send address */
    spiSenData((byAddr << 1) & RESET_ADDRESS);
    /* Send data */
    spiSenData(byVal);
    MFRC522_CS_DIS;
}

/**
 * @brief 
 * 
 * @param byAddr 
 * @return uint8_t 
 */
static uint8_t mfrc522ReadRegister(uint8_t byAddr)
{
    uint8_t byVal = 0;

    MFRC522_CS_ENB;
    /* Send address */
    byVal = spiSenData(((byAddr << 1) & RESET_ADDRESS) | SET_ADDRESS);
    MFRC522_CS_DIS;

    return byVal;
}

/**
 * @brief 
 * 
 * @param byReg 
 * @param byMask 
 */
static void mfrc522SetBitMask(uint8_t byReg, uint8_t byMask)
{
    mfrc522WriteRegister(byReg, mfrc522ReadRegister(byReg) | byMask);
}

/**
 * @brief 
 * 
 * @param byReg 
 * @param byMask 
 */
static void mfrc522ClearBitMask(uint8_t byReg, uint8_t byMask)
{
    mfrc522WriteRegister(byReg, mfrc522ReadRegister(byReg) & ~(byMask));
}

/**
 * @brief 
 * 
 */
static void mfrc522Reset(void)
{
    mfrc522WriteRegister(MFRC522_REG_COMMAND, PCD_RESETPHASE);
}

/**
 * @brief 
 * 
 */
static void mfrc522AntennaOn(void)
{
    uint8_t byTemp;

    byTemp = mfrc522ReadRegister(MFRC522_REG_TX_CONTROL);
    if ((byTemp & 0x03)) {
        mfrc522SetBitMask(MFRC522_REG_TX_CONTROL, 0x03);
    }
}

/**
 * @brief 
 * 
 * @param byCommand 
 * @param pbySendData 
 * @param bySendLen 
 * @param pbyBackData 
 * @param pwBackLen 
 * @return MFRC522Status 
 */
static MFRC522Status mfrc522ToCard(uint8_t byCommand, uint8_t* pbySendData, uint8_t bySendLen, uint8_t* pbyBackData, uint16_t* pwBackLen)
{
    MFRC522Status status = MI_ERR;
    uint8_t byIrqEn = 0x00;
    uint8_t byWaitIrq = 0x00;
    uint8_t byLastBits;
    uint8_t n;
    uint16_t i;

    switch (byCommand)
    {
        case PCD_AUTHENT:
        {
            byIrqEn = 0x12;
            byWaitIrq = 0x10;
            break;
        }
        case PCD_TRANSCEIVE:
        {
            byIrqEn = 0x77;
            byWaitIrq = 0x30;
            break;
        }
        default:
            break;
    }

    mfrc522WriteRegister(MFRC522_REG_COMM_IE_N, byIrqEn | 0x80);
    mfrc522ClearBitMask(MFRC522_REG_COMM_IRQ, 0x80);
    mfrc522SetBitMask(MFRC522_REG_FIFO_LEVEL, 0x80);
    mfrc522WriteRegister(MFRC522_REG_COMMAND, PCD_IDLE);
    for (i = 0; i < bySendLen; i++)
    {
        mfrc522WriteRegister(MFRC522_REG_FIFO_DATA, pbySendData[i]);
    }
    mfrc522WriteRegister(MFRC522_REG_COMMAND, byCommand);
    if (byCommand == PCD_TRANSCEIVE)
    {
        mfrc522SetBitMask(MFRC522_REG_BIT_FRAMING, 0x80);		//StartSend=1,transmission of data starts  
    }

    //Waiting to receive data to complete
    i = 2000;	//i according to the clock frequency adjustment, the operator M1 card maximum waiting time 25ms???
    do {
        //CommIrqReg[7..0]
        //Set1 TxIRq RxIRq IdleIRq HiAlerIRq LoAlertIRq ErrIRq TimerIRq
        n = mfrc522ReadRegister(MFRC522_REG_COMM_IRQ);
        i--;
    } while ((i!=0) && !(n&0x01) && !(n&byWaitIrq));
    mfrc522ClearBitMask(MFRC522_REG_BIT_FRAMING, 0x80);

    if (i != 0)
    {
        if (!(mfrc522ReadRegister(MFRC522_REG_ERROR) & 0x1B))
        {
            status = MI_OK;
            if (n & byIrqEn & 0x01)
            {
                status = MI_NOTAGERR;
            }

            if (byCommand == PCD_TRANSCEIVE)
            {
                n = mfrc522ReadRegister(MFRC522_REG_FIFO_LEVEL);
                byLastBits = mfrc522ReadRegister(MFRC522_REG_CONTROL) & 0x07;
                if (byLastBits)
                {
                    *pwBackLen = (n - 1) * 8 + byLastBits;
                }
                else
                {
                    *pwBackLen = n * 8;
                }

                if (n == 0)
                {
                    n = 1;
                }
                if (n > MFRC522_MAX_LEN)
                {
                    n = MFRC522_MAX_LEN;
                }

                //Reading the received data in FIFO
                for (i = 0; i < n; i++)
                {
                    pbyBackData[i] = mfrc522ReadRegister(MFRC522_REG_FIFO_DATA);
                }
            }
        }
        else
        {
            status = MI_ERR;
        }
    }

    return status;
}

/**
 * @brief 
 * 
 * @param byReqMode 
 * @param pbyTagType 
 * @return MFRC522Status 
 */
static MFRC522Status mfrc522Request(uint8_t byReqMode, uint8_t* pbyTagType)
{
    MFRC522Status status;
    uint16_t wBackBits;

    mfrc522WriteRegister(MFRC522_REG_BIT_FRAMING, 0x07);
    pbyTagType[0] = byReqMode;
    status = mfrc522ToCard(PCD_TRANSCEIVE, pbyTagType, 1, pbyTagType, &wBackBits);

    if ((status != MI_OK) || (wBackBits != 0x10))
    {
        status = MI_ERR;
    }

    return status;
}

/**
 * @brief 
 * 
 * @param pbySerNum 
 * @return MFRC522Status 
 */
static MFRC522Status mfrc522Anticoll(uint8_t* pbySerNum)
{
    MFRC522Status status;
    uint8_t i = 0;
    uint8_t bySerNumCheck = 0;
    uint16_t wUnLen;

    mfrc522WriteRegister(MFRC522_REG_BIT_FRAMING, 0x00);
    pbySerNum[0] = PICC_ANTICOLL;
    pbySerNum[1] = 0x20;
    status = mfrc522ToCard(PCD_TRANSCEIVE, pbySerNum, 2, pbySerNum, &wUnLen);

    if (status == MI_OK)
    {
        //Check card serial number
        for (i = 0; i < 4; i++)
        {
            bySerNumCheck ^= pbySerNum[i];
        }
        if (bySerNumCheck != pbySerNum[i])
        {
            status = MI_ERR;
        }
    }
    return status;
}

/**
 * @brief 
 * 
 * @param pbyIndata 
 * @param byLen 
 * @param pbyOutData 
 */
static void mfrc522CalculateCRC(uint8_t* pbyIndata, uint8_t byLen, uint8_t* pbyOutData)
{
    uint8_t i, n;

    mfrc522ClearBitMask(MFRC522_REG_DIV_IRQ, 0x04);             //CRCIrq = 0
    mfrc522SetBitMask(MFRC522_REG_FIFO_LEVEL, 0x80);            //Clear the FIFO pointer
    //Write_MFRC522(CommandReg, PCD_IDLE);

    //Writing data to the FIFO	
    for (i = 0; i < byLen; i++)
    {
        mfrc522WriteRegister(MFRC522_REG_FIFO_DATA, *(pbyIndata+i));
    }
    mfrc522WriteRegister(MFRC522_REG_COMMAND, PCD_CALCCRC);

    //Wait CRC calculation is complete
    i = 0xFF;
    do
    {
        n = mfrc522ReadRegister(MFRC522_REG_DIV_IRQ);
        i--;
    } while ((i != 0) && !(n & 0x04));        //CRCIrq = 1

    //Read CRC calculation result
    pbyOutData[0] = mfrc522ReadRegister(MFRC522_REG_CRC_RESULT_L);
    pbyOutData[1] = mfrc522ReadRegister(MFRC522_REG_CRC_RESULT_M);
}

/**
 * @brief 
 * 
 */
static void mfrc522Halt(void)
{
    uint16_t wUnLen;
    uint8_t byBuff[4];

    byBuff[0] = PICC_HALT;
    byBuff[1] = 0;
    mfrc522CalculateCRC(byBuff, 2, &byBuff[2]);

    mfrc522ToCard(PCD_TRANSCEIVE, byBuff, 4, byBuff, &wUnLen);
}

/**
 * @brief Initialize mfrc522
 * 
 */
void mfrc522Init(void)
{
    mfrc522Reset();
    mfrc522WriteRegister(MFRC522_REG_T_MODE, 0x8D);
    mfrc522WriteRegister(MFRC522_REG_T_PRESCALER, 0x3E);
    mfrc522WriteRegister(MFRC522_REG_T_RELOAD_L, 30);
    mfrc522WriteRegister(MFRC522_REG_T_RELOAD_H, 0);
    /* 48dB gain */
    mfrc522WriteRegister(MFRC522_REG_RF_CFG, 0x70);
    mfrc522WriteRegister(MFRC522_REG_TX_AUTO, 0x40);
    mfrc522WriteRegister(MFRC522_REG_MODE, 0x3D);
    mfrc522AntennaOn();
}

/**
 * @brief Check for RFID card existance
 * @param [uint8_t]: *pID
 * @retval MI_OK if card is detected
 */
MFRC522Status mfrc522Check(uint8_t *pbyId)
{
    MFRC522Status status;

    status = mfrc522Request(PICC_REQIDL, pbyId);
    if (status == MI_OK)
    {
        status = mfrc522Anticoll(pbyId);
    }
    mfrc522Halt();

    return status;
}
