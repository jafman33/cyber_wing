#include <SPI.h>
#include "drv_spi.h"


#include "drv_canfdspi_api.h"
// Receive Channels
#define APP_RX_FIFO CAN_FIFO_CH1
const int test_pin = 23;
int counter = 0;
uint32_t t0;

SPISettings settingsA(15000000, MSBFIRST, SPI_MODE0);

int8_t DRV_SPI_TransferData(uint8_t spiSlaveDeviceIndex,
                            uint8_t *SpiTxData,
                            uint8_t *SpiRxData,
                            uint16_t spiTransferSize)
{
  uint8_t i;
      
  SPI.beginTransaction(settingsA);
  digitalWriteFast(CANFD_PIN_SS,LOW);
  
  for(i=0;i<spiTransferSize;i++)
  {
      SpiRxData[i] = SPI.transfer(SpiTxData[i]); 
  }
 
  digitalWriteFast(CANFD_PIN_SS,HIGH);
  SPI.endTransaction();

  return 0;
}

CAN_RX_FIFO_EVENT rxFlags;
extern CAN_RX_MSGOBJ rxObj;
extern uint8_t rxd[MAX_DATA_BYTES];

