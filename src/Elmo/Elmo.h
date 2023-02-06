#ifndef __ELMO_H_INCLUDED__
#define __ELMO_H_INCLUDED__

#include "Arduino.h"
#include "FlexCAN.h"
#include "../../SO_Config.h"

namespace Cyberpod
{
  class Elmo
  {
    public:
      enum class STATUS : uint8_t
      {
        OFF     = 0,
        INIT     = 1,
        MOTOR_OFF = 2,
        MOTOR_ON = 3
      };
    public:
      Elmo(FlexCAN &port, const uint16_t &IDX, const uint32_t &receiveTimeout);
      int32_t init(void);
      int32_t startMotor(void);
      int32_t getMaxCurrent(void);
      int32_t sendCurrent(float amps);
      int32_t stopMotor(void);

      STATUS status_;
      uint32_t maxCurrent_;
      uint32_t baud_;
    protected:
      FlexCAN &port_;
      CAN_message_t msgIn_;
      CAN_message_t msgOut_;
      const uint32_t receiveTimeout_;
      const uint16_t IDX_;
  };

}

#endif
