#include "Elmo.h"

namespace Cyberpod
{
  Elmo::Elmo(FlexCAN &port,  const uint16_t &IDX, const uint32_t &receiveTimeout):
  IDX_(IDX),
  receiveTimeout_(receiveTimeout),
  baud_(ELMO_BAUD),
  status_(Elmo::STATUS::OFF),
  port_(port),
  msgOut_(),
  msgIn_()
  {}

  int32_t Elmo::init(void)
  {
    port_.begin(1000000);

    msgOut_.ext = 0;
    msgOut_.len = 8;

    //check to see if it's on
    num16_t n16;
    n16.ui = static_cast<uint16_t>(0x6041);
    msgOut_.ext = 0;
    msgOut_.id = 0x600 + IDX_;
    msgOut_.len = 8;
    msgOut_.buf[0] = 0x48;
    msgOut_.buf[1] = n16.c[0];
    msgOut_.buf[2] = n16.c[1];
    msgOut_.buf[3] = 0;
    msgOut_.buf[4] = 0;
    msgOut_.buf[5] = 0;
    msgOut_.buf[6] = 0;
    msgOut_.buf[7] = 0;

    port_.write(msgOut_);
    uint32_t t0 = micros();

    while (micros()-t0 < receiveTimeout_) {
      while (port_.available())
        {
          port_.read(msgIn_);
          if (msgIn_.id == IDX_ + 0x580) {
            status_ = Elmo::STATUS::INIT;
            Serial.println("Elmo is turned on, first finding max current");
            break;
          }
        }
        if (status_ == Elmo::STATUS::INIT) {
          break;
        }
    }
    if (status_ == Elmo::STATUS::INIT) {
      //get max current
      getMaxCurrent();

      //turn off motor
      stopMotor();
    }
    if (status_ == Elmo::STATUS::MOTOR_OFF) {
      return 1;
    }
    if (status_ == Elmo::STATUS::OFF) {
      // Serial.println("Elmo is not turned on");
      return 0;
    }
    if (status_ == Elmo::STATUS::INIT) {
      Serial.println("Something went wrong when turning the motor on, still in INIT");
      return -1;
    }
    Serial.println("Should never be reached");
    return -2;
  }

  int32_t Elmo::startMotor(void)
  {
    if (status_ != Elmo::STATUS::MOTOR_OFF) {
      Serial.print("Need to be in Motor off for this to work!");
      return -1;
    }
    Serial.println("Now we are turning on the motor");
    num16_t n16;
    n16.ui = static_cast<uint16_t>(0x6040);
    msgOut_.ext = 0;
    msgOut_.id = 0x600 + IDX_;
    msgOut_.len = 8;
    msgOut_.buf[0] = 0x2B;
    msgOut_.buf[1] = n16.c[0];
    msgOut_.buf[2] = n16.c[1];
    msgOut_.buf[3] = 0;
    msgOut_.buf[4] = 15;
    msgOut_.buf[5] = 0;
    msgOut_.buf[6] = 0;
    msgOut_.buf[7] = 0;
    port_.write(msgOut_);
    uint32_t t0 = micros();
    while (micros()-t0 < receiveTimeout_) {
      while (port_.available())
        {
          port_.read(msgIn_);
          if (msgIn_.id == IDX_ + 0x580) {
            status_ = Elmo::STATUS::MOTOR_ON;
            Serial.println("Motor is turned on!");
            return 1;
          }
        }
    }
    Serial.println("Did not get a response. Setting state to OFF");
    status_ = Elmo::STATUS::OFF;
    return 0;
  }

  int32_t Elmo::sendCurrent(float amps)
  {
    if (status_ != Elmo::STATUS::MOTOR_ON) {
      // Serial.println("Motor is not turned on! exiting");
      return -1;
    }
    num16_t n16;
    num32_t n32;
    n16.ui = static_cast<uint16_t>(0x6071);

    msgOut_.id = 0x600 + IDX_;
    msgOut_.len = 8;
    msgOut_.buf[0] = 0x2B;
    msgOut_.buf[1] = n16.c[0];
    msgOut_.buf[2] = n16.c[1];
    float currentVal = amps*1000.0*1000.0/(float)maxCurrent_;
    n16.i = round(currentVal);
    if ((n16.i > (int) maxCurrent_)){
      n16.i = (int) maxCurrent_;
    }
    if ((n16.i< -(int) maxCurrent_)){
      n16.i = -(int) maxCurrent_;
    }
    msgOut_.buf[3] = 0;
    msgOut_.buf[4] = n16.c[0];
    msgOut_.buf[5] = n16.c[1];
    msgOut_.buf[6] = 0;
    msgOut_.buf[7] = 0;
    port_.write(msgOut_);
    uint32_t t0 = micros();
    while (micros()-t0 < receiveTimeout_) {
      while (port_.available())
        {
          port_.read(msgIn_);
          if (msgIn_.id == IDX_ + 0x580) {
            return 1;
          }
        }
    }
    status_ = Elmo::STATUS::OFF;
    return 0;
  }

  int32_t Elmo::getMaxCurrent(void)
  {
    num16_t n16;
    n16.ui = static_cast<uint16_t>(0x6075);
    msgOut_.ext = 0;
    msgOut_.id = 0x600 + IDX_;
    msgOut_.len = 8;
    msgOut_.buf[0] = 0x40;
    msgOut_.buf[1] = n16.c[0];
    msgOut_.buf[2] = n16.c[1];
    msgOut_.buf[3] = 0;
    msgOut_.buf[4] = 0;
    msgOut_.buf[5] = 0;
    msgOut_.buf[6] = 0;
    msgOut_.buf[7] = 0;
    port_.write(msgOut_);

    uint32_t t0 = micros();
    while (micros()-t0 < receiveTimeout_) {
      while (port_.available())
        {
          port_.read(msgIn_);
          if (msgIn_.id == IDX_ + 0x580) {
            num32_t n32;
            n32.c[0] = msgIn_.buf[4];
            n32.c[1] = msgIn_.buf[5];
            n32.c[2] = msgIn_.buf[6];
            n32.c[3] = msgIn_.buf[7];
            maxCurrent_ = n32.ui;
            Serial.print("The max current for this motor is (in mA): ");
            Serial.println(n32.ui);
            return 1;
          }
        }
    }
    return 0;
  }

  int32_t Elmo::stopMotor(void)
  {
    Serial.println("Now, we are shutting down the motor");
    num16_t n16;

    n16.ui = static_cast<uint16_t>(0x6040);
    msgOut_.ext = 0;
    msgOut_.id = 0x600 + IDX_;
    msgOut_.len = 8;
    msgOut_.buf[0] = 0x2B;
    msgOut_.buf[1] = n16.c[0];
    msgOut_.buf[2] = n16.c[1];
    msgOut_.buf[3] = 0;
    msgOut_.buf[4] = 6;
    msgOut_.buf[5] = 0;
    msgOut_.buf[6] = 0;
    msgOut_.buf[7] = 0;
    port_.write(msgOut_);
    uint32_t t0 = micros();
    while (micros()-t0 < receiveTimeout_) {
      while (port_.available())
        {
          port_.read(msgIn_);
          if (msgIn_.id == IDX_ + 0x580) {
            status_ = Elmo::STATUS::MOTOR_OFF;
            Serial.println("Motor is shutdown and ready to be turned on");
            return 1;
          }
        }
    }
    return 0;
  }
}
