#include "LPMS_CU2.h"

namespace Cyberpod
{

	LPMS_CU2::LPMS_CU2(const IMUAbstract::MODE &mode,
		                 FlexCAN &port,
		                 const uint32_t &baud,
		                 const uint32_t &readTimeout,
		                 const uint32_t &receiveTimeout,
		                 const uint32_t &ackTimeout,
		                 const uint32_t &resyncTimeout):
	IMUAbstract(mode,receiveTimeout),
	port_(port),
	baud_(baud),
	readTimeout_(readTimeout),
	ackTimeout_(ackTimeout),
	resyncTimeout_(resyncTimeout),
	config_(),
	messageLength_((mode==MODE::RAW)?config_.rawMessageLength:config_.fusionedMessageLength)
	{}

	LPMS_CU2::~LPMS_CU2()
	{}

	int32_t LPMS_CU2::init(bool resetSettings)
	{
		Serial.println("LPMS_CU2 - Initializing...");

		memcpy(quatOffset_,config_.quatOffset0,16);

		port_.begin(baud_); 
		msgOut_.ext = 0;
		msgOut_.id = 0x515;
		msgOut_.len = 8;

		if(stopStream()!=1)
		{
			Serial.println("LPMS_CU2 - init - stopStream failed");
			status_ = STATUS::FAILURE;
			return -1;
		}
		Serial.println("LPMS_CU2 - init - Stream Stopped");

		if(setStreamFreq()!=1)
		{
			Serial.println("LPMS_CU2 - init - setStreamFreq failed");
			status_ = STATUS::FAILURE;
			return -1;
		}
		Serial.println("LPMS_CU2 - init - Stream frequency set");

		if(setCanChannelMode()!=1)
		{
			Serial.println("LPMS_CU2 - init - setCanChannelMode failed");
			status_ = STATUS::FAILURE;
			return -1;
		}
		Serial.println("LPMS_CU2 - init - CAN channel mode set");


		if(setCanPointMode()!=1)
		{
			Serial.println("LPMS_CU2 - init - setCanPointMode failed");
			status_ = STATUS::FAILURE;
			return -1;
		}
		Serial.println("LPMS_CU2 - init - CAN point mode set");

		if(setTransmitData()!=1)
		{
			Serial.println("LPMS_CU2 - init - setTransmitData failed");
			status_ = STATUS::FAILURE;
			return -1;
		}
		Serial.println("LPMS_CU2 - init - Transmit data set");

		if(setLpbusDataMode()!=1)
		{
			Serial.println("LPMS_CU2 - init - setLpbusDataMode failed");
			status_ = STATUS::FAILURE;
			return -1;
		}
		Serial.println("LPMS_CU2 - init - BUS data mode set");

		resetTimeout();
		status_ = STATUS::RUNNING;
		Serial.println("LPMS_CU2 - Initialized!");

		return 1;
	}

	int32_t LPMS_CU2::update(void)
	{
		COMMAND cmd;
		std::vector<uint8_t> data;
		int32_t rtCode;
		uint32_t msgCounter = 0;

		while(newStreamCheck())
		{
			rtCode = readMessage(cmd,data);
			if(rtCode!=1)
			{
				if(rtCode==-2)
				{
					if(reSync()==1)
					{
						return 2;
					}
					else
					{
						Serial.println("LPMS_CU2 - update - resync failed");
						status_ = STATUS::FAILURE;
						return -2;
					}
				}
				else
				{
					Serial.println("LPMS_CU2 - update - readMessage failed");
					return -1;
				}
			}

			if(cmd!=COMMAND::GET_SENSOR_DATA)
			{
				Serial.print("LPMS_CU2 - update - unkown command: ");
				Serial.println(static_cast<uint16_t>(cmd));
				return -3;
			}

			rtCode = parseStream(data);
			if(rtCode!=1)
			{
				Serial.println("LPMS_CU2 - update - parse failed");
				status_ = STATUS::FAILURE;
				return -5;
			}

			resetTimeout();
			msgCounter++;
			dataCounter_++;
			status_ = STATUS::RUNNING;
		}

		if(msgCounter == 0 &&
			 (micros()-tnm1Received_)>receiveTimeout_)
		{
			Serial.println("LPMS_CU2 - update - timed out");
			Serial.print("msgCounter: ");
			Serial.println(msgCounter);
			Serial.print("dt: ");
			Serial.println((micros()-tnm1Received_));
			Serial.print("dataCounter_: ");
			Serial.println(dataCounter_);
			Serial.print("data available: ");
			Serial.println(static_cast<int32_t>(port_.available()));
			status_ = STATUS::TIMEDOUT;
			return -2;
		}
		else if(msgCounter==0)
		{
			return 2;
		}
		else if(msgCounter==1)
		{
			return 1;
		}
		else if(msgCounter>1)
		{
			Serial.print("LPMS_CU2 - update - overrun of: ");
			Serial.print(msgCounter);
			Serial.println("msgs");

			return 1;
		}
	}

	int32_t LPMS_CU2::startStream(void)
	{	
		if(gotoStreamMode()!=1)
		{
			Serial.println("LPMS_CU2 couldn't start stream");
			status_ = STATUS::FAILURE;
			return -1;
		}
		return 1;
	}
	
	int32_t LPMS_CU2::stopStream(void)
	{

		if(gotoCommandMode()!=1)
		{
			Serial.println("LPMS_CU2 couldn't got to command mode");
			return -1;
		}

		delay(10);
		uint32_t t0 = micros();
		while(newMsgCheck())
		{
			if(micros()-t0>ackTimeout_)
			{
				Serial.println("LPMS_CU2 couldn't stop stream timedout");
				return -2;
			}
			port_.read(msgIn_);
		}
		return 1;
	}

	bool LPMS_CU2::newMsgCheck(void)
	{
		if(port_.available()>0)
			return true;
		else
			return false;
	}

	bool LPMS_CU2::newStreamCheck(void)
	{
		if(port_.available()>=messageLength_)
			return true;
		else
			return false;
	}

	int32_t LPMS_CU2::readMessage(LPMS_CU2::COMMAND &cmd,
		                              std::vector<uint8_t> &data)
	{
		int32_t rtCode = port_.read(msgIn_);

		if(rtCode != 1)
		{
			Serial.println("LPMS_CU2 Nothing to read");
			return -1;
		}

		uint8_t SOP = msgIn_.buf[0];
		num16_t n16;
		uint16_t dataLen;
		uint16_t checkSum = 0;
		uint16_t cmdUi;

		if(SOP==0x3A)
		{
			// Reading header
			uint8_t *header = msgIn_.buf+1;

			// Checking openMATID
			n16.c[0] = header[0];
			n16.c[1] = header[1];
			if(n16.ui!=config_.openMATID)
			{
				Serial.println("LPMS_CU2 wrong openMATID received");
				return -4;
			}

			// Checking Command
			n16.c[0] = header[2];
			n16.c[1] = header[3];
			cmdUi = n16.ui;
			if(!isValidCommand(cmdUi))
			{
				Serial.println("LPMS_CU2 unkown command received");
				return -5;
			}
			cmd = static_cast<COMMAND>(cmdUi);

			// Checking packet length
			n16.c[0] = header[4];
			n16.c[1] = header[5];
			dataLen = n16.ui;

			uint32_t msgLeft = (dataLen+11)/8;
			uint32_t lastMessageLen = (dataLen+11)%8;
			if(lastMessageLen > 0)
				msgLeft++;
			msgLeft --;

			uint8_t dataTemp[dataLen+4];
			dataTemp[0] = msgIn_.buf[7];

			uint32_t msgReceived = 0;

			uint32_t tNow = micros();
			while(true)
			{
				if(newMsgCheck())
				{
					rtCode = port_.read(msgIn_);
					if(rtCode != 1)
					{
						Serial.println("LPMS_CU2 Nothing to read");
						return -1;
					}

					msgReceived++;

					if(msgReceived==msgLeft)
					{
						memcpy(dataTemp+1+8*(msgReceived-1),msgIn_.buf,lastMessageLen);
						break;
					}
					else
					{
						memcpy(dataTemp+1+8*(msgReceived-1),msgIn_.buf,8);
					}
				}

				if((micros()-tNow)>readTimeout_)
				{
					Serial.println("LPMS_CU2 read timeout");
					return -3;				
				}
			}

			if(!(dataTemp[dataLen+2]==0x0D && dataTemp[dataLen+3]==0x0A))
			{
				Serial.println("LPMS_CU2 wrong packet size");
				Serial.println("LPMS_CU2 now out of sync");
				return -2;
			}

			// Checking checksum
			n16.c[0] = dataTemp[dataLen];
			n16.c[1] = dataTemp[dataLen+1];

			checkSum = config_.openMATID + cmdUi + dataLen;
			for(uint16_t i = 0; i<dataLen; i++)
			{
				checkSum+=dataTemp[i];
			}

			if(checkSum!=n16.ui)
			{
				Serial.println("LPMS_CU2 checksum failed");
				return -6;
			}

			//Reading packet
			if(dataLen>0)
			{
				data.clear();
				data.insert(data.begin(),dataTemp,dataTemp + dataLen);
			}		
			return 1;
		}
		else
		{
			Serial.println("LPMS_CU2 - readMessage - SOP failed");
			return -2;
		}
	}

	int32_t LPMS_CU2::sendMessage(const LPMS_CU2::COMMAND &cmd,
						                      const uint16_t &dataLen,
												          const uint8_t data[])
	{
		uint32_t msgLen = dataLen+11;
		uint8_t msg[msgLen] = {0x00};
		uint16_t cmdUi = static_cast<uint16_t>(cmd);
		num16_t n16;

		//Fill Packet with header
		msg[0] = 0x3A;

		n16.ui = config_.openMATID;
		msg[1] = n16.c[0];
		msg[2] = n16.c[1];

		n16.ui = cmdUi;
		msg[3] = n16.c[0];
		msg[4] = n16.c[1];

		n16.ui = dataLen;
		msg[5] = n16.c[0];
		msg[6] = n16.c[1];

		//Copy data
		memcpy(msg+7,data,dataLen);	

		uint16_t checkSum = config_.openMATID + cmdUi + dataLen;
		for(uint32_t i = 0; i<dataLen; i++)
		{
			checkSum+=data[i];
		}
		n16.ui = checkSum;

		msg[msgLen-4] = n16.c[0];
		msg[msgLen-3] = n16.c[1];
		msg[msgLen-2] = 0x0D;
		msg[msgLen-1] = 0x0A;

		uint32_t msgToSend = msgLen/8;
		uint32_t lastMessageLen = msgLen%8;

		if(lastMessageLen > 0)
			msgToSend++;


		for(uint32_t i = 0; i<msgToSend-1; i++)
		{
			memcpy(msgOut_.buf,msg + 8*i,sizeof(msgOut_.buf));
			port_.write(msgOut_);
		}

		if(lastMessageLen>0)
		{
			memset(msgOut_.buf,0x00,sizeof(msgOut_.buf));
			memcpy(msgOut_.buf,msg + 8*(msgToSend-1),lastMessageLen);
			port_.write(msgOut_);
		}

		return 1;
	}

	int32_t LPMS_CU2::gotoCommandMode(void)
	{

		COMMAND cmd = COMMAND::GOTO_COMMAND_MODE;
		uint16_t dataLen = 0;
		if(sendMessage(cmd,dataLen,nullptr)!=1)
		{
			Serial.println("LPMS_CU2 couldn't got to command mode");
			return -1;
		}
		return 1;
	}

	int32_t LPMS_CU2::gotoStreamMode(void)
	{
		COMMAND cmd = COMMAND::GOTO_STREAM_MODE;
		uint16_t dataLen = 1;
		if(sendMessage(cmd,dataLen,nullptr)!=1)
		{
			Serial.println("LPMS_CU2 couldn't got to stream mode");
			return -1;
		}
		if(checkACK()!=1)
		{
			Serial.println("LPMS_CU2 couldn't got to stream mode");
			return -1;
		}
		return 1;
	}

	int32_t LPMS_CU2::tareReset(void)
	{
		COMMAND cmd = COMMAND::RESET_ORIENTATION_OFFSET;
		uint16_t dataLen = 1;
		if(sendMessage(cmd,dataLen,nullptr)!=1)
		{
			Serial.println("LPMS_CU2 couldn't reset orientation offset");
			return -1;
		}
		if(checkACK()!=1)
		{
			Serial.println("LPMS_CU2 couldn't reset orientation offset");
			return -1;
		}
		return 1;
	}

	int32_t LPMS_CU2::setStreamFreq(void)
	{

		COMMAND cmd = COMMAND::SET_STREAM_FREQ;
		uint16_t dataLen = 4;
		num32_t n32;
		n32.i = config_.streamFreq;

		if(sendMessage(cmd,dataLen,n32.c)!=1)
		{
			Serial.println("LPMS_CU2 couldn't set stream frequency");
			return -1;
		}

		if(checkACK()!=1)
		{
			Serial.println("LPMS_CU2 couldn't set stream frequency");
			return -1;
		}
		return 1;
	}

	int32_t LPMS_CU2::setTransmitData(void)
	{

		COMMAND cmd = COMMAND::SET_TRANSMIT_DATA;
		uint16_t dataLen = 4;
		num32_t n32;
		if(mode_==MODE::RAW)
			n32.i = config_.rawStreamData;
		else if(mode_==MODE::FUSIONED)
			n32.i = config_.fusionedStreamData;
		else
		{
			Serial.println("LPMS_CU2 - setTransmitData - unkown mode");
			return -1;
		}

		if(sendMessage(cmd,dataLen,n32.c)!=1)
		{
			Serial.println("LPMS_CU2 - setTransmitData - couldn't set transmit data");
			return -1;
		}

		if(checkACK()!=1)
		{
			Serial.println("LPMS_CU2 - setTransmitData - couldn't set transmit data");
			return -1;
		}
		return 1;
	}

	int32_t LPMS_CU2::setUartFormat(void)
	{
		COMMAND cmd = COMMAND::SET_UART_FORMAT;
		uint16_t dataLen = 4;
		num32_t n32;
		n32.i = config_.uartFormat;
		if(sendMessage(cmd,dataLen,n32.c)!=1)
		{
			Serial.println("LPMS_CU2 - setUartFormat - couldn't set transmit data");
			return -1;
		}

		if(checkACK()!=1)
		{
			Serial.println("LPMS_CU2 - setUartFormat - couldn't set transmit data");
			return -1;
		}
		return 1;
	}

	int32_t LPMS_CU2::setLpbusDataMode(void)
	{
		COMMAND cmd = COMMAND::SET_LPBUS_DATA_MODE;
		uint16_t dataLen = 4;
		num32_t n32;
		n32.i = config_.busDataMode;
		if(sendMessage(cmd,dataLen,n32.c)!=1)
		{
			Serial.println("LPMS_CU2 - setLpbusDataMode - couldn't set transmit data");
			return -1;
		}

		if(checkACK()!=1)
		{
			Serial.println("LPMS_CU2 - setLpbusDataMode - couldn't set transmit data");
			return -1;
		}
		return 1;
	}

	int32_t LPMS_CU2::setCanChannelMode(void)
	{
		COMMAND cmd = COMMAND::SET_CAN_CHANNEL_MODE;
		uint16_t dataLen = 4;
		num32_t n32;
		n32.i = config_.canChannelMode;
		if(sendMessage(cmd,dataLen,n32.c)!=1)
		{
			Serial.println("LPMS_CU2 - setLpbusDataMode - couldn't set transmit data");
			return -1;
		}

		if(checkACK()!=1)
		{
			Serial.println("LPMS_CU2 - setLpbusDataMode - couldn't set transmit data");
			return -1;
		}
		return 1;
	}

	int32_t LPMS_CU2::setCanPointMode(void)
	{
		COMMAND cmd = COMMAND::SET_CAN_POINT_MODE;
		uint16_t dataLen = 4;
		num32_t n32;
		n32.i = config_.canPointMode;
		if(sendMessage(cmd,dataLen,n32.c)!=1)
		{
			Serial.println("LPMS_CU2 - setLpbusDataMode - couldn't set transmit data");
			return -1;
		}

		if(checkACK()!=1)
		{
			Serial.println("LPMS_CU2 - setLpbusDataMode - couldn't set transmit data");
			return -1;
		}
		return 1;
	}

	int32_t LPMS_CU2::reSync(void)
	{
		uint32_t t0 = micros();
		
		while(port_.available()>0)
		{
			if(micros()-t0>resyncTimeout_)
			{
				Serial.println("LPMS_CU2 - reSync - timedout");
				return -1;	
			}
			port_.read(msgIn_);
		}

		return 1;
	}

	int32_t LPMS_CU2::checkACK(void)
	{
		uint32_t t0 = micros();
		std::vector<uint8_t> data;
		int32_t rtCode;
		COMMAND cmd;

		while(micros()-t0<ackTimeout_)
		{
			if(newMsgCheck())
			{
				rtCode = readMessage(cmd,data);
				if(rtCode!=1)
				{
					Serial.println("LPMS_CU2 checkACK readMessage failed");
					return rtCode;
				}
				else if(cmd==COMMAND::REPLY_ACK)
					return 1;
				else if(cmd==COMMAND::REPLY_NACK)
					return 0;
				else
				{
					Serial.println("LPMS_CU2 checkACK unknow reply");
					return -1;
				}
			}
		}
		Serial.println("LPMS_CU2 checkACK timedout");
		return -2;
	}

	int32_t LPMS_CU2::parseStream(const std::vector<uint8_t> &data)
	{
		num32_t n32;
		if(mode_==MODE::FUSIONED)
		{
			//Check length
			if(data.size()!=32)
			{
				Serial.print("LPMS_CU2 - parseStream - wrong length");
				return -1; 
			}

			//Parse rates
			for(uint32_t i = 0; i<3; i++)
			{
				for(uint32_t j = 0; j<4; j++)
				{
					n32.c[j] = data[j+i*4+4];
				}
				fusionedData_.rates[i] = n32.f;
			}

			//Parse quat
			for(uint32_t i = 0; i<4; i++)
			{
				for(uint32_t j = 0; j<4; j++)
				{
					n32.c[j] = data[j+i*4+16];
				}
				fusionedData_.quatRaw[i] = n32.f;
			}

			fusionedData_.quatRaw[0] *= -1.0F;

			setOffset();
			computeEuler();
		}
		else if(mode_==MODE::RAW)
		{
			//Check length
			if(data.size()!=40)
			{
				Serial.print("LPMS_CU2 - parseStream - wrong length");
				return -1; 
			}

			//Parse gyro
			for(uint32_t i = 0; i<3; i++)
			{
				for(uint32_t j = 0; j<4; j++)
				{
					n32.c[j] = data[j+i*4+4];
				}
				rawData_.gyro[i] = n32.f;
			}

			//Parse acc
			for(uint32_t i = 0; i<3; i++)
			{
				for(uint32_t j = 0; j<4; j++)
				{
					n32.c[j] = data[j+i*4+16];
				}
				rawData_.acc[i] = n32.f;
			}

			//Parse mag
			for(uint32_t i = 0; i<3; i++)
			{
				for(uint32_t j = 0; j<4; j++)
				{
					n32.c[j] = data[j+i*4+28];
				}
				rawData_.mag[i] = n32.f;
			}
		}
		else
		{
			Serial.println("LPMS_CU2 - parseStream - unkown mode");
			return -2;
		}

		return 1;
	}

	bool LPMS_CU2::isValidCommand(const uint16_t &cmd)
	{
		if(cmd==0 ||
			 cmd==1 ||
			 cmd==2 ||
			 cmd==3 ||
			 cmd==4 ||
			 cmd==5 ||
			 cmd==6 ||
			 cmd==7 ||
			 cmd==9 ||
			 cmd==10 ||
			 cmd==11 ||
			 cmd==20 ||
			 cmd==21 ||
			 cmd==25 ||
			 cmd==26 ||
			 cmd==31 ||
			 cmd==32 ||
			 cmd==33 ||
			 cmd==34 ||
			 cmd==41 ||
			 cmd==42 ||
			 cmd==43 ||
			 cmd==44 ||
			 cmd==60 ||
			 cmd==61 ||
			 cmd==66 ||
			 cmd==67 ||
			 cmd==68 ||
			 cmd==75 ||
			 cmd==84 ||
			 cmd==85 ||
			 cmd==86)
			return true;
		else
			return false;
	}
}