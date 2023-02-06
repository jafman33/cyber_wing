#include "SO.h"

extern uint8_t txd[MAX_DATA_BYTES];
extern CAN_TX_MSGOBJ txObj;
extern CAN_RX_MSGOBJ rxObj;
extern uint8_t rxd[MAX_DATA_BYTES];
static CAN_RX_FIFO_EVENT CanRxFlags;
static uint32_t dtLLC_;
static uint32_t dtTX2_;
static uint32_t dtELMO_;

namespace Cyberpod
{
	//Constructor
	SO::SO(void):
	LEFT_ELMO_(SO_CAN_PORT,
	           ELMO_MOTOR_LEFT_IDX,
	           ELMO_RECEIVE_TIMEOUT),
	RIGHT_ELMO_(SO_CAN_PORT,
	           ELMO_MOTOR_RIGHT_IDX,
	           ELMO_RECEIVE_TIMEOUT),
	imu_(YOST_TTS_LX_MODE,
	     SPI,
	     YOST_TTS_LX_PIN_SS,
	     YOST_TTS_LX_PIN_ATT,
	     YOST_TTS_LX_RECEIVE_TIMEOUT),
	status_(STATUS::INIT),
	publishing_(true),
	batteryMonitor_(BATTERY_CURRENT_PIN,
	                BATTERY_VOLTAGE_PIN,
	                BATTERY_CURRENT_TAU,
	                BATTERY_VOLTAGE_TAU),
	steering_(STEERING_PIN,
	          STEERING_TAU),
	encL_(ENCODER_PIN_A_LEFT,
	      ENCODER_PIN_B_LEFT,
	      ENCODER_CPR_WHEEL),
	encR_(ENCODER_PIN_A_RIGHT,
	      ENCODER_PIN_B_RIGHT,
	      ENCODER_CPR_WHEEL),
	filterL_(LPFILTER_TAU,
	         LPFILTER_TAU_DER),
	filterR_(LPFILTER_TAU,
	         LPFILTER_TAU_DER),
	encDataL_(),
	encDataR_(),
	iterT_(0),
	iterTnm1_(0),
	encLTnm1_(0),
	encRTnm1_(0),
	iterCount_(0),
	displayTnm1_(0),
	state_{0.0F},
	input_{0.0F},
	errorMessage_(0),
	overrun_(false),
	ctrlMsgErr_(0),
	ctrlStatus_(CTRL_STATUS::TIMEDOUT),
	idMotor_(false)
	{}

	void SO::init(void)
	{
		int32_t rtCode;

		//Initialize Serial port
		Serial.begin(9600);
		Serial.println("Initializing State Observer...");

		SPI.begin();
		delay(10);

		// Init SPI line
		imu_.initSPI();

		//Initialize can port and messages
		SO_CAN_PORT.begin(SO_CAN_BAUD);

		// Initialize CANFD
		pinMode (CANFD_PIN_INT, INPUT);
		pinMode (CANFD_PIN_INT0, INPUT);
		pinMode (CANFD_PIN_INT1, INPUT);
		pinMode (CANFD_PIN_SS, OUTPUT);
		digitalWrite(CANFD_PIN_SS, HIGH);
		APP_CANFDSPI_Init(CAN_1000K_8M);
		txObj.bF.ctrl.IDE = 0;      // Extended CAN ID false
		txObj.bF.id.SID = 0x101;    // CAN ID
		txObj.bF.ctrl.BRS = 1;      // Switch Bitrate true (switch to 2Mbps)
		txObj.bF.ctrl.FDF = 1;      // CAN FD true
		txObj.bF.ctrl.DLC = DRV_CANFDSPI_DataBytesToDlc(64);

    //Initialize IMU
		rtCode = imu_.init(YOST_TTS_LX_RESET_SETTINGS);
		if(rtCode!=1)
		{
			Serial.println("IMU init failed");
			status_ = STATUS::FAILURE;
			errorMessage_ = 1;
			return;
		}

		//Initialize steering
		steering_.init();

    	//Initialize battery monitor
		batteryMonitor_.init();

		// Left elmo init
		rtCode = LEFT_ELMO_.init();
		if (rtCode!=1)
		{
			Serial.println("Left motor init failed");
		}
		else {
			Serial.println("Left Elmo ready to be turned on");
		}

		// Right elmo init
		rtCode = RIGHT_ELMO_.init();
		if (rtCode!=1)
		{
			Serial.println("Right motor init failed");
		}
		else {
			Serial.println("Right Elmo ready to be turned on");
			delay(10);
		}

		//Ready
		Serial.println("State Observer initialized!");
		status_ = STATUS::RUNNING;
		iterT_ = micros();
		displayTnm1_ = iterT_;
		return;
	}

	//Update internal state
	void SO::update(void)
	{
		int32_t rtCode;

		switch(status_)
		{
			case STATUS::RUNNING:
			{
				//Updating the sensors
				while(true)
				{
					//Check for user commands
					if(Serial.available())
					{
						char cmd = (char)Serial.read();
						switch(cmd)
						{
							case 'a':
							{
								publishing_ = true;
								Serial.println("Activating Publishing");
								break;
							}
							case 'i':
							{
								Serial.println("Changing state outputs for idenfitying motor constant");
								idMotor_ = true;
								break;
							}
							case 'o':
							{
								publishing_ = false;
								Serial.println("De-activating Publishing");
								break;
							}
							case 't':
							{
								imu_.tareCustom();
								Serial.println("Taring IMU");
								break;
							}

							default :
							{
								Serial.println("Unkown command");
								break;
							}
						}
					}

					//Updating main IMU
					rtCode = imu_.update();
					if(imu_.status_ != IMUAbstract::STATUS::RUNNING)
					{
						Serial.println("State Observer - update - IMU stopped running");
						errorMessage_ = 3;
						status_ = STATUS::FAILURE;
						return;
					}
					else if(rtCode==1)
					{
						uint32_t t0Temp = micros();
						if((t0Temp-iterT_)>SO_DT_TRAGET_LOW)
						{
							if((t0Temp-iterT_)>SO_DT_TRAGET_HIGH) {
								overrun_ = true;
							}
							else
								overrun_ = false;

							iterTnm1_ = iterT_;
							iterT_ = t0Temp;
							iterCount_++;
							break;
						}
					}
				}

				float iterTfloat = static_cast<float>(iterT_)*1.0E-6F;

				//Read encoders
				encDataL_ = encL_.read();
				filterL_.update(iterTfloat,encDataL_.pos);
				encDataR_ = encR_.read();
				filterR_.update(iterTfloat,encDataR_.pos);

				//Update Steering
				steering_.update();

				//Update Battery monitor
				batteryMonitor_.update();

				//Update and publish state
				if(updateState()==1 && publishing_)
				{
					publishState();
					uint32_t tNow = micros();
					uint32_t tNow2 = tNow;
					dtLLC_ = tNow - iterT_;

					// Catch up with ctrl
					while(updateCtrl()>0)
					{}

					while(true)
					{
						int drew = updateCtrl();
						tNow2 = micros();
						if(drew>0) {
							dtTX2_ = tNow2 - tNow;
			              if (drew == 3) {
			                state_[0] = 0.;
			                state_[1] = 0.;
			                state_[2] = 0.;
			                state_[3] = 0.;
			                state_[4] = 0.;
			                state_[5] = 0.;
			                state_[6] = 0.;
			              }
			              break;
			            }
						else if((micros()-tNow)>SO_CTRL_TIMEOUT)
						{
							if(ctrlStatus_!=CTRL_STATUS::TIMEDOUT) {
								Serial.println("Controller timeout");
								LEFT_ELMO_.stopMotor();
								RIGHT_ELMO_.stopMotor();
							}
							ctrlStatus_ = CTRL_STATUS::TIMEDOUT;
							break;
						}
					}

					//Make sure elmos are initialized
					if (LEFT_ELMO_.status_ == Elmo::STATUS::OFF) {
						LEFT_ELMO_.init();
					}
					if (RIGHT_ELMO_.status_ == Elmo::STATUS::OFF) {
						RIGHT_ELMO_.init();
					}

					//Forward control input to motor controller
					if(ctrlStatus_==CTRL_STATUS::RUNNING){
						forwardCtrl();
					}

					dtELMO_ = micros() - tNow2;

				} else {
					// Serial.println("Note publishing");
				}

				//Display info
				if(iterT_ - displayTnm1_ > SO_DISPLAY_DT)
				{
					displayTnm1_ = iterT_;
					if(overrun_)
						Serial.println("IMU Overrun.");
					if(publishing_)
						Serial.println("Publishing");
					else
						Serial.println("Not publishing");

					if(ctrlStatus_ == CTRL_STATUS::TIMEDOUT)
						Serial.println("Controller timedout.");
					else if(ctrlStatus_ == CTRL_STATUS::CHKSUM_ERR)
						Serial.println("Controller checksum failed.");
					else if(ctrlStatus_ == CTRL_STATUS::FAILURE)
						Serial.println("Controller in failure.");
					else if(ctrlStatus_ == CTRL_STATUS::BAD_IDX)
						Serial.println("Controller sent bad idx.");
					else
						Serial.println("Controller active.");
					Serial.print("Control message errors: ");
					Serial.println(ctrlMsgErr_);

					Serial.print("Battery: ");
					Serial.print(batteryMonitor_.getVoltage());
					Serial.println("V");

					Serial.print("dtLLC: ");
					Serial.print(dtLLC_);
					Serial.println("us");

					Serial.print("dtTX2: ");
					Serial.print(dtTX2_);
					Serial.println("us");

					Serial.print("dtELMO: ");
					Serial.print(dtELMO_);
					Serial.println("us");					

					Serial.print("x: ");
					Serial.println(state_[0],6);
					Serial.print("y: ");
					Serial.println(state_[1],6);
					Serial.print("theta: ");
					Serial.println(state_[2],6);
					Serial.print("v: ");
					Serial.println(state_[3],6);
					Serial.print("thetaDot: ");
					Serial.println(state_[4],6);
					Serial.print("psi: ");
					Serial.println(state_[5],6);
					Serial.print("psiDot: ");
					Serial.println(state_[6],6);
					Serial.print("psi derivative filter: ");
					Serial.println(d_psi_filter,6);
					Serial.print("psiDot bias: ");
					Serial.println(bias,6);
					Serial.println("________________");

				}

				break;
			}

			case STATUS::FAILURE:
			{
				iterT_ = micros();
				if(iterT_-displayTnm1_>SO_FAILURE_DT)
				{
					displayTnm1_ = iterT_;

					//Update Battery monitor
					batteryMonitor_.update();
					LEFT_ELMO_.stopMotor();
					RIGHT_ELMO_.stopMotor();

					publishState();

					Serial.print("State Observer - update - in failure: ");
					Serial.println(errorMessage_);
				}
				break;
			}

			default:
			{
				Serial.println("State Observer - update - unknown SO state...");
				status_ = STATUS::FAILURE;
				errorMessage_ = 4;
				break;
			}
		}// end switch

		return;
	}


	int32_t SO::updateState(void)
	{
		if(iterCount_>0)
		{
			if (idMotor_ == false)
			{
				float stateNew_[7];
				memcpy(stateNew_,state_,sizeof(stateNew_));

				float dt = static_cast<float>(iterT_ - iterTnm1_)/1000000.0F;

				float psiDot = imu_.fusionedData_.rates[1];
				float d_psi = (imu_.fusionedData_.euler[1] - state_[5])/dt;

				d_psi_filter += .05*(d_psi-d_psi_filter);

				if (fabs(d_psi_filter) < 0.05)
					bias = bias + .001*(d_psi_filter-bias);
				psiDot-=bias;

				stateNew_[3] = 0.5F*Rw*(filterL_.data_.yDot+psiDot + filterR_.data_.yDot+psiDot);
				stateNew_[4] = Rw*(filterR_.data_.yDot - filterL_.data_.yDot)/L;

				stateNew_[5] = imu_.fusionedData_.euler[1];
				stateNew_[6] = psiDot;
				stateNew_[2] += dt*0.5F*(state_[4] + stateNew_[4]);

				if(stateNew_[2]>M_PI)
					stateNew_[2] -= 2*M_PI;
				else if(stateNew_[2]<-M_PI)
					stateNew_[2] += 2*M_PI;

				stateNew_[0] += dt*0.5F*(state_[3]*cos(state_[2]) + stateNew_[3]*cos(stateNew_[2]));
				stateNew_[1] += dt*0.5F*(state_[3]*sin(state_[2]) + stateNew_[3]*sin(stateNew_[2]));

				memcpy(state_,stateNew_,sizeof(state_));
				return 1;
			}
			else
			{
				float stateNew_[7];
				stateNew_[0] = filterL_.data_.yDot;
				stateNew_[1] = filterR_.data_.yDot;
				stateNew_[2] = 0;
				stateNew_[3] = 0;
				stateNew_[4] = 0;
				stateNew_[5] = 0;
				stateNew_[6] = 0;
				memcpy(state_,stateNew_,sizeof(state_));
				return 1;
			}
		}

		return 2;
	}

	void SO::publishState(void)
	{
		const uint32_t packetLen = 1                //Status
		                           +sizeof(float)   //iterT
		                           +7*sizeof(float) //State
		                           +sizeof(float)   //Steering
		                           +sizeof(float)   // Voltage
		                           +1;              //checksum
		uint8_t packet[packetLen];
		num32_t n32;
		uint8_t checkSum = 0;
		uint32_t packetOffset = 0;

		//Status
		packet[0] = static_cast<uint8_t>(status_);
		packetOffset++;

		//IterT
		n32.ui = iterT_;
		memcpy(packet+packetOffset,n32.c,4);
		packetOffset+=4;

		//State
		for(uint32_t i = 0; i<7; i++)
		{
			n32.f = state_[i];
			memcpy(packet+packetOffset,n32.c,4);
			packetOffset+=4;
		}

		//Steering
		n32.f = steering_.data_.filtered;
		memcpy(packet+packetOffset,n32.c,4);
		packetOffset+=4;

		//Voltage
		n32.f = batteryMonitor_.getVoltage();
		memcpy(packet+packetOffset,n32.c,4);
		packetOffset+=4;

		//Replace 2nd byte by error message if in failure mode
		if(status_ == STATUS::FAILURE)
			packet[1] = errorMessage_;

		//Checksum
		for(uint32_t i = 0; i<packetLen-1; i++)
		{
			checkSum+=packet[i];
		}
		packet[packetLen-1] = checkSum;

		//Send
		memcpy(txd, packet, packetLen);
		APP_TransmitMessageQueue();
	}

	int32_t SO::updateCtrl(void)
	{
		if(digitalReadFast(CANFD_PIN_INT)==0)
		{
			DRV_CANFDSPI_ReceiveChannelEventGet(DRV_CANFDSPI_INDEX_0, APP_RX_FIFO, &CanRxFlags);

			if (CanRxFlags & CAN_RX_FIFO_NOT_EMPTY_EVENT)
			{
				DRV_CANFDSPI_ReceiveMessageGet(DRV_CANFDSPI_INDEX_0, APP_RX_FIFO, &rxObj, rxd, MAX_DATA_BYTES);
				if (static_cast<uint16_t>(rxObj.bF.id.SID) == 0x101)
				{
					uint8_t chksum = 0;
					for (uint32_t i = 0; i < 9; i++)
					{
						chksum += rxd[i];
					}
					if (chksum == rxd[9])
					{
						ctrlMsgErr_ = 0;
						if(rxd[0]!=2)
						{
							ctrlStatus_ = CTRL_STATUS::FAILURE;
							LEFT_ELMO_.stopMotor();
							RIGHT_ELMO_.stopMotor();
							Serial.println("FBL stopped updatectrl");
							return 4;
						}

						const float *uTemp = reinterpret_cast<const float*>(rxd + 1);
						input_[0] = uTemp[0];
						input_[1] = uTemp[1];

						if (input_[0] > SO_UMAX)
							input_[0] = SO_UMAX;
						else if (input_[0] < -SO_UMAX)
							input_[0] = -SO_UMAX;

						if (input_[1] > SO_UMAX)
							input_[1] = SO_UMAX;
						else if (input_[1] < -SO_UMAX)
							input_[1] = -SO_UMAX;

						if(ctrlStatus_ != CTRL_STATUS::RUNNING){
							LEFT_ELMO_.startMotor();
							RIGHT_ELMO_.startMotor();
							Serial.println("Started Elmos");
						}
						ctrlStatus_ = CTRL_STATUS::RUNNING;
						return 1;
					}
					else
					{
						ctrlStatus_ = CTRL_STATUS::CHKSUM_ERR;
						ctrlMsgErr_++;
					}
				} else if (static_cast<uint16_t>(rxObj.bF.id.SID) == 0x102) {
			          Serial.println("IDX to Reset SO");
			          return 3;
			        }
				else
				{
					Serial.println("BAD IDX");
					ctrlStatus_ = CTRL_STATUS::BAD_IDX;
					ctrlMsgErr_++;
				}

				if(ctrlMsgErr_==SO_CTRL_MAX_CHKSUM_ERR+1) {
					LEFT_ELMO_.stopMotor();
					RIGHT_ELMO_.stopMotor();
					Serial.println("FBL stopped chk");
				}
				return 2;
			}
		}
		return -1;
	}

	void SO::forwardCtrl(void)
	{
		LEFT_ELMO_.sendCurrent(input_[0]);
		RIGHT_ELMO_.sendCurrent(input_[1]);
	}

}
