#ifndef FUTABA_SBUS_h
#define FUTABA_SBUS_h

#include <Arduino.h>

#define  RXD2	40
#define  TXD2  16
#define SBUS_SIGNAL_OK          0x00
#define SBUS_SIGNAL_LOST        0x01
#define SBUS_SIGNAL_FAILSAFE    0x03
#define BAUDRATE 100000
#define port Serial2
//#define ALL_CHANNELS

#define CH0_MIN		(316.f)
#define CH0_MAX		(1680.f)
#define CH0_CEN		(998.f)

#define CH1_MIN		(323.f)
#define CH1_MAX		(1688.f)
#define CH1_CEN		(1006.f)

#define CH2_MIN		(332.f)
#define CH2_MAX		(1704.f)
#define CH2_CEN		(1006.f)

#define CH3_MIN		(316.f)
#define CH3_MAX		(1680.f)
#define CH3_CEN		(998.f)

#define CH4_LOW		(340.f)
#define CH4_HIGH	(1704.f)

#define CH5_LOW		(340.f)
#define CH5_MID		(1024.f)
#define CH5_HIGH	(1704.f)

#define CH6_LOW		(340.f)
#define CH6_HIGH	(1704.f)

#define CH7_LOW		(340.f)
#define CH7_HIGH	(1704.f)

class FUTABA_SBUS
{
	public:
		uint8_t sbusData[25];
		int16_t channels[18];
		int16_t servos[18];
		uint8_t  failsafe_status;
		int sbus_passthrough;
		int toChannels;
		void begin(void);
		int16_t Channel(uint8_t ch);
		uint8_t DigiChannel(uint8_t ch);
		void Servo(uint8_t ch, int16_t position);
		void DigiServo(uint8_t ch, uint8_t position);
		uint8_t Failsafe(void);
		void PassthroughSet(int mode);
		int PassthroughRet(void);
		void UpdateServos(void);
		void UpdateChannels(void);
		void FeedLine(void);
	private:
		uint8_t byte_in_sbus;
		uint8_t bit_in_sbus;
		uint8_t ch;
		uint8_t bit_in_channel;
		uint8_t bit_in_servo;
		uint8_t inBuffer[25];
		int bufferIndex;
		uint8_t inData;
		int feedState;

};

#endif