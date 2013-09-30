// This is based on the ADK class from the usb host library
#ifndef ADB2_H
#define ADB2_H

#include <inttypes.h>
#include <avr/pgmspace.h>
#include <avrpins.h>
#include <max3421e.h>
#include <usbhost.h>
#include <usb_ch9.h>
#include <Usb.h>

#include "Arduino.h"
#include "printhex.h"
#include "hexdump.h"
#include "message.h"

#include "confdescparser.h"

#define ADB2_MAX_ENDPOINTS 3 //endpoint 0, bulk_IN, bulk_OUT

#define MAX_PAYLOAD 4096

#define A_SYNC 0x434e5953
#define A_CNXN 0x4e584e43
#define A_OPEN 0x4e45504f
#define A_OKAY 0x59414b4f
#define A_CLSE 0x45534c43
#define A_WRTE 0x45545257
#define A_AUTH 0x48545541

#define A_VERSION 0x01000000        // ADB protocol version

#define ADB_VERSION_MAJOR 1         // Used for help/version information
#define ADB_VERSION_MINOR 0         // Used for help/version information

#define ADB_SERVER_VERSION    31    // Increment this when we want to force users to start a new adb server

#define ADB_USB_PACKETSIZE 0x40
#define ADB_CONNECTION_RETRY_TIME 1000

#define ADK_VID   0x18D1
#define ADK_PID   0x2D00
#define ADB_PID   0x2D01

namespace Adb2 {
class AdbMessage
{
	public:
	uint32_t command;
	uint32_t arg0;
	uint32_t arg1;
	uint32_t data_length;
	uint32_t data_check;
	uint32_t magic;
};

class Adb2 : public USBDeviceConfig, public UsbConfigXtracter
{
	public:
	Adb2(USB *pUsb);

	// Methods for receiving and sending data
//	uint8_t RcvData(uint16_t *nbytesptr, uint8_t *dataptr);
//	uint8_t SndData(uint16_t nbytes, uint8_t *dataptr);

	// USBDeviceConfig implementation
	virtual uint8_t Init(uint8_t parent, uint8_t port, bool lowspeed);
	virtual uint8_t Release();

	virtual uint8_t Poll();

	virtual uint8_t GetAddress() {
		return bAddress;
	};

	virtual bool isReady() {
		return ready;
	};

	//UsbConfigXtracter implementation
	virtual void EndpointXtract(uint8_t conf, uint8_t iface, uint8_t alt, uint8_t proto, const USB_ENDPOINT_DESCRIPTOR *ep);

	protected:
	static const uint8_t epDataInIndex; // DataIn endpoint index
	static const uint8_t epDataOutIndex; // DataOUT endpoint index
	
	unsigned long lastConnectAttemptTime;
	USB *pUsb;
	uint8_t bAddress;
	uint8_t bConfNum; // configuration number
	
	uint8_t bNumEP; // total number of EP in the configuration
	bool ready;
	
	/* Endpoint data structure */
	EpInfo epInfo[ADB2_MAX_ENDPOINTS];
	
	void PrintEndpointDescriptor(const USB_ENDPOINT_DESCRIPTOR* ep_ptr);
	uint8_t sendConnectionRequest();
	uint8_t sendMessage(AdbMessage &m, uint8_t *data);

};
}

#endif
