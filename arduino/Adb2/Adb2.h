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

#define ADK_MAX_ENDPOINTS 3 //endpoint 0, bulk_IN, bulk_OUT

namespace Adb2 {
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

	virtual uint8_t Poll() {
		return 0;
	};

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
	
	USB *pUsb;
	uint8_t bAddress;
	uint8_t bConfNum; // configuration number
	
	uint8_t bNumEP; // total number of EP in the configuration
	bool ready;
	
	/* Endpoint data structure */
	EpInfo epInfo[ADK_MAX_ENDPOINTS];
	
	void PrintEndpointDescriptor(const USB_ENDPOINT_DESCRIPTOR* ep_ptr);

};
}

#endif
