#include "Adb2.h"

namespace Adb2 {

const uint8_t Adb2::epDataInIndex = 1;
const uint8_t Adb2::epDataOutIndex = 2;

Adb2::Adb2(USB *p=NULL) : pUsb(p)
{
	bAddress = 0;
	bConfNum = 0;
	bNumEP = 1;
	ready = false;

	for (uint8_t i = 0; i < ADB2_MAX_ENDPOINTS; i++)
	{
		epInfo[i].epAddr = 0;
		epInfo[i].maxPktSize = (i) ? 0 : 8;
		epInfo[i].epAttribs = (0xfc & (USB_NAK_MAX_POWER << 2));
	}

	//set bulk-IN EP naklimit to 1
	epInfo[epDataInIndex].epAttribs = (0xfc & (USB_NAK_NOWAIT << 2));

	// register in USB subsystem
	if (pUsb) {
		pUsb->RegisterDeviceClass(this); //set devConfig[] entry
	}

	lastConnectAttemptTime = 0;
}

uint8_t Adb2::Init(uint8_t parent, uint8_t port, bool lowspeed)
{
	uint8_t buf[sizeof (USB_DEVICE_DESCRIPTOR)];
	uint8_t rcode;
	UsbDevice *p = NULL;
	EpInfo *oldep_ptr = NULL;
	uint8_t num_of_conf; // number of configurations

	// get memory address of USB device address pool
	AddressPool &addrPool = pUsb->GetAddressPool();

	USBTRACE("ADK Init\n");

	// check if address has already been assigned to an instance
	if (bAddress) {
		USBTRACE("\r\nAddress in use\n");
		return USB_ERROR_CLASS_INSTANCE_ALREADY_IN_USE;
	}

	// Get pointer to pseudo device with address 0 assigned
	p = addrPool.GetUsbDevicePtr(0);

	if (!p) {
		USBTRACE("\r\nAddress not found");
		return USB_ERROR_ADDRESS_NOT_FOUND_IN_POOL;
	}

	if (!p->epinfo) {
		USBTRACE("epinfo is null\r\n");
		return USB_ERROR_EPINFO_IS_NULL;
	}

	// Save old pointer to EP_RECORD of address 0
	oldep_ptr = p->epinfo;

	// Temporary assign new pointer to epInfo to p->epinfo in order to avoid toggle inconsistence
	p->epinfo = epInfo;

	p->lowspeed = lowspeed;

	// Get device descriptor
	rcode = pUsb->getDevDescr(0, 0, sizeof (USB_DEVICE_DESCRIPTOR), (uint8_t*)buf);

	// Restore p->epinfo
	p->epinfo = oldep_ptr;

	if (rcode) {
		Serial.println("FailGetDevDescr");
		return rcode;
	}

	// Allocate new address according to device class
	bAddress = addrPool.AllocAddress(parent, false, port);

	// Extract Max Packet Size from device descriptor
	epInfo[0].maxPktSize = (uint8_t)((USB_DEVICE_DESCRIPTOR*)buf)->bMaxPacketSize0;

	// Assign new address to the device
	rcode = pUsb->setAddr(0, 0, bAddress);
	if (rcode) {
		p->lowspeed = false;
		addrPool.FreeAddress(bAddress);
		bAddress = 0;
		//USBTRACE2("setAddr:",rcode);
		return rcode;
	}//if (rcode...

	//USBTRACE2("\r\nAddr:", bAddress);

	p->lowspeed = false;

	//get pointer to assigned address record
	p = addrPool.GetUsbDevicePtr(bAddress);
	if (!p) {
		return USB_ERROR_ADDRESS_NOT_FOUND_IN_POOL;
	}

	p->lowspeed = lowspeed;

	// Assign epInfo to epinfo pointer - only EP0 is known
	rcode = pUsb->setEpInfoEntry(bAddress, 1, epInfo);
	if (rcode) {
		Serial.println("FailSetDevTblEntry");
		return rcode;
	}

	//check if ADK device is already in accessory mode; if yes, configure and exit
//	if (((USB_DEVICE_DESCRIPTOR*)buf)->idVendor == ADK_VID &&
//			(((USB_DEVICE_DESCRIPTOR*)buf)->idProduct == ADK_PID || ((USB_DEVICE_DESCRIPTOR*)buf)->idProduct == ADB_PID))
	{
		USBTRACE("Acc.mode device detected\n");
		/* go through configurations, find first bulk-IN, bulk-OUT EP, fill epInfo and quit */
		num_of_conf = ((USB_DEVICE_DESCRIPTOR*)buf)->bNumConfigurations;

		USBTRACE2("NC: ",num_of_conf);
		USBTRACE("\n");

		for (uint8_t i = 0; i < num_of_conf; i++) {
			ConfigDescParser < 0, 0, 0, 0 > confDescrParser(this);
			rcode = pUsb->getConfDescr(bAddress, 0, i, &confDescrParser);
			if (rcode) {
				Serial.println("FailGetConfDescr");
				return rcode;
			}
			if (bNumEP > 2) {
				break;
			}
		} // for (uint8_t i=0; i<num_of_conf; i++...

		if (bNumEP == 3) {
			// Assign epInfo to epinfo pointer - this time all 3 endpoins
			rcode = pUsb->setEpInfoEntry(bAddress, 3, epInfo);
			if (rcode) {
				Serial.println("FailSetDevTblEntry");
				return rcode;
			}
		}

		// Set Configuration Value
		rcode = pUsb->setConf(bAddress, 0, bConfNum);
		if (rcode) {
			Serial.println("FailSetConfDescr");
			return rcode;
		}
		/* print endpoint structure */
		//		          USBTRACE("\r\nEndpoint Structure:");
		//		          USBTRACE("\r\nEP0:");
		//		          USBTRACE2("\r\nAddr: ", epInfo[0].epAddr );
		//	            USBTRACE2("\r\nMax.pkt.size: ", epInfo[0].maxPktSize );
		//	            USBTRACE2("\r\nAttr: ", epInfo[0].epAttribs );
		//	            USBTRACE("\r\nEpout:");
		//		          USBTRACE2("\r\nAddr: ", epInfo[epDataOutIndex].epAddr );
		//	            USBTRACE2("\r\nMax.pkt.size: ", epInfo[epDataOutIndex].maxPktSize );
		//	            USBTRACE2("\r\nAttr: ", epInfo[epDataOutIndex].epAttribs );
		//	            USBTRACE("\r\nEpin:");
		//		          USBTRACE2("\r\nAddr: ", epInfo[epDataInIndex].epAddr );
		//	            USBTRACE2("\r\nMax.pkt.size: ", epInfo[epDataInIndex].maxPktSize );
		//	            USBTRACE2("\r\nAttr: ", epInfo[epDataInIndex].epAttribs );

		USBTRACE("Configuration successful\n");
		ready = true;
		return 0; //successful configuration
	}//if( buf->idVendor == ADK_VID...

	return -1;
}

uint8_t Adb2::Release()
{
	pUsb->GetAddressPool().FreeAddress(bAddress);

	bNumEP = 1; //must have to be reset to 1

	bAddress = 0;
	ready = false;
	return 0;
}

/* Extracts bulk-IN and bulk-OUT endpoint information from config descriptor */
void Adb2::EndpointXtract(uint8_t conf, uint8_t iface, uint8_t alt, uint8_t proto, const USB_ENDPOINT_DESCRIPTOR *pep)
{
	//ErrorMessage<uint8_t>(PSTR("Conf.Val"),	conf);
	//ErrorMessage<uint8_t>(PSTR("Iface Num"),iface);
	//ErrorMessage<uint8_t>(PSTR("Alt.Set"),	alt);

	//added by Yuuichi Akagawa
	if (bNumEP == 3) {
		return;
	}

	bConfNum = conf;

	uint8_t index;

	//	if ((pep->bmAttributes & 0x02) == 2) {
	index = ((pep->bEndpointAddress & 0x80) == 0x80) ? epDataInIndex : epDataOutIndex;
	//  }

	// Fill in the endpoint info structure
	epInfo[index].epAddr = (pep->bEndpointAddress & 0x0F);
	epInfo[index].maxPktSize = (uint8_t)pep->wMaxPacketSize;

	bNumEP++;

	PrintEndpointDescriptor(pep);
}

void Adb2::PrintEndpointDescriptor(const USB_ENDPOINT_DESCRIPTOR* ep_ptr) {
        Notify(PSTR("Endpoint descriptor:"), 0x80);
        Notify(PSTR("\r\nLength:\t\t"), 0x80);
        D_PrintHex<uint8_t > (ep_ptr->bLength, 0x80);
        Notify(PSTR("\r\nType:\t\t"), 0x80);
        D_PrintHex<uint8_t > (ep_ptr->bDescriptorType, 0x80);
        Notify(PSTR("\r\nAddress:\t"), 0x80);
        D_PrintHex<uint8_t > (ep_ptr->bEndpointAddress, 0x80);
        Notify(PSTR("\r\nAttributes:\t"), 0x80);
        D_PrintHex<uint8_t > (ep_ptr->bmAttributes, 0x80);
        Notify(PSTR("\r\nMaxPktSize:\t"), 0x80);
        D_PrintHex<uint16_t > (ep_ptr->wMaxPacketSize, 0x80);
        Notify(PSTR("\r\nPoll Intrv:\t"), 0x80);
        D_PrintHex<uint8_t > (ep_ptr->bInterval, 0x80);
        Notify(PSTR("\r\n"), 0x80);
}

uint8_t Adb2::Poll()
{
	if(!ready)
		return 0;

	uint8_t rcode=0;
	bool triedConnect = false;
	if(millis()-lastConnectAttemptTime > ADB_CONNECTION_RETRY_TIME && ready)
	{
		rcode = sendConnectionRequest();
		if(rcode != 0)
			return rcode;
		delay(500);
		lastConnectAttemptTime = millis();
		triedConnect = true;
	}

	uint16_t nBytes = ADB_USB_PACKETSIZE;
	uint8_t buff[ADB_USB_PACKETSIZE];
	rcode = pUsb->inTransfer(bAddress, epInfo[epDataInIndex].epAddr, &nBytes, buff);

	if(rcode == 0)
		Serial.println("Success!");
	else if(triedConnect)
	{ Serial.print("read problem 1: "); Serial.println(rcode,HEX); }

	return rcode;
}

uint8_t Adb2::sendConnectionRequest()
{
	AdbMessage m;
	m.command = A_CNXN;
	m.arg0 = A_VERSION;
	m.arg1 = 4096;
	char * data = (char*)"host::microbridge";
	m.data_length = strlen(data)+1;

	Serial.println("Sending connection request");
	return sendMessage(m, (uint8_t*)data);
}

uint8_t Adb2::sendMessage(AdbMessage &m, uint8_t *data)
{
	// checksum
	uint32_t count, sum=0;
	count = m.data_length;
	uint8_t *x = data;
	while(count-- > 0) sum += *x++;

	m.data_check = sum;
	m.magic = m.command ^ 0xffffffff;

//Serial.print("size of adb_message: "); Serial.println(sizeof(m));
//Serial.print("length: "); Serial.println(m.data_length);
//Serial.print("sum: "); Serial.println(sum);
//Serial.print("magic: "); Serial.println(m.magic);
//uint8_t *p = (uint8_t*)&m;
//for(int i=0; i<sizeof(m); i++)
//	Serial.print(*p++, HEX);
//Serial.print("\n");

	uint8_t rcode;
	rcode = pUsb->outTransfer(bAddress, epInfo[epDataOutIndex].epAddr, sizeof(m), (uint8_t*)&m);
	if(rcode != 0)
	{
		Serial.print("Send problem 1: "); Serial.println(rcode,HEX);
		return rcode;
	}

//p = data;
//for(int i=0; i<m.data_length; i++)
//	Serial.print(*p++, HEX);
//Serial.print("\n");
	rcode = pUsb->outTransfer(bAddress, epInfo[epDataOutIndex].epAddr, m.data_length, data);
	if(rcode != 0)
	{ Serial.print("Send problem 2: "); Serial.println(rcode,HEX); }

	return rcode;
}

}
