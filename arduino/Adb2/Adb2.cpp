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

	for (uint8_t i = 0; i < ADK_MAX_ENDPOINTS; i++)
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

	USBTRACE("\r\nADK Init");

	// check if address has already been assigned to an instance
	if (bAddress) {
		USBTRACE("\r\nAddress in use");
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
//		goto FailGetDevDescr;
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
//		goto FailSetDevTblEntry;
	}

	//check if ADK device is already in accessory mode; if yes, configure and exit
//	if (((USB_DEVICE_DESCRIPTOR*)buf)->idVendor == ADK_VID &&
//			(((USB_DEVICE_DESCRIPTOR*)buf)->idProduct == ADK_PID || ((USB_DEVICE_DESCRIPTOR*)buf)->idProduct == ADB_PID))
	{
		USBTRACE("\r\nAcc.mode device detected");
		/* go through configurations, find first bulk-IN, bulk-OUT EP, fill epInfo and quit */
		num_of_conf = ((USB_DEVICE_DESCRIPTOR*)buf)->bNumConfigurations;

		//USBTRACE2("\r\nNC:",num_of_conf);

		for (uint8_t i = 0; i < num_of_conf; i++) {
			ConfigDescParser < 0, 0, 0, 0 > confDescrParser(this);
			rcode = pUsb->getConfDescr(bAddress, 0, i, &confDescrParser);
			if (rcode) {
//				goto FailGetConfDescr;
			}
			if (bNumEP > 2) {
				break;
			}
		} // for (uint8_t i=0; i<num_of_conf; i++...

		if (bNumEP == 3) {
			// Assign epInfo to epinfo pointer - this time all 3 endpoins
			rcode = pUsb->setEpInfoEntry(bAddress, 3, epInfo);
			if (rcode) {
//				goto FailSetDevTblEntry;
			}
		}

		// Set Configuration Value
		rcode = pUsb->setConf(bAddress, 0, bConfNum);
		if (rcode) {
//			goto FailSetConfDescr;
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

		USBTRACE("\r\nConfiguration successful");
		ready = true;
Serial.println("Init successful");
		return 0; //successful configuration
	}//if( buf->idVendor == ADK_VID...

	return -1;
}

uint8_t Adb2::Release()
{
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

	//PrintEndpointDescriptor(pep);
}

}
