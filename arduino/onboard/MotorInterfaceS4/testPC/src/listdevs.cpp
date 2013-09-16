/*
 * libusbx example program to list devices on the bus
 * Copyright © 2007 Daniel Drake <dsd@gentoo.org>
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2.1 of the License, or (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with this library; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA
 */

#include <iostream>
#include <stdio.h>

#include <libusb-1.0/libusb.h>

enum test_type
{
	USE_GENERIC,
	USE_PS3,
	USE_XBOX,
	USE_SCSI,
	USE_HID,
} test_mode;

struct command_block_wrapper {
	uint8_t dCBWSignature[4];
	uint32_t dCBWTag;
	uint32_t dCBWDataTransferLength;
	uint8_t bmCBWFlags;
	uint8_t bCBWLUN;
	uint8_t bCBWCBLength;
	uint8_t CBWCB[16];
};

// returns the first MegaADK device
// vid 0x2341, pid 0x0044
static libusb_device* findMegaADK(libusb_device **devs)
{
	libusb_device *dev;
	int i = 0, j = 0;
	uint8_t path[8]; 

	while ((dev = devs[i++]) != NULL) {
		struct libusb_device_descriptor desc;
		int r = libusb_get_device_descriptor(dev, &desc);
		if (r < 0) {
			fprintf(stderr, "failed to get device descriptor");
			return NULL;
		}

		if(desc.idVendor == 0x2341 && desc.idProduct == 0x0044)
		{
			std::cout << "Found an ADK on bus " << (int) libusb_get_bus_number(dev);
			std::cout << " device " << (int)libusb_get_device_address(dev) << std::endl;
			return dev;
		}

//		printf("%04x:%04x (bus %d, device %d)",
//			desc.idVendor, desc.idProduct,
//			libusb_get_bus_number(dev), libusb_get_device_address(dev));
//
//		r = libusb_get_port_numbers(dev, path, sizeof(path));
//		if (r > 0) {
//			printf(" path: %d", path[0]);
//			for (j = 1; j < r; j++)
//				printf(".%d", path[j]);
//		}
//		printf("\n");
	}

	return NULL;
}

int main(void)
{
	int r;
	ssize_t cnt;

	r = libusb_init(NULL);
	if (r < 0)
		return r;

	libusb_device **devs;
	cnt = libusb_get_device_list(NULL, &devs);
	if (cnt < 0)
		return (int) cnt;

	libusb_device* adkDev = findMegaADK(devs);
	libusb_free_device_list(devs, 1);

	if(adkDev == NULL)
	{
		std::cout << "Failed to find Mega ADK" << std::endl;
		return 0;
	}

	uint8_t bus = libusb_get_bus_number(adkDev);

	// Show some device configuration info
	const struct libusb_endpoint_descriptor *endpointDesc;
	uint8_t endpoint_in = 0, endpoint_out = 0;	// default IN and OUT endpoints
	struct libusb_config_descriptor *conf_desc;
	int nb_ifaces, first_iface = -1;
	r = libusb_get_config_descriptor(adkDev, 0, &conf_desc);
	if(r != LIBUSB_SUCCESS){ std::cout << "error loc 1" << std::endl; return 0;}
	nb_ifaces = conf_desc->bNumInterfaces;
	if(nb_ifaces > 0)
		first_iface = conf_desc->interface[0].altsetting[0].bInterfaceNumber;
	for (int i=0; i<nb_ifaces; i++)
	{
		printf("              interface[%d]: id = %d\n", i,
			conf_desc->interface[i].altsetting[0].bInterfaceNumber);
		for (int j=0; j<conf_desc->interface[i].num_altsetting; j++) {
			printf("interface[%d].altsetting[%d]: num endpoints = %d\n",
				i, j, conf_desc->interface[i].altsetting[j].bNumEndpoints);
			printf("   Class.SubClass.Protocol: %02X.%02X.%02X\n",
				conf_desc->interface[i].altsetting[j].bInterfaceClass,
				conf_desc->interface[i].altsetting[j].bInterfaceSubClass,
				conf_desc->interface[i].altsetting[j].bInterfaceProtocol);
			if ( (conf_desc->interface[i].altsetting[j].bInterfaceClass == LIBUSB_CLASS_MASS_STORAGE)
			  && ( (conf_desc->interface[i].altsetting[j].bInterfaceSubClass == 0x01)
			  || (conf_desc->interface[i].altsetting[j].bInterfaceSubClass == 0x06) )
			  && (conf_desc->interface[i].altsetting[j].bInterfaceProtocol == 0x50) ) {
				// Mass storage devices that can use basic SCSI commands
				test_mode = USE_SCSI;
			}
			for (int k=0; k<conf_desc->interface[i].altsetting[j].bNumEndpoints; k++) {
				struct libusb_ss_endpoint_companion_descriptor *ep_comp = NULL;
				endpointDesc = &conf_desc->interface[i].altsetting[j].endpoint[k];
				printf("       endpointDesc[%d].address: %02X\n", k, endpointDesc->bEndpointAddress);
				// Use the first interrupt or bulk IN/OUT endpoints as default for testing
				if ((endpointDesc->bmAttributes & LIBUSB_TRANSFER_TYPE_MASK) & (LIBUSB_TRANSFER_TYPE_BULK | LIBUSB_TRANSFER_TYPE_INTERRUPT)) {
					if (endpointDesc->bEndpointAddress & LIBUSB_ENDPOINT_IN) {
						if (!endpoint_in)
							endpoint_in = endpointDesc->bEndpointAddress;
					} else {
						if (!endpoint_out)
							endpoint_out = endpointDesc->bEndpointAddress;
					}
				}
				printf("           max packet size: %04X\n", endpointDesc->wMaxPacketSize);
				printf("          polling interval: %02X\n", endpointDesc->bInterval);
				libusb_get_ss_endpoint_companion_descriptor(NULL, endpointDesc, &ep_comp);
				if (ep_comp) {
					printf("                 max burst: %02X   (USB 3.0)\n", ep_comp->bMaxBurst);
					printf("        bytes per interval: %04X (USB 3.0)\n", ep_comp->wBytesPerInterval);
					libusb_free_ss_endpoint_companion_descriptor(ep_comp);
				}
			}
		}
	}
	libusb_free_config_descriptor(conf_desc);

	// claim the interfaces
	libusb_device_handle *adkHandle;
	libusb_open(adkDev, &adkHandle);
	libusb_set_auto_detach_kernel_driver(adkHandle, 1);
	for(int iface=0; iface < nb_ifaces; iface++)
	{
		std::cout << "claiming interface " << iface << std::endl;
		r = libusb_claim_interface(adkHandle, iface);
		if(r != LIBUSB_SUCCESS){ std::cout << "error loc 2" << std::endl;}
	}

	// see if we can talk
	uint8_t master_address[] = {0x80, 0x25, 0x00, 0x00, 0x00, 0x00, 0x08};
	unsigned int timeoutMS = 100;
//	r = libusb_control_transfer(adkHandle, LIBUSB_REQUEST_TYPE_CLASS | LIBUSB_RECIPIENT_INTERFACE, 34, 0, 0, NULL, 0, timeoutMS);
	r = libusb_control_transfer(adkHandle, 0x21, 34, 0, 0, NULL, 0, timeoutMS);
	if(r < 0){ std::cout << "error loc 3" << std::endl;}
	r = libusb_control_transfer(adkHandle, 0x21, 32, 0, 0, master_address, sizeof(master_address), timeoutMS);
	if(r < 0){ std::cout << "error loc 4: r = " << r << std::endl;}

	uint8_t endpoint = LIBUSB_ENDPOINT_IN;
	unsigned char byte = 1;
	int retryCnt = 0;
	int retryMax = 5;
	int transferred;
	timeoutMS = 500;
	do
	{
		r = libusb_bulk_transfer(adkHandle, endpoint, (unsigned char*)&byte, 1, &transferred, timeoutMS);
	} while (( r == LIBUSB_ERROR_PIPE) && (++retryCnt < retryMax));

	for(int iface=0; iface < nb_ifaces; iface++)
	{
		std::cout << "release interface " << iface << std::endl;
		libusb_release_interface(adkHandle, iface);
	}

	libusb_exit(NULL);
	return 0;
}
