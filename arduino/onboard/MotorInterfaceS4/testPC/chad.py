#!/usr/bin/python
import usb

dev = usb.core.find(idVendor=0x2341, idProduct=0x0044);
if dev is None:
	raise ValueError("Device not found")

dev.set_configuration();

# get an endpoint instance
cfg = dev.get_active_configuration()
interface_number = cfg[(0,0)].bInterfaceNumber
alternate_setting = usb.control.get_interface(cfg,interface_number);
intf = usb.util.find_descriptor(
		cfg, bInterfaceNumber = interface_number,
		bAlternateSetting = alternate_setting
)
