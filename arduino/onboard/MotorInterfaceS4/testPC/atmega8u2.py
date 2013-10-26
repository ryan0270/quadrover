#!/usr/bin/python
import usb
#import thread

def setup():
	busses = usb.busses()
	#handle = 0
	print("setting up")
	for bus in busses:
		devices = bus.devices
		for dev in devices:
			if dev.idVendor == 0x2341:
				handle = dev.open()
				handle.setConfiguration(dev.configurations[0]);
				handle.setConfiguration(dev.configurations[0])
				handle.claimInterface(dev.configurations[0].interfaces[1][0])

	handle.controlMsg(requestType = 0x21, request = 34, value = 0, index = 0, buffer = 0, timeout = 0) # SetControlLineState
	handle.controlMsg(requestType = 0x21, request = 32, value = 0, index = 0, buffer = [0x80, 0x25, 0x00, 0x00, 0x00, 0x00, 0x08], timeout = 0) # SetLineEncoding: 8N1, 9600 baud

	print("done")
	return #handle
	
		
def sendCommand(handle):
	print("Sending command")
	while True:
		msg = raw_input("enter command in hex (0xA4): ")
		try:
			num = int(msg, 16)
			print("sending "+hex(num))
			handle.bulkWrite(0x04, [num,],0)
		except Exception as inst:
			print(inst)
			print("cannot parse number" )
		
if __name__=='__main__':
	handle = setup()
	sendCommand(handle)
	
