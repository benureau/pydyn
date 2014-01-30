import pyudev
"""
(u'BUSNUM', u'002')
(u'DEVNAME', u'/dev/bus/usb/002/005')
(u'DEVNUM', u'005')
(u'DEVPATH', u'/devices/pci0000:00/0000:00:1d.0/usb2/2-1/2-1.1')
(u'DEVTYPE', u'usb_device')
(u'DRIVER', u'usb')
(u'ID_BUS', u'usb')
(u'ID_MM_DEVICE_IGNORE', u'1')
(u'ID_MODEL', u'FT232R_USB_UART')
(u'ID_MODEL_ENC', u'FT232R\\x20USB\\x20UART')
(u'ID_MODEL_FROM_DATABASE', u'FT232 USB-Serial (UART) IC')
(u'ID_MODEL_ID', u'6001')
(u'ID_REVISION', u'0600')
(u'ID_SERIAL', u'FTDI_FT232R_USB_UART_A8006BPa')
(u'ID_SERIAL_SHORT', u'A8006BPa')
(u'ID_USB_INTERFACES', u':ffffff:')
(u'ID_VENDOR', u'FTDI')
(u'ID_VENDOR_ENC', u'FTDI')
(u'ID_VENDOR_FROM_DATABASE', u'Future Technology Devices International, Ltd')
(u'ID_VENDOR_ID', u'0403')
(u'MAJOR', u'189')
(u'MINOR', u'132')
(u'PRODUCT', u'403/6001/600')
(u'SUBSYSTEM', u'usb')
(u'TYPE', u'0/0/0')
(u'USEC_INITIALIZED', u'22063859428')
"""

def usb_syspath():
    context = pyudev.Context()
    return {device['ID_SERIAL_SHORT']: device.sys_path for device in context.list_devices(subsystem='usb', ID_VENDOR_ID='0403', ID_MODEL_ID='6001')}

print(usb_syspath())