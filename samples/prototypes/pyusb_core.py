import usb.core
dev = usb.core.find()
print(dev.bDescriptorType)
print(dev)