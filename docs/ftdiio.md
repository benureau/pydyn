

    $ kextstat | grep FTDI
       98    0 0xffffff7f81820000 0x8000     0x8000     com.FTDI.driver.FTDIUSBSerialDriver (2.2.18) <97 34 5 4 3 1>
    $ sudo kextunload --bundle com.FTDI.driver.FTDIUSBSerialDriver
    Password:
    $ kextstat | grep FTDI

Unplug and replub the usb to serial adaptater

    $ kextstat | grep FTDI
      133    0 0xffffff7f822e1000 0x7000     0x7000     com.apple.driver.AppleUSBFTDI (1.0.1b3) <97 34 5 4 3>
    $ sudo kextunload --bundle com.apple.driver.AppleUSBFTDI
