'''
Created on 29.05.2013

@author: Leander_John
'''

import usb.core, usb.util

if __name__ == "__main__":
        
        # find our device
        dev = usb.core.find(idVendor=0xfffe, idProduct=0x0001)
        print dev
        
        if dev is None:
            raise ValueError('Device not found')
        
        # set the active configuration. With no arguments, the first
        # configuration will be the active one
        dev.set_configuration()
        
        # get an endpoint instance
        cfg = dev.get_active_configuration()
        interface_number = cfg[0].bInterfaceNumber
        alternate_setting = usb.control.get_interface(interface_number)
        intf = usb.util.find_descriptor(
            cfg, bInterfaceNumber = interface_number,
            bAlternateSetting = alternate_setting
        )
        
        ep = usb.util.find_descriptor(
        intf,
        # match the first OUT endpoint
        custom_match = \
        lambda e: \
            usb.util.endpoint_direction(e.bEndpointAddress) == \
            usb.util.ENDPOINT_OUT
    )
        
        # write the data
        ep.write('test')
        
        