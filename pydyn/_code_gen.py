import protocol

a = sorted((protocol.REG_ADDRESS(key), key) for key in protocol.DXL_CONTROLS.keys())
for _, key in a:
    print 'DXL_{} = REG_ADDRESS(\'{}\')'.format(key, key)