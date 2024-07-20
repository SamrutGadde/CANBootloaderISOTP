import isotp

isotpSocket = isotp.socket()

isotpSocket.set_fc_opts(stmin=5, bs=5)

isotpSocket.bind("can0", isotp.Address(rxid=0x701, txid=0x700))
isotpSocket.send(b"Hello, World!")
print(isotpSocket.recv())