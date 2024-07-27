import isotp
import os
import sys
import logging

from can.interfaces.socketcan import SocketcanBus
from can import Notifier, Message

ISOTP_CAN_ID_SEND = 0x700
ISOTP_CAN_ID_RECV = 0x701

CAN_ID_SEND = 0x703
CAN_ID_RECV = 0x702

BUFFERING_SIZE = 4096
MAX_FILESIZE = 1024 * 50

def main(*args, **kwargs):
  # First argument is filename of binary file to send
  filename = args[0]
  os.path.exists(filename) or sys.exit("File not found")

  # Initialize CAN interface
  bus = SocketcanBus(channel="can0")
  addr = isotp.Address(isotp.AddressingMode.Normal_11bits, txid=ISOTP_CAN_ID_SEND, rxid=ISOTP_CAN_ID_RECV)
  params = {
    'blocking_send': True
  }

  notifier = Notifier(bus, listeners=[])
  stack = isotp.NotifierBasedCanStack(bus, address=addr, notifier=notifier, error_handler=error_handler, params=params)

  # Open file buffered by 4KB
  with open(filename, "rb") as f:
    # Read file in chunks of 4KB
    chunk = f.read(BUFFERING_SIZE)
    while chunk:
      # Send chunk over ISOTP
      stack.start()
      stack.send(chunk)
      logging.debug("Sent chunk of size: ", len(chunk))
      stack.stop()
      
      # Wait for response
      res = bus.recv()
      if res.arbitration_id == CAN_ID_RECV:
        if res.data == 0x1:
          logging.debug("Received ACK")
        elif res.data == 0x0:
          raise Exception("Received NACK, aborting")
      else:
        raise Exception("Received unexpected message: ", res)
        
      # Read next chunk
      chunk = f.read(BUFFERING_SIZE)

  # File sent, close connection
  print("File sent successfully")
  success_msg = Message(arbitration_id=CAN_ID_SEND, data=[0x1])
  bus.send(success_msg)
  
  bus.shutdown()

def error_handler(error):
  logging.warning('IsoTp error happened: %s - %s', (error.__class__.__name__, str(error)))

if __name__ == "__main__":
  main(*sys.argv[1:])