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

ACK = 0x06
NACK = 0x15

BUFFERING_SIZE = 4096
MAX_FILESIZE = 1024 * 50

logging.basicConfig(level=logging.INFO)

def main(*args, **kwargs):
  # First argument is filename of binary file to send
  filename = args[0]
  os.path.exists(filename) or sys.exit("File not found")

  # Initialize CAN interface
  bus = SocketcanBus(channel="can0")
  addr = isotp.Address(isotp.AddressingMode.Normal_11bits, txid=ISOTP_CAN_ID_SEND, rxid=ISOTP_CAN_ID_RECV)
  params = {
    'blocking_send': False 
  }

  notifier = Notifier(bus, listeners=[])
  stack = isotp.NotifierBasedCanStack(bus, address=addr, notifier=notifier, error_handler=error_handler, params=params)

  logging.info("Waiting for inital NACK from receiver")
  stack.start()
  
  # Wait for NACK from receiver
  try:
    res = stack.recv(block=True)
    if res[0] == NACK:
      logging.info("Received Initial NACK!")
    else:
      raise Exception("Received unexpected message: ", res)
  except Exception as e:
    stack.stop()
    bus.shutdown()
    raise e
  
  try:
    # Send start ACK
    stack.send(data=ACK.to_bytes(1), send_timeout=2)
  except Exception as e:
    stack.stop()
    bus.shutdown()
    raise e

  stack.stop()

  logging.info("Sent initial ACK, starting file transfer.")

  # Open file buffered by 4KB
  with open(filename, "rb") as f:
    # Read file in chunks of 4KB
    chunk = f.read(BUFFERING_SIZE)
    while chunk:
      # Send chunk over ISOTP
      stack.start()
      stack.send(chunk)
      logging.info(f"Sent chunk of size: {len(chunk)}")
      
      # Wait for response
      try:
        res = stack.recv(block=True)
        if res[0] == ACK:
          logging.info("Received ACK on chunk receive!")
        elif res[0] == NACK:
          raise Exception("Received NACK on chunk receive, aborting.")
      except Exception as e:
        stack.stop()
        bus.shutdown()
        raise e

      # Read next chunk
      chunk = f.read(BUFFERING_SIZE)

  # File sent, close connection
  print("File sent successfully!")
  stack.send(data=ACK.to_bytes(1), send_timeout=2)
  
  stack.stop()
  bus.shutdown()

def error_handler(error):
  logging.warning('IsoTp error happened: %s - %s', (error.__class__.__name__, str(error)))

if __name__ == "__main__":
  main(*sys.argv[1:])