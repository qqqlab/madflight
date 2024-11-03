#dump bin log to stdout

#pip install pymavlink
from pymavlink import DFReader
import sys

filename = sys.argv[1]
log = DFReader.DFReader_binary(filename)

while True:
    m = log.recv_msg()
    if m is None:
        break
    #bout.write(m.get_msgbuf())
    print(m)

