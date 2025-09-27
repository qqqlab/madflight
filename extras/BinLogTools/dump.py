#dump bin log to stdout

#pip install pymavlink
from pymavlink import DFReader
import sys

if len(sys.argv) < 2: 
  print("Usage: dump [binlog filename] <num lines -OR- msgtype>")
  exit()

filename = sys.argv[1]
num_lines = 999999
filter_type = None
if len(sys.argv) >= 3:
    s = sys.argv[2]
    if s.isnumeric():
        num_lines = int(s)
    else:
        filter_type = s

log = DFReader.DFReader_binary(filename)

lineno = 0
while True:
    m = log.recv_msg()
    if m is None:
        break
    if filter_type is None:
        print(m)
    elif m.get_type() == filter_type:
        print(m)
    lineno += 1
    if(lineno >= num_lines) :
        break;

