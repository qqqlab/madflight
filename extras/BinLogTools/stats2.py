#stats of a bin log

#pip install pymavlink
from pymavlink import DFReader
import sys

if len(sys.argv) < 2: 
  print("Usage: stats2 [binlog filename]")
  exit()

filename = sys.argv[1]


print("------------------")
print("first message only")
print("------------------")

log = DFReader.DFReader_binary(filename)
mtypes = {}
while True:
    m = log.recv_msg()
    if m is None:
        break
    t = m.get_type()
    if t in mtypes:
        mtypes[t] += 1
    else:
        mtypes[t] = 1
        print(m)

print("--------------")
print("message counts")
print("--------------")
print(mtypes)
