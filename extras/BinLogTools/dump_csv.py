#dump bin log to file out.txt

#pip install pymavlink
from pymavlink import DFReader
import sys
import csv

if len(sys.argv) < 3: 
    print("Usage: dump_csv.py [filename] [type] --> dumps output to file out.txt")
    exit()

filename = sys.argv[1]
type = sys.argv[2]

log = DFReader.DFReader_binary(filename)

printheader = True
with open('out.txt', 'w', newline='') as f:
    while True:
        m = log.recv_msg()
        if m is None:
            break
        if m.get_type() == type:
            d = m.to_dict()
            if printheader:
                fieldnames = d.keys();
                writer = csv.DictWriter(f, fieldnames=fieldnames, delimiter='\t', extrasaction='ignore')
                writer.writeheader()
                printheader = False
            writer.writerow(d)

