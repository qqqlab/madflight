#print bin log statistics - does not depend on pymavlink

import sys

if len(sys.argv) < 2: 
  print("ERROR: need filename on command line")
  exit()

filename = sys.argv[1]

fh = open(filename, 'rb')
#d = bytearray(fh.read())
d = fh.read()
fh.close()

dlen = len(d)

name = [''] * 256
fmt = [''] * 256
cols = [''] * 256
len = [0] * 256
cnt = [0] * 256
fmtpos = [0] * 256
recpos = [0] * 256

miss_cnt = 0

i = 0

while i<dlen:
#    if(d[i]==0xa3 and d[i+1]==0x95): print("\n")
#    print("{:02X} ".format(d[i]), end="")
    
    if d[i]==0xa3 and d[i+1]==0x95 and d[i+2]==0x80 :
        t = d[i+3]
        len[t] = d[i+4]
        name[t] = d[i+5:i+5+4].decode("ascii").rstrip('\x00')
        fmt[t] = d[i+9:i+9+16].decode("ascii").rstrip('\x00')
        cols[t] = d[i+25:i+25+64].decode("ascii").rstrip('\x00')
        cnt[0x80] += 1
        fmtpos[t] = i
        #print("{:5d} FMT: typ:{:02X} len:{:2d} name:{:<4}".format(i,t,ln,n))
        i += len[0x80]
    elif d[i]==0xa3 and d[i+1]==0x95 :
        t = d[i+2]
        n = name[t]
        ln = len[t]
        cnt[t] += 1
        if recpos[t]==0: 
            recpos[t]=i
            #print("{:5d} {}: len:{:2d}".format(i, n, ln))
        i += ln
    else:
        miss_cnt += 1
        #print("{:02X} ".format(d[i]), end="")
        i += 1

#print unused types
typ_cnt = 0
for i in range(256):
    if name[i] != '' and cnt[i]==0: typ_cnt += 1
if typ_cnt>0:
    print("==== UNUSED ===")
    print("Msg   Count Type Len Format           Columns")
    print("---- ------ ---- --- ---------------- -----------------------------------------")
    max_len = 0
    for i in range(256):
        if name[i] != '' and cnt[i]==0:
            if max_len<len[i]: max_len = len[i]    
            #print("{:02X} len:{:3d} cnt:{:6d} fmtpos:{:6d} recpos:{:6d} name:{}".format(i,len[i],cnt[i],fmtpos[i],recpos[i],name[i]))
        print("{: <4} {:6d} 0x{:02X} {:3d} {: <16} {}".format(name[i],cnt[i],i,len[i],fmt[i],cols[i]))
    print("Type Count     :",typ_cnt)
    print("Max Record Len :",max_len)
    print("");
    print("==== UNUSED ===")

#print used types
print("Msg   Count Type Len Format           Columns")
print("---- ------ ---- --- ---------------- -----------------------------------------")
typ_cnt = 0
rec_cnt = 0
max_len = 0
for i in range(256):
    if name[i] != '' and cnt[i]>0:
        typ_cnt += 1
        rec_cnt += cnt[i]
        if max_len<len[i]: max_len = len[i]
        #print("{:02X} len:{:3d} cnt:{:6d} fmtpos:{:6d} recpos:{:6d} name:{}".format(i,len[i],cnt[i],fmtpos[i],recpos[i],name[i]))
        print("{: <4} {:6d} 0x{:02X} {:3d} {: <16} {}".format(name[i],cnt[i],i,len[i],fmt[i],cols[i]))
print("Type Count     :",typ_cnt)
print("Max Record Len :",max_len)
print("Record Count   :",rec_cnt)
print("Parse Errors   :",miss_cnt)


"""
  struct PACKED {
      uint8_t h1 = HEAD_BYTE1;
      uint8_t h2 = HEAD_BYTE2;
      uint8_t type = 0x80;
      uint8_t msg_type;
      uint8_t length;
      char name[4];
      char format[16];
      char labels[64];
  } FMT;
"""
