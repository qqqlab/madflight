# GPS Messages

What information is send with each NMEA/PUBX message?

|Message|Data Types|
|-|-|
PUBX00 | S-TLAHSNPV
RMC    | SDTL-HS---
GGA    | S-TLA--NP-
GLL    | S-TL------
GSA    | S------NP-
VTG    | S----HS---

Data Types
 - S: Status
 - D: Date
 - T: Time
 - L: Lat/lon
 - A: Altitude
 - S: Speed
 - H: Heading
 - N: Number of sats
 - P: DOP
 - V: Vertical velocity

## PUBX00 u-blox Proprietary Message
 
`$PUBX,00,081350.00,4717.113210,N,00833.915187,E,546.589,G3,2.1,2.0,0.007,77.52,0.007,,0.92,1.19,0.77,9,0,0*5F`

|Field No.|Example|Format|Description|
|-|-|-|-|
0 | $PUBX | string $PUBX | Message ID, UBX protocol header, proprietary sentence
1 | 00 | numeric | Proprietary message identifier: 00
2 | 081350.00 | hhmmss.sss | UTC Time, Current time
3 | 4717.113210 | ddmm.mmmm | Latitude, Degrees + minutes
4 | N | character N/S | N/S Indicator, N=north or S=south
5 | 00833.915187 | Longitude - Longitude, Degrees + minutes
6 | E | character E/W | E/W indicator, E=east or W=west
7 | 546.589 | numeric | Altitude in meters above user datum ellipsoid
8 | G3 | string | Navigation Status, see table below
9 | 2.1 | numeric | Horizontal accuracy estimate in meters
10 | 2.0 | numeric | Vertical accuracy estimate in meters
11 | 0.007 | numeric | Speed over ground in km/h
12 | 77.52 | numeric | Course over ground in degrees
13 | 0.007 | numeric | Vertical downward velocity
14 | - | numeric | Age of most recent DGPS corrections in seconds, empty = none available
15 | 0.92 | numeric | HDOP Horizontal Dilution of Precision
16 | 1.19 | numeric | VDOP Vertical Dilution of Precision
17 | 0.77 | numeric | TDOP Time Dilution of Precision
18 | 9 | numeric | Number of GPS satellites
19 | 0 | numeric | Number of GLONASS satellites
20 | 0 | numeric | DR used
21 | *5F | hexadecimal | Checksum
22 | - | character <CR><LF> | Carriage Return and Line Feed

*Navigation Status*  
NF No Fix  
DR Dead reckoning only solution  
G2 Stand alone 2D solution  
G3 Stand alone 3D solution  
D2 Differential 2D solution  
D3 Differential 3D solution  
RK Combined GPS + dead reckoning solution  
TT Time only solution  

Send: `$PUBX,00*33<CR><LF>` to poll PUBX00 message

## GGA Message

`$GPGGA,092725.00,4717.11399,N,00833.91590,E,1,8,1.01,499.6,M,48.0,M,,0*5B`

|Field No.|Example|Format|Description|
|-|-|-|-|
0 | $GPGGA | string $GPGGA | Message ID, GGA protocol header
1 | 092725.00 | hhmmss.sss | UTC Time, Current time
2 | 4717.11399 | ddmm.mmmm | Latitude, Degrees + minutes
3 | N | character N/S | N/S Indicator, N=north or S=south
4 | 00833.91590 | dddmm.mmmm | Longitude - Longitude, Degrees + minutes
5 | E | character E/W | E/W indicator, E=east or W=west
6 | 1 | digit | Position Fix Status Indicator, See Table below
7 | 8 | numeric | Satellites Used, Range 0 to 12
8 | 1.01 | numeric HDOP | HDOP, Horizontal Dilution of Precision
9 | 499.6 | numeric | MSL Altitude in m
10 | M | character M | MSL Altitude Unit, Meters (fixed field)
11 | 48.0 | numeric | Geoid Separation in m
12 | M | character M | Geoid Separation Unit, Meters (fixed field)
13 | - | numeric | Age of Differential Corrections in seconds, Blank (Null) fields when DGPS is not used
14 | 0 | numeric | Diff. Reference Station ID
15 | *5B | hexadecimal | Checksum
16 | - | character <CR><LF> | Carriage Return and Line Feed

*Fix Status*  
0 No Fix / Invalid  
1 Standard GPS (2D/3D)  
2 Differential GPS  
6 Estimated (DR) Fix  

## RMC Message

`$GPRMC,083559.00,A,4717.11437,N,00833.91522,E,0.004,77.52,091202,,,A*57`

|Field No.|Example|Format|Description|
|-|-|-|-|
0 | $GPGGA | string $GPRMC | Message ID, RMC protocol header
1 | 083559.00 | hhmmss.sss | UTC Time, Current time
2 | A | character | Status - Status, V = Navigation receiver warning, A = Data valid
3 | 4717.11437 | ddmm.mmmm | Latitude, Degrees + minutes
4 | N | character N/S | N/S Indicator, N=north or S=south
5 | 00833.91522 | dddmm.mmmm | Longitude - Longitude, Degrees + minutes, see Format description
6 | E | character E/W | E/W indicator, E=east or W=west
7 | 0.004 | numeric | Speed over ground in knots
8 | 77.52 | numeric | Course over ground in degrees
9 | 091202 | ddmmyy date | Date in day, month, year format
10 | - | numeric | Magnetic variation value, not being output by receiver
11 | - | character | Magnetic variation E/W indicator, not being outputby receiver
12 | - | character | Mode Indicator, see Position Fix Flags description
13 | *57 | hexadecimal | Checksum
14 | - | character <CR><LF> | Carriage Return and Line Feed