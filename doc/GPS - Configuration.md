# Configuration of GPS Receivers for madflight

Handy tool: NMEA checksum calculator: https://nmeachecksum.eqth.net/

## u-blox 6 and later

Folow these instructions to enable 10Hz messages on u-blox M8 receivers.

1. Download and install u-center software from u-blox
2. Menu: Receiver -> Connection: set serial port
3. Menu: Receiver -> Baudrate: set baud rate
4. Menu: View -> Text Console: should have output now
5. Menu: View -> Configuration: View

Press send after each changed setting.

PRT: 
 
MSG: Set GGA, RMC, PUBX00 10Hz, others 1Hz
 - F0-00 NMEA GxGGA: (keep ON 1)
 - F0-01 NMEA GxGGL: ON 10 (was ON 1)
 - F0-02 NMEA GxGSA: ON 10 (was ON 1)
 - F0-03 NMEA GxGSV: ON 10 (was ON 1)
 - F0-04 NMEA GxRMC: (keep ON 1) 
 - F0-05 NMEA GxVTG: ON 10 (was ON 1)
 - F1-00 PUBX00: ON 1 (was OFF)

RATE: 
 - Measurement Period: 100 ms

CFG:
 - select option: save current configuration
 - devices: select all (BBR, FLASH, I2C-EEPROM, SPI-FLASH) but probably only gets saved to BBR (Battery Backed RAM) which will loose the settings eventually.

Disconnect, reconnect, wait up to 30 seconds and receiver output should be according new settings.

If it didn't work, try a factory reset: in CFG set "Revert to default configuration" and press send.


## Locosys MC-1513

```
$PMTK251,115200*1F //baud rate 115200
$PMTK314,10,1,10,1,10,10,10,0,0,0,0,0,0,0,0,0,0,0,0*19 //GLL,RMC,VTG,GGA,GSA,GSV,ZDA message rates
$PMTK220,100*2F //100ms interval
```

## AI-Thinker GT-01-KIT / Quectel L76K

Setup of AT6558R based GPS modules such as AI-Thinker GT-01-KIT / Quectel L76K.

```
//start at 9600 baud
$PCAS01,5*19  //baud rate 1115200
//change to 115200 baud
$PCAS03,1,0,0,0,1,0,0,0,0,0,,,0,0*02 //Enable GGA, RMC
$PCAS02,100*1E //Positioning interval: 100ms
$PCAS04,7*1E //GPS + BeiDou + GLONASS + QZSS
```

The settings appear to be saved to at least battery backed RAM, as the settings survived a short disconnect.

### Set baud rate

`$PCAS01,<CMD>*<Checksum>`

<CMD>The following baud rates are supported:  
0 = 4800  
1 = 9600  
2 = 19200  
3 = 38400  
4 = 57600  
5 = 115200  


```$PCAS01,0*1C //baud rate 4800
$PCAS01,1*1D //baud rate 9600
$PCAS01,5*19 //baud rate 115200```

### Set Positioning frequency

```$PCAS02,<Interval>*<Checksum>

$PCAS02,1000*2E //Positioning interval: 1000ms
$PCAS02,200*1D //Positioning interval: 200ms
$PCAS02,100*1E //Positioning interval: 100ms```

### Set NMEA sentence output type and output frequency

```$PCAS03,<nGGA>,<nGLL>,<nGSA>,<nGSV>,<nRMC>,<nVTG>,<nZDA>,<nANT>,<Res>,<Res>,<Res>,<Res >,<Res>,<Res>*<Checksum>

$PCAS03,1,1,1,1,1,1,1,1,0,0,,,0,0*02 //Enable GGA, GLL, GSA, GSV, RMC, VTG, ZDA, ANT
$PCAS03,1,10,10,10,1,10,10,10,0,0,,,0,0*02 //Enable GGA, RMC every interval, rest every 10 intervals
$PCAS03,1,0,0,0,1,0,0,0,0,0,,,0,0*02 //Enable GGA, RMC```

### Set Mode

`$PCAS04,<Mode>*<Checksum>`

<Mode>GNSS configuration:  
1 = GPS  
2 = BeiDou  
3 = GPS + BeiDou (default)  
4 = GLONASS  
5 = GPS + GLONASS  
6 = BeiDou + GLONASS  
7 = GPS + BeiDou + GLONASS  
(QZSS is enabled by default and does not support configuration.)

```$PCAS04,3*1A //3 = GPS + BeiDou (default)
$PCAS04,7*1E //7 = GPS + BeiDou + GLONASS```

### Restart Module

`$PCAS10,<Flag>*<Checksum>`

<Flag>Restart mode:  
0 = warm start  
1 = warm start  
2 = cold start  
3 = Cold boot and factory reset  

```$PCAS10,0*1C //warm start```