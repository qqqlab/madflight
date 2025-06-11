import re
import os
import datetime 

DEBUG = False
#DEBUG = True

source_dirname = "betaflight_source" # copy from https://github.com/betaflight/unified-targets/tree/master/configs/default
destination_path = "../../src/brd/betaflight/"
destination_prefix = ""

CMT = " // "

def main() :
    for filename in os.listdir(source_dirname) :
        convert(filename)
        if DEBUG: return

def toInt(d) :
    try:
        return int(d)
    except ValueError:
        return 0

def convert(filename) :
    def toint(s) :
        try:
            return int(s)
        except ValueError:
            return 0
    
    #in and output files
    infile           = source_dirname + "/" + filename
    strippedfilename = re.sub(r"(.config$)", r"", filename)
    outfilename      = destination_prefix + strippedfilename + ".h"
    outfile          = destination_path + outfilename

    #read lines from infile
    f = open(infile,"r")
    lines = f.readlines()
    f.close()

    #don't parse files that are too short
    if len(lines)<10 : return

    #parse lines to topics
    defines = []
    board_name = ""
    manufacturer_id = ""

    for line in lines:
        line = line.strip();
        line = line.replace("\t" ," ")
        line = re.sub(" +", " ", line) #replace multiple spaces with one space 
        p = (line + "   ").split(" ") #add dummy spaces to fill at least p[4]
        for i in range(4): p[i] = p[i].strip()

        if p[0] == "#define" : # example: #define USE_ACC_SPI_MPU6000
            nme = p[1]
            val = p[2]
            if nme.startswith("USE_GYRO_SPI_") :
                val = nme.replace("USE_GYRO_SPI_", "") #remove prefix
                defines.append( "imu_gizmo " + val + CMT + line )
            elif nme.startswith("USE_BARO_") :
                val = nme.replace("USE_BARO_", "") #remove prefix
                defines.append( "bar_gizmo " + val + CMT + line )
            elif nme.startswith("USE_MAG_") :
                val = nme.replace("USE_MAG_", "") #remove prefix
                defines.append( "mag_gizmo " + val + CMT + line )
            #else:
                #defines.append( "# " + line )

        if p[0] == "board_name" : # example: board_name MATEKH743
            board_name = p[1]

        if p[0] == "manufacturer_id" : # example: manufacturer_id MTKS
            manufacturer_id = p[1] 

        if p[0] == "resource" : # example: resource SERIAL_TX 1 A09
            r = p[1]
            i = toInt(p[2]) # 1-based instance int
            bus = str(i - 1) # 0-based string
            pin = "P" + re.sub(r"0([0-9])", r"\1", p[3]) #convert pin name from C01 to PC1
            if   r == "SERIAL_TX" : defines.append( "pin_ser" + bus + "_tx " + pin + CMT + line )
            elif r == "SERIAL_RX" : defines.append( "pin_ser" + bus + "_rx " + pin + CMT + line )
            elif r == "INVERTER"  : defines.append( "pin_ser" + bus + "_inv " + pin + CMT + line )
            elif r == "SPI_SCK"   : defines.append( "pin_spi" + bus + "_sclk " + pin + CMT + line )
            elif r == "SPI_MISO"  : defines.append( "pin_spi" + bus + "_miso " + pin + CMT + line)
            elif r == "SPI_MOSI"  : defines.append( "pin_spi" + bus + "_mosi " + pin + CMT + line)
            elif r == "I2C_SCL"   : defines.append( "pin_i2c" + bus + "_scl " + pin + CMT + line )
            elif r == "I2C_SDA"   : defines.append( "pin_i2c" + bus + "_sda " + pin + CMT + line )
            elif r == "MOTOR"     : defines.append( "pin_out" + bus + " " + pin + CMT + line )
            elif r == "LED" and i==1 : defines.append( "pin_led " + pin + CMT + line )
            elif r == "PPM" and i==1 : defines.append( "pin_rcl_ppm " + pin + CMT + line )
            elif r == "ADC_BATT" and i==1 : 
                defines.append( "pin_bat_v " + pin + CMT + line )
                defines.append( "bat_gizmo ADC" )
            elif r == "ADC_CURR" and i==1 : 
                defines.append( "pin_bat_i " + pin + CMT + line )
                defines.append( "bat_gizmo ADC" )
            elif r == "SDIO_CK" and i==1 : defines.append( "pin_mmc_clk " + pin + CMT + line )
            elif r == "SDIO_CMD" and i==1 : defines.append( "pin_mmc_cmd " + pin + CMT + line )
            elif r == "SDIO_D0" and i==1 : 
                defines.append( "pin_mmc_dat " + pin + CMT + line )
                defines.append( "bbx_gizmo SDMMC" )
            elif r == "GYRO_EXTI" and i==1 : defines.append( "pin_imu_int " + pin + CMT + line )
            elif r == "GYRO_CS" and i==1 : defines.append( "pin_imu_cs " + pin + CMT + line )
            elif r == "SDCARD_CS" and i==1 : 
                defines.append( "pin_bbx_cs " + pin + CMT + line )
                defines.append( "bbx_gizmo SDSPI" )
            else :
                defines.append( CMT + line )

        if p[0] == "set" : #example: set mag_i2c_device = 1
            n = p[1]
            v = p[3]
            bus = str(toInt(v) - 1);
            if   n == "mag_i2c_device" : defines.append( "mag_i2c_bus " + bus + CMT + line )
            elif n == "baro_i2c_device" : defines.append( "bar_i2c_bus " + bus + CMT + line )
            #not needed --> elif n == "gyro_1_bustype" : defines.append( "imu_bus_type " + v + CMT + line )
            elif n == "gyro_1_spibus" : defines.append( "imu_spi_bus " + bus + CMT + line )
            elif n == "gyro_1_sensor_align" : defines.append( "imu_align " + v + CMT + line )
            elif n == "sdcard_spi_bus" : defines.append( "bbx_spi_bus " + bus + CMT + line )
            else :
                defines.append( CMT + line )

    #debug print topics
    # print( "defines:", defines )
    # print( "board_name:", board_name )
    # print( "manufacturer_id:", manufacturer_id )

    #output madflight target header file
    f = open(outfile,"w")

    def fprint(txt) :
        if DEBUG : print( txt )
        else : f.write(txt + "\n")

    fprint( "/*==============================================================================" )
    fprint( "Generated on: " + str(datetime.datetime.now()) )
    fprint( "Generated by: betaflight_target_converter.py" )
    fprint( "Source: https://github.com/betaflight/unified-targets" )
    fprint( "Board name: " + board_name )
    fprint( "Manufacturer ID: " + manufacturer_id )
    fprint( "" )
    fprint( "//copy this line to madflight.ino to use this flight controller" )
    fprint( "#define MF_BOARD \"brd/betaflight/" + outfilename + "\"" )
    fprint( "" )
    fprint( "Note: madflight will pick the last sensor defined here, this might not be the sensor that is actually on the board. Comment the offending sensors out, or use madflight_config to override." )
    fprint( "==============================================================================*/" )

    fprint( "" )
    fprint( "#define MF_BOARD_NAME \"BETAFLIGHT-" + strippedfilename + "\"" )
    mcu_re = re.search(r"\bSTM32\w+", lines[0] + " " + lines[1]);
    if mcu_re : fprint( "#define MF_MCU_NAME \"" + mcu_re.group() + "\"" )

    fprint( "" )

    fprint( 'const char madflight_board[] = R""(' )
    fprint( 'imu_bus_type SPI' )  #all bf controllers use SPI
    for define in defines:
        fprint( define.strip() )
    fprint( ')""; //end of madflight_board' )

    fprint( "" )
    fprint( "" )
    fprint( "/*" )
    fprint( "#==============================================================================" )
    fprint( "# BetaFlight Source file" )
    fprint( "#==============================================================================" )
    fprint( "".join(lines) )
    fprint( "*/" )

    f.close()

main()