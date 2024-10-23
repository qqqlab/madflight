betaflight_target_converter.py converts betaflight targets to madflight board definitions.

To convert a new batch of files: 
 - Copy the betaflight targets from https://github.com/betaflight/unified-targets/tree/master/configs/default to be converted to the betaflight_source sub directory.
 - Execute betaflight_target_converter.py from this directory.
 - Converted Betaflight targets are placed in the ../../src folder. The conversion is not perfect, but gives working base to start from. Include (or copy/paste) the converted header file in madflight.ino