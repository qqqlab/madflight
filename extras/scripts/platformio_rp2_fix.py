Import("env")

# Apply -Wno-volatile only to C++ files to silence C++20 warnings in FreeRTOS portmacro.h
env.Append(CXXFLAGS=["-Wno-volatile"])
