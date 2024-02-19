//compiler preprocessor test, this setup is used for the modules header files like imu.h
//just compile and see if correct #error message appears

//test cases
//#define XXX_USE XXX_USE_TYPEA //expect "XXX_USE_TYPEA"
//#define XXX_USE XXX_USE_TYPEB //expect "XXX_USE_TYPEB"
//#define XXX_USE XXX_USE_TYPEB2 //expect "invalid"
//not defined XXX_USE expect "XXX_USE_NONE"

//option values can be defined after defining XXX_USE
#define XXX_USE_NONE 1
#define XXX_USE_TYPEA 2
#define XXX_USE_TYPEB 3

//=====================================================================================================================
// Type A
//=====================================================================================================================
#if XXX_USE == XXX_USE_TYPEA
  #error "XXX_USE_TYPEA"
  
//=====================================================================================================================
// Type B
//=====================================================================================================================
#elif XXX_USE == XXX_USE_TYPEB
  #error "XXX_USE_TYPEB"
  
//=====================================================================================================================
// None or undefined
//=====================================================================================================================
#elif XXX_USE == XXX_USE_NONE || !defined XXX_USE
  #error "XXX_USE_NONE"
  
//=====================================================================================================================
// Invalid value
//=====================================================================================================================
#else
  #error "invalid XXX_USE value"
#endif


void setup() {}
void loop() {}