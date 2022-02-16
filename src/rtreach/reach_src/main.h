#ifdef __cplusplus
extern "C" {
#endif

#ifndef MAIN_H_
#define MAIN_H_

#include <math.h>

#ifdef ARDUINO
#include <stdfix.h> // fixed point library
#endif

// to look at:
// mips: http://www.sniff.org.uk/2014/05/building-gcc-for-yun-in-12-not-easy.html
// mips: http://stackoverflow.com/questions/4751709/cross-compiling-for-mips-router-from-x86
// https://github.com/arduino/openwrt-yun
// https://github.com/makerbot/MightyBoardFirmware/blob/master/firmware/src/MightyBoard/Motherboard/avrfix/avrfix.h

// fixed point library: http://sourceforge.net/p/fixedptc/code/ci/default/tree/fixedptc.h
// other library: https://github.com/mbedded-ninja/MFixedPoint

// fixed point gcc: https://gcc.gnu.org/wiki/FixedPointArithmetic
// fixed point standard: http://www.open-std.org/jtc1/sc22/wg14/www/docs/n1169.pdf

// cross-platform reals as fixed point or floating point
#if defined(ARDUINO)
//#define _Accum long int // define fixed point length here (e.g., could use ints)
// note: _Accum == accum, defined in stdfix
// see here: C:\Program Files (x86)\Arduino\hardware\tools\avr\lib\gcc\avr\4.8.1\include
//#define REAL accum
//#define REAL float // software floats on AVR
#define REAL double // software floats on AVR
#else
#define REAL double
#endif

// weird C++ (.ino) vs. C problems, solution found here: http://allgood38.io/understand-arduino-development.html

//int main_avr( int argc, const char* argv[] );

int main_avr( int runtimeMs, REAL* startState );

#define DEBUG 1

// detect if we're in matlab using mex to compile
// http://stackoverflow.com/questions/24679744/check-if-compiling-with-matlab-mex
#ifdef MATLAB_MEX_FILE
	#define MATLAB 1
#endif

//
//#if !defined(MATLAB)
//	#if defined(ARDUINO)
//		extern int main_avr( int argc, const char* argv[] );
//	#else
//		extern int main( int argc, const char* argv[] );
//	#endif
//#endif
// http://stackoverflow.com/questions/1941307/c-debug-print-macros
#if DEBUG
	#define DEBUG_PRINT(...) do{ fprintf( stderr, __VA_ARGS__ ); } while( false )
#else
	#define DEBUG_PRINT(...) do{ } while ( false )
#endif

#endif // MAIN_H_


#ifdef __cplusplus
}
#endif
