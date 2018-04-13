//
// Created by David Kadish on 12/02/2018.
//

#ifndef RUBE_LOGGING_H
#define RUBE_LOGGING_H

#ifdef LOGGING

#define FLOAT(fl_var) (int)(fl_var), abs((int)((fl_var)*100.0 - ((float)((int)(fl_var)*100))))

#if LOGGING >= 1
    #define FATAL(str, ...) \
       Serial.print("FATAL ("); \
       Serial.print(millis()); \
       Serial.print(" ms): "); \
       Serial.print(__FUNCTION__); \
       Serial.print("() in "); \
       Serial.print(__FILE__); \
       Serial.print(':'); \
       Serial.print(__LINE__); \
       Serial.print(' '); \
       Serial.printf(str, ##__VA_ARGS__); \
       Serial.println();
#endif
#if LOGGING >= 2
    #define ERROR(str, ...) \
       Serial.print("ERROR ("); \
       Serial.print(millis()); \
       Serial.print(" ms): "); \
       Serial.print(__FUNCTION__); \
       Serial.print("() in "); \
       Serial.print(__FILE__); \
       Serial.print(':'); \
       Serial.print(__LINE__); \
       Serial.print(' '); \
       Serial.printf(str, ##__VA_ARGS__); \
       Serial.println();
#endif
#if LOGGING >= 3
    #define WARNING(str, ...) \
       Serial.print("WARNING ("); \
       Serial.print(millis()); \
       Serial.print(" ms): "); \
       Serial.print(__FUNCTION__); \
       Serial.print("() in "); \
       Serial.print(__FILE__); \
       Serial.print(':'); \
       Serial.print(__LINE__); \
       Serial.print(' '); \
       Serial.printf(str, ##__VA_ARGS__); \
       Serial.println();
#endif
#if LOGGING >= 4
    #define INFO(str, ...) \
       Serial.print("INFO ("); \
       Serial.print(millis()); \
       Serial.print(" ms): "); \
       Serial.print(__FUNCTION__); \
       Serial.print("() in "); \
       Serial.print(__FILE__); \
       Serial.print(':'); \
       Serial.print(__LINE__); \
       Serial.print(' '); \
       Serial.printf(str, ##__VA_ARGS__); \
       Serial.println();
#endif
#if LOGGING >= 5
    #define DEBUG(str, ...) \
       Serial.print("DEBUG ("); \
       Serial.print(millis()); \
       Serial.print(" ms): "); \
       Serial.print(__FUNCTION__); \
       Serial.print("() in "); \
       Serial.print(__FILE__); \
       Serial.print(':'); \
       Serial.print(__LINE__); \
       Serial.print(' '); \
       Serial.printf(str, ##__VA_ARGS__); \
       Serial.println();
#endif
#if LOGGING >= 6
    #define TRACE(str, ...) \
       Serial.print("TRACE ("); \
       Serial.print(millis()); \
       Serial.print(" ms): "); \
       Serial.print(__FUNCTION__); \
       Serial.print("() in "); \
       Serial.print(__FILE__); \
       Serial.print(':'); \
       Serial.print(__LINE__); \
       Serial.print(' '); \
       Serial.printf(str, ##__VA_ARGS__); \
       Serial.println();
#endif

#endif

#ifndef FATAL
    #define FATAL(str, ...)
#endif
#ifndef ERROR
    #define ERROR(str, ...)
#endif
#ifndef WARNING
    #define WARNING(str, ...)
#endif
#ifndef INFO
    #define INFO(str, ...)
#endif
#ifndef DEBUG
    #define DEBUG(str, ...)
#endif
#ifndef TRACE
    #define TRACE(str, ...)
#endif

#endif //RUBE_LOGGING_H
