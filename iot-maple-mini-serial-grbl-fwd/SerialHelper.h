#ifndef _SERIAL_HELPER
#define _SERIAL_HELPER

#include <Arduino.h>

typedef void (*SerialLineProc)(String &);

class SerialHelper
{
    Stream &stream;
    bool insideNewline = false;
    String line = "";
    SerialLineProc proc = NULL;
    bool handleBackspace;
    bool xonXoffSupport;
    bool busy;
public:
    SerialHelper(Stream &_stream, SerialLineProc proc, bool _handleBackspace = false, bool _xonXoffSupport = false);
    bool isBusy() const;
    void evalRx();
    void println(String &line);
    void printMultiple(char c, int times);
};

#endif
