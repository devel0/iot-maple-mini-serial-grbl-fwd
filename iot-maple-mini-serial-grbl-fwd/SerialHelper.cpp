#include "SerialHelper.h"

#define BACKSPACE_CHAR 8
#define XON_CHAR 17
#define XOFF_CHAR 19

SerialHelper::SerialHelper(Stream &_stream, SerialLineProc _proc, bool _handleBackspace, bool _xonXoffSupport) : stream(_stream)
{
    proc = _proc;
    handleBackspace = _handleBackspace;
    xonXoffSupport = _xonXoffSupport;
}

bool SerialHelper::isBusy() const { return busy; }

void SerialHelper::evalRx()
{
    if (stream.available())
    {
        char c = stream.read();
        if (xonXoffSupport)
        {
            if (busy)
            {
                if (c == XON_CHAR)
                    busy = false;
            }
            else
            {
                if (c == XOFF_CHAR)
                    busy = true;
            }
        }
        bool newLineChar = c == '\r' || c == '\n';
        bool isBackspace = c == BACKSPACE_CHAR;
        if (isBackspace && handleBackspace)
        {
            if (line.length() > 0)
            {
                stream.write(' ');
                stream.write(BACKSPACE_CHAR);
                line.remove(line.length() - 1);
            }
            else
            {
                stream.write("\e[C");
            }
        }
        else if (newLineChar)
        {
            if (!insideNewline)
            {
                insideNewline = true;
                proc(line);
                line = "";
            }
        }
        else
        {
            if (insideNewline)
            {
                insideNewline = false;
            }

            line += c;
        }
    }
}

void SerialHelper::println(String &line)
{
    stream.println(line);
}

void SerialHelper::printMultiple(char c, int times)
{
    while (times--)
        stream.print(c);
}