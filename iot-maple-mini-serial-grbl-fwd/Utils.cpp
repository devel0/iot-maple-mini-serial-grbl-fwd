#include "Utils.h"

#include <limits.h>

unsigned long timeDiff(unsigned long now, unsigned long start)
{
    if (start <= now)
        return now - start;
    else
        return (ULONG_MAX - start) + now + 1;
}

// convert string to int by given string length
int atoin(const char *str, int len)
{
    int res = 0;
    if (len > 0)
    {
        const char *p = str + len - 1;
        int base = 1;
        while (len > 0)
        {
            res += base * ((*str) - 48);
            base *= 10;
            --len;
        }
    }

    return res;
}

const uint32_t KSIZE = 1024;
const uint32_t MSIZE = 1024 * 1024;
const uint32_t GSIZE = 1024 * 1024 * 1024;

void printHumanSize(Print &stream, uint32_t size)
{
    if (size < KSIZE)
        stream.print(size);
    else if (size < MSIZE)
    {
        stream.print((float)size / KSIZE);
        stream.print('K');
    }
    else if (size < GSIZE)
    {
        stream.print((float)size / MSIZE);
        stream.print('M');
    }
    else
    {
        stream.print((float)size / GSIZE);
        stream.print('G');
    }
}

void printHumanSeconds(Print &stream, uint32_t secs)
{
    if (secs < 60)
    {
        stream.print(secs);
        stream.print('s');
    }
    else if (secs < 3600)
    {
        stream.print((float)secs / 60.0);
        stream.print('m');
    }
    else
    {
        stream.print((float)secs / 3600.0);
        stream.print('h');
    }
}

// skip beginning whitespaces then gather float nr and stop to first non digit or if str end
// side effect: ptr will increase to first non digit char occurrence
float strPtrGetFloatWhileDigits(const char **ptr)
{
    String s = "";
    while (**ptr == ' ')
        ++(*ptr);
    while (true)
    {
        char c = **ptr;
        if (c == 0)
            break;
        if (isdigit(c) || c == '.' || c == '-')
        {
            s += c;
        }
        else
            break;
        (*ptr)++;
    }
    if (s.length() == 0)
        return 0;

    return s.toFloat();
}
