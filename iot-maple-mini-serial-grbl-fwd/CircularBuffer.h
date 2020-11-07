#ifndef _CIRCULAR_BUFFER_H
#define _CIRCULAR_BUFFER_H

#include <Stream.h>

// can read what you print using given backing circular buffer
class CircularBuffer : public Stream
{
    uint8_t *buf;
    size_t bufSize;
    size_t bufHead = 0;
    size_t bufTail = 0;

public:
    CircularBuffer(uint8_t *_buf, size_t _bufSize)
    {
        buf = _buf;
        bufSize = _bufSize;
    }

    // retrieve bytes inserted in the circular buffer
    int available()
    {
        if (bufHead != bufTail)
        {
            if (bufHead < bufTail)
                return bufTail - bufHead;
            else
                return bufSize - bufHead + bufHead;
        }

        return 0;
    }

    void flush()
    {
    }

    int read()
    {
        if (bufHead != bufTail)
        {
            int res = buf[bufHead++];

            if (bufHead == bufSize)
                bufHead = 0;

            return res;
        }

        return -1;
    }

    int peek()
    {
        if (bufHead != bufTail)
        {
            int res = buf[bufHead];

            return res;
        }

        return -1;
    }

    size_t write(uint8_t c)
    {
        size_t nextBufTail = bufTail + 1;
        if (nextBufTail == bufSize)
            nextBufTail = 0;

        if (nextBufTail == bufHead)
        {
            setWriteError();
            return 0;
        }
        else
        {
            buf[bufTail] = c;
            bufTail = nextBufTail;
        }

        return 1;
    }

    size_t write(const uint8_t *data, size_t size)
    {
        if (size == 0)
            return 0;

        size_t res = 0;
        while (size--)
        {
            uint8_t c = *data++;

            {
                size_t nextBufTail = bufTail + 1;
                if (nextBufTail == bufSize)
                    nextBufTail = 0;

                if (nextBufTail == bufHead)
                {
                    setWriteError();
                    break;
                }
                else
                {
                    ++res;
                    buf[bufTail] = c;
                    bufTail = nextBufTail;
                }
            }
        }
        return res;
    }
};

#endif
