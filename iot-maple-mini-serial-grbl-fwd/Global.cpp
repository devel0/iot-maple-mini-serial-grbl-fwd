#include "Global.h"

#include "SDCARD.h"
#include "MemoryInfo.h"
#include "SList.h"
#include "Utils.h"

#include "CircularBuffer.h"

#include <StreamUtils.h>

#define BACKSPACE_CHAR 8

const char *FWDSTATE_PATHFILENAME = "/fwdstate.txt";
const char *G61_RESPONSE_TOKEN = "Restoring position S0 X";

StateEnum state = StateEnum::Setup;

int speedUpReqCnt = 0;
int speedDownReqCnt = 0;
bool querySpeedSent = false;
bool querySpeedReceived = false;
int currentSpeedPercent = 100;

//bool pauseResumeInProgress = false;

int marlin_cmds_avail = MARLIN_BUFSIZE;

HardwareSerial Serial2(UART_SECOND_RX, UART_SECOND_TX);

uint32_t lastRx1Timestamp = 0;
char rx1Buf[UART_FIFO_SIZE];
int rx1BufHead = 0;
int rx1BufTail = 0;

char rx1Line[RX_LINE_SIZE];
int rx1LineOff = 0;
bool rx1LineWaitingSend = false;
bool executingScript = false;

uint32_t lastRx2Timestamp = 0;
char rx2Buf[UART_FIFO_SIZE];
int rx2BufHead = 0;
int rx2BufTail = 0;

char _deferredSerialOutBuf[UART_FIFO_SIZE];
CircularBuffer dSerial((uint8_t *)_deferredSerialOutBuf, UART_FIFO_SIZE);

char rx2Line[RX_LINE_SIZE];
int rx2LineOff = 0;

char tmpstr[RX_LINE_SIZE];

bool inhibitM114LogResult = false;

File f;
bool fOpened = false;
uint32_t fOff = 0;
uint32_t fOffSent = 0;
uint32_t fSize = 0;
String fToSend = "";
char fBuf[F_BUFFER_SIZE];
int fBufHead = 0;
int fBufTail = 0;
int fPercentPrint = 0;
bool fPercentPrintInitialDone = false;
// updated when percent print report
// to get actual print time use (fPrintTimeSecs + timeDiff(millis(), fPrintTimestamp) / 1000)
uint32_t fPrintTimestamp = 0;
int fPrintTimeSecs = 0;
bool fBufInsideNewline = false;

char fLine[RX_LINE_SIZE];
int fLineOff = 0;
bool fLineWaitingToBeSend = false;
int fOffIfWaitingLineSent = 0;

String parsedFName = "";
uint32_t parsedFOff = 0;

float parsedLocalX, parsedLocalY, parsedLocalZ;
float parsedGlobalX, parsedGlobalY, parsedGlobalZ;
float toolChangeNewGlobalZ;
bool toolChangeNewGlobalZSet = false;
bool parsedLocalAbs = true;
int parsedPrintTimeSecs = 0;
int parsedPrintPercent = 0;
String parsedTool = "";
bool parsedToolchangeInProgress = false;

bool debugSerial2Out = false;

void printHelp();
void printVersion();
void doReset();
void printSDCardFiles();
String getInfo();
void executeScript(const char *s);
void readFileInto(File &f, const String &filename, String &dest);
void dSerialPrintGlobalAndLocal();
void notifyNotSynced();
// check sd card init otherwise print error message; return true if ok
bool check_init_sdcard();
// try parse current pos from G61 response formatted like Restoring position S0 Xxx Yyy Zzz
// precondition: s must start with "Restoring position S0 X"
// store result in queriedPos{X,Y,Z}
bool tryParseCurrentPos(const char *s, float *x, float *y, float *z);

// X:0.00 Y:0.00 Z:200.00 E:0.00 Count A:29856 B:29856 C:29856
bool tryParseM114CurrentPos(const char *s, float *x, float *y, float *z);

void saveFwdState();
bool parseStateFile(const char *s);
void cleanParsed();

// prerequisite: marlin_cmds_avail>0
// sent string to serial2 and decrement avail marlin_cmds_avail
void Serial2_println(const char *s);

// prerequisite: marlin_cmds_avail>0
// sent string to serial2 and decrement avail marlin_cmds_avail
void Serial2_println(const String &s);

void doSetup()
{
    Serial.println("Initializing...");
    Serial2_println("G90");

    // wellcome msg
    Serial.println("Serial gcode forwarder ready (? for help)\n");

    ledOff();

    state = StateEnum::Normal;
}

#define BUF_SYNCED() (                     \
    marlin_cmds_avail == MARLIN_BUFSIZE && \
    rx1BufHead == rx1BufTail &&            \
    rx2BufHead == rx2BufTail)

#define SYNCED() (BUF_SYNCED() && state == StateEnum::Normal)

void loop()
{
    //
    // read from serial 2 if any
    //

    if (Serial2.available())
    {
        ledOn();
        do
        {
            bool some = false;
            while (Serial2.available())
            {
                char c = Serial2.read();
                rx2Buf[rx2BufTail++] = c;
                some = true;
                if (rx2BufTail == UART_FIFO_SIZE)
                    rx2BufTail = 0;
            }

            if (some)
                lastRx2Timestamp = millis();
        } while (timeDiff(millis(), lastRx2Timestamp) < SERIAL_READ_TIMEOUT_MS);
        ledOff();
    }

    //
    // flush rx2screen if any
    //

    if (dSerial.available())
    {
        Serial.write((char)dSerial.read());
    }

    //
    // read from serial 1 if any
    //

    if (Serial.available())
    {
        do
        {
            bool some = false;
            while (Serial.available())
            {
                char c = Serial.read();
                rx1Buf[rx1BufTail++] = c;
                some = true;
                if (rx1BufTail == UART_FIFO_SIZE)
                    rx1BufTail = 0;
            }

            if (some)
                lastRx1Timestamp = millis();
        } while (timeDiff(millis(), lastRx1Timestamp) < SERIAL_READ_TIMEOUT_MS);
    }

    //
    // evaluate data read from serial2
    //

    {
        int rx2BufHeadB_Backup = rx2BufHead;

        while (rx2BufHead != rx2BufTail)
        {
            uint8_t c = rx2Buf[rx2BufHead++];
            if (rx2BufHead == UART_FIFO_SIZE)
                rx2BufHead = 0;

            if (c == 10 || c == 13)
            {
                int rx2LineLen = rx2LineOff;
                rx2Line[rx2LineOff] = 0;
                rx2LineOff = 0;
                rx2BufHeadB_Backup = rx2BufHead;

                if (rx2LineLen > 0)
                {
                    //
                    // ok Pxx Byy
                    //
                    if (rx2Line[0] == 'o' && rx2Line[1] == 'k')
                    {
                        if (rx2Line[2] != ' ')
                        {
                            Serial.println("System halted: ADVANCED_OK required on Marlin side");
                            while (1)
                                ;
                        }
                        int Bfree = -1;
                        int i = 3;
                        while (i < rx2LineLen)
                        {
                            if (rx2Line[i] == ' ' && rx2Line[i + 1] == 'B')
                            {
                                int from = i + 2;
                                int to = from;
                                while (to < rx2LineLen && isdigit(rx2Line[to]))
                                    ++to;
                                if (from != to)
                                {
                                    Bfree = atoin(rx2Line + from, to - from);
                                }
                                break;
                            }
                            ++i;
                        }

                        if (Bfree == -1)
                        {
                            Serial.print("System halted: not found 'ok Pxx Byy' pattern on [");
                            Serial.print(rx2Line);
                            Serial.println("]");
                            while (1)
                                ;
                        }

                        ++marlin_cmds_avail;
                    }

                    //
                    // FR:xxx%
                    //
                    else if (querySpeedSent && strncmp(rx2Line, "FR:", 3) == 0)
                    {
                        String s = "";
                        char *ptr = rx2Line + 3;
                        while (true)
                        {
                            char c = *ptr++;
                            if (c == 0 || c == '%')
                                break;
                            s += c;
                        }
                        currentSpeedPercent = s.toInt();
                        querySpeedReceived = true;
                    }

                    //
                    // parse local pos ( result of M114 )
                    // format: "X:0.00 Y:0.00 Z:200.00 E:0.00 Count A:29856 B:29856 C:29856"
                    //
                    else if (
                        (state == StateEnum::SendSDStartingQueryLocalPosSent ||
                         state == StateEnum::SendSDPausingQueryCurrentLocalPosSent) &&
                        strncmp(rx2Line, "X:", 2) == 0)
                    {
                        float x, y, z;
                        if (tryParseM114CurrentPos(rx2Line, &x, &y, &z))
                        {
                            if (state == StateEnum::SendSDStartingQueryLocalPosSent &&
                                (x != 0 || y != 0 || z != 0))
                            {
                                dSerial.print("(W) : Current coord ");
                                dSerial.print(x);
                                dSerial.print(',');
                                dSerial.print(y);
                                dSerial.print(',');
                                dSerial.print(z);
                                dSerial.println(" isn't 0,0,0");
                                dSerial.println("Use /zero to set current coord as 0,0,0 and /send again,");
                                dSerial.println("or use /unsafesend to disable this check");

                                state = StateEnum::Normal;
                            }
                            else
                            {
                                parsedLocalX = x;
                                parsedLocalY = y;
                                parsedLocalZ = z;

                                switch (state)
                                {
                                case StateEnum::SendSDStartingQueryLocalPosSent:
                                    state = StateEnum::SendSDBegin;
                                    break;

                                case StateEnum::SendSDPausingQueryCurrentLocalPosSent:
                                    state = StateEnum::SendSDPausingQueryCurrentGlobalPos;
                                    break;
                                }
                            }
                        }
                        else
                        {
                            dSerial.println("error parsing local pos");
                            state = StateEnum::Normal;
                        }
                    }

                    //
                    // parse global position ( result of G60\nG61XYZ\n )
                    // format: "Restoring position S0 X0.00 Y0.00 Z530.00"
                    //
                    else if (state == StateEnum::SendSDPausingQueryCurrentGlobalPosSent &&
                             strncmp(rx2Line, G61_RESPONSE_TOKEN, 23) == 0)
                    {
                        String s = "";
                        float x, y, z;

                        if (tryParseCurrentPos(rx2Line, &x, &y, &z))
                        {
                            parsedGlobalX = x;
                            parsedGlobalY = y;
                            parsedGlobalZ = z;

                            state = StateEnum::SendSDPaused;
                        }
                        else
                        {
                            dSerial.println("error parsing global pos");
                            state = StateEnum::Normal;
                        }
                    }

                    //
                    // echo:busy: processing
                    //
                    else if (strcmp(rx2Line, "echo:busy: processing") == 0)
                    {
                    }

                    //
                    // ( unmanaged rx2 output )
                    //
                    else
                    {
                        if (inhibitM114LogResult && (rx2Line[0] == 'X' && rx2Line[1] == ':'))
                        {
                            inhibitM114LogResult = true;
                        }
                        else
                            // will flushed to screen 1 char at main loop to
                            // avoid writing serial1 interrupt important rx from serial2
                            dSerial.println(rx2Line);
                    }
                }

                while (rx2BufHead != rx2BufTail)
                {
                    c = rx2Buf[rx2BufHead];
                    if (c == 10 || c == 13)
                    {
                        ++rx2BufHead;
                        if (rx2BufHead == UART_FIFO_SIZE)
                            rx2BufHead = 0;
                    }
                    else
                        break;
                }
            }
            else
            {
                rx2Line[rx2LineOff++] = c;
            }
        }
    }

    //
    // buf synced processes
    //
    if (BUF_SYNCED())
    {
        if (executingScript)
        {
            dSerial.println("Done.\n");

            executingScript = false;
        }

        switch (state)
        {

        case StateEnum::SendSDStartingQueryLocalPos:
        {
            inhibitM114LogResult = true;
            executeScript("M114\n");

            state = StateEnum::SendSDStartingQueryLocalPosSent;
        }
        break;

        case StateEnum::SendSDPausingQueryCurrentLocalPos:
        {
            inhibitM114LogResult = true;
            executeScript("M114\n");

            state = StateEnum::SendSDPausingQueryCurrentLocalPosSent;
        }
        break;

        case StateEnum::SendSDPausingQueryCurrentGlobalPos:
        {
            executeScript("G60\nG61XYZ\n");

            state = StateEnum::SendSDPausingQueryCurrentGlobalPosSent;
        }
        break;

        case StateEnum::SendSDBegin:
        {
            if (check_init_sdcard())
            {
                if (fOpened)
                    f.close();
                f = SD.open(fToSend.c_str());
                if (f)
                {
                    fOpened = true;

                    fSize = f.size();
                    fPercentPrint = 0;
                    fPercentPrintInitialDone = false;
                    fPrintTimeSecs = 0;
                    fPrintTimestamp = millis();

                    dSerial.print("Sending SD file [");
                    dSerial.print(tmpstr);
                    dSerial.print("] size:");
                    printHumanSize(dSerial, fSize);
                    dSerial.println();

                    fOff = 0;
                    fOffSent = 0;
                    fBufHead = fBufTail = 0;

                    state = StateEnum::SendSD;
                }
                else
                {
                    dSerial.print("error opening file [");
                    dSerial.print(tmpstr);
                    dSerial.println("]\n");
                }
            }
            else
                state = StateEnum::Normal;
        }
        break;

        case StateEnum::ResumeHoming:
        {

            dSerial.print("Enter y to resume position ");
            dSerialPrintGlobalAndLocal();
            dSerial.println("\nor n to cancel resume process");
            state = StateEnum::ResumeStartPositionAsk;
        }
        break;

        case StateEnum::ResumePosition:
        {
            dSerial.print("Resuming position ");
            dSerialPrintGlobalAndLocal();
            dSerial.println();

            // G90
            String s = "G90\n";

            // G0 XaaYbb
            s += "G0 X";
            s += String(parsedGlobalX);
            s += "Y";
            s += String(parsedGlobalY);
            s += "\n";

            // G0 Zcc
            s += "G0 Z";
            s += String(parsedGlobalZ);
            s += "\n";

            // G92XaaYbbZcc
            s += "G92X";
            s += String(parsedLocalX);
            s += "Y";
            s += String(parsedLocalY);
            s += "Z";
            s += String(parsedLocalZ);
            s += "\n";

            // G90/G91
            if (parsedLocalAbs)
                s += "G90\n";
            else
                s += "G91\n";

            executeScript(s.c_str());

            dSerial.println("Enter y to resume sdfile exec, n to cancel resume process");
            state = StateEnum::ResumeStartSDFileAsk;
        }
        break;

        case StateEnum::ResumeStartSDFile:
        {
            if (fOpened)
                f.close();

            f = SD.open(parsedFName);
            if (f)
            {
                fOpened = true;

                fOff = parsedFOff;
                f.seek(fOff);

                dSerial.print("Resuming from offset ");
                dSerial.println(fOff);

                fSize = f.size();
                fPercentPrint = parsedPrintPercent;
                fPercentPrintInitialDone = false;
                fPrintTimeSecs = parsedPrintTimeSecs;
                fPrintTimestamp = millis();

                fOffSent = fOff;
                fBufHead = fBufTail = 0;

                state = StateEnum::SendSD;
            }
        }
        break;

        case StateEnum::Aborting:
        {
            state = StateEnum::Normal;
            dSerial.println("Aborted.\n");
        }
        break;
        }

        // if (pauseResumeInProgress)
        // {
        //     pauseResumeInProgress = false;
        //     dSerial.println("Done.\n");
        // }
    }

    //
    // process interleaved commands
    //
    if (marlin_cmds_avail > 0)
    {
        //
        // check for speed variation request
        //
        int variation = speedUpReqCnt + speedDownReqCnt;

        if (variation != 0)
        {
            if (!querySpeedSent)
            {
                Serial2_println("M220");

                querySpeedSent = true;
                querySpeedReceived = false;
            }
            else if (querySpeedReceived)
            {
                int newSpeed = max(currentSpeedPercent + variation * 10, MIN_SPEED_PERCENT);

                dSerial.print("set speed to ");
                dSerial.print(newSpeed);
                dSerial.println("%");

                speedUpReqCnt = speedDownReqCnt = 0;
                String s = "M220 S";
                s += String(newSpeed);

                Serial2_println(s);

                querySpeedSent = querySpeedReceived = false;
            }
        }
    }

    //
    // process console commands
    //

    if (rx1LineWaitingSend)
    {
        if (marlin_cmds_avail > 0)
        {
            Serial2_println(rx1Line);
            rx1LineWaitingSend = false;
        }
    }
    else
    {
        while (rx1BufHead != rx1BufTail)
        {
            uint8_t c = rx1Buf[rx1BufHead++];
            if (rx1BufHead == UART_FIFO_SIZE)
                rx1BufHead = 0;

            if (c == 10 || c == 13)
            {
                int rx1LineLen = rx1LineOff;
                rx1Line[rx1LineOff] = 0;
                rx1LineOff = 0;

                if (rx1LineLen > 0)
                {
                    int start = 0;
                    while (start < rx1LineLen && rx1Line[start] == ' ')
                        ++start;

                    //
                    // empty
                    //
                    if (start == rx1LineLen)
                    {
                    }

                    //
                    // resume position y/n
                    //
                    else if (state == StateEnum::ResumeStartPositionAsk && BUF_SYNCED())
                    {
                        if (strcmp(rx1Line + start, "y") == 0)
                        {
                            if (parsedToolchangeInProgress)
                                parsedToolchangeInProgress = false;
                            state = StateEnum::ResumePosition;
                        }
                        else if (strcmp(rx1Line + start, "n") == 0)
                        {
                            state = StateEnum::Normal;
                            dSerial.println("Resume process aborted.\n");
                        }
                        else
                        {
                            dSerial.println("y/n required");
                        }
                    }

                    //
                    // resume sd file exec y/n
                    //
                    else if (state == StateEnum::ResumeStartSDFileAsk && BUF_SYNCED())
                    {
                        if (strcmp(rx1Line + start, "y") == 0)
                        {
                            state = StateEnum::ResumeStartSDFile;
                        }
                        else if (strcmp(rx1Line + start, "n") == 0)
                        {
                            state = StateEnum::Normal;
                            dSerial.println("Resume process aborted.\n");
                        }
                        else
                        {
                            dSerial.println("y/n required");
                        }
                    }

                    //
                    // help
                    //
                    else if (rx1Line[start] == '?')
                    {
                        printHelp();
                    }

                    //
                    // sys / command
                    //
                    else if (rx1Line[start] == '/')
                    {
                        //
                        // ver
                        //
                        if (strcmp(rx1Line + start + 1, "ver") == 0)
                        {
                            dSerial.print(VERSION_NAME_STR);
                            dSerial.print(" ");
                            dSerial.println(VERSION_NR_STR);
                        }

                        //
                        // ls
                        //
                        else if (strcmp(rx1Line + start + 1, "ls") == 0)
                        {
                            if (state == StateEnum::SendSD)
                            {
                                dSerial.println("can't list files during sd execution");
                            }
                            else
                            {
                                printSDCardFiles();
                            }
                        }

                        //
                        // home
                        //
                        else if (strcmp(rx1Line + start + 1, "home") == 0)
                        {
                            if (!SYNCED())
                            {
                                notifyNotSynced();
                            }
                            else
                            {
                                if (parsedToolchangeInProgress)
                                {
                                    dSerial.println("Issueing a light home ( without safe zone ) to change tool");
                                    executeScript(HOMING_SCRIPT_TOOLCHANGE);
                                }
                                else
                                    executeScript(HOMING_SCRIPT);

                                if (parsedToolchangeInProgress)
                                {
                                    parsedToolchangeInProgress = false;

                                    dSerial.println("===> Change tool NOW then issue a /resume when ready");
                                    dSerial.println("     hint: use /toolzdiff to specify a tool length change");
                                }
                            }
                        }

                        //
                        // zero
                        //
                        else if (strcmp(rx1Line + start + 1, "zero") == 0)
                        {
                            if (!SYNCED())
                            {
                                notifyNotSynced();
                            }
                            else
                            {
                                executeScript(ZERO_SCRIPT);
                            }
                        }

                        //
                        // send
                        //
                        else if (strncmp(rx1Line + start + 1, "send ", 5) == 0)
                        {
                            if (!SYNCED())
                            {
                                notifyNotSynced();
                            }
                            else
                            {
                                cleanParsed();

                                const char *ptr = rx1Line + start + 6;
                                int p = 0;

                                while (*ptr)
                                    tmpstr[p++] = (uint8_t)*ptr++;
                                tmpstr[p] = 0;

                                fToSend = tmpstr;

                                state = StateEnum::SendSDStartingQueryLocalPos;
                            }
                        }

                        //
                        // toolzdiff <diff>
                        //
                        else if (strncmp(rx1Line + start + 1, "toolzdiff ", 10) == 0)
                        {
                            if (!SYNCED())
                            {
                                notifyNotSynced();
                            }
                            else
                            {
                                {
                                    if (check_init_sdcard())
                                    {
                                        if (fOpened)
                                            f.close();
                                        f = SD.open(FWDSTATE_PATHFILENAME);
                                        if (f)
                                        {
                                            fOpened = true;

                                            String str = "";
                                            readFileInto(f, String(FWDSTATE_PATHFILENAME), str);

                                            cleanParsed();

                                            if (parseStateFile(str.c_str()))
                                            {
                                                const char *ptr = rx1Line + start + 10;
                                                float val = strPtrGetFloatWhileDigits(&ptr);
                                                Serial.print("you inserted ");
                                                Serial.println(val);
                                                toolChangeNewGlobalZ = parsedGlobalZ + val;
                                                Serial.print("global z will tricked from [");
                                                Serial.print(parsedGlobalZ);
                                                Serial.print("] to [");
                                                Serial.print(toolChangeNewGlobalZ);
                                                Serial.println("]");
                                                toolChangeNewGlobalZSet = true;
                                                fOffSent = parsedFOff;
                                                fPrintTimeSecs = parsedPrintTimeSecs;
                                                fPercentPrint = parsedPrintPercent;
                                                Serial.println("issue a /save to confirm changes, then /resume to start with new tool");
                                            }
                                            else
                                            {
                                                dSerial.println("Parse error");

                                                state = StateEnum::Normal;
                                            }

                                            f.close();
                                            fOpened = false;
                                        }
                                        else
                                        {
                                            dSerial.print(FWDSTATE_PATHFILENAME);
                                            dSerial.println(" not available");
                                        }
                                    }
                                }
                            }
                        }

                        //
                        // more
                        //
                        else if (strncmp(rx1Line + start + 1, "more ", 5) == 0)
                        {
                            if (!SYNCED())
                            {
                                notifyNotSynced();
                            }
                            else
                            {

                                if (check_init_sdcard())
                                {
                                    const char *pfname = rx1Line + start + 1 + 5;
                                    if (fOpened)
                                        f.close();
                                    f = SD.open(pfname);
                                    Serial.print("Open file ");
                                    Serial.println(pfname);

                                    if (f)
                                    {
                                        fOpened = true;

                                        fOff = 0;
                                        fBufHead = fBufTail = 0;
                                        fBufInsideNewline = false;
                                        fLineOff = 0;
                                        int printedLines = 0;

                                        while (true)
                                        {
                                            if (printedLines >= MORE_LINES_MAX)
                                            {
                                                Serial.print("-- Hit a key to continue or 'q' to exit");
                                                bool qpressed = false;
                                                while (true)
                                                {
                                                    if (Serial.available())
                                                    {
                                                        char c = Serial.read();
                                                        if (c == 'q')
                                                            qpressed = true;
                                                        printedLines = 0;
                                                        break;
                                                    }
                                                }
                                                Serial.print("\r                                        ");
                                                Serial.print('\r');
                                                if (qpressed)
                                                {
                                                    Serial.println();
                                                    break;
                                                }
                                            }
                                            if (fBufHead == fBufTail)
                                            {
                                                int freadSize = f.read(fBuf, F_BUFFER_SIZE);
                                                if (freadSize < 0)
                                                {
                                                    Serial.println("System halted: error during SD file read");
                                                    while (1)
                                                        ;
                                                }

                                                if (freadSize == 0)
                                                {
                                                    Serial.println("<EOF>\n");
                                                    break;
                                                }
                                                else
                                                {
                                                    fBufHead = 0;
                                                    fBufTail = freadSize;
                                                }
                                            }
                                            else
                                            {
                                                while (fBufHead != fBufTail)
                                                {
                                                    char c = fBuf[fBufHead++];
                                                    ++fOff;
                                                    if (c == 10 || c == 13)
                                                    {
                                                        if (fBufInsideNewline)
                                                            continue;

                                                        fBufInsideNewline = true;
                                                        fLine[fLineOff] = 0;
                                                        Serial.println(fLine);
                                                        fLineOff = 0;
                                                        ++printedLines;
                                                        break;
                                                    }
                                                    else
                                                    {
                                                        if (fBufInsideNewline)
                                                            fBufInsideNewline = false;
                                                        fLine[fLineOff++] = c;
                                                    }
                                                }
                                            }
                                        }

                                        f.close();
                                        fOpened = false;
                                    }
                                    else
                                    {
                                        Serial.print("error opening file [");
                                        Serial.print(tmpstr);
                                        Serial.println("]");
                                    }
                                }
                            }
                        }

                        //
                        // reset
                        //
                        else if (strcmp(rx1Line + start + 1, "reset") == 0)
                        {
                            doReset();
                        }

                        //
                        // pause
                        //
                        else if (strcmp(rx1Line + start + 1, "pause") == 0)
                        {
                            doPauseResume();
                        }

                        //
                        // save
                        //
                        else if (strcmp(rx1Line + start + 1, "save") == 0)
                        {
                            if (state != StateEnum::SendSDPaused && state != StateEnum::Normal)
                            {
                                dSerial.println("save requires a paused job");
                            }
                            else if (!BUF_SYNCED())
                            {
                                dSerial.println("Sync required, buffer not yet flushed");
                            }
                            else
                            {
                                saveFwdState();
                            }
                        }

                        //
                        // resume
                        //
                        else if (strcmp(rx1Line + start + 1, "resume") == 0)
                        {
                            if (!SYNCED())
                            {
                                notifyNotSynced();
                            }
                            else if (parsedToolchangeInProgress)
                            {
                                dSerial.println("Tool change in progress, issue /home to change tool");
                            }
                            else if (check_init_sdcard())
                            {
                                if (fOpened)
                                    f.close();
                                f = SD.open(FWDSTATE_PATHFILENAME);
                                if (f)
                                {
                                    fOpened = true;

                                    String str = "";
                                    readFileInto(f, String(FWDSTATE_PATHFILENAME), str);

                                    cleanParsed();

                                    if (parseStateFile(str.c_str()))
                                    {
                                        state = StateEnum::ResumeHoming;

                                        executeScript(HOMING_SCRIPT);
                                    }
                                    else
                                    {
                                        dSerial.println("Parse error");

                                        state = StateEnum::Normal;
                                    }

                                    f.close();
                                    fOpened = false;
                                }
                                else
                                {
                                    dSerial.print(FWDSTATE_PATHFILENAME);
                                    dSerial.println(" not available");
                                }
                            }
                        }

                        //
                        // abort
                        //
                        else if (strcmp(rx1Line + start + 1, "abort") == 0)
                        {
                            if (state != StateEnum::SendSD && state != StateEnum::SendSDPaused)
                            {
                                dSerial.println("abort job required a print in progress or paused");
                            }
                            else
                            {
                                dSerial.print("Aborting in progress...");
                                state = StateEnum::Aborting;
                            }
                        }

                        //
                        // info
                        //
                        else if (strcmp(rx1Line + start + 1, "info") == 0)
                        {
                            dSerial.print(getInfo());
                        }

                        //
                        // debug on
                        //
                        else if (strcmp(rx1Line + start + 1, "debug on") == 0)
                        {
                            debugSerial2Out = true;
                            dSerial.println("debug gcode enabled");
                        }

                        //
                        // debug off
                        //
                        else if (strcmp(rx1Line + start + 1, "debug off") == 0)
                        {
                            debugSerial2Out = false;
                            dSerial.println("debug gcode disabled");
                        }

                        //
                        // (cmd to forward to grbl controller)
                        //
                        else
                        {
                            printHelp();

                            if (rx1Line[start + 1] != '?')
                            {
                                dSerial.print("unknown console cmd [");
                                dSerial.print(rx1Line + start);
                                dSerial.println("]");
                            }
                        }
                    }
                    else
                    {
                        if (marlin_cmds_avail > 0)
                        {
                            Serial2_println(rx1Line + start);
                        }
                        else
                            rx1LineWaitingSend = true;
                    }
                }

                while (rx1BufHead != rx1BufTail)
                {
                    c = rx1Buf[rx1BufHead];
                    if (c == 10 || c == 13)
                    {
                        ++rx1BufHead;
                        if (rx1BufHead == UART_FIFO_SIZE)
                            rx1BufHead = 0;
                    }
                    else
                        break;
                }

                if (rx1LineWaitingSend)
                    break;
            }
            else
            {
                if (c == BACKSPACE_CHAR)
                {
                    if (rx1LineOff > 0)
                        --rx1LineOff;
                }
                else
                    rx1Line[rx1LineOff++] = c;
            }
        }
    }

    //
    // read SD if any
    //
    if (state == StateEnum::SendSD)
    {
        if (fBufHead == fBufTail)
        {
            int freadSize = f.read(fBuf, F_BUFFER_SIZE);
            if (freadSize < 0)
            {
                Serial.println("System halted: error during SD file read");
                while (1)
                    ;
            }

            if (freadSize == 0)
            {
                fPrintTimeSecs += timeDiff(millis(), fPrintTimestamp) / 1000;
                fPercentPrint = 100;
                dSerial.print("p:");
                dSerial.print(fPercentPrint);
                dSerial.print("%  total:");
                printHumanSeconds(dSerial, fPrintTimeSecs);
                dSerial.println();

                dSerial.println("File sent");
                state = StateEnum::Normal;
            }
            else
            {
                fBufHead = 0;
                fBufTail = freadSize;
            }
        }
        else if (!fLineWaitingToBeSend)
        {
            while (fBufHead != fBufTail)
            {
                char c = fBuf[fBufHead++];
                ++fOff;
                if (c == 10 || c == 13)
                {
                    fBufInsideNewline = true;
                    fLine[fLineOff] = 0;
                    fLineWaitingToBeSend = true;
                    fOffIfWaitingLineSent = fOff;

                    break;
                }
                else
                {
                    if (fBufInsideNewline)
                        fBufInsideNewline = false;
                    fLine[fLineOff++] = c;
                }
            }
        }

        if (fLineWaitingToBeSend && marlin_cmds_avail > 0)
        {
            uint32_t T = timeDiff(millis(), fPrintTimestamp) / 1000;
            if (fPercentPrintInitialDone || T > F_PERCENT_STAT_MINSEC)
            {
                uint32_t progress =
                    ((int)((float)fOffSent / fSize * 100.0)) / F_PERCENT_PRINT_BASE * F_PERCENT_PRINT_BASE;

                if (progress > fPercentPrint || !fPercentPrintInitialDone)
                {
                    fPrintTimeSecs += T;
                    fPercentPrint = progress;

                    uint32_t remainSecs = (fSize - fOffSent) * fPrintTimeSecs / fOffSent;

                    dSerial.print("p:");
                    dSerial.print(fPercentPrint);
                    dSerial.print("%   elapsed:");
                    printHumanSeconds(dSerial, (uint32_t)fPrintTimeSecs);
                    dSerial.print("   remain:");
                    printHumanSeconds(dSerial, remainSecs);
                    dSerial.println();

                    fPrintTimestamp = millis();
                    if (!fPercentPrintInitialDone)
                        fPercentPrintInitialDone = true;
                }
            }

            Serial2_println(fLine);
            fLineOff = 0;

            fOffSent = fOffIfWaitingLineSent;
            fLineWaitingToBeSend = false;
        }
    }
}

// precondition: SYNCED()
void executeScript(const char *s)
{
    executingScript = true;

    int slen = strlen(s);
    for (int i = 0; i < slen; ++i)
    {
        rx1Buf[rx1BufTail++] = s[i];
        if (rx1BufTail == UART_FIFO_SIZE)
            rx1BufTail = 0;
    }
}

void printHelp()
{
    dSerial.println();
    dSerial.println("Syntax");
    for (int i = 0; i < 70; ++i)
        dSerial.print('-');
    dSerial.println();
    dSerial.println("/ver                display version");
    dSerial.println("/ls                 list sdcard content");
    dSerial.println("/home               do homing G28, go to safe zone and switch to G54 working");
    dSerial.println("/zero               set zero 0,0,0 here");
    dSerial.println("/send <file>        load sdcard file and send to gcode controller");
    dSerial.println("/more <file>        view file content");
    dSerial.println("/reset              reset gcode controller");
    dSerial.println("/pause              pause/resume gcode controller print");
    dSerial.println("/save               save paused printing job");
    dSerial.println("/resume             resume saved printing job");
    dSerial.println("/abort              cancel current sd print");
    dSerial.println("/info               sys info");
    dSerial.println("/toolzdiff <diff>   specify new installed tool <diff> len; + if longer, - if shorter");
    dSerial.println("/debug <on|off>     enable/disable debug");    
    dSerial.println();
    dSerial.println("GCode ref examples");
    for (int i = 0; i < 70; ++i)
        dSerial.print('-');
    dSerial.println();
    dSerial.println("M114           report current position");
    dSerial.println("G28            homing");
    dSerial.println("M503           report robot settings");
    dSerial.println("G92 X0 Y0 Z0   set here as origin X0 Y0 Z0");
    dSerial.println("M220 S200      set speed 200%");
    dSerial.println();
}

String getInfo()
{
    StringPrint ss;

    ss.println();

    ss.print("marlin_cmds_avail: ");
    ss.println(marlin_cmds_avail);

    ss.print("buf(h,t) rx1/rx2/f: (");
    ss.print(rx1BufHead);
    ss.print(",");
    ss.print(rx1BufTail);
    ss.print(")/(");
    ss.print(rx2BufHead);
    ss.print(",");
    ss.print(rx2BufTail);
    ss.print(")/(");
    ss.print(fBufHead);
    ss.print(",");
    ss.print(fBufTail);
    ss.println(")\n");

    ss.print("state: ");
    switch (state)
    {
    case StateEnum::Setup:
        ss.println("Setup");
        break;

    case StateEnum::Normal:
        ss.println("Normal");
        break;

    case StateEnum::Aborting:
        ss.println("Aborting");
        break;

    case StateEnum::SendSDStartingQueryLocalPos:
        ss.println("SendSDStartingQueryLocalPos");
        break;

    case StateEnum::SendSDStartingQueryLocalPosSent:
        ss.println("SendSDStartingQueryLocalPosSent");
        break;

    case StateEnum::SendSDBegin:
        ss.println("SendSDBegin");
        break;

    case StateEnum::SendSD:
        ss.println("SendSD");
        break;

    case StateEnum::SendSDPausingQueryCurrentLocalPos:
        ss.println("SendSDPausingQueryCurrentLocalPos");
        break;

    case StateEnum::SendSDPausingQueryCurrentLocalPosSent:
        ss.println("SendSDPausingQueryCurrentLocalPosSent");
        break;

    case StateEnum::SendSDPausingQueryCurrentGlobalPos:
        ss.println("SendSDPausingQueryCurrentGlobalPos");
        break;

    case StateEnum::SendSDPausingQueryCurrentGlobalPosSent:
        ss.println("SendSDPausingQueryCurrentGlobalPosSent");
        break;

    case StateEnum::SendSDPaused:
        ss.println("SendSDPaused");
        break;

    case StateEnum::ResumeHoming:
        ss.println("ResumeHoming");
        break;

    case StateEnum::ResumeStartPositionAsk:
        ss.println("ResumeStartPositionAsk");
        break;

    case StateEnum::ResumePosition:
        ss.println("ResumePosition");
        break;

    case StateEnum::ResumeStartSDFileAsk:
        ss.println("ResumeStartSDFileAsk");
        break;

    case StateEnum::ResumeStartSDFile:
        ss.println("ResumeStartSDFile");
        break;

    default:
        ss.println(state);
        break;
    }

    ss.print("local coord: X");
    ss.print(parsedLocalX);
    ss.print(" Y");
    ss.print(parsedLocalY);
    ss.print(" Z");
    ss.println(parsedLocalZ);

    ss.print("local mode: ");
    if (parsedLocalAbs)
        ss.println("G90");
    else
        ss.println("G91");

    ss.print("global coord: X");
    ss.print(parsedGlobalX);
    ss.print(" Y");
    ss.print(parsedGlobalY);
    ss.print(" Z");
    ss.println(parsedGlobalZ);

    ss.print("current tool: ");
    ss.println(parsedTool);

    ss.print("tool change: ");
    ss.println(parsedToolchangeInProgress);

    ss.print("file offset: ");
    printHumanSize(ss, fOffSent);
    ss.print(" of ");
    printHumanSize(ss, fSize);
    ss.println();

    ss.print("print time: ");
    printHumanSeconds(ss, fPrintTimeSecs > 0 ? (fPrintTimeSecs + timeDiff(millis(), fPrintTimestamp) / 1000) : 0);
    ss.println();

    ss.print("sys uptime: ");
    printHumanSeconds(ss, millis() / 1000);
    ss.println();

    ss.print("mem free: ");
    ss.println(freeMemory());

    ss.println();

    return ss.str();
}

void dSerialPrintGlobalAndLocal()
{
    dSerial.print("global:(");
    dSerial.print(parsedGlobalX);
    dSerial.print(",");
    dSerial.print(parsedGlobalY);
    dSerial.print(",");
    dSerial.print(parsedGlobalZ);
    dSerial.print(") local:(");
    dSerial.print(parsedLocalX);
    dSerial.print(",");
    dSerial.print(parsedLocalY);
    dSerial.print(",");
    dSerial.print(parsedLocalZ);
    dSerial.print(") ");
    if (parsedLocalAbs)
        dSerial.print(" absMode");
    else
        dSerial.print(" relMode");
}

void readFileInto(File &f, const String &filename, String &dest);

// precondition: state not in SDPrint because share same buffer
void readFileInto(File &f, const String &filename, String &dest)
{
    StringPrint ss;

    // Serial.print("loading ");
    // Serial.print(filename);
    // Serial.print(" from sdcard...");

    while (true)
    {
        int freadSize = f.read(fBuf, F_BUFFER_SIZE);
        if (freadSize < 0)
        {
            Serial.println("System halted: error during SD file read");
            while (1)
                ;
        }
        if (freadSize > 0)
        {
            char *p = fBuf;
            while (freadSize--)
            {
                ss.print(*p++);
            }
        }
        else
        {
            break;
        }
    }

    dest = ss.str();
}

void doReset()
{
    if (millis() > 1000)
    {
        Serial.println(">> RESET");
        NVIC_SystemReset();
    }
}

void doPauseResume()
{
    if (state == StateEnum::SendSD)
    {
        dSerial.println("Pausing...");
        state = StateEnum::SendSDPausingQueryCurrentLocalPos;
    }
    else if (state == StateEnum::SendSDPaused)
    {
        dSerial.println("Resuming...");
        state = StateEnum::SendSD;
    }
    //pauseResumeInProgress = true;
}

void doSpeedUp()
{
    ++speedUpReqCnt;
    dSerial.print("speed ");
    int variation = speedUpReqCnt + speedDownReqCnt;
    if (variation > 0)
        dSerial.print('+');

    dSerial.print(variation * 10);
    dSerial.println('%');
}

void doSpeedDown()
{
    --speedDownReqCnt;
    dSerial.print("speed ");
    int variation = speedUpReqCnt + speedDownReqCnt;
    if (variation > 0)
        dSerial.print('+');

    dSerial.print(variation * 10);
    dSerial.println('%');
}

void printSDCardFiles()
{
    if (check_init_sdcard())
        sdcard_ls();
}

void notifyNotSynced()
{
    dSerial.println("Sync required, buffer not yet flushed");
    if (state == StateEnum::SendSDPaused)
    {
        dSerial.println("SD print in progress, either abort or unpause\n");
    }
}

void toggleLed()
{
    digitalWrite(LED_BUILTIN, digitalRead(LED_BUILTIN) == HIGH ? LOW : HIGH);
}

void ledOn()
{
    digitalWrite(LED_BUILTIN, HIGH);
}

void ledOff()
{
    digitalWrite(LED_BUILTIN, LOW);
}

bool check_init_sdcard()
{
    if (!init_sd_card())
    {
        dSerial.println("(W): SD card init failed\n");
        return false;
    }
    return true;
}

bool tryParseCurrentPos(const char *s, float *x, float *y, float *z)
{
    const char *ptr = s + 23;

    bool parseError = false;

    *x = strPtrGetFloatWhileDigits(&ptr);

    if (*ptr != ' ' || *(ptr + 1) != 'Y')
    {
        parseError = true;
    }
    else
    {
        ptr += 2;
        *y = strPtrGetFloatWhileDigits(&ptr);

        if (*ptr != ' ' || *(ptr + 1) != 'Z')
        {
            parseError = true;
        }
        else
        {
            ptr += 2;
            *z = strPtrGetFloatWhileDigits(&ptr);

            if (*ptr != 0)
            {
                parseError = true;
            }
        }
    }

    if (parseError)
    {
        dSerial.println("Received inconsistent current pos string");
        dSerial.println(s);
        dSerial.println("Expecting a format like the follow:");
        dSerial.println("Restoring position S0 X0.00 Y0.00 Z530.00");
    }
    else
    {
        dSerial.print("Acquired global X");
        dSerial.print(*x);
        dSerial.print(" Y");
        dSerial.print(*y);
        dSerial.print(" Z");
        dSerial.println(*z);
    }

    return !parseError;
}

// precondition s format:
// X:0.00 Y:0.00 Z:200.00 E:0.00 Count A:29856 B:29856 C:29856
bool tryParseM114CurrentPos(const char *s, float *x, float *y, float *z)
{
    const char *ptr = s + 2;
    bool parseError = false;

    *x = strPtrGetFloatWhileDigits(&ptr);

    if (*ptr != ' ' || *(ptr + 1) != 'Y' || *(ptr + 2) != ':')
    {
        parseError = true;
    }
    else
    {
        ptr += 3;
        *y = strPtrGetFloatWhileDigits(&ptr);

        if (*ptr != ' ' || *(ptr + 1) != 'Z' || *(ptr + 2) != ':')
        {
            parseError = true;
        }
        else
        {
            ptr += 3;
            *z = strPtrGetFloatWhileDigits(&ptr);
        }
    }

    if (parseError)
    {
        dSerial.println("Received inconsistent current pos M114 string");
        dSerial.println(s);
        dSerial.println("Expecting a format like the follow:");
        dSerial.println("X:0.00 Y:0.00 Z:200.00 E:0.00 Count A:29856 B:29856 C:29856");
    }
    else
    {
        dSerial.print("Acquired local X");
        dSerial.print(*x);
        dSerial.print(" Y");
        dSerial.print(*y);
        dSerial.print(" Z");
        dSerial.println(*z);
    }

    return !parseError;
}

void saveFwdState()
{
    if (state != StateEnum::SendSDPaused && state != StateEnum::Normal)
    {
        Serial.println("System halted: state should SendSDPaused");
        while (1)
            ;
    }
    if (!fOpened && state == SendSDPaused)
    {
        Serial.println("System halted: file not opened");
        while (1)
            ;
    }

    String fname;
    if (state == SendSDPaused)
    {
        fname = f.name();
        f.close();
    }
    else
    {
        fname = parsedFName;
    }
    fOpened = false;

    if (SD.exists(FWDSTATE_PATHFILENAME))
    {
        SD.remove(FWDSTATE_PATHFILENAME);
    }

    if (fOpened)
        f.close();
    f = SD.open(FWDSTATE_PATHFILENAME, FILE_WRITE);
    if (f)
    {
        fOpened = true;

        f.print("local X");
        f.print(parsedLocalX);
        f.print(" Y");
        f.print(parsedLocalY);
        f.print(" Z");
        f.print(parsedLocalZ);
        f.println();

        f.print("posmode ");
        if (parsedLocalAbs)
            f.println("G90");
        else
            f.println("G91");

        f.print("global X");
        f.print(parsedGlobalX);
        f.print(" Y");
        f.print(parsedGlobalY);
        f.print(" Z");
        if (toolChangeNewGlobalZSet)
            f.print(toolChangeNewGlobalZ);
        else
            f.print(parsedGlobalZ);
        f.println();

        f.print("filename ");
        f.println(fname);

        f.print("foffset ");
        f.println(fOffSent);

        f.print("printPercent ");
        f.println(fPercentPrint);

        f.print("printTimeSecs ");
        f.println(fPrintTimeSecs);

        f.print("tool ");
        f.println(parsedTool);

        f.print("toolchange ");
        if (parsedToolchangeInProgress)
            f.println("1");
        else
            f.println("0");

        f.close();
        fOpened = false;

        if (parsedToolchangeInProgress)
        {
            dSerial.println("Issue a /home to change the tool");
        }
        else
            dSerial.println("Issue a /resume to restart print");
    }
    else
    {
        Serial.println("System halted: error during create /fwdstate.txt");
        while (1)
            ;
    }

    state = StateEnum::Normal;

    dSerial.println("Job stopped, Current state: Normal\n");
}

bool parseStateFile(const char *s)
{
    int l = strlen(s);
    const char *ptr = s;

    if (strncmp(ptr, "local X", 7) != 0)
        return false;
    else
    {
        ptr += 7;
        parsedLocalX = strPtrGetFloatWhileDigits(&ptr);

        if (*ptr != ' ' && *(ptr + 1) != 'Y')
            return false;
        else
        {
            ptr += 2;
            parsedLocalY = strPtrGetFloatWhileDigits(&ptr);

            if (*ptr != ' ' && *(ptr + 1) != 'Z')
                return false;
            else
            {
                ptr += 2;
                parsedLocalZ = strPtrGetFloatWhileDigits(&ptr);

                if (*ptr != 13 && *(ptr + 1) != 10)
                    return false;
                else
                    ptr += 2;
            }
        }
    }

    if (strncmp(ptr, "posmode G9", 10) != 0)
        return false;
    else
    {
        ptr += 10;
        if (*ptr == '1')
            parsedLocalAbs = false;
        ++ptr;

        if (*ptr != 13 && *(ptr + 1) != 10)
            return false;
        else
            ptr += 2;
    }

    if (strncmp(ptr, "global X", 8) != 0)
        return false;
    else
    {
        ptr += 8;
        parsedGlobalX = strPtrGetFloatWhileDigits(&ptr);

        if (*ptr != ' ' && *(ptr + 1) != 'Y')
            return false;
        else
        {
            ptr += 2;
            parsedGlobalY = strPtrGetFloatWhileDigits(&ptr);

            if (*ptr != ' ' && *(ptr + 1) != 'Z')
                return false;
            else
            {
                ptr += 2;
                parsedGlobalZ = strPtrGetFloatWhileDigits(&ptr);

                if (*ptr != 13 && *(ptr + 1) != 10)
                    return false;
                else
                    ptr += 2;
            }
        }
    }

    if (strncmp(ptr, "filename ", 9) != 0)
        return false;
    else
    {
        ptr += 9;
        while (*ptr)
        {
            char c = *ptr;
            if (c == 13)
                break;
            ++ptr;
            parsedFName += c;
        }
        if (*ptr != 13 && *(ptr + 1) != 10)
            return false;
        else
            ptr += 2;
    }

    if (strncmp(ptr, "foffset ", 8) != 0)
        return false;
    else
    {
        ptr += 8;
        String s = "";
        while (*ptr)
        {
            char c = *ptr;
            if (c == 13)
                break;
            ++ptr;
            s += c;
        }
        if (*ptr != 13 && *(ptr + 1) != 10)
            return false;
        else
        {
            ptr += 2;
            parsedFOff = s.toInt();
        }
    }

    if (strncmp(ptr, "printPercent ", 13) != 0)
        return false;
    else
    {
        ptr += 13;
        String s = "";
        while (*ptr)
        {
            char c = *ptr;
            if (c == 13)
                break;
            ++ptr;
            s += c;
        }
        if (*ptr != 13 && *(ptr + 1) != 10)
            return false;
        else
        {
            ptr += 2;
            parsedPrintPercent = s.toInt();
        }
    }

    if (strncmp(ptr, "printTimeSecs ", 14) != 0)
        return false;
    else
    {
        ptr += 14;
        String s = "";
        while (*ptr)
        {
            char c = *ptr;
            if (c == 13)
                break;
            ++ptr;
            s += c;
        }
        if (*ptr != 13 && *(ptr + 1) != 10)
            return false;
        else
        {
            ptr += 2;
            parsedPrintTimeSecs = s.toInt();
        }
    }

    if (strncmp(ptr, "tool ", 5) != 0)
        return false;
    else
    {
        ptr += 5;
        String s = "";
        while (*ptr)
        {
            char c = *ptr;
            if (c == 13)
                break;
            ++ptr;
            s += c;
        }
        if (*ptr != 13 && *(ptr + 1) != 10)
            return false;
        else
        {
            ptr += 2;
            parsedTool = s;
        }
    }

    if (strncmp(ptr, "toolchange ", 11) != 0)
        return false;
    else
    {
        ptr += 11;
        parsedToolchangeInProgress = *ptr == '1';
        ++ptr;
        if (*ptr != 13 && *(ptr + 1) != 10)
            return false;
        else
            ptr += 2;
    }

    return true;
}

void cleanParsed()
{
    parsedFName = "";
    parsedFOff = 0;
    parsedLocalAbs = true;
    parsedTool = "";
    parsedToolchangeInProgress = false;
    parsedLocalX = parsedLocalY = parsedLocalZ = 0.0;
    parsedGlobalX = parsedGlobalY = parsedGlobalZ = 0.0;
    toolChangeNewGlobalZSet = false;
}

void Serial2_println(const char *s)
{
    if (debugSerial2Out)
    {
        dSerial.printf("fOff:%lu ", fOff);
        dSerial.println(s);
    }

    // preprocess gcode
    {
        // trim begin
        while (s[0] == ' ')
            s++;

        //
        // empty/comment
        //
        if (s[0] == 0 || s[0] == ';' || s[0] == '(' || s[0] == '%')
        {
            return;
        }
        //
        // prepend G1 for commands that starts with direct coordinate
        //
        else if (s[0] == 'X' || s[0] == 'Y' || s[0] == 'Z')
            Serial2.print("G1");
        //
        // M6 change tool
        //
        else if (s[0] == 'M' && s[1] == '6' && s[2] == ' ')
        {
            int i = 3;
            parsedTool = "";
            while (true)
            {
                char c = s[i++];
                if (c == 0)
                    break;
                parsedTool += c;
            }
            doPauseResume();

            dSerial.print("\n===> Tool change [");
            dSerial.print(s + 3);
            dSerial.println("] requested.");
            dSerial.println("Now paused, please issue /save to continue tool change process.");

            parsedToolchangeInProgress = true;

            return;
        }
        //
        // G90 positioning mode absolute
        //
        else if (!parsedLocalAbs && s[0] == 'G' && s[1] == '9' && s[1] == '0')
        {
            parsedLocalAbs = true;
        }
        //
        // G91 positioning mode relative
        //
        else if (parsedLocalAbs && s[0] == 'G' && s[1] == '9' && s[1] == '1')
        {
            parsedLocalAbs = false;
        }
    }

    Serial2.println(s);
    --marlin_cmds_avail;
}

void Serial2_println(const String &s)
{
    Serial2_println(s.c_str());
}
