#include "Global.h"

#include "SerialHelper.h"
#include "SDCARD.h"
#include "MemoryInfo.h"
#include "SList.h"
#include "Utils.h"

#include "CircularBuffer.h"
#include "ConfigData.h"

#include <StreamUtils.h>

#define BACKSPACE_CHAR 8

const char *FWDSTATE_PATHFILENAME = "/fwdstate.txt";

StateEnum state = StateEnum::Setup;

int speedUpReqCnt = 0;
int speedDownReqCnt = 0;
bool querySpeedSent = false;
bool querySpeedReceived = false;
int currentSpeedPercent = 100;

bool currentPositioningModeAbsolute = true;

bool pauseResumeInProgress = false;

bool queryPos = false;
bool queryPosSent = false;
bool queryPosReceived = false;
float queriedPosX, queriedPosY, queriedPosZ;

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

File f;
bool fOpened = false;
uint32_t fOff = 0;
uint32_t fOffSent = 0;
uint32_t fSize = 0;
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

String homing_script = "";
String zero_script = "";

String parsedFName = "";
uint32_t parsedFOff = 0;
bool parsedAbs = true;
float parsedX, parsedY, parsedZ;
int parsedPrintTimeSecs = 0;
int parsedPrintPercent = 0;

bool debugSerial2Out = false;

void printHelp();
void printVersion();
void doReset();
void printSDCardFiles();
void loadScripts();
String getInfo();
void executeScript(String &s);
void readFileInto(File &f, const String &filename, String &dest);
void notifyNotSynced();

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

    loadScripts();

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

void mainLoop()
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
                    // query pos
                    //
                    else if (queryPosSent && strncmp(rx2Line, "Restoring position S0 X", 23) == 0)
                    {
                        String s = "";
                        const char *ptr = rx2Line + 23;

                        bool parseError = false;

                        queriedPosX = strPtrGetFloatWhileDigits(&ptr);

                        if (*ptr != ' ' && *(ptr + 1) != 'Y')
                        {
                            parseError = true;
                        }
                        else
                        {
                            ptr += 2;
                            queriedPosY = strPtrGetFloatWhileDigits(&ptr);

                            if (*ptr != ' ' && *(ptr + 1) != 'Z')
                            {
                                parseError = true;
                            }
                            else
                            {
                                ptr += 2;
                                queriedPosZ = strPtrGetFloatWhileDigits(&ptr);

                                if (*ptr != 0)
                                {
                                    parseError = true;
                                }
                                else
                                {
                                    queryPosReceived = true;
                                }
                            }
                        }

                        queryPos = false;

                        if (parseError)
                        {
                            dSerial.println("Received inconsistent current pos string");
                            dSerial.println(rx2Line);
                            dSerial.println("Expecting a format like the follow:");
                            dSerial.println("Restoring position S0 X0.00 Y0.00 Z530.00");
                        }
                        else
                        {
                            dSerial.print("Acquired current X");
                            dSerial.print(queriedPosX);
                            dSerial.print(" Y");
                            dSerial.print(queriedPosY);
                            dSerial.print(" Z");
                            dSerial.println(queriedPosZ);

                            if (state != StateEnum::SendSDPaused)
                            {
                                Serial.println("System halted: state should SendSDPaused");
                                while (1)
                                    ;
                            }
                            if (!fOpened)
                            {
                                Serial.println("System halted: file not opened");
                                while (1)
                                    ;
                            }

                            String fname = f.name();
                            f.close();
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

                                f.print("curpos X");
                                f.print(queriedPosX);
                                f.print(" Y");
                                f.print(queriedPosY);
                                f.print(" Z");
                                f.print(queriedPosZ);
                                f.println();

                                f.print("posmode ");
                                if (currentPositioningModeAbsolute)
                                    f.println("G90");
                                else
                                    f.println("G91");

                                f.print("filename ");
                                f.println(fname);

                                f.print("foffset ");
                                f.println(fOffSent);

                                f.print("printPercent ");
                                f.println(fPercentPrint);

                                f.print("printTimeSecs ");
                                f.println(fPrintTimeSecs);

                                f.close();
                                fOpened = false;

                                dSerial.print("Saved print state to ");
                                dSerial.println(FWDSTATE_PATHFILENAME);
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
            executingScript = false;
            dSerial.println("Done.");
        }

        switch (state)
        {

        case StateEnum::ResumeHoming:
        {
            dSerial.println("Enter y to resume position, n to cancel resume process");
            state = StateEnum::ResumeStartPositionAsk;
        }
        break;

        case StateEnum::ResumePosition:
        {
            dSerial.println("Enter y to resume sdfile exec, n to cancel resume process");
            state = StateEnum::ResumeStartSDFileAsk;
        }
        break;

        case StateEnum::Aborting:
        {
            state = StateEnum::Normal;
            dSerial.println("Done.\n");
        }
        break;
        }

        if (pauseResumeInProgress)
        {
            pauseResumeInProgress = false;
            dSerial.println("Done.\n");
        }
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

        if (queryPos && marlin_cmds_avail == MARLIN_BUFSIZE)
        {
            if (!queryPosSent)
            {
                Serial2_println("G60");
                Serial2_println("G61 XYZ");                

                queryPosSent = true;
                queryPosReceived = false;
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

            if (currentPositioningModeAbsolute)
            {
                if (strncmp(rx1Line, "G91", 3) == 0)
                    currentPositioningModeAbsolute = false;
            }
            else
            {
                if (strncmp(rx1Line, "G90", 3) == 0)
                    currentPositioningModeAbsolute = true;
            }
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

                    else if (state == StateEnum::ResumeStartPositionAsk)
                    {
                        if (strcmp(rx1Line + start, "y") == 0)
                        {
                            dSerial.print("Resuming position X");
                            dSerial.print(parsedX);
                            dSerial.print(" Y");
                            dSerial.print(parsedY);
                            dSerial.print(" Z");
                            dSerial.println(parsedZ);

                            // G90
                            String s = "G90\n";

                            // G0 Zcc
                            s += "G0 Z";
                            s += String(parsedZ);
                            s += "\n";

                            // G0 XaaYbb
                            s += "G0 X";
                            s += String(parsedX);
                            s += "Y";
                            s += String(parsedY);
                            s += "\n";

                            // G90/G91
                            if (parsedAbs)
                                s += "G90\n";
                            else
                                s += "G91\n";

                            executeScript(s);
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

                    else if (state == StateEnum::ResumeStartSDFileAsk)
                    {
                        if (strcmp(rx1Line + start, "y") == 0)
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
                                fPercentPrint = 0;
                                fPercentPrintInitialDone = false;
                                fPrintTimeSecs = 0;
                                fPrintTimestamp = millis();

                                fOffSent = fOff;
                                fBufHead = fBufTail = 0;

                                state = StateEnum::SendSD;
                            }
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
                    // comment
                    //
                    else if (rx1Line[start] == ';' || rx1Line[start] == '(')
                    {
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
                                executeScript(homing_script);
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
                                executeScript(zero_script);
                            }
                        }

                        //
                        // send
                        //
                        else if (strncmp(rx1Line + start + 1, "send ", 5) == 0)
                        {
                            if (state == StateEnum::SendSD)
                            {
                                dSerial.println("Can't send during SD print");
                            }
                            else if (state == StateEnum::SendSDPaused)
                            {
                                dSerial.println("Can't send during SD paused");
                            }
                            else
                            {
                                const char *ptr = rx1Line + start + 6;
                                int p = 0;

                                while (*ptr)
                                    tmpstr[p++] = (uint8_t)*ptr++;
                                tmpstr[p] = 0;

                                if (init_sd_card())
                                {

                                    if (fOpened)
                                        f.close();
                                    f = SD.open(tmpstr);
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
                                        dSerial.println("]");
                                    }
                                }
                                else
                                {
                                    dSerial.println("init sd card failed");
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

                                if (init_sd_card())
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
                                else
                                {
                                    Serial.println("init sd card failed");
                                }
                            }
                        }

                        //
                        // scripts
                        //
                        else if (strcmp(rx1Line + start + 1, "scripts") == 0)
                        {
                            if (!SYNCED())
                            {
                                notifyNotSynced();
                            }
                            else
                            {
                                if (fOpened)
                                    f.close();
                                loadScripts();
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
                            if (state != StateEnum::SendSDPaused)
                            {
                                dSerial.println("save requires a paused job");
                            }
                            else if (!BUF_SYNCED())
                            {
                                dSerial.println("Sync required, buffer not yet flushed");
                            }
                            else if (queryPos)
                            {
                                dSerial.println("save already in progress");
                            }
                            else
                            {
                                queryPos = true;
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
                            else
                            {
                                if (fOpened)
                                    f.close();
                                f = SD.open(FWDSTATE_PATHFILENAME);
                                if (f)
                                {
                                    fOpened = true;

                                    String str = "";
                                    readFileInto(f, String(FWDSTATE_PATHFILENAME), str);

                                    parsedFName = "";
                                    parsedFOff = 0;
                                    parsedAbs = true;
                                    parsedX = parsedY = parsedZ = 0.0;

                                    int l = str.length();
                                    const char *ptr = str.c_str();

                                    bool parseError = false;

                                    if (strncmp(ptr, "curpos X", 8) != 0)
                                    {
                                        parseError = true;
                                    }
                                    else
                                    {
                                        ptr += 8;
                                        parsedX = strPtrGetFloatWhileDigits(&ptr);

                                        if (*ptr != ' ' && *(ptr + 1) != 'Y')
                                        {
                                            parseError = true;
                                        }
                                        else
                                        {
                                            ptr += 2;
                                            parsedY = strPtrGetFloatWhileDigits(&ptr);

                                            if (*ptr != ' ' && *(ptr + 1) != 'Z')
                                            {
                                                parseError = true;
                                            }
                                            else
                                            {
                                                ptr += 2;
                                                parsedZ = strPtrGetFloatWhileDigits(&ptr);

                                                if (*ptr != 13 && *(ptr + 1) != 10)
                                                    parseError = true;
                                                else
                                                    ptr += 2;
                                            }
                                        }
                                    }

                                    if (!parseError)
                                    {
                                        if (strncmp(ptr, "posmode G9", 10) != 0)
                                        {
                                            parseError = true;
                                        }
                                        else
                                        {
                                            ptr += 10;
                                            if (*ptr == '1')
                                                parsedAbs = false;
                                            ++ptr;

                                            if (*ptr != 13 && *(ptr + 1) != 10)
                                                parseError = true;
                                            else
                                                ptr += 2;
                                        }
                                    }

                                    if (!parseError)
                                    {
                                        if (strncmp(ptr, "filename ", 9) != 0)
                                        {
                                            parseError = true;
                                        }
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
                                                parseError = true;
                                            else
                                                ptr += 2;
                                        }
                                    }

                                    if (!parseError)
                                    {
                                        if (strncmp(ptr, "foffset ", 8) != 0)
                                        {
                                            parseError = true;
                                        }
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
                                                parseError = true;
                                            else
                                            {
                                                ptr += 2;
                                                parsedFOff = s.toInt();
                                            }
                                        }
                                    }

                                    if (!parseError)
                                    {
                                        if (strncmp(ptr, "printPercent ", 13) != 0)
                                        {
                                            parseError = true;
                                        }
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
                                                parseError = true;
                                            else
                                            {
                                                ptr += 2;
                                                parsedPrintPercent = s.toInt();
                                            }
                                        }
                                    }

                                    if (!parseError)
                                    {
                                        if (strncmp(ptr, "printTimeSecs ", 14) != 0)
                                        {
                                            parseError = true;
                                        }
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
                                                parseError = true;
                                            else
                                            {
                                                ptr += 2;
                                                parsedPrintTimeSecs = s.toInt();
                                            }
                                        }
                                    }

                                    if (parseError)
                                    {
                                        dSerial.println("Parse error");
                                    }
                                    else
                                    {
                                        state = StateEnum::ResumeHoming;

                                        executeScript(homing_script);
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

                            if (currentPositioningModeAbsolute)
                            {
                                if (strncmp(rx1Line + start, "G91", 3) == 0)
                                    currentPositioningModeAbsolute = false;
                            }
                            else
                            {
                                if (strncmp(rx1Line + start, "G90", 3) == 0)
                                    currentPositioningModeAbsolute = true;
                            }
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

            if (currentPositioningModeAbsolute)
            {
                if (strncmp(fLine, "G91", 3) == 0)
                {
                    currentPositioningModeAbsolute = false;
                }
            }
            else
            {
                if (strncmp(fLine, "G90", 3) == 0)
                {
                    currentPositioningModeAbsolute = true;
                }
            }
        }
    }
}

// precondition: SYNCED()
void executeScript(String &s)
{
    Serial.print("Script execution...");
    executingScript = true;

    int slen = s.length();
    for (int i = 0; i < slen; ++i)
    {
        rx1Buf[rx1BufTail++] = s[i];
        if (rx1BufTail == UART_FIFO_SIZE)
            rx1BufTail = 0;
    }
}

String getInfo()
{
    StringPrint ss;

    ss.print("state:");
    switch (state)
    {
    case StateEnum::Setup:
        ss.println("Setup");
        break;

    case StateEnum::Normal:
        ss.println("Normal");
        break;

    case StateEnum::SendSD:
        ss.println("SendSD");
        break;

    case StateEnum::SendSDPaused:
        ss.println("SendSDPaused");
        break;

    case StateEnum::Aborting:
        ss.println("Aborting");
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

    case StateEnum::Error:
        ss.println("Error");
        break;

    default:
        ss.println(state);
        break;
    }

    ss.println();

    ss.print("last acquired pos: X");
    ss.print(queriedPosX);
    ss.print(" Y");
    ss.print(queriedPosY);
    ss.print(" Z");
    ss.println(queriedPosZ);

    ss.print("positioning mode: ");
    if (currentPositioningModeAbsolute)
        ss.println("G90");
    else
        ss.println("G91");

    ss.print("file offset:");
    printHumanSize(ss, fOffSent);
    ss.print(" of ");
    printHumanSize(ss, fSize);
    ss.println();

    ss.print("print time:");
    printHumanSeconds(ss, fPrintTimeSecs > 0 ? (fPrintTimeSecs + timeDiff(millis(), fPrintTimestamp) / 1000) : 0);
    ss.println();

    ss.print("sys uptime:");
    printHumanSeconds(ss, millis() / 1000);
    ss.println();

    ss.println();

    ss.print("mem free:");
    ss.println(freeMemory());

    ss.print("marlin_cmds_avail:");
    ss.println(marlin_cmds_avail);

    ss.println();

    ss.print("rx1Buf ( head:");
    ss.print(rx1BufHead);
    ss.print(" tail:");
    ss.print(rx1BufTail);
    ss.println(" )");

    ss.print("rx2Buf ( head:");
    ss.print(rx2BufHead);
    ss.print(" tail:");
    ss.print(rx2BufTail);
    ss.println(" )");

    ss.print("fBuf ( head:");
    ss.print(fBufHead);
    ss.print(" tail:");
    ss.print(fBufTail);
    ss.println(" )");

    ss.println();

    return ss.str();
}

void printHelp()
{
    dSerial.println();
    dSerial.println("Syntax");
    for (int i = 0; i < 70; ++i)
        dSerial.print('-');
    dSerial.println();
    dSerial.println("/ver           display version");
    dSerial.println("/ls            list sdcard content");
    dSerial.println("/home          do homing G28, go to safe zone and switch to G54 working");
    dSerial.println("/zero          set zero 0,0,0 here");
    dSerial.println("/send <file>   load sdcard file and send to gcode controller");
    dSerial.println("/more <file>   view file content");
    dSerial.println("/scripts       reload scripts from sdcard");
    dSerial.println("/reset         reset gcode controller");
    dSerial.println("/pause         pause/resume gcode controller print");
    dSerial.println("/save          save paused printing job");
    dSerial.println("/resume        resume saved printing job");
    dSerial.println("/abort         cancel current sd print");
    dSerial.println("/info          sys info");
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

void readFileInto(File &f, const String &filename, String &dest);

// prerq: synced
void loadScripts()
{
    homing_script = "";
    zero_script = "";

    if (init_sd_card())
    {
        File fwdscr = SD.open("fwdscr");
        if (fwdscr)
        {
            if (fwdscr.isDirectory())
            {
                while (true)
                {
                    File entry = fwdscr.openNextFile();
                    if (!entry)
                    {
                        break;
                    }
                    if (!entry.isDirectory())
                    {
                        if (String(entry.name()).equalsIgnoreCase("homing.nc"))
                        {
                            readFileInto(entry, "homing.nc", homing_script);
                        }
                        else if (String(entry.name()).equalsIgnoreCase("zero.nc"))
                        {
                            readFileInto(entry, "zero.nc", zero_script);
                        }
                    }
                    entry.close();
                }
            }
            fwdscr.close();
        }
    }
    else
        Serial.println("sd card init failed");

    if (homing_script.length() == 0)
    {
        StringPrint ss;

        Serial.println("loaded default homing script");
        ss.println("G28");

        homing_script = ss.str();
    }

    if (zero_script.length() == 0)
    {
        StringPrint ss;

        Serial.println("loaded default zero script");
        ss.println("G92X0Y0Z0");

        zero_script = ss.str();
    }
}

// precondition: state not in SDPrint because share same buffer
void readFileInto(File &f, const String &filename, String &dest)
{
    StringPrint ss;

    Serial.print("loading ");
    Serial.print(filename);
    Serial.print(" from sdcard...");

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
            Serial.println("finished");
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
        dSerial.print("Pausing...");
        state = StateEnum::SendSDPaused;
    }
    else if (state == StateEnum::SendSDPaused)
    {
        dSerial.println("Resuming...");
        state = StateEnum::SendSD;
    }
    pauseResumeInProgress = true;
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
    if (init_sd_card())
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

void Serial2_println(const char *s)
{
    if (debugSerial2Out)
        dSerial.println(s);
    Serial2.println(s);
    --marlin_cmds_avail;
}

void Serial2_println(const String &s)
{
    if (debugSerial2Out)
        dSerial.println(s);
    Serial2.println(s);
    --marlin_cmds_avail;
}
