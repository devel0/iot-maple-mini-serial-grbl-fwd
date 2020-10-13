#include "config.h"

#include <SPI.h>
#include <SD.h>

Sd2Card card;
SdVolume volume;
SdFile root;
File f;

String line;
uint8_t state = STATE_NORMAL;
bool paused = false;
int speedPercent = 100;
int prevSpeedPercent = 0;
unsigned long bootTimestamp = millis();

volatile int resetFlag = 0;
volatile int pauseFlag = 0;
volatile int speedUpFlag = 0;
volatile int speedDownFlag = 0;

unsigned long resetStart = 0;
unsigned long pausedStart = 0;
unsigned long speedUpStart = 0;
unsigned long speedDownStart = 0;

HardwareSerial Serial2(UART_SECOND_RX, UART_SECOND_TX);
unsigned long Serial2OkReceived = 0;

void init_sd_card()
{
    SD.begin();

    if (!card.init(SPI_HALF_SPEED, SDCARD_CHIPSEL))
    {
        while (1)
        {
            Serial.println("SD card init failed");
            delay(3000);
        }
    }
}

void sdcard_ls()
{
    Serial.print("\nCard type: ");

    switch (card.type())
    {

    case SD_CARD_TYPE_SD1:
        Serial.println("SD1");
        break;

    case SD_CARD_TYPE_SD2:
        Serial.println("SD2");
        break;

    case SD_CARD_TYPE_SDHC:
        Serial.println("SDHC");
        break;

    default:
        Serial.println("Unknown");
    }

    if (!volume.init(card))
    {
        Serial.println("FAT16/FAT32 not found");

        while (1)
            ;
    }

    uint32_t volumesize;

    Serial.print("Volume type: FAT");
    Serial.println(volume.fatType(), DEC);

    volumesize = volume.blocksPerCluster();
    volumesize *= volume.clusterCount();
    volumesize /= 2; // 1kb = 2 blocks x 512byte

    volumesize /= 1024;

    Serial.print("Volume size (Gb): ");
    Serial.println((float)volumesize / 1024.0);

    Serial.println();

    root.openRoot(volume);

    root.ls(LS_R | LS_DATE | LS_SIZE);

    root.close();

    Serial.println();
}

void sdcard_write_begin(String filename)
{
    f = SD.open(filename.c_str(), FILE_WRITE);
}

void sdcard_write_end()
{
    f.close();
}

void sdcard_cat(String filename)
{
    f = SD.open(filename.c_str());
    if (f)
    {
        while (f.available())
        {
            Serial.write(f.read());
        }
        f.close();
    }
    else
    {
        Serial.print("Unable to open [");
        Serial.print(filename.c_str());
        Serial.println("]");
    }
}

void checkSpeed()
{
    if (prevSpeedPercent != speedPercent)
    {
        prevSpeedPercent = speedPercent;

        Serial2.print("M220 S");
        Serial2.println(speedPercent);
        waitSerial2Ok();
        Serial2.println("M220");
        waitSerial2Ok();
        Serial.print("Speed changed to:");
        Serial.print(speedPercent);
        Serial.println("%");
    }
}

// 0: ok
// 1: error
int waitSerial2Ok()
{
    String okLine;
    char okc;    

    okLine = "";
    while (true)
    {
        handleSpeedUpFlag();
        handleSpeedDownFlag();
        handlePauseFlag();
        handleResetFlag();

        if (Serial2.available())
        {
            okc = Serial2.read();
            
            okLine += okc;

            int l = okLine.length();
            if (okLine[l - 1] == '\r' || okLine[l - 1] == '\n')
            {
                okLine.trim();

                if (okLine == "ok")
                    return 0;

                if (okLine.startsWith("error:"))
                {
                    Serial.println(okLine);
                    return 1;
                }
                okLine = "";
            }
        }
    }

    return 0;
}

void sdcard_send(String filename)
{
    state = STATE_SEND_SD;

    int oldSpeedPercent = speedPercent;

    f = SD.open(filename.c_str());

    if (f)
    {
        line = "";
        bool newlineChars = false;
        while (f.available())
        {
            while (paused)
                handlePauseFlag();

            checkSpeed();

            char c = f.read();
            if (newlineChars)
            {
                if (c == '\n')
                    line += c;

                int ll = line.length();
                bool isempty = false;

                if (ll <= 2)
                {
                    isempty = true;
                    for (int i = 0; i < 2; ++i)
                    {
                        if (!(line[i] == '\r' || line[i] == '\n'))
                        {
                            isempty = false;
                            break;
                        }
                    }
                }

                if (!isempty)
                {
                    bool commentfound = false;
                    for (int i = 0; i < ll; ++i)
                    {
                        if (line[i] == ' ')
                            continue;
                        if (line[i] == ';')
                        {
                            commentfound = true;
                            break;
                        }
                        break;
                    }
                    if (commentfound)
                        isempty = true;
                }

                if (!isempty)
                {
                    Serial2.print(line.c_str());                    
                    line = "";
                    newlineChars = false;
                    if (waitSerial2Ok() == 1)
                    {
                        state = STATE_NORMAL;
                        return;
                    }
                }
                else
                {
                    line = "";
                    newlineChars = false;
                }
            }
            if (c != '\n')
                line += c;
            if ((c == '\r' || c == '\n') && line.length() > 0)
                newlineChars = true;            
        }
        if (line.length() > 0)
        {
            Serial2.print(line.c_str());
        }
        f.close();

        state = STATE_NORMAL;
    }
    else
    {
        Serial.print("Unable to open [");
        Serial.print(filename.c_str());
        Serial.println("]");
    }
}

void sdcard_rm(String filename)
{
    SD.remove(filename);
}

void reset_ISR()
{
    if (resetFlag == 0)
        resetFlag = 1;
}

void pause_ISR()
{    
    if (pauseFlag == 0)
        pauseFlag = 1;
}

void speedUp_ISR()
{
    if (speedUpFlag == 0)
        speedUpFlag = 1;
}

void speedDown_ISR()
{
    if (speedDownFlag == 0)
        speedDownFlag = 1;
}

void setup()
{
    pinMode(RESETOUT_PIN, OUTPUT);

    // ensure reset of external gcode controller
    unsigned long mm = micros();
    digitalWrite(RESETOUT_PIN, LOW);
    while (micros() - mm < 20)
        ;
    digitalWrite(RESETOUT_PIN, HIGH);

    Serial.begin(BAUD_RATE);
    while (!Serial)
    {
    }

    Serial2.begin(115200);
    while (!Serial2)
    {
    }

    pinMode(LED_BUILTIN, OUTPUT);
    digitalWrite(LED_BUILTIN, LOW);

    init_sd_card();

    //Serial.print("\e[2J\e[H");
    Serial.println("Serial gcode forwarder ready (? for help)");

    pinMode(RESET_PIN, INPUT_PULLUP);
    pinMode(PAUSE_RESUME_PIN, INPUT_PULLUP);
    pinMode(SPEED_UP_PIN, INPUT_PULLUP);
    pinMode(SPEED_DOWN_PIN, INPUT_PULLUP);

    attachInterrupt(digitalPinToInterrupt(RESET_PIN), reset_ISR, FALLING);
    attachInterrupt(digitalPinToInterrupt(PAUSE_RESUME_PIN), pause_ISR, FALLING);
    attachInterrupt(digitalPinToInterrupt(SPEED_UP_PIN), speedUp_ISR, FALLING);
    attachInterrupt(digitalPinToInterrupt(SPEED_DOWN_PIN), speedDown_ISR, FALLING);
}

void doResetGCodeCtrl()
{
    Serial.println("RESET");

    unsigned long mm = micros();
    digitalWrite(RESETOUT_PIN, LOW);
    while (micros() - mm < 20)
        ;
    digitalWrite(RESETOUT_PIN, HIGH);
}

void doPauseResume()
{
    paused = !paused;
    if (paused)
        Serial.println("PAUSED");
    else
        Serial.println("RESUMED");
}

unsigned long resetPushTotal = 0;

void handleResetFlag()
{

    if (resetFlag == 1)
    {
        resetFlag = 2;
        resetStart = millis();
    }
    else if (resetFlag == 2)
    {
        if (digitalRead(RESET_PIN) == HIGH)
        {
            resetFlag = 0;
        }
        else if (millis() - resetStart > BTN_DEBOUNCE_SLOWEST_MS)
        {
            NVIC_SystemReset();
            resetFlag = 0;
        }
    }
}

void handlePauseFlag()
{
    if (pauseFlag == 1)
    {
        pauseFlag = 2;
        pausedStart = millis();        
    }
    else if (pauseFlag == 2)
    {
        if (digitalRead(PAUSE_RESUME_PIN) == HIGH)
        {
            pauseFlag = 0;
        }
        else if (millis() - pausedStart > BTN_DEBOUNCE_SLOW_MS)
        {
            doPauseResume();
            pauseFlag = 0;
        }
    }
}

void handleSpeedUpFlag()
{
    if (speedUpFlag == 1)
    {
        speedUpFlag = 2;
        speedUpStart = millis();
    }
    else if (speedUpFlag == 2)
    {
        if (digitalRead(SPEED_UP_PIN) == HIGH)
        {
            speedUpFlag = 0;
        }
        else if (millis() - speedUpStart > BTN_DEBOUNCE_MID_MS)
        {
            speedPercent += 10;
            Serial.print("REQ SPEED:");
            Serial.print(speedPercent);
            Serial.println("%");
            speedUpFlag = 0;
        }
    }
}

void handleSpeedDownFlag()
{
    if (speedDownFlag == 1)
    {
        speedDownFlag = 2;
        speedDownStart = millis();
    }
    else if (speedDownFlag == 2)
    {
        if (digitalRead(SPEED_DOWN_PIN) == HIGH)
        {
            speedDownFlag = 0;
        }
        else if (millis() - speedDownStart > BTN_DEBOUNCE_MID_MS)
        {
            if (speedPercent >= 20)
            {
                speedPercent -= 10;
                Serial.print("REQ SPEED:");
                Serial.print(speedPercent);
                Serial.println("%");
            }
            speedDownFlag = 0;
        }
    }
}

void loop()
{
    handlePauseFlag();
    handleSpeedUpFlag();
    handleSpeedDownFlag();
    handleResetFlag();

    if (Serial2.available())
        Serial.write(Serial2.read());

    if (Serial.available())
    {
        char c = Serial.read();
        if (c == BACKSPACE)
        {
            if (line.length() > 0)
            {
                Serial.write(' ');
                Serial.write(BACKSPACE);
                line.remove(line.length() - 1);
            }
            else
            {
                Serial.write("\e[C");
            }
        }
        else if (c == '\r' || c == '\n')
        {
            if (line.length() > 0 && (line[0] == '?' || line[0] == '/'))
            {
                if (line == String("?"))
                {
                    Serial.println();
                    Serial.println("/ver                 display version");
                    Serial.println("/ls                  list sdcard content");
                    Serial.println("/write <sdfilename>  tbegin write/append sdcard file until /eof cmd");
                    Serial.println("/cat <sdfilename>    dump sdcard file to serial screen");
                    Serial.println("/rm <sdfilename>     delete sdcard file");
                    Serial.println("/send <sdfilename>   load sdcard file and send to gcode controller");
                    Serial.println("/reset               reset gcode controller");
                    Serial.println("/pause               pause/resume gcode controller print");
                    Serial.println("/s+               speed +10%");
                    Serial.println("/s-               speed -10%");

                    Serial.println();
                }
                else if (line == "/ver")
                {
                    Serial.println(VERSION_STR);
                }
                else if (line == "/ls")
                {
                    sdcard_ls();
                }
                else if (line.startsWith("/write "))
                {
                    String filename = "";
                    for (int i = 7; i < line.length(); ++i)
                    {
                        filename += line[i];
                    }
                    sdcard_write_begin(filename);
                    state = STATE_WRITE_SD;
                }
                else if (line.startsWith("/cat "))
                {
                    String filename = "";
                    for (int i = 5; i < line.length(); ++i)
                    {
                        filename += line[i];
                    }
                    sdcard_cat(filename);
                }
                else if (line.startsWith("/send "))
                {
                    String filename = "";
                    for (int i = 6; i < line.length(); ++i)
                    {
                        filename += line[i];
                    }
                    sdcard_send(filename);
                    Serial.println("File sent.");
                }
                else if (line.startsWith("/rm "))
                {
                    String filename = "";
                    for (int i = 4; i < line.length(); ++i)
                    {
                        filename += line[i];
                    }
                    sdcard_rm(filename);
                }
                else if (line == "/eof")
                {
                    sdcard_write_end();
                    Serial.println("File written.");
                    state = STATE_NORMAL;
                }
                else if (line == "/reset")
                {
                    doResetGCodeCtrl();
                }
                else if (line == "/pause")
                {
                    doPauseResume();
                }
                else if (line == "/s+")
                {
                    speedPercent += 10;
                    Serial.print("REQ SPEED:");
                    Serial.print(speedPercent);
                    Serial.println("%");
                }
                else if (line == "/s-")
                {
                    if (speedPercent >= 20)
                    {
                        speedPercent -= 10;
                        Serial.print("REQ SPEED:");
                        Serial.print(speedPercent);
                        Serial.println("%");
                    }
                }
            }
            else
            {
                if (state == STATE_WRITE_SD)
                {
                    f.println(line.c_str());
                    Serial.print("written:");
                    Serial.println(line.length() + 2);
                }
                else
                {
                    line += c;
                    Serial2.write(line.c_str());
                }
            }
            line = "";
        }
        else
        {
            line += c;
        }
    }
}
