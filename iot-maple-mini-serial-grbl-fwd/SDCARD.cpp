#include "SDCARD.h"

Sd2Card card;
SdVolume volume;
SdFile root;

// return true:(ok) false:(err)
bool init_sd_card()
{
    SD.begin();

    if (!card.init(SPI_HALF_SPEED, SDCARD_CHIPSEL))    
        return false;    

    return true;
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

        return;
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

    root.ls(/*LS_R | */LS_DATE | LS_SIZE);

    root.close();

    Serial.println();
}

void sdcard_rm(String filename)
{
    SD.remove(filename);
}

File sdcard_open(String filename)
{
    SD.open(filename);
}