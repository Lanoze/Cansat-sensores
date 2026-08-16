#ifndef __SD_READ_WRITE_H
#define __SD_READ_WRITE_H

// Substituímos as bibliotecas antigas pela SdFat V2
#include "SdFat.h" 
#include "SPI.h" //[cite: 3]

// Pinos do modulo SD (VSPI padrao da ESP32 DevKit V1)[cite: 3]
#define SD_SCK   14 //[cite: 3]
#define SD_MISO  27 //[cite: 3]
#define SD_MOSI  26 //[cite: 3]
#define SD_CS    25 //[cite: 3]

// Ajustamos o tipo FS para SdFs (classe da SdFat para exFAT/FAT32)
void listDir(SdFs &fs, const char * dirname, uint8_t levels); //[cite: 3]
void createDir(SdFs &fs, const char * path); //[cite: 3]
void removeDir(SdFs &fs, const char * path); //[cite: 3]
void readFile(SdFs &fs, const char * path); //[cite: 3]
void writeFile(SdFs &fs, const char * path, const char * message); //[cite: 3]
void appendFile(SdFs &fs, const char * path, const char * message); //[cite: 3]
void renameFile(SdFs &fs, const char * path1, const char * path2); //[cite: 3]
void deleteFile(SdFs &fs, const char * path); //[cite: 3]

#endif //[cite: 3]