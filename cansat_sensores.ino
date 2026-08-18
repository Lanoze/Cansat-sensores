/*
  CANSAT - LEITURA DE SENSORES APENAS (SEM ÁUDIO / SEM I2S)
  ARQUITETURA: PRODUTOR-CONSUMIDOR (ABRE/FECHA SEGURO)
  FORMATO CSV: Padrão Internacional (Delimitador: Vírgula / Decimal: Ponto)
  PROTEÇÃO SD: Pré-Alocação de 2MB para evitar travamentos
*/

//Versão sem microfone

#include <Wire.h>
#include <SPI.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_BMP280.h>
#include <Adafruit_SHT31.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include "sd_read_write.h"

// ---------- PINOS (I2C / OneWire) ----------
#define SDA_PIN     17 
#define SCL_PIN     4  
#define ONE_WIRE_BUS 18 

#define SEALEVEL_HPA 1013.25 

// ---------- OBJETOS DOS SENSORES ----------
Adafruit_MPU6050 mpu;
Adafruit_BMP280  bmp;
Adafruit_SHT31   sht30 = Adafruit_SHT31(); 
OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature ds18b20(&oneWire);

// ---------- SD E GESTÃO DE MEMÓRIA RAM ----------
SPIClass sd_spi(VSPI); 
SdFs SD;               
SemaphoreHandle_t ramMutex; 

// Arquivo mantido aberto globalmente
FsFile csvFile; 

// Buffer aumentado para 4096 bytes para segurança contra acúmulos na RAM
char csvBuffer[4096] = ""; 

bool mpuOk = false;
bool bmpOk = false;
bool shtOk = false;
bool dsOk  = false;
volatile bool sdOk  = false; 

const char* LOG_FILE = "/data.csv"; 

uint32_t pacoteId = 1; 
uint8_t falhasConsecutivas = 0;

// Protótipos
void sdInit(void);
bool reiniciarSD(void);
void sdManagerTask(void *pvParameters); 
void enviaParaRAM(float accX, float accY, float accZ, float gyroX, float gyroY, float gyroZ, float tempMpu, float tempBmp, float pressao, float altitude, float tempSht, float umidSht, float tempDs); 

void setup() {
  Serial.begin(115200); 
  delay(1500); 

  ramMutex = xSemaphoreCreateMutex();
  Wire.begin(SDA_PIN, SCL_PIN); 
  
  csvBuffer[0] = '\0';

  // ----- INICIALIZAÇÃO DOS SENSORES -----
  Serial.print("MPU6050: "); 
  if (mpu.begin()) {
    mpu.setAccelerometerRange(MPU6050_RANGE_8_G); 
    mpu.setGyroRange(MPU6050_RANGE_500_DEG); 
    mpu.setFilterBandwidth(MPU6050_BAND_21_HZ); 
    mpuOk = true;
    Serial.println("OK"); 
  } else {
    Serial.println("FALHOU");
  }

  Serial.print("BMP280: "); 
  if (bmp.begin(0x76) || bmp.begin(0x77)) { 
    bmp.setSampling(Adafruit_BMP280::MODE_NORMAL, Adafruit_BMP280::SAMPLING_X2, Adafruit_BMP280::SAMPLING_X16, Adafruit_BMP280::FILTER_X16, Adafruit_BMP280::STANDBY_MS_500); 
    bmpOk = true;
    Serial.println("OK"); 
  } else {
    Serial.println("FALHOU");
  }

  Serial.print("SHT30: ");
  if (sht30.begin(0x44) || sht30.begin(0x45)) { 
    shtOk = true;
    Serial.println("OK"); 
  } else {
    Serial.println("FALHOU");
  }

  Serial.print("DS18B20: ");
  ds18b20.begin(); 
  if (ds18b20.getDeviceCount() > 0) { 
    dsOk = true;
    Serial.println("OK");
  } else {
    Serial.println("FALHOU");
  }

  // ----- INICIALIZAÇÃO DO CARTÃO SD -----
  sdInit();

  if (sdOk) {
    bool csvExiste = SD.exists(LOG_FILE);
    
    csvFile = SD.open(LOG_FILE, O_WRITE | O_CREAT | O_APPEND);
    if (!csvExiste && csvFile) {
      Serial.println("-> Pre-alocando 2MB para o arquivo CSV...");
      csvFile.preAllocate(2ULL * 1024ULL * 1024ULL); 
      csvFile.print("Pacote,Tempo_ms,AccX,AccY,AccZ,GyroX,GyroY,GyroZ,Temp_MPU(C),Temp_BMP(C),Pressao(hPa),Altitude(m),Temp_SHT(C),Umidade_SHT(%),Temp_DS18B20(C)\r\n");
      csvFile.sync();
    }

    if (!csvFile) {
      Serial.println("-> [ERRO CRITICO] Falha ao abrir o arquivo CSV no SD.");
    } else {
      Serial.println("-> Arquivo CSV criado e travado aberto! Iniciando Core 0.");
      
      // Cria a tarefa de esvaziamento do buffer de dados no Core 0
      xTaskCreatePinnedToCore(
        sdManagerTask,  
        "SD_Manager",      
        8192,             // Alterado de 4096 para 8192 bytes
        NULL,             
        1,                
        NULL,             
        0                 
      );
    }
  }

  Serial.println("=== Setup concluido ==="); 
}

// O LOOP APENAS PRODUZ DADOS PARA A RAM (Roda no Core 1)
void loop() {
  float accX = 0, accY = 0, accZ = 0, gyroX = 0, gyroY = 0, gyroZ = 0, tempMpu = 0; 
  float tempBmp = 0, pressao = 0, altitude = 0; 
  float tempSht = 0, umidSht = 0, tempDs = 0; 

  if (mpuOk) {
    sensors_event_t a, g, temp; 
    mpu.getEvent(&a, &g, &temp); 
    accX = a.acceleration.x; accY = a.acceleration.y; accZ = a.acceleration.z; 
    gyroX = g.gyro.x; gyroY = g.gyro.y; gyroZ = g.gyro.z; tempMpu = temp.temperature; 
  }

  if (bmpOk) {
    tempBmp  = bmp.readTemperature(); 
    pressao  = bmp.readPressure() / 100.0F; 
    altitude = bmp.readAltitude(SEALEVEL_HPA); 
  }

  if (shtOk) {
    tempSht = sht30.readTemperature(); 
    umidSht = sht30.readHumidity(); 
  }

  if (dsOk) {
    ds18b20.requestTemperatures(); 
    tempDs = ds18b20.getTempCByIndex(0); 
  }

  Serial.println("\n===== LEITURA " + String(pacoteId) + " ====="); 
  Serial.printf("MPU  -> acc: %.2f %.2f %.2f | gyro: %.2f %.2f %.2f | T: %.2f C\n", accX, accY, accZ, gyroX, gyroY, gyroZ, tempMpu);
  Serial.printf("BMP  -> T: %.2f C | P: %.2f hPa | Alt: %.2f m\n", tempBmp, pressao, altitude);
  Serial.printf("SHT  -> T: %.2f C | U: %.2f %%\n", tempSht, umidSht);
  Serial.printf("DS18 -> T: %.2f C\n", tempDs);

  enviaParaRAM(accX, accY, accZ, gyroX, gyroY, gyroZ, tempMpu, tempBmp, pressao, altitude, tempSht, umidSht, tempDs); 

  pacoteId++; 
  delay(200); // Taxa de leitura dos sensores (~5 Hz)
}

// ==========================================
// FUNÇÕES DE GERENCIAMENTO DO HARDWARE SD
// ==========================================
void sdInit(void) {
  sd_spi.begin(SD_SCK, SD_MISO, SD_MOSI, SD_CS); 
  
  // SPI travado a 1 MHz para máxima imunidade a ruídos na protoboard
  if (!SD.begin(SdSpiConfig(SD_CS, SHARED_SPI, SD_SCK_MHZ(1), &sd_spi))) {
    Serial.println("Falha ao montar cartao SD.");
    return;
  }
  
  sdOk = true;
  Serial.println("Cartão SD pronto para gravação.");
}

bool reiniciarSD() {
  Serial.println("\n-> [SISTEMA] Tentando reiniciar modulo SD (Soft Reset)...");
  
  if (csvFile) {
    csvFile.close();
  }
  
  SD.end();
  sd_spi.end();
  delay(200); 

  sd_spi.begin(SD_SCK, SD_MISO, SD_MOSI, SD_CS);
  
  if (!SD.begin(SdSpiConfig(SD_CS, SHARED_SPI, SD_SCK_MHZ(1), &sd_spi))) {
    Serial.println("-> [FALHA] SD recusou reinicio.");
    sdOk = false;
    return false;
  }
  
  sdOk = true;
  csvFile = SD.open(LOG_FILE, O_WRITE | O_CREAT | O_APPEND);
  
  if (!csvFile) {
    Serial.println("-> [FALHA] Falhou ao reabrir arquivo CSV.");
    return false;
  }

  Serial.println("-> [SUCESSO] SD reiniciado! Retomando gravacao...");
  falhasConsecutivas = 0;
  return true;
}

// ==========================================
// FUNÇÃO DO PRODUTOR (Escreve no buffer RAM)
// ==========================================
void enviaParaRAM(float accX, float accY, float accZ, float gyroX, float gyroY, float gyroZ, float tempMpu, float tempBmp, float pressao, float altitude, float tempSht, float umidSht, float tempDs) { 
  if (!sdOk) return; 

  char linha[256];
  snprintf(linha, sizeof(linha), "%lu,%lu,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f\r\n", 
           (unsigned long)pacoteId, millis(), accX, accY, accZ, gyroX, gyroY, gyroZ, 
           tempMpu, tempBmp, pressao, altitude, tempSht, umidSht, tempDs);

  if (xSemaphoreTake(ramMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
    if (strlen(csvBuffer) + strlen(linha) < sizeof(csvBuffer)) {
      strcat(csvBuffer, linha);
    } else {
      Serial.println("-> [ALERTA] RAM estourada! Cartao lento, linhas perdidas.");
    }
    xSemaphoreGive(ramMutex);
  }
}

// ==========================================
// TAREFA DO CONSUMIDOR (Descarrega a RAM no SD)
// ==========================================
void sdManagerTask(void *pvParameters) {
  uint32_t ultimoWriteCSV = millis();
  uint32_t ultimoSync = millis();
  char localCsvBuffer[4096] = ""; 

  while(1) {
    // Se houver falhas consecutivas, aciona a rotina de recuperação automática
    if (!sdOk || falhasConsecutivas >= 5) {
      if (!reiniciarSD()) {
        vTaskDelay(pdMS_TO_TICKS(2000));
      }
      continue; 
    }

    // DESCARREGA O BUFFER DA RAM NO SD A CADA 2 SEGUNDOS
    if (millis() - ultimoWriteCSV >= 2000) {
      localCsvBuffer[0] = '\0'; 
      bool temDadosParaGravar = false;
      
      if (xSemaphoreTake(ramMutex, pdMS_TO_TICKS(15)) == pdTRUE) {
        if (csvBuffer[0] != '\0') {
          strcpy(localCsvBuffer, csvBuffer);
          csvBuffer[0] = '\0'; 
          temDadosParaGravar = true;
        }
        xSemaphoreGive(ramMutex);
      }

      if (temDadosParaGravar && sdOk && csvFile) {
        size_t escr = csvFile.print(localCsvBuffer);
        
        if (escr > 0) {
          Serial.print("-> Lote CSV salvo no SD! Bytes gravados: ");
          Serial.println(escr);
          falhasConsecutivas = 0;
        } else {
          Serial.println("-> [ERRO] O SD falhou ao registrar bloco de dados.");
          falhasConsecutivas++;
        }
      }
      ultimoWriteCSV = millis();
    }

    // FORÇA O SYNC FÍSICO COM O CARTÃO A CADA 5 SEGUNDOS
    if (millis() - ultimoSync >= 5000) {
      if (sdOk && csvFile) {
        csvFile.sync();
      }
      ultimoSync = millis();
    }
    
    vTaskDelay(pdMS_TO_TICKS(10));
  }
}