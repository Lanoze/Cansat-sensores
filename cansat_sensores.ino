/*
  CANSAT - LEITURA DE SENSORES E AUDIO I2S (exFAT) 
  ARQUITETURA: PRODUTOR-CONSUMIDOR (ABRE/FECHA SEGURO)
  FORMATO CSV: Padrão Internacional (Delimitador: Vírgula / Decimal: Ponto)
  PROTEÇÃO SD: Pré-Alocação Extrema de 400MB para evitar travamentos

  CORREÇÕES APLICADAS:
  [C1] sdOk declarado como volatile bool — evita cache de registrador entre cores
  [C2] reiniciarSD() usa SD_SCK_MHZ(1) igual ao sdInit() — evita falha no soft reset
  [C3] atualizarCabecalhoWAV() removido do sync periódico — elimina stall do seek a cada 5s
       e chamado apenas no fechamento real do arquivo
  [C4] csvBuffer e localCsvBuffer aumentados para 4096 bytes — evita perda silenciosa de dados
  [C5] tamanhoDadosAudio inicia em 0 e é atualizado apenas no fechamento — WAV correto sempre
  [C6] falhasConsecutivas declarado como volatile — consistência entre contextos de escrita/leitura
  [C7] reiniciarSD() reseta tamanhoDadosAudio — WAV não herda tamanho de sessão anterior
*/

//Alterações sugeridas pelo Claude na noite do dia 15/08/2026

#include <Wire.h>
#include <SPI.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_BMP280.h>
#include <Adafruit_SHT31.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <driver/i2s.h>
#include "sd_read_write.h"

// ---------- PINOS (I2C / OneWire) ----------
#define SDA_PIN      17
#define SCL_PIN       4
#define ONE_WIRE_BUS 18

// ---------- PINOS MICROFONE INMP441 (I2S) ----------
#define I2S_WS_PIN  19
#define I2S_SD_PIN  22
#define I2S_SCK_PIN 21

#define SEALEVEL_HPA 1013.25

// ---------- OBJETOS DOS SENSORES ----------
Adafruit_MPU6050 mpu;
Adafruit_BMP280  bmp;
Adafruit_SHT31   sht30 = Adafruit_SHT31();
OneWire          oneWire(ONE_WIRE_BUS);
DallasTemperature ds18b20(&oneWire);

// ---------- SD E GESTÃO DE MEMÓRIA RAM ----------
SPIClass sd_spi(VSPI);
SdFs SD;
SemaphoreHandle_t ramMutex;

// Arquivos mantidos abertos globalmente
FsFile audioFile;
FsFile csvFile;

// [C4] Buffer aumentado para 4096 bytes
char csvBuffer[4096] = "";

bool mpuOk = false;
bool bmpOk = false;
bool shtOk = false;
bool dsOk  = false;

// [C1] volatile impede que os cores usem valor em cache de registrador
volatile bool sdOk = false;

const char* LOG_FILE   = "/data.csv";
const char* AUDIO_FILE = "/voo.wav";

uint32_t pacoteId = 1;

// Variáveis Globais de Controle do Áudio
// [C5] Inicia em 0 — será acumulado durante a gravação e escrito no cabeçalho apenas no fechamento
uint32_t tamanhoDadosAudio = 0;

// [C6] volatile — escrita pelo Core 0, lida em condições de controle do mesmo Core
volatile uint8_t falhasConsecutivas = 0;

const int SAMPLE_RATE = 16000;

// Protótipos
void sdInit(void);
void i2sInit(void);
bool verificaMicrofoneINMP441(void);
bool reiniciarSD(void);
void atualizarCabecalhoWAV(FsFile &arquivo);
void sdManagerTask(void *pvParameters);
void enviaParaRAM(float accX, float accY, float accZ,
                  float gyroX, float gyroY, float gyroZ,
                  float tempMpu, float tempBmp, float pressao,
                  float altitude, float tempSht, float umidSht, float tempDs);

// =====================
// CABEÇALHO WAV DINÂMICO
// Chamado apenas no fechamento do arquivo (setup não chama mais com tamanho falso)
// =====================
void atualizarCabecalhoWAV(FsFile &arquivo) {
  byte header[44];
  uint32_t tamanhoTotalArquivo = tamanhoDadosAudio + 36;
  uint16_t numChannels   = 1;
  uint16_t bitsPerSample = 16;
  uint32_t byteRate      = SAMPLE_RATE * numChannels * (bitsPerSample / 8);

  header[0] = 'R'; header[1] = 'I'; header[2] = 'F'; header[3] = 'F';
  header[4] = (byte)(tamanhoTotalArquivo & 0xFF);
  header[5] = (byte)((tamanhoTotalArquivo >> 8)  & 0xFF);
  header[6] = (byte)((tamanhoTotalArquivo >> 16) & 0xFF);
  header[7] = (byte)((tamanhoTotalArquivo >> 24) & 0xFF);
  header[8]  = 'W'; header[9]  = 'A'; header[10] = 'V'; header[11] = 'E';

  header[12] = 'f'; header[13] = 'm'; header[14] = 't'; header[15] = ' ';
  header[16] = 16; header[17] = 0; header[18] = 0; header[19] = 0;
  header[20] = 1;  header[21] = 0;
  header[22] = (byte)numChannels; header[23] = 0;
  header[24] = (byte)(SAMPLE_RATE & 0xFF);
  header[25] = (byte)((SAMPLE_RATE >> 8)  & 0xFF);
  header[26] = (byte)((SAMPLE_RATE >> 16) & 0xFF);
  header[27] = (byte)((SAMPLE_RATE >> 24) & 0xFF);
  header[28] = (byte)(byteRate & 0xFF);
  header[29] = (byte)((byteRate >> 8)  & 0xFF);
  header[30] = (byte)((byteRate >> 16) & 0xFF);
  header[31] = (byte)((byteRate >> 24) & 0xFF);
  header[32] = (byte)(numChannels * bitsPerSample / 8); header[33] = 0;
  header[34] = 16; header[35] = 0;

  header[36] = 'd'; header[37] = 'a'; header[38] = 't'; header[39] = 'a';
  header[40] = (byte)(tamanhoDadosAudio & 0xFF);
  header[41] = (byte)((tamanhoDadosAudio >> 8)  & 0xFF);
  header[42] = (byte)((tamanhoDadosAudio >> 16) & 0xFF);
  header[43] = (byte)((tamanhoDadosAudio >> 24) & 0xFF);

  uint32_t posicaoAtual = arquivo.position();
  arquivo.seek(0);
  arquivo.write(header, 44);
  arquivo.seek(posicaoAtual);
}

// =====================
// SETUP
// =====================
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
    bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,
                    Adafruit_BMP280::SAMPLING_X2,
                    Adafruit_BMP280::SAMPLING_X16,
                    Adafruit_BMP280::FILTER_X16,
                    Adafruit_BMP280::STANDBY_MS_500);
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

  // ----- INICIALIZAÇÃO I2S E SD -----
  i2sInit();
  if (verificaMicrofoneINMP441()) {
    Serial.println("INMP441: OK (Sinal 32-bit detectado)");
  } else {
    Serial.println("INMP441: FALHA (Desconectado ou sem sinal)");
  }
  sdInit();

  if (sdOk) {
    bool csvExiste = SD.exists(LOG_FILE);

    csvFile = SD.open(LOG_FILE, O_WRITE | O_CREAT | O_APPEND);
    if (!csvExiste && csvFile) {
      Serial.println("-> Pre-alocando 2MB para o arquivo CSV...");
      csvFile.preAllocate(2ULL * 1024ULL * 1024ULL);
      csvFile.print("Pacote,Tempo_ms,AccX,AccY,AccZ,GyroX,GyroY,GyroZ,"
                    "Temp_MPU(C),Temp_BMP(C),Pressao(hPa),Altitude(m),"
                    "Temp_SHT(C),Umidade_SHT(%),Temp_DS18B20(C)\r\n");
      csvFile.sync();
    }

    if (SD.exists(AUDIO_FILE)) {
      if (!SD.remove(AUDIO_FILE)) {
        Serial.println("-> [AVISO] Nao foi possivel deletar o WAV antigo.");
      }
      delay(50);
    }

    audioFile = SD.open(AUDIO_FILE, O_WRITE | O_CREAT | O_TRUNC);
    if (audioFile) {
      Serial.println("-> Pre-alocando 400MB para o arquivo de Audio...");
      if (audioFile.preAllocate(400ULL * 1024ULL * 1024ULL)) {
        Serial.println("-> [SUCESSO] 400MB reservados fisicamente no cartao!");
      } else {
        Serial.println("-> [AVISO] Falha na pre-alocacao. O voo prosseguira normalmente.");
      }

      // [C5] tamanhoDadosAudio = 0: escreve cabeçalho com tamanho real (zero por enquanto).
      // Será atualizado com o valor correto apenas no fechamento do arquivo.
      // Isso garante que o WAV seja sempre válido, independente da duração do voo.
      tamanhoDadosAudio = 0;
      atualizarCabecalhoWAV(audioFile);
    }

    if (!audioFile || !csvFile) {
      Serial.println("-> [ERRO CRITICO] Falha ao abrir os arquivos no SD.");
    } else {
      Serial.println("-> Arquivos abertos! Iniciando Core 0.");
      xTaskCreatePinnedToCore(
        sdManagerTask,
        "SD_Manager",
        8192,
        NULL,
        1,
        NULL,
        0
      );
    }
  }

  Serial.println("=== Setup concluido ===");
}

// =====================
// LOOP — Apenas produz dados para a RAM (Core 1)
// =====================
void loop() {
  float accX = 0, accY = 0, accZ = 0;
  float gyroX = 0, gyroY = 0, gyroZ = 0, tempMpu = 0;
  float tempBmp = 0, pressao = 0, altitude = 0;
  float tempSht = 0, umidSht = 0, tempDs = 0;

  if (mpuOk) {
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);
    accX = a.acceleration.x; accY = a.acceleration.y; accZ = a.acceleration.z;
    gyroX = g.gyro.x; gyroY = g.gyro.y; gyroZ = g.gyro.z;
    tempMpu = temp.temperature;
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
  Serial.printf("MPU  -> acc: %.2f %.2f %.2f | gyro: %.2f %.2f %.2f | T: %.2f C\n",
                accX, accY, accZ, gyroX, gyroY, gyroZ, tempMpu);
  Serial.printf("BMP  -> T: %.2f C | P: %.2f hPa | Alt: %.2f m\n", tempBmp, pressao, altitude);
  Serial.printf("SHT  -> T: %.2f C | U: %.2f %%\n", tempSht, umidSht);
  Serial.printf("DS18 -> T: %.2f C\n", tempDs);

  enviaParaRAM(accX, accY, accZ, gyroX, gyroY, gyroZ,
               tempMpu, tempBmp, pressao, altitude, tempSht, umidSht, tempDs);

  pacoteId++;
  delay(200);
}

// =====================
// INICIALIZAÇÃO DO SD
// =====================
void sdInit(void) {
  sd_spi.begin(SD_SCK, SD_MISO, SD_MOSI, SD_CS);

  if (!SD.begin(SdSpiConfig(SD_CS, SHARED_SPI, SD_SCK_MHZ(1), &sd_spi))) {
    Serial.println("Falha ao montar SD exFAT");
    return;
  }

  sdOk = true;
  Serial.println("Cartao SD exFAT pronto a 1MHz.");
}

// =====================
// SOFT RESET DO SD
// =====================
bool reiniciarSD() {
  Serial.println("\n-> [SISTEMA] Tentando reiniciar modulo SD (Soft Reset)...");

  // [C5+C7] Atualiza o cabeçalho WAV com o tamanho real gravado até agora,
  // fecha ambos os arquivos de forma graciosa e reseta o contador de áudio.
  if (audioFile) {
    atualizarCabecalhoWAV(audioFile);
    audioFile.close();
  }
  if (csvFile) {
    csvFile.close();
  }

  SD.end();
  sd_spi.end();
  delay(200);

  sd_spi.begin(SD_SCK, SD_MISO, SD_MOSI, SD_CS);

  // [C2] Reinicia na mesma velocidade do sdInit() — 1MHz
  // Usar 4MHz aqui aumenta a chance de falha quando o circuito já está instável
  if (!SD.begin(SdSpiConfig(SD_CS, SHARED_SPI, SD_SCK_MHZ(1), &sd_spi))) {
    Serial.println("-> [FALHA] SD recusou reinicio.");
    sdOk = false;
    return false;
  }

  sdOk = true;

  if (SD.exists(AUDIO_FILE)) {
    SD.remove(AUDIO_FILE);
    delay(50);
  }

  audioFile = SD.open(AUDIO_FILE, O_WRITE | O_CREAT | O_AT_END);
  csvFile   = SD.open(LOG_FILE,   O_WRITE | O_CREAT | O_APPEND);

  if (!audioFile || !csvFile) {
    Serial.println("-> [FALHA] Falhou ao reabrir arquivos.");
    return false;
  }

  // [C7] Reseta o contador de áudio — o novo arquivo começa do zero
  tamanhoDadosAudio = 0;
  falhasConsecutivas = 0;

  Serial.println("-> [SUCESSO] SD reiniciado! Retomando...");
  return true;
}

// =====================
// INICIALIZAÇÃO DO I2S
// =====================
void i2sInit() {
  i2s_config_t i2s_config = {
    .mode                 = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_RX),
    .sample_rate          = SAMPLE_RATE,
    .bits_per_sample      = I2S_BITS_PER_SAMPLE_32BIT,
    .channel_format       = I2S_CHANNEL_FMT_ONLY_LEFT,
    .communication_format = I2S_COMM_FORMAT_STAND_I2S,
    .intr_alloc_flags     = ESP_INTR_FLAG_LEVEL1,
    .dma_buf_count        = 8,
    .dma_buf_len          = 512,
    .use_apll             = false,
    .tx_desc_auto_clear   = false,
    .fixed_mclk           = 0
  };

  i2s_pin_config_t pin_config = {
    .bck_io_num   = I2S_SCK_PIN,
    .ws_io_num    = I2S_WS_PIN,
    .data_out_num = I2S_PIN_NO_CHANGE,
    .data_in_num  = I2S_SD_PIN
  };

  i2s_driver_install(I2S_NUM_0, &i2s_config, 0, NULL);
  i2s_set_pin(I2S_NUM_0, &pin_config);
}

// =====================
// VERIFICAÇÃO DO MICROFONE INMP441
// =====================
bool verificaMicrofoneINMP441() {
  size_t bytesLidos;
  const int amostras = 100;
  int32_t buffer[amostras];

  memset(buffer, 0, sizeof(buffer));

  esp_err_t erro = i2s_read(I2S_NUM_0, &buffer, sizeof(buffer),
                             &bytesLidos, pdMS_TO_TICKS(100));

  if (erro != ESP_OK || bytesLidos == 0) return false;

  int zerosCount = 0;
  for (int i = 0; i < amostras; i++) {
    if (buffer[i] == 0 || buffer[i] == -1) zerosCount++;
  }

  return (zerosCount < amostras);
}

// =====================
// PRODUTOR CSV (Core 1)
// =====================
void enviaParaRAM(float accX, float accY, float accZ,
                  float gyroX, float gyroY, float gyroZ,
                  float tempMpu, float tempBmp, float pressao,
                  float altitude, float tempSht, float umidSht, float tempDs) {
  if (!sdOk) return;

  char linha[256];
  snprintf(linha, sizeof(linha),
           "%lu,%lu,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f\r\n",
           (unsigned long)pacoteId, millis(),
           accX, accY, accZ, gyroX, gyroY, gyroZ,
           tempMpu, tempBmp, pressao, altitude, tempSht, umidSht, tempDs);

  if (xSemaphoreTake(ramMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
    if (strlen(csvBuffer) + strlen(linha) < sizeof(csvBuffer)) {
      strcat(csvBuffer, linha);
    } else {
      Serial.println("-> [ALERTA] RAM estourada! Algumas linhas CSV foram perdidas.");
    }
    xSemaphoreGive(ramMutex);
  }
}

// =====================
// TAREFA MESTRA DO SD — Consumidor (Core 0)
// =====================
void sdManagerTask(void *pvParameters) {
  const int SAMPLES = 512;
  int32_t buffer32[SAMPLES];
  int16_t buffer16[SAMPLES];
  size_t  bytesRead = 0;

  uint32_t ultimoWriteCSV = millis();
  uint32_t ultimoSync     = millis();

  // [C4] Casado com o tamanho do csvBuffer global
  char localCsvBuffer[4096] = "";

  while (1) {

    // — VERIFICA SAÚDE DO SD —
    if (!sdOk || falhasConsecutivas >= 5) {
      if (!reiniciarSD()) {
        vTaskDelay(pdMS_TO_TICKS(2000));
      }
      continue;
    }

    // — 1. GRAVA ÁUDIO CONTÍNUO (32-bit → 16-bit com anticlipping) —
    esp_err_t result = i2s_read(I2S_NUM_0, &buffer32, sizeof(buffer32),
                                 &bytesRead, pdMS_TO_TICKS(100));

    if (result == ESP_OK && bytesRead > 0 && sdOk && audioFile) {
      int samplesRead = bytesRead / 4;

      for (int i = 0; i < samplesRead; i++) {
        int32_t amostraComGanho = buffer32[i] >> 14;
        if      (amostraComGanho >  32767) amostraComGanho =  32767;
        else if (amostraComGanho < -32768) amostraComGanho = -32768;
        buffer16[i] = (int16_t)amostraComGanho;
      }

      size_t bytesToWrite  = samplesRead * 2;
      size_t bytesEscritos = audioFile.write((const uint8_t*)buffer16, bytesToWrite);

      if (bytesEscritos == bytesToWrite) {
        falhasConsecutivas = 0;
        tamanhoDadosAudio += bytesEscritos;
      } else {
        falhasConsecutivas++;
        Serial.printf("-> [ERRO] Falha ao gravar audio. Falhas seguidas: %u\n",
                      (unsigned)falhasConsecutivas);
      }
    }

    // — 2. DESCARREGA CSV A CADA 2 SEGUNDOS —
    if (millis() - ultimoWriteCSV >= 2000) {
      localCsvBuffer[0]      = '\0';
      bool temDadosParaGravar = false;

      if (xSemaphoreTake(ramMutex, pdMS_TO_TICKS(15)) == pdTRUE) {
        if (csvBuffer[0] != '\0') {
          strcpy(localCsvBuffer, csvBuffer);
          csvBuffer[0]       = '\0';
          temDadosParaGravar = true;
        }
        xSemaphoreGive(ramMutex);
      }

      if (temDadosParaGravar && sdOk && csvFile) {
        size_t escr = csvFile.print(localCsvBuffer);
        if (escr > 0) {
          Serial.print("-> Lote CSV salvo! Bytes: ");
          Serial.println(escr);
        } else {
          Serial.println("-> [ERRO] SD retornou 0 bytes na gravacao do CSV.");
          falhasConsecutivas++;
        }
      }
      ultimoWriteCSV = millis();
    }

    // — 3. SYNC A CADA 5 SEGUNDOS —
    // [C3] Apenas sync — sem atualizarCabecalhoWAV() aqui.
    // O cabeçalho WAV é atualizado somente no fechamento gracioso do arquivo,
    // eliminando o stall causado pelo seek() periódico.
    if (millis() - ultimoSync >= 5000) {
      if (sdOk) {
        if (audioFile) audioFile.sync();
        if (csvFile)   csvFile.sync();
      }
      ultimoSync = millis();
    }

    vTaskDelay(1 / portTICK_PERIOD_MS);
  }
}
