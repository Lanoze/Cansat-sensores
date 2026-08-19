/*
  TESTE ISOLADO DO CARTAO SD (CORRIGIDO E OTIMIZADO)
  ==================================================
  Sem sensores, sem I2S, sem FreeRTOS. Apenas SPI + SdFat.
  Grava em 4 velocidades diferentes e reporta onde falha.

  CORREÇÕES APLICADAS:
   - Reinício físico do barramento SPI (end e begin) a cada 
     mudança de frequência, evitando o erro 0x01 (CMD0).
   - f.sync() feito em lotes (a cada 50 linhas) em vez de linha a linha,
     evitando o erro 0x24 (TIMEOUT) por estouro de cache.
*/

#include <SPI.h>
#include "SdFat.h"

// Mesmos pinos do CanSat
#define SD_SCK   14
#define SD_MISO  27
#define SD_MOSI  26
#define SD_CS    25

SPIClass sd_spi(VSPI);
SdFs SD;

const uint8_t velocidades[] = {16, 4, 2, 1};
const int LINHAS = 100; // 400 linhas para garantir que ultrapassa o gargalo das 322

void infoCartao() {
  uint8_t tipo = SD.fatType();
  Serial.printf("Formato: %s\n",
                tipo == 64 ? "exFAT" : (tipo == 32 ? "FAT32" : "FAT16/12"));
  int32_t cl = SD.freeClusterCount();
  if (cl >= 0) {
    Serial.printf("Livre: %lu MB | Cluster: %lu KB\n",
                  (unsigned long)((uint64_t)cl * SD.bytesPerCluster() / 1048576UL),
                  (unsigned long)(SD.bytesPerCluster() / 1024UL));
  }
}

void testa(uint8_t mhz) {
  Serial.printf("\n========== TESTE A %u MHz ==========\n", mhz);

  // Reinicia o barramento SPI fisicamente a cada teste para o SD aceitar a nova frequência
  sd_spi.end(); 
  sd_spi.begin(SD_SCK, SD_MISO, SD_MOSI, SD_CS);

  if (!SD.begin(SdSpiConfig(SD_CS, SHARED_SPI, SD_SCK_MHZ(mhz), &sd_spi))) {
    Serial.printf("[X] Nem montou. err=0x%02X data=0x%02X\n",
                  SD.sdErrorCode(), SD.sdErrorData());
    return;
  }
  
  if (mhz == velocidades[0]) infoCartao();

  char nome[32];
  snprintf(nome, sizeof(nome), "/teste_%umhz.csv", mhz);
  SD.remove(nome);

  // Dá um pequeno tempo para o controlador do SD atualizar o diretório
  //delay(50);

  FsFile f = SD.open(nome, O_WRITE | O_CREAT | O_TRUNC);
  if (!f) {
    Serial.printf("[X] Nao abriu %s. err=0x%02X data=0x%02X\n",
                  nome, SD.sdErrorCode(), SD.sdErrorData());
    SD.end();
    return;
  }

  int ok = 0, falhas = 0, primeiraFalha = -1;
  uint32_t pior = 0, soma = 0;
  char linha[64];

  for (int i = 1; i <= LINHAS; i++) {
    int n = snprintf(linha, sizeof(linha), "%d;%lu;abcdefghijklmnop\r\n",
                     i, (unsigned long)millis());

    uint32_t t0 = millis();
    
    // Grava os dados no buffer do cartão
    bool bom = (f.write((const uint8_t*)linha, n) == (size_t)n);
    
    // Faz o sync (gravação física + FAT) apenas a cada 50 linhas ou na última
    if (bom && (i % 50 == 0 || i == LINHAS)) {
        if (!f.sync()) bom = false;
    }
    
    uint32_t dt = millis() - t0;

    if (dt > pior) pior = dt;
    soma += dt;

    if (bom) {
      ok++;
    } else {
      falhas++;
      if (primeiraFalha < 0) primeiraFalha = i;
      Serial.printf("  FALHA na linha %d -> err=0x%02X data=0x%02X\n",
                    i, SD.sdErrorCode(), SD.sdErrorData());
      if (falhas >= 5) {
        Serial.println("  (5 falhas, abortando esta velocidade)");
        break;
      }
    }
    delay(200); // Simulando a taxa de amostragem
  }

  f.close();

  Serial.printf("-> RESULTADO %u MHz: %d ok / %d falhas", mhz, ok, falhas);
  if (primeiraFalha > 0) Serial.printf(" | 1a falha na linha %d", primeiraFalha);
  Serial.printf(" | write+sync medio %lu ms, pior %lu ms\n",
                (unsigned long)(soma / (ok + falhas > 0 ? ok + falhas : 1)),
                (unsigned long)pior);

  SD.end();
  delay(500);
}

void setup() {
  Serial.begin(115200);
  delay(2000);
  Serial.println("\n\n### TESTE ISOLADO DO SD ###");
  Serial.printf("Pinos: SCK=%d MISO=%d MOSI=%d CS=%d\n", SD_SCK, SD_MISO, SD_MOSI, SD_CS);

  for (uint8_t i = 0; i < sizeof(velocidades); i++) {
    testa(velocidades[i]);
  }

  Serial.println("\n### FIM. Compare os resultados acima. ###");
}

void loop() {
  // Nada aqui
}