/*
 * DVS Sine Master – I2S DMA Engine (Zero Jitter)
 * Receptor WiFi AP  →  Gera timecode 1kHz quadratura via DAC interno
 * GPIO 25 = Left (DAC1)   GPIO 26 = Right (DAC2)
 */
#include <WiFi.h>
#include <WiFiUdp.h>
#include "driver/i2s.h"
#include <math.h>

// ── REDE ──────────────────────────────────────────────
const char* AP_SSID = "DVS_LINK";
const char* AP_PASS = "sine_master_dvs";
const uint16_t UDP_PORT = 5000;

// === SELEÇÃO DE DECK (A=1, B=2) ===
// Mude este número ao gravar no 2º Receptor (Deck B do Serato)
const uint8_t TARGET_DECK_ID = 1;

struct MotionMessage {
    int32_t  phaseTicks;
    float    gyroDps;
    uint8_t  motionActive;
    uint8_t  transmitterId;
    uint32_t sequence;
} __attribute__((packed));

WiFiUDP udp;

// ── ÁUDIO I2S ─────────────────────────────────────────
#define SAMPLE_RATE    44100
#define CARRIER_FREQ   1000.0f
#define LUT_BITS       8
#define LUT_SIZE       (1 << LUT_BITS)          // 256
#define LUT_MASK       (LUT_SIZE - 1)           // 0xFF
#define BUF_FRAMES     64                       // frames por bloco DMA

// Tabela seno 256 pontos, amplitude 100/128 (evita clipping)
uint8_t sineLUT[LUT_SIZE];

// Fase acumulada (ponto fixo 24.8 para suavidade)
volatile float phaseInc = 0.0f;   // incremento por sample (em unidades LUT)
float phase = 0.0f;               // acumulador
unsigned long lastPktMs = 0;

// ── SETUP ─────────────────────────────────────────────
void setup() {
    Serial.begin(115200);

    // Gera Seno LUT
    for (int i = 0; i < LUT_SIZE; i++) {
        float a = 2.0f * M_PI * i / LUT_SIZE;
        sineLUT[i] = (uint8_t)(128.0f + 100.0f * sinf(a));
    }

    // Configura I2S no modo DAC interno (DMA zero-jitter)
    i2s_config_t cfg = {};
    cfg.mode            = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_TX | I2S_MODE_DAC_BUILT_IN);
    cfg.sample_rate     = SAMPLE_RATE;
    cfg.bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT;
    cfg.channel_format  = I2S_CHANNEL_FMT_RIGHT_LEFT;
    cfg.communication_format = I2S_COMM_FORMAT_STAND_I2S;
    cfg.intr_alloc_flags = ESP_INTR_FLAG_LEVEL1;
    cfg.dma_buf_count   = 8;
    cfg.dma_buf_len     = BUF_FRAMES;
    cfg.use_apll        = true;   // Clock APLL para taxa exata

    i2s_driver_install(I2S_NUM_0, &cfg, 0, NULL);
    i2s_set_pin(I2S_NUM_0, NULL);                    // DAC interno
    i2s_set_dac_mode(I2S_DAC_CHANNEL_BOTH_EN);       // GPIO25 + GPIO26

    // WiFi Access-Point
    WiFi.softAP(AP_SSID, AP_PASS);
    // DESLIGA O SLEEP DA ANTENA (MODO ANTI-LATÊNCIA ABSOLUTO)
    WiFi.setSleep(false);
    
    udp.begin(UDP_PORT);

    Serial.printf("DVS I2S Ready | AP: %s | IP: %s | SR: %d\n",
                  AP_SSID, WiFi.softAPIP().toString().c_str(), SAMPLE_RATE);
}

// ── LOOP ──────────────────────────────────────────────
// Buffer estéreo: [R16, L16, R16, L16, …]
uint16_t dmaBuf[BUF_FRAMES * 2];

void loop() {
    // ── 1. Recebe UDP ────────────────────────────────
    int pkt = udp.parsePacket();
    if (pkt) {
        // --- DRENO DE LATÊNCIA ---
        // Se houverem muitos pacotes na fila WiFi, esvaziamos até o último.
        MotionMessage m;
        bool gotNew = false;
        
        while (pkt) {
            uint8_t raw[32];
            int len = udp.read(raw, sizeof(raw));
            if (len >= (int)sizeof(MotionMessage)) {
                memcpy(&m, raw, sizeof(m));
                gotNew = true;
            }
            pkt = udp.parsePacket(); // Pega a próxima, se tiver.
        }

        if (gotNew && m.transmitterId == TARGET_DECK_ID) {
            float dps = m.gyroDps; // Removemos fabsf() para permitir valores negativos!
            if (fabsf(dps) < 1.0f) {
                phaseInc = 0.0f;
            } else {
                // 200 DPS (33⅓ RPM) → 1000 Hz. Se for negativo (-200 DPS), a freq fica -1000 Hz.
                float freq = (dps / 200.0f) * CARRIER_FREQ;
                phaseInc = (freq * (float)LUT_SIZE) / (float)SAMPLE_RATE;
            }
            lastPktMs = millis();
        }
    }

    // Timeout de link
    if (millis() - lastPktMs > 200) phaseInc = 0.0f;

    // ── 2. Preenche buffer DMA ───────────────────────
    float inc = phaseInc;
    for (int i = 0; i < BUF_FRAMES; i++) {
        uint8_t vL, vR;
        if (fabsf(inc) < 0.001f) {
            vL = 128; vR = 128;           // Silêncio (DC mid-point)
        } else {
            int idxL = ((int)phase) & LUT_MASK;
            int idxR = ((int)(phase + LUT_SIZE / 4)) & LUT_MASK;  // +90°
            vL = sineLUT[idxL];
            vR = sineLUT[idxR];
            phase += inc;
            // Wrap-around seguro para ambas as direções (Frente/Trás)
            if (phase >= (float)LUT_SIZE) phase -= (float)LUT_SIZE;
            else if (phase < 0.0f) phase += (float)LUT_SIZE;
        }
        // I2S DAC: upper 8 bits of each 16-bit word → DAC value
        // Interleaved: [Right, Left]
        dmaBuf[i * 2]     = (uint16_t)vL << 8;   // DAC1 GPIO25
        dmaBuf[i * 2 + 1] = (uint16_t)vR << 8;   // DAC2 GPIO26
    }

    size_t written;
    i2s_write(I2S_NUM_0, dmaBuf, sizeof(dmaBuf), &written, portMAX_DELAY);

    // ── 3. Log periódico ─────────────────────────────
    static unsigned long lastLog = 0;
    if (millis() - lastLog > 1000) {
        lastLog = millis();
        float freq = (inc * (float)SAMPLE_RATE) / (float)LUT_SIZE;
        bool link = (millis() - lastPktMs) < 200;
        Serial.printf("DVS I2S | Freq: %.1f Hz | Inc: %.4f | Link: %s\n",
                      freq, inc, link ? "OK" : "--");
    }
}
