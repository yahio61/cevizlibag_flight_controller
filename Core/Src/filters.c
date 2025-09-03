/*
 * filters.c
 *
 *  Created on: Aug 28, 2025
 *      Author: yahya
 */

#include "filters.h"
#include <math.h>
#include "main.h"

#define ACCEL_BUFFER_SIZE 5
static float accel_buffer[ACCEL_BUFFER_SIZE];
static uint8_t accel_buffer_index = 0;
static uint8_t accel_buffer_full = 0;

#define ALT_BUFFER_SIZE 5
static float alt_buffer[ALT_BUFFER_SIZE];
static uint8_t alt_buffer_index = 0;
static uint8_t alt_buffer_full = 0;

#define KB	0.001
float last_time = 0;

static uint8_t detect_accel_failure(float accel);
static uint8_t detect_altitude_failure(float altitude);

void baf_init(BaroAccelFilter *f, float a0, float alpha, float beta, float time)
{
	f->h = 0.0;
    f->v = 0;
    f->abias = a0 * TO_SI;
    f->alpha = alpha;
    f->beta = beta;
    f->last_time = time;
}

// h_meas: baro'dan hesaplanmış anlık irtifa (m)
// a_meas: ivme sensörünün z-ekseni ölçümü (m/s^2), + yukarı (sensör eksenini uygun yönde ayarla)
void baf_step(BaroAccelFilter *f, float h_meas, float a_meas, float time)
{
	a_meas = a_meas * TO_SI;

	//if(detect_altitude_failure(h_meas) || detect_accel_failure(a_meas))
		//return;

	float dt = time - f->last_time;
	dt = dt / 1000.0;
	f->dt = dt;
	f->last_time = time;

    float a_corr = a_meas - f->abias;   // bias düzeltilmiş ivme
    float v_pred = f->v + a_corr * dt;
    float h_pred = f->h + v_pred * dt;  // bir adım öngörü (basit Euler)

	// alpha_c = 0.98 -> %98 model (ivme) %2 baro
	float alpha_c = 0.8f;

	h_pred = f->h + f->v * dt + 0.5f * a_corr * dt * dt;
	v_pred = f->v + a_corr * dt;

	f->h = alpha_c * h_pred + (1.0f - alpha_c) * h_meas;
	f->v = alpha_c * v_pred + (1.0f - alpha_c) * (/* baro-based v if available or keep v_pred */ (h_meas - f->last_h_meas) / dt);
	f->last_h_meas = h_meas;

/*
    // 1) Öngörü: ivmeyi kullanarak hız/durum tahmini
    float a_corr = a_meas - f->abias;   // bias düzeltilmiş ivme
    float v_pred = f->v + a_corr * dt;
    float h_pred = f->h + v_pred * dt;  // bir adım öngörü (basit Euler)

    // 2) Ölçüm hatası
    float r = h_meas - h_pred;

    // 3) Güncelleme (α–β)
    f->h = h_pred + f->alpha * r;
    // v güncellemesi: beta bölü dt ile r kullan
    f->v = v_pred + (f->beta / dt) * r;

    // 4) Ivme bias adaptif güncelle (very slow)
    // Eğer baro güvenliyse residual'dan bias'ı azalt
    // kb çok küçük olmalı (örn 1e-3 .. 1e-4)
    f->abias += KB * r; // r pozitifse bias azalması/artanına göre ayarla

    // (Opsiyonel) limitleme
    if (f->abias > 10.0f) f->abias = 10.0f;
    if (f->abias < -10.0f) f->abias = -10.0f;
    */
}



static float calculate_alt_std_deviation(void)
{
    if (!alt_buffer_full && alt_buffer_index < 2) return 0.0f;
    int count = alt_buffer_full ? ALT_BUFFER_SIZE : alt_buffer_index;
    float sum = 0.0f, mean = 0.0f, variance = 0.0f;
    for (int i = 0; i < count; i++) {
        sum += alt_buffer[i];
    }
    mean = sum / count;
    for (int i = 0; i < count; i++) {
        variance += (alt_buffer[i] - mean) * (alt_buffer[i] - mean);
    }
    variance /= count;
    return sqrtf(variance);
}

static uint8_t detect_altitude_failure(float altitude)
{
    alt_buffer[alt_buffer_index] = altitude;
    alt_buffer_index = (alt_buffer_index + 1) % ALT_BUFFER_SIZE;
    if (alt_buffer_index == 0) {
        alt_buffer_full = 1;
    }
    float std_dev = calculate_alt_std_deviation();
    if (altitude > ALT_MAX_VALUE || altitude < ALT_MIN_VALUE || std_dev > ALT_MAX_STD) {
        return 1;
    }
    return 0;
}

/**
 * @brief İvme değerlerinin standart sapmasını hesapla
 * @return Standart sapma değeri
 */
static float calculate_accel_std_deviation(void)
{
    // Buffer dolmadıysa ve çok az veri varsa
    if (!accel_buffer_full && accel_buffer_index < 2) return 0.0f;

    int count = accel_buffer_full ? ACCEL_BUFFER_SIZE : accel_buffer_index;
    float sum = 0.0f;
    float mean = 0.0f;
    float variance = 0.0f;

    // Ortalama hesapla
    for (int i = 0; i < count; i++) {
        sum += accel_buffer[i];
    }
    mean = sum / count;

    // Varyans hesapla
    for (int i = 0; i < count; i++) {
        variance += (accel_buffer[i] - mean) * (accel_buffer[i] - mean);
    }
    variance /= count;

    return sqrtf(variance);
}

/**
 * @brief İvme sensörünün arızalı olup olmadığını kontrol et
 * @param accel İvme değeri (m/s²)
 * @return 1: Arıza tespit edildi, 0: Arıza yok
 */
static uint8_t detect_accel_failure(float accel)
{
    float accel_abs = fabsf(accel);


    // Buffer güncelleme
    accel_buffer[accel_buffer_index] = accel_abs;
    accel_buffer_index = (accel_buffer_index + 1) % ACCEL_BUFFER_SIZE;
    if (accel_buffer_index == 0) {
        accel_buffer_full = 1;
    }

    // Standart sapma hesapla
    float std_dev = calculate_accel_std_deviation();

    // İvme değeri veya standart sapma limitler dışındaysa
    if (accel_abs > ACCEL_MAX_VALUE_THRUST || std_dev > ACCEL_MAX_STD_THRUST) {
        return 1;  // Arıza tespit edildi
    }

    return 0;  // Arıza yok
}
