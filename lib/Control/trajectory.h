#ifndef TRAJECTORY_H
#define TRAJECTORY_H

#include <stdint.h>

/*
 * S-curve trajectory generator 7-segmen (jerk-limited).
 *
 * Profil kecepatan dibangun dari 7 fase:
 *   1. jerk +j      -> percepatan naik 0 -> alim
 *   2. percepatan konstan alim
 *   3. jerk -j      -> percepatan turun alim -> 0, kecepatan capai vlim
 *   4. kecepatan konstan vlim
 *   5. jerk -j      -> percepatan turun 0 -> -alim
 *   6. percepatan konstan -alim
 *   7. jerk +j      -> percepatan naik -alim -> 0, kecepatan capai 0
 *
 * Jika jarak terlalu pendek untuk mencapai v_max dan/atau a_max, durasi
 * tiap fase otomatis dihitung ulang (profil segitiga / murni-jerk).
 * Asumsi: kecepatan awal & akhir = 0 (start/stop dari diam).
 */

typedef struct {
    /* ---- Parameter input ---- */
    float start_pos;
    float target_pos;
    float v_max;    /* batas kecepatan maksimum (magnitude, > 0) */
    float a_max;    /* batas percepatan maksimum (magnitude, > 0) */
    float j_max;    /* batas jerk maksimum (magnitude, > 0) */

    /* ---- Hasil perencanaan (diisi oleh s_curve_start) ---- */
    float direction;   /* +1.0f atau -1.0f, arah gerak */
    float distance;    /* |target_pos - start_pos| */

    float Tj1;   /* durasi tiap fase jerk */
    float Ta;    /* durasi total fase percepatan (fase 1-3) */
    float Tv;    /* durasi fase kecepatan konstan (fase 4) */
    float Td;    /* durasi total fase perlambatan (fase 5-7), = Ta */
    float alim;  /* percepatan puncak aktual yang tercapai (<= a_max) */
    float vlim;  /* kecepatan puncak aktual yang tercapai (<= v_max) */

    /* waktu kumulatif akhir tiap fase, diukur dari t = 0 */
    float t1, t2, t3, t4, t5, t6, t7;

    /* posisi & kecepatan (magnitude sepanjang arah gerak) di tiap
     * titik batas fase, dipakai s_curve_update supaya tidak perlu
     * integrasi ulang dari t = 0 setiap kali dipanggil */
    float p1, v1;
    float p2, v2;
    float p3, v3;
    float p4;
    float p5, v5;
    float p6, v6;

    /* ---- State runtime ---- */
    float time;
    float pos;
    float velocity;
    float acceleration;
    uint8_t finished;
} s_curve_t;

int8_t s_curve_start(s_curve_t *sc, float start_pos, float target_pos);
void s_curve_set_v_max(s_curve_t *sc, float v_max);
void s_curve_set_a_max(s_curve_t *sc, float a_max);
void s_curve_set_j_max(s_curve_t *sc, float j_max);

/* Memajukan trajektori sebesar dt detik.
 * Return 0 jika sukses, -1 jika trajektori sudah selesai / sc NULL. */
int8_t s_curve_update(s_curve_t *sc, float dt);

float s_curve_get_pos(s_curve_t *sc);
float s_curve_get_vel(s_curve_t *sc);
float s_curve_get_accel(s_curve_t *sc);
float s_curve_get_duration(const s_curve_t *sc);

#endif /* TRAJECTORY_H */