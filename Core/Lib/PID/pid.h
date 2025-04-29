/*
 * pid.h
 *
 *  Created on: Mar 29, 2025
 *      Author: KORISNIK
 */

#ifndef LIB_PID_PID_H_
#define LIB_PID_PID_H_
void
pid_brzina_m1(float ref);
void
pid_brzina_m2(float ref);


float
sinteza_trejktorije(float ubrzanje, float max_brzina, float rastojenje, float predjeni_put);
float
sinteza_trajektorije_rotacija(float ubrzanje, float max_brzina, float rastojanje, float predjeni_put);
float
racunanje_rastojanja(float rtx, float rty);
void
start_x();
void
start_y();
void
start_fi();
float
get_start_x();
float
get_start_y();
float
get_startni_fi();
float
predjeni_put();
float
racunanje_rastojanja_ugla(float rfi);
float
predjeni_put_fi();
float
pd_regulacija(float ref);
float
racunanje_ofseta_ugala_prva_faza_kretanja();
void
kretanje(float referenca_x, float referenca_y, float orjentacija);
void
kretanje_lin(float referenca_x, float referenca_y, float orjentacija);
void
kretanje_rot(float referenca_x, float referenca_y, float orjentacija);
void
celo_kretanje(float referenca_x, float referenca_y, float orjentacija);

#endif /* LIB_PID_PID_H_ */
