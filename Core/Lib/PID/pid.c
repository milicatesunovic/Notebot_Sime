/*
 * pid.c
 *
 *  Created on: Mar 29, 2025
 *      Author: KORISNIK
 */
#include "pid.h"
#include <math.h>
#include <tim.h>
#include "stm32l432xx.h"
#include "../Odometrija/odometrija.h"
#include "../PID/pid.h"
#include"../Lib/Motori/motori.h"

//ovo zakomentarisano je Rakiceva vrijednost za parametre, ti skontaj sta ti treba
static float kp = 300; //400
static float ki = 0.2; //0.35

//promjenjive za motor1 u pid brzine1
static float e_m1 = 0.0;
static float e_pre_m1 = 0.0;

static float merena_m1 = 0.0;

static float u_p_m1 = 0.0;
static float u_i_m1 = 0.0;
static float u_m1 = 0.0;

//promjenjive za motor2 u pid brzine2
static float e_m2 = 0.0;
static float e_pre_m2 = 0.0;

static float merena_m2 = 0.0;

static float u_p_m2 = 0.0;
static float u_i_m2 = 0.0;
static float u_m2 = 0.0;

//provera smijera obrtanja motora
static uint16_t temp_pid = 0;
static uint8_t smer = 0;

static float s1 = 0.0;
static float s2 = 0.0;
static float s3 = 0.0;

static float brzine_motora = 0.0;

//Koordinate referentne tacke u ks robota pre pocetka bilo kakvog kretanja
static float reftksr_x = 0.0;//x koordinata referentne tacke u koordinatnom sistemu robota
static float reftksr_y = 0.0;//y koordinata referentne tacke u koordinatnom sistemu robota
static float teta = 0.0;//koordinata ugla referentne tacke u koordinatnom sistemu robota

float rastojanje_robota = 0.0; //rastojanje robota od referentne tacke
float rastojanje_robota_fi = 0.0;

volatile extern uint32_t sys_time;

static uint32_t brojac_pid = 0;

 float startna_pozicija_rob_x = 0.0;
 float startna_pozicija_rob_y = 0.0;
 float startni_fi = 0.0;

uint8_t flag_startne_pozicije = 0;
uint8_t flag_kraja_kretanja = 0;


//parametri pd regulacije ugla za linerno kretanje
static float merena_fi = 0.0;
static float e_fi = 0.0;
static float e_pre_fi = 0.0;
static float e_pre2_fi = 0.0;
static float u_fi = 0.0;
static float u_d_fi = 0.0;
static float u_p_fi = 0.0;
static float u_i_fi = 0.0;
static float kp_fi = 50.0;
static float kd_fi = 0.01;
static float ki_fi = 20.0;

uint8_t flag_stanje_kretanja = 1;

float epsilon_rastojanja = 0.0;
float epsilon_fi = 0.0;

//float niz_brz[1024];
//uint16_t brojac = 0;

uint8_t flag_kretanja = 1;
uint8_t flag_podaci = 1;
float fi_prva_faza = 0.0;
float fi_treca_faza = 0.0;
float rastojanje_pid_fajl = 0.0;
float predjeni_put_linearno = 0.0;
float predjeni_put_fi_prom = 0.0;
float brzine_pid_fajl = 0.0;
float rastojanje_rob_linearno = 0.0;


uint8_t flag_sledece_kretanje = 0;

uint8_t broj_puta = 0;

float udeo_brzine = 0.0 ;

int8_t n = 0;



void pid_brzina_m1(float ref)
{
	merena_m1 = odometrija_brzina_d();
	e_m1 =  ref  - merena_m1;

	u_p_m1 = kp * (e_m1 - e_pre_m1);
	u_i_m1 = ki * e_m1;
	u_m1 = u_m1 + u_p_m1 + u_i_m1;

	e_pre_m1 = e_m1;

	//saturacija, counter period za TIM15 i TIM16 je 100-1
	if(u_m1 > 1999)
	{
		u_m1 = 1999;
	}
	else if(u_m1 < -1999.0)
	{
		u_m1 = -1999.0;
	}

	if(u_m1 > 0)
	{
		smer = 2;
	}
	else
	{

		smer = 1;
	}

	//TIM16->CCR2 = (uint16_t)fabs(u_m1+2000.0);
	temp_pid = (uint16_t)fabs(u_m1+2000.0);
	motor1_init(smer, temp_pid);
}

void pid_brzina_m2(float ref)
{
	merena_m2 = odometrija_brzina_l();
	e_m2 =  ref  - merena_m2;

	u_p_m2 = kp * (e_m2 - e_pre_m2);
	u_i_m2 = ki * e_m2;
	u_m2 = u_m2 + u_p_m2 + u_i_m2;

	e_pre_m2 = e_m2;

	//saturacija
	if(u_m2 > 1999.0)
	{
		u_m2 = 1999.0;
	}
	else if(u_m2 < -1999.0)
	{
		u_m2 = -1999.0;
	}

	if(u_m2 > 0)
	{
		smer = 1;
	}
	else
	{
		smer = 2;
	}

	temp_pid = (uint16_t)fabs(u_m1+2000.0);
		motor2_init(smer, temp_pid);
}

float racunanje_rastojanja(float reftx, float refty){ //rtx - x koordinnata referentne tacke u globalnom ks stola / rty - ...

	float x = odom_x();
	float y = odom_y();
	float fi = odom_fi();

	reftksr_x = cos(fi) * reftx + sin(fi) * refty - (cos(fi) * x + sin(fi) * y);
	reftksr_y = cos(fi) * refty - sin(fi) * reftx - (cos(fi) * y - sin(fi) * x);

	rastojanje_robota = sqrt(reftksr_x * reftksr_x + reftksr_y * reftksr_y);

	return rastojanje_robota;
}

float racunanje_ofseta_ugala_prva_faza_kretanja(){

	teta = atan2(reftksr_y, reftksr_x);
	teta = teta * 57.2957795;
	return teta;
}

float racunanje_rastojanja_ugla(float rfi){

	float fi = odom_fi_deg();

	rastojanje_robota_fi = rfi - fi;

	return rastojanje_robota_fi;
}

float predjeni_put(){

	float predjeni_put = 0.0;
	float predjeni_put_x = 0.0;
	float predjeni_put_y = 0.0;

	float x = odom_x();
	float y = odom_y();
	//float fi = odom_fi();

	predjeni_put_x = x - startna_pozicija_rob_x;
	predjeni_put_y = y - startna_pozicija_rob_y;

	predjeni_put = sqrt(predjeni_put_x * predjeni_put_x + predjeni_put_y * predjeni_put_y);

	return predjeni_put;

}

float predjeni_put_fi(){

	float predjeni_put_fi = 0.0;

	float fi = odom_fi_deg();

	predjeni_put_fi = fi - startni_fi;

	return predjeni_put_fi;
}

float sinteza_trejktorije(float ubrzanje, float max_brzina, float rastojenje, float predjeni_put){

	rastojenje = fabs(rastojenje);
	predjeni_put = fabs(predjeni_put);

    max_brzina = (max_brzina * M_PI * 82.5) / 1000.0;

    if(rastojenje == 0){
    	flag_kraja_kretanja = 1;
    	return 0.0;
    }

	s1 = (max_brzina * max_brzina) / (2.0 * ubrzanje);
	s3 = s1;

	if((s1 + s3) == rastojenje){
		s2 = 0;
	}
	else if((s1 + s3) > rastojenje){
		s1 = rastojenje / 2.0;
		s3 = s1;
		s2 = 0;
	}
	else{
		s2 = rastojenje - s1 - s3;
	}

	predjeni_put = predjeni_put / 1000.0;

	if(predjeni_put <= s1){
		brzine_motora = sqrt(2.0 * predjeni_put * ubrzanje) + 0.01;
		brzine_motora = 1000.0 * (brzine_motora / (M_PI * 82.5));

		return brzine_motora;

	}
	else if((predjeni_put >= s1) && (predjeni_put <= (s1 +  s2))){
		brzine_motora = max_brzina;
		brzine_motora = 1000.0 * (brzine_motora / (M_PI * 82.5));

		return brzine_motora;

	}
	else if((predjeni_put >= (s1 + s2)) && (predjeni_put <= rastojenje)){
		brzine_motora = sqrt(max_brzina * max_brzina - 2.0 * ubrzanje * (predjeni_put - s1 - s2));
		brzine_motora = 1000.0 * (brzine_motora / (M_PI * 82.5));

		return brzine_motora;

    }
	else{
		flag_kraja_kretanja = 1;

		return 0.0;
	}

}

float sinteza_trajektorije_rotacija(float ubrzanje, float max_brzina, float rastojanje, float predjeni_put){

	rastojanje = fabs(rastojanje);
	predjeni_put = fabs(predjeni_put);

	if(rastojanje == 0){
		flag_kraja_kretanja = 1;
		return 0.0;
	}

	s1 = (max_brzina * max_brzina) / (2.0 * ubrzanje);
	s3 = s1;

	if((s1 + s3) == rastojanje){
		s2 = 0;
	}
	else if((s1 + s3) > rastojanje){
		s1 = rastojanje / 2.0;
		s3 = s1;
		s2 = 0;
	}
	else{
		s2 = rastojanje - s1 - s3;
	}

	if(predjeni_put <= s1){
			brzine_motora = sqrt(2.0 * predjeni_put * ubrzanje) + 30;
			brzine_motora = brzine_motora / 360.0;
			return brzine_motora;

		}
		else if((predjeni_put >= s1) && (predjeni_put <= (s1 +  s2))){
			brzine_motora = max_brzina;
			brzine_motora = brzine_motora / 360.0;
			return brzine_motora;

		}
		else if((predjeni_put >= (s1 + s2)) && (predjeni_put <= rastojanje)){
			brzine_motora = sqrt(max_brzina * max_brzina - 2.0 * ubrzanje * (predjeni_put - s1 - s2));
		    brzine_motora = brzine_motora / 360.0;
			return brzine_motora;

	    }
		else{
			flag_kraja_kretanja = 1;
			return 0.0;
		}
}
float pd_regulacija(float ref){

	merena_fi = odom_fi_deg();
	if((merena_fi > 360) || (merena_fi < -360)){
		n = (int)(merena_fi / 360);
		merena_fi = merena_fi - n * 360;
	}

	e_fi = ref - merena_fi;

	if(((ref >= 0 && ref <= 180) && (merena_fi >= 0 && merena_fi <= 360)) ||
			((ref <= 0 && ref >= -180) && (merena_fi <= 0 && merena_fi >= -360))){
		e_fi = ref - merena_fi;
	}

	else if((ref >= 0 && ref <= 180) && (merena_fi <= 0 && merena_fi >= -360)){
		merena_fi += 360;
		e_fi = ref - merena_fi;
	}

	else if((ref <= 0 && ref >= -180) && (merena_fi >= 0 && merena_fi <= 360)){
		merena_fi -= 360;
		e_fi = ref - merena_fi;
	}

	u_p_fi = kp_fi * (e_fi - e_pre_fi);
	u_i_fi = ki_fi * e_fi;
    u_d_fi = kd_fi * (e_fi - 2 * e_pre_fi + e_pre2_fi);
	u_fi = u_fi + u_p_fi + u_i_fi;

	u_fi /= 360.0;

    e_pre2_fi = e_pre_fi;
	e_pre_fi = e_fi;

	return u_fi;
}

void kretanje(float referenca_x, float referenca_y, float orjentacija){

	if(flag_stanje_kretanja == 1){

		if(flag_podaci == 1){
				start_x();
				start_y();
				start_fi();
				rastojanje_pid_fajl = racunanje_rastojanja(referenca_x, referenca_y);
				fi_prva_faza = racunanje_ofseta_ugala_prva_faza_kretanja();
				if(fi_prva_faza > 0.0){
					smer = 1;
				}
				else if(fi_prva_faza < 0.0){
					smer = 2;
				}
		       flag_podaci = 2;
			}

		if(sys_time % 15 == 0){
					predjeni_put_fi_prom = predjeni_put_fi();
					brzine_pid_fajl =  sinteza_trajektorije_rotacija(3500, 300, fi_prva_faza, predjeni_put_fi_prom);
				}


				if(smer == 1){
					pid_brzina_m1(brzine_pid_fajl);
					pid_brzina_m2(-brzine_pid_fajl);
				}
				else if(smer == 2){
					pid_brzina_m1(-brzine_pid_fajl);
					pid_brzina_m2(brzine_pid_fajl);
				}
	}

}
void kretanje_rot(float referenca_x, float referenca_y, float orjentacija){

	if(flag_stanje_kretanja == 3){

		if(flag_podaci == 3 && flag_stanje_kretanja == 3){
			    brojac_pid++;
			    start_fi();

				fi_prva_faza = orjentacija - startni_fi;

				if(fi_prva_faza > 180){
					broj_puta = (int)(fi_prva_faza / 360.0);
					fi_prva_faza -= (broj_puta * 360.0);

					if(fi_prva_faza > 180.0){
						fi_prva_faza -= 360.0;
					}
				}

				else if(fi_prva_faza < -180.0){
					broj_puta = (int)(fi_prva_faza / (-360.0));
					fi_prva_faza += (broj_puta * 360.0);

					if(fi_prva_faza < -180.0){
						fi_prva_faza += 360.0;
				    }
				}

				if(fi_prva_faza > 0.0){
					smer = 1;
				}
				else if(fi_prva_faza < 0.0){
					smer = -1;
				}

		       flag_podaci = 1;
			}

		if(sys_time % 15 == 0){
			        //kp = 150 ki = 8
			        // za ugao 90 ubrzanje 3500 brzina 450
			        // za ugao 45 ubrzanje 3500 brzina 300
					predjeni_put_fi_prom = predjeni_put_fi();
					brzine_pid_fajl =  sinteza_trajektorije_rotacija(3500, 300, fi_prva_faza, predjeni_put_fi_prom);
				}



				if(smer == 1){
					pid_brzina_m1(brzine_pid_fajl);
					pid_brzina_m2(-brzine_pid_fajl);
				}
				else if(smer == 2){
					pid_brzina_m1(-brzine_pid_fajl);
					pid_brzina_m2(brzine_pid_fajl);
				}
	}

}
void kretanje_lin(float referenca_x, float referenca_y, float orjentacija){

	if(flag_stanje_kretanja == 2){
		if(flag_podaci == 2){
			start_x();
			start_y();
			start_fi();
			rastojanje_rob_linearno = racunanje_rastojanja(referenca_x, referenca_y);
			rastojanje_rob_linearno = rastojanje_rob_linearno / 1000.0;
			u_fi = 0.0;
			u_i_fi = 0.0;
			u_p_fi = 0.0;
			flag_podaci = 3;
        }
	   if(sys_time % 15 == 0){
		    predjeni_put_linearno = predjeni_put();
		    brzine_pid_fajl = sinteza_trejktorije(0.785, 2.5 , rastojanje_rob_linearno, predjeni_put_linearno);
		    udeo_brzine = pd_regulacija(orjentacija);
	    }


	   if(rastojanje_rob_linearno - predjeni_put_linearno > 15){
		   pid_brzina_m1((brzine_pid_fajl + udeo_brzine));
		   pid_brzina_m2((brzine_pid_fajl - udeo_brzine));
	   }

	   else{
		   pid_brzina_m1((brzine_pid_fajl ));
		   pid_brzina_m2((brzine_pid_fajl));
	   }


    }
}
void celo_kretanje(float referenca_x, float referenca_y, float orjentacija){

	if(flag_stanje_kretanja == 1){
				flag_kraja_kretanja = 0;
				kretanje(referenca_x, referenca_y, orjentacija);
				if(flag_kraja_kretanja == 1 && flag_stanje_kretanja == 1){
					if(timer_delay(40)){
						flag_kraja_kretanja = 0;
						flag_stanje_kretanja = 2;
						flag_podaci = 2;
					}

				}
			}

			else if(flag_stanje_kretanja == 2){
	            flag_kraja_kretanja = 0;
	            kretanje_lin(referenca_x, referenca_y, orjentacija);
	            if(flag_kraja_kretanja == 1 && flag_stanje_kretanja == 2){
	            	if(timer_delay(40)){
	            	   flag_kraja_kretanja = 0;
	            	   flag_stanje_kretanja = 4;
	            	   flag_podaci = 4;
	            	}

	            }

			}

//			else if(flag_stanje_kretanja == 3){
//				 flag_kraja_kretanja = 0;
//				 kretanje_rot(referenca_x, referenca_y, orjentacija);
//				 if(flag_kraja_kretanja == 1 && flag_stanje_kretanja == 3){
//					 if(timer_delay(20)){
//					 	flag_kraja_kretanja = 0;
//					    flag_stanje_kretanja = 4;
//					 	flag_podaci = 4;
//					 }
//
//				 }
//			}

			else if(flag_stanje_kretanja == 4){
				pid_brzina_m1(0);
				pid_brzina_m2(0);
				flag_sledece_kretanje++;
				flag_kraja_kretanja = 0;
				flag_stanje_kretanja = 1;
                flag_podaci = 1;
			}
			else{
				pid_brzina_m1(0);
			    pid_brzina_m2(0);
			}

}

void start_x(){//dodbijanje startne pozicije robota za zapoceto kretanje

	startna_pozicija_rob_x = odom_x();
}

void start_y(){

	startna_pozicija_rob_y = odom_y();
}

void start_fi(){

	startni_fi = odom_fi_deg();
}


float get_start_x(){//dodbijanje startne pozicije robota za zapoceto kretanje

	return startna_pozicija_rob_x;
}

float get_start_y(){

	return startna_pozicija_rob_y;
}

float get_startni_fi(){

	return startni_fi;
}




