/*
 * odometrija.c
 *
 *  Created on: Mar 26, 2025
 *      Author: KORISNIK
 */


#include "odometrija.h"
#include "tim.h"
#include <math.h>
#include "stm32l432xx.h"


static float R=83.2;// precnik pogonskog tocka
static float b=129;


static int16_t vd_inc=0;
static int16_t vl_inc=0;
static float vd=0;
static float vl=0;

static float vd_mm=0;
static float vl_mm=0;

static float V=0;
static float w=0;
static float fi=0;
static float fi_deg=0;
static float x=0;
static float y=0;

static float inc2mm;


void odometrija_init() {
	    R = 82.5;
		b = 128.60;

		float Oot = R * M_PI;
		float N = 1000.0 / Oot;
		float n = N * 546 * 4;   //13*42
		inc2mm = 1000.0 / n;
}

void odometrija() {
	vd_inc=tim1_brzina();
	vl_inc = tim2_brzina();

	vd = ((float)vd_inc /2184.0)*1000; //(float) vd_inc * inc2mm;
//	vd_mm = (float) vd_inc * 3.14*R;
	vl = ((float)vl_inc /2184.0) *1000; //(float) vl_inc * inc2mm;
//	vl_mm = (float) vl_inc * 3.14*R;

	vd_mm = (float) vd_inc * inc2mm;
	vl_mm = (float) vl_inc * inc2mm;
	//vd=vd_mm;
	//vl=vl_mm;

	V = (vd_mm + vl_mm) / 2.0;
	w = (vd_mm - vl_mm) / b;

	fi += w;
    fi_deg = fi * 57.2957795;    //ovo je za racunanje u stepenima, ali je bolje racunati sin i cos u radianima!!!
//  x  += (V * cos(fi_deg));
//  y  += (V * sin(fi_deg));
//	fi_deg = fi * 57.2957795;
	x += (V* cos(fi));
	y += (V * sin(fi));
}

//ovo su sve funkcije za vracanje vrednosti

float odom_x(){
	return x;
}

float odom_y(){
	return y;
}

float odom_fi(){
	return fi;
}

float odom_fi_deg(){
	return fi_deg;
}

float odometrija_brzina_d()
{
	return vd_mm;
}

float odometrija_brzina_l()
{
	return vl_mm;
}
