#include <stdio.h>
#include <math.h>
#include "currentSensor.h"


#define VOLTS 127
#define ASCII_VALUE 48


float cost=.793;
float acumulateMoney=0;

float getCurrent(uint16_t *pp)
{
	float suma;
	for(uint8_t i=0;i<15;i++)
	{
		if(pp[i]>3000)
		{
		suma=sqrt((.080566*pp[i])*(.080566*pp[i]));
		}
	}
	return ((suma/15)/100);
}


float getWatts(float pp)
{
	return (pp*VOLTS);
}
float getCost(uint32_t pp)
{
	return ((getWatts(pp)/1000)*.793);
}
void setCost(uint32_t newcost)
{
	cost=newcost;
}
void proCost(uint32_t money)
{
	acumulateMoney=money;
}

uint8_t Converter_IntToChar(uint16_t ent)/**This function is for convert in number ASCII.*/
{
	return (ent+ASCII_VALUE);
}
void separar_2d_unidades(float entero ,uint16_t *c1, uint16_t *c2)/**	 	 This function is used when
 we need separate tens and units in a float*/
{	uint16_t c1_p;
	uint16_t c2_p=entero/1;

	for(c1_p=0;(c2_p-9)>0;c1_p++)
	{
		c2_p=c2_p-10;
	}

	*c1=c1_p;
	*c2=c2_p;

}
uint8_t separar_2d_enteros_2d_decimales(float entero,uint16_t *c1, uint16_t *c2, uint16_t *d1, uint16_t *d2)/** This function is used when we
need separate tens and units in a float*/
{
		uint8_t sig;
		uint16_t ent;
		uint16_t c1_p=0;
		uint16_t c2_p=0;
		uint16_t d1_p=0;
		uint16_t d2_p=0;
		uint16_t ent1;
		sig=0;
		if(entero<0){
			sig=1;
			entero=entero*-1;
		}
		ent=(entero*100)/1;
		separar_2d_unidades(ent,&ent1,&d2_p);
		separar_3d_enteros(ent1,&c1_p,&c2_p,&d1_p);
		*c1=c1_p;
		*c2=c2_p;
		*d1=d1_p;
		*d2=d2_p;
		return sig;
}
void separar_2d_unidades_uin8(uint8_t entero,uint8_t *c1, uint8_t *c2)/**This function is used when we need separate tens and units*/
{	uint8_t c1_p;
	uint8_t c2_p=entero/1;

	for(c1_p=0;(c2_p-9)>0;c1_p++)
	{
		c2_p=c2_p-10;
	}

	*c1=c1_p;
	*c2=c2_p;

}
void separar_3d_enteros(float entero,uint16_t *c1, uint16_t *c2, uint16_t *c3)/** This function is used when we need separate decimals and enters*/
{	uint16_t ent2;
	uint16_t c1_p=0;
	uint16_t c2_p=0;
	uint16_t c3_p=0;

	separar_2d_unidades((entero/1),&ent2,&c3_p);
	separar_2d_unidades(ent2,&c1_p,&c2_p);
	*c1=c1_p;
	*c2=c2_p;
	*c3=c3_p;
}
uint8_t separar_3d_enteros_2d_decimales(float entero,uint16_t *c1, uint16_t *c2, uint16_t *c3,uint16_t *d1, uint16_t *d2)/**  This function is used when we need separate decimals and enters in numbers >100*/
{
	uint8_t sig;
	uint16_t ent1;
	uint16_t ent;
	uint16_t c1_p=0;
	uint16_t c2_p=0;
	uint16_t c3_p=0;
	uint16_t d1_p=0;
	uint16_t d2_p=0;
	sig=0;
	if(entero<0){
		sig=1;
		entero=entero*-1;
	}
		ent=(entero*100)/1;
		separar_3d_enteros(ent,&ent1,&d2_p,&d1_p);
		separar_3d_enteros(ent1,&c1_p,&c2_p,&c3_p);
		*c1=c1_p;
		*c2=c2_p;
		*c3=c3_p;
		*d1=d1_p;
		*d2=d2_p;
		return sig;
	}
