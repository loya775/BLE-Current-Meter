#ifndef _CURRENTSENSOR_H_
#define _CURRENTSENSOR_H_

#include "fsl_debug_console.h"
#include "fsl_flash.h"
#include "fsl_clock.h"


float getCurrent(uint16_t *pp);
float getWatts(float pp);
float getCost(uint32_t pp);
void setCost(uint32_t newcost);
void proCost(uint32_t money);
uint8_t Converter_IntToChar(uint16_t ent);
void separar_2d_unidades(float entero ,uint16_t *c1, uint16_t *c2);
/********************************************************************************************/
/********************************************************************************************/
/********************************************************************************************/
/*!
	 \brief
	 	 This function is used when we need separate tens and units
	 \param[in] uint8 entero,uint8 *c1, uint8 *c2
	 \return void

*/
void separar_2d_unidades_uin8(uint8_t entero,uint8_t *c1, uint8_t *c2);
/********************************************************************************************/
/********************************************************************************************/
/********************************************************************************************/
/*!
	 \brief
	 	 This function is used when we need separate decimals and enters
	 \param[in] float64 entero,uint16 *c1, uint16 *c2, uint16 *d1, uint16 *d2
	 \return void

*/
uint8_t separar_2d_enteros_2d_decimales(float entero,uint16_t *c1, uint16_t *c2, uint16_t *d1, uint16_t *d2);
/********************************************************************************************/
/********************************************************************************************/
/********************************************************************************************/
/*!
	 \brief
	 	 This function is used when we need separate decimals and enters
	 \param[in] float64 entero,uint16 *c1, uint16 *c2, uint16 *d1, uint16 *d2
	 \return void

*/
void separar_3d_enteros(float entero,uint16_t *c1, uint16_t *c2, uint16_t *c3);
/********************************************************************************************/
/********************************************************************************************/
/********************************************************************************************/
/*!
	 \brief
	 	 This function is used when we need separate decimals and enters in numbers >100
	 \param[in] float64 entero,uint16 *c1, uint16 *c2, uint16 *c3,uint16 *d1, uint16 *d2
	 \return void

*/
uint8_t separar_3d_enteros_2d_decimales(float entero,uint16_t *c1, uint16_t *c2, uint16_t *c3,uint16_t *d1, uint16_t *d2);


#endif /* _APP_H_ */
