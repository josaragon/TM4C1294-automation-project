/******************************************************************************
*  Filename:       tmp007.h
*  Revised:        
*  Revision:       
*
*  Description:    Interface to the IR temperature sensor driver TI TMP007.
*
*  Copyright (C) 2014 Texas Instruments Incorporated - http://www.ti.com/
*
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*    Redistributions of source code must retain the above copyright
*    notice, this list of conditions and the following disclaimer.
*
*    Redistributions in binary form must reproduce the above copyright
*    notice, this list of conditions and the following disclaimer in the
*    documentation and/or other materials provided with the distribution.
*
*    Neither the name of Texas Instruments Incorporated nor the names of
*    its contributors may be used to endorse or promote products derived
*    from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
*  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
*  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
*  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
*  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
*  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
*  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
*  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
*  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*
******************************************************************************/
#ifndef TMP007_H
#define TMP007_H

#ifdef __cplusplus
extern "C"
{
#endif

/*********************************************************************
 * INCLUDES
 */
  
/*********************************************************************
 * CONSTANTS
 */
/* TMP006 register addresses */
#define TMP007_REG_ADDR_VOLTAGE         0x00
#define TMP007_REG_ADDR_LOCAL_TEMP      0x01
#define TMP007_REG_ADDR_CONFIG          0x02
#define TMP007_REG_ADDR_OBJ_TEMP        0x03
#define TMP007_REG_ADDR_STATUS          0x04
#define TMP007_REG_ADDR_STATUS_ENABLE   0x05
#define TMP007_REG_ADDR_TC0_COEFFICIENT 0x11
#define TMP007_REG_ADDR_TC1_COEFFICIENT 0x12
#define TMP007_REG_PROD_ID              0x1F

#define TMP007_I2C_ADDRESS              0x40

/*********************************************************************
 * TYPEDEFS
 */

/*********************************************************************
 * FUNCTIONS
 */
extern bool sensorTmp007Init(void);
extern bool sensorTmp007Enable(bool enable);
extern bool sensorTmp007EnableInterruptConversion(bool enable);
extern bool sensorTmp007Test(void);
extern bool sensorTmp007Read(int16_t *rawTemp, int16_t *rawObjTemp);

extern void sensorTmp007Convert(int16_t rawTemp, int16_t rawObjTemp, float *tObj, float *tAmb);
extern uint16_t sensorTmp007DevID(void);
extern uint16_t sensorTmp007REG(uint8_t TMP007_REG);


#ifdef __cplusplus
}
#endif

#endif /* TMP007_H */
