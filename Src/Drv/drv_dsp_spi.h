/*
 * drv_dsp_spi.h
 *
 *  Created on: 2019. 6. 12.
 *      Author: hrjung
 */

#ifndef SRC_DRV_DSP_SPI_H_
#define SRC_DRV_DSP_SPI_H_

#define SPI_NAK 0
#define SPI_ACK 1

extern int8_t SPI_writeDSP(uint16_t *pBuf, uint16_t len);
extern int8_t SPI_readDSP(uint16_t *rxBuf, uint16_t rxlen);

#endif /* SRC_DRV_DSP_SPI_H_ */
