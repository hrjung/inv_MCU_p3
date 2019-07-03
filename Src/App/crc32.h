/*
 * crc32.h
 *
 *  Created on: 2019. 6. 3.
 *      Author: hrjung
 */

#ifndef SRC_CRC32_H_
#define SRC_CRC32_H_

#include <stdint.h>

#define POLYNOMIAL 0x04c11db7L      // Standard CRC-32 ppolynomial

void gen_crc_table(void);
uint32_t update_crc(uint32_t crc_accum, uint8_t* data_blk_ptr, uint32_t data_blk_size);

#endif /* SRC_CRC32_H_ */
