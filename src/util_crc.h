/*
 * util_crc.h
 *
 *  Created on: 2019年12月13日
 *      Author: kerwin.lo
 */

#ifndef AGV_CONTROLS_AGV_NOVEL_CONTROL_INCLUDE_UTIL_CRC_H_
#define AGV_CONTROLS_AGV_NOVEL_CONTROL_INCLUDE_UTIL_CRC_H_

#include <stdint.h>
// ---------------------------------------------------------------------------
// Initialization values for CRC high and low byte accumulators
// ---------------------------------------------------------------------------
#define CRC_HI      0xFF
#define CRC_LO      0xFF

// pilot2_Bingsyun_0029 start 改使用耗時更少的硬體CRC，原運算法持續於軟體模擬使用

#define CRCLEN      2
namespace agv {
uint16_t util_crc_calc(uint8_t *pdata, uint16_t ncount);
}
#endif  /* AGV_CONTROLS_AGV_NOVEL_CONTROL_INCLUDE_UTIL_CRC_H_ */
