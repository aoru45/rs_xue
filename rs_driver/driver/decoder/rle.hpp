/******************************************************************************/
/*                                                                            */
/* Copyright (C) 2014 - 2022 RoboSense, Co., Ltd.  All rights reserved.       */
/*                                                                            */
/******************************************************************************/
/**
 * @file    PcC.c
 * @brief   Point Cloud Compression code.
 */
#pragma once
#ifdef _MSC_VER
//#include <boost/cstdint.hpp>
//typedef boost::uint8_t uint8_t;
//#else
#include <stdint.h>
#endif
#include <stdio.h>
#include <stdlib.h>
#include <iostream>
#include <rs_driver/common/rs_log.hpp>
#define ENCODE_ERROR_FLAG                  (0xFFFFU)
#define DECODE_ERROR_FLAG                  (0xFFFFU)
/******************************************************************************/
/*                Include common and project definition header                */
/******************************************************************************/

/******************************************************************************/
/*                      Include headers of the component                      */
/******************************************************************************/
//#include "rle.h"

/******************************************************************************/
/*                           Include other headers                            */
/******************************************************************************/
//#include "PcDrv.h"

/******************************************************************************/
/*                   Definition of local symbolic constants                   */
/******************************************************************************/

/******************************************************************************/
/*                  Definition of local function like macros                  */
/******************************************************************************/
#define REPETITION_COMPRESSION_MIN_LEN     3U
#define OFFSET_COMPRESSION_MIN_LEN         3U
#define MAX_DISTANCE                       (0x7FU)
#define MAX_COMPRESSION_BLOCK_LEN          (0x7FU)
#define COMPRESSION_BLOCK_LEN_MASK         (0x7FU)
#define COMPRESSION_BLOCK_TYPE_MASK        (0x80U)

/******************************************************************************/
/*          Definition of local types (typedef, enum, struct, union)          */
/******************************************************************************/

/******************************************************************************/
/*                       Definition of local variables                        */
/******************************************************************************/
//uint8_t au8OffsetBuf[128];

/******************************************************************************/
/*                     Definition of local constant data                      */
/******************************************************************************/

/******************************************************************************/
/*                      Definition of exported variables                      */
/******************************************************************************/

/******************************************************************************/
/*                    Definition of exported constant data                    */
/******************************************************************************/

/******************************************************************************/
/*                    Definition of exported constant data                    */
/******************************************************************************/

/******************************************************************************/
/*                  Declaration of local function prototypes                  */
/******************************************************************************/


namespace robosense
{
namespace lidar
{
class RLE
{
public:
  static bool u32OffsetRLEDecode(uint8_t* pu8RawDataBuf, uint32_t u32RawDataLen, 
                                    uint16_t* pu16OutputBuf, uint32_t u32OutputBufSize, uint32_t& u32OutPutDecodeCnt)
  {
    uint32_t u32RemainLen = u32RawDataLen;
    uint16_t u16StartVal = 0;
    uint8_t* pu8RawData = pu8RawDataBuf;
    uint32_t u32DecodeCnt = 0;
    uint32_t u32Ret = 0;

    while(u32RemainLen > 0)
    {
        uint8_t u8Head = *pu8RawData;
        uint8_t u8Len = u8Head & COMPRESSION_BLOCK_LEN_MASK;
        pu8RawData += 1;
        u32RemainLen -= 1;
        if ((u8Head & COMPRESSION_BLOCK_TYPE_MASK) != 0)
        {
            if ((u32DecodeCnt + u8Len) > u32OutputBufSize)
            {
                u32Ret = DECODE_ERROR_FLAG;
                break;
            }
            u16StartVal = (uint16_t)pu8RawData[0] << 8 | (uint16_t)pu8RawData[1];
            pu8RawData += 2;
            u32RemainLen -= 2;
            if (u32DecodeCnt > 519) {
              RS_WARNING << "u32DecodeCnt > 519 " << " val:" << u32DecodeCnt << RS_REND;
              break;
            }

            pu16OutputBuf[u32DecodeCnt++] = u16StartVal;
            for (uint8_t u8Index = 0; u8Index < (u8Len - 1); u8Index++)
            {
                if (u32DecodeCnt > 519) {
                  RS_WARNING << "u32DecodeCnt > 519 " << " val:" << u32DecodeCnt << RS_REND;
                  break;
                }

                u16StartVal = (uint16_t)(u16StartVal + (int8_t)(*pu8RawData));
                pu16OutputBuf[u32DecodeCnt++] = u16StartVal;
                pu8RawData += 1;
                u32RemainLen -= 1;
            }
        }
        else
        {
            if ((u32DecodeCnt + u8Len) > u32OutputBufSize)
            {
                u32Ret = DECODE_ERROR_FLAG;
                break;
            }
            for (uint8_t u8Index = 0; u8Index < u8Len; u8Index++)
            {
                if (u32DecodeCnt > 519) {
                  RS_WARNING << "u32DecodeCnt > 519 " << " val:" << u32DecodeCnt << RS_REND;
                  break;
                }
                pu16OutputBuf[u32DecodeCnt++] = (uint16_t)pu8RawData[0] << 8 | (uint16_t)pu8RawData[1];
                pu8RawData += 2;
                u32RemainLen -= 2;
            }
        }
    }

    u32OutPutDecodeCnt = u32DecodeCnt;

    return u32Ret != DECODE_ERROR_FLAG;
  }

  bool PcC_u32OffsetRLEDecode(uint8_t* pu8RawDataBuf, uint32_t u32RawDataLen, 
                                    uint16_t* pu16OutputBuf, uint32_t u32OutputBufSize, uint32_t& u32DecodeCnt)
  {
    return u32OffsetRLEDecode(pu8RawDataBuf, u32RawDataLen, 
                                pu16OutputBuf, u32OutputBufSize, u32DecodeCnt);
  }

  ~RLE(){};
};



}  // namespace lidar
}  // namespace robosense
