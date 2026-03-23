/*********************************************************************************************************************
Copyright (c) 2020 RoboSense
All rights reserved

By downloading, copying, installing or using the software you agree to this license. If you do not agree to this
license, do not download, install, copy or use the software.

License Agreement
For RoboSense LiDAR SDK Library
(3-clause BSD License)

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the
following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following
disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following
disclaimer in the documentation and/or other materials provided with the distribution.

3. Neither the names of the RoboSense, nor Suteng Innovation Technology, nor the names of other contributors may be used
to endorse or promote products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES,
INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*********************************************************************************************************************/

#pragma once

#include <array>

#include <rs_driver/driver/decoder/decoder.hpp>
#include <rs_driver/driver/decoder/rle.hpp>

#define EM4_SURFACE_NUM 4
#define EM4_PIXELS_PER_COLUMN 520
#define EM4_VECSELS_PER_COLUMN 26
#define EM4_PIXELS_PER_VCSEL 20
#define EM4_COMPRESS_SPLIT_LEN 1368  // must less than EM4_COMPRESS_MAX_LEN
#define EM4_COMPRESS_MAX_LEN 2704
namespace robosense
{
namespace lidar
{
#pragma pack(push, 1)

typedef struct
{
  uint8_t id[8];
  uint8_t reserved_0[106];
  int8_t yaw_offset[26];
  int16_t pitch_angle[520];
  int16_t surface_pitch_offset[4];
  uint8_t reserved_1[110];
  uint16_t data_length;
  uint16_t counter;
  uint32_t data_id;
  uint32_t crc32;
} RSEM4DifopPkt;

typedef struct
{
  uint8_t id[4];
  uint8_t reserved0[63];
  uint8_t surface_id;
  uint8_t pixelCnt;  // 1 vcsel: 20 pixel
  uint8_t vcselCnt;  // 1 column: 26 vcsel
  int8_t yaw_offset[26];
  int16_t pitch_angle[520];
  int16_t surface_pitch_offset[4];
  int16_t roll_offset;
  uint8_t reserved1[4];
  uint16_t data_length;
  uint16_t counter;
  uint32_t data_id;
  uint32_t crc32;
} RSEM4Difop2Pkt;

typedef struct
{
  uint16_t distance;
  uint8_t intensity;
  uint8_t point_attribute;
} RSEM4Channel;  // 4-bytes

typedef struct
{
  RSEM4Channel channel[1];
} RSEM4Block;

typedef struct
{
  uint8_t id[4];
  uint16_t pkt_seq;
  uint16_t protocol_version;
  uint8_t return_mode;
  uint8_t time_mode;
  RSTimestampUTC timestamp;
  uint8_t fram_sync;
  uint8_t frame_rate;
  uint16_t column_num;
  int16_t yaw_angle;
  uint8_t pack_mode;
  uint8_t surface_id;
  uint16_t reserved;
  uint8_t lidar_type;
  uint8_t temperature;
} RSEM4MsopHeader;  // 32-bytes

typedef struct
{
  RSEM4MsopHeader header;
  RSEM4Block blocks[260];
  uint16_t data_length;
  uint16_t counter;
  uint32_t data_id;
  uint32_t crc32;
} RSEM4MsopPkt;  // 1084-bytes

#pragma pack(pop)

class PacketInfo
{
public:
  PacketInfo() : seq(0), type(0), length(0), yaw_angle(0), timestamp(0.0)
  {
    std::memset(data, 0, EM4_COMPRESS_SPLIT_LEN);  // Initialize pkt_data to 0
  }

  PacketInfo(uint16_t s, uint8_t t, const uint8_t* raw_data, uint16_t len, int16_t yaw, double ts)
    : seq(s), type(t), length(len), yaw_angle(yaw), timestamp(ts)
  {
    std::memcpy(data, raw_data, len);  // Copy the provided data into pkt_data
  }

  PacketInfo& operator=(const PacketInfo& other)
  {
    if (this != &other)
    {
      seq = other.seq;
      type = other.type;
      length = other.length;
      yaw_angle = other.yaw_angle;
      timestamp = other.timestamp;
      std::memcpy(data, other.data, EM4_COMPRESS_SPLIT_LEN);  // Copy data
    }
    return *this;
  }

  uint16_t seq;
  uint8_t type;
  uint8_t data[EM4_COMPRESS_SPLIT_LEN];
  uint16_t length;
  int16_t yaw_angle;
  double timestamp;
};

template <typename T_PointCloud>
class DecoderRSEM4 : public Decoder<T_PointCloud>
{
public:
  constexpr static double FRAME_DURATION = 0.1;
  constexpr static uint32_t SINGLE_PKT_NUM = 2400;

  virtual bool decodeMsopPkt(const uint8_t* pkt, size_t size) override;
  virtual ~DecoderRSEM4(){};

  virtual void decodeDifopPkt(const uint8_t* pkt, size_t size) override;

  explicit DecoderRSEM4(const RSDecoderParam& param);

private:
  RLE* rle_;  // RLE methods

  std::array<int16_t, EM4_VECSELS_PER_COLUMN> yaw_offset_;
  std::array<int16_t, EM4_PIXELS_PER_COLUMN> pitch_angle_;
  std::array<int16_t, EM4_SURFACE_NUM> surface_pitch_offset_;
  std::array<uint16_t, 2 * EM4_PIXELS_PER_COLUMN> dual_return_pitch_index_;
  PacketInfo pre_pkt_info_;

  static RSDecoderConstParam& getConstParam();
  bool decodeGeneralPkt(const uint8_t* pkt, size_t size);
  bool decodeCompPkt(const uint8_t* pkt, size_t size);

  SplitStrategyBySeq split_strategy_;

  void parseCompDataCombine(uint8_t* udp_payload_input, const uint16_t payload_len, uint16_t* radius_decode_array,
                            uint16_t* ref_attr_decode_array, const int16_t yaw_angle, const double pkt_ts);

  void parsePrePkt(uint16_t* radius_decode_array, uint16_t* ref_attr_decode_array);

  void parseCurPkt(uint8_t* udp_payload_input, const uint16_t payload_len, uint16_t* radius_decode_array,
                   uint16_t* ref_attr_decode_array, const int16_t yaw_angle, const double pkt_ts);

  void saveCurPkt(uint8_t* udp_payload_input, const uint16_t payload_len, const int16_t yaw_angle, const double pkt_ts);

  void rleDecodeMethod(uint8_t* udp_payload, const uint16_t payload_len, uint16_t* radius_decode_array,
                       uint16_t* ref_attr_decode_array, bool is_data_truncation, const int16_t yaw_angle,
                       const double pkt_ts);
};
template <typename T_PointCloud>
inline RSDecoderConstParam& DecoderRSEM4<T_PointCloud>::getConstParam()
{
  static RSDecoderConstParam param = {
    1084  // msop len
    ,
    1310  // difop len
    ,
    4  // msop id len
    ,
    3  // difop id len
    ,
    { 0x55, 0xAA, 0x5A, 0xA5 }  // msop id
    ,
    { 0xA5, 0xFF, 0x00, 0x5A, 0x11, 0x11, 0x55, 0x55 }  // difop id
    ,
    { 0x00, 0x00 },
    1  // laser number
    ,
    260  // blocks per packet
    ,
    1  // channels per block
    ,
    0.5f  // distance min
    ,
    350.0f  // distance max
    ,
    0.005f  // distance resolution
    ,
    80.0f  // initial value of temperature
  };

  return param;
}

template <typename T_PointCloud>
inline DecoderRSEM4<T_PointCloud>::DecoderRSEM4(const RSDecoderParam& param)
  : Decoder<T_PointCloud>(getConstParam(), param)
{
  this->packet_duration_ = FRAME_DURATION / SINGLE_PKT_NUM;
  this->bCheckMsopLen_ = false;
  this->bCheckDifopLen_ = false;

  this->yaw_offset_.fill(0);

  std::array<int16_t, EM4_PIXELS_PER_COLUMN> defaultAngle;
  constexpr int16_t START_ANGLE = -1300;
  constexpr int16_t ANGLE_STEP = 5;
  for (int i = 0; i < EM4_PIXELS_PER_COLUMN; ++i)
  {
    defaultAngle[i] = START_ANGLE + i * ANGLE_STEP;
  }
  this->pitch_angle_ = defaultAngle;

  this->surface_pitch_offset_.fill(0);

  for (int i = 0, j = 0; i < EM4_PIXELS_PER_COLUMN; i++)
  {
    this->dual_return_pitch_index_[j++] = i;
    this->dual_return_pitch_index_[j++] = i;
  }
}

template <typename T_PointCloud>
inline void DecoderRSEM4<T_PointCloud>::decodeDifopPkt(const uint8_t* packet, size_t size)
{
  if (packet == nullptr)
    return;

  const static uint16_t DIFOP1_LEN = sizeof(RSEM4DifopPkt);
  const static uint16_t DIFOP2_LEN = sizeof(RSEM4Difop2Pkt);

  auto processDifopPkt = [this](const auto& pkt) {
    for (int i = 0; i < EM4_VECSELS_PER_COLUMN; ++i)
    {
      this->yaw_offset_[i] = RS_INT8(pkt.yaw_offset[i]);
    }
    for (int i = 0; i < EM4_PIXELS_PER_COLUMN; ++i)
    {
      this->pitch_angle_[i] = RS_SWAP_INT16(pkt.pitch_angle[i]);
    }
    for (int i = 0; i < EM4_SURFACE_NUM; ++i)
    {
      this->surface_pitch_offset_[i] = RS_SWAP_INT16(pkt.surface_pitch_offset[i]);
    }

    this->angles_ready_ = true;
  };

  if (size == DIFOP1_LEN)
  {
    processDifopPkt(*reinterpret_cast<const RSEM4DifopPkt*>(packet));
  }
  else if (size == DIFOP2_LEN)
  {
    processDifopPkt(*reinterpret_cast<const RSEM4Difop2Pkt*>(packet));
  }
}

template <typename T_PointCloud>
inline bool DecoderRSEM4<T_PointCloud>::decodeMsopPkt(const uint8_t* packet, size_t size)
{
  const RSEM4MsopHeader& header = *(RSEM4MsopHeader*)packet;

  uint8_t pack_mode = header.pack_mode & 0x03;
  if (pack_mode == 0x01 || pack_mode == 0x0)  // without compression
  {
    return this->decodeGeneralPkt(packet, size);
  }
  else if (pack_mode == 0x03)  // with compression
  {
    return this->decodeCompPkt(packet, size);
  }
  return false;
}

template <typename T_PointCloud>
inline bool DecoderRSEM4<T_PointCloud>::decodeCompPkt(const uint8_t* packet, size_t size)
{
  if (size < sizeof(RSEM4MsopHeader))
  {
    return false;
  }
  bool ret = false;
  const RSEM4MsopHeader& header = *(RSEM4MsopHeader*)packet;

  double pkt_ts = 0;
  if (this->param_.use_lidar_clock)
  {
    pkt_ts = parseTimeUTCWithUs(&header.timestamp) * 1e-6;
  }
  else
  {
    uint64_t ts = getTimeHost();

    // roll back to first block to approach lidar ts as near as possible.
    pkt_ts = getTimeHost() * 1e-6 - this->getPacketDuration();

    if (this->write_pkt_ts_)
    {
      createTimeUTCWithUs(ts, (RSTimestampUTC*)&header.timestamp);
    }
  }
  uint16_t pkt_seq = ntohs(header.pkt_seq);
  uint8_t split_pack_num = header.pack_mode >> 4;
  this->temperature_ = static_cast<float>((int)header.temperature - this->const_param_.TEMPERATURE_RES);

  if (split_pack_num != 0x2 && split_strategy_.newPacket(pkt_seq))
  {
    this->cb_split_frame_(this->const_param_.LASER_NUM, this->cloudTs());
    this->first_point_ts_ = pkt_ts;
    ret = true;
  }

  int16_t yaw_angle = ntohs(header.yaw_angle);

  if ((this->pre_pkt_info_.type == 0) && (split_pack_num == 0x2))
  {
    return false;
  }

  if ((this->pre_pkt_info_.type == 2) && (split_pack_num == 0x2))
  {
    return false;
  }

  uint16_t radius_decode_array[EM4_PIXELS_PER_COLUMN] = { 0 };
  uint16_t ref_attr_decode_array[EM4_PIXELS_PER_COLUMN] = { 0 };

  uint8_t* udp_payload_input = (uint8_t*)packet;
  if (this->pre_pkt_info_.type == 1)
  {
    if (split_pack_num == 0x0)
    {
      parsePrePkt(radius_decode_array, ref_attr_decode_array);
      parseCurPkt(udp_payload_input, size, radius_decode_array, ref_attr_decode_array, yaw_angle, pkt_ts);
    }
    else if (split_pack_num == 0x1)
    {
      parsePrePkt(radius_decode_array, ref_attr_decode_array);
      saveCurPkt(udp_payload_input, size, yaw_angle, pkt_ts);
    }
    else if (split_pack_num == 0x2)
    {
      if (this->pre_pkt_info_.length != EM4_COMPRESS_SPLIT_LEN || this->pre_pkt_info_.seq != pkt_seq)
      {
        parsePrePkt(radius_decode_array, ref_attr_decode_array);
      }
      else
      {
        parseCompDataCombine(udp_payload_input, size, radius_decode_array, ref_attr_decode_array, yaw_angle, pkt_ts);
      }
    }
  }
  else if (split_pack_num == 0x1)
  {
    saveCurPkt(udp_payload_input, size, yaw_angle, pkt_ts);
  }
  else
  {
    parseCurPkt(udp_payload_input, size, radius_decode_array, ref_attr_decode_array, yaw_angle, pkt_ts);
  }
  this->pre_pkt_info_.type = split_pack_num;
  this->pre_pkt_info_.seq = pkt_seq;

  return ret;
}

template <typename T_PointCloud>
inline void DecoderRSEM4<T_PointCloud>::parsePrePkt(uint16_t* radius_decode_array, uint16_t* ref_attr_decode_array)
{
  std::vector<uint8_t> udp_payload(EM4_COMPRESS_SPLIT_LEN);

  std::copy(this->pre_pkt_info_.data, this->pre_pkt_info_.data + EM4_COMPRESS_SPLIT_LEN, udp_payload.begin());
  std::fill(this->pre_pkt_info_.data, this->pre_pkt_info_.data + EM4_COMPRESS_SPLIT_LEN, 0);
  rleDecodeMethod(udp_payload.data(), this->pre_pkt_info_.length, radius_decode_array, ref_attr_decode_array, true,
                  this->pre_pkt_info_.yaw_angle, this->pre_pkt_info_.timestamp);
}

template <typename T_PointCloud>
inline void DecoderRSEM4<T_PointCloud>::parseCurPkt(uint8_t* udp_payload_input, const uint16_t payload_len,
                                                    uint16_t* radius_decode_array, uint16_t* ref_attr_decode_array,
                                                    const int16_t yaw_angle, const double pkt_ts)
{
  uint16_t cpLen = payload_len;
  bool is_data_truncation = false;
  if (cpLen > EM4_COMPRESS_SPLIT_LEN)
  {
    cpLen = EM4_COMPRESS_SPLIT_LEN;
    is_data_truncation = true;
  }
  rleDecodeMethod(udp_payload_input, cpLen, radius_decode_array, ref_attr_decode_array, is_data_truncation, yaw_angle,
                  pkt_ts);
}

template <typename T_PointCloud>
inline void DecoderRSEM4<T_PointCloud>::saveCurPkt(uint8_t* udp_payload_input, const uint16_t payload_len,
                                                   const int16_t yaw_angle, const double pkt_ts)
{
  uint16_t cpLen = payload_len;
  if (cpLen > EM4_COMPRESS_SPLIT_LEN)
  {
    cpLen = EM4_COMPRESS_SPLIT_LEN;
  }
  std::copy(udp_payload_input, udp_payload_input + cpLen, this->pre_pkt_info_.data);
  this->pre_pkt_info_.length = cpLen;
  this->pre_pkt_info_.yaw_angle = yaw_angle;
  this->pre_pkt_info_.timestamp = pkt_ts;
}

template <typename T_PointCloud>
inline void DecoderRSEM4<T_PointCloud>::parseCompDataCombine(uint8_t* udp_payload_input, const uint16_t payload_len,
                                                             uint16_t* radius_decode_array,
                                                             uint16_t* ref_attr_decode_array, const int16_t yaw_angle,
                                                             const double pkt_ts)
{
  std::vector<uint8_t> udp_payload(EM4_COMPRESS_MAX_LEN, 0);

  std::copy(this->pre_pkt_info_.data, this->pre_pkt_info_.data + EM4_COMPRESS_SPLIT_LEN, udp_payload.begin());
  bool is_data_truncation = false;
  uint16_t cpLen = payload_len - sizeof(RSEM4MsopHeader);
  if (cpLen > EM4_COMPRESS_SPLIT_LEN - sizeof(RSEM4MsopHeader))
  {
    cpLen = EM4_COMPRESS_SPLIT_LEN - sizeof(RSEM4MsopHeader);
    is_data_truncation = true;
  }

  std::copy(udp_payload_input + sizeof(RSEM4MsopHeader), udp_payload_input + sizeof(RSEM4MsopHeader) + cpLen,
            udp_payload.begin() + EM4_COMPRESS_SPLIT_LEN);

  uint16_t udp_payload_len = EM4_COMPRESS_SPLIT_LEN + cpLen;
  rleDecodeMethod(udp_payload.data(), udp_payload_len, radius_decode_array, ref_attr_decode_array, is_data_truncation,
                  yaw_angle, pkt_ts);
  std::fill(this->pre_pkt_info_.data, this->pre_pkt_info_.data + EM4_COMPRESS_SPLIT_LEN, 0);
}

template <typename T_PointCloud>
inline void DecoderRSEM4<T_PointCloud>::rleDecodeMethod(uint8_t* udp_payload, const uint16_t payload_len,
                                                        uint16_t* radius_decode_array, uint16_t* ref_attr_decode_array,
                                                        bool is_data_truncation, const int16_t yaw_angle,
                                                        const double pkt_ts)
{
  constexpr uint8_t RADIUS_HEADER_ID[2] = { 0x55, 0xAA };
  constexpr uint8_t INTENSITY_HEADER_ID[2] = { 0xAA, 0x55 };
  constexpr size_t HEADER_SIZE = sizeof(RSEM4MsopHeader);

  constexpr size_t SECTION_HEADER_SIZE = 4;
  constexpr uint8_t BYTE_MASK = 0xFF;
  constexpr uint8_t FEATURE_SHIFT = 7;

  uint32_t parse_index = HEADER_SIZE;
  if ((parse_index + SECTION_HEADER_SIZE) >= payload_len)
  {
    return;
  }

  const auto& header = *reinterpret_cast<const RSEM4MsopHeader*>(udp_payload);
  // Convert surface_index from 1-based to 0-based index.
  uint8_t surface_index = header.surface_id - 1;
  if (surface_index >= EM4_SURFACE_NUM)
  {
    RS_WARNING << "Invalid surface index: " << (uint16_t)surface_index << ", set as 0" << RS_REND;
    surface_index = 0;
  }

  // distance
  uint32_t radius_decode_len = 0;
  uint16_t radius_raw_len = 0;
  if (memcmp(udp_payload + parse_index, RADIUS_HEADER_ID, 2) != 0)
  {
    return;
  }

  radius_raw_len = (uint16_t)((udp_payload[parse_index + 2] << 8) | udp_payload[parse_index + 3]);

  parse_index += SECTION_HEADER_SIZE;
  bool radius_rle_ret = this->rle_->PcC_u32OffsetRLEDecode(
      udp_payload + parse_index, radius_raw_len, radius_decode_array, EM4_PIXELS_PER_COLUMN, radius_decode_len);
  /* Prevent the compiler warning about the unused parameter. */
  (void)radius_rle_ret;

  // intensity
  uint32_t intensity_decode_len = 0;
  uint16_t intensity_raw_len = 0;

  // update parse index
  parse_index += radius_raw_len;
  if ((parse_index + SECTION_HEADER_SIZE) >= payload_len)
  {
    return;
  }

  if (memcmp(udp_payload + parse_index, INTENSITY_HEADER_ID, 2) != 0)
  {
    return;
  }

  intensity_raw_len = (uint16_t)((udp_payload[parse_index + 2] << 8) | udp_payload[parse_index + 3]);
  parse_index += SECTION_HEADER_SIZE;
  bool intensity_rle_ret = this->rle_->PcC_u32OffsetRLEDecode(
      udp_payload + parse_index, intensity_raw_len, ref_attr_decode_array, EM4_PIXELS_PER_COLUMN, intensity_decode_len);

  int actual_decode_len = EM4_PIXELS_PER_COLUMN;
  if ((intensity_rle_ret || is_data_truncation) && intensity_decode_len < EM4_PIXELS_PER_COLUMN &&
      intensity_decode_len > 0)
  {
    actual_decode_len = intensity_decode_len;
  }

  const int surface_pitch_offset = this->surface_pitch_offset_[surface_index];
  for (int i = 0; i < actual_decode_len; i++)
  {
    const int real_chan = i;
    const float distance = radius_decode_array[i] * this->const_param_.DISTANCE_RES;
    uint8_t point_attribute = (uint8_t)((ref_attr_decode_array[i] >> 8) & BYTE_MASK);
    const uint8_t feature = point_attribute >> FEATURE_SHIFT;
    if (this->distance_section_.in(distance))
    {
      const int vecsel = real_chan / EM4_PIXELS_PER_VCSEL;
      const int yaw = yaw_angle + this->yaw_offset_[vecsel];
      const int pitch = this->pitch_angle_[real_chan] + surface_pitch_offset;
      float x = distance * COS(pitch) * COS(yaw);
      float y = distance * COS(pitch) * SIN(yaw);
      float z = distance * SIN(pitch);
      this->transformPoint(x, y, z);
      uint8_t intensity = (uint8_t)(ref_attr_decode_array[i] & BYTE_MASK);

      typename T_PointCloud::PointT point;

      setX(point, x);
      setY(point, y);
      setZ(point, z);
      setTimestamp(point, pkt_ts);
      setRing(point, real_chan);
      setIntensity(point, intensity);
      setFeature(point, feature);
      this->point_cloud_->points.emplace_back(point);
    }
    else if (!this->param_.dense_points)
    {
      typename T_PointCloud::PointT point;
      setX(point, NAN);
      setY(point, NAN);
      setZ(point, NAN);
      setIntensity(point, 0);
      setTimestamp(point, pkt_ts);
      setRing(point, real_chan);
      setFeature(point, feature);
      this->point_cloud_->points.emplace_back(point);
    }
    this->prev_point_ts_ = pkt_ts;
  }
  this->prev_pkt_ts_ = pkt_ts;
}

template <typename T_PointCloud>
inline bool DecoderRSEM4<T_PointCloud>::decodeGeneralPkt(const uint8_t* packet, size_t size)
{
  if (size < sizeof(RSEM4MsopPkt))
  {
    RS_WARNING << "decodeGeneralPkt: packet size < sizeof(RSEM4MsopPkt)" << RS_REND;
    return false;
  }
  const RSEM4MsopPkt& pkt = *(RSEM4MsopPkt*)packet;
  bool ret = false;
  this->temperature_ = static_cast<float>((int)pkt.header.temperature - this->const_param_.TEMPERATURE_RES);

  double pkt_ts = 0;
  if (this->param_.use_lidar_clock)
  {
    pkt_ts = parseTimeUTCWithUs(&pkt.header.timestamp) * 1e-6;
  }
  else
  {
    uint64_t ts = getTimeHost();

    // roll back to first block to approach lidar ts as near as possible.
    pkt_ts = getTimeHost() * 1e-6 - this->getPacketDuration();

    if (this->write_pkt_ts_)
    {
      createTimeUTCWithUs(ts, (RSTimestampUTC*)&pkt.header.timestamp);
    }
  }

  // Convert pkt_seq from 1-based to 0-based index.
  uint16_t pkt_seq = ntohs(pkt.header.pkt_seq) - 1;
  if (split_strategy_.newPacket(pkt_seq))
  {
    this->cb_split_frame_(this->const_param_.LASER_NUM, this->cloudTs());
    this->first_point_ts_ = pkt_ts;
    ret = true;
  }

  constexpr uint16_t PIX_PER_COL_HALF = EM4_PIXELS_PER_COLUMN / 2;
  const uint16_t blocks_per_pkt = this->const_param_.BLOCKS_PER_PKT;
  const uint16_t channels_per_block = this->const_param_.CHANNELS_PER_BLOCK;
  const float distance_res = this->const_param_.DISTANCE_RES;
  const bool dense_points = this->param_.dense_points;
  // Convert surface_index from 1-based to 0-based index.
  uint8_t surface_index = pkt.header.surface_id - 1;
  if (surface_index >= EM4_SURFACE_NUM)
  {
    RS_WARNING << "Invalid surface index: " << (uint16_t)surface_index << ", set as 0" << RS_REND;
    surface_index = 0;
  }
  const int16_t yaw_base = RS_SWAP_INT16(pkt.header.yaw_angle);
  const int surface_pitch_offset = this->surface_pitch_offset_[surface_index];

  const uint16_t seq_mod = pkt_seq % 2;

  for (uint16_t blk = 0; blk < blocks_per_pkt; ++blk)
  {
    const auto& block = pkt.blocks[blk];
    const double point_time = pkt_ts;
    const uint16_t real_chan = static_cast<uint16_t>(blk + seq_mod * PIX_PER_COL_HALF);

    const int vecsel = real_chan / EM4_PIXELS_PER_VCSEL;
    const int yaw = yaw_base + this->yaw_offset_[vecsel];
    const int pitch = this->pitch_angle_[real_chan] + surface_pitch_offset;

    const float cos_pitch = COS(pitch);
    const float sin_pitch = SIN(pitch);
    const float cos_yaw = COS(yaw);
    const float sin_yaw = SIN(yaw);

    for (uint16_t chan = 0; chan < channels_per_block; ++chan)
    {
      const auto& channel = block.channel[chan];
      const float distance = ntohs(channel.distance) * distance_res;
      const uint8_t feature = (channel.point_attribute >> 5) & 0x01;

      typename T_PointCloud::PointT point;
      if (this->distance_section_.in(distance))
      {
        float x = distance * cos_pitch * cos_yaw;
        float y = distance * cos_pitch * sin_yaw;
        float z = distance * sin_pitch;
        this->transformPoint(x, y, z);

        setX(point, x);
        setY(point, y);
        setZ(point, z);
        setIntensity(point, channel.intensity);
        setTimestamp(point, point_time);
        setRing(point, real_chan);
        setFeature(point, feature);
        this->point_cloud_->points.emplace_back(std::move(point));
      }
      else if (!dense_points)
      {
        setX(point, NAN);
        setY(point, NAN);
        setZ(point, NAN);
        setIntensity(point, 0);
        setTimestamp(point, point_time);
        setRing(point, real_chan);
        setFeature(point, feature);
        this->point_cloud_->points.emplace_back(std::move(point));
      }
    }
  }

  this->prev_point_ts_ = pkt_ts;

  return ret;
}

}  // namespace lidar
}  // namespace robosense
