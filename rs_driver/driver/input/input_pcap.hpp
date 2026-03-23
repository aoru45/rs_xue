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
#include <rs_driver/driver/input/input.hpp>

#include <sstream>
#include <iostream>
#include <chrono>

#ifdef _WIN32
#define WIN32
#else //__linux__
#endif

#include <pcap.h>

namespace robosense
{
namespace lidar
{
class InputPcap : public Input
{
public:
  InputPcap(const RSInputParam& input_param, double sec_to_delay)
    : Input(input_param), pcap_(NULL), pcap_offset_(ETH_HDR_LEN), pcap_tail_(0), difop_filter_valid_(false), 
    msec_to_delay_((uint64_t)(sec_to_delay / input_param.pcap_rate * 1000000))
  {
    if (input_param.use_vlan)
    {
      pcap_offset_ += VLAN_HDR_LEN;
    }

    pcap_offset_ += input_param.user_layer_bytes;
    pcap_tail_   += input_param.tail_layer_bytes;

    std::stringstream msop_stream, difop_stream;
    if (input_param_.use_vlan)
    {
      msop_stream << "vlan && ";
      difop_stream << "vlan && ";
    }

    msop_stream << "udp dst port " << input_param_.msop_port;
    difop_stream << "udp dst port " << input_param_.difop_port;

    msop_filter_str_ = msop_stream.str();
    difop_filter_str_ = difop_stream.str();
  }

  virtual bool init();
  virtual bool start();
  virtual ~InputPcap();

private:
  void recvPacket();

private:
  pcap_t* pcap_;
  size_t pcap_offset_;
  size_t pcap_tail_;
  std::string msop_filter_str_;
  std::string difop_filter_str_;
  bpf_program msop_filter_;
  bpf_program difop_filter_;
  bool difop_filter_valid_;
  uint64_t msec_to_delay_;
};

inline bool InputPcap::init()
{
  if (init_flag_)
    return true;

  char errbuf[PCAP_ERRBUF_SIZE];
  pcap_ = pcap_open_offline(input_param_.pcap_path.c_str(), errbuf);
  if (pcap_ == NULL)
  {
    cb_excep_(Error(ERRCODE_PCAPWRONGPATH));
    return false;
  }

  pcap_compile(pcap_, &msop_filter_, msop_filter_str_.c_str(), 1, 0xFFFFFFFF);

  if ((input_param_.difop_port != 0) && (input_param_.difop_port != input_param_.msop_port))
  {
    pcap_compile(pcap_, &difop_filter_, difop_filter_str_.c_str(), 1, 0xFFFFFFFF);
    difop_filter_valid_ = true;
  }

  init_flag_ = true;
  return true;
}

inline bool InputPcap::start()
{
  if (start_flag_)
    return true;

  if (!init_flag_)
  {
    cb_excep_(Error(ERRCODE_STARTBEFOREINIT));
    return false;
  }

  to_exit_recv_ = false;
  recv_thread_ = std::thread(std::bind(&InputPcap::recvPacket, this));

  start_flag_ = true;
  return true;
}

inline InputPcap::~InputPcap()
{
  stop();

  if (pcap_ != NULL)
  {
    pcap_close(pcap_);
    pcap_ = NULL;
  }
}

inline void InputPcap::recvPacket()
{
  // Statistics & lightweight profiling
  uint64_t forwarded_count = 0;
  uint64_t iter_count = 0;
  uint64_t t_pcap_next = 0;
  uint64_t t_filter = 0;
  uint64_t t_alloc_memcpy = 0;
  uint64_t t_push = 0;

  using clock = std::chrono::steady_clock;
  auto last_report_time = clock::now();

  // per-packet target duration (microseconds) — if zero -> no pacing
  const std::chrono::microseconds per_packet_us(static_cast<int64_t>(msec_to_delay_));

  // next_target is the steady_clock time when the next packet should be forwarded.
  // Initialize to now so the first packet is processed immediately.
  auto next_target = clock::now();

  while (!to_exit_recv_)
  {
    auto t0 = clock::now();
    struct pcap_pkthdr* header;
    const u_char* pkt_data;

    // 1) read next packet
    auto t_start_pcap = clock::now();
    int ret = pcap_next_ex(pcap_, &header, &pkt_data);
    auto t_end_pcap = clock::now();
    t_pcap_next += static_cast<uint64_t>(std::chrono::duration_cast<std::chrono::microseconds>(t_end_pcap - t_start_pcap).count());

    if (ret < 0)  // reach file end.
    {
      pcap_close(pcap_);
      pcap_ = NULL;

      if (input_param_.pcap_repeat)
      {
        cb_excep_(Error(ERRCODE_PCAPREPEAT));

        char errbuf[PCAP_ERRBUF_SIZE];
        pcap_ = pcap_open_offline(input_param_.pcap_path.c_str(), errbuf);
        // reset pacing baseline after reopen
        next_target = clock::now();
        continue;
      }
      else
      {
        cb_excep_(Error(ERRCODE_PCAPEXIT));
        break;
      }
    }

    bool forwarded = false;

    // 2) filter evaluation
    auto t_start_filter = clock::now();
    bool msop_match = (pcap_offline_filter(&msop_filter_, header, pkt_data) != 0);
    bool difop_match = false;
    if (!msop_match && difop_filter_valid_)
    {
      difop_match = (pcap_offline_filter(&difop_filter_, header, pkt_data) != 0);
    }
    auto t_end_filter = clock::now();
    t_filter += static_cast<uint64_t>(std::chrono::duration_cast<std::chrono::microseconds>(t_end_filter - t_start_filter).count());

    if (msop_match || difop_match)
    {
      // 3) allocate/copy
      auto t_start_alloc = clock::now();
      std::shared_ptr<Buffer> pkt = cb_get_pkt_(ETH_LEN);
      size_t copy_len = header->len - pcap_offset_ - pcap_tail_;
      if (copy_len > 0 && pkt && pkt->data())
      {
        memcpy(pkt->data(), pkt_data + pcap_offset_, copy_len);
        pkt->setData(0, copy_len);
      }
      auto t_end_alloc = clock::now();
      t_alloc_memcpy += static_cast<uint64_t>(std::chrono::duration_cast<std::chrono::microseconds>(t_end_alloc - t_start_alloc).count());

      // 4) push to downstream
      auto t_start_push = clock::now();
      if (pkt) pushPacket(pkt);
      auto t_end_push = clock::now();
      t_push += static_cast<uint64_t>(std::chrono::duration_cast<std::chrono::microseconds>(t_end_push - t_start_push).count());

      forwarded = true;
      ++forwarded_count;
    }

    ++iter_count;

    // 5) pacing: hybrid sleep + busy-wait for microsecond accuracy
    if (per_packet_us.count() > 0)
    {
      // move the next target forward by one packet interval
      next_target += per_packet_us;

      // if we're behind by more than a full interval, resync to now to avoid unbounded backlog
      auto now = clock::now();
      if (next_target < now)
      {
        next_target = now;
      }
      else
      {
        auto remaining = next_target - now;
        // coarse sleep for large remainder (let OS handle it), but leave ~1ms for spinning
        const auto coarse_threshold = std::chrono::milliseconds(2);
        const auto spin_reserve = std::chrono::milliseconds(1);
        if (remaining > coarse_threshold)
        {
          std::this_thread::sleep_for(remaining - spin_reserve);
        }
        // busy-wait for the last short interval for higher precision
        while (clock::now() < next_target)
        {
          // tight spin. If CPU usage becomes a concern, replace with small yields for less precision:
          // std::this_thread::yield();
        }
      }
    }

    // 6) periodic reporting
    auto now_report = clock::now();
    auto elapsed = std::chrono::duration_cast<std::chrono::seconds>(now_report - last_report_time);
    if (elapsed.count() >= 1)
    {
      double iters = (iter_count == 0) ? 1.0 : static_cast<double>(iter_count);
      std::cout << "[InputPcap][PROFILE] forwarded=" << forwarded_count
                << " iters=" << iter_count
                << " avg_pcap_next(us)=" << (t_pcap_next / iters)
                << " avg_filter(us)=" << (t_filter / iters)
                << " avg_alloc_memcpy(us)=" << (t_alloc_memcpy / iters)
                << " avg_push(us)=" << (t_push / iters)
                << " per_packet_us=" << per_packet_us.count()
                << std::endl;

      forwarded_count = 0;
      iter_count = 0;
      t_pcap_next = t_filter = t_alloc_memcpy = t_push = 0;
      last_report_time = now_report;
    }
  }
}

}  // namespace lidar
}  // namespace robosense
