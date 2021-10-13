/**
 *
 * \section COPYRIGHT
 *
 * Copyright (c) 2019-2021 - Adjacent Link LLC, Bridgewater, New Jersey
 *
 * \section LICENSE
 *
 * This file is part of srsRAN-emane.
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Affero General Public License as
 * published by the Free Software Foundation, either version 3 of the
 * License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Affero General Public License for more details.
 *
 * See toplevel COPYRIGHT for more information.
 *
 */

#include "libemanelte/uestatisticmanager.h"
#include "srsue/hdr/metrics_ostatistic.h"

namespace srsue{

metrics_ostatistic::metrics_ostatistic():
  ue(NULL)
{ }

void metrics_ostatistic::set_ue_handle(ue_metrics_interface *ue_)
{
  ue = ue_;
}

void metrics_ostatistic::set_metrics(const ue_metrics_t &m, const uint32_t)
{
  const auto & stack = m.stack;
  const auto & mac   = stack.mac;

  UESTATS::MACMetrics metrics;

  for(int cc = 0; cc < SRSRAN_MAX_CARRIERS; ++cc)
   {
     metrics.emplace_back(
      UESTATS::MACMetric(mac[cc].tx_pkts,
                         mac[cc].tx_errors,
                         mac[cc].tx_brate,
                         mac[cc].rx_pkts,
                         mac[cc].rx_errors,
                         mac[cc].rx_brate,
                         mac[cc].ul_buffer,
                         mac[cc].dl_retx_avg,
                         mac[cc].ul_retx_avg));
   }

   UESTATS::setMACMetrics(metrics);
}

} // end namespace srsue
