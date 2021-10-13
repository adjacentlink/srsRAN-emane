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

/******************************************************************************
 * File:        metrics_ostatistic.h
 * Description: Metrics class for open statistic
 *****************************************************************************/

#ifndef SRSENB_METRICS_OSTATISTIC_H
#define SRSENB_METRICS_OSTATISTIC_H

#include "ue_metrics_interface.h"

namespace srsue {

class metrics_ostatistic : public srsran::metrics_listener<ue_metrics_t>
{
public:
  metrics_ostatistic();

  void set_metrics(const ue_metrics_t &m, const uint32_t period_usec);
  void set_ue_handle(ue_metrics_interface *ue_);
  void stop() {};

private:
  ue_metrics_interface* ue;
};

} // namespace srsue

#endif // SRSUE_METRICS_OSTATISTIC_H
