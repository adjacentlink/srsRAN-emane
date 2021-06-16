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

#ifndef PHY_ADAPTER_COMMON_H
#define PHY_ADAPTER_COMMON_H


#include <cstdint>
#include <tuple>
#include <vector>
#include <set>
#include <map>

const size_t MAX_NUM_CARRIERS = 5;

// always rx/tx
using FrequencyPair = std::pair<uint64_t, uint64_t>;

// carrier index to freq pair
using CarrierIndexFrequencyTable = std::map<uint32_t, FrequencyPair>;

using FrequencyToCarrierIndex = std::map<std::uint64_t, uint32_t>;

// lookup carrier that matches the frequency, create if needed
template<typename R, typename T>
R *  getCarrier(T & msg, const uint64_t frequencyHz)
 {
   for(int idx = 0; idx < msg.carriers().size(); ++idx)
    {
      // check freq to the msg carrier tx center freq
      if(frequencyHz == msg.carriers(idx).frequency_hz())
       {
         return msg.mutable_carriers(idx);
       }
    }
  
   auto ptr = msg.add_carriers();

   ptr->set_frequency_hz(frequencyHz);

   return ptr;
 }

template<typename R, typename T>
R *  getCarrier(T & msg, const uint64_t frequencyHz, const uint32_t cellId)
 {
   for(int idx = 0; idx < msg.carriers().size(); ++idx)
    {
      // check freq and pci to the msg carrier tx center freq
      if(frequencyHz == msg.carriers(idx).frequency_hz() &&
         cellId == msg.carriers(idx).phy_cell_id())
       {
         return msg.mutable_carriers(idx);
       }
    }
  
   auto ptr = msg.add_carriers();

   ptr->set_frequency_hz(frequencyHz);

   ptr->set_phy_cell_id(cellId);

   return ptr;
 }

#endif
