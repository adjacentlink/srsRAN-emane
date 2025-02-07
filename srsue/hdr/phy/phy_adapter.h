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

#ifndef EMU_SRSUE_PHY_ADAPTER_H
#define EMU_SRSUE_PHY_ADAPTER_H

#include "srsran/config.h"
#include "srsran/srsran.h"
#include "srsran/phy/ue/ue_dl.h"
#include "srsran/interfaces/ue_interfaces.h"
#include "libemanelte/mhalconfig.h"
#include "srsran/phy/sync/refsignal_dl_sync.h"

#include <string>
#include <set>

namespace srsue {
class sync;
namespace phy_adapter {

void ue_initialize(const uint32_t sf_interval,
                   EMANELTE::MHAL::mhal_config_t & mhal_config);

void ue_start();

void ue_stop();

void ue_set_frequency(const uint32_t cc_idx,
                      const uint32_t cellid,
                      const bool scell,
                      const float rx_freq_hz,
                      const float tx_freq_hz);

void ue_set_cellid(const uint32_t cellid);

void ue_set_earfcn(const float rx_freq_hz,
                   const float tx_freq_hz,
                   const uint32_t earfcn);

void ue_set_bandwidth(const int n_prb);

void ue_set_prach_freq_offset(const uint32_t freq_offset, const uint32_t cell_id);

void ue_set_sync(sync * sync);

std::set<uint32_t> ue_get_detected_cells(const srsran_cell_t & cell);

void ue_get_refsignals(srsran_refsignal_dl_sync_t & refsignal_dl_sync, const uint32_t cell_id);

inline float ue_snr_to_rsrp(const float snr) { return snr - 100; };

inline float ue_snr_to_rsrq(const float snr) { return snr - 20; };

inline float ue_snr_to_rssi(const float snr, const float nf) { return snr - nf; };

// rx frame for this tti, common to all (4) states below
int ue_dl_read_frame(srsran_timestamp_t* rx_time);


// 1 cell cearch
int ue_dl_cellsearch_scan(srsran_ue_cellsearch_t * cs,
                          srsran_ue_cellsearch_result_t * fc,
                          const int force_nid_2,
                          uint32_t *max_peak);

// 2 mib search 
int ue_dl_mib_search(const srsran_ue_cellsearch_t * cs,
                     srsran_ue_mib_sync_t * ue_mib_sync,
                     srsran_cell_t * cell);

// 3 sfn search 
int ue_dl_system_frame_search(srsran_ue_sync_t * ue_sync, 
                              uint32_t * tti);

// 4 syncd search
int ue_dl_sync_search(srsran_ue_sync_t * ue_sync,
                      const uint32_t tti);

// get snr
float ue_dl_get_snr(const uint32_t cc_idx);

float ue_dl_get_nf (const uint32_t cc_idx);

// get dl dci
int ue_dl_cc_find_dl_dci(srsran_ue_dl_t*     q,
                         srsran_dl_sf_cfg_t* sf,
                         srsran_ue_dl_cfg_t* cfg,
                         const uint16_t            rnti,
                         srsran_dci_dl_t     dci_dl[SRSRAN_MAX_DCI_MSG],
                         const uint32_t            cc_idx);

// get ul dci
int ue_dl_cc_find_ul_dci(srsran_ue_dl_t*     q,
                         srsran_dl_sf_cfg_t* sf,
                         srsran_ue_dl_cfg_t* cfg,
                         const uint16_t            rnti,
                         srsran_dci_ul_t     dci_ul[SRSRAN_MAX_DCI_MSG],
                         const uint32_t cc_idx);

// decode pdsch
int ue_dl_cc_decode_pdsch(srsran_ue_dl_t*     q,
                          srsran_dl_sf_cfg_t* sf,
                          srsran_pdsch_cfg_t* pdsch_cfg,
                          srsran_pdsch_res_t  data[SRSRAN_MAX_CODEWORDS],
                          const uint32_t cc_idx);

// get phich
int ue_dl_cc_decode_phich(srsran_ue_dl_t*       q,
                          srsran_dl_sf_cfg_t*   sf,
                          srsran_ue_dl_cfg_t*   cfg,
                          srsran_phich_grant_t* grant,
                          srsran_phich_res_t*   result,
                          const uint16_t rnti,
                          const uint32_t cc_idx);


// get pmch
int ue_dl_cc_decode_pmch(srsran_ue_dl_t*     q,
                         srsran_dl_sf_cfg_t* sf,
                         srsran_pmch_cfg_t*  pmch_cfg,
                         srsran_pdsch_res_t  data[SRSRAN_MAX_CODEWORDS],
                         const uint32_t cc_idx);


// tx init
void ue_ul_tx_init();

// send to mhal with sot
void ue_ul_send_signal(const time_t sot_secs,
                       const float frac_sec,
                       const srsran_cell_t & cell);

// set prach
void ue_ul_put_prach(int index);

// set pucch, pusch
int ue_ul_encode(srsran_ue_ul_t* q,
                 srsran_ul_sf_cfg_t* sf,
                 srsran_ue_ul_cfg_t* cfg,
                 srsran_pusch_data_t* data,
                 const uint32_t cc_idx);

} // end namespace phy_adapter
} // end namespace srsue

#endif //EMU_SRSUE_PHY_ADAPTER_H
