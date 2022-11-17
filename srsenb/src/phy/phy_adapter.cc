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

#include "srsran/config.h"
#include "srsran/srslog/srslog.h"
#include "srsran/phy/adapter/phy_adapter_common.h"

extern "C" {
#include "srsran/phy/phch/ra.h"
#include "srsran/phy/phch/dci.h"
#include "srsran/phy/phch/phich.h"
#include "srsran/phy/phch/pucch.h"
}

#include "lib/include/srsran/phy/phch/pdsch_cfg.h"
#include "srsenb/hdr/phy/phy_adapter.h"

#include "libemanelte/enbotamessage.pb.h"
#include "libemanelte/ueotamessage.pb.h"

#include "libemanelte/mhalenb.h"
#include "libemanelte/enbstatisticmanager.h"
#include "libemanelte/sinrtester.h"

#include <mutex>

// private namespace for misc helpers and state for PHY_ADAPTER
namespace {
  EMANELTE::MHAL::ENB_DL_Message     dlMessage_;
  EMANELTE::MHAL::TxControlMessage   txControl_;

  EMANELTE::MHAL::SINRTester         sinrTester_{};

  // ul ue msg
  #define UL_Message_Message(x)    std::get<0>((x))
  #define UL_Message_RxControl(x)  std::get<1>((x))
  #define UL_Message_SINRTester(x) std::get<2>((x))

  using UL_Message = std::tuple<EMANELTE::MHAL::UE_UL_Message, 
                                EMANELTE::MHAL::RxControl,
                                EMANELTE::MHAL::SINRTester>;

  // 0 or more ue ul messages for this tti
  using UL_Messages = std::vector<UL_Message>;

  // search for carrier result
  using CarrierResults = std::vector<EMANELTE::MHAL::UE_UL_Message_CarrierMessage>;

  UL_Messages ulMessages_;

  // track carrier index to rx/tx freq
  CarrierIndexFrequencyTable carrierIndexFrequencyTable_;

  // track pci to carrier
  std::map<uint32_t,uint32_t> pciTable_;

  uint32_t pdcch_seqnum_ = 0; // seqnum per pdcch
  uint32_t pdsch_seqnum_ = 0; // seqnum per pdsch
  uint32_t pmch_seqnum_  = 0; // seqnum per pmch
  uint32_t phich_seqnum_ = 0; // seqnum per phich
  uint64_t tx_seqnum_    = 0; // seqnum per ota msg

  uint32_t tti_rx_       = 0; // curr or rx tti
  uint32_t tti_tx_       = 0; // next tx tti
  uint32_t pdcch_ref_    = 0;
  uint32_t pdsch_ref_    = 0;

  // referenceSignalPower as set by sib.conf sib2.rr_config_common_sib.pdsch_cnfg.rs_power
  float pdsch_rs_power_milliwatt_ = 0.0;

  // scaling between pdsch res in symbols with reference signals to symbols without reference signals
  float pdsch_rho_b_over_rho_a_ = 1.0;

  // scaling between reference signal res and pdsch res in symbols without reference signals, by tti and rnti
  using RHO_A_DB_MAP_t = std::map<uint16_t, float>; // map of rnti to rho_a

  RHO_A_DB_MAP_t rho_a_db_map_[10];                 // vector of rho_a maps by subframe number

  // cyclic prefix normal or extended for this cell
  srsran_cp_t cell_cp_ = SRSRAN_CP_NORM;

  std::mutex dl_mutex_;
  std::mutex ul_mutex_;


  srslog::basic_logger * logger_phy = nullptr;

  const uint8_t zeros_[0xffff] = {0};

  inline void
  initDownlinkChannelMessage(EMANELTE::MHAL::ChannelMessage * channelMessage,
                             EMANELTE::MHAL::CHANNEL_TYPE ctype,
                             EMANELTE::MHAL::MOD_TYPE modType,
                             uint16_t rnti,
                             uint32_t infoBits,
                             float txPowerScaledB = 0.0)
  {
    channelMessage->set_channel_type(ctype);
    channelMessage->set_modulation_type(modType);
    channelMessage->set_number_of_bits(infoBits);
    channelMessage->set_tx_power_scale_db(txPowerScaledB);

    if(rnti)
     {
       channelMessage->set_rnti(rnti);
     }
  }

  inline uint32_t tbs_to_bytes(const uint32_t tbs) { return tbs/8; }
}

#if 0 // enable to check log formats here
// compile time log format checks are no longer used, runtime is often too late
void Error(const char* fmt, ...)   __attribute__ ((format (printf, 1, 2)));
void Warning(const char* fmt, ...) __attribute__ ((format (printf, 1, 2)));
void Info(const char* fmt, ...)    __attribute__ ((format (printf, 1, 2)));
void Debug(const char* fmt, ...)   __attribute__ ((format (printf, 1, 2)));

void Error(const char* , ...)   { }
void Warning(const char* , ...) { }
void Info(const char* , ...)    { }
void Debug(const char* , ...)   { }

#else

#define Error(fmt, ...)                    \
  if (SRSRAN_DEBUG_ENABLED && logger_phy)  \
  logger_phy->error(fmt, ##__VA_ARGS__)

#define Warning(fmt, ...)                  \
  if (SRSRAN_DEBUG_ENABLED && logger_phy)  \
  logger_phy->warning(fmt, ##__VA_ARGS__)

#define Info(fmt, ...)                     \
  if (SRSRAN_DEBUG_ENABLED && logger_phy)  \
  logger_phy->info(fmt, ##__VA_ARGS__)

#define Debug(fmt, ...)                    \
  if (SRSRAN_DEBUG_ENABLED && logger_phy)  \
  logger_phy->debug(fmt, ##__VA_ARGS__)

#endif

namespace srsenb {
namespace phy_adapter {

static inline EMANELTE::MHAL::DCI_FORMAT convert(srsran_dci_format_t format)
 {
   switch(format)
    {
      case SRSRAN_DCI_FORMAT0:
        return(EMANELTE::MHAL::DCI_FORMAT_0);

      case SRSRAN_DCI_FORMAT1:
        return(EMANELTE::MHAL::DCI_FORMAT_1);

      case SRSRAN_DCI_FORMAT1A:
        return(EMANELTE::MHAL::DCI_FORMAT_1A);

      case SRSRAN_DCI_FORMAT1C:
        return(EMANELTE::MHAL::DCI_FORMAT_1C);

      case SRSRAN_DCI_FORMAT1B:
        return(EMANELTE::MHAL::DCI_FORMAT_1B);

      case SRSRAN_DCI_FORMAT1D:
        return(EMANELTE::MHAL::DCI_FORMAT_1D);

      case SRSRAN_DCI_FORMAT2:
        return(EMANELTE::MHAL::DCI_FORMAT_2);

      case SRSRAN_DCI_FORMAT2A:
        return(EMANELTE::MHAL::DCI_FORMAT_2A);

      case SRSRAN_DCI_FORMAT2B:
        return(EMANELTE::MHAL::DCI_FORMAT_2B);

      default:
       throw("MHAL:convert: invalid dci format");

      return (EMANELTE::MHAL::DCI_FORMAT_ERR);
   }
}

static inline EMANELTE::MHAL::MOD_TYPE convert(srsran_mod_t type)
{
   switch(type)
    {
       case SRSRAN_MOD_BPSK: 
         return (EMANELTE::MHAL::MOD_BPSK);

       case SRSRAN_MOD_QPSK:
         return (EMANELTE::MHAL::MOD_QPSK);

       case SRSRAN_MOD_16QAM:
         return (EMANELTE::MHAL::MOD_16QAM);

       case SRSRAN_MOD_64QAM:
         return (EMANELTE::MHAL::MOD_64QAM);

       default:
         throw("MHAL:convert: invalid mod type");

       return (EMANELTE::MHAL::MOD_ERR);
    }
}


// lookup tx freq that matches the frequencies associated with the cc_idx
static inline uint64_t getTxFrequency(uint32_t cc_idx)
{
   const auto iter = carrierIndexFrequencyTable_.find(cc_idx);

   if(iter != carrierIndexFrequencyTable_.end())
    {
      return iter->second.second; // tx
    }

  return 0;
}


// lookup rx freq that matches the frequencies associated the our cc_idx
static inline uint64_t getRxFrequency(uint32_t cc_idx)
{
   const auto iter = carrierIndexFrequencyTable_.find(cc_idx);

   if(iter != carrierIndexFrequencyTable_.end())
    {
      return iter->second.first; // rx
    }

  return 0;
}


// lookup carrier that matches the frequency associated with the cc_idx
static CarrierResults
getRxCarriers(const UL_Message & ulMessage, const uint32_t cc_idx, const uint32_t cell_id, const char * caller)
{
   CarrierResults carrierResults;

   const auto & ue_ul_msg = UL_Message_Message(ulMessage);

   // get the carrier frequency for this cc worker
   const auto rxFrequencyHz = getRxFrequency(cc_idx);

   if(rxFrequencyHz != 0)
    {
      for(const auto & carrier : ue_ul_msg.carriers())
       {
         // check match our rx freq to the msg carrier tx center freq and cell id
         if((rxFrequencyHz == carrier.frequency_hz()) && 
            (cell_id == carrier.phy_cell_id()))
          {
            carrierResults.emplace_back(carrier);
           
            break; // expect 1 match at most
          }
       }
    }

#if 0
   if(carrierResults.empty())
     Info("%s, %s, tti %u, cc_idx %u, cell_id %u, rxFrequency %lu, %s",
          __func__,
          caller,
          tti_rx_,
          cc_idx,
          cell_id,
          rxFrequencyHz,
          ue_ul_msg.DebugString());
#endif

  return carrierResults;
}


// see lib/src/phy/phch/pdcch.c
#define PDCCH_NOF_FORMATS               4
#define PDCCH_FORMAT_NOF_CCE(i)          (1<<i)
#define PDCCH_FORMAT_NOF_REGS(i)        ((1<<i)*9)
#define PDCCH_FORMAT_NOF_BITS(i)        ((1<<i)*72)

#define NOF_CCE(cfi)  ((cfi>0&&cfi<4)?q->pdcch.nof_cce [cfi-1]:0)
#define NOF_REGS(cfi) ((cfi>0&&cfi<4)?q->pdcch.nof_regs[cfi-1]:0)

// srsran_pdcch_encode(&q->pdcch, &q->dl_sf, &dci_msg, q->sf_symbols)
static int enb_dl_put_dl_pdcch_i(const srsran_enb_dl_t * q,
                                 const srsran_dci_msg_t * dci_msg,
                                 uint32_t ref,
                                 int type,  // 0 for DL, 1 for UL
                                 uint32_t cc_idx)
 {
   const auto rnti = dci_msg->rnti;

   // see lib/src/phy/phch/regs.c int srsran_regs_pdcch_put_offset(srsran_regs_t *h, 
   //                                                              uint32_t cfi, 
   //                                                              uint32_t start_reg,
   //                                                              uint32_t nof_regs)
   const uint32_t nof_regs = PDCCH_FORMAT_NOF_REGS(dci_msg->location.L);
   uint32_t start_reg      = dci_msg->location.ncce * 9;

   // see lib/src/phy/phch/pdcch.c srsran_pdcch_encode(srsran_pdcch_t*     q,
   //                                                  srsran_dl_sf_cfg_t* sf,
   //                                                  srsran_dci_msg_t*   msg,
   if(!((dci_msg->location.ncce + PDCCH_FORMAT_NOF_CCE(dci_msg->location.L) <= NOF_CCE(q->dl_sf.cfi)) &&
        (dci_msg->nof_bits < (SRSRAN_DCI_MAX_BITS - 16)))) 
    {
      Warning("PDCCH:%s cc=%u, type %s, rnti 0x%hx, cfi %d, illegal dci msg, ncce %d, format_ncce %d, cfi_ncce %d, nof_bits %d, max_bits %d", 
            __func__,
            cc_idx,
            type ? "UL" : "DL",
            rnti,
            q->dl_sf.cfi,
            dci_msg->location.ncce, 
            PDCCH_FORMAT_NOF_CCE(dci_msg->location.L),
            NOF_CCE(q->dl_sf.cfi),
            dci_msg->nof_bits,
            (SRSRAN_DCI_MAX_BITS - 16));

      // ALINK_XXX TODO
      // srsran p/r #299 amd issue #347 temp fix set start_reg to 0
      start_reg = 0;
    }

   const uint32_t regs_len = start_reg + nof_regs;

   if(regs_len > NOF_REGS(q->dl_sf.cfi))
    {
      Warning("PDCCH:%s cc=%u, type %s, rnti 0x%hx, cfi %d, pdccd->nof_regs %d, regs_len %u, ncce %d -> start_reg %d, L %d -> nof_regs %d", 
              __func__,
              cc_idx,
              type ? "UL" : "DL",
              rnti,
              q->dl_sf.cfi,
              NOF_REGS(q->dl_sf.cfi),
              regs_len,
              dci_msg->location.ncce, 
              start_reg,
              dci_msg->location.L,
              nof_regs);

      return SRSRAN_ERROR;
   }

  const auto txFrequencyHz = getTxFrequency(cc_idx);

  auto carrier = getTxCarrier<EMANELTE::MHAL::ENB_DL_Message_CarrierMessage,
                              EMANELTE::MHAL::ENB_DL_Message>(dlMessage_, txFrequencyHz, q->cell.id, cc_idx);

  auto control = getTxCarrier<EMANELTE::MHAL::TxControlCarrierMessage, 
                              EMANELTE::MHAL::TxControlMessage>(txControl_, txFrequencyHz, q->cell.id, cc_idx);

  auto pdcch_message = carrier->mutable_pdcch();

  // set seqnum for first entry
  if(! pdcch_message->has_seqnum())
   {
     pdcch_message->set_seqnum(pdcch_seqnum_++);
   }

  auto channelMessage = control->mutable_downlink()->add_pdcch();

  initDownlinkChannelMessage(channelMessage,
                             EMANELTE::MHAL::CHAN_PDCCH,
                             EMANELTE::MHAL::MOD_QPSK,
                             rnti,
                             dci_msg->nof_bits);

  for(uint32_t i = start_reg; i < regs_len; ++i)
   {
    const auto reg = q->pdcch.regs->pdcch[q->dl_sf.cfi-1].regs[i];

    if(reg)
     {
       const uint32_t  k0 = reg->k0;
       const uint32_t  l  = reg->l;
       const uint32_t* k  = &reg->k[0];

       const uint32_t rb = k0 / 12;

#if 0
       Debug("PDCCH DCI group sf_idx=%d, reg=%d, rnti=0x%hx placement: "
             "(l=%u, "
             "k0=%u, "
             "k[0]=%u "
             "k[1]=%u "
             "k[2]=%u "
             "k[3]=%u) in rb=%u", tti_tx_ % 10, i, rnti, l, k0, k[0], k[1], k[2], k[3], rb);
#endif

       channelMessage->add_resource_block_frequencies_slot1(EMANELTE::MHAL::ENB::get_tx_prb_frequency(rb, txFrequencyHz));
     }
   }

  // 1 pdcch message can contain 1 or more ul/dl dci components
  if(type == 0)
   {
     // dl dci
     auto dl_dci_message = pdcch_message->add_dl_dci(); 

     dl_dci_message->set_rnti(dci_msg->rnti);
     dl_dci_message->set_refid(pdcch_ref_++);

     // dci msg
     auto dl_dci_msg = dl_dci_message->mutable_dci_msg();

     dl_dci_msg->set_num_bits(dci_msg->nof_bits);
     dl_dci_msg->set_l_level(dci_msg->location.L);
     dl_dci_msg->set_l_ncce(dci_msg->location.ncce);
     dl_dci_msg->set_data(dci_msg->payload, dci_msg->nof_bits);
     dl_dci_msg->set_format(convert(dci_msg->format));
   }
  else
   {
     // ul dci
     auto ul_dci_message = pdcch_message->add_ul_dci();

     ul_dci_message->set_rnti(dci_msg->rnti);

     // dci msg
     auto ul_dci_msg = ul_dci_message->mutable_dci_msg();

     ul_dci_msg->set_num_bits(dci_msg->nof_bits);
     ul_dci_msg->set_l_level(dci_msg->location.L);
     ul_dci_msg->set_l_ncce(dci_msg->location.ncce);
     ul_dci_msg->set_data(dci_msg->payload, dci_msg->nof_bits);
     ul_dci_msg->set_format(convert(dci_msg->format));
   }

#if 1
   Info("PDCCH:%s: cc=%u, cellId %u, rnti 0x%hx, pdcch_seqnum %u, type %s",
        __func__, cc_idx, q->cell.id, rnti, pdcch_message->seqnum(), type ? "UL" : "DL");
#endif

  return SRSRAN_SUCCESS;
}

// lib/src/phy/phch/pdsch.c
// srsran_pdsch_encode(srsran_pdsch_t* q, 
//                     srsran_dl_sf_cfg_t* sf, 
//                     srsran_pdsch_cfg_t* cfg, 
//                     uint8_t*data[SRSRAN_MAX_CODEWORDS] ...);

// set pdsch dl
static int enb_dl_put_dl_pdsch_i(const srsran_enb_dl_t * q,
                                 srsran_pdsch_cfg_t* pdsch, 
                                 uint8_t* data,
                                 uint32_t ref,
                                 uint32_t tb,
                                 uint32_t cc_idx)
 {
   const auto grant = pdsch->grant;
   const auto rnti  = pdsch->rnti;

   const uint32_t sf_idx = (tti_tx_ % 10);

   float rho_a_db = 0.0;

   const auto riter = rho_a_db_map_[sf_idx].find(rnti);

   if(riter != rho_a_db_map_[sf_idx].end())
    {
      rho_a_db = riter->second;
    }

   const auto txFrequencyHz = getTxFrequency(cc_idx);

   auto carrier = getTxCarrier<EMANELTE::MHAL::ENB_DL_Message_CarrierMessage,
                               EMANELTE::MHAL::ENB_DL_Message>(dlMessage_, txFrequencyHz, q->cell.id, cc_idx);

   auto control = getTxCarrier<EMANELTE::MHAL::TxControlCarrierMessage, 
                               EMANELTE::MHAL::TxControlMessage>(txControl_, txFrequencyHz, q->cell.id, cc_idx);

   auto channelMessage = control->mutable_downlink()->add_pdsch();

   initDownlinkChannelMessage(channelMessage,
                              EMANELTE::MHAL::CHAN_PDSCH,
                              convert(grant.tb[tb].mod),
                              rnti,
                              grant.tb[tb].tbs,
                              rho_a_db);

   // Add resource block assignment from the phy_grant
   for(uint32_t rb = 0; rb < q->cell.nof_prb; ++rb)
    {
      if(grant.prb_idx[0][rb])
       {
         channelMessage->add_resource_block_frequencies_slot1(EMANELTE::MHAL::ENB::get_tx_prb_frequency(rb, txFrequencyHz));
       }

      if(grant.prb_idx[1][rb])
       {
         channelMessage->add_resource_block_frequencies_slot2(EMANELTE::MHAL::ENB::get_tx_prb_frequency(rb, txFrequencyHz));
       }
    }

   auto pdsch_message = carrier->mutable_pdsch();

  // set seqnum for first entry
   if(! pdsch_message->has_seqnum())
    {
      pdsch_message->set_seqnum(pdsch_seqnum_++);
    }

   // 1 pdsch message can contain 1 or more sub messages
   auto pdsch_subMsg = pdsch_message->add_submsg();

   const auto nbytes = tbs_to_bytes(grant.tb[tb].tbs);

   pdsch_subMsg->set_refid(pdsch_ref_++);
   pdsch_subMsg->set_tb(tb);
   pdsch_subMsg->set_tbs(grant.tb[tb].tbs);
   pdsch_subMsg->set_data(data, nbytes);
   
   ENBSTATS::putDLGrant(rnti);

#if 1
   Info("PDSCH:%s: cc=%u, cellId %u, rnti 0x%hx, nbytes %u, pdsch_seqnum %u",
        __func__, cc_idx, q->cell.id, rnti, nbytes, pdsch_message->seqnum());
#endif

   return SRSRAN_SUCCESS;
}


static int enb_dl_put_pmch_i(const srsran_enb_dl_t * q,
                            srsran_pmch_cfg_t* pmch_cfg,
                            uint8_t* data,
                            uint16_t rnti,
                            uint32_t cc_idx)
 {
   const auto grant = pmch_cfg->pdsch_cfg.grant;

   if(grant.nof_tb != 1)
    {
      Error("PMCH:%s cc=%u, rnti 0x%hx, nof_tb %u, expected 1", 
            __func__, cc_idx, rnti, grant.nof_tb);

      return SRSRAN_ERROR;
    }

   const uint32_t tb = 0;

   const auto txFrequencyHz = getTxFrequency(cc_idx);

   // pmch
   auto carrier = getTxCarrier<EMANELTE::MHAL::ENB_DL_Message_CarrierMessage,
                               EMANELTE::MHAL::ENB_DL_Message>(dlMessage_, txFrequencyHz, q->cell.id, cc_idx);

   auto control = getTxCarrier<EMANELTE::MHAL::TxControlCarrierMessage, 
                               EMANELTE::MHAL::TxControlMessage>(txControl_, txFrequencyHz, q->cell.id, cc_idx);

   auto pmch_message = carrier->mutable_pmch();

   // set seqnum for first entry
   if(! pmch_message->has_seqnum())
    {
      pmch_message->set_seqnum(pmch_seqnum_++);
    }

   // 1 pmch message can contain 1 or more sub messages
   auto pmch_subMsg = pmch_message->add_submsg();

   const auto nbytes = tbs_to_bytes(grant.tb[tb].tbs);

   pmch_subMsg->set_area_id(pmch_cfg->area_id);
   pmch_subMsg->set_tbs(grant.tb[tb].tbs);
   pmch_subMsg->set_rnti(rnti);
   pmch_subMsg->set_data(data ? data : zeros_, nbytes);

   auto channelMessage = control->mutable_downlink()->mutable_pmch();

   initDownlinkChannelMessage(channelMessage,
                              EMANELTE::MHAL::CHAN_PMCH,
                              convert(grant.tb[tb].mod),
                              rnti,
                              grant.tb[tb].tbs);

   // channelMessage.add_resource_blocks();
   for(uint32_t rb = 0; rb < q->cell.nof_prb; ++rb)
     {
       channelMessage->add_resource_block_frequencies_slot1(EMANELTE::MHAL::ENB::get_tx_prb_frequency(rb, txFrequencyHz));
       channelMessage->add_resource_block_frequencies_slot2(EMANELTE::MHAL::ENB::get_tx_prb_frequency(rb, txFrequencyHz));
     }

#if 1
   Info("PMCH:%s: cc=%u, cellId %u, rnti 0x%hx, nbytes %u, pmch_seqnum %u",
        __func__, cc_idx, q->cell.id, rnti, nbytes, pmch_message->seqnum());
#endif

   return SRSRAN_SUCCESS;
}


void enb_init_i(uint32_t idx,
                uint32_t sf_interval_msec, 
                uint32_t physical_cell_id, 
                srsran_cp_t cp,
                float ul_freq_hz, // rx
                float dl_freq_hz, // tx
                int n_prb, 
                EMANELTE::MHAL::mhal_config_t & mhal_config,
                rrc_cfg_t * rrc_cfg)
{
  pdsch_rs_power_milliwatt_ = powf(10.0f, static_cast<float>(rrc_cfg->sibs[1].sib2().rr_cfg_common.pdsch_cfg_common.ref_sig_pwr) / 10.0f);

  cell_cp_ = cp;

  Info("INIT:%s idx=%u, PCI=%u\n"
       "\tsf_interval=%u msec\n"
       "\trx_freq=%lu Hz\n"
       "\ttx_freq=%lu Hz\n"
       "\tn_prb=%d\n"
       "\trs_power=%d\n"
       "\tpdsch_rs_power_milliwatt=%0.2f\n"
       "\tp_b=%d\n"
       "\tpdsch_rho_b_over_rho_a=%.02f",
       __func__,
       idx,
       physical_cell_id,
       sf_interval_msec,
       round_freq(ul_freq_hz),
       round_freq(dl_freq_hz),
       n_prb,
       rrc_cfg->sibs[1].sib2().rr_cfg_common.pdsch_cfg_common.ref_sig_pwr,
       pdsch_rs_power_milliwatt_,
       rrc_cfg->sibs[1].sib2().rr_cfg_common.pdsch_cfg_common.p_b,
       pdsch_rho_b_over_rho_a_);

  EMANELTE::MHAL::ENB::initialize(idx,
     mhal_config,
     EMANELTE::MHAL::ENB::mhal_enb_config_t(physical_cell_id,
                                            sf_interval_msec,
                                            cp == SRSRAN_CP_NORM ? SRSRAN_CP_NORM_NSYMB : SRSRAN_CP_EXT_NSYMB,
                                            round_freq(ul_freq_hz), // rx
                                            round_freq(dl_freq_hz), // tx
                                            n_prb,
                                            pdsch_rs_power_milliwatt_,
                                            pdsch_rho_b_over_rho_a_));
}



// BEGIN phy_adapter enb api
void enb_initialize(uint32_t sf_interval_msec, 
                    phy_cell_cfg_list_t cfg_list,
                    EMANELTE::MHAL::mhal_config_t & mhal_config,
                    rrc_cfg_t * rrc_cfg)
{
   logger_phy = &srslog::fetch_basic_logger("PHY");

   carrierIndexFrequencyTable_.clear();

   uint32_t idx = 0;
   for(auto & cell_cfg : cfg_list)
    {
       enb_init_i(idx++,
                  sf_interval_msec, 
                  cell_cfg.cell.id, 
                  cell_cfg.cell.cp, 
                  round_freq(cell_cfg.ul_freq_hz), 
                  round_freq(cell_cfg.dl_freq_hz), 
                  cell_cfg.cell.nof_prb, 
                  mhal_config,
                  rrc_cfg);
    }
}


void enb_set_frequency(uint32_t cc_idx,
                       float rx_freq_hz,
                       float tx_freq_hz)
{
   carrierIndexFrequencyTable_[cc_idx] = FrequencyPair{round_freq(rx_freq_hz), round_freq(tx_freq_hz)}; // rx/tx

   Warning("%s cc=%u, rx_freq %lu Hz, tx_freq %lu Hz",
           __func__,
           cc_idx,
           round_freq(rx_freq_hz),
           round_freq(tx_freq_hz));
}



void enb_start()
{
  Info("INIT:%s", __func__);

  pthread_mutexattr_t mattr;

  if(pthread_mutexattr_init(&mattr) < 0)
   {
     Error("INIT:%s pthread_mutexattr_init error %s, exit", __func__, strerror(errno));

     exit(1);
   }
  else
   {
     if(pthread_mutexattr_setprotocol(&mattr, PTHREAD_PRIO_INHERIT) < 0)
       {
         Error("INIT:%s pthread_mutexattr_setprotocol error %s, exit", __func__, strerror(errno));
         exit(1);
       }

     pthread_mutexattr_destroy(&mattr);
  }

  dlMessage_.Clear();

  txControl_.Clear();

  EMANELTE::MHAL::ENB::start();
}


void enb_stop()
{
  Info("STOP:%s", __func__);

  EMANELTE::MHAL::ENB::stop();
}


void enb_dl_cc_tx_init(const srsran_enb_dl_t *q,
                       uint32_t tti_tx,
                       uint32_t cfi,
                       uint32_t cc_idx)
{
#if 0
  Info("%s, tti %u, cc=%u, cell_id %u", __func__, tti_tx, cc_idx, q->cell.id);
#endif

  // cc workers called in sequence 0 -> n
  if(cc_idx == 0)
   {
     // lock here, unlocked after tx_end to prevent any worker thread(s)
     // from attempting to start a new tx sequence before the current tx sequence
     // is finished
     dl_mutex_.lock();

     dlMessage_.Clear();

     txControl_.Clear();
   }

  // subframe index
  const uint32_t sf_idx = (tti_tx % 10);

  rho_a_db_map_[sf_idx].clear();

  dlMessage_.set_tti(tti_tx);

  const auto txFrequencyHz = getTxFrequency(cc_idx);

  // note - cfi should be nof_ctrl_symbols on regular frames and
  //        non_mbsfn_region_length (from sib13) on mbsfn frames
  auto carrier = getTxCarrier<EMANELTE::MHAL::ENB_DL_Message_CarrierMessage,
                              EMANELTE::MHAL::ENB_DL_Message>(dlMessage_, txFrequencyHz, q->cell.id, cc_idx);

  auto control = getTxCarrier<EMANELTE::MHAL::TxControlCarrierMessage, 
                              EMANELTE::MHAL::TxControlMessage>(txControl_, txFrequencyHz, q->cell.id, cc_idx);

  auto downlink = control->mutable_downlink();

  carrier->set_cfi(cfi);

  downlink->set_num_resource_blocks(q->cell.nof_prb);
  downlink->set_cfi(carrier->cfi());

  // cell_id cc_idx map
  pciTable_[q->cell.id] = cc_idx;

  // save the tti_tx
  tti_tx_ = tti_tx;

  // PCFICH encoding
  auto channelMessage = downlink->mutable_pcfich();

  initDownlinkChannelMessage(channelMessage,
                             EMANELTE::MHAL::CHAN_PCFICH,
                             EMANELTE::MHAL::MOD_QPSK,
                             0,
                             2);  // 2 bit to encode dfi

  for(int i=0; i<3; ++i)
    {
      const srsran_pcfich_t*   p1  = &q->pcfich;
      const srsran_regs_t*     p2  = p1->regs;
      const srsran_regs_ch_t*  rch = &p2->pcfich;
      const srsran_regs_reg_t* reg = rch->regs[i];

      uint32_t k0 = reg->k0;
      uint32_t l  = reg->l;
      const uint32_t * k = &reg->k[0];

      //srsran_regs_ch_t * pcfich = &((q->pcfich.regs)->pcfich);
      uint32_t rb = k0 / 12;

#if 0
      Debug("TX:%s PCFICH cc=%u group i=%d on this subframe placed at resource starting at "
            "(l=%u, "
            "k0=%u, "
            "k[0]=%u "
            "k[1]=%u "
            "k[2]=%u "
            "k[3]=%u) in resource block=%u", __func__, cc_idx, i, l, k0, k[0], k[1], k[2], k[3], rb);
#endif

      channelMessage->add_resource_block_frequencies_slot1(EMANELTE::MHAL::ENB::get_tx_prb_frequency(rb, txFrequencyHz));
    }

  // Set side chain PSS, SSS and MIB information on appropriate subframes
  if(sf_idx == 0 || sf_idx == 5) 
    {
      // physical cell group and id derived from pci

      // set cyclical prefix mode
      carrier->mutable_pss_sss()->set_cp_mode(q->cell.cp == SRSRAN_CP_NORM ? 
                                              EMANELTE::MHAL::CP_NORM : 
                                              EMANELTE::MHAL::CP_EXTD);

      // MIB on first subframe
      if(sf_idx == 0)
       {
         auto pbch = carrier->mutable_pbch();

         auto channelMessage = downlink->mutable_pbch();

         initDownlinkChannelMessage(channelMessage,
                                    EMANELTE::MHAL::CHAN_PBCH,
                                    EMANELTE::MHAL::MOD_QPSK,
                                    0,
                                    40);  // MIB + 16 bit CRC

         // MIB occupies the middle 72 resource elements of the second slot of subframe 0, which
         // is the middle 6 or 7 resource blocks depending on nof_prb being even or odd.
         // Approximate this by sending a segment for each fullly occupied resource block,
         // So 5 blocks when num_prb is odd.
         int first_prb = q->cell.nof_prb / 2 - 3 + (q->cell.nof_prb % 2);

         int num_prb = q->cell.nof_prb % 2 ? 5 : 6;

         for(int i=0; i<num_prb; ++i)
           {
             channelMessage->add_resource_block_frequencies_slot2(EMANELTE::MHAL::ENB::get_tx_prb_frequency(first_prb + i, txFrequencyHz));
           }

         switch(q->cell.phich_resources) 
          {
            case SRSRAN_PHICH_R_1_6:
               pbch->set_phich_resources(EMANELTE::MHAL::PR_ONE_SIXTH);
            break;

            case SRSRAN_PHICH_R_1_2:
               pbch->set_phich_resources(EMANELTE::MHAL::PR_ONE_HALF);
            break;

            case SRSRAN_PHICH_R_1:
               pbch->set_phich_resources(EMANELTE::MHAL::PR_ONE);
            break;

            case SRSRAN_PHICH_R_2:
               pbch->set_phich_resources(EMANELTE::MHAL::PR_TWO);
            break;

            default:
             throw("MHAL:enb_dl_put_base: unhandled cell phich_resources type");
          }

         switch(q->cell.phich_length) 
          {
            case SRSRAN_PHICH_NORM:
               pbch->set_phich_length(EMANELTE::MHAL::ENB_DL_Message_PBCH_PHICH_LENGTH_LEN_NORM);
            break;

            case SRSRAN_PHICH_EXT:
               pbch->set_phich_length(EMANELTE::MHAL::ENB_DL_Message_PBCH_PHICH_LENGTH_LEN_EXTD);
            break;

            default:
             throw("MHAL:enb_dl_put_base: unhandled cell phich_length type");
          }

         pbch->set_num_prb(q->cell.nof_prb);

         pbch->set_num_antennas(q->cell.nof_ports);
      }
   }
}


// send msg to mhal
void enb_dl_send_signal(time_t sot_sec, float frac_sec)
{
  EMANELTE::MHAL::Data data;

  if(dlMessage_.SerializeToString(&data))
   {
     txControl_.set_reference_signal_power_milliwatt(pdsch_rs_power_milliwatt_);

     // align sot to sf time
     const timeval tv_sf_time = {sot_sec, (time_t)(round(frac_sec * 1e3)*1e3)};
     
     auto ts = txControl_.mutable_sf_time();
     ts->set_ts_sec(tv_sf_time.tv_sec);
     ts->set_ts_usec(tv_sf_time.tv_usec);

     txControl_.set_message_type(EMANELTE::MHAL::DOWNLINK);
     txControl_.set_tx_seqnum(tx_seqnum_++);
     txControl_.set_tti_tx(tti_tx_);

#undef  DL_PHY_DEBUG
#ifdef  DL_PHY_DEBUG
     Info("MHAL:%s dlMessage %s\n", __func__, dlMessage_.DebugString().c_str());
#endif

     EMANELTE::MHAL::ENB::send_msg(data, txControl_);
   }
  else
   {
     Error("TX:%s SerializeToString ERROR len %zu", __func__, data.length());
   }

  dl_mutex_.unlock();
}


// XXX TODO this needs review
// set the power scaling on a per rnti basis
void enb_dl_set_power_allocation(uint32_t tti, uint16_t rnti, float rho_a_db, float rho_b_db)
{
  const uint32_t sf_idx = (tti % 10);

  rho_a_db_map_[sf_idx].emplace(rnti, rho_a_db);

  Debug("MHAL:%s sf_idx %d, rnti 0x%hx, rho_a_db %0.2f",
        __func__, sf_idx, rnti, rho_a_db);
}


// see lib/src/phy/enb/enb_dl.c 
// int srsran_enb_dl_put_pdcch_dl(srsran_enb_dl_t* q, srsran_dci_cfg_t* dci_cfg, srsran_dci_dl_t* dci_dl)
int enb_dl_put_pdcch_dl_i(srsran_enb_dl_t* q, 
                          srsran_dci_cfg_t* dci_cfg,
                          srsran_dci_dl_t* dci_dl, 
                          uint32_t ref,
                          uint32_t cc_idx)
{
  srsran_dci_msg_t dci_msg;
  bzero(&dci_msg, sizeof(dci_msg));

  if(srsran_dci_msg_pack_pdsch(&q->cell, &q->dl_sf, dci_cfg, dci_dl, &dci_msg) == SRSRAN_SUCCESS)
    {
      return enb_dl_put_dl_pdcch_i(q, &dci_msg, ref, 0, cc_idx); // DL
    }
  else
    {
      Error("PDCCH:%s error calling srsran_dci_msg_pack_pdsch(), ref %u", __func__, ref);

      return SRSRAN_ERROR;
    }
}


int enb_dl_cc_put_pdcch_dl(srsran_enb_dl_t* q, 
                           srsran_dci_cfg_t* dci_cfg,
                           mac_interface_phy_lte::dl_sched_grant_t* grant,
                           uint32_t ref,
                           uint32_t cc_idx)
{
  for(uint32_t tb = 0; tb < SRSRAN_MAX_TB; ++tb)
    {
      // check if data is ready
      if(grant->data[tb])
       {
         if(enb_dl_put_pdcch_dl_i(q, dci_cfg, &grant->dci, ref, cc_idx) != SRSRAN_SUCCESS)
          {
             Error("PDCCH:%s cc=%u, error ref %u, tb %u, rnti 0x%hx", 
                   __func__, cc_idx, ref, tb, grant->dci.rnti);
          }
       }
   }

   return SRSRAN_SUCCESS;
}


int enb_dl_cc_put_pdsch_dl(srsran_enb_dl_t* q, 
                        srsran_pdsch_cfg_t* pdsch, 
                        mac_interface_phy_lte::dl_sched_grant_t* grant,
                        uint32_t ref,
                        uint32_t cc_idx)
{
  for(uint32_t tb = 0; tb < SRSRAN_MAX_TB; ++tb)
    {
      // check if data is ready
      if(grant->data[tb])
       {
         if(enb_dl_put_dl_pdsch_i(q, pdsch, grant->data[tb], ref, tb, cc_idx) != SRSRAN_SUCCESS)
          {
             Error("PDSCH:%s cc=%u, error ref %u, tb %u, rnti 0x%hx", 
                   __func__, cc_idx, ref, tb, grant->dci.rnti);
          }
       }
    }

   return SRSRAN_SUCCESS;
}



// see lib/src/phy/enb/enb_dl.c
// int srsran_enb_dl_put_pmch(srsran_enb_dl_t* q, srsran_pmch_cfg_t* pmch_cfg, uint8_t* data)
int enb_dl_cc_put_pmch(srsran_enb_dl_t* q, 
                       srsran_pmch_cfg_t* pmch_cfg, 
                       mac_interface_phy_lte::dl_sched_grant_t* dl_sched_grant,
                       uint32_t cc_idx)
{
  uint16_t rnti = pmch_cfg->pdsch_cfg.rnti;

  if(rnti == 0)
   {
     Warning("PMCH:%s cc=%u, rnti 0x%hx, set to 0xfffd", __func__, cc_idx, rnti);

     rnti = 0xfffd;
   }

  return enb_dl_put_pmch_i(q, pmch_cfg, dl_sched_grant->data[0], rnti, cc_idx);
}

// see lib/src/phy/enb/enb_dl.c
// int srsran_enb_dl_put_pdcch_ul(srsran_enb_dl_t* q, srsran_dci_cfg_t* dci_cfg, srsran_dci_ul_t* dci_ul)
int enb_dl_cc_put_pdcch_ul(srsran_enb_dl_t* q, 
                           srsran_dci_cfg_t* dci_cfg,
                           srsran_dci_ul_t* dci_ul,
                           uint32_t ref,
                           uint32_t cc_idx)
{
  srsran_dci_msg_t dci_msg;
  bzero(&dci_msg, sizeof(dci_msg));

  if(srsran_dci_msg_pack_pusch(&q->cell, &q->dl_sf, dci_cfg, dci_ul, &dci_msg) == SRSRAN_SUCCESS)
    {
      return enb_dl_put_dl_pdcch_i(q, &dci_msg, ref, 1, cc_idx); // UL
    }
  else
    {
      Error("PDCCH:%s cc=%u, error calling srsran_dci_msg_pack_pdcch(), ref %u", __func__, cc_idx, ref);

      return SRSRAN_ERROR;
    }
}


// see lib/src/phy/enb/enb_dl.c
int enb_dl_cc_put_phich(srsran_enb_dl_t* q,
                        srsran_phich_grant_t* grant,
                        mac_interface_phy_lte::ul_sched_ack_t * ack,
                        uint32_t cc_idx)
{
  srsran_phich_resource_t resource;
  bzero(&resource, sizeof(resource));

  srsran_phich_calc(&q->phich, grant, &resource);

  const auto txFrequencyHz = getTxFrequency(cc_idx);

  auto carrier = getTxCarrier<EMANELTE::MHAL::ENB_DL_Message_CarrierMessage,
                              EMANELTE::MHAL::ENB_DL_Message>(dlMessage_, txFrequencyHz, q->cell.id, cc_idx);

  auto control = getTxCarrier<EMANELTE::MHAL::TxControlCarrierMessage, 
                              EMANELTE::MHAL::TxControlMessage>(txControl_, txFrequencyHz, q->cell.id, cc_idx);

  auto phich_message = carrier->mutable_phich();

  if(! phich_message->has_seqnum())
   {
     phich_message->set_seqnum(phich_seqnum_++);
   }

  auto phich_subMsg = phich_message->add_submsg();

  phich_subMsg->set_rnti(ack->rnti);
  phich_subMsg->set_ack(ack->ack);
  phich_subMsg->set_num_prb_low(grant->n_prb_lowest);
  phich_subMsg->set_num_dmrs(grant->n_dmrs);

  auto channelMessage = control->mutable_downlink()->add_phich();

  initDownlinkChannelMessage(channelMessage,
                             EMANELTE::MHAL::CHAN_PHICH,
                             EMANELTE::MHAL::MOD_BPSK,
                             ack->rnti,
                             3);  // phich is 000 for nak, 
                                  // 111 for ack. each bit is BPSK modulated 
                                  // to a symbol, and each symbol spread to 4 REs (12 REs total)

   const auto regs = q->phich.regs;

   if (SRSRAN_CP_ISEXT(regs->cell.cp)) {
     resource.ngroup /= 2;
   }

   const auto & rch = regs->phich[resource.ngroup];

   // nof_regs is 3 for phich groups (12 REs total per group).
   // l should always be 0 for Normal PHICH duration and [0,2] for Extended
   for (uint32_t i = 0; i < rch.nof_regs; i++) {
     uint32_t rb = rch.regs[i]->k0 / 12;

     channelMessage->add_resource_block_frequencies_slot1(EMANELTE::MHAL::ENB::get_tx_prb_frequency(rb, txFrequencyHz));
   }

#if 1
   Info("PHICH:%s cc=%u, cellId %u, rnti 0x%hx, ack %d, n_prb_L %d, n_dmrs %d, phich_seqnum %u", 
        __func__,
        cc_idx,
        q->cell.id,
        ack->rnti,
        ack->ack,
        grant->n_prb_lowest,
        grant->n_dmrs,
        phich_message->seqnum());
#endif

   return SRSRAN_SUCCESS;
}


bool enb_ul_get_signal(uint32_t tti, srsran_timestamp_t * ts)
{
  std::lock_guard<std::mutex> lock(ul_mutex_);

  tti_rx_ = tti;

  EMANELTE::MHAL::ENB::set_tti(tti);

  // clear any old testers
  for(auto & ulMessage : ulMessages_)
   {
     UL_Message_SINRTester(ulMessage).release();
   }

  ulMessages_.clear();

  EMANELTE::MHAL::RxMessages rxMessages;

  struct timeval tv_tti;

  EMANELTE::MHAL::ENB::get_messages(rxMessages, tv_tti);

  // set rx time 
  ts->full_secs = tv_tti.tv_sec;
  ts->frac_secs = tv_tti.tv_usec/1e6;

  // for each rx msg
  for(const auto & rxMessage : rxMessages)
   {
     EMANELTE::MHAL::UE_UL_Message ue_ul_msg;

     if(ue_ul_msg.ParseFromString(rxMessage.data_))
      {
        const auto & rxControl = rxMessage.rxControl_;
#undef  UL_PHY_DEBUG
#ifdef  UL_PHY_DEBUG
        Info("MHAL:%s ulMessage%s\n", __func__, ue_ul_msg.DebugString().c_str());
#endif

        for(auto & carrier : ue_ul_msg.carriers())
         {
           if(pciTable_.count(carrier.phy_cell_id()))
            {
              // found a match, save entry
              ulMessages_.emplace_back(ue_ul_msg, rxControl, rxMessage.sinrTesters_);

              // done with this pci from this carrier/ue
              break;
            }
         }
      }
    else
      {
        Error("RX:%s ParseFromString ERROR", __func__);
      }
   }

  return (! ulMessages_.empty());
}


int enb_ul_cc_get_prach(const srsran_cell_t * cell,
                        uint32_t * indices, 
                        float * offsets, 
                        float * p2avg,
                        uint32_t max_entries,
                        uint32_t & num_entries,
                        uint32_t cc_idx)
{
  std::lock_guard<std::mutex> lock(ul_mutex_);

  int result = SRSRAN_SUCCESS;

  std::set<uint32_t> unique;

  num_entries = 0;

  for(const auto & ulMessage : ulMessages_)
    {
      if(num_entries >= max_entries)
       {
         Warning("PRACH:%s num_entries %u > max_entries %u, quit", __func__, num_entries, max_entries);
         break;
       }

      const auto carrierResults = getRxCarriers(ulMessage, cc_idx, cell->id, __func__);

      if(! carrierResults.empty())
       {
         const auto & carrier = carrierResults[0];

         if(carrier.has_prach())
          {
            const auto txFrequencyHz = carrier.frequency_hz();
            const auto txCarrierId   = carrier.carrier_id();

            const auto sinrResult = 
              UL_Message_SINRTester(ulMessage).sinrCheck2(EMANELTE::MHAL::CHAN_PRACH,
                                                          txFrequencyHz,
                                                          cc_idx,
                                                          txCarrierId);

            if(sinrResult.bPassed_)
             {
               Info("PRACH:%s: pass, cc=%u, txCarrierId %u, snr %f, noise %f",
                    __func__, cc_idx, txCarrierId, sinrResult.sinr_dB_, sinrResult.noiseFloor_dBm_);
             }
            else
             {
               Warning("PRACH:%s: fail, cc=%u, txCarrierId %u, snr %f, noise %f", 
                       __func__, cc_idx, txCarrierId, sinrResult.sinr_dB_, sinrResult.noiseFloor_dBm_);

               continue;
             }

            const auto & prach    = carrier.prach();
            const auto & preamble = prach.preamble();

            // check for unique
            if(unique.count(preamble.index()) == 0)
             {
              unique.insert(preamble.index());

              indices[num_entries] = preamble.index();

              // timing offset estimation not currently implemented
              offsets[num_entries] = 0.0;
              p2avg[num_entries]   = 0.0;

              ++num_entries;

              Info("PRACH:%s cc=%u, txCarrierId %u, entry[%u], accept index %d",
                    __func__, cc_idx, txCarrierId, num_entries, preamble.index());
            }
          else
           {
             Info("PRACH:%s cc=%u, txCarrierId %u, entry[%u], ignore duplicate index %d",
                  __func__, cc_idx, txCarrierId, num_entries, preamble.index());
          }
        }
      }
    }

  return result;
}


// see lib/src/phy/enb/enb_ul.c
/* int srsran_enb_ul_get_pucch(srsran_enb_ul_t*    q,
                               srsran_ul_sf_cfg_t* ul_sf,
                               srsran_pucch_cfg_t* cfg,
                               srsran_pucch_res_t* res)
*/

int enb_ul_cc_get_pucch(srsran_enb_ul_t*    q,
                        srsran_ul_sf_cfg_t* ul_sf,
                        srsran_pucch_cfg_t* cfg,
                        srsran_pucch_res_t* res,
                        uint32_t cc_idx)
{
  std::lock_guard<std::mutex> lock(ul_mutex_);

  // see lib/src/phy/enb/enb_ul.c get_pucch()
  if (!srsran_pucch_cfg_isvalid(cfg, q->cell.nof_prb)) {
    Error("PUCCH %s, Invalid PUCCH configuration", __func__);
    return -1;
  }

  int                ret                               = SRSRAN_SUCCESS;
  uint32_t           n_pucch_i[SRSRAN_PUCCH_MAX_ALLOC] = {};
  srsran_pucch_res_t pucch_res                         = {};
  uint32_t           uci_cfg_total_ack                 = srsran_uci_cfg_total_ack(&cfg->uci_cfg);

  // Drop CQI if there is collision with ACK
  if (!cfg->simul_cqi_ack && uci_cfg_total_ack > 0 && cfg->uci_cfg.cqi.data_enable) {
    cfg->uci_cfg.cqi.data_enable = false;
  }

  // Select format
  cfg->format = srsran_pucch_proc_select_format(&q->cell, cfg, &cfg->uci_cfg, NULL);
  if (cfg->format == SRSRAN_PUCCH_FORMAT_ERROR) {
    ERROR("Returned error while selecting PUCCH format");
    return SRSRAN_ERROR;
  }

  // Get possible resources
  int nof_resources = srsran_pucch_proc_get_resources(&q->cell, cfg, &cfg->uci_cfg, NULL, n_pucch_i);
  if (nof_resources < 1 || nof_resources > SRSRAN_PUCCH_CS_MAX_ACK) {
    ERROR("No PUCCH resource could be calculated (%d)", nof_resources);
    return SRSRAN_ERROR;
  }

  // see lib/src/phy/enb/enb_ul.c get_pucch()
  // and lib/src/phy/ue/test/pucch_resource_test.c
  // this is needed to set cfg->format
  srsran_uci_value_t uci_data;
  ZERO_OBJECT(uci_data);

  const auto rnti = cfg->rnti;

  res->dmrs_correlation = 1.0;
  res->correlation      = 1.0;
  res->detected         = false;

  // for each ue uplink message
  for(const auto & ulMessage : ulMessages_)
   {
     if(res->detected)
      {
        Info("PUCCH:%s, cc=%u, res->detected set for rnti 0x%hx, break", __func__, cc_idx, rnti);
        break;
      } 

     const auto carrierResults = getRxCarriers(ulMessage, cc_idx, q->cell.id, __func__);

     if(! carrierResults.empty())
      {
        const auto & carrier = carrierResults[0];

        if(carrier.has_pucch())
         {
           const auto & pucch_message = carrier.pucch();

           // for each grant
           for(const auto & grant : pucch_message.grant())
            {
              if(grant.rnti() == rnti)
               {
                 const auto txFrequencyHz = carrier.frequency_hz();
                 const auto txCarrierId   = carrier.carrier_id();

                 const auto sinrResult = 
                   UL_Message_SINRTester(ulMessage).sinrCheck2(EMANELTE::MHAL::CHAN_PUCCH, 
                                                               rnti, 
                                                               txFrequencyHz,
                                                               cc_idx,
                                                               txCarrierId);

                 q->chest_res.snr_db              = sinrResult.sinr_dB_;
                 q->chest_res.noise_estimate_dbFs = sinrResult.noiseFloor_dBm_;

                 const auto & uci_message = grant.uci();
                 const auto uci_data = (srsran_uci_value_t *) uci_message.data();

#if 1
                 char logbuf[256] = {0};
                 srsran_uci_data_info(&cfg->uci_cfg, uci_data, logbuf, sizeof(logbuf));

                 Info("PUCCH:%s cc=%u, tcCarrierId %u, rnti 0x%hx, %d grants, pucch_seqnum %u, uci_info [%s]", 
                       __func__, cc_idx, txCarrierId, rnti, pucch_message.grant_size(), pucch_message.seqnum(), logbuf);
#endif

                 if(sinrResult.bPassed_)
                  {
                    Info("PUCCH:%s: pass, cc=%u, rnti 0x%hx, pucch_seqnum %u, format %d, snr %f, noise %f", 
                            __func__, cc_idx, rnti, pucch_message.seqnum(), cfg->format, sinrResult.sinr_dB_, sinrResult.noiseFloor_dBm_);

                    memcpy(&res->uci_data, uci_data, uci_message.length());

                    res->detected = true;

                    // from lib/src/phy/phch/pucch.c srsran_pucch_decode()
                    switch (cfg->format) {
                     case SRSRAN_PUCCH_FORMAT_1A:
                     case SRSRAN_PUCCH_FORMAT_1B:
                       res->uci_data.ack.valid = true;
                     break;

                     case SRSRAN_PUCCH_FORMAT_2:
                     case SRSRAN_PUCCH_FORMAT_2A:
                     case SRSRAN_PUCCH_FORMAT_2B:
                       res->uci_data.ack.valid    = true;
                       res->uci_data.cqi.data_crc = true;
                     break;

                     case SRSRAN_PUCCH_FORMAT_1:
                     case SRSRAN_PUCCH_FORMAT_3:
                     break;

                     case SRSRAN_PUCCH_FORMAT_ERROR:
                     break;
                    }

                    // pass
                    ENBSTATS::getPUCCH(rnti, true);
                  }
                 else
                  {
                    Warning("PUCCH:%s: fail, cc=%u, rnti 0x%hx, pucch_seqnum %u, sinr %f, noise %f", 
                            __func__, cc_idx, rnti, pucch_message.seqnum(), sinrResult.sinr_dB_, sinrResult.noiseFloor_dBm_);

                    res->detected = false;

                    // PUCCH failed snr, ignore
                    ENBSTATS::getPUCCH(rnti, false);
                  }

                  // done with this rnti
                  break;
                }
               else
                {
                  Info("PUCCH:%s: cc=%u, skip rnti 0x%hx != 0X%hx", 
                        __func__, cc_idx, rnti, grant.rnti());
                }
             }
          }
       }
     else
       {
         Info("PUCCH:%s cc=%u, rnti 0x%hx, cellId %u, no carriers found",
              __func__, cc_idx, rnti, q->cell.id);
       }
    }

  if(! res->detected)
   {
     Info("PUCCH:%s, cc=%u, nothing for rnti 0x%hx", __func__, cc_idx, rnti);
   } 

  return SRSRAN_SUCCESS;
}


int enb_ul_cc_get_pusch(srsran_enb_ul_t*    q,
                        srsran_ul_sf_cfg_t* ul_sf,
                        srsran_pusch_cfg_t* cfg,
                        srsran_pusch_res_t* res,
                        uint16_t rnti,
                        uint32_t cc_idx)
{
  std::lock_guard<std::mutex> lock(ul_mutex_);

  int result = SRSRAN_SUCCESS;

  res->crc            = false;
  res->uci.ack.valid  = false;

  // for each uplink message
  for(const auto & ulMessage : ulMessages_)
   {
     if(res->crc)
      {
        Info("PUSCH:%s, cc=%u, res->crc set for rnti 0x%hx, break", __func__, cc_idx, rnti);
        break;
      }

     const auto carrierResults = getRxCarriers(ulMessage, cc_idx, q->cell.id, __func__);
 
     if(! carrierResults.empty())
      {
        const auto & carrier = carrierResults[0];

        if(carrier.has_pusch())
         {
           const auto & pusch_message = carrier.pusch();

           const auto txCarrierId   = carrier.carrier_id();
           const auto txFrequencyHz = carrier.frequency_hz();

           // for each grant
           for(const auto & grant : pusch_message.grant())
            {
              if(grant.rnti() == rnti)
               {
                 const auto sinrResult = 
                   UL_Message_SINRTester(ulMessage).sinrCheck2(EMANELTE::MHAL::CHAN_PUSCH,
                                                               rnti,
                                                               txFrequencyHz,
                                                               cc_idx,
                                                               txCarrierId);

                 q->chest_res.snr_db              = sinrResult.sinr_dB_;
                 q->chest_res.noise_estimate_dbFs = sinrResult.noiseFloor_dBm_;

                 const auto & uci_message = grant.uci();
                 const auto uci_data = (srsran_uci_value_t *) uci_message.data();

#if 1
                 char logbuf[256] = {0};
                 srsran_uci_data_info(&cfg->uci_cfg, uci_data, logbuf, sizeof(logbuf));

                 Info("PUSCH:%s cc=%u, txCarrierId %u, rnti 0x%hx, cellId %u, %d grants, pusch_seqnum %u, uci_info [%s]",
                       __func__, cc_idx, txCarrierId, rnti, q->cell.id, pusch_message.grant_size(), pusch_message.seqnum(), logbuf);
#endif

                 if(sinrResult.bPassed_)
                  {
                    Info("PUSCH:%s: pass, cc=%u, rnti 0x%hx, pusch_seqnum %u, sinr %f, noise %f", 
                          __func__, cc_idx, rnti, pusch_message.seqnum(), sinrResult.sinr_dB_, sinrResult.noiseFloor_dBm_);

                    const auto & ul_grant_message = grant.ul_grant();
                    const auto & uci_message      = grant.uci();
                    const auto & payload          = grant.payload();

                    // srsran_pusch_grant_t  
                    memcpy(&cfg->grant, ul_grant_message.data(), ul_grant_message.length());

                    // srsran_uci_value_t
                    memcpy(&res->uci, uci_message.data(), uci_message.length());

                    // payload
                    memcpy(res->data, payload.data(), payload.length());

                    // see lib/src/phy/phch/pusch.c srsran_pusch_decode()
                    res->avg_iterations_block = 1;
                    res->crc                  = true;
                    res->uci.ack.valid        = true;

                    // pass
                    ENBSTATS::getPUSCH(rnti, true);
                  }
                else
                  {
                    Warning("PUSCH:%s: fail, cc=%u, rnti 0x%hx, pusch_seqnum %u, sinr %f, noise %f", 
                            __func__, cc_idx, rnti, pusch_message.seqnum(), sinrResult.sinr_dB_, sinrResult.noiseFloor_dBm_);

                    res->crc                  = false;
                    res->uci.ack.valid        = false;

                    // PUSCH failed snr, ignore
                    ENBSTATS::getPUSCH(rnti, false);
                  }

                // done with this rnti
                break;
              }
            }
         }
      }
     else
      {
        Info("PUSCH:%s cc=%u, rnti 0x%hx, cellId %u, no carriers found",
             __func__, cc_idx, rnti, q->cell.id);
      }
   }

  if(! res->crc)
   {
     Info("PUSCH:%s, cc=%u, nothing for rnti 0x%hx", __func__, cc_idx, rnti);
   }

  return result;
}


} // end namespace phy_adapter
} // end namespace srsenb
