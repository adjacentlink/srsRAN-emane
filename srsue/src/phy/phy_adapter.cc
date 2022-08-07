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

#include <stdint.h> // radio.h needs this

#include "srsran/config.h"
#include "srsran/radio/radio.h"
#include "srsran/srslog/srslog.h"
#include "srsran/phy/adapter/phy_adapter_common.h"

#include "srsue/hdr/phy/phy_adapter.h"
#include "srsue/hdr/phy/sync.h"

#include "libemanelte/enbotamessage.pb.h"
#include "libemanelte/ueotamessage.pb.h"
#include "libemanelte/mhalue.h"
#include "libemanelte/uestatisticmanager.h"
#include "libemanelte/sinrtester.h"

#include <mutex>

// private namespace for misc helpers and state for PHY_ADAPTER
namespace {
 // uplink
 EMANELTE::MHAL::UE_UL_Message     ulMessage_;
 EMANELTE::MHAL::TxControlMessage  txControl_;

 // enb dl msg, rx control info and sinr tester impls
 using DL_Message = std::tuple<EMANELTE::MHAL::ENB_DL_Message, 
                               EMANELTE::MHAL::RxControl,
                               EMANELTE::MHAL::SINRTester>;
 // helpers
 #define DL_EnbMsg_Get(x)     std::get<0>((x))
 #define DL_RxControl_Get(x)  std::get<1>((x))
 #define DL_SINRTester_Get(x) std::get<2>((x))

 // dl message for this frame
 DL_Message dlMessage_{};


 // vector of dl signals from each enb
 using DL_Messages = std::vector<DL_Message>;

 // status, time stamp and mhal rx messages for a frame
 using FrameSignals = std::tuple<bool, struct timeval, EMANELTE::MHAL::RxMessages>;
 // helpers
 #define FrameMessage_IsSet_Get(x)      std::get<0>((x))
 #define FrameMessage_Timestamp_Get(x)  std::get<1>((x))
 #define FrameMessage_RxMessages_Get(x) std::get<2>((x))

 // all rx messages for this frame
 FrameSignals frameSignals_{false, {0,0}, {}};


 // search for carrier result
 using CarrierResults = std::vector<EMANELTE::MHAL::ENB_DL_Message_CarrierMessage>;

 // track carrierIndex to rx/tx carrier center frequency
 CarrierIndexFrequencyTable carrierIndexFrequencyTable_;

 // our configured carrier frequencies
 FrequencyToCarrierIndex rxCarrierTable_;

 // for use into srsran lib calls
 srsran::rf_buffer_t buffer_(1);

 // cc_idx, cellid
 std::map<uint32_t, uint32_t> cellIdInfo_;

 // cell_id, sinr, noise floor, timestamp
 using NbrCell = std::tuple<uint32_t, float, float, time_t>;
 // helpers
 #define NbrCell_Cellid_Get(x)     std::get<0>((x))
 #define NbrCell_Snr_Get(x)        std::get<1>((x))
 #define NbrCell_Nf_Get(x)         std::get<2>((x))
 #define NbrCell_Timestamp_Get(x)  std::get<3>((x))

 std::list<NbrCell> nbr_cells_;

 srslog::basic_logger * logger_phy = nullptr;

 struct SignalQuality {
  const float sinr_dB_;
  const float noiseFloor_dBm_;

  SignalQuality(const float sinr, const float noiseFloor) :
   sinr_dB_(sinr),
   noiseFloor_dBm_(noiseFloor)
  { }
 };


 class SINRManager {
  public:
    // 1000 frames
    SINRManager(const size_t maxEntries = 1000) :
     maxEntries_(maxEntries)
    { 
      sumSignal_ = 0;
      sumNoise_  = 0;
    }
 
    void clear()
     {
       std::lock_guard<std::mutex> lock(mutex_);

       entries_.clear();
       sumSignal_ = 0;
       sumNoise_  = 0;
     }

    void update(const float x, const float y)
     {
       std::lock_guard<std::mutex> lock(mutex_);

       sumSignal_ += x;
       sumNoise_  += y;
       entries_.emplace_front(x,y);

       if(entries_.size() > maxEntries_)
        {
          sumSignal_ -= entries_.back().first;
          sumNoise_ -= entries_.back().second;
          entries_.pop_back();
        }
     }

    float snr()
     {
       std::lock_guard<std::mutex> lock(mutex_);

       if(entries_.empty())
        {
          return -150;
        }
       else
        {
          return sumSignal_/entries_.size();
        }
     }

    float nf()
     {
       std::lock_guard<std::mutex> lock(mutex_);

       if(entries_.empty())
        {
          return -150;
        }
       else
        {
          return sumNoise_/entries_.size();
        }
     }

   private:
    std::deque<std::pair<float, float>> entries_;
    const size_t maxEntries_;
    float        sumSignal_;
    float        sumNoise_;
    std::mutex   mutex_;
 };
   

 SINRManager sinrManager_[MAX_NUM_CARRIERS];
  

 // pdsch rnti/messages with signal quality
 using ENB_DL_Message_PDSCH_Entry = std::pair<EMANELTE::MHAL::ENB_DL_Message_PDSCH_SubMsg, SignalQuality>;

 // rnti, entry
 using ENB_DL_PDSCH_MESSAGES = std::map<uint16_t, ENB_DL_Message_PDSCH_Entry>;

 ENB_DL_PDSCH_MESSAGES enb_dl_pdsch_messages_;

 uint64_t tx_seqnum_         = 0; // seqnum per ota msg
 uint32_t pucch_seqnum_      = 0; // seqnum per pucch
 uint32_t pusch_seqnum_      = 0; // seqnum per pusch

 uint16_t crnti_             = 0;
 uint32_t earfcn_            = 0;
 uint32_t tti_rx_            = 0; // curr or rx tti
 uint32_t tti_tx_            = 0; // next tx tti

 uint32_t prach_freq_offset_ = 0;
 srsue::sync * sync_         = nullptr;

 std::mutex ul_mutex_;

 static inline bool is_valid_n_id_2(const int n_id_2)
  {
    return(n_id_2 >= 0 && n_id_2 < 3);
  }

 srsran_dci_format_t get_msg_format(EMANELTE::MHAL::DCI_FORMAT format)
  {
   switch(format)
    {
     case EMANELTE::MHAL::DCI_FORMAT_0:
       return  SRSRAN_DCI_FORMAT0;

     case EMANELTE::MHAL::DCI_FORMAT_1:
       return  SRSRAN_DCI_FORMAT1;
 
     case EMANELTE::MHAL::DCI_FORMAT_1A:
       return  SRSRAN_DCI_FORMAT1A;
 
     case EMANELTE::MHAL::DCI_FORMAT_1B:
       return  SRSRAN_DCI_FORMAT1B;

     case EMANELTE::MHAL::DCI_FORMAT_1C:
       return  SRSRAN_DCI_FORMAT1C;

     case EMANELTE::MHAL::DCI_FORMAT_1D:
       return  SRSRAN_DCI_FORMAT1D;

     case EMANELTE::MHAL::DCI_FORMAT_2:
       return  SRSRAN_DCI_FORMAT2;

     case EMANELTE::MHAL::DCI_FORMAT_2A:
       return  SRSRAN_DCI_FORMAT2A;

     case EMANELTE::MHAL::DCI_FORMAT_2B:
       return  SRSRAN_DCI_FORMAT2B;

     default:
       return SRSRAN_DCI_NOF_FORMATS;
    }
  }


  void initUplinkChannelMessage(EMANELTE::MHAL::ChannelMessage * channelMessage,
                                EMANELTE::MHAL::CHANNEL_TYPE ctype,
                                EMANELTE::MHAL::MOD_TYPE modType,
                                const uint32_t infoBits,
                                const float txPowerScaledB=0.0)
  {
    channelMessage->set_channel_type(ctype);
    channelMessage->set_modulation_type(modType);
    channelMessage->set_number_of_bits(infoBits);
    channelMessage->set_tx_power_scale_db(txPowerScaledB);
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



namespace srsue {
namespace phy_adapter {

typedef std::vector<EMANELTE::MHAL::ENB_DL_Message_PDCCH_DL_DCI> DL_DCI_Results;

// message, sinr
typedef std::pair<EMANELTE::MHAL::ENB_DL_Message_PDCCH_UL_DCI, SignalQuality> UL_DCI_Result;

typedef std::vector<UL_DCI_Result> UL_DCI_Results;


// message, sinr
typedef std::pair<EMANELTE::MHAL::ENB_DL_Message_PDSCH_SubMsg, SignalQuality> PDSCH_Result;

typedef std::vector<PDSCH_Result> PDSCH_Results;


static inline EMANELTE::MHAL::MOD_TYPE convert(const srsran_mod_t type)
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
static inline uint64_t getTxFrequency(const uint32_t cc_idx)
{
   const auto iter = carrierIndexFrequencyTable_.find(cc_idx);

   if(iter != carrierIndexFrequencyTable_.end())
    {
      return iter->second.second; // tx
    }
 
  return 0;
 }


// lookup rx freq that matches the frequencies associated with the cc_idx
static inline uint64_t getRxFrequency(const uint32_t cc_idx)
{
   const auto iter = carrierIndexFrequencyTable_.find(cc_idx);

   if(iter != carrierIndexFrequencyTable_.end())
    {
      return iter->second.first; // rx
    }

   return 0; 
 }


// lookup carrier that matches the frequency associated with the cc_idx and cell_id
static CarrierResults
getRxCarriers(const DL_Message & dlMessage, const uint32_t cc_idx, const uint32_t cell_id, const char * caller)
 {
   CarrierResults carrierResults;

   const auto & enb_dl_msg = DL_EnbMsg_Get(dlMessage);
   const auto & rxControl  = DL_RxControl_Get(dlMessage);

   // get the carrier frequency for this cc worker
   const auto rxFrequencyHz = getRxFrequency(cc_idx);

   if(rxFrequencyHz != 0)
    {
      // for each carrier in the dl msg
      for(const auto & carrier : enb_dl_msg.carriers())
       {
         // check carrier is valid and match our rx freq to the msg tx center freq
         if(rxControl.is_valid_[carrier.carrier_id()] && 
            (rxFrequencyHz == carrier.frequency_hz()))
          {
            // check for cell id match
            if((cell_id == 0) || (cell_id == carrier.phy_cell_id()))
             {
               carrierResults.emplace_back(carrier);

               break; // expect 1 match at most
             }
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
          enb_dl_msg.DebugString());
#endif

   return carrierResults;
 }




// see sf_worker::update_measurements() -> cc_worker::update_measurements()
// lib/include/srsran/phy/ch_estimation/chest_dl.h
/*
typedef struct SRSRAN_API {
  cf_t*    ce[SRSRAN_MAX_PORTS][SRSRAN_MAX_PORTS];
  uint32_t nof_re;
  float    noise_estimate;
  float    noise_estimate_dbm;
  float    snr_db;
  float    snr_ant_port_db[SRSRAN_MAX_PORTS][SRSRAN_MAX_PORTS];
  float    rsrp;
  float    rsrp_dbm;
  float    rsrp_neigh;
  float    rsrp_port_dbm[SRSRAN_MAX_PORTS];
  float    rsrp_ant_port_dbm[SRSRAN_MAX_PORTS][SRSRAN_MAX_PORTS];
  float    rsrq;
  float    rsrq_db;
  float    rsrq_ant_port_db[SRSRAN_MAX_PORTS][SRSRAN_MAX_PORTS];
  float    rssi_dbm;
  float    cfo;
  float    sync_error;
} srsran_chest_dl_res_t;
*/

static void ue_dl_update_chest_i(srsran_chest_dl_res_t * chest_res, const float snr_db, const float noise_db)
{
    //  from faux_rf
    chest_res->cfo                = 0;
    chest_res->sync_error         = 0;
    chest_res->snr_db             = snr_db;
    chest_res->noise_estimate_dbm = noise_db;
    chest_res->noise_estimate     = noise_db;
}

// get all ota messages (all enb dl messages and rx control info)
static DL_Messages ue_dl_get_signals_i(srsran_timestamp_t * ts)
{
  if(! FrameMessage_IsSet_Get(frameSignals_))
   {
      Error("No Messages:%s:", __func__);

      return DL_Messages{};
   }

  if(ts)
   {
     ts->full_secs = FrameMessage_Timestamp_Get(frameSignals_).tv_sec;
     ts->frac_secs = FrameMessage_Timestamp_Get(frameSignals_).tv_usec / 1e6;
   }

  // all msgs from all enbs
  DL_Messages dlMessages;

  // for each rx message
  for(const auto & rxMessage : FrameMessage_RxMessages_Get(frameSignals_))
   {
     EMANELTE::MHAL::ENB_DL_Message enb_dl_msg;

     if(enb_dl_msg.ParseFromString(rxMessage.data_))
      {
        const auto & rxControl = rxMessage.rxControl_;

        // check each carrier for this enb
        for(const auto & carrier : enb_dl_msg.carriers())
         {
           const auto txFrequencyHz = carrier.frequency_hz();
           const auto txCarrierId   = carrier.carrier_id();

#if 0
           Info("%s, txFrequencyHz %lu, txCarrierId %u, isValid %d",
                __func__, txFrequencyHz), txCarrierId, rxControl.is_valid_[txCarrierId]);
#endif

           // check for valid carrier
           if(rxControl.is_valid_[txCarrierId])
            {
              const auto iter = rxCarrierTable_.find(txFrequencyHz);

              if(iter != rxCarrierTable_.end())
               {
                 sinrManager_[iter->second].update(rxControl.avg_snr_[txCarrierId],
                                                   rxControl.avg_nf_[txCarrierId]);

                 nbr_cells_.emplace_back(NbrCell(carrier.phy_cell_id(),
                                                 rxControl.avg_snr_[txCarrierId],
                                                 rxControl.avg_nf_[txCarrierId],
                                                 ts->full_secs));
               }
            }
         }

#undef  DL_PHY_DEBUG
#ifdef  DL_PHY_DEBUG
         Info("MHAL:%s dlMessage %s\n", __func__, enb_dl_msg.DebugString().c_str());
#endif

         // save msg compenents
         dlMessages.emplace_back(DL_Message(enb_dl_msg, rxControl, rxMessage.sinrTesters_));
      }
     else
      {
        Error("MHAL:%s ParseFromString ERROR", __func__);
      }
   }

  return (dlMessages);
}


// return message for a specific pci (enb)
static DL_Messages ue_dl_enb_subframe_get_pci_i(srsran_ue_sync_t * ue_sync, const uint32_t * tti)
{
   const auto dlMessages = ue_dl_get_signals_i(&ue_sync->last_timestamp);

   // for all dl messages
   for(auto & dlMessage : dlMessages)
    {
      const auto & enb_dl_msg = DL_EnbMsg_Get(dlMessage);

      for(const auto & carrier : enb_dl_msg.carriers())
       {
         const auto pci = carrier.phy_cell_id();

         if(pci == ue_sync->cell.id)
          {
            const uint32_t sf_idx = tti ? (*tti) % 10 : 0;

            ue_sync->sf_idx        = sf_idx;
            ue_sync->strack.sf_idx = sf_idx;
            ue_sync->sfind.sf_idx  = sf_idx;

            ++ue_sync->nof_recv_sf;

            if(sf_idx == 0)
             {
               ++ue_sync->frame_find_cnt;
               ++ue_sync->frame_ok_cnt;
               ++ue_sync->frame_total_cnt;
             }

            // 1 and done
            return DL_Messages{dlMessage};
          }
       }
    }

  return DL_Messages{};
}


static UL_DCI_Results get_ul_dci_list_i(const uint16_t rnti, const uint32_t cc_idx, const uint32_t cell_id)
{
  UL_DCI_Results ul_dci_results;

  const auto carrierResults = getRxCarriers(dlMessage_, cc_idx, cell_id, __func__);

  if(! carrierResults.empty())
   {
     // ue supports 1 antenna
     const uint32_t rxAntennaId = 0;

     const auto & carrier = carrierResults[rxAntennaId];

     const auto txFrequencyHz = carrier.frequency_hz();
     const auto txCarrierId   = carrier.carrier_id();

     if(carrier.has_pdcch())
      {
        const auto & pdcch_message = carrier.pdcch();
      
        for(const auto & ul_dci_message : pdcch_message.ul_dci())
         {
           if(ul_dci_message.rnti() == rnti)
            {
              const auto sinrResult = 
                DL_SINRTester_Get(dlMessage_).sinrCheck2(EMANELTE::MHAL::CHAN_PDCCH,
                                                         rnti, 
                                                         txFrequencyHz,
                                                         rxAntennaId,
                                                         txCarrierId);

              if(sinrResult.bPassed_)
               {
                 ul_dci_results.emplace_back(ul_dci_message, 
                                             SignalQuality{sinrResult.sinr_dB_, sinrResult.noiseFloor_dBm_});

                 Info("PUCCH:%s: pass, cc=%u, txCarrierId %u, txFrequency %lu, rnti 0x%hx, pdcch_seqnum %u, cnt %zu, sinr %f, noise %f",
                       __func__, cc_idx, txCarrierId, txFrequencyHz, rnti, pdcch_message.seqnum(), 
                       ul_dci_results.size(), sinrResult.sinr_dB_, sinrResult.noiseFloor_dBm_);
               }
              else
               {
                 Warning("PUCCH:%s: fail, cc=%u, txCarrierId %u, txFrequency %lu, rnti 0x%hx, pdcch_seqnum %u, sinr %f, noise %f",
                         __func__, cc_idx, txCarrierId, txFrequencyHz, rnti, pdcch_message.seqnum(),
                        sinrResult.sinr_dB_, sinrResult.noiseFloor_dBm_);
               }
            }
         }
      }
   }

  return ul_dci_results;
}


static DL_DCI_Results get_dl_dci_list_i(const uint16_t rnti, const uint32_t cc_idx, const uint32_t cell_id)
{
  DL_DCI_Results dl_dci_results;

  const auto carrierResults = getRxCarriers(dlMessage_, cc_idx, cell_id, __func__);

  if(!carrierResults.empty())
   {
     // ue supports 1 antenna
     const uint32_t rxAntennaId = 0;

     const auto & carrier = carrierResults[rxAntennaId];

     const auto txFrequencyHz = carrier.frequency_hz();
     const auto txCarrierId   = carrier.carrier_id();

     if(carrier.has_pdcch())
      {
        const auto & pdcch_message = carrier.pdcch();

        for(const auto & dl_dci_message : pdcch_message.dl_dci())
         {
           if(dl_dci_message.rnti() == rnti)
            {
              const auto sinrResult = 
                 DL_SINRTester_Get(dlMessage_).sinrCheck2(EMANELTE::MHAL::CHAN_PDCCH,
                                                          rnti,
                                                          txFrequencyHz,
                                                          rxAntennaId,
                                                          txCarrierId);

              if(sinrResult.bPassed_)
               {
                 INFO("PDSCH:%s: pass, cc=%u, txCarrierId %u, txFrequency %lu, rnti 0x%hx, pdcch_seqnum %u, sinr %f, noise %f",
                      __func__,
                      cc_idx,
                      txCarrierId,
                      txFrequencyHz,
                      rnti,
                      pdcch_message.seqnum(),
                      sinrResult.sinr_dB_,
                      sinrResult.noiseFloor_dBm_);

                 dl_dci_results.emplace_back(dl_dci_message);
               }
              else
               {
                 Warning("PDSCH:%s: fail, cc=%u, txCarrierId %u, txFrequency %lu, rnti 0x%hx, pdcch_seqnum %u, sinr %f, noise %f",
                      __func__,
                      cc_idx,
                      txCarrierId,
                      txFrequencyHz,
                      rnti,
                      pdcch_message.seqnum(),
                      sinrResult.sinr_dB_,
                      sinrResult.noiseFloor_dBm_);
               }

              // done with this rnti
              break;
            }
         }
      }
   }

  return dl_dci_results;
}



static PDSCH_Results ue_dl_get_pdsch_data_list_i(const uint32_t refid, 
                                                 const uint16_t rnti, 
                                                 const uint32_t cc_idx,
                                                 const uint32_t cell_id)
{
  PDSCH_Results pdsch_results;

  const auto carrierResults = getRxCarriers(dlMessage_, cc_idx, cell_id, __func__);

  if(! carrierResults.empty())
   {
     // ue supports 1 antenna
     const uint32_t rxAntennaId = 0;

     const auto & carrier = carrierResults[rxAntennaId];

     if(carrier.has_pdsch())
      {
        const auto & pdsch_message = carrier.pdsch();

        const auto txFrequencyHz = carrier.frequency_hz();
        const auto txCarrierId   = carrier.carrier_id();

        const auto sinrResult = 
          DL_SINRTester_Get(dlMessage_).sinrCheck2(EMANELTE::MHAL::CHAN_PDSCH,
                                                   rnti,
                                                   txFrequencyHz,
                                                   rxAntennaId,
                                                   txCarrierId);

        if(sinrResult.bPassed_)
         {
           Info("PDSCH:%s: pass, cc=%u, txCarrierId %u, txFrequency %lu, pdsch_seqnum %u, sinr %f, noise %f",
                 __func__, cc_idx, txCarrierId, txFrequencyHz, pdsch_message.seqnum(), sinrResult.sinr_dB_, sinrResult.noiseFloor_dBm_);

           // now search for the matching pdsch msg by refid
           for(const auto & pdsch_SubMsg : pdsch_message.submsg())
            {
             if(pdsch_SubMsg.refid() == refid)
               {
                 Info("PDSCH:%s: found refid %u", __func__, pdsch_SubMsg.refid());
 
                 pdsch_results.emplace_back(pdsch_SubMsg, SignalQuality(sinrResult.sinr_dB_, sinrResult.noiseFloor_dBm_));

                 // done with this ref id
                 break;
               }
            }
         }
        else
         {
           Warning("PDSCH:%s: fail, cc=%u, txCarrierId %u, txFrequency %lu, pdsch_seqnum %u, sinr %f, noise %f",
                   __func__, cc_idx, txCarrierId, txFrequencyHz, pdsch_message.seqnum(), sinrResult.sinr_dB_, sinrResult.noiseFloor_dBm_);
         }
      }
   }

  return pdsch_results;
}


static void ue_set_crnti_i(uint16_t crnti)
{
  if(crnti_ != crnti)
   {
     Info("MHAL:%s from 0x%hx to 0x%hx", __func__, crnti_, crnti);
     crnti_ = crnti;

     UESTATS::setCrnti(crnti);
   }
}


void ue_initialize(const uint32_t sf_interval_msec, EMANELTE::MHAL::mhal_config_t & mhal_config)
{
  logger_phy = &srslog::fetch_basic_logger("PHY");

  carrierIndexFrequencyTable_.clear();

  rxCarrierTable_.clear();

  Info("INIT:%s sf_interval %u msec", __func__, sf_interval_msec);

  EMANELTE::MHAL::UE::initialize(sf_interval_msec, mhal_config);
}


void ue_set_earfcn(const float rx_freq_hz, const float tx_freq_hz, const uint32_t earfcn)
{
  Info("INIT:%s rx_freq %lu Hz, tx_freq %lu Hz, earfcn %u -> %u",
       __func__,
       round_freq(rx_freq_hz),
       round_freq(tx_freq_hz),
       earfcn_,
       earfcn);

  earfcn_ = earfcn;
}


void ue_set_frequency(const uint32_t cc_idx,
                      const uint32_t cellid,
                      const bool scell,
                      const float rx_freq_hz,
                      const float tx_freq_hz)
{
   // set frequencies
   const auto rx_freq_hz_rounded = round_freq(rx_freq_hz);

   const auto tx_freq_hz_rounded = round_freq(tx_freq_hz);

   const bool searchMode = (scell == false) && (cc_idx == 0);

   if(searchMode)
    {
      carrierIndexFrequencyTable_.clear();

      rxCarrierTable_.clear();

      cellIdInfo_.clear();
    }
   else if(scell)
    {
      // secondary cell
      cellIdInfo_[cc_idx] = cellid;

      for(const auto e : cellIdInfo_)
       {
         Info("%s cc %u, cellid %u", __func__, e.first, e.second);
       }
    }

   carrierIndexFrequencyTable_[cc_idx] = FrequencyPair{rx_freq_hz_rounded, tx_freq_hz_rounded}; // rx/tx

   rxCarrierTable_[rx_freq_hz_rounded] = cc_idx;

   Warning("%s cellid %u, cc=%u, scell %s, rx_freq %lu Hz, tx_freq %lu Hz",
           __func__,
           cellid,
           cc_idx,
           scell ? "yes" : "no",
           rx_freq_hz_rounded,
           tx_freq_hz_rounded);

   EMANELTE::MHAL::UE::set_frequencies(cc_idx,
                                       cellid,
                                       scell,
                                       rx_freq_hz_rounded,
                                       tx_freq_hz_rounded);
}


void ue_set_sync(srsue::sync * sync)
{
  sync_ = sync;
}


void ue_set_cellid(const uint32_t cellid)
{
  cellIdInfo_.clear();

  cellIdInfo_[0] = cellid;

  Info("%s cellid %u", __func__, cellid);

}


void ue_set_bandwidth(const int n_prb)
{
  Info("INIT:%s n_prb %d", __func__, n_prb);

  EMANELTE::MHAL::UE::set_num_resource_blocks(n_prb);
}


void ue_start()
{
  Info("START:%s", __func__);

  pthread_mutexattr_t mattr;

  if(pthread_mutexattr_init(&mattr) < 0)
   {
     Error("START:%s pthread_mutexattr_init error %s, exit", __func__, strerror(errno));
     exit(1);
   }
  else
   {
     if(pthread_mutexattr_setprotocol(&mattr, PTHREAD_PRIO_INHERIT) < 0)
       {
         Error("START:%s pthread_mutexattr_setprotocol error %s, exit", __func__, strerror(errno));
         exit(1);
       }

     pthread_mutexattr_destroy(&mattr);
  }

  ulMessage_.Clear();

  txControl_.Clear();

  EMANELTE::MHAL::UE::start();
}


void ue_stop()
{
  Info("STOP:%s", __func__);

  EMANELTE::MHAL::UE::stop();
}


void ue_set_prach_freq_offset(const uint32_t freq_offset, const uint32_t cell_id)
{
  Info("MHAL:%s freq_offset %u, cell_id %u", __func__, freq_offset, cell_id);

  prach_freq_offset_ = freq_offset;
}


// read frame for this tti common to all states
int ue_dl_read_frame(srsran_timestamp_t* rx_time)
{
  auto & tv_tti = FrameMessage_Timestamp_Get(frameSignals_);

  EMANELTE::MHAL::UE::get_messages(FrameMessage_RxMessages_Get(frameSignals_), tv_tti);

  FrameMessage_IsSet_Get(frameSignals_) = true;

  // set rx time for caller
  if(rx_time)
   {
     rx_time->full_secs = tv_tti.tv_sec; 
     rx_time->frac_secs = tv_tti.tv_usec / 1e6;
   }

  return 1;
}


// 1 initial state cell search
int ue_dl_cellsearch_scan(srsran_ue_cellsearch_t * cs,
                          srsran_ue_cellsearch_result_t * res,
                          int force_nid_2,
                          uint32_t *max_peak)
{
  // cell search seems to be done in blocks of 5 sf's
  const uint32_t max_tries = cs->max_frames * 5; // 40 sf

  // search on the primary carrier
  const uint32_t cc_idx = 0;

  // n_id_2's
  std::set<uint32_t> n_id2s;

  UESTATS::Cells cells;

  uint32_t num_pss_sss_found = 0;
  uint32_t try_num           = 0;

  // notify in cell search
  EMANELTE::MHAL::UE::cell_search();

  while(++try_num <= max_tries)
   {
     // radio recv is done here during search
     sync_->radio_recv_fnc(buffer_, 0);

     // get all dl messages
     const auto dlMessages = ue_dl_get_signals_i(&cs->ue_sync.last_timestamp);

     // for each enb msg (if any)
     for(const auto & dlMessage : dlMessages)
      {
        const auto carrierResults = getRxCarriers(dlMessage, cc_idx, 0, __func__);

        for(const auto & carrier : carrierResults)
         {
           const auto & rxControl = DL_RxControl_Get(dlMessage);

           const auto carrierId = carrier.carrier_id(); 
           const auto pci       = carrier.phy_cell_id();

           const uint32_t n_id_1 = pci / 3;
           const uint32_t n_id_2 = pci % 3;

           // force is enabled, but this cell id group does not match
           if(is_valid_n_id_2(force_nid_2) && n_id_2 != (uint32_t)force_nid_2)
            {
              Info("RX:%s: n_id_1 %u, n_id_2 %u != %d, ignore", __func__, n_id_1, n_id_2, force_nid_2);
 
              continue;
            }

           // no sinr check for pss/sss, just search for pss/sss
           if(carrier.has_pss_sss())
            {
              const auto & pss_sss = carrier.pss_sss();

              const auto cp = pss_sss.cp_mode() == EMANELTE::MHAL::CP_NORM ? SRSRAN_CP_NORM : SRSRAN_CP_EXT;
              
              const int num_samples = rxControl.num_samples_[carrierId];

              const float peak_sum = rxControl.peak_sum_[carrierId];

              ++num_pss_sss_found;

              Info("RX:%s: detected PCI %u, carrierId %u, peak_sum %0.1f, num_samples %d",
                   __func__, pci, carrierId, peak_sum, num_samples);

              if(num_samples > 0)
               {
                 const float peak_avg = peak_sum / num_samples;

                 // save cell info
                 cells[pci] = peak_avg;

                 // cell id [0,1,2]
                 if(n_id2s.insert(n_id_2).second == true)
                  {
                    res[n_id_2].cell_id     = pci;
                    res[n_id_2].cp          = cp;
                    res[n_id_2].peak        = peak_avg;
                    res[n_id_2].mode        = 1.0;
                    res[n_id_2].psr         = 0.0;
                    res[n_id_2].cfo         = 0.0;
                    res[n_id_2].frame_type  = SRSRAN_FDD;

                    Info("RX:%s: new PCI %u, carrierId %u, n_id_1 %u, n_id_2 %u, peak_avg %f",
                         __func__, pci, carrierId, n_id_1, n_id_2, peak_avg);
                  }
                 else
                  {
                    // tie goes to the first entry (numeric lowest id)
                    if(peak_avg > res[n_id_2].peak)
                     {
                       Info("RX:%s: replace PCI %u, n_id_1 %u, n_id_2 %u, peak_avg %f",
                             __func__, pci, n_id_1, n_id_2, peak_avg);

                       res[n_id_2].cell_id = pci;
                       res[n_id_2].cp      = cp;
                       res[n_id_2].peak    = peak_avg;
                     } 
                    else
                     {
                       Info("RX:%s: ignore PCI %u, n_id_1 %u, n_id_2 %u, peak_avg %f <= current peak %f",
                             __func__, pci, n_id_1, n_id_2, peak_avg, res[n_id_2].peak);
                     }
                  }
               }
            }
         }
      } // end for each enb msg
   } // end while

  cs->ue_sync.pss_stable_cnt = num_pss_sss_found;
  cs->ue_sync.pss_is_stable  = num_pss_sss_found > 0 ? true : false;

  float max_avg = 0.0f;

  // now find the best
  for(const auto & id : n_id2s)
    {
      if(res[id].peak > max_avg)
        {
          *max_peak = id;

          max_avg = res[id].peak;
        }
    }

  Info("RX:%s: sf_idx %u, DONE, num_cells %zu, max_peak id %u, max_avg %f",
          __func__, cs->ue_sync.sf_idx, n_id2s.size(), *max_peak, max_avg);

  UESTATS::enterCellSearch(cells, earfcn_);

  return n_id2s.size();
}


// 2 mib search
int ue_dl_mib_search(const srsran_ue_cellsearch_t * cs,
                     srsran_ue_mib_sync_t * ue_mib_sync,
                     srsran_cell_t * cell)
{
  // 40 sf
  const uint32_t max_tries = cs->max_frames * 5;

  // search on the primary carrier
  const uint32_t cc_idx = 0;

  uint32_t try_num = 0;

  while(++try_num <= max_tries)
   {
     // radio recv called here
     sync_->radio_recv_fnc(buffer_, 0);

     // get specific pci dl msg
     const auto dlMessages = ue_dl_enb_subframe_get_pci_i(&ue_mib_sync->ue_sync, NULL);

     // expect 1 and only 1
     if(dlMessages.size() == 1)
      {
        const auto carrierResults = getRxCarriers(dlMessages[0], cc_idx, cell->id, __func__);

        if(! carrierResults.empty())
         {
           // ue supports 1 antenna
           const uint32_t rxAntennaId = 0;

           const auto & carrier = carrierResults[rxAntennaId];

           if(carrier.has_pbch())
            {
              const auto txFrequencyHz = carrier.frequency_hz();
              const auto txCarrierId   = carrier.carrier_id();

              const auto sinrResult = 
                 DL_SINRTester_Get(dlMessages[0]).sinrCheck2(EMANELTE::MHAL::CHAN_PBCH,
                                                             txFrequencyHz,
                                                             rxAntennaId,
                                                             txCarrierId);

              if(sinrResult.bPassed_)
               {
                 Info("PBCH:%s pass, cc=%u, txCarrierId %u, txFrequency %lu, sinr %f, noise %f",
                         __func__, cc_idx, txCarrierId, txFrequencyHz, sinrResult.sinr_dB_, sinrResult.noiseFloor_dBm_);
               }
              else
               {
                 Warning("PBCH:%s fail, cc=%u, txCarrierId %u, txFrequency %lu, sinr %f, noise %f",
                         __func__, cc_idx, txCarrierId, txFrequencyHz, sinrResult.sinr_dB_, sinrResult.noiseFloor_dBm_);

                 continue;
               }

              if(carrier.has_pss_sss())
               {
                 const auto & pss_sss = carrier.pss_sss();

                 const auto & pbch = carrier.pbch();

                 Info("RX:ue_dl_mib_search: found PBCH");

                 cell->nof_prb   = pbch.num_prb();
                 cell->nof_ports = pbch.num_antennas();

                 ue_mib_sync->ue_sync.state          = SF_TRACK;
                 ue_mib_sync->ue_sync.pss_stable_cnt = 1;
                 ue_mib_sync->ue_sync.pss_is_stable  = true;

                 switch(pbch.phich_resources())
                  {
                    case EMANELTE::MHAL::PR_ONE_SIXTH:
                     cell->phich_resources = SRSRAN_PHICH_R_1_6;
                    break;

                    case EMANELTE::MHAL::PR_ONE_HALF:
                     cell->phich_resources = SRSRAN_PHICH_R_1_2;
                    break;

                    case EMANELTE::MHAL::PR_ONE:
                     cell->phich_resources = SRSRAN_PHICH_R_1;
                    break;

                    case EMANELTE::MHAL::PR_TWO:
                     cell->phich_resources = SRSRAN_PHICH_R_2;
                    break;
                  }

                 switch(pbch.phich_length())
                  {
                    case EMANELTE::MHAL::ENB_DL_Message_PBCH_PHICH_LENGTH_LEN_NORM:
                     cell->phich_length = SRSRAN_PHICH_NORM;
                    break;

                    case EMANELTE::MHAL::ENB_DL_Message_PBCH_PHICH_LENGTH_LEN_EXTD:
                     cell->phich_length = SRSRAN_PHICH_EXT;
                    break;
                  }

                 UESTATS::enterMibSearch(true);

                 return 1;
               }
             }
          }
       }
     else
       {
          Warning("RX:%s: pci %hu, try %d/%u, expected 1, got %zu dl_messages", 
                  __func__, ue_mib_sync->ue_sync.cell.id, try_num, max_tries, dlMessages.size());
       }
    }

  UESTATS::enterMibSearch(false);

  return 0;
}


// 3 system frame search
int ue_dl_system_frame_search(srsran_ue_sync_t * ue_sync, uint32_t * sfn)
{
  const uint32_t max_tries = 1;

  // search on primary carrier
  const uint32_t cc_idx = 0;

  uint32_t try_num = 0;

  while(++try_num <= max_tries)
   {
     // radio recv called here
     sync_->radio_recv_fnc(buffer_, 0);

     // get specific pci dl msg
     const auto dlMessages = ue_dl_enb_subframe_get_pci_i(ue_sync, NULL);

     // expect 1 and only 1
     if(dlMessages.size() == 1)
      {
        const auto carrierResults = getRxCarriers(dlMessages[0], cc_idx, ue_sync->cell.id, __func__);

        if(! carrierResults.empty())
         {
           // ue supports 1 antenna
           const uint32_t rxAntennaId = 0;

           const auto & carrier = carrierResults[rxAntennaId];

           if(carrier.has_pbch())
            {
              const auto txFrequencyHz = carrier.frequency_hz();
              const auto txCarrierId   = carrier.carrier_id();

              // check for PSS SSS if PBCH is good
              const auto sinrResult = 
                DL_SINRTester_Get(dlMessages[0]).sinrCheck2(EMANELTE::MHAL::CHAN_PBCH,
                                                            txFrequencyHz,
                                                            rxAntennaId,
                                                            txCarrierId);

              if(sinrResult.bPassed_)
               {
                 Info("PBCH:%s pass, cc=%u, txCarrierId %u, txFrequency %lu, sinr %f, noise %f",
                      __func__, cc_idx, txCarrierId, txFrequencyHz, sinrResult.sinr_dB_, sinrResult.noiseFloor_dBm_);
               }
              else
               {
                 Warning("PBCH:%s fail, cc=%u, txCarrierId %u, txFrequency %lu, sinr %f, noise %f",
                      __func__, cc_idx, txCarrierId, txFrequencyHz, sinrResult.sinr_dB_, sinrResult.noiseFloor_dBm_);

                 continue;
               }

              if(carrier.has_pss_sss())
               {
                  const auto & pss_sss = carrier.pss_sss();
 
                  const auto & pbch = carrier.pbch();
 
                  Info("RX:%s: found PSS_SSS, try %u/%u", __func__, try_num, max_tries);
 
                  ue_sync->state           = SF_TRACK;
                  ue_sync->pss_stable_cnt  = 1;
                  ue_sync->pss_is_stable   = true;

                  // set system frame number
                  *sfn = DL_EnbMsg_Get(dlMessages[0]).tti();
                
                  UESTATS::enterSysFrameSearch(true);

                 return 1;
               }
            }
         }
      }
     else
      {
        Info("RX:%s: pci %hu, try %d/%u, expected 1, got %zu dl_messages", 
                __func__, ue_sync->cell.id, try_num, max_tries, dlMessages.size());
      }
   }

  UESTATS::enterSysFrameSearch(false);

  return 0;
}


// 4 this is the main rx handler
int ue_dl_sync_search(srsran_ue_sync_t * ue_sync, const uint32_t tti)
{
   // curr tti
   tti_rx_ = tti;

   // set next tx tti
   tti_tx_ = (tti+4)%10240;

   EMANELTE::MHAL::UE::set_tti(tti);

   // lower level radio recv called here
   sync_->radio_recv_fnc(buffer_, 0);

   DL_SINRTester_Get(dlMessage_).release();

   enb_dl_pdsch_messages_.clear();

   // get specific pci dl msg
   const auto dlMessages = ue_dl_enb_subframe_get_pci_i(ue_sync, &tti);

   // expect 1 and only 1 for single antenna mode
   if(dlMessages.size() == 1)
    {
      dlMessage_ = dlMessages[0];

      UESTATS::enterSyncSearch(true);
    }
   else
    {
      Warning("RX:%s: pci %hu, expected 1, got %zu dl_messages", 
              __func__, ue_sync->cell.id, dlMessages.size());
    }

   return dlMessages.size();
}


float ue_dl_get_snr(const uint32_t cc_idx)
{
   return sinrManager_[cc_idx].snr();
}


float ue_dl_get_nf(const uint32_t cc_idx)
{
   return sinrManager_[cc_idx].nf();
}


// see ue_dl_find_dl_dc
// int srsran_ue_dl_find_dl_dci(srsran_ue_dl_t*     q,
//                              srsran_dl_sf_cfg_t* sf,
//                              srsran_ue_dl_cfg_t* dl_cfg,
//                              uint16_t            rnti,
//                              srsran_dci_dl_t     dci_dl[SRSRAN_MAX_DCI_MSG])
int ue_dl_cc_find_dl_dci(srsran_ue_dl_t*     q,
                         srsran_dl_sf_cfg_t* sf,
                         srsran_ue_dl_cfg_t* dl_cfg,
                         const uint16_t      rnti,
                         srsran_dci_dl_t     dci_dl[SRSRAN_MAX_DCI_MSG],
                         const uint32_t      cc_idx)

{
  srsran_dci_msg_t dci_msg[SRSRAN_MAX_DCI_MSG] = {{}};

  int nof_msg = 0;

  // get the dl dci for this rnti
  const auto dl_dci_results = get_dl_dci_list_i(rnti, cc_idx, q->cell.id);

  // expecting 1 dci per rnti
  if(dl_dci_results.size() == 1)
    {
      const auto & dci_message = dl_dci_results[nof_msg];

      // get the pdsch pointed to be the dci
      const auto pdsch_results = ue_dl_get_pdsch_data_list_i(dci_message.refid(), rnti, cc_idx, q->cell.id);

      UESTATS::getPDCCH(rnti, true);

      // expecting 1 pdsch per dci
      if(pdsch_results.size() == 1)
        {
          const auto & pdsch_result = pdsch_results.front();

          UESTATS::getPDSCH(rnti, true);

          // save the grant for pdsch_decode
          enb_dl_pdsch_messages_.emplace(rnti, 
                                         ENB_DL_Message_PDSCH_Entry{pdsch_result.first, 
                                                                    pdsch_result.second});

          ue_dl_update_chest_i(&q->chest_res,
                               pdsch_result.second.sinr_dB_,
                               pdsch_result.second.noiseFloor_dBm_);

          const auto & dl_dci_message      = dci_message.dci_msg();
          const auto & dl_dci_message_data = dl_dci_message.data();

          auto & dci_entry = dci_msg[nof_msg];

          dci_entry.nof_bits      = dl_dci_message.num_bits();
          dci_entry.rnti          = rnti;
          dci_entry.format        = get_msg_format(dl_dci_message.format());
          dci_entry.location.L    = dl_dci_message.l_level();
          dci_entry.location.ncce = dl_dci_message.l_ncce();

          memcpy(dci_entry.payload, dl_dci_message_data.data(), dl_dci_message_data.size());

          // Unpack DCI messages see lib/src/phy/phch/dci.c
          if (srsran_dci_msg_unpack_pdsch(&q->cell, sf, &dl_cfg->cfg.dci, &dci_entry, &dci_dl[nof_msg++])) {
            Error("PDCCH:%s Unpacking DL DCI", __func__);
            return SRSRAN_ERROR;
          }

          Info("PDCCH:%s cc=%u, dl_dci refid %u, rnti 0x%hx, dci_len %zu, nof_msg %d", 
                __func__, cc_idx, dci_message.refid(), rnti, dl_dci_message_data.size(), nof_msg);
       }
      else
       {
         Error("PDCCH:%s expected 1, found %zu dl_dci for rnti 0x%hx", 
               __func__, pdsch_results.size(), rnti);
       }
    }
   else
    {
      if(dl_dci_results.size() > 1)
       {
         Warning("PDCCH:%s expected 1, found %zu dl_dci for rnti 0x%hx", 
                 __func__, dl_dci_results.size(), rnti);
       }
    }

  return nof_msg;
}


// see lib/src/phy/ue/ue_dl.c
int ue_dl_cc_find_ul_dci(srsran_ue_dl_t*     q,
                         srsran_dl_sf_cfg_t* sf,
                         srsran_ue_dl_cfg_t* dl_cfg,
                         const uint16_t      rnti,
                         srsran_dci_ul_t     dci_ul[SRSRAN_MAX_DCI_MSG],
                         const uint32_t cc_idx)
{
  srsran_dci_msg_t dci_msg[SRSRAN_MAX_DCI_MSG] = {{}};

  int nof_msg = 0;

  if(rnti) 
   {
     const auto ul_dci_results = get_ul_dci_list_i(rnti, cc_idx, q->cell.id);

     // expecting 1 ul dci per rnti
     if(ul_dci_results.size() == 1)
      {
        const auto & dci_message         = ul_dci_results[nof_msg].first;
        const auto & ul_dci_message      = dci_message.dci_msg();
        const auto & ul_dci_message_data = ul_dci_message.data();
 
        ue_dl_update_chest_i(&q->chest_res,
                             ul_dci_results[nof_msg].second.sinr_dB_,
                             ul_dci_results[nof_msg].second.noiseFloor_dBm_);

        auto & dci_entry = dci_msg[nof_msg];

        dci_entry.nof_bits      = ul_dci_message.num_bits();
        dci_entry.rnti          = rnti;
        dci_entry.format        = get_msg_format(ul_dci_message.format());
        dci_entry.location.L    = ul_dci_message.l_level();
        dci_entry.location.ncce = ul_dci_message.l_ncce();

        memcpy(dci_entry.payload, ul_dci_message_data.data(), ul_dci_message_data.size());

        // Unpack DCI messages
        if(srsran_dci_msg_unpack_pusch(&q->cell, sf, &dl_cfg->cfg.dci, &dci_entry, &dci_ul[nof_msg++])) {
          Error("PUCCH:%s Unpacking UL DCI", __func__);
          return SRSRAN_ERROR;
        }

        Info("PUCCH:%s found ul_dci rnti 0x%hx, nof_msg %d", __func__, rnti, nof_msg);
      }
     else
      {
        if(ul_dci_results.size() > 1)
         {
           Warning("PUCCH:%s expected 1, found %zu ul_dci for rnti 0x%hx", 
                   __func__, ul_dci_results.size(), rnti);
         }
      }
   }
  else
   {
     Warning("PUCCH:%s invalid rnti 0x%hx", __func__, rnti);
   }
 
  return nof_msg;
}


// see lib/src/phy/phch/pdsch.c srsran_pdsch_decode()
int ue_dl_cc_decode_pdsch(srsran_ue_dl_t*     q,
                          srsran_dl_sf_cfg_t* sf,
                          srsran_pdsch_cfg_t* cfg,
                          srsran_pdsch_res_t  data[SRSRAN_MAX_CODEWORDS],
                          const uint32_t cc_idx)
{
   int rc = SRSRAN_SUCCESS;

   const auto rnti = cfg->rnti;

   for(uint32_t tb = 0; tb < SRSRAN_MAX_CODEWORDS; ++tb)
    {
     if(cfg->grant.tb[tb].enabled)
       {
         const auto iter = enb_dl_pdsch_messages_.find(rnti);

         if(iter != enb_dl_pdsch_messages_.end())
           {
             const auto & pdsch_result  = iter->second;
             const auto & pdsch_subMsg  = pdsch_result.first;

             memcpy(data[tb].payload, pdsch_subMsg.data().data(), pdsch_subMsg.data().size());

             data[tb].avg_iterations_block = 1;
             data[tb].crc = true;

             ue_dl_update_chest_i(&q->chest_res,
                                  pdsch_result.second.sinr_dB_,
                                  pdsch_result.second.noiseFloor_dBm_);

             Info("PDSCH:%s: rnti 0x%hx, refid %d, tb[%d], payload %zu bytes, sinr %f",
                   __func__, rnti, pdsch_subMsg.refid(), tb, pdsch_subMsg.data().size(), q->chest_res.snr_db);
           }
         else
           {
             Error("PDSCH:%s: rnti 0x%hx, not found in %zu sub messages",
                   __func__, rnti, enb_dl_pdsch_messages_.size());

             rc = SRSRAN_ERROR;
           }
       }
      else
       {
         if(tb == 0)
           Info("PDSCH:%s: rnti 0x%hx, tb %u is not enabled", __func__, rnti, tb);
       }
    }

  return rc;
}


// see lib/src/phy/ue/ue_dl.c
int ue_dl_cc_decode_phich(srsran_ue_dl_t*       q,
                          srsran_dl_sf_cfg_t*   sf,
                          srsran_ue_dl_cfg_t*   cfg,
                          srsran_phich_grant_t* grant,
                          srsran_phich_res_t*   result,
                          const uint16_t rnti,
                          const uint32_t cc_idx)
{
  int rc = SRSRAN_SUCCESS;

  srsran_phich_resource_t n_phich;

  srsran_phich_calc(&q->phich, grant, &n_phich);

  const auto carrierResults = getRxCarriers(dlMessage_, cc_idx, q->cell.id, __func__);

  if(! carrierResults.empty())
   {
     // ue supports 1 antenna
     const uint32_t rxAntennaId = 0;

     const auto & carrier = carrierResults[rxAntennaId];

     if(carrier.has_phich())
      {
        const auto & phich_message = carrier.phich();

        const auto txFrequencyHz = carrier.frequency_hz();
        const auto txCarrierId   = carrier.carrier_id();

        const auto sinrResult = 
          DL_SINRTester_Get(dlMessage_).sinrCheck2(EMANELTE::MHAL::CHAN_PHICH,
                                                   rnti,
                                                   txFrequencyHz,
                                                   rxAntennaId,
                                                   txCarrierId);

        if(sinrResult.bPassed_)
         {
           Info("PHICH:%s pass, cc=%u, txCarrierId %u, txFrequency %lu, phich_seqnum %u, sinr %f, noise %f",
                 __func__, cc_idx, txCarrierId, txFrequencyHz, phich_message.seqnum(), sinrResult.sinr_dB_, sinrResult.noiseFloor_dBm_);

           ue_dl_update_chest_i(&q->chest_res,
                                sinrResult.sinr_dB_,
                                sinrResult.noiseFloor_dBm_);

           for(const auto & phich_subMsg : phich_message.submsg())

           if(rnti                == phich_subMsg.rnti()        && 
              grant->n_prb_lowest == phich_subMsg.num_prb_low() &&
              grant->n_dmrs       == phich_subMsg.num_dmrs())
            {
              Info("PHICH:%s found cc=%u, rnti 0x%hx, ack %d", __func__, cc_idx, rnti, phich_subMsg.ack());

              result->ack_value = phich_subMsg.ack();
              result->distance  = 1.0;
            }
           else
            {
              Debug("PHICH:%s not for us cc=%u, rnti 0x%hx/0x%hx, n_prbl %d/%d, n_bmrs %d/%d, ack %d",
                    __func__, 
                    cc_idx,
                    rnti,
                    phich_subMsg.rnti(), 
                    grant->n_prb_lowest,
                    phich_subMsg.num_prb_low(),
                    grant->n_dmrs,
                    phich_subMsg.num_dmrs(),
                    phich_subMsg.ack());
            }
         }
        else
         {
           Warning("PHICH:%s fail, cc=%u, txCarrierId %u, txFrequency %lu, phich_seqnum %u, sinr %f, noise %f",
                 __func__, cc_idx, txCarrierId, txFrequencyHz, phich_message.seqnum(), sinrResult.sinr_dB_, sinrResult.noiseFloor_dBm_);

           rc = SRSRAN_ERROR;
         }
      }
   }

   return rc;
}


int ue_dl_cc_decode_pmch(srsran_ue_dl_t*     q,
                         srsran_dl_sf_cfg_t* sf,
                         srsran_pmch_cfg_t*  cfg,
                         srsran_pdsch_res_t  data[SRSRAN_MAX_CODEWORDS],
                         const uint32_t cc_idx)
{
   int rc = SRSRAN_SUCCESS;

   const auto area_id = cfg->area_id;

   for(uint32_t tb = 0; tb < SRSRAN_MAX_CODEWORDS; ++tb)
    {
      if(cfg->pdsch_cfg.grant.tb[tb].enabled)
       {
         const auto carrierResults = getRxCarriers(dlMessage_, cc_idx, q->cell.id, __func__);

         if(! carrierResults.empty())
          {
            // ue supports 1 antenna
            const uint32_t rxAntennaId = 0;

            const auto & carrier = carrierResults[rxAntennaId];

            if(carrier.has_pmch())
             {
               const auto & pmch_message = carrier.pmch();

               for(const auto & pmch_subMsg : pmch_message.submsg())
                {
                  if(area_id == pmch_subMsg.area_id())
                   {
                     const auto txFrequencyHz = carrier.frequency_hz();
                     const auto txCarrierId   = carrier.carrier_id();

                     const auto sinrResult = 
                       DL_SINRTester_Get(dlMessage_).sinrCheck2(EMANELTE::MHAL::CHAN_PMCH,
                                                                txFrequencyHz,
                                                                rxAntennaId,
                                                                txCarrierId);

                     if(sinrResult.bPassed_)
                      {
                        Info("PMCH:%s: pass, cc=%u, tb %u, tbs %u, nbytes %d, seqnum %u, txCarrierId %u, sinr %f, noise %f",
                             __func__, 
                             cc_idx, 
                             tb,
                             cfg->pdsch_cfg.grant.tb[tb].tbs,
                             pmch_subMsg.data().length(),
                             pmch_message.seqnum(),
                             txCarrierId,
                             sinrResult.sinr_dB_,
                             sinrResult.noiseFloor_dBm_);

                        ue_dl_update_chest_i(&q->chest_res,
                                             sinrResult.sinr_dB_,
                                             sinrResult.noiseFloor_dBm_);

                        memcpy(data[tb].payload, pmch_subMsg.data().data(), pmch_subMsg.data().length());
 
                        data[tb].avg_iterations_block = 1;
                        data[tb].crc = true;
                      }
                     else
                      {
                        data[tb].avg_iterations_block = 0;
                        data[tb].crc = false;

                        Warning("PMCH:%s: fail, cc=%u, seqnum %u, txCarrierId %u, sinr %f, noise %f",
                            __func__, cc_idx, pmch_message.seqnum(), txCarrierId, sinrResult.sinr_dB_, sinrResult.noiseFloor_dBm_);

                        rc = SRSRAN_ERROR;
                      }

                     // done with this entry
                     break;
                   }
                  else
                   {
                     Info("PMCH:%s: cc=%u, dl_area_id %u != area_id %hu, skip", __func__, cc_idx, pmch_subMsg.area_id(), area_id);
                   }
                }
             }
          }
       }
      else
       {
         if(tb == 0)
           Info("PMCH:%s: cc=%u, tb %u is not enabled ", __func__, cc_idx, tb);
       }
     }

  return rc;
}


void ue_ul_tx_init()
{
  Debug("TX:%s:", __func__);
}


// send to mhal
void ue_ul_send_signal(const time_t sot_sec, const float frac_sec, const srsran_cell_t & cell)
{
  // end of tx sequence, tx_end will release lock
  std::lock_guard<std::mutex> lock(ul_mutex_);

  ulMessage_.set_crnti(crnti_);
  ulMessage_.set_tti(tti_tx_);

  EMANELTE::MHAL::Data data;

  if(ulMessage_.SerializeToString(&data))
   {
     // align sot to sf time
     const timeval tv_sf_time = {sot_sec, (time_t)(round(frac_sec * 1e3)*1e3)};
     
     auto ts = txControl_.mutable_sf_time();
     ts->set_ts_sec(tv_sf_time.tv_sec);
     ts->set_ts_usec(tv_sf_time.tv_usec);

     txControl_.set_message_type(EMANELTE::MHAL::UPLINK);
     txControl_.set_tx_seqnum(tx_seqnum_++);
     txControl_.set_tti_tx(tti_tx_);

#undef  UL_PHY_DEBUG
#ifdef  UL_PHY_DEBUG
     Info("MHAL:%s ulMessage %s\n", __func__, ulMessage_.DebugString().c_str());
#endif

     EMANELTE::MHAL::UE::send_msg(data, txControl_);
   }
 else
   {
     Error("TX:%s: SerializeToString ERROR len %zu", __func__, data.length());
   }

  // msg sent clear old data
  ulMessage_.Clear();

  txControl_.Clear();
}


void ue_ul_put_prach(const int index)
{
  std::lock_guard<std::mutex> lock(ul_mutex_);

  // use cc_idx 0 for prach
  const uint32_t cc_idx = 0;

  // tx frequency for carrier idx
  const auto txFrequencyHz = getTxFrequency(cc_idx);

  auto control = getTxCarrier<EMANELTE::MHAL::TxControlCarrierMessage, 
                              EMANELTE::MHAL::TxControlMessage>(txControl_, txFrequencyHz, cellIdInfo_[cc_idx], cc_idx);

  auto channelMessage = control->mutable_uplink()->mutable_prach();

  initUplinkChannelMessage(channelMessage,
                           EMANELTE::MHAL::CHAN_PRACH,
                           EMANELTE::MHAL::MOD_BPSK,   // modtype
                           839);                       // PRACH sequence is 839 for formats 0-3 (all allowed by FDD) 

  // The upstream PRACH message is not really a slotted message
  // and can span 2 or 3 subframes. Set slot1 and slot2 resource blocks the same.
  // prach spans the 6 resource blocks starting from prach_freq_offset
  for(int i = 0; i < 6; ++i)
   {
     channelMessage->add_resource_block_frequencies_slot1(EMANELTE::MHAL::UE::get_tx_prb_frequency(prach_freq_offset_ + i, txFrequencyHz));
     channelMessage->add_resource_block_frequencies_slot2(EMANELTE::MHAL::UE::get_tx_prb_frequency(prach_freq_offset_ + i, txFrequencyHz));
   }

  auto carrier = getTxCarrier<EMANELTE::MHAL::UE_UL_Message_CarrierMessage,
                              EMANELTE::MHAL::UE_UL_Message>(ulMessage_, txFrequencyHz, cellIdInfo_[cc_idx], cc_idx);

  auto prach    = carrier->mutable_prach();
  auto preamble = prach->mutable_preamble();

  preamble->set_index(index);

  Info("PRACH:%s: index %d", __func__, index);
}


int ue_ul_put_pucch_i(srsran_ue_ul_t* q, 
                      srsran_ul_sf_cfg_t* sf,
                      srsran_ue_ul_cfg_t* cfg,
                      srsran_uci_value_t* uci_data,
                      const uint32_t cc_idx)
{
   std::lock_guard<std::mutex> lock(ul_mutex_);

   const auto txFrequencyHz = getTxFrequency(cc_idx);

   auto carrier = getTxCarrier<EMANELTE::MHAL::UE_UL_Message_CarrierMessage,
                               EMANELTE::MHAL::UE_UL_Message>(ulMessage_, txFrequencyHz, cellIdInfo_[cc_idx], cc_idx);

   auto pucch_message = carrier->mutable_pucch();
   auto grant_message = pucch_message->add_grant();
   auto pucch_cfg     = cfg->ul_cfg.pucch;
   const auto rnti    = pucch_cfg.rnti;

   pucch_message->set_seqnum(pucch_seqnum_++);

   srsran_uci_value_t uci_data2 = *uci_data;

#if 1 // XXX is this needed
   // see lib/src/phy/ue/ue_ul.c
   srsran_ue_ul_pucch_resource_selection(&q->cell, 
                                         &cfg->ul_cfg.pucch,
                                         &cfg->ul_cfg.pucch.uci_cfg,
                                         &uci_data2,
                                         uci_data2.ack.ack_value);
#endif

   // default: SRSRAN_PUCCH_FORMAT_1
   EMANELTE::MHAL::MOD_TYPE modType = EMANELTE::MHAL::MOD_BPSK;

   uint32_t bits = 0;

   switch(pucch_cfg.format)
     {
     case SRSRAN_PUCCH_FORMAT_1:   // 1 HARQ ACK
       bits = 0;
       modType = EMANELTE::MHAL::MOD_BPSK;
       break;
     case SRSRAN_PUCCH_FORMAT_1A:  // 1 HARQ ACK
       bits = 1;
       modType = EMANELTE::MHAL::MOD_BPSK;
       break;
     case SRSRAN_PUCCH_FORMAT_1B:  // 2 HARQ ACK
       bits = 2;
       modType = EMANELTE::MHAL::MOD_QPSK;
       break;
     case SRSRAN_PUCCH_FORMAT_2:   // CSI
       bits = 20;
       modType = EMANELTE::MHAL::MOD_QPSK;
       break;
     case SRSRAN_PUCCH_FORMAT_2A:  // CSI + 1 HARQ ACK
       bits = 21;
       modType = EMANELTE::MHAL::MOD_QPSK;
       break;
     case SRSRAN_PUCCH_FORMAT_2B:  // CSI + 2 HARQ ACK
       bits = 22;
       modType = EMANELTE::MHAL::MOD_QPSK;
       break;
     case SRSRAN_PUCCH_FORMAT_ERROR:
     default:
       Error("PUCCH:ue_ul_put_pucch: unknown pucch format: %d",
             pucch_cfg.format);
     }

   auto control = getTxCarrier<EMANELTE::MHAL::TxControlCarrierMessage, 
                               EMANELTE::MHAL::TxControlMessage>(txControl_, txFrequencyHz, cellIdInfo_[cc_idx], cc_idx);

   auto channelMessage = control->mutable_uplink()->add_pucch();

   initUplinkChannelMessage(channelMessage,
                            EMANELTE::MHAL::CHAN_PUCCH,
                            modType,
                            bits);

   channelMessage->set_rnti(pucch_cfg.rnti);

   // see lib/src/phy/phch/pucch.c 
   // pucch_cp(srsran_pucch_t* q, 
   //          srsran_ul_sf_cfg_t* sf, 
   //          srsran_pucch_cfg_t* cfg, 
   //          cf_t* source, cf_t* dest,
   //          bool source_is_grid)
 
   // Determine n_prb
   uint16_t n_prb[2] = {0};

   for(int ns = 0; ns < 2; ++ns)
     {
       if(! ((n_prb[ns] = srsran_pucch_n_prb(&q->cell, &pucch_cfg, ns)) < q->cell.nof_prb))
         {
           Error("PUCCH:%s ns %d, n_prb=%d > cell_nof_prb %d", 
                 __func__, ns, n_prb[ns], q->cell.nof_prb);

           return SRSRAN_ERROR;
         }
     }

   // flag when resource blocks are different on slot 1 and 2 of the subframe
   channelMessage->add_resource_block_frequencies_slot1(EMANELTE::MHAL::UE::get_tx_prb_frequency(n_prb[0], txFrequencyHz));
   channelMessage->add_resource_block_frequencies_slot2(EMANELTE::MHAL::UE::get_tx_prb_frequency(n_prb[1], txFrequencyHz));

   grant_message->set_num_prb(n_prb[1]);
   grant_message->set_num_pucch(pucch_cfg.n_pucch);
   grant_message->set_rnti(rnti);
   grant_message->set_uci(&uci_data2, sizeof(srsran_uci_value_t));

#if 1
   char logbuf[256] = {0};
   srsran_uci_data_info(&cfg->ul_cfg.pucch.uci_cfg, &uci_data2, logbuf, sizeof(logbuf));

   Info("PUCCH:%s: cc=%u, txFrequency %lu, rnti 0x%hx, pucch_seqnum %u, uci_info [%s]", 
         __func__, cc_idx, txFrequencyHz, rnti, pucch_message->seqnum(), logbuf);
#endif

   // signal ready
   return 1;
}


static int ue_ul_put_pusch_i(srsran_pusch_cfg_t* cfg, srsran_pusch_data_t* data, const uint32_t cc_idx)
{
   std::lock_guard<std::mutex> lock(ul_mutex_);

   const auto txFrequencyHz = getTxFrequency(cc_idx);

   auto control = getTxCarrier<EMANELTE::MHAL::TxControlCarrierMessage, 
                               EMANELTE::MHAL::TxControlMessage>(txControl_, txFrequencyHz, cellIdInfo_[cc_idx], cc_idx);

   auto channelMessage = control->mutable_uplink()->add_pucch();

   const auto grant = &cfg->grant;
   const auto rnti  = cfg->rnti;

   initUplinkChannelMessage(channelMessage,
                            EMANELTE::MHAL::CHAN_PUSCH,
                            convert(grant->tb.mod),
                            grant->tb.tbs);

   channelMessage->set_rnti(rnti);

   for(size_t i = 0; i < grant->L_prb; ++i)
    {
      channelMessage->add_resource_block_frequencies_slot1(EMANELTE::MHAL::UE::get_tx_prb_frequency(grant->n_prb[0] + i, txFrequencyHz));
      channelMessage->add_resource_block_frequencies_slot2(EMANELTE::MHAL::UE::get_tx_prb_frequency(grant->n_prb[1] + i, txFrequencyHz));
    }

   auto carrier = getTxCarrier<EMANELTE::MHAL::UE_UL_Message_CarrierMessage,
                               EMANELTE::MHAL::UE_UL_Message>(ulMessage_, txFrequencyHz, cellIdInfo_[cc_idx], cc_idx);

   auto pusch_message = carrier->mutable_pusch();
   pusch_message->set_seqnum(pusch_seqnum_++);

   auto grant_message = pusch_message->add_grant();

   grant_message->set_rnti(rnti);

   // srsran_pusch_grant_t
   grant_message->set_ul_grant(grant, sizeof(srsran_pusch_grant_t));

   // srsran_uci_value_t
   grant_message->set_uci(&data->uci, sizeof(srsran_uci_value_t));

   // payload
   grant_message->set_payload(data->ptr, tbs_to_bytes(grant->tb.tbs));

#if 1
   char logbuf[256] = {0};
   srsran_uci_data_info(&cfg->uci_cfg, &data->uci, logbuf, sizeof(logbuf));

   Info("PUSCH:%s: cc=%u, txFrequency %lu, rnti 0x%hx, pusch_seqnum %u, uci_info [%s]",
        __func__, cc_idx, txFrequencyHz, rnti, pusch_message->seqnum(), logbuf);
#endif

   UESTATS::putULGrant(rnti);

   // signal ready
   return 1;
}


// see lib/src/phy/ue/ue_ul.c
// srsran_ue_ul_encode(srsran_ue_ul_t* q, 
//                     srsran_ul_sf_cfg_t* sf,
//                     srsran_ue_ul_cfg_t* cfg,
//                     srsran_pusch_data_t* data);
//
int ue_ul_encode(srsran_ue_ul_t* q, srsran_ul_sf_cfg_t* sf, srsran_ue_ul_cfg_t* cfg, srsran_pusch_data_t* data, const uint32_t cc_idx)
{
  /* Convert DTX to NACK in channel-selection mode (Release 10 only)*/
  if(cfg->ul_cfg.pucch.ack_nack_feedback_mode != SRSRAN_PUCCH_ACK_NACK_FEEDBACK_MODE_NORMAL) {
    uint32_t dtx_count = 0;
    for(uint32_t a = 0; a < srsran_uci_cfg_total_ack(&cfg->ul_cfg.pusch.uci_cfg); a++) {
      if(data->uci.ack.ack_value[a] == 2) {
        data->uci.ack.ack_value[a] = 0;
        dtx_count++;
      }
    }

    /* If all bits are DTX, do not transmit HARQ */
    if(dtx_count == srsran_uci_cfg_total_ack(&cfg->ul_cfg.pusch.uci_cfg)) {
      for (int i = 0; i < 2; i++) { // Format 1b-CS only supports 2 CC
       cfg->ul_cfg.pusch.uci_cfg.ack[i].nof_acks = 0;
      }
    }
  }

  ue_set_crnti_i(cfg->ul_cfg.pucch.rnti);

   // see lib/src/phy/ue/ue_ul.c
#define uci_pending(cfg) (srsran_uci_cfg_total_ack(&cfg) > 0 || cfg.cqi.data_enable || cfg.cqi.ri_len > 0)
   if(cfg->grant_available) 
    {
      return ue_ul_put_pusch_i(&cfg->ul_cfg.pusch, data, cc_idx);
    } 
   else if(uci_pending(cfg->ul_cfg.pucch.uci_cfg) || data->uci.scheduling_request)
    // Send PUCCH over PCell only
    {
      if(cfg->cc_idx == 0)
       {
         return ue_ul_put_pucch_i(q, sf, cfg, &data->uci, cc_idx);
       }
      else
       {
         Warning("PUCCH:%s: cc=%u != 0, skip rnti 0x%hx on non pcell", __func__, cc_idx, cfg->ul_cfg.pucch.rnti);
         return 0;
       }
    }
   else
    {
      return 0;
    }
}


std::set<uint32_t>
ue_get_detected_cells(const srsran_cell_t & cell)
{
   const auto time_now = time(NULL);

   // time out > 10 sec
   const time_t timeout = 10;

   std::set<uint32_t> detected_cells;

   for(auto iter = nbr_cells_.begin(); iter != nbr_cells_.end(); /* check/bump below */)
    {
      if(NbrCell_Timestamp_Get(*iter) + timeout >= time_now)
       {
         detected_cells.insert(NbrCell_Cellid_Get(*iter));
   
         ++iter;
       }
     else
       {
         nbr_cells_.erase(iter++);
       }
    }

   return detected_cells;
}


void ue_get_refsignals(srsran_refsignal_dl_sync_t & refsignal_dl_sync, const uint32_t cell_id)
{
   for(const auto & nbr : nbr_cells_)
    {
      if(NbrCell_Cellid_Get(nbr) == cell_id)
        {
          refsignal_dl_sync.found      = true;
          refsignal_dl_sync.rsrp_dBfs  = phy_adapter::ue_snr_to_rsrp(NbrCell_Snr_Get(nbr));
          refsignal_dl_sync.rsrq_dB    = phy_adapter::ue_snr_to_rsrq(NbrCell_Snr_Get(nbr));
          refsignal_dl_sync.rssi_dBfs  = phy_adapter::ue_snr_to_rssi(NbrCell_Snr_Get(nbr), NbrCell_Nf_Get(nbr));
          refsignal_dl_sync.cfo_Hz     = 0;
          refsignal_dl_sync.peak_index = 0;

          return;
       }
    }

   refsignal_dl_sync.found      = false;
   refsignal_dl_sync.rsrp_dBfs  = 0;
   refsignal_dl_sync.rssi_dBfs  = 0;
   refsignal_dl_sync.rsrq_dB    = 0;
   refsignal_dl_sync.cfo_Hz     = 0;
   refsignal_dl_sync.peak_index = 0;
}


} // end namespace phy_adapter
} // end namepsace srsue
