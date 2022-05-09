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

#include "srsran/srsran.h"
#include "rf_dummy_imp.h"
#include "rf_helper.h"
#include "rf_plugin.h"
#include "srsran/phy/rf/rf.h"

#include <assert.h>
#include <unistd.h>

static bool log_stdout = true;

#define LOG_FMT "%02d:%02d:%02d.%06ld [DMY] [%c] %s "

#define LOG_INFO(_fmt, ...) do { if(log_stdout) {                             \
                                   struct timeval _tv_now;                    \
                                   struct tm _tm;                             \
                                   gettimeofday(&_tv_now, NULL);              \
                                   localtime_r(&_tv_now.tv_sec, &_tm);        \
                                   fprintf(stdout, LOG_FMT _fmt "\n",         \
                                           _tm.tm_hour,                       \
                                           _tm.tm_min,                        \
                                           _tm.tm_sec,                        \
                                           _tv_now.tv_usec,                   \
                                           'I',                               \
                                           __func__,                          \
                                           ##__VA_ARGS__);                    \
                               }                                              \
                             } while(0);


// rf dev info
typedef struct {
   int                  nodetype;
   uint32_t             nof_tx_ports;
   uint32_t             nof_rx_ports;
   double               rx_gain;
   double               tx_gain;
   double               rx_srate;
   double               tx_srate;
   double               rx_freq;
   double               tx_freq;
   double               clock_rate;
   srsran_rf_error_handler_t error_handler;
   bool                 rx_stream;
   srsran_rf_info_t     rf_info;
} rf_dummy_info_t;


static uint32_t delay_usec = 1000;


static  rf_dummy_info_t rf_dummy_info = { .nof_tx_ports    = 1,
                                          .nof_rx_ports    = 1,
                                          .rx_gain         = 0.0,
                                          .tx_gain         = 0.0,
                                          .rx_srate        = SRSRAN_CS_SAMP_FREQ,
                                          .tx_srate        = SRSRAN_CS_SAMP_FREQ,
                                          .rx_freq         = 0.0,
                                          .tx_freq         = 0.0,
                                          .clock_rate      = 0.0,
                                          .rx_stream       = false,
                                          .rf_info         = {}
                                        };

#define GET_DEV_INFO(h)  assert(h); rf_dummy_info_t *_info = (rf_dummy_info_t *)(h)


// begin RF API

void rf_dummy_suppress_stdout(void *h)
 {
   log_stdout = false;
 }


const char* rf_dummy_devname(void *h)
 {
   return "dummy";
 }


bool rf_dummy_rx_wait_lo_locked(void *h)
 {
   LOG_INFO("");

   return false;
 }


int rf_dummy_start_rx_stream(void *h, bool now)
 {
   GET_DEV_INFO(h);
   
   _info->rx_stream = true;

   LOG_INFO("");

   return 0;
 }


int rf_dummy_stop_rx_stream(void *h)
 {
   GET_DEV_INFO(h);

   _info->rx_stream = false;

   LOG_INFO("");

   return 0;
 }


void rf_dummy_flush_buffer(void *h)
 {
   LOG_INFO("");
 }


bool rf_dummy_has_rssi(void *h)
 {
   return false;
 }


float rf_dummy_get_rssi(void *h)
 {
   return 0.0;
 }


void rf_dummy_register_error_handler(void *h, srsran_rf_error_handler_t error_handler, void * arg)
 {
   // nop
 }


int rf_dummy_open(char *args, void **h)
 {
   LOG_INFO("");

   return rf_dummy_open_multi(args, h, 1);
 }


int rf_dummy_open_multi(char *args, void **h, uint32_t nof_channels)
 {
   LOG_INFO("num_channels %d, args %s", nof_channels, args);

   *h = &rf_dummy_info;

   parse_uint32(args, "delay", 1000, &delay_usec);

   return 0;
 }


int rf_dummy_close(void *h)
 {
   LOG_INFO("");

   return 0;
 }


void rf_dummy_set_master_clock_rate(void *h, double rate)
 {
   GET_DEV_INFO(h);

   LOG_INFO("rate %6.4lf to %6.4lf", _info->clock_rate, rate);

   _info->clock_rate = rate;
 }


bool rf_dummy_is_master_clock_dynamic(void *h)
 {
   LOG_INFO("false");

   return false;
 }


int rf_dummy_set_rx_gain(void *h, double gain)
 {
   GET_DEV_INFO(h);

   LOG_INFO("gain %3.2lf to %3.2lf", _info->rx_gain, gain);

   _info->rx_gain = gain;

   return SRSRAN_SUCCESS;

 }


int rf_dummy_set_tx_gain(void *h, double gain)
 {
   GET_DEV_INFO(h);

   LOG_INFO("gain %3.2lf to %3.2lf", _info->tx_gain, gain);

   _info->tx_gain = gain;

   return SRSRAN_SUCCESS;
 }


double rf_dummy_get_rx_gain(void *h)
 {
   GET_DEV_INFO(h);

   LOG_INFO("gain %3.2lf", _info->rx_gain);

   return _info->rx_gain;
 }


double rf_dummy_get_tx_gain(void *h)
 {
   GET_DEV_INFO(h);

   LOG_INFO("gain %3.2lf", _info->tx_gain);

   return _info->tx_gain;
 }

srsran_rf_info_t * rf_dummy_get_rf_info(void *h)
  {
     GET_DEV_INFO(h);

     LOG_INFO("tx_gain min/max %3.2lf/%3.2lf, rx_gain min/max %3.2lf/%3.2lf",
                  _info->rf_info.min_tx_gain,
                  _info->rf_info.max_tx_gain,
                  _info->rf_info.min_rx_gain,
                  _info->rf_info.max_rx_gain);

     return &_info->rf_info;
  }

double rf_dummy_set_rx_srate(void *h, double rate)
 {
   GET_DEV_INFO(h);

   LOG_INFO("srate %4.2lf MHz to %4.2lf MHz", 
            _info->rx_srate / 1e6, rate / 1e6);

   _info->rx_srate = rate;

   return _info->rx_srate;
 }


double rf_dummy_set_tx_srate(void *h, double rate)
 {
   GET_DEV_INFO(h);

   LOG_INFO("srate %4.2lf MHz to %4.2lf MHz", 
            _info->tx_srate / 1e6, rate / 1e6);

   _info->tx_srate = rate;

   return _info->tx_srate;
 }


double rf_dummy_set_rx_freq(void *h, uint32_t ch, double freq)
 {
   GET_DEV_INFO(h);

   LOG_INFO("freq %4.2lf MHz to %4.2lf MHz", 
            _info->rx_freq / 1e6, freq / 1e6);

   _info->rx_freq = freq;

   return _info->rx_freq;
 }


double rf_dummy_set_tx_freq(void *h, uint32_t ch, double freq)
 {
   GET_DEV_INFO(h);

   LOG_INFO("freq %4.2lf MHz to %4.2lf MHz", 
             _info->tx_freq / 1e6, freq / 1e6);

   _info->tx_freq = freq;

   return _info->tx_freq;
 }


void rf_dummy_get_time(void *h, time_t *full_secs, double *frac_secs)
 {
   if(full_secs && frac_secs)
     {
       struct timeval tv;
       gettimeofday(&tv, NULL);

       *full_secs = tv.tv_sec; 
       *frac_secs = tv.tv_usec / 1e6;
     }
 }



int rf_dummy_recv_with_time(void *h, void *data, uint32_t nsamples, 
                            bool blocking, time_t *full_secs, double *frac_secs)
 {
   void *d[4] = {data, NULL, NULL, NULL};

   return rf_dummy_recv_with_time_multi(h, 
                                        d,
                                        nsamples, 
                                        blocking,
                                        full_secs,
                                        frac_secs);
 }



int rf_dummy_recv_with_time_multi(void *h, void **data, uint32_t nsamples, 
                                  bool blocking, time_t *full_secs, double *frac_secs)
{
   usleep(delay_usec);

   rf_dummy_get_time(h, full_secs, frac_secs);

   if(full_secs && frac_secs)
    {
      LOG_INFO("nsamples %u %ld:%0.6lf", nsamples, *full_secs, *frac_secs);
    }

   return nsamples;
}


int rf_dummy_send_timed(void *h, void *data, int nsamples,
                       time_t full_secs, double frac_secs, bool has_time_spec,
                       bool blocking, bool is_sob, bool is_eob)
{
   void *d[4] = {data, NULL, NULL, NULL};

   return rf_dummy_send_timed_multi(h, d, nsamples, full_secs, frac_secs, has_time_spec, blocking, is_sob, is_eob);
}


int rf_dummy_send_timed_multi(void *h, void *data[4], int nsamples,
                             time_t full_secs, double frac_secs, bool has_time_spec,
                             bool blocking, bool is_sob, bool is_eob)
{
   LOG_INFO("nsamples %u, sob %d, eob %d", nsamples, is_sob, is_eob);

   return nsamples;
}

#ifdef ENABLE_RF_PLUGINS
rf_dev_t srsran_rf_dev_dummy = {
  .name                              = "dummy",
  .srsran_rf_devname                 = rf_dummy_devname,
  .srsran_rf_start_rx_stream         = rf_dummy_start_rx_stream,
  .srsran_rf_stop_rx_stream          = rf_dummy_stop_rx_stream,
  .srsran_rf_flush_buffer            = rf_dummy_flush_buffer,
  .srsran_rf_has_rssi                = rf_dummy_has_rssi,
  .srsran_rf_get_rssi                = rf_dummy_get_rssi,
  .srsran_rf_suppress_stdout         = rf_dummy_suppress_stdout,
  .srsran_rf_register_error_handler  = rf_dummy_register_error_handler,
  .srsran_rf_open                    = rf_dummy_open,
  .srsran_rf_open_multi              = rf_dummy_open_multi,
  .srsran_rf_close                   = rf_dummy_close,
  .srsran_rf_set_rx_srate            = rf_dummy_set_rx_srate,
  .srsran_rf_set_rx_gain             = rf_dummy_set_rx_gain,
  .srsran_rf_set_tx_gain             = rf_dummy_set_tx_gain,
  .srsran_rf_get_rx_gain             = rf_dummy_get_rx_gain,
  .srsran_rf_get_tx_gain             = rf_dummy_get_tx_gain,
  .srsran_rf_get_info                = rf_dummy_get_rf_info,
  .srsran_rf_set_rx_freq             = rf_dummy_set_rx_freq,
  .srsran_rf_set_tx_srate            = rf_dummy_set_tx_srate,
  .srsran_rf_set_tx_freq             = rf_dummy_set_tx_freq,
  .srsran_rf_get_time                = rf_dummy_get_time,
  .srsran_rf_recv_with_time          = rf_dummy_recv_with_time,
  .srsran_rf_recv_with_time_multi    = rf_dummy_recv_with_time_multi,
  .srsran_rf_send_timed              = rf_dummy_send_timed,
  .srsran_rf_send_timed_multi        = rf_dummy_send_timed_multi
};

int register_plugin(rf_dev_t** rf_api)
{
  if (rf_api == NULL) {
    return SRSRAN_ERROR;
  }
  *rf_api = &srsran_rf_dev_dummy;
  return SRSRAN_SUCCESS;
}
#endif /* ENABLE_RF_PLUGINS */
