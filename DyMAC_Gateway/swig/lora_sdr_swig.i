/* -*- c++ -*- */

#define LORA_SDR_API

%include "gnuradio.i"			// the common stuff

//load generated python docstrings
//%include "lora_sdr_swig_doc.i"

%{
#include "lora_sdr/add_crc.h"
#include "lora_sdr/crc_verif.h"
#include "lora_sdr/dewhitening.h"
#include "lora_sdr/gray_decode.h"
#include "lora_sdr/gray_enc.h"
#include "lora_sdr/hamming_dec.h"
#include "lora_sdr/hamming_enc.h"
#include "lora_sdr/header_decoder.h"
#include "lora_sdr/header.h"
#include "lora_sdr/interleaver.h"
#include "lora_sdr/modulate.h"
#include "lora_sdr/whitening.h"
#include "lora_sdr/RH_RF95_header.h"
#include "lora_sdr/fft_demod.h"
#include "lora_sdr/data_source.h"
#include "lora_sdr/frame_sync.h"
#include "lora_sdr/deinterleaver.h"
#include "lora_sdr/mu_detection.h"
#include "lora_sdr/mu_synchro.h"
#include "lora_sdr/partial_ml.h"
#include "lora_sdr/noise_est.h"
#include "lora_sdr/frame_src.h"
#include "lora_sdr/signal_detector.h"
#include "lora_sdr/sdrCadDetect.h"
#include "lora_sdr/multiCadLora.h"
#include "lora_sdr/sendBeacon.h"
#include "lora_sdr/readParam.h"
#include "lora_sdr/recordParams.h"
#include "lora_sdr/SendRecordParam.h"
#include "lora_sdr/messageStrobeLoRa.h"
#include "lora_sdr/btmaSendBeacon.h"
#include "lora_sdr/messageAndTimer.h"
#include "lora_sdr/snr_estimate.h"
%}


%include "lora_sdr/add_crc.h"
GR_SWIG_BLOCK_MAGIC2(lora_sdr, add_crc);
%include "lora_sdr/crc_verif.h"
GR_SWIG_BLOCK_MAGIC2(lora_sdr, crc_verif);

%include "lora_sdr/dewhitening.h"
GR_SWIG_BLOCK_MAGIC2(lora_sdr, dewhitening);

%include "lora_sdr/gray_decode.h"
GR_SWIG_BLOCK_MAGIC2(lora_sdr, gray_decode);
%include "lora_sdr/gray_enc.h"
GR_SWIG_BLOCK_MAGIC2(lora_sdr, gray_enc);
%include "lora_sdr/hamming_dec.h"
GR_SWIG_BLOCK_MAGIC2(lora_sdr, hamming_dec);
%include "lora_sdr/hamming_enc.h"
GR_SWIG_BLOCK_MAGIC2(lora_sdr, hamming_enc);
%include "lora_sdr/header_decoder.h"
GR_SWIG_BLOCK_MAGIC2(lora_sdr, header_decoder);
%include "lora_sdr/header.h"
GR_SWIG_BLOCK_MAGIC2(lora_sdr, header);
%include "lora_sdr/interleaver.h"
GR_SWIG_BLOCK_MAGIC2(lora_sdr, interleaver);
%include "lora_sdr/modulate.h"
GR_SWIG_BLOCK_MAGIC2(lora_sdr, modulate);

%include "lora_sdr/whitening.h"
GR_SWIG_BLOCK_MAGIC2(lora_sdr, whitening);
%include "lora_sdr/RH_RF95_header.h"
GR_SWIG_BLOCK_MAGIC2(lora_sdr, RH_RF95_header);


%include "lora_sdr/fft_demod.h"
GR_SWIG_BLOCK_MAGIC2(lora_sdr, fft_demod);
%include "lora_sdr/data_source.h"
GR_SWIG_BLOCK_MAGIC2(lora_sdr, data_source);
%include "lora_sdr/frame_sync.h"
GR_SWIG_BLOCK_MAGIC2(lora_sdr, frame_sync);

%include "lora_sdr/deinterleaver.h"
GR_SWIG_BLOCK_MAGIC2(lora_sdr, deinterleaver);


%include "lora_sdr/mu_detection.h"
GR_SWIG_BLOCK_MAGIC2(lora_sdr, mu_detection);

%include "lora_sdr/mu_synchro.h"
GR_SWIG_BLOCK_MAGIC2(lora_sdr, mu_synchro);
%include "lora_sdr/partial_ml.h"
GR_SWIG_BLOCK_MAGIC2(lora_sdr, partial_ml);

%include "lora_sdr/noise_est.h"
GR_SWIG_BLOCK_MAGIC2(lora_sdr, noise_est);
%include "lora_sdr/frame_src.h"
GR_SWIG_BLOCK_MAGIC2(lora_sdr, frame_src);
%include "lora_sdr/signal_detector.h"
GR_SWIG_BLOCK_MAGIC2(lora_sdr, signal_detector);
%include "lora_sdr/sdrCadDetect.h"
GR_SWIG_BLOCK_MAGIC2(lora_sdr, sdrCadDetect);
%include "lora_sdr/multiCadLora.h"
GR_SWIG_BLOCK_MAGIC2(lora_sdr, multiCadLora);
%include "lora_sdr/sendBeacon.h"
GR_SWIG_BLOCK_MAGIC2(lora_sdr, sendBeacon);
%include "lora_sdr/readParam.h"
GR_SWIG_BLOCK_MAGIC2(lora_sdr, readParam);
%include "lora_sdr/recordParams.h"
GR_SWIG_BLOCK_MAGIC2(lora_sdr, recordParams);
%include "lora_sdr/SendRecordParam.h"
GR_SWIG_BLOCK_MAGIC2(lora_sdr, SendRecordParam);
%include "lora_sdr/messageStrobeLoRa.h"
GR_SWIG_BLOCK_MAGIC2(lora_sdr, messageStrobeLoRa);
%include "lora_sdr/btmaSendBeacon.h"
GR_SWIG_BLOCK_MAGIC2(lora_sdr, btmaSendBeacon);
%include "lora_sdr/messageAndTimer.h"
GR_SWIG_BLOCK_MAGIC2(lora_sdr, messageAndTimer);
%include "lora_sdr/snr_estimate.h"
GR_SWIG_BLOCK_MAGIC2(lora_sdr, snr_estimate);
