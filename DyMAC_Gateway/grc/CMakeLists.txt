# Copyright 2011 Free Software Foundation, Inc.
#
# This file is part of GNU Radio
#
# GNU Radio is free software; you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation; either version 3, or (at your option)
# any later version.
#
# GNU Radio is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with GNU Radio; see the file COPYING.  If not, write to
# the Free Software Foundation, Inc., 51 Franklin Street,
# Boston, MA 02110-1301, USA.
install(FILES
    lora_sdr_add_crc.block.yml
    lora_sdr_crc_verif.block.yml
    lora_sdr_dewhitening.block.yml
    lora_sdr_gray_decode.block.yml
    lora_sdr_gray_enc.block.yml
    lora_sdr_hamming_dec.block.yml
    lora_sdr_hamming_enc.block.yml
    lora_sdr_header_decoder.block.yml
    lora_sdr_header.block.yml
    lora_sdr_interleaver.block.yml
    lora_sdr_modulate.block.yml
    lora_sdr_whitening.block.yml
    lora_sdr_RH_RF95_header.block.yml
    lora_sdr_fft_demod.block.yml
    lora_sdr_data_source.block.yml
    lora_sdr_frame_sync.block.yml
    lora_sdr_deinterleaver.block.yml
    lora_sdr_mu_detection.block.yml
    lora_sdr_mu_synchro.block.yml
    lora_sdr_partial_ml.block.yml
    lora_sdr_mu_demod.block.yml
    lora_sdr_noise_est.block.yml
    lora_sdr_frame_src.block.yml
    lora_sdr_signal_detector.block.yml
    lora_sdr_sdrCadDetect.block.yml
    lora_sdr_multiCadLora.block.yml
    lora_sdr_sendBeacon.block.yml
    lora_sdr_readParam.block.yml
    lora_sdr_recordParams.block.yml
    lora_sdr_SendRecordParam.block.yml
    lora_sdr_messageStrobeLoRa.block.yml
    lora_sdr_btmaSendBeacon.block.yml
    lora_sdr_messageAndTimer.block.yml
    lora_sdr_snr_estimate.block.yml DESTINATION share/gnuradio/grc/blocks
)
