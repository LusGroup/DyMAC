
#ifndef INCLUDED_LORA_SDR_SNR_ESTIMATE_IMPL_H
#define INCLUDED_LORA_SDR_SNR_ESTIMATE_IMPL_H
#include <gnuradio/gr_complex.h>
#include <vector>
#define GRLORA_MEASUREMENTS

#include <lora_sdr/snr_estimate.h>
#include <iostream>
#include <fstream>
#include <volk/volk.h>
#include <lora_sdr/utilities.h>
#include <gnuradio/io_signature.h>
extern "C" {
  #include "kiss_fft.h"
}

namespace gr {
  namespace lora_sdr {

    class snr_estimate_impl : public snr_estimate
    {
     private:
      // Nothing to declare in this block.
      enum DecoderState {
             DETECT,
             SYNC,
             FRAC_CFO_CORREC,
             STOP
         };
         enum SyncState {
             NET_ID1,
             NET_ID2,
             DOWNCHIRP1,
             DOWNCHIRP2,
             QUARTER_DOWN
         };
          uint8_t m_state;        ///< Current state of the synchronization
         uint32_t m_bw;          ///< Bandwidth
         uint32_t m_samp_rate;   ///< Sampling rate
         uint8_t m_sf;           ///< Spreading factor
         uint8_t m_cr;           ///< Coding rate
         uint32_t m_pay_len;     ///< payload length
         uint8_t m_has_crc;      ///< CRC presence
         uint8_t m_invalid_header;///< invalid header checksum
         bool m_impl_head;       ///< use implicit header mode
         std::vector<uint16_t> m_sync_words; ///< vector containing the two sync words (network identifiers)
         pmt::pmt_t m_cad_port;
        uint8_t m_freqIndex;

         uint32_t m_number_of_bins;      ///< Number of bins in each lora Symbol
         uint32_t m_samples_per_symbol;  ///< Number of samples received per lora symbols
         uint32_t m_symb_numb;             ///<number of payload lora symbols
         bool m_received_head;          ///< indicate that the header has be decoded and received by this block
         bool m_pkt_detected;//自己定义
         std::vector<gr_complex> in_down; ///< downsampled input
        // std::vector<gr_complex> in_double_down; ///< downsampled input(double chirp samples)
         
         std::vector<gr_complex> m_downchirp; ///< Reference downchirp
         std::vector<gr_complex> m_upchirp;   ///< Reference upchirp
        std::vector<gr_complex> m_double_upchirp;   ///< Reference double_upchirp

         int32_t symbol_cnt;         ///< Number of symbols already received
         int32_t bin_idx;            ///< value of previous lora symbol
         int32_t bin_idx_new;        ///< value of newly demodulated symbol

         uint32_t n_up;              ///< Number of consecutive upchirps in preamble
         uint8_t symbols_to_skip;    ///< Number of integer symbol to skip after consecutive upchirps

         kiss_fft_cpx *cx_in;        ///<input of the FFT
         kiss_fft_cpx *cx_out;       ///<output of the FFT

         kiss_fft_cpx *cx_in_double;        ///<input of the FFT
         kiss_fft_cpx *cx_out_double;       ///<output of the FFT

         int items_to_consume;       ///< Number of items to consume after each iteration of the general_work function

         std::vector<gr_complex> preamble_raw;///<vector containing the preamble upchirps without any synchronization
         std::vector<gr_complex> preamble_up; ///<vector containing the preamble upchirps

         int up_symb_to_use; ///<number of upchirp symbols to use for CFO and STO frac estimation
         int k_hat;          ///<integer part of CFO+STO
         float lambda_cfo;  ///<fractional part of CFO
         float lambda_bernier; ///<fractional part of CFO using Berniers algo
         float lambda_sto;  ///<fractional part of CFO
         bool cfo_sto_est; ///< indicate that the estimation of lambda_cfo/sto has been performed
         int usFactor;       ///<upsampling factor used by the FIR interpolator
         std::vector<gr_complex> CFO_frac_correc; ///<cfo frac correction vector


         std::vector<gr_complex> symb_corr; ///< symbol with CFO frac corrected
         std::vector<gr_complex> symb_corr_all;//
         std::vector<gr_complex> symb_corr_doublechirp;//
         float max_mag=0;
         int down_val; ///< value of the preamble downchirps
         float up_max_mag=0;
         float down_max_mag=0;

         int CFOint; ///< integer part of CFO
         int net_id_off; ///<offset of the network identifier

         unsigned char d_preamble_len;//
         unsigned char d_header_len;//

         float current_snr=0;
         float current_EE;
         float PDR;
         double DR;

         pmt::pmt_t m_inSFPort;
         void setSF(pmt::pmt_t msg);


        #ifdef GRLORA_MEASUREMENTS
         int off_by_one_id; ///< Indicate that the network identifiers where off by one and corrected
         std::ofstream sync_log; ///< savefile containing the offset estimation and the signal strength estimation
         #endif
      /**
         *   \brief  Handle the reception of the explicit header information, received from the header_decoder block 
         */
         void frame_info_handler(pmt::pmt_t frame_info);

         /**
          *  \brief  Estimate the value of fractional part of the CFO using RCTSL
          *  \param  samples
          *          The pointer to the preamble beginning.(We might want to avoid the
          *          first symbol since it might be incomplete)
          */
         void estimate_CFO(gr_complex* samples);
         /**
          *  \brief  (not used) Estimate the value of fractional part of the CFO using Berniers algorithm
          */
         void estimate_CFO_Bernier();
         /**
          *  \brief  Estimate the value of fractional part of the STO from m_consec_up
          **/
         void estimate_STO();
         /**
          *  \brief  Recover the lora symbol value using argmax of the dechirped symbol FFT. Returns -1 in case of an fft window containing no energy to handle noiseless simulations.
          *
          *  \param  samples
          *          The pointer to the symbol beginning.
          *  \param  ref_chirp
          *          The reference chirp to use to dechirp the lora symbol.
          */
         uint32_t get_symbol_val(const gr_complex *samples,gr_complex *ref_chirp);
         float get_double_symbol_val(const gr_complex *samples, gr_complex *ref_chirp);
         float get_symbol_val_upchirp(const gr_complex *samples, gr_complex *ref_chirp);
          /**
          *  \brief  Determine the energy of a symbol.
          *
          *  \param  samples
          *          The complex symbol to analyse.
          */
         float determine_energy(const gr_complex *samples);

         float determine_snr(const gr_complex *samples);

          float calculate_DR(unsigned char sf,unsigned int bw);

          float calculate_PDR(float snr,unsigned char sf,float n,float lh,float lp);

          float calculate_EE(float dr,float pdr,uint8_t tp);
          
          float calculate_P_b(float snr,unsigned char sf);
          float calculate_P_preamble(float snr,unsigned char sf,float preamble_len);
          float calculate_P_header(float snr,unsigned char sf,float header_len);
          float calculate_P_payload(float snr,unsigned char sf,float payload_len);
          float Q_function(float x);
          void ChannelEfficiency();//Generate transmission energy efficiency for channel transmission.

          /**
          *  \brief  Handles the error message coming from the header decoding.
          */
         void header_err_handler(pmt::pmt_t payload_len);

     public:
      snr_estimate_impl(float samp_rate, uint32_t bandwidth, uint8_t sf, bool impl_head, std::vector<uint16_t> sync_word,uint8_t  freqIndex);
      ~snr_estimate_impl();

      // Where all the action really happens
      void forecast (int noutput_items, gr_vector_int &ninput_items_required);

      int general_work(int noutput_items,
           gr_vector_int &ninput_items,
           gr_vector_const_void_star &input_items,
           gr_vector_void_star &output_items);

    };

  } // namespace lora_sdr
} // namespace gr

#endif /* INCLUDED_LORA_SDR_SNR_ESTIMATE_IMPL_H */

