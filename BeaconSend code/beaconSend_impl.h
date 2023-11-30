/* -*- c++ -*- */
/*
 * Copyright 2023 gr-lora author.
 *
 * This is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 3, or (at your option)
 * any later version.
 *
 * This software is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this software; see the file COPYING.  If not, write to
 * the Free Software Foundation, Inc., 51 Franklin Street,
 * Boston, MA 02110-1301, USA.
 */

#ifndef INCLUDED_LORA_BEACONSEND_IMPL_H
#define INCLUDED_LORA_BEACONSEND_IMPL_H

#include <cstdint>
#include <lora/beaconSend.h>
#include <eigen3/Eigen/Dense>

namespace gr {
  namespace lora {

    class beaconSend_impl : public beaconSend
    {
     private:
      pmt::pmt_t m_inEEPort;
      pmt::pmt_t m_inPcntPort;
      pmt::pmt_t m_inTimePeriodPort;

      pmt::pmt_t m_outBeaconPort;
      
      float d_packet_num = 0;//Total number of packets detected by the gateway


      std::ifstream fin;
      std::fstream m_outFile;
     
      std::vector<std::vector<float>> load; //local load matrix
      std::vector<std::vector<float>> e;//local energy efficiency matrix

     public:
      beaconSend_impl();
      ~beaconSend_impl();

      void EE_matrixUpdate(pmt::pmt_t msg);//Gateway local matrix update function
      void channel_load_matrixUpdate(pmt::pmt_t msg);//Gateway local matrix update function
      void sendBeacon(pmt::pmt_t msg);//Beacon content generation function
      void matrix_init();//Matrix initialization function
      // Where all the action really happens
      void forecast (int noutput_items, gr_vector_int &ninput_items_required);

      int general_work(int noutput_items,
           gr_vector_int &ninput_items,
           gr_vector_const_void_star &input_items,
           gr_vector_void_star &output_items);

    };

  } // namespace lora
} // namespace gr

#endif /* INCLUDED_LORA_BEACONSEND_IMPL_H */

