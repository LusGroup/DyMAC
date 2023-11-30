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

#include "lora/beaconSend.h"
#include <cstdint>
#include <gnuradio/gr_complex.h>
#include <pmt/pmt_sugar.h>
#include <string>
#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include <gnuradio/io_signature.h>
#include "beaconSend_impl.h"


namespace gr {
  namespace lora {

    beaconSend::sptr
    beaconSend::make()
    {
      return gnuradio::get_initial_sptr
        (new beaconSend_impl());
    }


    /*
     * The private constructor
     */
    beaconSend_impl::beaconSend_impl()
      : gr::block("beaconSend",
              gr::io_signature::make(0, 0, 0),
              gr::io_signature::make(0, 0, 0))
    {
       m_inEEPort = pmt::intern("inEE");
       m_inPcntPort=pmt::intern("inPcnt");
		   m_outBeaconPort = pmt::intern("outBeacon");
       m_inTimePeriodPort=pmt::intern("inTimePeriod");
      
		   message_port_register_in(m_inEEPort);
       message_port_register_in(m_inPcntPort);
       message_port_register_in(m_inTimePeriodPort);
		   message_port_register_out(m_outBeaconPort);

       set_msg_handler(m_inEEPort, boost::bind(&beaconSend_impl::EE_matrixUpdate, this, _1));
       set_msg_handler(m_inTimePeriodPort, boost::bind(&beaconSend_impl::sendBeacon, this, _1));
       set_msg_handler(m_inPcntPort, boost::bind(&beaconSend_impl::channel_load_matrixUpdate, this, _1));

       matrix_init();
      //Open the ‘load_distribution.txt’ file to record the output data
		   m_outFile.open("load_distribution.txt",std::ios::out | std::ios::trunc);
		   //close file
		   m_outFile.close();
      

    }

    /*
     * Our virtual destructor.
     */
    beaconSend_impl::~beaconSend_impl()
    {
    }

    void 
    beaconSend_impl::matrix_init(){
          load.resize(5);
        for(int i=0;i<5;i++)
          load[i].resize(6);
      
      for(int i=0;i<5;i++){
        for(int j=0;j<6;j++)
          load[i][j] =0;
        }

        e.resize(5);
        for(int i=0;i<5;i++)
        e[i].resize(6);
      
      for(int i=0;i<5;i++){
        for(int j=0;j<6;j++)
          e[i][j] =0;
        }
    }
    
    void 
    beaconSend_impl::EE_matrixUpdate(pmt::pmt_t msg)
    {
      pmt::pmt_t pmt_freq = pmt::dict_ref(msg,pmt::intern("logicalFrequency"),NULL);
      pmt::pmt_t pmt_sf = pmt::dict_ref(msg,pmt::intern("logicalSf"),NULL);
      pmt::pmt_t pmt_ee = pmt::dict_ref(msg,pmt::intern("energyEfficiency"),NULL);
      
      
      
      uint8_t u8_freq = pmt::to_uint64(pmt_freq);
      uint8_t u8_sf = pmt::to_uint64(pmt_sf);
      float ee = pmt::to_float(pmt_ee);
      uint8_t x = u8_freq;
      uint8_t y = u8_sf % 7;
      e[x][y]=ee;
      

    }
    void 
    beaconSend_impl::channel_load_matrixUpdate(pmt::pmt_t msg)
    {      
      pmt::pmt_t pmt_freq = pmt::dict_ref(msg,pmt::intern("logicalFrequency"),NULL);
      pmt::pmt_t pmt_sf = pmt::dict_ref(msg,pmt::intern("logicalSf"),NULL);
      uint8_t u8_freq = pmt::to_uint64(pmt_freq);
      uint8_t u8_sf = pmt::to_uint64(pmt_sf);

      m_outFile.open("load_distribution.txt",std::ios::out | std::ios::app);
      m_outFile<<"u8_freq: "<<std::to_string(u8_freq) << " \t u8_sf:"<<std::to_string(u8_sf)<<std::endl;
      m_outFile.close();
      uint8_t x = u8_freq;
      uint8_t y = u8_sf % 7;
    
      load[x][y]++;
      d_packet_num++;
   
    }
    void
    beaconSend_impl::sendBeacon(pmt::pmt_t msg)
    {
      std::string load_str="";
      std::string energy_str="";
      std::string beacon_str="";
      std::string str_idle="000000000000000000000000000000000000000000000000000000000000";
      int value=0;
      float temp=0;
       //When no packet transmission is detected on the network, an idle string of all 0s is sent.
      if(d_packet_num==0){
                message_port_pub(m_outBeaconPort,pmt::intern(str_idle));
                std::cout<<"none pkt!!"<<std::endl;
      }
      //When a packet transmission is detected in the network, a string carrying load and energy efficiency information is sent.
      else {
              for(uint8_t i=0;i<5;i++){
              for(uint8_t j=0;j<6;j++){
                  temp=load[i][j];
                  
                  temp = temp / d_packet_num;
                  temp *= 10;
                    if(temp == 10){
                        load_str += "*";
                    }
                    else if(temp<1){
                      load_str += "0";
                    }
                    else {
                      value = (int)temp;
                      load_str += std::to_string(value);
                    }

                 }
              }
                //Print payload string content
                std::cout<<"load msg is "<<std::endl<<load_str<<std::endl;
                
              
              for(uint8_t i=0;i<5;i++){
              for(uint8_t j=0;j<6;j++){
                        temp=e[i][j];
                        
                        temp *= 10;
                      if(temp == 10){
                          energy_str += "*";
                      }
                      else if(temp<1){
                        energy_str += "0";
                      }
                      else {
                        value = (int)temp;
                        energy_str += std::to_string(value);
                      }

                   }
                }
                //Print the contents of the energy efficiency string
              std::cout<<"energy msg is "<<std::endl<<energy_str<<std::endl;
              beacon_str=load_str+energy_str;
              message_port_pub(m_outBeaconPort,pmt::intern(beacon_str));
            
      }
    }
    
   

    void
    beaconSend_impl::forecast (int noutput_items, gr_vector_int &ninput_items_required)
    {
      /* <+forecast+> e.g. ninput_items_required[0] = noutput_items */
    }

    int
    beaconSend_impl::general_work (int noutput_items,
                       gr_vector_int &ninput_items,
                       gr_vector_const_void_star &input_items,
                       gr_vector_void_star &output_items)
    {
      const gr_complex *in = (const gr_complex *) input_items[0];
      gr_complex *out = (gr_complex *) output_items[0];

      // Do <+signal processing+>
      // Tell runtime system how many input items we consumed on
      // each input stream.
      consume_each (noutput_items);

      // Tell runtime system how many output items we produced.
      return noutput_items;
    }

  } /* namespace lora */
} /* namespace gr */

