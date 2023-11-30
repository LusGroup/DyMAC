#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include <gnuradio/io_signature.h>
#include "gray_decode_impl.h"
#include <lora_sdr/utilities.h>

namespace gr {
  namespace lora_sdr {

    gray_decode::sptr
    gray_decode::make(uint8_t sf)
    {
      return gnuradio::get_initial_sptr
        (new gray_decode_impl(sf));
    }
    /*
     * The private constructor
     */
    gray_decode_impl::gray_decode_impl(uint8_t sf)
      : gr::sync_block("gray_decode",
              gr::io_signature::make(1, 1, sizeof(uint32_t)),
              gr::io_signature::make(1, 1, sizeof(uint32_t)))
    {
        m_sf=sf;
        set_tag_propagation_policy(TPP_ONE_TO_ONE);
        m_inSFPort = pmt::mp("inSF");
        message_port_register_in(m_inSFPort);
        set_msg_handler(m_inSFPort,boost::bind(&gray_decode_impl::setSF,this,_1));
    }

    /*
     * Our virtual destructor.
     */
    gray_decode_impl::~gray_decode_impl()
    {}
    
    void
    gray_decode_impl::setSF(pmt::pmt_t msg){
      pmt::pmt_t key = pmt::intern("sf");
      pmt::pmt_t sfNum = pmt::dict_ref(msg,key,pmt::PMT_NIL);
      uint32_t sf = pmt::to_uint64(sfNum);
      m_sf = sf;
    }

    int
    gray_decode_impl::work(int noutput_items,
        gr_vector_const_void_star &input_items,
        gr_vector_void_star &output_items)
    {
      const uint32_t *in = (const uint32_t *) input_items[0];
      uint32_t *out = (uint32_t *) output_items[0];
      for(int i=0;i<noutput_items;i++){
        #ifdef GRLORA_DEBUG
        std::cout<<std::hex<<"0x"<<in[i]<<" -->  ";
        #endif
        out[i]=in[i];
        for(int j=1;j<m_sf;j++){
             out[i]=out[i]^(in[i]>>j);
        }
        //do the shift of 1
         out[i]=mod(out[i]+1,(1<<m_sf));
         #ifdef GRLORA_DEBUG
         std::cout<<"0x"<<out[i]<<std::dec<<std::endl;
         #endif
      }

      return noutput_items;
    }
  } /* namespace lora */
} /* namespace gr */
