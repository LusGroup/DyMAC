#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include <gnuradio/io_signature.h>
#include "messageAndTimer_impl.h"

namespace gr {
  namespace lora_sdr {

    messageAndTimer::sptr
    messageAndTimer::make(std::string filename)
    {
      return gnuradio::get_initial_sptr
        (new messageAndTimer_impl(filename));
    }


    /*
     * The private constructor
     */
    messageAndTimer_impl::messageAndTimer_impl(std::string filename)
      : gr::block("messageAndTimer",
              gr::io_signature::make(0, 0,0),
              gr::io_signature::make(0, 0, 0))
    {
      count = 0;
      m_filename = filename;
      recordString = "";
      out_file.open(m_filename,std::ios::out | std::ios::trunc);
      m_inMesPort = pmt::intern("mesPort");
      m_inTimePort = pmt::intern("TimePort");
      message_port_register_in(m_inMesPort);
      set_msg_handler(m_inMesPort,boost::bind(&messageAndTimer_impl::recordMsg,this,_1));
      message_port_register_in(m_inTimePort);
      set_msg_handler(m_inTimePort,boost::bind(&messageAndTimer_impl::recordTime,this,_1));
      recordString.append("PacketCount\tresultMsg\tTime\n");
    }

    /*
     * Our virtual destructor.
     */
    messageAndTimer_impl::~messageAndTimer_impl()
    {
      out_file<<recordString<<std::endl;
      out_file.close();
    }
    void
    messageAndTimer_impl::recordMsg(pmt::pmt_t msg){
      resultMsg = pmt::symbol_to_string(msg);
      if(resultMsg != "" && timeMsg != ""){
        count++;
        recordString.append(std::to_string(count)+"\t"+resultMsg+"\t"+timeMsg);//Perform string concatenation
        resultMsg = "";
        timeMsg = "";
      }
    }
    void
    messageAndTimer_impl::recordTime(pmt::pmt_t msg){
      timeMsg = pmt::symbol_to_string(msg);
      if(resultMsg != "" && timeMsg != ""){
          count++;
          recordString.append(std::to_string(count)+"\t"+resultMsg+"\t"+timeMsg);
          resultMsg = "";
          timeMsg = "";
        }
    }
    void
    messageAndTimer_impl::forecast (int noutput_items, gr_vector_int &ninput_items_required)
    {
      /* <+forecast+> e.g. ninput_items_required[0] = noutput_items */
    }

    int
    messageAndTimer_impl::general_work (int noutput_items,
                       gr_vector_int &ninput_items,
                       gr_vector_const_void_star &input_items,
                       gr_vector_void_star &output_items)
    {
      const gr_complex *in = (const gr_complex *) input_items[0];
      gr_complex *out = (gr_complex*) output_items[0];

      // Do <+signal processing+>
      // Tell runtime system how many input items we consumed on
      // each input stream.
      consume_each (noutput_items);

      // Tell runtime system how many output items we produced.
      return noutput_items;
    }

  } /* namespace lora_sdr */
} /* namespace gr */

