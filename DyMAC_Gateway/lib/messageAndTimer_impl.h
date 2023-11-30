#ifndef INCLUDED_LORA_SDR_MESSAGEANDTIMER_IMPL_H
#define INCLUDED_LORA_SDR_MESSAGEANDTIMER_IMPL_H

#include <lora_sdr/messageAndTimer.h>
#include <string>
#include <iostream>

namespace gr {
  namespace lora_sdr {

    class messageAndTimer_impl : public messageAndTimer
    {
     private:
      // Nothing to declare in this block.
      std::string m_filename;
      std::ofstream out_file;
      int count;
      pmt::pmt_t m_inMesPort,m_inTimePort;
      void recordMsg(pmt::pmt_t msg);
      void recordTime(pmt::pmt_t msg);
      std::string resultMsg,timeMsg;
      std::string recordString;
     public:
      messageAndTimer_impl(std::string filename);
      ~messageAndTimer_impl();

      // Where all the action really happens
      void forecast (int noutput_items, gr_vector_int &ninput_items_required);

      int general_work(int noutput_items,
           gr_vector_int &ninput_items,
           gr_vector_const_void_star &input_items,
           gr_vector_void_star &output_items);

    };

  } // namespace lora_sdr
} // namespace gr

#endif /* INCLUDED_LORA_SDR_MESSAGEANDTIMER_IMPL_H */

