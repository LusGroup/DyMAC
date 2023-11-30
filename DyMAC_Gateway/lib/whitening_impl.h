

#ifndef INCLUDED_LORA_WHITENING_IMPL_H
#define INCLUDED_LORA_WHITENING_IMPL_H


#include <lora_sdr/whitening.h>
#include <lora_sdr/utilities.h>
#include <sys/time.h>
#include <unistd.h>
namespace gr {
  namespace lora_sdr {

    class whitening_impl : public whitening
    {
     private:
         std::vector<uint8_t> m_payload; ///< store the payload bytes
         std::vector<std::string> payload_str;
         bool m_file_source; ///< indicate that the payload are provided by a file through an input stream
         void msg_handler(pmt::pmt_t message);
         pmt::pmt_t m_paraOutPort;
         double m_start_time;
         double m_realBytes;
         struct timeval m_time;
         uint32_t m_packetCount;
     public:
      whitening_impl();
      ~whitening_impl();

      // Where all the action really happens
      int work(
              int noutput_items,
              gr_vector_const_void_star &input_items,
              gr_vector_void_star &output_items
      );
    };
  } // namespace lora
} // namespace gr

#endif /* INCLUDED_LORA_WHITENING_IMPL_H */
