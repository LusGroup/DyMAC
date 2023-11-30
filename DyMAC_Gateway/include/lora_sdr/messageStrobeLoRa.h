#ifndef INCLUDED_LORA_SDR_MESSAGESTROBELORA_H
#define INCLUDED_LORA_SDR_MESSAGESTROBELORA_H

#include <lora_sdr/api.h>
#include <gnuradio/block.h>

namespace gr {
  namespace lora_sdr {

    /*!
     * \brief <+description of block+>
     * \ingroup lora_sdr
     *
     */
    class LORA_SDR_API messageStrobeLoRa : virtual public gr::block
    {
     public:
      typedef boost::shared_ptr<messageStrobeLoRa> sptr;

      /*!
       * \brief Return a shared_ptr to a new instance of lora_sdr::messageStrobeLoRa.
       *
       * To avoid accidental use of raw pointers, lora_sdr::messageStrobeLoRa's
       * constructor is in a private implementation
       * class. lora_sdr::messageStrobeLoRa::make is the public interface for
       * creating new instances.
       */
      static sptr make(pmt::pmt_t msg, long period_ms,long year,long mon,long day,long hour,long min,long sec);
    };

  } // namespace lora_sdr
} // namespace gr

#endif /* INCLUDED_LORA_SDR_MESSAGESTROBELORA_H */

