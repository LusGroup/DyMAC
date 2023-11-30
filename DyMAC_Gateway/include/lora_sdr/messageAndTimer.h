#ifndef INCLUDED_LORA_SDR_MESSAGEANDTIMER_H
#define INCLUDED_LORA_SDR_MESSAGEANDTIMER_H

#include <lora_sdr/api.h>
#include <gnuradio/block.h>

namespace gr {
  namespace lora_sdr {

    /*!
     * \brief <+description of block+>
     * \ingroup lora_sdr
     *
     */
    class LORA_SDR_API messageAndTimer : virtual public gr::block
    {
     public:
      typedef boost::shared_ptr<messageAndTimer> sptr;

      /*!
       * \brief Return a shared_ptr to a new instance of lora_sdr::messageAndTimer.
       *
       * To avoid accidental use of raw pointers, lora_sdr::messageAndTimer's
       * constructor is in a private implementation
       * class. lora_sdr::messageAndTimer::make is the public interface for
       * creating new instances.
       */
      static sptr make(std::string filename);
    };

  } // namespace lora_sdr
} // namespace gr

#endif /* INCLUDED_LORA_SDR_MESSAGEANDTIMER_H */

