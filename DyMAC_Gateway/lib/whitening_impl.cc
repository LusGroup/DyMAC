

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include <gnuradio/io_signature.h>
#include "whitening_impl.h"
#include "tables.h"

namespace gr
{
    namespace lora_sdr
    {

        whitening::sptr
        whitening::make()
        {
            return gnuradio::get_initial_sptr(new whitening_impl());
        }

        /*
     * The private constructor
     */
        whitening_impl::whitening_impl()
            : gr::sync_interpolator("whitening",
                             gr::io_signature::make(0, 1, sizeof(uint8_t)),
                             gr::io_signature::make(1, 1, sizeof(uint8_t)),2)
        {
            m_file_source = false;
            message_port_register_in(pmt::mp("msg"));
            set_msg_handler(pmt::mp("msg"), [this](pmt::pmt_t msg) { this->msg_handler(msg); });

            m_paraOutPort = pmt::mp("paraOut");
            message_port_register_out(m_paraOutPort);
            m_realBytes = 0;
            m_start_time = 0;
            m_packetCount = 0;
        }

        /*
     * Our virtual destructor.
     */
        whitening_impl::~whitening_impl()
        {
        }
        
        void whitening_impl::msg_handler(pmt::pmt_t message)
        {
            if(m_file_source){
                std::cout<<RED<<"Whitening don't support both input used simultaneously"<<RESET<<std::endl;
            }
            //  payload_str.push_back(random_string(rand()%253+2));
            // payload_str.push_back(rand()%2?"12345":"abcdefghijklmnop");
            payload_str.push_back(pmt::symbol_to_string(message));
            m_packetCount++;
                        
            // std::copy(payload_str.begin(), payload_str.end(), std::back_inserter(m_payload));
        }

        int whitening_impl::work(int noutput_items,
                                 gr_vector_const_void_star &input_items,
                                 gr_vector_void_star &output_items)
        {
            // std::cout<<"start Whitening"<<std::endl;
            //check if input file
            gettimeofday(&m_time,NULL);
            
            uint8_t *in;
            if(input_items.size()){
                m_file_source = true;
                in = (uint8_t *)input_items[0];
                std::string s;
                for (int i = 0; i < noutput_items/2; i++) //read payload
                {
                    if(in[i]==','){
                        consume_each(1);//consume the ',' character
                        payload_str.push_back(s);
                        break;
                    }
                    s.push_back(in[i]);

                }
            }
                            
                // payload_str.push_back(pmt::symbol_to_string(message));
              
            std::string realData = "";
            if (payload_str.size()>=100 && !(payload_str.size()%100) && !m_file_source)
                std::cout<<RED<<payload_str.size()<<" frames in waiting list. Transmitter has issue to keep up at that transmission frequency."<<RESET<<std::endl;
            if (payload_str.size() && noutput_items >= 2*payload_str.front().length())
            {
                m_realBytes = payload_str.front().length();
                realData = payload_str.front();
                pmt::pmt_t frame_len = pmt::from_long(2*payload_str.front().length());
                add_item_tag(0, nitems_written(0), pmt::string_to_symbol("frame_len"), frame_len);

                add_item_tag(0, nitems_written(0), pmt::string_to_symbol("payload_str"), pmt::string_to_symbol(payload_str.front()));

                uint8_t *out = (uint8_t *)output_items[0];

                std::copy(payload_str.front().begin(), payload_str.front().end(), std::back_inserter(m_payload));;

                for (uint i = 0; i < m_payload.size(); i++)
                {
                    out[2 * i] = (m_payload[i] ^ whitening_seq[i]) & 0x0F;
                    out[2 * i + 1] = (m_payload[i] ^ whitening_seq[i]) >> 4;
                }

                noutput_items = 2 * m_payload.size();
                m_payload.clear();
                payload_str.erase(payload_str.begin());
            }
            else
                noutput_items = 0;
            if(noutput_items != 0){
                m_start_time = m_time.tv_sec * 1000 + m_time.tv_usec/1000;
             
               pmt::pmt_t dict = pmt::make_dict();
               dict =  pmt::dict_add(dict,pmt::intern("startTime"),pmt::from_double(m_start_time));
               dict = pmt::dict_add(dict,pmt::intern("realBytes"),pmt::from_double(m_realBytes));
               dict = pmt::dict_add(dict,pmt::intern("PacketCount"),pmt::from_uint64(m_packetCount));
               dict =  pmt::dict_add(dict,pmt::intern("realData"),pmt::intern(realData));
                message_port_pub(m_paraOutPort,dict);
            }
            // std::cout<<"ending whitening"<<std::endl;
            

            
            return noutput_items;
        }

    } /* namespace lora */
} /* namespace gr */
