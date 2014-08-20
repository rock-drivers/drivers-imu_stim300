#ifndef _STIM300_REV_B_HPP_
#define _STIM300_REV_B_HPP_

#include "Stim300Base.hpp"

namespace imu_stim300
{
    class Stim300RevB: public imu_stim300::Stim300Base
    {
        private:
            /** Datagram for the Stim300 Revision D **/
            struct packet
            {
                uint8_t content;
                struct sensor_value gyro_x;
                struct sensor_value gyro_y;
                struct sensor_value gyro_z;
                uint8_t gyros_status;
                struct sensor_value acc_x;
                struct sensor_value acc_y;
                struct sensor_value acc_z;
                uint8_t acc_status;
                struct sensor_value incl_x;
                struct sensor_value incl_y;
                struct sensor_value incl_z;
                uint8_t incl_status;
                int16_t gtemp_x;
                int16_t gtemp_y;
                int16_t gtemp_z;
                uint8_t gtemp_status;
                uint8_t counter;
                uint16_t latency;
                uint32_t checksum;
            } __attribute__ ((__packed__)); //very important to do not align with architecture words

        protected:

	    static const unsigned int NUMBER_PADDING_BYTES = 3; //Number of dummy-bytes to be added for CRC-calculation (see Table 6.15 of the manual)
	    static const unsigned int MAX_PACKET_SIZE = 2 * sizeof(struct packet);

	    uint8_t buffer[MAX_PACKET_SIZE]; /** Buffer with the current package **/
	    packet *currentP; /** Pointer to the current packet (to buffer) **/

        public:

            /** Constructor **/
	    Stim300RevB();

            /** Find a packet into the currently accumulated data.
            */
	    int extractPacket(uint8_t const* buffer, size_t buffer_size) const;

            /** \brief read the package and puts the data into the corresponding structures. *
	     */
	    int processPacket();

        public:

            /**@brief Get sensors information
             */
	     void printInfo();

	protected:

            double convertAcc2Acceleration(uint8_t const* buffer);

            bool verifyChecksum(boost::uint32_t &expintCRC, boost::uint32_t &calintCRC);
    };
}
#endif
