#ifndef _STIM300_BASE_HPP_
#define _STIM300_BASE_HPP_

#include <iostream>
#include <math.h>
#include <string>
#include <boost/crc.hpp>
#include <boost/math/special_functions/fpclassify.hpp>
#include <Eigen/Geometry> 
#include <iodrivers_base/Driver.hpp>
#include <base/Time.hpp>

namespace imu_stim300
{
    /** STIM300 state machines modes **/
    enum STIM300_MODES{INIT, NORMAL, SERVICE};

    /** STIM300 datagram content **/
    enum DATAGRAM_CONTENT{
        RATE = 0x90,
        RATE_ACC = 0x91,
        RATE_INCLI = 0x92,
        RATE_ACC_INCLI_TEMP_REVB = 0x97,
        RATE_ACC_INCLI_TEMP_REVD = 0xA7
    };

    /** Accelerometers Type output **/
    enum ACC_OUTPUT{ACCELERATION, INCREMENTAL_VELOCITY, AVG_ACCELERATION}; // By default it is ACCELERATION but others are possible(read the manual)

    static const double STIM300_GRAVITY = 9.80665; // Internal definition of the standard gravity in the STIM300

    static const unsigned int DEFAULT_SAMPLING_FREQUENCY = 2000; //Selected by default as frequency in the firmware

    class Stim300Base: public iodrivers_base::Driver
    {
	
	protected:
            struct sensor_value
            {
                signed int value:24;
            } __attribute__ ((__packed__));

            struct temp_value
            {
                signed int value:16;
            } __attribute__ ((__packed__));

            typedef struct
            {
                uint8_t counter;
                double gyro[3];
                double acc[3];
                double incl[3];
                std::vector<double> temp;
                double latency;
                bool checksum;
            }sensor_values;

	    int	baudrate; /** Package baud rate **/
	    int sampling_frequency; /** Sampling Frequency**/
	    int counter_ratio; /** Packet counter ratio **/
        base::Time pckgTimeout; /** Estimate time to have a whole package **/
	    uint8_t prev_counter; /** Counter is incremented by 9 units; Here the previous to the current packge is saved **/
	    sensor_values inertial_values; /** Struct with the processes STIM300 values **/
	    STIM300_MODES modes; /** The three states of the automata of the STIM300 **/
	    bool internal_error; /** If Acc, Gyros or Incl report error it would be true (false by default)**/
            imu_stim300::DATAGRAM_CONTENT content; /** Content of the datagram (package) configuration NOTE: So far only RATE_ACC_INCLI_TEMP is implemented **/
	    ACC_OUTPUT acc_output; /** Type of values returned by the STIM300. By defaults it is Acceleration in m/s^2 **/
            boost::uint32_t expectedCRC, calculatedCRC; /* Expected and calculated checksum value **/

	public:

            /** Constructors **/
	    Stim300Base(int max_packet_size);
	    ~Stim300Base();

            /**
	    * Print a welcome to stdout
	    * \return nothing
	    */
	    void welcome();

	    /** Open the device, reset it and read device information.
	    * the baud rate is 921600 */
	    bool open(std::string const& filename);

	    /** Sets the device baudrate (only possible for serial devices). +brate+ is
	    * the baud rate in bps, can be one of 19200, 57600 and 115200
	    *
	    * If the device is not open yet, this baud rate will be set on startup (i.e.
	    * in open())
	    */
	    bool setBaudrate(int brate);

	    /** Performs a full reset of the device. Sending a RESET command */
	    bool fullReset();

            /** \brief Enter in Service mode (not transferring data, configuration only)
	     */
	    bool enterServiceMode();

            /** Find a packet into the currently accumulated data.
            */
	    virtual int extractPacket(uint8_t const* buffer, size_t buffer_size) const = 0;

	    /** \brief read the package and puts the data into the corresponding structures. *
	     */
	    virtual int processPacket() = 0;

        public:
	    /** \brief Set Accelerometers output to Incremental velocity
	     * Internally enters in SERVICEMODE and 
	     * It always returns to a NORMAL mode after changing the Accelerometers output
	     */
	    bool setAcctoIncrementalVelocity();

        /** \brief Ste the package Timeout
	     *
	     * Expected time to have a whole package
	     */
	    void setPackageTimeout(double timeoutSeconds);

        /** \brief Ste the driver sampling frequency
	     *
	     * Expected frequency (normally connected to timeout)
	     */

        void setFrequency (int sampling_frequency);

        public:
	    /** \brief Return the STIM300 internal status
	     * Return the negation value of the internal_error variable
	     */
	    bool getStatus();

            /** \brief It gets the file descriptor (fd)
	     * 
	     * @return int fd
	     */
	    int getFileDescriptor();

            /**@brief Get Accelerometer Output Mode
             */
	    imu_stim300::ACC_OUTPUT getAccOutputType();

	    /** \brief Return the Incremental counter
	     * Return the value of the value of the counter in inertial_values
	     */
	    int getPacketCounter();

	    /** \brief Return the Accelerometers values
	     */
	    Eigen::Vector3d getAccData ();

	    /** \brief Return the Gyroscopes values
	     */
	    Eigen::Vector3d getGyroData ();

	    /** \brief Return the Inclinometers values
	     */
	    Eigen::Vector3d getInclData ();

	    /** \brief Return the Temperature values from Gyros
	     */
	    std::vector<double> getTempData ();

            /** \brief Return the datagram latency in microseconds (us)
	     * Return the latency calculated by the device.
            * Latency is the time between the moment at which the sample has been
            * digitized + low pass-filtered and the transmitted time.
	    */
	    uint64_t getPacketLatency();

	    /** \brief Return Checksum verification from the last computation
	     */
	    bool getChecksumStatus();

        public:

            /**@brief Get sensors information
             */
	    virtual void printInfo() = 0;

        protected:
            double convertGyro2AngularRate(uint8_t const* buffer);

            virtual double convertAcc2Acceleration(uint8_t const* buffer) = 0;

	    double convertIncl2Acceleration(uint8_t const* buffer);

	    double convertAcc2IncreVel(uint8_t const* buffer);

	    double convertTemp2Celsius(uint8_t const* buffer);

	    double convertLatency2Microseconds(uint8_t const* buffer);

    };
} // end namespace

#endif //
