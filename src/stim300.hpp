#ifndef _STIM300_HPP_
#define _STIM300_HPP_

#include <iostream>
#include <math.h>
#include <string>
#include <boost/crc.hpp>
#include <boost/math/special_functions/fpclassify.hpp>
#include <Eigen/Geometry> 
#include <iodrivers_base/Driver.hpp>
#include <base/Time.hpp>

#ifndef STIM300_REV
    #define STIM300_REV 'D'
#endif

namespace stim300
{
    /** STIM300 state machines modes **/
    enum STIM300_MODES{INIT, NORMAL, SERVICE};

    /** STIM300 datagram content **/
    enum DATAGRAM_CONTENT{
        RATE = 0x90,
        RATE_ACC = 0x91,
        RATE_INCLI = 0x92,
#if STIM300_REV<='C'
        RATE_ACC_INCLI_TEMP = 0x97
#else
        RATE_ACC_INCLI_TEMP = 0xA7
#endif
    };

    /** Accelerometers Type output **/
    enum ACC_OUTPUT{ACCELERATION, INCREMENTAL_VELOCITY, AVG_ACCELERATION}; // By default it is ACCELERATION but others are possible(read the manual)

    class STIM300Driver: public iodrivers_base::Driver
    {
		
	struct imu_value
	{
	    signed int value:24;
	} __attribute__ ((__packed__));
	
	struct temp_value
	{
	    signed int value:16;
	} __attribute__ ((__packed__));
	
        struct packet
	{
	    uint8_t content;
	    struct imu_value gyro_x;
	    struct imu_value gyro_y;
	    struct imu_value gyro_z;
	    uint8_t gyros_status;
	    struct imu_value acc_x;
	    struct imu_value acc_y;
	    struct imu_value acc_z;
	    uint8_t acc_status;
	    struct imu_value incl_x;
	    struct imu_value incl_y;
	    struct imu_value incl_z;
	    uint8_t incl_status;
	    int16_t gtemp_x;
	    int16_t gtemp_y;
	    int16_t gtemp_z;
            uint8_t gtemp_status;
#if STIM300_REV > 'C'
            int16_t atemp_x;
	    int16_t atemp_y;
	    int16_t atemp_z;
            uint8_t atemp_status;
            int16_t itemp_x;
	    int16_t itemp_y;
	    int16_t itemp_z;
            uint8_t itemp_status;
#endif
            uint8_t counter;
	    uint16_t latency;
	    uint32_t checksum;
	} __attribute__ ((__packed__));

	typedef struct
	{
	    uint8_t counter;
	    double gyro[3];
	    double acc[3];
	    double incl[3];
	    double gtemp[3];
	    double atemp[3];
	    double itemp[3];
	    double latency;
            bool checksum;
	}sensor_values;
	
	private:
#if STIM300_REV < 'C'
	    static const unsigned int PACKET_SIZE_RATE_ACC_INCLI_TEMP_CRC = 45;// Size until CRC (AUX is not in the datagram)
	    static const unsigned int NUMBER_DUMMY_BYTES = 3; //Number of dummy-bytes to be added for CRC-calculation (see Table 6.15 of the manual)
#else
	    static const unsigned int PACKET_SIZE_RATE_ACC_INCLI_TEMP_CRC = 59;// Size until CRC (AUX is not in the datagram)
	    static const unsigned int NUMBER_DUMMY_BYTES = 1; //Number of dummy-bytes to be added for CRC-calculation (see Table 6.15 of the manual)
#endif
	    static const unsigned int MAX_PACKET_SIZE = 2*PACKET_SIZE_RATE_ACC_INCLI_TEMP_CRC;
	    static const double STIM300_GRAVITY = 9.80665; // Internal definition of the standard gravity in the STIM300 


	    int	baudrate; /** Package baud rate **/
            base::Time pckgTimeout; /** Estimate time to have a whole package **/
	    uint8_t prev_counter; /** Counter is incremented by 9 units; Here the previous to the current packge is saved **/
	    uint8_t buffer[MAX_PACKET_SIZE]; /** Buffer with the current package **/
	    packet *currentP; /** Pointer to the current packet (to buffer) **/
	    sensor_values inertial_values; /** Struct with the processes STIM300 values **/
	    STIM300_MODES modes; /** The three states of the automata of the STIM300 **/
	    bool internal_error; /** If Acc, Gyros or Incl report error it would be true (false by default)**/
	    DATAGRAM_CONTENT content; /** Content of the datagram (package) configuration NOTE: So far only RATE_ACC_INCLI_TEMP is implemented **/
	    ACC_OUTPUT acc_output; /** Type of values returned by the STIM300. By defaults it is Acceleration in m/s^2 **/

	public:

	    static const unsigned int DEFAULT_SAMPLING_FREQUENCY = 125;

	    STIM300Driver();
	    ~STIM300Driver();


	    void getInfo();


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

	    /** \brief It gets the file descriptor (fd)
	     * 
	     * @return int fd
	     */
	    int getFileDescriptor();

	    /** \brief Ste the package Timeout
	     * 
	     * Expected time to have a whole package
	     * 
	     */
	    void setPackageTimeout(uint64_t timeoutMilliSeconds);

	    /** \brief read the package and and puts the data into the corresponding structures. *
	     */
	    int processPacket();

	    /** \brief Enter in Service mode (not transfering data, configuration only)
	     */
	    bool enterServiceMode();

	    /** \brief Set Accelerometers output to Incremental velocity
	     * Internally enters in SERVICEMODE and 
	     * It always returns to a NORMAL mode after changing the Accelerometers output
	     */
	    bool setAcctoIncrementalVelocity();

	    /** \brief Return the STIM300 internal status
	     * Return the negation value of the internal_error variable
	     */
	    bool getStatus();

	    stim300::ACC_OUTPUT getAccOutputType();

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
	    Eigen::Vector3d getGyroTempData ();

#if STIM300_REV > 'C'
            /** \brief Return the Temperature values from Acc
	     */
	    Eigen::Vector3d getAccTempData ();

            /** \brief Return the Temperature values from Incl
	     */
	    Eigen::Vector3d getInclTempData ();
#endif

	    /** \brief Return the Temperature value for the X axis
	     */
	    double getGyroTempDataX();

	    /** \brief Return the Temperature value for the Y axis
	     */
	    double getGyroTempDataY();

	    /** \brief Return the Temperature value for the Z axis
	     */
	    double getGyroTempDataZ();

            /** \brief Return the datagram latency in microseconds (us)
	     * Return the latency calculated by the device.
            * Latency is the time between the moment at which the sample has been
            * digitized + low pass-filtered and the transmitted time.
	    */
	    uint64_t getPacketLatency();

	    /**
	    * Print a welcome to stdout
	    * \return nothing
	    */
	    void welcome();

	protected:
	    int extractPacket(uint8_t const* buffer, size_t buffer_size) const;

	    double convertGyro2AngularRate(uint8_t const* buffer);

	    double convertAcc2Acceleration(uint8_t const* buffer);

	    double convertIncl2Acceleration(uint8_t const* buffer);

	    double convertAcc2IncreVel(uint8_t const* buffer);

	    double convertTemp2Celsius(uint8_t const* buffer);

	    double convertLatency2Microseconds(uint8_t const* buffer);

            bool verifyChecksum();
    };

} // end namespace

#endif //
