#ifndef _STIM300_HPP_
#define _STIM300_HPP_

#include <iostream>
#include <math.h>
#include <string>
#include <boost/crc.hpp>
#include <boost/math/special_functions/fpclassify.hpp>
#include <Eigen/Geometry> 
#include <iodrivers_base/Driver.hpp>

namespace stim300
{
    /** General defines **/
    #ifndef OK
    #define OK	0 /** Integer value in order to return when everything is all right. */
    #endif
    
    #ifndef ERROR_OUT
    #define ERROR_OUT -1 /** Integer value in order to return when an error occur. */
    #endif
    
    /** STIM300 state machines modes **/
    enum STIM300_MODES{INIT, NORMAL, SERVICE};
    
    /** STIM300 datagram content **/
    enum DATAGRAM_CONTENT{RATE = 0x90,
		    RATE_ACC = 0x91,
		    RATE_INCLI = 0x92,
		    RATE_ACC_INCLI_TEMP = 0x97};
		    
    /** Accelerometers Type output **/
    enum ACC_OUTPUT{ACCELERATION, INCREMENTAL_VELOCITY, AVG_ACCELERATION}; // By default it is ACCELERATION but others are possible(read the manual)
    

    class STIM300Driver: public iodrivers_base::Driver
    {
	
			
	struct packet
	{
	    uint8_t content;
	    signed gyro_x:24;
	    signed gyro_y:24;
	    signed gyro_z:24;
	    uint8_t gyros_status;
	    signed acc_x:24;
	    signed acc_y:24;
	    signed acc_z:24;
	    uint8_t acc_status;
	    signed incl_x:24;
	    signed incl_y:24;
	    signed incl_z:24;
	    uint8_t incl_status;
	    int16_t temp_x;
	    int16_t temp_y;
	    int16_t temp_z;
	    uint8_t temp_status;
	    uint8_t counter;
	    uint16_t latency;
	} __attribute__ ((__packed__));
	
	struct imu_value
	{
	    signed int value:24;
	} __attribute__ ((__packed__));
	
	struct temp_value
	{
	    signed int value:16;
	} __attribute__ ((__packed__));
	
	typedef struct
	{
	    short int counter;
	    double acc[3];
	    double gyro[3];
	    double incl[3];
	    double temp[3];
	    double latency;
	    
	}sensor_values;
	
	    
	private:
	    static const unsigned int PACKET_SIZE_RATE_ACC_INCLI_TEMP = 45;
	    static const unsigned int MAX_PACKET_SIZE = 2*PACKET_SIZE_RATE_ACC_INCLI_TEMP;
	    static const double STIM300_GRAVITY = 9.80665; // Internal definition of the standard gravity in the STIM300 

	    int	baudrate; /** Packager baud rate **/
	    uint64_t pckgTimeout; /** Estimate time to have a whole package **/
	    uint8_t prev_counter; /** Counter is incremented by 9 units; Here the previous to the current packge is saved **/
	    uint8_t buffer[MAX_PACKET_SIZE]; /** Buffer with the current packeg **/
	    packet *currentP; /** Pointer to the current packet (to buffer) **/
	    sensor_values inertial_values; /** Struct with the processes STIM300 values **/
	    STIM300_MODES modes; /** The three states of the automata of the STIM300 **/
	    DATAGRAM_CONTENT content; /** Content of the datagram (package) configuration NOTE: So far only RATE_ACC_INCLI_TEMP is implemented **/
	    ACC_OUTPUT acc_output; /** Type of values returned by the STIM300. By defaults it is Acceleration in m/s^2 **/
	    bool internal_error; /** If Acc, Gyros or Incl report error it would be true (false by default)**/

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
	    
	    /** Closes the device */
	    bool close();
	    
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
	    
	    /** \brief Return the Temperature values
	     */
	    Eigen::Vector3d getTempData ();
	    
	    /** \brief Return the Temperature value for the X axis
	     */
	    double getTempDataX();
	    
	    /** \brief Return the Temperature value for the Y axis
	     */
	    double getTempDataY();
	    
	    /** \brief Return the Temperature value for the Z axis
	     */
	    double getTempDataZ();
	    
	    
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
	    
    };

} // end namespace

#endif // 
