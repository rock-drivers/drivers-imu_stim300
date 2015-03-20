#include "Stim300Base.hpp"

using namespace imu_stim300;

Stim300Base::Stim300Base(int max_packet_size)
:Driver(max_packet_size)
,baudrate(iodrivers_base::Driver::SERIAL_921600)
,sampling_frequency(imu_stim300::DEFAULT_SAMPLING_FREQUENCY)
,modes(NORMAL)
,internal_error(false)
,acc_output(ACCELERATION)
{
}

Stim300Base::~Stim300Base()
{
    	if (isValid())
        close();
}

void Stim300Base::welcome()
{
	std::cout << "You successfully compiled and executed STIM300. Welcome!" << std::endl;
}

bool Stim300Base::open(const std::string& filename)
{
    if (! Driver::openSerial(filename, this->baudrate))
        return false;
    return true;
}

bool Stim300Base::setBaudrate(int brate)
{
    this->baudrate = brate;

    return true;
}

bool Stim300Base::fullReset()
{
    std::cout<<"IN FULL RESET\n";
    if (this->modes == NORMAL)
    {
	uint8_t reset[2];
	reset[0] = 'R';
	reset[1] = '\r';

	std::cout<<"SEND R COMMAND IN NORMAL MODE\n";
	Driver::writePacket(reset, sizeof(reset), 0);
	
	return true;
    }
    else if (this->modes == SERVICE)
    {
	uint8_t reset[4];
	reset[0] = 'x';
	reset[1] = ' ';
	reset[2] = 'n';
	reset[3] = '\r';
	uint8_t* ptr_cmd = (uint8_t*) &reset;

	std::cout<<"SEND x n COMMAND IN SERVICE MODE (RETURN TO NORMAL MODE)\n";
	Driver::writePacket(ptr_cmd, sizeof(reset), 0);
	this->modes = NORMAL;
	
	return true;
    }
    return false;
}

bool Stim300Base::enterServiceMode()
{
    if (this->modes == NORMAL)
    {
	uint8_t command[12];
	command[0] = 'S';
	command[1] = 'E';
	command[2] = 'R';
	command[3] = 'V';
	command[4] = 'I';
	command[5] = 'C';
	command[6] = 'E';
	command[7] = 'M';
	command[8] = 'O';
	command[9] = 'D';
	command[10] = 'E';
	command[11] = '\r';

	std::cout<<"ENTERING IN SERVICEMODE\r";
	Driver::writePacket(command, sizeof(command), 0);
	usleep (1);
	this->modes = SERVICE;
	
	return true;
    }

    return false;

}

bool Stim300Base::setAcctoIncrementalVelocity()
{

    if (this->modes == NORMAL)
    {
	this->enterServiceMode();
	
    }

    if (this->modes == SERVICE)
    {
	uint8_t command[6];
	command[0] = 'u';
	command[1] = ' ';
	command[2] = 'a';
	command[3] = ',';
	command[4] = '1';
	command[5] = '\r';
// 	uint8_t* ptr_cmd = (uint8_t*) &command;

	std::cout<<"SETTING ACC-INCREMENTAL VELOCITY\n";
	Driver::writePacket(command, sizeof(command), 0);
	sleep (1);
	this->processPacket();
	usleep (80000);
	
	
	
	std::cout<<"EXIT FROM SERVICE MODE\n";
	uint8_t exit[4];
	exit[0] = 'x';
	exit[1] = ' ';
	exit[2] = 'n';
	exit[3] = '\r';
	Driver::writePacket(exit, sizeof(exit), 0);
	sleep (1);
	this->processPacket();
	usleep (80000);
	
	
	uint8_t confirmation[2];
	confirmation[0] = 'Y';
	confirmation[1] = '\r';
	Driver::writePacket(confirmation, sizeof(confirmation), 0);
	sleep (5);
	this->acc_output = INCREMENTAL_VELOCITY;
	
	
	this->modes = NORMAL;
	
	return true;
    }

    return false;

}

void Stim300Base::setPackageTimeout(double timeoutSeconds)
{
    this->pckgTimeout = base::Time::fromSeconds(timeoutSeconds);
}

void Stim300Base::setFrequency (int sampling_frequency)
{
    this->sampling_frequency = sampling_frequency;
    this->counter_ratio = imu_stim300::DEFAULT_SAMPLING_FREQUENCY / this->sampling_frequency;
    this->counter_ratio--;
}

bool Stim300Base::getStatus()
{
    return ~internal_error;
}

int Stim300Base::getFileDescriptor()
{
    return Driver::getFileDescriptor();
}

ACC_OUTPUT Stim300Base::getAccOutputType()
{
    return this->acc_output;
}

int Stim300Base::getPacketCounter()
{
    return (int) this->inertial_values.counter;
}

Eigen::Vector3d Stim300Base::getAccData ()
{
    return Eigen::Vector3d (this->inertial_values.acc[0], this->inertial_values.acc[1], this->inertial_values.acc[2]);
}

Eigen::Vector3d Stim300Base::getGyroData()
{
    return Eigen::Vector3d (this->inertial_values.gyro[0], this->inertial_values.gyro[1], this->inertial_values.gyro[2]);
}

Eigen::Vector3d Stim300Base::getInclData()
{
    return Eigen::Vector3d (this->inertial_values.incl[0], this->inertial_values.incl[1], this->inertial_values.incl[2]);
}

std::vector<double> Stim300Base::getTempData()
{
    return this->inertial_values.temp;
}

uint64_t Stim300Base::getPacketLatency()
{
    return static_cast<uint64_t>(this->inertial_values.latency);
}

bool Stim300Base::getChecksumStatus()
{
    return this->inertial_values.checksum;
}

double Stim300Base::convertGyro2AngularRate(const uint8_t* buffer)
{
    double gyro_double = std::numeric_limits<double>::quiet_NaN();
    uint8_t copy_buffer[3];
    double twopower14 = 16384;//2^14
    struct sensor_value *gyro_value;


    if (buffer != NULL)
    {
	copy_buffer[2] = buffer[0];
	copy_buffer[1] = buffer[1];
	copy_buffer[0] = buffer[2];
	
// 	printf ("Converting %X %X %X\n", buffer[0], buffer[1], buffer[2]);
	
	gyro_value = reinterpret_cast<struct sensor_value *>(copy_buffer);
	
	gyro_double = (double)(gyro_value->value) / twopower14;
// 	std::cout<<"gyro_double: "<<gyro_double<<"\n";
	
    }

    return gyro_double * (M_PI/180.00);//units are rad/second
}

double Stim300Base::convertIncl2Acceleration(const uint8_t* buffer)
{
    double incl_double = std::numeric_limits<double>::quiet_NaN();
    uint8_t copy_buffer[3];
    double twopower22 = 4194304;//2^22
    struct sensor_value *incl_value;

    if (buffer != NULL)
    {
	
//  	printf ("Converting %X %X %X\n", buffer[0], buffer[1], buffer[2]);
	
	copy_buffer[2] = buffer[0];
	copy_buffer[1] = buffer[1];
	copy_buffer[0] = buffer[2];
	incl_value = reinterpret_cast<struct sensor_value *>(copy_buffer);
	
	incl_double = (double)(incl_value->value) / twopower22;
// 	std::cout<<"acc_double: "<<acc_double<<"\n";

    }

    return incl_double * STIM300_GRAVITY;//units are in m/s^2
}

double Stim300Base::convertAcc2IncreVel(const uint8_t* buffer)
{
    double acc_double = std::numeric_limits<double>::quiet_NaN();
    uint8_t copy_buffer[3];
    double twopower22 = 4194304;//2^22
    struct sensor_value *acc_value;

    if (buffer != NULL)
    {
	
//  	printf ("Converting %X %X %X\n", buffer[0], buffer[1], buffer[2]);
	
	copy_buffer[2] = buffer[0];
	copy_buffer[1] = buffer[1];
	copy_buffer[0] = buffer[2];
	acc_value = reinterpret_cast<struct sensor_value *>(copy_buffer);
	
	acc_double = (double)(acc_value->value) / twopower22;
// 	std::cout<<"acc_double: "<<acc_double<<"\n";
	    
    }

    return acc_double; //units are m/s/sample
}

double Stim300Base::convertTemp2Celsius(const uint8_t* buffer)
{
    double temp_double = std::numeric_limits<double>::quiet_NaN();
    uint8_t copy_buffer[2];
    struct temp_value *temp_value;

    if (buffer != NULL)
    {
	
//  	printf ("Converting %X %X %X\n", buffer[0], buffer[1], buffer[2]);
	copy_buffer[1] = buffer[0];
	copy_buffer[0] = buffer[1];
	temp_value = reinterpret_cast<struct temp_value *>(copy_buffer);
	
	temp_double = (double)(temp_value->value) / 256.00;
// 	std::cout<<"temp_double: "<<temp_double<<"\n";
    }

    return temp_double; //units are celsius degrees
}

double Stim300Base::convertLatency2Microseconds(const uint8_t* buffer)
{
    double latency_double = std::numeric_limits<double>::quiet_NaN();

    if (buffer != NULL)
    {
	latency_double = buffer[0] * 256.00 + buffer[1];
    }

    return latency_double;// units are microseconds

}

