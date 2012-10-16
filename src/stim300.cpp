#include "stim300.hpp"

namespace stim300
{



STIM300Driver::STIM300Driver()
:Driver(MAX_PACKET_SIZE)
,baudrate(921600)
,modes(NORMAL)
,internal_error(false)
,content(RATE_ACC_INCLI_TEMP)
,acc_output(ACCELERATION)
{

    this->currentP = reinterpret_cast<struct packet *>(this->buffer);
    
    this->prev_counter = std::numeric_limits<double>::quiet_NaN();
    
    this->inertial_values.counter = 0;
    this->inertial_values.acc[0] = std::numeric_limits<double>::quiet_NaN();
    this->inertial_values.acc[1] = std::numeric_limits<double>::quiet_NaN();
    this->inertial_values.acc[2] = std::numeric_limits<double>::quiet_NaN();
    
    this->inertial_values.gyro[0] = std::numeric_limits<double>::quiet_NaN();
    this->inertial_values.gyro[1] = std::numeric_limits<double>::quiet_NaN();
    this->inertial_values.gyro[2] = std::numeric_limits<double>::quiet_NaN();
    
    this->inertial_values.incl[0] = std::numeric_limits<double>::quiet_NaN();
    this->inertial_values.incl[1] = std::numeric_limits<double>::quiet_NaN();
    this->inertial_values.incl[2] = std::numeric_limits<double>::quiet_NaN();
    
    this->inertial_values.temp[0] = std::numeric_limits<double>::quiet_NaN();
    this->inertial_values.temp[1] = std::numeric_limits<double>::quiet_NaN();
    this->inertial_values.temp[2] = std::numeric_limits<double>::quiet_NaN();
    
    this->inertial_values.latency = std::numeric_limits<double>::quiet_NaN();
    
}

void STIM300Driver::getInfo()
{
    std::cout<<"** STIM300 Serial Driver Infor **\n";
    std::cout<<"Baudrate: "<<this->baudrate <<"\n";
    std::cout<<"Datagram content: "<<this->content <<"\n";
    std::cout<<"Current mode (Automata): "<<this->modes <<"\n";
    printf ("Datagram counter: %X Package counter: %d (%X)\n", this->currentP->counter, this->inertial_values.counter, this->inertial_values.counter);
    printf ("Gyros Status: %X\n", this->currentP->gyros_status);
    std::cout<<"Gyros[rad/s]: "<<this->inertial_values.gyro[0]<<" "<<this->inertial_values.gyro[1]<<" "<<this->inertial_values.gyro[2]<<"\n";
    printf ("Acc Status: %X\n", this->currentP->acc_status);
    std::cout<<"Acc[m/s^2]: "<<this->inertial_values.acc[0]<<" "<<this->inertial_values.acc[1]<<" "<<this->inertial_values.acc[2]<<"\n";
    printf ("Incl Status: %X\n", this->currentP->incl_status);
    std::cout<<"Incl[m/s^2]: "<<this->inertial_values.incl[0]<<" "<<this->inertial_values.incl[1]<<" "<<this->inertial_values.incl[2]<<"\n";
    std::cout<<"Temperature[Celsius]: "<<this->inertial_values.temp[0]<<" "<<this->inertial_values.temp[1]<<" "<<this->inertial_values.temp[2]<<"\n";
    std::cout<<"Latency[microsecond]: "<<this->inertial_values.latency<<"\n";
}


bool STIM300Driver::getStatus()
{
    return ~internal_error;
}



STIM300Driver::~STIM300Driver()
{
    	if (isValid())
        close();

}

void STIM300Driver::welcome()
{
	std::cout << "You successfully compiled and executed STIM300. Welcome!" << std::endl;
}

bool STIM300Driver::open(const std::string& filename)
{
    if (! Driver::openSerial(filename, this->baudrate))
        return false;
    return true;
}

bool STIM300Driver::setBaudrate(int brate)
{

    this->baudrate = brate;
    
    return OK;
}


bool STIM300Driver::close()
{
    Driver::close();
    return true;
}

int STIM300Driver::getFileDescriptor()
{
    return Driver::getFileDescriptor();
}


int STIM300Driver::extractPacket(const uint8_t* buffer, size_t buffer_size) const
{
//     boost::crc_32_type crc_32;
// 
//     if(buffer_size < 1)
// 	return 0;
//     
//     int cnt = 0;
//     while(buffer[cnt] != 0x97 && cnt < buffer_size)
// 	cnt++;
//     
//     if(cnt != 0)
// 	return -cnt;
//     
//     
//     if(buffer_size < PACKET_SIZE_RATE_ACC_INCLI_TEMP)
// 	return 0;
// 
//     /** CRC-32 **/
//      std::cout<<"Checksum\n";
//      crc_32.process_bytes(buffer, 41);		    
//      printf("Checksum: %X \n", crc_32.checksum());
//     
    int packet_state = 0; //0-> no synchronized, 1->Semi synchonized, 2-> package found, 3-> package found and CRC-Ok
    unsigned short start_position = 0;
    boost::crc_32_type crc_32;
    
//     std::cout<<"**** extractPacket ****\n";
//     std::cout<<"buffer_size: "<< buffer_size <<"\n";
//     for ( int i=0; i < (int)buffer_size; i++)
// 	printf("%X \n", buffer[i]);
//    
   if (this->modes == NORMAL)
   {
    
    for (unsigned int i=0; i < (unsigned int)buffer_size; i++)
    {
	switch(packet_state)
	{
	    case 0: //found start of the packet
		if (buffer[i] == this->content)
		{
		    start_position = i;
		    packet_state = 1;
		}
		break;
		
	    case 1: //verify there is another start of the packet afterwards
		if (buffer[i] == this->content)
		{
		    if (i == start_position + PACKET_SIZE_RATE_ACC_INCLI_TEMP)
		    {
			packet_state = 2;
		    }
		}
		else if (i == (start_position+(PACKET_SIZE_RATE_ACC_INCLI_TEMP-1))) //end of the CRC
		{
// 		    std::cout<<"Checksum\n";
		    crc_32.process_bytes((buffer+start_position), 41);		    
// 		    printf("Checksum: %X \n", crc_32.checksum());
		    packet_state = 2;
		}
		break;
		
	    default:
		
		break;
		
	}
//  	printf("%X \n", buffer[i]);
    }
    

   /* std::cout<<"packet_state: "<<packet_state<<"\n";
    std::cout<<"**** end extractPacket ****\n";
   */ 
    
    if (packet_state == 0)
	return -buffer_size;
    else if (packet_state == 1)
    {
	return -start_position;
    }
    else
    {
	if (start_position > 0)
	    return -start_position;
	else
	    return PACKET_SIZE_RATE_ACC_INCLI_TEMP;
    }
    
   }
   else //if (this->modes == SERVICE)
   {
       return PACKET_SIZE_RATE_ACC_INCLI_TEMP;
   }
   
   
	
	
}

int STIM300Driver::processPacket()
{
    int buf_size = 0;
//     std::cout<<"processPacket\n";

    try {
        buf_size = Driver::readPacket (this->buffer, MAX_PACKET_SIZE, 0);
    } catch (iodrivers_base::TimeoutError& e ) { 
	std::cerr<<"TimeoutError\n";
	return ERROR;
    }
    
//     std::cout<<"buf_size: "<<buf_size<<"\n";
//      for (int i =0; i<buf_size; i++)
//  	    printf("%X \n", buffer[i]);
        
    if (this->modes == NORMAL)
    {
// 	    if ((this->currentP->acc_status + this->currentP->gyros_status + this->currentP->incl_status) == 0)
// 	    {
// 		
		this->internal_error = false;
		
		/** If everything is alright convert the values **/
		if (prev_counter == std::numeric_limits<double>::quiet_NaN())
		    prev_counter = this->currentP->counter;
		else
		{
		    this->inertial_values.counter += (this->currentP->counter - prev_counter)% 0x07;
		    prev_counter = this->currentP->counter;
		}
		
		
		this->inertial_values.gyro[0] = convertGyro2AngularRate((this->buffer+1));
		this->inertial_values.gyro[1] = convertGyro2AngularRate((this->buffer+4));
		this->inertial_values.gyro[2] = convertGyro2AngularRate((this->buffer+7));
		
		if (acc_output == ACCELERATION)
		{
		    this->inertial_values.acc[0] = convertAcc2Acceleration((buffer+11));
		    this->inertial_values.acc[1] = convertAcc2Acceleration((buffer+14));
		    this->inertial_values.acc[2] = convertAcc2Acceleration((buffer+17));
		}
		else if (acc_output == INCREMENTAL_VELOCITY)
		{  
		    this->inertial_values.acc[0] = convertAcc2IncreVel((buffer+11));
		    this->inertial_values.acc[1] = convertAcc2IncreVel((buffer+14));
		    this->inertial_values.acc[2] = convertAcc2IncreVel((buffer+17));
		}
		
		this->inertial_values.incl[0] = convertIncl2Acceleration((buffer+21));
		this->inertial_values.incl[1] = convertIncl2Acceleration((buffer+24));
		this->inertial_values.incl[2] = convertIncl2Acceleration((buffer+27));
		
		this->inertial_values.temp[0] = convertTemp2Celsius((buffer+31));
		this->inertial_values.temp[1] = convertTemp2Celsius((buffer+33));
		this->inertial_values.temp[2] = convertTemp2Celsius((buffer+35));
		
		this->inertial_values.latency = convertLatency2Microseconds((buffer+39));
		
		return OK;
// 	    }
// 	    else
// 	    {
// 		std::cout<<"ERROR IN INTERNAL DATA\n";
// 		this->internal_error = true;
// 		this->fullReset();
// 		
// 	    }
    }
    else if (this->modes == SERVICE)
    {
	std::string s(this->buffer, this->buffer + sizeof(this->buffer));
	std::cout<<s<<"\n";
	return OK;
	
    }
    
    
    return ERROR;

}

double STIM300Driver::convertGyro2AngularRate(const uint8_t* buffer)
{
    double gyro_double = std::numeric_limits<double>::quiet_NaN();
    uint8_t copy_buffer[3];
    double twopower14 = 16384;//2^14
    struct imu_value *gyro_value;
    
    
    if (buffer != NULL)
    {
	copy_buffer[2] = buffer[0];
	copy_buffer[1] = buffer[1];
	copy_buffer[0] = buffer[2];
	
// 	printf ("Converting %X %X %X\n", buffer[0], buffer[1], buffer[2]);
	
	gyro_value = reinterpret_cast<struct imu_value *>(copy_buffer);
	
	gyro_double = (double)(gyro_value->value) / twopower14;
// 	std::cout<<"gyro_double: "<<gyro_double<<"\n";
	
    }
    
    return gyro_double * (M_PI/180.00);//units are rad/second
}

double STIM300Driver::convertAcc2Acceleration(const uint8_t* buffer)
{
    double acc_double = std::numeric_limits<double>::quiet_NaN();
    uint8_t copy_buffer[3];
    double twopower19 = 524288;//2^19
    struct imu_value *acc_value;
    
    if (buffer != NULL)
    {
	
//  	printf ("Converting %X %X %X\n", buffer[0], buffer[1], buffer[2]);
	
	copy_buffer[2] = buffer[0];
	copy_buffer[1] = buffer[1];
	copy_buffer[0] = buffer[2];
	acc_value = reinterpret_cast<struct imu_value *>(copy_buffer);
	
	acc_double = (double)(acc_value->value) / twopower19;
// 	std::cout<<"acc_double: "<<acc_double<<"\n";
	    
    }
    
    return acc_double * STIM300_GRAVITY;//units are in m/s^2
    
}


double STIM300Driver::convertIncl2Acceleration(const uint8_t* buffer)
{
    double incl_double = std::numeric_limits<double>::quiet_NaN();
    uint8_t copy_buffer[3];
    double twopower22 = 4194304;//2^22
    struct imu_value *incl_value;
    
    if (buffer != NULL)
    {
	
//  	printf ("Converting %X %X %X\n", buffer[0], buffer[1], buffer[2]);
	
	copy_buffer[2] = buffer[0];
	copy_buffer[1] = buffer[1];
	copy_buffer[0] = buffer[2];
	incl_value = reinterpret_cast<struct imu_value *>(copy_buffer);
	
	incl_double = (double)(incl_value->value) / twopower22;
// 	std::cout<<"acc_double: "<<acc_double<<"\n";
	    
    }
    
    return incl_double * STIM300_GRAVITY;//units are in m/s^2
}

double STIM300Driver::convertAcc2IncreVel(const uint8_t* buffer)
{
    double acc_double = std::numeric_limits<double>::quiet_NaN();
    uint8_t copy_buffer[3];
    double twopower22 = 4194304;//2^22
    struct imu_value *acc_value;
    
    if (buffer != NULL)
    {
	
//  	printf ("Converting %X %X %X\n", buffer[0], buffer[1], buffer[2]);
	
	copy_buffer[2] = buffer[0];
	copy_buffer[1] = buffer[1];
	copy_buffer[0] = buffer[2];
	acc_value = reinterpret_cast<struct imu_value *>(copy_buffer);
	
	acc_double = (double)(acc_value->value) / twopower22;
// 	std::cout<<"acc_double: "<<acc_double<<"\n";
	    
    }
    
    return acc_double; //units are m/s/sample
}

double STIM300Driver::convertTemp2Celsius(const uint8_t* buffer)
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

double STIM300Driver::convertLatency2Microseconds(const uint8_t* buffer)
{
    double latency_double = std::numeric_limits<double>::quiet_NaN();
    
    if (buffer != NULL)
    {
	latency_double = buffer[0] * 256.00 + buffer[1];
    }
    
    return latency_double;// units are microseconds

}


bool STIM300Driver::fullReset()
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

bool STIM300Driver::enterServiceMode()
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


bool STIM300Driver::setAcctoIncrementalVelocity()
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

ACC_OUTPUT STIM300Driver::getAccOutputType()
{
    return this->acc_output;
}


Eigen::Vector3d STIM300Driver::getAccData ()
{
    return Eigen::Vector3d (this->inertial_values.acc[0], this->inertial_values.acc[1], this->inertial_values.acc[2]);
}

Eigen::Vector3d STIM300Driver::getGyroData()
{
    return Eigen::Vector3d (this->inertial_values.gyro[0], this->inertial_values.gyro[1], this->inertial_values.gyro[2]);
}

Eigen::Vector3d STIM300Driver::getInclData()
{
    return Eigen::Vector3d (this->inertial_values.incl[0], this->inertial_values.incl[1], this->inertial_values.incl[2]);
}


Eigen::Vector3d STIM300Driver::getTempData()
{
    return Eigen::Vector3d (this->inertial_values.temp[0], this->inertial_values.temp[1], this->inertial_values.temp[2]);
}


int STIM300Driver::getPacketCounter()
{
    return (int) this->inertial_values.counter;
}

double STIM300Driver::getTempDataX()
{

    return this->inertial_values.temp[0];
}

double STIM300Driver::getTempDataY()
{

    return this->inertial_values.temp[1];
}


double STIM300Driver::getTempDataZ()
{

    return this->inertial_values.temp[2];
}

}