#include "Stim300RevB.hpp"

using namespace imu_stim300;

Stim300RevB::Stim300RevB()
:Stim300Base(MAX_PACKET_SIZE)
{

    this->currentP = reinterpret_cast<struct packet *>(this->buffer);

    this->content = RATE_ACC_INCLI_TEMP_REVB;

    this->prev_counter = std::numeric_limits<double>::quiet_NaN();

    this->counter_ratio = imu_stim300::DEFAULT_SAMPLING_FREQUENCY/this->sampling_frequency;

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

    /** Revision D has 9 temperature sensors **/
    this->inertial_values.temp.resize(9);
    for (std::vector<double>::iterator it = this->inertial_values.temp.begin() ;
            it != this->inertial_values.temp.end(); ++it)
        *it = std::numeric_limits<double>::quiet_NaN();

    this->inertial_values.latency = std::numeric_limits<double>::quiet_NaN();
    this->inertial_values.checksum = false;

}

int Stim300RevB::extractPacket(const uint8_t* buffer, size_t buffer_size) const
{

    int packet_state = 0; //0-> no synchronized, 1->Semi synchonized, 2-> package found
    unsigned short start_position = 0;

//    std::cout<<"**** extractPacket ****\n";
//    std::cout<<"buffer_size: "<< buffer_size <<"\n";
//    for ( int i=0; i < (int)buffer_size; i++)
//        printf("%X \n", buffer[i]);

    if (this->modes == NORMAL)
    {
        /** Found start of the packet **/
        for (unsigned int i=0; i < (unsigned int)buffer_size; i++)
        {
                if (buffer[i] == this->content)
                {
                    start_position = i;
                    packet_state = 1;
                    break;
                }
        }

        /** Verify there is another start of the packet afterwards **/
        if (packet_state == 1)
        {
            for (unsigned int i=0; i < (unsigned int)buffer_size; i++)
            {
                if (buffer[i] == this->content)
                {
                    if (i == start_position + sizeof(struct packet))
                    {
                        packet_state = 2;
                        break;
                    }
                }
                else if (i == (start_position+(sizeof(struct packet)-1))) //end of the CRC
                {
                    packet_state = 2;
                    break;
                }
            }
        }


//        std::cout<<"packet_state: "<<packet_state<<"\n";
//        std::cout<<"**** end extractPacket ****\n";


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
	        return sizeof(struct packet);
        }

    }
    else //if (this->modes == SERVICE)
    {
        return sizeof(struct packet);
    }
	
}

int Stim300RevB::processPacket()
{
    int buf_size = 0;
    //std::cout<<"processPacket\n";

    try {
        buf_size = Driver::readPacket (this->buffer, MAX_PACKET_SIZE, this->pckgTimeout);
    } catch (iodrivers_base::TimeoutError& e ) { 
	std::cerr<<"TimeoutError buffer size: "<<buf_size<<"\n";
	return false;
    }

//     std::cout<<"buf_size: "<<buf_size<<"\n";
//      for (int i =0; i<buf_size; i++)
//  	    printf("%X \n", buffer[i]);

    if (this->modes == NORMAL)
    {

        //std::cout<<"sizeof(struct packet): "<<sizeof(struct packet)<<"\n";
        //std::cout<<"sizeof(buffer): "<<sizeof(buffer)<<"\n";

        /** Verify the Checksum **/
        this->inertial_values.checksum = verifyChecksum(this->expectedCRC, this->calculatedCRC);

        //if ((this->currentP->acc_status + this->currentP->gyros_status + this->currentP->incl_status) == 0)
        //{
            this->internal_error = false;

            /** If everything is alright convert the values **/
            if (prev_counter == std::numeric_limits<double>::quiet_NaN())
                prev_counter = this->currentP->counter;
            else
            {
                this->inertial_values.counter += (this->currentP->counter - prev_counter)%this->counter_ratio;
                prev_counter = this->currentP->counter;
            }


            /** Convert gyros raw to calibrated values **/
            this->inertial_values.gyro[0] = convertGyro2AngularRate(reinterpret_cast<uint8_t*>(&(*this->currentP).gyro_x));
            this->inertial_values.gyro[1] = convertGyro2AngularRate(reinterpret_cast<uint8_t*>(&(*this->currentP).gyro_y));
            this->inertial_values.gyro[2] = convertGyro2AngularRate(reinterpret_cast<uint8_t*>(&(*this->currentP).gyro_z));

            /** Convert accelerometers raw to calibrated values **/
            if (acc_output == ACCELERATION)
            {
                this->inertial_values.acc[0] = convertAcc2Acceleration(reinterpret_cast<uint8_t*>(&(*this->currentP).acc_x));
                this->inertial_values.acc[1] = convertAcc2Acceleration(reinterpret_cast<uint8_t*>(&(*this->currentP).acc_y));
                this->inertial_values.acc[2] = convertAcc2Acceleration(reinterpret_cast<uint8_t*>(&(*this->currentP).acc_z));
            }
            else if (acc_output == INCREMENTAL_VELOCITY)
            {
                this->inertial_values.acc[0] = convertAcc2IncreVel(reinterpret_cast<uint8_t*>(&(*this->currentP).acc_x));
                this->inertial_values.acc[1] = convertAcc2IncreVel(reinterpret_cast<uint8_t*>(&(*this->currentP).acc_y));
                this->inertial_values.acc[2] = convertAcc2IncreVel(reinterpret_cast<uint8_t*>(&(*this->currentP).acc_z));
            }

            /** Convert inclinometers raw to calibrated values **/
            this->inertial_values.incl[0] = convertIncl2Acceleration(reinterpret_cast<uint8_t*>(&(*this->currentP).incl_x));
            this->inertial_values.incl[1] = convertIncl2Acceleration(reinterpret_cast<uint8_t*>(&(*this->currentP).incl_y));
            this->inertial_values.incl[2] = convertIncl2Acceleration(reinterpret_cast<uint8_t*>(&(*this->currentP).incl_z));

            /** Convert gyros temperature raw to calibrated values **/
            this->inertial_values.temp[0] = convertTemp2Celsius(reinterpret_cast<uint8_t*>(&(*this->currentP).gtemp_x));
            this->inertial_values.temp[1] = convertTemp2Celsius(reinterpret_cast<uint8_t*>(&(*this->currentP).gtemp_y));
            this->inertial_values.temp[2] = convertTemp2Celsius(reinterpret_cast<uint8_t*>(&(*this->currentP).gtemp_z));

            this->inertial_values.latency = convertLatency2Microseconds(reinterpret_cast<uint8_t*>(&(*this->currentP).latency));

            return true;
        //}
        //else
        //{
        //    std::cout<<"ERROR IN INTERNAL DATA\n";
        //    this->internal_error = true;
        //    this->fullReset();
        //}
    }
    else if (this->modes == SERVICE)
    {
	std::string s(this->buffer, this->buffer + sizeof(this->buffer));
	std::cout<<s<<"\n";
	return true;
	
    }

    return false;

}

void Stim300RevB::printInfo()
{
    std::cout<<"** STIM300 Serial Driver Information **\n";
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
    std::cout<<"Gyros Temperature[Celsius]: "<<this->inertial_values.temp[0]<<" "<<this->inertial_values.temp[1]<<" "<<this->inertial_values.temp[2]<<"\n";
    std::cout<<"Latency[microsecond]: "<<this->inertial_values.latency<<"\n";
    printf("Checksum: %X [%X]\n", this->calculatedCRC, this->expectedCRC);
    std::cout<<"*********************************\n";
}

double Stim300RevB::convertAcc2Acceleration(const uint8_t* buffer)
{
    double acc_double = std::numeric_limits<double>::quiet_NaN();
    uint8_t copy_buffer[3];

    double twopower19 = 524288;//2^19

    struct sensor_value *acc_value;

    if (buffer != NULL)
    {
	
//  	printf ("Converting %X %X %X\n", buffer[0], buffer[1], buffer[2]);
	
	copy_buffer[2] = buffer[0];
	copy_buffer[1] = buffer[1];
	copy_buffer[0] = buffer[2];
	acc_value = reinterpret_cast<struct sensor_value *>(copy_buffer);
	
	acc_double = (double)(acc_value->value) / twopower19;

// 	std::cout<<"acc_double: "<<acc_double<<"\n";

    }

    return acc_double * STIM300_GRAVITY;//units are in m/s^2

}

bool Stim300RevB::verifyChecksum(boost::uint32_t &expintCRC, boost::uint32_t &calintCRC)
{
    boost::crc_basic<32>  crc_32(0x04C11DB7, 0xFFFFFFFF, 0x00, false, false);

    /** Here the variable buffer is the global variable **/
    uint8_t bufferCRC[sizeof(struct packet)-sizeof(uint32_t)+NUMBER_PADDING_BYTES];
    std::copy (this->buffer, this->buffer+sizeof(bufferCRC), bufferCRC);

    /** Fill the Dummy bytes with 0x00. There are at the end of the buffer **/
    for (size_t i=0; i<NUMBER_PADDING_BYTES; ++i)
        bufferCRC[sizeof(bufferCRC)-(1+i)] = 0x00;

    crc_32.process_bytes(bufferCRC, sizeof(bufferCRC));
    //printf("Checksum: %X \n", crc_32.checksum());

    /** Calculated CRC **/
    calintCRC = crc_32.checksum();

    /** Expected CRC endianness is corrected **/
    uint8_t expectedCRC[sizeof(uint32_t)] = {this->buffer[sizeof(struct packet)-1],
                                             this->buffer[sizeof(struct packet)-2],
                                             this->buffer[sizeof(struct packet)-3],
                                             this->buffer[sizeof(struct packet)-4]};

    expintCRC = *(reinterpret_cast<boost::uint32_t*> (expectedCRC));

    //printf("exp CRC: %X \n", expintCRC);
    if(calintCRC == expintCRC)
    {
        return true;
    }
    else
        return false;

}



