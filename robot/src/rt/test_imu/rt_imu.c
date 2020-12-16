/**
 * @file rt_imu.c
 * @brief Hardware interface for serial IMU
 */
#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>
#include <errno.h>
#include <fcntl.h>
#include <string.h>
#include <termios.h>
#include <errno.h>
#include <math.h>
#include <pthread.h>

#include <endian.h>
#include <stdint.h>

#include "config.h"\

#include "rt_imu.h"

//***********CRC**************//
uint32_t crc_table[256];
const uint8_t CRC_ORDER = 32;
const uint32_t CRC_HIGHBIT = (uint32_t) (1UL << 31);
const uint32_t CRC_POLY = (uint32_t) (0x04c11db7);
const uint32_t CRC_INIT = (uint32_t) (0xffffffff);

/**
 * @brief Generates CRC Checksum table. Must be called before checksums can be verified.
 *
 * This code came from Wikipedia.
 */
void crc_generate_table(void)
{
    printf("[IMU-Serial Init] Generating CRC Table...\n");
    uint8_t i;
    uint16_t dividend;
    uint32_t remainder;

    for (dividend=0; dividend<256; dividend++) {
        remainder = (uint32_t) dividend;
        remainder <<= CRC_ORDER-8;

        for (i=0; i<8; i++) {
            if (remainder & CRC_HIGHBIT) {;
                remainder = (remainder << 1) ^ CRC_POLY;
            }
            else {
                remainder = (remainder << 1);
            }
        }
        crc_table[dividend] = remainder;
    }
    printf("[IMU-Serial Init] Done!\n");
}

/**
 * @brief Calculates the CRC checksum
 *
 * This code came from Wikipedia
 * @param packet Pointer to packet array
 * @param num_bytes Size of packet array in bytes
 * @return Checksum
 */
uint32_t crc_calc(uint8_t *packet, uint8_t num_bytes)
{
    num_bytes = 32;
    uint32_t crc = CRC_INIT;

    /* Divide packet by polynomial one byte at a time. */
    while (num_bytes--) {
        crc = (crc << 8) ^ crc_table[ ((crc >> 24) & 0xff) ^ *packet++];
    }

    return crc;
}

//verify checksum code
//void crc_debug_check_crc()
//{
//    // Test CRC with packet from IMU.
//    uint8_t str[] = {0xfe,0x81,0xff,0x55,0xb6,0x06,0xd8,0x1e,0xb6,0xc4,0x94,0x0f,0x36,0xd9,0x5a,0x83,0x3c,0x61,0xad,0x86,0x3b,0xe5,0xd0,0x86,0x3f,0x7f,0xdc,0xd4,0x77,0x72,0x00,0x26};
//    uint32_t crc = 0xc0f2d540;
//    uint8_t len = 32;
//
//    printf("CRC %x compare result: %x\n", crc_calc(str, len),  crc);
//}

//reliable send WILL BLOCK until message is sent!
/**
 * @brief Block until message is sent over serial
 *
 * @param fd Serial port FD
 * @param start Pointer to start of message
 * @param size Size of message
 */
void reliable_send(int fd, const void* start, size_t size)
{
#ifndef SIMULATOR
    int to_send = size;
    int sent = 0;
    while(to_send > 0)
    {
        int new_sent = write(fd, start + sent, to_send);
        to_send -= new_sent;
        sent += new_sent;
    }
#endif
}

/**
 * @brief Block until message is read from serial
 *
 * @param fd Serial port FD
 * @param start Pointer to start of message buffer
 * @param size Size of message to read
 */
void reliable_read(int fd, void* start, size_t size)
{
    int to_read = size;
    int bytes_read = 0;
    //while there are bytes to read...
    while(to_read > 0)
    {
        //read into the buffer, starting from where we left off
        int new_read = read(fd, start + bytes_read, to_read);
        //update the number of bytes left to read
        to_read -= new_read;
        //and update where we need to start writing into the buffer
        bytes_read += new_read;

    }

}

/**
 * @brief Configure serial port
 *
 * @param fd Serial port FD
 * @param speed Baud rate
 * @param parity Parity
 * @param port Port number
 */


int set_interface_attribs (int fd, int speed, int parity, int port)
{
    char error_msg[100];
#ifndef SIMULATOR
    //printf("[IMU-Serial Init] Configuring serial device...\n");
    printf("[IMU-Serial Init] Configuring serial device on port: %d and with baud: %d \n",port,speed);
    struct termios tty;
    memset (&tty, 0, sizeof tty);
    if (tcgetattr (fd, &tty) != 0)
    {
        sprintf(error_msg,"[Serial Error] Error %d from tcgetattr on port %d\n", errno, port);

        return -1;
    }
    if (port == 3){ //RC NEEDS DIFFERENT CONFIGURATION
        // Set the baud rates to 9600
        printf("[IMU]setting interface attribs on port: %d\n",port);
        cfsetispeed(&tty, B9600);
        cfsetospeed(&tty, B9600);

        cfmakeraw(&tty);

        tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;     // 8-bit chars
        tty.c_iflag &= ~IGNBRK;         // disable break processing
        tty.c_lflag = 0;
        tty.c_oflag = 0;                // no remapping, no delays
        tty.c_cc[VMIN]  = 0;
        tty.c_cc[VTIME] = 5;
        tty.c_iflag &= ~(IXON | IXOFF | IXANY); // shut off xon/xoff ctrl
        tty.c_cflag |= (CLOCAL | CREAD);   // Enable the receiver and set local mode
        tty.c_cflag |= 0;
        tty.c_cflag &= ~CSTOPB;            // 1 stop bit
        tty.c_cflag |= CRTSCTS;           // Disable hardware flow control
        tty.c_cc[VMIN]  = 1 ? 1 : 0;
        tty.c_cc[VTIME] = 5;
    }
    else

    {
        printf("[IMU]setting interface attribs on port: %d\n",port);
        cfsetospeed (&tty, speed);
        cfsetispeed (&tty, speed);

        tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;     // 8-bit chars
        // disable IGNBRK for mismatched speed tests; otherwise receive break
        // as \000 chars
        tty.c_iflag &= ~IGNBRK;         // disable break processing
        tty.c_lflag = 0;                // no signaling chars, no echo,
        // no canonical processing
        tty.c_oflag = 0;                // no remapping, no delays
        tty.c_cc[VMIN]  = 0;            // read doesn't block
        tty.c_cc[VTIME] = 5;            // 0.5 seconds read timeout

        tty.c_iflag &= ~(IXON | IXOFF | IXANY); // shut off xon/xoff ctrl

        tty.c_cflag |= (CLOCAL | CREAD);// ignore modem controls,
        // enable reading
        //tty.c_cflag &= ~(PARENB | PARODD);      // shut off parity
        tty.c_cflag |= parity;
        tty.c_cflag |= CSTOPB;
        tty.c_cflag &= ~CRTSCTS;
        cfmakeraw(&tty);
    }

    if (tcsetattr (fd, TCSANOW, &tty) != 0)
    {
        sprintf(error_msg,"[IMU-Serial Error] Error %d from tcsetattr on port %d\n", errno, port);        printf("[IMU-Serial Init] Second Error...\n");

        return -1;
    }
    printf("[IMU-Serial Init] Done!\n");
    return 0;
#endif
}

/**
 * @brief Set up serial port for blocking/nonblocking mode
 *
 * @param fd Serial port FT
 * @param should_block Enable/disable blocking
 * @param port Serial port number
 */
void set_blocking (int fd, int should_block, int port)
{
    printf("[IMU-Serial Init] setting blocking on port: %d\n",port);
    char error_msg[100];
    struct termios tty;
    memset (&tty, 0, sizeof tty);
    if (tcgetattr (fd, &tty) != 0)
    {
        sprintf(error_msg,"[Serial Error] Error %d from tggetattr on port %d\n", errno, port);

        return;
    }

    tty.c_cc[VMIN]  = should_block ? 1 : 0;
    tty.c_cc[VTIME] = 5;            // 0.5 seconds read timeout

    if (tcsetattr (fd, TCSANOW, &tty) != 0)
    {
        sprintf(error_msg,"[Serial Error] Error %d setting term attributes on port %d\n", errno, port);

    }
}


volatile int crc_fails = 0;
volatile int crc_matches = 0;
volatile int message_buffer[12];

/**
 * @brief get the number of checksum match fails
 * @return Number of fails
 */
int get_fails()   {return crc_fails;}
/**
 * @brief get the number of checksum matches
 * @return Number of matches
 */
int get_matches() {return crc_matches;}

float imu_valid_data[6];
/**
 * @brief Loop to read data from the IMU as fast as possible
 *
 * This contains most of the serial decoding logic.
 */
void serial_read()
{
    char error_msg[100];
#ifndef SIMULATOR
    crc_generate_table();
    //    crc_debug_check_crc();
    const char config1[] = K_IMU_INIT_1;
    const char config2[] = K_IMU_INIT_2;
    const char config3[] = K_IMU_INIT_3;
    printf("[IMU-Serial Init] Opening serial port...\n");
    char *portname = K_SERIAL_PORT_NAME;
    printf("[IMU-Serial Init] Done!\n");

    int fd = open (portname, O_RDWR | O_NOCTTY | O_SYNC);
    if (fd < 0)
    {
        sprintf(error_msg,"[IMU-Serial Error] Error %d opening %s: %s", errno, portname, strerror (errno));
        return;

    }
    set_interface_attribs (fd, B921600, 0, K_IMU_SERIAL_PORT);
    set_blocking (fd, 1, K_IMU_SERIAL_PORT);
    usleep(1000000);
    printf("[IMU-Serial Init] Configuring IMU...\n");

    reliable_send(fd, config1, sizeof(config1));
    usleep(1000000);
    reliable_send(fd, config2, sizeof(config2));
    usleep(1000000);
    reliable_send(fd, config3, sizeof(config3));
    printf("[IMU-Serial Init] IMU Ready!  Starting read loop.\n");
    int header_buffer;
    char buffer_check[4] = {0};

    int not_ready = 0;
    int imuc = 0;
    // while(1==1)
    while(imuc < 2)
    {
        //read 4 byte chunks until we get to the magic number
        reliable_read(fd, &buffer_check[3], 1);
      	int * pHeader = (int *) buffer_check;
      	header_buffer = *pHeader;
      	buffer_check[0] = buffer_check[1];
      	buffer_check[1] = buffer_check[2];
      	buffer_check[2] = buffer_check[3];


        if(header_buffer == 1442808318)
        {
            //store header in buffer for CRC check
            message_buffer[0] = be32toh(1442808318);
            //printf("header: %x ", message_buffer[0]);

            //loop through the 6 floats for acceleration/angle data
            for(int i = 0; i < 6; i++)
            {
                int float_buffer = -1;
                //read flfoat
                reliable_read(fd, &float_buffer, sizeof(int));
                //flip byte ordering
                float_buffer = be32toh(float_buffer);
                //add to message buffer
                message_buffer[i + 1] = float_buffer;
                //printf("float %d: %*.*f ",i, 5,3, *(float*)(&message_buffer[i+1]) );
            }
            //read status byte
            uint8_t status_buffer = 0;
            reliable_read(fd, &status_buffer, 1);
            //printf("status: %x ", status_buffer);
            //verify we got the "ready" byte

            uint8_t iteration_buffer = 0;
            //read iteration number
            reliable_read(fd, &iteration_buffer, 1);
            //printf("iteration: %d ", iteration_buffer);

            int16_t temp_buffer = 0;
            //read temperature
            reliable_read(fd, &temp_buffer, 2);
            temp_buffer = be16toh(temp_buffer);
            //printf("temp: %d", temp_buffer);

            //from iteration, temperature, and status byte, reconstruct the status message
            int32_t status_message = (status_buffer << 24) + (iteration_buffer << 16) + temp_buffer;
            //int32_t status_message = iteration_buffer << 16;
            message_buffer[7] = status_message;
            //printf("\n");
            //printf("desired status bytes: %hhx %hhx %04x actual: %x \n", status_buffer, iteration_buffer, temp_buffer, status_message );
            // for(int i = 0; i < 8; i++)
            // {
            //     printf("message_buffer: %x ", message_buffer[i]);
            // }
            // printf("\n");

            //read crc checksum
            uint32_t crc_buffer = 0;
            reliable_read(fd, &crc_buffer, 4);
            crc_buffer = be32toh(crc_buffer);

            //array of bytes to use CRC on
            uint8_t crc_format[32];
            //change byte ordering
            crc_format[0] = *((uint8_t*)&(message_buffer[0]) + 3);
            crc_format[1] = *((uint8_t*)&(message_buffer[0]) + 2);
            crc_format[2] = *((uint8_t*)&(message_buffer[0]) + 1);
            crc_format[3] = *((uint8_t*)&(message_buffer[0]) + 0);

            for(int i = 0; i < 6; i++)
            {
                crc_format[4*(i + 1) + 0] = *((uint8_t*)&(message_buffer[i + 1]) + 3);
                crc_format[4*(i + 1) + 1] = *((uint8_t*)&(message_buffer[i + 1]) + 2);
                crc_format[4*(i + 1) + 2] = *((uint8_t*)&(message_buffer[i + 1]) + 1);
                crc_format[4*(i + 1) + 3] = *((uint8_t*)&(message_buffer[i + 1]) + 0);
            }

            crc_format[28] = *((uint8_t*)&(message_buffer[0]) + 31);
            crc_format[29] = *((uint8_t*)&(message_buffer[0]) + 30);
            crc_format[30] = *((uint8_t*)&(message_buffer[0]) + 29);
            crc_format[31] = *((uint8_t*)&(message_buffer[0]) + 28);

            //compute checksum
            uint32_t computed_crc = crc_calc(crc_format, 32);
            //printf("got crc: %x , expected: %x\n", crc_buffer, computed_crc);

            //check match
            if(computed_crc == crc_buffer)
            {
                crc_matches++;
                for(int i = 0; i < 6; i++)
                {
                    //if we've matched, update the valid IMU data
                    imu_valid_data[i] = *((float*)&message_buffer[i + 1]);
                }
                if(status_buffer != 0x77)
                {
                    not_ready++;
                    printf("[IMU-Serial Error] Got bad status bit: %x \n", status_buffer);
                }
            }
            else
            {
                //if we've failed, count the failure and log over ethercat
                crc_fails++;
                // increment_bad_imu_packet();
            }

            //every 1000 messages, print status update
            if((crc_matches + crc_fails)%1000 == 0)
            {

                printf("\x1b[32m---------------------------------\n");
                printf("          IMU Statistics  \n");
                printf("---------------------------------\x1b[0m \n");
                printf("Total serial reads: \x1b[36m%d\x1b[0m \n", get_matches() + get_fails());
                printf("Percentage valid IMU packets:  \x1b[36m%f\x1b[0m \n", (float)get_matches()/(float)(get_matches() + get_fails()));
                printf("Valid IMU packets: \x1b[36m%d\x1b[0m\n", get_matches());
                float* imu_data = get_imu_data();
                printf("Data: %f %f %f %f %f %f \n", imu_data[0],imu_data[1],imu_data[2],imu_data[3],imu_data[4],imu_data[5]);
                printf("\n\n\n");
                printf("counter_imu: %d\n",imuc);
                imuc++;
            }
        }
    }
#else
    if (pthread_mutex_init(&imu_data_mutex, NULL) != 0)
    {
        printf("rt_imu mutex init failed\n");
        return;
    }
#endif
}

//return imu data which may be invalid
/**
 * @brief Get most recent data from IMU - MAY BE INVALID!
 * @return Pointer to array of IMU data
 */
float* get_raw_imu_data()
{
    return (float*)&(message_buffer[1]);
}

//return imu data guarenteed to be valid
/**
 * @brief Get most recent valid data from IMU.
 * @return Pointer to array of IMU data
 */
float* get_imu_data()
{
    return imu_valid_data;
}

//array index off by 1 for matlab
/**
 * @brief Get IMU data from MATLAB/SIMULINK
 * @param index Index of IMU data (starts at 1 for MATLAB compatibility
 * @return Value of array at @c index
 */
float get_imu_data_matlab(int index)
{
    float* imu_data = get_imu_data();
    //offset the index by one for matlab.
    return imu_data[index - 1];
}

/**
 * @brief Initialize serial for IMU
 */
void init_serial()
{
    printf("[IMU-Serial] Initializing IMU Serial connection\n");
    serial_read();
}
