#include <stdlib.h>
#include <stdio.h>
#include <fcntl.h>
#include <string.h>
#include <SDL/SDL.h>

#include <errno.h>
#include <termios.h>
#include <unistd.h>

#include "data.h"

#define BUF_SIZE 			1024

// Sniffer modes
#define SLEEP_MODE			0
#define BURST_MODE			1
#define QUIT_MODE			2

#define CMD_SIZE			4
// Sniffer commands
#define CONNECT_BLUETOOTH	"CONN"
#define QUIT 				"QUIT"
#define BURST_READ			"BURS"
#define READ_LAST_ACQ		"LAST"
#define CONFIGURE_SENSORS	"SENS"
#define GET_STATUS			"STAT"

// Connexion status
#define DISCONNECTED		0
#define CONNECTED 			1

int set_interface_attribs (int fd, int speed, int parity);
void set_blocking (int fd, int should_block);
void connectBluetooth(int* fd);
void enterBurstMode(int* fd);
int parseData(int* fd);

int bluetooth_byte_cnt, byte_cnt;
char bluetooth_buf[BUF_SIZE];
int reception_mode;
ACQ_s acq;
int data_ready = 0;

unsigned int test_acq_cnt_requested = 0;
unsigned int test_acq_cnt_provided = 0;
unsigned int t_echo = 0;

bool connecting = 0;

#define HEADER 	0
#define DATA 	1

#define DISPLAY 0


int main(int argc, char* argv[])
{
	int p_mosi, p_miso;
	char buf_mosi[BUF_SIZE];
	char buf_bluetooth[BUF_SIZE];
	int mode;
	int connection_status;
	int mosi_byte_cnt, bluetooth_byte_cnt;
	int bluetooth_fd;
	int res_mosi, res_bluetooth;

	printf("Sniffer in position !\n");

	p_mosi = atoi(argv[1]);
	p_miso = atoi(argv[2]);

	int flags = fcntl(p_mosi, F_GETFL, 0);
	fcntl(p_mosi, F_SETFL, flags | O_NONBLOCK);

	connection_status = DISCONNECTED;
	mode = SLEEP_MODE;
	mosi_byte_cnt = 0;
	bluetooth_byte_cnt = 0;
	reception_mode = HEADER;
	char sensors_tmp;
	data_ready = 0;



	while(1)
	{
		switch(mode)
		{
			case SLEEP_MODE:
				res_mosi = read(p_mosi, buf_mosi + mosi_byte_cnt, CMD_SIZE - mosi_byte_cnt);
				if(res_mosi > 0)
					mosi_byte_cnt += res_mosi;

				if(connection_status == CONNECTED || connecting)
				{
					res_bluetooth = read(bluetooth_fd, buf_bluetooth + bluetooth_byte_cnt, CMD_SIZE - bluetooth_byte_cnt);
					if(res_bluetooth > 0)
						bluetooth_byte_cnt += res_bluetooth;
				}

				if(mosi_byte_cnt == CMD_SIZE)
				{
					buf_mosi[CMD_SIZE] = 0;
					if(strncmp(buf_mosi, GET_STATUS, CMD_SIZE) != 0)
						printf("SLAVE: RECEIVED %s\n", buf_mosi);

					mosi_byte_cnt = 0;
					if(strncmp(buf_mosi, QUIT, CMD_SIZE) == 0)
						mode = QUIT_MODE;
					else if(strncmp(buf_mosi, CONNECT_BLUETOOTH, CMD_SIZE) == 0)
					{
						printf("SLAVE: CONNEXION ATTEMPT\n");
						close(bluetooth_fd);
						connectBluetooth(&bluetooth_fd);
					}
					else if(strncmp(buf_mosi, BURST_READ, CMD_SIZE) == 0)
					{
						if(connection_status == CONNECTED)
						{
							enterBurstMode(&bluetooth_fd);
							mode = BURST_MODE;
						}
					}
					else if(strncmp(buf_mosi, CONFIGURE_SENSORS, CMD_SIZE) == 0)
					{
						read(p_mosi, buf_mosi, 1);
						write(bluetooth_fd, "SENS", 4);
						write(bluetooth_fd, buf_mosi, 1);
					}
					else if(strncmp(buf_mosi, GET_STATUS, CMD_SIZE) == 0)
					{
						if(connection_status == CONNECTED)
							write(p_miso, "C", 1);
						else
							write(p_miso, "D", 1);
					}
				}

				if(bluetooth_byte_cnt == CMD_SIZE)
				{
					buf_bluetooth[CMD_SIZE] = 0;
					bluetooth_byte_cnt = 0;
					if(strncmp(buf_bluetooth, "ECHO", 4) == 0)
					{
						printf("SUCCESS: CONNECTED TO DEVICE\n");
						connection_status = CONNECTED;
						connecting = 0;
					}
					else
					{
						printf("ERROR: BAD ECHO %s\n", buf_bluetooth);
						connection_status = DISCONNECTED;
						connecting = 1;
						while(read(bluetooth_fd, buf_bluetooth, BUF_SIZE) > 0)
							;
					}
				}

				if(connecting && ((SDL_GetTicks() - t_echo)/1000 > 5))
				{
					printf("ERROR: CONNECTION TIMED OUT\n");
					printf("SLAVE: CONNEXION ATTEMPT\n");
					close(bluetooth_fd);
					connectBluetooth(&bluetooth_fd);
				}


				SDL_Delay(5);
				break;
			case BURST_MODE:
				SDL_Delay(10);
				if(connection_status == CONNECTED)
					connection_status = parseData(&bluetooth_fd); // à faire : connection timed out

				res_mosi = read(p_mosi, buf_mosi + mosi_byte_cnt, CMD_SIZE - mosi_byte_cnt);
				if(res_mosi > 0)
					mosi_byte_cnt += res_mosi;

				if(mosi_byte_cnt == CMD_SIZE)
				{
					//buf_mosi[4] = 0;
					//printf("SLAVE: RECEIVED %s\n", buf_mosi);
					mosi_byte_cnt = 0;
					if(strncmp(buf_mosi, QUIT, CMD_SIZE) == 0)
					{
						mode = QUIT_MODE;
					}
					else if(strncmp(buf_mosi, READ_LAST_ACQ, CMD_SIZE) == 0)
					{
						test_acq_cnt_requested++;
						//printf("SNIFFER: ACQ_CNT_REQUESTED = %u\n", test_acq_cnt_requested);
						
						if(data_ready == 1)
						{
							test_acq_cnt_provided++;
							//printf("SNIFFER: ACQ_CNT_PROVIDED = %u\n", test_acq_cnt_provided);
							
							data_ready = 0;
							write(p_miso, (char*)&acq, sizeof(ACQ_s));
						}
						else
						{
							sensors_tmp = acq.header.sensors;
							acq.header.sensors = 0;
							write(p_miso, (char*)&acq, sizeof(ACQ_s));
							acq.header.sensors = sensors_tmp;
						}
					}
				}

				if(connection_status == DISCONNECTED)
				{
					close(bluetooth_fd);
					printf("SLAVE: CONNECTION ATTEMPT\n");
					connectBluetooth(&bluetooth_fd);
				}
				break;
			case QUIT_MODE:
				printf("Sniffer over and out !\n");
				close(p_mosi);
				close(p_miso);
				close(bluetooth_fd);
				return 0;
			default:
				break;
		}
	}
	return -1;
}
/*
int connectBluetooth(int* fd)
{
	char buf[BUF_SIZE];
	unsigned int t;
	int n = 0;
	int res;

	*fd = open ("/dev/rfcomm0", O_RDWR | O_NOCTTY | O_SYNC);

	int flags = fcntl(*fd, F_GETFL, 0);
	fcntl(*fd, F_SETFL, flags | O_NONBLOCK);

	set_interface_attribs (*fd, B115200, 0);		// set speed to 115,200 bps, 8n1 (no parity)
	set_blocking (*fd, 0);                			// set no blocking

	while(read(*fd, buf, BUF_SIZE) > 0)
		;

	
	write(*fd, "ECHO", 4);


	t = SDL_GetTicks();
	while(n < 4 && SDL_GetTicks() - t < 5000)
	{
		res = read(*fd, buf + n, 4 - n);
		if(res > 0)
			n += res;
	}
	if(n == 4)
	{
		if(strncmp(buf, "ECHO", 4) == 0)
		{
			printf("SUCCESS: CONNECTED TO DEVICE\n");
			return CONNECTED;
		}
		else
		{
			buf[n] = 0;
			printf("ERROR: BAD ECHO %s\n", buf);
			while(read(*fd, buf, BUF_SIZE) > 0)
				;
			return DISCONNECTED;
		}
	}
	else
	{
		printf("ERROR: CONNECTION TIMED OUT\n");
		return DISCONNECTED;
	}
	
	return CONNECTED;
}
*/


void connectBluetooth(int* fd)
{
	char buf[BUF_SIZE];
	unsigned int t;
	int n = 0;
	int res;

	connecting = 1;

	*fd = open ("/dev/rfcomm0", O_RDWR | O_NOCTTY | O_SYNC);

	int flags = fcntl(*fd, F_GETFL, 0);
	fcntl(*fd, F_SETFL, flags | O_NONBLOCK);

	set_interface_attribs (*fd, B115200, 0);		// set speed to 115,200 bps, 8n1 (no parity)
	set_blocking (*fd, 0);                			// set no blocking

	while(read(*fd, buf, BUF_SIZE) > 0)
		;
	
	write(*fd, "ECHO", 4);

	t_echo = SDL_GetTicks();
}

void enterBurstMode(int* fd)
{
	char buf[BUF_SIZE];
	strncpy(buf, "BURS", CMD_SIZE);
	write(*fd, buf, CMD_SIZE);
}

int parseData(int* fd)
{
	int cursor;
	int res;

	// The device has 5 sec to send a new acquisition. Otherwise, a conection timed out is triggered
	// To begin with, only the header is read since the total size of the data transmission can vary
	if(reception_mode == HEADER)
	{
		res = read(*fd, (char*)&(acq.header) + bluetooth_byte_cnt, sizeof(header_s) - bluetooth_byte_cnt);
		if(res > 0)
			bluetooth_byte_cnt += res;
	}
	else if(reception_mode == DATA)
	{
		res = read(*fd, bluetooth_buf + bluetooth_byte_cnt, byte_cnt - bluetooth_byte_cnt);
		if(res > 0)
			bluetooth_byte_cnt += res;
	}

	if(reception_mode == HEADER)
	{
		if(bluetooth_byte_cnt == sizeof(header_s))
		{
			reception_mode = DATA;
			bluetooth_byte_cnt = 0;
			if(DISPLAY)
			{
				printf("\nsensors: %X\n", acq.header.sensors);
				printf("time_stamp: %f\n", acq.header.time_stamp);
			}

			// The remaining amount of bytes to be read is calculated
			byte_cnt = 0;
			if(acq.header.sensors & GPS_MASK)
				byte_cnt += sizeof(GPS_s);
			if(acq.header.sensors & BAR_MASK)
				byte_cnt += sizeof(BAR_s);
			if(acq.header.sensors & ACC_MASK)
				byte_cnt += sizeof(ACC_s);
			if(acq.header.sensors & GYR_MASK)
				byte_cnt += sizeof(GYR_s);
			if(acq.header.sensors & MAG_MASK)
				byte_cnt += sizeof(MAG_s);
			if(acq.header.sensors & BAT_MASK)
				byte_cnt += sizeof(float);
		}
		else
		{
			return CONNECTED;
		}
	}
	else if(reception_mode == DATA)
	{
		if(bluetooth_byte_cnt == byte_cnt)
		{
			reception_mode = HEADER;
			bluetooth_byte_cnt = 0;
			cursor = 0;
			data_ready = 1;
			if(acq.header.sensors & GPS_MASK)
			{
				memcpy(&(acq.gps), bluetooth_buf + cursor, sizeof(GPS_s));
				cursor += sizeof(GPS_s);

				if(DISPLAY)
				{
					printf("data_valid: %c\n", acq.gps.data_valid);
					printf("nb_sat: %u\n", acq.gps.nb_sat);
					if(acq.gps.nb_sat != 0)
					{
						printf("date: %u/%u/20%u\n", acq.gps.day, acq.gps.month, acq.gps.year);
						printf("time: %u:%u.%u\n", acq.gps.hour, acq.gps.min, acq.gps.sec);
						printf("latitude: %f %c\n", acq.gps.latitude, acq.gps.lat);
						printf("longitude: %f %c\n", acq.gps.longitude, acq.gps.lon);
						printf("alt: %f\n", acq.gps.alt_gps);
						printf("speed: %f\n", acq.gps.speed);
						printf("angle: %f\n", acq.gps.angle);
					}
				}
			}

			if(acq.header.sensors & BAR_MASK)
			{
				memcpy(&(acq.bar), bluetooth_buf + cursor, sizeof(BAR_s));
				cursor += sizeof(BAR_s);
				if(DISPLAY)
				{
					printf("tmp: %f °C\n", acq.bar.temperature);
					printf("pressure: %f hPa\n", acq.bar.pressure);
					printf("altitude: %f m\n", acq.bar.altitude);
				}
			}

			if(acq.header.sensors & ACC_MASK)
			{
				memcpy(&(acq.acc), bluetooth_buf + cursor, sizeof(ACC_s));
				cursor += sizeof(ACC_s);
				if(DISPLAY)
				{
					printf("X accel: %f\n", acq.acc.ax);
				}
			}

			if(acq.header.sensors & GYR_MASK)
			{
				memcpy(&(acq.gyr), bluetooth_buf + cursor, sizeof(GYR_s));
				cursor += sizeof(GYR_s);
				if(DISPLAY)
				{
					printf("gx: %f °/s\n", acq.gyr.gx);
				}
			}

			if(acq.header.sensors & MAG_MASK)
			{
				memcpy(&(acq.mag), bluetooth_buf + cursor, sizeof(MAG_s));
				cursor += sizeof(MAG_s);
				if(DISPLAY)
				{
					printf("hx: %f uT\n", acq.mag.hx);
					printf("hy: %f uT\n", acq.mag.hy);
					printf("hz: %f uT\n", acq.mag.hz);
				}
			}

			if(acq.header.sensors & BAT_MASK)
			{
				memcpy(&(acq.voltage), bluetooth_buf + cursor, sizeof(float));
				if(DISPLAY)
				{
					printf("voltage: %f\n", acq.voltage*3.3*6.6*3.85/(4.4*5.0));
				}
			}
		}
		else
		{
			return CONNECTED;
		}
	}
	return CONNECTED;
}



int set_interface_attribs (int fd, int speed, int parity)
{
    struct termios tty;
    memset (&tty, 0, sizeof tty);
    if (tcgetattr (fd, &tty) != 0)
    {
        //error_message ("error %d from tcgetattr", errno);
        return -1;
    }

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
    tty.c_cflag &= ~(PARENB | PARODD);      // shut off parity
    tty.c_cflag |= parity;
    tty.c_cflag &= ~CSTOPB;
    tty.c_cflag &= ~CRTSCTS;

    if (tcsetattr (fd, TCSANOW, &tty) != 0)
    {
        //error_message ("error %d from tcsetattr", errno);
        return -1;
    }
    return 0;
}

void set_blocking (int fd, int should_block)
{
    struct termios tty;
    memset (&tty, 0, sizeof tty);
    if (tcgetattr (fd, &tty) != 0)
    {
        //error_message ("error %d from tggetattr", errno);
        return;
    }

    tty.c_cc[VMIN]  = should_block ? 1 : 0;
    tty.c_cc[VTIME] = 5;            // 0.5 seconds read timeout

    if (tcsetattr (fd, TCSANOW, &tty) != 0)
    	return;
            //error_message ("error %d setting term attributes", errno);
}