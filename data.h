#ifndef DATA_H_
#define DATA_H_

#define FILE_NAME_SIZE 31		// Taille maximale du nom d'un fichier

// Masks
#define GPS_MASK 		0x20
#define BAR_MASK		0X10
#define ACC_MASK		0x08
#define GYR_MASK		0X04
#define MAG_MASK		0x02
#define BAT_MASK		0X01

/***************** Structures utilisées pour échanger les données *****************/

typedef struct 
{
	uint8_t sensors;				// Flags indiquant les capteurs utilisés lors de l'acquisition : | XXX | XXX | GPS | BAR | ACC | GYR | MAG | BAT |
	//unsigned int time_stamp;		// temps écoulé au moment de l'acquisition, en secondes
	float time_stamp;
	//uint32_t time_stamp;
	//uint8_t time_stamp;
}__attribute__((packed)) header_s;

typedef struct
{
	unsigned char year, month, day, hour, min, sec;
	char lat, lon; 					// 'E'/'W' and 'N'/'S'
	float latitude, longitude; 		// °
	unsigned char nb_sat;			// Number of satellites
	float alt_gps;					// Altitude (cm)
	char data_valid;				// 'A' or 'B'
	float speed;					// m/s
	float angle;					// °
}__attribute__((packed)) GPS_s;

typedef struct
{
	float pressure;					// hPa
	float temperature;				// °C
	float altitude;					// m
}__attribute__((packed)) BAR_s;

typedef struct
{
	float ax, ay, az;				// m/s²
}__attribute__((packed)) ACC_s;

typedef struct
{
	float gx, gy, gz;				// °/s
}__attribute__((packed)) GYR_s;

typedef struct
{
	float hx, hy, hz;				// °
}__attribute__((packed)) MAG_s;


// Classe utilisée pour stocker les données d'une acquisition
typedef struct
{
	// header
	header_s header;
	// GPS
	GPS_s gps;
	// Barometer
	BAR_s bar;
	// Accelerometer
	ACC_s acc;
	// Gyroscope
	GYR_s gyr;
	// Magnetometer
	MAG_s mag;
	// Battery
	float voltage;	// V
}__attribute__((packed)) ACQ_s;


#endif /* DATA_H_ */
