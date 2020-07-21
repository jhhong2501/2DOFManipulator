﻿#ifndef DATATYPE_H_
#define DATATYPE_H_

typedef struct _packet_data{
	unsigned char header[4];
	unsigned char size, id;
	unsigned char mode, check;
	int32_t pos;
	int32_t velo;
	int32_t cur;
}Packet_data_t;


typedef union _packet{
	Packet_data_t data;
	unsigned char buffer[sizeof(Packet_data_t)];
}Packet_t;

#endif