#include "dataPacking.h"
#include "main.h"
#include "queternion.h"
#include "usr_gnss_l86_parser.h"

union DataPack veriler;
uint8_t bufferPrint[400];

static uint8_t calculateCRC()
{
	int check_sum = 0;
	for(int i = 1; i < sizeof(veriler.dataYapi) - 3; i++){
		check_sum += veriler.arr[i];
	}
	return (uint8_t) (check_sum % 256);
}

static void sendRF()
{

		//HAL_UART_Transmit_DMA(&huart4, "hello\n\r", 7);

}

static void sendPC()
{

	HAL_UART_Transmit(&TTL_HNDLR, veriler.arr , sizeof(veriler.dataYapi), 50);

}

uint8_t* packDatas(bmi088_struct_t *bmi, bme280_struct_t *bme, S_GPS_L86_DATA* gps , power_t *power_s, uint16_t status)
{
	veriler.dataYapi.basla = 0xFF;

	uint8_t min = 0;
	uint8_t sec = 0;
	int gpsTime = (int)gps->timeDateBuf;
	sec = gpsTime % 100;
	gpsTime /= 100;
	min = gpsTime % 100;
	min = (min << 2) | (sec >> 4);
	sec = (sec << 4) | (status);
	veriler.dataYapi.zaman = min;
	veriler.dataYapi.durum = sec;

	veriler.dataYapi.voltaj = (uint16_t)(int)(power_s->voltage * 100);
	veriler.dataYapi.akim = (uint16_t)(int)(power_s->current);

	veriler.dataYapi.sicaklik = (int8_t)(int)(bme->datas.temperature * 2);
	veriler.dataYapi.nem = (uint8_t)(int)(bme->datas.humidity);

	veriler.dataYapi.yukseklik_p = bme->datas.altitude;
#ifdef ROCKET_CARD
	veriler.dataYapi.maxAltitude = (int16_t)(int)bme->parameters->max_alt;
#else
	veriler.dataYapi.maxAltitude = 0;
#endif
	veriler.dataYapi.yukseklik_gps = gps->altitudeInMeter;

	veriler.dataYapi.lat = gps->lat;
	veriler.dataYapi.lon = gps->lon;

	veriler.dataYapi.gyroX = -bmi->datas.gyro_x;
	veriler.dataYapi.gyroY = -bmi->datas.gyro_z;
	veriler.dataYapi.gyroZ = -bmi->datas.gyro_y;

	veriler.dataYapi.accX = bmi->datas.acc_x / 1000;
	veriler.dataYapi.accY = bmi->datas.acc_z / 1000;
	veriler.dataYapi.accZ = (status > STAT_ROCKET_READY) ? (-bmi->datas.acc_y / 1000) - 1.0 : bmi->datas.acc_y / 1000;

	veriler.dataYapi.uyduSayisi = ((uint8_t)gps->satInUse << 3) | (((int)euler[0] & 0x8000) >> 13) | (((int)euler[1] & 0x8000) >> 14) | (((int)euler[2] & 0x8000) >> 15);
	veriler.dataYapi.hiz = (int16_t)(int)(bme->datas.velocity * 10);

	veriler.dataYapi.aci = quaternionToTheta();
	veriler.dataYapi.pitch = (uint8_t)((int)abs(euler[0]));
	veriler.dataYapi.roll = (uint8_t)((int)abs(euler[1]));
	veriler.dataYapi.yaw = (uint8_t)((int)abs(euler[2]));

	veriler.dataYapi.checkSum = calculateCRC();
	veriler.dataYapi.CR	= '\r';
	veriler.dataYapi.LF	= '\n';

	return veriler.arr;

#ifdef PRINT_DECODED
	if(power_s->voltage > LOW_BAT){
		//lora_activate();
		//sendRF();
		//sendPC();
	}
	else{
		//sendPC();
		//lora_deactivate();
	}
#endif
#ifndef PRINT_DECODED
	printDatas();
#endif
}

void printDatas()
{

	  sprintf((char*)bufferPrint, "\r\n\n\n\r%X min: %d  sec: %d  stat: %d  volt: %.2f  mWatt/s: %d  temp: %.1f\r\n"
			  "hum:%d  alt:%.1f maxAlt:%d altGps: %.1f  lat: %f  lon: %f\r\n"
			  "angle:%.1f  pitch:%d  roll:%d  yaw:%d | real pitch: %.1f  roll: %.1f  yaw: %.1f\r\n"
			  "sat:%d  velocity:%.1f  ivmeX:%.3f  ivmeY:%.3f  ivmeZ:%.3f CRC:%d", veriler.dataYapi.basla,
			  (veriler.dataYapi.zaman >> 2), ((veriler.dataYapi.zaman & 3) << 4) | (veriler.dataYapi.durum >> 4),
			  veriler.dataYapi.durum & 0x0F, ((float)veriler.dataYapi.voltaj) / 100 ,
			  veriler.dataYapi.akim , (float)veriler.dataYapi.sicaklik / 3, veriler.dataYapi.nem,
			  veriler.dataYapi.yukseklik_p, (int)veriler.dataYapi.maxAltitude, veriler.dataYapi.yukseklik_gps, veriler.dataYapi.lat,
			  veriler.dataYapi.lon, veriler.dataYapi.aci, (veriler.dataYapi.pitch * ((veriler.dataYapi.uyduSayisi & 0x04)?-1:1)),
			  (veriler.dataYapi.roll * (((veriler.dataYapi.uyduSayisi & 0x02)?-1:1))),
			  (veriler.dataYapi.yaw * ((veriler.dataYapi.uyduSayisi & 0x01)?-1:1)),
			  euler[0], euler[1], euler[2],
			  veriler.dataYapi.uyduSayisi >> 3, (float)veriler.dataYapi.hiz / 10,
			  veriler.dataYapi.accX, veriler.dataYapi.accY,
			  veriler.dataYapi.accZ, veriler.dataYapi.checkSum);

	  HAL_UART_Transmit(&TTL_HNDLR, bufferPrint , strlen((char*)bufferPrint), 50);
}
