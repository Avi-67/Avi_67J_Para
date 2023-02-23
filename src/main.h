#pragma once

#ifndef PROJECT_H
#define PROJECT_H
#include <SPICREATE.h>
#include <Arduino.h>


#include <Arduino.h>
#include <SPICREATE.h>
#include <LPS25HB.h>
#include <ICM20948.h>
#include <S25FL512S.h>
#include <ESP32Servo.h>
#include <SPIflash.h>

#define LPSCS 13
#define LED 32
#define ICMCS 14
#define SPIFLASHCS 21
#define SERVO1_PIN 22
#define SERVO_PIN 23
#define SCK1 33
#define MISO1 25
#define MOSI1 26
#define RX_PIN 16
#define TX_PIN 17

	uint32_t counter = 0;
	uint32_t flag = 0, flag_40 = 0, triger = 0;
	uint32_t t_launch = 0, t_vertex = 0, t_vertex1 = 0;
	uint32_t t_open = 14, t_open1 = 16;
	uint32_t t_openwait = 0;
	uint8_t servo_status = 0, servo1_status = 0;
	uint8_t LaunchCount =0, count = 0, countup = 0;
	int16_t average0 =0, average1 = 0, average2 = 0;
	int32_t sum0 = 0, sum1 = 0, sum2=0;
	uint8_t difference_Pressure_count = 0, sum_Pressure_count = 0, log_count = 0;
	int32_t Pressure, sum_Pressure = 0, ave_Pressure = 0, ave_Pressure_old = 0;
	uint8_t launched = 0, descent = 0,/*servo.No1*/ descent1 = 0;/*servo.No2*/
	int16_t ICM_data[6], LPS25_data[3];
	uint8_t icm_time_tx_buf[256], lps_time_tx_buf[256],flash_buf[256],flash_rx[256],flash_addr = 0x000;
	uint8_t rx_buf[256], k = 1, l = 1, j = 0, mode = 0,flag_LPSdata_send_later = 0, servo_mode = 0, top = 0, bottom =0;
	uint32_t addr = 0x00, time = 0;
	uint16_t length_ = 0x100, num_used_pages = 0, num_use_pages = 65535; /*change true =65535, experiment = 625*x(about 10*x second) */ //SPIFlash capacity

class PROJECT{

public:
    void set_variables_default();
    void launch_check_P();
	void vertex_check_P();// servo.No1
	void vertex1_check_P();// servo.No2
    void launch_check_2G();
	void vertex_check_t();// servo.No1
	void vertex1_check_t();// servo.No2
    void convert_ICM20948_buf(uint8_t k);
	void convert_LPS25HB_buf(uint8_t l);
	void flashread();
	void flasherase();
	void flashwrite();
	void data_debug_print();
	IRAM_ATTR void test();
	IRAM_ATTR void tack();

    unsigned long Gettime_record()
  	{
    	time = micros();
    	time -= start_time;
    	return time;
  	}
  	unsigned long start_time;
  	unsigned long time;
  	bool start_flag = true;
};

// 時間
unsigned long Record_time;

// チェッカー(logging関数でこれを動かす)
uint8_t checker = 0;

// Serial2で受け取るchar型の変数
char receive;

// Serial2で使う
bool exitLoop = false;

// class Timer
// {
// public:
//   unsigned long Gettime_record()
//   {
//     time = micros();
//     time -= start_time;
//     return time;
//   }
//   unsigned long start_time;
//   unsigned long time;
//   bool start_flag = true;
// };

// 気圧の回数の測定(5回に1回)
uint32_t count_lps = 0;

// CountSPIFlashDataSetExistInBuffは列
int CountSPIFlashDataSetExistInBuff = 0;

// SPI_FlashBuffは送る配列
uint8_t SPI_FlashBuff[256] = {};

// SPIFlashLatestAddressは書き込むアドレス。初期値は0x000
// 0x000はreboot対策のどこまでSPI Flashに書き込んだかを記録するページ
// setup()で初期値でも0x100にしている
uint32_t SPIFlashLatestAddress = 0x000;

// SPI Flashの最大のアドレス (1回で1/2ページ書き込んでいる点に注意)
// (512 * 1024 * 1024 / 8 / 256 ページ * 256) * 2 = 524288 * 256
uint32_t SPI_FLASH_MAX_ADDRESS = 0x8000000;

//#define SPIFREQ 5000000

// #define loggingPeriod 2
#define loggingPeriod2 1

TimerHandle_t thand_test;
xTaskHandle xlogHandle;

Flash flash;
//Timer times;
SPICREATE::SPICreate SPIC1;
LPS lps;
ICM icm;
Servo myservo;
Servo myservo1;
PROJECT project;

void PROJECT::convert_ICM20948_buf(uint8_t k){ //ICMdata convert & store
	for(uint8_t i = 0; i < 6; i++){  //{12(=6axis data) + 4(=time data)} * 16 = 256
		icm_time_tx_buf[16*(k-1) + 2*i+1] 	= (ICM_data[i] >> 8);
	    icm_time_tx_buf[16*(k-1) + 2*i] 		= (ICM_data[i] );
	}
	for(uint8_t i = 0; i < 4; i++){
		icm_time_tx_buf[16*(k-1)+ 12 + i] 	= ((counter >> (8 * i)) & 0xFF);
	}
}

void PROJECT::convert_LPS25HB_buf(uint8_t l){ 	//LPSdata convert & store
	// for(uint8_t i = 0; i < 3; i++){ 	//{3(=pressure data) + 9(=empty data)} * 4(time data)} * 16 =256
	// 	lps_time_tx_buf[16*(l-1) + i] 	= ( LPS25_data[i] >> 8);
	// }
	// for(uint8_t i = 3; i < 12; i++){
	// 	lps_time_tx_buf[16*(l-1) + i] = 0xFF; 
	// }
	// for(uint8_t i = 12; i < 16; i++){
	// 	lps_time_tx_buf[16*(l-1) + i] = ((counter >> (8 * i)) & 0xFF);
	// }
	if(k % 40 == 0){
		for(uint8_t i =0; i< 4; i++){
    		lps_time_tx_buf[16*k + i] = ((counter >> (8 * i)) & 0xFF) % 256;
  		}
	}
}

// void PROJECT::flashread(){
// 	flash.read(uint32_t addr, uint8_t *rx);
// }

void PROJECT::flasherase(){
    flash.erase();
}

void PROJECT::flashwrite(){
	if(log_count >= 8){
    	flash.write(flash_addr,flash_buf);
    	flash_addr += 0x100;
    	log_count =0;
	}
}
//   if (SPIFlashLatestAddress >= SPI_FLASH_MAX_ADDRESS)
//   {
//     Serial.printf("SPIFlashLatestAddress: %u\n", SPIFlashLatestAddress);
//     Serial2.write("SPI Flash is full");
//     Serial2.write("Started At: ");
//     Serial2.write(project.start_time);
//     Serial2.write("Now: ");
//     Serial2.write(project.Gettime_record());
//     return;
//   }
//   // Serial.println("Running");
//   if (project.start_flag)
//   {
//     project.start_time = micros();
//     project.start_flag = false;
//   }
//   Record_time = project.Gettime_record();// CountSPIFlashDataSetExistInBuffは列。indexは行。
//   // From SPI, Get data is tx
// //   int16_t H3lisReceiveData[3] = {};
// //   uint8_t H3lis_rx_buf[6] = {};
//   int16_t Icm20948ReceiveData[6] = {};
//   uint8_t Icm20948_rx_buf[12] = {};
//   uint8_t lps_rx[3] = {};

//   for (int index = 0; index < 4; index++)
//   {
//     SPI_FlashBuff[32 * CountSPIFlashDataSetExistInBuff + index] = 0xFF & (Record_time >> (8 * index));
//   }

//   icm.Get(ICM_data);

//   for (int index = 10; index < 16; index++)// ICM20948の加速度をとる
//   {
//     SPI_FlashBuff[32 * CountSPIFlashDataSetExistInBuff + index] = Icm20948_rx_buf[index - 10];
//   }

//   for (int index = 16; index < 22; index++)// ICM20948の角速度をとる
//   {
//     SPI_FlashBuff[32 * CountSPIFlashDataSetExistInBuff + index] = Icm20948_rx_buf[index - 10];
//   }

//   if ( flag_40 % 40 == 0)// LPSの気圧をとる
//   {
// 	lps.Get(lps_rx);
//     for (int index = 28; index < 31; index++)
//     {
//       SPI_FlashBuff[32 * CountSPIFlashDataSetExistInBuff + index] = lps_rx[index - 28];
//       flag_40 = 0;
//     }
//   }
// }

void PROJECT::launch_check_2G(){//Launch Confirmation Condition No.1 timer, call cycle 1ms 
	if(LaunchCount<20){
		sum0 += ICM_data[0]; sum1 += ICM_data[1]; sum2 += ICM_data[2];
		LaunchCount++;
	}
	if(LaunchCount >= 20){
		average0 = sum0 / 20; average1 = sum1 / 20; average2 = sum2 / 20;
		if(countup < 4 /*50*/){
			if(average0*average0 + average1*average1 + average2*average2 > 32767*32767/ 64){/*change true =64, experiment = 240*/
				countup++;
			}else{
				countup = 0;
			}
		  	LaunchCount = 0;
		  	sum0 = 0,sum1 = 0,sum2 = 0;
		}
		if(countup >= 4 /*50*/){/*change true = 50, experiment = 10 */ //detection complete
			launched = 1;
			t_launch = counter - 100000; //set the launch time to 1 second ago
			for(uint8_t i = 0; i < 4; i++)
            {
				lps_time_tx_buf[7 + i] = ((t_launch >> (8 * i)) & 0xFF); //store launched detection time
			}
			lps_time_tx_buf[11] = 0x01; //launch_check_2G number
		}
	}
}


void PROJECT::launch_check_P(){ //launch confirmation condition No.2 pressure, call cycle 20ms
	uint8_t LPS25_data[3];
	lps.Get(LPS25_data);
	uint32_t Pressure = (LPS25_data[0]+LPS25_data[1]*256+LPS25_data[2]*65536)*100/4096;
	sum_Pressure += Pressure;
	sum_Pressure_count ++;
	if(sum_Pressure_count >= 10){/*change true = 5, experiment = 20*/
		ave_Pressure_old = ave_Pressure ;
		ave_Pressure = sum_Pressure * 10 / sum_Pressure_count;
		Serial2.print("ave_Pressure * 10 = ");
		Serial2.println(ave_Pressure);
		sum_Pressure = 0; sum_Pressure_count = 0;
		if((ave_Pressure_old - ave_Pressure) > 5){ /*change true = 10, experiment = 0.5*/ //0.1hPa over
			difference_Pressure_count++;
		} else {
			difference_Pressure_count = 0;
		}
	}
	if(difference_Pressure_count >= 5){/*change true = 10, experiment = 3*/ //detection complete
		launched = 1;
		t_launch = counter - 100000; //set the launch time to 1 second ago
		sum_Pressure = 0; sum_Pressure_count = 0; difference_Pressure_count = 0;
		for(uint8_t i = 0; i < 4; i++){
			lps_time_tx_buf[7 + i] = ((t_launch >> (8 * i)) & 0xFF); //store launched detection time
		}
	lps_time_tx_buf[11] = 0x02; //launch_check_P number
	}
}

void PROJECT::vertex_check_t(){
	if((t_launch + t_open) <= counter){ // vertex confirmation condition No.1, timer
		descent = 1;
		t_vertex = counter;
		for(uint8_t i = 0; i < 4; i++){
			lps_time_tx_buf[7 + i] = ((t_vertex >> (8 * i)) & 0xFF);
		}
		lps_time_tx_buf[12] = 0x03; //vertex_check_t number
	}
}

void PROJECT::vertex_check_P(){ //vertex confirmation condition No.2 pressure, call cycle 20ms
	uint8_t LPS25_data[3];
	lps.Get(LPS25_data);
	uint32_t Pressure = (LPS25_data[0]+LPS25_data[1]*256+LPS25_data[2]*65536)*100/4096;
	sum_Pressure += Pressure;
	sum_Pressure_count ++;
	if(sum_Pressure_count >= 10){/*change true = 5, experiment = 20*/
		ave_Pressure_old = ave_Pressure;
		ave_Pressure = sum_Pressure * 10 / sum_Pressure_count;
		Serial2.print("Hello, world!");
		sum_Pressure = 0; sum_Pressure_count = 0;
		if(ave_Pressure > ave_Pressure_old){ //if this barometric pressure value is higher than the previous value.
			difference_Pressure_count++;
		} else {
			difference_Pressure_count = 0;
		}
	}
	if(difference_Pressure_count >= 5){/*change true = 10, experiment = 3*/ //detection complete
		descent = 1;
		t_vertex = counter;
		for(uint8_t i = 0; i < 4; i++){
			lps_time_tx_buf[7 + i] = ((t_vertex >> (8 * i)) & 0xFF); //store vertex detection time
		}
		lps_time_tx_buf[12] = 0x04;   //vertex_check_P number
	}
}

void PROJECT::vertex1_check_t(){
	if((t_launch + t_open1) <= counter){ // vertex1 confirmation condition No.1, timer
		descent1 = 1;
		t_vertex1 = counter;
		for(uint8_t i = 0; i < 4; i++){
			lps_time_tx_buf[7 + i] = ((t_vertex >> (8 * i)) & 0xFF);
		}
		lps_time_tx_buf[13] = 0x05; //vertex1_check_t number
	}
}

void PROJECT::set_variables_default(){
countup = 0;  LaunchCount = 0;
sum0 = 0; sum1 = 0; sum2 = 0;
average0 = 0; average1 = 0; average2 = 0;
difference_Pressure_count = 0; sum_Pressure_count = 0; sum_Pressure = 0;
ave_Pressure = 0; ave_Pressure_old = 0;
}

IRAM_ATTR void PROJECT:: test()
{
  digitalWrite(LED, HIGH);
  digitalWrite(LED, LOW);
}

#endif