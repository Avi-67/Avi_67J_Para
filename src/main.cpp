#include<main.h>

hw_timer_t *timer = NULL;
IRAM_ATTR void tack()
{
  triger = 1;
}


void setup()
{
  delay(1000);
  Serial.begin(9600);
  Serial2.begin(9600, SERIAL_8N1, RX_PIN, TX_PIN);
  //Serial2.print("Hello, world!");
  while(!Serial2.availableForWrite());
  SPIC1.begin(VSPI, SCK1, MISO1, MOSI1);
  lps.begin(&SPIC1, LPSCS, 6000000);
  icm.begin(&SPIC1, ICMCS, 6000000);
  flash.begin(&SPIC1, SPIFLASHCS, 6000000);
  myservo.attach(SERVO_PIN);
  myservo1.attach(SERVO1_PIN);
  pinMode(LED,OUTPUT); 
  timer = timerBegin(0, 80, true); // timer=1us
  timerAttachInterrupt(timer, &tack, true);
  timerAlarmWrite(timer, 1000, true); // 1us * 1e5 = 10ms -> 100Hz
  timerAlarmEnable(timer);
  //project.flasherase();
  //Serial2.print("Hello, world!");
  //put your setup code here, to run once:
}

void loop() {
	char uartData[1];
	if (triger != 0){
		flag++;
		//Serial2.print(flag);
		flag_40++;
		//Serial2.print(flag_40);
		counter++;
		//Serial2.print(counter);
		//Serial2.println(micros());//to check time to finish a cycle of "loop function".
		count_lps++;
		if(mode == 2){
			//time = micros();
			//project.flashwrite();//write data to SPIFlash
			if(k == 16){ //if 1 page is filled
				project.flashwrite();//write data to SPIFlash
				addr += 0x100; k = 0; num_used_pages++;
				//if(num_used_pages == num_use_pages){ //if data is filled
				//Serial2.print('z'); //about 17min10
				//}
			}

			if(flag > 0){ //ICM20948 logging, 1ms
				icm.Get(ICM_data);
				uint32_t a = ICM_data[0] * 16 / 32767;
				uint32_t b = ICM_data[1] * 16 / 32767;	
				uint32_t c = ICM_data[2] * 16 / 32767;
				// Serial2.print(a);
				// Serial2.print(",");
				// Serial2.print(b);
				// Serial2.print(",");
				// Serial2.print(c);
				//Serial2.print(",");
				// Serial.print(ICM_data[3]);
				// Serial.print(",");
				// Serial.print(ICM_data[4]);
				// Serial.print(",");
				// Serial.print(ICM_data[5]);
				k++;
				project.convert_ICM20948_buf(k);
				if(launched == 0){
					project.launch_check_2G(); //launch check by ICM
				}
				if(launched == 1){
					//Serial2.print("Hello, world!!");
					// if(descent == 0){
					// 	project.vertex_check_t();//vertex check by timer        				}
					// 	if(descent ==1){
					// 		myservo.write(90);
					// 		servo_status = 1; //90 degree rotation
					// 		//Serial2.print('x');
					// 		descent = 2;
					// 	}
					// 	if(descent == 2){
					// 		project.vertex1_check_t();//second vertex check by timer
					// 		if(descent1 == 1){
					// 			myservo1.write(0);
					// 			servo1_status = 1;
					// 			//Serial2.print("x1");
					// 		}
					// 	}		
				    //}
				}
			flag = 0;
			}

			if(flag_40 > 39){ //LPS25HB logging, 40ms
				uint8_t LPS25_data[3];
				lps.Get(LPS25_data);

				// Serial.print(LPS25_data[0]);
				// Serial.print(",");
				// Serial.print(LPS25_data[1]);
				// Serial.print(",");
				// Serial.print(LPS25_data[2]);
				// Serial.print(",");
				uint32_t Pressure = (LPS25_data[0]+LPS25_data[1]*256+LPS25_data[2]*65536)*100/4096;
				//Serial2.println(Pressure);
				l++;
				project.convert_LPS25HB_buf(l);
				if(launched == 0){ 
					project.launch_check_P(); //launch check by LPS
				}
				if(launched == 1){
					Serial2.print("launched = 1");
					if(descent == 0){
						project.vertex_check_P(); //vertex check by LPS
					}else if(descent == 1){
						Serial2.print("descent = 1");
						myservo.write(90);
						servo_status = 1; //90 degree rotation
						//Serial2.print('x');
						descent = 2;				
					}
				}
			flag_40 = 0;			
			}
			//triger = 0;
			//Serial2.print(time);
		}
	triger = 0;
	}

	if(Serial2.available()){ //if data comes from uart2
		uartData[0] = Serial2.read();

		if(uartData[0] == 's'){
			mode = 0;
			Serial2.print('s');
		}

		if(mode == 0){ //default mode //for rotation of servo during rocket assembly
			if(uartData[0] == 'p'){
				mode = 1;
				Serial2.print('p');
				Serial2.print("mode = 1");
				digitalWrite(LED,HIGH);
				delay(1000);
				digitalWrite(LED,LOW);
			}else if(uartData[0] == 't'){
				top = 1;
			}else if(uartData[0] == 'b'){
				bottom = 1;
			}else if(uartData[0] == 'u'){
				digitalWrite(LED,HIGH);
				delay(1000);
				digitalWrite(LED,LOW);
			}
			if(top == 1){
				if(uartData[0] == 'c'){
					Serial2.print('c');
					myservo.write(55); //open
  					servo_status = 1;
					pinMode(SERVO_PIN,LOW);
					top = 0;
				}else if(uartData[0] == 'o'){
					Serial2.print('o');
					myservo.write(0); //close
					servo_status = 0;
					delay(2000);
					pinMode(SERVO_PIN,LOW);
					top = 0;
				}
			}
			if(bottom == 1){
				if(uartData[0] == 'c'){
					Serial2.print('c');
					myservo1.write(60); //open
       				servo1_status = 1;
					delay(1000);
					pinMode(SERVO1_PIN,LOW);
					bottom = 0;
				}else if(uartData[0] == 'o'){
					Serial2.print('o');
					myservo1.write(0); //close
					delay(1000);
					pinMode(SERVO1_PIN,LOW);
					bottom = 0;				
				}
			}	
		}else if (mode == 1){ //preparation mode
			switch(uartData[0]){
				case 'd':
					mode = 3;
					Serial2.print('d');
					Serial2.print("mode = 3");
					break;
				case 'l':
	  				mode = 2;
					flag = 0;
					flag_40 = 0;
	  				Serial2.print('l');
					Serial2.print("mode = 2");	
					// digitalWrite(LED,HIGH);
					// delay(1000);
					// digitalWrite(LED,LOW);
					break;
				case 'r':
					mode = 4;
					Serial2.print("mode = 4");
				default:
					// mode = 0;
					// Serial.print("mode = 0");
					break;
			}
		}else if(mode == 3){ //delete mode
			counter = 0; flag = 0; flag_40 = 0;
			addr = 0x00;
			num_used_pages = 0; k = 0; l = 0;
			launched = 0;
			flag_LPSdata_send_later = 0;
			//Serial2.println("default");
			project.set_variables_default();
			//Serial2.print("erase start");
			project.flasherase();
			//Serial2.print("erase fin");
			mode = 0;
			digitalWrite(LED,HIGH);
			delay(5000);
			digitalWrite(LED,LOW);
		}
		// else if(mode == 4){
		// 	project.flashread();
		// }
	}
  // if (Serial2.available()){

  //   char receive;

  //   receive = Serial2.read();
  //   Serial.write(receive);
  //   if (receive == 'e')
  //   {
  //     Serial2.print("OK");
  //     digitalWrite(32,HIGH);
  //     delay(100);
  //     digitalWrite(32,LOW);
  //     delay(100);
  //   }
  // }

  // uint8_t a; 
  // a = lps.WhoAmI();
  // Serial.print(a);
  // delay(100);

  // uint8_t b; 
  // Serial.print("Hello World!");
  // b = icm.WhoAmI();
  // Serial.print(b);
  // delay(100);

  // myservo.write(0);
  // delay(2000);
  // myservo.write(90);
  // delay(2000);

  // myservo1.write(0);
  // delay(2000);
  // myservo1.write(90);
  // delay(2000);

  // Serial.print("HIGH");
  // digitalWrite(LED,HIGH);
  // delay(2500);

  // Serial.print("LOW");
  // digitalWrite(LED,LOW);
  // delay(2500);

  //put your main code here, to run repeatedly:
}

