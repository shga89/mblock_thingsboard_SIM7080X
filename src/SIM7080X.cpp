#include "SIM7080X.h"



SIM7080X::SIM7080X(){
}

SIM7080X::~SIM7080X(){
}


void SIM7080X::init(Stream* espSerial)
{
    Serial.println("Initializing SIM7080X module");
	SIM_Serial = espSerial;
}

/**************************Power on Sim7x00**************************/
void SIM7080X::PowerOn(int PowerKey = powerkey){
   uint8_t answer = 0;

sendATcommand("AT+CFUN=6", "OK", 2000);

  // 이미 모듈이 시작되었는지 확인
  answer = sendATcommand("AT", "OK", 2000);
  if (answer == 0)
  {
    Serial.println("[SIM] Starting up...");
    
    pinMode(PowerKey, OUTPUT);
    // power on pulse
    digitalWrite(PowerKey, HIGH);
    delay(500);
    digitalWrite(PowerKey, LOW);
    
    // waits for an answer from the module
    while (answer == 0) {     // Send AT every two seconds and wait for the answer
      answer = sendATcommand("AT", "OK", 2000);
    }
  }
}

bool SIM7080X::connectCellular(unsigned int timeout=5000) {
 
  Serial.println("[SIM] Checking the registration with the carrier...");
  while ((sendATcommand("AT+CREG?", "+CREG: 0,1", 500) || sendATcommand("AT+CREG?", "+CREG: 0,5", 500)) == 0)
  delay(500);
  Serial.println("[SIM] Connection is established.");


 uint8_t x=0,  answer=0;
    char response[100];
    unsigned long previous;
	memset(response, '\0', 100);    // Initialize the string
	
	previous = millis();
	if (sendATcommand("AT+CNACT=0,1", "OK", 2000)) {
		if (sendATcommand("AT+CNACT?", "+CNACT: 0,1,", 2000)){ // 네트워크 IP받아오기
			Serial.print("[SIM] IP Address: ");
			SIM_Serial->read();   //" 한글자 버리고,
			do{
				if(SIM_Serial->available() != 0){    
					response[x] = SIM_Serial->read();      
					x++;
					if (response[x-1]=='"') {response[x-1]='\0'; answer=1;}
				}
				 // Waits for the asnwer with time out
			}while((answer == 0) && ((millis() - previous) < timeout));
			Serial.println(response);
		}
		else {
			Serial.println("[SIM] Reading IP Address is failed!");
			return false;
		}
	} else {
		Serial.println("[SIM] Getting IP Address is failed!");
		return false;
	}
	delay(1000);
	return true;
}

bool SIM7080X::connectServer(const char* host, uint16_t port, const char* token, unsigned int timeout=5000, unsigned int keeptime=60) {
	char cmdBuf[50];
	sprintf_P(cmdBuf, PSTR("AT+SMCONF=\"URL\",\"%s\",\"%d\""), host, port);
	sendATcommand(cmdBuf, "OK", 1000);
	sprintf_P(cmdBuf, PSTR("AT+SMCONF=\"USERNAME\",\"%s\""), token);
	sendATcommand(cmdBuf, "OK", 1000);
	sprintf_P(cmdBuf, PSTR("AT+SMCONF=\"KEEPTIME\",%d"), keeptime);
	sendATcommand(cmdBuf, "OK", 1000);
	
	if (sendATcommand("AT+SMCONN", "OK", timeout)) {
		Serial.println("[SIM] Server has been connected!");
		delay(1000);
		return true;
	} else {
		Serial.println("[SIM] Failed to connect the server!");
		return false;
	}
}

bool SIM7080X::isServer(unsigned int timeout = 1000){
	if (sendATcommand("AT+SMSTATE?", "+SMSTATE: 1", timeout)) return true;
	else return false;
}

bool SIM7080X::sendDataJson(const char* value, unsigned int timeout = 5000) {
	char cmdBuf[150];
	sprintf_P(cmdBuf,PSTR("AT+SMPUB=\"v1/devices/me/telemetry\",%d,1,1\r\n"),strlen(value));
	if (sendATcommand(cmdBuf, ">", timeout)) {
		if (sendATcommand(value,"OK",timeout)) {
			Serial.print("[SIM] Sending is completed : ");
			Serial.println(value);
			return true;
		} else {
			Serial.print("[SIM] JSON Sending is failed!");
			return false;
		}
	} else {
		Serial.print("[SIM] Publish trying is failed!");
		return false;
	}
}

/**************************GPS positoning**************************/
bool SIM7080X::GPSPositioning(){
  // 아직 미개발상태
  return true;
}

/**************************Other functions**************************/
uint8_t SIM7080X::sendATcommand(const char* ATcommand, const char* expected_answer, unsigned int timeout) {

    uint8_t x=0,  answer=0;
    char response[100];
    unsigned long previous;

    memset(response, '\0', 100);    // Initialize the string
	delay(50);
    cleanBuf();
    SIM_Serial->println(ATcommand);    // Send the AT command

    x = 0;
    previous = millis();

    // this loop waits for the answer
    do{
        if(SIM_Serial->available() != 0){    
            response[x] = SIM_Serial->read();      
            x++;
            // check if the desired answer  is in the response of the module
			  
            if (strstr(response, expected_answer) != NULL){
                answer = 1;
            }
        }
         // Waits for the asnwer with time out
    }while((answer == 0) && ((millis() - previous) < timeout));
	//Serial.println(response);//Debug
    return answer;
}


void SIM7080X::cleanBuf() {
	while(SIM_Serial->available() > 0) SIM_Serial->read();    // Clean the input buffer
}