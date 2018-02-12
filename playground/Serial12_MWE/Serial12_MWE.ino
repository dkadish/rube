String WifiRequest = "";

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  Serial1.begin(115200);

  delay(2000);
}

void loop() {
  // put your main code here, to run repeatedly:
  serial_loop();
}

void wifiResponse(char* response)
{
    Serial.printf("%s\n", response);
    Serial1.printf("B%s\n", response);
}

void serial_loop(){
    if (Serial.available())
    {
        char character = Serial.read();
        if (character != '\n') {
            WifiRequest.concat(character);
        } else {
            char resp[100];
            WifiRequest.toCharArray(resp,100);
            wifiResponse(resp);

            WifiRequest = "";
        }
    }
    
    if (Serial1.available())
    {
        char character = Serial1.read();
        if (character != '\n') {
            WifiRequest.concat(character);
        } else {
            char resp[100];
            WifiRequest.toCharArray(resp,100);
            wifiResponse(resp);

            WifiRequest = "";
        }
    }
}
