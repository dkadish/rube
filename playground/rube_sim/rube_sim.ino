#include <elapsedMillis.h>

float kp_float, kv_float, ki_float;

void setup() {
  // put your setup code here, to run once:

  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);

  Serial.begin(9600);

  delay(2000);
}

void loop() {
  // put your main code here, to run repeatedly:

  print_loop();
  
  serial_loop();
}

String WifiRequest = "";

void wifiResponse(char* response)
{
    Serial.printf("B%s\n", response);
}

void serial_loop(){
    while (Serial.available())
    {
        char character = Serial.read();
        if (character != '\n')
            WifiRequest.concat(character);
        else
        {
            if (WifiRequest[0] == 'G'){
                int newSetPt = WifiRequest.substring(1).toInt();

                //A.pos_setpt = (float)newSetPt;
            }
            else if (WifiRequest[0] == 'S'){
                /*A.control_mode = POSITION;
                A.motor.brake();

                A.pos_setpt = A.current_position();*/
            }
            else if (WifiRequest[0] == 'D'){
                double newSpeed = ((double)WifiRequest.substring(1).toInt()) / 1000.0;
                /*A.control_mode = SPEED;

                A.pos_out = newSpeed;*/
            }
            else if (WifiRequest[0] == 'A'){
                int newSetPt = WifiRequest.substring(1).toInt();
                //NextPoint1 = newSetPt;
            }
            else if (WifiRequest[0] == 'H'){
                //A.pos_setpt = (float)NextPoint1;
            }
            else if (WifiRequest[0] == 'P'){
                int kp = WifiRequest.substring(1).toInt();
                float kp_float = ((float)kp)/100.0;
                Serial.printf("Setting Kp to %i.%i\n", (int)kp_float, ((int)(kp_float*100.0))%100);
                //A.position.SetTunings(kp_float, 0.0, 0.0);
            }
            else if (WifiRequest[0] == 'V'){
                int kv = WifiRequest.substring(1).toInt();
                float kv_float = ((float)kv)/100.0;
                Serial.printf("Setting Kv to %i.%i\n", (int)kv_float, ((int)(kv_float*100.0))%100);
                //A.speed.SetTunings(kv_float, A.speed.GetKi(), 0.0);
            }
            else if (WifiRequest[0] == 'I'){
                int ki = WifiRequest.substring(1).toInt();
                float ki_float = ((float)ki)/100.0;
                Serial.printf("Setting Ki to %i.%i\n", (int)ki_float, ((int)(ki_float*100.0))%100);
                //A.speed.SetTunings(A.speed.GetKp(), ki_float, 0.0);
            }
            else if (WifiRequest[0] == 'K'){
                Serial.printf("Param > Kp: %i.%i, Kv: %i.%i, Ki: %i.%i\n",
                    
                    (int)kp_float, ((int)(kp_float*100.0))%100,
                    (int)kv_float, ((int)(kv_float*100.0))%100,
                    (int)ki_float, ((int)(ki_float*100.0))%100
                );
            }
            else if (WifiRequest == "ON")
            {
                wifiResponse("Stuff turned on");
                digitalWrite(LED_BUILTIN, HIGH);
            }
            else if (WifiRequest == "OFF")
            {
                wifiResponse("Stuff turned off");
                digitalWrite(LED_BUILTIN, LOW);
            }
            else
            {
                wifiResponse("Unknown command");
            }

            WifiRequest = "";
        }
    }
}
