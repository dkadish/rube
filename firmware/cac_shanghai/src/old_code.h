//recieve the emergency stop command and enforce it on the motors
#include <Arduino.h>
#include <Encoder.h>
#define BTSerial Serial1

int pinSpdA=3;
int pinDirA1=8;
int pinDirA2=7;
int butA=22;

int pinSpdB=4;
int pinDirB1=10;
int pinDirB2=9;
int butB=21;

int pinSpdC=5;
int pinDirC1=12;
int pinDirC2=11;
int butC=20;

Encoder encA(29,28);
Encoder encB(30,27);
Encoder encC(26,31);

int currentDirA=1;
int currentDirB=1;
int currentDirC=1;
int posA=0;
int posB=0;
int posC=0;
//int posAS=0;
//int posBS=0;
//int posCS=0;
int cntMove=0;
int cnt=0;
int maxSpeed=10; //10 encoder units per 100ms=100 encoder units per second or 1/10ms or .1/1ms
long oldPosition[]={-999,-999,-999};
long posNow[]={0,0,0};
long time [] ={0,0,0};
//____________________________MeasureValues
//HEIGHT(s)
float height=3166;//16000; //currently the rube must be mounted so that the lineLengths are in the same height.
//DISTANCES BETWEEN TOP PTS
float distOP= 3850;//ljusg15570;//crf628;//origo -> første  (o) O------P  (x)
float distPQ= 3752;//ljusg15980;//crf600;//første til anden      \  R /
float distOQ= 3300;//ljusg25640;//crf437;//anden til origo        \  /
//DISTANCES TO RUBE                                          (y)   Q
float distOR =2600;//20750;
float distPR =2870;//16450; //1444 er højden fra gulv sjuss
float distQR =2456;//21520;
//____________________________MeasureValues
int lineO=distOR;
int lineP=distPR;
int lineQ=distQR;
long Qx,Qy,Qz;
float gox=0;
float goy=0;
float goz=0;
int stopCmd=0;
//PID variables
float Pu =2;
float Iu =1;
float Du =0.5;

float Pd =0.2;
float Id =0.1;
float Dd =0.5;

//float pointOx,pointOy,pointOz,pointPx,pointPy,pointPz,pointRx,pointRy,pointRz;
//PVector pointO, pointP, pointQ, pointR;
/*CLASS*//////////////////////////////////////////////////////
class PID{
public: //public otherwise error: test::test private
    int M; // a variable
    unsigned long lastTime;
    double Input;
    double Output;
    double Setpoint;
    double ITerm, lastInput;
    double kp, ki, kd,kup,kui,kud,kdp,kdi,kdd;
    int SampleTime = 20; //1 sec
    double outMin, outMax;

    PID(int motor){ //the constructor
        M=motor;
        lastTime=0;
        Input=0;
        Output=0;
        Setpoint=0;
        ITerm=0;
        lastInput=0;

        kp=0;
        ki=1;
        kd=2;

        kup=0;
        kui=1;
        kud=2;

        kdp=0;
        kdi=1;
        kdd=2;

        SampleTime=20;
        outMin=-35;
        outMax=255;

    }

    void Compute(int SetPt)
    {  Setpoint=SetPt;
        boolean dir=true;
        unsigned long now = millis();
        int timeChange = (now - lastTime);
        if(timeChange>=SampleTime)
        {
            /*Compute all the working error variables*/
            double error =  Setpoint -posNow[M] ;
//NB here see what is actually up or down -change <to>
            if(error<0) {kp=kup;ki=kui;kd=kud;}
            else {kp=kdp;ki=kdi;kd=kdd;}

            ITerm+= (ki * error);
            if(ITerm> outMax) ITerm= outMax;
            else if(ITerm< outMin) ITerm= outMin;
            double dInput = (Input - lastInput);

            /*Compute PID Output*/
            Output = kp * error + ITerm- kd * dInput;
            //Output=Output*-1;
            if(Output > outMax) Output = outMax;
            else if(Output < outMin) Output = outMin;

            /*Remember some variables for next time*/
            lastInput = Input;
            lastTime = now;
        }
    }

    void SetTuningsDown(double Kp, double Ki, double Kd)
    {
        double SampleTimeInSec = ((double)SampleTime)/1000;
        kdp = Kp;
        kdi = Ki * SampleTimeInSec;
        kdd = Kd / SampleTimeInSec;
    }
    void SetTuningsUp(double Kp, double Ki, double Kd)
    {
        double SampleTimeInSec = ((double)SampleTime)/1000;
        kup = Kp;
        kui = Ki * SampleTimeInSec;
        kud = Kd / SampleTimeInSec;
    }

    void SetSampleTime(int NewSampleTime)
    {
        if (NewSampleTime > 0)
        {
            double ratio  = (double)NewSampleTime
                            / (double)SampleTime;
            ki *= ratio;
            kd /= ratio;
            SampleTime = (unsigned long)NewSampleTime;
        }
    }

    void SetOutputLimits(double Min, double Max)
    {
        if(Min > Max) return;
        outMin = Min;
        outMax = Max;

        if(Output > outMax) Output = outMax;
        else if(Output < outMin) Output = outMin;

        if(ITerm> outMax) ITerm= outMax;
        else if(ITerm< outMin) ITerm= outMin;
    }
};
PID pidA =PID(0);
PID pidB =PID(1);
PID pidC =PID(2);
int Rx=0;
int Ry=0;
int Rz=0;
int incomingByte = 0;
String stringToSend="";

void CalibratePosition() {
//first finding the XY position of point Q
    float r1=distOQ;
    float r2=distPQ;
    float d=distOP;
    Qx=((d*d)-(r2*r2)+(r1*r1))/(2*d);
    Qy=((1/d) * sqrt((-d+r2-r1)*(-d-r2+r1)*(-d+r2+r1)*(d+r2+r1)))/2;
    Qz =height;
    //now finding the pointR based on the three other lineLengths OPQ and the distances from these to pointR
    r1=distOR;
    r2=distPR;
    int r3=distQR;
    d=distOP;
    float i=Qx; //pointQ.x
    float j=Qy; //pointQ.y

    Rx=int(( ( (r1*r1) - (r2*r2) + (d*d) ) /  (2*d) ));
    Ry=int(( ( (r1*r1) - (r3*r3) + (i*i) + (j*j) ) / (2*j) ) - ( (i/j)*Rx )); // ((r1*r1)-(r3*r3)-(xc*xc)+( (xc-i)*(xc-i) )+(j*j) / (2*j));
    Rz=int( sqrt( (r1*r1) - (Rx*Rx) - (Ry*Ry) ))  ;
    Rz=height-Rz;
}
//_________________________________SETUP_________SETUP_________SETUP_________SETUP
//_________________________________SETUP_________SETUP_________SETUP_________SETUP
//_________________________________SETUP_________SETUP_________SETUP_________SETUP
//_________________________________SETUP_________SETUP_________SETUP_________SETUP
//_________________________________SETUP_________SETUP_________SETUP_________SETUP
void setup() {
    //pointR = new PVector(Rx, Ry, heightO-Rz); //now zc should be zero???
    //moving the lineLengths up to their height
    //%%%%%%%%%%%%%%%%%%%%%%%%%
    CalibratePosition();
    setAllPIDs();

    pidA.SetOutputLimits(55,-5);
    pidB.SetOutputLimits(55,-5);
    pidC.SetOutputLimits(55,-5);

    pidA.SetSampleTime(10);
    pidB.SetSampleTime(10);
    pidC.SetSampleTime(10);

    Serial.begin(9600);
    BTSerial.begin(9600);

    pinMode(pinSpdA,OUTPUT);
    pinMode(pinDirA1,OUTPUT);
    pinMode(pinDirA2,OUTPUT);
    pinMode(butA,INPUT);
    pinMode(butB,INPUT);
    pinMode(butC,INPUT);

    pinMode(pinSpdB,OUTPUT);
    pinMode(pinDirB1,OUTPUT);
    pinMode(pinDirB2,OUTPUT);
    pinMode(24,OUTPUT);

    pinMode(pinSpdC,OUTPUT);
    pinMode(pinDirC1,OUTPUT);
    pinMode(pinDirC2,OUTPUT);
    //Stop and set motor direction to start one way.
    analogWrite(pinSpdA,0);
    analogWrite(pinSpdB,0);
    analogWrite(pinSpdC,0);
    digitalWrite(pinDirA1,HIGH);
    digitalWrite(pinDirB1,HIGH);
    digitalWrite(pinDirC1,HIGH);
    digitalWrite(pinDirA2,LOW);
    digitalWrite(pinDirB2,LOW);
    digitalWrite(pinDirC2,LOW);

    tone(24,2000,500); //start tone
    delay(9);

    while(digitalRead(butA)==HIGH) {
        motor(0,190);
    }
    motor(0,0);
    motor(1,0);
    motor(2,0);
    while(digitalRead(butB)==HIGH) {
        motor(1,190);
    }
    motor(0,0);
    motor(1,0);
    motor(2,0);
    while(digitalRead(butC)==HIGH) {
        motor(2,190);
    }
    motor(0,0);
    motor(1,0);
    motor(2,0);
    tone(24,1000,200);
    delay(400);
    tone(24,3000,2000);
    delay(2200);
    tone(24,1000,200);
    delay(400);
    tone(24,3000,2000);
    CalculateLineOPQ(Rx,Ry,Rz);

    Serial.print("cal to Qx:"); Serial.println(Qx);
    Serial.print("cal to Qy:"); Serial.println(Qy);

    Serial.print("cal to Rx:"); Serial.println(Rx);
    Serial.print("cal to Ry:"); Serial.println(Ry);
    Serial.println("cal to Rz"); Serial.println(Rz);

    encA.write(lineO*2);
    encB.write(lineP*2);
    encC.write(lineQ*2);
}
///////////////////////////////////////////////////_LOOP///_LOOP///_LOOP///_LOOP
///////////////////////////////////////////////////_LOOP///_LOOP///_LOOP///_LOOP
///////////////////////////////////////////////////_LOOP///_LOOP///_LOOP///_LOOP
///////////////////////////////////////////////////_LOOP///_LOOP///_LOOP///_LOOP
void loop() {
    long goTime=millis();

    pidA.Compute(lineO*2);
    pidB.Compute(lineP*2);
    pidC.Compute(lineQ*2);

    //makeTone(19,50);
    posNow[0] = encA.read(); //probably starts in 0;
    posNow[1] = encB.read();
    posNow[2] = encC.read();

    // delay(9);
    //Serial.print("  PidA:    "); Serial.print(pidA.Output);
    //Serial.print("  nowPosX: "); Serial.print(Rx);
    //Serial.print("  nowPosY: "); Serial.print(Ry);
    //Serial.print("  nowPosZ: "); Serial.println(Rz);

    if(digitalRead(butA)==LOW&&digitalRead(butB)==LOW&&digitalRead(butC)==LOW)
    {
        motor(0,int(pidA.Output));
        motor(1,int(pidB.Output));
        motor(2,int(pidC.Output));
    }
    else{
        motor(0,1);
        motor(1,1);
        motor(2,1);

    }
    // Serial.print(".");
    // delay(6);
    //tone(24,1000,200);
// WHEN INCOMING do:
    if (BTSerial.available() > 0) {

        //Serial.println("Bluetooth signal ");
        int btPacketSize = 1+(4*3);
        while(BTSerial.available()>=btPacketSize) { //have we received a full data packet yet?
            tone(24,1000,10);
            char dataIdentifyer = (char)BTSerial.read();
            switch (dataIdentifyer) {
                case 'c': //coordinates
                    Rx = int(parseInt32());
                    Ry = int(parseInt32());
                    Rz = int(parseInt32());
                    break;
                case 'u': //pid-parameters up
                    Pu = float(parseInt32());Pu=Pu/1000;
                    Iu = float(parseInt32());Iu=Iu/1000;
                    Du = float(parseInt32());Du=Du/1000;
                    setAllPIDs();
                    break;
                case 'd': //pid-parameters down
                    Pd = float(parseInt32());Pd=Pd/1000;
                    Id = float(parseInt32());Pd=Pd/1000;
                    Dd = float(parseInt32());Pd=Pd/1000;
                    setAllPIDs();
                    break;
                case 's':
                    stopCmd=float(parseInt32());
                    break;
                case 'k':
                    height= float(parseInt32());//16000; //currently the rube must be mounted so that the lineLengths are in the same height.
                    distOP= float(parseInt32());
                    distPQ= float(parseInt32());
                    distOQ= float(parseInt32());
                    distOR =float(parseInt32());
                    distPR =float(parseInt32());
                    distQR =float(parseInt32());
            }
            //BTSerial.clear();
            //  Serial.print("received serial x: ");
            // Serial.println(Rx);
        } //END WHILE
    }//END SERIAL AVAILABLE
    else {BTSerial.clear();}
    CalculateLineOPQ(Rx,Ry,Rz);//give RUBE new go to XYZ here.

//delay(200);
    cnt++;
    if (cnt==200){
        cnt=0;
        delay(1);
        stringToSend+= Rx;//0
        stringToSend+= ",";
        stringToSend+= Ry;
        stringToSend+= ",";
        stringToSend+= Rz;
        stringToSend+= ",";
        stringToSend+= lineO;//3
        stringToSend+= ",";
        stringToSend+= lineP;
        stringToSend+= ",";
        stringToSend+= lineQ;
        stringToSend+= ",";
        stringToSend+= int(posNow[0]/2);
        stringToSend+= ",";
        stringToSend+= int(posNow[1]/2);
        stringToSend+= ",";
        stringToSend+= int(posNow[2]/2);
        stringToSend+= ",";
        stringToSend+= pidA.Output;//[9]
        stringToSend+= ",";
        stringToSend+= pidB.Output;
        stringToSend+= ",";
        stringToSend+= pidC.Output;
        stringToSend+= ",";
        if(digitalRead(butA)==HIGH) stringToSend+=2 ;
        else stringToSend+=1;
        stringToSend+= ",";
        if(digitalRead(butB)==HIGH) stringToSend+=2 ;
        else stringToSend+=1 ;
        stringToSend+= ",";
        if(digitalRead(butC)==HIGH) stringToSend+=2 ;
        else stringToSend+=1;//14 */
//stringToSend+= ",";stringToSend+= "666";
//stringToSend+= ",";stringToSend+= "666";stringToSend+= ",";stringToSend+= "666";stringToSend+= ",";stringToSend+= "666";
        stringToSend+='\t';
        BTSerial.print(stringToSend);
        BTSerial.println();
        stringToSend="";
    }

}////////////END MAIN LOOP
////////////////
void CalculateLineOPQ(int x, int y, int z){
    gox=x;
    goy=y;
    goz=z;
    float d= distOP;
    float i= Qx;
    float j= Qy;
    float h=height;
    lineO =int(sqrt((gox*gox)+(goy*goy)+((goz-h)*(goz-h))));
    lineP =int(sqrt(((gox-d)*(gox-d))+(goy*goy)+((goz-h)*(goz-h)))) ;
    lineQ =int(sqrt(((gox-i)*(gox-i))+((goy-j)*(goy-j))+((goz-h)*(goz-h)))) ;
}

long parseInt32(){
    char d1 = (char)BTSerial.read();
    char d2 = (char)BTSerial.read();
    char d3 = (char)BTSerial.read();
    char d4 = (char)BTSerial.read();

    return ((d1<<24) +(d2<<16) + (d3<<8) + d4);
}


void  setAllPIDs(){
    pidA.SetTuningsUp(Pu,Iu,Du);
    pidB.SetTuningsUp(Pu,Iu,Du);
    pidC.SetTuningsUp(Pu,Iu,Du);

    pidA.SetTuningsDown(Pd,Id,Dd);
    pidA.SetTuningsDown(Pd,Id,Dd);
    pidA.SetTuningsDown(Pd,Id,Dd);
}

void motor(int abc,int spd){
    switch(abc) {
        case 0:
            if (spd==0) { digitalWrite (pinDirA1,HIGH);digitalWrite (pinDirA2,HIGH);analogWrite(pinSpdA, abs(spd));}//analogWrite(pinSpdA, 200);}
            else if(spd>0){digitalWrite(pinDirA1,HIGH); digitalWrite (pinDirA2,LOW);analogWrite(pinSpdA, abs(spd));}
            else if(spd<0){digitalWrite(pinDirA1,LOW); digitalWrite (pinDirA2,HIGH);analogWrite(pinSpdA, abs(spd));}
            break;
        case 1:
            if (spd==0) { digitalWrite (pinDirB1,HIGH);digitalWrite (pinDirB2,HIGH);analogWrite(pinSpdB, abs(spd));}
            else if(spd>0){digitalWrite(pinDirB1,HIGH); digitalWrite (pinDirB2,LOW);analogWrite(pinSpdB, abs(spd));}
            else if(spd<0){digitalWrite(pinDirB1,LOW); digitalWrite (pinDirB2,HIGH);analogWrite(pinSpdB, abs(spd));}
            break;
        case 2:
            if (spd==0) { digitalWrite (pinDirC1,HIGH);digitalWrite (pinDirC2,HIGH);analogWrite(pinSpdC, abs(spd));}
            else if(spd>0){digitalWrite(pinDirC1,HIGH); digitalWrite (pinDirC2,LOW);analogWrite(pinSpdC, abs(spd));}
            else if(spd<0){digitalWrite(pinDirC1,LOW); digitalWrite (pinDirC2,HIGH);analogWrite(pinSpdC, abs(spd));}
            break;
        default:
            break;
    }
}
