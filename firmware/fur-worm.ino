SYSTEM_MODE(SEMI_AUTOMATIC);

#include "math.h"
// switch to time as a determinant instead of pressure

// servo definitions
int s0pin=D0;
int s1pin=D1;
int s2pin=D2;

int powerPin=A3;

int nervePins[3]={B4,A1,A4};
// faceL=A1; faceR=A4; faceT=B4;

/*
Potential pins for sensing include:
B2;
B3;
B4;
B5;
A0;
A1;
A2;
A3;
A4;
*/

int nerveReads[3];
int nerveReadsLast[3];

int nerveReadsAvg[3];


int petDiffOriginal[3]={10,10,10};
int painDiffOriginal[3]={20,20,20};

int petDiff[3]={10,10,10};
int painDiff[3]={20,20,20};

int petCounter[3]={0,0,0};
int painCounter[3]={0,0,0};

int soundPins[5]={D4,C3,C2,C1,C0};

Servo s0;   // head
Servo s1;   // mid
Servo s2;   // tail

// angle arrays show angle[0] as the current and angle[1] as the previous angle
int s0angle[2]={40,0};
int s1angle[2]={40,0};
int s2angle[2]={40,0};

int lastServo=0;

//random noise variables
int ampPerlin;  // amplitude
int servoPerlin;    // servo

int squeezed=120;   // the minimum before it is decided that you are squeezing it
int millisAtStateChange=0;

int state=0;

// // sound probability (out of 10000 (10 seconds))
// // if below, pin low. If above, pin high.
int soundProbability[5]={
    0,
    5000,
    8000,
    8000, // these don't count since they will be on always
    10000
};


int servoAvailability[3]={1,1,1};

// servo availability timers
Timer setServo0Timeout(50,set0Available,true);
Timer setServo1Timeout(50,set1Available,true);
Timer setServo2Timeout(50,set2Available,true);
Timer setCry1Timeout(100,silence1,true);
Timer setCry2Timeout(100,silence2,true);

void set0Available() {
  servoAvailability[0]=1;
}
void set1Available() {
  servoAvailability[1]=1;
}
void set2Available() {
  servoAvailability[2]=1;
}

void silence1() {
  setCry(1,0);
}

void silence2() {
  setCry(2,0);
}

void setup() {
    // activate all servos
    pinMode(s0pin,OUTPUT);
    pinMode(s1pin,OUTPUT);
    pinMode(s2pin,OUTPUT);
    s0.attach(s0pin);
    s1.attach(s1pin);
    s2.attach(s2pin);

    // power pin
    pinMode(powerPin,OUTPUT);
    digitalWrite(powerPin,HIGH);

    // speaking pins
    pinMode(soundPins[1],OUTPUT);
    pinMode(soundPins[2],OUTPUT);
    pinMode(soundPins[3],OUTPUT);
    pinMode(soundPins[4],OUTPUT);
    digitalWrite(soundPins[1],HIGH);
    digitalWrite(soundPins[2],HIGH);
    digitalWrite(soundPins[3],HIGH);
    digitalWrite(soundPins[4],HIGH);

    // call acceleration remotely
    Particle.function("a",accelerate);
    Particle.function("v",linear);

    // set up serial for debug
    Serial.begin(9600);

    // calibrate
    calibrate();

    // set to wakeup position
    startPos();

}

void loop() {

    readNerves();
    processImpulses(0);
    processImpulses(1);
    processImpulses(2);

    parseCounters(petCounter[0],painCounter[0],0);
    parseCounters(petCounter[1],painCounter[1],1);
    parseCounters(petCounter[2],painCounter[2],2);

    decayThreshold(0);
    decayThreshold(1);
    decayThreshold(2);

    // thinky-looking delay
    delay(10);

}

void calibrate() {

  nerveReadsAvg[0]=analogRead(nervePins[0]);
  nerveReadsAvg[1]=analogRead(nervePins[1]);
  nerveReadsAvg[2]=analogRead(nervePins[2]);
  // take reads for 3 seconds
  int timer=0;

   while (timer<1000) {
    nerveReadsAvg[0]=(nerveReadsAvg[0]+analogRead(nervePins[0]))/2;
    nerveReadsAvg[1]=(nerveReadsAvg[1]+analogRead(nervePins[1]))/2;
    nerveReadsAvg[2]=(nerveReadsAvg[2]+analogRead(nervePins[2]))/2;
    timer++;
  }

  Serial.println("-------------Calbirated-------------");
  Serial.println("avg: "+String(nerveReadsAvg[0])+","+String(nerveReadsAvg[1])+","+String(nerveReadsAvg[2]));
  delay(1000);

}

void decayThreshold(int p) {
  if (painDiff[p]>painDiffOriginal[p]*10) {
    painDiff[p]=floor(lowerLinear(painDiff[p],painDiffOriginal[p],1,0),2*painDiffOriginal[p]);
  }
  if (petDiff[p]>petDiffOriginal[p]*10) {
    petDiff[p]=floor(lowerLinear(petDiff[p],petDiffOriginal[p],1,0),2*petDiffOriginal[p]);
  }
}

void silenceCries() {
  setCry(0,0);
  setCry(1,0);
  setCry(2,0);
  setCry(3,0);
  setCry(4,0);
}

void readNerves() {

  // cleanup from last time:
  nerveReadsLast[0]=nerveReads[0];
  nerveReadsLast[1]=nerveReads[1];
  nerveReadsLast[2]=nerveReads[2];

  // read each nerve, put into an array representing the body:

  nerveReads[0]=readNerve(0);
  nerveReads[1]=readNerve(1);
  nerveReads[2]=readNerve(2);

  Serial.print(nerveReads[0]); Serial.print("  ");
  Serial.print(nerveReads[1]); Serial.print("  ");
  Serial.print(nerveReads[2]); Serial.println();
}

int readNerve(int p) {
  int q=analogRead(nervePins[p])*.2+.8*nerveReadsLast[p];
  return q;
}

int getDiff(int p) {
  int diff = nerveReads[p]-nerveReadsAvg[p];
  return diff;
}

int evaluateDiff(int p, int diff, int threshold) {
  // evaluates the diff and returns 0 if it is far from the original diff value
  // evaluate by taking the average for the next X loops
  int avg=diff;

  avg=averageNextDiffs(p,avg,3);
  if (avg>threshold) {return 1;}
  else {return 0;}
}

int averageNextDiffs(int p, int avg, int loops) {
  int i=0;
  int finalAvg=avg;
  while (i<loops) {
    finalAvg=((readNerve(p)-finalAvg)+finalAvg)/2;
    i++;
  }
  return finalAvg;
}

int lowerLinear(int counter, int min, float m, int b) {
  // linear incr of counter
  // y=mx
  // y = amount to add
  // x=counter
  // m=slope
  // b= exponent-- how much is added to begin with
  // max out at max

  int y;
  if (counter>=min) {
    y = floor(m*counter+b, 1);
  }
  else {
    y = 0;
  }
  return counter-y;

}

int raiseLinear(int counter, int max, float m, int b) {

  // MODIFY 


  // linear incr of counter
  // y=mx
  // y = amount to add
  // x=counter
  // m=slope
  // b= exponent-- how much is added to begin with
  // max out at max

  int y;
  if (counter<=max) {
    y = floor(m*counter+b, 1);
  }
  else {
    y = 0;
  }
  return counter+y;

}

int lowerExp(int counter, int min, float a) {
  // linear incr of counter
  // y=mx
  // y = amount to add
  // x=counter
  // m=slope
  // b= exponent-- how much is added to begin with
  // max out at max

  int y;
  if (counter>=min) {
    int percent=a*100;
    int q = counter^(percent/100);
    y=floor(q,1);
  }
  else {
    y = 0;
  }
  return counter-y;

}

int raiseExp(int counter, int min, float a) {
  // determines the amount to subtract from a counter
  // y=x^a
  // y=the amount to add to the counter
  // x=counter
  // a=exponent
  // max out at max

  int y;
  if (counter>=min) {
    int percent=a*100;
    int q = counter^(percent/100);
    y=floor(q,1);
  }
  else {
    y = 0;
  }
  return counter+y;

}

void processImpulses(int p) {
  // processes the nerve impulses to determine if it is a real value or noise

  // check the difference between the current read and the average
  int diff = getDiff(p);
  // Serial.println(String(p)+": "+String(diff)+" / "+String(petDiff[p])+" / "+String(painDiff[p]));

  // if it is negative, then re-average the read
  if (diff<0) {
    nerveReadsAvg[p]=(nerveReadsAvg[p]+nerveReads[p])/2;
  }
  else if (diff>petDiff[p]) { // if it is above the petDiff value, then you are being petted or squished
    if (diff>painDiff[p]) {
      // get the average of the next 5 or so values, and get info on if you are above threshold
      if (evaluateDiff(p,diff,painDiff[p])) {  // if it is above threshold
        // average it to threshold
        painDiff[p]=painDiff[p]*0.8+diff*0.2;
        // ADD TO PAIN
        painCounter[p]=ceil(raiseLinear(painCounter[p], 1000, 0,2),1000); // this is a linear increase of 2 each time
      }
      else {
        // SUBTRACT FROM PAIN
        painCounter[p]=floor(lowerExp(painCounter[p],0,0.5),0); // this subtracts rapidly at the beginning and less at the end
      }
    }
    else {
      // check if being pet
      if (evaluateDiff(p,diff,petDiff[p])) {  // if it is above threshold
        // average it to threshold
        petDiff[p]=petDiff[p]*0.8+diff*0.2;
        // ADD TO PET
        petCounter[p]=ceil(raiseLinear(petCounter[p], 1000, 0,2),500); // this is a linear increase of 2 each time
      }
      else {
        // SUBTRACT FROM PET
        petCounter[p]=floor(lowerExp(petCounter[p],0,0.5),0)
        ; // this subtracts rapidly at the beginning and less at the end
      }
    }
  }

}

int averageNextReads(int p, int avg, int loops) {
  int i=0;
  int finalAvg;
  while (i<loops) {
    finalAvg=(avg+readNerve(p))/2;
    i++;
  }
  return finalAvg;
}

void parseCounters(int pet, int pain, int p) {

  Serial.println(String(p)+": "+String(pet)+", "+String(pain));

  // p: 0=top of head, 1=left side of head, 2=right side of head

  // rebalance if both are nonzero
  if (pet!=0 && pain!=0) {
    if (pet>pain) {
      pain=0;
      pet=pet-pain;
    }
    else if (pain>pet) {
      pet=0;
      pain=pain-pet;
    }
  }

  // max out at 500
  pet=ceil(pet,500);
  pain=ceil(pain,1000);

  if (p==0) {
    // top of head

    if (pet<4 && pet>0) {
      // start tail wag
      wag(0.25);
    }

    else if(pet>=4) {
      // wag tail at rate that matches the petting time
      wag((float)pet/40.00);
    }

    if(random(0,100)<pain) {
      // thrash
      thrash((float)pet/10.00);
    }
  }

  if (p==1) {
    // left side-- turn left
    if (pet<4 && pet>0) {
      accelerate("s0,120,1.5");
    }
    else if(pet>=4) {
      // wag tail at rate that matches the petting time
      wag((float)pet/40.00);
    }
  }

  if (p==2) {
    // right side-- turn right
    if (pet<4 && pet>0) {
      accelerate("s0,60,1.5");
    }
    else if(pet>=4) {
      // wag tail at rate that matches the petting time
      wag((float)pet/40.00);
    }
  }

}

void startPos() {
    linear("s0,60");
    linear("s1,60");
    linear("s2,60");
}

void setCry(int s, int onoff) {
  if (onoff==1) { // turning on
    if (random(0,10001)<=soundProbability[s]) {
        digitalWrite(soundPins[s],LOW);
    }
    else {
        digitalWrite(soundPins[s],HIGH);
    }
  }
  else {  // turning off
    digitalWrite(soundPins[s],HIGH);
  }
}

int floor(int x, int min) {
  if (x<min) {x=min;}
  return x;
}

float floorFloat(float x, float min) {
  if (x<min) {x=min;}
  return x;
}

int ceil(int x, int max) {
  if (x>max) {x=max;}
  return x;
}

int bound(int x, int min, int max) {
  if (x<min) {x=min;}
  if (x>max) {x=max;}
  return x;
}

void wag(float intensity) {
    // resting pattern
    // sometimes move your tail servo back and forth
    // always back and forth
    // very very infrequent

    int servo=2;
    float speedFactor=floorFloat(2.0-intensity,0.1);
    int angleInverse=intensity*60;
    int angle;
    if (s2angle[1]<90) {angle=90+angleInverse; if (angle>150){angle=150;}}
    if (s2angle[1]>90) {angle=90-angleInverse; if (angle<30){angle=30;}}

    accelerate("s"+String(servo)+","+String(angle)+","+String(speedFactor));


    //also yell
    if (!random(0,100)) {
      setCry(1,1);
      setCry1Timeout.start();
    }
}

void thrash(float intensity) {
  int servo=random(0,3);
  int speedFactor=random(0,floorFloat(1.5-intensity,0.1));
  int angle;
  // random servo
  if (servo==0) {
    if (s0angle[0]==20) {angle=160;}
    else {angle=20;}
  }
  else if (servo==1) {
    if (s1angle[1]==20) {angle=160;}
    else {angle=20;}
  }
  else if (servo==2) {
    if (s2angle[2]==20) {angle=160;}
    else {angle=20;}
  }
    setCry2Timeout.changePeriod(intensity*100);
    setCry(2,1);
      setCry2Timeout.start();
      accelerate("s"+String(servo)+","+String(angle)+","+String(speedFactor));
}

int randomPerlin(int perlin, int min, int max, int variance){

    perlin += variance - 10*random(0,(max-min)/20+1);

    if(perlin < min){perlin = min;}
    else if (perlin > max){perlin = max;}

    return perlin;

}

void setAvailable(int x) {
  servoAvailability[x]=1;
}

int accelerate(String command) {

    Serial.println(command);

    char inputStr[64];
    command.toCharArray(inputStr,64);
    char *p = strtok(inputStr,",");
    String servo = p;
    p = strtok(NULL,",");
    int angle = atoi(p);
    p = strtok(NULL,",");
    float speedFactor = atof(p);

    /*p = strtok(NULL,",");
    int delay=atoi(p);*/

    // build in delay here?

    if (servo=="s0") {
      if (servoAvailability[0]==1) {
        servoAvailability[0]=0;
        setServo0Timeout.changePeriod(speedFactor*10*(angle-s0angle[0]));
        setServo0Timeout.start();
        accelWrite(s0,angle,s0angle[0],speedFactor);
        s0angle[1]=s0angle[0];
        s0angle[0]=angle;
        lastServo=0;
        return angle;
      }
      else {return -1;}
    }

    else if (servo=="s1") {
      if (servoAvailability[1]==1) {
        servoAvailability[1]=0;
        setServo1Timeout.changePeriod(speedFactor*10*(angle-s1angle[0]));
        setServo1Timeout.start();
        accelWrite(s1,angle,s1angle[0],speedFactor);
        s1angle[1]=s1angle[0];
        s1angle[0]=angle;
        lastServo=1;
        return angle;
      }
      else {return -1;}
    }

    else if (servo=="s2") {
      if (servoAvailability[2]==1) {
        servoAvailability[2]=0;
        setServo2Timeout.changePeriod(speedFactor*10*(angle-s2angle[0]));
        setServo2Timeout.start();
        accelWrite(s2,angle,s2angle[0],speedFactor);
        s2angle[1]=s2angle[0];
        s2angle[0]=angle;
        lastServo=2;
        return angle;
      }
      else {return -1;}
    }

    else {
        return -1;
    }

}

int linear(String command) {
    char inputStr[64];
    command.toCharArray(inputStr,64);
    char *p = strtok(inputStr,",");
    String servo = p;
    p = strtok(NULL,",");
    int angle = atoi(p);
    if (servo=="s0") {
      if (servoAvailability[0]==1) {
        servoAvailability[0]=0;
        setServo0Timeout.changePeriod((angle-s0angle[0]));
        setServo0Timeout.start();
        s0.write(angle);
        s0angle[1]=s0angle[0];
        s0angle[0]=angle;
        lastServo=0;
        return angle;
      }
      else {return -1;}
    }

    else if (servo=="s1") {
      if (servoAvailability[1]==1) {
        servoAvailability[1]=0;
        setServo1Timeout.changePeriod((angle-s1angle[0]));
        setServo1Timeout.start();
        s1.write(angle);
        s1angle[1]=s1angle[0];
        s1angle[0]=angle;
        lastServo=1;
        return angle;
      }
      else {return -1;}
    }

    else if (servo=="s2") {
      if (servoAvailability[2]==1) {
        servoAvailability[2]=0;
        setServo2Timeout.changePeriod((angle-s2angle[0]));
        setServo2Timeout.start();
        s2.write(angle);
        s2angle[1]=s2angle[0];
        s2angle[0]=angle;
        lastServo=2;
        return angle;
      }
      else {return -1;}
    }

    else {
        return -1;
    }
}

void accelWrite(Servo servo, int angle, int lastAngle, float speedFactor) {
    // always accelerate into movement. Never be binary. Be alive.
    if (angle>lastAngle) {
        for (int i=lastAngle; i<angle; i++) {
          servo.write(i);
          delay(sqrt(speedFactor*i));
        }
    }
    else if (angle<lastAngle) {
        for (int i=lastAngle; i>angle; i--) {
          servo.write(i);
          delay(sqrt(speedFactor*(lastAngle-i)));
        }
    }
}
