SYSTEM_MODE(SEMI_AUTOMATIC);

#include "math.h"
// switch to time as a determinant instead of pressure

// servo definitions
int s0pin=D0;
int s1pin=D1;
int s2pin=D2;

int powerPin=A3;

int facePins[3]={B4,A1,A4};
// int faceL=A1;
// int faceR=A4;
// int faceT=B4;

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

int faceReads[3];
int faceReadsLast[3];

int faceReadsAvg[3];

// if not using percents, set to 1.0
float percentagePet=1; // percent of average that reads must be to assume petting
float percentagePain=1; // percent of average that reads must be to assume pain

// if not using diffs, set to 0
int petDiff=10;
int painDiff=20;

int petThresholdFace[3];
int painThresholdFace[3];
int petCounterFace[3]={0,0,0};
int painCounterFace[3]={0,0,0};

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
    react();
    checkAverages(0);
    checkAverages(1);
    checkAverages(2);

    // thinky-looking delay
    delay(10);

}

void calibrate() {
  faceReadsAvg[0]=analogRead(facePins[0]);
  faceReadsAvg[1]=analogRead(facePins[1]);
  faceReadsAvg[2]=analogRead(facePins[2]);
  // take reads for 3 seconds
  int timer=0;
  while (timer<5000) {
    faceReadsAvg[0]=(faceReadsAvg[0]+analogRead(facePins[0]))/2;
    faceReadsAvg[1]=(faceReadsAvg[1]+analogRead(facePins[1]))/2;
    faceReadsAvg[2]=(faceReadsAvg[2]+analogRead(facePins[2]))/2;
    timer++;
  }
  petThresholdFace[0]=petDiff+percentagePet*faceReadsAvg[0];
  petThresholdFace[1]=petDiff+percentagePet*faceReadsAvg[1];
  petThresholdFace[2]=petDiff+percentagePet*faceReadsAvg[2];

  painThresholdFace[0]=painDiff+percentagePain*faceReadsAvg[0];
  painThresholdFace[1]=painDiff+percentagePain*faceReadsAvg[1];
  painThresholdFace[2]=painDiff+percentagePain*faceReadsAvg[2];

  Serial.println("-------------Calbirated-------------");
  Serial.println("avg: "+String(faceReadsAvg[0])+","+String(faceReadsAvg[1])+","+String(faceReadsAvg[2]));
  Serial.println("pet: "+String(petThresholdFace[0])+","+String(petThresholdFace[1])+","+String(petThresholdFace[2]));
  Serial.println("pain: "+String(painThresholdFace[0])+","+String(painThresholdFace[1])+","+String(painThresholdFace[2]));
  delay(1000);
}

void silenceCries() {
  setCry(0,0);
  setCry(1,0);
  setCry(2,0);
  setCry(3,0);
  setCry(4,0);
}

void readNerves() {

  //cleanup from last time:
  faceReadsLast[0]=faceReads[0];
  faceReadsLast[1]=faceReads[1];
  faceReadsLast[2]=faceReads[2];

  // read each nerve, put into an array representing the body:

  faceReads[0]=analogRead(facePins[1])*.2+.8*faceReadsLast[1];
  faceReads[1]=analogRead(facePins[2])*.2+.8*faceReadsLast[2];
  faceReads[2]=analogRead(facePins[0])*.2+.8*faceReadsLast[0];

  // Serial.print(faceReads[0]); Serial.print("  ");
  // Serial.print(faceReads[1]); Serial.print("  ");
  // Serial.print(faceReads[2]); Serial.println();

}

void checkAverages(int p) {
  /*
  // if 150% ('percentagePet') of average, assume petting. otherwise, incorporate into average.
  if (faceReadsAvg[p]*petPercent<faceReads[p]) {
    // take more reads to see if it continues to be 150% higher
    // take the next 5 reads and check for if it is also above
    int i=0;
    int tempAvg=(analogRead(facePins[p]));
    while (i<loopLength) {
      tempAvg=(analogRead(facePins[p])+tempAvg)/2;
      i++;
    }

    // if it is under the pain percent threshold, it is in the pet range
    if (faceReads[p]<faceReadsAvg[p]*painPercent) {
      if (faceReadsAvg[p]*petPercent<tempAvg) {
        // this is a true value
        petThresholdFace[p]=(faceReads[p]+petThresholdFace[p])/2;
        faceReads[p]=tempAvg;
      }
      else {
        // this is not a true value, set back to regular avg
        faceReads[p]=faceReadsAvg[p];
      }
    }
    // if it is above the pain percent threshold, it is in the pain range. check for this.
    else if (faceReads[p]>=faceReadsAvg[p]*painPercent) {
      if (faceReadsAvg[p]*painPercent<tempAvg) {
        // this is a true value
        painThresholdFace[p]=(faceReads[p]+painThresholdFace[p])/2;
        faceReads[p]=tempAvg;
      }
      else {
        // not a true value, set back to regular
        faceReads[p]=faceReadsAvg[p];
      }
    }

  }
  else {
    */
    // do average as usual
    petThresholdFace[p]=faceReadsAvg[p]*percentagePet+petDiff;
    painThresholdFace[p]=faceReadsAvg[p]*percentagePain+painDiff;
    faceReadsAvg[p]=  0.1*faceReads[p]+0.9*faceReadsAvg[p];

    Serial.print(faceReads[0]); Serial.print(" / "); Serial.print(faceReadsAvg[0]); Serial.print(" / "); Serial.print(petThresholdFace[0]);; Serial.print(" / "); Serial.println(painThresholdFace[0]);

  // }

}

void react() {

  evaluateReads(0);
  evaluateReads(1);
  evaluateReads(2);

  // otherwise, random dice roll for twitching and basic noise.
  if (faceReads[0]<petThresholdFace[0] && faceReads[1]<petThresholdFace[1] && faceReads[2]<petThresholdFace[2]) {
    if (!random(0,8000)) {
      wag(0.25);
    }
  }

  parseCounters(petCounterFace[0],painCounterFace[0],0);
  parseCounters(petCounterFace[1],painCounterFace[1],1);
  parseCounters(petCounterFace[2],painCounterFace[2],2);

}

void evaluateReads(int p) {
  if (petThresholdFace[p]<faceReads[p] && faceReads[p]<painThresholdFace[p]) {

    // if (p==0) {Serial.println("Head pats!");}
    // else if (p==1) {Serial.println("Left ear pats!");}
    // else if (p==2) {Serial.println("Right ear pats!");}

    // get average
    petThresholdFace[p]=0.2*faceReads[p]+0.8*petThresholdFace[p];

    // set counters
    petCounterFace[p]=petCounterFace[p]+2;
    if (painCounterFace[p]<0) {painCounterFace[p]=0;} else {painCounterFace[p]--;}

  }

  else if (faceReads[p]>=painThresholdFace[p]) {  // pain

    // if (p==0) {Serial.println("Head squish!");}
    // else if (p==1) {Serial.println("Left ear squish!");}
    // else if (p==2) {Serial.println("Right ear squish!");}

    // get average
    painThresholdFace[p]=0.2*faceReads[p]+0.8*painThresholdFace[p];

    // set counters
    painCounterFace[p]=painCounterFace[p]+2;
    if (petCounterFace[p]<0) {petCounterFace[p]=0;} else {petCounterFace[p]--;}
  }

  else {  // no movement
    // get average
    faceReadsAvg[p]=0.1*faceReads[p]+0.9*faceReadsAvg[p];
    //set counters
    petCounterFace[p]=floor(petCounterFace[p]--,0);
    painCounterFace[p]=floor(painCounterFace[p]--,0);
  }

}

void setCounters(int r,int petThreshold,int painThreshold,int petCounter,int painCounter) {
  if (petThreshold<r && r<painThreshold) {
    // head is petted, make happy noises and twitch tail a lot, the longer that you pet
    // Serial.println("Head pats!");
    petCounter=petCounter+2;
    if (painCounter<0) {painCounter=0;} else {painCounter--;}

    // twitch tail, with greater intensity the more petCounterFace you have.
  }
  else if (r>=painThreshold) {
    // head is in pain, twitch back and forth
    // Serial.println("Head squish!");
    painCounter=painCounter+2;
    if (petCounter<0) {petCounter=0;} else {petCounter--;}
  }
  else {
    if (petCounter<0) {petCounter=0;} else {petCounter--;}
    if (painCounter<0) {painCounter=0;} else {painCounter--;}
  }
}

void parseCounters(int pet, int pain, int spot) {

  Serial.println(String(spot)+": "+String(pet)+", "+String(pain));

  // spot: 0=top of head, 1=left side of head, 2=right side of head

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

  if (spot==0) {
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

  if (spot==1) {
    // left side-- turn left
    if (pet<4 && pet>0) {
      accelerate("s0,120,1.5");
    }
    else if(pet>=4) {
      // wag tail at rate that matches the petting time
      wag((float)pet/40.00);
    }
  }

  if (spot==2) {
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

// head turn
  // head must turn towards L or R and then at random rate turn and touch


// tail wag

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
