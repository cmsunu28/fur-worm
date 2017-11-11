SYSTEM_MODE(SEMI_AUTOMATIC);

#include "math.h"
// switch to time as a determinant instead of pressure

// servo definitions
int s0pin=D0;
int s1pin=D1;
int s2pin=D2;

int powerPin=A3;


int faceL=A1;
int faceR=A4;
int faceT=B4;

/*
int faceT=B2;
int faceL=B3;
int faceR=B4;
int rib1L=B5;
int rib1R=A0;
int midT=A1;
int rib2L=A2;
int rib2R=A3;
int tailT=A4;
*/

int faceReads[3];
int faceReadsLast[3];

int faceReadsAvg[3];

float percentagePet=1.5; // percent of average that reads must be to assume petting
float percentagePain=2.0; // percent of average that reads must be to assume pain

int petThresholdFace[3]={40,20,40};
int painThresholdFace[3]={80,30,60};
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
// 0: recover
// 1: undulate (resting)
// 2: squirming (with variable speedFactor and amplitude)
// 3: flailing (with variable speedFactor and amplitude)
// 4: death throes
// 5: sleep (dead)
//
// int monitorInterval=250; // ms interval to take average and monitor for state change
// int lastMonitor=0;
//
// // time thresholds to move onto the next state-- you must be held for this long in order to move on. recover has none b/c it is a pre-set function. death throes will eventually kill the robot.
// int stateThresholds[5]={
//     300,
//     300,
//     3000,
//     8000,
//     20000
// };
//
// // servo position mins and maxes by state
// int statePosRange[8]={
//     80,
//     100,
//     0,
//     180,
//     40,
//     140,
//     0,
//     180
// };
//
// // speedFactor mins and maxes by state
// // being linear, state 7 (death throes) has no speed range
// float stateSpeedRange[6]={
//     2.0,
//     1.5,
//     1.5,
//     1.2,
//     1.2,
//     0.5,
// };
//
// // bias by state:
// // 0 1 and 2 are s0, s1, and s2
// // 3: none
// // 4: head and center
// // 5: tail and center
// int stateServoBias[5]={
//     0,
//     4,
//     3,
//     5,
//     1
// };
// // increase this bias intensity with time
//
// // sound probability (out of 10000 (10 seconds))
// // if below, pin low. If above, pin high.
int soundProbability[5]={
    0,
    5000,
    8000,
    8000, // these don't count since they will be on always
    10000
};
//
//
// // this should be the likelihood of motion, out of 10000 (10 seconds)
// // this is before intensity calculation
// int motionProbability[5]={
//   10000,  // always
//   50, // 5x/second
//   6500,  // 50 times a second
//   8000, // 100 times a second
//   9500 // frequently
// };

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

    // set to wakeup position
    startPos();

}

void loop() {

    readNerves();
    react();

    // thinky-looking delay
    delay(10);

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

  // do average
  faceReadsAvg[0]=(faceReads[0]+faceReadsAvg[0])/2;
  faceReadsAvg[1]=(faceReads[1]+faceReadsAvg[1])/2;
  faceReadsAvg[2]=(faceReads[2]+faceReadsAvg[2])/2;

  // read each nerve, put into an array representing the body:

  int leftRead=analogRead(faceL)*.2+.8*faceReadsLast[1];
  int rightRead=analogRead(faceR)*.2+.8*faceReadsLast[2];
  int topRead=analogRead(faceT)*.2+.8*faceReadsLast[0];

  faceReads[0]=topRead;
  faceReads[1]=leftRead;
  faceReads[2]=rightRead;

  Serial.print(faceReads[0]); Serial.print("  ");
  Serial.print(faceReads[1]); Serial.print("  ");
  Serial.print(faceReads[2]); Serial.println();

  // if 150% of average, assume petting. otherwise, incorporate into average.
  // if 200% of average, assume pain. otherwise, incorporate into average.



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

    if (p==0) {Serial.println("Head pats!");}
    else if (p==1) {Serial.println("Left ear pats!");}
    else if (p==2) {Serial.println("Right ear pats!");}

    petCounterFace[p]=petCounterFace[p]+2;
    if (painCounterFace[p]<0) {painCounterFace[p]=0;} else {painCounterFace[p]--;}

  }

  else if (faceReads[p]>=painThresholdFace[p]) {

    if (p==0) {Serial.println("Head squish!");}
    else if (p==1) {Serial.println("Left ear squish!");}
    else if (p==2) {Serial.println("Right ear squish!");}

    painCounterFace[p]=painCounterFace[p]+2;
    if (petCounterFace[p]<0) {petCounterFace[p]=0;} else {petCounterFace[p]--;}
  }

  else {
    if (petCounterFace[p]<0) {petCounterFace[p]=0;} else {petCounterFace[p]--;}
    if (painCounterFace[p]<0) {painCounterFace[p]=0;} else {painCounterFace[p]--;}
  }

}

void setCounters(int r,int petThreshold,int painThreshold,int petCounter,int painCounter) {
  if (petThreshold<r && r<painThreshold) {
    // head is petted, make happy noises and twitch tail a lot, the longer that you pet
    Serial.println("Head pats!");
    petCounter=petCounter+2;
    if (painCounter<0) {painCounter=0;} else {painCounter--;}

    // twitch tail, with greater intensity the more petCounterFace you have.
  }
  else if (r>=painThreshold) {
    // head is in pain, twitch back and forth
    Serial.println("Head squish!");
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



//
// int checkForHeld(int s) {
//     int t=millis();
//     int r=-1;
//     // set timer over stated checkInterval and check to see if the average is above the average needed
//     // if (t>lastMonitor+monitorInterval) {
//     //     // time is up! evaluate the average
//     //     if (senseAvg>squeezed) {
//     //         Serial.println("                    squeezed");
//     //         if (t>millisAtStateChange+stateThresholds[s]) {
//     //             millisAtStateChange=millis();
//     //             r=s+1; // progress to the next state
//     //             Serial.println("                    progressing to "+String(r));
//     //         }
//     //         else {r=s;}
//     //     }
//     //     else {
//     //         millisAtStateChange=millis();
//     //         if (s>=2){
//     //             Serial.println("                    recover");
//     //             movement(0);
//     //             r=1;
//     //             } // recover
//     //         else {r=1;}
//     //     }
//     //     lastMonitor=millis();
//     // }
//     // else {
//     //     // add to average
//     //     senseAvg=(sense+9*senseAvg)/10;  // bias this average
//     //     r=s;
//     // }
//     return r;
// }
//
// int takeAction(int s) {
//   // use probabilities
//   // grant greater likelihood to greater intensity
//   int t=millis()-millisAtStateChange;
//   float intensity=(float)t/(float)stateThresholds[s];
//   int max=10001;
//   if (s>1) {
//     max=10001-intensity*5000;
//   }
//   if (random(0,max)<=motionProbability[s]) {
//     movement(s);
//   }
// }
//
// int movement(int s) {
//     // do the movement attributed to the state
//     // make the changes necessary based on how long it has been squeezed so far
//
//     // get the passage of time
//     int t=millis()-millisAtStateChange;
//     float intensity=(float)t/(float)stateThresholds[s];
//     // use intensity to skew to the extremes
//
//     if (s==1) {
//         undulate(intensity);
//     }
//     else if (s==2) {
//         squirm(intensity);
//     }
//     else if (s==3) {
//         twitch(intensity);
//     }
//     else if (s==4) {
//         flail(intensity);
//         // deathThroes();
//     }
//     else if (s==0) {
//         recover();
//         state=1;
//     }
// }

// float adjustedSpeedFactor(int s) {
//     int min=stateThresholds[s-1];
//     int max=stateThresholds[s];
//     float sfmin=stateSpeedRange[s-1];
//     float sfmax=stateSpeedRange[s];
//     float percent=((float)sense-(float)min)/((float)max-(float)min);
//   	float x=sfmin+(percent*(sfmax-sfmin));
//     return x;
// }
//

//
// int calibrateExtremes(int s, float intensity) {
//     //get statePosRange for the max and min, and bias towards more frequent extremes depending on intensity
//     int min=statePosRange[(s-1)*2];
//     int max=statePosRange[(s-1)*2+1];
//
//     int range=(max-min);
//     int midRange[2]={min+range/4,max-range/4};
//     int lowRange[2]={min,min+range/4};
//     int highRange[2]={max-range/4,max};
//
//     int amp;
//
//     // get randoms
//     // bias by intensity by getting a random between 0 and intensity*100.
//     int r=random(0,120);
//     if (r>(intensity*100)) {
//         // do midrange, but decrease midrange the higher the intensity gets
//         // will always preserve at least a 1/6 chance of midrange regardless
//         amp=random(midRange[0],midRange[1]);
//     }
//     else {
//         if (random(0,2)) {
//             amp=random(lowRange[0],lowRange[1]);
//         }
//         else {
//             amp=random(highRange[1],highRange[1]);
//         }
//     }
//     return amp;
// }
//
// int calibrateSpeedFactor(int s, float intensity) {
//     int min=stateSpeedRange[(s-1)*2];
//     int max=stateSpeedRange[(s-1)*2+1];
//     float sf = min+intensity*(max-min);
//     return sf;
// }
//
// int calibrateBias(int s,float intensity) {
//     int calibrateMax=15; // 10/calibrateMax represents the frequency of the random rate
//     int r=random(0,3);  // default return is random
//     int servos=stateServoBias[s];
//     if (servos>2) {
//         if (servos==4) {
//             // bias 0 and 1 with incr frequency
//             int f=intensity*10;
//             if (random(0,calibrateMax)<f) {
//                 if (random(0,2)) {
//                     r=0;
//                 }
//                 else {
//                     r=1;
//                 }
//             }
//             else {
//                 r=2;
//             }
//         }
//         else if (servos==5) {
//             // bias 1 and 2 with incr frequency
//             int f=intensity*10;
//             if (random(0,calibrateMax)<f) {
//                 if (random(0,2)) {
//                     r=1;
//                 }
//                 else {
//                     r=2;
//                 }
//             }
//             else {
//                 r=0;
//             }
//         }
//     }
//     else {
//         r=servos;
//     }
//     return r;
// }
//
// void recover() {
//     // shake head
//     accelerate("s0,90,1.4");
//     accelerate("s2,90,1.5");
//     accelerate("s1,90,1.5");
//     millisAtStateChange=millis();
//     lastMonitor=millis();
//     silenceCries();
// }
//

//
// void squirm(float intensity) {
//
//     int servo=calibrateBias(2,intensity);
//     int speedFactor=calibrateSpeedFactor(2,intensity);
//     int angle;
//     // random servo
//     if (servo==0) {
//       if (s0angle[0]==20) {angle=160;}
//       else {angle=20;}
//     }
//     else if (servo==1) {
//       if (s1angle[1]==20) {angle=160;}
//       else {angle=20;}
//     }
//     else if (servo==2) {
//       if (s2angle[2]==20) {angle=160;}
//       else {angle=20;}
//     }
//       setCry2Timeout.changePeriod(intensity*100);
//       setCry(2,1);
//         setCry2Timeout.start();
//         accelerate("s"+String(servo)+","+String(angle)+","+String(speedFactor));
// }
//
// void twitch(float intensity) {
//     // bias towards front twitches (servo 0 and 1, bias 4), sometimes don't squirm
//     int servo=calibrateBias(2, intensity);
//
//     // get amplitude
//     int amplitude=calibrateExtremes(2,intensity);
//
//     // also increase speedFactor by intensity
//     int speedFactor=calibrateSpeedFactor(2,intensity);
//
//     if (random(0,10001)<=soundProbability[2]) {
//       setCry(2,1);
//         setCry2Timeout.start();
//     }
//         accelerate("s"+String(servo)+","+String(amplitude)+","+String(speedFactor));
// }
//
// void flail(float intensity) {
//     // bias towards front twitches (servo 0 and 1, bias 4), sometimes don't squirm
//     int servo=calibrateBias(3, intensity);
//
//     // get amplitude
//     int amplitude=calibrateExtremes(3,intensity);
//     // also increase speedFactor by intensity
//     int speedFactor=calibrateSpeedFactor(3,intensity);
//
//     // sometimes it shouldn't move. This is based off of the x/10 frequency.
//         accelerate("s"+String(servo)+","+String(amplitude)+","+String(speedFactor));
//
// }
//
// void deathThroes() {
//     // bias towards front twitches (servo 0 and 1, bias 4), sometimes don't squirm
//     int servo=calibrateBias(4, 0.80);
//
//     // calculate most extreme
//     int randomServo=random(0,3);
//     int extreme;
//     if (randomServo==0) {
//         extreme=abs(s0angle[0]-random(100,180));
//     }
//     else if (randomServo==1) {
//         extreme=abs(s1angle[0]-random(100,180));
//     }
//     else if (randomServo==2) {
//         extreme=abs(s2angle[0]-random(100,180));
//     }
//
//     // sometimes it shouldn't move. This is based off of the x/10 frequency.
//         linear("s"+String(servo)+","+String(extreme));
// }

//
// void undulate(float intensity) {
//     // resting pattern
//     // sometimes move your tail servo back and forth
//     // always back and forth
//     // very very infrequent
//
//     int servo=1+random(0,2);
//     float speedFactor=1.5;
//     int angle;
//     // random servo
//     if (servo==0) {
//       if (s0angle[0]==60) {angle=120;}
//       else {angle=60;}
//     }
//     else if (servo==1) {
//       if (s1angle[1]==60) {angle=120;}
//       else {angle=60;}
//     }
//     else if (servo==2) {
//       if (s2angle[2]==60) {angle=120;}
//       else {angle=60;}
//     }
//
//
//       //also yell
//       accelerate("s"+String(servo)+","+String(angle)+","+String(speedFactor));
//       setCry(1,1);
//       setCry1Timeout.start();
// }
