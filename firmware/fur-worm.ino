SYSTEM_MODE(SEMI_AUTOMATIC);

#include "math.h"
// switch to time as a determinant instead of pressure

// servo definitions
int s0pin=A5;
int s1pin=RX;
int s2pin=TX;

int powerPin=A3;
int skinPin=A0;
int sense;  // the feedback from skinPin
int senseAvg;

int soundPins[5]={D0,D1,D2,D3,D4};

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

int squeezed=500;   // the minimum before it is decided that you are squeezing it
int millisAtStateChange=0;

int state=0;
// 0: recover
// 1: undulate (resting)
// 2: squirming (with variable speedFactor and amplitude)
// 3: flailing (with variable speedFactor and amplitude)
// 4: death throes
// 5: sleep (dead)

int monitorInterval=250; // ms interval to take average and monitor for state change
int lastMonitor=0;

// time thresholds to move onto the next state-- you must be held for this long in order to move on. recover has none b/c it is a pre-set function. death throes will eventually kill the robot.
int stateThresholds[5]={
    300,
    300,
    3000,
    9000,
    10000
};

// servo position mins and maxes by state
int statePosRange[8]={
    80,
    100,
    0,
    180,
    40,
    140,
    0,
    180
};

// speedFactor mins and maxes by state
// being linear, state 7 (death throes) has no speed range
float stateSpeedRange[6]={
    2.0,
    1.5,
    1.5,
    1.2,
    1.2,
    0.5,
};

// bias by state:
// 0 1 and 2 are s0, s1, and s2
// 3: none
// 4: head and center
// 5: tail and center
int stateServoBias[5]={
    0,
    4,
    3,
    5,
    1
};
// increase this bias intensity with time

// sound probability (out of 10000 (10 seconds))
// if below, pin low. If above, pin high.
int soundProbability[5]={
    0,
    5000,
    8000,
    8000, // these don't count since they will be on always
    10000
};


// this should be the likelihood of motion, out of 10000 (10 seconds)
// this is before intensity calculation
int motionProbability[5]={
  10000,  // always
  50, // 5x/second
  6500,  // 50 times a second
  8000, // 100 times a second
  9500 // frequently
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

    // activate fsr or fabric
    pinMode(skinPin,INPUT_PULLDOWN);

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

    sense = 3824-analogRead(skinPin);

    // Serial.println("Got sense: "+String(sense));

    int newState=checkForHeld(state);
    Serial.println("Sense: "+String(sense));
    // Serial.println("New State: " + String(newState));

    if (newState!=state) {
      // actual new state
      if (newState==0) {
        setCry(1,0);
        setCry(2,0);
        setCry(3,0);
        setCry(4,0);
        recover();
        state=1;
      }
      else if (newState==3) {
        digitalWrite(soundPins[3],LOW);
        state=newState;
      }
      else if (newState==4) {
        setCry(3,0);
        digitalWrite(soundPins[4],LOW);
        state=newState;
      }
      else if (newState==5) {
        setCry(4,0);
        state=0;
        System.sleep(120000);
      }
      else {
        state=newState;
      }
    }
    else {  // regular state
        state=newState;
        takeAction(state);
    }

    // Serial.println("------");

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

int checkForHeld(int s) {
    int t=millis();
    int r=-1;
    // set timer over stated checkInterval and check to see if the average is above the average needed
    if (t>lastMonitor+monitorInterval) {
        // time is up! evaluate the average
        if (senseAvg>squeezed) {
            Serial.println("                    squeezed");
            if (t>millisAtStateChange+stateThresholds[s]) {
                millisAtStateChange=millis();
                r=s+1; // progress to the next state
                Serial.println("                    progressing to "+String(r));
            }
            else {r=s;}
        }
        else {
            millisAtStateChange=millis();
            if (s>=2){
                Serial.println("                    recover");
                movement(0);
                r=1;
                } // recover
            else {r=1;}
        }
        lastMonitor=millis();
    }
    else {
        // add to average
        senseAvg=(sense+9*senseAvg)/10;  // bias this average
        r=s;
    }
    return r;
}

int takeAction(int s) {
  // use probabilities
  // grant greater likelihood to greater intensity
  int t=millis()-millisAtStateChange;
  float intensity=(float)t/(float)stateThresholds[s];
  int max=10001;
  if (s>1) {
    max=10001-intensity*5000;
  }
  if (random(0,max)<=motionProbability[s]) {
    movement(s);
  }
}

int movement(int s) {
    // do the movement attributed to the state
    // make the changes necessary based on how long it has been squeezed so far

    // get the passage of time
    int t=millis()-millisAtStateChange;
    float intensity=(float)t/(float)stateThresholds[s];
    // use intensity to skew to the extremes

    if (s==1) {
        undulate(intensity);
    }
    else if (s==2) {
        twitch(intensity);
    }
    else if (s==3) {
        squirm(intensity);
    }
    else if (s==4) {
        flail(intensity);
        // deathThroes();
    }
    else if (s==0) {
        recover();
        state=1;
    }
}

float adjustedSpeedFactor(int s) {
    int min=stateThresholds[s-1];
    int max=stateThresholds[s];
    float sfmin=stateSpeedRange[s-1];
    float sfmax=stateSpeedRange[s];
    float percent=((float)sense-(float)min)/((float)max-(float)min);
  	float x=sfmin+(percent*(sfmax-sfmin));
    return x;
}

void startPos() {
    linear("s0,60");
    linear("s1,60");
    linear("s2,60");
}

int calibrateExtremes(int s, float intensity) {
    //get statePosRange for the max and min, and bias towards more frequent extremes depending on intensity
    int min=statePosRange[(s-1)*2];
    int max=statePosRange[(s-1)*2+1];

    int range=(max-min);
    int midRange[2]={min+range/4,max-range/4};
    int lowRange[2]={min,min+range/4};
    int highRange[2]={max-range/4,max};

    int amp;

    // get randoms
    // bias by intensity by getting a random between 0 and intensity*100.
    int r=random(0,120);
    if (r>(intensity*100)) {
        // do midrange, but decrease midrange the higher the intensity gets
        // will always preserve at least a 1/6 chance of midrange regardless
        amp=random(midRange[0],midRange[1]);
    }
    else {
        if (random(0,2)) {
            amp=random(lowRange[0],lowRange[1]);
        }
        else {
            amp=random(highRange[1],highRange[1]);
        }
    }
    return amp;
}

int calibrateSpeedFactor(int s, float intensity) {
    int min=stateSpeedRange[(s-1)*2];
    int max=stateSpeedRange[(s-1)*2+1];
    float sf = min+intensity*(max-min);
    return sf;
}

int calibrateBias(int s,float intensity) {
    int calibrateMax=15; // 10/calibrateMax represents the frequency of the random rate
    int r=random(0,3);  // default return is random
    int servos=stateServoBias[s];
    if (servos>2) {
        if (servos==4) {
            // bias 0 and 1 with incr frequency
            int f=intensity*10;
            if (random(0,calibrateMax)<f) {
                if (random(0,2)) {
                    r=0;
                }
                else {
                    r=1;
                }
            }
            else {
                r=2;
            }
        }
        else if (servos==5) {
            // bias 1 and 2 with incr frequency
            int f=intensity*10;
            if (random(0,calibrateMax)<f) {
                if (random(0,2)) {
                    r=1;
                }
                else {
                    r=2;
                }
            }
            else {
                r=0;
            }
        }
    }
    else {
        r=servos;
    }
    return r;
}

void recover() {
    // shake head
    accelerate("s0,90,1.4");
    accelerate("s2,90,1.5");
    accelerate("s1,90,1.5");
    millisAtStateChange=millis();
    lastMonitor=millis();
    silenceCries();
}

void setCry(int s, int onoff) {
  if (onoff==1) { // turning on
    if (random(0,10001)<=soundProbability[1]) {
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

void undulate(float intensity) {
    // resting pattern
    // sometimes move your middle servo back and forth and then your head servo or tail servo back and forth
    // always back and forth
    // very very infrequent

    int servo=calibrateBias(1,intensity);
    float speedFactor=1.5;
    int angle;
    // random servo
    if (servo==0) {
      if (s0angle[0]==60) {angle=120;}
      else {angle=60;}
    }
    else if (servo==1) {
      if (s1angle[1]==60) {angle=120;}
      else {angle=60;}
    }
    else if (servo==2) {
      if (s2angle[2]==60) {angle=120;}
      else {angle=60;}
    }


      //also yell
      accelerate("s"+String(servo)+","+String(angle)+","+String(speedFactor));
      setCry(1,1);
      setCry1Timeout.start();
}


void twitch(float intensity) {

    int servo=calibrateBias(2,intensity);
    int amplitude=calibrateExtremes(2,intensity);
    int speedFactor=calibrateSpeedFactor(2,intensity);
      setCry2Timeout.changePeriod(intensity*100);
      setCry(2,1);
        setCry2Timeout.start();
        accelerate("s"+String(servo)+","+String(amplitude)+","+String(speedFactor));
        accelerate("s"+String(servo)+","+String(180-amplitude)+","+String(speedFactor));
}

void squirm(float intensity) {
    // bias towards front twitches (servo 0 and 1, bias 4), sometimes don't squirm
    int servo=calibrateBias(2, intensity);

    // get amplitude
    int amplitude=calibrateExtremes(2,intensity);

    // also increase speedFactor by intensity
    int speedFactor=calibrateSpeedFactor(2,intensity);

    if (random(0,10001)<=soundProbability[2]) {
      setCry(2,1);
        setCry2Timeout.start();
    }
        accelerate("s"+String(servo)+","+String(amplitude)+","+String(speedFactor));
}

void flail(float intensity) {
    // bias towards front twitches (servo 0 and 1, bias 4), sometimes don't squirm
    int servo=calibrateBias(3, intensity);

    // get amplitude
    int amplitude=calibrateExtremes(3,intensity);
    // also increase speedFactor by intensity
    int speedFactor=calibrateSpeedFactor(3,intensity);

    // sometimes it shouldn't move. This is based off of the x/10 frequency.
        accelerate("s"+String(servo)+","+String(amplitude)+","+String(speedFactor));

}

void deathThroes() {
    // bias towards front twitches (servo 0 and 1, bias 4), sometimes don't squirm
    int servo=calibrateBias(4, 0.80);

    // calculate most extreme
    int randomServo=random(0,3);
    int extreme;
    if (randomServo==0) {
        extreme=abs(s0angle[0]-random(100,180));
    }
    else if (randomServo==1) {
        extreme=abs(s1angle[0]-random(100,180));
    }
    else if (randomServo==2) {
        extreme=abs(s2angle[0]-random(100,180));
    }

    // sometimes it shouldn't move. This is based off of the x/10 frequency.
        linear("s"+String(servo)+","+String(extreme));
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
