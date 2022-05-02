// for front ultrasonic sensor
const int frontTrigPin = 22;
const int frontEchoPin = 23;

// for left ultrasonic sensor
const int leftTrigPin = 24;
const int leftEchoPin = 25;

// for right ultrasonic sensor
const int rightTrigPin = 26;
const int rightEchoPin = 27;

// for relay pin
const int relayPin = 12;

float frontDistance = 0.0;
float leftDistance = 0.0;
float rightDistance = 0.0;

// for motor driver
const int IN1 = 8;   //control right side motor
const int IN2 = 9;   //control right side motor
const int IN3 = 10;  //control left side motor
const int IN4 = 11;  //control left side motor

// for motor speed control connect to PWM pin
const int ENA = 2;  //control speed of right side motor
const int ENB = 3;  //control speed of left side motor


// for robot path
// now it is fixd and path style = 'U'
String pathStyle = "U";

// for drive the robot left or right direction
int isTurnRight = 1;
int isTurnLeft = 0;

// obstacle distance to sense = ODS in 'cm'
int ODS = 20;

// turn specific distance after each path comlete, here turnD = 5cm
// int turnD = 5;

// vacume cleaner width of each path in cm
int turnD = 16;


// move backword with specific distance in 'cm'
int backword = 15;

// forward speed
int fLeftSpeed = 50;
int fRightSpeed = 52;

// turn speed
int tLeftSpeed = 70;
int tRightSpeed = 72;

// for turn right and left time
int leftRotateTime = 753;
int rightRotateTime = 710;

void setup() {
  // put your setup code here, to run once:
  // for ultrasonic sensor
  pinMode(frontTrigPin, OUTPUT);
  pinMode(leftTrigPin, OUTPUT);
  pinMode(rightTrigPin, OUTPUT);
  pinMode(frontEchoPin, INPUT);
  pinMode(leftEchoPin, INPUT);
  pinMode(rightEchoPin, INPUT);

  // for motor driver
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(ENA, OUTPUT);
  pinMode(ENB, OUTPUT);

  // for relay pin
  pinMode(relayPin, OUTPUT);

  // for serial port communicate speed
  Serial.begin(9600);
}

// int flag = 1;
void loop() {
  // if path style == 'U' then run 'U' path algo
  if (pathStyle == "U") {
    frontDistance = getFrontDistance();
    leftDistance = getLeftDistance();
    rightDistance = getRightDistance();

    if (frontDistance > ODS) {
      // speedControl(fLeftSpeed, fRightSpeed);
      // relayStart();
      moveForward();
    } else {
      stop();
      delay(1000);

      if (leftDistance < ODS) {
        if (isTurnRight == 1 && isTurnLeft == 0) {
          // call here turnRobotRightDirection
          moveRobotRightDirection();
        } else {
          // call here turnRobotBack and check if it is posible
          moveRobotBWToTurnLeftDirection();
        }
      } else {
        if (isTurnRight == 0 && isTurnLeft == 1) {
          // call here turnRobotLeftDirection
          turnRobotLeftDirection();
        }else{
          // call here turnRobotRightDirection
          moveRobotRightDirection();
        }
      }
    }
  }


  // if (flag) {
  //   moveForward();
  //   delay(5000);
  //   stop();
  //   delay(500);
  //   // turnRight90degReverse();
  //   turnLeft90degReverse();
  //   // turnRight90deg();
  //   // turnLeft90deg();
  //   // delay(500);
  //   // turnRight90deg();
  //   // turnLeft90deg();
  //   flag = 0;
  // }
}

// to find front distance
int getFrontDistance() {
  float distance;
  distance = getDistance(frontTrigPin, frontEchoPin);
  return distance;
}

// to find left distance
int getLeftDistance() {
  float distance;
  distance = getDistance(leftTrigPin, leftEchoPin);
  return distance;
}

// to find right distance
int getRightDistance() {
  float distance;
  distance = getDistance(rightTrigPin, rightEchoPin);
  return distance;
}

// common funcion to measure distance
int getDistance(int trig, int echo) {
  float duration, distance;
  digitalWrite(trig, LOW);
  delayMicroseconds(2);
  digitalWrite(trig, HIGH);
  delayMicroseconds(2);
  digitalWrite(trig, LOW);
  duration = pulseIn(echo, HIGH);
  distance = (duration * 0.0343) / 2;
  return distance;
}

// for motor driver function
// for stop robot
void stop() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
}

// for move forward robot
void moveForward() {
  speedControl(fLeftSpeed, fRightSpeed);
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
}

// for move backword robot
void moveBackword() {
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
}

// for turn robot at forward left direction
void turnLeft() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
}

// for turn robot at forward right direction
void turnRight() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
}

// for turn robot at reverse left direction
void reverseTurnLeft() {
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
}

// for turn robot at reverse right direction
void reverseTurnRight() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
}

// for control the robot speed
void speedControl(int left, int right) {
  analogWrite(ENA, right);
  analogWrite(ENB, left);
}

// for turn the robot right by 90deg angle
void turnRight90deg() {
  speedControl(tLeftSpeed, tRightSpeed);
  turnRight();
  delay(rightRotateTime);
  stop();
  delay(1000);
}

// for turn the robot right by 90deg angle and look distance from right sensor
void turnRight90degLookRight() {
  turnRight90deg();
  rightDistance = getRightDistance();
}


// for turn the robot left by 90deg angle
void turnLeft90deg() {
  speedControl(tLeftSpeed, tRightSpeed);
  turnLeft();
  delay(leftRotateTime);
  stop();
  delay(1000);
}

// for turn the robot left by 90deg angle and look distance from left sensor
void turnLeft90degLookLeft() {
  turnLeft90deg();
  leftDistance = getLeftDistance();
}
// move backword 15 cm to check if there is box corner or not
void moveBackwordSpecificDistance() {
  speedControl(fLeftSpeed, fRightSpeed);
  moveBackword();
  delay(1000);
  stop();
}

// move backword 15 cm to check if there is box corner or not and look right sensor distance
void moveBWSpDistLookRight() {
  moveBackwordSpecificDistance();
  rightDistance = getRightDistance();
}

// move backword 15 cm to check if there is box corner or not and look left sensor distance
void moveBWSpDistLookLeft() {
  moveBackwordSpecificDistance();
  leftDistance = getLeftDistance();
}

// turn the robot right direction with reverse or backword movement
void turnRight90degReverse() {
  speedControl(tLeftSpeed, tRightSpeed);
  reverseTurnLeft();
  delay(leftRotateTime);
  stop();
}

// turn the robot left direction with reverse or backword movement
void turnLeft90degReverse() {
  speedControl(tLeftSpeed, tRightSpeed);
  reverseTurnRight();
  delay(rightRotateTime+70);
  stop();
}

// move the robot right turn with reverse way. it's use to cover the 0.75*turnD area clean in right turn
void moveRobotRightWithReverseTurn() {
  turnRight90degReverse();
  turnRight90deg();
  stop();
}

// move the robot left turn with reverse way. it's use to cover the 0.75*turnD area clean in left turn
void moveRobotLeftWithReverseTurn() {
  turnLeft90degReverse();
  turnLeft90deg();
  stop();
}


// relay start for start vacume cleaner 
void relayStart(){
  digitalWrite(relayPin, HIGH);
}

void relayStop(){
  digitalWrite(relayPin, LOW);
}

/*============================================================*/


// move robot right direction and check if it is possible
void moveRobotRightDirection() {
  if (rightDistance > ODS) {
    if (rightDistance > turnD) {
      turnRight90degLookRight();
      if (rightDistance > ODS) {
        turnRight90deg();
        isTurnRight = 0;
        isTurnLeft = 1;
      } else {
        // wall is end and cleaning is complete
        // robot will stop and vacume cleaner also stop and break the void loop.
        stop();
        relayStop();
        exit(0);
      }
    } else {
      // run block1
      moveRobotRightDirCleanPartial();
    }
  } else {
    /*
    move the robot backword direction and see right sensor distance 
    if right distance > ODS then recursive this function again
    or stop the robot and cleaned complete
    */
    moveBWSpDistLookRight();
    if (rightDistance < ODS) {
      moveBWSpDistLookRight();
      if (rightDistance < ODS) {
        stop();
        relayStop();
        exit(0);

        // wall is end and cleaning is complete
        // robot will stop and vacume cleaner also stop and break the void loop.
      } else {
        moveRobotRightDirection();
      }

    } else {
      moveRobotRightDirection();
    }
  }
}


// move robot left direction and check if it is possible
void turnRobotLeftDirection() {
  if (leftDistance > turnD) {
    turnLeft90degLookLeft();
    if (leftDistance > ODS) {
      turnLeft90deg();
      isTurnRight = 1;
      isTurnLeft = 0;
    } else {
      // wall is end and cleaning is complete
      // robot will stop and vacume cleaner also stop and break the void loop.

      stop();
      relayStop();
      exit(0);
    }
  } else {
    /*
    run block1 but in left directional
    */
    moveRobotLeftDirCleanPartial();

  }
}


// move the robot backword and check if there is end of wall or box corner
void moveRobotBWToTurnLeftDirection() {
  moveBWSpDistLookLeft();
  if (leftDistance < ODS) {
    moveBWSpDistLookLeft();
    if (leftDistance < ODS) {
      stop();
      relayStop();
      exit(0);

      // wall is end and cleaning is complete
      // robot will stop and vacume cleaner also stop and break the void loop.
    } else {
      turnRobotLeftDirection();
    }
  } else {
    turnRobotLeftDirection();
  }
}


/*
block1 with right turn
actually this function is run the robot to cover or clean the above 0.75 - 1 of turnD 
*/

void moveRobotRightDirCleanPartial() {
  if (rightDistance > 0.7 * turnD) {
    moveRobotRightWithReverseTurn();
    frontDistance = getFrontDistance();
    if (frontDistance < ODS) {
      // wall is end and cleaning is complete
      // robot will stop and vacume cleaner also stop and break the void loop.

      stop();
      relayStop();
      exit(0);
    }

  } else {
    // wall is end and cleaning is complete
    // robot will stop and vacume cleaner also stop and break the void loop.

    stop();
    relayStop();
    exit(0);
  }
}


/*
block1 with left turn
actually this function is run the robot to cover or clean the above 0.75 - 1 of turnD 
*/

void moveRobotLeftDirCleanPartial() {
  if (leftDistance > 0.7 * turnD) {
    moveRobotLeftWithReverseTurn();
    frontDistance = getFrontDistance();
    if (frontDistance < ODS) {
      // wall is end and cleaning is complete
      // robot will stop and vacume cleaner also stop and break the void loop.

      stop();
      relayStop();
      exit(0);
    }

  } else {
    // wall is end and cleaning is complete
    // robot will stop and vacume cleaner also stop and break the void loop.
    moveRobotBWToTurnLeftDirection();
    
  }
}
