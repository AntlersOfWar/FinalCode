#include <FEHLCD.h>
#include <FEHIO.h>
#include <FEHUtility.h>
#include <FEHMotor.h>
#include <FEHRPS.h>
#include <FEHServo.h>
#include <FEHSD.h>
#include <math.h>


// QR_OFFSET used because QR code is not centered on robot.
#define WHEEL_RADIUS 1.375
#define COUNTS_PER_REV 48
#define PI 3.1415926535
#define ROBOT_RADIUS 4.7
#define QR_OFFSET 2.0

// X_coord and Y_coord used to store previous location when using relative RPS X and Y checks.
float X_coord;
float Y_coord;

FEHServo lever_servo(FEHServo::Servo6);
FEHServo token_servo(FEHServo::Servo0);

// Declare encoders
/* NOTE: P3_6 and P3_7 cannot be used for digital encoders. Also, "fr" means front-right. "bl" means back-left. */
DigitalEncoder fl_encoder(FEHIO::P1_1);
DigitalEncoder br_encoder(FEHIO::P2_0);

// Declare motors
FEHMotor bl_motor(FEHMotor::Motor0, 5.0);
FEHMotor fr_motor(FEHMotor::Motor3, 5.0);
FEHMotor fl_motor(FEHMotor::Motor1, 5.0);
FEHMotor br_motor(FEHMotor::Motor2, 5.0);


// Declare CdS cell
AnalogInputPin cds(FEHIO::P0_4);

// Declare global positioning variables
float startingPointY;
float ddrLightX;
float foosballDistY;
float bumpY;

// Declare global light variable and general difference between ambient light and red light
float ambient;
float redDiff;

/*
 * Given a distance in inches (@param inches), returns the theoretical counts.
 * @Returns [theoretical counts for a desired distance]
 */
int theoreticalCounts(float inches) {
    int counts = (inches * COUNTS_PER_REV) / (2 * PI * WHEEL_RADIUS);
    return counts;
}

/*
 * Given a degree (@param degrees), returns the theoretical counts (if the robot were to turn about the center).
 * @Returns [theoretical counts for a desired angle]
 */
int theoreticalDegree(float degrees) {
    float radians = degrees * PI / 180;
    float arclength = ROBOT_RADIUS * radians;
    return theoreticalCounts(arclength);
}

/* NOTE: Here 'move_forward' means positive movement. Our coordinate system for this program is a top-down view of the course,
 * with the starting point as the origin. DDR is in positive X and lever is in positive Y. */

/*
 * Given a motor speed (@param percent) and a desired distance (@param inches),
 * drives the robot forward in the direction it is facing.
 */
void move_forward(int percent, float inches) {
    //Reset all encoder counts
    fl_encoder.ResetCounts();
    br_encoder.ResetCounts();

    int counts = theoreticalCounts(inches);

    //Set motors to desired percent. Some motors have to turn backwards, so make percent negative.
    bl_motor.SetPercent(percent);
    fr_motor.SetPercent(-1 * percent);
    fl_motor.SetPercent(percent);
    br_motor.SetPercent(-1 * percent);

    //While the average of the left or right encoders is less than theoretical counts,
    //keep running motors
    while(fl_encoder.Counts() < counts || br_encoder.Counts() < counts) {
        LCD.Clear();
        LCD.Write("Moving forward ");
        LCD.Write(inches);
        LCD.WriteLine(" inches");
        LCD.Write("THEORETICAL COUNTS: ");
        LCD.WriteLine(counts);
        LCD.Write("Actual BRE Counts: ");
        LCD.WriteLine(br_encoder.Counts());
        LCD.Write("Actual FLE Counts: ");
        LCD.WriteLine(fl_encoder.Counts());
    }

    //Turn off motors
    bl_motor.Stop();
    fr_motor.Stop();
    fl_motor.Stop();
    br_motor.Stop();
}

/*
 * Given a motor speed (@param percent) and a desired distance (@param inches),
 * drives the robot in the opposite direction from move_forward.
 */
void move_backward(int percent, float inches) {
    //Reset all encoder counts
    fl_encoder.ResetCounts();
    br_encoder.ResetCounts();

    int counts = theoreticalCounts(inches);

    //Set motors to desired percent. Some motors have to turn backwards, so make percent negative.
    bl_motor.SetPercent(-1 * percent);
    fr_motor.SetPercent(percent);
    fl_motor.SetPercent(-1 * percent);
    br_motor.SetPercent(percent);

    //While the average of the left or right encoders is less than theoretical counts,
    //keep running motors
    while(fl_encoder.Counts() < counts || br_encoder.Counts() < counts) {
        LCD.Clear();
        LCD.Write("Moving forward ");
        LCD.Write(inches);
        LCD.WriteLine(" inches");
        LCD.Write("THEORETICAL COUNTS: ");
        LCD.WriteLine(counts);
        LCD.Write("Actual BRE Counts: ");
        LCD.WriteLine(br_encoder.Counts());
        LCD.Write("Actual FLE Counts: ");
        LCD.WriteLine(fl_encoder.Counts());
    }

    //Turn off motors
    bl_motor.Stop();
    fr_motor.Stop();
    fl_motor.Stop();
    br_motor.Stop();
}

/*
 * Given a motor speed (@param percent) and a desired degree (@param degrees),
 * turns the robot to the left about the centerpoint of the robot.
 */
void turnLeft(int percent, float degrees) {
    //Reset all encoder counts
    fl_encoder.ResetCounts();
    br_encoder.ResetCounts();

    int counts = theoreticalDegree(degrees);

    //Set motors to desired percent. Some motors have to turn backwards, so make percent negative.
    bl_motor.SetPercent(-1 * percent);
    fr_motor.SetPercent(-1 * percent);
    fl_motor.SetPercent(-1 * percent);
    br_motor.SetPercent(-1 * percent);

    //While the average of the left and right encoders is less than theoretical counts,
    //keep running motors
    while(fl_encoder.Counts() < counts || br_encoder.Counts() < counts) {
        LCD.Clear();
        LCD.Write("Turning left ");
        LCD.Write(degrees);
        LCD.WriteLine(" degrees");
        LCD.Write("THEORETICAL COUNTS: ");
        LCD.WriteLine(counts);
        LCD.Write("Actual BRE Counts: ");
        LCD.WriteLine(br_encoder.Counts());
        LCD.Write("Actual FLE Counts: ");
        LCD.WriteLine(fl_encoder.Counts());
    }

    //Turn off motors
    bl_motor.Stop();
    fr_motor.Stop();
    fl_motor.Stop();
    br_motor.Stop();
}

/*
 * Given a motor speed (@param percent) and a desired degree (@param degrees),
 * turns the robot to the right about the centerpoint of the robot.
 */
void turnRight(int percent, float degrees) {
    //Reset all encoder counts
    fl_encoder.ResetCounts();
    br_encoder.ResetCounts();

    int counts = theoreticalDegree(degrees);

    //Set motors to desired percent. Some motors have to turn backwards, so make percent negative.
    bl_motor.SetPercent(percent);
    fr_motor.SetPercent(percent);
    fl_motor.SetPercent(percent);
    br_motor.SetPercent(percent);

    //While the average of the left and right encoders is less than theoretical counts,
    //keep running motors
    while(fl_encoder.Counts() < counts || br_encoder.Counts() < counts) {
        LCD.Clear();
        LCD.Write("Turning right ");
        LCD.Write(degrees);
        LCD.WriteLine(" degrees");
        LCD.Write("THEORETICAL COUNTS: ");
        LCD.WriteLine(counts);
        LCD.Write("Actual BRE Counts: ");
        LCD.WriteLine(br_encoder.Counts());
        LCD.Write("Actual FLE Counts: ");
        LCD.WriteLine(fl_encoder.Counts());
    }

    //Turn off motors
    bl_motor.Stop();
    fr_motor.Stop();
    fl_motor.Stop();
    br_motor.Stop();
}

/*
 * If robot move_forward direction faces positive X:
 * Given a reference point (@param startX) and the desired displacement (@param inches),
 * moves robot in positive X direction to the location relative to the starting point.
 */
void RPS_Xinc(float startX, float inches) {
    Sleep(100);
    if (RPS.X() < startX + (inches - 0.2)) {
        LCD.Clear();
        LCD.WriteLine("Too short!");
        bl_motor.SetPercent(30);
        fr_motor.SetPercent(-30);
        fl_motor.SetPercent(30);
        br_motor.SetPercent(-30);
        while (RPS.X() < startX + inches) {
            LCD.WriteRC(RPS.X(),2,12);
            LCD.WriteRC(RPS.Y(),3,12);
            LCD.WriteRC(RPS.Heading(),4,12);
        }
        bl_motor.Stop();
        fr_motor.Stop();
        fl_motor.Stop();
        br_motor.Stop();
    } else if (RPS.X() > startX + (inches + 0.2)) {
        LCD.Clear();
        LCD.WriteLine("Too far!");
        bl_motor.SetPercent(-30);
        fr_motor.SetPercent(30);
        fl_motor.SetPercent(-30);
        br_motor.SetPercent(30);
        while (RPS.X() > startX + inches) {
            LCD.WriteRC(RPS.X(),2,12);
            LCD.WriteRC(RPS.Y(),3,12);
            LCD.WriteRC(RPS.Heading(),4,12);
        }
        bl_motor.Stop();
        fr_motor.Stop();
        fl_motor.Stop();
        br_motor.Stop();
    }
    Sleep(100);
}

/*
 * If robot move_forward direction faces negative X:
 * Given a reference point (@param startX) and the desired displacement (@param inches),
 * moves robot in positive X direction to the location relative to the starting point.
 */
void RPS_Xinc_rev(float startX, float inches) {
    Sleep(100);
    if (RPS.X() < startX + (inches - 0.2)) {
        LCD.Clear();
        LCD.WriteLine("Too short!");
        bl_motor.SetPercent(-30);
        fr_motor.SetPercent(30);
        fl_motor.SetPercent(-30);
        br_motor.SetPercent(30);
        while (RPS.X() < startX + inches) {
            LCD.WriteRC(RPS.X(),2,12);
            LCD.WriteRC(RPS.Y(),3,12);
            LCD.WriteRC(RPS.Heading(),4,12);
        }
        bl_motor.Stop();
        fr_motor.Stop();
        fl_motor.Stop();
        br_motor.Stop();
    } else if (RPS.X() > startX + (inches + 0.2)) {
        LCD.Clear();
        LCD.WriteLine("Too far!");
        bl_motor.SetPercent(30);
        fr_motor.SetPercent(-30);
        fl_motor.SetPercent(30);
        br_motor.SetPercent(-30);
        while (RPS.X() > startX + inches) {
            LCD.WriteRC(RPS.X(),2,12);
            LCD.WriteRC(RPS.Y(),3,12);
            LCD.WriteRC(RPS.Heading(),4,12);
        }
        bl_motor.Stop();
        fr_motor.Stop();
        fl_motor.Stop();
        br_motor.Stop();
    }
    Sleep(100);
}

/*
 * Given a reference point (@param startX) and the desired displacement (@param inches),
 * moves robot in negative X direction to the location relative to the starting point.
 * (if robot move_forward direction faces negative X)
 */
void RPS_Xdec(float startX, float inches) { /* NOTE: UNUSED FUNCTION */
    Sleep(100);
    if (RPS.X() > startX + (inches - 0.2)) {
        LCD.Clear();
        LCD.WriteLine("Too short!");
        bl_motor.SetPercent(30);
        fr_motor.SetPercent(-30);
        fl_motor.SetPercent(30);
        br_motor.SetPercent(-30);
        while (RPS.X() > startX + inches) {
            LCD.WriteRC(RPS.X(),2,12);
            LCD.WriteRC(RPS.Y(),3,12);
            LCD.WriteRC(RPS.Heading(),4,12);
        }
        bl_motor.Stop();
        fr_motor.Stop();
        fl_motor.Stop();
        br_motor.Stop();
    } else if (RPS.X() < startX + (inches + 0.2)) {
        LCD.Clear();
        LCD.WriteLine("Too far!");
        bl_motor.SetPercent(-30);
        fr_motor.SetPercent(30);
        fl_motor.SetPercent(-30);
        br_motor.SetPercent(30);
        while (RPS.X() < startX + inches) {
            LCD.WriteRC(RPS.X(),2,12);
            LCD.WriteRC(RPS.Y(),3,12);
            LCD.WriteRC(RPS.Heading(),4,12);
        }
        bl_motor.Stop();
        fr_motor.Stop();
        fl_motor.Stop();
        br_motor.Stop();
    }
    Sleep(100);
}

/*
 * If robot move_forward direction faces positive Y:
 * Given a reference point (@param startY) and the desired displacement (@param inches),
 * moves robot in positive Y direction to the location relative to the starting point.
 */
void RPS_Yinc(float startY, float inches) {
    Sleep(100);
    if (RPS.Y() < startY + (inches - 0.2)) {
        LCD.Clear();
        LCD.WriteLine("Too short!");
        bl_motor.SetPercent(30);
        fr_motor.SetPercent(-30);
        fl_motor.SetPercent(30);
        br_motor.SetPercent(-30);
        while (RPS.Y() < startY + inches) {
            LCD.WriteRC(RPS.X(),2,12);
            LCD.WriteRC(RPS.Y(),3,12);
            LCD.WriteRC(RPS.Heading(),4,12);
        }
        bl_motor.Stop();
        fr_motor.Stop();
        fl_motor.Stop();
        br_motor.Stop();
    } else if (RPS.Y() > startY + (inches + 0.2)) {
        LCD.Clear();
        LCD.WriteLine("Too far!");
        bl_motor.SetPercent(-30);
        fr_motor.SetPercent(30);
        fl_motor.SetPercent(-30);
        br_motor.SetPercent(30);
        while (RPS.Y() > startY + inches) {
            LCD.WriteRC(RPS.X(),2,12);
            LCD.WriteRC(RPS.Y(),3,12);
            LCD.WriteRC(RPS.Heading(),4,12);
        }
        bl_motor.Stop();
        fr_motor.Stop();
        fl_motor.Stop();
        br_motor.Stop();
    }
    Sleep(100);
}

/*
 * If robot move_forward direction faces negative Y:
 * Given a reference point (@param startY) and the desired displacement (@param inches),
 * moves robot in negative Y direction to the location relative to the starting point.
 */
void RPS_Ydec(float startY, float inches) {
    Sleep(100);
    if (RPS.Y() > startY - (inches + 0.2)) {
        LCD.Clear();
        LCD.WriteLine("Too short!");
        bl_motor.SetPercent(30);
        fr_motor.SetPercent(-30);
        fl_motor.SetPercent(30);
        br_motor.SetPercent(-30);
        while (RPS.Y() > startY + inches) {
            LCD.WriteRC(RPS.X(),2,12);
            LCD.WriteRC(RPS.Y(),3,12);
            LCD.WriteRC(RPS.Heading(),4,12);
        }
        bl_motor.Stop();
        fr_motor.Stop();
        fl_motor.Stop();
        br_motor.Stop();
    } else if (RPS.Y() < startY - (inches - 0.2)) {
        LCD.Clear();
        LCD.WriteLine("Too far!");
        bl_motor.SetPercent(-30);
        fr_motor.SetPercent(30);
        fl_motor.SetPercent(-30);
        br_motor.SetPercent(30);
        while (RPS.Y() < startY + inches) {
            LCD.WriteRC(RPS.X(),2,12);
            LCD.WriteRC(RPS.Y(),3,12);
            LCD.WriteRC(RPS.Heading(),4,12);
        }
        bl_motor.Stop();
        fr_motor.Stop();
        fl_motor.Stop();
        br_motor.Stop();
    }
    Sleep(100);
}

/*
 * Given a desired angle (@param desiredDeg), rotates the robot until desired angle is achieved.
 * This program aims to ensure the robot will always take the shortest path to the desired heading
 */
void RPS_Angle(float desiredDeg){
    Sleep(200);
    while(abs(desiredDeg - RPS.Heading()) > 1.0){
        if(desiredDeg - RPS.Heading() > 180.0){ // Example: Robot going from Q1 to Q4
            LCD.Clear();
            LCD.WriteLine("Turning CW");
            LCD.Write("Angle: ");
            LCD.Write(RPS.Heading());

            // turn the robot clockwise
            bl_motor.SetPercent(30);
            fr_motor.SetPercent(30);
            fl_motor.SetPercent(30);
            br_motor.SetPercent(30);

            Sleep(75);

            // Stop motors
            bl_motor.Stop();
            fr_motor.Stop();
            fl_motor.Stop();
            br_motor.Stop();
        }
        else if(desiredDeg - RPS.Heading() < -180.0){ // Example: Robot going from Q3 to Q1
            LCD.Clear();
            LCD.WriteLine("Turning CCW");
            LCD.Write("Angle: ");
            LCD.Write(RPS.Heading());

            // turn the robot counterclockwise
            bl_motor.SetPercent(-30);
            fr_motor.SetPercent(-30);
            fl_motor.SetPercent(-30);
            br_motor.SetPercent(-30);

            Sleep(75);

            // Stop motors
            bl_motor.Stop();
            fr_motor.Stop();
            fl_motor.Stop();
            br_motor.Stop();
        }
        else if(desiredDeg - RPS.Heading() > -180.0 && desiredDeg - RPS.Heading() < 0.0){ // Example: Robot going from Q3 to Q2
            LCD.Clear();
            LCD.WriteLine("Turning CW");
            LCD.Write("Angle: ");
            LCD.Write(RPS.Heading());

            // turn the robot clockwise
            bl_motor.SetPercent(30);
            fr_motor.SetPercent(30);
            fl_motor.SetPercent(30);
            br_motor.SetPercent(30);

            Sleep(75);

            // Stop motors
            bl_motor.Stop();
            fr_motor.Stop();
            fl_motor.Stop();
            br_motor.Stop();
        }
        else if(desiredDeg - RPS.Heading() > 0 && desiredDeg - RPS.Heading() < 180.0){ // Example: Robot going from Q1 to Q2
            LCD.Clear();
            LCD.WriteLine("Turning CCW");
            LCD.Write("Angle: ");
            LCD.Write(RPS.Heading());

            // turn the robot counterclockwise
            bl_motor.SetPercent(-30);
            fr_motor.SetPercent(-30);
            fl_motor.SetPercent(-30);
            br_motor.SetPercent(-30);

            Sleep(75);

            // Stop motors
            bl_motor.Stop();
            fr_motor.Stop();
            fl_motor.Stop();
            br_motor.Stop();
        }
        else{   // Just in case the difference is exactly 180.0 degrees
            // turn the robot counterclockwise
            bl_motor.SetPercent(-30);
            fr_motor.SetPercent(-30);
            fl_motor.SetPercent(-30);
            br_motor.SetPercent(-30);

            Sleep(75);

            // Stop motors
            bl_motor.Stop();
            fr_motor.Stop();
            fl_motor.Stop();
            br_motor.Stop();
        }
    }
    // Stop motors
    bl_motor.Stop();
    fr_motor.Stop();
    fl_motor.Stop();
    br_motor.Stop();

    Sleep(200);
}

/*
 * If robot move_forward direction faces negative X:
 * Given an absolute desired X position (@param inches),
 * moves robot in X direction to that X position.
 */
void RPS_X_dec_abs(float inches) { /* NOTE: UNUSED FUNCTION */
    Sleep(100);
    if (RPS.X() > (inches - 0.2)) {
        LCD.Clear();
        LCD.WriteLine("Too short!");
        bl_motor.SetPercent(30);
        fr_motor.SetPercent(-30);
        fl_motor.SetPercent(30);
        br_motor.SetPercent(-30);
        while (RPS.X() > inches) {
            LCD.WriteRC(RPS.X(),2,12);
            LCD.WriteRC(RPS.Y(),3,12);
            LCD.WriteRC(RPS.Heading(),4,12);
        }
        bl_motor.Stop();
        fr_motor.Stop();
        fl_motor.Stop();
        br_motor.Stop();
    } else if (RPS.X() < (inches + 0.2)) {
        LCD.Clear();
        LCD.WriteLine("Too far!");
        bl_motor.SetPercent(-30);
        fr_motor.SetPercent(30);
        fl_motor.SetPercent(-30);
        br_motor.SetPercent(30);
        while (RPS.X() < inches) {
            LCD.WriteRC(RPS.X(),2,12);
            LCD.WriteRC(RPS.Y(),3,12);
            LCD.WriteRC(RPS.Heading(),4,12);
        }
        bl_motor.Stop();
        fr_motor.Stop();
        fl_motor.Stop();
        br_motor.Stop();
    }
    Sleep(100);
}

/*
 * If robot move_forward direction faces positive X:
 * Given an absolute desired X position (@param inches),
 * moves robot in X direction to that X position.
 */
void RPS_X_inc_abs(float inches) {
    Sleep(100);
    if (RPS.X() < (inches - 0.2)) {
        LCD.Clear();
        LCD.WriteLine("Too short!");
        bl_motor.SetPercent(30);
        fr_motor.SetPercent(-30);
        fl_motor.SetPercent(30);
        br_motor.SetPercent(-30);
        while (RPS.X() < inches) {
            LCD.WriteRC(RPS.X(),2,12);
            LCD.WriteRC(RPS.Y(),3,12);
            LCD.WriteRC(RPS.Heading(),4,12);
        }
        bl_motor.Stop();
        fr_motor.Stop();
        fl_motor.Stop();
        br_motor.Stop();
    } else if (RPS.X() > (inches + 0.2)) {
        LCD.Clear();
        LCD.WriteLine("Too far!");
        bl_motor.SetPercent(-30);
        fr_motor.SetPercent(30);
        fl_motor.SetPercent(-30);
        br_motor.SetPercent(30);
        while (RPS.X() > inches) {
            LCD.WriteRC(RPS.X(),2,12);
            LCD.WriteRC(RPS.Y(),3,12);
            LCD.WriteRC(RPS.Heading(),4,12);
        }
        bl_motor.Stop();
        fr_motor.Stop();
        fl_motor.Stop();
        br_motor.Stop();
    }
    Sleep(100);
}

/*
 * If robot move_forward direction faces positive Y:
 * Given an absolute desired Y position (@param inches),
 * moves robot in Y direction to that Y position.
 */
void RPS_Y_inc_abs(float inches) {
    Sleep(100);
    if (RPS.Y() < (inches - 0.1)) { //Was 0.2 tolerance before 4/3
        LCD.Clear();
        LCD.WriteLine("Too short!");
        bl_motor.SetPercent(30);
        fr_motor.SetPercent(-30);
        fl_motor.SetPercent(30);
        br_motor.SetPercent(-30);
        while (RPS.Y() < inches) {
            LCD.WriteRC(RPS.X(),2,12);
            LCD.WriteRC(RPS.Y(),3,12);
            LCD.WriteRC(RPS.Heading(),4,12);
        }
        bl_motor.Stop();
        fr_motor.Stop();
        fl_motor.Stop();
        br_motor.Stop();
    } else if (RPS.Y() > (inches + 0.1)) {
        LCD.Clear();
        LCD.WriteLine("Too far!");
        bl_motor.SetPercent(-30);
        fr_motor.SetPercent(30);
        fl_motor.SetPercent(-30);
        br_motor.SetPercent(30);
        while (RPS.Y() > inches) {
            LCD.WriteRC(RPS.X(),2,12);
            LCD.WriteRC(RPS.Y(),3,12);
            LCD.WriteRC(RPS.Heading(),4,12);
        }
        bl_motor.Stop();
        fr_motor.Stop();
        fl_motor.Stop();
        br_motor.Stop();
    }
    Sleep(100);
}

/*
 * If robot move_forward direction faces negative Y:
 * Given an absolute desired Y position (@param inches),
 * moves robot in Y direction to that Y position.
 */
void RPS_Y_dec_abs(float inches) {
    Sleep(100);
    if (RPS.Y() > (inches - 0.2)) {
        LCD.Clear();
        LCD.WriteLine("Too short!");
        bl_motor.SetPercent(30);
        fr_motor.SetPercent(-30);
        fl_motor.SetPercent(30);
        br_motor.SetPercent(-30);
        while (RPS.Y() > inches) {
            LCD.WriteRC(RPS.X(),2,12);
            LCD.WriteRC(RPS.Y(),3,12);
            LCD.WriteRC(RPS.Heading(),4,12);
        }
        bl_motor.Stop();
        fr_motor.Stop();
        fl_motor.Stop();
        br_motor.Stop();
    } else if (RPS.Y() < (inches + 0.2)) {
        LCD.Clear();
        LCD.WriteLine("Too far!");
        bl_motor.SetPercent(-30);
        fr_motor.SetPercent(30);
        fl_motor.SetPercent(-30);
        br_motor.SetPercent(30);
        while (RPS.Y() < inches) {
            LCD.WriteRC(RPS.X(),2,12);
            LCD.WriteRC(RPS.Y(),3,12);
            LCD.WriteRC(RPS.Heading(),4,12);
        }
        bl_motor.Stop();
        fr_motor.Stop();
        fl_motor.Stop();
        br_motor.Stop();
    }
    Sleep(100);
}

/*
 * Function called at the beginning to start off based on a light difference,
 * or if 30 seconds has passed. Also stores the general difference between ambient and red light.
 */
void waitForLight() {

    float time = TimeNow();

    // If 30 seconds pass and no light is read, just start
    while(cds.Value() > ambient - 0.4 && TimeNow() - time < 30) {
        LCD.Clear();
        LCD.WriteLine("Looking for Red Light...");
        LCD.WriteLine(cds.Value());
    }

    Sleep(50);

    redDiff = ambient - cds.Value();
}

/*
 * Given a desired motor speed (@param percent), moves robot forward at that speed,
 * toward the closest DDR light. The CdS cell should go over this light.
 * @Returns [red light was detected]
 */
bool checkDDRLight(int percent) {
    Sleep(100);

    //Set motors to desired percent. Some motors have to turn backwards, so make percent negative.
    bl_motor.SetPercent(percent);
    fr_motor.SetPercent(-1 * percent);
    fl_motor.SetPercent(percent);
    br_motor.SetPercent(-1 * percent);

    bool lightFound = false;
    bool redLight;

    int failsafe = 0;

    while (!lightFound && failsafe < 10) {
        if (ambient - cds.Value() >= redDiff - 0.25) { //Was 0.2 before, made more generous
            bl_motor.Stop();
            fr_motor.Stop();
            fl_motor.Stop();
            br_motor.Stop();
            lightFound = true;
            redLight = true;
        } else if (ambient - cds.Value() <= redDiff - 0.255) {
            bl_motor.Stop();
            fr_motor.Stop();
            fl_motor.Stop();
            br_motor.Stop();
            lightFound = true;
            redLight = false;
        }
        else{
            failsafe++;
        }
   }
   if(failsafe == 10){
       redLight = false;
   }

   return redLight;
}

/*
 * TODO: Fill in all the functions with appropriate movements. As of 3/6/19, all functions will do their respective task starting from the start.
 * Later on, only one of the functions (doDDR()) will have the waitForLight() function. The others will have to go off the previous task function called.
 */

/*
 * Does everything from starting off to the end of DDR, facing towards the ramp.
 */
void doDDR() {
    // Adjust heading
    RPS_Angle(45.0);

    // Adjust y-position
    RPS_Y_inc_abs(startingPointY);

    // Turn right
    turnRight(40, 40.0);

    // Adjust heading
    RPS_Angle(0.0);

    // Go straight
    move_forward(70, 3);

    // Adjust heading
    RPS_Angle(356.0);

    // Go straight
    move_forward(70, 3.5);

    // Adjust heading
    RPS_Angle(351.0);

    // Go to specific x-location
    RPS_X_inc_abs(ddrLightX);

    // Go straight and check for DDR light color (new as of 3/26)
    bool redLight = checkDDRLight(20);

    if (redLight) {
        LCD.SetBackgroundColor(RED);
        LCD.Clear();
        LCD.Write(cds.Value());

        Sleep(1000);

        turnRight(40, 20);

        move_backward(70, 1.0);

        turnRight(40, 30);

        move_forward(70, 1.5);

        turnRight(40, 40.0);

        RPS_Angle(270.0);

        move_forward(70, 3.0);

        bl_motor.SetPercent(50);
        fl_motor.SetPercent(50);

        Sleep(5500);

        bl_motor.Stop();
        fl_motor.Stop();

        move_backward(70, 2.1);

        turnLeft(40, 80.0);

        Sleep(100);

        RPS_Angle(358.0);

        RPS_X_inc_abs(30.5);

    }
    else {
        LCD.SetBackgroundColor(BLUE);
        LCD.Clear();
        LCD.Write(cds.Value());

        Sleep(1000);

        RPS_Angle(0.0);

        move_forward(70, 7.0);

        turnRight(50, 103.0);

        RPS_Angle(270.0);

        bl_motor.SetPercent(50);
        fr_motor.SetPercent(-1 * 50);
        fl_motor.SetPercent(50);
        br_motor.SetPercent(-1 * 50);

        Sleep(5700);

        bl_motor.Stop();
        fr_motor.Stop();
        fl_motor.Stop();
        br_motor.Stop();

        move_backward(70, 2.3); // 4/3

        turnLeft(40, 80.0);

        Sleep(100);

        RPS_Angle(358.0);

        move_backward(70, 3.5); // 4/3

        RPS_X_inc_abs(30.5);
    }

    // Adjust heading
    RPS_Angle(0.0);

    // Press RPS button
    lever_servo.SetDegree(0.0);
    Sleep(5500);
    lever_servo.SetDegree(90.0);

    // Move backward
    move_backward(50, 1.0);

    // Turn left
    turnLeft(40, 20.0);

    // Adjust heading
    RPS_Angle(20.0);

    // Go straight
    move_forward(50, 1.5);

    // Turn left
    turnLeft(40, 65.0);

    // Face towards acrylic ramp
    RPS_Angle(88.0);
}

/*
 * Does everything from going up the ramp to immediately before turning to face towards the lever.
 */
void doFoosball() {

    // Store current location
    X_coord = RPS.X();
    Y_coord = RPS.Y();

    // Move forward to top of course
    move_forward(80, 25.0);

    // Adjust heading on top of the ramp
    RPS_Angle(90.0);

    // Go straight off ramp
    move_forward(70, 12.0);
    Sleep(100);

    if (RPS.X() < 30.2) {
        RPS_Angle(89.0);
    } else if (RPS.X() > 30.7) {
        RPS_Angle(93.0);
    } else {
        RPS_Angle(90.0);
    }
    /* Adjust heading
    RPS_Angle(90.0); //Was 88.0 before 4/3 */

    // Go straight towards foosball
    move_forward(70, 12.5);

    RPS_Angle(90.0);

    Sleep(250); //Sleep functions added as of 4/3

    // Adjust y-location
    RPS_Y_inc_abs(foosballDistY);

    Sleep(100);

    // Adjust heading
    RPS_Angle(90.0);

    // Turn right
    turnRight(50, 40.0);

    // Go straight
    move_backward(40, 2.5);

    // Turn right
    turnRight(50, 25.0);

    // Go straight
    move_forward(30, 1.5);

    // Turn right
    turnRight(50, 15.0);

    // Go straight
    move_forward(50, 1.5);

    // Turn right
    turnRight(50, 10.0);

    // Go straight 1500 ms
    bl_motor.SetPercent(50);
    fr_motor.SetPercent(-1 * 50);
    fl_motor.SetPercent(50);
    br_motor.SetPercent(-1 * 50);
    Sleep(1500);
    bl_motor.Stop();
    fr_motor.Stop();
    fl_motor.Stop();
    br_motor.Stop();

    // Go straight
    move_backward(50, 0.5);

    // Grab foosball rings
    lever_servo.SetDegree(168.0);

    // Store current location
    X_coord = RPS.X();
    Y_coord = RPS.Y();

    // Go straight
    move_backward(30, 6.0);

    // Raise lever arm
    lever_servo.SetDegree(90.0);

    // Adjust heading
    RPS_Angle(358.0);

    // Go straight
    move_forward(50, 3.0);

    // Grab foosball rings
    lever_servo.SetDegree(168.0);

    // Go straight
    move_backward(40, 6.5);

    // Raise lever arm a little
    lever_servo.SetDegree(150.0);

    Sleep(200);

    // Go straight
    move_forward(50, 1.0);

    Sleep(200);

    // Raise lever arm
    lever_servo.SetDegree(90.0);

    // Adjust heading
    RPS_Angle(357.0); //Used to be 0.0

    // Go straight
    move_backward(50, 1.0);
}

/*
 * Does everything from end of foosball task to immediately before turning to square-up against the left wall.
 */
void doLever() {
    // Go straight
    move_backward(50, 6.8);

    // Turn right
    turnRight(40, 20.0);
    Sleep(100); //Break into two turns as of 4/3

    move_backward(40, 1.2);
    Sleep(100);

    // Turn right
    turnRight(40, 45.0);

    // Adjust heading
    RPS_Angle(308.0); //Originally 315.0

    // Go straight
    move_backward(80, 5.5);

    // Push down lever
    lever_servo.SetDegree(5.0);

    Sleep(500);

    // Raise lever arm
    lever_servo.SetDegree(90.0);

    Sleep(500);

    // Go straight
    move_forward(50, 3.7);

    // Turn right
    turnRight(50, 120.0);

    // Go straight
    move_forward(70, 15.0);

    // Turn right
    turnLeft(40, 25.0);

    // Adjust heading
    RPS_Angle(270.0);

    // Go straight
    move_forward(70, 9.0);

    // Adjust y position
    RPS_Y_dec_abs(bumpY);
}

/*
 * Does the squaring up, leading to the token task.
 */
void doToken() {
    // Go straight
    move_backward(70, 1.0);

    // Turn right
    turnRight(40, 100.0);

    // Adjust heading
    RPS_Angle(180.0);

    // Go straight 2000 ms
    bl_motor.SetPercent(50);
    fr_motor.SetPercent(-1 * 50);
    fl_motor.SetPercent(50);
    br_motor.SetPercent(-1 * 50);
    Sleep(2000);
    bl_motor.Stop();
    fr_motor.Stop();
    fl_motor.Stop();
    br_motor.Stop();

    // Store current position
    X_coord = RPS.X();
    Y_coord = RPS.Y();

    // Go to token slot
    move_backward(50, 2.0);

    // Turn right a little
    turnRight(40, 25.0);

    // Go straight
    move_backward(40, 2.5);

    // Turn left a little
    turnLeft(40, 25.0);

    // Adjust heading
    RPS_Angle(180.0);

    // Go straight
    move_backward(50, 2.5);

    // Adjust x position
    RPS_Xinc_rev(X_coord, 9.0);

    // Drop token
    token_servo.SetDegree(170.0);
    Sleep(2000);
    token_servo.SetDegree(90.0);
    Sleep(500);
}

/*
 * Final function. From after completing the token task to pressing the final button.
 */
void finish() {

    move_forward(70, 10.0);

    turnLeft(40, 90.0);

    move_forward(80, 20.0);

    RPS_Angle(270.0);

    // Hit final red button
    bl_motor.SetPercent(90);
    fr_motor.SetPercent(-90);
    fl_motor.SetPercent(90);
    br_motor.SetPercent(-90);
}

/*
 * This function is used to store 4 essential locations to execute a perfect run.
 * Utilizes manual placement of robot and touch screen to store current robot cordinates.
 */
void calibrate(){
    int i = 1;
    float x_position, y_position;

    // Store location of desired distance from start
    while(i == 1){
        // Print menu
        LCD.DrawRectangle(55, 45, 200, 150);
        LCD.WriteAt("Store POS1", 100, 126);

        LCD.Touch(&x_position, &y_position);

        while(!LCD.Touch(&x_position, &y_position)){
            // Print RPS values
            LCD.WriteAt("RPS X: ", 10, 210);
            LCD.WriteAt(RPS.X(), 70, 210);
            LCD.WriteAt("RPS Y: ", 130, 210);
            LCD.WriteAt(RPS.Y(), 190, 210);
        }
        Sleep(500);
        if(y_position > 45 && y_position < 195 && x_position > 55 && x_position < 255){
            startingPointY = RPS.Y();
            i++;
        }
    }

    // Store location of DDR light
    while(i == 2){
        // Print menu
        LCD.DrawRectangle(55, 45, 200, 150);
        LCD.WriteAt("Store POS2", 100, 126);

        LCD.Touch(&x_position, &y_position);

        while(!LCD.Touch(&x_position, &y_position)){
            // Print RPS values
            LCD.WriteAt("RPS X: ", 10, 210);
            LCD.WriteAt(RPS.X(), 70, 210);
            LCD.WriteAt("RPS Y: ", 130, 210);
            LCD.WriteAt(RPS.Y(), 190, 210);
        }
        Sleep(500);
        if(y_position > 45 && y_position < 195 && x_position > 55 && x_position < 255){
            ddrLightX = RPS.X();
            i++;
        }
    }

    // Store location of distance from foosball structure
    while(i == 3){
        // Print menu
        LCD.DrawRectangle(55, 45, 200, 150);
        LCD.WriteAt("Store POS3", 100, 126);

        LCD.Touch(&x_position, &y_position);

        while(!LCD.Touch(&x_position, &y_position)){
            // Print RPS values
            LCD.WriteAt("RPS X: ", 10, 210);
            LCD.WriteAt(RPS.X(), 70, 210);
            LCD.WriteAt("RPS Y: ", 130, 210);
            LCD.WriteAt(RPS.Y(), 190, 210);
        }
        Sleep(500);
        if(y_position > 45 && y_position < 195 && x_position > 55 && x_position < 255){
            foosballDistY = RPS.Y();
            i++;
        }
    }

    // Store location of bump
    while(i == 4){
        // Print menu
        LCD.DrawRectangle(55, 45, 200, 150);
        LCD.WriteAt("Store POS4", 100, 126);

        LCD.Touch(&x_position, &y_position);

        while(!LCD.Touch(&x_position, &y_position)){
            // Print RPS values
            LCD.WriteAt("RPS X: ", 10, 210);
            LCD.WriteAt(RPS.X(), 70, 210);
            LCD.WriteAt("RPS Y: ", 130, 210);
            LCD.WriteAt(RPS.Y(), 190, 210);
        }
        Sleep(500);
        if(y_position > 45 && y_position < 195 && x_position > 55 && x_position < 255){
            bumpY = RPS.Y();
            i++;
        }
    }
}

/*
 * Critical function in that it sets up everything beforehand:
 * the servo initializations and their initial positions and calibration.
 */
void initialize(){
    // min for lever servo: 725
    // max for lever servo: 2468
    // min for token servo: 514
    // max for token servo: 2430

    float x_position, y_position;

    RPS.InitializeTouchMenu();

    //Initialize the screen
    LCD.Clear(BLACK);
    LCD.SetFontColor(WHITE);
    LCD.WriteLine("Initializing...");

    lever_servo.SetMin(725);
    lever_servo.SetMax(2468);
    token_servo.SetMin(514);
    token_servo.SetMax(2430);
    Sleep(500);

    lever_servo.SetDegree(90);
    token_servo.SetDegree(85);
    Sleep(1000);

    LCD.Clear();
    LCD.WriteLine("Begin Calibration");

    calibrate();

    Sleep(500);
    // Wait for final action
    LCD.Clear();
    LCD.Write("Touch anywhere to begin");
    while(!LCD.Touch(&x_position, &y_position));

    // Store ambient light condition
    ambient = cds.Value();
}

/*
 * Main function.
 */
int main() {
    initialize();
    waitForLight();
    doDDR();
    doFoosball();
    doLever();
    doToken();
    finish();
}
