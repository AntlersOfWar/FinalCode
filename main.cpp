#include <FEHLCD.h>
#include <FEHIO.h>
#include <FEHUtility.h>
#include <FEHMotor.h>
#include <FEHRPS.h>
#include <FEHServo.h>
#include <FEHSD.h>

#define WHEEL_RADIUS 1.375
#define COUNTS_PER_REV 48
#define PI 3.14159265
#define ROBOT_RADIUS 4.5
#define QR_OFFSET 2.0
#define RED_THRESH 0.7
#define BLUE_THRESH 1.5
float X_coord;
float Y_coord;

/* TODO: Make sure you have the right number of motors/encoders declared. */
FEHServo lever_servo(FEHServo::Servo6);
FEHServo token_servo(FEHServo::Servo0);

/* NOTE: P3_6 and P3_7 cannot be used for digital encoders. Also, "fr" means front-right. "bl" means back-left. */
DigitalEncoder fl_encoder(FEHIO::P1_1);
DigitalEncoder br_encoder(FEHIO::P2_0);

//Declare motors
FEHMotor bl_motor(FEHMotor::Motor0, 5.0);
FEHMotor fr_motor(FEHMotor::Motor3, 5.0);
FEHMotor fl_motor(FEHMotor::Motor1, 5.0);
FEHMotor br_motor(FEHMotor::Motor2, 5.0);


//Declare CdS cell
AnalogInputPin cds(FEHIO::P0_4);

/*
 * Given a distance in inches, returns the theoretical counts.
 */
int theoreticalCounts(float inches) {
    int counts = (inches * COUNTS_PER_REV) / (2 * PI * WHEEL_RADIUS);
    return counts;
}

/*
 * Given a degree, returns the theoretical counts (if the robot were to turn about the center).
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
    Sleep(100);
    fl_encoder.ResetCounts();
    br_encoder.ResetCounts();
    Sleep(100);

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
    Sleep(100);
}

/*
 * Given a motor speed (@param percent) and a desired distance (@param inches),
 * drives the robot in the opposite direction from move_forward.
 */
void move_backward(int percent, float inches) {
    //Reset all encoder counts
    Sleep(100);
    fl_encoder.ResetCounts();
    br_encoder.ResetCounts();
    Sleep(100);

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
    Sleep(100);
}

/*
 * Given a motor speed (@param percent) and a desired degree (@param degrees),
 * turns the robot to the left about the centerpoint of the robot.
 */
void turnLeft(int percent, float degrees) {
    //Reset all encoder counts
    Sleep(100);
    fl_encoder.ResetCounts();
    br_encoder.ResetCounts();
    Sleep(100);

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
    Sleep(100);
}

/*
 * Given a motor speed (@param percent) and a desired degree (@param degrees),
 * turns the robot to the right about the centerpoint of the robot.
 */
void turnRight(int percent, float degrees) {
    //Reset all encoder counts
    Sleep(100);
    fl_encoder.ResetCounts();
    br_encoder.ResetCounts();
    Sleep(100);

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
    Sleep(100);
}

/*
 * Given a reference point (@param startX) and the desired displacement (@param inches),
 * moves robot in X direction to the location relative to the starting point.
 * (if robot move_forward direction faces positive X)
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
 * Given a reference point (@param startX) and the desired displacement (@param inches),
 * moves robot in X direction to the location relative to the starting point.
 * (if robot move_forward direction faces negative X)
 */
void RPS_Xdec(float startX, float inches) {
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
 * Given a reference point (@param startY) and the desired displacement (@param inches),
 * moves robot in Y direction to the location relative to the starting point.
 * (if robot move_forward direction faces positive Y)
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
 * Given a reference point (@param startY) and the desired displacement (@param inches),
 * moves robot in Y direction to the location relative to the starting point.
 * (if robot move_forward direction faces negative Y)
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
 * Given a desired angle (@param desiredDeg), rotates the
 * robot until desired angle is achieved.
 */
void RPS_Angle(float desiredDeg) {
    Sleep(100);
    // If desired heading is zero degrees, go to next task
    if (desiredDeg > 5.0) {
        // If robot heading is below desired value
        if (RPS.Heading() < desiredDeg - 1.0) {
            LCD.Clear();
            LCD.WriteLine("Angle short!");

            // Turn left
            bl_motor.SetPercent(-30);
            fr_motor.SetPercent(-30);
            fl_motor.SetPercent(-30);
            br_motor.SetPercent(-30);

            // Loop until heading is desired angle
            while (RPS.Heading() < desiredDeg) {
                // Write RPS values to the screen
                LCD.WriteRC(RPS.X(),2,12);
                LCD.WriteRC(RPS.Y(),3,12);
                LCD.WriteRC(RPS.Heading(),4,12);

                // Wait until signal is regained if lost
                if (RPS.X() < 0){
                    // Stop motors
                    bl_motor.Stop();
                    fr_motor.Stop();
                    fl_motor.Stop();
                    br_motor.Stop();
                    // Wait
                    while(RPS.X() < 0);
                    // Restart motors
                    bl_motor.SetPercent(-30);
                    fr_motor.SetPercent(-30);
                    fl_motor.SetPercent(-30);
                    br_motor.SetPercent(-30);
                }
            }
            // Stop motors
            bl_motor.Stop();
            fr_motor.Stop();
            fl_motor.Stop();
            br_motor.Stop();
        }
        // If robot heading is above desired value
        else if (RPS.Heading() > desiredDeg + 1.0) {
            LCD.Clear();
            LCD.WriteLine("Angle over!");

            // Turn right
            bl_motor.SetPercent(30);
            fr_motor.SetPercent(30);
            fl_motor.SetPercent(30);
            br_motor.SetPercent(30);

            // Loop until heading is desired angle
            while (RPS.Heading() > desiredDeg) {
                // Write RPS values to the screen
                LCD.WriteRC(RPS.X(),2,12);
                LCD.WriteRC(RPS.Y(),3,12);
                LCD.WriteRC(RPS.Heading(),4,12);

                // Wait until signal is regained if lost
                if (RPS.X() < 0){
                    // Stop motors
                    bl_motor.Stop();
                    fr_motor.Stop();
                    fl_motor.Stop();
                    br_motor.Stop();
                    // Wait
                    while(RPS.X() < 0);
                    // Restart motors
                    bl_motor.SetPercent(-30);
                    fr_motor.SetPercent(-30);
                    fl_motor.SetPercent(-30);
                    br_motor.SetPercent(-30);
                }
            }
            // Stop motors
            bl_motor.Stop();
            fr_motor.Stop();
            fl_motor.Stop();
            br_motor.Stop();
        }
    }
    // Run this task if the desired heading is zero degrees
    else {
        // If robot is angled below the positive x-axis (4th quadrant)
        if (RPS.Heading() > 270.0 && RPS.Heading() < 359.9) {
            LCD.Clear();
            LCD.WriteLine("Angle short!");

            // Turn left
            bl_motor.SetPercent(-30);
            fr_motor.SetPercent(-30);
            fl_motor.SetPercent(-30);
            br_motor.SetPercent(-30);

            // While the robot has a large angle (flips to 0.0 degrees after reaching 359.9 degrees)
            while (RPS.Heading() > 2.0) {
                // Write RPS values to the screen
                LCD.WriteRC(RPS.X(),2,12);
                LCD.WriteRC(RPS.Y(),3,12);
                LCD.WriteRC(RPS.Heading(),4,12);

                // Wait until signal is regained if lost
                if (RPS.X() < 0){
                    // Stop motors
                    bl_motor.Stop();
                    fr_motor.Stop();
                    fl_motor.Stop();
                    br_motor.Stop();
                    // Wait
                    while(RPS.X() < 0);
                    // Restart motors
                    bl_motor.SetPercent(-30);
                    fr_motor.SetPercent(-30);
                    fl_motor.SetPercent(-30);
                    br_motor.SetPercent(-30);
                }
            }
            // Stop motors
            bl_motor.Stop();
            fr_motor.Stop();
            fl_motor.Stop();
            br_motor.Stop();
        }
        // If robot is angled above the positive x-axis (1st quadrant)
        else if (RPS.Heading() < 90.0 && RPS.Heading() > 0.1) {
            LCD.Clear();
            LCD.WriteLine("Angle over!");

            // Turn left
            bl_motor.SetPercent(30);
            fr_motor.SetPercent(30);
            fl_motor.SetPercent(30);
            br_motor.SetPercent(30);

            // While the robot has a small angle (flips to 359.9 degrees after reaching 0.0 degrees)
            while (RPS.Heading() < 358.0) {
                // Write RPS values to the screen
                LCD.WriteRC(RPS.X(),2,12);
                LCD.WriteRC(RPS.Y(),3,12);
                LCD.WriteRC(RPS.Heading(),4,12);

                // Wait until signal is regained if lost
                if (RPS.X() < 0){
                    // Stop motors
                    bl_motor.Stop();
                    fr_motor.Stop();
                    fl_motor.Stop();
                    br_motor.Stop();
                    // Wait
                    while(RPS.X() < 0);
                    // Restart motors
                    bl_motor.SetPercent(-30);
                    fr_motor.SetPercent(-30);
                    fl_motor.SetPercent(-30);
                    br_motor.SetPercent(-30);
                }
            }
            // Stop motors
            bl_motor.Stop();
            fr_motor.Stop();
            fl_motor.Stop();
            br_motor.Stop();
        }
    }
    Sleep(100);
}

/*
 * Given an absolute desired X position (@param inches),
 * moves robot in X direction to that X position.
 * (if robot move_forward direction faces negative X)
 */
void RPS_X_dec_abs(float inches) {
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
 * Function called at the beginning to start off based off the red start light,
 * or if 30 seconds has passed.
 */
void waitForLight() {

    float time = TimeNow();

    // If 30 seconds pass and no light is read, just start
    while(cds.Value() > RED_THRESH && TimeNow() - time < 30) {
        LCD.Clear();
        LCD.WriteLine("Looking for Red Light...");
        LCD.WriteLine(cds.Value());
    }
}

/*
 * Given a desired motor speed (@param percent), moves robot forward at that speed,
 * toward the DDR floor lights. The CdS cell should go over these lights.
 * Returns [red light was detected]
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
    while (!lightFound) {
        if (cds.Value() <= RED_THRESH) {
            bl_motor.Stop();
            fr_motor.Stop();
            fl_motor.Stop();
            br_motor.Stop();
            lightFound = true;
            redLight = true;
        } else if (cds.Value() > RED_THRESH && cds.Value() <= BLUE_THRESH) {
            bl_motor.Stop();
            fr_motor.Stop();
            fl_motor.Stop();
            br_motor.Stop();
            lightFound = true;
            redLight = false;
        }
    }
   return redLight;
}

/*
 * TODO: Fill in all the functions with appropriate movements. As of 3/6/19, all functions will do their respective task starting from the start.
 * Later on, only one of the functions (doDDR()) will have the waitForLight() function. The others will have to go off the previous task function called.
 */

void doDDR() {
    // Start
    move_forward(50, 2.8);

    // Turn right
    turnRight(40, 40.0);

    // Adjust heading
    RPS_Angle(3.0);

    // Store current location
    X_coord = RPS.X();
    Y_coord = RPS.Y();

    // Go straight
    move_forward(50, 10.0); //move_forward(50, 17.0);

    // Go straight and check for DDR light color (new as of 3/26)
    bool redLight = checkDDRLight(20);

    if (redLight) {
        //hit red button
    } else {
        //hit blue button
    }

    // Adjust x-location
    RPS_Xinc(X_coord, 16.0 + QR_OFFSET);

    // Press RPS button
    lever_servo.SetDegree(10.0);
    Sleep(5000);
    lever_servo.SetDegree(90.0);

    // Move backward
    move_backward(50, 1.0);

    // Turn left
    turnLeft(40, 20.0);

    // Adjust heading
    RPS_Angle(20.0);

    // Go straight
    move_forward(50, 1.0);

    // Turn left
    turnLeft(40, 65.0);

    // Face towards acrylic ramp
    RPS_Angle(88.0);
}

void doLever() {
        move_backward(50, 5.0);

        turnLeft(40, 40.0);

        /*
         * TODO: Fill stuff in for lever, look at course and determine what needs to be done (instead of pushing it down, sweep it)
         */

        lever_servo.SetDegree(40.0);
        Sleep(2000);

        lever_servo.SetDegree(120.0);
        Sleep(500);

        turnRight(50, 130.0);

        RPS_Angle(270.0);
}

void doToken() {

    /*
    // Go straight
    move_forward(50, 2.8);

    // Turn right
    turnRight(40, 50.0);

    // Adjust heading
    RPS_Angle(3.0);

    // Store current location
    X_coord = RPS.X();
    Y_coord = RPS.Y();

    // Go straight
    move_forward(50, 17.0);

    // Adjust x-location
    RPS_Xinc(X_coord, 16.0 + QR_OFFSET);

    // Press RPS button
    lever_servo.SetDegree(10.0);
    Sleep(5000);
    lever_servo.SetDegree(90.0);

    // Move backward
    move_backward(50, 1.0);

    // Turn left
    turnLeft(40, 20.0);

    // Adjust heading
    RPS_Angle(20.0);

    // Store current location
    X_coord = RPS.X();
    Y_coord = RPS.Y();

    // Go straight
    move_forward(50, 1.0);

    // Turn left
    turnLeft(40, 65.0);

    // Face towards acrylic ramp
    RPS_Angle(88.0);

    // Store current location
    X_coord = RPS.X();
    Y_coord = RPS.Y();

    // Move forward to top of course
    move_forward(80, 23.0);

    // Adjust heading on the ramp
    RPS_Angle(90.0);

    // Go straight
    move_forward(50, 23.0);

    // Adjust y-location
    RPS_Yinc(Y_coord, 45.0 + QR_OFFSET);

    // Adjust heading
    RPS_Angle(90.0);

    // Turn left
    turnLeft(40, 121.0);

    // Adjust heading
    RPS_Angle(235.0);

    // Go forward
    move_forward(50, 26.0);

    for(int i = 1; i < 4; i++){
        // Turn right
        turnRight(40, 19.0);

        // Go straight
        move_forward(50, 1.5);
    }


    // Adjust heading
    RPS_Angle(180.0);

    // Store current location
    X_coord = RPS.X();
    Y_coord = RPS.Y();

    // Go forward
    move_forward(50, 3.0);


    // Adjust x-location in reference to absolute coordinates
    RPS_X_dec_abs(11.0);

    Sleep(1000);

    // Drop token
    token_servo.SetDegree(167.0);
    Sleep(1000);
    token_servo.SetDegree(90.0);
    Sleep(500);

    for(int i = 1; i < 4; i++){
        // Turn right
        turnRight(40, 30.0);

        // Go straight
        move_forward(50, 2.0);
    }

    // Go straight
    move_forward(50, 11.0);

    // Turn right
    turnRight(40, 120.0);

    // Move arm down
    lever_servo.SetDegree(160.0);

    // Turn right
    turnRight(40, 120.0);

    // Move arm up
    lever_servo.SetDegree(90.0);

    // Go straight
    move_forward(80, 35.0);
    */

    // Go straight, towards ramp, to try to align with token
    move_forward(50, 10.0);

    turnRight(50, 90.0);

    RPS_Angle(180.0);

    // Adjust x-location in reference to absolute coordinates
    RPS_X_dec_abs(11.0);

    Sleep(1000);

    // Drop token
    token_servo.SetDegree(167.0);
    Sleep(1000);
    token_servo.SetDegree(90.0);
    Sleep(500);
}

void doFoosball() {

    /*
    // Go straight
    move_forward(50, 2.8);

    // Turn right
    turnRight(40, 40.0);

    // Adjust heading
    RPS_Angle(3.0);

    // Store current location
    X_coord = RPS.X();
    Y_coord = RPS.Y();

    // Go straight
    move_forward(50, 17.0);

    // Adjust x-location
    RPS_Xinc(X_coord, 16.0 + QR_OFFSET);

    // Press RPS button
    lever_servo.SetDegree(10.0);
    Sleep(5000);
    lever_servo.SetDegree(90.0);

    // Move backward
    move_backward(50, 1.0);

    // Turn left
    turnLeft(40, 20.0);

    // Adjust heading
    RPS_Angle(20.0);

    // Store current location
    X_coord = RPS.X();
    Y_coord = RPS.Y();

    // Go straight
    move_forward(50, 1.0);

    // Turn left
    turnLeft(40, 65.0);

    Sleep(100);

    // Face towards acrylic ramp
    RPS_Angle(88.0);
    */

    // Store current location
    X_coord = RPS.X();
    Y_coord = RPS.Y();

    // Move forward to top of course
    move_forward(80, 23.0);
    Sleep(300);

    // Adjust heading on top of the ramp
    RPS_Angle(89.0);

    // Go straight towards foosball
    move_forward(50, 23.0);

    // Adjust y-location
    RPS_Yinc(Y_coord, 46.5 + QR_OFFSET);

    // Adjust heading
    RPS_Angle(90.0);
//Sleep(250);

    // Turn right
    turnRight(50, 20.0);
//Sleep(250);

    // Go straight
    move_backward(40, 2.5);
//Sleep(250);

    // Turn right
    turnRight(50, 30.0);

    // Go straight
    move_forward(30, 1.5);

    // Turn right
    turnRight(50, 25.0);

    // Go straight
    move_forward(50, 1.5);

    // Turn right
    turnRight(50, 10.0);

    // Go straight
    move_forward(50, 2.0);

    Sleep(250);

    // Go straight
    move_forward(60, 1.0);

    Sleep(1000);

    // Go straight
    move_backward(50, 0.5);

    // Grab foosball rings
    lever_servo.SetDegree(173.0);

    // Store current location
    X_coord = RPS.X();
    Y_coord = RPS.Y();

    // Go straight
    move_backward(30, 6.0);

    // Raise lever arm
    lever_servo.SetDegree(90.0);

    // Go straight
    move_forward(50, 2.5);

    // Grab foosball rings
    lever_servo.SetDegree(173.0);

    // Go straight
    move_backward(30, 6.0);

    // Raise lever arm
    lever_servo.SetDegree(90.0);

    /*
    // Turn right
    turnRight(40, 15.0);

    // Adjust angle
    RPS_Angle(345.0);

    // Go straight
    move_forward(50, 7.0);

    // Turn right
    turnRight(40, 45.0);

    // Adjust angle
    RPS_Angle(270.0);

    // Store current location
    X_coord = RPS.X();
    Y_coord = RPS.Y();

    // Move forward to bottom of course
    move_forward(80, 23.0);

    Sleep(500);

    // Adjust heading on the ramp
    RPS_Angle(270.0);

    // Go straight
    move_forward(50, 23.0);

    // Adjust y-location
    RPS_Ydec(Y_coord, 46.0 + QR_OFFSET);

    // Turn left
    turnRight(40, 80.0);

    // Adjust angle
    RPS_Angle(170.0);

    // Go straight
    move_forward(50, 24.0);
    */
}

void finish() {
    turnRight(40, 20.0);

    move_forward(50, 6.0);

    turnLeft(50, 110.0);

    RPS_Angle(270.0);

    //Clear bump
    move_forward(40, 5.0);

    //Hit final red button
    bl_motor.SetPercent(90);
    fr_motor.SetPercent(-90);
    fl_motor.SetPercent(90);
    br_motor.SetPercent(-90);
}

void initialize(){
    // min for lever servo: 725
    // max for lever servo: 2468
    // min for token servo: 514
    // max for token servo: 2430

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
    token_servo.SetDegree(90);
    Sleep(1000);

    LCD.Clear();
    LCD.WriteLine("Ready!!!");

    Sleep(3000);
}

int main() {
    initialize();   // Run through startup sequence
    waitForLight(); // Wait for start light
    doDDR();        // Execute DDR task
    doFoosball();   // Execute foosball task
    doLever();      // Execute lever task
    doToken();
    finish();
}

