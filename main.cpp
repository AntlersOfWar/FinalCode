#include <FEHLCD.h>
#include <FEHIO.h>
#include <FEHUtility.h>
#include <FEHMotor.h>
#include <FEHRPS.h>
#include <FEHServo.h>

#define WHEEL_RADIUS 1.375
#define COUNTS_PER_REV 48
#define PI 3.14159265
#define ROBOT_RADIUS 4.0
#define OFFSET 8.0
#define RED_THRESH 0.7

/* TODO: Make sure you have the right number of motors/encoders declared. */
FEHServo lever_servo(FEHServo::Servo6);

/* NOTE: P3_6 and P3_7 cannot be used for digital encoders. Also, "fr" means front-right. "bl" means back-left. */
DigitalEncoder fl_encoder(FEHIO::P1_0);
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
}

/*
 * Given a motor speed (@param percent) and a desired distance (@param inches),
 * drives the robot in the opposite direction from move_forward.
 */
void move_backward(int percent, float inches) {
    //Reset all encoder counts
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
}

/*
 * Given a motor speed (@param percent) and a desired degree (@param degrees),
 * turns the robot to the left about the centerpoint of the robot.
 */
void turnLeft(int percent, float degrees) {
    //Reset all encoder counts
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
}

/*
 * Given a motor speed (@param percent) and a desired degree (@param degrees),
 * turns the robot to the right about the centerpoint of the robot.
 */
void turnRight(int percent, float degrees) {
    //Reset all encoder counts
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

void RPS_X(float startX, float inches) {
    if (RPS.X() < startX + (inches - 0.2)) {
        LCD.Clear();
        LCD.WriteLine("Too short!");
        bl_motor.SetPercent(15);
        fr_motor.SetPercent(-15);
        fl_motor.SetPercent(15);
        br_motor.SetPercent(-15);
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
        bl_motor.SetPercent(-15);
        fr_motor.SetPercent(15);
        fl_motor.SetPercent(-15);
        br_motor.SetPercent(15);
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
}

void RPS_Y(float startY, float inches) {
    if (RPS.Y() < startY + (inches - 0.2)) {
        LCD.Clear();
        LCD.WriteLine("Too short!");
        bl_motor.SetPercent(15);
        fr_motor.SetPercent(-15);
        fl_motor.SetPercent(15);
        br_motor.SetPercent(-15);
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
        bl_motor.SetPercent(-15);
        fr_motor.SetPercent(15);
        fl_motor.SetPercent(-15);
        br_motor.SetPercent(15);
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
}

void RPS_Angle(float desiredDeg) {
    if (RPS.Heading() < desiredDeg - 1.0) {
        LCD.Clear();
        LCD.WriteLine("Angle short!");
        bl_motor.SetPercent(-15);
        fr_motor.SetPercent(-15);
        fl_motor.SetPercent(-15);
        br_motor.SetPercent(-15);
        while (RPS.Heading() < desiredDeg) {
            LCD.WriteRC(RPS.X(),2,12);
            LCD.WriteRC(RPS.Y(),3,12);
            LCD.WriteRC(RPS.Heading(),4,12);
        }
        bl_motor.Stop();
        fr_motor.Stop();
        fl_motor.Stop();
        br_motor.Stop();
    } else if (RPS.Heading() > desiredDeg + 1.0) {
        LCD.Clear();
        LCD.WriteLine("Angle over!");
        bl_motor.SetPercent(15);
        fr_motor.SetPercent(15);
        fl_motor.SetPercent(15);
        br_motor.SetPercent(15);
        while (RPS.Heading() > desiredDeg) {
            LCD.WriteRC(RPS.X(),2,12);
            LCD.WriteRC(RPS.Y(),3,12);
            LCD.WriteRC(RPS.Heading(),4,12);
        }
        bl_motor.Stop();
        fr_motor.Stop();
        fl_motor.Stop();
        br_motor.Stop();
    }
}

void waitForLight() {
    //Initialize the screen
    LCD.Clear(BLACK);
    LCD.SetFontColor(WHITE);

    while(cds.Value() > RED_THRESH) {
        LCD.Clear();
        LCD.WriteLine("Looking for Red Light...");
        LCD.WriteLine(cds.Value());
    }
}

/*
 * TODO: Fill in all the functions with appropriate movements. As of 3/6/19, all functions will do their respective task starting from the start.
 * Later on, only one of the functions (doDDR()) will have the waitForLight() function. The others will have to go off the previous task function called.
 */

void doDDR() {
    waitForLight();

}

void doLever() {
    waitForLight();
}

void doToken() {
    waitForLight();
}

void doFoosball() {
    waitForLight();
}

void finish() {

}

int main()
{
    // min for main servo: 725
    // max for main servo: 2468
    // min for lever servo: 514
    // max for lever servo: 2430

}

