#include <FEHLCD.h>
#include <FEHIO.h>
#include <FEHUtility.h>
#include <FEHMotor.h>
#include <FEHServo.h>


#define PI 3.141592653589793238462
#define SHAFT_ENCODER_CYCLES_PER_INCH 100/(2.5*PI) // Modify this value
#define SHAFT_ENCODER_INCHES_PER_CYCLE 1/SHAFT_ENCODER_CYCLES_PER_INCH
#define SHAFT_ENCODER_COUNTS_FOR_45 46
#define SHAFT_ENCODER_COUNTS_FOR_90 92 //Determine these two by trial and error


//Define world states

#define WAITING_FOR_START 1
#define GO_UP_RAMP 2
#define PUSH_BUTTON 3
#define MANUEVER_FOR_LEVER 4
#define PULL_LEVER 5
#define MANUEVER_FOR_SAMPLE 6
#define RETRIEVE_SAMPLE 7
#define MANUEVER_TO_LOWER_LEVEL 8
#define MANUEVER_TO_LIGHT 9
#define DROP_IN_BUCKET 10
#define MANUEVER_TO_ANTENNA 11
#define TURN_ANTENNA 12
#define MOVE_TO_BUNKER 13
#define DED 14


//Note that the front (left) motor turns in the opposite direction of the back wheel. This must be accounted for through code.
FEHMotor leftGMH(FEHMotor::Motor0,7.2);
FEHMotor rightGMH(FEHMotor::Motor1,7.2);

AnalogInputPin CdSCell(FEHIO::P1_0);//Change the port as needed.



DigitalInputPin NWBump(FEHIO::P1_0);
DigitalInputPin NEBump(FEHIO::P0_2);
DigitalInputPin SWBump(FEHIO::P3_1);
DigitalInputPin SEBump(FEHIO::P0_6);

DigitalEncoder right_encoder(FEHIO::P0_0);
DigitalEncoder left_encoder(FEHIO::P3_2); // Break beam, counts per transition.


FEHServo forkliftFutaba(FEHServo::Servo0);
// Add more sensors after this.
//Optosenser and such

//Function prototypes from here on.


void waitForStart(); // Wait for the light to toggle on

void killMotors(int restPeriod);//Stop both motors. Will wait for rest period that is inputted after motors are stopped.

void turn90CW(); // Turn 90 degrees clockwise

void turn45CW(); // Turn 45 degrees Clockwise

void turn90CCW(); // Turn 90 degrees counterclockwise

void turn45CCW(); // Turn 45 degrees counterclockwise

void driveUntilCollision(int percent); // Drive in a straightline until collision

void reverseUntilCollision(int percent); //Reverse until a collision

void driveFor(float distance,int percent); //Drive straight for distance

void setForklift(int degrees); //Lowers the forklift using servos 0 degrees is the resting position 90 is the horizontal degree




//From here, the more complex methods for specific tasks will be prototyped

void startManuevers(); // Perform the initial backing up

void driveUpRamp(); // Requires some unique stuff

void followLine();//Follows the line until the forklift is under the core sample.

void alignToButton(); //From the wall on the top part of the course.
//Align the robot appropriatley in the horizontal direction such that if it were to turn 90 degrees and drive straight it would push the seismograph button.

void alignToLever(); // From the button, reverse and align to the lever.

void pullLever(); // Pull the lever and raise the forklift.

void checkErrors(int state); // Check and correct errors



int main(void){
    int state = WAITING_FOR_START;
    char color = 'n'; // for none

    while(true){

       switch(state){

            case WAITING_FOR_START:
            waitForStart();
            state = GO_UP_RAMP;
           break;

            case GO_UP_RAMP:
            startManuevers();
            driveUpRamp();
            state = PUSH_BUTTON;
           break;

            case PUSH_BUTTON:
            alignToButton();
            driveFor(14.0,60);
            killMotors(7000);
            state = MANUEVER_FOR_LEVER;
           break;

            case MANUEVER_FOR_LEVER:
            alignToLever();
            driveFor(14,60);
            killMotors(0);
            state = PULL_LEVER;
           break;

            case PULL_LEVER:
                pullLever();
                driveFor(-10.0,50);
                turn90CCW();
                state = DED;
           break;
            case MANUEVER_FOR_SAMPLE:

            case RETRIEVE_SAMPLE:

            case MANUEVER_TO_LOWER_LEVEL:
//                driveFor(20.0,30);
//                turn();//New method turn to face the light
            case MANUEVER_TO_LIGHT:
//                  driveUntilLight();
//                  printLight(); Two new methods for strech goal.
            case DROP_IN_BUCKET:

            case MANUEVER_TO_ANTENNA:


            case TURN_ANTENNA:


            case MOVE_TO_BUNKER:


            case DED:
            killMotors(0);
            LCD.Clear();
            while(true){
                LCD.WriteLine("This is the end.");
                Sleep(100);
                LCD.WriteLine("Esto es el fin");
                Sleep(100);
                LCD.WriteLine("C'est la fin");
                Sleep(100);
                LCD.WriteLine("यह अंत है");
                Sleep(100);
                 LCD.WriteLine("هذه هي النهاية");
                Sleep(100);
                LCD.WriteLine("這就是結局");
                Sleep(100);
                LCD.WriteLine("これで終わりだ");
                Sleep(100);
            }
            break;
        }
}



}

void waitForStart(){
    float x,y;
   while(!LCD.Touch(&x,&y));
   while(LCD.Touch(&x,&y));

   while(CdSCell.Value() > 1.0){
       LCD.Clear();
       LCD.Write("Voltage = " );
       Sleep(100);
       LCD.WriteLine(CdSCell.Value());
   } // Determine threshold based on color of lights
}

void killMotors(int restPeriod){
    leftGMH.Stop();
    rightGMH.Stop();
    Sleep(restPeriod);
}

void driveFor(float distance, int percent){
    //Shaft encoding required.
    left_encoder.ResetCounts();
    right_encoder.ResetCounts();
    leftGMH.SetPercent(percent);
    rightGMH.SetPercent(percent);

    while((SHAFT_ENCODER_INCHES_PER_CYCLE)* ((left_encoder.Counts() + right_encoder.Counts())/2) <distance){
        //Check for possible errors in here.
    }

    leftGMH.Stop();
    rightGMH.Stop();
}

void turn90CW(){
    left_encoder.ResetCounts();
    right_encoder.ResetCounts();
    leftGMH.SetPercent(50);
    rightGMH.SetPercent(-50);

    while((left_encoder.Counts() + right_encoder.Counts())/2<SHAFT_ENCODER_COUNTS_FOR_90){
        //Check for errors
    }
    leftGMH.Stop();
    rightGMH.Stop();

}

void turn90CCW(){
    left_encoder.ResetCounts();
    right_encoder.ResetCounts();
    leftGMH.SetPercent(-50);
    rightGMH.SetPercent(50);

    while((left_encoder.Counts() + right_encoder.Counts())/2<SHAFT_ENCODER_COUNTS_FOR_90){
        //Check for errors
    }
    leftGMH.Stop();
    rightGMH.Stop();
}

void turn45CW(){
    left_encoder.ResetCounts();
    right_encoder.ResetCounts();
    leftGMH.SetPercent(50);
    rightGMH.SetPercent(-50);

    while((left_encoder.Counts() + right_encoder.Counts())/2 <SHAFT_ENCODER_COUNTS_FOR_45){
        //Check for errors
    }
    leftGMH.Stop();
    rightGMH.Stop();
}

void turn45CCW(){
    left_encoder.ResetCounts();
    right_encoder.ResetCounts();
    leftGMH.SetPercent(-50);
    rightGMH.SetPercent(50);

    while((left_encoder.Counts() + right_encoder.Counts())/2 <SHAFT_ENCODER_COUNTS_FOR_45){
        //Check for errors
    }
    leftGMH.Stop();
    rightGMH.Stop();
} //Consider consolidation of all these methods into one turn method.

void driveUntilCollision(int percent){
    leftGMH.SetPercent(percent);
    rightGMH.SetPercent(percent);
    while(NEBump.Value() || NWBump.Value()){}//Will continue until both switches are pressed.

    killMotors(100);
}

void reverseUntilCollision(int percent){
    leftGMH.SetPercent(-percent);
    rightGMH.SetPercent(-percent);
    while(SWBump.Value() || SEBump.Value()){}//Will continue until both switches are pressed.

    killMotors(100);
}

void setForklift(float degWRYYYYYYYYYYY){
    forkliftFutaba.SetDegree(degWRYYYYYYYYYYY);
    //Check for problemos.
}

//Complex methods from here on out.

void startManuevers(){
    driveFor(6.5,-50);
    turn90CW();
    driveFor(10,40);
    turn90CCW(); // Face the ramp.
}

void driveUpRamp(){
    leftGMH.SetPercent(85);
    rightGMH.SetPercent(85);

    left_encoder.ResetCounts();
    right_encoder.ResetCounts();

    while((SHAFT_ENCODER_INCHES_PER_CYCLE)* (left_encoder.Counts() + right_encoder.Counts())/2 < 20.0){
        //Check for possible errors in here.
    }
    killMotors(0);
}

void alignToButton(){
    turn90CW();
    driveUntilCollision(40);
    driveFor(10.0,-60);
    turn90CCW();
}

void alignToLever(){
    driveFor(20,-60);
    turn90CCW();
}

void pullLever(){
    setForklift(90);
    Sleep(500);
    setForklift(0);

}
