#include <FEHLCD.h>
#include <FEHIO.h>
#include <FEHUtility.h>
#include <FEHMotor.h>
#include <FEHServo.h>
#include <FEHRPS.h>

#define PI 3.141592653589793238462
#define SHAFT_ENCODER_CYCLES_PER_INCH 12.7323954474 // Modify this value
#define SHAFT_ENCODER_INCHES_PER_CYCLE (1/SHAFT_ENCODER_CYCLES_PER_INCH)
#define SHAFT_ENCODER_COUNTS_FOR_45 46
#define SHAFT_ENCODER_COUNTS_FOR_90 92 //Determine these two by trial and error

//Servo min 500 max 2409


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
//TO DO: MOVE THE RPS POINT OF TURNING BACK. FIX THE SERVO DEGREES TO THAT THE SAMPLE CAN BE PICKED UP.

//Note that the front (left) motor turns in the opposite direction of the back wheel. This must be accounted for through code.
FEHMotor leftGMH(FEHMotor::Motor0,7.2);
FEHMotor rightGMH(FEHMotor::Motor1,7.2);

AnalogInputPin CdSCell(FEHIO::P1_0);//Change the port as needed.

AnalogInputPin OptoSensor(FEHIO::P2_0);

DigitalInputPin NWBump(FEHIO::P2_6);
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

void setForklift(float degWRYYYYYYYYYYY); //Lowers the forklift using servos 0 degrees is the resting position 90 is the horizontal degree

void pullLever();


//From here, the more complex methods for specific tasks will be prototyped

void startManuevers(); // Perform the initial backing up

void driveUpRamp(); // Requires some unique stuff

void followLine();//Follows the line until the forklift is under the core sample.

void alignToButton(); //From the wall on the top part of the course.
//Align the robot appropriatley in the horizontal direction such that if it were to turn 90 degrees and drive straight it would push the seismograph button.

void alignToLever(); // From the button, reverse and align to the lever.

void pullLever(); // Pull the lever and raise the forklift.

void acquireSample();//Pull out the core sample.

void bucketDeposition();

void moveToLine();

void gitToTheAntenna();

void checkErrors(int state); // Check and correct errors


//RPS control methods
void check_y_plus(float y_coordinate);
void check_y_minus(float y_coordinate);
void check_x_plus(float x_coordinate);
void check_heading(float heading);
void turn_right(int percent, int counts);
void move_forward(int percent, int counts);
void check_x_minus(float x_coordinate);
void check_heading(float heading);


char BucketColor = 'n'; // for none


int main(void){

    forkliftFutaba.SetMin(800);
    forkliftFutaba.SetMax(2050);
    forkliftFutaba.SetDegree(0.);
        RPS.InitializeTouchMenu();
    int state = WAITING_FOR_START;







    while(true){

       switch(state){

            case WAITING_FOR_START:
            waitForStart();
            state = GO_UP_RAMP;
           break;

            case GO_UP_RAMP:
            startManuevers();
            check_heading(357.);
            driveUpRamp();
            Sleep(1000);
            check_heading(3);
            //state = MANUEVER_TO_LOWER_LEVEL;
            state = MANUEVER_FOR_SAMPLE;
           break;

            case PUSH_BUTTON:
            alignToButton();
            driveFor(14.0,60);
            killMotors(7000);
            state = MANUEVER_FOR_LEVER;
           break;

            case MANUEVER_FOR_LEVER:
            alignToLever();
            driveFor(9,60);
            killMotors(0);
            state = MANUEVER_TO_LOWER_LEVEL;
           break;

            case PULL_LEVER:
                  pullLever();
//                driveFor(-10.0,50);
//                turn90CCW();
                state = DED;
           break;

            case MANUEVER_FOR_SAMPLE:

                turn45CCW();
                turn45CCW();
                check_heading(45);
                forkliftFutaba.SetDegree(180.);
                driveFor(15.,40);
                state = RETRIEVE_SAMPLE;
                break;

            case RETRIEVE_SAMPLE:
                 acquireSample(); // Gets the sample
                 driveFor(10,-40);
                 state = MANUEVER_TO_LOWER_LEVEL;
           break;

            case MANUEVER_TO_LOWER_LEVEL:
//                check_x_plus(18.5); // Will this work?!?!?!?!?!??!?!?
//                turn45CCW();
//                check_heading(90);

//                turn90CCW();
//                check_heading(180);
                check_heading(0);
                driveFor(25.0,-60);
                Sleep(200);
                check_y_minus(18.50);
                state = MANUEVER_TO_ANTENNA;
           break;
            case MANUEVER_TO_LIGHT:

                check_heading(180);
                check_y_minus(19.5);
                turn90CW();
                check_heading(90);
                state = DROP_IN_BUCKET;
           break;
            case DROP_IN_BUCKET:
                //Two methods for the two different buckets.
                  bucketDeposition();
                  state = MANUEVER_TO_ANTENNA;
           break;

            case MANUEVER_TO_ANTENNA:
                turn90CW();
                check_heading(270);
                driveUntilCollision(40);
                driveFor(1.0,-30);
                turn90CW();
                check_heading(180);
                leftGMH.SetPercent(30);
                rightGMH.SetPercent(30);
                while(NWBump.Value() && NEBump.Value()){}
                leftGMH.Stop();
                rightGMH.Stop();
                driveFor(.25,-30);
                turn90CCW();
                check_heading(270);
                driveFor(4.0,30);
                setForklift(90);



            state = TURN_ANTENNA;


           break;
            case TURN_ANTENNA:
                driveFor(12.0,-30);
                Sleep(100);
                setForklift(160);
                driveFor(4.0,-30);
                turn90CCW();
                check_heading(0);
                state = MOVE_TO_BUNKER;
           break;
            case MOVE_TO_BUNKER:
                setForklift(0);
                driveFor(2.5,40);
                turn90CCW();
                check_heading(90);
                driveUntilCollision(50);
                driveFor(1.0,-30);
                turn90CW();
                check_heading(0);
                driveUntilCollision(40);
                state = DED;
                break;
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

    while((SHAFT_ENCODER_INCHES_PER_CYCLE* ((right_encoder.Counts() + left_encoder.Counts()) /2) ) <distance){
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

    while((right_encoder.Counts() + left_encoder.Counts()) / 2 <SHAFT_ENCODER_COUNTS_FOR_90){
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

    while((right_encoder.Counts() + left_encoder.Counts()) / 2<SHAFT_ENCODER_COUNTS_FOR_90){
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

    while((right_encoder.Counts() + left_encoder.Counts()) / 2 <SHAFT_ENCODER_COUNTS_FOR_45){
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

    while((right_encoder.Counts() + left_encoder.Counts()) / 2  <SHAFT_ENCODER_COUNTS_FOR_45){
        //Check for errors
    }
    leftGMH.Stop();
    rightGMH.Stop();
} //Consider consolidation of all these methods into one turn method.

void driveUntilCollision(int percent){
    float startTime = TimeNow();
    leftGMH.SetPercent(percent);
    rightGMH.SetPercent(percent);
    setForklift(0); // TO ensure that forklift won't get in the way.
    while((NEBump.Value() || NWBump.Value())  && !(TimeNow() - startTime > 6.0)){
        if(!(NEBump.Value()) && NWBump.Value()){
            rightGMH.Stop();
            leftGMH.SetPercent(percent);
        }

        if(!(NWBump.Value()) && NEBump.Value()){
            leftGMH.Stop();
            rightGMH.SetPercent(percent);




        }

    }//Will continue until both switches are pressed.

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
    driveFor(6.7,-40);
    turn90CW();
    check_heading(270);
    //Shaft encoding required.
//    left_encoder.ResetCounts();
//    right_encoder.ResetCounts();

//    leftGMH.SetPercent(40);
//    rightGMH.SetPercent(40);
//    float voltage;
//    while(CdSCell.Value() > 1.5){
//        voltage = CdSCell.Value();
//    }
//    killMotors(1000);


//    if(voltage > 1.5 ){
//        //Bad times, should never happen. Just guess at this point.
//        BucketColor = 'r';
//    }
//    else if(voltage< 1.5 && voltage > 0.35){
//        BucketColor = 'b';
//    }
//    else if(voltage < 1.5 && voltage < 0.35){
//        BucketColor = 'r';
//    }

//    LCD.Clear();
//    LCD.WriteLine(BucketColor);
//    killMotors(100);

    driveFor(10.5,50);
    turn90CCW(); // Face the ramp.
    check_heading(0.);

}

void driveUpRamp(){
    check_heading(9.);
    leftGMH.SetPercent(90);
    rightGMH.SetPercent(90);

    left_encoder.ResetCounts();
    right_encoder.ResetCounts();

    while((SHAFT_ENCODER_INCHES_PER_CYCLE)* ((right_encoder.Counts() + left_encoder.Counts()) / 2) < 24.0){
        //Check for possible errors in here.
    }

    killMotors(0);
}

void alignToButton(){
    turn90CW();
    turn45CW();
    driveFor(20.0,40);
    driveFor(10.0,-60);
    turn90CCW();
    turn45CCW();
}

void alignToLever(){
    driveFor(21,-40);
    turn90CCW();
}

void pullLever(){
    setForklift(120.);
    Sleep(2000);
    driveFor(13.0,-30);

}

void acquireSample(){
    forkliftFutaba.SetDegree(170.);
    // Precise distance until it is under the sample.
    //Check error will go here.
    move_forward(-50,4);
    forkliftFutaba.SetDegree(85);
    Sleep(500);
    move_forward(-50,20);
    Sleep(500);
    forkliftFutaba.SetDegree(65);
    move_forward(-50,2);
    Sleep(500);
    forkliftFutaba.SetDegree(60);
}

void bucketDeposition(){

    //Replace these, use rps to determine how far in the x direction to go, then do the same thing.
    if(BucketColor == 'r'){
        check_heading(90);
        driveFor(2.0,30);
        turn90CCW();
        check_heading(180);
        driveFor(2.0,30);
        setForklift(140);
        driveFor(2.0,-30);
        setForklift(0);
        driveFor(4.0,30);
        Sleep(1000);
        driveFor(4.0,-30);

    }
    else if (BucketColor == 'b'){
        check_heading(90);
        driveFor(3.0,-30);
        turn90CCW();
        check_heading(180);
        driveFor(2.0,30);
        setForklift(140);
        driveFor(2.0,-30);
        setForklift(0);
        driveFor(4.0,30);
        Sleep(1000);
        driveFor(4.0,-30);
    }
    else{

    }


}



//RPS methods

void turn_right(int percent,int counts)
{
    right_encoder.ResetCounts();
    left_encoder.ResetCounts();

    rightGMH.SetPercent(-percent);
    leftGMH.SetPercent(percent);

    while((right_encoder.Counts() + left_encoder.Counts()) / 2  < counts);

    rightGMH.Stop();
    leftGMH.Stop();

}

void check_x_plus(float x_coordinate) //using RPS while robot is in the +x direction
{
    //check whether the robot is within an acceptable range
    while(RPS.X() < x_coordinate - .2 || RPS.X() > x_coordinate + .2)
    {
        if(RPS.X() > x_coordinate)
        {
            //pulse the motors for a short duration in the correct direction

           move_forward(-40,2);
        }
        else if(RPS.X() < x_coordinate)
        {
            //pulse the motors for a short duration in the correct direction


            move_forward(40,2);
        }
    }
}

void check_x_minus(float x_coordinate) //using RPS while robot is in the -x direction
{
    //check whether the robot is within an acceptable range
    while(RPS.X() < x_coordinate - .2 || RPS.X() > x_coordinate + .2)
    {
        if(RPS.X() > x_coordinate)
        {
            //pulse the motors for a short duration in the correct direction

           move_forward(40,2);
        }
        else if(RPS.X() < x_coordinate)
        {
            //pulse the motors for a short duration in the correct direction


            move_forward(-40,2);
        }
    }
}


void check_y_minus(float y_coordinate) //using RPS while robot is in the -y direction
{
    //check whether the robot is within an acceptable range
    while(RPS.Y() < y_coordinate - .2 || RPS.Y() > y_coordinate + .2)
    {
        if(RPS.Y() > y_coordinate)
        {
            //pulse the motors for a short duration in the correct direction

            move_forward(-40,2);
        }
        else if(RPS.Y() < y_coordinate)
        {
            //pulse the motors for a short duration in the correct direction

            move_forward(40,2);
        }
    }
}

void check_y_plus(float y_coordinate) //using RPS while robot is in the +y direction
{
    //check whether the robot is within an acceptable range
    while(RPS.Y() < y_coordinate - .2 || RPS.Y() > y_coordinate + .2)
    {
        if(RPS.Y() > y_coordinate)
        {
            //pulse the motors for a short duration in the correct direction

            move_forward(40,2);
        }
        else if(RPS.Y() < y_coordinate)
        {
            //pulse the motors for a short duration in the correct direction

            move_forward(-40,2);
        }
    }
}

void check_heading(float heading) //using RPS
{
    while(RPS.Heading() < heading - 1|| RPS.Heading() > heading + 1)
    {
        Sleep(120);
        if(RPS.Heading() > heading )
        {
            //pulse the motors for a short duration in the correct direction
            if(RPS.Heading() - heading > 180){

                 turn_right(-30,2);
            }
            else{
               turn_right(30,2);
            }

        }else if(RPS.Heading() < heading)
        {
            //pulse the motors for a short duration in the correct direction

            if(heading - RPS.Heading() > 180){
                 turn_right(30,2);
            }
            else{

                   turn_right(-30,2);
            }
        }
    }
    leftGMH.Stop();
    rightGMH.Stop();
}

void move_forward(int percent, int counts) //using encoders
{
    //Reset encoder counts
    right_encoder.ResetCounts();
    left_encoder.ResetCounts();

    //Set both motors to desired percent
    rightGMH.SetPercent(percent);
    leftGMH.SetPercent(percent);
    //While the average of the left and right encoder is less than counts,
    //keep running motors
    while((right_encoder.Counts() + left_encoder.Counts()) / 2  < counts);

    //Turn off motors
    leftGMH.Stop();
    rightGMH.Stop();
}

void check_heading_zero(){
    while(RPS.Heading() < 359 || RPS.Heading() >  1)
    {

        if(RPS.Heading() > 0 )
        {
            //pulse the motors for a short duration in the correct direction
            if(RPS.Heading() > 180){

                 turn_right(-40,1);
            }
            else{
               turn_right(40,1);
            }

        }else if(RPS.Heading() < 360)
        {
            //pulse the motors for a short duration in the correct direction

            if(360 - RPS.Heading() > 180){
                 turn_right(40,1);
            }
            else{

                   turn_right(-40,1);
            }
        }
    }
    killMotors(0);


}


