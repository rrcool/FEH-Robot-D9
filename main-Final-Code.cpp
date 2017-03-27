#include <FEHLCD.h>
#include <FEHIO.h>
#include <FEHUtility.h>
#include <FEHMotor.h>
#include <FEHServo.h>
#include <FEHRPS.h>

//Define constant values
#define SHAFT_ENCODER_CYCLES_PER_INCH 12.7323954474 // Modify this value
#define SHAFT_ENCODER_INCHES_PER_CYCLE (1/SHAFT_ENCODER_CYCLES_PER_INCH)
#define SHAFT_ENCODER_COUNTS_FOR_45 46
#define SHAFT_ENCODER_COUNTS_FOR_90 92 //Determine these two by trial and error
#define SHAFT_ENCODER_COUNTS_PER_DEGREE(45/SHAFT_ENCODER_COUNTS_FOR_45);
#define MINIMUM_SERVO_ANGLE 500
#define MAXIMUM_SERVO_ANGLE 2409

//Define important RPS values here.

//Define States that the robot will toggle between.
#define WAITING_FOR_START 1
#define MANUEVER_TO_ANTENNA 2
#define TURN_ANTENNA 3
#define GO_UP_RAMP 4
#define MANUEVER_FOR_LEVER 5
#define PULL_LEVER 6
#define PUSH_BUTTON 7
#define MANUEVER_FOR_SAMPLE 8
#define RETRIEVE_SAMPLE 9
#define MANUEVER_TO_LOWER_LEVEL 10
#define MANUEVER_TO_LIGHT 11
#define DROP_IN_BUCKET 12
#define MOVE_TO_BUNKER 13
#define DED 14


int state; // A variable that holds the state value of hte robot.
char color = 'r'; // Character that denotes the color of light. Red by default.

//Declare different parts of the robot
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

//Function prototypes from here on out.

    //General Functions
        void waitForStart(); //The first executed method. The robot will wait until the light is picked up by the CDS cell. After the light is detected, execution will continue.

        void killMotors(int restPeriod); // Turns off both of the motors. A sleep will be executed for "restPeriod" number of milliseconds.

        void driveFor(float distance,int percent); //Drive straight for distance with motors set to percent.

        void turnCW(float degrees); // Turn the robot degrees number of degrees in the clockwise direction

        void turnCCW(float degrees); // Turn the robot degrees number of degrees in the Counterclockwise direction

        void driveUntilCollision(int percent,float timeOut); // Drive in a straightline until collision with motors set to percent

        void reverseUntilCollision(int percent, float timeOut); //Reverse until a collision with motors set to percent.

        void setForklift(float degree); //Lowers the forklift using servos 0 degrees is the resting position.

        //Complex Functions
        void startManuevers(); // Performs the starting manuevers of leaving the bunker and travelling in front of the ramp.

        void alignWithAntenna(); // Align the robot to the antenna so that it can perform the turn.

        void turnAntenna(); //Turn the antenna

        void driveUpRamp(); // Travel from the antenna to the ramp, and then drive up the ramp.

        void alignToLever(); // At the top of the ramp, align the robot to the lever.

        void pullLever(); // Once aligned, pull the lever.

        void alignToButton(); // Align to face the far wall. Drive until collision and then align the robot to face the button.

        void alignToLine(); //Drive straight until either the optosensor picks up the orange line, or the robot travels too far.

        void followLine(); // Once the line has been detected, follow the line in a straight path for a fixed distance.

        void acquireSample(); // Once lined up with the sample, use the forklift to remove it from the wall.

        void manueverToLowerLevel(); // Line up with the ramp and travel down the ramp.

        void driveToLight(); // Drive to the light and measure its value.


        void bucketDeposition(); // Drop the sample in the (a) bucket.

        void manueverToFinalButton(); //Line up with the button in the bunker.

        void checkError(); // Check the state of the robot at various points to ensure everything is working properly.

    //RPS Functions
        float rpsXtemp,rpsYtemp; //Variables that will temporarily store RPS coordinates. Intended to be overwritten and used very sparingly.

        void check_y_plus(float y_coordinate); // Check the y RPS coordinate when the robot is facing towards the far wall (upper level side).

        void check_y_minus(float y_coordinate); // Check the y RPS coordinate when the robot is facing towards the near wall (lower level side).

        void check_x_plus(float x_coordinate); // Check the x RPS coordinate when the robot is facing towards the far wall (satellite side).

        void check_x_minus(float x_coordinate); // Check the x RPS coordinate when the robot is facing towards the near wall (bunker side).

        void checkHeading(float heading); //Check the heading angle of the robot using the RPS.

        void checkHeadingZero(); //Check the heading angle of the robot using the RPS when the angle to check is zero.

        void move_forward(int percent, int counts); // Move forward a fixed number of counts.


int main(void)
{
    forkliftFutaba.SetMin(800);
    forkliftFutaba.SetMax(2050);
    forkliftFutaba.SetDegree(0.);

    RPS.InitializeTouchMenu();

    state = WAITING_FOR_START;

    while(true){
        switch (state){
                case WAITING_FOR_START:
                    waitForStart();
                    startManuevers();   //Note RPS X temp and Y temp are set in this method.
                    state = MANUEVER_TO_ANTENNA;

            break;
                    /**
                      Summary: Wait for the starting light, and then perform starting
                      manuevers that place the robot in front of the ramp.
                      */

                case MANUEVER_TO_ANTENNA:
                    alignWithAntenna();
                    driveUntilCollision(30,6.0);
                    setForklift(90);
                    state = TURN_ANTENNA;

            break;

                    /**
                      Summary: From the ramp align the robot horizontally with the antenna.
                      Then, turn and collide with the antenna for alignment purposes.
                      Finally, turn once more and drop the forklift.
                      */

                case TURN_ANTENNA:
                    driveFor(12.0,-30);
                    Sleep(100);
                    setForklift(160);
                    driveFor(4.0,-30);
                    check_x_plus(rpsXtemp);
                    turnCCW(90);
                    check_heading(0);
                    setForklift(0);
                    state = GO_UP_RAMP;
            break;
                    /**
                      Summary:Reverse to turn the antenna using the forklift.
                      Once this is accomplished, lower the forklift and continue to reverse.
                      Align horizontally with the ramp, and then turn to face the ramp.
                      */

                case GO_UP_RAMP:
                    check_y_plus(rpsYtemp);
                    driveUpRamp();
                    state = MANUEVER_FOR_LEVER;
            break;
                   /**
                     Summary: Adjust vertical position to allow the robot to drive up the ramp.
                     Then drive up the ramp.
                     */

                case MANUEVER_FOR_LEVER:
                    alignToLever();
                    driveFor(9,60);
                    killMotors(100);
                    state = PULL_LEVER;
            break;
                    /**
                      Summary: From the top of the ramp, align the robot with the lever,
                      then, turn and face the lever. Finally, drive towards the lever.
                      */

                case PULL_LEVER:
                    pullLever();
                    turnCW(180);
                    checkHeading(270);
                    state = PUSH_BUTTON;
            break;
                    /**
                      Summary: Pull the lever, and then back up. Finally, turn the robot 180 degrees
                      to prepare for alignment to the button.
                      */

                case PUSH_BUTTON:
                    alignToButton();

                    driveUntilCollision(30,5.0);
                    setForklift(90);
                    killMotors(6000);
                    setForklift(0);
                    state = MANUEVER_FOR_SAMPLE;
            break;
                    /**
                      Summary: Align the robot to the seismograph button. Then
                      drive the robot forward to hit the button. Use the forklift to hit it.
                      */

                case MANUEVER_FOR_SAMPLE:
                    driveFor(3.0,-40);
                    turnCCW(90);
                    alignToLine();
                    followLine();
                    state = RETRIEVE_SAMPLE;
            break;
                    /**
                      Summary: Drive straight until the robot is over the line, then
                      Straighten out and follow the line for a fixed distance.
                      */

                case RETRIEVE_SAMPLE:   
                    acquireSample();
                    driveFor(7.0,-40);
                    state = MANUEVER_TO_LOWER_LEVEL;
            break;
                    /**
                     * Summary: Collect the sample, and prepare to go to the lower level.
                     */

                case MANUEVER_TO_LOWER_LEVEL:
                    turnCCW(45);
                    checkHeading(90);
                    check_x_minus(rpsXtemp);
                    turnCW(90);
                    checkHeadingZero();
                    check_y_plus(rpsYtemp); // This will make the robot go down the ramp.
                    state = MANUEVER_TO_LIGHT;
            break;
                    /**
                     * Summary: Align the robot and go down the ramp.
                     */

                case MANUEVER_TO_LIGHT:
                    check_y_minus(); // Find coordinate for light.
                    turnCCW(90);
                    checkHeading(90);
                    driveToLight();
                    state = DROP_IN_BUCKET;
            break;
                    /**
                      * Summary: Drive to the light and read the value.
                      */

                case DROP_IN_BUCKET:
                    bucketDeposition();
                    state = MOVE_TO_BUNKER;

                    /**
                      * Summary: Based on the color of light, drop the sample in the right bucket.
                      */
            break;
                case MOVE_TO_BUNKER:
                    manueverToFinalButton();
                    driveUntilCollision(40,10.0);
                    state = DED;

                    /**
                      * Summary: Align with the final button and push it.
                      */

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
                    /**
                      * Summary: This is the end.
                      */
        }

    }
}



/**
 * Method implementations
 */



    /**
     * @brief waitForStart
     *
     * Purpose: Wait in the bunker until the light turns on.
     *
     * Requires: The robot must be in the bunker, over the light.
          */

    void waitForStart(){

        float x,y;
        while(!LCD.Touch(&x,&y));
        while(LCD.Touch(&x,&y));

        LCD.WriteLine("Ready, Waiting for light.....");

        while(CdSCell.Value() > 1.0){
            LCD.Write("Voltage = ");
            LCD.WriteLine(CdSCell.Value());
            Sleep(200);
        }
        LCD.WriteLine("Light Detected, beginning run.....");
    }

    /**
     * @brief killMotors
     *
     * Purpose: Stop both motors and wait for a certain period of time.
     *
     * @param restPeriod:
     * The amount of milliseconds the robot will rest for after stopping motors.
     *
     * Requires: None, can be used at any time.
     *
     */
    void killMotors(int restPeriod){
        LCD.WriteLine("Killing power to motors...");
        leftGMH.Stop();
        rightGMH.Stop();
        LCD.Write("Motors Stopped. Waiting for: ");
        LCD.Write(restPeriod);
        LCD.WriteLine(" seconds....");

        Sleep(restPeriod);
    }

    /**
     * @brief driveFor
     *
     * Purpose: Drive the robot for a fixed distance.
     *
     * @param distance:
     * The distance in inches that the robot will travel.
     *
     * @param percent:
     * The percentage that both motors will be set to. -100 < percent < 100
     *
     * requires: Magnitude of percent must be < 100. Be wary of large distance values.
     *
     */
    void driveFor(float distance, int percent){
        left_encoder.ResetCounts();
        right_encoder.ResetCounts();

        leftGMH.SetPercent(percent);
        rightGMH.SetPercent(percent);
        LCD.Write("Drive for distance: ");
        LCD.WriteLine(distance);

        while((SHAFT_ENCODER_INCHES_PER_CYCLE) * ((right_encoder.Counts() + left_encoder.Counts())/2) < distance){

        }

        killMotors(0);
    }


    /**
     * @brief turnCW
     *
     * Purpose: Turn the robot a certain number of degrees CW.
     *
     * @param degrees:
     * The number of degrees that the robot will turn. Can be a decimal.
     *
     * Requires: While not explicitly disallowed, degrees should be <= 180.
     *
     */
    void turnCW(float degrees){
        left_encoder.ResetCounts();
        right_encoder.ResetCounts();

        leftGMH.SetPercent(70);
        rightGMH.SetPercent(-70);

        while((right_encoder.Counts() + left_encoder.Counts()) / 2 < (degrees * SHAFT_ENCODER_COUNTS_PER_DEGREE)){

        }

        killMotors(10);
    }

    /**
     * @brief turnCCW
     *
     * Purpose: Turn the robot a certain number of degrees CW.
     *
     * @param degrees:
     * The number of degrees that the robot will turn. Can be a decimal.
     *
     * Requires: While not explicitly disallowed, degrees should be <= 180.
     */
    void turnCCW(float degrees){
        left_encoder.ResetCounts();
        right_encoder.ResetCounts();

        leftGMH.SetPercent(-70);
        rightGMH.SetPercent(70);

        while((right_encoder.Counts() + left_encoder.Counts()) / 2 < (degrees * SHAFT_ENCODER_COUNTS_PER_DEGREE)){

        }

        killMotors(10);
    }

    /**
     * @brief driveUntilCollision
     *
     * Purpose: Drive forward until both bump switches are triggered.
     *
     * @param percent:
     * The percent that both motors will be set at.
     *
     * @param timeOut:
     * After this much time has passed, execution will continue.
     *
     * requires: Percent must be positive, since the method only checks the front switches.
     */
    void driveUntilCollision(int percent, float timeOut){
        float startTime = TimeNow();
        leftGMH.SetPercent(percent);
        rightGMH.SetPercent(percent);
        setForklift(0);

        while((NEBump.Value() || NWBump.Value()) && !(TimeNow() - startTime  > timeOut)){

            if(!(NEBump.Value()) && NWBump.Value()){
                rightGMH.Stop();
                leftGMH.SetPercent(percent);
            }

            if(!(NWBump.Value()) && NEBump.Value()){
                leftGMH.Stop();
                rightGMH.SetPercent(percent);

          }
        }
    }
    /**
     * @brief reverseUntilCollision
     *
     * Purpose: Drive in reverse until both rear bump switches are triggered.
     *
     * @param percent:
     * The percent that both motors will be set at IN REVERSE.
     *
     * requires: Percent must be positive, the motors will be set to this percent * -1.
     */
    void reverseUntilCollision(int percent,float timeOut){
        float startTime = TimeNow();
        leftGMH.SetPercent(percent);
        rightGMH.SetPercent(percent);


        while((SEBump.Value() || SWBump.Value()) && !(TimeNow() - startTime  > timeOut)){

            if(!(SEBump.Value()) && SWBump.Value()){
                rightGMH.Stop();
                leftGMH.SetPercent(percent);
            }

            if(!(SWBump.Value()) && SEBump.Value()){
                leftGMH.Stop();
                rightGMH.SetPercent(percent);

          }
        }
    }

    /**
     * @brief setForklift
     *
     * Purpose: Set the forklift to a certain degree.
     *
     * @param degree:
     * The degree that the servo will be set at.
     */
    void setForklift(float degree){
        forkliftFutaba.SetDegree(degree);
    }


    // Complex functions

    /**
     * @brief startManuevers
     *
     * Purpose: Perform the starting manuevers for the robot.
     *
     * requires: The robot is in the bunker over the light.
     *
     */
    void startManuevers() {
        LCD.WriteLine("Starting Manuevers.....");
        driveFor(6.7,-40);
        turnCW(90);
        checkHeading(270);
        driveFor(10.5,50);
        rpsXtemp = RPS.X();
        turnCCW(90);
        rpsYtemp = RPS.Y(); // These two values will be used for aligning the robot with the ramp.
        checkHeadingZero();
        LCD.WriteLine("Starting Mnauevers complete.");
    }

    /**
     * @brief alignWithAntenna
     * Purpose: Align the robot with the antenna.
     *
     * requires: The robot should be in front of the ramp post starting manuevers.
     */
    void alignWithAntenna(){
        LCD.WriteLine("Aligning with Antenna....");
        check_y_plus(18.5);
        turnCW(90);
        checkHeading(90);
        driveUntilCollision(30,4.5);
        driveFor(1.0,-30);
        turnCW(90);
        checkHeading(180);

        leftGMH.SetPercent(30);
        rightGMH.SetPercent(30);
        while(NWBump.Value() && NEBump.Value()){}
        leftGMH.Stop();
        rightGMH.Stop();
        LCD.WriteLine("Collision with Antenna Confirmed, continuing alignment.... ");

        driveFor(.25,-30);
        turnCCW(90);
        checkHeading(270);

        LCD.WriteLine("Alignment Complete.");
    }

    /**
     * @brief turnAntenna
     *
     * Purpose: Use the forklift to turn the antenna 90 degrees.
     *
     *
     * requires: alignWithAntenna must have been succesfully run.
     */
    void turnAntenna(){
        driveFor(12.0,-30);
        Sleep(100);
        setForklift(160);
        driveFor(4.0,-30);
        turnCCW(90);
        check_heading(0);
    }

    /**
     * @brief driveUpRamp
     *
     * Purpose: Align the robot to the ramp and then drive up the ramp.
     *
     * requires: The robot must already be aligned in the x-direction.
     */
    void driveUpRamp(){
        LCD.WriteLine("Driving up Ramp....");
        checkHeading(5);
        driveFor(24.0, 90);
        checkHeadingZero();
        LCD.WriteLine("Ascension of Ramp complete. ");
    }

    /**
     * @brief alignToLever
     * Purpose: At the top of the ramp, align the robot to the lever, and then turn to face lever.
     *
     * requires: The robot is at the top of the ramp.
     */
    void alignToLever(){
        check_y_plus(); // Find the correct coordinate.
        turnCCW(90);
        checkHeading(90);
    }

    /**
     * @brief pullLever
     * Purpose: Use the forklift to pull the lever.
     *
     * requires: The robot is aligned with the levers.
     */
    void pullLever(){
        setForklift(120.);
        Sleep(1000);
        driveFor(21,-40);

    }

    /**
     * @brief alignToButton
     * Purpose: Align the robot to face the seismograph button.
     *
     * requires: The robot should be facing the far wall (antenna side)
     */
    void alignToButton(){
        driveUntilCollision(30,8.0);
        driveFor(10.0,-60);
        turnCCW(90);
        checkHeadingZero();
    }

    /**
     * @brief alignToLine
     *
     * Purpose: The robot will drive until it detects the orange line, then it will straighten out.
     *
     * requires: The robot should be facing the general direction of the line.
     */
    void alignToLine(){}

    /**
     * @brief followLine
     *
     * Purpose: The robot will drive straight while following the line for a fixed distance.
     *
     * requires: The robot must be on the line.
     */
    void followLine(){}

    /**
     * @brief acquireSample
     *
     * Purpose: The robot will use the forklift to pull out the sample.
     *
     * Requires: The robot is aligned under the sample.
     *
     */
    void acquireSample(){}

    /**
     * @brief manueverToLowerLevel
     *
     * Purpose: The robot will move from the upper level to the lower level.
     *
     * requires: The robot is on the upper level.
     */
    void manueverToLowerLevel(){}

    /**
     * @brief bucketDeposition
     *
     * Purpose: The robot will drop the sample into the appropriate bucket based on the color of light.
     *
     * requires: The robot must have detected the color of light.
     */
    void bucketDeposition(){}

    /**
     * @brief manueverToFinalButton
     *
     * Purpose: Move the robot from the buckets to alignment with the final button.
     *
     *
     * requires: The robot is on the lower level.
     */
    void manueverToFinalButton(){}

    /**
     * @brief checkError
     *
     * Purpose: Check for and correct any errors that may occur.
     *
     * requires: Can only be used at certain points of the program flow.
     */
    void checkError(){}


    //RPS methods

    /**
     * @brief check_y_plus
     *
     * Purpose: Check RPS y-coordinate when the robot is at an angle of zero.
     *
     * @param y_coordinate:
     * The y-coordinate that will be checked
     *
     * requires: The y-coordinate is within the range of rps coordinates. The robot is at an angle of 0.
     */
    void check_y_plus(float y_coordinate){}

    /**
     * @brief check_y_minus
     *
     * Purpose: Check RPS y-coordinate when the robot is at an angle of 180.
     *
     * @param y_coordinate:
     * The y-coordinate that will be checked
     *
     * requires: The y-coordinate is within the range of rps coordinates. The robot is at an angle of 180.
     *
     */
    void check_y_minus(float y_coordinate){}

    /**
     * @brief check_x_plus
     *
     * Purpose: Check the rps X coordinate when the robot is at an angle of 270.
     *
     * @param x_coordinate:
     * The x-coordinate to be checked.
     *
     * requires: The x-coordinate is within the range of rps coordinates. The robot is at an angle of 270.
     */
    void check_x_plus(float x_coordinate){}

    /**
     * @brief check_x_minus
     *
     * Purpose: Check the rps X coordinate when the robot is at an angle of 90.
     *
     * @param x_coordinate:
     * The x-coordinate to be checked.
     *
     * requires: The x-coordinate is within the range of rps coordinates. The robot is at an angle of 90.
     */
    void check_x_minus(float x_coordinate){}

    /**
     * @brief checkHeading
     *
     * Purpose: Check the heading angle of the robot and adjust if incorrect.
     *
     * @param heading:
     * The heading angle to be checked.
     *
     * requires: heading < 360 && heading > 0.
     */
    void checkHeading(float heading){}

    /**
     * @brief checkHeadingZero
     *
     * Purpose: Check and adjust the angle of the robot when the desired angle is 0.
     *
     *
     * requires: Nothing.
     */
    void checkHeadingZero(){}

    /**
     * @brief move_forward
     *
     * Purpose: Move the robot by a certain number of counts.
     *
     * @param percent:
     * The percent that both motors will be set to.
     *
     * @param counts:
     * The number of counts for the robot to move.
     *
     * The number of counts that the robot will travel for.
     */
    void move_forward(int percent, int counts){


    }
