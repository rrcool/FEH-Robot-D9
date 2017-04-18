
#include <FEHLCD.h>
#include <FEHIO.h>
#include <FEHUtility.h>
#include <FEHMotor.h>
#include <FEHServo.h>
#include <FEHRPS.h>

//Define constant values
#define SHAFT_ENCODER_CYCLES_PER_INCH 12.7323954474 // Modify this value
#define SHAFT_ENCODER_INCHES_PER_CYCLE (1./SHAFT_ENCODER_CYCLES_PER_INCH)
#define SHAFT_ENCODER_COUNTS_FOR_45 46
#define SHAFT_ENCODER_COUNTS_FOR_90 92 //Determine these two by trial and error
#define SHAFT_ENCODER_COUNTS_PER_DEGREE (45./SHAFT_ENCODER_COUNTS_FOR_45)
#define MINIMUM_SERVO_ANGLE 800
#define MAXIMUM_SERVO_ANGLE 2100

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
#define MANUEVER_TO_LIGHT 11 // Not used for final Competition
#define DROP_IN_BUCKET 12 // Not used for final Competition
#define MOVE_TO_BUNKER 13
#define DED 14


int state; // A variable that holds the state value of hte robot.
char color = 'r'; // Character that denotes the color of light. Red by default.

//Declare different parts of the robot
FEHMotor leftGMH(FEHMotor::Motor1,7.2);
FEHMotor rightGMH(FEHMotor::Motor0,7.2);

AnalogInputPin CdSCell(FEHIO::P1_0);//Change the port as needed.

AnalogInputPin CenterOptoSensor(FEHIO::P2_3);
AnalogInputPin RightOptoSensor(FEHIO::P2_0);
AnalogInputPin LeftOptoSensor(FEHIO::P2_6);

DigitalInputPin NWBump(FEHIO::P3_1);
DigitalInputPin NEBump(FEHIO::P0_2);
DigitalInputPin SWBump(FEHIO::P3_4);
DigitalInputPin SEBump(FEHIO::P0_6);

DigitalEncoder right_encoder(FEHIO::P0_0);
DigitalEncoder left_encoder(FEHIO::P3_5); // Break beam, counts per transition.


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

   //RPS Functions
       float rpsXtemp,rpsYtemp; //Variables that will temporarily store RPS coordinates. Intended to be overwritten and used very sparingly.

       void check_y_plus(float y_coordinate); // Check the y RPS coordinate when the robot is facing towards the far wall (upper level side).

       void check_y_minus(float y_coordinate); // Check the y RPS coordinate when the robot is facing towards the near wall (lower level side).

       void check_x_plus(float x_coordinate); // Check the x RPS coordinate when the robot is facing towards the far wall (satellite side).

       void check_x_minus(float x_coordinate); // Check the x RPS coordinate when the robot is facing towards the near wall (bunker side).

       void checkHeading(float heading); //Check the heading angle of the robot using the RPS.

       void checkHeadingZero(); //Check the heading angle of the robot using the RPS when the angle to check is zero.

       void move_forward(int percent, int counts); // Move forward a fixed number of counts.

       void turn_right(int percent, int counts);   // Move right a fixed number of counts.

int main(void)
{
   forkliftFutaba.SetMin(800);
   forkliftFutaba.SetMax(2100);
   forkliftFutaba.SetDegree(0.);
   float currentTime;

   currentTime = TimeNow();

   LCD.Clear();
   LCD.WriteLine("Push NW Bump to perform starting diagnostic, or just wait 4 seconds. ");
   while((TimeNow() -currentTime < 4.0 )){
       if(!NWBump.Value()){
           driveFor(3.0,-60);
           turnCW(90);
           turnCCW(90);
           driveFor(3.0,60);

//            leftGMH.SetPercent(60);
//            Sleep(300);
//            killMotors(150);
       }
   }

   RPS.InitializeTouchMenu();

   state = WAITING_FOR_START;

   while(true){
       LCD.Write("State  = ");
       LCD.WriteLine(state);

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
                   killMotors(150);
                   setForklift(90);
                   state = TURN_ANTENNA;

           break;

                   /**
                     Summary: From the ramp align the robot horizontally with the antenna.
                     Then, turn and collide with the antenna for alignment purposes.
                     Finally, turn once more and drop the forklift.
                     */

               case TURN_ANTENNA:
                   driveFor(12.0,-40);
                   Sleep(100);
                   setForklift(160);
                   checkHeading(270);
                   driveFor(4.0,-30);
                   check_x_plus(rpsXtemp + 3.5);
                   turnCCW(90);
                   checkHeadingZero();
                   setForklift(0);
                   state = GO_UP_RAMP;
           break;
                   /**
                     Summary:Reverse to turn the antenna using the forklift.
                     Once this is accomplished, lower the forklift and continue to reverse.
                     Align horizontally with the ramp, and then turn to face the ramp.
                     */

               case GO_UP_RAMP:
                   driveFor(5.0,40);
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
                   driveFor(9,50);
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
                   driveUntilCollision(50,8.0);
                   driveFor(5.0,-40);
                   setForklift(138);
                   currentTime = TimeNow();
                   leftGMH.SetPercent(35);
                   rightGMH.SetPercent(35);
                   while(TimeNow() - currentTime < 6.5){


                   }
                   killMotors(0);
                   driveFor(1.0,-30);
                   setForklift(0);
                   driveFor(.60,30);
                   state = MANUEVER_FOR_SAMPLE;
           break;
                   /**
                     Summary: Align the robot to the seismograph button. Then
                     drive the robot forward to hit the button. Use the forklift to hit it.
                     */

               case MANUEVER_FOR_SAMPLE:
                   driveFor(1.0,-40);
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
                   driveFor(7.0,-60);
                   state = MANUEVER_TO_LOWER_LEVEL;
           break;
                   /**
                    * Summary: Collect the sample, and prepare to go to the lower level.
                    */

               case MANUEVER_TO_LOWER_LEVEL:
                   manueverToLowerLevel();
                  // state = MANUEVER_TO_LIGHT; Not used for indv comp.
                   state = MOVE_TO_BUNKER;
           break;
                   /**
                    * Summary: Align the robot and go down the ramp.
                    */

               case MANUEVER_TO_LIGHT:
                   check_y_minus(15.4); // Find coordinate for light.
                   turnCCW(90);
                   checkHeading(90);
                   driveToLight();
                   state = DROP_IN_BUCKET;
           break;
                   /**
                     * Summary: Drive to the light and read the value. NOT USED FOR INDV COMP.
                     */

               case DROP_IN_BUCKET:
                   bucketDeposition();
                   state = MOVE_TO_BUNKER;

                   /**
                     * Summary: Based on the color of light, drop the sample in the right bucket. NOT USED FOR INDV COMP.
                     */
           break;
               case MOVE_TO_BUNKER:
                   manueverToFinalButton();
                   driveUntilCollision(60,10.0);
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
       float startTime = TimeNow();
       while(CdSCell.Value() > 1.0 && TimeNow()-startTime < 30 ){
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
       LCD.WriteLine(" milliseconds....");

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

       float startTime = TimeNow();

       while((SHAFT_ENCODER_INCHES_PER_CYCLE) * ((right_encoder.Counts() + left_encoder.Counts())/2) < distance && TimeNow() - startTime < 6.0){

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
       LCD.WriteLine("Turning Clockwise....");
       left_encoder.ResetCounts();
       right_encoder.ResetCounts();

       leftGMH.SetPercent(65);
       rightGMH.SetPercent(-65);

       while ((right_encoder.Counts() + left_encoder.Counts()) / 2 < degrees * SHAFT_ENCODER_COUNTS_PER_DEGREE){}


       killMotors(0);
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
       LCD.WriteLine("Turning Counter-Clockwise....");
       left_encoder.ResetCounts();
       right_encoder.ResetCounts();

       leftGMH.SetPercent(-65);
       rightGMH.SetPercent(65);


       while((right_encoder.Counts() + left_encoder.Counts() ) / 2 < degrees * SHAFT_ENCODER_COUNTS_PER_DEGREE){}


       killMotors(0);
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
       LCD.WriteLine("Driving until collision.....");

       float startTime = TimeNow();
       leftGMH.SetPercent(percent);
       rightGMH.SetPercent(percent);
       setForklift(0);

       while((NEBump.Value() || NWBump.Value()) && !(TimeNow() - startTime  > timeOut)){

           if(!(NEBump.Value()) && NWBump.Value()){
               LCD.WriteLine("NE Bump has hit, but not NW Bump. Correcting....");
               rightGMH.Stop();
               leftGMH.SetPercent(percent);
           }

           if(!(NWBump.Value()) && NEBump.Value()){
               LCD.WriteLine("NW Bump has hit, but not NE Bump. Correcting....");
               leftGMH.Stop();
               rightGMH.SetPercent(percent);

         }
       }

       LCD.WriteLine("Collision detected. Continuing execution......");
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
               LCD.WriteLine("SE Bump has hit, but not SW Bump. Correcting....");
               rightGMH.Stop();
               leftGMH.SetPercent(percent);
           }

           if(!(SWBump.Value()) && SEBump.Value()){
               LCD.WriteLine("SW Bump has hit, but not SE Bump. Correcting....");
               leftGMH.Stop();
               rightGMH.SetPercent(percent);

         }
       }
       LCD.WriteLine("Collision detected. Continuing execution......");

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
       LCD.Write("Changing forklift degree to: ");
       LCD.WriteLine(degree);
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
       driveFor(6.7,-50);
       turnCW(90);
       checkHeading(270);
       driveFor(10.5,55);
       rpsXtemp = RPS.X();
       turnCCW(90);
       rpsYtemp = RPS.Y(); // These two values will be used for aligning the robot with the ramp.
       checkHeadingZero();
       LCD.WriteLine("Starting Manuevers complete.");
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
       checkHeading(270);
       driveUntilCollision(60,4.5);
       driveFor(1.0,-30);
       turnCW(90);
       checkHeading(180);

       leftGMH.SetPercent(30);
       rightGMH.SetPercent(30);
       float startTime = TimeNow();
       while((NWBump.Value() && NEBump.Value()) && TimeNow() - startTime < 2.5){}
       Sleep(100);
       leftGMH.Stop();
       rightGMH.Stop();
       LCD.WriteLine("Collision with Antenna Confirmed, continuing alignment.... ");
       driveFor(.1,-25);
       turnCCW(90);
       checkHeading(267);
       driveUntilCollision(30,2);
       Sleep(200);
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
       LCD.WriteLine("Turning antenna......");
       driveFor(12.0,-35);
       Sleep(100);
       setForklift(160);
       driveFor(2.0,-30);
       turnCCW(90);
       checkHeadingZero();

       LCD.WriteLine("Antenna turning and repositioning complete.");
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
       driveFor(24.0, 95);
       if(RPS.Y() == -1){
        driveFor(4.0,30);
        if(RPS.Y() == -1){
            driveFor(3.0,-30);
            checkHeadingZero();
            driveFor(24.0,95);
        }
       }

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
       LCD.WriteLine("Aligning to lever....");
       check_y_plus(42.7); // Find the correct coordinate.
       turnCCW(90);
       checkHeading(90);
       LCD.WriteLine("Alignment Complete");
   }

   /**
    * @brief pullLever
    * Purpose: Use the forklift to pull the lever.
    *
    * requires: The robot is aligned with the levers.
    */
   void pullLever(){
       LCD.WriteLine("Pulling Lever....");
       setForklift(120.);
       Sleep(1000);
       driveFor(3,-30);
       setForklift(0);
       Sleep(100);
       driveFor(3,-30);
       LCD.WriteLine("The lever has been pulled.");
   }

   /**
    * @brief alignToButton
    * Purpose: Align the robot to face the seismograph button.
    *
    * requires: The robot should be facing the far wall (antenna side)
    */
   void alignToButton(){
       LCD.WriteLine("Aligning to seismograph button....");
       driveUntilCollision(50,8.0);
       driveFor(5.0,-60);
       turnCCW(90);
       checkHeadingZero();
       LCD.WriteLine("Alignment Complete.");
   }

   /**
    * @brief alignToLine
    *
    * Purpose: The robot will drive until it detects the orange line, then it will straighten out.
    *
    * requires: The robot should be facing the general direction of the line.
    */
   void alignToLine(){
       LCD.WriteLine("Aligning to line.....");
       checkHeading(90);
       leftGMH.SetPercent(25);
       rightGMH.SetPercent(25);

       while(CenterOptoSensor.Value() > 3.25){ // Determine threshold for the line.

       }

       while(CenterOptoSensor.Value() < 3.25){ // Determine threshold for the line.

       }
       killMotors(100);

       leftGMH.SetPercent(40);

       while(LeftOptoSensor.Value() > 3.25){}

       killMotors(100);

       rightGMH.SetPercent(40);
       while(CenterOptoSensor.Value() > 3.25){}
       LCD.WriteLine("Alignment Complete.");
       killMotors(100);
   }

   /**
    * @brief followLine
    *
    * Purpose: The robot will drive straight while following the line for a fixed distance.
    *
    * requires: The robot must be on the line.
    */
   void followLine(){ // Measure values to complete method.
       LCD.WriteLine("Following line....");

       float threshold = 3.25;
       float startTime = TimeNow();
       leftGMH.SetPercent(40);
       rightGMH.SetPercent(40);

       left_encoder.ResetCounts();
       right_encoder.ResetCounts();

       while(TimeNow() - startTime < 7.5){
           if(CenterOptoSensor.Value() > threshold){
               killMotors(100);

               if(RightOptoSensor.Value() < threshold){
                   leftGMH.SetPercent(30);
                   while(LeftOptoSensor.Value() > threshold && TimeNow() - startTime < 5){

                   }
                   killMotors(150);
                   rightGMH.SetPercent(30);
                   while(CenterOptoSensor.Value() > threshold && TimeNow() - startTime < 5){

                   }

                   killMotors(150);

                   leftGMH.SetPercent(40);
                   rightGMH.SetPercent(40);
               }

               else if (LeftOptoSensor.Value() < threshold){
                   rightGMH.SetPercent(30);
                   while(RightOptoSensor.Value() > threshold && TimeNow() - startTime < 5){

                   }
                   killMotors(150);
                   leftGMH.SetPercent(30);
                   while(CenterOptoSensor.Value() > threshold && TimeNow() - startTime < 5){

                   }

                   killMotors(150);

                   leftGMH.SetPercent(40);
                   rightGMH.SetPercent(40);
               }

           }


       }




       killMotors(100);
       move_forward(40,20);
       move_forward(-30,2);
       LCD.WriteLine("Line following complete.");
   }

   /**
    * @brief acquireSample
    *
    * Purpose: The robot will use the forklift to pull out the sample.
    *
    * Requires: The robot is aligned under the sample.
    *
    */
   void acquireSample(){
       LCD.WriteLine("Acquiring sample......");

       driveFor(4.5,-25);

       setForklift(180);

       driveFor(4.3,25);

       setForklift(140);

       driveFor(4.0,-30);

       setForklift(120);

       LCD.WriteLine("Sample acquired.");

   }

   /**
    * @brief manueverToLowerLevel
    *
    * Purpose: The robot will move from the upper level to the lower level.
    *
    * requires: The robot is on the upper level.
    */
   void manueverToLowerLevel(){
       LCD.WriteLine("Manuevering to lower level.....");
       driveFor(4.0,-50);
       turnCCW(45);
       checkHeading(90);
       check_x_minus(rpsXtemp + 9.);
       turnCW(90);
       checkHeading(3);
       driveFor(30.0,-60);
       LCD.WriteLine("Manuever complete.");


   }

   /**
    * @brief driveToLight
    * Purpose: The robot will slowly drive straight until the CdS cell picks up the light, the robot will then measure the value for 3 seconds and determine the color.
    *
    * Requires: The robot is in front of the light.
    *
    * Note: Not implemeneted for final competition.
    */
   void driveToLight(){
       float startTime = TimeNow();
       leftGMH.SetPercent(25);
       while(CdSCell.Value() > 1.5  || TimeNow() - startTime > 4.0){

       }
       rightGMH.SetPercent(25);

       startTime = TimeNow();
       float sumVoltage = 0;
       int counter = 1;
       float avgVoltage = 0;
       while(TimeNow() - startTime < 3.0){
           Sleep(400);
           sumVoltage += CdSCell.Value();
           counter++;
       }

       avgVoltage = sumVoltage / counter;

       if(avgVoltage > 1.5){
           //Bad times, should not ever happen.
           color = 'b';
       }
       else if(avgVoltage > 0.35){
           color = 'b';
       }
       else if (avgVoltage < 0.35){
           color = 'r';
       }

   }


   /**
    * @brief bucketDeposition
    *
    * Purpose: The robot will drop the sample into the appropriate bucket based on the color of light.
    *
    * requires: The robot must have detected the color of light.
    *
    * Note: Not implemented for Final Competition.
    */
   void bucketDeposition(){
       LCD.WriteLine("Depositing sample in bucket.....");


       LCD.WriteLine("Deposition complete.");
   }

   /**
    * @brief manueverToFinalButton
    *
    * Purpose: Move the robot from the buckets to alignment with the final button.
    *
    *
    * requires: The robot is on the lower level. Robot is low enough to not run into bunker walls.
    */
   void manueverToFinalButton(){
       LCD.WriteLine("Manuevering to Final Button.");
       turnCCW(90);
       checkHeading(92);
       driveUntilCollision(50,6.0);
       driveFor(.5,-30);
       turnCW(90);
       checkHeadingZero();
       LCD.WriteLine("Manuevering complete. Proceeding to push final button.");
   }



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
   void check_y_plus(float y_coordinate){

       while(RPS.Y() < y_coordinate - .2 || RPS.Y() > y_coordinate + .2){
           Sleep(120);

           if(RPS.Y() > y_coordinate){
               move_forward(-35,2);
           }
           else if(RPS.Y() < y_coordinate){
               move_forward(35,2);
           }

       }



   }

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
   void check_y_minus(float y_coordinate){

       while(RPS.Y() < y_coordinate - .2 || RPS.Y() > y_coordinate + .2){
           Sleep(120);

           if(RPS.Y() > y_coordinate){
               move_forward(35,2);
           }
           else if(RPS.Y() < y_coordinate){
               move_forward(-35,2);
           }

       }




   }

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
   void check_x_plus(float x_coordinate){

       while(RPS.X() < x_coordinate - .2 || RPS.X() > x_coordinate + .2){
           Sleep(120);

           if(RPS.X() > x_coordinate){
               move_forward(-35,2);
           }
           else if(RPS.X() < x_coordinate){
               move_forward(35,2);
           }

       }


   }

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
   void check_x_minus(float x_coordinate){
       while(RPS.X() < x_coordinate - .2 || RPS.X() > x_coordinate + .2){
           Sleep(120);

           if(RPS.X() > x_coordinate){
               move_forward(35,2);
           }
           else if(RPS.X() < x_coordinate){
               move_forward(-35,2);
           }

       }

   }

   /**
    * @brief turn_right
    * purpose: Turn the robot by a fixed number of counts
    * @param percent:
    * The percent to set the motors at. Can be positive or negative.
    * @param counts:
    * The number of counts to turn by.
    *
    * requires: nothing
    */

   void turn_right(int percent,int counts){
       right_encoder.ResetCounts();
       left_encoder.ResetCounts();
       rightGMH.SetPercent(-percent);
       leftGMH.SetPercent(percent);

       while((right_encoder.Counts() + left_encoder.Counts()) / 2  < counts);

       killMotors(0);
   }


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
   void checkHeading(float heading){
       while(RPS.Heading() < heading - .5|| RPS.Heading() > heading + .5)
       {
           if(RPS.Heading() > heading )
           {
               //pulse the motors for a short duration in the correct direction
               if(RPS.Heading() - heading > 180){ // CCW

                   turn_right(-30,1);
               }
               else{  //CW
                   turn_right(30,1);

               }

           }else if(RPS.Heading() <= heading)
           {
               //pulse the motors for a short duration in the correct direction

               if(heading - RPS.Heading() > 180){ // CW
                   turn_right(30,1);
               }
               else{ // CCW
                   turn_right(-30,1);


               }
           }

          Sleep(100);
       }

       leftGMH.Stop();
       rightGMH.Stop();
   }

   /**
    * @brief checkHeadingZero
    *
    * Purpose: Check and adjust the angle of the robot when the desired angle is 0.
    *
    *
    * requires: Nothing.
    */
   void checkHeadingZero(){
       while(RPS.Heading() < 359 && RPS.Heading() > 1)
       {
           if(RPS.Heading() < 180) // CW
           {
               //pulse the motors for a short duration in the correct direction
                    turn_right(30,1);

           }else if(RPS.Heading() > 180) // CCW
           {
               //pulse the motors for a short duration in the correct direction

                   turn_right(-30,1);
           }

          Sleep(100);
       }

       leftGMH.Stop();
       rightGMH.Stop();
   }

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
       left_encoder.ResetCounts();
       right_encoder.ResetCounts();
       leftGMH.SetPercent(percent);
       rightGMH.SetPercent(percent);

       float startTime = TimeNow();

       while((left_encoder.Counts() + right_encoder.Counts()) / 2 < counts && TimeNow() - startTime < 2.0){}

       killMotors(0);
   }


