#include <FEHLCD.h>
#include <FEHIO.h>
#include <FEHUtility.h>
#include <FEHMotor.h>
#include <FEHServo.h>


#define PI 3.141592653589793238462


//FEHMotor leftGMH = new FEHMotor()
//FEHMotor rightGMH = new FEHMotor()
//FEHServo forkliftFutaba = new FEHServo()
//AnalogInputPin CdSCell = new AnalogInputPin();
// Add more sensors after this.
//Optosenser and such

//Function prototypes from here on.

struct worldState {
	//Relevant info about the worldstate must be included here
	//This will be vital for error checking and the like.
	char task; // A character that denotes what task will be performed


} wState;

void waitForStart(); // Wait for the light to toggle on

void startManuevers(); // Perform the initial backing up

void turn90CW(); // Turn 90 degrees clockwise

void turn45CW(); // Turn 45 degrees Clockwise

void turn90CCW(); // Turn 90 degrees counterclockwise

void turn45CCW(); // Turn 45 degrees counterclockwise

void driveUntilCollision(int percent); // Drive in a straightline until collision

void reverseUntilCollision(int percent); //Reverse until a collision

void driveFor(float distance, int percent); //Drive straight for distance

void reverseFor(float distance, int percent);//Drive backwards for distance.

void alignToButton(); //From the wall on the top part of the course.
					  //Align the robot appropriatley in the horizontal direction such that if it were to turn 90 degrees and drive straight it would push the seismograph button.

void driveToOrangeLine();//Will drive straight until reaches orange line. Then stops.

void followLine();//Follows the line until the forklift is under the core sample.

void lowerForklift(int degrees); //Lowers the forklift using servos

void raiseForklift(int degrees); //Raises the forklift using servos

char updateCurrentTask(wState state); // Checks if all requirements for current task are done. Use flags at certain methods. Once all flags are triggered update task.

void performTask(wState state);//A monstrous method that will be repeatedly called by main. Will check what the current task is and will perform it.

void checkForProblem(wState state); // Another powerful method that runs various diagnostics very quickly to check for common errors.

int main(void)
{
	// wState state;

	//Eventually the robot will be a lot more procedural
	//    while(true){
	//        state.task = updateCurrentTask(state);
	//        performTask(state);
	//    }

	waitForStart();

	startManuevers();

	driveFor(12.0, 40);

	turn90CCW();

	reverseUntilCollision(20);

	alignToButton();
	turn90CW();
	driveUntilCollision(20);
	sleep(1000);

	reverseFor(1.0, 10);
	turn90CCW();
	turn45CCW();
	driveFor(20.5, 20);
	turn45CW();
	driveUntilCollision(20);

}

//void waitForStart(){
//    while(CdSCell.value() < threshold){}
//}

//    void startManuevers(){
//        reverseFor(6.5,20);
//        turn90CW();
//        reverseUntilCollision(20); // Straighten out.
//        driveUntilCollision(20); // Drive until hits opposite wall
//        turn90CCW(); // Face the ramp.
//    }

void driveFor(float distance, int percent) {
	//Shaft encoding required.


}


