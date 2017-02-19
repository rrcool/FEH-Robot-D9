#include <FEHLCD.h>
#include <FEHIO.h>
#include <FEHUtility.h>
#include <FEHMotor.h>
#include <FEHServo.h>


#define PI 3.141592653589793238462
#define SHAFT_ENCODER_CYCLES_PER_INCH 40.489 // Modify this value
#define SHAFT_ENCODER_COUNTS_FOR_45 10
#define SHAFT_ENCODER_COUNTS_FOR_90 10 //Determine these two by trial and error
FEHMotor leftGMH(FEHMotor::Motor0, voltage_here);
FEHMotor rightGMH(FEHMotor::Motor1, voltage_here);

AnalogInputPin CdSCell(FEHIO::P0_0);//Change the port as needed.



DigitalInputPin NWBump(FEHIO::P1_0);
DigitalInputPin NEBump(FEHIO::P3_7);
DigitalInputPin SWBump(FEHIO::P2_0);
DigitalInputPin SEBump(FEHIO::P2_7);

DigitalEncoder left_encoder(FEHIO::P0_2);
DigitalEncoder right_encoder(FEHIO::P3_2);


//FEHServo forkliftFutaba = new FEHServo()
// Add more sensors after this.
//Optosenser and such

//Function prototypes from here on.

struct worldState {
	//Relevant info about the worldstate must be included here
	//This will be vital for error checking and the like.
	int task; // An int that denotes what task will be performed


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

void alignToButton(); //From the wall on the top part of the course.
					  //Align the robot appropriatley in the horizontal direction such that if it were to turn 90 degrees and drive straight it would push the seismograph button.

void driveToOrangeLine();//Will drive straight until reaches orange line. Then stops.

void followLine();//Follows the line until the forklift is under the core sample.

void lowerForklift(int degrees); //Lowers the forklift using servos

void raiseForklift(int degrees); //Raises the forklift using servos

								 //char updateCurrentTask(wState state); // Checks if all requirements for current task are done. Use flags at certain methods. Once all flags are triggered update task.

								 //void performTask(wState state);//A monstrous method that will be repeatedly called by main. Will check what the current task is and will perform it.

								 //void checkForProblem(wState state); // Another powerful method that runs various diagnostics very quickly to check for common errors.

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
	Sleep(1000);

	driveFor(1.0, -10);
	turn90CCW();
	turn45CCW();
	driveFor(20.5, -20);
	turn45CW();
	driveUntilCollision(20);

}

void waitForStart() {
	while (CdSCell.Value() < threshold) {} // Determine threshold based on color of lights
}

void startManuevers() {
	driveFor(6.5, -20);
	turn90CW();
	reverseUntilCollision(20); // Straighten out.
	driveUntilCollision(20); // Drive until hits opposite wall
	turn90CCW(); // Face the ramp.
}

void driveFor(float distance, int percent) {
	//Shaft encoding required.
	left_encoder.ResetCounts();
	right_encoder.ResetCounts();
	leftGMH.SetPercent(percent);
	rightGMH.SetPercent(percent);

	while ((1 / SHAFT_ENCODER_CYCLES_PER_INCH)*left_encoder.Counts()<distance) {
		//Check for possible errors in here.
	}

	leftGMH.Stop();
	rightGMH.Stop();
}

void turn90CW() {
	left_encoder.ResetCounts();
	right_encoder.ResetCounts();
	leftGMH.SetPercent(10);
	rightGMH.SetPercent(-10);

	while ((left_encoder.Counts()<SHAFT_ENCODER_COUNTS_FOR_90) && (right_encoder.Counts()<SHAFT_ENCODER_COUNTS_FOR_90)) {
		//Check for errors
	}
	leftGMH.Stop();
	rightGMH.Stop();

}

void turn90CCW() {
	left_encoder.ResetCounts();
	right_encoder.ResetCounts();
	leftGMH.SetPercent(-10);
	rightGMH.SetPercent(10);

	while ((left_encoder.Counts()<SHAFT_ENCODER_COUNTS_FOR_90) && (right_encoder.Counts()<SHAFT_ENCODER_COUNTS_FOR_90)) {
		//Check for errors
	}
	leftGMH.Stop();
	rightGMH.Stop();
}

void turn45CW() {
	left_encoder.ResetCounts();
	right_encoder.ResetCounts();
	leftGMH.SetPercent(10);
	rightGMH.SetPercent(-10);

	while ((left_encoder.Counts()<SHAFT_ENCODER_COUNTS_FOR_45) && (right_encoder.Counts()<SHAFT_ENCODER_COUNTS_FOR_45)) {
		//Check for errors
	}
	leftGMH.Stop();
	rightGMH.Stop();
}

void turn45CCW() {
	left_encoder.ResetCounts();
	right_encoder.ResetCounts();
	leftGMH.SetPercent(-10);
	rightGMH.SetPercent(10);

	while ((left_encoder.Counts()<SHAFT_ENCODER_COUNTS_FOR_45) && (right_encoder.Counts()<SHAFT_ENCODER_COUNTS_FOR_45)) {
		//Check for errors
	}
	leftGMH.Stop();
	rightGMH.Stop();
} //Consider consolidation of all these methods into one turn method.

void driveUntilCollision(int percent) {
	leftGMH.SetPercent(20);
	rightGMH.SetPercent(20);
	while (NWBump.Value() || NEBump.Value()) {}//Will continue until both switches are pressed.

	leftGMH.Stop();
	rightGMH.Stop();
}

void reverseUntilCollision(int percent) {
	leftGMH.SetPercent(-20);
	rightGMH.SetPercent(-20);
	while (SWBump.Value() || SEBump.Value()) {}//Will continue until both switches are pressed.

	leftGMH.Stop();
	rightGMH.Stop();
}

void alignToButton() { //Consider removal.
	leftGMH.Stop();
	rightGMH.Stop();

	driveFor(4, 10);
}
