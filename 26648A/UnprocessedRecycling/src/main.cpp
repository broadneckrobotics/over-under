#pragma region VEXcode Generated Robot Configuration
// Make sure all required headers are included.
#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <math.h>
#include <string.h>


#include "vex.h"

using namespace vex;

// Brain should be defined by default
brain Brain;


// START V5 MACROS
#define waitUntil(condition)                                                   \
  do {                                                                         \
    wait(5, msec);                                                             \
  } while (!(condition))

#define repeat(iterations)                                                     \
  for (int iterator = 0; iterator < iterations; iterator++)
// END V5 MACROS


// Robot configuration code.
controller Controller1 = controller(primary);
motor leftMotorA = motor(PORT2, ratio18_1, false);
motor leftMotorB = motor(PORT11, ratio18_1, false);
motor_group LeftDriveSmart = motor_group(leftMotorA, leftMotorB);
motor rightMotorA = motor(PORT19, ratio18_1, true);
motor rightMotorB = motor(PORT9, ratio18_1, true);
motor_group RightDriveSmart = motor_group(rightMotorA, rightMotorB);
inertial DrivetrainInertial = inertial(PORT1);
smartdrive Drivetrain = smartdrive(LeftDriveSmart, RightDriveSmart, DrivetrainInertial, 319.19, 320, 40, mm, 1);

motor IntakeMotorA = motor(PORT21, ratio6_1, false);
motor IntakeMotorB = motor(PORT10, ratio6_1, true);
motor_group Intake = motor_group(IntakeMotorA, IntakeMotorB);

motor LauncherMotorA = motor(PORT13, ratio36_1, false);
motor LauncherMotorB = motor(PORT14, ratio36_1, true);
motor_group Launcher = motor_group(LauncherMotorA, LauncherMotorB);

/*vex-vision-config:begin*/
vision VisionSensor = vision (PORT20, 50);
/*vex-vision-config:end*/
digital_out DigitalOutA = digital_out(Brain.ThreeWirePort.A);

void calibrateDrivetrain() {
  wait(200, msec);
  Brain.Screen.print("Calibrating");
  Brain.Screen.newLine();
  Brain.Screen.print("Inertial");
  DrivetrainInertial.calibrate();
  while (DrivetrainInertial.isCalibrating()) {
    wait(25, msec);
  }

  // Clears the screen and returns the cursor to row 1, column 1.
  Brain.Screen.clearScreen();
  Brain.Screen.setCursor(1, 1);
}

// Helper to make playing sounds from the V5 in VEXcode easier and
// keeps the code cleaner by making it clear what is happening.
void playVexcodeSound(const char *soundName) {
  printf("VEXPlaySound:%s\n", soundName);
  wait(5, msec);
}



// define variable for remote controller enable/disable
bool RemoteControlCodeEnabled = true;
// define variables used for controlling motors based on controller inputs
bool DrivetrainLNeedsToBeStopped_Controller1 = true;
bool DrivetrainRNeedsToBeStopped_Controller1 = true;

// define a task that will handle monitoring inputs from Controller1
int rc_auto_loop_function_Controller1() {
  // process the controller input every 20 milliseconds
  // update the motors based on the input values
  while(true) {
    if(RemoteControlCodeEnabled) {
      // stop the motors if the brain is calibrating
      if (DrivetrainInertial.isCalibrating()) {
        LeftDriveSmart.stop();
        RightDriveSmart.stop();
        while (DrivetrainInertial.isCalibrating()) {
          wait(25, msec);
        }
      }
      
      // calculate the drivetrain motor velocities from the controller joystick axies
      // left = Axis3 + Axis1
      // right = Axis3 - Axis1
      int drivetrainLeftSideSpeed = Controller1.Axis3.position() + Controller1.Axis1.position();
      int drivetrainRightSideSpeed = Controller1.Axis3.position() - Controller1.Axis1.position();
      
      // check if the value is inside of the deadband range
      if (drivetrainLeftSideSpeed < 5 && drivetrainLeftSideSpeed > -5) {
        // check if the left motor has already been stopped
        if (DrivetrainLNeedsToBeStopped_Controller1) {
          // stop the left drive motor
          LeftDriveSmart.stop();
          // tell the code that the left motor has been stopped
          DrivetrainLNeedsToBeStopped_Controller1 = false;
        }
      } else {
        // reset the toggle so that the deadband code knows to stop the left motor nexttime the input is in the deadband range
        DrivetrainLNeedsToBeStopped_Controller1 = true;
      }
      // check if the value is inside of the deadband range
      if (drivetrainRightSideSpeed < 5 && drivetrainRightSideSpeed > -5) {
        // check if the right motor has already been stopped
        if (DrivetrainRNeedsToBeStopped_Controller1) {
          // stop the right drive motor
          RightDriveSmart.stop();
          // tell the code that the right motor has been stopped
          DrivetrainRNeedsToBeStopped_Controller1 = false;
        }
      } else {
        // reset the toggle so that the deadband code knows to stop the right motor next time the input is in the deadband range
        DrivetrainRNeedsToBeStopped_Controller1 = true;
      }
      
      // only tell the left drive motor to spin if the values are not in the deadband range
      if (DrivetrainLNeedsToBeStopped_Controller1) {
        LeftDriveSmart.setVelocity(drivetrainLeftSideSpeed, percent);
        LeftDriveSmart.spin(forward);
      }
      // only tell the right drive motor to spin if the values are not in the deadband range
      if (DrivetrainRNeedsToBeStopped_Controller1) {
        RightDriveSmart.setVelocity(drivetrainRightSideSpeed, percent);
        RightDriveSmart.spin(forward);
      }
    }
    // wait before repeating the process
    wait(20, msec);
  }
  return 0;
}

task rc_auto_loop_task_Controller1(rc_auto_loop_function_Controller1);

#pragma endregion VEXcode Generated Robot Configuration

// Make sure all required headers are included.
#include "vex.h"
#include <string>
  
using namespace vex;

// Code by Hamza
// Last edit Feb 9 2024
// MIT licensed

// ======================
// Launcher && Intake && Wings
// ======================

int initLauncherAndIntake() {
  bool Controller1UpDownButtonsControlMotorsStopped = true;
  bool Controller1ForwardBackButtonsControlMotorsStopped = true;
  while (true) {
      // Turn launcher motor
      if (Controller1.ButtonY.pressing()) {
        Launcher.spin(forward);
        Controller1UpDownButtonsControlMotorsStopped = false;
      } else if (Controller1.ButtonA.pressing()) {
        Launcher.spin(reverse);
        Controller1UpDownButtonsControlMotorsStopped = false;
      } else if (!Controller1UpDownButtonsControlMotorsStopped) {
        Launcher.stop();
        Controller1UpDownButtonsControlMotorsStopped = true;
      }

      // Turn intake motor
      if (Controller1.ButtonR2.pressing()) {
        Intake.spin(forward);
        Controller1ForwardBackButtonsControlMotorsStopped = false;
      } else if (Controller1.ButtonR1.pressing()) {
        Intake.spin(reverse);
        Controller1ForwardBackButtonsControlMotorsStopped = false;
      } else if (!Controller1ForwardBackButtonsControlMotorsStopped) {
        Intake.stop();
        Controller1ForwardBackButtonsControlMotorsStopped = true;
      }
  }
  wait(20, msec);
}

void wingsOut() {
  DigitalOutA.set(true);
}

void wingsIn() {
  DigitalOutA.set(false);
}

// ======================
// Autonomous/Drive Code
// ======================

competition Competition = competition();

// Function to run when the "autonomous" signal is received
void runOnAutonomous() {
  Brain.Screen.print("Running Autonomous...");
  Brain.Screen.newLine();

  calibrateDrivetrain();
  Brain.Screen.print("Drivetrain Calibrated");
  Brain.Screen.newLine();

  Drivetrain.setDriveVelocity(90, percent);
  Drivetrain.driveFor(reverse, 35, inches);
  Drivetrain.driveFor(forward, 10, inches);

  Brain.Screen.print("Finished Running Autonomous");
  Brain.Screen.newLine();
}

// Function to run when the "driver control" signal is received
void runOnDriverControl() {
  Brain.Screen.print("Running Driver Control");
  Brain.Screen.newLine();

  initLauncherAndIntake();
}


// ======================
// Main
// ======================

int main() {
  Brain.Screen.print("Program Started");
  Brain.Screen.newLine();

  Competition.autonomous(runOnAutonomous);
  Competition.drivercontrol(runOnDriverControl);

  Intake.setVelocity(50, percent);

  Launcher.setVelocity(80, percent);
  Launcher.setMaxTorque(80, percent);

  Drivetrain.setTurnVelocity(100, percent);

  Controller1.ButtonX.pressed(wingsOut);
  Controller1.ButtonB.pressed(wingsIn);

  if (Competition.isCompetitionSwitch()) {
    Brain.Screen.print("Competition Switch Connected");
    Brain.Screen.newLine();
  }
}
