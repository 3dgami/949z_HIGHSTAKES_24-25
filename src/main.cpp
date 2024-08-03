#include "main.h"
//#include "selection.h"
//#include "selection.ccp"
#include "lemlib/api.hpp" // IWYU pragma: keep
#include "lemlib/chassis/trackingWheel.hpp"
#include "pros/misc.h"
#include "pros/rtos.hpp"
//#include <cstddef>



//update all motor ports if needed
pros::Controller master{CONTROLLER_MASTER};	
pros::Motor right_front(10);
pros::Motor left_front(20);
pros::Motor left_back(11);
pros::Motor right_back(1);
pros::Motor right_mid(33);
pros::Motor left_mid(22);
pros::Motor Intake(14);
pros::Motor IntakeB(13);
pros::MotorGroup driveL_train({20, 11});
pros::MotorGroup driveR_train({10, 1});
pros::IMU imu(2);

// drivetrain settings
lemlib::Drivetrain drivetrain(&driveL_train, // left motor group
                              &driveR_train, // right motor group
                              14, // 10 inch track width
                              lemlib::Omniwheel::NEW_325, // using new 4" omnis
                              360, // drivetrain rpm is 360
                              2 // horizontal drift is 2. If we had traction wheels, it would have been 8
);

// lateral motion controller
lemlib::ControllerSettings linearController(10, // proportional gain (kP)
                                            0, // integral gain (kI)
                                            3, // derivative gain (kD)
                                            3, // anti windup
                                            1, // small error range, in inches
                                            100, // small error range timeout, in milliseconds
                                            3, // large error range, in inches
                                            500, // large error range timeout, in milliseconds
                                            20 // maximum acceleration (slew)
);

// angular motion controller
lemlib::ControllerSettings angularController(2, // proportional gain (kP)
                                             0, // integral gain (kI)
                                             10, // derivative gain (kD)
                                             3, // anti windup
                                             1, // small error range, in degrees
                                             100, // small error range timeout, in milliseconds
                                             3, // large error range, in degrees
                                             500, // large error range timeout, in milliseconds
                                             0 // maximum acceleration (slew)
);

// sensors for odometry
lemlib::OdomSensors sensors(nullptr, // vertical tracking wheel
                            nullptr, // vertical tracking wheel 2, set to nullptr as we don't have a second one
                            nullptr, // horizontal tracking wheel
                            nullptr, // horizontal tracking wheel 2, set to nullptr as we don't have a second one
                            &imu // inertial sensor
);

// input curve for throttle input during driver control
lemlib::ExpoDriveCurve throttleCurve(3, // joystick deadband out of 127
                                     10, // minimum output where drivetrain will move out of 127
                                     1.019 // expo curve gain
);

// input curve for steer input during driver control
lemlib::ExpoDriveCurve steerCurve(3, // joystick deadband out of 127
                                  10, // minimum output where drivetrain will move out of 127
                                  1.019 // expo curve gain
);

// create the chassis
lemlib::Chassis chassis(drivetrain, linearController, angularController, sensors, &throttleCurve, &steerCurve);



void SetDriveRelative(int ticks, int Lspeed, int Rspeed)
	{

		left_front.move_relative(-(ticks), Lspeed);
		left_back.move_relative(-(ticks), Lspeed);
		left_mid.move_relative(-(ticks), Lspeed);
		right_front.move_relative(ticks, Rspeed);
		right_back.move_relative(ticks, Rspeed);
		right_mid.move_relative(ticks, Rspeed);
	}

void SetDrive(int Lspeed, int Rspeed)
	{
		left_mid.move(-(Lspeed));
		left_front.move(-(Lspeed));
		left_back.move(-(Lspeed));
		right_front.move(Rspeed);
		right_back.move(Rspeed);
		right_mid.move(Rspeed);
	}

double getLeftPos()
{
	return (left_front.get_position() + left_back.get_position() + left_mid.get_position()) / 3;
}

double getRightPos()
{
	return (right_front.get_position() + right_back.get_position() + right_mid.get_position()) / 3;
}

double getPos()
{
	return (getLeftPos() + getRightPos()) / 2;
}

void driveTrain(int distance, int timeout)
{

	driveL_train.set_reversed(true);
	int startPos = getPos();
	double kp = 12.00;
	double ki = 1.0;
	double kd = -10.50;   /*derivitive should control and stop overshooting this can be done
						  by having kd be negative or having a (P + I - D) for the output PS 
						*/
	double P;
	double I = 0;
	double D;
	int lastError = 0;
	int errorTerm = 0;
	int errorTotal = 0;
	int sign;
	int count = 0;
	int SERT;
	int SERT_count = 0;
	bool SERT_bool = false;

	sign = (distance < 0) ? -1 : 1;
	
	errorTerm = distance + startPos - getPos();

	while (errorTerm > 1 or errorTerm < -1 and count < timeout) // and SERT_count < SERTx
	{
		if(count > timeout or SERT_count > SERT)
		{
			break;
			printf("TIMEOUT \n");
		}

		errorTerm = distance + startPos - getPos();

		int Pos = getPos();

		errorTotal = errorTotal + errorTerm;

		sign = (errorTerm < 0) ? -1 : 1;


		errorTotal = (errorTotal > 500 / ki) ? 500 / ki : errorTotal;

		P = errorTerm * kp;
		//I = errorTotal * ki;
		D = (lastError - errorTerm) * kd;
		double output = (((P + I + D) + (1000 * sign)));

		printf("O=%0.2f, P=%0.2f, D=%0.2f, Position=%d, startPos=%d Err=%d\n",output, P, D, Pos, startPos, errorTerm);

		driveL_train.move_voltage(output);
		driveR_train.move_voltage(output);

		lastError = errorTerm;
		pros::delay(20);
		
		//SERT_bool = (errorTerm < 15) ? true : false;
		//SERT_count = (SERT_bool = true) ? SERT_count + 20 : SERT_count;
		count += 20;

	}
	driveL_train.move_voltage(0);
	driveR_train.move_voltage(0);
	printf("End\nErr=%d", errorTerm);
	driveL_train.set_reversed(false);

	return;
}

void turn(int angle)
{
	driveL_train.set_reversed(false);
	double CircleTicks = 2750;
	int turnTicks = (CircleTicks/360) * angle;
	int count = 0;



	int startPos = getPos();
	double kp = 11.0;
	double ki = 0.1;
	double kd = -5.50; /*derivitive should control and stop overshooting this can be done
						  by having kd be negative or having a (P + I - D) for the output
						*/
	double P;
	double I;
	double D;
	int lastError = 0;
	int errorTerm;
	int errorTotal = 0;
	int sign = 1;

	sign = (angle > 0) ? 1 : -1;

	errorTerm = (turnTicks + startPos) - floor(getPos());


	printf("start\n");
	while (errorTerm > 1 or errorTerm < -1 and count <= 2000)
	{

		if(count > 2000)
		{
			break;
			printf("TIMEOUT \n");
		}

		errorTerm = (turnTicks + startPos) - floor(getPos());

		sign = (errorTerm < 0) ? -1 : 1;

		int pos = getPos();

		errorTotal = errorTotal + errorTerm;

		errorTotal = (errorTotal > 500 / ki) ? 500 / ki : errorTotal;


		P = errorTerm * kp;
		I = errorTotal * ki;
		D = (lastError - errorTerm) * kd;
		int output = (((P + D)) + (1500 * sign));


		printf("step err=%d, P=%.02f, D=%.02f, StartPos=%d, Pos=%d, O=%d turn=%d count=%d \n", errorTerm, P, D, startPos, pos, output, turnTicks, count);


		driveL_train.move_voltage(output);
		driveR_train.move_voltage(output);

		lastError = errorTerm;
		pros::delay(10);
		count += 10;
	}
	driveL_train.move_voltage(0);
	driveR_train.move_voltage(0);
	driveL_train.set_reversed(true);

	pros::delay(10);
	printf("\nDone err=%d\n, O=%d", errorTerm, turnTicks);

	return;
}

void Centralturn(int angle, bool side)
{	
	driveL_train.set_reversed(true);
	double CircleTicks = 5400;
	int turnTicks = (CircleTicks/360) * angle;
	int count = 0;
	int startPos;
	double Pos;

	double kp = 2.0;
	double ki = 0.1;
	double kd = -5.50; /*derivitive should control and stop overshooting this can be done
						  by having kd be negative or having a (P + I - D) for the output
						*/
	double P;
	double I;
	double D;
	int lastError = 0;
	int errorTerm;
	int errorTotal = 0;
	int sign = 1;

	sign = (angle > 0) ? 1 : -1;


	if(side == 1)
	{
		startPos = getRightPos();
	}
	
	else if(side == 0)
	{
		startPos = getLeftPos();
	}
	Pos = (side == 1) ? getRightPos() : getLeftPos();

	errorTerm = (turnTicks + startPos) - Pos;

	while(errorTerm > 1 or errorTerm < 1 and count <= 3000)
	{
		if(count > 3000)
		{
			break;
			printf("TIMEOUT \n");
		}

		Pos = (side == 1) ? getRightPos() : getLeftPos();
		errorTerm = (turnTicks + startPos) - floor(Pos);

		sign = (errorTerm < 0) ? -1 : 1;

		int pos = getPos();

		errorTotal = errorTotal + errorTerm;

		errorTotal = (errorTotal > 500 / ki) ? 500 / ki : errorTotal;


		P = errorTerm * kp;
		I = errorTotal * ki;
		D = (lastError - errorTerm) * kd;
		int output = (((P + D)) + (1250 * sign));


		printf("step err=%d, P=%.02f, D=%.02f, StartPos=%d, Pos=%d, O=%d turn=%d count=%d \n", errorTerm, P, D, startPos, pos, output, turnTicks, count);

		if(side == 1)
		{
			driveR_train.move_voltage(output);
		}
		else if(side == 0)
		{
			driveL_train.move_voltage(output);
		}
		
		lastError = errorTerm;
		pros::delay(10);
		count += 10;
	}
	driveL_train.move_voltage(0);
	driveR_train.move_voltage(0);

	pros::delay(10);
	printf("\nDone err=%d\n, O=%d", errorTerm, turnTicks);

	return;
}

void gyroTurn(int angle)
{
	printf("start \n");
	driveL_train.set_reversed(true);
	driveR_train.set_reversed(true);

	double heading = imu.get_heading();

	double kp = 80.0;
	double ki = 0.1;
	double kd = -6.50; /*derivitive should control and stop overshooting this can be done
						  by having kd be negative or having a (P + I - D) for the output
						*/
	double P;
	double I;
	double D;
	int lastError = 0;
	double errorTerm;
	int errorTotal = 0;
	int sign = 1; 
	int count = 0;
	double ActualAngle;

	ActualAngle = (heading + angle) > 360 ? heading + angle - 360 : heading + angle;
	double diff = heading - ActualAngle;
	if(diff < -180)
	{
		errorTerm = -(360 + diff);
		sign = (errorTerm < 0) ? sign = -1 : sign = 1;
	}
	else if(diff > 180)
	{
		errorTerm = 360 - diff;
		sign = (errorTerm < 0) ? sign = -1 : sign = 1;
	}
	else
	{
		errorTerm = -diff;
		sign = (errorTerm < 0) ? sign = -1 : sign = 1;
	}
	errorTerm = fabs(errorTerm);


	printf("start\n");
	while (errorTerm > 0.5 or errorTerm < -0.5 and count <= 2000) 
	{

		if(count > 2000)
		{
			break;
			printf("TIMEOUT \n");
		}

		heading = imu.get_heading();
		diff = heading - angle;
		if(diff < -180)
		{
			errorTerm = -(360 + diff);
			sign = (errorTerm < 0) ? sign = -1 : sign = 1;
		}
		else if(diff > 180)
		{
			errorTerm = 360 - diff;
			sign = (errorTerm < 0) ? sign = -1 : sign = 1;
		}
		else
		{
			errorTerm = -diff;
			sign = (errorTerm < 0) ? sign = -1 : sign = 1;
		}
		errorTerm = fabs(errorTerm);

		errorTotal = errorTotal + errorTerm;

		errorTotal = (errorTotal > 500 / ki) ? 500 / ki : errorTotal;


		P = errorTerm * kp;
		I = errorTotal * ki;
		D = (lastError - errorTerm) * kd;
		int output = (((P + D) + 1500) * sign);


		printf("err=%0.2f, P=%.02f, D=%.02f, O=%d, heading=%0.2f count=%d \n", errorTerm, P, D, output, heading, count);


		driveL_train.move_voltage(output);
		driveR_train.move_voltage(output);

		lastError = errorTerm;
		pros::delay(10);
		count += 10;
	}
	driveL_train.move_voltage(0);
	driveR_train.move_voltage(0);
	driveL_train.set_reversed(true);
	driveR_train.set_reversed(false);

	pros::delay(100);
	printf("Heading=%0.2f \n", imu.get_heading());

	return;
}

void AbsGyroTurn(int angle)
{
	printf("start \n");
	driveL_train.set_reversed(true);
	driveR_train.set_reversed(true);

	double heading = imu.get_heading();

	double kp = 80.0;
	double ki = 0.1;
	double kd = -6.50; /*derivitive should control and stop overshooting this can be done
						  by having kd be negative or having a (P + I - D) for the output
						*/
	double P;
	double I;
	double D;
	int lastError = 0;
	double errorTerm;
	int errorTotal = 0;
	int sign = 1; 
	int count = 0;

	double diff = heading - angle;
	if(diff < -180)
	{
		errorTerm = -(360 + diff);
		sign = (errorTerm < 0) ? sign = -1 : sign = 1;
	}
	else if(diff > 180)
	{
		errorTerm = 360 - diff;
		sign = (errorTerm < 0) ? sign = -1 : sign = 1;
	}
	else
	{
		errorTerm = -diff;
		sign = (errorTerm < 0) ? sign = -1 : sign = 1;
	}
	errorTerm = fabs(errorTerm);


	printf("start\n");
	while (errorTerm > 0.5 or errorTerm < -0.5 and count <= 2000) 
	{

		if(count > 2000)
		{
			break;
			printf("TIMEOUT \n");
		}

		heading = imu.get_heading();
		diff = heading - angle;
		if(diff < -180)
		{
			errorTerm = -(360 + diff);
			sign = (errorTerm < 0) ? sign = -1 : sign = 1;
		}
		else if(diff > 180)
		{
			errorTerm = 360 - diff;
			sign = (errorTerm < 0) ? sign = -1 : sign = 1;
		}
		else
		{
			errorTerm = -diff;
			sign = (errorTerm < 0) ? sign = -1 : sign = 1;
		}
		errorTerm = fabs(errorTerm);

		errorTotal = errorTotal + errorTerm;

		errorTotal = (errorTotal > 500 / ki) ? 500 / ki : errorTotal;


		P = errorTerm * kp;
		I = errorTotal * ki;
		D = (lastError - errorTerm) * kd;
		int output = (((P + D) + 1500) * sign);


		printf("err=%0.2f, P=%.02f, D=%.02f, O=%d, heading=%0.2f count=%d \n", errorTerm, P, D, output, heading, count);


		driveL_train.move_voltage(output);
		driveR_train.move_voltage(output);

		lastError = errorTerm;
		pros::delay(10);
		count += 10;
	}
	driveL_train.move_voltage(0);
	driveR_train.move_voltage(0);
	driveL_train.set_reversed(true);
	driveR_train.set_reversed(false);

	pros::delay(100);
	printf("Heading=%0.2f \n", imu.get_heading());

	return;
}

void on_center_button() {}

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize()
{
	/*pros::lcd::initialize(); // initialize brain screen
    chassis.calibrate(); // calibrate sensors

    // the default rate is 50. however, if you need to change the rate, you
    // can do the following.
    // lemlib::bufferedStdout().setRate(...);
    // If you use bluetooth or a wired connection, you will want to have a rate of 10ms

    // for more information on how the formatting for the loggers
    // works, refer to the fmtlib docs

    // thread to for brain screen and position logging
    pros::Task screenTask([&]() {
        while (true) {
            // print robot location to the brain screen
            pros::lcd::print(0, "X: %f", chassis.getPose().x); // x
            pros::lcd::print(1, "Y: %f", chassis.getPose().y); // y
            pros::lcd::print(2, "Theta: %f", chassis.getPose().theta); // heading
            // log position telemetry
            lemlib::telemetrySink()->info("Chassis pose: {}", chassis.getPose());
            // delay to save resources
            pros::delay(50);
        }
    });*/
	//selector::init();
	//Inertial_Sensor.reset(true /*true*/);
	
}

/**
 * Runs while the robot is in the disabled state of Field Management System or
 * the VEX Competition Switch, following either autonomous or opcontrol. When
 * the robot is enabled, this task will exit.
 */
void disabled() {}

/**
 * Runs after initialize(), and before autonomous when connected to the Field
 * Management System or the VEX Competition Switch. This is intended for
 * competition-specific initialization routines, such as an autonomous selector
 * on the LCD.
 *
 * This task will exit when the robot is enabled and autonomous or opcontrol
 * starts.
 */
void competition_initialize()
{}

/**
 * Runs the user autonomous code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the autonomous
 * mode. Alternatively, this function may be called in initialize or opcontrol
 * for non-competition testing purposes.
 *
 * If the robot is disabled or communications is lost, the autonomous task
 * will be stopped. Re-enabling the robot will restart the task, not re-start it
 * from where it left off.
 */

 // path file name is "example.txt".
// "." is replaced with "_" to overcome c++ limitations
ASSET(Path_01_txt);
ASSET(Path_02_txt);
ASSET(Path_03_txt);
ASSET(example_txt);

void autonomous() 
{
	 // set chassis pose
    //chassis.setPose(0, 0, 0);
    // lookahead distance: 15 inches
    // timeout: 2000 ms
    //chassis.follow(Path_01_txt, 15, 2000);
    // follow the next path, but with the robot going backwards
    //chassis.follow(Path_01_txt, 15, 2000, false);


	/*if(selector::auton == 1)
	{

	}

	if(selector::auton == 2)
	{	
	}

	if(selector::auton == 3)
	{
		//do nothing 
	
	}

	if(selector::auton == 0)
	{
		 //skills
	}*/
	
	driveR_train.move_voltage(0);
	driveL_train.move_voltage(0);
	
	
}

/**
 * Runs the operator control code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the operator
 * control mode.
 *
 * If no competition control is connected, this function will run immediately
 * following initialize().
 *
 * If the robot is disabled or communications is lost, the
 * operator control task will be stopped. Re-enabling the robot will restart the
 * task, not resume it from where it left off.
 */

void opcontrol()
{
	int left;
	int right;
	bool IntakeState;
	bool ExpansionState;
	bool ExpansionState2;
	auto ExpansionHook = 'A';
	auto ExpansionIntake ='B';

	pros::c::adi_pin_mode(ExpansionIntake, OUTPUT);
	pros::c::adi_digital_write(ExpansionIntake, LOW);

	pros::c::adi_pin_mode(ExpansionHook, OUTPUT);
	pros::c::adi_digital_write(ExpansionHook, LOW);

	while(true){
		
		/*TANK CONTROL*/
		/*
		driveR_train.set_reversed(true);
		driveL_train.move(master.get_analog(ANALOG_LEFT_Y));
		driveR_train.move(master.get_analog(ANALOG_RIGHT_Y));
		*/
		
		/*ARCADE CONTROLL*/

		int power = -(master.get_analog(ANALOG_RIGHT_X));
		int turn = master.get_analog(ANALOG_LEFT_Y);
		left = power - turn;
		right = power + turn;

		driveL_train.move(left);
		driveR_train.move(right);

		if(master.get_digital_new_press(DIGITAL_A))
		{
			if(IntakeState == true)
			{
				Intake.move_velocity(0);
				IntakeB.move_velocity(0);
				IntakeState = false;
			}
			else
			{
				Intake.move_velocity(100);
				IntakeB.move_velocity(100);
				IntakeState = true;
			}
			printf("Intake state=%d \n", IntakeState);
		}

		if(Intake.get_actual_velocity() < 10 and IntakeState == true)
		{
			Intake.move_velocity(-100);
			pros::delay(200);
			Intake.move_velocity(100);
		}

		if(master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_R1))
		{
			if(ExpansionState == true)
			{
				pros::c::adi_digital_write(ExpansionHook, LOW);
				ExpansionState = false;
			}
			else
			{
				pros::c::adi_digital_write(ExpansionHook, HIGH);
				ExpansionState = true;
			}
			printf("Expansion state=%d \n", ExpansionState);
		}

		if(master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_L1))
		{
			if(ExpansionState2 == true)
			{
				pros::c::adi_digital_write(ExpansionIntake, LOW);
				ExpansionState2 = false;
			}
			else
			{
				pros::c::adi_digital_write(ExpansionIntake, HIGH);
				ExpansionState2 = true;
			}
			printf("Expansion state=%d \n", ExpansionIntake);
		}
	};
}
