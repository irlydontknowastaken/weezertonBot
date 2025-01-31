#include "main.h"
#include "lemlib/api.hpp" // IWYU pragma: keep
#include "config.hpp"
using namespace pros;


const int greenGearset = 200;
const int redGearset = 100;
pros::Controller controller(pros::E_CONTROLLER_MASTER);

MotorGroup LeftMotorGroup({FRONTLEFTMOTORPORT, MIDDLELEFTMOTORPORT, BACKLEFTMOTORPORT}, MotorGearset::green);
MotorGroup RightMotorGroup({FRONTRIGHTMOTORPORT, MIDDLERIGHTMOTORPORT, BACKRIGHTMOTORPORT}, MotorGearset::green);

Motor BackGoonMotor(BACKGOONPORT, MotorGearset::green);

Motor GoonVeyerMotor(GOONVEYERPORT, MotorGearset::green);

/*Change the 15 into the length between the front and back wheels, starting from middle of the wheel*/
lemlib::Drivetrain DriveTrain(&LeftMotorGroup, &RightMotorGroup, 10, lemlib::Omniwheel::OLD_4, 200, 2);

// lateral PID controller
lemlib::ControllerSettings lateral_controller(10, // proportional gain (kP)
                                              0, // integral gain (kI)
                                              3, // derivative gain (kD)
                                              3, // anti windup
                                              1, // small error range, in inches
                                              100, // small error range timeout, in milliseconds
                                              3, // large error range, in inches
                                              500, // large error range timeout, in milliseconds
                                              20 // maximum acceleration (slew)
);

// angular PID controller
lemlib::ControllerSettings angular_controller(2, // proportional gain (kP)
                                              0, // integral gain (kI)
                                              10, // derivative gain (kD)
                                              3, // anti windup
                                              1, // small error range, in degrees
                                              100, // small error range timeout, in milliseconds
                                              3, // large error range, in degrees
                                              500, // large error range timeout, in milliseconds
                                              0 // maximum acceleration (slew)
);

lemlib::OdomSensors sensors(nullptr,nullptr,nullptr,nullptr,nullptr);

lemlib::ExpoDriveCurve throttle_curve(3, // joystick deadband out of 127
                                     10, // minimum output where drivetrain will move out of 127
                                     1.019 // expo curve gain
);

// input curve for steer input during driver control
lemlib::ExpoDriveCurve steer_curve(3, // joystick deadband out of 127
                                  10, // minimum output where drivetrain will move out of 127
                                  1.019 // expo curve gain
);

// create the chassis
lemlib::Chassis chassis(DriveTrain, // drivetrain settings
                        lateral_controller, // lateral PID settings
                        angular_controller,
						sensors,
                        &throttle_curve, 
                        &steer_curve // angular PID settings
);

void on_center_button() {
	static bool pressed = false;
	pressed = !pressed;
	if (pressed) {
		pros::lcd::set_text(2, "I was pressed!");
	} else {
		pros::lcd::clear_line(2);
	}
}

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
    pros::lcd::initialize(); // initialize brain screen
    chassis.calibrate(); // calibrate sensors
    BackGoonMotor.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
    GoonVeyerMotor.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);

    // print position to brain screen
    pros::Task screen_task([&]() {
        while (true) {
            // print robot location to the brain screen
            pros::lcd::print(0, "X: %f", chassis.getPose().x); // x
            pros::lcd::print(1, "Y: %f", chassis.getPose().y); // y
            pros::lcd::print(2, "Theta: %f", chassis.getPose().theta); // heading
            // delay to save resources
            pros::delay(20);
        }
    });
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
void competition_initialize() {}

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
void autonomous() {}

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
void opcontrol() {
    // loop forever
    while (true) {
        // get left y and right x positions
        int leftY = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
        int leftX = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_X);

        bool backMotorEngaged = true;
        bool yButton = false;
        bool aButton = false;
        bool bButton = false;
        bool l1Button = false;
        bool l2Button = false;
        
        
        chassis.curvature(leftY, leftX);

        if(controller.get_digital(pros::E_CONTROLLER_DIGITAL_R1)){
            BackGoonMotor.move_velocity(redGearset);
        }
        else if(controller.get_digital(pros::E_CONTROLLER_DIGITAL_R2)){
            BackGoonMotor.move_velocity(-redGearset);
        }
        else{
            BackGoonMotor.brake();
        }

        if(controller.get_digital(pros::E_CONTROLLER_DIGITAL_L1)){
            GoonVeyerMotor.move_velocity(greenGearset);
        }
        else if(controller.get_digital(pros::E_CONTROLLER_DIGITAL_L2)){
            GoonVeyerMotor.move_velocity(-greenGearset);
        }
        else{
            GoonVeyerMotor.brake();
        }




        if(controller.get_digital(pros::E_CONTROLLER_DIGITAL_L1)){
            l1Button = !l1Button;
        }
        if(controller.get_digital(pros::E_CONTROLLER_DIGITAL_L2)){
            l2Button = !l2Button;
        }

        // delay to save resources
        pros::delay(goon);
    }
}