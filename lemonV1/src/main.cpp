#include "main.h"
#include "lemlib/api.hpp" // IWYU pragma: keep


// Drivetrain motors. negative = reversed direction
pros::MotorGroup leftMotors({-11, 12, -13}, pros::MotorGearset::blue);
pros::MotorGroup rightMotors({14, -15, 16}, pros::MotorGearset::blue); 
// Motors, Sensors, Pneumatics, Controller go here
pros::Controller controller(pros::E_CONTROLLER_MASTER);
//Defines front, back, top, indexer motors
pros::Motor frIntake(17);
pros::Motor bkIntake(-19);
pros::Motor tpIntake(18);
pros::Motor indexer(20);
//Defines sensors
pros::Optical color(2);
pros::Imu imu(9);
//Defines Pneumatics
pros::adi::DigitalOut lwMech('H');
pros::adi::DigitalOut hood('E');
// Odometry Ports
pros::Rotation horizontalEnc(10);
pros::Rotation verticalEnc(8);
//Finding offsets -4.75 inches from the line of symetry parallel to the wheel
lemlib::TrackingWheel horizontal(&horizontalEnc, lemlib::Omniwheel::NEW_2, -4.75);
lemlib::TrackingWheel vertical(&verticalEnc, lemlib::Omniwheel::NEW_2, -0.5);
//Drivetrain Configuration
lemlib::Drivetrain drivetrain(
    &leftMotors, //Drivetrain
    &rightMotors,
    15, //wideness
    lemlib::Omniwheel::NEW_325, //Wheel Size
    450, //Drivetrain RPM
    8 //2 = Omni. 8 = Traction
);

// lateral motion controller
lemlib::ControllerSettings linearController(
    8.5, // proportional gain (kP)
    0.01, // integral gain (kI)
    9, // derivative gain (kD)
    3, // anti windup
    0.5, // small error range, in inches
    150, // small error range timeout, in milliseconds
    3.5, // large error range, in inches
    500, // large error range timeout, in milliseconds
    15 // maximum acceleration (slew)
);

// angular motion controller
lemlib::ControllerSettings angularController(
    1.8,   // kP (less aggressive)
    0,     // kI (leave off unless drift over time)
    11.5,    // kD (more damping)
    3,     // anti-windup
    2,     // small error (deg)
    150,   // small timeout (ms)
    5,     // large error (deg)
    500,   // large timeout (ms)
    5      // small angular slew rate
);

// sensors for odometry
lemlib::OdomSensors sensors(
    &vertical, // vertical tracking wheel
    nullptr, //Config for 2nd Wheel = Null
    &horizontal, // horizontal tracking wheel
    nullptr, //Config for 2nd Wheel = Null
    &imu // inertial sensor
);

// input curve for throttle input during driver control
lemlib::ExpoDriveCurve throttleCurve(
    3, // joystick deadband out of 127
    10, // minimum output where drivetrain will move out of 127
    1.019 // expo curve gain
);

// input curve for steer input during driver control
lemlib::ExpoDriveCurve steerCurve(
    3, // joystick deadband out of 127
    10, // minimum output where drivetrain will move out of 127
    1.019 // expo curve gain
);

//Your Chassis
lemlib::Chassis chassis(drivetrain, linearController, angularController, sensors, &throttleCurve, &steerCurve);

//Init
void initialize() {
    pros::lcd::initialize(); //Init Brain
    chassis.calibrate(); //Calibration

    // the default rate is 50.
    // lemlib::bufferedStdout().setRate(...); to change
    //Thread
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
    });
}

//Runs while Disabled
void disabled() {}

//Runs when connected to field controller
void competition_initialize() {}

//Pathing for Path.Jerry.io
ASSET(test2_txt); // '.' replaced with "_" to make c++ happy

//Autonomous Mode(15 and 1 minute here)
void autonomous() {
    /*
    // Move to x: 20 and y: 15, and face heading 90. Timeout set to 4000 ms
    chassis.moveToPose(20, 15, 90, 4000);
    // Move to x: 0 and y: 0 and face heading 270, going backwards. Timeout set to 4000ms
    chassis.moveToPose(0, 0, 270, 4000, {.forwards = false});
    // cancel the movement after it has traveled 10 inches
    chassis.waitUntil(10);
    chassis.cancelMotion();
    // Turn to face the point x:45, y:-45. Timeout set to 1000
    // dont turn faster than 60 (out of a maximum of 127)
    chassis.turnToPoint(45, -45, 1000, {.maxSpeed = 60});
    // Turn to face a direction of 90º. Timeout set to 1000
    // will always be faster than 100 (out of a maximum of 127)
    // also force it to turn clockwise, the long way around
    chassis.turnToHeading(90, 1000, {.direction = AngularDirection::CW_CLOCKWISE, .minSpeed = 100});
    // Follow the path in path.txt. Lookahead at 15, Timeout set to 4000
    // following the path with the back of the robot (forwards = false)
    // see line 116 to see how to define a path
    chassis.follow(example_txt, 15, 4000, false);
    // wait until the chassis has traveled 10 inches. Otherwise the code directly after
    // the movement will run immediately
    // Unless its another movement, in which case it will wait
    chassis.waitUntil(10);
    pros::lcd::print(4, "Traveled 10 inches during pure pursuit!");
    // wait until the movement is done
    chassis.waitUntilDone();
    pros::lcd::print(4, "pure pursuit finished!");
    */
   chassis.setPose(0, 0, 0);
   chassis.moveToPose(0, 24, 0, 10000);
}
//Pneumatic Control 
int a = 0;
//Color Sort
float hue;
std::string toEject = "blue";
bool isEjecting = false;

void ejectBallTask(void*) {
    isEjecting = true;

    frIntake.move(127);
    bkIntake.move(-127);
    indexer.move(-127);
    tpIntake.move(127);
    pros::delay(75);
    hood.set_value(1);
    pros::delay(500);  
    hood.set_value(0);

    isEjecting = false;
}

//Driver Control
void opcontrol() {
    // Function to eject the ball
    auto ejectBall = [&]() {
        frIntake.move(127);
        bkIntake.move(-127);
        indexer.move(-127);
        tpIntake.move(127);
        pros::delay(75);
        hood.set_value(1);
        pros::delay(0);
        hood.set_value(0);
    };
    // controller
    // loop to continuously update motors
    while (true) {
        hue = color.get_hue();
        if (controller.get_digital(DIGITAL_A) && controller.get_digital(DIGITAL_LEFT)) {
        //printf("Autonomous Selected");
        autonomous();
      }
        int leftY = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
        int rightX = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);
        // move the chassis with controller
        chassis.arcade(leftY, rightX);
        //Entire Intake/Outtake Sequence with color sort
        if(controller.get_digital(DIGITAL_R1)){
            frIntake.move(127);
            bkIntake.move(127);
            indexer.move(127);
            color.set_led_pwm(0);
        }else if(controller.get_digital(DIGITAL_R2)){
            frIntake.move(-127);
            bkIntake.move(-127);
            color.set_led_pwm(0);
        }else if(controller.get_digital(DIGITAL_L2)){
            color.set_led_pwm(100);
            //Needs Color Sort
            hue = color.get_hue();
            frIntake.move(127);
            bkIntake.move(-127);
            indexer.move(-127);
            tpIntake.move(-127);
        }else if(controller.get_digital(DIGITAL_L1)){
            color.set_led_pwm(100);
            frIntake.move(127);
            bkIntake.move(-127);
            indexer.move(-127);
            tpIntake.move(127);
            if (!isEjecting) {
                if ((toEject == "red" && ((hue >= 340 && hue <= 359) || (hue >= 0 && hue <= 20))) ||
                    (toEject == "blue" && (hue >= 200 && hue <= 240))) {
                    pros::Task task1(ejectBallTask);
                }
    }
        }else{
            frIntake.move(0);
            bkIntake.move(0);
            indexer.move(0);
            tpIntake.move(0);
        }
        //Pneumatic Control for LW Mech
        if (controller.get_digital(DIGITAL_Y)){
            //A controls the extend/retract of the piston
            if (a==0){
                a = 1;
                lwMech.set_value(1);
                while(controller.get_digital(DIGITAL_Y)){pros::delay(10);}
            } else if(a == 1){
                a = 0;
                lwMech.set_value(0);
                while(controller.get_digital(DIGITAL_Y)){pros::delay(10);}
            }
        }
    
       
        pros::delay(10);
        //final delay for busy waiting
}
}
