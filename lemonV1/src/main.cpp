#include "main.h"
#include "lemlib/api.hpp"
#include "pros/apix.h"
#include "liblvgl/lvgl.h"



//DRIVETRAIN. negative = reversed direction
pros::MotorGroup leftMotors({-11, 12, -13}, pros::MotorGearset::blue);
pros::MotorGroup rightMotors({14, -15, 16}, pros::MotorGearset::blue); 
//BRAIN PORTS
pros::Controller controller(pros::E_CONTROLLER_MASTER);
//INTAKE MOTORS
pros::Motor frIntake(17);
pros::Motor bkIntake(-19);
pros::Motor tpIntake(18);
pros::Motor indexer(20);
//SENSORS
pros::Optical color(2);
pros::Imu imu(9);
pros::Distance tpDist(7);
//PNEUMATICS
pros::adi::DigitalOut lwMech('H');
pros::adi::DigitalOut hood('E');
//ODOMETRY
pros::Rotation horizontalEnc(10);
pros::Rotation verticalEnc(8);
//Finding offsets -4.75 inches from the line of symetry parallel to the wheel
lemlib::TrackingWheel horizontal(&horizontalEnc, lemlib::Omniwheel::NEW_2, 6.975);
lemlib::TrackingWheel vertical(&verticalEnc, lemlib::Omniwheel::NEW_2, -0.01);
//DT CONFIGURATION
lemlib::Drivetrain drivetrain(
    &leftMotors, //Drivetrain
    &rightMotors,
    12.5, //wideness
    lemlib::Omniwheel::NEW_325, //Wheel Size
    450, //Drivetrain RPM
    8 //2 = Omni. 8 = Traction
);
//PID
lemlib::ControllerSettings linearController(
    7.5, // proportional gain (kP)
    0.01, // integral gain (kI)
    11, // derivative gain (kD)
    3, // anti windup
    0.5, // small error range, in inches
    150, // small error range timeout, in milliseconds
    3.5, // large error range, in inches
    500, // large error range timeout, in milliseconds
    20 // maximum acceleration (slew)
);
//TURN PID
lemlib::ControllerSettings angularController(
    1.9,   // kP (less aggressive)
    0,     // kI (leave off unless drift over time)
    11,    // kD (more damping)
    3,     // anti-windup
    2,     // small error (deg)
    150,   // small timeout (ms)
    5,     // large error (deg)
    500,   // large timeout (ms)
    5      // small angular slew rate
);
//ODOMETRY DEFINITION
lemlib::OdomSensors sensors(
    &vertical, // vertical tracking wheel
    nullptr, //Config for 2nd Wheel = Null
    &horizontal, // horizontal tracking wheel
    nullptr, //Config for 2nd Wheel = Null
    &imu // inertial sensor
);
//CONTROLLER SETTING
lemlib::ExpoDriveCurve throttleCurve(
    3, // joystick deadband out of 127
    10, // minimum output where drivetrain will move out of 127
    1.019 // expo curve gain
);
lemlib::ExpoDriveCurve steerCurve(
    3, // joystick deadband out of 127
    10, // minimum output where drivetrain will move out of 127
    1.019 // expo curve gain
);
//CHASSIS
lemlib::Chassis chassis(drivetrain, linearController, angularController, sensors, &throttleCurve, &steerCurve);


//VARIABLES
int a = 0;
//Color Sort
float hue;
std::string toEject = "red";
bool isEjecting = false;
void ejectBallTask(void*) {
    isEjecting = true;
    frIntake.move(127);
    bkIntake.move(-127);
    indexer.move(-127);
    tpIntake.move(127);
    pros::delay(75);
    hood.set_value(1);
    while(tpDist.get_distance() >= 200){
        pros::delay(10);
        //First While loop checks if the ball has entered the distance sensor laser
    }
    while(tpDist.get_distance() <= 150 ) {
        pros::delay(25);
        //Second While loop checks if the ball has left the distance sensor laser
    }
    pros::delay(300);
    hood.set_value(0);
    isEjecting = false;
}


// AUTONOMOUS SELECTOR
typedef struct {
    const char* name;
    const char* description;
    void (*run)();
} AutonRoutine;

ASSET(p1Auto_txt); // '.' replaced with "_" to make c++ happy
ASSET(p2Auto_txt);
ASSET(p4Auto_txt);

ASSET(p1AutoL_txt);
ASSET(p2AutoL_txt);
ASSET(p4AutoL_txt);

ASSET(p1AutobR_txt);
ASSET(p2AutobR_txt);
ASSET(p4AutobR_txt);

ASSET(p1AutobL_txt);
ASSET(p2AutobL_txt);
ASSET(p4AutobL_txt);

ASSET(skill1_txt);
ASSET(skill2_txt);
ASSET(skill3_txt);
ASSET(skill4_txt);
ASSET(skill5_txt);

void rRCode() { 
    chassis.setBrakeMode(MOTOR_BRAKE_HOLD);
    chassis.setPose(-45.284, -7.984, 180);
    frIntake.move(127); 
    bkIntake.move(127); 
    indexer.move(127);
    chassis.follow(p1Auto_txt, 20, 2000);
    chassis.turnToHeading(270, 2500);
    lwMech.set_value(1);
    chassis.follow(p2Auto_txt, 15, 1000);
    pros::delay(1200);
    chassis.moveToPoint(-45.284, -47.5, 1000, {.forwards = false});
    lwMech.set_value(0);
    chassis.turnToHeading(90, 1000);
    chassis.moveToPose(-29.75, -48, 90, 1500, {}, false);
    bkIntake.move(-127); 
    indexer.move(-127);
    tpIntake.move(127);
    pros::delay(3500);
    tpIntake.move(0);
    chassis.moveToPose(-45.808, -47.5, 90, 1000, {.forwards = false});
    chassis.turnToHeading(45, 1500);
    bkIntake.move(127);
    indexer.move(127);
    chassis.follow(p4Auto_txt, 20, 5000, true, false); 
    chassis.turnToHeading(50, 500, {}, false);
    bkIntake.move(-50);
    frIntake.move(-87); //tune
    indexer. move(-127);
    pros::delay(2000);
    chassis.moveToPoint(-45.284, -47.5, 1000, {.forwards = false});
}
void rLCode() { 
    chassis.setBrakeMode(MOTOR_BRAKE_HOLD);
    chassis.setPose(-45.284, 7.984, 0);
    frIntake.move(127); 
    bkIntake.move(127); 
    indexer.move(127);
    chassis.follow(p1AutoL_txt, 15, 2000);
    chassis.turnToHeading(270, 2000);
    lwMech.set_value(1);
    chassis.follow(p2AutoL_txt, 20, 1000);
    pros::delay(1200);
    chassis.moveToPoint(-45.284, 47.5, 1000, {.forwards = false});
    lwMech.set_value(0);
    chassis.turnToHeading(90, 1500);
    chassis.moveToPose(-29, 49, 90, 1500, {}, false);
    bkIntake.move(-127); 
    indexer.move(-127);
    tpIntake.move(127);
    pros::delay(3000);
    tpIntake.move(0);
    chassis.moveToPose(-45.808, 47.5, 90, 1000, {.forwards = false});
    chassis.turnToHeading(135, 1500);
    bkIntake.move(127);
    indexer.move(127);
    chassis.follow(p4AutoL_txt, 20, 4000, true, false); 
    chassis.turnToHeading(140, 500);
    pros::delay(500);
    bkIntake.move(-127);
    frIntake.move(87); //tune
    indexer. move(-127);
    tpIntake.move(-50);
    pros::delay(3500);
    chassis.moveToPoint(-45.284, 47.5, 1000, {.forwards = false});
}
void autonThree() {
    chassis.setBrakeMode(MOTOR_BRAKE_HOLD);
    chassis.setPose(-45.284, -7.984, 180);
    frIntake.move(127); 
    bkIntake.move(127); 
    indexer.move(127);
    chassis.follow(skill1_txt, 15, 5000);
    pros::delay(250);
    chassis.turnToHeading(272, 2500);
    lwMech.set_value(1);
    chassis.follow(skill2_txt, 20, 2000);
    pros::delay(1500);
    chassis.turnToHeading(270, 500, {}, false);
    pros::delay(1500);
    //first loader
    chassis.moveToPoint(-45.284, -47.5, 1000, {.forwards = false});
    lwMech.set_value(0);
    chassis.turnToHeading(90, 1500);
    bkIntake.move(0);
    indexer.move(0);
    chassis.moveToPose(-29.5, -48.25, 90, 1500, {}, false);
    bkIntake.move(-127); 
    indexer.move(-127);
    tpIntake.move(127);
    pros::delay(5000);
    tpIntake.move(0);
    chassis.moveToPose(-45.808, -47.5, 90, 1000, {.forwards = false});
    chassis.turnToHeading(0, 1500);
    tpIntake.move(-127);
    chassis.follow(skill3_txt, 25, 5000);
    chassis.turnToHeading(270, 3000);
    pros::delay(500);
    lwMech.set_value(1);
    //2nd loader
    bkIntake.move(127); 
    indexer.move(127);
    chassis.moveToPose(-68.681, 51, 270, 4000, {}, false);
    chassis.turnToHeading(270, 500, {}, false);
    pros::delay(500);
    chassis.moveToPoint(-45.284, 51.125, 1000, {.forwards = false});
    lwMech.set_value(0);
    bkIntake.move(0);
    indexer.move(0);
    chassis.turnToHeading(90, 1500);
    chassis.moveToPose(-33.5, 51.5, 90, 1500, {}, false);
    bkIntake.move(-127); 
    indexer.move(-127);
    tpIntake.move(127);
    pros::delay(5000);
    tpIntake.move(0);
    chassis.moveToPose(-45.808, 51.5, 90, 1000, {.forwards = false});
    chassis.turnToHeading(35, 1500);
    chassis.follow(skill4_txt, 20, 5000);
    bkIntake.move(127); 
    indexer.move(127);
    chassis.turnToHeading(90, 2500); 
    lwMech.set_value(1);
    //go to loader
    chassis.moveToPose(67.681, 47.25, 90, 4000, {}, false);
    pros::delay(2000);
    //3rd loader
    chassis.moveToPoint(45.284, 48, 1000, {.forwards = false});
    lwMech.set_value(0);
    bkIntake.move(0);
    indexer.move(0);
    chassis.turnToHeading(270, 1500);
    chassis.moveToPose(30.5, 47, 270, 1500, {}, false);
    bkIntake.move(-127); 
    indexer.move(-127);
    tpIntake.move(127);
    pros::delay(5000);
    chassis.moveToPoint(50.808, 47.5, 1000, {.forwards = false});
    chassis.turnToHeading(160, 3000);
    bkIntake.move(127); 
    indexer.move(127);
    chassis.moveToPose(57, 24, 180, 3000);
    chassis.turnToHeading(180, 1000);
    hood.set_value(1);
    frIntake.move(-127);
    /*
    chassis.follow(skill5_txt, 20, 5000);
    chassis.turnToHeading(90, 3000);
    lwMech.set_value(1);
    //final loader
    chassis.moveToPose(67.681, -52, 90, 4000);
    pros::delay(2000);
    chassis.moveToPose(43.681, -53.25, 90, 4000, {.forwards = false});
    lwMech.set_value(0);
    chassis.turnToHeading(270, 4000);
    chassis.moveToPose(31.5, -53, 270, 1500, {}, false);
    bkIntake.move(-127); 
    indexer.move(-127);
    tpIntake.move(127);
    pros::delay(5000);
    */
}
void bRCode(){
    chassis.setBrakeMode(MOTOR_BRAKE_HOLD);
    chassis.setPose(45.284, 7.984, 0);
    frIntake.move(127); 
    bkIntake.move(127); 
    indexer.move(127);
    chassis.follow(p1AutobR_txt, 15, 2000);
    chassis.turnToHeading(90, 2000);
    lwMech.set_value(1);
    chassis.follow(p2AutobR_txt, 20, 1000);
    pros::delay(1200);
    chassis.moveToPoint(45.284, 47.5, 1000, {.forwards = false});
    lwMech.set_value(0);
    chassis.turnToHeading(270, 1500);
    chassis.moveToPose(29, 49, 270, 1500, {}, false);
    bkIntake.move(-127); 
    indexer.move(-127);
    tpIntake.move(127);
    pros::delay(3000);
    tpIntake.move(0);
    chassis.moveToPose(45.808, 47.5, 270, 1000, {.forwards = false});
    chassis.turnToHeading(225, 1500);
    bkIntake.move(127);
    indexer.move(127);
    chassis.follow(p4AutobR_txt, 20, 4000, true, false); 
    chassis.turnToHeading(220, 500);
    pros::delay(500);
    bkIntake.move(-127);
    frIntake.move(-87); //tune
    indexer. move(-127);
    pros::delay(3500);
    chassis.moveToPoint(45.284, 47.5, 1000, {.forwards = false});
}
void bLCode(){
    chassis.setBrakeMode(MOTOR_BRAKE_HOLD);
    chassis.setPose(45.284, -7.984, 180);
    frIntake.move(127); 
    bkIntake.move(127); 
    indexer.move(127);
    chassis.follow(p1AutobL_txt, 20, 2000);
    chassis.turnToHeading(90, 2500);
    lwMech.set_value(1);
    chassis.follow(p2AutobL_txt, 15, 1000);
    pros::delay(1200);
    chassis.moveToPoint(45.284, -47.5, 1000, {.forwards = false});
    lwMech.set_value(0);
    chassis.turnToHeading(270, 1000);
    chassis.moveToPose(29.75, -48.5, 270, 1500, {}, false);
    bkIntake.move(-127); 
    indexer.move(-127);
    tpIntake.move(127);
    pros::delay(3500);
    tpIntake.move(0);
    chassis.moveToPose(45.808, -47.5, 270, 1000, {.forwards = false});
    chassis.turnToHeading(315, 1500);
    bkIntake.move(127);
    indexer.move(127);
    chassis.follow(p4AutobL_txt, 20, 5000, true, false); 
    chassis.turnToHeading(315, 500, {}, false);
    bkIntake.move(-127);
    frIntake.move(127); //tune
    indexer.move(-127);
    tpIntake.move(-80);
    pros::delay(2000);
    chassis.moveToPoint(45.284, -47.5, 1000, {.forwards = false});
}

int autonPage = 0;
AutonRoutine autons[] = {
    { "Auton 1", "Red Right Auton. Scores 4 Top 3 Bottom", autonThree},
    { "Auton 2", "Red Left Auton. Scores 4 Top 3 Middle", rLCode},
    { "Auton 3", "Skills WIP(54 Max)", autonThree }, 
    {"Auton 4", "Blue Right Auton. Scores 4 Top 3 Bottom, ", bRCode}, 
    {"Auton 5", "Blue Left Auton. Scores 4 Top 3 Bottom", bLCode}, 
};
const int autonCount = sizeof(autons) / sizeof(autons[0]);
lv_obj_t* autonTitleLabel;
lv_obj_t* autonDescLabel;

void updateAutonDisplay() {
    lv_label_set_text(autonTitleLabel, autons[autonPage].name);
    lv_label_set_text(autonDescLabel, autons[autonPage].description);
}

void leftBtnCB(lv_event_t* e) {
    if (lv_event_get_code(e) != LV_EVENT_CLICKED) return;
    // debug feedback
    if (autonPage > 0) {
        autonPage--;
        updateAutonDisplay();
    }
}

void rightBtnCB(lv_event_t* e) {
    if (lv_event_get_code(e) != LV_EVENT_CLICKED) return;
    if (autonPage < autonCount - 1) {
        autonPage++;
        updateAutonDisplay();
    }
}


void createAutonSelector() {

    lv_obj_t* screen = lv_screen_active();

    autonTitleLabel = lv_label_create(screen);
    lv_obj_align(autonTitleLabel, LV_ALIGN_TOP_MID, 0, 10);

    autonDescLabel = lv_label_create(screen);
    lv_label_set_long_mode(autonDescLabel, LV_LABEL_LONG_WRAP);
    lv_obj_set_width(autonDescLabel, 200);
    lv_obj_align(autonDescLabel, LV_ALIGN_CENTER, 0, 0);

    lv_obj_t* leftBtn = lv_button_create(screen);
    lv_obj_align(leftBtn, LV_ALIGN_LEFT_MID, 10, 0);
    lv_obj_add_event_cb(leftBtn, leftBtnCB, LV_EVENT_ALL, NULL);
    lv_obj_t* leftLabel = lv_label_create(leftBtn);
    lv_label_set_text(leftLabel, "<");
    lv_obj_center(leftLabel);

    lv_obj_t* rightBtn = lv_button_create(screen);
    lv_obj_align(rightBtn, LV_ALIGN_RIGHT_MID, -10, 0);
    lv_obj_add_event_cb(rightBtn, rightBtnCB, LV_EVENT_ALL, NULL);
    lv_obj_t* rightLabel = lv_label_create(rightBtn);
    lv_label_set_text(rightLabel, ">");
    lv_obj_center(rightLabel);
    lv_obj_set_size(leftBtn, 50, 50);
    lv_obj_set_size(rightBtn, 50, 50);
    updateAutonDisplay();
}



//INITIALIZATION
void initialize() {
    //add pros::lcd::initialize(); if lv_init does not work
    lv_init();
    createAutonSelector();
    chassis.calibrate(); //Calibration
    //Thread
    pros::Task screenTask([&]() {
        while (true) {
            lv_timer_handler();  // LVGL processes touch and drawing
            pros::delay(5);      // Small delay to prevent CPU overload
        }
    });
}

//Runs while Disabled
void disabled() {}
//Runs when connected to field controller
void competition_initialize() {}
//Pathing for Path.Jerry.io

//Autonomous Mode(15 and 1 minute here)
void autonomous() {
    autons[autonPage].run();
}
//Driver Control
void opcontrol() {
    chassis.setPose(0, 0, 0);
    // controller
    // loop to continuously update motors
    while (true) {
        chassis.setBrakeMode(MOTOR_BRAKE_COAST);
        hue = color.get_hue();
        if (controller.get_digital(DIGITAL_A) && controller.get_digital(DIGITAL_LEFT)) {
            autonomous();
      }
        int leftY = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
        int rightX = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);
        // move the chassis with controller
        chassis.arcade(leftY, rightX);
        //Entire Intake/Outtake Sequence with color sort
        if(controller.get_digital(DIGITAL_R1)){
            //toEject = "red";
            frIntake.move(127);
            bkIntake.move(127);
            indexer.move(127);
            color.set_led_pwm(0);
        }else if(controller.get_digital(DIGITAL_R2)){
            frIntake.move(-127);
            bkIntake.move(-127);
            indexer.move(-127);
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
                if ((toEject == "red" && ((hue >= 340 && hue <= 359) || (hue >= 0 && hue <= 20)))){
                // || (toEject == "blue" && (hue >= 200 && hue <= 240))) 
                    //pros::Task task1(ejectBallTask);
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
