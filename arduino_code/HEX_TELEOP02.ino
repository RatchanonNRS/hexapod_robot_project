#include <ArduinoHardware.h>
#include <ros.h>
#include <sensor_msgs/JointState.h> // For publishing joint states
#include <std_msgs/Char.h>          // For subscribing to char commands

#include <Servo.h>
#include <Wire.h> // Keep Wire for other potential I2C uses, but not strictly needed without MPU6050

// IMU related includes - COMMENTED OUT
// #include <MPU6050_light.h> // Library for MPU6050
// #include <sensor_msgs/Imu.h> // ROS IMU message type
// #include <tf/tf.h>         // Not directly used on Arduino for IMU msg, but useful for conversions if needed


#ifndef PI
#define PI 3.14159265358979323846f // Changed to float literal by adding 'f'
#endif

// --- SERVO OBJECTS ---
Servo coxa[6], femur[6], tibia[6];

// --- PIN ASSIGNMENTS (VERIFY THESE CAREFULLY!) ---
const int COXA_PINS[6] = {22, 25, 28, 31, 34, 37};
const int FEMUR_PINS[6] = {23, 26, 29, 32, 35, 38};
const int TIBIA_PINS[6] = {24, 27, 30, 33, 36, 39};

// --- HOME POSITION ANGLES (ADJUST IF NEEDED) ---
int COXA_HOME[6] = {90, 90, 90, 80, 90, 90};
int FEMUR_HOME[6] = {90, 90, 90, 90, 110, 90};
int TIBIA_HOME[6] = {90, 90, 90, 90, 90, 90};

// --- ARRAYS TO STORE COMMANDED ANGLES FOR PUBLISHING ---
int commanded_coxa_angles[6];
int commanded_femur_angles[6];
int commanded_tibia_angles[6];

// --- GAIT STATE MANAGEMENT VARIABLES ---
unsigned long stepStartTime = 0;
const long STEP_DURATION = 220; // milliseconds for each gait step (adjust for speed)
int currentGaitStep = 0;        // Current step in the gait cycle

// New: Enum to manage the currently active gait type
enum GaitType {
  NONE,
  FORWARD_MODE,
  BACKWARD_MODE,
  ROTATE_CW_MODE,
  ROTATE_CCW_MODE
};

GaitType activeGaitType = NONE; // Variable to store the currently active gait

// --- ROS NODE HANDLE AND JOINT STATE MESSAGES ---
ros::NodeHandle nh; // Declare NodeHandle
sensor_msgs::JointState joint_state_msg; // Declare JointState message object
ros::Publisher joint_state_pub("joint_states", &joint_state_msg); // Declare JointState publisher

// IMU RELATED DECLARATIONS - REMOVED
// MPU6050 mpu(Wire);
// sensor_msgs::Imu imu_msg;
// ros::Publisher imu_pub("imu/data", &imu_msg);
// unsigned long timer_imu = 0;
// const int IMU_PUBLISH_RATE = 20;


// --- HELPER FUNCTION: DEGREES TO RADIANS (Maps 0-180 servo to -90 to +90 relative degrees, then to radians) ---
// Changed return type and parameter type to float for consistency with sensor_msgs::JointState::position
float deg2rad(float degrees) {
  float Cdegrees = degrees - 90.0f; // Subtract 90 to center the range around 0 degrees
                                  // Use float literal for 90.0f
  return Cdegrees * (PI / 180.0f); // Convert the centered degrees to radians
                                  // Use float literal for 180.0f
}

// --- BASIC LEG MOVEMENT FUNCTION ---
void moveLeg(int legIndex, int coxaAngle, int femurAngle, int tibiaAngle) {
  coxa[legIndex].write(coxaAngle);
  femur[legIndex].write(femurAngle);
  tibia[legIndex].write(tibiaAngle);

  // Update commanded angles for ROS publishing
  commanded_coxa_angles[legIndex] = coxaAngle;
  commanded_femur_angles[legIndex] = femurAngle;
  commanded_tibia_angles[legIndex] = tibiaAngle;
}

// --- MOVE LEG TO HOME POSITION ---
void moveLegToHome(int legIndex) {
  coxa[legIndex].write(COXA_HOME[legIndex]);
  femur[legIndex].write(FEMUR_HOME[legIndex]);
  tibia[legIndex].write(TIBIA_HOME[legIndex]);

  // Update commanded angles for ROS publishing
  commanded_coxa_angles[legIndex] = COXA_HOME[legIndex];
  commanded_femur_angles[legIndex] = FEMUR_HOME[legIndex];
  commanded_tibia_angles[legIndex] = TIBIA_HOME[legIndex];
}

// --- FORWARD GAIT FUNCTION (NON-BLOCKING) ---
void Forward() {
  if (currentGaitStep == 0) {
    Serial.println("Gait Step 0: Lift & move tripod (0,2,4) forward");
    moveLeg(0, COXA_HOME[0] + 30, FEMUR_HOME[0] + 30, TIBIA_HOME[0]);
    moveLeg(1, COXA_HOME[1] - 30, FEMUR_HOME[1] - 30, TIBIA_HOME[1]);
    moveLeg(2, COXA_HOME[2] + 30, FEMUR_HOME[2] + 30, TIBIA_HOME[2]);
    stepStartTime = millis();
    currentGaitStep = 1;
  }
  else if (currentGaitStep == 1) {
    if (millis() - stepStartTime >= STEP_DURATION) {
      Serial.println("Gait Step 1: Body forward with tripod (1,3,5)");
      moveLeg(0, COXA_HOME[0] + 30, FEMUR_HOME[0], TIBIA_HOME[0]);
      moveLeg(1, COXA_HOME[1] - 30, FEMUR_HOME[1], TIBIA_HOME[1]);
      moveLeg(2, COXA_HOME[2] + 30, FEMUR_HOME[2], TIBIA_HOME[2]);

      moveLeg(3, COXA_HOME[3] - 30, FEMUR_HOME[3] - 30, TIBIA_HOME[3]);
      moveLeg(4, COXA_HOME[4] + 30, FEMUR_HOME[4] + 30, TIBIA_HOME[4]);
      moveLeg(5, COXA_HOME[5] - 30, FEMUR_HOME[5] - 30, TIBIA_HOME[5]);
      stepStartTime = millis();
      currentGaitStep = 2;
    }
  }
  else if (currentGaitStep == 2) {
    if (millis() - stepStartTime >= STEP_DURATION) {
      Serial.println("Gait Step 2: Lower tripod (0,2,4)");
      moveLeg(3, COXA_HOME[3] - 30, FEMUR_HOME[3], TIBIA_HOME[3]);
      moveLeg(4, COXA_HOME[4] + 30, FEMUR_HOME[4], TIBIA_HOME[4]);
      moveLeg(5, COXA_HOME[5] - 30, FEMUR_HOME[5], TIBIA_HOME[5]);
      stepStartTime = millis();
      currentGaitStep = 3;
    }
  }
  else if (currentGaitStep == 3) {
    if (millis() - stepStartTime >= STEP_DURATION) {
      Serial.println("Gait Step 3: Lift & move tripod (1,3,5) forward");
      moveLeg(0, COXA_HOME[0], FEMUR_HOME[0], TIBIA_HOME[0]);
      moveLeg(1, COXA_HOME[1], FEMUR_HOME[1], TIBIA_HOME[1]);
      moveLeg(2, COXA_HOME[2], FEMUR_HOME[2], TIBIA_HOME[2]);

      moveLeg(3, COXA_HOME[3], FEMUR_HOME[3], TIBIA_HOME[3]);
      moveLeg(4, COXA_HOME[4], FEMUR_HOME[4], TIBIA_HOME[4]);
      moveLeg(5, COXA_HOME[5], FEMUR_HOME[5], TIBIA_HOME[5]);
      stepStartTime = millis();
      currentGaitStep = 0;
      activeGaitType = NONE;
    }
  }
}

// --- BACKWARD GAIT FUNCTION (NON-BLOCKING) ---
void Backward() {
  if (currentGaitStep == 0) {
    Serial.println("Gait Step 0: Lift & move tripod (0,2,4) forward");
    moveLeg(0, COXA_HOME[0] - 30, FEMUR_HOME[0] + 30, TIBIA_HOME[0]);
    moveLeg(1, COXA_HOME[1] + 30, FEMUR_HOME[1] - 30, TIBIA_HOME[1]);
    moveLeg(2, COXA_HOME[2] - 30, FEMUR_HOME[2] + 30, TIBIA_HOME[2]);
    stepStartTime = millis();
    currentGaitStep = 1;
  }
  else if (currentGaitStep == 1) {
    if (millis() - stepStartTime >= STEP_DURATION) {
      Serial.println("Gait Step 1: Body forward with tripod (1,3,5)");
      moveLeg(0, COXA_HOME[0] - 30, FEMUR_HOME[0], TIBIA_HOME[0]);
      moveLeg(1, COXA_HOME[1] + 30, FEMUR_HOME[1], TIBIA_HOME[1]);
      moveLeg(2, COXA_HOME[2] - 30, FEMUR_HOME[2], TIBIA_HOME[2]);

      moveLeg(3, COXA_HOME[3] + 30, FEMUR_HOME[3] - 30, TIBIA_HOME[3]);
      moveLeg(4, COXA_HOME[4] - 30, FEMUR_HOME[4] + 30, TIBIA_HOME[4]);
      moveLeg(5, COXA_HOME[5] + 30, FEMUR_HOME[5] - 30, TIBIA_HOME[5]);
      stepStartTime = millis();
      currentGaitStep = 2;
    }
  }
  else if (currentGaitStep == 2) {
    if (millis() - stepStartTime >= STEP_DURATION) {
      Serial.println("Gait Step 2: Lower tripod (0,2,4)");
      moveLeg(3, COXA_HOME[3] + 30, FEMUR_HOME[3], TIBIA_HOME[3]);
      moveLeg(4, COXA_HOME[4] - 30, FEMUR_HOME[4], TIBIA_HOME[4]);
      moveLeg(5, COXA_HOME[5] + 30, FEMUR_HOME[5], TIBIA_HOME[5]);
      stepStartTime = millis();
      currentGaitStep = 3;
    }
  }
  else if (currentGaitStep == 3) {
    if (millis() - stepStartTime >= STEP_DURATION) {
      Serial.println("Gait Step 3: Lift & move tripod (1,3,5) forward");
      moveLeg(0, COXA_HOME[0], FEMUR_HOME[0], TIBIA_HOME[0]);
      moveLeg(1, COXA_HOME[1], FEMUR_HOME[1], TIBIA_HOME[1]);
      moveLeg(2, COXA_HOME[2], FEMUR_HOME[2], TIBIA_HOME[2]);

      moveLeg(3, COXA_HOME[3], FEMUR_HOME[3], TIBIA_HOME[3]);
      moveLeg(4, COXA_HOME[4], FEMUR_HOME[4], TIBIA_HOME[4]);
      moveLeg(5, COXA_HOME[5], FEMUR_HOME[5], TIBIA_HOME[5]);
      stepStartTime = millis();
      currentGaitStep = 0;
      activeGaitType = NONE;
    }
  }
}

// --- ROTATE CLOCKWISE GAIT (NON-BLOCKING) ---
void rotateCW() {
  if (currentGaitStep == 0) {
    Serial.println("Rotate CW Step 0: Lift & rotate right (0,3,5) inward");
    moveLeg(0, COXA_HOME[0] + 30, FEMUR_HOME[0] + 30, TIBIA_HOME[0]);
    moveLeg(1, COXA_HOME[1] + 30, FEMUR_HOME[1] - 30, TIBIA_HOME[1]);
    moveLeg(2, COXA_HOME[2] + 30, FEMUR_HOME[2] + 30, TIBIA_HOME[2]);
    stepStartTime = millis();
    currentGaitStep = 1;
  }
  else if (currentGaitStep == 1) {
    if (millis() - stepStartTime >= STEP_DURATION) {
      Serial.println("Rotate CW Step 1: Lower right tripod (0,3,5)");
      moveLeg(0, COXA_HOME[0] + 30, FEMUR_HOME[0], TIBIA_HOME[0]);
      moveLeg(1, COXA_HOME[1] + 30, FEMUR_HOME[1], TIBIA_HOME[1]);
      moveLeg(2, COXA_HOME[2] + 30, FEMUR_HOME[2], TIBIA_HOME[2]);

      moveLeg(3, COXA_HOME[3] + 30, FEMUR_HOME[3] - 30, TIBIA_HOME[3]);
      moveLeg(4, COXA_HOME[4] + 30, FEMUR_HOME[4] + 30, TIBIA_HOME[4]);
      moveLeg(5, COXA_HOME[5] + 30, FEMUR_HOME[5] - 30, TIBIA_HOME[5]);
      stepStartTime = millis();
      currentGaitStep = 2;
    }
  }
  else if (currentGaitStep == 2) {
    if (millis() - stepStartTime >= STEP_DURATION) {
      Serial.println("Rotate CW Step 2: Push left (1,2,4) outward");
      moveLeg(3, COXA_HOME[3] + 30, FEMUR_HOME[3], TIBIA_HOME[3]);
      moveLeg(4, COXA_HOME[4] + 30, FEMUR_HOME[4], TIBIA_HOME[4]);
      moveLeg(5, COXA_HOME[5] + 30, FEMUR_HOME[5], TIBIA_HOME[5]);
      stepStartTime = millis();
      currentGaitStep = 3;
    }
  }
  else if (currentGaitStep == 3) {
    if (millis() - stepStartTime >= STEP_DURATION) {
      moveLeg(0, COXA_HOME[0], FEMUR_HOME[0], TIBIA_HOME[0]);
      moveLeg(1, COXA_HOME[1], FEMUR_HOME[1], TIBIA_HOME[1]);
      moveLeg(2, COXA_HOME[2], FEMUR_HOME[2], TIBIA_HOME[2]);

      moveLeg(3, COXA_HOME[3], FEMUR_HOME[3], TIBIA_HOME[3]);
      moveLeg(4, COXA_HOME[4], FEMUR_HOME[4], TIBIA_HOME[4]);
      moveLeg(5, COXA_HOME[5], FEMUR_HOME[5], TIBIA_HOME[5]);
      stepStartTime = millis();
      currentGaitStep = 0;
      activeGaitType = NONE;
    }
  }
}

// --- ROTATE COUNTER-CLOCKWISE GAIT (NON-BLOCKING) ---
void rotateCCW() {
  if (currentGaitStep == 0) {
    Serial.println("Rotate CW Step 0: Lift & rotate right (0,3,5) inward");
    moveLeg(0, COXA_HOME[0] - 30, FEMUR_HOME[0] + 30, TIBIA_HOME[0]);
    moveLeg(1, COXA_HOME[1] - 30, FEMUR_HOME[1] - 30, TIBIA_HOME[1]);
    moveLeg(2, COXA_HOME[2] - 30, FEMUR_HOME[2] + 30, TIBIA_HOME[2]);
    stepStartTime = millis();
    currentGaitStep = 1;
  }
  else if (currentGaitStep == 1) {
    if (millis() - stepStartTime >= STEP_DURATION) {
      Serial.println("Rotate CW Step 1: Lower right tripod (0,3,5)");
      moveLeg(0, COXA_HOME[0] - 30, FEMUR_HOME[0], TIBIA_HOME[0]);
      moveLeg(1, COXA_HOME[1] - 30, FEMUR_HOME[1], TIBIA_HOME[1]);
      moveLeg(2, COXA_HOME[2] - 30, FEMUR_HOME[2], TIBIA_HOME[2]);

      moveLeg(3, COXA_HOME[3] - 30, FEMUR_HOME[3] - 30, TIBIA_HOME[3]);
      moveLeg(4, COXA_HOME[4] - 30, FEMUR_HOME[4] + 30, TIBIA_HOME[4]);
      moveLeg(5, COXA_HOME[5] - 30, FEMUR_HOME[5] - 30, TIBIA_HOME[5]);
      stepStartTime = millis();
      currentGaitStep = 2;
    }
  }
  else if (currentGaitStep == 2) {
    if (millis() - stepStartTime >= STEP_DURATION) {
      Serial.println("Rotate CW Step 2: Push left (1,2,4) outward");
      moveLeg(3, COXA_HOME[3] - 30, FEMUR_HOME[3], TIBIA_HOME[3]);
      moveLeg(4, COXA_HOME[4] - 30, FEMUR_HOME[4], TIBIA_HOME[4]);
      moveLeg(5, COXA_HOME[5] - 30, FEMUR_HOME[5], TIBIA_HOME[5]);
      stepStartTime = millis();
      currentGaitStep = 3;
    }
  }
  else if (currentGaitStep == 3) {
    if (millis() - stepStartTime >= STEP_DURATION) {
      moveLeg(0, COXA_HOME[0], FEMUR_HOME[0], TIBIA_HOME[0]);
      moveLeg(1, COXA_HOME[1], FEMUR_HOME[1], TIBIA_HOME[1]);
      moveLeg(2, COXA_HOME[2], FEMUR_HOME[2], TIBIA_HOME[2]);

      moveLeg(3, COXA_HOME[3], FEMUR_HOME[3], TIBIA_HOME[3]);
      moveLeg(4, COXA_HOME[4], FEMUR_HOME[4], TIBIA_HOME[4]);
      moveLeg(5, COXA_HOME[5], FEMUR_HOME[5], TIBIA_HOME[5]);
      stepStartTime = millis();
      currentGaitStep = 0;
      activeGaitType = NONE;
    }
  }
}

// --- STUBS FOR OTHER GAIT FUNCTIONS ---
void StepLeft() { /* Implement non-blocking step left gait here */ Serial.println("StepLeft Gait Called (Stub)"); }
void StepRight() { /* Implement non-blocking step right gait here */ Serial.println("StepRight Gait Called (Stub)"); }
void SitDown() { /* Implement non-blocking sit down here */ Serial.println("SitDown Gait Called (Stub)"); }


// --- ROS COMMAND CALLBACK FUNCTION ---
void commandCallback(const std_msgs::Char& cmd_msg) {
  char command = cmd_msg.data; // Get the character from the ROS message

  Serial.print("Arduino received ROS command: [");
  Serial.print(command);
  Serial.println("]");

  // Reset gait state for a new command
  currentGaitStep = 0;

  // Process commands received via ROS topic
  if (command == 'w' || command == 'W') {
    Serial.println("COMMAND: FORWARD");
    activeGaitType = FORWARD_MODE;
  }
  else if (command == 'x' || command == 'X') {
    Serial.println("COMMAND: BACKWARD");
    activeGaitType = BACKWARD_MODE;
  }
  else if (command == 'e' || command == 'E') {
    Serial.println("COMMAND: ROTATE CW");
    activeGaitType = ROTATE_CW_MODE;
  }
  else if (command == 'q' || command == 'Q') {
    Serial.println("COMMAND: ROTATE CCW");
    activeGaitType = ROTATE_CCW_MODE;
  }
  else if (command == 's' || command == 'S') {
    Serial.println("COMMAND: HOME POSITION");
    activeGaitType = NONE; // Stop any active gait
    for (int i = 0; i < 6; i++) {
      moveLegToHome(i);
    }
  }
  // Other commands will fall through if not handled, or you can add stubs/error messages.
  else if (command == 'a' || command == 'A') {
    Serial.println("COMMAND: STEP LEFT (Stub)");
  } else if (command == 'd' || command == 'D') {
    Serial.println("COMMAND: STEP RIGHT (Stub)");
  } else if (command == 'c' || command == 'C') {
    Serial.println("COMMAND: SIT DOWN (Stub)");
  }
}

// --- ROS SUBSCRIBER OBJECT ---
ros::Subscriber<std_msgs::Char> cmd_sub("/cmd_char", &commandCallback);


// --- ARDUINO SETUP FUNCTION ---
void setup() {
  Serial.begin(57600); // Initialize serial communication for debugging

  // Initialize ROS Node Handle
  nh.initNode();
  nh.advertise(joint_state_pub); // Advertise the joint_states publisher
  nh.subscribe(cmd_sub);        // Subscribe to the command topic
  
  Wire.begin(); // Initialize I2C communication (Still needed for some Arduino boards, even if no other I2C devices are explicitly used)

  // IMU related initialization - REMOVED
  // byte status = mpu.begin();
  // Serial.print(F("MPU6050 status: "));
  // Serial.println(status);
  // while(status!=0){
  //   Serial.println(F("MPU6050 connection failed. Check wiring."));
  //   delay(1000);
  //   status = mpu.begin();
  // }
  // mpu.calcOffsets(true, true);
  // Serial.println(F("MPU6050 calibration complete."));
  // nh.advertise(imu_pub); // IMU publisher removed

  // --- JOINT STATE MESSAGE SETUP (18 JOINTS for 6-legged robot) ---
  joint_state_msg.name_length = 18;
  joint_state_msg.position_length = 18;
  joint_state_msg.velocity_length = 0; // Not used currently
  joint_state_msg.effort_length = 0;   // Not used currently

  // Allocate memory for joint names and positions
  joint_state_msg.name = (char**)malloc(sizeof(char*) * 18);
  joint_state_msg.position = (float*)malloc(sizeof(float) * 18); // Changed to float*

  // Assign joint names (Crucial: These names must match your URDF/robot_description)
  const char* all_joint_names[] = {
    "LF_Coxa", "LF_Femur", "LF_tibia", // Leg 0
    "RM_Coxa", "RM_Femur", "RM_Tibia", // Leg 1
    "LR_Coxa", "LR_Femur", "LR_Tibia", // Leg 2
    "RF_Coxa", "RF_Femur", "RF_Tibia", // Leg 3
    "LM_Coxa", "LM_Femur", "LM_Tibia", // Leg 4
    "RR_Coxa", "RR_Femur", "RR_Tibia", // Leg 5
  };
  for (int i = 0; i < 18; ++i) {
    joint_state_msg.name[i] = (char*)all_joint_names[i];
  }

  // --- SERVO ATTACHMENT ---
  for (int i = 0; i < 6; i++) {
    coxa[i].attach(COXA_PINS[i]);
    femur[i].attach(FEMUR_PINS[i]);
    tibia[i].attach(TIBIA_PINS[i]);
  }

  // Move all legs to their home position initially
  for (int i = 0; i < 6; i++) {
    moveLegToHome(i);
  }

  Serial.println("Hexapod ready!");
}


// --- ARDUINO LOOP FUNCTION ---
void loop() {
  // Process incoming ROS messages (subscribers) and outgoing messages (publishers)
  nh.spinOnce();

  // Call the active gait function if one is set
  switch (activeGaitType) {
    case FORWARD_MODE:
      Forward();
      break;
    case BACKWARD_MODE:
      Backward();
      break;
    case ROTATE_CW_MODE:
      rotateCW();
      break;
    case ROTATE_CCW_MODE:
      rotateCCW();
      break;
    case NONE:
      // No gait active, robot is stationary or at home
      break;
  }

  // --- JOINT STATE PUBLISHING ---
  // Populate joint positions from your commanded_angles arrays.
  // Ensure the order matches the `all_joint_names` array in setup().
  joint_state_msg.position[0] = deg2rad(commanded_coxa_angles[0]);
  joint_state_msg.position[1] = deg2rad(commanded_femur_angles[0]);
  joint_state_msg.position[2] = deg2rad(commanded_tibia_angles[0]);

  joint_state_msg.position[3] = deg2rad(commanded_coxa_angles[1]);
  joint_state_msg.position[4] = deg2rad(commanded_femur_angles[1]);
  joint_state_msg.position[5] = deg2rad(commanded_tibia_angles[1]);

  joint_state_msg.position[6] = deg2rad(commanded_coxa_angles[2]);
  joint_state_msg.position[7] = deg2rad(commanded_femur_angles[2]);
  joint_state_msg.position[8] = deg2rad(commanded_tibia_angles[2]);

  joint_state_msg.position[9] = deg2rad(commanded_coxa_angles[3]);
  joint_state_msg.position[10] = deg2rad(commanded_femur_angles[3]);
  joint_state_msg.position[11] = deg2rad(commanded_tibia_angles[3]);

  joint_state_msg.position[12] = deg2rad(commanded_coxa_angles[4]);
  joint_state_msg.position[13] = deg2rad(commanded_femur_angles[4]);
  joint_state_msg.position[14] = deg2rad(commanded_tibia_angles[4]);

  joint_state_msg.position[15] = deg2rad(commanded_coxa_angles[5]);
  joint_state_msg.position[16] = deg2rad(commanded_femur_angles[5]);
  joint_state_msg.position[17] = deg2rad(commanded_tibia_angles[5]);

  // --- ADDED THIS LINE: Timestamp the JointState message ---
  joint_state_msg.header.stamp = nh.now();

  joint_state_pub.publish(&joint_state_msg); // Publish the updated joint states


  // IMU Data Publishing - REMOVED
  // if (millis() - timer_imu > (1000 / IMU_PUBLISH_RATE)) {
  //   mpu.update();
  //   imu_msg.header.stamp = nh.now();
  //   imu_msg.header.frame_id = "imu_link";
  //   imu_msg.orientation.x = 0;
  //   imu_msg.orientation.y = 0;
  //   imu_msg.orientation.z = 0;
  //   imu_msg.orientation.w = 1;
  //   imu_msg.orientation_covariance[0] = -1;
  //   imu_msg.angular_velocity.x = mpu.getGyroX() * (PI / 180.0f);
  //   imu_msg.angular_velocity.y = mpu.getGyroY() * (PI / 180.0f);
  //   imu_msg.angular_velocity.z = mpu.getGyroZ() * (PI / 180.0f);
  //   imu_msg.angular_velocity_covariance[0] = 0;
  //   imu_msg.linear_acceleration.x = mpu.getAccX() * 9.81f;
  //   imu_msg.linear_acceleration.y = mpu.getAccY() * 9.81f;
  //   imu_msg.linear_acceleration.z = mpu.getAccZ() * 9.81f;
  //   imu_msg.linear_acceleration_covariance[0] = 0;
  //   imu_pub.publish(&imu_msg);
  //   timer_imu = millis();
  // }

  delay(10); // Small delay to prevent overwhelming the serial buffer
}