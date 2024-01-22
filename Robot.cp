// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Robot.h"

void Robot::RobotInit() {
  // Smart Dashboard
  sd_chooser.SetDefaultOption("Default Auto", "Default");
  sd_chooser.AddOption("Left Auto", "Left");
  sd_chooser.AddOption("Middle Auto", "Middle");
  sd_chooser.AddOption("Right Auto", "Right");
  frc::SmartDashboard::PutData("Auto Modes", &sd_chooser);

  // Initialize Encoders

  //e_shoulder = m_shoulder.GetEncoder();
  //e_elbow = m_elbow.GetEncoder();
  //e_wrist = m_wrist.GetEncoder();
    


  // Enable Camera (Simple)
  frc::CameraServer::StartAutomaticCapture();
  
  // Enable Camera (Advanced)
  //std::thread visionThread(VisionThread);
  //visionThread.detach();

  frc::SmartDashboard::PutBoolean("compressorDisabled",false);
  frc::SmartDashboard::PutNumber("Shoulder Multiplier",.5);
  frc::SmartDashboard::PutNumber("Elbow Multiplier",.5);

  // Set Solenoids to starting position
  s_brakeLeft.Set(frc::DoubleSolenoid::kForward);
  s_brakeRight.Set(frc::DoubleSolenoid::kForward);
  s_claw.Set(frc::DoubleSolenoid::kReverse);

}

/**
 * This function is called every 20 ms, no matter the mode. Use
 * this for items like diagnostics that you want ran during disabled,
 * autonomous, teleoperated and test.
 *
 * <p> This runs after the mode specific periodic functions, but before
 * LiveWindow and SmartDashboard integrated updating.
 */
void Robot::RobotPeriodic() 
{
  // Wheel Speeds
  frc::SmartDashboard::PutNumber("Front Right", -speed_frontRight);
  frc::SmartDashboard::PutNumber("Front Left", -speed_frontLeft);
  frc::SmartDashboard::PutNumber("Back Right", -speed_backRight);
  frc::SmartDashboard::PutNumber("Back Left", -speed_backLeft);

  // Brake Status
  frc::SmartDashboard::PutBoolean("Brake1", brakeOn);
  // Claw Status
  frc::SmartDashboard::PutBoolean("Claw", clawGrab);

  // Arm Motor Power
  frc::SmartDashboard::PutNumber("Shoulder Power",speed_shoulder);
  frc::SmartDashboard::PutNumber("Elbow Power",speed_elbow);
  frc::SmartDashboard::PutNumber("Wrist Power",speed_wrist);
  // Arm Motor Speeds
  frc::SmartDashboard::PutNumber("Shoulder Velocity",e_shoulder.GetVelocity());
  frc::SmartDashboard::PutNumber("Elbow Velocity",e_elbow.GetVelocity());
  frc::SmartDashboard::PutNumber("Wrist Velocity",0.0);
  //frc::SmartDashboard::PutNumber("Wrist Velocity",e_wrist.GetVelocity());
  // Arm Encoders
  frc::SmartDashboard::PutNumber("Shoulder Position",e_shoulder.GetPosition());
  frc::SmartDashboard::PutNumber("Elbow Position",e_elbow.GetPosition());
  frc::SmartDashboard::PutNumber("Wrist Position",0.0);
  //frc::SmartDashboard::PutNumber("Wrist Position",e_wrist.GetPosition());
  
  double num[2] = {1,2};
  frc::SmartDashboard::PutNumberArray("Shoulder Values",num);
}

/**
 * This autonomous (along with the chooser code above) shows how to select
 * between different autonomous modes using the dashboard. The sendable chooser
 * code works with the Java SmartDashboard. If you prefer the LabVIEW Dashboard,
 * remove all of the chooser code and uncomment the GetString line to get the
 * auto name from the text box below the Gyro.
 *
 * You can add additional auto modes by adding additional comparisons to the
 * if-else structure below with additional strings. If using the SendableChooser
 * make sure to add them to the chooser code above as well.
 */
void Robot::AutonomousInit() {
  sd_autoSelected = sd_chooser.GetSelected();
  // m_autoSelected = SmartDashboard::GetString("Auto Selector",
  //     kAutoNameDefault);
  fmt::print("Auto selected: {}\n", sd_autoSelected);

  if (sd_autoSelected == kAutoName1) {
    m_backLeft.Set(0.2);
  } 
  if else (sd_autoSelected)
  else {
    m_backLeft.Set(0);
  }
}

void Robot::AutonomousPeriodic() {
  if (sd_autoSelected == "Left") {
    m_backLeft.Set(0.1);
    m_backRight.Set(0.1);
  } 
  
  else {
    // Default Auto goes here
  }
}

void Robot::TeleopInit() { }

void Robot::TeleopPeriodic() 
{
  // Arm Code
  
  // Get Shoulder and Elbow Multipliers
  //shoulderScale = frc::SmartDashboard::GetNumber("Shoulder Multiple",.5);
  //elbowScale = frc::SmartDashboard::GetNumber("Elbow Multiple",.5);

  // Stage 1
  speed_shoulder = Zero(c_operator.GetLeftY(),.2);
  if (speed_shoulder < 0 && !ls_stage1Lower.Get())
  {
    speed_shoulder = 0;
  }
  if (speed_shoulder > 0 && !ls_stage1Upper.Get())
  {
    speed_shoulder = 0;
  }
  m_shoulder.Set(shoulderScale*speed_shoulder);

  // Stage 2
  speed_elbow = Zero(c_operator.GetRightY(),.2);
  if (speed_elbow < 0 && !ls_stage2Lower.Get())
  {
    speed_elbow = 0;
  }
  if (speed_elbow > 0 && !ls_stage2Upper.Get())
  {
    speed_elbow = 0;
  }
  if (speed_elbow <= 0) //up
  {
    m_elbow.Set(elbowScaleUp*speed_elbow);
  }
  else //down
  {
    m_elbow.Set(elbowScaleDown*speed_elbow);
  }
  

  // Wrist
  speed_wrist = Zero(c_operator.GetRightTriggerAxis()-c_operator.GetLeftTriggerAxis(),0.05);
  m_wrist.Set(-wristScale * speed_wrist);

  if (c_driver.GetRightTriggerAxis() > .75)
  {
    scale_control = .5;
  }
  else if (c_driver.GetLeftTriggerAxis() > .75){
    scale_control = .25;
  }
  else{
    scale_control = 1;
  }

  // Drive Code - Mecanum Drive
  move_r = scale_turn * Zero(c_driver.GetRightX(),deadzone_r);
  move_x = scale_strafe * Zero(c_driver.GetLeftX(),deadzone_x);
  move_y = Zero(c_driver.GetLeftY(),deadzone_y);

  speed_frontLeft = move_y - move_r - move_x;
  speed_backLeft = move_y - move_r + move_x;
  speed_frontRight = move_y + move_r + move_x;
  speed_backRight = move_y + move_r - move_x;

  m_frontLeft.Set(-speed_frontLeft * scale_control);
  m_backLeft.Set(-speed_backLeft * scale_control);
  m_frontRight.Set(speed_frontRight * scale_control);
  m_backRight.Set(speed_backRight * scale_control);

  // Brake Controls
  if (c_driver.GetStartButtonPressed()) // Always On
  {
    s_brakeLeft.Set(frc::DoubleSolenoid::kForward);
    s_brakeRight.Set(frc::DoubleSolenoid::kForward);
    brakeOn = true;
  }
  if (c_driver.GetBackButtonPressed()) // Always Off
  {
    s_brakeLeft.Set(frc::DoubleSolenoid::kReverse);
    s_brakeRight.Set(frc::DoubleSolenoid::kReverse);
    brakeOn = false;
  }
  if (c_driver.GetAButtonPressed()) // Toggle
  {
    s_brakeLeft.Toggle();
    s_brakeRight.Toggle();
    brakeOn = !brakeOn;
  }

  // Claw Controls
  if (c_operator.GetRightBumperPressed())
  {
    s_claw.Set(frc::DoubleSolenoid::kForward);
    clawGrab = true;
  }
  if (c_operator.GetLeftBumperPressed())
  {
    s_claw.Set(frc::DoubleSolenoid::kReverse);
    clawGrab = false;
  }

  // Compressor Turn Off Override
  {
    bool newCompState = frc::SmartDashboard::GetBoolean("compressorDisabled",false);
    if (newCompState != compressorState)
    {
      if (newCompState)
      {
        compressor.Disable();
        compressorState = false;
      }
      else
      {
        compressor.IsEnabled();
        compressorState = true;
      }
    }
  }

}

void Robot::DisabledInit() {}

void Robot::DisabledPeriodic() {}

void Robot::TestInit() {}

void Robot::TestPeriodic() {}

void Robot::SimulationInit() {}

void Robot::SimulationPeriodic() {}

// Custom Functions
double Robot::Zero(double input, double range)
{
  if (input < range && input > -range)
  {
    return(0);
  }
  else
  {
    if (input > 0)
    {
      return((input - range) / (1 - range));
    }
    else
    {
      return((input + range) / (1 - range));
    }
  }
}

/*void VisionThread() {

    cs::UsbCamera camera = frc::CameraServer::StartAutomaticCapture();

    camera.SetResolution(640, 480);


    cs::CvSink cvSink = frc::CameraServer::GetVideo();

    cs::CvSource outputStream = frc::CameraServer::PutVideo("Feed1", 640, 480);

 
    cv::Mat mat;

    while (true) {
      // Tell the CvSink to grab a frame from the camera and put it
      // in the source mat.  If there is an error notify the output.
      if (cvSink.GrabFrame(mat) == 0) {
        // Send the output the error.
        outputStream.NotifyError(cvSink.GetError());
        // skip the rest of the current iteration
        continue;
      }
      // Put a rectangle on the image
      //rectangle(mat, cv::Point(100, 100), cv::Point(400, 400), cv::Scalar(255, 255, 255), 5);
      // Give the output stream a new image to display
      outputStream.PutFrame(mat);
    }
  }*/

#ifndef RUNNING_FRC_TESTS
int main() {
  return frc::StartRobot<Robot>();
}
#endif
