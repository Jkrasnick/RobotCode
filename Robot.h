// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <string>
#include <fmt/core.h>


#include <frc/TimedRobot.h>

#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/smartdashboard/SendableChooser.h>
#include <frc/shuffleboard/Shuffleboard.h>

// Simple Camera
#include <cameraServer/CameraServer.h>
// Advanced Camera
//#include <opencv2/core/core.hpp>
//#include <opencv2/core/types.hpp>
//#include <opencv2/imgproc/imgproc.hpp>


#include <frc/Timer.h>
#include <frc/XboxController.h>
#include <frc/Compressor.h>
#include <frc/DoubleSolenoid.h>

#include <rev/CANSparkMax.h>

// Limit Switches
#include <frc/DigitalInput.h>

// Dashboard
#include <networktables/NetworkTableEntry.h>


class Robot : public frc::TimedRobot {
 public:
  void RobotInit() override;
  void RobotPeriodic() override;
  void AutonomousInit() override;
  void AutonomousPeriodic() override;
  void TeleopInit() override;
  void TeleopPeriodic() override;
  void DisabledInit() override;
  void DisabledPeriodic() override;
  void TestInit() override;
  void TestPeriodic() override;
  void SimulationInit() override;
  void SimulationPeriodic() override;

 private:

  frc::SendableChooser<std::string> sd_chooser;
  const std::string kAutoNameDefault = "Stationary";
  const std::string kAutoName1 = "Mode 1";
  const std::string kAutoName2 = "Mode 2";
  std::string sd_autoSelected;

  frc::XboxController c_driver{0};
  frc::XboxController c_operator{1};

  // Drive Motors
  rev::CANSparkMax m_backRight{1, rev::CANSparkMax::MotorType::kBrushless};
  rev::CANSparkMax m_frontRight{2, rev::CANSparkMax::MotorType::kBrushless};

  rev::CANSparkMax m_frontLeft{6, rev::CANSparkMax::MotorType::kBrushless};
  rev::CANSparkMax m_backLeft{7, rev::CANSparkMax::MotorType::kBrushless};

  // Arm Motors
  rev::CANSparkMax m_shoulder{4, rev::CANSparkMax::MotorType::kBrushless};
  rev::CANSparkMax m_elbow{5, rev::CANSparkMax::MotorType::kBrushless};
  rev::CANSparkMax m_wrist{3, rev::CANSparkMax::MotorType::kBrushless};

  // Arm Encoders
  rev::SparkMaxRelativeEncoder e_shoulder = m_shoulder.GetEncoder();
  rev::SparkMaxRelativeEncoder e_elbow = m_elbow.GetEncoder();
  //rev::SparkMaxRelativeEncoder e_wrist = m_wrist.GetEncoder();

  // Arm Limits
  frc::DigitalInput ls_stage1Lower{0}; // Normally Open
  frc::DigitalInput ls_stage1Upper{1}; // Normally Open
  frc::DigitalInput ls_stage2Lower{2};
  frc::DigitalInput ls_stage2Upper{3};


  // Compressor & Solenoids
  frc::Compressor compressor{1,frc::PneumaticsModuleType::REVPH};
  frc::DoubleSolenoid s_brakeRight{frc::PneumaticsModuleType::REVPH, 0, 1};
  frc::DoubleSolenoid s_brakeLeft{frc::PneumaticsModuleType::REVPH, 2, 3};
  frc::DoubleSolenoid s_claw{frc::PneumaticsModuleType::REVPH, 8, 9};

  double speed_shoulder = 0;
  double speed_elbow = 0;
  double speed_wrist = 0;

  double shoulderScale = .5;
  double elbowScaleUp = .75;
  double elbowScaleDown = .3;
  double wristScale = .4;

  double scale_turn = .2;
  double scale_strafe = .75;
  double scale_control = .5;

  double move_r = 0;
  double move_x = 0;
  double move_y = 0;

  double speed_frontRight = 0;
  double speed_backRight = 0;
  double speed_frontLeft = 0;
  double speed_backLeft = 0;

  double deadzone_x = .2;
  double deadzone_y = .2;
  double deadzone_r = .2;

  // Dashboard Stuff
  bool brakeOn = false;
  bool clawGrab = false;

  //bool compressorDisabled = false;
  bool compressorState = true;

  // Custom Functions
  double Zero(double input, double range);
};
