🤖 TITUS WROVER
ESP32-Based Line Follower Robot with Real-Time PID Web Tuning


📌 Project Overview

TITUS WROVER is an autonomous line-following robot built using an ESP32 WROVER module.
The robot follows a black line using an 8-channel IR sensor array and a PID control algorithm to achieve smooth and stable motion.

A key highlight of this project is a Wi-Fi-based web dashboard that allows real-time tuning of PID parameters and motor speed without reflashing the firmware.
This project demonstrates the practical integration of embedded systems, control theory, robotics, and IoT.

🚀 Features

Autonomous black line following

PID-based motor control

Real-time PID tuning via Wi-Fi

ESP32 running as a Wi-Fi Access Point

Mobile-friendly web dashboard

Start and Stop motor control from browser

Automatic line-loss detection and recovery

Differential drive control using PWM

🌐 Web Application Explanation

The ESP32 hosts a lightweight web application that can be accessed through a browser.

🔹 How It Works

ESP32 creates a Wi-Fi Access Point

User connects using a mobile phone or laptop

A web dashboard allows live parameter tuning

🔹 Web Dashboard Controls

Base Speed – Sets the nominal motor speed

Kp (Proportional Gain) – Controls immediate response to error

Ki (Integral Gain) – Reduces steady-state error

Kd (Derivative Gain) – Stabilizes motion and reduces oscillation

START Button – Enables robot motion and resets PID states

STOP Button – Immediately stops motors for safety

Changes take effect instantly, enabling real-time PID optimization.

⚙️ System Architecture

The system follows a closed-loop control structure:

IR sensor array detects the position of the line

ESP32 computes the position error

PID controller calculates correction value

Motor driver adjusts left and right motor speeds

Robot continuously corrects its path

This architecture ensures smooth tracking and stability even at higher speeds.

🧩 Hardware Used

ESP32 WROVER Module

SmartElex RLS-08 Line Sensor Array

L298N Dual H-Bridge Motor Driver

N20 6V 60 RPM Micro Metal Gear DC Motors (2 Nos.)

43 mm Rubber Wheel Tyres

Ball Caster Wheel

2200 mAh Battery Pack

XT60 Connector

3D Printed Chassis

Jumper Wires and Perfboard

🛠 Skills Demonstrated
Embedded Systems

ESP32 GPIO and PWM (LEDC)

Real-time control loops

Wi-Fi Access Point configuration

Control Systems

PID controller design and tuning

Error modeling and scaling

Stability vs responsiveness optimization

Robotics

Differential drive control

Sensor placement and alignment

Line-loss recovery logic

Software & Web

Async web server on ESP32

HTTP-based parameter control

Embedded UI design

Hardware

PCB soldering and wiring

Power distribution and current handling

Mechanical assembly and balance

🏁 Outcome

Achieved smooth and stable line tracking

Enabled fast PID tuning during real-world testing

Improved reliability through automatic line-loss recovery

Successfully integrated hardware, software, and web control into one system

🔮 Future Scope

The project can be extended by integrating a Raspberry Pi 5 to run ROS, enabling higher-level control and modular software architecture. The robot can be connected to a laptop for development and debugging, along with a USB or CSI webcam to implement computer vision capabilities. Using ArUco marker–based visual detection, the system can learn visual localization and perception, paving the way for vision-guided navigation and advanced autonomous behaviors.

