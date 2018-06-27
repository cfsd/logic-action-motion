/**
* Copyright (C) 2017 Chalmers Revere
*
* This program is free software; you can redistribute it and/or
* modify it under the terms of the GNU General Public License
* as published by the Free Software Foundation; either version 2
* of the License, or (at your option) any later version.
*
* This program is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with this program; if not, write to the Free Software
* Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301,
* USA.
*/

#include <iostream>
#include "cluon-complete.hpp"
#include "logic-motion.hpp"

#define PI 3.14159265359f



Motion::Motion(bool verbose, uint32_t id, cluon::OD4Session &od4)
  : m_od4(od4)
  , m_aimPoint()
  , m_speed()
{
  setUp();
  (void)verbose;
  (void)od4;
  (void)id;
}

Motion::~Motion()
{
  tearDown();
}


void Motion::nextContainer(cluon::data::Envelope &a_container)
{
  (void) a_container;
  if (a_container.dataType() == opendlv::proxy::GroundAccelerationRequest::ID()) {
    auto accelerationRequest = cluon::extractMessage<opendlv::proxy::GroundAccelerationRequest>(std::move(a_container));
    float acceleration = accelerationRequest.groundAcceleration();

    calcTorque(acceleration);
  }

  if (a_container.dataType() == opendlv::proxy::GroundDecelerationRequest::ID()) {
    auto decelerationRequest = cluon::extractMessage<opendlv::proxy::GroundDecelerationRequest>(std::move(a_container));
    float deceleration = decelerationRequest.groundDeceleration();

    if (m_speed > float(5/3.6)){
      calcTorque(deceleration);
    }
  }

  if (a_container.dataType() == opendlv::sim::KinematicState::ID()) { // change this to whatever container marcus sends out
    auto vehicleSpeed = cluon::extractMessage<opendlv::sim::KinematicState>(std::move(a_container));
    m_speed = vehicleSpeed.vx();
  }

  if (a_container.dataType() == opendlv::logic::action::AimPoint::ID()) {
    m_aimPoint = cluon::extractMessage<opendlv::logic::action::AimPoint>(std::move(a_container));
  }
}

void Motion::setUp()
{
  // std::string const exampleConfig =
  std::cout << "Setting up motion" << std::endl;
}

void Motion::tearDown()
{
}

void Motion::calcTorque(float a_arg)
{
  uint32_t leftMotorID = 1502;
  uint32_t rightMotorID = 1503;
  float dT = 0.5;


  float mass = 217.4f;
  float wheelRadius = 0.22f;
  float torque = a_arg*mass*wheelRadius;
  float Iz = 133.32f;

  float yawRateRef = calcYawRateRef(m_aimPoint);

  float e_yawRate = -yawRateRef; // Add yaw rate here when Marcus is done with message
  (void) (e_yawRate*dT);
  float dTorque = Iz*0;

  // Torque distribution
  float torqueLeft = torque*0.5f - dTorque;
  float torqueRight = torque-torqueLeft;

  sendActuationContainer(leftMotorID,torqueLeft);
  sendActuationContainer(rightMotorID,torqueRight);
}

float Motion::calcYawRateRef(opendlv::logic::action::AimPoint aimPoint)
{
  float headingReq = aimPoint.azimuthAngle();  // Angle to the aim point
  float dist  = aimPoint.distance();           // Distance to the aim point
  float u = m_speed;
  float R = dist/(float)(sqrt(2.0f*(1.0f-(float)cos(2.0f*headingReq))));
  // Calculate the average yaw rate to turn for that specific curve
  float r = std::copysign(u/R,headingReq);

  return r;
}

void Motion::sendActuationContainer(int32_t a_arg, float torque)
{
  auto senderStamp = a_arg;
  opendlv::proxy::TorqueRequest torqueRequest;
  torqueRequest.torque(torque);

  std::chrono::system_clock::time_point tp = std::chrono::system_clock::now();
  cluon::data::TimeStamp sampleTime = cluon::time::convert(tp);

  m_od4.send(torqueRequest,sampleTime,senderStamp);
}
