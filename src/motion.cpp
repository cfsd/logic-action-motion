/*
 * Copyright (C) 2018  Christian Berger
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "cluon-complete.hpp"
#include "opendlv-standard-message-set.hpp"

#include "logic-motion.hpp"

#include <cstdint>
#include <iostream>
#include <sstream>
#include <string>
#include <thread>
#include <cmath>
#include <ctime>
#include <chrono>

int32_t main(int32_t argc, char **argv) {
    int32_t retCode{0};
    bool readyState = 0;
    auto commandlineArguments = cluon::getCommandlineArguments(argc, argv);
    if ((0 == commandlineArguments.count("cid")) || (0 == commandlineArguments.count("verbose"))) {
        std::cout << argv[0] << " not enought input arguments." << std::endl;
        std::cout << "Usage:   " << argv[0] << " --cid=<OpenDaVINCI session> [--id=<Identifier in case of multiple beaglebone units>] [--verbose]" << std::endl;
        std::cout << "Default: " << argv[0] << " --cid=111 --cidSM=219 --constantSpeed=0 --id=1 --verbose=1 --speedRequest=2" << std::endl;
    }

      const uint32_t ID{(commandlineArguments["id"].size() != 0) ? static_cast<uint32_t>(std::stoi(commandlineArguments["id"])) : 1};
      const bool VERBOSE{(commandlineArguments["verbose"].size() != 0) ? commandlineArguments.count("verbose") != 0 : 1};
      const uint16_t cid{(commandlineArguments["cid"].size() != 0) ? static_cast<uint16_t>(std::stoi(commandlineArguments["cid"])) : (uint16_t) 111};
      const uint16_t cidSM{(commandlineArguments["cidSM"].size() != 0) ? static_cast<uint16_t>(std::stoi(commandlineArguments["cidSM"])) : (uint16_t) 219};

      //const float FREQ{std::stof(commandlineArguments["freq"])};
      std::cout << "Micro-Service ID:" << ID << std::endl;

      // Interface to a running OpenDaVINCI session.

      cluon::OD4Session od4{cid};
      cluon::OD4Session od4StateMachine{cidSM};

      Motion motion(VERBOSE, ID, od4StateMachine);

      using namespace std::literals::chrono_literals;

      auto catchState{[&readyState, &od4StateMachine, &motion](cluon::data::Envelope &&envelope)
      {
        auto ASstate = cluon::extractMessage<opendlv::proxy::SwitchStateReading>(std::move(envelope));

        if (envelope.senderStamp() == 1401 && readyState == 0 && (ASstate.state() == 2) ){
          motion.nextContainer(envelope);
          std::this_thread::sleep_for(5.0s);
          for (int i=0; i < 20; i++) {
            std::this_thread::sleep_for(0.1s);

            opendlv::proxy::TorqueRequest torqueRequest;
            cluon::data::TimeStamp sampleTime = cluon::time::now();

            torqueRequest.torque(i*37.5);
            od4StateMachine.send(torqueRequest,sampleTime,1502);
            od4StateMachine.send(torqueRequest,sampleTime,1503);
          }

        readyState = (ASstate.state() == 2 ? 1 : 0);
        }
      }};

      od4.dataTrigger(opendlv::proxy::SwitchStateReading::ID(), catchState);

      auto catchContainer{[&motion, &readyState](cluon::data::Envelope &&envelope)
      {
        if(readyState){
          motion.nextContainer(envelope);
        }
      }};

      od4.dataTrigger(opendlv::proxy::GroundAccelerationRequest::ID(), catchContainer);
      od4.dataTrigger(opendlv::proxy::GroundDecelerationRequest::ID(), catchContainer);
      od4.dataTrigger(opendlv::sim::KinematicState::ID(), catchContainer);
      od4.dataTrigger(opendlv::logic::action::AimPoint::ID(), catchContainer);
      od4StateMachine.dataTrigger(opendlv::proxy::GroundSpeedReading::ID(), catchContainer);


      // Just sleep as this microservice is data driven.
      while (od4.isRunning()) {
        std::this_thread::sleep_for(0.05s);
        opendlv::system::SignalStatusMessage heartBeat;
        heartBeat.code(1);

        std::chrono::system_clock::time_point tp = std::chrono::system_clock::now();
        cluon::data::TimeStamp sampleTime = cluon::time::convert(tp);

        od4.send(heartBeat,sampleTime,313);
      }

  return retCode;
}
