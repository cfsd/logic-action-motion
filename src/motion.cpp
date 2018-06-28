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
      const bool constSpeed{(commandlineArguments["constantSpeed"].size() != 0) ? commandlineArguments.count("constantSpeed") : 0};
      const float speedRequest{(commandlineArguments["speedRequest"].size() != 0) ? std::stof(commandlineArguments["speedRequest"]) : 2.0f};

      //const float FREQ{std::stof(commandlineArguments["freq"])};
      std::cout << "Micro-Service ID:" << ID << std::endl;

      // Interface to a running OpenDaVINCI session.

      cluon::OD4Session od4{cid};
      cluon::OD4Session od4StateMachine{cidSM};

      Motion motion(VERBOSE, ID, od4StateMachine);

     auto catchContainer{[&motion](cluon::data::Envelope &&envelope)
      {
        // if (!motion.getInitialised()){
        //     return;
        // }
        motion.nextContainer(envelope);
      }};

      if(constSpeed==0){
        od4.dataTrigger(opendlv::proxy::GroundAccelerationRequest::ID(), catchContainer);
        od4.dataTrigger(opendlv::proxy::GroundDecelerationRequest::ID(), catchContainer);
      }
      od4.dataTrigger(opendlv::sim::KinematicState::ID(), catchContainer);
      od4.dataTrigger(opendlv::logic::action::AimPoint::ID(), catchContainer);
      od4.dataTrigger(opendlv::proxy::GroundSpeedReading::ID(), catchContainer);


      // Just sleep as this microservice is data driven.
      using namespace std::literals::chrono_literals;
      while (od4.isRunning()) {
        std::this_thread::sleep_for(0.05s);
        opendlv::system::SignalStatusMessage heartBeat;
        heartBeat.code(1);

        std::chrono::system_clock::time_point tp = std::chrono::system_clock::now();
        cluon::data::TimeStamp sampleTime = cluon::time::convert(tp);

        od4.send(heartBeat,sampleTime,313);

        if(constSpeed==1){
          motion.setSpeedRequest(speedRequest);
        }
      }

  return retCode;
}
