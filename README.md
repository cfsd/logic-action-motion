## OpenDLV Microservice for Beaglebone

This repository provides source code for beaglebones for the OpenDLV.io software ecosystem.

[![Build Status](https://travis-ci.org/cfsd/logic-action-motion.svg?branch=pre-release)](https://travis-ci.org/cfsd/logic-action-motion)


## Table of Contents
* [Dependencies](#dependencies)
* [Usage](#usage)
* [Build from sources on the example of Ubuntu 16.04 LTS](#build-from-sources-on-the-example-of-ubuntu-1604-lts)
* [License](#license)


## Dependencies
No dependencies! You just need a C++14-compliant compiler to compile this project as it ships the following dependencies as part of the source distribution:

* [libcluon](https://github.com/chrberger/libcluon) - [![License: GPLv3](https://img.shields.io/badge/license-GPL--3-blue.svg
)](https://www.gnu.org/licenses/gpl-3.0.txt)
* [Unit Test Framework Catch2](https://github.com/catchorg/Catch2/releases/tag/v2.1.1) - [![License: Boost Software License v1.0](https://img.shields.io/badge/License-Boost%20v1-blue.svg)](http://www.boost.org/LICENSE_1_0.txt)


## Usage
This microservice is created automatically on changes to this repository via Docker's public registry for:
* [x86_64](https://hub.docker.com/r/chalmersrevere/opendlv.io-amd64/tags/)
* [armhf](https://hub.docker.com/r/chalmersrevere/opendlv.io-armhf/tags/)
* [aarch64](https://hub.docker.com/r/chalmersrevere/opendlv.io-aarch64/tags/)

To run this microservice using our pre-built Docker multi-arch images to connect to an OXTS GPS/INSS unit broadcasting data to `195.0.0.33:3000` and to publish the messages according to OpenDLV Standard Message Set into session 111 in Google Protobuf format, simply start it as follows:
```
docker run --rm --net=host chalmersrevere/opendlv.io-multi:proxy-beaglebone-v0.0.1 beaglebone --cid=111 --verbose
```

## Build from sources on the example of Ubuntu 16.04 LTS
To build this software, you need cmake, C++14 or newer, and make. Having these
preconditions, just run `cmake` and `make` as follows:

```
mkdir build && cd build
cmake -D CMAKE_BUILD_TYPE=Release ..
make && make test && make install
```

## Build it with docker
Make sure you have the latest docker version. 1.17

AMD64:
Run
```
docker build -t chalmersrevere/opendlv.io-multi:proxy-beaglebone-v0.0.1 -f Dockerfile.amd64 .
```

ARMHF:
Run
```
docker build -t chalmersrevere/opendlv.io-multi-armhf:proxy-beaglebone-v0.0.1 -f Dockerfile.armhf .
```

## Execute with Docker-compose
Make sure you have the latest docker-compose verison.

AMD64:
Run
```
cd usecase
docker-compose up
```

ARMHF:
Run
```
cd usecase
docker-compose -f beaglebone.yml up
```


## License

* This project is released under the terms of the GNU GPLv3 License

# Details

This service is responsible for calculating the appropriate torque request values to send to the inverters, given some reference values calculated by the cognition services. Currently only uses GroundAccelerationRequest and GroundDecelerationRequest.

## Messages

### **Ingoing**

- GroundAccelerationRequest *(From cognition-track or cognition-acceleration)*
- GroundDecelerationRequest *(From cognition-track or cognition-acceleration)*
- KinematicState *(NOT USED)*
- AimPoint *(NOT USED, MEANT FOR TORQUE VECTORING?)*
- GroundSpeedReading *(From WHERE?)*

**Outgoing**

- x2 TorqueRequest *(One for left and one for right inverter)*

## logic-action-x

In our car, we have three main components that we can control to make the car move: The linear actuator (steering), the brakes (braking) and the inverters (motion).

The logic-action-x services are responsible for calculating an appropriate control signal for their respective components given some reference signal(s) sent from the main cognition service (cognition-acceleration or cognition-track).

There are two different OD4Sessions in each service: One that is responsible for receiving messages from cognition-services, and one that is responsible for sending messages to low-level services.

**To improve**: We believe that the solution to have three independent microservices responsible for generating the control signals for each of the control components (steering, inverters, brakes) is restrictive. As it is now, it's hard to implement a control scheme where different control signals are dependent by design.

**To improve**: The naming of the OD4Session which communicates to the low-level stuff (StateMachine, aka. BeagleBone) is very inconsistent between the logic-action services. Basically, od4_proxy (steering), od4Brakes (braking) and od4StateMachine (motion) all live on the same conference with defaullt CID=219.

## Comments

- This service can be used in two modes: ConstantSpeed on/off. If the commandline argument constSpeed==1, the service will simply send TorqueRequests corresponding to acceleration=(speedRequest-m_groundSpeedReading) where speedRequest is a fixed speed that is set as a command line argument, and m_groundSpeedReading is the current speed reading.
- Difference between GroundAccelerationRequest and GroundDecelerationRequest? It seems that they are interchangeable the way that they're used.
- We haven't found out which service that uses the TorqueRequests sent by this service.
