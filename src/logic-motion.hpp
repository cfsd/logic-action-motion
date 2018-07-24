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

 #ifndef MOTION
 #define MOTION

 #include "opendlv-standard-message-set.hpp"


class Motion {
 public:
  Motion(bool verbose, uint32_t id, cluon::OD4Session &od4);
  ~Motion();
  void nextContainer(cluon::data::Envelope &);
  void setSpeedRequest(float);

 private:
  void setUp();
  void tearDown();
  void calcTorque(float);
  void sendActuationContainer(int32_t, float);
  float calcYawRateRef(opendlv::logic::action::AimPoint);

 private:
   cluon::OD4Session &m_od4;
   opendlv::logic::action::AimPoint m_aimPoint;
   float m_groundSpeedReading;
   float m_groundSpeedReadingLeft;
   float m_groundSpeedReadingRight;
   int32_t m_readyState;
};
#endif
