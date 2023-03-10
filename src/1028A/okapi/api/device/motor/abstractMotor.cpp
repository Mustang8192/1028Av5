/*
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */
#include "1028A/okapi/api/device/motor/abstractMotor.hpp"

namespace okapi {
AbstractMotor::~AbstractMotor() = default;

AbstractMotor::GearsetRatioPair operator*(const AbstractMotor::gearset gearset,
                                          const double ratio) {
  return AbstractMotor::GearsetRatioPair(gearset, ratio);
}

double AbstractMotor::getPositionError() {
  return getTargetPosition() - getPosition();
}

double AbstractMotor::getVelocityError() {
  return getTargetVelocity() - getActualVelocity();
}
} // namespace okapi
