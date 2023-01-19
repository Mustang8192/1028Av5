/*
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */
#pragma once

/** @mainpage OkapiLib Index Page
 *
 * @section intro_sec Introduction
 *
 * **OkapiLib** is a PROS library for programming VEX V5 robots. This library is intended to raise
 * the floor for teams with all levels of experience. New teams should have an easier time getting
 * their robot up and running, and veteran teams should find that OkapiLib doesn't get in the way or
 * place any limits on functionality.
 *
 * For tutorials on how to get the most out of OkapiLib, see the
 * [Tutorials](docs/tutorials/index.md) section. For documentation on using the OkapiLib API, see
 * the [API](docs/api/index.md) section.
 *
 * @section getting_started Getting Started
 * Not sure where to start? Take a look at the
 * [Getting Started](docs/tutorials/walkthrough/gettingStarted.md) tutorial.
 * Once you have OkapiLib set up, check out the
 * [Clawbot](docs/tutorials/walkthrough/clawbot.md) tutorial.
 *
 * @section using_docs Using The Documentation
 *
 * Start with reading the [Tutorials](docs/tutorials/index.md). Use the [API](docs/api/index.md)
 * section to explore the class hierarchy. To see a list of all available classes, use the
 * [Classes](annotated.html) section.
 *
 * This documentation has a powerful search feature, which can be brought up with the keyboard
 * shortcuts `Tab` or `T`. All exports to the `1028A/okapi` namespace such as enums, constants, units, or
 * functions can be found [here](@ref 1028A/okapi).
 */

#include "1028A/okapi/api/chassis/controller/chassisControllerIntegrated.hpp"
#include "1028A/okapi/api/chassis/controller/chassisControllerPid.hpp"
#include "1028A/okapi/api/chassis/controller/chassisScales.hpp"
#include "1028A/okapi/api/chassis/controller/defaultOdomChassisController.hpp"
#include "1028A/okapi/api/chassis/controller/odomChassisController.hpp"
#include "1028A/okapi/api/chassis/model/hDriveModel.hpp"
#include "1028A/okapi/api/chassis/model/readOnlyChassisModel.hpp"
#include "1028A/okapi/api/chassis/model/skidSteerModel.hpp"
#include "1028A/okapi/api/chassis/model/threeEncoderSkidSteerModel.hpp"
#include "1028A/okapi/api/chassis/model/threeEncoderXDriveModel.hpp"
#include "1028A/okapi/api/chassis/model/xDriveModel.hpp"
#include "1028A/okapi/impl/chassis/controller/chassisControllerBuilder.hpp"

#include "1028A/okapi/api/control/async/asyncLinearMotionProfileController.hpp"
#include "1028A/okapi/api/control/async/asyncMotionProfileController.hpp"
#include "1028A/okapi/api/control/async/asyncPosIntegratedController.hpp"
#include "1028A/okapi/api/control/async/asyncPosPidController.hpp"
#include "1028A/okapi/api/control/async/asyncVelIntegratedController.hpp"
#include "1028A/okapi/api/control/async/asyncVelPidController.hpp"
#include "1028A/okapi/api/control/async/asyncWrapper.hpp"
#include "1028A/okapi/api/control/controllerInput.hpp"
#include "1028A/okapi/api/control/controllerOutput.hpp"
#include "1028A/okapi/api/control/iterative/iterativeMotorVelocityController.hpp"
#include "1028A/okapi/api/control/iterative/iterativePosPidController.hpp"
#include "1028A/okapi/api/control/iterative/iterativeVelPidController.hpp"
#include "1028A/okapi/api/control/util/controllerRunner.hpp"
#include "1028A/okapi/api/control/util/flywheelSimulator.hpp"
#include "1028A/okapi/api/control/util/pidTuner.hpp"
#include "1028A/okapi/api/control/util/settledUtil.hpp"
#include "1028A/okapi/impl/control/async/asyncMotionProfileControllerBuilder.hpp"
#include "1028A/okapi/impl/control/async/asyncPosControllerBuilder.hpp"
#include "1028A/okapi/impl/control/async/asyncVelControllerBuilder.hpp"
#include "1028A/okapi/impl/control/iterative/iterativeControllerFactory.hpp"
#include "1028A/okapi/impl/control/util/controllerRunnerFactory.hpp"
#include "1028A/okapi/impl/control/util/pidTunerFactory.hpp"

#include "1028A/okapi/api/odometry/odomMath.hpp"
#include "1028A/okapi/api/odometry/odometry.hpp"
#include "1028A/okapi/api/odometry/threeEncoderOdometry.hpp"

#include "1028A/okapi/api/device/rotarysensor/continuousRotarySensor.hpp"
#include "1028A/okapi/api/device/rotarysensor/rotarySensor.hpp"
#include "1028A/okapi/impl/device/adiUltrasonic.hpp"
#include "1028A/okapi/impl/device/button/adiButton.hpp"
#include "1028A/okapi/impl/device/button/controllerButton.hpp"
#include "1028A/okapi/impl/device/controller.hpp"
#include "1028A/okapi/impl/device/distanceSensor.hpp"
#include "1028A/okapi/impl/device/motor/adiMotor.hpp"
#include "1028A/okapi/impl/device/motor/motor.hpp"
#include "1028A/okapi/impl/device/motor/motorGroup.hpp"
#include "1028A/okapi/impl/device/opticalSensor.hpp"
#include "1028A/okapi/impl/device/rotarysensor/IMU.hpp"
#include "1028A/okapi/impl/device/rotarysensor/adiEncoder.hpp"
#include "1028A/okapi/impl/device/rotarysensor/adiGyro.hpp"
#include "1028A/okapi/impl/device/rotarysensor/integratedEncoder.hpp"
#include "1028A/okapi/impl/device/rotarysensor/potentiometer.hpp"
#include "1028A/okapi/impl/device/rotarysensor/rotationSensor.hpp"

#include "1028A/okapi/api/filter/averageFilter.hpp"
#include "1028A/okapi/api/filter/composableFilter.hpp"
#include "1028A/okapi/api/filter/demaFilter.hpp"
#include "1028A/okapi/api/filter/ekfFilter.hpp"
#include "1028A/okapi/api/filter/emaFilter.hpp"
#include "1028A/okapi/api/filter/filter.hpp"
#include "1028A/okapi/api/filter/filteredControllerInput.hpp"
#include "1028A/okapi/api/filter/medianFilter.hpp"
#include "1028A/okapi/api/filter/passthroughFilter.hpp"
#include "1028A/okapi/api/filter/velMath.hpp"
#include "1028A/okapi/impl/filter/velMathFactory.hpp"

#include "1028A/okapi/api/units/QAcceleration.hpp"
#include "1028A/okapi/api/units/QAngle.hpp"
#include "1028A/okapi/api/units/QAngularAcceleration.hpp"
#include "1028A/okapi/api/units/QAngularJerk.hpp"
#include "1028A/okapi/api/units/QAngularSpeed.hpp"
#include "1028A/okapi/api/units/QArea.hpp"
#include "1028A/okapi/api/units/QForce.hpp"
#include "1028A/okapi/api/units/QFrequency.hpp"
#include "1028A/okapi/api/units/QJerk.hpp"
#include "1028A/okapi/api/units/QLength.hpp"
#include "1028A/okapi/api/units/QMass.hpp"
#include "1028A/okapi/api/units/QPressure.hpp"
#include "1028A/okapi/api/units/QSpeed.hpp"
#include "1028A/okapi/api/units/QTime.hpp"
#include "1028A/okapi/api/units/QTorque.hpp"
#include "1028A/okapi/api/units/QVolume.hpp"
#include "1028A/okapi/api/units/RQuantityName.hpp"

#include "1028A/okapi/api/util/abstractRate.hpp"
#include "1028A/okapi/api/util/abstractTimer.hpp"
#include "1028A/okapi/api/util/mathUtil.hpp"
#include "1028A/okapi/api/util/supplier.hpp"
#include "1028A/okapi/api/util/timeUtil.hpp"
#include "1028A/okapi/impl/util/configurableTimeUtilFactory.hpp"
#include "1028A/okapi/impl/util/rate.hpp"
#include "1028A/okapi/impl/util/timeUtilFactory.hpp"
#include "1028A/okapi/impl/util/timer.hpp"
