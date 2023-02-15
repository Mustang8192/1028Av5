#include "1028A/gterm.h"
#include "1028A/init.h"
#include "1028A/log.h"
#include "1028A/robot.h"
#include "1028A/vars.h"
#include "pros/misc.hpp"
#include "pros/rtos.hpp"

typedef FILE *pc_file_t;
static lv_fs_res_t _1028A::ui::pcfs_open(void *file_p, const char *fn,
                                         lv_fs_mode_t mode) {
  errno = 0;
  const char *flags = "";
  if (mode == LV_FS_MODE_WR)
    flags = "wb";
  else if (mode == LV_FS_MODE_RD)
    flags = "rb";
  else if (mode == (LV_FS_MODE_WR | LV_FS_MODE_RD))
    flags = "a+";

  char buf[256];
  sprintf(buf, "/%s", fn);
  pc_file_t f = fopen(buf, flags);

  if (f == NULL)
    return LV_FS_RES_UNKNOWN;
  else {
    fseek(f, 0, SEEK_SET);
    pc_file_t *fp = (pc_file_t *)file_p;
    *fp = f;
  }

  return LV_FS_RES_OK;
}

static lv_fs_res_t _1028A::ui::pcfs_close(void *file_p) {
  pc_file_t *fp = (pc_file_t *)file_p;
  fclose(*fp);
  return LV_FS_RES_OK;
}

static lv_fs_res_t _1028A::ui::pcfs_read(void *file_p, void *buf, uint32_t btr,
                                         uint32_t *br) {
  pc_file_t *fp = (pc_file_t *)file_p;
  *br = fread(buf, 1, btr, *fp);
  return LV_FS_RES_OK;
}

static lv_fs_res_t _1028A::ui::pcfs_seek(void *file_p, uint32_t pos) {
  pc_file_t *fp = (pc_file_t *)file_p;
  fseek(*fp, pos, SEEK_SET);
  return LV_FS_RES_OK;
}

static lv_fs_res_t _1028A::ui::pcfs_tell(void *file_p, uint32_t *pos_p) {
  pc_file_t *fp = (pc_file_t *)file_p;
  *pos_p = ftell(*fp);
  return LV_FS_RES_OK;
}

/**
 * @brief
 *  This function is a callback function for auton selector
 * @param btn
 * @return lv_res_t
 */

lv_res_t _1028A::ui::left_sideBtnAction(lv_obj_t *btn) {
  autonSelect = 1;
  return LV_RES_OK;
}

/**
 * @brief
 *  This function is a callback function for auton selector
 * @param btn
 * @return lv_res_t
 */

lv_res_t _1028A::ui::right_sideBtnAction(lv_obj_t *btn) {
  autonSelect = 2;
  return LV_RES_OK;
}

/**
 * @brief
 *  This function is a callback function for auton selector
 * @param btn
 * @return lv_res_t
 */

lv_res_t _1028A::ui::do_nothingBtnAction(lv_obj_t *btn) {
  autonSelect = 0;
  return LV_RES_OK;
}

/**
 * @brief
 *  This function is a callback function for auton selector
 * @param btn
 * @return lv_res_t
 */

lv_res_t _1028A::ui::soloWPBtnAction(lv_obj_t *btn) {
  autonSelect = 3;
  return LV_RES_OK;
}

/**
 * @brief
 *  This function is a callback function for auton selector
 * @param btn
 * @return lv_res_t
 */

lv_res_t _1028A::ui::skillsBtnAction(lv_obj_t *btn) {
  autonSelect = 8;
  return LV_RES_OK;
}

/**
 * @brief
 *  This function is a callback function for auton selector
 * @param btn
 * @return lv_res_t
 */

lv_res_t _1028A::ui::Flywheelbeastmode(lv_obj_t *btn) {
  flywheelMode = 2;
  return LV_RES_OK;
}

/**
 * @brief
 *  This function is a callback function to start the grafana task
 * @param btn
 * @return lv_res_t
 */

lv_res_t _1028A::ui::GrafanaAction(lv_obj_t *btn) {
  task::start("grafana", Grafana::grafanaInit);
  return LV_RES_OK;
}

/**
 * @brief
 *  This function is a callback function to start the odometry debug task
 * @param btn
 * @return lv_res_t
 */

lv_res_t _1028A::ui::OdomDebugAction(lv_obj_t *btn) {
  lv_obj_clean(lv_scr_act());
  _1028A::OdomDebug Display(lv_scr_act(), LV_COLOR_ORANGE);
  _1028A::task::start("OdomDebug", Display.startOdomDebug);
  return LV_RES_OK;
}

lv_res_t motorTesterAction(lv_obj_t *btn) {
  lv_obj_clean(lv_scr_act());
  /* Find our motor port and sensor port */
  int motor_port = -1;
  int sensor_port = -1;

  for (int i = 0; i < 21; i++) {
    pros::c::v5_device_e_t type = pros::c::registry_get_plugged_type(i);
    LOG_DEBUG("Port %02d has device class %03d", (i + 1), type);

    /* If it's a motor, set it to motor port */
    if (pros::c::E_DEVICE_MOTOR == type) {
      motor_port = i + 1; /* i is zero indexed, need to be 1 indexed */
    } else if (pros::c::E_DEVICE_ROTATION == type) {
      sensor_port = i + 1;
    }
  }

  if (motor_port > 0 && sensor_port > 0) {
    LOG_ALWAYS("Using motor %d and sensor %d", motor_port, sensor_port);
    gterm_print("Using motor %d and sensor %d", motor_port, sensor_port);
  } else if (motor_port > 0) {
    LOG_ERROR(
        "Found motor %d but could not find sensor, please connect and restart",
        motor_port);
    gterm_print("Found motor %d but #ff0000 could not find sensor#",
                motor_port);
    gterm_print("#ff8000 Please connect sensor and restart program#");
  } else if (sensor_port > 0) {
    LOG_ERROR(
        "Found sensor %d but could not find motor, please connect and restart",
        sensor_port);
    gterm_print("Found sensor %d but #ff0000 could not find motor#",
                sensor_port);
    gterm_print("#ff8000 Please connect motor and restart program#");
  } else {
    LOG_ERROR("Could not find motor or sensor, please connect and restart");
    gterm_print("#ff0000 Could not find motor or sensor#");
    gterm_print("#ff8000 Please connect both and restart program#");
  }

  /* Now run self-test programs whenever a motor is connected */
  while (1) {
    /* Wait for motor to be connected */
    bool motor_status;
    do {
      motor_status = (pros::c::registry_get_plugged_type(motor_port - 1) ==
                      pros::c::E_DEVICE_MOTOR);
      LOG_INFO("Waiting for motor to be plugged in");
      pros::delay(1000);
    } while (!motor_status);

    pros::delay(1000);
    LOG_INFO("Running self-test sequence now");
    gterm_print("Running self-test sequence");

    for (int i = 0; i < 2; i++) {
      /* Configure the motor for green, coast */
      pros::c::motor_set_brake_mode(motor_port, pros::E_MOTOR_BRAKE_COAST);
      pros::c::motor_set_gearing(motor_port, pros::E_MOTOR_GEAR_GREEN);
      pros::c::motor_set_zero_position(motor_port, 0.0);
      pros::c::motor_set_encoder_units(motor_port,
                                       pros::E_MOTOR_ENCODER_ROTATIONS);
      pros::c::motor_set_reversed(motor_port, false);

      /* Reset the sensor */
      pros::c::rotation_reset_position(sensor_port);

      /* i=0 is forward, i=1 is reverse */
      if (!i) {
        /* TEST 1 - Run motor forward for 3 seconds, track position counters and
         * disconnect */
        LOG_INFO("TEST 1 - FORWARD performance");
        gterm_print("TEST 1 - #0000ff FORWARD# performance");
        pros::c::motor_move_voltage(motor_port, 12000);
      } else {
        /* TEST 2 - Run motor backward for 3 seconds, track position counters
         * and disconnect */
        LOG_INFO("TEST 2 - REVERSE performance");
        gterm_print("TEST 2 - #0000ff REVERSE# performance");
        pros::c::motor_move_voltage(motor_port, -12000);
      }
      int cnt_poserr = 0;
      double position_track = 0.0;
      double position_sense = 0.0;
      double speed_track = 0.0;
      double speed_sense = 0.0;
      for (int i = 0; i < (3000 / 10); i++) {
        /* Read data from motor and sensor */
        speed_track = pros::c::motor_get_actual_velocity(motor_port);
        speed_sense = (pros::c::rotation_get_velocity(sensor_port) * 1.0) *
                      (1.0 / 360.0) * (60.0);
        position_track = pros::c::motor_get_position(motor_port);
        position_sense = (pros::c::rotation_get_position(sensor_port) * 1.0) *
                         (1.0 / 36000.0);

        /* If sensed position is different from tracked position, report it */
        if (fabs(position_track - position_sense) > 0.2) {
          cnt_poserr++;
        }

        LOG_DEBUG("Measured speed %f, sensed %f", speed_track, speed_sense);
        LOG_DEBUG("Measured position %f, sensed %f", position_track,
                  position_sense);

        pros::delay(10);
      }

      /* Stop the motor after the test */
      pros::c::motor_move(motor_port, 0);

      /* Strings for pass/fail */
      static const char *str_pf[] = {"FAIL", "PASS"};
      static const char *str_pf_col[] = {"#ff0000 FAIL#", "#00ff00 PASS#"};

      LOG_ALWAYS("REPORT:");
      gterm_print("REPORT:");
      /* Can reach 200rpm (green cartridge)? */
      bool speed_reached = (speed_sense > 200.0);
      if (i)
        speed_reached = (speed_sense < -200.0);
      LOG_ALWAYS("Motor reached speed of %f > 200? %s", speed_sense,
                 str_pf[speed_reached]);
      gterm_print("Motor reached speed of %f > 200? %s", speed_sense,
                  str_pf_col[speed_reached]);

      /* No position errors during test */
      bool pos_pass = (cnt_poserr == 0);
      LOG_ALWAYS("Motor tracked position matched? %s", str_pf[pos_pass]);
      gterm_print("Motor tracked position matched? %s", str_pf_col[pos_pass]);
    }

    /* Delay 10 seconds before attempting another test */
    pros::delay(10000);
  }
}
/**
 * @brief
 * This function is a callback function to start Grafana data reporting
 */

void _1028A::Grafana::grafanaInit(void *ptr) {
  Grafana::GUIManager manager;
  manager.setRefreshRate(200);

  Grafana::Variable<pros::Motor> leftFrontMotorVar("Left Front Motor",
                                                   pros::LeftFront);
  Grafana::Variable<pros::Motor> rightFrontMotorVar("Right Front Motor",
                                                    pros::RightFront);
  Grafana::Variable<pros::Motor> leftMidMotorVar("Left Mid Motor",
                                                 pros::LeftMid);
  Grafana::Variable<pros::Motor> rightMidMotorVar("Right Mid Motor",
                                                  pros::RightMid);
  Grafana::Variable<pros::Motor> leftBackMotorVar("Left Back Motor",
                                                  pros::LeftBack);
  Grafana::Variable<pros::Motor> rightBackMotorVar("Right Back Motor",
                                                   pros::RightBack);

  Grafana::VariableGroup<pros::Motor> chassisVars(
      {leftFrontMotorVar, rightFrontMotorVar, leftMidMotorVar, rightMidMotorVar,
       leftBackMotorVar, rightBackMotorVar});

  chassisVars.add_getter("Temperature", &pros::Motor::get_temperature);
  chassisVars.add_getter("Actual Velocity", &pros::Motor::get_actual_velocity);
  chassisVars.add_getter("Voltage", &pros::Motor::get_voltage);
  chassisVars.add_getter("Efficiency", &pros::Motor::get_efficiency);

  manager.registerDataHandler(&chassisVars);

  manager.startTask();

  while (1) {
    pros::delay(200);
  }
}

/**
 * @brief
 *  This function is a callback function to start the flywheel grapher task
 * @param btn
 * @return lv_res_t
 */

lv_res_t _1028A::ui::FlywheelGRCB(lv_obj_t *btn) {
  std::shared_ptr<_1028A::AsyncGrapher> grapher(
      new _1028A::AsyncGrapher("Flywheel Velocity vs. Time"));
  grapher->addDataType("Actual Vel", COLOR_AQUAMARINE);
  grapher->addDataType("Desired Vel", COLOR_ORANGE);
  grapher->startTask();
  while (1) {
    grapher->update("Desired Vel", 1);
    grapher->update("Actual Vel", pros::FlyWheel.get_actual_velocity() / 200);
    pros::delay(10);
  }
  return LV_RES_OK;
}
/**
 * @brief
 *  This function is a callback function to start the intake grapher task
 * @param btn
 * @return lv_res_t
 */

lv_res_t _1028A::ui::IntakeGRCB(lv_obj_t *btn) {
  std::shared_ptr<_1028A::AsyncGrapher> grapher(
      new _1028A::AsyncGrapher("Intake Velocity vs. Time"));
  grapher->addDataType("Desired Vel", COLOR_ORANGE);
  grapher->addDataType("Actual Vel", COLOR_AQUAMARINE);
  grapher->startTask();
  while (1) {
    grapher->update("Desired Vel", 0);
    grapher->update("Actual Vel", pros::Intake.get_actual_velocity() / 200);
    pros::delay(10);
  }
  return LV_RES_OK;
}
/**
 * @brief
 *  This function is a callback function to start the drive grapher task
 * @param btn
 * @return lv_res_t
 */

lv_res_t _1028A::ui::DriveGRCB(lv_obj_t *btn) {
  std::shared_ptr<AsyncGrapher> grapher(
      new AsyncGrapher("Drive Velocity vs. Time"));
  grapher->addDataType("Left Front", COLOR_ORANGE);
  grapher->addDataType("Left Back", COLOR_RED);
  grapher->addDataType("Right Front", COLOR_AQUAMARINE);
  grapher->addDataType("Right Back", COLOR_GREEN);
  grapher->startTask();
  while (1) {
    grapher->update("Left Front",
                    std::abs(pros::LeftFront.get_actual_velocity() / 200));
    grapher->update("Left Back",
                    std::abs(pros::LeftBack.get_actual_velocity() / 200));
    grapher->update("Right Front",
                    std::abs(pros::RightFront.get_actual_velocity() / 200));
    grapher->update("Right Back",
                    std::abs(pros::RightBack.get_actual_velocity() / 200));
    pros::delay(10);
  }
  return LV_RES_OK;
}
/**
 * @brief
 *  quick function to start the battery grapher task
 * @param btn
 * @return lv_res_t
 */
lv_res_t _1028A::ui::BatteryGRCB(lv_obj_t *btn) {
  std::shared_ptr<_1028A::AsyncGrapher> grapher(
      new _1028A::AsyncGrapher("Battery vs. Time"));
  grapher->addDataType("Temperature", COLOR_ORANGE);
  grapher->addDataType("Current", COLOR_RED);
  grapher->addDataType("Voltage", COLOR_AQUAMARINE);
  grapher->addDataType("Capacity", COLOR_GREEN);
  grapher->startTask();
  while (1) {
    grapher->update("Temperature", pros::battery::get_temperature() / 120);
    grapher->update("Current", pros::battery::get_current() / 2000.0);
    grapher->update("Voltage", pros::battery::get_voltage() / 20000.0);
    grapher->update("Capacity", pros::battery::get_capacity() / 100);
    pros::delay(10);
  }
  return LV_RES_OK;
}

/**
 * @brief
 *  This function creates a button
 * @param location
 * @param callback
 * @param alignment
 * @param offsetx
 * @param offsety
 * @param sizeX
 * @param sizeY
 * @param text
 */

void _1028A::ui::createBtn(lv_obj_t *location, lv_res_t callback(lv_obj_t *btn),
                           lv_align_t alignment, int offsetx, int offsety,
                           int sizeX, int sizeY, std::string text) {
  lv_obj_t *Btn = lv_btn_create(location, NULL);
  lv_obj_align(Btn, NULL, alignment, offsetx, offsety);
  lv_obj_set_size(Btn, sizeX, sizeY);
  lv_cb_set_action(Btn, callback);

  lv_obj_t *BtnLbl = lv_label_create(Btn, NULL);
  lv_label_set_text(BtnLbl, text.c_str());
}

void _1028A::ui::uiLockout(void *ptr) {
  int imageDrawn = 0;
  while (1) {
    if (!pros::competition::is_disabled() &&
        !pros::competition::is_autonomous() && autonSelect != 0 &&
        imageDrawn == 1) {
      lv_fs_drv_t pcfs_drv;
      memset(&pcfs_drv, 0, sizeof(lv_fs_drv_t));

      pcfs_drv.file_size = sizeof(pc_file_t);
      pcfs_drv.letter = 'S';
      pcfs_drv.open = pcfs_open;
      pcfs_drv.close = pcfs_close;
      pcfs_drv.read = pcfs_read;
      pcfs_drv.seek = pcfs_seek;
      pcfs_drv.tell = pcfs_tell;
      lv_fs_add_drv(&pcfs_drv);

      lv_obj_t *img_var = lv_img_create(lv_scr_act(), NULL);
      lv_img_set_src(img_var, "S:/usd/BT1.bin");
      lv_obj_set_pos(img_var, 0, 0);
      imageDrawn = 1;
    }
    pros::delay(1000);
  }
}

/**
 * @brief
 *  This function starts the Ui
 * @param hue
 * @param default_auton
 */

void _1028A::ui::init(int hue, bool repeated) {
  // lvgl theme
  lv_theme_t *th = lv_theme_alien_init(hue, NULL);
  lv_theme_set_current(th);

  // create a tab view object
  lv_obj_t *tabview = lv_tabview_create(lv_scr_act(), NULL);

  // add 3 tabs (the tabs are page (lv_page) and can be scrolled
  lv_obj_t *Tab1 = lv_tabview_add_tab(tabview, "Selector 1");
  lv_obj_t *Tab2 = lv_tabview_add_tab(tabview, "Selector 2");
  lv_obj_t *Tab3 = lv_tabview_add_tab(tabview, "Graphs");
  lv_obj_t *Tab4 = lv_tabview_add_tab(tabview, "Misc");

  _1028A::ui::createBtn(Tab1, _1028A::ui::left_sideBtnAction,
                        LV_ALIGN_IN_LEFT_MID, 0, 0, 100, 100, "Left_side");
  _1028A::ui::createBtn(Tab1, _1028A::ui::right_sideBtnAction,
                        LV_ALIGN_IN_LEFT_MID, 120, 0, 100, 100, "Right_side");
  _1028A::ui::createBtn(Tab1, _1028A::ui::soloWPBtnAction, LV_ALIGN_IN_LEFT_MID,
                        240, 0, 100, 100, "Solo_WP");
  _1028A::ui::createBtn(Tab1, _1028A::ui::do_nothingBtnAction,
                        LV_ALIGN_IN_LEFT_MID, 360, 0, 100, 100, "_");
  _1028A::ui::createBtn(Tab2, _1028A::ui::do_nothingBtnAction,
                        LV_ALIGN_IN_LEFT_MID, 0, 0, 100, 100, "_");
  _1028A::ui::createBtn(Tab2, _1028A::ui::do_nothingBtnAction,
                        LV_ALIGN_IN_LEFT_MID, 120, 0, 100, 100, "_");
  _1028A::ui::createBtn(Tab2, _1028A::ui::do_nothingBtnAction,
                        LV_ALIGN_IN_LEFT_MID, 240, 0, 100, 100, "_");
  _1028A::ui::createBtn(Tab2, _1028A::ui::skillsBtnAction, LV_ALIGN_IN_LEFT_MID,
                        360, 0, 100, 100, "Skills");
  _1028A::ui::createBtn(Tab4, _1028A::ui::Flywheelbeastmode,
                        LV_ALIGN_IN_LEFT_MID, 240, 0, 100, 100,
                        "Flywheel \n  Big Boy \n  Mode");
  _1028A::ui::createBtn(Tab3, _1028A::ui::FlywheelGRCB, LV_ALIGN_IN_LEFT_MID, 0,
                        0, 100, 100, "Flywheel");
  _1028A::ui::createBtn(Tab3, _1028A::ui::IntakeGRCB, LV_ALIGN_IN_LEFT_MID, 120,
                        0, 100, 100, "Intake");
  _1028A::ui::createBtn(Tab3, _1028A::ui::DriveGRCB, LV_ALIGN_IN_LEFT_MID, 240,
                        0, 100, 100, "Drive");
  _1028A::ui::createBtn(Tab3, _1028A::ui::BatteryGRCB, LV_ALIGN_IN_LEFT_MID,
                        360, 0, 100, 100, "Battery");
  _1028A::ui::createBtn(Tab4, _1028A::ui::OdomDebugAction, LV_ALIGN_IN_LEFT_MID,
                        0, 0, 100, 100, "Odom \nDebug");
  _1028A::ui::createBtn(Tab4, _1028A::ui::GrafanaAction, LV_ALIGN_IN_LEFT_MID,
                        120, 0, 100, 100, "Start \nGrafana");

  _1028A::task::start("uiLockout", _1028A::ui::uiLockout);
}