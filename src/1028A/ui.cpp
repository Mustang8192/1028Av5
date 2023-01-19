#include "1028A/init.h"
#include "1028A/robot.h"
#include "1028A/vars.h"

int done = 0;
/**
* @brief 
*  This function is a callback function for auton selector
* @param btn 
* @return lv_res_t 
*/

lv_res_t _1028A::ui::station_sideBtnAction(lv_obj_t *btn){
	autonSelect = 1;
  done = 1;
	return LV_RES_OK;
}

/**
* @brief 
*  This function is a callback function for auton selector
* @param btn 
* @return lv_res_t 
*/

lv_res_t _1028A::ui::right_sideBtnAction(lv_obj_t *btn){
	autonSelect = 2;
  done = 1;
	return LV_RES_OK;
}

/**
* @brief 
*  This function is a callback function for auton selector
* @param btn 
* @return lv_res_t 
*/

lv_res_t _1028A::ui::do_nothingBtnAction(lv_obj_t *btn){
	autonSelect = 0;
  done = 1;
	return LV_RES_OK;
}

/**
* @brief 
*  This function is a callback function for auton selector
* @param btn 
* @return lv_res_t 
*/

lv_res_t _1028A::ui::soloWPBtnAction(lv_obj_t *btn){
	autonSelect = 3;
  done = 1;
	return LV_RES_OK;
}

/**
* @brief 
*  This function is a callback function for auton selector
* @param btn 
* @return lv_res_t 
*/

lv_res_t _1028A::ui::skillsBtnAction(lv_obj_t *btn){
	autonSelect = 8;
  done = 1;
	return LV_RES_OK;
}

/**
* @brief 
*  This function is a callback function for auton selector
* @param btn 
* @return lv_res_t 
*/

lv_res_t _1028A::ui::Flywheelbeastmode(lv_obj_t *btn){
	flywheelMode = 2;
	return LV_RES_OK;
}

/**
* @brief 
*  This function is a callback function to start the grafana task
* @param btn 
* @return lv_res_t 
*/

lv_res_t _1028A::ui::GrafanaAction(lv_obj_t *btn){
  task::start("grafana", Grafana::grafanaInit);
	return LV_RES_OK;
}

/**
* @brief 
*  This function is a callback function to start the odometry debug task
* @param btn 
* @return lv_res_t 
*/

lv_res_t _1028A::ui::OdomDebugAction(lv_obj_t *btn){
	lv_obj_clean(lv_scr_act());
	_1028A::OdomDebug Display(lv_scr_act(), LV_COLOR_ORANGE);
	_1028A::task::start("OdomDebug", Display.startOdomDebug);
	return LV_RES_OK;
}


/**
* @brief 
* This function is a callback function to start Grafana data reporting
*/


void _1028A::Grafana::grafanaInit (void *ptr){
	Grafana::GUIManager manager;
	manager.setRefreshRate(200);

	Grafana::Variable<pros::Motor> leftFrontMotorVar("Left Front Motor", pros::LeftFront);
	Grafana::Variable<pros::Motor> rightFrontMotorVar("Right Front Motor", pros::RightFront);
  Grafana::Variable<pros::Motor> leftMidMotorVar("Left Mid Motor", pros::LeftMid);
	Grafana::Variable<pros::Motor> rightMidMotorVar("Right Mid Motor", pros::RightMid);
	Grafana::Variable<pros::Motor> leftBackMotorVar("Left Back Motor", pros::LeftBack);
	Grafana::Variable<pros::Motor> rightBackMotorVar("Right Back Motor", pros::RightBack);
	
	
	Grafana::VariableGroup<pros::Motor> chassisVars({leftFrontMotorVar, rightFrontMotorVar, leftMidMotorVar, rightMidMotorVar, leftBackMotorVar, rightBackMotorVar});


	chassisVars.add_getter("Temperature", &pros::Motor::get_temperature);
	chassisVars.add_getter("Actual Velocity", &pros::Motor::get_actual_velocity);
	chassisVars.add_getter("Voltage", &pros::Motor::get_voltage);
	chassisVars.add_getter("Efficiency", &pros::Motor::get_efficiency);

	manager.registerDataHandler(&chassisVars);

	manager.startTask();

  while (1){
    pros::delay(200);
  }
}

/**
* @brief 
*  This function is a callback function to start the flywheel grapher task
* @param btn 
* @return lv_res_t 
*/

lv_res_t _1028A::ui::FlywheelGRCB(lv_obj_t * btn){
    std::shared_ptr<_1028A::AsyncGrapher> grapher  (new _1028A::AsyncGrapher("Flywheel Velocity vs. Time"));
    grapher->addDataType("Actual Vel", COLOR_AQUAMARINE);
    grapher->addDataType("Desired Vel", COLOR_ORANGE);
    grapher->startTask();
    while (1){
      grapher->update("Desired Vel", 1);
      grapher->update("Actual Vel", pros::FlyWheel.get_actual_velocity()/200);
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

lv_res_t _1028A::ui::IntakeGRCB(lv_obj_t * btn){
    std::shared_ptr<_1028A::AsyncGrapher> grapher  (new _1028A::AsyncGrapher("Intake Velocity vs. Time"));
    grapher->addDataType("Desired Vel", COLOR_ORANGE);
    grapher->addDataType("Actual Vel", COLOR_AQUAMARINE);
    grapher->startTask();
    while (1){
      grapher->update("Desired Vel", 0);
      grapher->update("Actual Vel", pros::Intake.get_actual_velocity()/200);
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

lv_res_t _1028A::ui::DriveGRCB(lv_obj_t * btn){
    std::shared_ptr<AsyncGrapher> grapher  (new AsyncGrapher("Drive Velocity vs. Time"));
    grapher->addDataType("Left Front", COLOR_ORANGE);
    grapher->addDataType("Left Back", COLOR_RED);
    grapher->addDataType("Right Front", COLOR_AQUAMARINE);
    grapher->addDataType("Right Back", COLOR_GREEN);
    grapher->startTask();
    while (1){
      grapher->update("Left Front", std::abs(pros::LeftFront.get_actual_velocity()/200));
      grapher->update("Left Back", std::abs(pros::LeftBack.get_actual_velocity()/200));
      grapher->update("Right Front", std::abs(pros::RightFront.get_actual_velocity()/200));
      grapher->update("Right Back", std::abs(pros::RightBack.get_actual_velocity()/200));
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
lv_res_t _1028A::ui::BatteryGRCB(lv_obj_t * btn){
    std::shared_ptr<_1028A::AsyncGrapher> grapher  (new _1028A::AsyncGrapher("Battery vs. Time"));
    grapher->addDataType("Temperature", COLOR_ORANGE);
    grapher->addDataType("Current", COLOR_RED);
    grapher->addDataType("Voltage", COLOR_AQUAMARINE);
    grapher->addDataType("Capacity", COLOR_GREEN);
    grapher->startTask();
    while (1){
      grapher->update("Temperature", pros::battery::get_temperature()/120);
      grapher->update("Current", pros::battery::get_current()/2000.0);
      grapher->update("Voltage", pros::battery::get_voltage()/20000.0);
      grapher->update("Capacity", pros::battery::get_capacity()/100);
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

void _1028A::ui::createBtn (lv_obj_t *location, lv_res_t callback(lv_obj_t *btn), lv_align_t alignment, int offsetx, int offsety, int sizeX, int sizeY, std::string text){
	lv_obj_t * Btn = lv_btn_create(location, NULL);
	lv_obj_align(Btn, NULL, alignment, offsetx, offsety);
	lv_obj_set_size(Btn, sizeX, sizeY);
	lv_cb_set_action(Btn, callback);

	lv_obj_t *BtnLbl = lv_label_create(Btn, NULL);
	lv_label_set_text(BtnLbl, text.c_str());
}

/**
* @brief 
*  This function watches all the button matrixes and updates the variables
*/

/**
* @brief 
*  This function starts the Ui
* @param hue 
* @param default_auton 
*/

void _1028A::ui::init(int hue, bool comp){

	// lvgl theme
	lv_theme_t *th = lv_theme_alien_init(hue, NULL); //Set a HUE value and keep font default RED
	lv_theme_set_current(th);

	// create a tab view object
	lv_obj_t *tabview = lv_tabview_create(lv_scr_act(), NULL);

	// add 3 tabs (the tabs are page (lv_page) and can be scrolled
	lv_obj_t *Tab1 = lv_tabview_add_tab(tabview, "Selector 1");
  lv_obj_t *Tab2 = lv_tabview_add_tab(tabview, "Selector 2");
	lv_obj_t *Tab3 = lv_tabview_add_tab(tabview, "Graphs");
    lv_obj_t *Tab4 = lv_tabview_add_tab(tabview, "Misc");

    _1028A::ui::createBtn(Tab1, _1028A::ui::station_sideBtnAction, LV_ALIGN_IN_LEFT_MID, 0,0, 100, 100, "Left_side");
	_1028A::ui::createBtn(Tab1, _1028A::ui::right_sideBtnAction, LV_ALIGN_IN_LEFT_MID, 120,0, 100, 100, "Right_side");
	_1028A::ui::createBtn(Tab1, _1028A::ui::do_nothingBtnAction, LV_ALIGN_IN_LEFT_MID, 240,0, 100, 100, "_");
	_1028A::ui::createBtn(Tab1, _1028A::ui::do_nothingBtnAction, LV_ALIGN_IN_LEFT_MID, 360, 0, 100, 100, "_");
  _1028A::ui::createBtn(Tab2, _1028A::ui::do_nothingBtnAction, LV_ALIGN_IN_LEFT_MID, 0,0, 100, 100, "_");
	_1028A::ui::createBtn(Tab2, _1028A::ui::do_nothingBtnAction, LV_ALIGN_IN_LEFT_MID, 120,0, 100, 100, "_");
	_1028A::ui::createBtn(Tab2, _1028A::ui::do_nothingBtnAction, LV_ALIGN_IN_LEFT_MID, 240,0, 100, 100, "_");
	_1028A::ui::createBtn(Tab2, _1028A::ui::skillsBtnAction, LV_ALIGN_IN_LEFT_MID, 360, 0, 100, 100, "Skills");
	_1028A::ui::createBtn(Tab4, _1028A::ui::Flywheelbeastmode, LV_ALIGN_IN_LEFT_MID, 240,0, 100, 100, "Flywheel \n  Big Boy \n  Mode");
    _1028A::ui::createBtn(Tab3, _1028A::ui::FlywheelGRCB, LV_ALIGN_IN_LEFT_MID, 0,0, 100, 100, "Flywheel");
	_1028A::ui::createBtn(Tab3, _1028A::ui::IntakeGRCB, LV_ALIGN_IN_LEFT_MID, 120,0, 100, 100, "Intake");
	_1028A::ui::createBtn(Tab3, _1028A::ui::DriveGRCB, LV_ALIGN_IN_LEFT_MID, 240,0, 100, 100, "Drive");
	_1028A::ui::createBtn(Tab3, _1028A::ui::BatteryGRCB, LV_ALIGN_IN_LEFT_MID, 360, 0, 100, 100, "Battery");
    _1028A::ui::createBtn(Tab4, _1028A::ui::OdomDebugAction, LV_ALIGN_IN_LEFT_MID, 0, 0, 100, 100, "Odom \nDebug");
    _1028A::ui::createBtn(Tab4, _1028A::ui::GrafanaAction, LV_ALIGN_IN_LEFT_MID, 120, 0, 100, 100, "Start \nGrafana");

    if (comp){
      while (1){
        if (done){
          break;
        }
        pros::delay(200);
      }
    }
}