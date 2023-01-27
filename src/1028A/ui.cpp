#include "1028A/init.h"
#include "1028A/robot.h"
#include "1028A/vars.h"
#include "pros/misc.hpp"
#include "pros/rtos.hpp"

typedef FILE *pc_file_t;
static lv_fs_res_t _1028A::ui::pcfs_open( void * file_p, const char * fn, lv_fs_mode_t mode){
    errno = 0;
    const char * flags = "";
    if(mode == LV_FS_MODE_WR) flags = "wb";
    else if(mode == LV_FS_MODE_RD) flags = "rb";
    else if(mode == (LV_FS_MODE_WR | LV_FS_MODE_RD)) flags = "a+";

    char buf[256];
    sprintf(buf, "/%s", fn);
    pc_file_t f = fopen(buf, flags);

    if(f == NULL)
      return LV_FS_RES_UNKNOWN;
    else {
      fseek(f, 0, SEEK_SET);
      pc_file_t * fp = (pc_file_t *)file_p;
      *fp = f;
    }

    return LV_FS_RES_OK;
}

static lv_fs_res_t _1028A::ui::pcfs_close( void * file_p){
    pc_file_t * fp = (pc_file_t *)file_p;
    fclose(*fp);
    return LV_FS_RES_OK;
}

static lv_fs_res_t _1028A::ui::pcfs_read( void * file_p, void * buf, uint32_t btr, uint32_t * br){
    pc_file_t * fp =  (pc_file_t *)file_p;
    *br = fread(buf, 1, btr, *fp);
    return LV_FS_RES_OK;
}

static lv_fs_res_t _1028A::ui::pcfs_seek( void * file_p, uint32_t pos){
    pc_file_t * fp = (pc_file_t *)file_p;
    fseek(*fp, pos, SEEK_SET);
    return LV_FS_RES_OK;
}

static lv_fs_res_t _1028A::ui::pcfs_tell( void * file_p, uint32_t * pos_p){
    pc_file_t * fp =  (pc_file_t *)file_p;
    *pos_p = ftell(*fp);
    return LV_FS_RES_OK;
}

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

void _1028A::ui::uiWatcher(void *ptr){
  int hasPrinted = 0;
  while (true){
    if (pros::competition::is_connected() && !pros::competition::is_disabled() && !pros::competition::is_autonomous() && hasPrinted == 0){
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

      lv_obj_t * img_var = lv_img_create(lv_scr_act(), NULL);
      lv_img_set_src(img_var, "S:/usd/BT1.bin");
      lv_obj_set_pos(img_var, 0, 0);
      hasPrinted = 1;
    }

    if (pros::competition::is_disabled() && pros::competition::is_connected()){
       pros::mainController.print(1, 1, "Auton: %f", autonSelect);
    }
    pros::delay(200);
  }
}

/**
* @brief 
*  This function starts the Ui
* @param hue 
* @param default_auton 
*/

void _1028A::ui::init(int hue, bool repeated){

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

    if (!repeated) _1028A::task::start("uiTabwatcher", _1028A::ui::uiWatcher);
}