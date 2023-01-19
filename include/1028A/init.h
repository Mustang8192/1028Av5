#include "main.h"
#include "1028A/gifdec.h"
#include "1028A/vars.h"
#include "1028A/robot.h"
#include "1028A/json.hpp"
#include <cmath>
#include <memory>
#include <utility>
#include <vector>
#include "okapi/api/control/util/pathfinderUtil.hpp"
#include "okapi/api/units/QLength.hpp"
#include "okapi/api/util/logging.hpp"
#include "okapi/api/util/mathUtil.hpp"
#include "okapi/pathfinder/include/pathfinder.h"
#include <initializer_list>
#include <vector>
#include <cmath>

namespace _1028A{
    namespace robot{
        void resetDrive();
        void DriveStop();
        void auton();
    }

    namespace pid{
        void turn(double RequestedValue, double spd, double thre, double time);
        void forward(double RequestedValue, double spd, double thre, double time);
        int math(float Error, float lastError, float Kp, float Ki, float Kd, double maxSpd);
        bool exit(float Error, float Threshold, float currTime, float startTime, float timeExit);
    }

    namespace ui{
        void init(int hue = 190, bool comp = false);
        lv_res_t station_sideBtnAction(lv_obj_t *btn);
        lv_res_t right_sideBtnAction(lv_obj_t *btn);
        lv_res_t do_nothingBtnAction(lv_obj_t *btn);
        lv_res_t skillsBtnAction(lv_obj_t *btn);
        lv_res_t soloWPBtnAction(lv_obj_t *btn);
        lv_res_t Flywheelbeastmode(lv_obj_t *btn);
        lv_res_t OdomDebugAction(lv_obj_t *btn);
        lv_res_t GrafanaAction(lv_obj_t *btn);
        lv_res_t FlywheelGRCB(lv_obj_t * btn);
        lv_res_t IntakeGRCB(lv_obj_t * btn);
        lv_res_t DriveGRCB(lv_obj_t * btn);
        lv_res_t BatteryGRCB(lv_obj_t * btn);
        void createStyles();
        void createDoneBtn(lv_obj_t *location, lv_res_t callback(lv_obj_t *btn), lv_align_t alignment, int offsetx, int offsety, std::string text);
        void createBtn (lv_obj_t *location, lv_res_t callback(lv_obj_t *btn), lv_align_t alignment, int offsetx, int offsety, int sizeX, int sizeY, std::string text);
        void grafanaInit (void *ptr);
    }

    namespace time {
        void forward (double spd, double time);
        void turn (double spd, double time);
    }
    namespace driver{
        void checkBrakeType(void *ptr);
        void FlywheelCTRL(void *ptr);
        void DriveCTRL(void *ptr);
        void IntakeCTRL(void *ptr);
        void ExpansionCTRL(void *ptr);
        void ModeCTRL(void *ptr);
    }

    namespace task{
        void start (std::string name, void (*func)(void *));
        bool exists(std::string name);
        void kill (std::string name);
    }

    namespace flywheel{
        void startFlywheel (double target);
        void startFlywheelTask (void *ptr);
    }

    class OdomDebug {
            public:
                struct state_t {
                    okapi::QLength x {0.0};
                    okapi::QLength y {0.0};
                    okapi::QAngle theta {0.0}; 
                    state_t(okapi::QLength ix, okapi::QLength iy, okapi::QAngle itheta); 
                    state_t(double ix, double iy, double itheta);
                };

                struct sensors_t {
                    double left {0.0};
                    double right {0.0};
                    double middle {0.0};
                    sensors_t(double ileft, double iright);
                    sensors_t(double ileft, double iright, double imiddle);

                private:
                    bool hasMiddle {false};
                    friend class OdomDebug;
                };

                OdomDebug(lv_obj_t* parent);
                OdomDebug(lv_obj_t* parent, lv_color_t mainColor);
                ~OdomDebug();

                void setStateCallback(std::function<void(state_t state)> callback);
                void setResetCallback(std::function<void()> callback);
                void setData(state_t state, sensors_t sensors);
                static void startOdomDebug(void *ptr);
                static void resetSens();

                private:

                lv_obj_t* container = nullptr; 
                lv_style_t cStyle;

                lv_style_t fStyle; 
                double fieldDim = 0; 

                lv_style_t grey;
                lv_style_t red;
                lv_style_t blue;

                lv_obj_t* led = nullptr; 
                lv_style_t ledStyle;

                lv_obj_t* line = nullptr; 
                lv_style_t lineStyle;
                std::vector<lv_point_t> linePoints = {{0, 0}, {0, 0}};
                int lineWidth = 0;
                int lineLength = 0;

                lv_obj_t* statusLabel = nullptr;
                lv_style_t textStyle;

                lv_style_t resetRel;
                lv_style_t resetPr;

                std::function<void(state_t state)> stateFnc = nullptr;
                std::function<void()> resetFnc = nullptr;

                static lv_res_t tileAction(lv_obj_t*); 
                static lv_res_t resetAction(lv_obj_t*);
        };

        class TaskWrapper {
            protected:
            TaskWrapper() = default;
            TaskWrapper(const TaskWrapper &itask) = delete;
            TaskWrapper(TaskWrapper &&itask) = default;
            virtual ~TaskWrapper() = default;

            virtual void loop();

            public:

            virtual void startTask(const char *iname = "TaskWrapper");

            virtual void resumeTask();

            virtual void pauseTask();

            virtual void stopTask();

            virtual char const *getName();

            private:
            static void trampoline(void *iparam);
            std::unique_ptr<pros::Task> task{nullptr};
        };

        class AsyncGrapher : public TaskWrapper {
            private:
            std::map<std::string, std::vector<double>> container;
            std::map<std::string, uint32_t> colors;
            std::string title;
            okapi::QTime refreshRate;
            int cnt;

            public:

            AsyncGrapher(const std::string &title, const okapi::QTime &rate = 10 * okapi::millisecond);

            void addDataType(const std::string &name, const uint32_t color);

            void update(const std::string &name, double val);

            void setRefreshRate(const okapi::QTime &rate);

            okapi::QTime getRefreshRate();

            protected:
            void loop() override;
        };

        namespace Grafana{
            class VariableDataHandler{
                public:
                    virtual std::map<std::string, double> get_data() = 0;
            };

            template <typename T>
            class Variable : public VariableDataHandler{
                private:
                    std::map<std::string, std::function<double(T)>> getters;

                    std::string name;

                    T value;

                public:
                    template <typename TF>
                    Variable(std::string name, TF &&value) : value(std::forward<TF>(value)), name(std::move(name)) {}

                    void add_getter(const std::string &varName, std::function<double(T)> func)
                    {
                        getters[varName] = func;
                    }

                    std::map<std::string, double> get_data() 
                    {
                        std::map<std::string, double> toReturn;

                        for (const auto &kv : getters)
                        {
                            toReturn[name + " " + kv.first] = kv.second(value);
                        }

                        return toReturn;
                    }
            };

            template <typename T>
            class VariableGroup : public VariableDataHandler{
                private:
                    std::vector<Variable<T>> variables;

                public:
                    
                    VariableGroup(const std::initializer_list<Variable<T>> &variables) : variables(variables) {}

                    void add_getter(const std::string &varName, std::function<double(T &&)> func)
                    {
                        for (Variable<T> &variable : variables)
                        {
                            variable.add_getter(varName, func);
                        }
                    }

                    std::map<std::string, double> get_data() override
                    {
                        std::map<std::string, double> toReturn;

                        for (Variable<T> &variable : variables)
                        {
                            for (const auto &kv : variable.get_data())
                            {
                                toReturn[kv.first] = kv.second;
                            }
                        }

                        return toReturn;
                    }
            };

            class GUIManager{
                private:
                    std::vector<VariableDataHandler *> variableData;

                public:
                    void setRefreshRate(int refreshRate);

                    int getRefreshRate();

                    void startTask();

                    void stopTask();

                    void registerDataHandler(VariableDataHandler *dataHandler)
                    {
                        variableData.push_back(dataHandler);
                    }

                private:
                    bool alive;

                    int refreshRate = 20;

                    void sendData();

                    void guiTask(void* pParams);

                    void sendConfiguration();
            };

            static void grafanaInit(void *ptr);
        };

        class Gif {
            public:
                Gif(const char* fname, lv_obj_t* parent);

                ~Gif();

                void pause();

                void resume();

                void clean();

                private:

                gd_GIF* _gif = nullptr; 
                void* _gifmem = nullptr;
                uint8_t* _buffer = nullptr;

                lv_color_t* _cbuf = nullptr;
                lv_obj_t* _canvas = nullptr;

                pros::task_t _task = nullptr; 

                void _cleanup();

                void _render();

                static void _render_task(void* arg);

            };

        struct Pose{
            okapi::QLength x;
            okapi::QLength y;
            okapi::QAngle yaw;
        };

        struct InternalPose{
            double x;
            double y;
            double yaw;
        };

        struct PurePursuitTriangle{
            InternalPose currentPose;
            InternalPose localGoalPose;
            double l;
            double r;
        };

        struct InternalPoseIndexed{
            InternalPose pose;
            int index;
        };

        struct InternalDistancePoseIndexed{
            InternalPose pose;
            int index;
            double distance;
        };

        struct PosePath{
            std::string id;
            std::vector<InternalPose> path;
        };

        struct IndexedPosePath{
            std::string id;
            std::vector<InternalPoseIndexed> path;
        };

        struct IndexedDistancePosePath{
            std::string id;
            std::vector<InternalDistancePoseIndexed> path;
        };

        class PathGenerator{

            public:
            PathGenerator( const okapi::PathfinderLimits &ilimits );
            
            void generatePath(const std::initializer_list<Pose> &iwaypoints, const std::string &iid);

            void generatePath(  const std::initializer_list<Pose> &iwaypoints, 
                                const std::string &iid,
                                const okapi::PathfinderLimits &ilimits);
            void showPath();
            
            std::vector<IndexedDistancePosePath> &getPaths();

            protected:
            std::vector<IndexedDistancePosePath> paths;
            std::shared_ptr<okapi::Logger> logger;
            okapi::PathfinderLimits limits;
        };

        class PurePursuit {
            public:
    
            PurePursuit( const std::vector<IndexedDistancePosePath> &ipaths, const okapi::QLength &ilookaheadDistance );

            PurePursuitTriangle run( const okapi::OdomState &ipose, const std::string &iid );
            PurePursuitTriangle run( const Pose &ipose, const std::string &iid );

            static void updateChassis( const double &reqVelocity, const PurePursuitTriangle &triangle, const std::shared_ptr<okapi::OdomChassisController> &controller );

            static double findDistanceBetweenStateAndPose( const okapi::OdomState &state, const InternalPose &pose2 ){
                InternalPose pose1{ state.x.convert(okapi::meter), state.y.convert(okapi::meter), state.theta.convert(okapi::radian) };
                return findDistanceBetweenPoses( pose1, pose2 );
            }

            InternalPose getLastPoseInPath( const std::string &id ){
                for( const auto &ipath: paths ){
                    if( ipath.id == id ){
                        return ipath.path.at(ipath.path.size() - 1).pose;
                    }
                }
            }

            protected:
            const std::vector<IndexedDistancePosePath> paths;
            IndexedDistancePosePath path;
            double avgDistanceBetweenPoses;

            IndexedDistancePosePath nearestPoses;
            PosePath currentPoses;
            IndexedPosePath goalPoses;
            std::vector<PurePursuitTriangle> triangles;

            double lookaheadDistance;

            void findPath( const std::string & );
            void findNearestPose( const Pose & );
            static double findDistanceBetweenPoses( const InternalPose &, const InternalPose & );

            void findGoalPose();
            PurePursuitTriangle findPurePursuitTriangle();
            bool isPoseWithinCircle( const InternalPose &point, const InternalPose &centerPose, const double &lookaheadDistance );

        };
}
