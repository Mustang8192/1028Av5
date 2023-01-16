#include "1028A/init.h"

_1028A::PathGenerator::PathGenerator( const okapi::PathfinderLimits &ilimits ){
    std::cout << "\nPath Generator: Initializ";
    limits = ilimits;

    const std::shared_ptr<okapi::Logger> &ilogger = okapi::Logger::getDefaultLogger();
    logger = ilogger;
    std::cout << "ed\n";
}

void _1028A::PathGenerator::generatePath(const std::initializer_list<Pose> &iwaypoints, const std::string &iid){
    generatePath( iwaypoints, iid, limits );
}

void _1028A::PathGenerator::generatePath(   const std::initializer_list<Pose> &iwaypoints,
                                    const std::string &iid,
                                    const okapi::PathfinderLimits &ilimits ){
    std::cout << "Path Generator: Generating Path\n";
    int time = pros::millis();

    if (iwaypoints.size() == 0) {
        return;
    }

    std::vector<Waypoint> points;
    points.reserve(iwaypoints.size());
    for (auto &point : iwaypoints) {
        points.push_back(
        Waypoint{point.x.convert(okapi::meter), point.y.convert(okapi::meter), point.yaw.convert(okapi::radian)});
    }

    TrajectoryCandidate candidate;
    pathfinder_prepare(points.data(),
                        static_cast<int>(points.size()),
                        FIT_HERMITE_CUBIC,
                        PATHFINDER_SAMPLES_FAST,
                        0.010,
                        ilimits.maxVel,
                        ilimits.maxAccel,
                        ilimits.maxJerk,
                        &candidate);

    const int length = candidate.length;

    if (length < 0) {
        std::string message = "AsyncMotionProfileController: Length was negative. ";

        if (candidate.laptr) {
        free(candidate.laptr);
        }

        if (candidate.saptr) {
        free(candidate.saptr);
        }

        throw std::runtime_error(message);
    }

    auto *trajectory = new Segment[length];

    if (trajectory == nullptr) {
        std::string message = "AsyncMotionProfileController: Could not allocate trajectory. ";

        if (candidate.laptr) {
        free(candidate.laptr);
        }

        if (candidate.saptr) {
        free(candidate.saptr);
        }
        throw std::runtime_error(message);
    }

    pathfinder_generate(&candidate, trajectory);


    std::vector<InternalDistancePoseIndexed> poses;
    for( int i = 0; i < candidate.length; i++ ){
        poses.emplace_back( InternalDistancePoseIndexed{ InternalPose{trajectory[i].x, trajectory[i].y, trajectory[i].heading}, candidate.path_length, candidate.totalLength } );
    }
    IndexedDistancePosePath path{ iid,  };

    paths.emplace_back( path );

    std::cout << "Path Generator: Done in " << pros::millis() - time << " millis\n";
  
}

void _1028A::PathGenerator::showPath(){
    std::cout << "Path Generator: Showing Paths\n";
}

std::vector<_1028A::IndexedDistancePosePath> &_1028A::PathGenerator::getPaths(){
    std::cout << "Path Generator: Getting Paths\n";
    return paths;
}