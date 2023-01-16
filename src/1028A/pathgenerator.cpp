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

/**
 * This code was taken from OkapiLib most credit regarding this code goes to the authors of the file linked
 * below:
 *      https://github.com/OkapiLib/OkapiLib/blob/f0da55095b128fdfed18fab68c232569e2a69d06/src/api/control/async/asyncMotionProfileController.cpp#L67
 */
void _1028A::PathGenerator::generatePath(   const std::initializer_list<Pose> &iwaypoints,
                                    const std::string &iid,
                                    const okapi::PathfinderLimits &ilimits ){
    std::cout << "Path Generator: Generating Path\n";
    int time = pros::millis();

    if (iwaypoints.size() == 0) {
        // No point in generating a path
//        LOG_WARN(std::string(
//        "AsyncMotionProfileController: Not generating a path because no waypoints were given."));
        return;
    }

    std::vector<Waypoint> points;
    points.reserve(iwaypoints.size());
    for (auto &point : iwaypoints) {
        points.push_back(
        Waypoint{point.x.convert(okapi::meter), point.y.convert(okapi::meter), point.yaw.convert(okapi::radian)});
    }

//    LOG_INFO(std::string("AsyncMotionProfileController: Preparing trajectory"));

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
        std::string message = "AsyncMotionProfileController: Length was negative. " /*+
                            getPathErrorMessage(points, ipathId, length)*/;

        if (candidate.laptr) {
        free(candidate.laptr);
        }

        if (candidate.saptr) {
        free(candidate.saptr);
        }

//        LOG_ERROR(message);
        throw std::runtime_error(message);
    }

    auto *trajectory = new Segment[length];

    if (trajectory == nullptr) {
        std::string message = "AsyncMotionProfileController: Could not allocate trajectory. " /*+
                            getPathErrorMessage(points, ipathId, length)*/;

        if (candidate.laptr) {
        free(candidate.laptr);
        }

        if (candidate.saptr) {
        free(candidate.saptr);
        }

//        LOG_ERROR(message);
        throw std::runtime_error(message);
    }

//    LOG_INFO(std::string("AsyncMotionProfileController: Generating path"));

    pathfinder_generate(&candidate, trajectory);

//    free(trajectory);

    // Free the old path before overwriting it
//    forceRemovePath(ipathId);

    std::vector<InternalDistancePoseIndexed> poses;
    for( int i = 0; i < candidate.length; i++ ){
        //LOG_INFO(std::string("PathGenerator: Pose Generated") );
        poses.emplace_back( InternalDistancePoseIndexed{ InternalPose{trajectory[i].x, trajectory[i].y, trajectory[i].heading}, candidate.path_length, candidate.totalLength } );
    }
    IndexedDistancePosePath path{ iid,  };

    paths.emplace_back( path );

    std::cout << "Path Generator: Done in " << pros::millis() - time << " millis\n";

//    LOG_INFO("AsyncMotionProfileController: Completely done generating path " + ipathId);
//    LOG_DEBUG("AsyncMotionProfileController: Path length: " + std::to_string(length));    
}

void _1028A::PathGenerator::showPath(){
    std::cout << "Path Generator: Showing Paths\n";
    //I'm accepting PRs ;)
}

std::vector<_1028A::IndexedDistancePosePath> &_1028A::PathGenerator::getPaths(){
    std::cout << "Path Generator: Getting Paths\n";
    return paths;
}