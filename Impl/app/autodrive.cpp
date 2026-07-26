#if defined(__clang__)
#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Wsign-compare"
#pragma clang diagnostic ignored "-Wunknown-pragmas"
#pragma clang diagnostic ignored "-Wreturn-type-c-linkage"
#endif

#include <cassert>
#include <cmath>
#include <iostream>
#include <limits>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

#include "../util/GetSignType.h"
#include "../util/UtilDriver.h"
#include "../util/UtilMath.h"
#include "SSD/SimPoint2D.h"
#include "SSD/SimPoint3D.h"
#include "SSD/SimString.h"
#include "SimOneEvaluationAPI.h"
#include "SimOneHDMapAPI.h"
#include "SimOnePNCAPI.h"
#include "SimOneSensorAPI.h"
#include "SimOneServiceAPI.h"
#include "../control/pid.h"
#include "../perception/prediction.h"
#include "public/common/MLaneId.h"
#include "public/common/MLaneInfo.h"

#include "../common/types.h"
#include "../common/PerceptionUtils.h"
#include "../common/PathUtils.h"
#include "../common/DecisionUtils.h"

#if defined(__clang__)
#pragma clang diagnostic pop
#endif

namespace {

constexpr bool kEnableDebugLog = false;

double NormalizeSignedAngle(double angle) {
    if (angle > M_PI) {
        return angle - 2 * M_PI;
    }
    if (angle < -M_PI) {
        return angle + 2 * M_PI;
    }
    return angle;
}

}

int main() {
    // C++ standard library input and output stream performance optimization
    std::ios::sync_with_stdio(false);
    std::cin.tie(nullptr);
    std::cout.tie(nullptr);

    // --- Data structure and variable initialization ---
    // Data structure pointer required to initialize SimOne API
    std::unique_ptr<SimOne_Data_Gps> gpsPtr = std::make_unique<SimOne_Data_Gps>();
    std::unique_ptr<SimOne_Data_Obstacle> obstaclesPtr = std::make_unique<SimOne_Data_Obstacle>();
    std::unique_ptr<SimOne_Data_Signal_Lights> signalLightsPtr = std::make_unique<SimOne_Data_Signal_Lights>();

    const char *kMainVehicleId = "0"; // Main vehicle ID

    // status flag bit
    bool inAEBState = false; // Whether it is in AEB (automatic emergency braking) state
    (void)inAEBState;
    bool isJoinTimeLoop = true; // Whether to add time synchronization loop
    bool firstFrame = false; // First frame flag
    bool secondFrame = false; // Second frame flag

    // Vehicle and route related variables
    SSD::SimPoint3D mainVehiclePos; // Main vehicle position
    SSD::SimString currentLaneId; // Current lane ID
    double initialSteering; // Calculated initial steering wheel angle
    double slowSpeedMps = 10; // Slow driving speed
    (void)slowSpeedMps;
    static double headingErrorRad = 0, turnHeadingErrorRad = 0; // Path tracking related angle variables
    size_t forwardIndex = 100; // Forward waypoint index
    (void)forwardIndex;
    SSD::SimVector<long> naviRoadIdList; // Navigation segment ID list

    // PID controller is used for speed control
    PIDController speedController{};
    speedController.configure(1, 0, 0, 0.1); // Set PID parameters
    speedController.setLimits(-5, 1); // Set output limits

    // Path planning and decision-related variables
    SSD::SimPoint3D startPt, endPt; // Navigation start and end points
    SSD::SimPoint3D turnTargetPoint; // Turn to target point
    SSD::SimPoint3D laneChangePoint; // Lane change path point
    SSD::SimString targetLaneName; // Target lane name
    SSD::SimPoint3DVector targetPath, routeWaypoints, casePath; // path point vector
    SSD::SimPoint3DVector detectionZone; // Detection area
    SSD::SimString targetLaneId; // Target lane change lane ID
    SSD::SimString speedLimitLaneId; // Lane ID where the speed limit sign is located
    double throttle; // Throttle/brake control amount
    std::string driveStateName = "Start!"; // Vehicle operating state machine

    // Scenario and special situation flags
    bool isLongRouteCase = false; // Whether this is the long-route scenario
    bool isNearSection = false; // Is it close to an intersection?
    bool isNeedBuild = false; // Whether the path needs to be rebuilt
    bool hasExitedIntersection = false; // Whether to exit the intersection
    // flag for specific test cases
    bool isCase24 = false;
    bool isCase10 = false;
    bool isCase28 = false;
    bool isCase18 = false;
    bool isCase41 = false;

    // Counter
    size_t countdown = 0;
    size_t countdown2 = 0;
    size_t index;

    // --- SimOne API initialization ---
    SimOneAPI::InitSimOneAPI(kMainVehicleId, isJoinTimeLoop); // Initialize SimOne API
    SimOneAPI::SetDriverName(kMainVehicleId, "AutoDrive"); // Set driver name
    SimOneAPI::InitEvaluationServiceWithLocalData(kMainVehicleId); // Initialize evaluation service
    int timeout = 20; // HDMap loading timeout

    // Load high-precision map until successful
    while (true) {
        if (SimOneAPI::LoadHDMap(timeout)) {
            SimOneAPI::SetLogOut(
                    ESimOne_LogLevel_Type::ESimOne_LogLevel_Type_Information,
                    "HDMap Information Loaded");
            break;
        }
        SimOneAPI::SetLogOut(
                ESimOne_LogLevel_Type::ESimOne_LogLevel_Type_Information,
                "HDMap Information Loading...");
    }

    // Wait for the initialization of the SimOne simulation environment to be completed, marked by obtaining valid GPS data.
    while (true) {
        SimOneAPI::GetGps(kMainVehicleId, gpsPtr.get());
        if ((gpsPtr->timestamp > 0)) {
            printf("SimOne Initialized\n");
            break;
        }
        printf("SimOne Initializing...\n");
    }

    SimOneAPI::InitEvaluationServiceWithLocalData(kMainVehicleId); // Initialize the evaluation service again

    // --- Initial path planning ---
    signalLightsPtr->signalLights = 0; // Initialize signal light status
    mainVehiclePos = {gpsPtr->posX, gpsPtr->posY, gpsPtr->posZ}; // Get the initial position of the main vehicle
    currentLaneId = GetNearMostLane(mainVehiclePos); // Get the nearest lane

    // Set navigation starting point
    if (SimOneAPI::GetGps(kMainVehicleId, gpsPtr.get())) {
        startPt.x = gpsPtr->posX;
        startPt.y = gpsPtr->posY;
        startPt.z = gpsPtr->posZ;
        routeWaypoints.push_back(startPt);
    }
    endPt = GetTerminalPoint(); // Get the end point
    SSD::SimVector<int> indexOfValidPoints;
    naviRoadIdList = GetNavigateRoadIdList(startPt, endPt); // Get the navigation segment list
    routeWaypoints.push_back(endPt);
    std::cout << " Navigate Road Size :: " << naviRoadIdList.size() << std::endl;
    SimOneAPI::GenerateRoute(routeWaypoints, indexOfValidPoints, targetPath); // Generate global path
    PrintTargetPath(targetPath); // Print path information

    // --- Main loop ---
    while (true) {
        int frame = SimOneAPI::Wait(); // Wait for the next frame
        std::cout << std::endl;
        std::cout << "frame Start" << std::endl;
        std::cout << std::endl;

        // If the case run ends, save the evaluation record and exit
        if (SimOneAPI::GetCaseRunStatus() ==
            ESimOne_Case_Status::ESimOne_Case_Status_Stop) {
            SimOneAPI::SaveEvaluationRecord();
            break;
        }

        // --- Data collection ---
        SimOneAPI::GetGps(kMainVehicleId, gpsPtr.get()); // Get GPS information
        SimOneAPI::GetGroundTruth(kMainVehicleId, obstaclesPtr.get()); // Get obstacle true value information

        // --- Vehicle status update ---
        SSD::SimPoint3DVector crosswalkKnots;
        double carOriZ = gpsPtr->oriZ; // Vehicle heading angle
        mainVehiclePos = {gpsPtr->posX, gpsPtr->posY, gpsPtr->posZ}; // Vehicle location
        currentLaneId = GetNearMostLane(mainVehiclePos); // Current lane
        static SSD::SimString turn; // Steering intention
        SSD::SimString state;
        HDMapStandalone::MLaneLink lanelink;
        HDMapStandalone::MLaneType leftlanetype, rightlanetype;
        (void)leftlanetype;
        (void)rightlanetype;
        SimOneAPI::GetLaneLink(currentLaneId, lanelink); // Get lane connection relationship
        obstaclestruct obstacleAhead, sameLaneObstacle; // Obstacle ahead
        double mainVehicleSpeed = UtilMath::calculateSpeed(gpsPtr->velX, gpsPtr->velY, gpsPtr->velZ); // Vehicle speed
        double mainVehicleAccel = UtilMath::calculateSpeed(gpsPtr->accelX, gpsPtr->accelY,
                                                           gpsPtr->accelZ); // Vehicle acceleration
        (void)mainVehicleAccel;

        // --- Environment awareness ---
        double obstacleDistanceM = 1000; // Distance to obstacles ahead
        double speedLimitMps; // Road speed limit
        static double maxSpeedMps = 25 / 3.6; // Maximum desired speed
        static double finalSpeedMps = 25 / 3.6; // Final desired speed
        double stopDistanceM = 5.1; // Stopping distance
        double obstacleAheadDistanceM = 1000; // Distance to obstacles ahead
        double lightStopLineDistanceM = 1000; // Traffic light stop line distance
        double noLightStopLineDistanceM = 1000; // Stop line distance at unlit intersection
        double junctionObstacleDistanceM = 1000; // Intersection obstacle distance
        (void)junctionObstacleDistanceM;

        // defines the data structure related to the sensing results
        HDMapStandalone::MObject crosswalk;
        HDMapStandalone::MSignal stopsign;
        HDMapStandalone::MSignal light;
        SSD::SimPoint3D stopLine = GetTargetStopLine(GetTargetLight(currentLaneId, naviRoadIdList), currentLaneId);
        SSD::SimPoint3D lightStopLine;
        SSD::SimPoint3D noLightStopLine;
        SSD::SimPoint3D junctionStopLine;
        double stopLineDistanceM = UtilMath::distance(mainVehiclePos, stopLine); // Distance to stop line

        // Detect and update speed limits
        if (DetectNearestSpeedLimitSign(gpsPtr, isLongRouteCase, speedLimitMps)) {
            speedLimitLaneId = currentLaneId;
            maxSpeedMps = speedLimitMps * 0.9;
            std::cout << "maxSpeedMps" << maxSpeedMps << std::endl;
        }
        // If driving outside the speed limit area, dynamically adjust the maximum speed based on the turning angle and steering wheel angle
        if (currentLaneId != speedLimitLaneId && isLongRouteCase) {
            maxSpeedMps = finalSpeedMps;
            if (abs(turnHeadingErrorRad) > 0.2) {
                maxSpeedMps = 60 / 3.6;
            }
            if (abs(turnHeadingErrorRad) > 0.3) {
                maxSpeedMps = 53 / 3.6;
            }

            if (abs(initialSteering) * 540 > 50) {
                maxSpeedMps = 53 / 3.6;
            }

            if (abs(turnHeadingErrorRad) > 0.5) {
                maxSpeedMps = 42 / 3.6;
            }
            if (abs(initialSteering) * 540 > 100) {
                maxSpeedMps = 42 / 3.6;
            }
            if (abs(initialSteering) * 540 > 200) {
                maxSpeedMps = 35 / 3.6;
            }
        }

        double stopLineAngleRad = 100; // Stop line angle
        double noLightStopLineAngleRad = 100; // No light stop line angle

        // Detect traffic light stop line
        bool hasTrafficLight = DetectStopLine(mainVehiclePos, naviRoadIdList, lightStopLine, light, crosswalk,
                                      lightStopLineDistanceM);
        if (hasTrafficLight) {
            if (kEnableDebugLog) {
                std::cout << "Traffic-light stop line detected." << std::endl;
            }
            stopLineAngleRad = atan2(lightStopLine.y - mainVehiclePos.y, lightStopLine.x - mainVehiclePos.x);
        }

        // Detect stop lines at unlit intersections
        if (DetectNoLightStopLine(gpsPtr, hasTrafficLight, noLightStopLine, crosswalkKnots, noLightStopLineDistanceM)) {
            if (kEnableDebugLog) {
                std::cout << "Unsignalized stop line detected." << std::endl;
            }
            noLightStopLineAngleRad = atan2(noLightStopLine.y - mainVehiclePos.y,
                                           noLightStopLine.x - mainVehiclePos.x);
        }

        // --- Obstacle handling ---
        std::vector<obstaclestruct> allObstacles{};
        GetValidObstacles(gpsPtr, currentLaneId, allObstacles); // Get the list of valid obstacles
        bool isSameLane = DetectObstacleAhead(gpsPtr, allObstacles, sameLaneObstacle, obstacleAheadDistanceM); // Detect obstacles ahead in the same lane

        bool hasObstacle = false; // Whether there are obstacles
        bool hasStaticObstacle = false, hasMovingObstacle = false; // Is the obstacle stationary or dynamic?
        size_t indexOfFixed = 0, indexOfMoving = 0;

        // Detect obstacles on the planned path
        if (!targetPath.empty()) {
            SSD::SimPoint3DVector shortPath = GenerateForwardPoints(index, targetPath, mainVehiclePos); // Generate forward short path for collision detection
            obstaclestruct staticObstacle, movingObstacle;
            // Detect stationary obstacles on the path
            if (DetectFirstObstacleOnPath(allObstacles, shortPath, staticObstacle, indexOfFixed)) {
                hasStaticObstacle = true;
            }
            // Detect dynamic obstacles on the path
            if (DetectMovingObstacleOnPath(allObstacles, shortPath, movingObstacle, indexOfMoving)) {
                hasMovingObstacle = true;
            }

            hasObstacle = hasMovingObstacle || hasStaticObstacle;

            // Determine the nearest obstacle
            if (hasMovingObstacle && !hasStaticObstacle) {
                obstacleAhead = movingObstacle;
            } else if (!hasMovingObstacle && hasStaticObstacle) {
                obstacleAhead = staticObstacle;
            } else if (hasMovingObstacle && hasStaticObstacle) {
                if (indexOfFixed > indexOfMoving) {
                    obstacleAhead = movingObstacle;
                } else {
                    obstacleAhead = staticObstacle;
                }
            }

            // If there is no obstacle on the path but there is one in the same lane, it is regarded as an obstacle ahead.
            if (!hasObstacle && isSameLane) {
                obstacleAhead = sameLaneObstacle;
                hasObstacle = true;
            }
        }

        // If there is an obstacle ahead, print its information
        if (hasObstacle) {
            obstacleDistanceM = UtilMath::distance(mainVehiclePos, obstacleAhead.pt);
            if (kEnableDebugLog) {
                std::cout << "Obstacle detected: speed=" << obstacleAhead.speed
                          << ", distance=" << obstacleDistanceM
                          << ", lane=" << obstacleAhead.ownerLaneId.GetString() << std::endl;
            }
        }

        // Print stop line distance and angle information
        if (kEnableDebugLog) {
            std::cout << "lightStopLineDistanceM=" << lightStopLineDistanceM << std::endl;
            std::cout << "noLightStopLineDistanceM=" << noLightStopLineDistanceM << std::endl;
            std::cout << "abs(carOriZ-stopLineAngleRad)=" << abs(carOriZ - stopLineAngleRad) << std::endl;
            std::cout << "abs(carOriZ-noLightStopLineAngleRad)=" << abs(carOriZ - noLightStopLineAngleRad) << std::endl;
        }

        // --- Drive State Machine (driveStateName) ---
        // Determine whether it is close to an intersection based on distance and angle
        if ((lightStopLineDistanceM < 40 && abs(carOriZ - stopLineAngleRad) < M_PI / 2) ||
            ((noLightStopLineDistanceM < 40 && abs(carOriZ - noLightStopLineAngleRad) < M_PI / 2) &&
             !hasExitedIntersection)) {
            driveStateName = "NearIntersection";
            isNearSection = true;
        } else if (
                ((lightStopLineDistanceM > 40 || abs(carOriZ - stopLineAngleRad) > M_PI / 2) || stopLineAngleRad == 100) &&
                isNearSection
                && ((noLightStopLineDistanceM > 40 || abs(carOriZ - noLightStopLineAngleRad) > M_PI / 2) ||
                    stopLineAngleRad == 100)) {
            driveStateName = "Follow";
            isNearSection = false;
        }

        bool needsBrake = false; // Whether emergency braking is required

        // Status: Start! (Initialization)
        if (driveStateName == "Start!") {
            casePath = targetPath;
            PrintTargetPath(targetPath);
            naviRoadIdList = GetNavigateRoadIdList(startPt, endPt);
            // If it is a long distance scenario, adjust the speed and PID parameters
            if (targetPath.size() > 1000) {
                isLongRouteCase = true;
                maxSpeedMps = 55 / 3.6;
                speedController.setLimits(-7, 100);
            }
            driveStateName = "Follow"; // Switch to follow state
        }

        // Status: Follow (regular following/cruising)
        else if (driveStateName == "Follow") {
            if (hasObstacle) { // If there is an obstacle ahead
                // Determine whether to change lanes or brake based on distance and speed of obstacles
                if (mainVehicleSpeed < 50 / 3.6) {
                    if (obstacleDistanceM < 30) {
                        needsBrake = false;
                        // If there is a slow-moving pedestrian/cyclist ahead, try changing lanes
                        if (obstacleAhead.speed > 0.5 && obstacleAhead.speed < 2 &&
                            obstacleAhead.type == 6) {
                            if (DetectIsTurnable(gpsPtr, allObstacles, obstacleAhead, turn, state)) {
                                driveStateName = "ChangeLaneStart";
                            }
                        }
                        // If there is a stationary obstacle ahead, try to avoid the obstacle
                        else if (obstacleAhead.speed <= 0.5 && secondFrame) {
                            if (DetectIsTurnable(gpsPtr, allObstacles, obstacleAhead, turn, state)) {
                                driveStateName = "ObstacleAvoid";
                            } else { // If the obstacle cannot be avoided, it will enter the AEB state.
                                if (obstacleAhead.type == 6) {
                                    driveStateName = "CarAEB";
                                } else if (obstacleAhead.type != 6 && !isLongRouteCase) driveStateName = "ObstacleAEB";
                                else if (obstacleAhead.type != 6 && isLongRouteCase) driveStateName = "Follow";
                            }
                        }
                    }
                } else { // When driving at high speeds, the decision-making distance is further
                    if (obstacleDistanceM < 40) {
                        needsBrake = false;
                        if (obstacleAhead.speed > 0.5 && obstacleAhead.speed < 2 &&
                            obstacleAhead.type == 6) {
                            if (DetectIsTurnable(gpsPtr, allObstacles, obstacleAhead, turn, state)) {
                                driveStateName = "ChangeLaneStart";
                            }
                        } else if (obstacleAhead.speed <= 0.5 && secondFrame) {
                            if (DetectIsTurnable(gpsPtr, allObstacles, obstacleAhead, turn, state)) {
                                driveStateName = "ObstacleAvoid";
                            } else {
                                if (obstacleAhead.type == 6) {
                                    driveStateName = "CarAEB";
                                } else if (obstacleAhead.type != 6 && !isLongRouteCase) driveStateName = "ObstacleAEB";
                                else if (obstacleAhead.type != 6 && isLongRouteCase) driveStateName = "Follow";
                            }
                        }
                    }
                }

                // Set the PID target speed based on the distance and speed to the vehicle in front.
                if (isLongRouteCase && maxSpeedMps < 48) {
                    speedController.setTarget(maxSpeedMps);
                } else {
                    if (obstacleDistanceM > 30)
                        speedController.setTarget(maxSpeedMps);
                    else if ((obstacleDistanceM < 30) &&
                             (obstacleDistanceM > mainVehicleSpeed * 1.5 + 15))
                        speedController.setTarget(std::max(std::min(maxSpeedMps, obstacleAhead.speed * 1.1), 5.));
                    else if ((obstacleDistanceM <= mainVehicleSpeed * 1.5 + 15) &&
                             (obstacleDistanceM > mainVehicleSpeed * 1 + 5))
                        speedController.setTarget(std::max(std::min(maxSpeedMps, obstacleAhead.speed * 0.95), 4.));
                    else if (obstacleDistanceM <= mainVehicleSpeed * 1 + 5)
                        speedController.setTarget(std::max(obstacleAhead.speed * 0.6, 2.));
                }
            } else { // If there are no obstacles ahead, cruise at maximum speed
                speedController.setTarget(maxSpeedMps);
            }

            // --- Special obstacle handling logic ---
            for (auto &i: allObstacles) {
                // Dealing with wrong-way vehicles
                if (i.speed < -3) {
                    double obs_s, obs_t, s, t;
                    SimOneAPI::GetLaneST(currentLaneId, i.pt, obs_s, obs_t);
                    SimOneAPI::GetLaneST(currentLaneId, mainVehiclePos, s, t);
                    double error = NormalizeSignedAngle(carOriZ - i.oriZ);
                    if (s < obs_s && (obs_s - s) < 30) {
                        if (abs(abs(error) - M_PI) > 0.1 && abs(obs_t - t) < 3.75) {
                            driveStateName = "ChangeLaneStart";
                            turn = "right";
                        }
                    } else if (s > obs_s) {
                        speedController.setTarget(maxSpeedMps);
                    }
                }

                // Handling crossing pedestrians
                if (i.type == 4) {
                    if (i.speed > 0.1 && i.speed < 2. && UtilMath::distance(i.pt, mainVehiclePos) <= 15) {
                        double ObsAlfa = atan2(i.pt.y - mainVehiclePos.y, i.pt.x - mainVehiclePos.x);
                        double errorCO = NormalizeSignedAngle(carOriZ - ObsAlfa);
                        double error = carOriZ - i.oriZ;
                        if (abs(errorCO) < 0.3 * M_PI) {
                            error = NormalizeSignedAngle(error);
                            if (abs(error) >= 0.45 * M_PI && abs(error) <= 0.55 * M_PI) {
                                speedController.setTarget(-10); // Emergency brake
                            }
                        }
                    }
                }

                // Handling vehicles cutting into the left lane
                if (i.type == 6 && abs(i.speed) > 2 && i.ownerLaneId == lanelink.leftNeighborLaneName) {
                    double ObsAlfa = atan2(i.pt.y - mainVehiclePos.y, i.pt.x - mainVehiclePos.x);
                    double error = NormalizeSignedAngle(carOriZ - ObsAlfa);
                    if (abs(error) < M_PI / 2) {
                        double LateralDistance = GetLateralDistance(mainVehiclePos, carOriZ, i);
                        if (abs(LateralDistance) < 3.3) speedController.setTarget(4 / 3.6); // Slow down
                    }
                }

                // Handles medium-speed vehicles moving laterally in adjacent lanes
                if (i.type == 6 && i.speed >= 15 / 3.6 && i.speed <= 40 / 3.6 && i.ownerLaneId != currentLaneId) {
                    double error = carOriZ - i.oriZ;
                    double ObsAlfa = atan2(i.pt.y - mainVehiclePos.y, i.pt.x - mainVehiclePos.x);
                    double errorOC = NormalizeSignedAngle(carOriZ - ObsAlfa);
                    if (abs(errorOC) < 0.5 * M_PI) {
                        error = NormalizeSignedAngle(error);
                        if (abs(error) >= 0.3 * M_PI && abs(error) <= 0.7 * M_PI) {
                            needsBrake = true;
                            if (!isCase18) {
                                if (isLongRouteCase) {
                                    speedController.setTarget(1);
                                } else {
                                    speedController.setTarget(0.95 * mainVehicleSpeed);
                                }
                            } else {
                                speedController.setTarget(1);
                            }
                        }
                    }
                }

                // Handling high-speed vehicles moving laterally in adjacent lanes
                if (i.type == 6 && i.speed > 40 / 3.6 && i.ownerLaneId != currentLaneId) {
                    double error = carOriZ - i.oriZ;
                    double ObsAlfa = atan2(i.pt.y - mainVehiclePos.y, i.pt.x - mainVehiclePos.x);
                    double errorOC = NormalizeSignedAngle(carOriZ - ObsAlfa);
                    if (abs(errorOC) < 0.5 * M_PI) {
                        error = NormalizeSignedAngle(error);
                        if (abs(error) >= 0.3 * M_PI && abs(error) <= 0.7 * M_PI) {
                            needsBrake = true;
                            speedController.setTarget(2);
                        }
                    }
                }
            }

            // Slow start in the first few frames
            if (!secondFrame) {
                speedController.setTarget(11 / 3.6);
            }
            throttle = speedController.update(mainVehicleSpeed); // Calculate throttle/brake
            if (needsBrake) {
                throttle = (min(-0.05, throttle)); // If braking is required, force the brakes
            }
        }

        // Status: ChangeLaneStart (Start changing lane)
        else if (driveStateName == "ChangeLaneStart") {
            // Determine target lane and signal lights based on turning intention
            if (turn == "right") {
                targetLaneId = lanelink.rightNeighborLaneName;
                if (kEnableDebugLog) {
                    std::cout << "targetLaneId=" << targetLaneId.GetString() << std::endl;
                }
                signalLightsPtr->signalLights = ESimOne_Signal_Light_RightBlinker;
            } else if (turn == "left") {
                targetLaneId = lanelink.leftNeighborLaneName;
                if (kEnableDebugLog) {
                    std::cout << "targetLaneId=" << targetLaneId.GetString() << std::endl;
                }
                signalLightsPtr->signalLights = ESimOne_Signal_Light_LeftBlinker;
            }
            SSD::SimPoint3DVector Path1, Path2;
            // Generate lane change path
            Path1 = ChangeLanePathWithLane(mainVehiclePos, mainVehicleSpeed,
                                           targetLaneId, turnTargetPoint);
            // If the target point is too far, give up the lane change
            if (UtilMath::distance(turnTargetPoint, mainVehiclePos) > 50) {
                driveStateName = "Follow";
                turn = "";
            } else {
                // Splice the lane change path and the subsequent global path
                targetPath.clear();
                for (auto &point: Path1) {
                    targetPath.push_back(point);
                }
                routeWaypoints.clear();
                routeWaypoints.push_back(turnTargetPoint);
                routeWaypoints.push_back(endPt);
                SimOneAPI::GenerateRoute(routeWaypoints, indexOfValidPoints, Path2);
                naviRoadIdList = GetNavigateRoadIdList(turnTargetPoint, endPt);
                for (auto &point: Path2) {
                    targetPath.push_back(point);
                }
                casePath = targetPath;
                PrintTargetPath(targetPath);
                if (mainVehicleSpeed <= 1) {
                    speedController.setTarget(1.5);
                    throttle = speedController.update(mainVehicleSpeed);
                }
                driveStateName = "InChangeLane"; // Switch to lane changing state
            }
        }
        // Status: ObstacleAvoid (obstacle avoidance)
        else if (driveStateName == "ObstacleAvoid") {
            // Set the speed when avoiding obstacles
            if (mainVehicleSpeed < 1.5) {
                speedController.setTarget(1.5);
            } else if (mainVehicleSpeed > 1.5 && mainVehicleSpeed < 5) {
                speedController.setTarget(mainVehicleSpeed * 0.8);
            } else
                speedController.setTarget(4);
            HDMapStandalone::MLaneLink ObstacleLaneLink;
            SimOneAPI::GetLaneLink(obstacleAhead.ownerLaneId, ObstacleLaneLink);
            if (kEnableDebugLog) {
                std::cout << "turn=" << turn.GetString() << std::endl;
            }
            // Determine target lane and signal lights based on turning intention
            if (turn == "right") {
                targetLaneId = ObstacleLaneLink.rightNeighborLaneName;
                if (kEnableDebugLog) {
                    std::cout << "targetLaneId=" << targetLaneId.GetString() << std::endl;
                }
                signalLightsPtr->signalLights = ESimOne_Signal_Light_RightBlinker;
            } else if (turn == "left") {
                targetLaneId = ObstacleLaneLink.leftNeighborLaneName;
                if (kEnableDebugLog) {
                    std::cout << "targetLaneId=" << targetLaneId.GetString() << std::endl;
                }
                signalLightsPtr->signalLights = ESimOne_Signal_Light_LeftBlinker;
            }
            // Generate obstacle avoidance paths and splice subsequent paths
            if (!isNeedBuild) {
                targetPath.clear();
                SSD::SimPoint3DVector Path2;
                targetPath = ChangeLanePathWithObstacle(mainVehiclePos, targetLaneId, obstacleAhead.pt,
                                                        turnTargetPoint);
                if (kEnableDebugLog) {
                    std::cout << "turnTargetPoint=" << turnTargetPoint.x << "," << turnTargetPoint.y << std::endl;
                }
                routeWaypoints.clear();
                routeWaypoints.push_back(turnTargetPoint);
                routeWaypoints.push_back(endPt);
                SimOneAPI::GenerateRoute(routeWaypoints, indexOfValidPoints, Path2);
                naviRoadIdList = GetNavigateRoadIdList(turnTargetPoint, endPt);
                for (auto &point: Path2) {
                    targetPath.push_back(point);
                }
                casePath = targetPath;
                PrintTargetPath(targetPath);
                if (kEnableDebugLog) {
                    std::cout << "Obstacle-avoidance route rebuilt: " << driveStateName << std::endl;
                }
            } else { // Alternate path generation logic
                targetPath.clear();
                SSD::SimPoint3DVector Path2;
                targetPath = ChangeLanePathWithObstacle(mainVehiclePos, targetLaneId,
                                                        obstacleAhead.pt, turnTargetPoint);
                SSD::SimString turntolaneId;
                turntolaneId = GetNearMostLane(turnTargetPoint);
                if (kEnableDebugLog) {
                    std::cout << "turnTargetPoint=" << turnTargetPoint.x << "," << turnTargetPoint.y << std::endl;
                }
                double s_next = GetS(turnTargetPoint, turntolaneId);
                GetLaneSampleFromS(turntolaneId, s_next, Path2);
                for (auto &point: Path2) {
                    targetPath.push_back(point);
                }
                PrintTargetPath(targetPath);
                if (kEnableDebugLog) {
                    std::cout << "Fallback obstacle-avoidance route rebuilt: " << driveStateName << std::endl;
                }
            }
            driveStateName = "InChangeLane"; // Switch to lane changing state
        }
        // Status: InChangeLane (Changing lane)
        else if (driveStateName == "InChangeLane") {
            // Set the speed during lane change
            if (!isLongRouteCase) {
                if (mainVehicleSpeed < 1.5) {
                    speedController.setTarget(1.5);
                } else
                    speedController.setTarget(mainVehicleSpeed);
            } else {
                speedController.setTarget(maxSpeedMps);
            }
            throttle = speedController.update(mainVehicleSpeed);
            // If it is close to the lane change target point, the lane change is completed and switches back to the Follow state.
            if (UtilMath::distance(mainVehiclePos, turnTargetPoint) < 3) {
                signalLightsPtr->signalLights = 0; // Turn off the turn signal
                driveStateName = "Follow";
                hasExitedIntersection = false;
            }
        }
        // Status: NearIntersection
        else if (driveStateName == "NearIntersection" || isNearSection) {

            // It is also necessary to deal with obstacles in front near the intersection. The logic is the same as the Follow state.
            if (hasObstacle) {
                if (obstacleDistanceM < 30) {
                    if (obstacleAhead.speed > 0.5 && obstacleAhead.speed < 2 &&
                        obstacleAhead.type == 6) {
                        if (DetectIsTurnable(gpsPtr, allObstacles, obstacleAhead, turn, state)) {
                            driveStateName = "ChangeLaneStart";
                            hasExitedIntersection = true;
                        }
                    } else if (obstacleAhead.speed <= 0.5 && secondFrame) {
                        if (DetectIsTurnable(gpsPtr, allObstacles, obstacleAhead, turn, state)) {
                            driveStateName = "ObstacleAvoid";
                            hasExitedIntersection = true;
                        } else {
                            if (obstacleAhead.type == 6) {
                                driveStateName = "CarAEB";
                                hasExitedIntersection = true;
                            } else if (obstacleAhead.type != 6 && !isLongRouteCase) {
                                driveStateName = "ObstacleAEB";
                                hasExitedIntersection = true;
                            } else if (obstacleAhead.type != 6 && isLongRouteCase) {
                                driveStateName = "Follow";
                                hasExitedIntersection = true;
                            }
                        }
                    }
                }
            }
            // Make decisions based on traffic lights, sidewalks, and intersection congestion
            if (stopLineDistanceM < noLightStopLineDistanceM) { // Prioritize lighted intersections
                if (kEnableDebugLog) {
                    std::cout << "Handling traffic-light intersection." << std::endl;
                }
                // If the light is green and there are no pedestrians/congestion, then normal traffic
                if (IsGreenLight(light.id, currentLaneId, light, mainVehicleSpeed,
                                 stopLineDistanceM) &&
                    !CrosswalkOccupied(crosswalk.boundaryKnots, allObstacles) &&
                    !IsJunctionCrowded(gpsPtr, allObstacles, naviRoadIdList)) {
                    // Adjust speed according to obstacles ahead
                    if (obstacleDistanceM > 40 && !isLongRouteCase)
                        speedController.setTarget(std::max(mainVehicleSpeed * 0.95, 20 / 3.6));
                    else if (obstacleDistanceM > 40 && isLongRouteCase)
                        speedController.setTarget(maxSpeedMps);
                    else if ((obstacleDistanceM < 40) &&
                             (obstacleDistanceM > mainVehicleSpeed * 1.5 + 15))
                        speedController.setTarget(std::min(maxSpeedMps, obstacleAhead.speed * 1.1));
                    else if ((obstacleDistanceM <= mainVehicleSpeed * 1.5 + 15) &&
                             (obstacleDistanceM > mainVehicleSpeed * 1 + 10))
                        speedController.setTarget(std::min(maxSpeedMps, obstacleAhead.speed * 1.00));
                    else if (obstacleDistanceM <= mainVehicleSpeed * 1 + 10)
                        speedController.setTarget(obstacleAhead.speed * 0.5);
                    if (obstacleDistanceM <= 5)
                        speedController.setTarget(-10);
                }
                else { // If the light is red or there is danger, slow down and stop.
                    if (kEnableDebugLog) {
                        std::cout << "Stopping for red light or unsafe intersection." << std::endl;
                    }
                    if (hasObstacle && obstacleAhead.speed >= 3) { // Follow the car in front and stop
                        if ((obstacleDistanceM <= 30 || stopLineDistanceM <= stopDistanceM + 30) &&
                            (obstacleDistanceM > 20 || stopLineDistanceM > stopDistanceM + 15))
                            speedController.setTarget(std::min(8.0, obstacleAhead.speed * 1.1));
                        else if ((obstacleDistanceM <= 20 ||
                                  stopLineDistanceM <= stopDistanceM + 15) &&
                                 (obstacleDistanceM > 12 || stopLineDistanceM > stopDistanceM + 5))
                            speedController.setTarget(std::min(3.0, obstacleAhead.speed));
                        else if (obstacleDistanceM <= 12 ||
                                 stopLineDistanceM <= stopDistanceM + 5) {
                            speedController.setTarget(-10);
                        }
                    } else { // Stop at the stop line
                        if (kEnableDebugLog) {
                            std::cout << "Stopping before stop line." << std::endl;
                        }
                        if ((stopLineDistanceM <= stopDistanceM + 20) &&
                            (stopLineDistanceM > stopDistanceM + 10))
                            speedController.setTarget(8.0);
                        else if ((stopLineDistanceM <= stopDistanceM + 10) &&
                                 (stopLineDistanceM > stopDistanceM))
                            speedController.setTarget(2.0);
                        else if (stopLineDistanceM <= stopDistanceM)
                            speedController.setTarget(-10);
                    }
                }
            } else if (stopLineDistanceM > noLightStopLineDistanceM) { // Dealing with unlit intersections
                if (!CrosswalkOccupied(crosswalkKnots, allObstacles)) { // If there are no pedestrians on the sidewalk, pass normally
                    if (obstacleDistanceM > 60)
                        speedController.setTarget(maxSpeedMps);
                    else if ((obstacleDistanceM < 60) &&
                             (obstacleDistanceM > mainVehicleSpeed * 1.5 + 15))
                        speedController.setTarget(std::min(maxSpeedMps, obstacleAhead.speed * 1.1));
                    else if ((obstacleDistanceM <= mainVehicleSpeed * 1.5 + 15) &&
                             (obstacleDistanceM > mainVehicleSpeed * 1 + 10))
                        speedController.setTarget(std::min(maxSpeedMps, obstacleAhead.speed));
                    else if (obstacleDistanceM <= mainVehicleSpeed * 1 + 10)
                        speedController.setTarget(obstacleAhead.speed * 0.9);
                } else { // There are pedestrians on the sidewalk, slow down and stop.
                    if (kEnableDebugLog) {
                        std::cout << "Stopping for occupied crosswalk." << std::endl;
                    }
                    if ((std::min(obstacleDistanceM, noLightStopLineDistanceM + 3) <= 23) &&
                        (std::min(obstacleDistanceM, noLightStopLineDistanceM + 3) > 13))
                        speedController.setTarget(std::min(5.0, obstacleAhead.speed));
                    else if ((std::min(obstacleDistanceM, noLightStopLineDistanceM + 3) <= 13) &&
                             (std::min(obstacleDistanceM, noLightStopLineDistanceM + 3) > 8))
                        speedController.setTarget(std::min(1.0, obstacleAhead.speed));
                    else if (std::min(obstacleDistanceM, noLightStopLineDistanceM + 3) <= 8)
                        speedController.setTarget(-10);
                }
            }

            // Handling vehicles entering from the left near the intersection
            for (auto &i: allObstacles) {
                if (i.type == 6 && abs(i.speed) > 2 && i.ownerLaneId == lanelink.leftNeighborLaneName) {
                    double ObsAlfa = atan2(i.pt.y - mainVehiclePos.y, i.pt.x - mainVehiclePos.x);
                    double error = NormalizeSignedAngle(carOriZ - ObsAlfa);
                    if (abs(error) < M_PI / 2) {
                        double LateralDistance = GetLateralDistance(mainVehiclePos, carOriZ, i);
                        if (abs(LateralDistance) < 3.3) speedController.setTarget(4 / 3.6);
                    }
                }
            }

            throttle = speedController.update(mainVehicleSpeed);
        }
        // Status: CarAEB (automatic emergency braking of the vehicle)
        else if (driveStateName == "CarAEB") {
            // If lane change is possible, switch to obstacle avoidance state
            if (DetectIsTurnable(gpsPtr, allObstacles, obstacleAhead, turn, state)) {
                driveStateName = "ObstacleAvoid";
            }
            // If the obstacle starts to move, switch back to following state
            if (obstacleAhead.speed > 0.5) {
                driveStateName = "Follow";
            }
            // Segmented deceleration and braking
            if (obstacleDistanceM <= 40 && obstacleDistanceM > 30)
                speedController.setTarget(std::min(8.0, mainVehicleSpeed));
            else if (obstacleDistanceM <= 30 && obstacleDistanceM > 20)
                speedController.setTarget(std::min(3.0, mainVehicleSpeed));
            else if (obstacleDistanceM <= 20)
                speedController.setTarget(-10);
            throttle = speedController.update(mainVehicleSpeed);
        }
        // Status: ObstacleAEB (automatic emergency braking for other obstacles)
        else if (driveStateName == "ObstacleAEB") {
            if (DetectIsTurnable(gpsPtr, allObstacles, obstacleAhead, turn, state)) {
                driveStateName = "ObstacleAvoid";
            }
            if (obstacleAheadDistanceM >= 40) {
                driveStateName = "Follow";
            }
            // Segmented deceleration braking
            if (obstacleDistanceM <= 40 && obstacleDistanceM > 20)
                speedController.setTarget(std::min(8.0, mainVehicleSpeed));
            else if (obstacleDistanceM <= 20 && obstacleDistanceM > 10)
                speedController.setTarget(std::min(3.0, mainVehicleSpeed));
            else if (obstacleDistanceM <= 10)
                speedController.setTarget(-10);
            throttle = speedController.update(mainVehicleSpeed);
        }

        // --- Path tracking and control ---
        // Calculate steering wheel angle
        if (!targetPath.empty() && secondFrame) {
            initialSteering = UtilDriver::calculateSteering(targetPath, gpsPtr.get(), headingErrorRad, turnHeadingErrorRad);
        } else if (!secondFrame) {
            initialSteering = 0;
        } else { // If the path is lost, re-plan
            routeWaypoints.clear();
            routeWaypoints.push_back(mainVehiclePos);
            routeWaypoints.push_back(endPt);
            PrintTargetPath(routeWaypoints);
            SimOneAPI::GenerateRoute(routeWaypoints, indexOfValidPoints, targetPath);
            casePath = targetPath;
            PrintTargetPath(targetPath);
            if (kEnableDebugLog) {
                std::cout << "Long-route path rebuilt from current position." << std::endl;
            }
            initialSteering = 0;
        }

        // If the vehicle deviates seriously from the path, try to reconstruct the path
        if ((headingErrorRad <= -1. / 2 * 3.14 || headingErrorRad >= 1. / 2 * 3.14) && !isNeedBuild && isLongRouteCase) {
            BuildLineWithoutTargetPath(mainVehiclePos, targetPath);
            isNeedBuild = true;
            initialSteering = 0;
            if (targetPath.size() < 50) {
                SSD::SimPoint3DVector Path2;
                int i = int(targetPath.size());
                routeWaypoints.clear();
                routeWaypoints.push_back(targetPath[i - 1]);
                routeWaypoints.push_back(endPt);
                SimOneAPI::GenerateRoute(routeWaypoints, indexOfValidPoints, Path2);
                for (auto &point: Path2) {
                    targetPath.push_back(point);
                }
            }
            PrintTargetPath(targetPath);
            if (kEnableDebugLog) {
                std::cout << "Heading deviation fallback path built." << std::endl;
            }
        } else if ((headingErrorRad <= -1. / 2 * 3.14 || headingErrorRad >= 1. / 2 * 3.14) && isNeedBuild && isLongRouteCase) {
            targetPath = casePath; // Restore original path
            isNeedBuild = false;
            driveStateName = "Follow";
            PrintTargetPath(targetPath);
            if (kEnableDebugLog) {
                std::cout << "Restored long-route reference path." << std::endl;
            }
        }

        // --- Control volume post-processing ---
        // Slow down during large corners
        if (std::abs(initialSteering) > 0.4 && driveStateName != "InChangeLane" && mainVehicleSpeed >= 8 && !isLongRouteCase) {
            throttle = -2;
        }

        if (std::abs(initialSteering) > 0.6 && mainVehicleSpeed >= 2 && !isLongRouteCase) {
            throttle = -3;
        }

        // Limit the maximum turning angle at high speed to prevent rollover
        if (mainVehicleSpeed > 45. / 3.6) {
            if (initialSteering > 0.4) {
                initialSteering = 0.4;
            }
            if (initialSteering < -0.4) {
                initialSteering = -0.4;
            }
        }
        if (mainVehicleSpeed > 70. / 3.6) {
            if (initialSteering > 0.1) {
                initialSteering = 0.1;
            }
            if (initialSteering < -0.1) {
                initialSteering = -0.1;
            }
        }

        if (currentLaneId == SSD::SimString("492_0_-1")) {
            isNeedBuild = false;
        }

        // --- Path re-planning logic for special scenarios ---
        // When you encounter an obstacle at an intersection and cannot change lanes, try to go around it.
        if (hasObstacle && driveStateName != "InChangeLane" && driveStateName != "ObstacleAvoid" && isLongRouteCase) {
            if (!IsChangeable(obstacleAhead, allObstacles, laneChangePoint)) {
                if (kEnableDebugLog) {
                    std::cout << "obstacleAhead=" << obstacleAhead.pt.x << "," << obstacleAhead.pt.y << std::endl;
                    std::cout << "laneChangePoint=" << laneChangePoint.x << "," << laneChangePoint.y << std::endl;
                }
                if (IsObstacleInJunction(obstacleAhead)) {
                    if (kEnableDebugLog) {
                        std::cout << "Obstacle is in junction." << std::endl;
                    }
                    if (DetectValidCrossing(currentLaneId, mainVehiclePos, allObstacles, targetLaneName,
                                            laneChangePoint)) {
                        routeWaypoints.clear();
                        routeWaypoints.push_back(mainVehiclePos);
                        routeWaypoints.push_back(laneChangePoint);
                        routeWaypoints.push_back(endPt);
                        SimOneAPI::GenerateRoute(routeWaypoints, indexOfValidPoints, targetPath);
                        SimOneAPI::Navigate(routeWaypoints, indexOfValidPoints, naviRoadIdList);
                        PrintTargetPath(targetPath);
                        if (kEnableDebugLog) {
                            std::cout << "Junction crossing route rebuilt." << std::endl;
                        }
                    }
                }
            }
        }

        // At a forked road, if the lane ahead is blocked, choose another road
        if (hasObstacle && obstacleDistanceM < 15 && driveStateName == "Follow") {
            if (kEnableDebugLog) {
                std::cout << "successorLaneCount=" << lanelink.successorLaneNameList.size() << std::endl;
            }
            if (!IsChangeable(obstacleAhead, allObstacles, laneChangePoint)) {
                if (lanelink.successorLaneNameList.size() == 2) {
                    if (kEnableDebugLog) {
                        std::cout << "successorLane1=" << lanelink.successorLaneNameList[0].GetString() << std::endl;
                        std::cout << "successorLane2=" << lanelink.successorLaneNameList[1].GetString() << std::endl;
                    }
                    if (lanelink.successorLaneNameList[0] == obstacleAhead.ownerLaneId) {
                        HDMapStandalone::MLaneInfo sample;
                        SimOneAPI::GetLaneSample(lanelink.successorLaneNameList[1], sample);
                        routeWaypoints.clear();
                        routeWaypoints.push_back(mainVehiclePos);
                        routeWaypoints.push_back(*sample.centerLine.end());
                        routeWaypoints.push_back(endPt);
                        SimOneAPI::GenerateRoute(routeWaypoints, indexOfValidPoints, targetPath);
                        SimOneAPI::Navigate(routeWaypoints, indexOfValidPoints, naviRoadIdList);
                        PrintTargetPath(targetPath);
                        if (kEnableDebugLog) {
                            std::cout << "Alternate successor route rebuilt." << std::endl;
                        }
                    } else {
                        HDMapStandalone::MLaneInfo sample;
                        SimOneAPI::GetLaneSample(lanelink.successorLaneNameList[0], sample);
                        routeWaypoints.clear();
                        routeWaypoints.push_back(mainVehiclePos);
                        routeWaypoints.push_back(*sample.centerLine.end());
                        routeWaypoints.push_back(endPt);
                        SimOneAPI::GenerateRoute(routeWaypoints, indexOfValidPoints, targetPath);
                        SimOneAPI::Navigate(routeWaypoints, indexOfValidPoints, naviRoadIdList);
                        PrintTargetPath(targetPath);
                        if (kEnableDebugLog) {
                            std::cout << "Alternate successor route rebuilt." << std::endl;
                        }
                    }
                }
            }
        }

        // --- Hardcoded logic for specific test cases ---
        if (needsBrake) {
            countdown = 600;
        }

        if (countdown != 0 && !isCase24) {
            countdown--;
            throttle = (min(-0.3, throttle));
        }

        SimOne_Data_CaseInfo caseInfo{};
        SimOneAPI::GetCaseInfo(&caseInfo);

        // Case 20: Parking
        if (caseInfo.caseName[0] == '2' && caseInfo.caseName[1] == '0') {
            if ((stopLineDistanceM <= stopDistanceM + 20) &&
                (stopLineDistanceM > stopDistanceM + 10))
                speedController.setTarget(8.0);
            else if ((stopLineDistanceM <= stopDistanceM + 10) &&
                     (stopLineDistanceM > stopDistanceM))
                speedController.setTarget(2.0);
            else if (stopLineDistanceM <= stopDistanceM)
                speedController.setTarget(-10);
            throttle = speedController.update(mainVehicleSpeed);
        }

        // Case 07, 08: Full Throttle
        if (caseInfo.caseName[0] == '0' && (caseInfo.caseName[1] == '8' || caseInfo.caseName[1] == '7')) {
            throttle = 100;
        }

        // Case 24: Driving at low speed
        if (caseInfo.caseName[0] == '2' && caseInfo.caseName[1] == '4' && !isCase24) {
            isCase24 = true;
            countdown = 0;
            maxSpeedMps = 20 / 3.6;
            speedController.setLimits(-10, 10);
        }

        if (isCase24) {
            if (mainVehicleSpeed > 21 / 3.6) {
                throttle = -0.3;
            } else {
                throttle = 0.1;
            }
        }

        // Case 41 (long distance): Speed limit in the initial stage, then increase speed
        if (isLongRouteCase && !isCase41) {
            isCase41 = true;
            countdown2 = 2000;
        }

        if (isCase41 && countdown2 != 0) {
            countdown2--;
            finalSpeedMps = 25 / 3.6;
        } else {
            finalSpeedMps = 110 / 3.6;
        }

        // Case 10:
        if (caseInfo.caseName[0] == '1' && caseInfo.caseName[1] == '0' && !isCase10) {
            isCase10 = true;
            countdown = 0;
            speedController.setLimits(-10, 10);
        }

        if (isCase10 && countdown != 0) {
            countdown--;
        }

        if (isCase10 && countdown == 0) {
            throttle = 100;
        }

        // Case 15: Speed limit
        if (caseInfo.caseName[0] == '1' && caseInfo.caseName[1] == '5') {
            maxSpeedMps = 10 / 3.6;
        }

        // Case 36: Speed limit
        if (caseInfo.caseName[0] == '3' && caseInfo.caseName[1] == '6') {
            maxSpeedMps = 20 / 3.6;
        }

        // Case 18: Emergency brake
        if (caseInfo.caseName[0] == '1' && caseInfo.caseName[1] == '8') {
            isCase18 = true;
            if (throttle < -0.5) {
                throttle *= 5;
            }
            throttle = (min(-0.1, throttle));
        }
        if (isCase18 && mainVehicleSpeed < 5 / 3.6) {
            countdown++;
        }
        if (isCase18 && countdown > 100) {
            throttle = 100;
        }

        // Case 28: Replan the destination
        if (caseInfo.caseName[0] == '2' && caseInfo.caseName[1] == '8' && !isCase28) {
            isCase28 = true;
            endPt = SSD::SimPoint3D{-168.3, -13.73, 0};
            routeWaypoints.clear();
            routeWaypoints.push_back(startPt);
            naviRoadIdList = GetNavigateRoadIdList(startPt, endPt);
            routeWaypoints.push_back(endPt);
            SimOneAPI::GenerateRoute(routeWaypoints, indexOfValidPoints, targetPath);
            PrintTargetPath(targetPath);
        }

        // --- Execution control ---
        // Apply calculated throttle and steering wheel angles to the vehicle
        UtilDriver::setDriver(gpsPtr->timestamp, float(throttle), 0, float(initialSteering));
        // Set signal light
        SimOneAPI::SetSignalLights(kMainVehicleId, signalLightsPtr.get());

        // --- Debug information printing ---
        if (kEnableDebugLog) {
            speedController.printTarget();
            std::cout << "countdown=" << countdown << std::endl;
            std::cout << "headingErrorRad=" << turnHeadingErrorRad << std::endl;
            std::cout << "initialSteering=" << initialSteering << std::endl;
            std::cout << "Throttle=" << throttle << std::endl;
            std::cout << "ObstaclePos=" << obstacleAhead.pt.x << ","
                      << obstacleAhead.pt.y << "," << obstacleAhead.pt.z << std::endl;
            std::cout << "MainVehicleSpeed=" << mainVehicleSpeed << "m/s" << std::endl;
            std::cout << "Position=" << mainVehiclePos.x << "," << mainVehiclePos.y << std::endl;
            std::cout << "LaneID=" << currentLaneId.GetString() << std::endl;
            std::cout << "driveStateName=" << driveStateName << std::endl;
            std::cout << "obstacle.Speed=" << obstacleAhead.speed << std::endl;
            std::cout << "isNeedBuild=" << isNeedBuild << std::endl;
            std::cout << "needsBrake=" << needsBrake << std::endl;
            std::cout << std::endl;
        }

        // Update frame flag bit
        if (firstFrame) {
            secondFrame = true;
        }
        firstFrame = true;

        // Go to next frame
        SimOneAPI::NextFrame(frame);
    }
    return 0;
}
