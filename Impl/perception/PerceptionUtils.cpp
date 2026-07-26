#if defined(__clang__)
#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Wsign-compare"
#pragma clang diagnostic ignored "-Wunknown-pragmas"
#pragma clang diagnostic ignored "-Wreturn-type-c-linkage"
#endif

#include "../common/PerceptionUtils.h"
#include <cmath>
#include <iostream>
#include "SimOneSensorAPI.h"
#include "prediction.h"
#include "../util/GetSignType.h"
#include "../util/UtilMath.h"
#include "../common/PathUtils.h"
#include "../common/DecisionUtils.h"

#if defined(__clang__)
#pragma clang diagnostic pop
#endif

namespace {

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

/**
 * @brief Gets the information list of all obstacles from the simulation environment.
 * @param gps GPS data of the main vehicle, used to obtain the direction of the main vehicle to correct the speed and direction of obstacles.
 * @return A vector containing all obstacle information.
 */
std::vector<obstaclestruct> GetObstacleList(const std::unique_ptr<SimOne_Data_Gps> &gps) {
    std::vector<obstaclestruct> allObstacles;
    double carOriZ = gps->oriZ;
    std::unique_ptr<SimOne_Data_Obstacle> obstaclesPtr = std::make_unique<SimOne_Data_Obstacle>();
    if (SimOneAPI::GetGroundTruth(0, obstaclesPtr.get())) {
        for (int i = 0; i < obstaclesPtr->obstacleSize; i++) {
            obstaclestruct obs;
            float posX = obstaclesPtr->obstacle[i].posX;
            float posY = obstaclesPtr->obstacle[i].posY;
            float posZ = obstaclesPtr->obstacle[i].posZ;
            SSD::SimPoint3D pos(posX, posY, posZ);
            obs.pt = pos;
            SSD::SimString laneId = GetNearMostLane(pos);
            obs.ownerLaneId = laneId;
            obs.type = obstaclesPtr->obstacle[i].type;
            obs.id = obstaclesPtr->obstacle[i].id;
            obs.speed = UtilMath::calculateSpeed(obstaclesPtr->obstacle[i].velX,
                                                 obstaclesPtr->obstacle[i].velY,
                                                 obstaclesPtr->obstacle[i].velZ);
            obs.width = obstaclesPtr->obstacle[i].width;

            obs.oriZ = obstaclesPtr->obstacle[i].oriZ;
            double headingErrorRad = NormalizeSignedAngle(carOriZ - obs.oriZ);
            if (abs(headingErrorRad) > 0.75 * M_PI) {
                obs.speed *= -1;
            }
            allObstacles.push_back(obs);
        }
    }
    return allObstacles;
}

/**
 * @brief Screen the obstacles in front and side according to the direction of the main vehicle.
 * @param gps GPS data of the main vehicle.
 * @param stationaryObstacles Receives a reference to filtered stationary obstacles.
 * @return Vector containing dynamic obstacles ahead and to the sides.
 */
std::vector<obstaclestruct>
GetObstaclesByOrientation(const std::unique_ptr<SimOne_Data_Gps> &gps, std::vector<obstaclestruct> &stationaryObstacles) {
    std::vector<obstaclestruct> allObstacles_ahead = {};
    double carOriZ = gps->oriZ;
    float PosX = gps->posX;
    float PosY = gps->posY;
    float PosZ = gps->posZ;
    SSD::SimPoint3D M_Pt(PosX, PosY, PosZ);
    SSD::SimString M_LaneId = GetNearMostLane(M_Pt);
    std::unique_ptr<SimOne_Data_Obstacle> obstaclesPtr = std::make_unique<SimOne_Data_Obstacle>();
    if (SimOneAPI::GetGroundTruth(0, obstaclesPtr.get())) {
        for (int i = 0; i < obstaclesPtr->obstacleSize; i++) {
            obstaclestruct obs;
            float obsPosX = obstaclesPtr->obstacle[i].posX;
            float obsPosY = obstaclesPtr->obstacle[i].posY;
            float obsPosZ = obstaclesPtr->obstacle[i].posZ;
            (void)obsPosZ;
            SSD::SimPoint3D obs_Pt(PosX, PosY, PosZ);
            SSD::SimString obs_LaneId = GetNearMostLane(obs_Pt);
            double M_O_Distance = UtilMath::distance(M_Pt, obs_Pt);
            double arccos_right = std::acos((obsPosX - PosX) / M_O_Distance);
            double cos_left = (PosX - obsPosX) / M_O_Distance;
            double arccos_up = std::acos((obsPosY - PosY) / M_O_Distance);
            double arccos_down = std::acos((PosY - obsPosY) / M_O_Distance);
            obs.pt = obs_Pt;
            obs.ownerLaneId = obs_LaneId;
            obs.type = obstaclesPtr->obstacle[i].type;
            obs.id = obstaclesPtr->obstacle[i].id;
            obs.speed = UtilMath::calculateSpeed(obstaclesPtr->obstacle[i].velX, obstaclesPtr->obstacle[i].velY,
                                                 obstaclesPtr->obstacle[i].velZ);
            obs.width = obstaclesPtr->obstacle[i].width;
            obs.oriZ = obstaclesPtr->obstacle[i].oriZ;
            if (carOriZ <= (M_PI / 4) && carOriZ > (-M_PI / 4)) {
                if (M_LaneId == obs_LaneId || abs(arccos_right - abs(carOriZ)) < 0.01)
                {
                    if (obsPosX > PosX)
                        allObstacles_ahead.push_back(obs);
                } else if (obs.speed != 0) {
                    if (obsPosY > PosY && obs.oriZ <= 0 && obs.oriZ > -M_PI)
                        allObstacles_ahead.push_back(obs);
                    else if (obsPosY < PosY && obs.oriZ >= 0 && obs.oriZ < M_PI)
                        allObstacles_ahead.push_back(obs);
                } else
                    stationaryObstacles.push_back(obs);
            }
            else if (carOriZ <= (3 * M_PI / 4) && carOriZ > (M_PI / 4)) {
                if (M_LaneId == obs_LaneId ||
                    abs(arccos_up - abs(carOriZ - M_PI / 2)) < 0.01)
                {
                    if (obsPosY > PosY)
                        allObstacles_ahead.push_back(obs);
                } else if (obs.speed != 0) {
                    if (obsPosX > PosX) {
                        if ((obs.oriZ >= M_PI / 2 && obs.oriZ <= M_PI) || (obs.oriZ < -M_PI / 2 && obs.oriZ > -M_PI))
                            allObstacles_ahead.push_back(obs);
                        else if (obsPosX < PosX && obs.oriZ > -M_PI / 2 && obs.oriZ <= M_PI / 2)
                            allObstacles_ahead.push_back(obs);
                    }
                } else
                    stationaryObstacles.push_back(obs);
            }
            else if ((carOriZ >= 3 * M_PI / 4 && carOriZ <= M_PI) || (carOriZ <= -3 * M_PI / 4 && carOriZ > -M_PI)) {
                if (M_LaneId == obs_LaneId ||
                    cos_left == abs(std::cos(carOriZ)) < 0.01)
                {
                    if (obsPosX < PosX)
                        allObstacles_ahead.push_back(obs);
                } else if (obs.speed != 0) {
                    if (obsPosY > PosY && obs.oriZ < 0 && obs.oriZ >= -M_PI)
                        allObstacles_ahead.push_back(obs);
                    else if (obsPosY < PosY && obs.oriZ > 0 && obs.oriZ <= M_PI)
                        allObstacles_ahead.push_back(obs);
                } else
                    stationaryObstacles.push_back(obs);
            }
            else if (carOriZ >= (-3 * M_PI / 4) && carOriZ < (-M_PI / 4)) {
                if (M_LaneId == obs_LaneId ||
                    abs(arccos_down - abs(carOriZ + M_PI / 2)) < 0.01)
                {
                    if (obsPosY < PosY)
                        allObstacles_ahead.push_back(obs);
                } else if (obs.speed != 0) {
                    if (obsPosX < PosX) {
                        if ((obs.oriZ > M_PI / 2 && obs.oriZ <= M_PI) || (obs.oriZ <= -M_PI / 2 && obs.oriZ > -M_PI))
                            allObstacles_ahead.push_back(obs);
                        else if (obsPosX > PosX && obs.oriZ >= -M_PI / 2 && obs.oriZ < M_PI / 2)
                            allObstacles_ahead.push_back(obs);
                    }
                } else
                    stationaryObstacles.push_back(obs);
            }
        }
    }
    return allObstacles_ahead;
}

/**
 * @brief Get the valid obstacles within a certain range in front of the main vehicle.
 * @param gps GPS data of the main vehicle.
 * @param lane The current lane ID of the main vehicle.
 * @param validObstacleList Receives a vector reference of valid obstacles.
 * @return Returns true if a valid obstacle is found, otherwise returns false.
 */
bool GetValidObstacles(const std::unique_ptr<SimOne_Data_Gps> &gps, SSD::SimString &lane,
                      std::vector<obstaclestruct> &validObstacleList) {
    double carOriZ = gps->oriZ;
    SSD::SimPoint3D carPos{gps->posX, gps->posY, gps->posZ};
    std::vector<obstaclestruct> obstacleList = GetObstacleList(gps);
    for (auto &obs: obstacleList) {
        double angle = atan2(obs.pt.y - gps->posY, obs.pt.x - gps->posX);
        double error = NormalizeSignedAngle(carOriZ - angle);

        if ((abs(error) <= M_PI / 4 || obs.ownerLaneId == lane || UtilMath::distance(carPos, obs.pt) < 25) &&
            UtilMath::distance(carPos, obs.pt) < 100) {
            validObstacleList.push_back(obs);
        }
    }
    if (validObstacleList.empty()) {
        return false;
    }
    return true;
}

/**
 * @brief Find the index of the nearest obstacle from the obstacle list.
 * @param mainVehiclePos Main vehicle position.
 * @param mainVehiclespeed Main vehicle speed.
 * @param obstacleList obstacle list.
 * @param currentLaneId The current lane ID of the main vehicle.
 * @return the index of the nearest obstacle in the list, or -1 if not found.
 */
int GetNearObstacleIndex(const SSD::SimPoint3D &mainVehiclePos,
                         const double &mainVehiclespeed,
                         const std::vector<obstaclestruct> &obstacleList,
                         const SSD::SimString &currentLaneId) {
    int obstacleClosestIndex = -1;
    SSD::SimString mainVehicleLane = GetNearMostLane(mainVehiclePos);
    double minDist = std::numeric_limits<double>::max();
    SSD::SimPoint3D vehiclePos3D(mainVehiclePos.x, mainVehiclePos.y,
                                 mainVehiclePos.z);
    HDMapStandalone::MLaneLink lanelink;
    SimOneAPI::GetLaneLink(currentLaneId, lanelink);
    for (unsigned int i = 0; i < obstacleList.size(); i++) {

        auto &obstacle = obstacleList[i];
        double distanceSign = UtilMath::distance(
                vehiclePos3D,
                SSD::SimPoint3D(obstacle.pt.x, obstacle.pt.y, obstacle.pt.z));
        if (GetS(mainVehiclePos, mainVehicleLane) >
            GetS(
                    obstacle.pt,
                    mainVehicleLane))
        {
            if (obstacle.speed > mainVehiclespeed + 2 && distanceSign < minDist &&
                (obstacle.ownerLaneId == currentLaneId ||
                 count(lanelink.predecessorLaneNameList.begin(),
                       lanelink.predecessorLaneNameList.end(),
                       obstacle.ownerLaneId))) {
                minDist = distanceSign;
                obstacleClosestIndex = i;
            }
        } else
        {
            if (distanceSign < minDist &&
                obstacle.ownerLaneId != lanelink.leftNeighborLaneName &&
                obstacle.ownerLaneId != lanelink.rightNeighborLaneName) {
                minDist = distanceSign;
                obstacleClosestIndex = i;
            }
        }
    }
    return obstacleClosestIndex;
}

/**
 * @brief Detect obstacles in the specified area ahead.
 * @param gps GPS data of the main vehicle.
 * @param obstacle A reference to receive the detected obstacle.
 * @param currentDistance A reference to receive the distance to the detected obstacle.
 * @return Returns true if an obstacle is detected, otherwise returns false.
 */
bool DetectObstacleAheadInZone(const std::unique_ptr<SimOne_Data_Gps> &gps,
                                     obstaclestruct &obstacle,
                                     double &currentDistance) {
    SSD::SimPoint3D mainVehiclePos(gps->posX, gps->posY, gps->posZ);
    std::vector<obstaclestruct> allObstacles = GetObstacleList(gps);
    double minDist = std::numeric_limits<double>::max();
    int obstacleClosestIndex = -1;
    SSD::SimPoint3DVector detectZone =
            GetDetectionZoneRealtime(gps, 25, 0, -1.5, 1.5);

    for (unsigned int i = 0; i < allObstacles.size(); i++) {
        auto &curObstacle = allObstacles[i];
        double distanceSign = UtilMath::distance(
                mainVehiclePos,
                SSD::SimPoint3D(curObstacle.pt.x, curObstacle.pt.y, curObstacle.pt.z));

        if (IsOccupied(curObstacle.pt, detectZone) && distanceSign < minDist) {
            minDist = distanceSign;
            obstacleClosestIndex = i;
        }
    }

    if (obstacleClosestIndex == -1) {
        return false;
    }

    obstacle = allObstacles[obstacleClosestIndex];
    currentDistance = UtilMath::distance(mainVehiclePos, SSD::SimPoint3D(obstacle.pt.x, obstacle.pt.y, obstacle.pt.z));
    return true;
}

/**
 * @brief Detects the nearest obstacle in front of the same lane.
 * @param gps GPS data of the main vehicle.
 * @param allObstacles List of all valid obstacles in the scene.
 * @param obstacle A reference to receive the detected obstacle.
 * @param currentDistance A reference to receive the distance to the detected obstacle.
 * @return Returns true if an obstacle in the same lane is detected, otherwise returns false.
 */
bool DetectObstacleAhead(const std::unique_ptr<SimOne_Data_Gps> &gps, std::vector<obstaclestruct> &allObstacles,
                         obstaclestruct &obstacle,
                         double &currentDistance) {
    int obstacleClosestIndex = -1;
    double minDist = std::numeric_limits<double>::max();
    obstaclestruct minObs{};
    SSD::SimPoint3D mainVehiclePos(gps->posX, gps->posY, gps->posZ);
    SSD::SimString laneId = GetNearMostLane(mainVehiclePos), laneObs;
    double SOfObs, SOfV, TOfObs, TOfCenter;
    SOfV = GetS(mainVehiclePos, laneId);
    for (size_t i = 0; i < allObstacles.size(); i++) {
        obstaclestruct obstacle1 = allObstacles[i];
        if (obstacle1.ownerLaneId == laneId) {
            double dis = UtilMath::distance(obstacle1.pt, mainVehiclePos);
            SSD::SimPoint3D center, dir;
            SOfObs = GetS(obstacle1.pt, laneId);
            TOfObs = GetT(obstacle1.pt, laneId);
            SimOneAPI::GetLaneMiddlePoint(obstacle1.pt, laneId, center, dir);
            TOfCenter = GetT(center, laneId);
            if (abs(TOfObs - TOfCenter) < 1.5 && SOfV < SOfObs) {
                if (dis < minDist) {
                    obstacleClosestIndex = i;
                    minDist = dis;
                }
            }
        }
    }

    if (obstacleClosestIndex != -1) {
        obstacle = allObstacles[obstacleClosestIndex];
        currentDistance = minDist;
        return true;
    }
    currentDistance = 1000;
    return false;
}

/**
 * @brief Detect the first stationary obstacle on the planned path.
 * @param obstacleList obstacle list.
 * @param shortPath short-term planning path.
 * @param firstObstacle Receives a reference to the first obstacle detected.
 * @param obstaclePathIndex A reference used to receive the index of the obstacle on the path.
 * @return Returns true if detected, false otherwise.
 */
bool DetectFirstObstacleOnPath(std::vector<obstaclestruct> &obstacleList,
                               SSD::SimPoint3DVector &shortPath,
                               obstaclestruct &firstObstacle, size_t &obstaclePathIndex) {
    size_t index = 300;
    obstaclestruct fir;
    for (auto &obstacle: obstacleList) {
        if (obstacle.speed < 0.2) {
            for (size_t i = 0; i < shortPath.size(); i++) {
                if (UtilMath::distance(obstacle.pt, shortPath[i]) < 0.7) {
                    if (i < index) {
                        index = i;
                        fir = obstacle;
                    }
                    break;
                }
            }
        }
    }
    if (index == 300) return false;
    firstObstacle = fir;
    obstaclePathIndex = index;
    return true;
}

/**
 * @brief Detect the first dynamic obstacle on the planned path.
 * @param obstacleList obstacle list.
 * @param shortPath short-term planning path.
 * @param movingObstacle Receives a reference to the first dynamic obstacle detected.
 * @param obstaclePathIndex A reference used to receive the index of the obstacle on the path.
 * @return Returns true if detected, false otherwise.
 */
bool DetectMovingObstacleOnPath(std::vector<obstaclestruct> &obstacleList, SSD::SimPoint3DVector &shortPath,
                          obstaclestruct &movingObstacle, size_t &obstaclePathIndex) {
    size_t index = 300;
    obstaclestruct fir;
    for (auto &obstacle: obstacleList) {
        if (obstacle.speed > 0.2) {
            for (size_t i = 0; i < shortPath.size(); i++) {
                if (UtilMath::distance(obstacle.pt, shortPath[i]) < 0.7) {
                    if (i < index) {
                        index = i;
                        fir = obstacle;
                    }
                    break;
                }
            }
        }
    }
    if (index == 300) return false;
    movingObstacle = fir;
    obstaclePathIndex = index;
    return true;
}

/**
 * @brief Detects the traffic sign closest to the host vehicle.
 * @param gps GPS data of the main vehicle.
 * @param warningSignIndex Receives a reference to the most recent sign index.
 * @param distance Receives a reference to the distance to the nearest marker.
 * @return Returns true if the traffic sign is found, false otherwise.
 */
bool DetectClosestSign(const std::unique_ptr<SimOne_Data_Gps> &gps,
                       int &warningSignIndex, double &distance) {
    SSD::SimVector<HDMapStandalone::MSignal> trafficSignList;
    SimOneAPI::GetTrafficSignList(trafficSignList);
    SSD::SimPoint3D mainVehiclePos(gps->posX, gps->posY, gps->posZ);
    double minDist = std::numeric_limits<double>::max();
    int closestSignIndex = -1;
    for (size_t i = 0; i < trafficSignList.size(); i++) {
        auto &sign = trafficSignList[i];
        TrafficSignType signs = GetTrafficSignType(sign.type);
        std::cout << "sign" << i << "   " << signs << std::endl;
        std::cout << "sign PT::" << sign.pt.x << "," << sign.pt.y << std::endl;
        std::cout << std::endl;
        double distanceSign = UtilMath::distance(mainVehiclePos, SSD::SimPoint3D(sign.pt.x, sign.pt.y, sign.pt.z));
        if (distanceSign < minDist) {
            minDist = distanceSign;
            closestSignIndex = i;
        }
    }
    if (closestSignIndex == -1) {
        return false;
    }
    warningSignIndex = closestSignIndex;
    distance = minDist;
    return true;
}

/**
 * @brief Detect speed limit signs within a specified radius.
 * @param gps GPS data of the main vehicle.
 * @param warningSignIndex A reference to receive the sign index.
 * @param r Detection radius.
 * @param speedLimitMps Receives the speed limit in meters per second.
 * @return Returns true if a speed limit sign is found within the radius, false otherwise.
 */
bool DetectSpeedLimitSignWithinRadius(const SimOne_Data_Gps &gps, int &warningSignIndex,
                             double r, double &speedLimitMps) {
    SSD::SimVector<HDMapStandalone::MSignal> trafficSignList;
    SimOneAPI::GetTrafficSignList(trafficSignList);
    const double kAlertDistance = r;
    const SSD::SimPoint2D gps2D(gps.posX, gps.posY);
    double minDist = std::numeric_limits<double>::max();
    int closestSignIndex = -1;
    for (auto &sign: trafficSignList) {
        const double distanceSign =
                UtilMath::distance(gps2D, SSD::SimPoint2D(sign.pt.x, sign.pt.y));
        if (distanceSign < minDist) {
            minDist = distanceSign;
            closestSignIndex = &sign - &trafficSignList[0];
        }
    }
    if (minDist > kAlertDistance) {
        return false;
    }
    warningSignIndex = closestSignIndex;
    const auto &targetSign = trafficSignList[closestSignIndex];
    TrafficSignType sign = GetTrafficSignType(targetSign.type);

    if (sign == TrafficSignType::SpeedLimit_Sign) {
        std::string speedLimitValue = targetSign.value.GetString();
        double speedLimit;
        std::istringstream ss(speedLimitValue);
        ss >> speedLimit;
        speedLimitMps = speedLimit / 3.6;
        return true;
    }
    return false;
}

/**
 * @brief Detects the nearest speed limit sign and obtains the speed limit value.
 * @param gps GPS data of the main vehicle.
 * @param isLongRouteCase Whether to apply the long-route-specific logic.
 * @param speedLimitMps Receives the speed limit in meters per second.
 * @return Returns true if a valid speed limit sign is detected, false otherwise.
 */
bool DetectNearestSpeedLimitSign(const std::unique_ptr<SimOne_Data_Gps> &gps, bool isLongRouteCase,
                                double &speedLimitMps) {
    SSD::SimVector<HDMapStandalone::MSignal> trafficSignList;
    SimOneAPI::GetTrafficSignList(trafficSignList);
    float mainVehicleVel = sqrtf(pow(gps->velX, 2) + pow(gps->velY, 2));
    (void)mainVehicleVel;
    SSD::SimPoint3D mainVehiclePos(gps->posX, gps->posY, gps->posZ);
    double currentDistance;
    const double kAlertDistance = 40;
    int closestSignIndex = -1;
    if (DetectClosestSign(gps, closestSignIndex,
                          currentDistance))
    {
        auto &targetSign = trafficSignList[closestSignIndex];
        TrafficSignType sign =
                GetTrafficSignType(targetSign.type);
        if (sign == TrafficSignType::SpeedLimit_Sign) {

            double distance = UtilMath::distance(
                    mainVehiclePos,
                    SSD::SimPoint3D(targetSign.pt.x, targetSign.pt.y, targetSign.pt.z));
            auto psi = (double) gps->oriZ;
            double headingErrorRad = atan2(targetSign.pt.y - gps->posY, targetSign.pt.x - gps->posX) - psi;
            std::cout << "targetsignalfa: " << headingErrorRad << std::endl;
            std::cout << "targetsigndistance: " << distance << std::endl;
            if (isLongRouteCase) {
                if (distance > kAlertDistance ||
                    (distance > kAlertDistance && (headingErrorRad >= 0.155 || headingErrorRad <= -0.155)))
                {
                    return false;
                } else if (distance > 20 && (headingErrorRad >= 0.155 || headingErrorRad <= -0.155)) {
                    return false;
                } else if (distance > 12 && (headingErrorRad >= 0.26 || headingErrorRad <= -0.26)) {
                    return false;
                } else if (distance > 6 && (headingErrorRad >= 0.53 || headingErrorRad <= -0.53)) {
                    return false;
                }
            } else {
                if (distance > 5) {
                    return false;
                }
            }


            std::string speedLimitValue =
                    targetSign.value.GetString();

            double speedLimit;
            std::stringstream ss;
            ss << speedLimitValue;
            ss >> speedLimit;
            speedLimitMps = speedLimit / 3.6;
            return true;
        }
        return false;
    }
    return false;
}

/**
 * @brief Detect obstacles within the intersection area.
 * @param gps GPS data of the main vehicle.
 * @param stopLine stop line position.
 * @param distance Receives the distance to the obstacle.
 * @return Returns true if an obstacle is detected in the intersection area, otherwise returns false.
 */
bool DetectJunctionObstacle(const std::unique_ptr<SimOne_Data_Gps> &gps,
                            SSD::SimPoint3D &stopLine, double &distance) {
    SSD::SimPoint3D mainVehiclePos(gps->posX, gps->posY, gps->posZ);
    SSD::SimString laneId = GetNearMostLane(mainVehiclePos);
    if (DetectJunction(gps)) {
        SSD::SimPoint3DVector centerline = GetLaneSample(laneId);
        stopLine = centerline.back();

        SSD::SimPoint3DVector junction_detectZone;
        double s, t, z, s_front, s_back, t_left, t_right;
        SSD::SimPoint3D Zone, dir2;

        SimOneAPI::GetRoadST(laneId, stopLine, s, t, z);
        t_left = t + 14.5;
        t_right = t - 4.5;
        s_front = s + 27.5;
        s_back = s + 8.5;

        SimOneAPI::GetInertialFromLaneST(laneId, s_front, t_right, Zone, dir2);
        junction_detectZone.push_back(Zone);
        SimOneAPI::GetInertialFromLaneST(laneId, s_front, t_left, Zone, dir2);
        junction_detectZone.push_back(Zone);
        SimOneAPI::GetInertialFromLaneST(laneId, s_back, t_left, Zone, dir2);
        junction_detectZone.push_back(Zone);
        SimOneAPI::GetInertialFromLaneST(laneId, s_back, t_right, Zone, dir2);
        junction_detectZone.push_back(Zone);

        std::vector<obstaclestruct> allObstacles;
        GetValidObstacles(gps, laneId, allObstacles);
        for (auto &obstacle: allObstacles) {
            if (obstacle.type == 6 && obstacle.speed != 0) {
                if (IsOccupied(obstacle.pt, junction_detectZone)) {
                    distance = UtilMath::distance(mainVehiclePos,
                                                  obstacle.pt);
                    return true;
                }
            }
        }
    }
    return false;
}

/**
 * @brief Detects whether the crosswalk is occupied by dynamic obstacles (pedestrians, non-motorized vehicles).
 * @param crosswalkBoundary Crosswalk boundary point vector.
 * @param allObstacles List of all obstacles in the scene.
 * @return Returns true if occupied, otherwise returns false.
 */
bool CrosswalkOccupied(
        const SSD::SimPoint3DVector &crosswalkBoundary,
        std::vector<obstaclestruct> &allObstacles) {
    for (size_t i = 0; i < allObstacles.size(); i++) {
        obstaclestruct obs = allObstacles[i];
        double xfront, xbehind, yfront, ybehind;
        xfront = obs.pt.x + obs.width / 2;
        xbehind = obs.pt.x - obs.width / 2;
        yfront = obs.pt.y + obs.width / 2;
        ybehind = obs.pt.y - obs.width / 2;
        SSD::SimPoint3DVector Zone;
        Zone.push_back(SSD::SimPoint3D(xfront, yfront, 0));
        Zone.push_back(SSD::SimPoint3D(xfront, ybehind, 0));
        Zone.push_back(SSD::SimPoint3D(xbehind, ybehind, 0));
        Zone.push_back(SSD::SimPoint3D(xbehind, yfront, 0));
        if ((obs.type == 6 || obs.type == 4) && obs.speed != 0) {
            for (auto &pt: Zone) {
                if (IsOccupied(pt, crosswalkBoundary)) {
                    return true;
                }
            }
        }
    }
    return false;
}

/**
 * @brief Calculate the total velocity of the obstacle from the velocity components.
 * @param obs Obstacle entry data.
 * @return Speed magnitude.
 */
double CalculateObstacleSpeed(const SimOne_Data_Obstacle_Entry obs) {
    double speed;
    speed = sqrtf(pow(obs.velX, 2) + pow(obs.velY, 2));
    return speed;
}
