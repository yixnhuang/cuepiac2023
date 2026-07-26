#if defined(__clang__)
#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Wsign-compare"
#pragma clang diagnostic ignored "-Wunknown-pragmas"
#pragma clang diagnostic ignored "-Wreturn-type-c-linkage"
#endif

#include "../common/DecisionUtils.h"
#include <algorithm>
#include <iostream>
#include "SimOneHDMapAPI.h"
#include "SimOnePNCAPI.h"
#include "SimOneSensorAPI.h"
#include "../perception/prediction.h"
#include "../util/UtilMath.h"
#include "../common/PathUtils.h"

#if defined(__clang__)
#pragma clang diagnostic pop
#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Wtautological-overlap-compare"
#pragma clang diagnostic ignored "-Wuninitialized"
#endif

/**
 * @brief Gets and defines a rectangular area used to detect lane changing areas.
 * @param currentLaneId The current lane ID of the main vehicle.
 * @param changeToLaneName Receives the target lane ID for the lane change.
 * @param mainVehiclePos The current position of the main vehicle.
 * @param obstacle target obstacle.
 * @param detectingZone Receives the four corner points of the detection area.
 * @param turn A reference used to receive the lane change direction ("left" or "right").
 * @return Returns true if a valid lane change direction and area can be determined.
 */
bool GetDetectingZone(const SSD::SimString &currentLaneId,
                      SSD::SimString &changeToLaneName,
                      const SSD::SimPoint3D &mainVehiclePos,
                      const obstaclestruct &obstacle,
                      SSD::SimPoint3DVector &detectingZone,
                      SSD::SimString &turn) {
    SSD::SimString turntolane;
    HDMapStandalone::MLaneLink lanelink;
    HDMapStandalone::MLaneType leftlanetype, rightlanetype;
    SimOneAPI::GetLaneLink(currentLaneId, lanelink);
    HDMapStandalone::MRoadMark left, right;

    SimOneAPI::GetRoadMark(mainVehiclePos, currentLaneId, left, right);
    SimOneAPI::GetLaneType(lanelink.leftNeighborLaneName, leftlanetype);
    SimOneAPI::GetLaneType(lanelink.rightNeighborLaneName, rightlanetype);
    if (lanelink.leftNeighborLaneName.Empty() == 0 &&
        leftlanetype == HDMapStandalone::MLaneType::driving &&
        left.type == HDMapStandalone::ERoadMarkType::broken) {
        turntolane = lanelink.leftNeighborLaneName;
        turn = "left";
    } else if (lanelink.rightNeighborLaneName.Empty() == 0 &&
               rightlanetype == HDMapStandalone::MLaneType::driving &&
               right.type == HDMapStandalone::ERoadMarkType::broken) {
        turntolane = lanelink.rightNeighborLaneName;
        turn = "right";
    } else {
        return false;
    }
    changeToLaneName = turntolane;
    SSD::SimPoint3D changeToPoint, changeBackPoint;
    SSD::SimPoint3D dir, dir2;
    double s, t, s_front, s_back, t_left, t_right;
    double distance =
            UtilMath::distance(mainVehiclePos, obstacle.pt);

    SimOneAPI::GetLaneMiddlePoint(
            obstacle.pt, turntolane, changeToPoint,
            dir);
    t_left = t - 1.7;
    t_right = t + 1.7;
    s_front = s + 10;
    s_back = s - distance - 7;

    SimOneAPI::GetInertialFromLaneST(turntolane, s_front, t_right,
                                     changeBackPoint, dir2);
    detectingZone.push_back(changeBackPoint);
    SimOneAPI::GetInertialFromLaneST(turntolane, s_front, t_left, changeBackPoint,
                                     dir2);
    detectingZone.push_back(changeBackPoint);
    SimOneAPI::GetInertialFromLaneST(turntolane, s_back, t_left, changeBackPoint,
                                     dir2);
    detectingZone.push_back(changeBackPoint);
    SimOneAPI::GetInertialFromLaneST(turntolane, s_back, t_right, changeBackPoint,
                                     dir2);
    detectingZone.push_back(changeBackPoint);

    return true;
}

/**
 * @brief Checks whether a specified detection area is occupied by any obstacles.
 * @param obstacleList obstacle list.
 * @param detectingZone The specified detection area (defined by four corner points).
 * @return Returns true if there are obstacles in the area, otherwise returns false.
 */
bool IsChangeLaneOccupied(const std::vector<obstaclestruct> &obstacleList, SSD::SimPoint3DVector &detectingZone) {
    for (auto &obstacle: obstacleList) {
        if (IsOccupied(obstacle.pt, detectingZone)) {
            std::cout << "obstacle" << obstacle.pt.x << "   " << obstacle.pt.y << std::endl;
            return true;
        }
    }
    return false;
}

/**
 * @brief Determines whether the host vehicle has passed the specified obstacle.
 * @param vehiclePos Main vehicle position.
 * @param obstacle target obstacle.
 * @param laneId Lane ID where the obstacle is located.
 * @return Returns true if the host vehicle has passed the obstacle.
 */
bool PassedObstacle(const SSD::SimPoint3D &vehiclePos,
                    const obstaclestruct &obstacle,
                    const SSD::SimString &laneId) {
    double s, t;
    bool found = SimOneAPI::GetLaneST(laneId, obstacle.pt, s,
                                      t);
    assert(found);
    double s_vehicle, t_vehicle;
    found = SimOneAPI::GetLaneST(laneId, vehiclePos, s_vehicle,
                                 t_vehicle);
    assert(found);
    return s_vehicle > s;
}

/**
 * @brief Determines whether the specified traffic light is green and there is enough time to pass.
 * @param lightId traffic light ID.
 * @param laneId The current lane ID of the main vehicle.
 * @param light Traffic light object.
 * @param speed Main vehicle speed.
 * @param distance The distance from the main vehicle to the stop line.
 * @return If the light is green and the time is sufficient, return true.
 */
bool IsGreenLight(const long &lightId, const SSD::SimString &laneId,
                  const HDMapStandalone::MSignal &light, double speed,
                  double distance) {
    (void)laneId;
    (void)light;
    SimOne_Data_TrafficLight trafficLight;
    if (SimOneAPI::GetTrafficLight(0, lightId,
                                   &trafficLight))
    {
        if (trafficLight.status !=
            ESimOne_TrafficLight_Status::ESimOne_TrafficLight_Status_Green) {
            return false;
        }
        else {
            if (trafficLight.countDown != -1) {
                if (distance / speed < (trafficLight.countDown - 2) ||
                    trafficLight.countDown > 10)
                    return true;
            } else if (trafficLight.countDown == -1)
                return true;
            else
                return false;
        }
    }
    return false;
}

/**
 * @brief Obtains the target subsequent lane based on the validity information of the signal light.
 * @param successorLaneNameList Successor lane list.
 * @param validities Traffic-signal validity list.
 * @return ID of the target successor lane.
 */
SSD::SimString GetTargetSuccessorLane(
        const SSD::SimStringVector &successorLaneNameList,
        const SSD::SimVector<HDMapStandalone::MSignalValidity> &validities) {
    SSD::SimString targetSuccessorLaneId;
    for (auto &successorLane: successorLaneNameList) {
        HDMapStandalone::MLaneId id(
                successorLane);
        for (auto &validity: validities) {
            if ((id.roadId == validity.roadId &&
                 id.sectionIndex == validity.sectionIndex &&
                 id.sectionIndex == validity.fromLaneId) ||
                (id.roadId == validity.roadId &&
                 id.sectionIndex == validity.sectionIndex &&
                 id.sectionIndex == validity.toLaneId)) {
                targetSuccessorLaneId = successorLane;
                break;
            }
        }
    }
    return targetSuccessorLaneId;
}

/**
 * @brief Determine whether the main vehicle has crossed a certain point (such as the stop line).
 * @param vehiclePos Main vehicle position.
 * @param light Target point position.
 * @param laneId Lane ID of the target point.
 * @return Returns true if it has been exceeded.
 */
bool Passed(const SSD::SimPoint3D &vehiclePos, const SSD::SimPoint3D &light,
            const SSD::SimString &laneId) {
    double s, t;
    bool found =
            SimOneAPI::GetLaneST(laneId, light, s, t);
    double sveh, tveh;
    found = SimOneAPI::GetLaneST(laneId, vehiclePos, sveh,
                                 tveh);
    (void)found;
    std::cout << "sveh: " << sveh << " s: " << s << std::endl;
    return sveh > s;
}

/**
 * @brief Detects stop lines, traffic lights and crosswalk information ahead.
 * @param carPos Main car position.
 * @param roadIdList The road ID list of the navigation path.
 * @param stopLine Receives the stop-line position.
 * @param currentLight Receives the current traffic-light object.
 * @param crosswalk Receives a reference to the crosswalk object.
 * @param currentDistance Receives a reference to the stopping line distance.
 * @return Returns true if valid stop line information is detected.
 */
bool DetectStopLine(SSD::SimPoint3D &carPos, SSD::SimVector<long> &roadIdList,
                    SSD::SimPoint3D &stopLine,
                    HDMapStandalone::MSignal &currentLight,
                    HDMapStandalone::MObject &crosswalk,
                    double &currentDistance) {
    SSD::SimString laneId = GetNearMostLane(carPos);
    HDMapStandalone::MSignal light1 = GetTargetLight(laneId, roadIdList);
    stopLine = GetTargetStopLine(light1, laneId);
    double distance = UtilMath::distance(carPos, stopLine);
    currentLight = light1;
    SSD::SimVector<HDMapStandalone::MObject> crosswalklist;
    SimOneAPI::GetCrosswalkList(light1, laneId, crosswalklist);
    currentDistance = distance;
    if (distance < 150) {
        crosswalk = crosswalklist.front();
        std::cout << "crosswalk.boundaryKnots 0  " << crosswalk.boundaryKnots[0].x << "  "
                  << crosswalk.boundaryKnots[0].y << std::endl;
        std::cout << "crosswalk.boundaryKnots 1  " << crosswalk.boundaryKnots[1].x << "  "
                  << crosswalk.boundaryKnots[1].y << std::endl;
        std::cout << "crosswalk.boundaryKnots 2  " << crosswalk.boundaryKnots[2].x << "  "
                  << crosswalk.boundaryKnots[2].y << std::endl;
        std::cout << "crosswalk.boundaryKnots 3  " << crosswalk.boundaryKnots[3].x << "  "
                  << crosswalk.boundaryKnots[3].y << std::endl;
    } else {
        std::cout << "No crosswalk" << std::endl;
        return false;
    }
    return true;
}

/**
 * @brief Detect stop lines and crosswalks in the absence of traffic lights.
 * @param gps GPS data of the main vehicle.
 * @param hasTrafficLight Whether there is a traffic light sign.
 * @param stopLine Receives the stop-line position.
 * @param crosswalk Receives a reference to the crosswalk boundary point.
 * @param currentDistance Receives a reference to the stopping line distance.
 * @return Returns true if a stop line without light control is detected.
 */
bool DetectNoLightStopLine(const std::unique_ptr<SimOne_Data_Gps> &gps, bool hasTrafficLight, SSD::SimPoint3D &stopLine,
                           SSD::SimPoint3DVector &crosswalk, double &currentDistance) {
    SSD::SimPoint3D mainVehiclePos(gps->posX, gps->posY, gps->posZ);
    SSD::SimString laneId = GetNearMostLane(mainVehiclePos);
    if (!hasTrafficLight) {
        SSD::SimVector<HDMapStandalone::MObject> stopLineList;
        SimOneAPI::GetSpecifiedLaneStoplineList(laneId, stopLineList);
        if (!stopLineList.empty()) {
            SSD::SimVector<HDMapStandalone::MObject> crosswalkList;
            SimOneAPI::GetSpecifiedLaneCrosswalkList(laneId, crosswalkList);
            if (!crosswalkList.empty()) {
                SSD::SimPoint3D dir;
                std::cout << "nolightstopline" << std::endl;
                SimOneAPI::GetLaneMiddlePoint(stopLineList[0].pt, laneId, stopLine, dir);
                currentDistance = UtilMath::distance(stopLine, mainVehiclePos);

                crosswalk = crosswalkList.front().boundaryKnots;
                std::cout << "crosswalkList" << std::endl;
                return true;
            }
        }
    }
    return false;
}

/**
 * @brief Determine whether the intersection ahead is congested.
 * @param gps GPS data of the main vehicle.
 * @param allObstacles List of all obstacles in the scene.
 * @param roadIdList The road ID list of the navigation path.
 * @return If there is congestion, return true.
 */
bool IsJunctionCrowded(const std::unique_ptr<SimOne_Data_Gps> &gps, std::vector<obstaclestruct> &allObstacles,
                      const SSD::SimVector<long> &roadIdList) {
    SSD::SimPoint3DVector centerline;
    SSD::SimPoint3D mainVehiclePos(gps->posX, gps->posY, gps->posZ);
    SSD::SimString laneId = GetNearMostLane(mainVehiclePos);

    HDMapStandalone::MLaneId id(laneId);
    HDMapStandalone::MLaneId nextLaneId;
    HDMapStandalone::MLaneId towardsLaneId;

    if (DetectJunction(gps)) {
        int index = std::find(roadIdList.begin(), roadIdList.end(), id.roadId) - roadIdList.begin();
        GetValidSuccessor(id, roadIdList[index], roadIdList[index + 1], nextLaneId);
        GetValidSuccessor(nextLaneId, roadIdList[index + 1], roadIdList[index + 2], towardsLaneId);
        centerline = GetLaneSample(towardsLaneId.ToString());

        SSD::SimPoint3D startLine = centerline.front();
        SSD::SimPoint3DVector crowdedDetectionZone;
        double s, t, s_front, s_back, t_left, t_right;
        SSD::SimPoint3D Zone, dir2;

        SimOneAPI::GetLaneST(towardsLaneId.ToString(), startLine, s, t);
        t_left = t - 1.7;
        t_right = t + 1.8;
        s_front = s + 25;
        s_back = s;

        SimOneAPI::GetInertialFromLaneST(towardsLaneId.ToString(), s_front, t_right, Zone, dir2);
        crowdedDetectionZone.push_back(Zone);
        SimOneAPI::GetInertialFromLaneST(towardsLaneId.ToString(), s_back, t_right, Zone, dir2);
        crowdedDetectionZone.push_back(Zone);
        SimOneAPI::GetInertialFromLaneST(towardsLaneId.ToString(), s_back, t_left, Zone, dir2);
        crowdedDetectionZone.push_back(Zone);
        SimOneAPI::GetInertialFromLaneST(towardsLaneId.ToString(), s_front, t_left, Zone, dir2);
        crowdedDetectionZone.push_back(Zone);

        std::cout << " 0: " << crowdedDetectionZone[0].x << " , " << crowdedDetectionZone[0].y << std::endl;
        std::cout << " 1: " << crowdedDetectionZone[1].x << " , " << crowdedDetectionZone[1].y << std::endl;
        std::cout << " 2: " << crowdedDetectionZone[2].x << " , " << crowdedDetectionZone[2].y << std::endl;
        std::cout << " 3: " << crowdedDetectionZone[3].x << " , " << crowdedDetectionZone[3].y << std::endl;

        for (auto &obstacle: allObstacles) {
            if (obstacle.type == 6 && obstacle.speed <= 1) {
                if (IsOccupied(obstacle.pt, crowdedDetectionZone)) {
                    return true;
                }
            }
        }
    }
    return false;
}

/**
 * @brief Check whether lane change (turn) can be performed.
 * @param gps GPS data of the main vehicle.
 * @param allObstacles List of all obstacles in the scene.
 * @param obstacle target obstacle.
 * @param turn Receives the lane-change direction.
 * @param state A reference to receive the vehicle state (such as "wait").
 * @return Returns true if lane change is possible.
 */
bool DetectIsTurnable(const std::unique_ptr<SimOne_Data_Gps> &gps, std::vector<obstaclestruct> &allObstacles,
                      const obstaclestruct &obstacle, SSD::SimString &turn,
                      SSD::SimString &state) {
    double s, t, s_obstacle, t_obstacle;

    SSD::SimPoint3DVector frontDetectionZone, rearDetectionZone;
    SSD::SimPoint3D mainVehiclePos(gps->posX, gps->posY, gps->posZ);
    SSD::SimPoint3D dir, testpoint;
    SSD::SimString currentlaneid = GetNearMostLane(mainVehiclePos);
    SSD::SimString turntolane;
    HDMapStandalone::MLaneLink lanelink;
    HDMapStandalone::MLaneType leftlanetype, rightlanetype;

    SimOneAPI::GetLaneLink(obstacle.ownerLaneId, lanelink);
    HDMapStandalone::MRoadMark left, right;

    SimOneAPI::GetRoadMark(mainVehiclePos, currentlaneid, left, right);
    SimOneAPI::GetLaneType(lanelink.leftNeighborLaneName, leftlanetype);
    SimOneAPI::GetLaneType(lanelink.rightNeighborLaneName, rightlanetype);
    SimOneAPI::GetLaneST(currentlaneid, mainVehiclePos, s, t);
    SimOneAPI::GetLaneST(currentlaneid, obstacle.pt, s_obstacle, t_obstacle);

    double s_front = s_obstacle + 3;
    double s_obsback = s_obstacle - 2;
    double s_back = s - 3;


    if (lanelink.rightNeighborLaneName.Empty() == 0 &&
        (right.type != HDMapStandalone::ERoadMarkType::solid ||
         right.type != HDMapStandalone::ERoadMarkType::solid_solid ||
         right.type != HDMapStandalone::ERoadMarkType::broken_solid ||
         right.type != HDMapStandalone::ERoadMarkType::curb)) {
        turn = "right";
        std::cout << "right broken" << std::endl;
        turntolane = lanelink.rightNeighborLaneName;
        SSD::SimPoint3D changeToPoint;
        SimOneAPI::GetLaneMiddlePoint(obstacle.pt, turntolane, changeToPoint, dir);

        frontDetectionZone = GetDetectionZoneFixed(changeToPoint, s_front, s_back, -1.70, +1.70);
        rearDetectionZone = GetDetectionZoneFixed(changeToPoint, s_front, s_obsback, -1.75, +1.75);

        if (!IsChangeLaneOccupied(allObstacles, frontDetectionZone) && !IsChangeLaneOccupied(allObstacles, rearDetectionZone)) {
            turn = "right";
            return true;
        } else {
            state = "wait";
        }
    } else if (lanelink.leftNeighborLaneName.Empty() == 0 &&
               (left.type != HDMapStandalone::ERoadMarkType::solid_broken ||
                left.type != HDMapStandalone::ERoadMarkType::solid ||
                left.type != HDMapStandalone::ERoadMarkType::solid_solid ||
                left.type != HDMapStandalone::ERoadMarkType::curb)) {
        turn = "left";
        std::cout << "left broken" << std::endl;
        turntolane = lanelink.leftNeighborLaneName;
        SSD::SimPoint3D changeToPoint;
        SimOneAPI::GetLaneMiddlePoint(obstacle.pt, turntolane, changeToPoint, dir);

        frontDetectionZone = GetDetectionZoneFixed(changeToPoint, s_front, s_back, -1.75, +1.75);
        rearDetectionZone = GetDetectionZoneFixed(changeToPoint, s_front, s_obsback, -1.75, +1.75);
        if (!IsChangeLaneOccupied(allObstacles, frontDetectionZone) && !IsChangeLaneOccupied(allObstacles, rearDetectionZone)) {
            turn = "left";
            std::cout << "DetectIsTurnable::left" << std::endl;
            return true;
        } else {
            state = "wait";
            std::cout << "DetectIsTurnable::wait" << std::endl;
        }
    } else {
        state = "cant_change";
        return false;
    }
    PrintTargetPath(frontDetectionZone);
    std::cout << "DetectIsTurnable false " << std::endl;
    return false;
}

/**
 * @brief Check whether the lane changing conditions are met.
 * @param obstacle target obstacle.
 * @param obstacleList List of all obstacles in the scene.
 * @param turnTargetPoint Receives the lane-change target point.
 * @return Returns true if lane change is possible.
 */
bool IsChangeable(obstaclestruct &obstacle, std::vector<obstaclestruct> &obstacleList, SSD::SimPoint3D &turnTargetPoint) {

    double s_obstacle, t_obstacle;
    SSD::SimString currentlaneid = GetNearMostLane(obstacle.pt);
    SSD::SimPoint3DVector detectionZone;
    SSD::SimPoint3D dir, testpoint;
    SSD::SimString turntolane;
    HDMapStandalone::MLaneLink lanelink;
    HDMapStandalone::MLaneType leftlanetype, rightlanetype;

    SimOneAPI::GetLaneLink(currentlaneid, lanelink);

    HDMapStandalone::MRoadMark left, right;

    SimOneAPI::GetRoadMark(obstacle.pt, currentlaneid, left, right);
    SimOneAPI::GetLaneType(lanelink.leftNeighborLaneName, leftlanetype);
    SimOneAPI::GetLaneType(lanelink.rightNeighborLaneName, rightlanetype);
    SimOneAPI::GetLaneST(currentlaneid, obstacle.pt, s_obstacle, t_obstacle);

    double s_front = s_obstacle + 3;
    double s_back = s_obstacle - 3;
    std::cout << "lanelink.leftNeighborLaneName.Empty() ==" << lanelink.leftNeighborLaneName.Empty() << std::endl;
    std::cout << "lanelink.rightNeighborLaneName.Empty() ==" << lanelink.rightNeighborLaneName.Empty() << std::endl;

    if (lanelink.leftNeighborLaneName.Empty() == 0 &&
        leftlanetype == HDMapStandalone::MLaneType::driving &&
        (left.type != HDMapStandalone::ERoadMarkType::solid_broken ||
         left.type != HDMapStandalone::ERoadMarkType::solid ||
         left.type != HDMapStandalone::ERoadMarkType::solid_solid ||
         left.type != HDMapStandalone::ERoadMarkType::curb)) {
        turntolane = lanelink.leftNeighborLaneName;
        SSD::SimPoint3D changeToPoint;
        SimOneAPI::GetLaneMiddlePoint(obstacle.pt, turntolane, changeToPoint, dir);
        detectionZone = GetDetectionZoneFixed(changeToPoint, s_front, s_back, -1.75, +1.75);

        if (!IsChangeLaneOccupied(obstacleList, detectionZone)) {
            turnTargetPoint = changeToPoint;
            return true;
        }
    } else if (lanelink.rightNeighborLaneName.Empty() == 0 &&
               rightlanetype == HDMapStandalone::MLaneType::driving &&
               (right.type != HDMapStandalone::ERoadMarkType::solid ||
                right.type != HDMapStandalone::ERoadMarkType::solid_solid ||
                right.type != HDMapStandalone::ERoadMarkType::broken_solid ||
                right.type != HDMapStandalone::ERoadMarkType::curb)) {
        std::cout << "right broken" << std::endl;
        turntolane = lanelink.rightNeighborLaneName;
        SSD::SimPoint3D changeToPoint;
        SimOneAPI::GetLaneMiddlePoint(obstacle.pt, turntolane, changeToPoint, dir);

        detectionZone = GetDetectionZoneFixed(changeToPoint, s_front, s_back, -1.70,
                                         +1.70);
        if (!IsChangeLaneOccupied(obstacleList, detectionZone)) {
            turnTargetPoint = changeToPoint;
            return true;
        }
    }
    return false;
}

/**
 * @brief Detects whether the obstacle is within the intersection.
 * @param obstacle target obstacle.
 * @return If within the intersection, return true.
 */
bool IsObstacleInJunction(obstaclestruct &obstacle) {
    SSD::SimString obstacleLaneId = GetNearMostLane(obstacle.pt);
    HDMapStandalone::MLaneInfo info;
    SimOneAPI::GetLaneSample(obstacleLaneId, info);
    SSD::SimPoint3DVector junctionLaneDetectionZone;
    junctionLaneDetectionZone.push_back(info.leftBoundary[0]);
    junctionLaneDetectionZone.push_back(info.leftBoundary[20]);
    junctionLaneDetectionZone.push_back(info.rightBoundary[0]);
    junctionLaneDetectionZone.push_back(info.rightBoundary[20]);
    PrintTargetPath(junctionLaneDetectionZone);
    return IsOccupied(obstacle.pt, junctionLaneDetectionZone);
}

/**
 * @brief Check whether there is a drivable lane on the right.
 * @param lane The current lane ID, which will be modified if the right lane is found.
 * @param mainVehiclePos Main vehicle position.
 * @return Returns true if it exists.
 */
bool DetectRightRoad(SSD::SimString &lane, SSD::SimPoint3D &mainVehiclePos) {
    (void)mainVehiclePos;
    HDMapStandalone::MLaneLink laneLink;
    SimOneAPI::GetLaneLink(lane, laneLink);
    if (laneLink.rightNeighborLaneName.Empty() == 0) {
        lane = laneLink.rightNeighborLaneName;
        return true;
    }
    return false;
}

/**
 * @brief Check whether there is a drivable lane on the left (and the road markings allow lane changes).
 * @param lane The current lane ID, which will be modified if the left lane is found.
 * @param mainVehiclePos Main vehicle position.
 * @return Returns true if it exists.
 */
bool DetectLeftRoad(SSD::SimString &lane, SSD::SimPoint3D &mainVehiclePos) {
    HDMapStandalone::MLaneLink laneLink;
    SimOneAPI::GetLaneLink(lane, laneLink);
    if (laneLink.leftNeighborLaneName.Empty()) {
        return false;
    }
    HDMapStandalone::MRoadMark left, right;
    SSD::SimPoint3D centerPoint, dir;
    SimOneAPI::GetLaneMiddlePoint(mainVehiclePos, lane, centerPoint, dir);
    SimOneAPI::GetRoadMark(centerPoint, lane, left, right);
    if (laneLink.leftNeighborLaneName.Empty() == 0 &&
        (left.type != HDMapStandalone::ERoadMarkType::solid ||
         left.type != HDMapStandalone::ERoadMarkType::solid_solid)) {
        lane = laneLink.rightNeighborLaneName;
        return true;
    }
    return false;
}

#if defined(__clang__)
#pragma clang diagnostic pop
#endif

/**
 * @brief At an intersection, detect valid, unoccupied subsequent lanes.
 * @param predecessorLane Lane ID before entering the intersection.
 * @param carPos Main car position.
 * @param obstacleList obstacle list.
 * @param validLane Receives the selected valid lane ID.
 * @param turnTargetPoint Receives the turn target point.
 * @return Returns true if a valid pass path is found.
 */
bool DetectValidCrossing(SSD::SimString &predecessorLane, SSD::SimPoint3D &carPos, std::vector<obstaclestruct> &obstacleList,
                         SSD::SimString &validLane, SSD::SimPoint3D &turnTargetPoint) {
    HDMapStandalone::MLaneLink predecessorLaneLink, laneLink;
    SSD::SimStringVector RoadList;
    SSD::SimString roadBuff = predecessorLane;
    SimOneAPI::GetLaneLink(predecessorLane, predecessorLaneLink);

    RoadList.push_back(roadBuff);
    while (DetectRightRoad(roadBuff, carPos)) {
        std::cout << "pushback" << std::endl;
        RoadList.push_back(roadBuff);
    }
    roadBuff = predecessorLane;
    while (DetectLeftRoad(roadBuff, carPos)) {
        RoadList.push_back(roadBuff);
    }


    for (auto &road: RoadList) {
        SimOneAPI::GetLaneLink(road, laneLink);
        for (auto &successorLane: laneLink.successorLaneNameList) {
            size_t n = 0;
            HDMapStandalone::MLaneInfo successorLaneInfo;
            SimOneAPI::GetLaneSample(successorLane, successorLaneInfo);
            for (auto &obs: obstacleList) {
                double dis = UtilMath::distance(*successorLaneInfo.centerLine.end(), obs.pt);
                if (dis < 1.75) n++;
            }
            if (n == 0) {
                validLane = successorLane;
                turnTargetPoint = successorLaneInfo.centerLine[1];
                return true;
            }
        }
    }
    return false;
}

/**
 * @brief Calculate the lateral distance between the main vehicle and the obstacle.
 * @param carPos Main car position.
 * @param carOriZ Main car heading angle.
 * @param obstacle target obstacle.
 * @return Lateral distance (negative values represent left; positive values represent right).
 */
double GetLateralDistance(SSD::SimPoint3D &carPos, double &carOriZ, obstaclestruct &obstacle) {
    double distance = UtilMath::distance(obstacle.pt, carPos);
    double headingErrorRad = atan2(obstacle.pt.y - carPos.y, obstacle.pt.x - carPos.x);
    double error = headingErrorRad - carOriZ;

    if (error > M_PI) error = -2 * M_PI + error;
    else if (error < -M_PI) error = 2 * M_PI + error;

    return -distance * sin(error);

}

/**
 * @brief Check whether there is an intersection ahead.
 * @param currentLaneId The current lane ID of the main vehicle.
 * @param mainVehiclePos Main vehicle position.
 * @param gpsPtr Main vehicle GPS data pointer.
 * @param lanelink Link information of the current lane of the main vehicle.
 * @return If it is an intersection, return true.
 */
bool DetectCross(SSD::SimString &currentLaneId, SSD::SimPoint3D &mainVehiclePos, SimOne_Data_Gps *gpsPtr,
                 HDMapStandalone::MLaneLink sourceLaneLink) {
    std::string driveStateName;
    HDMapStandalone::MLaneLink laneLink;
    HDMapStandalone::MLaneInfo successorLaneInfo, currentLaneInfo;

    SimOneAPI::GetLaneLink(currentLaneId, laneLink);
    SimOneAPI::GetLaneSample(currentLaneId, currentLaneInfo);
    size_t num = currentLaneInfo.centerLine.size();

    double lastPointDistance = UtilMath::distance(mainVehiclePos, currentLaneInfo.centerLine[num]);
    SSD::SimString successorLane;
    double laneHeadingRad, leftTurnCount = 0, rightTurnCount = 0, straightCount = 0;
    double carOriZ = gpsPtr->oriZ;
    std::cout << "successorLaneNameList.size():  " << laneLink.successorLaneNameList.size() << std::endl;
    if (lastPointDistance < 20) {
        if (laneLink.successorLaneNameList.size() >= 2) {
            for (const auto &i: laneLink.successorLaneNameList) {
                successorLane = i;
                SimOneAPI::GetLaneSample(successorLane, successorLaneInfo);
                laneHeadingRad = atan2(successorLaneInfo.centerLine[10].y - successorLaneInfo.centerLine[0].y,
                                       successorLaneInfo.centerLine[10].x - successorLaneInfo.centerLine[0].x);
                double headingDiffRad = laneHeadingRad - carOriZ;
                if (headingDiffRad < -0.1) {
                    leftTurnCount = 1;
                } else if (headingDiffRad > 0.1) {
                    rightTurnCount = 1;
                } else if (abs(headingDiffRad) <= 0.01) {
                    if (successorLaneInfo.centerLine.size() > 20) {
                        straightCount = 1;
                    }
                }
            }
            if (leftTurnCount == 1 && rightTurnCount != 1) {
                SimOneAPI::GetLaneLink(sourceLaneLink.rightNeighborLaneName, laneLink);
                for (size_t i = 0; i < laneLink.successorLaneNameList.size(); i++) {
                    successorLane = laneLink.successorLaneNameList[i];
                    SimOneAPI::GetLaneSample(successorLane, successorLaneInfo);
                    laneHeadingRad = atan2(successorLaneInfo.centerLine[10].y - successorLaneInfo.centerLine[0].y,
                                           successorLaneInfo.centerLine[10].x - successorLaneInfo.centerLine[0].x);
                    double headingDiffRad = laneHeadingRad - carOriZ;
                    std::cout << "laneHeadingRad-carOriZ1:  " << headingDiffRad << std::endl;
                    if (headingDiffRad > 0.1) {
                        rightTurnCount = 1;
                    }
                }
            }
            if (leftTurnCount != 1 && rightTurnCount == 1) {
                SimOneAPI::GetLaneLink(sourceLaneLink.rightNeighborLaneName, laneLink);
                for (size_t i = 0; i < laneLink.successorLaneNameList.size(); i++) {
                    successorLane = laneLink.successorLaneNameList[i];
                    SimOneAPI::GetLaneSample(successorLane, successorLaneInfo);
                    laneHeadingRad = atan2(successorLaneInfo.centerLine[10].y - successorLaneInfo.centerLine[0].y,
                                           successorLaneInfo.centerLine[10].x - successorLaneInfo.centerLine[0].x);
                    double headingDiffRad = laneHeadingRad - carOriZ;
                    std::cout << "laneHeadingRad-carOriZ2:  " << headingDiffRad << std::endl;
                    if (headingDiffRad < -0.1) {
                        leftTurnCount = 1;
                    }
                }
            }
        }
        if (leftTurnCount + rightTurnCount + straightCount >= 2) {
            driveStateName = "NearIntersection";
            return true;
        } else {
            driveStateName = "Follow";
        }

    }
    return false;
}
