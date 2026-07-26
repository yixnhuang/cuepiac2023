#if defined(__clang__)
#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Wsign-compare"
#pragma clang diagnostic ignored "-Wunknown-pragmas"
#pragma clang diagnostic ignored "-Wreturn-type-c-linkage"
#endif

#include "../common/PathUtils.h"
#include "SimOneHDMapAPI.h"
#include "SimOneServiceAPI.h"
#include "SimOnePNCAPI.h"
#include "../util/UtilMath.h"
#include "bezier.h"
#include <iostream>
#include <algorithm>

#if defined(__clang__)
#pragma clang diagnostic pop
#endif

/**
 * @brief Get the lane ID closest to the specified location.
 * @param pos The specified 3D coordinate point.
 * @return ID of the nearest lane (SimString).
 */
SSD::SimString GetNearMostLane(const SSD::SimPoint3D &pos) {
    SSD::SimString laneId;
    double s, t, s_toCenterLine, t_toCenterLine;
    if (!SimOneAPI::GetNearMostLane(pos, laneId, s, t, s_toCenterLine,
                                    t_toCenterLine)) {
        std::cout << "Error: lane is not found." << std::endl;
    }
    return laneId;
}

/**
 * @brief Get the S value (longitudinal mileage) of the specified point on the target lane.
 * @param pos The specified 3D coordinate point.
 * @param laneId ID of the target lane.
 * @return S value (double).
 */
double GetS(const SSD::SimPoint3D &pos, SSD::SimString laneId) {
    double s, t;
    if (!SimOneAPI::GetLaneST(laneId, pos, s, t)) {
    }
    return s;
}

/**
 * @brief Get the T value (lateral offset) of the specified point on the target lane.
 * @param pos The specified 3D coordinate point.
 * @param laneId ID of the target lane.
 * @return T value (double).
 */
double GetT(const SSD::SimPoint3D &pos, SSD::SimString laneId) {
    double s, t;
    if (!SimOneAPI::GetLaneST(laneId, pos, s, t)) {
    }
    return t;
}

/**
 * @brief Get the list of road IDs passed by the navigation path from the starting point to the ending point.
 * @param startPt starting point.
 * @param endPt end point.
 * @return Vector containing road IDs.
 */
SSD::SimVector<long> GetNavigateRoadIdList(const SSD::SimPoint3D &startPt,
                                           const SSD::SimPoint3D &endPt) {
    SSD::SimVector<long> naviRoadIdList;
    SSD::SimPoint3DVector ptList;
    ptList.push_back(startPt);
    ptList.push_back(endPt);
    SSD::SimVector<int> indexOfValidPoints;
    SimOneAPI::Navigate(ptList, indexOfValidPoints,
                        naviRoadIdList);
    return naviRoadIdList;
}

/**
 * @brief Get the effective successor lane of the current lane.
 * @param laneId ID of the current lane.
 * @param currentRoadId The ID of the current road.
 * @param nextRoadId The ID of the next road.
 * @param successor Used to receive a reference to the successor lane ID.
 * @return Returns true if a valid successor lane is found, false otherwise.
 */
bool GetValidSuccessor(const HDMapStandalone::MLaneId &laneId,
                       const long &currentRoadId, const long &nextRoadId,
                       HDMapStandalone::MLaneId &successor) {
    if (nextRoadId == -1)
    {
        return false;
    }
    HDMapStandalone::MLaneLink laneLink;
    bool valid =
            SimOneAPI::GetLaneLink(laneId.ToString(), laneLink);
    assert(valid);
    if (laneLink.successorLaneNameList.empty()) {

        return false;
    }

    for (auto &successorLane: laneLink.successorLaneNameList) {
        HDMapStandalone::MLaneId successorId(successorLane);
        if (successorId.roadId != currentRoadId)
        {
            if (successorId.roadId == nextRoadId)
            {
                successor = successorId;
                return true;
            }
        } else
        {
            HDMapStandalone::MLaneId successorOfSuccessor;
            if (successorId.sectionIndex ==
                laneId.sectionIndex + 1)
            {
                successor = successorId;
                return true;
            }
        }
    }
    return false;
}

/**
 * @brief Adds the sampling points of the specified lane to the path vector.
 * @param laneId ID of the target lane.
 * @param path Path vector used to receive sampling points.
 */
void AddSamples(const HDMapStandalone::MLaneId &laneId,
                SSD::SimPoint3DVector &path) {
    HDMapStandalone::MLaneInfo laneInfo;
    if (SimOneAPI::GetLaneSample(laneId.ToString(), laneInfo)) {
        path.reserve(path.size() +
                     laneInfo.centerLine.size());
        for (auto &pt: laneInfo.centerLine) {
            path.push_back(pt);
        }
    }
}

/**
 * @brief Generate a reference path based on the navigation road network.
 * @param startPt starting point.
 * @param naviRoadIdList The road ID list of the navigation path.
 * @return The generated reference path (3D point vector).
 */
SSD::SimPoint3DVector
GetReferencePath(const SSD::SimPoint3D &startPt,
                 const SSD::SimVector<long> &naviRoadIdList) {
    SSD::SimPoint3DVector path;
    SSD::SimString laneName;
    double s = 0, t = 0, s_toCenterLine, t_toCenterLine;
    HDMapStandalone::MLaneInfo info;

    if (!SimOneAPI::GetNearMostLane(startPt, laneName, s, t, s_toCenterLine,
                                    t_toCenterLine)) {
        return path;
    }
    SSD::SimString laneId;
    HDMapStandalone::MLaneId currentLaneId(laneId);
    HDMapStandalone::MLaneId nextLaneId;

    AddSamples(currentLaneId, path);

    for (size_t index = 0; index < naviRoadIdList.size(); ++index) {
        long roadId = naviRoadIdList[index];
        long nextRoadId =
                (index + 1 < naviRoadIdList.size()) ? naviRoadIdList[index + 1] : -1;

        if (GetValidSuccessor(currentLaneId, roadId, nextRoadId, nextLaneId)) {
            AddSamples(nextLaneId, path);
            currentLaneId = nextLaneId;
        } else {
            break;
        }
    }

    return path;
}

/**
 * @brief Starting from the specified S value of the specified lane, obtain the sampling points and add them to the target path.
 * @param laneId ID of the target lane.
 * @param s starting S value.
 * @param targetPath Path vector used to receive sampling points.
 */
void GetLaneSampleFromS(const SSD::SimString &laneId, const double &s,
                        SSD::SimPoint3DVector &targetPath) {
    HDMapStandalone::MLaneInfo info;
    if (SimOneAPI::GetLaneSample(laneId, info)) {
        double accumulated = 0.0;
        int startIndex = -1;
        for (unsigned int i = 0; i < info.centerLine.size() - 1; i++) {
            auto &pt = info.centerLine[i];
            auto &ptNext = info.centerLine[i + 1];
            double d = UtilMath::distance(pt, ptNext);
            accumulated += d;
            if (accumulated >= s) {
                startIndex = i + 1;
                break;
            }
        }
        for (unsigned int i = startIndex; i < info.centerLine.size(); i++) {
            SSD::SimPoint3D item = info.centerLine[i];
            targetPath.push_back(info.centerLine[i]);
        }
    }
}

/**
 * @brief Use Bezier curve to generate a lane change path between two points.
 * @param from starting point.
 * @param to target point.
 * @return The generated lane change path (3D point vector).
 */
SSD::SimPoint3DVector ChangeLanePathWithPoint(const SSD::SimPoint3D &from,
                                              const SSD::SimPoint3D &to) {
    SSD::SimPoint3DVector path;
    double s, t, s_c, t_c;
    SSD::SimPoint3D PointNextFrom, PointNextTo, dir;
    SSD::SimString lanefrom, laneto;
    SimOneAPI::GetNearMostLane(from, lanefrom, s, t, s_c, t_c);
    SimOneAPI::GetInertialFromLaneST(lanefrom, s + 1, t, PointNextFrom, dir);
    SimOneAPI::GetNearMostLane(to, laneto, s, t, s_c, t_c);
    SimOneAPI::GetInertialFromLaneST(laneto, s + 1, t, PointNextTo, dir);
    BuildLaneChangePath(int(UtilMath::distance(from, to)), from, PointNextFrom, to,
                        PointNextTo, path);
    return path;
}

/**
 * @brief Generates a lane change path around obstacles.
 * @param vel current speed.
 * @param obstacle target obstacle.
 * @param mainVehiclePos Main vehicle position.
 * @param changeToLaneName Target lane ID for lane change.
 * @param targetPath Receives the generated path.
 */
void GetChangeLanePath(const double &vel, const obstaclestruct &obstacle,
                       const SSD::SimPoint3D &mainVehiclePos,
                       const SSD::SimString &changeToLaneName,
                       SSD::SimPoint3DVector &targetPath) {
    targetPath.clear();
    SSD::SimString laneNow;
    SSD::SimPoint3D changeToPoint, changeToNextPoint, FromNextPoint;
    SSD::SimPoint3D dir;
    double tos, tot;
    if (SimOneAPI::GetLaneMiddlePoint(
            obstacle.pt, changeToLaneName, changeToPoint,
            dir))
    {
    }

    SimOneAPI::GetLaneST(changeToLaneName, changeToPoint, tos, tot);
    SimOneAPI::GetInertialFromLaneST(changeToLaneName, tos + vel * 4, tot,
                                     changeToPoint, dir);
    targetPath = ChangeLanePathWithPoint(mainVehiclePos, changeToPoint);
}

/**
 * @brief Gets all centerline sampling points of the specified lane.
 * @param laneId ID of the target lane.
 * @return Vector containing lane centerline sampling points.
 */
SSD::SimPoint3DVector GetLaneSample(const SSD::SimString &laneId) {
    SSD::SimPoint3DVector targetPath;
    HDMapStandalone::MLaneInfo info;
    if (SimOneAPI::GetLaneSample(laneId, info)) {
        for (auto &pt: info.centerLine) {
            targetPath.push_back(SSD::SimPoint3D(pt.x, pt.y, pt.z));
        }
    }
    return targetPath;
}

/**
 * @brief Get the real-time detection area around the main vehicle.
 * @param gps GPS data of the main vehicle.
 * @param s_front The longitudinal distance forward.
 * @param s_back The longitudinal distance backward.
 * @param t_left Horizontal distance to the left.
 * @param t_right The lateral distance to the right.
 * @return defines the 3D point vectors of the four corners of the detection area.
 */
SSD::SimPoint3DVector
GetDetectionZoneRealtime(const std::unique_ptr<SimOne_Data_Gps> &gps,
                       const double &s_front, const double &s_back,
                       const double &t_left, const double &t_right) {
    SSD::SimPoint3DVector detectZone;
    SSD::SimPoint3D Zone, dir;
    SSD::SimPoint3D mainVehiclePos(gps->posX, gps->posY, gps->posZ);
    SSD::SimString laneId = GetNearMostLane(mainVehiclePos);
    double s, t, S_front, S_back, T_left, T_right;

    SimOneAPI::GetLaneST(laneId, mainVehiclePos, s, t);
    T_left = t + t_left;
    T_right = t + t_right;
    S_front = s + s_front;
    S_back = s + s_back;

    SimOneAPI::GetInertialFromLaneST(laneId, S_front, T_right, Zone, dir);
    detectZone.push_back(Zone);
    SimOneAPI::GetInertialFromLaneST(laneId, S_front, T_left, Zone, dir);
    detectZone.push_back(Zone);
    SimOneAPI::GetInertialFromLaneST(laneId, S_back, T_left, Zone, dir);
    detectZone.push_back(Zone);
    SimOneAPI::GetInertialFromLaneST(laneId, S_back, T_right, Zone, dir);
    detectZone.push_back(Zone);

    return detectZone;
}

/**
 * @brief Get the detection area at a fixed position.
 * @param pos The 3D coordinate point of the center of the area.
 * @param s_front The longitudinal distance forward.
 * @param s_back The longitudinal distance backward.
 * @param t_left Horizontal distance to the left.
 * @param t_right The lateral distance to the right.
 * @return defines the 3D point vectors of the four corners of the detection area.
 */
SSD::SimPoint3DVector GetDetectionZoneFixed(const SSD::SimPoint3D &pos,
                                          const double &s_front,
                                          const double &s_back,
                                          const double &t_left,
                                          const double &t_right) {
    SSD::SimPoint3DVector detectZone;
    SSD::SimPoint3D Zone, dir;
    SSD::SimString laneId = GetNearMostLane(pos);
    std::cout << "laneId==" << laneId.GetString() << std::endl;

    SimOneAPI::GetInertialFromLaneST(laneId, s_front, t_right, Zone, dir);
    detectZone.push_back(Zone);
    SimOneAPI::GetInertialFromLaneST(laneId, s_front, t_left, Zone, dir);
    detectZone.push_back(Zone);
    SimOneAPI::GetInertialFromLaneST(laneId, s_back, t_left, Zone, dir);
    detectZone.push_back(Zone);
    SimOneAPI::GetInertialFromLaneST(laneId, s_back, t_right, Zone, dir);
    detectZone.push_back(Zone);

    return detectZone;
}

/**
 * @brief Calculate the longitudinal S distance between the main vehicle and the obstacle.
 * @param gps GPS data of the main vehicle.
 * @param obstacle target obstacle.
 * @param roadIdList The road ID list of the navigation path.
 * @return Longitudinal S distance.
 */
double GetSDistance(const std::unique_ptr<SimOne_Data_Gps> &gps,
                    const obstaclestruct &obstacle,
                    const SSD::SimVector<long> &roadIdList) {
    SSD::SimPoint3D mainVehiclePos(gps->posX, gps->posY, gps->posZ);
    SSD::SimString laneId = GetNearMostLane(mainVehiclePos);
    SSD::SimString obstacleLaneId = GetNearMostLane(obstacle.pt);

    HDMapStandalone::MLaneId carLaneId(laneId);
    HDMapStandalone::MLaneId obstacleLaneInfo(obstacleLaneId);

    double sDistance;
    double s, s_obstacle, t, z;
    SimOneAPI::GetRoadST(laneId, mainVehiclePos, s, t, z);
    SimOneAPI::GetRoadST(obstacleLaneId, obstacle.pt, s_obstacle, t, z);

    sDistance = s_obstacle - s;

    if (carLaneId.roadId <= obstacleLaneInfo.roadId) {
        HDMapStandalone::MLaneId current_id = carLaneId;
        while (current_id.roadId <= obstacleLaneInfo.roadId) {
            SSD::SimPoint3DVector centerline = GetLaneSample(current_id.ToString());
            double tempS;
            SimOneAPI::GetRoadST(current_id.ToString(), centerline.back(), tempS, t,
                                 z);
            sDistance += tempS;

            auto it =
                    std::find(roadIdList.begin(), roadIdList.end(), current_id.roadId);
            if (it != roadIdList.end() && std::next(it) != roadIdList.end()) {
                current_id.roadId = *(std::next(it));
            } else {
                break;
            }
        }
    } else {
        HDMapStandalone::MLaneId current_id = carLaneId;
        while (current_id.roadId >= obstacleLaneInfo.roadId) {
            SSD::SimPoint3DVector centerline = GetLaneSample(current_id.ToString());
            double tempS;
            SimOneAPI::GetRoadST(current_id.ToString(), centerline.back(), tempS, t,
                                 z);
            sDistance -= tempS;

            auto it =
                    std::find(roadIdList.begin(), roadIdList.end(), current_id.roadId);
            if (it != roadIdList.begin()) {
                current_id.roadId = *(std::prev(it));
            } else {
                break;
            }
        }
    }

    return sDistance;
}

/**
 * @brief Get the end point in the simulation environment.
 * @return 3D coordinates of the end point.
 */
SSD::SimPoint3D GetTerminalPoint() {
    SimOne_Data_WayPoints wayPoints;
    SSD::SimPoint3D endPt;
    const char *kMainVehicleId = "0";
    if (SimOneAPI::GetWayPoints(kMainVehicleId, &wayPoints)) {
        int waySize = wayPoints.wayPointsSize;
        endPt.x = wayPoints.wayPoints[waySize - 1].posX;
        endPt.y = wayPoints.wayPoints[waySize - 1].posY;
        endPt.z = 0;
    }
    return endPt;
}

/**
 * @brief Get the traffic lights associated with the current path.
 * @param laneId Current lane ID.
 * @param roadIdList The road ID list of the navigation path.
 * @return Target traffic light object.
 */
HDMapStandalone::MSignal GetTargetLight(const SSD::SimString &laneId, const SSD::SimVector<long> &roadIdList) {
    (void)laneId;
    HDMapStandalone::MSignal light;
    SSD::SimVector<HDMapStandalone::MSignal> lightList;
    SimOneAPI::GetTrafficLightList(lightList);


    for (auto &item: lightList) {
        int num = 0;

        for (auto &ptValidities: item.validities) {
            auto it = std::find(roadIdList.begin(), roadIdList.end(), ptValidities.roadId);
            if (it != roadIdList.end()) {
                ++num;
            }
        }

        if (num >= 2) {
            light = item;
            break;
        }
    }
    return light;
}

/**
 * @brief Get the associated stop line based on the traffic light.
 * @param light Traffic light object.
 * @param laneId Current lane ID.
 * @return The 3D coordinates of the center point of the stop line. If not found, a maximum coordinate is returned.
 */
SSD::SimPoint3D GetTargetStopLine(const HDMapStandalone::MSignal &light,
                                  const SSD::SimString &laneId) {
    SSD::SimVector<HDMapStandalone::MObject> stoplineList;
    SimOneAPI::GetStoplineList(light, laneId, stoplineList);
    if (stoplineList.empty()) {
        return SSD::SimPoint3D(std::numeric_limits<double>::max(), std::numeric_limits<double>::max(),
                               std::numeric_limits<double>::max());
    }
    SSD::SimPoint3D pt, dir;
    SimOneAPI::GetLaneMiddlePoint(stoplineList[0].pt, laneId, pt, dir);
    return pt;
}

/**
 * @brief Find the index of the point closest to the host vehicle in a long path.
 * @param longPath The point vector of the long path.
 * @param mainVehiclePos Main vehicle position.
 * @return The index of the nearest point.
 */
size_t IndexNumberOfLongPath(SSD::SimPoint3DVector &longPath, SSD::SimPoint3D &mainVehiclePos) {
    size_t Index = 0;
    std::vector<float> pts;
    for (auto &i: longPath) {
        pts.push_back(
                pow((float(mainVehiclePos.x) - (float) i.x), 2) + pow((float(mainVehiclePos.y) - (float) i.y), 2));
    }
    Index = std::min_element(pts.begin(), pts.end()) - pts.begin();
    return Index;
}

/**
 * @brief Cut the forward section from the long path as the short-term planning path.
 * @param index The current index of the main vehicle on the long path.
 * @param targetPath The complete long path.
 * @param mainVehiclePos Main vehicle position.
 * @return The extracted forward path points.
 */
SSD::SimPoint3DVector
GenerateForwardPoints(size_t &index, SSD::SimPoint3DVector &targetPath, SSD::SimPoint3D &mainVehiclePos) {
    SSD::SimPoint3DVector ForwardPoints;
    index = IndexNumberOfLongPath(targetPath, mainVehiclePos);
    const size_t endIndex = static_cast<size_t>(std::min(int(targetPath.size()), int(index + 100)));
    for (size_t i = index; i <= endIndex; i++) {
        ForwardPoints.push_back(targetPath[i]);
    }
    return ForwardPoints;
}

/**
 * @brief Generates a lane change path from the current position to a point on the target lane.
 * @param mainVehiclePos The current position of the main vehicle.
 * @param speed The current speed of the main vehicle.
 * @param changeToLane Target lane ID.
 * @param changeToPoint Receives the lane-change target point.
 * @return Generated lane change path point vector.
 */
SSD::SimPoint3DVector
ChangeLanePathWithLane(const SSD::SimPoint3D &mainVehiclePos, double speed,
                       const SSD::SimString &changeToLane,
                       SSD::SimPoint3D &changeToPoint) {
    double s, t, s_changePath;
    SSD::SimPoint3DVector changelane_path;
    SSD::SimPoint3D forwardPoint, dir;
    SSD::SimString currentlaneid = GetNearMostLane(mainVehiclePos);
    SimOneAPI::GetLaneST(currentlaneid, mainVehiclePos, s, t);
    s_changePath = s + std::max(speed * 3, 5.);
    SimOneAPI::GetInertialFromLaneST(currentlaneid, s_changePath, t, forwardPoint,
                                     dir);
    SimOneAPI::GetLaneMiddlePoint(forwardPoint, changeToLane, changeToPoint, dir);
    changelane_path = ChangeLanePathWithPoint(mainVehiclePos, changeToPoint);
    return changelane_path;
}

/**
 * @brief Generates a lane change path around obstacles.
 * @param mainVehiclePos The current position of the main vehicle.
 * @param changeToLane Target lane ID.
 * @param obstaclePos obstacle position.
 * @param turnTargetPoint Receives the lane-change target point.
 * @return Generated lane change path point vector.
 */
SSD::SimPoint3DVector ChangeLanePathWithObstacle(const SSD::SimPoint3D &mainVehiclePos,
                                                 const SSD::SimString &changeToLane,
                                                 SSD::SimPoint3D &obstaclePos,
                                                 SSD::SimPoint3D &turnTargetPoint) {
    SSD::SimPoint3DVector changelane_path;
    SSD::SimPoint3D dir;
    SimOneAPI::GetLaneMiddlePoint(obstaclePos, changeToLane, turnTargetPoint, dir);
    changelane_path = ChangeLanePathWithPoint(mainVehiclePos, turnTargetPoint);
    return changelane_path;
}

/**
 * @brief When there is no target path, generate a temporary path along the center line of the current lane based on the current position.
 * @param carPos The current position of the main car.
 * @param temporaryLine Receives the generated temporary path.
 */
void BuildLineWithoutTargetPath(SSD::SimPoint3D &carPos, SSD::SimPoint3DVector &temporaryLine) {
    HDMapStandalone::MLaneInfo laneinfo;
    SimOneAPI::GetLaneSampleByLocation(carPos, laneinfo);
    temporaryLine = laneinfo.centerLine;
}

/**
 * @brief (Debug) Print some points in the target path for debugging.
 * @param targetPath Point vector of the target path.
 */
void PrintTargetPath(SSD::SimPoint3DVector &targetPath) {
    std::cout << "targetPath.size()==  " << targetPath.size() << std::endl;
    for (size_t i = 0; i < targetPath.size(); i += 20) {
        std::cout << "Point[" << i << "]==  " << targetPath[i].x << ","
                  << targetPath[i].y << "," << targetPath[i].z << std::endl;
    }
}
