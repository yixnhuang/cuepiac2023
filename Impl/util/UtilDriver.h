#pragma once
#define USE_MATH_DEFINES
#include <cmath>
#include <algorithm>
#include <iostream>
#include <memory>
#include "SimOnePNCAPI.h"
#include "SimOneSensorAPI.h"
#include "SimOneHDMapAPI.h"
#include "Service/SimOneIOStruct.h"

#ifndef M_PI
#define M_PI 3.1415926
#endif

class UtilDriver
{
public:
	static void setDriver(long long& timestamp, const float& throttle, const float& brake, const float& steering)
	{
		std::unique_ptr<SimOne_Data_Control> pControl = std::make_unique<SimOne_Data_Control>();
		pControl->timestamp = timestamp;
		pControl->throttle = throttle;
		pControl->brake = brake;
		pControl->steering = steering;
                pControl->throttleMode=ESimOne_Throttle_Mode_Accel;
		pControl->handbrake = false;
		pControl->isManualGear = false;
		pControl->gear = static_cast<ESimOne_Gear_Mode>(1);
		SimOneAPI::SetDrive(0, pControl.get());
	}

    static double calculateSteering(const SSD::SimPoint3DVector& targetPath, SimOne_Data_Gps *pGps,double &headingErrorRad,double &turnHeadingErrorRad)
    {
        double ld,forwardIndex2;
        size_t forwardIndex;
        size_t index;
        // First, for each point on the target path, calculate the square of the distance from the current vehicle position to that point, and save these squared distances in an array named pts
        std::vector<float> pts;
        for (size_t i = 0; i < targetPath.size(); ++i){
            pts.push_back(pow((pGps->posX - (float)targetPath[i].x), 2) + pow((pGps->posY - (float)targetPath[i].y), 2));
        }

        // Then, find the index corresponding to the minimum value in the pts array, that is, find the index of the point on the target path that is closest to the current vehicle position. This index is stored in the variable index
        index = std::min_element(pts.begin(), pts.end()) - pts.begin();

        // Next, based on some preset parameters and the current speed of the vehicle, a "forward distance" (progDist) is calculated. This distance is used to select a forward point on the target path.
        forwardIndex = 0;
        forwardIndex2 = 0;
        float minProgDist = 5.f;
        float progTime1 = 0.8f;
        float progTime2 = 4.f;
        float mainVehicleSpeed = sqrtf(pGps->velX * pGps->velX + pGps->velY * pGps->velY + pGps->velZ * pGps->velZ);
        float progDist1 = mainVehicleSpeed * progTime1 > minProgDist ? mainVehicleSpeed * progTime1 : minProgDist;
        float progDist2 = mainVehicleSpeed * progTime2 > minProgDist ? mainVehicleSpeed * progTime2 : minProgDist;
        // Starting from the found index, traverse the target path and find the index of the first point whose distance is greater than or equal to the forward distance progDist. The index of this point is stored in the variable forwardIndex.
        for (; index < targetPath.size(); ++index)
        {
            float distance = sqrtf(((float)pow(targetPath[index].x - pGps->posX, 2) + pow((float)targetPath[index].y - pGps->posY, 2)));
            if(distance<=progDist1) {
              forwardIndex = index;
            }
            forwardIndex2 = index;
            if (distance >= progDist2)
            {
                break;
            }
        }

        // Calculate the angle headingErrorRad based on the vehicle's position, heading angle and the found forward point, which represents the angle between the vehicle and the forward point on the target path.
        auto psi = (double)pGps->oriZ;
        headingErrorRad = atan2(targetPath[forwardIndex].y - pGps->posY, targetPath[forwardIndex].x - pGps->posX) - psi;
        turnHeadingErrorRad=atan2(targetPath[forwardIndex2].y - pGps->posY, targetPath[forwardIndex2].x - pGps->posX) - psi;
        if (headingErrorRad > M_PI) {
            headingErrorRad = -2*M_PI + headingErrorRad;
        } else if (headingErrorRad < -M_PI) {
            headingErrorRad = 2*M_PI + headingErrorRad;
        }
        if (turnHeadingErrorRad > M_PI) {
            turnHeadingErrorRad = -2*M_PI + turnHeadingErrorRad;
        } else if (turnHeadingErrorRad < -M_PI) {
            turnHeadingErrorRad = 2*M_PI + turnHeadingErrorRad;
        }

        // Calculate variable ld, which represents the distance from the vehicle to the point ahead
        ld = sqrt(pow(targetPath[forwardIndex].y - pGps->posY, 2) + pow(targetPath[forwardIndex].x - pGps->posX, 2));

        // Finally, based on some geometric and physical parameters, calculate the steering wheel angle of the vehicle and return it
        double steering = -atan2(2. * (1.3 + 1.55) * sin(headingErrorRad), ld) * 36. / (7. * M_PI ) *1.7;

        return steering;
    }
};
