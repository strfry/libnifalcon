//////////////////////////////////////////////////////////
// Novint Falcon Kinematics/Dynamics based on R.E.Stamper's PhD(1997)
// with some modifications
//
// Using LibniFalcon Beta 4
//
// Alastair Barrow 26/08/09


#include <iostream>
#include <string>
#include <cmath>
#include <sys/time.h>

#include "falcon/core/FalconDevice.h"
#include "falcon/firmware/FalconFirmwareNovintSDK.h"
#include "falcon/util/FalconCLIBase.h"
#include "falcon/util/FalconFirmwareBinaryNvent.h"

#include "falcon/kinematic/stamper/StamperUtils.h"

#include "falcon/kinematic/FalconKinematicStamper.h"

#include "falcon/core/FalconGeometry.h"
#include "falcon/gmtl/gmtl.h"

using namespace libnifalcon;
using namespace std;
using namespace StamperKinematicImpl;

FalconDevice falcon;


//////////////////////////////////////////////////////////
/// Ask libnifalcon to get the Falcon ready for action
/// nothing clever here, straight from the examples

extern "C"
bool initialize(int index)
{
	falcon.setFalconFirmware<FalconFirmwareNovintSDK>();

	cout << "Setting up comm interface for Falcon comms" << endl;

	unsigned int count;
	falcon.getDeviceCount(count);
	cout << "Connected Device Count: " << count << endl;

	//Open the device number:
	int deviceNum = index;
	cout << "Attempting to open Falcon device:  " << deviceNum << endl;
	if(!falcon.open(deviceNum))
	{
		cout << "Cannot open falcon device index " << deviceNum << " - Lib Error Code: " << falcon.getErrorCode() << " Device Error Code: " << falcon.getFalconComm()->getDeviceErrorCode() << endl;
		return false;
	}
	else
	{
		cout << "Connected to Falcon device " << deviceNum << endl ;
	}

	//Load the device firmware:
	//There's only one kind of firmware right now, so automatically set that.
	falcon.setFalconFirmware<FalconFirmwareNovintSDK>();
	//Next load the firmware to the device
	
	bool skip_checksum = false;
	//See if we have firmware
	bool firmware_loaded = false;
	firmware_loaded = falcon.isFirmwareLoaded();
	if(!firmware_loaded)
	{
		std::cout << "Loading firmware" << std::endl;
		uint8_t* firmware_block;
		long firmware_size;
		{

			firmware_block = const_cast<uint8_t*>(NOVINT_FALCON_NVENT_FIRMWARE);
			firmware_size = NOVINT_FALCON_NVENT_FIRMWARE_SIZE;


			for(int i = 0; i < 10; ++i)
			{
				if(!falcon.getFalconFirmware()->loadFirmware(skip_checksum, NOVINT_FALCON_NVENT_FIRMWARE_SIZE, const_cast<uint8_t*>(NOVINT_FALCON_NVENT_FIRMWARE)))

				{
					cout << "Firmware loading try failed";
					//Completely close and reopen
					//falcon.close();
					//if(!falcon.open(m_varMap["device_index"].as<int>()))
					//{
					//	std::cout << "Cannot open falcon device index " << m_varMap["device_index"].as<int>() << " - Lib Error Code: " << m_falconDevice->getErrorCode() << " Device Error Code: " << m_falconDevice->getFalconComm()->getDeviceErrorCode() << std::endl;
					//	return false;
					//}
				}
				else
				{
					firmware_loaded = true;
					break;
				}
			}
		}
	}
	else if(!firmware_loaded)
	{
		std::cout << "No firmware loaded to device, and no firmware specified to load (--nvent_firmware, --test_firmware, etc...). Cannot continue" << std::endl;
		//return false;
	}
	else
	{
		//return true;
	}
	if(!firmware_loaded || !falcon.isFirmwareLoaded())
	{
		std::cout << "No firmware loaded to device, cannot continue" << std::endl;
		//return false;
	}
	std::cout << "Firmware loaded" << std::endl;

	//Seems to be important to run the io loop once to be sure of sensible values next time:
	falcon.runIOLoop();

	falcon.getFalconFirmware()->setHomingMode(true);
	//falcon.setFalconKinematic<libnifalcon::FalconKinematicStamper>();

	return true;
}

auto kinematic = FalconKinematicStamper();

static gmtl::Vec3d position = gmtl::Vec3d(0.0,0.0,0.11);

extern "C"
int run_io() {
	
	//kinematic.initialize();

	//Ask libnifalcon to update the encoder positions and apply any forces waiting:
	falcon.runIOLoop();
	
	//////////////////////////////////////////////
	//Request the current encoder positions:
	std::array<int, 3> encoderPos;
	encoderPos = falcon.getFalconFirmware()->getEncoderValues();
	gmtl::Vec3d encoderAngles;
	encoderAngles[0] = falcon.getFalconKinematic()->getTheta(encoderPos[0]);
	encoderAngles[1] = falcon.getFalconKinematic()->getTheta(encoderPos[1]);
	encoderAngles[2] = falcon.getFalconKinematic()->getTheta(encoderPos[2]);
	encoderAngles *= 0.0174532925;	//Convert to radians
	
	
	////////////////////////////////////
	//Forward Kinematics
	kinematic.FK(encoderAngles, position);
	
	return 0;
	
}

extern "C"
void get_position(float* pos) {
	
	//////////////////////////////////////////////
	//
	// memcopy positions to pos 
	pos[0] = position[0]; pos[1] = position[1]; pos[2] = position[2];	
}
	

extern "C"
void set_force(float Fx, float Fy, float Fz)
{
#ifdef DEBUG
	printf("set_force(%f, %f, %f)\n", Fx, Fy, Fz);
#endif
	/////////////////////////////////////////
	//Inverse kinematics:
	Angle angles;

	kinematic.IK(angles, position);

	////////////////////////////////////////
	//Jacobian:
	gmtl::Matrix33d J;
	J = kinematic.jacobian(angles);

	//////////////////////////////////////////////
	//


	//Offset Z so position origin is roughly in the centre of the workspace:
	gmtl::Vec3d offsetPos(position);
	offsetPos[2] -= 0.11;

	//Reset the forces:
	
	//gmtl::Vec3d force(0.0,0.0,0.0);
	gmtl::Vec3d force(Fx, Fy, Fz);
	//printf("setforce %f %f %f\n", forces[0], forces[1], forces[2]);
	
	// memcopy pos to positions = 
	//positions[0] = pos[0]; positions[1] = pos[1]; positions[2] = pos[2];
	
	//printf("getpos %f %f %f\n", positions[0], positions[1], positions[2]);


	//Test shapes:
	
	//Room
/*
	if( offsetPos[0]<-0.03 )
		force[0] = -(offsetPos[0]+0.03);
	if( offsetPos[0]>0.03 )
		force[0] = -(offsetPos[0]-0.03);
	if( offsetPos[1]<-0.03 )
		force[1] = -(offsetPos[1]+0.03);
	if( offsetPos[1]>0.03 )
		force[1] = -(offsetPos[1]-0.03);
	if( offsetPos[2]<-0.02 )
		force[2] = -(offsetPos[2]+0.02);
	if( offsetPos[2]>0.02 )
		force[2] = -(offsetPos[2]-0.02);
		force *= 800.0;
*/

	//Sphere

	double distance = sqrt(gmtl::dot(offsetPos,offsetPos));
	if( distance<0.028  )
	{
		gmtl::Vec3d direction = offsetPos;
		direction /= distance;
		direction *= 0.028-distance;
	//	force = direction*800.0;
	}

	////////////////////////////////////////////
	//Dynamics

	//Convert force to motor torque values:
	J.setTranspose(J.getData());
	gmtl::Vec3d torque = J * force;


	//Now, we must scale the torques to avoid saturation of a motor
	//changing the ratio of torques and thus the force direction

	
	//Find highest torque:
	double maxTorque=30.0;	//Rather random choice here, could be higher
	double largestTorqueValue=0.0;
	int largestTorqueAxis=-1;
	for(int i=0; i<3; i++)
	{
		if(abs(torque[i])>largestTorqueValue)
		{
			largestTorqueValue=abs(torque[i]);
			largestTorqueAxis=i;
		}
	}
	//If axis with the largest torque is over the limit, scale them all to
	//bring it back to the limit:
	if(largestTorqueValue>maxTorque)
	{
		double scale = largestTorqueValue/maxTorque;
		torque /= scale;
	}
	

	//Convert torque to motor voltages:
	torque *= 10000.0;
	std::array<int, 3> enc_vec;
	enc_vec[0] = -torque[0];
	enc_vec[1] = -torque[1];
	enc_vec[2] = -torque[2];

	//And send them off to libnifalcon
	falcon.getFalconFirmware()->setForces(enc_vec);
}
