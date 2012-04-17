//============================================================================
// Name        : KinectBiclops.cpp
// Author      : Dennis Lui
// Version     :
// Copyright   : 
// Description : Hello World in C++, Ansi-style
//============================================================================
#include <math.h>       // for sqrt()
#include <iostream>
#include <fstream>      // for ifstream
using namespace std;
#include <string.h>  // for strcpy, strcmp

#include "PMDUtils.h"   // access to file parsing
#include "PMDCollections.h"
#include "PMDGet.h"
#include "Biclops.h"    // class being implemented here
#include "kinect.h"
#include "utilities/utils.h"

// Definitions to make debug messages a little more succinct
static char dbgClass[] = "BiclopsBareBones::";
#define coutDbg cout << dbgClass << dbgMethod

/********************PAN & TILT ANGLES SPECIFIED for BICLOPS******************************************/
static double tiltAngles[] = {0.0,-30.0, 30.0};
//static double tiltAngles[] = {0.0};
static double panAngles[] = {-150.0, -120.0, -90.0, -60.0, -30.0, 0.0, 30.0, 60.0, 90.0, 120.0, 150.0, 180.0};
//static double panAngles[] = {-150.0, -120.0};
/******************************************************************************************************/

int main() {
	Biclops biclops;

	// Create a debug header message for use in this example application.
	const char dbgMethod[] = "main: ";
	char dbgHdr[32];
	sprintf(dbgHdr,"BiclopsPMDTest::main:");

	// Defines which axes we want to use.
	const int axisMask = Biclops::PanMask
					   + Biclops::TiltMask;

	// Pointers to each axis (populated once controller is initialized).
	PMDAxisControl *panAxis = NULL;
	PMDAxisControl *tiltAxis = NULL;

	// Set Debug level depending on how much info you want to see about
	// the inner workings of the API. Level 2 is highest with 0 being
	// the default (i.e., no messages).
	biclops.SetDebugLevel(2);

	//setting up the Kinect
	Mat rgbMat(Size(640,480),CV_8UC3);
	Mat depthMat(Size(640,480),CV_16UC1);
	//specify path to calibration file
	char calib_filepath[] = "calib/kinect_calibration2.yml";
	//initialize the kinect
	kinect myKinect(0, calib_filepath, false);
	myKinect.start();

	coutDbg << "Initializing Biclops\n";
	if (biclops.Initialize("data/BiclopsDefault.cfg")) {

		cout << "Initializing Kinect" << endl;
		for(int i=0;i<10;i++)
		{
			myKinect.getVideo(rgbMat);
			myKinect.getDepth(depthMat);
		}

		// Initialize the Biclops unit.
		coutDbg << "Initializing Biclops\n";
		// Initialization completed successfully.
		coutDbg << "Biclops initialized\n";
		biclops.SetDebugLevel(0);

		// Get shortcut references to each axis.
		panAxis = biclops.GetAxis(Biclops::Pan);
		tiltAxis = biclops.GetAxis(Biclops::Tilt);

		coutDbg << "Begin homing sequence." << endl;
		if (biclops.HomeAxes(axisMask,true)) {

			// Get the currently defined (default) motion profiles.
			PMDAxisControl::Profile panProfile,tiltProfile;
			panAxis->GetProfile(panProfile);
			tiltAxis->GetProfile(tiltProfile);

			// Set a position to move to by modifying the respective profiles.
			// NOTE: profile values are in revolutions, so here we convert
			// from degrees (divide by 360) for readability.
			panProfile.pos = PMDUtils::DegsToRevs(panAngles[0]);
			tiltProfile.pos = PMDUtils::DegsToRevs(tiltAngles[0]);

			// Inform the controller of the new desired position.
			panAxis->SetProfile(panProfile);
			tiltAxis->SetProfile(tiltProfile);

			// Move one axis to its destination in series.
			coutDbg << "Individual axis move" << endl;
			panAxis->Move();
			tiltAxis->Move();
			coutDbg << "Individual axis move complete" << endl;

			//create folder to store the Kinect data
			int fCount = 0;
			string mainFolder = "Scan";
			std::ostringstream folderPath;
			for(;;)
			{
				folderPath.str("");
				folderPath << mainFolder << fCount;
				if(!utils::checkDirectory(folderPath.str(), false))
				{
					utils::checkDirectory(folderPath.str(), true);
					break;
				}
				fCount++;
			}

			for(int i=0;i<sizeof(tiltAngles)/sizeof(double);i++)
			{
				for(int j=0;j<sizeof(panAngles)/sizeof(double);j++)
				{

					// Pick a new destination and tell the controller.
					panProfile.pos = PMDUtils::DegsToRevs(panAngles[j]);
					panAxis->SetProfile(panProfile);
					tiltProfile.pos = PMDUtils::DegsToRevs(tiltAngles[i]);
					tiltAxis->SetProfile(tiltProfile);

					coutDbg << "Moving to Pan: " << panAngles[j] << " Tilt: " << tiltAngles[i] << endl;
					// Issue the move command for all the axes simultaneously.
					coutDbg << "Multiple axis move" << endl;
					biclops.Move(axisMask);
					coutDbg << "Multiple axis move complete" << endl;
					if(j==0)
						sleep(4);
					else
						sleep(2);

					//creating folders and subfolders to store the images
					std::ostringstream subFolderPath, rgbFolderPath, depthFolderPath;
					subFolderPath << folderPath.str() << "/P" << panAngles[j] << "T" << tiltAngles[i];
					rgbFolderPath << subFolderPath.str() << "/rgb";
					depthFolderPath << subFolderPath.str() << "/depth";
					utils::checkDirectory(subFolderPath.str(), true);
					utils::checkDirectory(rgbFolderPath.str(), true);
					utils::checkDirectory(depthFolderPath.str(), true);

					//grabbing kinect frames
					for(int k=0;k<10;k++)
					{
						myKinect.getVideo(rgbMat);
						myKinect.getDepth(depthMat);

						//save kinect frames to folder
						std::ostringstream rgbPath, depthPath;
						rgbPath << rgbFolderPath.str() << "/rgb" << k << ".png";
						depthPath << depthFolderPath.str() << "/depth" << k << ".png";
						cv::imwrite(rgbPath.str().c_str(),rgbMat);
						cv::imwrite(depthPath.str().c_str(),depthMat);
					}
				}
			}

			//back to home position
			panProfile.pos = PMDUtils::DegsToRevs(0);
			panAxis->SetProfile(panProfile);
			tiltProfile.pos = PMDUtils::DegsToRevs(0);
			tiltAxis->SetProfile(tiltProfile);
			biclops.Move(axisMask);

			// Turn off motor amps to conserve power.
			panAxis->DisableAmp();
			tiltAxis->DisableAmp();
		}
		else {
			coutDbg << "initialization failed" << endl;
		}
	}

	// Stop the Kinect
	myKinect.stop();

	return 0;
}
