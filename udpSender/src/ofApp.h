#pragma once

#include "ofMain.h"
#include <sl/Camera.hpp>
#include "ofxOsc.h"
#include "ofxNetwork.h"
#include "ofxGrt.h"
using namespace sl;
class ofApp : public ofBaseApp{

	public:
		void setup();
		void update();
		void draw();

		void send_message(sl::ObjectData body, int id);

		void keyPressed(int key);
		void keyReleased(int key);
		void mouseMoved(int x, int y );
		void mouseDragged(int x, int y, int button);
		void mousePressed(int x, int y, int button);
		void mouseReleased(int x, int y, int button);
		void mouseEntered(int x, int y);
		void mouseExited(int x, int y);
		void windowResized(int w, int h);
		void dragEvent(ofDragInfo dragInfo);
		void gotMessage(ofMessage msg);

		// Communication Parameters
		ofxUDPManager udpConnection;
		void setup_comms();

		ofJson data;

		int get_closest_body(vector<sl::ObjectData> bodies);

		std::vector<std::string> JOINT_NAMES{
	"PELVIS",      "NAVAL_SPINE", "CHEST_SPINE",   "NECK",        "LEFT_CLAVICLE",  "LEFT_SHOULDER",  "LEFT_ELBOW",
	"LEFT_WRIST",  "LEFT_HAND",   "LEFT_HANDTIP",  "LEFT_THUMB",  "RIGHT_CLAVICLE", "RIGHT_SHOULDER", "RIGHT_ELBOW",
	"RIGHT_WRIST", "RIGHT_HAND",  "RIGHT_HANDTIP", "RIGHT_THUMB", "LEFT_HIP",       "LEFT_KNEE",      "LEFT_ANKLE",
	"LEFT_FOOT",   "RIGHT_HIP",   "RIGHT_KNEE",    "RIGHT_ANKLE", "RIGHT_FOOT",     "HEAD",           "NOSE",
	"LEFT_EYE",    "LEFT_EAR",    "RIGHT_EYE",     "RIGHT_EAR",   "LEFT_HEEL",      "RIGHT_HEEL"
		};

		ofxOscSender osc;
		Camera zed;
		ObjectDetectionRuntimeParameters objectTracker_parameters_rt;
		ofVec3f pos;

		//Create some variables for the demo
		ClassificationData trainingDataBody;
		ClassificationData trainingDataLeftArm;
		ClassificationData trainingDataRightArm;
		ClassificationData trainingDataLeftLeg; 
		ClassificationData trainingDataRightLeg; //This will store our training data
		GestureRecognitionPipeline pipeline;        //This is a wrapper for our classifier and any pre/post processing modules 
		bool recordTrainingData;                                //This is a flag that keeps track of when we should record training data
		bool trainingModeActive;
		bool predictionModeActive;
		bool drawInfo;
		VectorFloat body;
		VectorFloat left_arm;
		VectorFloat right_arm;
		VectorFloat left_leg;
		VectorFloat right_leg;
		UINT trainingClassLabel;                    //This will hold the current label for when we are training the classifier
		UINT predictedClassLabel;
		string infoText;                            //This string will be used to draw some info messages to the main app window
		ofTrueTypeFont largeFont;
		ofTrueTypeFont smallFont;
		ofTrueTypeFont hugeFont;
		ofxGrtTimeseriesPlot bodyPlot;
		ofxGrtTimeseriesPlot left_arm_plot;
		ofxGrtTimeseriesPlot right_arm_plot;
		ofxGrtTimeseriesPlot left_leg_plot;
		ofxGrtTimeseriesPlot right_leg_plot;
		ofxGrtTimeseriesPlot predictionPlot;
		Timer trainingTimer;
		int zedHeight, zedWidth;
		unsigned char* colorBuffer;

		vector<ofNode> nodeBody;
		ofEasyCam cam;
		
};
