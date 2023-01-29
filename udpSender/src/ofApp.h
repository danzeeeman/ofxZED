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
		ClassificationData trainingDataArms;
		ClassificationData trainingDataLegs;//This will store our training data
		GestureRecognitionPipeline pipelineBody;       
		GestureRecognitionPipeline pipelineLegs;
		GestureRecognitionPipeline pipelineArms;  //This is a wrapper for our classifier and any pre/post processing modules 
		bool recordTrainingData;                                //This is a flag that keeps track of when we should record training data
		bool trainingModeBodyActive;
		bool trainingModeArmsActive;
		bool trainingModeLegsActive;
		bool predictionModeActiveBody;
		bool predictionModeActiveArms;
		bool predictionModeActiveLegs;
		bool drawInfo;
		VectorFloat body;
		VectorFloat arms;
		VectorFloat legs;
		UINT trainingClassLabelBody;  
		UINT trainingClassLabelArms;  
		UINT trainingClassLabelLegs;  //This will hold the current label for when we are training the classifier
		UINT predictedClassLabelBody;
		UINT predictedClassLabelArms;
		UINT predictedClassLabelLegs;
		int frame;
		int numFrames;
		string infoText;                            //This string will be used to draw some info messages to the main app window
		ofTrueTypeFont largeFont;
		ofTrueTypeFont smallFont;
		ofTrueTypeFont hugeFont;
		ofxGrtTimeseriesPlot bodyPlot;
		ofxGrtTimeseriesPlot armsPlot;
		ofxGrtTimeseriesPlot legsPlot;
		Timer trainingTimer;
		int zedHeight, zedWidth;
		unsigned char* colorBuffer;

		vector<ofNode> nodeBody;
		ofEasyCam cam;
		bool grabFrame;
		
};
