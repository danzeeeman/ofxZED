#pragma once

#include "ofMain.h"
#include <sl/Camera.hpp>
#include "ofxOsc.h"
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

		ofxOscSender osc;
		Camera zed;
		ObjectDetectionRuntimeParameters objectTracker_parameters_rt;
		ofVec3f pos;

		
};
