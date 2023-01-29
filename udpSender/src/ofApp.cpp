#include "ofApp.h"

#define PRE_RECORDING_COUNTDOWN_TIME 2000
#define RECORDING_TIME 5000
#define HOST "192.168.125.100"
#define PORT 12345
//--------------------------------------------------------------
void ofApp::setup() {
	ofSetFrameRate(120);
	ofBackground(255, 255, 255);
	ofSetLogLevel(OF_LOG_VERBOSE);
	InitParameters init_parameters;

	init_parameters.svo_real_time_mode = true;
	sl::String path = sl::String(ofToDataPath("recordings/retire-tour-lente-4.svo").c_str());
	init_parameters.input.setFromSVOFile(path);
	init_parameters.camera_resolution = RESOLUTION::HD720;
	// On Jetson the object detection combined with an heavy depth mode could reduce the frame rate too much
	init_parameters.depth_mode = DEPTH_MODE::QUALITY;
	init_parameters.coordinate_system = COORDINATE_SYSTEM::RIGHT_HANDED_Y_UP;
	init_parameters.coordinate_units = UNIT::CENTIMETER;
	init_parameters.camera_fps = 60;
	init_parameters.sdk_verbose = 1;

	auto returned_state = zed.open(init_parameters);
	if (returned_state != ERROR_CODE::SUCCESS) {
		zed.close();
		ofExit(0);
	}

	//// Enable Positional tracking (mandatory for object detection)
	PositionalTrackingParameters positional_tracking_parameters;
	////If the camera is static, uncomment the following line to have better performances and boxes sticked to the ground.
	positional_tracking_parameters.set_as_static = true;
	returned_state = zed.enablePositionalTracking(positional_tracking_parameters);
	if (returned_state != ERROR_CODE::SUCCESS) {
		zed.close();
		ofExit(0);
	}

	ObjectDetectionParameters obj_det_params;
	obj_det_params.enable_tracking = true; // track people across images flow
	obj_det_params.enable_body_fitting = false;// smooth skeletons moves
	obj_det_params.body_format = sl::BODY_FORMAT::POSE_34;
	obj_det_params.detection_model = DETECTION_MODEL::HUMAN_BODY_FAST;
	returned_state = zed.enableObjectDetection(obj_det_params);
	if (returned_state != ERROR_CODE::SUCCESS) {
		zed.close();
		ofExit(0);
	}

	objectTracker_parameters_rt.detection_confidence_threshold = 40;


	largeFont.load("verdana.ttf", 12, true, true);
	largeFont.setLineHeight(14.0f);
	smallFont.load("verdana.ttf", 10, true, true);
	smallFont.setLineHeight(12.0f);
	hugeFont.load("verdana.ttf", 36, true, true);
	hugeFont.setLineHeight(38.0f);

	//Initialize the training and info variables
	infoText = "";
	trainingClassLabel = 1;
	predictedClassLabel = 0;
	trainingModeActive = false;
	recordTrainingData = false;
	predictionModeActive = false;
	drawInfo = true;
	body.resize(7 * 34);
	left_arm.resize(7 * 7);
	right_arm.resize(7 * 7);
	left_leg.resize(7 * 4);
	right_leg.resize(7 * 4);

	//The input to the training data will be the [x y z] from the left and right hand, so we set the number of dimensions to 6
	trainingDataBody.setNumDimensions(body.getSize());
	//trainingDataLeftArm.setNumDimensions(left_arm.getSize());
	//trainingDataRightArm.setNumDimensions(right_arm.getSize());
	//trainingDataLeftLeg.setNumDimensions(left_leg.getSize());
	//trainingDataRightLeg.setNumDimensions(right_leg.getSize());

	//set the default classifier
	ANBC naiveBayes;
	naiveBayes.enableNullRejection(true);
	naiveBayes.setNullRejectionCoeff(5.0);
	pipeline << MovingAverageFilter(5, trainingDataBody.getNumDimensions());
	pipeline << EnvelopeExtractor(15, trainingDataBody.getNumDimensions());
	pipeline << naiveBayes;

	bodyPlot.setup(500, body.getSize(), "body hand");
	bodyPlot.setDrawGrid(true);
	bodyPlot.setDrawInfoText(false);
	bodyPlot.setFont(smallFont);
	bodyPlot.setBackgroundColor(ofColor(255, 255, 255));
	colorBuffer = new unsigned char[zedWidth * zedHeight * 3];

	pos.set(2000, 0, 500);
	setup_comms();

	nodeBody.assign(34, ofNode());
}

void ofApp::setup_comms()
{
	// open an outgoing connection to HOST:PORT
	//create the socket and set to send to 127.0.0.1:11999
	ofxUDPSettings settings;
	settings.sendTo(HOST, PORT);
	settings.blocking = false;

	osc.setup("127.0.0.1", 7777);
	udpConnection.Setup(settings);
}

//--------------------------------------------------------------
void ofApp::update() {
	auto state = zed.grab();
	if (state == ERROR_CODE::SUCCESS) {

		// Retrieve Detected Human Bodies
		Objects bodies;
		zed.retrieveObjects(bodies, objectTracker_parameters_rt);
		std::vector<sl::ObjectData> objs = bodies.object_list;
		if (objs.size() > 0) {
			int closest = get_closest_body(objs);
			send_message(objs[closest], closest);
			auto zed_body = objs[closest];
			int j = 0;
			for (int i = 0; i < zed_body.keypoint.size(); i++)
			{
				body[j++] = zed_body.keypoint[i].x;
				body[j++] = zed_body.keypoint[i].y;
				body[j++] = zed_body.keypoint[i].z;
				body[j++] = zed_body.local_orientation_per_joint[i].x;
				body[j++] = zed_body.local_orientation_per_joint[i].y;
				body[j++] = zed_body.local_orientation_per_joint[i].z;
				body[j++] = zed_body.local_orientation_per_joint[i].w;

				nodeBody[i].setPosition(zed_body.keypoint[i].x, zed_body.keypoint[i].y, zed_body.keypoint[i].z);
			}
			
			bodyPlot.update(body);
			
			j = 0;
			for (int i = 4; i <= 10; i++) 
			{
				left_arm[j++] = zed_body.keypoint[i].x;
				left_arm[j++] = zed_body.keypoint[i].y;
				left_arm[j++] = zed_body.keypoint[i].z;
				left_arm[j++] = zed_body.local_orientation_per_joint[i].x;
				left_arm[j++] = zed_body.local_orientation_per_joint[i].y;
				left_arm[j++] = zed_body.local_orientation_per_joint[i].z;
				left_arm[j++] = zed_body.local_orientation_per_joint[i].w;	
			}

			left_arm_plot.update(left_arm);
			
			j = 0;
			for (int i = 11; i <= 16; i++)
			{
				right_arm[j++] = zed_body.keypoint[i].x;
				right_arm[j++] = zed_body.keypoint[i].y;
				right_arm[j++] = zed_body.keypoint[i].z;
				right_arm[j++] = zed_body.local_orientation_per_joint[i].x;
				right_arm[j++] = zed_body.local_orientation_per_joint[i].y;
				right_arm[j++] = zed_body.local_orientation_per_joint[i].z;
				right_arm[j++] = zed_body.local_orientation_per_joint[i].w;
			}

			right_arm_plot.update(right_arm);


			j = 0;
			for (int i = 18; i <= 21; i++)
			{
				left_leg[j++] = zed_body.keypoint[i].x;
				left_leg[j++] = zed_body.keypoint[i].y;
				left_leg[j++] = zed_body.keypoint[i].z;
				left_leg[j++] = zed_body.local_orientation_per_joint[i].x;
				left_leg[j++] = zed_body.local_orientation_per_joint[i].y;
				left_leg[j++] = zed_body.local_orientation_per_joint[i].z;
				left_leg[j++] = zed_body.local_orientation_per_joint[i].w;
			}

			left_leg_plot.update(left_leg);

			j = 0;
			for (int i = 22; i <= 25; i++)
			{
				right_leg[j++] = zed_body.keypoint[i].x;
				right_leg[j++] = zed_body.keypoint[i].y;
				right_leg[j++] = zed_body.keypoint[i].z;
				right_leg[j++] = zed_body.local_orientation_per_joint[i].x;
				right_leg[j++] = zed_body.local_orientation_per_joint[i].y;
				right_leg[j++] = zed_body.local_orientation_per_joint[i].z;
				right_leg[j++] = zed_body.local_orientation_per_joint[i].w;
			}

			right_leg_plot.update(right_leg);




			if (trainingModeActive) {

				//Add the current sample to the training data

				if (!trainingDataBody.addSample(trainingClassLabel, body)) {
					infoText = "WARNING: Failed to add training sample to training data!";
				}
				//if (!trainingDataLeftArm.addSample(trainingClassLabel, left_arm)) {
				//	infoText = "WARNING: Failed to add training sample to training data!";
				//}
				//if (!trainingDataRightArm.addSample(trainingClassLabel, right_arm)) {
				//	infoText = "WARNING: Failed to add training sample to training data!";
				//}
				//if (!trainingDataLeftLeg.addSample(trainingClassLabel, left_leg)) {
				//	infoText = "WARNING: Failed to add training sample to training data!";
				//}
				//if (!trainingDataRightLeg.addSample(trainingClassLabel, right_leg)) {
				//	infoText = "WARNING: Failed to add training sample to training data!";
				//}

			}

			//Update the prediction mode if active
			if (predictionModeActive) {
				if (pipeline.predict(body)) {
					predictedClassLabel = pipeline.getPredictedClassLabel();
					predictionPlot.update(pipeline.getClassLikelihoods());
				}
				else {
					
					infoText = "ERROR: Failed to run prediction!";
				}
			}
			else {
				pipeline.preProcessData(body);
			}
		}
	}
	else if (state == sl::ERROR_CODE::END_OF_SVOFILE_REACHED)
	{
		zed.setSVOPosition(0);
	}
}


//--------------------------------------------------------------
void ofApp::draw() {
	ofBackground(0, 0, 0);
	int marginX = 5;
	int marginY = 5;
	int graphX = marginX;
	int graphY = marginY;
	int graphW = ofGetWidth() - graphX * 2;
	int graphH = 100;


	ofPushMatrix();
	ofPushStyle();
	//Draw the info text
	if (drawInfo) {
		float infoX = marginX;
		float infoW = 250;
		float textX = 10;
		float textY = marginY;
		float textSpacer = smallFont.getLineHeight() * 1.5;

		ofFill();
		ofSetColor(100, 100, 100);
		ofDrawRectangle(infoX, 5, infoW, 225);
		ofSetColor(255, 255, 255);

		largeFont.drawString("GRT Classifier Example", textX, textY); textY += textSpacer * 2;

		smallFont.drawString("[i]: Toogle Info", textX, textY); textY += textSpacer;
		smallFont.drawString("[r]: Toggle Recording", textX, textY); textY += textSpacer;
		smallFont.drawString("[t]: Train Model", textX, textY); textY += textSpacer;
		smallFont.drawString("[1,2,3]: Set Class Label", textX, textY); textY += textSpacer;

		textY += textSpacer;
		smallFont.drawString("Class Label: " + ofToString(trainingClassLabel), textX, textY); textY += textSpacer;
		smallFont.drawString("Recording: " + ofToString(recordTrainingData), textX, textY); textY += textSpacer;
		smallFont.drawString("Num Samples: " + ofToString(trainingDataBody.getNumSamples()), textX, textY); textY += textSpacer;
		smallFont.drawString(infoText, textX, textY); textY += textSpacer;

		//Update the graph position
		graphX = infoX + infoW + 15;
		graphW = ofGetWidth() - graphX - 15;
	}

	//Draw the data graph
	bodyPlot.draw(graphX, graphY, graphW, graphH); graphY += graphH * 1.1;

	if (trainingModeActive) {
		char strBuffer[1024];

		ofSetColor(255, 150, 0);
		sprintf(strBuffer, "Training Mode Active");

		std::string txt = strBuffer;
		ofRectangle bounds = hugeFont.getStringBoundingBox(txt, 0, 0);
		hugeFont.drawString(strBuffer, ofGetWidth() / 2 - bounds.width * 0.5, ofGetHeight() - bounds.height * 3);
	}

	//If the model has been trained, then draw the texture
	if (pipeline.getTrained()) {
		predictionPlot.draw(graphX, graphY, graphW, graphH); graphY += graphH * 1.1;

		std::string txt = "Predicted Class: " + ofToString(predictedClassLabel);
		ofRectangle bounds = hugeFont.getStringBoundingBox(txt, 0, 0);
		ofSetColor(0, 0, 255);
		hugeFont.drawString(txt, ofGetWidth() / 2 - bounds.width * 0.5, ofGetHeight() - bounds.height * 3);
	}
	ofPopStyle();
	ofPopMatrix();

	ofPushMatrix();
	ofPushStyle();
	cam.begin();
	for (auto& node : nodeBody) {
		node.draw();
	}
	cam.end();
	ofPopStyle();
	ofPopMatrix();

}

int ofApp::get_closest_body(vector<sl::ObjectData> bodies)
{
	int index = -1;
	if (bodies.size() == 1)
	{
		return 0;
	}
	else
	{
		// Use the centroid of the 2 hands as the query point
		auto min_dist = FLT_MAX;
		auto origin = glm::vec3(0.0);
		for (int i = 0; i < bodies.size(); i++)
		{
			glm::vec3 center = glm::vec3(bodies[i].position.x, bodies[i].position.y, bodies[i].position.z);
			auto dist = glm::distance2(center, origin);
			if (dist < min_dist)
			{
				min_dist = dist;
				index = i;
			}
		}
	}
	return index;
}


void ofApp::send_message(sl::ObjectData body, int id)
{
	ofJson _body;
	ofJson _id;
	_id["id_zed"] = id;
	_body.push_back(_id);
	for (int i = 0; i < body.keypoint.size(); ++i)
	{
		ofJson _joint;
		_joint["name_zed"] = JOINT_NAMES[i];
		_joint["x"] = body.keypoint[i].x;
		_joint["y"] = body.keypoint[i].y;
		_joint["z"] = body.keypoint[i].z;
		_joint["qw"] = body.local_orientation_per_joint[i].w;
		_joint["qx"] = body.local_orientation_per_joint[i].x;
		_joint["qy"] = body.local_orientation_per_joint[i].y;
		_joint["qz"] = body.local_orientation_per_joint[i].z;

		_body.push_back(_joint);
	}
	auto message = _body.dump();
	//ofLog() << message << endl;
	udpConnection.Send(message.c_str(), message.length());
}

//--------------------------------------------------------------
void ofApp::keyPressed(int key) {
	infoText = "";
	bool buildTexture = false;

	switch (key) {
	case 'r':
		predictionModeActive = false;
		trainingModeActive = !trainingModeActive;
		break;
	case '1':
		trainingClassLabel = 1;
		break;
	case '2':
		trainingClassLabel = 2;
		break;
	case '3':
		trainingClassLabel = 3;
		break;
	case 't':
		if (pipeline.train(trainingDataBody)) {
			infoText = "Pipeline Trained";
			std::cout << "getNumClasses: " << pipeline.getNumClasses() << std::endl;
			predictionPlot.setup(500, pipeline.getNumClasses(), "prediction likelihoods");
			predictionPlot.setDrawGrid(true);
			predictionPlot.setDrawInfoText(true);
			predictionPlot.setFont(smallFont);
			predictionPlot.setBackgroundColor(ofColor(255, 255, 255));
			predictionModeActive = true;
		}
		else infoText = "WARNING: Failed to train pipeline";
		break;
	case 's':
		if (trainingDataBody.save(ofToDataPath("TrainingData.grt"))) {
			infoText = "Training data saved to file";
		}
		else infoText = "WARNING: Failed to save training data to file";
		break;
	case 'l':
		if (trainingDataBody.load(ofToDataPath("TrainingData.grt"))) {
			infoText = "Training data saved to file";
		}
		else infoText = "WARNING: Failed to load training data from file";
		break;
	case 'c':
		trainingDataBody.clear();
		infoText = "Training data cleared";
		break;
	case 'i':
		drawInfo = !drawInfo;
		break;
	default:
		break;
	}
}

//--------------------------------------------------------------
void ofApp::keyReleased(int key) {

}

//--------------------------------------------------------------
void ofApp::mouseMoved(int x, int y) {

}

//--------------------------------------------------------------
void ofApp::mouseDragged(int x, int y, int button) {

}

//--------------------------------------------------------------
void ofApp::mousePressed(int x, int y, int button) {

}

//--------------------------------------------------------------
void ofApp::mouseReleased(int x, int y, int button) {

}

//--------------------------------------------------------------
void ofApp::mouseEntered(int x, int y) {

}

//--------------------------------------------------------------
void ofApp::mouseExited(int x, int y) {

}

//--------------------------------------------------------------
void ofApp::windowResized(int w, int h) {

}

//--------------------------------------------------------------
void ofApp::gotMessage(ofMessage msg) {

}

//--------------------------------------------------------------
void ofApp::dragEvent(ofDragInfo dragInfo) {

}
