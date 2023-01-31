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
	sl::String path = sl::String(ofToDataPath("recordings/simple-rond-en-dedans-1.svo").c_str());
	init_parameters.input.setFromSVOFile(path);
	//init_parameters.camera_resolution = RESOLUTION::HD720;
	// On Jetson the object detection combined with an heavy depth mode could reduce the frame rate too much
	init_parameters.depth_mode = DEPTH_MODE::ULTRA;
	init_parameters.coordinate_system = COORDINATE_SYSTEM::RIGHT_HANDED_Y_UP;
	init_parameters.coordinate_units = UNIT::CENTIMETER;
	//init_parameters.camera_fps = 60;
	//init_parameters.sdk_verbose = 1;

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
	obj_det_params.detection_model = DETECTION_MODEL::HUMAN_BODY_ACCURATE;
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
	trainingClassLabelBody = 1;
	trainingClassLabelArms = 1;
	trainingClassLabelLegs = 1;
	predictedClassLabelBody = 0;
	predictedClassLabelArms = 0;
	predictedClassLabelLegs = 0;
	trainingModeBodyActive = false;
	trainingModeArmsActive = false;
	trainingModeLegsActive = false;
	trainingModeActive = false;
	grabFrame = true;
	predictionModeActiveBody = false;
	predictionModeActiveArms = false;
	predictionModeActiveLegs = false;
	predictionModeActive = false;
	drawInfo = true;
	body.resize(7 * 34);
	legs.resize(7 * 8);
	arms.resize(7 * 14);
	meta.resize(3);

	//The input to the training data will be the [x y z] from the left and right hand, so we set the number of dimensions to 6
	trainingDataBody.setNumDimensions(body.getSize());
	trainingDataArms.setNumDimensions(arms.getSize());
	trainingDataLegs.setNumDimensions(legs.getSize());
	trainingData.setNumDimensions(meta.getSize());

	//set the default classifier
	ANBC naiveBayesBody;
	naiveBayesBody.enableNullRejection(true);
	naiveBayesBody.setNullRejectionCoeff(5.0);
	pipelineBody << MovingAverageFilter(5, trainingDataBody.getNumDimensions());
	pipelineBody << EnvelopeExtractor(15, trainingDataBody.getNumDimensions());
	pipelineBody << naiveBayesBody;

	ANBC naiveBayesArms;
	naiveBayesArms.enableNullRejection(true);
	naiveBayesArms.setNullRejectionCoeff(5.0);
	pipelineArms << MovingAverageFilter(5, trainingDataArms.getNumDimensions());
	pipelineArms << EnvelopeExtractor(15, trainingDataArms.getNumDimensions());
	pipelineArms << naiveBayesArms;

	ANBC naiveBayesLegs;
	naiveBayesLegs.enableNullRejection(true);
	naiveBayesLegs.setNullRejectionCoeff(5.0);
	pipelineLegs << MovingAverageFilter(5, trainingDataLegs.getNumDimensions());
	pipelineLegs << EnvelopeExtractor(15, trainingDataLegs.getNumDimensions());
	pipelineLegs << naiveBayesLegs;

	ANBC naiveBayes;
	naiveBayes.enableNullRejection(true);
	naiveBayes.setNullRejectionCoeff(5.0);
	pipeline << MovingAverageFilter(5, trainingDataLegs.getNumDimensions());
	pipeline << EnvelopeExtractor(15, trainingDataLegs.getNumDimensions());
	pipeline << naiveBayes;


	bodyPlot.setup(500, body.getSize(), "body");
	bodyPlot.setDrawGrid(true);
	bodyPlot.setDrawInfoText(false);
	bodyPlot.setFont(smallFont);
	bodyPlot.setBackgroundColor(ofColor(255, 255, 255));

	armsPlot.setup(500, arms.getSize(), "arms");
	armsPlot.setDrawGrid(true);
	armsPlot.setDrawInfoText(false);
	armsPlot.setFont(smallFont);
	armsPlot.setBackgroundColor(ofColor(255, 255, 255));

	legsPlot.setup(500, legs.getSize(), "legs");
	legsPlot.setDrawGrid(true);
	legsPlot.setDrawInfoText(false);
	legsPlot.setFont(smallFont);
	legsPlot.setBackgroundColor(ofColor(255, 255, 255));

	metaPlot.setup(500, meta.getSize(), "meta");
	metaPlot.setDrawGrid(true);
	metaPlot.setDrawInfoText(false);
	metaPlot.setFont(smallFont);
	metaPlot.setBackgroundColor(ofColor(255, 255, 255));

	pos.set(2000, 0, 500);
	setup_comms();

	nodeBody.assign(34, ofNode());
	frame = 0;
	numFrames = zed.getSVONumberOfFrames();
	cam.setDistance(15);
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
	ERROR_CODE state = ERROR_CODE::FAILURE;
	if (grabFrame) {
		state = zed.grab();
		frame = (frame + 1) % numFrames;
	}
	
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


			j = 0;
			for (int i = 4; i <= 16; i++)
			{
				arms[j++] = zed_body.keypoint[i].x;
				arms[j++] = zed_body.keypoint[i].y;
				arms[j++] = zed_body.keypoint[i].z;
				arms[j++] = zed_body.local_orientation_per_joint[i].x;
				arms[j++] = zed_body.local_orientation_per_joint[i].y;
				arms[j++] = zed_body.local_orientation_per_joint[i].z;
				arms[j++] = zed_body.local_orientation_per_joint[i].w;
			}

			j = 0;
			for (int i = 18; i <= 25; i++)
			{
				legs[j++] = zed_body.keypoint[i].x;
				legs[j++] = zed_body.keypoint[i].y;
				legs[j++] = zed_body.keypoint[i].z;
				legs[j++] = zed_body.local_orientation_per_joint[i].x;
				legs[j++] = zed_body.local_orientation_per_joint[i].y;
				legs[j++] = zed_body.local_orientation_per_joint[i].z;
				legs[j++] = zed_body.local_orientation_per_joint[i].w;
			}
		}
	}
	else if (state == sl::ERROR_CODE::END_OF_SVOFILE_REACHED) {
		zed.setSVOPosition(0);
	}


	bodyPlot.update(body);
	armsPlot.update(arms);
	legsPlot.update(legs);
	


	if (trainingModeBodyActive) {
		//Add the current sample to the training data
		if (!trainingDataBody.addSample(trainingClassLabelBody, body)) {
			infoText = "WARNING: Failed to add training sample to training data!";
		}
	}

	if (trainingModeArmsActive) {
		if (!trainingDataArms.addSample(trainingClassLabelArms, arms)) {
			infoText = "WARNING: Failed to add training sample to training data!";
		}
	}

	if (trainingModeLegsActive) {
		if (!trainingDataLegs.addSample(trainingClassLabelLegs, legs)) {
			infoText = "WARNING: Failed to add training sample to training data!";
		}
	}

	//Update the prediction mode if active
	if (predictionModeActiveBody) {
		if (pipelineBody.predict(body)) {
			predictedClassLabelBody = pipelineBody.getPredictedClassLabel();
		}
		else {

			infoText = "ERROR: Failed to run prediction body!";
		}
	}
	else {
		pipelineBody.preProcessData(body);
	}

	if (predictionModeActiveArms) {
		if (pipelineArms.predict(arms)) {
			predictedClassLabelArms = pipelineArms.getPredictedClassLabel();
		}
		else {

			infoText = "ERROR: Failed to run prediction arms!";
		}
	}
	else {
		pipelineArms.preProcessData(arms);
	}

	if (predictionModeActiveLegs) {
		if (pipelineLegs.predict(legs)) {
			predictedClassLabelLegs = pipelineLegs.getPredictedClassLabel();
		}
		else {

			infoText = "ERROR: Failed to run prediction legs!";
		}
	}
	else {
		pipelineLegs.preProcessData(legs);
	}


	int j = 0;
	meta[j++] = predictedClassLabelBody;
	meta[j++] = predictedClassLabelArms;
	meta[j++] = predictedClassLabelLegs;
	metaPlot.update(meta);


	if (trainingModeActive) {
		if (!trainingData.addSample(trainingClassLabel, meta)) {
			infoText = "WARNING: Failed to add training sample to training data!";
		}
	}

	if (predictionModeActive) {
		if (pipeline.predict(meta)) {
			predictedClassLabel = pipeline.getPredictedClassLabel();
		}
		else {

			infoText = "ERROR: Failed to run prediction legs!";
		}
	}
	else {
		pipelineLegs.preProcessData(legs);
	}
}


//--------------------------------------------------------------
void ofApp::draw() {
	ofBackground(0, 0, 0);
	int marginX = 5;
	int marginY = 10;
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

		textY += textSpacer * 2;

		smallFont.drawString("[i]: Toogle Info", textX, textY); textY += textSpacer;
		smallFont.drawString("[r]: Toggle Recording", textX, textY); textY += textSpacer;
		smallFont.drawString("[t]: Train Model", textX, textY); textY += textSpacer;
		smallFont.drawString("[1,2,3]: Set Class Label", textX, textY); textY += textSpacer;

		textY += textSpacer;
		smallFont.drawString("Class Label Body: " + ofToString(trainingClassLabelBody), textX, textY); textY += textSpacer;
		smallFont.drawString("Class Label Arms: " + ofToString(trainingClassLabelArms), textX, textY); textY += textSpacer;
		smallFont.drawString("Class Label Legs: " + ofToString(trainingClassLabelLegs), textX, textY); textY += textSpacer;
		smallFont.drawString("Num Samples: " + ofToString(trainingDataBody.getNumSamples()), textX, textY); textY += textSpacer;
		smallFont.drawString("Num Samples: " + ofToString(trainingDataArms.getNumSamples()), textX, textY); textY += textSpacer;
		smallFont.drawString("Num Samples: " + ofToString(trainingDataLegs.getNumSamples()), textX, textY); textY += textSpacer;
		smallFont.drawString(infoText, textX, textY); textY += textSpacer;

		//Update the graph position
		graphX = infoX + infoW + 15;
		graphW = ofGetWidth() - graphX - 15;
	}

	//Draw the data graph
	bodyPlot.draw(graphX, graphY, graphW, graphH); graphY += graphH * 1.05;
	armsPlot.draw(graphX, graphY, graphW, graphH); graphY += graphH * 1.05;
	legsPlot.draw(graphX, graphY, graphW, graphH); graphY += graphH * 1.05;
	metaPlot.draw(graphX, graphY, graphW, graphH); graphY += graphH * 1.05;

	if (trainingModeBodyActive) {
		char strBuffer[1024];

		ofSetColor(255, 150, 0);
		sprintf(strBuffer, "Training Mode Active");

		std::string txt = strBuffer;
		ofRectangle bounds = hugeFont.getStringBoundingBox(txt, 0, 0);
		hugeFont.drawString(strBuffer, ofGetWidth() / 2 - bounds.width * 0.5, ofGetHeight() - bounds.height * 3);
	}

	if (trainingModeArmsActive) {
		char strBuffer[1024];

		ofSetColor(255, 150, 0);
		sprintf(strBuffer, "Training Mode Active");

		std::string txt = strBuffer;
		ofRectangle bounds = hugeFont.getStringBoundingBox(txt, 0, 0);
		hugeFont.drawString(strBuffer, ofGetWidth() / 2 - bounds.width * 0.5, ofGetHeight() - bounds.height * 2);
	}

	if (trainingModeLegsActive) {
		char strBuffer[1024];

		ofSetColor(255, 150, 0);
		sprintf(strBuffer, "Training Mode Active");

		std::string txt = strBuffer;
		ofRectangle bounds = hugeFont.getStringBoundingBox(txt, 0, 0);
		hugeFont.drawString(strBuffer, ofGetWidth() / 2 - bounds.width * 0.5, ofGetHeight() - bounds.height * 1);
	}

	//If the model has been trained, then draw the texture
	if (pipelineBody.getTrained() && predictionModeActiveBody) {
		std::string txt = "Predicted Class: " + ofToString(predictedClassLabelBody);
		ofRectangle bounds = hugeFont.getStringBoundingBox(txt, 0, 0);
		ofSetColor(0, 0, 255);
		hugeFont.drawString(txt, ofGetWidth() / 2 - bounds.width * 0.5, ofGetHeight() - bounds.height * 4);
	}
	if (pipelineArms.getTrained() && predictionModeActiveArms) {
		std::string txt = "Predicted Class: " + ofToString(predictedClassLabelArms);
		ofRectangle bounds = hugeFont.getStringBoundingBox(txt, 0, 0);
		ofSetColor(0, 0, 255);
		hugeFont.drawString(txt, ofGetWidth() / 2 - bounds.width * 0.5, ofGetHeight() - bounds.height * 3);
	}
	if (pipelineLegs.getTrained() && predictionModeActiveLegs) {
		std::string txt = "Predicted Class: " + ofToString(predictedClassLabelLegs);
		ofRectangle bounds = hugeFont.getStringBoundingBox(txt, 0, 0);
		ofSetColor(0, 0, 255);
		hugeFont.drawString(txt, ofGetWidth() / 2 - bounds.width * 0.5, ofGetHeight() - bounds.height * 2);
	}
	if (pipeline.getTrained() && predictionModeActive) {
		std::string txt = "Predicted Class: " + ofToString(predictedClassLabel);
		ofRectangle bounds = hugeFont.getStringBoundingBox(txt, 0, 0);
		ofSetColor(0, 0, 255);
		hugeFont.drawString(txt, ofGetWidth() / 2 - bounds.width * 0.5, ofGetHeight() - bounds.height * 1);
	}

	ofPopStyle();
	ofPopMatrix();

	ofPushMatrix();
	ofPushStyle();
	cam.begin(ofRectangle(0, graphY, ofGetWidth(), ofGetHeight() - graphY));
	for (auto& node : nodeBody) {
		ofPushMatrix();
		ofMultMatrix(node.getGlobalTransformMatrix());
		ofDrawBox(3);
		ofDrawAxis(6);
		ofPopMatrix();
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
	case 'q':
		predictionModeActiveBody = false;
		trainingModeBodyActive = !trainingModeBodyActive;
		break;
	case 'w':
		predictionModeActiveArms = false;
		trainingModeArmsActive = !trainingModeArmsActive;
		break;
	case 'e':
		predictionModeActiveLegs = false;
		trainingModeLegsActive = !trainingModeLegsActive;
		break;
	case 'n':
		predictionModeActive = false;
		trainingModeActive = !trainingModeActive;
		break;
	case '1':
		trainingClassLabelBody = 1;
		break;
	case '2':
		trainingClassLabelBody = 2;
		break;
	case '3':
		trainingClassLabelBody = 3;
		break;
	case '4':
		trainingClassLabelArms = 1;
		break;
	case '5':
		trainingClassLabelArms = 2;
		break;
	case '6':
		trainingClassLabelArms = 3;
		break;
	case '7':
		trainingClassLabelLegs = 1;
		break;
	case '8':
		trainingClassLabelLegs = 2;
		break;
	case '9':
		trainingClassLabelLegs = 3;
		break;
	case 't':
		if (pipelineBody.train(trainingDataBody)) {
			infoText = "Pipeline Trained";
			std::cout << "getNumClasses: " << pipelineBody.getNumClasses() << std::endl;
			predictionModeActiveBody = true;
		}
		break;
	case 'y':
		if (pipelineArms.train(trainingDataArms)) {
			infoText = "Pipeline Trained";
			std::cout << "getNumClasses: " << pipelineArms.getNumClasses() << std::endl;
			predictionModeActiveArms = true;
		}
		break;
	case 'u':
		if (pipelineLegs.train(trainingDataLegs)) {
			infoText = "Pipeline Trained";
			std::cout << "getNumClasses: " << pipelineLegs.getNumClasses() << std::endl;
			predictionModeActiveLegs = true;
		}
		else infoText = "WARNING: Failed to train pipeline";
		break;
	case 'm':
		if (pipeline.train(trainingData)) {
			infoText = "Pipeline Trained";
			std::cout << "getNumClasses: " << pipeline.getNumClasses() << std::endl;
			predictionModeActive = true;
		}
		else infoText = "WARNING: Failed to train pipeline";
		break;
	case 's':
		if (trainingDataBody.save(ofToDataPath("TrainingDataBody.grt"))) {
			infoText = "Training data saved to file";
		}
		if (trainingDataArms.save(ofToDataPath("TrainingDataArms.grt"))) {
			infoText = "Training data saved to file";
		}
		if (trainingDataLegs.save(ofToDataPath("TrainingDataLegs.grt"))) {
			infoText = "Training data saved to file";
		}
		if (trainingData.save(ofToDataPath("TrainingData.grt"))) {
			infoText = "Training data saved to file";
		}
		else infoText = "WARNING: Failed to save training data to file";
		break;
	case 'l':
		if (trainingDataBody.load(ofToDataPath("TrainingDataBody.grt"))) {
			infoText = "Training data saved to file";
		}
		if (trainingDataArms.load(ofToDataPath("TrainingDataArms.grt"))) {
			infoText = "Training data saved to file";
		}
		if (trainingDataLegs.load(ofToDataPath("TrainingDataLegs.grt"))) {
			infoText = "Training data saved to file";
		}
		if (trainingData.load(ofToDataPath("TrainingData.grt"))) {
			infoText = "Training data saved to file";
		}
		else infoText = "WARNING: Failed to load training data from file";
		break;
	case 'c':
		trainingDataBody.clear();
		trainingDataArms.clear();
		trainingDataLegs.clear();
		infoText = "Training data cleared";
		break;
	case 'i':
		drawInfo = !drawInfo;
		break;
	case ' ':
		grabFrame = !grabFrame;
		if (!grabFrame) {
			zed.setSVOPosition(frame);
		}
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
