#include "ofApp.h"

//--------------------------------------------------------------
void ofApp::setup(){
    ofSetFrameRate(60);
    ofSetLogLevel(OF_LOG_VERBOSE);
    InitParameters init_parameters;
    init_parameters.camera_resolution = RESOLUTION::HD1080;
    // On Jetson the object detection combined with an heavy depth mode could reduce the frame rate too much
    init_parameters.depth_mode = DEPTH_MODE::PERFORMANCE;
    init_parameters.coordinate_system = COORDINATE_SYSTEM::RIGHT_HANDED_Z_UP_X_FWD;
    init_parameters.coordinate_units = UNIT::METER;
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
    obj_det_params.detection_model = DETECTION_MODEL::HUMAN_BODY_ACCURATE;
    returned_state = zed.enableObjectDetection(obj_det_params);
    if (returned_state != ERROR_CODE::SUCCESS) {
        zed.close();
        ofExit(0);
    }

    objectTracker_parameters_rt.detection_confidence_threshold = 40;

    osc.setup("127.0.0.1", 7777);
    cout << "here " << endl;
    pos.set(2000, 0, 500);
}

//--------------------------------------------------------------
void ofApp::update(){

}

//--------------------------------------------------------------
void ofApp::draw(){
    ofBackground(0, 0, 0);
    // Grab images
    if (zed.grab() == ERROR_CODE::SUCCESS) {
        // Retrieve Detected Human Bodies
        Objects bodies;
        zed.retrieveObjects(bodies, objectTracker_parameters_rt);
        std::vector<sl::ObjectData> objs = bodies.object_list;
        if (objs.size() > 0) {
            for (unsigned int i = 0; i < objs.size(); i++) {
                if (objs[i].keypoint.size() && i == 0) {
                    for (int j = 0; j < objs[i].keypoint.size(); i++) {
                        ofJson json;
                        
                    }
                    //sl::float3 neck = objs[i].keypoint[getIdx(sl::BODY_PARTS_POSE_34::RIGHT_HAND)];
                    //pos = ofVec3f(ofLerp(pos.x, neck.x, 0.1), ofLerp(pos.y, neck.y, 0.1), ofLerp(pos.z, neck.z, 0.1));
                    //ofLog() << pos << endl;

                }
            }
        }
        else {
            pos = ofVec3f(ofLerp(pos.x, 2000, 0.1), ofLerp(pos.y, 0, 0.1), ofLerp(pos.z, 500, 0.1));
        }

        ofxOscMessage m;
        m.setAddress("/body");
        m.addFloatArg(pos.x );
        m.addFloatArg(pos.y );
        m.addFloatArg(pos.z );
        osc.sendMessage(m);
        
    }
    else {

    }
}


void ofApp::send_message(sl::ObjectData body, int id)
{
    ofJson _body;
    ofJson _id;
    _id["id"] = id;
    _body.push_back(_id);
    for (int i = 0; i < body.keypoint.size(); ++i)
    {
        ofJson _joint;
        _joint["name"] = name;
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
    udpConnection.Send(message.c_str(), message.length());
}

//--------------------------------------------------------------
void ofApp::keyPressed(int key){

}

//--------------------------------------------------------------
void ofApp::keyReleased(int key){

}

//--------------------------------------------------------------
void ofApp::mouseMoved(int x, int y ){

}

//--------------------------------------------------------------
void ofApp::mouseDragged(int x, int y, int button){

}

//--------------------------------------------------------------
void ofApp::mousePressed(int x, int y, int button){

}

//--------------------------------------------------------------
void ofApp::mouseReleased(int x, int y, int button){

}

//--------------------------------------------------------------
void ofApp::mouseEntered(int x, int y){

}

//--------------------------------------------------------------
void ofApp::mouseExited(int x, int y){

}

//--------------------------------------------------------------
void ofApp::windowResized(int w, int h){

}

//--------------------------------------------------------------
void ofApp::gotMessage(ofMessage msg){

}

//--------------------------------------------------------------
void ofApp::dragEvent(ofDragInfo dragInfo){ 

}
