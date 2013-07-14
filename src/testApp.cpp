#include "testApp.h"

float FEEDBACK_SCALE_AMOUNT = 1.02;

//--------------------------------------------------------------
void testApp::setup() {
    ofSetWindowShape(1230, 768);
	ofSetLogLevel(OF_LOG_VERBOSE);
	
	// enable depth->video image calibration
	kinect.setRegistration(true);
    
	kinect.init();

	kinect.open();		// opens first available kinect
    
	colorImg.allocate(kinect.width, kinect.height);
	grayImage.allocate(kinect.width, kinect.height);
	grayThreshNear.allocate(kinect.width, kinect.height);
	grayThreshFar.allocate(kinect.width, kinect.height);
	
	nearThreshold = 255;
	farThreshold = 117;
	bThreshWithOpenCV = true;
	
	ofSetFrameRate(60);
	
	// zero the tilt on startup
	angle = 0;
	
	// start from the front
	bDrawPointCloud = false;
    hue = 0;

    setupFbo(fbo);
    setupFbo(fbo2);
    useFbo2 = false;
}

void testApp::setupFbo(ofFbo& fbo) {
    fbo.allocate(kinect.width, kinect.height);
    fbo.begin();
    ofSetColor(0, 0, 0);
    ofRect(0, 0, fbo.getWidth(), fbo.getHeight());
    fbo.end();
}

//--------------------------------------------------------------
void testApp::update() {
	
	ofBackground(100, 100, 100);
	hue += 2;
    if (hue > 255) hue -= 255;
    if (hue < 0) hue = 0;
	kinect.update();
	
	// there is a new frame and we are connected
	if(kinect.isFrameNew()) {
		
		// load grayscale depth image from the kinect source
		grayImage.setFromPixels(kinect.getDepthPixels(), kinect.width, kinect.height);
		
		// we do two thresholds - one for the far plane and one for the near plane
		// we then do a cvAnd to get the pixels which are a union of the two thresholds
		if(bThreshWithOpenCV) {
			grayThreshNear = grayImage;
			grayThreshFar = grayImage;
			grayThreshNear.threshold(nearThreshold, true);
			grayThreshFar.threshold(farThreshold);
			cvAnd(grayThreshNear.getCvImage(), grayThreshFar.getCvImage(), grayImage.getCvImage(), NULL);
		} else {
			
			// or we do it ourselves - show people how they can work with the pixels
			unsigned char * pix = grayImage.getPixels();
			
			int numPixels = grayImage.getWidth() * grayImage.getHeight();
			for(int i = 0; i < numPixels; i++) {
				if(pix[i] < nearThreshold && pix[i] > farThreshold) {
					pix[i] = 255;
				} else {
					pix[i] = 0;
				}
			}
		}
		
		// update the cv images
		grayImage.flagImageChanged();

		// find contours which are between the size of 20 pixels and 1/3 the w*h pixels.
		// also, find holes is set to true so we will get interior contours as well....
		contourFinder.findContours(grayImage, 10, (kinect.width*kinect.height)/2, 20, false);

        contourPoly.clear();
        if (contourFinder.nBlobs > 0) {
            int biggestIndex = 0;
            for (int i = 1; i < contourFinder.nBlobs; ++i) {
                if (contourFinder.blobs[i].area > contourFinder.blobs[i - 1].area)
                    biggestIndex = i;
            }
            
            personPosition = contourFinder.blobs[biggestIndex].centroid;
            
            if (contourFinder.blobs[biggestIndex].pts.size() > 5) {
                ofPolyline tempPoly;
                tempPoly.addVertices(contourFinder.blobs[biggestIndex].pts);
                tempPoly.setClosed(true);
                
                ofPolyline smoothedTempPoly = tempPoly.getSmoothed(11, 0.5);
                if (!smoothedTempPoly.isClosed()) smoothedTempPoly.close();
                contourPoly.push_back(smoothedTempPoly);
            }
        }
	}
}


//--------------------------------------------------------------
void testApp::draw() {
	useFbo2 = !useFbo2;
	ofSetColor(255, 255, 255);
	
	if(bDrawPointCloud) {
		easyCam.begin();
		drawPointCloud();
		easyCam.end();
	} else {
		// draw from the live kinect
		kinect.drawDepth(10, 10, 400, 300);
		kinect.draw(10, 310, 400, 300);
		
//		contourFinder.draw(420, 10, 400*2, 300*2);
        if (useFbo2) {
            draw(fbo2, fbo);
        } else {
            draw(fbo, fbo2);
        }
	}

    drawInstructions();
}

void testApp::draw(ofFbo& ping, ofFbo& pong) {
    ping.begin();
    ofPushMatrix();
    // Scale around the person so that the feedback comes out uniformly
    // rather than away from the center of the screen.
    ofTranslate(personPosition.x, personPosition.y);
    ofScale(FEEDBACK_SCALE_AMOUNT, FEEDBACK_SCALE_AMOUNT);
    ofTranslate(-personPosition.x, -personPosition.y);
    pong.draw(0, 0);
    ofPopMatrix();
    ofEnableAlphaBlending();
    ofSetColor(0, 0, 0, 10);
    ofRect(0, 0, fbo.getWidth(), fbo.getHeight());
    
    ofSetColor(255,255,255);
    fill = ofColor::fromHsb(hue,255.0, 255.0/2);
    ofBeginShape();
    ofPath path;
    for(int i = 0; i < contourPoly.size(); ++i) {
        vector<ofPoint> points = contourPoly.at(i).getVertices();
        for (ofPoint p : points) {
            path.curveTo(p);
        }
    }
    path.setFillColor(fill);
    path.close();
    path.draw();
    ofEndShape();
    ping.end();
    ping.draw(410, 10);
}



void testApp::drawPointCloud() {
	int w = 640;
	int h = 480;
	ofMesh mesh;
	mesh.setMode(OF_PRIMITIVE_POINTS);
	int step = 2;
	for(int y = 0; y < h; y += step) {
		for(int x = 0; x < w; x += step) {
			if(kinect.getDistanceAt(x, y) > 0) {
				mesh.addColor(kinect.getColorAt(x,y));
				mesh.addVertex(kinect.getWorldCoordinateAt(x, y));
			}
		}
	}
	glPointSize(3);
	ofPushMatrix();
	// the projected points are 'upside down' and 'backwards' 
	ofScale(1, -1, -1);
	ofTranslate(0, 0, -1000); // center the points a bit
	glEnable(GL_DEPTH_TEST);
	mesh.drawVertices();
	glDisable(GL_DEPTH_TEST);
	ofPopMatrix();
}

//--------------------------------------------------------------
void testApp::exit() {
	kinect.setCameraTiltAngle(0); // zero the tilt on exit
	kinect.close();
}

//--------------------------------------------------------------
void testApp::keyPressed (int key) {
	switch (key) {
		case ' ':
			bThreshWithOpenCV = !bThreshWithOpenCV;
			break;
			
		case'p':
			bDrawPointCloud = !bDrawPointCloud;
			break;
			
		case '>':
		case '.':
			farThreshold ++;
			if (farThreshold > 255) farThreshold = 255;
			break;
			
		case '<':
		case ',':
			farThreshold --;
			if (farThreshold < 0) farThreshold = 0;
			break;
			
		case '+':
		case '=':
			nearThreshold ++;
			if (nearThreshold > 255) nearThreshold = 255;
			break;
			
		case '-':
			nearThreshold --;
			if (nearThreshold < 0) nearThreshold = 0;
			break;
			
		case 'w':
			kinect.enableDepthNearValueWhite(!kinect.isDepthNearValueWhite());
			break;
			
		case 'o':
			kinect.setCameraTiltAngle(angle); // go back to prev tilt
			kinect.open();
			break;
			
		case 'c':
			kinect.setCameraTiltAngle(0); // zero the tilt
			kinect.close();
			break;
			
		case '1':
			kinect.setLed(ofxKinect::LED_GREEN);
			break;
			
		case '2':
			kinect.setLed(ofxKinect::LED_YELLOW);
			break;
			
		case '3':
			kinect.setLed(ofxKinect::LED_RED);
			break;
			
		case '4':
			kinect.setLed(ofxKinect::LED_BLINK_GREEN);
			break;
			
		case '5':
			kinect.setLed(ofxKinect::LED_BLINK_YELLOW_RED);
			break;
			
		case '0':
			kinect.setLed(ofxKinect::LED_OFF);
			break;
			
		case OF_KEY_UP:
			angle++;
			if(angle>30) angle=30;
			kinect.setCameraTiltAngle(angle);
			break;
			
		case OF_KEY_DOWN:
			angle--;
			if(angle<-30) angle=-30;
			kinect.setCameraTiltAngle(angle);
			break;
	}
}

//--------------------------------------------------------------
void testApp::mouseDragged(int x, int y, int button)
{}

//--------------------------------------------------------------
void testApp::mousePressed(int x, int y, int button)
{}

//--------------------------------------------------------------
void testApp::mouseReleased(int x, int y, int button)
{}

//--------------------------------------------------------------
void testApp::windowResized(int w, int h)
{}

void testApp::drawInstructions() {
    // draw instructions
	ofSetColor(255, 255, 255);
	stringstream reportStream;
	reportStream << "accel is: " << ofToString(kinect.getMksAccel().x, 2) << " / "
	<< ofToString(kinect.getMksAccel().y, 2) << " / "
	<< ofToString(kinect.getMksAccel().z, 2) << endl
	<< "press p to switch between images and point cloud, rotate the point cloud with the mouse" << endl
	<< "using opencv threshold = " << bThreshWithOpenCV <<" (press spacebar)" << endl
	<< "set near threshold " << nearThreshold << " (press: + -)" << endl
	<< "set far threshold " << farThreshold << " (press: < >) num blobs found " << contourFinder.nBlobs
	<< ", fps: " << ofGetFrameRate() << endl
	<< "press c to close the connection and o to open it again, connection is: " << kinect.isConnected() << endl
	<< "press UP and DOWN to change the tilt angle: " << angle << " degrees" << endl
    << "number of blobs "<< contourFinder.nBlobs << endl 
	<< "press 1-5 & 0 to change the led mode (mac/linux only)" << endl;
	ofDrawBitmapString(reportStream.str(),20,652);
}
