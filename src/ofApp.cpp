#include "ofApp.h"

void Particle::setup(){
	radius = 8;
	m = 39;
	// assign it's starting position
	p.x = ofRandom(radius, ofGetWidth() - radius);
	p.y = ofRandom(radius, ofGetHeight() - radius);
	// assign it's velocity
	v.x = 1.0;
	v.y = -1.0;
	
	
}
//-----------------------------------------------------------------------------------------------
void Particle::updateByFlow(ofxKuRasterOptFlow &flow) {
	float maxX = ofGetWidth();
	float maxY = ofGetHeight();

	//kinect returns frames not from first frame after app start, so we need to check if we have optical flow
	if (flow.w() > 0 && flow.h() > 0) {
		int indexX = int(p.x * flow.w() / maxX);	//index of flow value depending on particle's position
		int indexY = int(p.y * flow.h() / maxY);
		if (indexX >= 0 && indexX < flow.w() && indexY >= 0 && indexY < flow.h()) {
			int index = indexX + flow.w() * indexY;	//get index in array of flow
			ofPoint flowValue;
			flowValue.x = flow.flowX()[index];	//get flow's value
			flowValue.y = flow.flowX()[index];
	
			//apply flow to particle as a force
			v += flowValue * 2;		//2 is an amplification of action flow on particles, you can change this!!!!!!!!!
		}

	}
	v *= 0.999;		//ADDED SOME VISCOSITY FOR PARTICLES SLOWLY STOP

}
//-----------------------------------------------------------------------------------------------
void Particle::update(){
	float minX = 0;
	float minY = 0;
	float maxX = ofGetWidth();
	float maxY = ofGetHeight();
	ofVec2f reverseX(-1, 1);
	ofVec2f reverseY(1, -1);
	if (p.x > maxX || p.x < minX) {
		v *= reverseX;
	}
	if (p.y > maxY || p.y < minY) {
		v *= reverseY;
	}
	if (updated) {
		return;
	}
	for (int i = 0; i < particles->size(); i++) {
		if (p.x == particles->at(i).p.x &&
			p.y == particles->at(i).p.y) {
			continue;
		}
		Particle * particle2 = &particles->at(i);
		if (isInteracting(particle2)) {
			// Using the following model to update veloctiy and angular velocity:
			// http://www.euclideanspace.com/physics/dynamics/collision/twod/
			// get all the required information:
			// mass
			float m1 = m;
			float m2 = particle2->m;
			// radius
			float radius1 = radius;
			float radius2 = particle2->radius;
			// positions
			ofVec2f p1 = p;
			ofVec2f p2 = particle2->p;
			// velocitys
			ofVec2f v1 = v;
			ofVec2f v2 = particle2->v;

			// inertiass
			float i1 = (PI / 2)*pow(radius1, 4);
			float i2 = (PI / 2)*pow(radius2, 4);
			// relative vector of collision point to centre of mass
			ofVec2f r1 = (p1 - p2).scale(radius1 / radius2);
			ofVec2f r2 = (p2 - p1).scale(radius2 / radius1);
			// Impulse
			ofVec2f j;
			// e is the coefficient of restitution. It's fun to vary this.
			float e = 0.9;
			float k = 1 / (m1*m1) + 2 / (m1*m2) + 1 / (m2*m2) - r1.x*r1.x / (m1*i1) - r2.x*r2.x / (m1*i2) - r1.y*r1.y / (m1*i1)
				- r1.y*r1.y / (m2*i1) - r1.x*r1.x / (m2*i1) - r2.x*r2.x / (m2*i2) - r2.y*r2.y / (m1*i2)
				- r2.y*r2.y / (m2*i2) + r1.y*r1.y*r2.x*r2.x / (i1*i2) + r1.x*r1.x*r2.y*r2.y / (i1*i2) - 2 * r1.x*r1.y*r2.x*r2.y / (i1*i2);
			// set the impulse in the two dimensions
			j.x = (e + 1) / k * (v1.x - v2.x)*(1 / m1 - r1.x*r1.x / i1 + 1 / m2 - r2.x*r2.x / i2)
				- (e + 1) / k * (v1.y - v2.y)* (r1.x*r1.y / i1 + r2.x*r2.y / i2);
			j.y = -(e + 1) / k * (v1.x - v2.x) * (r1.x*r1.y / i1 + r2.x*r2.y / i2)
				+ (e + 1) / k * (v1.y - v2.y) * (1 / m1 - r1.y*r1.y / i1 + 1 / m2 - r2.y*r2.y / i2);
			// velocity after the collision
			ofVec2f v1f = v1 - j / m1;
			ofVec2f v2f = v2 + j / m2;
			// update this particles velocities
			v = v1f;
			particles->at(i).v = v2f;
			particles->at(i).p += particles->at(i).v;
			updated = true;
			particles->at(i).updated;
		}
	}
	// update the position using the particles velocity
	p += v;
	//kinetic energy of average translation of a gas molecule
	K = 0.5 * m * pow(v.length(), 2);// J
}
//-----------------------------------------------------------------------------------------------
void Particle::draw(){
	ofDrawCircle(p.x, p.y, radius);
	if (*debug) {
		ofDrawBitmapString(K, p.x + radius, p.y + radius);
	}
}
//-----------------------------------------------------------------------------------------------
void Particle::setDebug(bool * d){
	debug = d;
}
//-----------------------------------------------------------------------------------------------
void Particle::setParticles(vector<Particle>* p){
	particles = p;
}
//-----------------------------------------------------------------------------------------------
bool Particle::isInteracting(Particle * particle){
	// calculate the distance between two particles
	float d = p.distance(particle->p);
	if (d < radius + particle->radius) {
		return true;
	}
	return false;
}
//-----------------------------------------------------------------------------------------------
void Particle::updateParticles(Particle * particle2){
}
//-----------------------------------------------------------------------------------------------
void ofApp::setup(){
	ofSetFrameRate(60);

	const int NUM_PARTICLES = 300;
	for (int i = 0; i<NUM_PARTICLES; i++) {
		Particle particle;
		particle.setup();
		particles.push_back(particle);
	}
	for (int i = 0; i<particles.size(); i++) {
		particles[i].setDebug(&debug);
		particles[i].setParticles(&particles);
	}

	ofSetLogLevel(OF_LOG_VERBOSE);

	// enable depth->video image calibration
	kinect.setRegistration(true);

	kinect.init();
	//kinect.init(true); // shows infrared instead of RGB video image
	//kinect.init(false, false); // disable video image (faster fps)

	kinect.open();		// opens first available kinect
						//kinect.open(1);	// open a kinect by id, starting with 0 (sorted by serial # lexicographically))
						//kinect.open("A00362A08602047A");	// open a kinect using it's unique serial #

						// print the intrinsic IR sensor values
	if (kinect.isConnected()) {
		ofLogNotice() << "sensor-emitter dist: " << kinect.getSensorEmitterDistance() << "cm";
		ofLogNotice() << "sensor-camera dist:  " << kinect.getSensorCameraDistance() << "cm";
		ofLogNotice() << "zero plane pixel size: " << kinect.getZeroPlanePixelSize() << "mm";
		ofLogNotice() << "zero plane dist: " << kinect.getZeroPlaneDistance() << "mm";
	}


	grayImage.allocate(kinect.width, kinect.height);
	grayThreshNear.allocate(kinect.width, kinect.height);
	grayThreshFar.allocate(kinect.width, kinect.height);

	OptFlow.setup(kinect.width, kinect.height);
	
	//ofxKuBox2dWorldParams prm;
	//particleWord.setup(prm);

	nearThreshold = 230;
	farThreshold = 150;
	bThreshWithOpenCV = false;

	ofSetFrameRate(60);

	// zero the tilt on startup
	angle = 90;
	kinect.setCameraTiltAngle(angle);
}
//-----------------------------------------------------------------------------------------------
void ofApp::setupinfo(){
	ofBackground(0);
}
//-----------------------------------------------------------------------------------------------
void ofApp::update() {

	kinect.update();
	

	// there is a new frame and we are connected
	if (kinect.isFrameNew()) {

		// load grayscale depth image from the kinect source
		grayImage.setFromPixels(kinect.getDepthPixels());
		// we do two thresholds - one for the far plane and one for the near plane
		// we then do a cvAnd to get the pixels which are a union of the two thresholds
		if (bThreshWithOpenCV) {
			grayThreshNear = grayImage;
			grayThreshFar = grayImage;
			grayThreshNear.threshold(nearThreshold, true);
			grayThreshFar.threshold(farThreshold);
			cvAnd(grayThreshNear.getCvImage(), grayThreshFar.getCvImage(), grayImage.getCvImage(), NULL);
		}
		else {

			ofPixels & pix = grayImage.getPixels();
			int numPixels = pix.size();
			for (int i = 0; i < numPixels; i++) {
				if (pix[i] < nearThreshold && pix[i] > farThreshold) {
					pix[i] = 255;
				}
				else {
					pix[i] = 0;
				}
			}
			vector<unsigned char> mask(kinect.width * kinect.height);
			for (int i = 0; i < numPixels; i++) {
				mask[i] = pix[i];
			}
			
			OptFlow.update(mask, kinect.width, kinect.height);

			//for (int i = 0; i < numPixels; i++) {
				//mask[i] = (mask[i] < 128) ? 0 : 255;
			//}
			//particleWord.applyMask(mask, kinect.width, kinect.height);
			
			// update the cv images
			grayImage.flagImageChanged();
		}
		
		for (int i = 0; i < particles.size(); i++) {
			particles[i].updated = false;
		}

		for (int i = 0; i < particles.size(); i++) {
			particles[i].updateByFlow(OptFlow);
		}


		for (int i = 0; i < particles.size(); i++) {
			particles[i].update();
		}
		K = 0;
		for (int i = 0; i < particles.size(); i++) {
			K += particles[i].K;
			//temperature for one mole of ideal gas---> T= 2*K/(3nR) n=1
			float R = 8.314472; // J/mol*K
			cv = 1.5*R; //J/mol*K
			T = K / cv; // K \ K= (3/2)RT , 1/cv = 0.7/R
						//calculation of the total pressure of the system
			V = 548633.09;//volume Litre L
			P = 0.7*K / V; // Atm   1atm = 101325 pa
						   //calculation of the internal energy of the system
			U = K; // U = Cv * T-- > U = 3 / 2 * R*T & T = (2 / 3 * R)*K
				   // if dU = dW + dQ & dW = -pdV but V = const implies that
				   //dV = 0 then dW = 0 implies that dU = dQ
				   //U = cv * T;
				   //Q = cv * T;
			Q = U;
			W = Q - U;
			CV = Q / T;
			//CP = CV + R;
		}

	}
}
//-----------------------------------------------------------------------------------------------
void ofApp::draw() {
	ofSetColor(255);

	float w = ofGetWidth();
	float h = ofGetHeight();


	kinect.drawDepth(0, 0, w, h);
	//depthtexture.draw(0, 0, w, h);  
	OptFlow.draw(0, 0, w, h, 10, 5);
	ofSetColor(255);
	for (int i = 0; i < particles.size(); i++) {
		particles[i].draw();
	}

}
//-------------------------------------------------------------------------------------------------
void ofApp::drawinfo(ofEventArgs & args){
	ofSetColor(255);
	string info = "FPS:        " + ofToString(ofGetFrameRate(), 0) + "\n";
	info += "Timer:      " + ofToString(ofGetElapsedTimeMillis() / 1e+3, 2) + " seconds\n";
	info += "----------------------------System data------------------------"     "\n";
	info += "Molar calorific capacity at constant volume Cv: " + ofToString(CV, 2) + " J/mol*K\n";
	//info += "Molar calorific capacity at constant presure Cp: " + ofToString(CP, 2) + " J/mol*K\n";
	info += "Total Kinetic Energy: " + ofToString(K, 2) + " J\n";
	info += "Total Temperature: " + ofToString(T / 1e+2, 2) + " K\n";
	info += "Total Presure: " + ofToString(P, 2) + " Atm\n";
	//info += "Internal Energy: "      + ofToString(U / 1e+2, 2) + " J\n";
	//info += "Heat: "                 + ofToString(Q / 1e+2, 2) + " J\n";
	info += "Work: "                 + ofToString(W, 2) + " J\n";
	ofDrawBitmapString(info, 20, 20);
}
//-----------------------------------------------------------------------------------------------
void ofApp::exit(){
	kinect.setCameraTiltAngle(0); // zero the tilt on exit
	kinect.close();
}
//-----------------------------------------------------------------------------------------------
void ofApp::keyPressed(int key){
	switch (key) {
	case 100: // d
		debug = !debug;
	}
}
//-----------------------------------------------------------------------------------------------
