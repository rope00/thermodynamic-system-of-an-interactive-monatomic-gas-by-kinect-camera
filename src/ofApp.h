#pragma once

#include "ofMain.h"
#include "ofxOpenCv.h"
#include "ofxKinect.h"
#include "ofxKuRasterOptFlow.h"




class Particle : public ofBaseApp {
public:
	void setup();
	void updateByFlow(ofxKuRasterOptFlow &flow);
	void update();
	void draw();
	void setDebug(bool* debug);
	void setParticles(vector<Particle>* particles);
	bool isInteracting(Particle * particle);
	void updateParticles(Particle * particle2);
	// position (centre of mass)
	ofVec2f p;
	// velocity
	ofVec2f v;
	//force
	ofVec2f force;
	// mass
	float m;
	float radius;
	//physical equations
	float K; //average kinetic energy per gas molecule

	bool updated; // has this particle been updated
	bool * debug; // pointer to apps debug flag
	vector<Particle>* particles; // pointer to other particles
	
	ofxKuRasterOptFlow OptFlow;
	
};

class ofApp : public ofBaseApp{

	public:
		void setup();
		void setupinfo();
		void update();
		void draw();
		void drawinfo(ofEventArgs & args);
		void exit();
		void keyPressed(int key);
		bool debug = false; //!!!!!!!!!!!!!!!! please set up all variales start values // used for displaying debug information
		float K; // calculation of the total energy kinetic of the system by molecular gas
		float T;//calculation of the total temperature K of the system by molecular gas
		float P;//presure
		float V;// volume const of 1920x1080x1 pixels Screen dimension
		float cv;////heat capacity at constant volume
		float CV;////calculation heat capacity at constant volume
		float CP;////heat capacity at constant presure
		float U;// internal energy
		float Q;//heat
		float W;//work
		vector<Particle> particles; // map of all the particles

		ofxKinect kinect;

		ofxKuRasterOptFlow OptFlow;

		ofxCvGrayscaleImage grayImage; // grayscale depth image
		ofxCvGrayscaleImage grayThreshNear; // the near thresholded image
		ofxCvGrayscaleImage grayThreshFar; // the far thresholded image

		bool bThreshWithOpenCV;

		int nearThreshold;
		int farThreshold;

		int angle;
};

