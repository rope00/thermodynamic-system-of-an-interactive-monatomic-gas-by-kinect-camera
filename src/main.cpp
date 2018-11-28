#include "ofMain.h"
#include "ofApp.h"
#include "ofAppGLFWWindow.h"


//========================================================================
int main( ){
	//ofSetupOpenGL(1024,768,OF_WINDOW);			// <-------- setup the GL context

	// this kicks off the running of my app
	// can be OF_WINDOW or OF_FULLSCREEN
	// pass in width and height too:
	//ofRunApp(new ofApp());

	ofGLFWWindowSettings settings;
	settings.setSize(1920,1080 );
	settings.setPosition(ofVec2f(0, 0));
	settings.resizable = true;
	shared_ptr<ofAppBaseWindow> mainWindow = ofCreateWindow(settings);

	settings.setSize(520, 120);
	settings.setPosition(ofVec2f(0, 100));
	settings.resizable = true;
	// uncomment next line to share main's OpenGL resources with gui
	//settings.shareContextWith = mainWindow;	
	shared_ptr<ofAppBaseWindow> infoWindow = ofCreateWindow(settings);
	infoWindow->setVerticalSync(false);

	shared_ptr<ofApp> mainApp(new ofApp);
	mainApp->setupinfo();
	ofAddListener(infoWindow->events().draw, mainApp.get(), &ofApp::drawinfo);

	ofRunApp(mainWindow, mainApp);
	ofRunMainLoop();

}
