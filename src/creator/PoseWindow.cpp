#include "PoseWindow.h"

#include <string>

using namespace std;
using namespace Ogre;


void PoseWindow::createGrid(){

	MaterialPtr mat;
	mat = Ogre::MaterialManager::getSingleton().create("gridMaterial", "General");
	mat->setReceiveShadows(false);
	mat->getTechnique(0)->setLightingEnabled(true);
	mat->getTechnique(0)->getPass(0)->setDiffuse(0, 0, 0, 0);
	mat->getTechnique(0)->getPass(0)->setAmbient(0, 0, 0);
	mat->getTechnique(0)->getPass(0)->setSelfIllumination(0.7, 0.7, 0.7);





	// create ManualObject
	ManualObject* manual = m_ogreSceneMgr->createManualObject("manualGrid");

	// specify the material (by name) and rendering type
	manual->begin("gridMaterial" /* "BaseWhiteNoLighting" */, RenderOperation::OT_LINE_LIST);



	for(int i = -5; i <= 5; i++){
		manual->position(i, -5, 0);
		manual->position(i, 5, 0);

		manual->position(-5, i, 0);
		manual->position(5, i, 0);
	}

	// tell Ogre, your definition has finished
	manual->end();

	// add ManualObject to the RootSceneNode (so it will be visible)
	m_ogreSceneMgr->getRootSceneNode()->attachObject(manual);


}


void PoseWindow::createAxes(){

	MaterialPtr mats[3];


	SceneNode *node = m_ogreSceneMgr->getRootSceneNode()->createChildSceneNode("manual1_node");
	node->setPosition(Vector3(0,0,0.5));


	for(int i = 0; i < 3; i++){

		string mname = "Color_" + to_string(i+1);

		mats[i] = Ogre::MaterialManager::getSingleton().create(mname, "General");
		mats[i]->setReceiveShadows(false);
		mats[i]->getTechnique(0)->setLightingEnabled(true);
		mats[i]->getTechnique(0)->getPass(0)->setDiffuse((i==0?1:0), (i==1?1:0), (i==2?1:0), 0);
		mats[i]->getTechnique(0)->getPass(0)->setAmbient((i==0?1:0), (i==1?1:0), (i==2?1:0));
		mats[i]->getTechnique(0)->getPass(0)->setSelfIllumination((i==0?1:0), (i==1?1:0), (i==2?1:0));


		Ogre::ManualObject* myManualObject =  m_ogreSceneMgr->createManualObject("manual" + to_string(i));


		myManualObject->begin(mname, Ogre::RenderOperation::OT_LINE_LIST);
		myManualObject->position(0, 0, 0);
		myManualObject->position((i==0?1:0), (i==1?1:0), (i==2?1:0));
		// etc
		myManualObject->end();

		node->attachObject(myManualObject);




	}

	axesNode = node;

}



void PoseWindow::createScene(){

	m_cameraMan->setStyle(OgreQtBites::CS_ORBIT);

	m_ogreSceneMgr->setAmbientLight(Ogre::ColourValue(0.5, 0.5, 0.5));


	this->createGrid();

	this->createAxes();


}
