#ifndef POSEWINDOW_H_
#define POSEWINDOW_H_



#include "QTOgreWindow.h"


class PoseWindow : public QTOgreWindow {

public:

	virtual void createScene();



	Ogre::SceneNode *axesNode;



private:

	// Makes the grid lines
	void createGrid();

	// Create axes for visualizing rotation
	void createAxes();

};








#endif
