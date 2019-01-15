#ifndef SONAR_HPP
#define SONAR_HPP

#include<math.h>

class Sonar {

	public:
	float maxRange;//cm
	float minRange;//Rmin cm
	float incertitudeRange;//cm
	float angleRange;//Omega rad
	float angleFromCenter;
	float distanceX;
	float distanceY;
	
	//the distance are in the world coordinates
	Sonar(float angleFromCenter, float distanceXFromRobotCenter, float distanceYFromRobotCenter );
	
	void setMaxRange(float newMaxRange);
	
	float compute_probability_t(float distanceObstacleDetected, float xCell, float yCell, float xRobotWorld, float yRobotWorld, float theta);
	
	//return distance sonar to cell if in range, -1 if not
	float isInRange(float xCell, float yCell, float xRobotWorld, float yRobotWorld, float thetaWorld);
	
	private:
	
	//returns the angle between the vectors (x,y) and (xs,ys)
	float compute_angle_between_vectors(float x, float y,float xs,float ys);

	//makes the angle inAngle between 0 and 2pi
	float rad_angle_check(float inAngle);

};

#endif

