#include "Sonar.hpp"

#define PI 3.14159

Sonar::Sonar(float angleFromCenter, float distanceXFromRobotCenter, float distanceYFromRobotCenter ){
	this->angleFromCenter=angleFromCenter;
	this->distanceX=distanceXFromRobotCenter;
	this->distanceY=distanceYFromRobotCenter;
	this->maxRange=50;//cm
	this->minRange=10;//Rmin cm
	this->incertitudeRange=10;//cm
	this->angleRange=3.14159/3;//Omega rad
}

void Sonar::setMaxRange(float newMaxRange){
	this->maxRange=newMaxRange;
}
//return distance sonar to cell if in range, -1 if not
float Sonar::isInRange(float xCell, float yCell, float xRobotWorld, float yRobotWorld, float thetaWorld){
	float xSonar=xRobotWorld+this->distanceX;
	float ySonar=yRobotWorld+this->distanceY;	
	//float distanceCellToSonar=sqrt(pow(xCell-xSonar,2)+pow(yCell-ySonar,2));
	float distanceCellToSonar=sqrt(pow(xSonar-xCell,2)+pow(ySonar-yCell,2));
	//check if the distance between the cell and the robot is within the circle of range RADIUS_WHEELS
    if( distanceCellToSonar < this->maxRange){
        //float anglePointToSonar=this->compute_angle_between_vectors(xCell,yCell,xSonar,ySonar);//angle beetween the point and the sonar beam
        float angleCellToSonar=atan2(yCell-yRobotWorld,xCell-xRobotWorld);//like world system
        
       	float angleOriginToMidleOfBeam=thetaWorld+this->angleFromCenter;//

        float angleDifference=angleCellToSonar-angleOriginToMidleOfBeam;
        if(angleDifference > PI)
            angleDifference=angleDifference-2*PI;
        if(angleDifference < -PI)
            angleDifference=angleDifference+2*PI;
        //check if absolute difference between the angles is no more than Omega/2
        if(angleDifference > 0 && angleDifference <= this->angleRange/2 ||angleDifference < 0 && angleDifference >= -this->angleRange/2 ){
        	return distanceCellToSonar;
        }
    }
    return -1;
}

//function that check if a cell A(x,y) is in the range of the front sonar S(xs,ys) (with an angle depending on the sonar used, front 0, left PI/3, right -PI/3) returns the probability it's occuPIed/empty [0;1]
float Sonar::compute_probability_t(float distanceObstacleDetected, float xCell, float yCell, float xRobotWorld, float yRobotWorld, float thetaWorld){
	float xSonar=xRobotWorld+this->distanceX;
	float ySonar=yRobotWorld+this->distanceY;
    float distancePointToSonar=sqrt(pow(xCell-xSonar,2)+pow(yCell-ySonar,2));
    //check if the distance between the cell and the robot is within the circle of range RADIUS_WHEELS
    if( distancePointToSonar < this->maxRange){
        //float anglePointToSonar=this->compute_angle_between_vectors(xCell,yCell,xSonar,ySonar);//angle beetween the point and the sonar beam
        float anglePointToSonar=atan2(yCell-yRobotWorld,xCell-xRobotWorld);//like world system
        
       	float angleOriginToMidleOfBeam=thetaWorld+this->angleFromCenter;//

        float angleDifference=anglePointToSonar-angleOriginToMidleOfBeam;
        if(angleDifference > PI)
            angleDifference=angleDifference-2*PI;
        if(angleDifference < -PI)
            angleDifference=angleDifference+2*PI;
        //check if absolute difference between the angles is no more than Omega/2
        if(angleDifference > 0 && angleDifference <= this->angleRange/2 ||angleDifference < 0 && angleDifference >= -this->angleRange/2 ){

            if( distancePointToSonar < (distanceObstacleDetected - this->incertitudeRange)){
            //point before obstacle, probably empty
            /*****************************************************************************/
                float Ea=1.f-pow((2*angleDifference)/this->angleRange,2);
                float Er;
                if(distancePointToSonar < this->minRange){
                    //point before minimum sonar range
                    Er=0.f;
                }else{
                    //point after minimum sonar range
                    Er=1.f-pow((distancePointToSonar-this->minRange)/(distanceObstacleDetected-this->incertitudeRange-this->minRange),2);
                }
             /*****************************************************************************/
                //if((1.f-Er*Ea)/2.f >1 || (1.f-Er*Ea)/2.f < 0)
                    //pc.printf("\n\r return value=%f,Er=%f,Ea=%f,angleDifference=%f",(1.f-Er*Ea)/2.f,Er,Ea,angleDifference);
                return (1.f-Er*Ea)/2.f;
            }else{
                //probably occuPIed
            /*****************************************************************************/
                float Oa=1.f-pow((2*angleDifference)/this->angleRange,2);
                float Or;
                if( distancePointToSonar <= (distanceObstacleDetected + this->incertitudeRange)){
                    //point between distanceObstacleDetected +- INCERTITUDE_SONAR
                    Or=1-pow((distancePointToSonar-distanceObstacleDetected)/(this->incertitudeRange),2);
                }else{
                    //point after in range of the sonar but after the zone detected
                    Or=0;
                }
            /*****************************************************************************/
                //if((1+Or*Oa)/2 >1 || (1+Or*Oa)/2 < 0)
                    //pc.printf("\n\r return value=%f,Er=%f,Ea=%f,angleDifference=%f",(1+Or*Oa)/2,Or,Oa,angleDifference);
                return (1+Or*Oa)/2;
            }
        }   
    }
    //not checked by the sonar
    return 0.5;
}

//returns the angle between the vectors (x,y) and (xs,ys)
float Sonar::compute_angle_between_vectors(float x, float y,float xs,float ys){
    //alpha angle between ->x and ->SA
    //vector S to A ->SA
    float vSAx=x-xs;
    float vSAy=y-ys;
    //norme SA
    float normeSA=sqrt(pow(vSAx,2)+pow(vSAy,2));
    //vector ->x (1,0)
    float cosAlpha=1*vSAy/*+0*vSAx*//normeSA;;
    //vector ->y (0,1)
    float sinAlpha=/*0*vSAy+*/1*vSAx/normeSA;//+0*vSAx;
    if (sinAlpha < 0)
        return -acos(cosAlpha);
    else
        return acos(cosAlpha);
}

//makes the angle inAngle between 0 and 2PI
float Sonar::rad_angle_check(float inAngle){
    if(inAngle > 0){
        while(inAngle > (2*PI))
            inAngle-=2*PI;
    }else{
        while(inAngle < 0)
            inAngle+=2*PI;
    }
    return inAngle;
}

