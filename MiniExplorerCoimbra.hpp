#ifndef MINIEXPLORERCOIMBRA_HPP
#define MINIEXPLORERCOIMBRA_HPP

#include "Map.hpp"
#include "Sonar.hpp"
#include "myMatrix.hpp"
#include<math.h>

	/*
Robot coordinate system:      			World coordinate system:
      ^                 							^
      |x                							|y
   <--R                 							O-->
    y                     							x
    
angles from pi/2 to 3pi/2->0 to pi              angles from 0 to pi->0 to pi 
angles from pi/2 to -pi/2->0 to -pi				angles from pi to 2pi-> -pi to 0

	The idea is that for every command to the robot we use to world coodinate system 
	
	The target are relative to the robot position, so if the robot is in 100,80
	and the target is -20,30
	the robot will go to 80,110
	TODO I m pretty sure that a target angle is not relative to the current robot angle
*/

class MiniExplorerCoimbra {
	
	public:
	float xWorld;
	float yWorld;
	float thetaWorld;
	Map map;
	Sonar sonarLeft;
	Sonar sonarFront;
	Sonar sonarRight;
	float speed;
	float radiusWheels;
	float distanceWheels; 
	float khro;
	float ka;
	float kb;
	float kv;
	float kh;
	float kd;
	float k_linear;
	float k_angular;

	float rangeForce;
	float attractionConstantForce;
	float repulsionConstantForce;
	
	myMatrix covariancePositionEstimationK;
	float D_cm;
	float L_cm;
	
	float myX_r;
    float myY_r;
    float mytheta_r;
    
    float myX_r_correct;
    float myY_r_correct;
    float mytheta_r_correct;
    
    long int ticks2d_Special;
    long int ticks2e_Special;

	MiniExplorerCoimbra(float defaultXWorld, float defaultYWorld, float defaultThetaWorld, float widthRealMap, float heightRealMap);
	
	void test_procedure_lab2(int nbIteration);
	
	//generate a position randomly and makes the robot go there while updating the map
	void randomize_and_map();
	
	//move of targetXWorld and targetYWorld ending in a targetAngleWorld
	void go_to_point_with_angle(float targetXWorld, float targetYWorld, float targetAngleWorld);
	
	//move of targetXWorld and targetYWorld ending in a targetAngleWorld without checking the sonars
	void go_to_point_with_angle_first_lab(float targetXWorld, float targetYWorld, float targetAngleWorld);
	
	//use virtual force field
	void try_to_reach_target(float targetXWorld,float targetYWorld);
	
	void go_to_point(float targetXWorld, float targetYWorld);
	
	void go_to_line_first_lab(float line_a, float line_b, float line_c);
	
	void print_map_with_robot_position_and_target(float targetXWorld, float targetYWorld);
	
	void print_map_with_robot_position();
	
	void test_sonars_and_map(int nbIteration);
	
	void test_procedure_lab_4(float sizeX, float sizeY);
	
	void go_straight_kalman(float targetXWorld, float targetYWorld, float targetAngleWorld);
	
	void go_turn_kalman(float targetXWorld, float targetYWorld, float targetAngleWorld);
	
	void go_to_point_kalman(float targetXWorld, float targetYWorld);
	
	void go_to_point_kalman2(float targetXWorld, float targetYWorld);
	
	void test_prediction_sonar();
	
	void OdometriaKalmanFilter(); 
	
	void printMatrix(myMatrix mat1);
	
	void myOdometriaBis();
	
	void testOdometria(float *D_Special,float *L_Special);
	
	void myOdometria();
	
	void setXYThetaAndXYThetaWorld(float defaultXWorld, float defaultYWorld, float defaultThetaWorld);
	
	private:
	
	float update_angular_speed_wheels_go_to_point_with_angle(float targetXWorld, float targetYWorld, float targetAngleWorld, float dt);
	
	void update_sonar_values(float leftMm,float frontMm,float rightMm);
	
	void do_half_flip();
	
	//Distance computation function
	float dist(float x1, float y1, float x2, float y2);
	
	float distFromLine(float robot_x, float robot_y, float line_a, float line_b, float line_c);
	
	/*angleToTarget is obtained through atan2 so it s:
	< 0 if the angle is bettween PI and 2pi on a trigo circle
	> 0 if it is between 0 and PI
	*/
	void turn_to_target(float angleToTarget);
	
	void vff(bool* reached, float targetXWorld, float targetYWorld);
	
	//compute the force on X and Y
	void compute_forceX_and_forceY(float* forceXWorld, float* forceYWorld, float targetXWorld, float targetYWorld);
	
	void calculate_line(float forceX, float forceY, float *line_a, float *line_b, float *line_c);
	
	//currently line_c is not used
	void go_to_line(float line_a, float line_b, float line_c,float targetXWorld, float targetYWorld);
	
	void update_force(int widthIndice, int heightIndice, float* forceRepulsionComputedX, float* forceRepulsionComputedY );

	//return 1 if positiv, -1 if negativ
	float sign1(float value);
	
	//return 1 if positiv, 0 if negativ
	int sign2(float value);
	
	void go_straight_line(float distanceCm);
	
	void procedure_lab_4(float xEstimatedK,float yEstimatedK, float thetaWorldEstimatedK, float distanceMoved, float angleMoved, float PreviousCovarianceOdometricPositionEstimate[3][3]);

};


#endif

