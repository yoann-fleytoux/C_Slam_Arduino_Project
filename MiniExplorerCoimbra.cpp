#include "MiniExplorerCoimbra.hpp"
#include "robot.h"

#define PI 3.14159

MiniExplorerCoimbra::MiniExplorerCoimbra(float defaultXWorld, float defaultYWorld, float defaultThetaWorld, float widthRealMap, float heightRealMap):map(widthRealMap,heightRealMap,18,12),sonarLeft(10*PI/36,-4,4),sonarFront(0,0,5),sonarRight(-10*PI/36,4,4),covariancePositionEstimationK(3,3){
    i2c1.frequency(100000);
    initRobot(); //Initializing the robot
    pc.baud(9600); // baud for the pc communication

    measure_always_on();//TODO check if needed

    this->setXYThetaAndXYThetaWorld(defaultXWorld,defaultYWorld,defaultThetaWorld);
    this->radiusWheels=3.25;
    this->distanceWheels=7.18; 
    
    //go to point
    this->k_linear=10;
    this->k_angular=200;
    
    //go to point with angle
    this->khro=0.3;
    this->ka=0.8;
    this->kb=-0.36;
    this->kv=200;//velocity
    
    //go to line kh > 0, kd > 0 (200,5), dont turn to line enough, (10,10) turn on itself, 
    this->kh=650;
    this->kd=10;//previous 5
    
    this->speed=300;

    this->rangeForce=75;
    this->attractionConstantForce=3;
    //-90 ok with attraction impacted by distance
    this->repulsionConstantForce=-200;
    
    this->D_cm=0;
    this->L_cm=0;
    
    this->ticks2d_Special=0;
    this->ticks2e_Special=0;
} 

void MiniExplorerCoimbra::setXYThetaAndXYThetaWorld(float defaultXWorld, float defaultYWorld, float defaultThetaWorld){
    this->xWorld=defaultXWorld;
    this->yWorld=defaultYWorld;
    this->thetaWorld=defaultThetaWorld;
    X=defaultYWorld;
    Y=-defaultXWorld;
	if(defaultThetaWorld < -PI/2)
		theta=PI/2+PI-defaultThetaWorld;
	else
		theta=defaultThetaWorld-PI/2;
	this->myX_r=X;
    this->myY_r=Y;
    this->mytheta_r=theta;
    
    this->myX_r_correct=X;
    this->myY_r_correct=Y;
    this->mytheta_r_correct=theta;
}

void MiniExplorerCoimbra::myOdometria(){
	Odometria();
	this->xWorld=-Y;
	this->yWorld=X;
	if(theta >PI/2)
		this->thetaWorld=-PI+(theta-PI/2);
	else
		this->thetaWorld=theta+PI/2;
}

void MiniExplorerCoimbra::go_to_point(float targetXWorld, float targetYWorld) {
    
    float angleError; //angle error
    float d; //distance from target
    float angularLeft, angularRight, linear, angular;
    int speed=300;

    do {        
        //Updating X,Y and theta with the odometry values
        this->myOdometria();

        //Computing angle error and distance towards the target value
        angleError = atan2((targetYWorld-this->yWorld),(targetXWorld-this->xWorld))-this->thetaWorld;
        if(angleError>PI) angleError=-(angleError-PI);
        else if(angleError<-PI) angleError=-(angleError+PI);
        pc.printf("\n\r error=%f",angleError);
        
        d=this->dist(this->xWorld, this->yWorld, targetXWorld, targetYWorld);
		pc.printf("\n\r dist=%f/n", d);

        //Computing linear and angular velocities
        linear=k_linear*d;
        angular=k_angular*angleError;
        angularLeft=(linear-0.5*this->distanceWheels*angular)/this->radiusWheels;
        angularRight=(linear+0.5*this->distanceWheels*angular)/this->radiusWheels;

        
        //Normalize speed for motors
        if(angularLeft>angularRight) {
            angularRight=speed*angularRight/angularLeft;
            angularLeft=speed;
        } else {
            angularLeft=speed*angularLeft/angularRight;
            angularRight=speed;
        }

        pc.printf("\n\r X=%f", this->xWorld);
        pc.printf("\n\r Y=%f", this->yWorld);
        pc.printf("\n\r theta=%f", this->thetaWorld);

        //Updating motor velocities
        if(angularLeft>0){
            leftMotor(1,angularLeft);
        }
        else{
            leftMotor(0,-angularLeft);
        }
        
        if(angularRight>0){
            rightMotor(1,angularRight);
        }
        else{
            rightMotor(0,-angularRight);
        }

        //wait(0.5);
    } while(d>1);

    //Stop at the end
    leftMotor(1,0);
    rightMotor(1,0);
}

void MiniExplorerCoimbra::test_procedure_lab2(int nbIteration){
	for(int i=0;i<nbIteration;i++){
        this->randomize_and_map();
        //this->print_map_with_robot_position();
    }
    while(1)
    	this->print_map_with_robot_position();
}

//generate a position randomly and makes the robot go there while updating the map
void MiniExplorerCoimbra::randomize_and_map() {
    //TODO check that it's aurelien's work
    float movementOnX=rand()%(int)(this->map.widthRealMap);
    float movementOnY=rand()%(int)(this->map.heightRealMap);

    float targetXWorld = movementOnX;
    float targetYWorld = movementOnY;
    float targetAngleWorld = 2*((float)(rand()%31416)-15708)/10000.0;
    
    //target between (0,0) and (widthRealMap,heightRealMap)
    this->go_to_point_with_angle(targetXWorld, targetYWorld, targetAngleWorld);
}

void MiniExplorerCoimbra::test_sonars_and_map(int nbIteration){
    float leftMm;
    float frontMm; 
    float rightMm;
    this->myOdometria();
    this->print_map_with_robot_position();
	for(int i=0;i<nbIteration;i++){
        leftMm = get_distance_left_sensor();
        frontMm = get_distance_front_sensor();
        rightMm = get_distance_right_sensor();
		pc.printf("\n\r 1 leftMm= %f",leftMm);
        pc.printf("\n\r 1 frontMm= %f",frontMm);
        pc.printf("\n\r 1 rightMm= %f",rightMm);
        this->update_sonar_values(leftMm, frontMm, rightMm);
        this->print_map_with_robot_position();
        wait(1);
	}
}


//generate a position randomly and makes the robot go there while updating the map
//move of targetXWorld and targetYWorld ending in a targetAngleWorld
void MiniExplorerCoimbra::go_to_point_with_angle(float targetXWorld, float targetYWorld, float targetAngleWorld) {
    bool keepGoing=true;
    float leftMm;
    float frontMm; 
    float rightMm;
    float dt;
    Timer t;
    float distanceToTarget;
    do {
        //Timer stuff
        dt = t.read();
        t.reset();
        t.start();
        
        //Updating X,Y and theta with the odometry values
        this->myOdometria();
        leftMm = get_distance_left_sensor();
        frontMm = get_distance_front_sensor();
        rightMm = get_distance_right_sensor();
        //if in dangerzone 150 mm 
        if((frontMm < 150 && frontMm > 0)|| (leftMm <150 && leftMm > 0) || (rightMm <150 && rightMm > 0) ){
            //stop motors
            leftMotor(1,0);
            rightMotor(1,0);
            //update the map
            this->update_sonar_values(leftMm, frontMm, rightMm);
            this->myOdometria();
            keepGoing=false;
            this->do_half_flip(); 
        }else{
            //if not in danger zone continue as usual
            this->update_sonar_values(leftMm, frontMm, rightMm);
            //Updating motor velocities
            distanceToTarget=this->update_angular_speed_wheels_go_to_point_with_angle(targetXWorld,targetYWorld,targetAngleWorld,dt);
            //wait(0.2);
            //Timer stuff
            t.stop();
            pc.printf("\n\rdist to target= %f",distanceToTarget);
        }
    } while((distanceToTarget>2 || (abs(targetAngleWorld-this->thetaWorld)>PI/3)) && keepGoing);

    //Stop at the end
    leftMotor(1,0);
    rightMotor(1,0);
    pc.printf("\r\nReached Target!");
}

//move of targetXWorld and targetYWorld ending in a targetAngleWorld
void MiniExplorerCoimbra::go_to_point_with_angle_first_lab(float targetXWorld, float targetYWorld, float targetAngleWorld) {
    float dt;
    Timer t;
    float distanceToTarget;
    do {
        //Timer stuff
        dt = t.read();
        t.reset();
        t.start();
        
        //Updating X,Y and theta with the odometry values
        this->myOdometria();
        
        //Updating motor velocities
        distanceToTarget=this->update_angular_speed_wheels_go_to_point_with_angle(targetXWorld,targetYWorld,targetAngleWorld,dt);
    
        //wait(0.2);
        //Timer stuff
        t.stop();
        pc.printf("\n\rdist to target= %f",distanceToTarget);
        
    } while(distanceToTarget>1 || (abs(targetAngleWorld-this->thetaWorld)>0.1));

    //Stop at the end
    leftMotor(1,0);
    rightMotor(1,0);
    pc.printf("\r\nReached Target!");
}

float MiniExplorerCoimbra::update_angular_speed_wheels_go_to_point_with_angle(float targetXWorld, float targetYWorld, float targetAngleWorld, float dt){
    //compute_angles_and_distance
    //atan2 take the deplacement on x and the deplacement on y as parameters
    float angleToPoint = atan2((targetYWorld-this->yWorld),(targetXWorld-this->xWorld))-this->thetaWorld;
	
	if(angleToPoint>PI) angleToPoint=-(angleToPoint-PI);
    else if(angleToPoint<-PI) angleToPoint=-(angleToPoint+PI);    
    
    //rho is the distance to the point of arrival
    float rho = dist(targetXWorld,targetYWorld,this->xWorld,this->yWorld);
    float distanceToTarget = rho;
    //TODO check that
    float beta = targetAngleWorld-angleToPoint-this->thetaWorld;        
    
    //Computing angle error and distance towards the target value
    rho += dt*(-this->khro*cos(angleToPoint)*rho);
    float temp = angleToPoint;
    angleToPoint += dt*(this->khro*sin(angleToPoint)-this->ka*angleToPoint-this->kb*beta);
    beta += dt*(-this->khro*sin(temp));

    //Computing linear and angular velocities
    float linear;
    float angular;
    if(angleToPoint>=-1.5708 && angleToPoint<=1.5708){
        linear=this->khro*rho;
        angular=this->ka*angleToPoint+this->kb*beta;
    }
    else{
        linear=-this->khro*rho;
        angular=-this->ka*angleToPoint-this->kb*beta;
    }
        
    float angularLeft=(linear-0.5*this->distanceWheels*angular)/this->radiusWheels;
    float angularRight=(linear+0.5*this->distanceWheels*angular)/this->radiusWheels;
    
    //Slowing down at the end for more precision
	if (distanceToTarget<30) {
    	this->speed = distanceToTarget*10;
	}
    
    //Normalize speed for motors
    if(angularLeft>angularRight) {
        angularRight=this->speed*angularRight/angularLeft;
        angularLeft=this->speed;
    } else {
        angularLeft=this->speed*angularLeft/angularRight;
        angularRight=this->speed;
    }

    //compute_linear_angular_velocities 
    leftMotor(1,angularLeft);
    rightMotor(1,angularRight);
    
    return distanceToTarget;
}

void MiniExplorerCoimbra::update_sonar_values(float leftMm,float frontMm,float rightMm){
    float xWorldCell;
    float yWorldCell;
    float probaLeft;
    float probaFront;
    float probaRight;
    float leftCm=leftMm/10;
    float frontCm=frontMm/10;
    float rightCm=rightMm/10;
    for(int i=0;i<this->map.nbCellWidth;i++){
        for(int j=0;j<this->map.nbCellHeight;j++){
        	xWorldCell=this->map.cell_width_coordinate_to_world(i);
            yWorldCell=this->map.cell_height_coordinate_to_world(j);
            
            probaLeft=this->sonarLeft.compute_probability_t(leftCm,xWorldCell,yWorldCell,this->xWorld,this->yWorld,this->thetaWorld);
			probaFront=this->sonarFront.compute_probability_t(frontCm,xWorldCell,yWorldCell,this->xWorld,this->yWorld,this->thetaWorld);
			probaRight=this->sonarRight.compute_probability_t(rightCm,xWorldCell,yWorldCell,this->xWorld,this->yWorld,this->thetaWorld);
			
			/*
			pc.printf("\n\r leftCm= %f",leftCm);
	        pc.printf("\n\r frontCm= %f",frontCm);
	        pc.printf("\n\r rightCm= %f",rightCm);
	        */
	        /*
			pc.printf("\n\r probaLeft= %f",probaLeft);
	        pc.printf("\n\r probaFront= %f",probaFront);
	        pc.printf("\n\r probaRight= %f",probaRight);
	        if(probaLeft> 1 || probaLeft < 0 || probaFront> 1 || probaFront < 0 ||probaRight> 1 || probaRight < 0)){
	        	 pwm_buzzer.pulsewidth_us(250);
	            wait_ms(50);
	            pwm_buzzer.pulsewidth_us(0);
	            wait(20);
	            pwm_buzzer.pulsewidth_us(250);
	            wait_ms(50);
	            pwm_buzzer.pulsewidth_us(0);
	        } 
        	*/
        	this->map.update_cell_value(i,j,probaLeft);
        	this->map.update_cell_value(i,j,probaFront);
        	this->map.update_cell_value(i,j,probaRight);
       	}
    }
}

void MiniExplorerCoimbra::do_half_flip(){
    this->myOdometria();
    float theta_plus_h_pi=theta+PI/2;//theta is between -PI and PI
    if(theta_plus_h_pi > PI)
        theta_plus_h_pi=-(2*PI-theta_plus_h_pi);
    leftMotor(0,100);
    rightMotor(1,100);
    while(abs(theta_plus_h_pi-theta)>0.05){
        this->myOdometria();
       // pc.printf("\n\r diff=%f", abs(theta_plus_pi-theta));
    }
    leftMotor(1,0);
    rightMotor(1,0);    
}

//Distance computation function
float MiniExplorerCoimbra::dist(float x1, float y1, float x2, float y2){
    return sqrt(pow(y2-y1,2) + pow(x2-x1,2));
}

//use virtual force field
void MiniExplorerCoimbra::try_to_reach_target(float targetXWorld,float targetYWorld){
    //atan2 gives the angle beetween PI and -PI
    this->myOdometria();
    /*
    float deplacementOnXWorld=targetXWorld-this->xWorld;
    float deplacementOnYWorld=targetYWorld-this->yWorld;
    */
    //float angleToTarget=atan2(targetYWorld-this->yWorld,targetXWorld-this->xWorld);
    //pc.printf("\n angleToTarget=%f",angleToTarget);
    //turn_to_target(angleToTarget);
    //TODO IDEA check if maybe set a low max range
    //this->sonarLeft.setMaxRange(30);
    //this->sonarFront.setMaxRange(30);
    //this->sonarRight.setMaxRange(30);
    bool reached=false;
    int print=0;
    int printLimit=1000;
    while (!reached) {
        this->vff(&reached,targetXWorld,targetYWorld);
        //test_got_to_line(&reached);
        if(print==printLimit){
            leftMotor(1,0);
            rightMotor(1,0);
            this->print_map_with_robot_position_and_target(targetXWorld,targetYWorld);
            print=0;
        }else
            print+=1;
    }
    //Stop at the end
    leftMotor(1,0);
    rightMotor(1,0);
    pc.printf("\r\n target reached");
    //wait(3);//
}

void MiniExplorerCoimbra::vff(bool* reached, float targetXWorld, float targetYWorld){
    float line_a;
    float line_b;
    float line_c;
    //Updating X,Y and theta with the odometry values
    float forceXWorld=0;
    float forceYWorld=0;
    //we update the odometrie
    this->myOdometria();
    //we check the sensors
    float leftMm = get_distance_left_sensor();
    float frontMm = get_distance_front_sensor();
    float rightMm = get_distance_right_sensor();
    //update the probabilities values 
    this->update_sonar_values(leftMm, frontMm, rightMm);
    this->myOdometria();
    //we compute the force on X and Y
    this->compute_forceX_and_forceY(&forceXWorld, &forceYWorld,targetXWorld,targetYWorld);
    //we compute a new ine
    this->calculate_line(forceXWorld, forceYWorld, &line_a,&line_b,&line_c);
    //Updating motor velocities
    //pc.printf("\r\nX=%f;Y=%f",xWorld,yWorld);
    //pc.printf("\r\n%f*x+%f*y+%f=0",line_a,line_b,line_c);
    this->go_to_line(line_a,line_b,line_c,targetXWorld,targetYWorld);
    
    //wait(0.1);
    this->myOdometria();
    if(dist(this->xWorld,this->yWorld,targetXWorld,targetYWorld)<3)
        *reached=true;
}

/*angleToTarget is obtained through atan2 so it s:
< 0 if the angle is bettween PI and 2pi on a trigo circle
> 0 if it is between 0 and PI
*/
void MiniExplorerCoimbra::turn_to_target(float angleToTarget){
    this->myOdometria();
    if(angleToTarget!=0){
    	if(angleToTarget>0){   
        	leftMotor(0,1);
        	rightMotor(1,1);
	    }else{
	        leftMotor(1,1);
	        rightMotor(0,1);
	    }
    	while(abs(angleToTarget-this->thetaWorld)>0.05)
        	this->myOdometria();
    }
    leftMotor(1,0);
    rightMotor(1,0);    
}


void MiniExplorerCoimbra::print_map_with_robot_position_and_target(float targetXWorld, float targetYWorld) {
    float currProba;
    
    float heightIndiceInOrthonormal;
    float widthIndiceInOrthonormal;
    
    float widthMalus=-(3*this->map.sizeCellWidth/2);
    float widthBonus=this->map.sizeCellWidth/2;
    
    float heightMalus=-(3*this->map.sizeCellHeight/2);
    float heightBonus=this->map.sizeCellHeight/2;

    pc.printf("\n\r");
    for (int y = this->map.nbCellHeight -1; y>-1; y--) {
        for (int x= 0; x<this->map.nbCellWidth; x++) {
            heightIndiceInOrthonormal=this->map.cell_height_coordinate_to_world(y);
            widthIndiceInOrthonormal=this->map.cell_width_coordinate_to_world(x);
            if(this->yWorld >= (heightIndiceInOrthonormal+heightMalus) && this->yWorld <= (heightIndiceInOrthonormal+heightBonus) && this->xWorld >= (widthIndiceInOrthonormal+widthMalus) && this->xWorld <= (widthIndiceInOrthonormal+widthBonus))
                pc.printf(" R ");
            else{
                if(targetYWorld >= (heightIndiceInOrthonormal+heightMalus) && targetYWorld <= (heightIndiceInOrthonormal+heightBonus) && targetXWorld >= (widthIndiceInOrthonormal+widthMalus) && targetXWorld <= (widthIndiceInOrthonormal+widthBonus))                    
                    pc.printf(" T ");
                else{
                    currProba=this->map.log_to_proba(this->map.cellsLogValues[x][y]);
                    if ( currProba < 0.5){
                        pc.printf("   ");
                        //pc.printf("%f",currProba);
                    }else{
                        if(currProba==0.5){
                            pc.printf(" . ");
                            //pc.printf("%f",currProba);
                        }else{
                            pc.printf(" X ");
                            //pc.printf("%f",currProba);
                        }
                    } 
                }
            }
        }
        pc.printf("\n\r");
    }
}

void MiniExplorerCoimbra::print_map_with_robot_position(){
    float currProba;
    
    float heightIndiceInOrthonormal;
    float widthIndiceInOrthonormal;
    
    float widthMalus=-(3*this->map.sizeCellWidth/2);
    float widthBonus=this->map.sizeCellWidth/2;
    
    float heightMalus=-(3*this->map.sizeCellHeight/2);
    float heightBonus=this->map.sizeCellHeight/2;

    pc.printf("\n\r");
    for (int y = this->map.nbCellHeight -1; y>-1; y--) {
        for (int x= 0; x<this->map.nbCellWidth; x++) {
            heightIndiceInOrthonormal=this->map.cell_height_coordinate_to_world(y);
            widthIndiceInOrthonormal=this->map.cell_width_coordinate_to_world(x);
            if(this->yWorld >= (heightIndiceInOrthonormal+heightMalus) && this->yWorld <= (heightIndiceInOrthonormal+heightBonus) && this->xWorld >= (widthIndiceInOrthonormal+widthMalus) && this->xWorld <= (widthIndiceInOrthonormal+widthBonus)){
                pc.printf(" R ");
                //pc.printf("%f",currProba);
            }else{
                currProba=this->map.log_to_proba(this->map.cellsLogValues[x][y]);
                if ( currProba < 0.5){
                    pc.printf("   ");
                    //pc.printf("%f",currProba);
                }else{
                    if(currProba==0.5){
                        pc.printf(" . ");
                        //pc.printf("%f",currProba);
                    }else{
                       pc.printf(" X ");
                        //pc.printf("%f",currProba);
                    }
                }
            }
        }
        pc.printf("\n\r");
    }
}

//robotX and robotY are from this->myOdometria(), calculate line_a, line_b and line_c
void MiniExplorerCoimbra::calculate_line(float forceX, float forceY, float *line_a, float *line_b, float *line_c){
    /*
    in the teachers maths it is 
    
    *line_a=forceY;
    *line_b=-forceX;
    
    because a*x+b*y+c=0
    a impact the vertical and b the horizontal
    and he has to put them like this because
    Robot space:      World space:
      ^                 ^
      |x                |y
   <- R                 O ->
    y                     x
    but since our forceX, forceY are already computed in the orthonormal space I m not sure we need to 
    */
    //*line_a=forceX;
    //*line_b=forceY;
    
    *line_a=forceY;
    *line_b=-forceX;
    //because the line computed always pass by the robot center we dont need lince_c
    *line_c=forceX*this->yWorld+forceY*this->xWorld;    
    //*line_c=0;
}
//compute the force on X and Y
void MiniExplorerCoimbra::compute_forceX_and_forceY(float* forceXWorld, float* forceYWorld, float targetXWorld, float targetYWorld){
	float forceRepulsionComputedX=0;
	float forceRepulsionComputedY=0;
	this->print_map_with_robot_position();
 	for(int i=0;i<this->map.nbCellWidth;i++){	//for each cell of the map we compute a force of repulsion
        for(int j=0;j<this->map.nbCellHeight;j++){
            this->update_force(i,j,&forceRepulsionComputedX,&forceRepulsionComputedY);
        }
    }
    //update with attraction force
    *forceXWorld=forceRepulsionComputedX;
    *forceYWorld=forceRepulsionComputedY;
    //this->print_map_with_robot_position();
    //pc.printf("\r\nForce X repul:%f",*forceXWorld);
    //pc.printf("\r\nForce Y repul:%f",*forceYWorld);
    
    //test without atraction being impacted by distance
    //*forceXWorld+=this->attractionConstantForce*(targetXWorld-this->xWorld);
    //*forceYWorld+=this->attractionConstantForce*(targetYWorld-this->yWorld);
    
    float distanceTargetRobot=sqrt(pow(targetXWorld-this->xWorld,2)+pow(targetYWorld-this->yWorld,2));
    if(distanceTargetRobot != 0){
        *forceXWorld+=this->attractionConstantForce*(targetXWorld-this->xWorld)/distanceTargetRobot;
        *forceYWorld+=this->attractionConstantForce*(targetYWorld-this->yWorld)/distanceTargetRobot;
    }else{
    	*forceXWorld+=this->attractionConstantForce*(targetXWorld-this->xWorld)/0.001;
        *forceYWorld+=this->attractionConstantForce*(targetYWorld-this->yWorld)/0.001;
    }
    
    //pc.printf("\r\nForce X after attract:%f",*forceXWorld);
    //pc.printf("\r\nForce Y after attract:%f",*forceYWorld);

    float amplitude=sqrt(pow(*forceXWorld,2)+pow(*forceYWorld,2));
    if(amplitude!=0){//avoid division by 0 if forceX and forceY  == 0
        *forceXWorld=*forceXWorld/amplitude;
        *forceYWorld=*forceYWorld/amplitude;
    }else{
    	*forceXWorld=*forceXWorld/0.001;
        *forceYWorld=*forceYWorld/0.001;
    }
}

//for vff
void MiniExplorerCoimbra::go_to_line(float line_a, float line_b, float line_c,float targetXWorld, float targetYWorld){
    float lineAngle;
    float angleError;
    float linear;
    float angular; 
    float d;
    
    //line angle is beetween pi/2 and -pi/2

   if(line_b!=0){
        lineAngle=atan(line_a/-line_b);
    }
    else{
        lineAngle=0;
    }
    
    this->myOdometria();
	//Computing angle error
	angleError = lineAngle-this->thetaWorld;//TODO that I m not sure
 	if(angleError>PI) 
 		angleError=-(angleError-PI);
    else 
    	if(angleError<-PI) 
    		angleError=-(angleError+PI);

    //d=this->distFromLine(this->xWorld, this->yWorld, line_a, line_b, line_c);//this could be 0
	d=0;
	//pc.printf("\r\ndistance from line:%f",d);
    //Calculating velocities
    linear= this->kv*(3.14);
    angular=-this->kd*d + this->kh*angleError;

    float angularLeft=(linear-0.5*this->distanceWheels*angular)/this->radiusWheels;
	float angularRight=(linear+0.5*this->distanceWheels*angular)/this->radiusWheels;
    
   //Normalize speed for motors
    if(abs(angularLeft)>abs(angularRight)) {  
        angularRight=this->speed*abs(angularRight/angularLeft)*this->sign1(angularRight);
        angularLeft=this->speed*this->sign1(angularLeft);
    }
    else {
        angularLeft=this->speed*abs(angularLeft/angularRight)*this->sign1(angularLeft);
        angularRight=this->speed*this->sign1(angularRight);
    }
    
    pc.printf("\r\nd = %f", d);
    pc.printf("\r\nerror = %f, lineAngle=%f, robotAngle=%f\n", angleError,lineAngle,this->thetaWorld);
    
    leftMotor(this->sign2(angularLeft),abs(angularLeft));
    rightMotor(this->sign2(angularRight),abs(angularRight));
}

void MiniExplorerCoimbra::go_to_line_first_lab(float line_a, float line_b, float line_c){
    float lineAngle;
    float angleError;
    float linear;
    float angular;
    float d;
    
    //line angle is beetween pi/2 and -pi/2
    	
    if(line_b!=0){
        lineAngle=atan(line_a/-line_b);
    }
    else{
        lineAngle=1.5708;
    }
    
    do{
    	this->myOdometria();
	    //Computing angle error
	    pc.printf("\r\nline angle = %f", lineAngle);
	    pc.printf("\r\nthetaWorld = %f", thetaWorld);
	    angleError = lineAngle-this->thetaWorld;//TODO that I m not sure
	    
	    if(angleError>PI) 
	    	angleError=-(angleError-PI);
	    else 
	    	if(angleError<-PI) 
	    		angleError=-(angleError+PI);
	    
	    pc.printf("\r\nangleError = %f\n", angleError);
	    d=this->distFromLine(xWorld, yWorld, line_a, line_b, line_c);
		pc.printf("\r\ndistance to line = %f", d);
		
	    //Calculating velocities
	    linear= this->kv*(3.14);
	    angular=-this->kd*d + this->kh*angleError;
	
	    float angularLeft=(linear-0.5*this->distanceWheels*angular)/this->radiusWheels;
	    float angularRight=(linear+0.5*this->distanceWheels*angular)/this->radiusWheels;
	    
	    //Normalize speed for motors
	    if(abs(angularLeft)>abs(angularRight)) {  
	        angularRight=this->speed*abs(angularRight/angularLeft)*this->sign1(angularRight);
	        angularLeft=this->speed*this->sign1(angularLeft);
	    }
	    else {
	        angularLeft=this->speed*abs(angularLeft/angularRight)*this->sign1(angularLeft);
	        angularRight=this->speed*this->sign1(angularRight);
	    }
	    
	    leftMotor(this->sign2(angularLeft),abs(angularLeft));
	    rightMotor(this->sign2(angularRight),abs(angularRight));
    }while(1);
}

void MiniExplorerCoimbra::update_force(int widthIndice, int heightIndice, float* forceRepulsionComputedX, float* forceRepulsionComputedY ){
    //get the coordonate of the map and the robot in the ortonormal space
    float xWorldCell=this->map.cell_width_coordinate_to_world(widthIndice);
    float yWorldCell=this->map.cell_height_coordinate_to_world(heightIndice);
    //compute the distance beetween the cell and the robot
    float distanceCellToRobot=sqrt(pow(xWorldCell-this->xWorld,2)+pow(yWorldCell-this->yWorld,2));
    float probaCell;
    //check if the cell is in range
    //float anglePointToRobot=atan2(yWorldCell-this->yWorld,xWorldCell-this->xWorld);//like world system
     float temp1;
     float temp2;       
    if(distanceCellToRobot <= this->rangeForce) {
        probaCell=this->map.get_proba_cell(widthIndice,heightIndice);
        //pc.printf("\r\nupdate force proba:%f",probaCell);
        temp1=this->repulsionConstantForce*probaCell/pow(distanceCellToRobot,2);
        temp2=(xWorldCell-this->xWorld)/distanceCellToRobot;
        *forceRepulsionComputedX+=temp1*temp2;
        temp2=(yWorldCell-this->yWorld)/distanceCellToRobot;
        *forceRepulsionComputedY+=temp1*temp2;
    }
}

//return 1 if positiv, -1 if negativ
float MiniExplorerCoimbra::sign1(float value){
    if(value>=0) 
        return 1;
    else 
        return -1;
}

//return 1 if positiv, 0 if negativ
int MiniExplorerCoimbra::sign2(float value){
    if(value>=0) 
        return 1;
    else 
        return 0;
}

float MiniExplorerCoimbra::distFromLine(float robot_x, float robot_y, float line_a, float line_b, float line_c){
    return abs((line_a*robot_x+line_b*robot_y+line_c)/sqrt(line_a*line_a+line_b*line_b));
}

//4th LAB
//starting position lower left

void MiniExplorerCoimbra::test_procedure_lab_4(float sizeX, float sizeY){
	
	this->map.fill_map_with_kalman_knowledge();
	this->go_to_point_kalman2(this->xWorld+sizeX,this->yWorld+sizeY);
}

void MiniExplorerCoimbra::myOdometriaBis(){
	float R,L;
	this->testOdometria(&R,&L);

	this->xWorld=-Y;
	this->yWorld=X;
	if(theta >PI/2)
		this->thetaWorld=-PI+(theta-PI/2);
	else
		this->thetaWorld=theta+PI/2;
}

void MiniExplorerCoimbra::go_to_point_kalman(float targetXWorld, float targetYWorld) {
    
    float angleError; //angle error
    float d; //distance from target
    float angularLeft, angularRight, linear, angular;
    int speed=300;

    do {        
        //Updating X,Y and theta with the odometry values
        this->OdometriaKalmanFilter();
		pc.printf("\n\r X=%f", this->xWorld);
		pc.printf("\n\r Y=%f", this->yWorld);
    	pc.printf("\n\r theta=%f", this->thetaWorld);
        //Computing angle error and distance towards the target value
        angleError = atan2((targetYWorld-this->yWorld),(targetXWorld-this->xWorld))-this->thetaWorld;
        if(angleError>PI) angleError=-(angleError-PI);
        else if(angleError<-PI) angleError=-(angleError+PI);
        //pc.printf("\n\r error=%f",angleError);
        
        d=this->dist(this->xWorld, this->yWorld, targetXWorld, targetYWorld);
		//pc.printf("\n\r dist=%f/n", d);

        //Computing linear and angular velocities
        linear=k_linear*d;
        angular=k_angular*angleError;
        angularLeft=(linear-0.5*this->distanceWheels*angular)/this->radiusWheels;
        angularRight=(linear+0.5*this->distanceWheels*angular)/this->radiusWheels;
        

        //Normalize speed for motors
        if(angularLeft>angularRight) {
            angularRight=speed*angularRight/angularLeft;
            angularLeft=speed;
        } else {
            angularLeft=speed*angularLeft/angularRight;
            angularRight=speed;
        }

        //Updating motor velocities
        if(angularLeft>0){
            leftMotor(1,angularLeft);
        }
        else{
            leftMotor(0,-angularLeft);
        }
        
        if(angularRight>0){
            rightMotor(1,angularRight);
        }
        else{
            rightMotor(0,-angularRight);
        }

        //wait(0.5);
    } while(d>1);
	pc.printf("\r\n REACHED");
    //Stop at the end
    leftMotor(1,0);
    rightMotor(1,0);
}

void MiniExplorerCoimbra::go_to_point_kalman2(float targetXWorld, float targetYWorld) {
    //float d; //distance from target
    float angularLeft, angularRight;
	float correctX;
    float correctY;
    float correctTheta;
    float xAtEnd=targetXWorld;
    do {        
        //Updating X,Y and theta with the odometry values
        this->OdometriaKalmanFilter();
    	
        correctX=-this->myY_r_correct;
	    correctY=this->myX_r_correct;
		if(this->mytheta_r_correct >PI/2)
			correctTheta=-PI+(this->mytheta_r_correct-PI/2);
		else
			correctTheta=this->mytheta_r_correct+PI/2;
			
		pc.printf("\n\rFalse: X=%f,Y=%f,theta=%f", this->xWorld,this->yWorld,this->thetaWorld);
		pc.printf("\n\rTrue : X=%f,Y=%f,theta=%f", correctX,correctY,correctTheta);	
		//pc.printf("\n\rFalse Robot: X=%f,Y=%f,theta=%f", this->myX_r,this->myY_r,this->mytheta_r);
		//pc.printf("\n\rTrue Robot: X=%f,Y=%f,theta=%f", this->myX_r_correct,this->myY_r_correct,this->mytheta_r_correct);
        //d=this->dist(correctX, correctY, targetXWorld, targetYWorld);
		//pc.printf("\n\r dist=%f/n", d);
        angularLeft=200;
        angularRight=200;
        leftMotor(1,angularLeft);
        rightMotor(1,angularRight);
        
    } while((xAtEnd-correctX)>1);
    //Stop at the end
    leftMotor(1,0);
    rightMotor(1,0);
	pc.printf("\r\n REACHED");
    
    while(1){
    	pc.printf("\n\rFalse: X=%f,Y=%f,theta=%f", this->xWorld,this->yWorld,this->thetaWorld);
		pc.printf("\n\rTrue : X=%f,Y=%f,theta=%f", correctX,correctY,correctTheta);
		//pc.printf("\n\rFalse Robot: X=%f,Y=%f,theta=%f", this->myX_r,this->myY_r,this->mytheta_r);
		//pc.printf("\n\rTrue Robot: X=%f,Y=%f,theta=%f", this->myX_r_correct,this->myY_r_correct,this->mytheta_r_correct);
    }
}

void MiniExplorerCoimbra::testOdometria(float *D_Special,float *L_Special){
	float encoderRightFailureRate_Special=1;
	float encoderLeftFailureRate_Special=0.95;
	
	long int ticks1d_Special=R_encoder();
    long int ticks1e_Special=L_encoder();

    long int D_ticks_Special=ticks1d_Special - this->ticks2d_Special;
    long int E_ticks_Special=ticks1e_Special - this->ticks2e_Special;

    this->ticks2d_Special=ticks1d_Special;
    this->ticks2e_Special=ticks1e_Special;
	
    *D_Special= (float)D_ticks_Special*((3.25f*3.1415f)/4096);
    *L_Special= (float)E_ticks_Special*((3.25f*3.1415f)/4096);
    
    float CM_O=(*D_Special + *L_Special)/2;

    //pc.printf("\r\nCM:%f",CM);
    this->mytheta_r_correct +=(*D_Special - *L_Special)/7.18;
 
 	//modify D, L
    float temp_Special=*D_Special;
    *D_Special=temp_Special*encoderRightFailureRate_Special;
	temp_Special=*L_Special;
	*L_Special=temp_Special*encoderLeftFailureRate_Special;
	
    float CM_Special=(*D_Special + *L_Special)/2;

    this->mytheta_r +=(*D_Special -*L_Special)/7.18;

    this->mytheta_r = atan2(sin(this->mytheta_r), cos(this->mytheta_r));
    this->myX_r += CM_Special*cos(this->mytheta_r);
    this->myY_r += CM_Special*sin(this->mytheta_r);
    
    this->mytheta_r_correct = atan2(sin(this->mytheta_r_correct), cos(this->mytheta_r_correct));
    this->myX_r_correct += CM_O*cos(this->mytheta_r_correct);
    this->myY_r_correct += CM_O*sin(this->mytheta_r_correct);
}


//the kalman filter is done but I have memory leaks (an issue with myMatrix destructor, but I can't fix it somehow, if I let it empty it will crash after a few seconds of working, if I fill it with what seems sensible it will crash when the destructor is called
void MiniExplorerCoimbra::OdometriaKalmanFilter(){
   	float L_CM=0;
	float R_CM=0;
	int skip=0;//1==skip Kalman correction, just use KINEMATICS
	//=====KINEMATICS===========================
	this->testOdometria(&R_CM,&L_CM);

    float distanceMoved=(R_CM+L_CM)/2;
	float angleMovedR=(R_CM-L_CM)/this->distanceWheels;
	float angleMoved=0;
	if(angleMovedR >PI/2)
		angleMoved=-PI+(angleMovedR-PI/2);
	else
		angleMoved=angleMovedR+PI/2;
		
	float newxWorld=this->xWorld+distanceMoved*cos(this->thetaWorld+angleMoved/2);
    float newyWorld=this->yWorld+distanceMoved*sin(this->thetaWorld+angleMoved/2);
    float newthetaWorld=this->thetaWorld+angleMoved;
    if(newthetaWorld>PI) newthetaWorld=-(newthetaWorld-PI);
        else if(newthetaWorld<-PI) newthetaWorld=-(newthetaWorld+PI);

	myMatrix poseKplus1K(3,1);
	poseKplus1K.data[0][0]=newxWorld;
	poseKplus1K.data[1][0]=newyWorld;
	poseKplus1K.data[2][0]=newthetaWorld;
	
	//pc.printf("\r\nR:%f,L:%f",R_CM,L_CM);
	if(skip==1){
		//update pose 
		this->xWorld=poseKplus1K.data[0][0];
		this->yWorld=poseKplus1K.data[1][0];
		this->thetaWorld=poseKplus1K.data[2][0];
		//pc.printf("\r\nWithout Kalman X=%f, Y=%f, theta=%f",xWorld,yWorld,thetaWorld);
	}else{
		//note if the encoders are correct it should work
	    //pc.printf("\r\nWithout correction: X=%f, Y=%f, theta=%f",poseKplus1K.get(0,0),poseKplus1K.get(1,0),poseKplus1K.get(2,0));
	    //=====ERROR_MODEL===========================
	
	   	//FP Matrix slide LocalizationKALMAN                     
	    myMatrix Fp(3,3);
	    Fp.data[0][0]=1;
	    Fp.data[1][1]=1;
	    Fp.data[2][2]=1;
	    Fp.data[0][2]=-1*distanceMoved*sin(this->thetaWorld+(angleMoved/2)); 
	    Fp.data[1][2]=distanceMoved*cos(this->thetaWorld+(angleMoved/2)); 
		
		myMatrix FpTranspose(3,3);
		FpTranspose.fillWithTranspose(Fp);
		//pc.printf("\r\nFp");;
	    //Frl matrix slide LocalizationKALMAN
	    myMatrix Frl(3,2);
	    Frl.data[0][0]=0.5*cos(this->thetaWorld+(angleMoved/2))-(distanceMoved/(2*this->distanceWheels))*sin(this->thetaWorld+(angleMoved/2));
	    Frl.data[0][1]=0.5*cos(this->thetaWorld+(angleMoved/2))+(distanceMoved/(2*this->distanceWheels))*sin(this->thetaWorld+(angleMoved/2));
	    Frl.data[1][0]=0.5*sin(this->thetaWorld+(angleMoved/2))+(distanceMoved/(2*this->distanceWheels))*cos(this->thetaWorld+(angleMoved/2));
	    Frl.data[1][1]=0.5*sin(this->thetaWorld+(angleMoved/2))-(distanceMoved/(2*this->distanceWheels))*cos(this->thetaWorld+(angleMoved/2));
	 	Frl.data[2][0]=(1/this->distanceWheels);
	    Frl.data[2][1]=-(1/this->distanceWheels) ;
	    
	    myMatrix FrlTranspose(2,3);
		FrlTranspose.fillWithTranspose(Frl);
	    //pc.printf("\r\nFrl");
	     //error constants...
	    float kr=1;
	    float kl=1;
	    myMatrix covar(2,2);
	    covar.data[0][0]=kr*abs(R_CM);
	    covar.data[1][1]=kl*abs(L_CM);
		//pc.printf("\r\ncovar");
		//uncertainty positionx, and theta at 
		//1,1,1
	    myMatrix Q(3,3);
	    Q.data[0][0]=1;
	    Q.data[1][1]=2;
	    Q.data[2][2]=0.01;
	    //pc.printf("\r\nQ");
	    //new covariance= Fp*old covariance*FpTranspose +Frl*covar*FrlTranspose +Q slide LocalizationKALMAN
		myMatrix covariancePositionEstimationKplus1K(3,3);
		myMatrix temp1(3,3);
		temp1.fillByMultiplication(Fp,this->covariancePositionEstimationK);//Fp*old covariance

		myMatrix temp2(3,3);
		temp2.fillByMultiplication(temp1,FpTranspose);//Fp*old covariance*FpTranspose
		//pc.printf("\r\ncovariancePositionEstimationKplus1K  1");
		temp1.fillWithZeroes();

		myMatrix temp3(3,2);
		temp3.fillByMultiplication(Frl,covar);//Frl*covar

		temp1.fillByMultiplication(temp3,FrlTranspose);//Frl*covar*FrlTranspose

		covariancePositionEstimationKplus1K.addition(temp2);//Fp*old covariance*FpTranspose
		covariancePositionEstimationKplus1K.addition(temp1);//Fp*old covariance*FpTranspose +Frl*covar*FrlTranspose
		covariancePositionEstimationKplus1K.addition(Q);//Fp*old covariance*FpTranspose +Frl*covar*FrlTranspose +Q
    	//pc.printf("\r\ncovariancePositionEstimationKplus1K");
		//=====OBSERVATION=====
	    //get the estimated measure we should have according to our knowledge of the map and the previously computed localisation
	    
	    float leftCmEstimated=this->sonarLeft.maxRange;
	    float frontCmEstimated=this->sonarFront.maxRange;
	    float rightCmEstimated=this->sonarRight.maxRange;
	    float xWorldCell;
	    float yWorldCell;
	    float currDistance;
	    float xClosestCellLeft;
	    float yClosestCellLeft;
	    float xClosestCellFront;
	    float yClosestCellFront;
	    float xClosestCellRight;
	    float yClosestCellRight;
	    
	    //note: sonar.isInRange already incorpore the sonar position and angle relative to the robot
	    //note curr change to isInRange: xs-xcell and not xcell-xs 
	    //get theorical distance to sonar
	    for(int i=0;i<this->map.nbCellWidth;i++){
	        for(int j=0;j<this->map.nbCellHeight;j++){
	            //check if occupied, if not discard
	            if(this->map.get_proba_cell(i,j)>0.5){
	                //check if in sonar range
	                xWorldCell=this->map.cell_width_coordinate_to_world(i);
	                yWorldCell=this->map.cell_height_coordinate_to_world(j);
	                //check left
	                currDistance=this->sonarLeft.isInRange(xWorldCell,yWorldCell,poseKplus1K.data[0][0],poseKplus1K.data[1][0],poseKplus1K.data[2][0]);           
	                if((currDistance < this->sonarLeft.maxRange) && currDistance > -1){
	                    //check if distance is lower than previous, update lowest if so
	                    if(currDistance < leftCmEstimated){
	                        leftCmEstimated= currDistance;
	                        xClosestCellLeft=xWorldCell;
	                        yClosestCellLeft=yWorldCell;
	                    }
	            	}
	                //check front
	                currDistance=this->sonarFront.isInRange(xWorldCell,yWorldCell,poseKplus1K.data[0][0],poseKplus1K.data[1][0],poseKplus1K.data[2][0]);         
	                if((currDistance < this->sonarFront.maxRange) && currDistance > -1){
	                    //check if distance is lower than previous, update lowest if so
	                    if(currDistance < frontCmEstimated){
	                        frontCmEstimated= currDistance;
	                        xClosestCellFront=xWorldCell;
	                        yClosestCellFront=yWorldCell;
	                    }
	                }
	                //check right
	                currDistance=this->sonarRight.isInRange(xWorldCell,yWorldCell,poseKplus1K.data[0][0],poseKplus1K.data[1][0],poseKplus1K.data[2][0]);             
	                if((currDistance < this->sonarRight.maxRange) && currDistance > -1){
	                    //check if distance is lower than previous, update lowest if so
	                    if(currDistance < rightCmEstimated){
	                        rightCmEstimated= currDistance;
	                        xClosestCellRight=xWorldCell;
	                        yClosestCellRight=yWorldCell;
	                    }
	                }
	            }
	        }
	    }
	
	    //check measurements from sonars, see if they passed the validation gate
	    float leftCm = get_distance_left_sensor()/10;
	    float frontCm = get_distance_front_sensor()/10;
	    float rightCm = get_distance_right_sensor()/10;
	   
	    //if superior to sonar range, put the value to sonar max range
	    if(leftCm > this->sonarLeft.maxRange)
	        leftCm=this->sonarLeft.maxRange;
	    if(frontCm > this->sonarFront.maxRange)
	        frontCm=this->sonarFront.maxRange;
	    if(rightCm > this->sonarRight.maxRange)
	        rightCm=this->sonarRight.maxRange;
	        
	    //pc.printf("\r\ndistance sonars read Lcm=%f, FCm=%f, RCm=%f",leftCm,frontCm,rightCm);
	    //pc.printf("\r\ndistance sonars estimated ELcm=%f, EFCm=%f, ERCm=%f",leftCmEstimated,frontCmEstimated,rightCmEstimated);
	
	    //======INNOVATION========
	    //compute jacobian of observation
	    myMatrix jacobianOfObservationLeft(1,3);
	    myMatrix jacobianOfObservationRight(1,3);
	    myMatrix jacobianOfObservationFront(1,3);
	    
	    //(5)
	    //note: for cos/sin having abs(angles) > PI is not an issue
	    float xSonarLeft=poseKplus1K.data[0][0]+this->sonarLeft.distanceX*cos(poseKplus1K.data[0][2])-this->sonarLeft.distanceY*sin(poseKplus1K.data[0][2]);
	    float ySonarLeft=poseKplus1K.data[1][0]+this->sonarLeft.distanceX*sin(poseKplus1K.data[0][2])-this->sonarLeft.distanceY*cos(poseKplus1K.data[0][2]);
	    float angleSonarLeft=poseKplus1K.data[0][2]+this->sonarLeft.angleFromCenter;
	    if(angleSonarLeft > PI)
            angleSonarLeft=angleSonarLeft-2*PI;
        if(angleSonarLeft < -PI)
            angleSonarLeft=angleSonarLeft+2*PI;
	    
	    float xSonarFront=poseKplus1K.data[0][0]+this->sonarFront.distanceX*cos(poseKplus1K.data[0][2])-this->sonarFront.distanceY*sin(poseKplus1K.data[0][2]);
	    float ySonarFront=poseKplus1K.data[1][0]+this->sonarFront.distanceX*sin(poseKplus1K.data[0][2])-this->sonarFront.distanceY*cos(poseKplus1K.data[0][2]);
	    float angleSonarFront=poseKplus1K.data[0][2]+this->sonarFront.angleFromCenter;
	    if(angleSonarFront > PI)
            angleSonarFront=angleSonarFront-2*PI;
        if(angleSonarFront < -PI)
            angleSonarFront=angleSonarFront+2*PI;
	    
	    float xSonarRight=poseKplus1K.data[0][0]+this->sonarRight.distanceX*cos(poseKplus1K.data[0][2])-this->sonarRight.distanceY*sin(poseKplus1K.data[0][2]);
	    float ySonarRight=poseKplus1K.data[1][0]+this->sonarRight.distanceX*sin(poseKplus1K.data[0][2])-this->sonarRight.distanceY*cos(poseKplus1K.data[0][2]);
		float angleSonarRight=poseKplus1K.data[0][2]+this->sonarRight.angleFromCenter;
		if(angleSonarRight > PI)
            angleSonarRight=angleSonarRight-2*PI;
        if(angleSonarRight < -PI)
            angleSonarRight=angleSonarRight+2*PI;
		
		//note: last line divided by d, [0][2] all angles are theta^ teacher's advice, error in paper
	    //left
	    jacobianOfObservationLeft.data[0][0]=(xSonarLeft-xClosestCellLeft)/leftCmEstimated;
	    jacobianOfObservationLeft.data[0][1]=(ySonarLeft-yClosestCellLeft)/leftCmEstimated;
	    jacobianOfObservationLeft.data[0][2]=(1/leftCmEstimated)*(xClosestCellLeft-xSonarLeft)*(this->sonarLeft.distanceX*sin(angleSonarLeft)+this->sonarLeft.distanceY*cos(angleSonarLeft))+(yClosestCellLeft-ySonarLeft)*(-this->sonarLeft.distanceX*cos(angleSonarLeft)+this->sonarLeft.distanceY*sin(angleSonarLeft));
	    
	    //front
	    jacobianOfObservationFront.data[0][0]=(xSonarFront-xClosestCellFront)/frontCmEstimated;
	    jacobianOfObservationFront.data[0][1]=(ySonarFront-yClosestCellFront)/frontCmEstimated;
	    jacobianOfObservationFront.data[0][2]=(1/frontCmEstimated)*(xClosestCellFront-xSonarFront)*(this->sonarFront.distanceX*sin(angleSonarFront)+this->sonarFront.distanceY*cos(angleSonarFront))+(yClosestCellFront-ySonarFront)*(-this->sonarFront.distanceX*cos(angleSonarFront)+this->sonarFront.distanceY*sin(angleSonarFront));
	    
	    //right
	    jacobianOfObservationRight.data[0][0]=(xSonarRight-xClosestCellRight)/rightCmEstimated;
	    jacobianOfObservationRight.data[0][1]=(ySonarRight-yClosestCellRight)/rightCmEstimated;
	    jacobianOfObservationRight.data[0][2]=(xClosestCellRight-xSonarRight)*(xSonarRight*sin(poseKplus1K.data[2][0])+ySonarRight*cos(poseKplus1K.data[2][0]))+(yClosestCellRight-ySonarRight)*(-xSonarRight*cos(poseKplus1K.data[2][0])+ySonarRight*sin(poseKplus1K.data[2][0]));
	    jacobianOfObservationRight.data[0][2]=(1/rightCmEstimated)*(xClosestCellRight-xSonarRight)*(this->sonarRight.distanceX*sin(angleSonarRight)+this->sonarRight.distanceY*cos(angleSonarRight))+(yClosestCellRight-ySonarRight)*(-this->sonarRight.distanceX*cos(angleSonarRight)+this->sonarRight.distanceY*sin(angleSonarRight));
	
		myMatrix jacobianOfObservationRightTranspose(3,1);
		jacobianOfObservationRightTranspose.fillWithTranspose(jacobianOfObservationRight);
		
		myMatrix jacobianOfObservationFrontTranspose(3,1);
		jacobianOfObservationFrontTranspose.fillWithTranspose(jacobianOfObservationFront);
		
		myMatrix jacobianOfObservationLeftTranspose(3,1);
		jacobianOfObservationLeftTranspose.fillWithTranspose(jacobianOfObservationLeft);
	    //error constants 0,0,0 sonars perfect;  must be found by experimenting; gives mean and standart deviation
	    //in centimeters
	    float R_front=4;
	    float R_left=4;
	    float R_right=4;
	    
	    //R-> 4 in diagonal
	
	    //for each sonar (concatenated covariance matrix of innovation)
	    //equ (12), innovationCovariance =JacobianObservation*covariancePositionEstimationKplus1K*JacobianObservationTranspose+R
	    myMatrix temp4(1,3);
	    temp4.fillByMultiplication(jacobianOfObservationFront,covariancePositionEstimationKplus1K);
	    myMatrix temp5(1,1);
	    temp5.fillByMultiplication(temp4,jacobianOfObservationFrontTranspose);
	    float innovationCovarianceFront=temp5.data[0][0]+R_front;
	    temp4.fillWithZeroes();
	    temp5.fillWithZeroes();
	
	    temp4.fillByMultiplication(jacobianOfObservationLeft,covariancePositionEstimationKplus1K);
	    temp5.fillByMultiplication(temp4,jacobianOfObservationLeftTranspose);
	    float innovationCovarianceLeft=temp5.data[0][0]+R_left;
	    temp4.fillWithZeroes();
	    temp5.fillWithZeroes();
	
	    temp4.fillByMultiplication(jacobianOfObservationRight,covariancePositionEstimationKplus1K);
	    temp5.fillByMultiplication(temp4,jacobianOfObservationRightTranspose);
	    float innovationCovarianceRight=temp5.data[0][0]+R_right;
		
		this->printMatrix(jacobianOfObservationLeft);
		this->printMatrix(jacobianOfObservationFront);
		this->printMatrix(jacobianOfObservationRight);
		//pc.printf("\r\ninnovationCovariance L:%f, F:%f, R:%f",innovationCovarianceLeft,innovationCovarianceFront,innovationCovarianceRight);
		
	    //get the innoncation: the value of the difference between the actual measure and what we anticipated
	    float innovationLeft=leftCm-leftCmEstimated;
	    float innovationFront=frontCm-frontCmEstimated;
	    float innovationRight=rightCm-rightCmEstimated;
		
	    //check if it pass the validation gate 
	    float gateScoreLeft=innovationLeft*innovationLeft/innovationCovarianceLeft;
	    float gateScoreFront=innovationFront*innovationFront/innovationCovarianceFront;
	    float gateScoreRight=innovationRight*innovationRight/innovationCovarianceRight;
	    
		//5cm -> 25
	    int errorsquare=25;//constant error
	    
		int nbPassed=0;
		
	    if(gateScoreLeft <= errorsquare)
	        nbPassed+=1;
	    if(gateScoreFront <= errorsquare)
	        nbPassed+=10;
	    if(gateScoreRight <= errorsquare)
	        nbPassed+=100;
	    //for those who passed
	    //compute composite innovation
	    
	    //pc.printf("\r\nscore L:%f, F:%f, R:%f",gateScoreLeft,gateScoreFront,gateScoreRight);
	    //pc.printf("\r\nnbPassed=%d",nbPassed);
    
		//compositeMeasurementJacobian 
	    myMatrix compositeMeasurementJacobian3x3(3,3);
		myMatrix compositeMeasurementJacobian2x3(2,3);
		myMatrix compositeMeasurementJacobian1x3(1,3);
		
		myMatrix compositeMeasurementJacobian3x3Transpose(3,3);
		myMatrix compositeMeasurementJacobian2x3Transpose(3,2);
		myMatrix compositeMeasurementJacobian1x3Transpose(3,1);
	
		//compositeInnovation
		myMatrix compositeInnovation3x1(3,1);
		myMatrix compositeInnovation2x1(2,1);
		myMatrix compositeInnovation1x1(1,1);
		
		//compositeInnovationCovariance 
		myMatrix compositeInnovationCovariance3x3(3,3);
		myMatrix compositeInnovationCovariance2x2(2,2);
		myMatrix compositeInnovationCovariance1x1(1,1);
		
		myMatrix tempCompositeInnovationCovariance3x3(3,3);
		myMatrix tempCompositeInnovationCovariance2x2(2,2);
		
		myMatrix compositeInnovationCovariance3x3Inverse(3,3);		
		myMatrix compositeInnovationCovariance2x2Inverse(2,2);
		myMatrix compositeInnovationCovariance1x1Inverse(1,1);							  
		
		//Kalman Gain
		myMatrix kalmanGain3X3(3,3);
		myMatrix kalmanGain3X2(3,2);
		myMatrix kalmanGain3X1(3,1);
		
		myMatrix tempKalmanGain3X3(3,3);
		myMatrix tempKalmanGain3X2(3,2);
		myMatrix tempKalmanGain3X1(3,1);
		
		myMatrix kalmanGain3X3Transpose(3,3);
		myMatrix kalmanGain3X2Transpose(2,3);
		myMatrix kalmanGain3X1Transpose(1,3);
		
		//new pose estimation
		myMatrix poseKplus1Kplus1(3,1);
		poseKplus1Kplus1.fillByCopy(poseKplus1K);
		myMatrix tempPoseKplus1Kplus1(3,1);
		
		//covariancePositionEstimationKplus1Kplus1
		myMatrix covariancePositionEstimationKplus1Kplus1(3,3);
		covariancePositionEstimationKplus1Kplus1.fillByCopy(covariancePositionEstimationKplus1K);
		
		myMatrix temp2CovariancePositionEstimationKplus1Kplus13x3(3,3);
		myMatrix tempCovariancePositionEstimationKplus1Kplus13x3(3,3);
		myMatrix tempCovariancePositionEstimationKplus1Kplus13x2(3,2);
		myMatrix tempCovariancePositionEstimationKplus1Kplus13x1(3,1);
		
	    //we do not use the composite measurement jacobian (16), we directly use the values from the measurement jacobian (jacobianOfObservation)
	    switch(nbPassed){
	        case 0://none
	           	//nothings happens it's okay
	           	//pc.printf("\r\n000");
	            break;
	        case 1://left
	   			//compute compositeMeasurementJacobian
	   			//here compositeMeasurementJacobian= jacobianOfObservationLeft
	   			compositeMeasurementJacobian1x3.fillByCopy(jacobianOfObservationLeft);
	   			//pc.printf("\r\n1:calcul compositeMeasurementJacobian1x3 passed");
	   			//get the compositeMeasurementJacobianTranspose
	   			compositeMeasurementJacobian1x3Transpose.fillWithTranspose(compositeMeasurementJacobian1x3);
	   			//compute compositeInnovation
	   			//here compositeInnovation=innovationLeft
	   			compositeInnovation1x1.data[0][0]=innovationLeft;
	   			//pc.printf("\r\n1:calcul compositeInnovation1x1 passed");
	   			//compute compositeInnovationCovariance=compositeMeasurementJacobian*covariancePositionEstimationKplus1K*compositeMeasurementJacobianTranpose
	   			//add the right R on the diagonal
	   			//here compositeInnovationCovariance=innovationCovarianceLeft
	   			compositeInnovationCovariance1x1.data[0][0]=innovationCovarianceLeft;
	   			//pc.printf("\r\n1:calcul compositeInnovationCovariance1x1 passed");
	   			//get the inverse of the compositeInnovationCovariance
	   			compositeInnovationCovariance1x1Inverse.data[0][0]=1/innovationCovarianceLeft;
	   			//pc.printf("\r\n1:calcul compositeInnovationCovariance1x1Inverse passed");
	   			//compute KalmanGain=covariancePositionEstimationKplus1K*compositeMeasurementJacobianTranspose*Inverse(compositeInnovationCovariance)
	            tempKalmanGain3X1.fillByMultiplication(covariancePositionEstimationKplus1K,compositeMeasurementJacobian1x3Transpose);
	            kalmanGain3X1.fillByMultiplication(tempKalmanGain3X1,compositeInnovationCovariance1x1Inverse);
	            //pc.printf("\r\n1:calcul kalmanGain3X1 passed");
	            //get the transpose of the kalman gain
	            kalmanGain3X1Transpose.fillWithTranspose(kalmanGain3X1);
	            //update pose estimation=old pose estimation + kalman gain*compositeInnovation
	            tempPoseKplus1Kplus1.fillByMultiplication(kalmanGain3X1,compositeInnovation1x1);
	            poseKplus1Kplus1.addition(tempPoseKplus1Kplus1);
	            //pc.printf("\r\n1:calcul poseKplus1Kplus1 passed");
	 			//compute covariancePositionEstimationKplus1Kplus1=covariancePositionEstimationKplus1K-kalmanGain*compositeInnovationCovariance*KalmanGainTranspose       
				tempCovariancePositionEstimationKplus1Kplus13x1.fillByMultiplication(kalmanGain3X1,compositeInnovationCovariance1x1);
				tempCovariancePositionEstimationKplus1Kplus13x3.fillByMultiplication(tempCovariancePositionEstimationKplus1Kplus13x1,kalmanGain3X1Transpose);
	            covariancePositionEstimationKplus1Kplus1.subtraction(tempCovariancePositionEstimationKplus1Kplus13x3);
	            //pc.printf("\r\n1:calcul covariancePositionEstimationKplus1Kplus1 passed");
	            break;
	        case 10://front
	        	//compute compositeMeasurementJacobian
	   			compositeMeasurementJacobian1x3.fillByCopy(jacobianOfObservationFront);
	   			//pc.printf("\r\n10:calcul compositeMeasurementJacobian1x3 passed");
	   			//get the compositeMeasurementJacobianTranspose
	   			compositeMeasurementJacobian1x3Transpose.fillWithTranspose(compositeMeasurementJacobian1x3);
	   			//compute compositeInnovation
	   			compositeInnovation1x1.data[0][0]=innovationFront;
	   			//pc.printf("\r\n10:calcul compositeInnovation1x1 passed");
	   			//compute compositeInnovationCovariance=compositeMeasurementJacobian*covariancePositionEstimationKplus1K*compositeMeasurementJacobianTranpose
	   			//add the right R on the diagonal
	   			compositeInnovationCovariance1x1.data[0][0]=innovationCovarianceFront;
	   			//pc.printf("\r\n10:calcul compositeInnovationCovariance1x1 passed");
	   			//get the inverse of the compositeInnovationCovariance
	   			compositeInnovationCovariance1x1Inverse.data[0][0]=1/innovationCovarianceFront;
	   			//pc.printf("\r\n10:calcul compositeInnovationCovariance1x1Inverse passed");
	   			//compute KalmanGain=covariancePositionEstimationKplus1K*compositeMeasurementJacobianTranspose*Inverse(compositeInnovationCovariance)
	            tempKalmanGain3X1.fillByMultiplication(covariancePositionEstimationKplus1K,compositeMeasurementJacobian1x3Transpose);
	            kalmanGain3X1.fillByMultiplication(tempKalmanGain3X1,compositeInnovationCovariance1x1Inverse);
	            //pc.printf("\r\n10:calcul kalmanGain3X1 passed");
	            //get the transpose of the kalman gain
	            kalmanGain3X1Transpose.fillWithTranspose(kalmanGain3X1);
	            //update pose estimation=old pose estimation + kalman gain*compositeInnovation
	            tempPoseKplus1Kplus1.fillByMultiplication(kalmanGain3X1,compositeInnovation1x1);
	            poseKplus1Kplus1.addition(tempPoseKplus1Kplus1);
	            //pc.printf("\r\n10:calcul poseKplus1Kplus1 passed");
	 			//compute covariancePositionEstimationKplus1Kplus1=covariancePositionEstimationKplus1K-kalmanGain*compositeInnovationCovariance*KalmanGainTranspose       
				tempCovariancePositionEstimationKplus1Kplus13x1.fillByMultiplication(kalmanGain3X1,compositeInnovationCovariance1x1);
				tempCovariancePositionEstimationKplus1Kplus13x3.fillByMultiplication(tempCovariancePositionEstimationKplus1Kplus13x1,kalmanGain3X1Transpose);
	            covariancePositionEstimationKplus1Kplus1.subtraction(tempCovariancePositionEstimationKplus1Kplus13x3);
	        	//pc.printf("\r\n10:calcul covariancePositionEstimationKplus1Kplus1 passed");
	            break;
	        case 11://left and front
	        	//compute compositeMeasurementJacobian
	   			compositeMeasurementJacobian2x3.data[0][0]=jacobianOfObservationLeft.data[0][0];
	   			compositeMeasurementJacobian2x3.data[0][1]=jacobianOfObservationLeft.data[0][1];
	   			compositeMeasurementJacobian2x3.data[0][2]=jacobianOfObservationLeft.data[0][2];
	   			
	   			compositeMeasurementJacobian2x3.data[1][0]=jacobianOfObservationFront.data[0][0];
	   			compositeMeasurementJacobian2x3.data[1][1]=jacobianOfObservationFront.data[0][1];
	   			compositeMeasurementJacobian2x3.data[1][2]=jacobianOfObservationFront.data[0][2];
	   			//pc.printf("\r\n11:calcul compositeMeasurementJacobian2x3 passed");
	   			//get the compositeMeasurementJacobianTranspose
	   			compositeMeasurementJacobian2x3Transpose.fillWithTranspose(compositeMeasurementJacobian2x3);
	   			//compute compositeInnovation
	   			compositeInnovation2x1.data[0][0]=innovationLeft;
	   			compositeInnovation2x1.data[1][0]=innovationFront;
	   			//pc.printf("\r\n11:calcul compositeInnovation2x1 passed");
	   			//compute compositeInnovationCovariance=compositeMeasurementJacobian*covariancePositionEstimationKplus1K*compositeMeasurementJacobianTranpose+R
	   			tempCompositeInnovationCovariance2x2.fillByMultiplication(compositeMeasurementJacobian2x3,covariancePositionEstimationKplus1K);
	   			compositeInnovationCovariance2x2.fillByMultiplication(tempCompositeInnovationCovariance2x2,compositeMeasurementJacobian2x3Transpose);
	   			//add the right R on the diagonal
	   			compositeInnovationCovariance2x2.data[0][0]+=R_left;
	   			compositeInnovationCovariance2x2.data[1][1]+=R_front;
	   			//pc.printf("\r\n11:calcul compositeInnovationCovariance2x2 passed");
	   			//get the inverse of the compositeInnovationCovariance
	   			compositeInnovationCovariance2x2Inverse.fillWithInverse(compositeInnovationCovariance2x2);
	   			//pc.printf("\r\n11:calcul compositeInnovationCovariance2x2Inverse passed");
	   			//compute KalmanGain=covariancePositionEstimationKplus1K*compositeMeasurementJacobianTranspose*Inverse(compositeInnovationCovariance)
	            tempKalmanGain3X2.fillByMultiplication(covariancePositionEstimationKplus1K,compositeMeasurementJacobian2x3Transpose);
	            kalmanGain3X2.fillByMultiplication(tempKalmanGain3X2,compositeInnovationCovariance2x2Inverse);
	            //pc.printf("\r\n11:calcul kalmanGain3X2 passed");
	            //get the transpose of the kalman gain
	            kalmanGain3X2Transpose.fillWithTranspose(kalmanGain3X2);
	            //update pose estimation=old pose estimation + kalman gain*compositeInnovation
	            tempPoseKplus1Kplus1.fillByMultiplication(kalmanGain3X2,compositeInnovation2x1);
	            poseKplus1Kplus1.addition(tempPoseKplus1Kplus1);
	            //pc.printf("\r\n11:calcul poseKplus1Kplus1 passed");
	 			//compute covariancePositionEstimationKplus1Kplus1=covariancePositionEstimationKplus1K-kalmanGain*compositeInnovationCovariance*KalmanGainTranspose       
				tempCovariancePositionEstimationKplus1Kplus13x2.fillByMultiplication(kalmanGain3X2,compositeInnovationCovariance2x2);
				tempCovariancePositionEstimationKplus1Kplus13x3.fillByMultiplication(tempCovariancePositionEstimationKplus1Kplus13x2,kalmanGain3X2Transpose);
	            covariancePositionEstimationKplus1Kplus1.subtraction(tempCovariancePositionEstimationKplus1Kplus13x3);
	        	//pc.printf("\r\n11:calcul covariancePositionEstimationKplus1Kplus1 passed");
	            break;
	            
	        case 100://right
	        	//compute compositeMeasurementJacobian
	   			compositeMeasurementJacobian1x3.fillByCopy(jacobianOfObservationRight);
	   			//pc.printf("\r\n100:calcul compositeMeasurementJacobian1x3 passed");
	   			//get the compositeMeasurementJacobianTranspose
	   			compositeMeasurementJacobian1x3Transpose.fillWithTranspose(compositeMeasurementJacobian1x3);
	   			//compute compositeInnovation
	   			compositeInnovation1x1.data[0][0]=innovationRight;
	   			//compute compositeInnovationCovariance=compositeMeasurementJacobian*covariancePositionEstimationKplus1K*compositeMeasurementJacobianTranpose
	   			//add the right R on the diagonal
	   			compositeInnovationCovariance1x1.data[0][0]=innovationCovarianceRight;
	   			//get the inverse of the compositeInnovationCovariance
	   			compositeInnovationCovariance1x1Inverse.data[0][0]=1/innovationCovarianceRight;
	   			//pc.printf("\r\n100:calcul compositeInnovationCovariance1x1Inverse passed");
	   			//compute KalmanGain=covariancePositionEstimationKplus1K*compositeMeasurementJacobianTranspose*Inverse(compositeInnovationCovariance)
	            tempKalmanGain3X1.fillByMultiplication(covariancePositionEstimationKplus1K,compositeMeasurementJacobian1x3Transpose);
	            kalmanGain3X1.fillByMultiplication(tempKalmanGain3X1,compositeInnovationCovariance1x1Inverse);
	            //pc.printf("\r\n100:calcul kalmanGain3X1 passed");
	            //get the transpose of the kalman gain
	            kalmanGain3X1Transpose.fillWithTranspose(kalmanGain3X1);
	            //update pose estimation=old pose estimation + kalman gain*compositeInnovation
	            tempPoseKplus1Kplus1.fillByMultiplication(kalmanGain3X1,compositeInnovation1x1);
	            poseKplus1Kplus1.addition(tempPoseKplus1Kplus1);
	            //pc.printf("\r\n100:calcul poseKplus1Kplus1 passed");
	 			//compute covariancePositionEstimationKplus1Kplus1=covariancePositionEstimationKplus1K-kalmanGain*compositeInnovationCovariance*KalmanGainTranspose       
				tempCovariancePositionEstimationKplus1Kplus13x1.fillByMultiplication(kalmanGain3X1,compositeInnovationCovariance1x1);
				tempCovariancePositionEstimationKplus1Kplus13x3.fillByMultiplication(tempCovariancePositionEstimationKplus1Kplus13x1,kalmanGain3X1Transpose);
	            covariancePositionEstimationKplus1Kplus1.subtraction(tempCovariancePositionEstimationKplus1Kplus13x3);
				//pc.printf("\r\n100:calcul covariancePositionEstimationKplus1Kplus1 passed");
	            break;
	        case 101://left and right 
	        	//compute compositeMeasurementJacobian
	   			compositeMeasurementJacobian2x3.data[0][0]=jacobianOfObservationLeft.data[0][0];
	   			compositeMeasurementJacobian2x3.data[0][1]=jacobianOfObservationLeft.data[0][1];
	   			compositeMeasurementJacobian2x3.data[0][2]=jacobianOfObservationLeft.data[0][2];
	   			
	   			compositeMeasurementJacobian2x3.data[1][0]=jacobianOfObservationRight.data[0][0];
	   			compositeMeasurementJacobian2x3.data[1][1]=jacobianOfObservationRight.data[0][1];
	   			compositeMeasurementJacobian2x3.data[1][2]=jacobianOfObservationRight.data[0][2];
	   			//pc.printf("\r\n101:calcul compositeMeasurementJacobian2x3 passed");
	   			//get the compositeMeasurementJacobianTranspose
	   			compositeMeasurementJacobian2x3Transpose.fillWithTranspose(compositeMeasurementJacobian2x3);
	   			//compute compositeInnovation
	   			compositeInnovation2x1.data[0][0]=innovationLeft;
	   			compositeInnovation2x1.data[1][0]=innovationRight;
	   			//compute compositeInnovationCovariance=compositeMeasurementJacobian*covariancePositionEstimationKplus1K*compositeMeasurementJacobianTranpose+R
	   			tempCompositeInnovationCovariance2x2.fillByMultiplication(compositeMeasurementJacobian2x3,covariancePositionEstimationKplus1K);
	   			compositeInnovationCovariance2x2.fillByMultiplication(tempCompositeInnovationCovariance2x2,compositeMeasurementJacobian2x3Transpose);
	   			//add the right R on the diagonal
	   			compositeInnovationCovariance2x2.data[0][0]+=R_left;
	   			compositeInnovationCovariance2x2.data[1][1]+=R_right;
	   			//pc.printf("\r\n101:calcul compositeInnovationCovariance2x2 passed");
	   			//get the inverse of the compositeInnovationCovariance
	   			compositeInnovationCovariance2x2Inverse.fillWithInverse(compositeInnovationCovariance2x2);
	   			//pc.printf("\r\n101:calcul compositeInnovationCovariance2x2Inverse passed");
	   			//compute KalmanGain=covariancePositionEstimationKplus1K*compositeMeasurementJacobianTranspose*Inverse(compositeInnovationCovariance)
	            tempKalmanGain3X2.fillByMultiplication(covariancePositionEstimationKplus1K,compositeMeasurementJacobian2x3Transpose);
	            kalmanGain3X2.fillByMultiplication(tempKalmanGain3X2,compositeInnovationCovariance2x2Inverse);
	            //pc.printf("\r\n101:calcul kalmanGain3X2 passed");
	            //get the transpose of the kalman gain
	            kalmanGain3X2Transpose.fillWithTranspose(kalmanGain3X2);
	            //update pose estimation=old pose estimation + kalman gain*compositeInnovation
	            tempPoseKplus1Kplus1.fillByMultiplication(kalmanGain3X2,compositeInnovation2x1);
	            poseKplus1Kplus1.addition(tempPoseKplus1Kplus1);
	            //pc.printf("\r\n101:calcul poseKplus1Kplus1 passed");
	 			//compute covariancePositionEstimationKplus1Kplus1=covariancePositionEstimationKplus1K-kalmanGain*compositeInnovationCovariance*KalmanGainTranspose       
				tempCovariancePositionEstimationKplus1Kplus13x2.fillByMultiplication(kalmanGain3X2,compositeInnovationCovariance2x2);
				tempCovariancePositionEstimationKplus1Kplus13x3.fillByMultiplication(tempCovariancePositionEstimationKplus1Kplus13x2,kalmanGain3X2Transpose);
	            covariancePositionEstimationKplus1Kplus1.subtraction(tempCovariancePositionEstimationKplus1Kplus13x3);
				//pc.printf("\r\n101:calcul covariancePositionEstimationKplus1Kplus1 passed");
	            break;
	        case 110:// front and right
	        	//compute compositeMeasurementJacobian
	   			compositeMeasurementJacobian2x3.data[0][0]=jacobianOfObservationFront.data[0][0];
	   			compositeMeasurementJacobian2x3.data[0][1]=jacobianOfObservationFront.data[0][1];
	   			compositeMeasurementJacobian2x3.data[0][2]=jacobianOfObservationFront.data[0][2];
	   			
	   			compositeMeasurementJacobian2x3.data[1][0]=jacobianOfObservationRight.data[0][0];
	   			compositeMeasurementJacobian2x3.data[1][1]=jacobianOfObservationRight.data[0][1];
	   			compositeMeasurementJacobian2x3.data[1][2]=jacobianOfObservationRight.data[0][2];
	   			
	   			//pc.printf("\r\n110:calcul compositeMeasurementJacobian2x3 passed");
	   			//get the compositeMeasurementJacobianTranspose
	   			compositeMeasurementJacobian2x3Transpose.fillWithTranspose(compositeMeasurementJacobian2x3);
	   			//compute compositeInnovation
	   			compositeInnovation2x1.data[0][0]=innovationFront;
	   			compositeInnovation2x1.data[1][0]=innovationRight;
	   			//compute compositeInnovationCovariance=compositeMeasurementJacobian*covariancePositionEstimationKplus1K*compositeMeasurementJacobianTranpose+R
	   			tempCompositeInnovationCovariance2x2.fillByMultiplication(compositeMeasurementJacobian2x3,covariancePositionEstimationKplus1K);
	   			compositeInnovationCovariance2x2.fillByMultiplication(tempCompositeInnovationCovariance2x2,compositeMeasurementJacobian2x3Transpose);
	   			//add the right R on the diagonal
	   			compositeInnovationCovariance2x2.data[0][0]+=R_front;
	   			compositeInnovationCovariance2x2.data[1][1]+=R_right;
	   			//pc.printf("\r\n110:calcul compositeInnovationCovariance2x2 passed");
	   			//get the inverse of the compositeInnovationCovariance
	   			compositeInnovationCovariance2x2Inverse.fillWithInverse(compositeInnovationCovariance2x2);
	   			//pc.printf("\r\n110:calcul compositeInnovationCovariance2x2Inverse passed");
	   			//compute KalmanGain=covariancePositionEstimationKplus1K*compositeMeasurementJacobianTranspose*Inverse(compositeInnovationCovariance)
	            tempKalmanGain3X2.fillByMultiplication(covariancePositionEstimationKplus1K,compositeMeasurementJacobian2x3Transpose);
	            kalmanGain3X2.fillByMultiplication(tempKalmanGain3X2,compositeInnovationCovariance2x2Inverse);
	            //pc.printf("\r\n110:calcul kalmanGain3X2 passed");
	            //get the transpose of the kalman gain
	            kalmanGain3X2Transpose.fillWithTranspose(kalmanGain3X2);
	            //update pose estimation=old pose estimation + kalman gain*compositeInnovation
	            tempPoseKplus1Kplus1.fillByMultiplication(kalmanGain3X2,compositeInnovation2x1);
	            poseKplus1Kplus1.addition(tempPoseKplus1Kplus1);
	            //pc.printf("\r\n110:calcul poseKplus1Kplus1 passed");
	 			//compute covariancePositionEstimationKplus1Kplus1=covariancePositionEstimationKplus1K-kalmanGain*compositeInnovationCovariance*KalmanGainTranspose       
				tempCovariancePositionEstimationKplus1Kplus13x2.fillByMultiplication(kalmanGain3X2,compositeInnovationCovariance2x2);
				tempCovariancePositionEstimationKplus1Kplus13x3.fillByMultiplication(tempCovariancePositionEstimationKplus1Kplus13x2,kalmanGain3X2Transpose);
	            covariancePositionEstimationKplus1Kplus1.subtraction(tempCovariancePositionEstimationKplus1Kplus13x3);
	    		//pc.printf("\r\n110:calcul covariancePositionEstimationKplus1Kplus1 passed");
	            break;
	        case 111://left front and right
				//compute compositeMeasurementJacobian
	   			compositeMeasurementJacobian3x3.data[0][0]=jacobianOfObservationLeft.data[0][0];
	   			compositeMeasurementJacobian3x3.data[0][1]=jacobianOfObservationLeft.data[0][1];
	   			compositeMeasurementJacobian3x3.data[0][2]=jacobianOfObservationLeft.data[0][2];
	   			
	   			compositeMeasurementJacobian3x3.data[1][0]=jacobianOfObservationFront.data[0][0];
	   			compositeMeasurementJacobian3x3.data[1][1]=jacobianOfObservationFront.data[0][1];
	   			compositeMeasurementJacobian3x3.data[1][2]=jacobianOfObservationFront.data[0][2];
	   			
	   			compositeMeasurementJacobian3x3.data[2][0]=jacobianOfObservationRight.data[0][0];
	   			compositeMeasurementJacobian3x3.data[2][1]=jacobianOfObservationRight.data[0][1];
	   			compositeMeasurementJacobian3x3.data[2][2]=jacobianOfObservationRight.data[0][2];
	   			
	   			//pc.printf("\r\n111:calcul compositeMeasurementJacobian3x3 passed");
	   			//get the compositeMeasurementJacobianTranspose
	   			compositeMeasurementJacobian3x3Transpose.fillWithTranspose(compositeMeasurementJacobian3x3);
	   			//compute compositeInnovation
	   			compositeInnovation3x1.data[0][0]=innovationLeft;
	   			compositeInnovation3x1.data[1][0]=innovationFront;
	   			compositeInnovation3x1.data[2][0]=innovationRight;
	   			//compute compositeInnovationCovariance=compositeMeasurementJacobian*covariancePositionEstimationKplus1K*compositeMeasurementJacobianTranpose+R
	   			tempCompositeInnovationCovariance3x3.fillByMultiplication(compositeMeasurementJacobian3x3,covariancePositionEstimationKplus1K);
	   			compositeInnovationCovariance3x3.fillByMultiplication(tempCompositeInnovationCovariance3x3,compositeMeasurementJacobian3x3Transpose);
	   			//add the right R on the diagonal
	   			compositeInnovationCovariance3x3.data[0][0]+=R_left;
	   			compositeInnovationCovariance3x3.data[1][1]+=R_front;
	   			compositeInnovationCovariance3x3.data[2][2]+=R_right;
	   			//pc.printf("\r\n111:calcul compositeInnovationCovariance3x3 passed");
	   			//get the inverse of the compositeInnovationCovariance
	   			compositeInnovationCovariance3x3Inverse.fillWithInverse(compositeInnovationCovariance3x3);
	   			//pc.printf("\r\n111:calcul compositeInnovationCovariance3x3Inverse passed");
	   			//compute KalmanGain=covariancePositionEstimationKplus1K*compositeMeasurementJacobianTranspose*Inverse(compositeInnovationCovariance)
	            tempKalmanGain3X3.fillByMultiplication(covariancePositionEstimationKplus1K,compositeMeasurementJacobian3x3Transpose);
	            kalmanGain3X3.fillByMultiplication(tempKalmanGain3X3,compositeInnovationCovariance3x3Inverse);
	            //pc.printf("\r\n111:calcul kalmanGain3X3 passed");
	            //get the transpose of the kalman gain
	            kalmanGain3X3Transpose.fillWithTranspose(kalmanGain3X3);
	            //update pose estimation=old pose estimation + kalman gain*compositeInnovation
	            tempPoseKplus1Kplus1.fillByMultiplication(kalmanGain3X3,compositeInnovation3x1);
	            poseKplus1Kplus1.addition(tempPoseKplus1Kplus1);
	            //pc.printf("\r\n111:calcul poseKplus1Kplus1 passed");
	 			//compute covariancePositionEstimationKplus1Kplus1=covariancePositionEstimationKplus1K-kalmanGain*compositeInnovationCovariance*KalmanGainTranspose       
				temp2CovariancePositionEstimationKplus1Kplus13x3.fillByMultiplication(kalmanGain3X3,compositeInnovationCovariance3x3);
				tempCovariancePositionEstimationKplus1Kplus13x3.fillByMultiplication(temp2CovariancePositionEstimationKplus1Kplus13x3,kalmanGain3X3Transpose);
	            covariancePositionEstimationKplus1Kplus1.subtraction(tempCovariancePositionEstimationKplus1Kplus13x3);
	        	//pc.printf("\r\n111:calcul covariancePositionEstimationKplus1Kplus1 passed");
	            break;
		}
		//update covariancePositionEstimationK =covariancePositionEstimationKplus1Kplus1
		this->covariancePositionEstimationK.fillByCopy(covariancePositionEstimationKplus1Kplus1);
		
		//////pc.printf("\r\nposeKplus1Kplus1 X=%f, Y=%f, theta=%f",poseKplus1Kplus1.data[0][0],poseKplus1Kplus1.data[1][0],poseKplus1Kplus1.data[2][0]);
		//update pose 
		this->xWorld=poseKplus1Kplus1.data[0][0];
		this->yWorld=poseKplus1Kplus1.data[1][0];
		this->thetaWorld=poseKplus1Kplus1.data[2][0];
		
	}
}

void MiniExplorerCoimbra::printMatrix(myMatrix mat1){
	for(int i = 0; i < mat1.nbRow; ++i) {
        for(int j = 0; j < mat1.nbColumn; ++j) {
            pc.printf("\r%f",mat1.data[i][j]);
        }
        pc.printf("\n");
    }
    pc.printf("\r\n");
}