#include "MiniExplorerCoimbra.hpp"

int main(){ 
    
    //defaultXWorld, defaultYWorld, defaultThetaWorld, widthRealMap, heightRealMap
    //middle arena ISR 60,40
    //MiniExplorerCoimbra myRobot(20,15,0,120,80);//0,0,0 : lower left, facing right
    
    //test lab1
    //MiniExplorerCoimbra myRobot(60,40,0,120,80);//0,0,0 : lower left, facing right
    //myRobot.go_to_point(20,55);//SELECT A TARGET BETWEEN (0,0) and(widthRealMap,heightRealMap)
    //myRobot.go_to_line_first_lab(1,0,-60);//I think it works but it will always go the right
    //myRobot.go_to_line_first_lab(-1,1,0);//I think it works but it will always go the right
    //myRobot.go_to_line_first_lab(0.375,-1,17.5);
    //gain distance line
    //myRobot.go_to_point_with_angle_first_lab(100,55,-3.14/2);//SELECT A TARGET BETWEEN (0,0) and(widthRealMap,heightRealMap)
    //small gain orientation
    
    //test lab2
    //reminder for now the sonar is in debugged, if no problem,comment the right code for presentaiton
    //MiniExplorerCoimbra myRobot(60,40,0,120,80);//0,0,0 : lower left, facing right
    //myRobot.test_sonars_and_map(25);    
     //myRobot.test_procedure_lab2(10);
    
    //test lab3
    //MiniExplorerCoimbra myRobot(20,15,0,120,80);//0,0,0 : lower left, facing right
    //myRobot.try_to_reach_target(100,55);//need to adjust the constants. 
     
     //test lab4
     MiniExplorerCoimbra myRobot(20,15,0,120,80);//0,0,0 : lower left, facing right
     //myRobot.test_prediction_sonar();
     myRobot.test_procedure_lab_4(25,0);
     
    return 0;
}

