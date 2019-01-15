#ifndef MAP_HPP
#define MAP_HPP

#include<math.h>

/*
Robot coordinate system:      World coordinate system:
      ^                                 ^
      |x                                |y
   <--R                                 O-->
    y                                    x

Screen coordinate system
   x
 O--->
y|
 v
 
how the float[2] arrays stock the position 
Start at 0,0 end top right
^
|heightIndice
|
_____>
widthIndice
*/


class Map {

public:
    float widthRealMap;
    float heightRealMap;
    int nbCellWidth;
    int nbCellHeight;
    float sizeCellWidth;
    float sizeCellHeight;
    float** cellsLogValues;
    float** initialLogValues;

    Map(float widthRealMap, float heightRealMap, int nbCellWidth, int nbCellHeight);
    
    float cell_width_coordinate_to_world(int i);

    float cell_height_coordinate_to_world(int j);
    
    float get_proba_cell(int widthIndice, int heightIndice);
    
    void fill_map_with_empty();
    
    void fill_initialLogValuesLab4();
    
    void fill_map_with_kalman_knowledge();
    
    
    
    //Updates map value
    void update_cell_value(int widthIndice,int heightIndice ,float proba);
        
    //returns the probability [0,1] that the cell is occupied from the log valAue lt
    float log_to_proba(float lt);

    //returns the log value that the cell is occupied from the probability value [0,1]
    float proba_to_log(float p);
    
    private:
    
    //fill initialLogValues with the values we already know (here the bordurs)
    void fill_initialLogValues();
    
    /*
    
    float robot_x_coordinate_in_world(float robot_x, float robot_y);

    float robot_y_coordinate_in_world(float robot_x, float robot_y);
    
    void print_final_map();

    void print_final_map_with_robot_position(float robot_x,float robot_y);

    void print_final_map_with_robot_position_and_target(float robot_x,float robot_y,float targetXWolrd, float targetYWorld);
    */
}; 

#endif

