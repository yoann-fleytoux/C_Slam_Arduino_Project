#include "Map.hpp"

Map::Map(float widthRealMap, float heightRealMap, int nbCellWidth, int nbCellHeight){
	this->widthRealMap=widthRealMap;
	this->heightRealMap=heightRealMap;
	this->nbCellWidth=nbCellWidth;
	this->nbCellHeight=nbCellHeight;
	this->sizeCellWidth=widthRealMap/(float)nbCellWidth;
	this->sizeCellHeight=heightRealMap/(float)nbCellHeight;
	
	this->cellsLogValues= new float*[nbCellWidth];
	for(int i = 0; i < nbCellWidth; ++i)
    	this->cellsLogValues[i] = new float[nbCellHeight];
    	
	this->initialLogValues= new float*[nbCellWidth];
	for(int i = 0; i < nbCellWidth; ++i)
    	this->initialLogValues[i] = new float[nbCellHeight];
    	
	this->fill_initialLogValues();
	this->fill_map_with_empty();
}

//fill initialLogValues with the values we already know (here the bordurs)
void Map::fill_initialLogValues(){
    //Fill map, we know the border are occupied
    for (int i = 0; i<this->nbCellWidth; i++) {
        for (int j = 0; j<this->nbCellHeight; j++) {
            if(j==0 || j==this->nbCellHeight-1 || i==0 || i==this->nbCellWidth-1)
                this->initialLogValues[i][j] = this->proba_to_log(1);
            else
                this->initialLogValues[i][j] = this->proba_to_log(0.5);
        }
    }
}


//fill initialLogValues with the values we already know (here the bordurs)
void Map::fill_initialLogValuesLab4(){
    //Fill map, we know the border are occupied
    for (int i = 0; i<this->nbCellWidth; i++) {
        for (int j = 0; j<this->nbCellHeight; j++) {
            if(j==0 || j==this->nbCellHeight-1 || i==0 || i==this->nbCellWidth-1)
                this->initialLogValues[i][j] = this->proba_to_log(1);
            else
                this->initialLogValues[i][j] = this->proba_to_log(0.5);
        }
    }
}

void Map::fill_map_with_empty(){
	for (int i = 0; i<this->nbCellWidth; i++) {
        for (int j = 0; j<this->nbCellHeight; j++) {
            this->cellsLogValues[i][j] = this->proba_to_log(0.5);
        }
    }
}

//returns the probability [0,1] that the cell is occupied from the log valAue lt
float Map::log_to_proba(float lt){
    return 1-1/(1+exp(lt));
}

void Map::update_cell_value(int widthIndice,int heightIndice ,float proba){
    this->cellsLogValues[widthIndice][heightIndice]=this->cellsLogValues[widthIndice][heightIndice]+this->proba_to_log(proba)+this->initialLogValues[widthIndice][heightIndice];//map is filled as map[0][0] get the data for the point closest to the origin
}

float Map::cell_width_coordinate_to_world(int i){
	return this->sizeCellWidth/2+i*this->sizeCellWidth;
}

float Map::cell_height_coordinate_to_world(int j){
	return this->sizeCellHeight/2+j*this->sizeCellHeight;
}

float Map::get_proba_cell(int widthIndice, int heightIndice){
	return  this->log_to_proba(this->cellsLogValues[widthIndice][heightIndice]);
}

//returns the log value that the cell is occupied from the probability value [0,1]
float Map::proba_to_log(float p){
    return log(p/(1-p));
}

void Map::fill_map_with_kalman_knowledge(){
	int lol1;
	int lol2;
	for (int i = 0; i<this->nbCellWidth; i++) {
        for (int j = 0; j<this->nbCellHeight; j++) {
            if(j==0 || j==this->nbCellHeight-1 || i==0 || i==this->nbCellWidth-1)
                this->cellsLogValues[i][j] = this->proba_to_log(1);
            else{
            	//rectangle in center 30 width, 20 height
            	if(((this->cell_width_coordinate_to_world(i) >= this->widthRealMap/2-15) && (this->cell_width_coordinate_to_world(i) <= this->widthRealMap/2+15)) && ((this->cell_height_coordinate_to_world(j) >= this->heightRealMap/2-10) && (this->cell_height_coordinate_to_world(j) <= this->heightRealMap/2+10))  ){
            		this->cellsLogValues[i][j] = this->proba_to_log(1);
            	}else
            		this->cellsLogValues[i][j] = this->proba_to_log(0.5);
            }
            
        }
    }
}

/*

float Map::robot_x_coordinate_in_world(float robot_x, float robot_y){
    return this->nbCellWidth*this->sizeCellWidth-robot_y;
}

float Map::robot_y_coordinate_in_world(float robot_x, float robot_y){
    return robot_x;
}


void MiniExplorerCoimbra::print_final_map() {
    float currProba;
    pc.printf("\n\r");
    for (int y = this->nbCellHeight -1; y>-1; y--) {
        for (int x= 0; x<this->nbCellWidth; x++) {
                currProba=this->log_to_proba(this->cellsLogValues[x][y]);
            if ( currProba < 0.5) {
                pc.printf("   ");
            } else {
                if(currProba==0.5)
                    pc.printf(" . ");
                else
                    pc.printf(" X ");
            }
        }
        pc.printf("\n\r");
    }
}


void Map::print_final_map_with_robot_position(float robot_x,float robot_y) {
    float currProba;
    float Xrobot=this->robot_x_coordinate_in_world(robot_x,robot_y);
    float Yrobot=this->robot_y_coordinate_in_world(robot_x,robot_y);
    
    float heightIndiceInOrthonormal;
    float widthIndiceInOrthonormal;
    
    float widthMalus=-(3*sizeCellWidth/2);
    float widthBonus=sizeCellWidth/2;
    
    float heightMalus=-(3*sizeCellHeight/2);
    float heightBonus=sizeCellHeight/2;

    pc.printf("\n\r");
    for (int y = this->nbCellHeight -1; y>-1; y--) {
        for (int x= 0; x<this->nbCellWidth; x++) {
            heightIndiceInOrthonormal=this->cell_height_coordinate_to_world(y);
            widthIndiceInOrthonormal=this->cell_width_coordinate_to_world(x);
            if(Yrobot >= (heightIndiceInOrthonormal+heightMalus) && Yrobot <= (heightIndiceInOrthonormal+heightBonus) && Xrobot >= (widthIndiceInOrthonormal+widthMalus) && Xrobot <= (widthIndiceInOrthonormal+widthBonus))                    
                pc.printf(" R ");
            else{
                currProba=this->log_to_proba(this->cellsLogValues[x][y]);
                if ( currProba < 0.5)
                    pc.printf("   ");
                else{
                    if(currProba==0.5)
                        pc.printf(" . ");
                    else
                        pc.printf(" X ");
                } 
            }
        }
        pc.printf("\n\r");
    }
}

void Map::print_final_map_with_robot_position_and_target(float robot_x,float robot_y,float targetXWorld, float targetYWorld) {
    float currProba;
    float Xrobot=this->robot_x_coordinate_in_world(robot_x,robot_y);
    float Yrobot=this->robot_y_coordinate_in_world(robot_x,robot_y);
    
    float heightIndiceInOrthonormal;
    float widthIndiceInOrthonormal;
    
    float widthMalus=-(3*sizeCellWidth/2);
    float widthBonus=sizeCellWidth/2;
    
    float heightMalus=-(3*sizeCellHeight/2);
    float heightBonus=sizeCellHeight/2;

    pc.printf("\n\r");
    for (int y = this->nbCellHeight -1; y>-1; y--) {
        for (int x= 0; x<this->nbCellWidth; x++) {
            heightIndiceInOrthonormal=this->cell_height_coordinate_to_world(y);
            widthIndiceInOrthonormal=this->cell_width_coordinate_to_world(x);
            if(Yrobot >= (heightIndiceInOrthonormal+heightMalus) && Yrobot <= (heightIndiceInOrthonormal+heightBonus) && Xrobot >= (widthIndiceInOrthonormal+widthMalus) && Xrobot <= (widthIndiceInOrthonormal+widthBonus))
                pc.printf(" R ");
            else{
                if(targetYWorld >= (heightIndiceInOrthonormal+heightMalus) && targetYWorld <= (heightIndiceInOrthonormal+heightBonus) && targetXWorld >= (widthIndiceInOrthonormal+widthMalus) && targetXWorld <= (widthIndiceInOrthonormal+widthBonus))                    
                    pc.printf(" T ");
                else{
                    currProba=this->log_to_proba(this->cellsLogValues[x][y]);
                    if ( currProba < 0.5)
                        pc.printf("   ");
                    else{
                        if(currProba==0.5)
                            pc.printf(" . ");
                        else
                            pc.printf(" X ");
                    } 
                }
            }
        }
        pc.printf("\n\r");
    }
}
*/