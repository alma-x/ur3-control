#include "ur3_control/aruco_service.h"
#include "ros/ros.h"
#include "ros/service.h"
#include "iostream"
#include "stdio.h"
using namespace std;
int main(int argc, char** argv){
  ros::init(argc, argv, "simulation");

  double vector[100];
  int dim[10];
  vector[0]=1;
  vector[1]=2;
  vector[2]=3;
  vector[3]=4;
  vector[4]=5;
  vector[5]=6;
  vector[6]=7;
  vector[7]=8;
  vector[8]=9;
  dim[0]=3;
  dim[1]=3;
  double new_vector[100][100];
  {
  int rows,columns,dimensione,i=0,c=0,r=-1;
  rows=dim[0];
  columns=dim[1];
  dimensione=rows*columns;
  do{
    if(c%columns==0){
      c=0;
      r++;
    }
    new_vector[r][c]=vector[i];
    c++;
    i++;
  }while(i!=dimensione);
  for(r=0;r<rows;r++){
    for(c=0;c<rows;c++){
      printf("%f ",new_vector[r][c]);
    }
    printf("\n");
  }
  }


}
