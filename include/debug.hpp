#ifndef __DEBUG_HPP_
#define __DEBUG_HPP_


#include <iostream>


//#define DUBUGSC   // Super Contact Debug Flag
#define DEBUGSSSG // Search State Successor Generators Debug Flag
//#define DEBUGSSSGV // Search State Successor Generators Debug Flag Visualization
//#define DEBUGSSQF // Search State Quality Functions Debug Flag
//#define DEBUGSS   // Search State Debug Flag
//#define DEBUGPI   // PCL Interface Debug Flag
#define DEBUGOGF  // Optimal Grasp FrameWork Debug Flag
#define DEBUGAS   // AStar Debug Flag
#define DEBUGVIS  // PCL Visualization Debug Flag
#define DEBUGPF   // Predict Functions Debug Flag


#define DBG(STMT) std::cerr << STMT


#ifdef DEBUGVIS
  #define DBGVIS(STMT) STMT
#else
  #define DBGVIS(STMT) 0
#endif


#ifdef DEBUGSSSGV
  #define DBGVISSG(STMT) STMT
#else
  #define DBGVISSG(STMT) 0
#endif


#endif // __DEBUG_HPP_
