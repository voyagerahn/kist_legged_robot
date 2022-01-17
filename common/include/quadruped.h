/*****************************************************************
 Copyright (c) 2021, Korea Institute of Science and Technology (KIST). All rights reserved.
******************************************************************/
#include <string>

using namespace std;

enum class LeggedType { 
	Aliengo,
	A1
};

enum class HighLevelType {
	Basic,
	Sport
};

// string VersionSDK();
void InitEnvironment();


// definition of each leg and joint
#define FR_ 0       // leg index
#define FL_ 1
#define RR_ 2
#define RL_ 3
#define FR_0  0      // joint index
#define FR_1  1      
#define FR_2  2
#define FL_0  3
#define FL_1  4
#define FL_2  5
#define RR_0  6
#define RR_1  7
#define RR_2  8
#define RL_0  9
#define RL_1  10
#define RL_2  11
