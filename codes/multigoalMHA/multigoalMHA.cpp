#include <iostream>
#include <string>

using namespace std;

#include <sbpl/headers.h>
#include <algorithm>
#include <cassert>
#include <cmath>
#include <limits>

#include <sbpl/discrete_space_information/environment.h>
#include <sbpl/utils/heap.h>
#include <sbpl/utils/key.h>
#include <sbpl/utils/list.h>
#include <sbpl/headers.h>
#include <sbpl/heuristics/embedded_heuristic.h>


#include <sbpl/planners/mhaplanner.h> //doesn't work without the "" 

#define DEBUG_P 1

struct envState{

float x; //x location of the state in meters
float y; //y location of that state in meters
float th; //theta at that state

};




//costmap2d

//generate multiple heuristics and call sbpl planner for mha*

//initialise the environment, throw an exception if it doesn't initialise

void initializeEnv( EnvironmentNAVXYTHETALAT& env, vector<sbpl_2Dpt_t>& perimeter1, //vector<sbpl_2Dpt_t>& perimeter2,
                    char* envCfgFilename, char* motPrimFilename)

{
    if (!env.InitializeEnv(envCfgFilename, perimeter1, //perimeter2, 
                        motPrimFilename))
        {
        printf("ERROR: InitializeEnv failed\n");
        throw SBPL_Exception();
        }
}

void createFootprint(vector<sbpl_2Dpt_t>& perimeter, double length, double width)
{
    sbpl_2Dpt_t pt_m;
    pt_m.x = -(length/2);
    pt_m.y = -(width/2);
    perimeter.push_back(pt_m);
    pt_m.x = (length/2);
    pt_m.y = -(width/2);
    perimeter.push_back(pt_m);
    pt_m.x = (length/2);
    pt_m.y = (width/2);
    perimeter.push_back(pt_m);
    pt_m.x = -(length/2);
    pt_m.y = (width/2);
    perimeter.push_back(pt_m);
}

//set a start goal- starting and finishing x,y and theta
//also mention a start ID and goal id for the lattice
void setEnvStartGoal(EnvironmentNAVXYTHETALAT& env, 
                     double start_x, double start_y, double start_theta,
                     double goal_x, double goal_y, double goal_theta, 
                     int& start_id, int& goal_id)
{
    start_id = env.SetStart(start_x, start_y, start_theta); 
    goal_id = env.SetGoal(goal_x, goal_y, goal_theta);
}

void initializePlanner(SBPLPlanner*& planner, 
                       EnvironmentNAVXYTHETALAT& env,
                       int start_id, int goal_id,
                       double initialEpsilon, 
                       bool bsearchuntilfirstsolution,Heuristic* hanchor, Heuristic* heurs,int hcount){
    
    bool bsearch = false;
    MHAPlanner *mha_planner = new MHAPlanner(&env,hanchor,&heurs,hcount);// intialise correctly
    mha_planner->set_initial_mha_eps(5);

    planner = mha_planner;
    //planner = new ARAPlanner(&env,bsearchuntilfirstsolution);// intialise correctly
 
    // set planner properties
    if (planner->set_start(start_id) == 0) {
        printf("ERROR: failed to set start state\n");
        throw new SBPL_Exception();
    }
    if (planner->set_goal(goal_id) == 0) {
        printf("ERROR: failed to set goal state\n");
        throw new SBPL_Exception();
    }
    planner->set_initialsolution_eps(initialEpsilon);
    //planner->set_initial_mha_eps(3);
    planner->set_search_mode(bsearchuntilfirstsolution);
}



 
int runPlanner(SBPLPlanner* planner, int allocated_time_secs, 
               vector<int>&solution_stateIDs){
    int bRet = planner->replan(allocated_time_secs, &solution_stateIDs);
 
    if (bRet) 
        printf("Solution is found\n");
    else 
        printf("Solution does not exist\n");
    return bRet;
}
 
void writeSolution(EnvironmentNAVXYTHETALAT& env, vector<int> solution_stateIDs,
                   const char* filename){
    std::string discrete_filename(std::string(filename) + std::string(".discrete"));
    FILE* fSol_discrete = fopen(discrete_filename.c_str(), "w");
    FILE* fSol = fopen(filename, "w");
    if (fSol == NULL) {
        printf("ERROR: could not open solution file\n");
        throw SBPL_Exception();
    }
 
    // write the discrete solution to file
    for (size_t i = 0; i < solution_stateIDs.size(); i++) {
        int x, y, theta;
        env.GetCoordFromState(solution_stateIDs[i], x, y, theta);
        double cont_x, cont_y, cont_theta;
        cont_x = DISCXY2CONT(x, 0.1);
        cont_y = DISCXY2CONT(y, 0.1);
        cont_theta = DiscTheta2Cont(theta, 16);
        fprintf(fSol_discrete, "%d %d %d\n", x, y, theta);
    }
    fclose(fSol_discrete);
 
    // write the continuous solution to file
    vector<sbpl_xy_theta_pt_t> xythetaPath;
    env.ConvertStateIDPathintoXYThetaPath(&solution_stateIDs, &xythetaPath);
    for (unsigned int i = 0; i < xythetaPath.size(); i++) {
        fprintf(fSol, "%.3f %.3f %.3f\n", xythetaPath.at(i).x, 
                                          xythetaPath.at(i).y, 
                                          xythetaPath.at(i).theta);
    }
    fclose(fSol);
}
 
void planxythetalat(EnvironmentNAVXYTHETALAT& env1)

{


    
    // specify a start and goal state
    int start_id, goal_id;
    
    std::vector<envState> start,goal;

    //TODO: Populate start and goal states

    Heuristic* hanchor = new EmbeddedHeuristic(&env1);
    Heuristic* h1 = new EmbeddedHeuristic(&env1);

    SBPLPlanner* planner = NULL;
    double initialEpsilon = 5.0;
    bool bsearchuntilfirstsolution = false;
    vector<int> solution_stateIDs;
    double allocated_time_secs = 30.0; // in seconds

  for (int i = 0; i<v.size(); i++)
    {
        
        setEnvStartGoal(env1, start[i].x, start[i].y, start[i].th, goal[i].x, goal[i].y, goal[i].th, 
                            start_id, goal_id);
 
        initializePlanner(planner, env1, start_id, goal_id, initialEpsilon, 
                      bsearchuntilfirstsolution, hanchor,h1,1);
        runPlanner(planner, allocated_time_secs, solution_stateIDs);
    
        
        if DEBUG_P
        {
            env1.PrintTimeStat(stdout);
            double trun=planner->get_initial_eps_planning_time();   
            int pathSize=solution_stateIDs.size();
            int expansions= planner->get_n_expands();

            cout<<"Expansions: "<<expansions<<endl;
            cout<<"Time to Run: "<<trun<<endl;;
            cout<<"Path length: "<<pathSize<<endl;

        }

        // write out solutions
        std::string filename("sol.txt");
        writeSolution(env1, solution_stateIDs, filename.c_str());
     
        delete planner;
    }
}
 

 
int main(int argc, char *argv[])
{   
    double l1=0.01,w1=0.01,l2=0.02,w2=0.02;
    // set the perimeter of the robot   
    vector<sbpl_2Dpt_t> perimeter1, perimeter2;
    createFootprint(perimeter1,l1,w1); //Passed perimeter by reference
    createFootprint(perimeter2,l2,w2); //Passed perimeter by reference

    // initialize an environment
    EnvironmentNAVXYTHETALAT env1;
    
    initializeEnv(env1, perimeter1,argv[1],argv[2]);
    
    planxythetalat(, argv[2]);
}
