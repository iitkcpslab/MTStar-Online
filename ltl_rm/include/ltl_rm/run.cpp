#include<bits/stdc++.h>
#include <stdexcept>
#include <sstream> 
#include"MTplanner_variables.h"
#include"MT_star.cpp"
using namespace std;
using namespace planner_info;
int main(int args,char **argv){
	int phi=1,no_robot=1;
	bool flag=true;
	while(flag){
	cout<<"Enter phi and no of robots followed by space"<<endl;
	cin>>phi;
	if(phi>1) flag=false;
	cin>>no_robot;
        vector<vector<pair<double, double>>> trajectory = mt_star(phi,
            no_robot,
            "/home/rotor/src/drdo_sim_ws/src/mrrtpe/ltl_rm/include/ltl_rm/");
        cout<<"Printing via Calling Function";
	for(int i=0;i<trajectory.size();i++){
	cout<<"Robot "<<i<<" Trajectory\n";
	for(int j=0;j<trajectory[i].size();j++){
		cout<<"("<<trajectory[i][j].first<<","<<trajectory[i][j].second<<") ";
	}
	cout<<"\n";
    }}
    return 0;
}
