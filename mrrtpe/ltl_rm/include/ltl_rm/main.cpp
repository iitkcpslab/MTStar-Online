#include "MT_star.cpp"
#include <exception>
#include <iostream>
int main(){
    vector<vector<pair<double, double>>> trajectory;
    trajectory = mt_star("map_uc2_2.dat", "query_uc2_2.dat", -1);
    // try {
    //     trajectory = mt_star("map_uc1_2.dat", "query_uc1_2.dat", -1);
    // } catch (const std::exception& e) {
    //     std::cout << "Caught exception: " << e.what() << std::endl;
    // } catch (...) {
    //     std::cout << "Caught unknown exception" << std::endl;
    // }
    // for (int i = 0; i < trajectory.size(); i++) {
    //     cout << "Robot " << i << " Trajectory\n";
    //     int j = trajectory[i].size();
    //     if (!(trajectory[i][0].first == trajectory[i][j - 1].first and
    //         trajectory[i][0].second == trajectory[i][j - 1].second)){
    //         trajectory[i].push_back({trajectory[i][0].first, trajectory[i][0].second});
    //         }
    //         for (int j = 0; j < trajectory[i].size(); j++) {
    //             cout << "(" << trajectory[i][j].first << "," << trajectory[i][j].second << ") ";
    //         }
    //     cout << "\n";
    // }
    return 0;
}