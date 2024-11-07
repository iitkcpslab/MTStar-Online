#include <iostream>
#include <unordered_map>
#include <vector>
namespace motion_planning::DisjointSet {
class DisjointSet {
  public:
    DisjointSet(int n)
        : parent(n)
        , rank(n, 0) {
        for (int i = 0; i < n; i++) {
            parent[i] = i;
        }
    }

    int find(int x) {
        if (parent[x] != x) {
            parent[x] = find(parent[x]);
        }
        return parent[x];
    }

    void merge(int x, int y) {
        std::cout<<"Merge called for"<<x<<","<<y<<std::endl;
        int root_x = find(x);
        int root_y = find(y);
        if (root_x != root_y) {
            if (rank[root_x] < rank[root_y]) {
                parent[root_x] = root_y;
            } else if (rank[root_y] < rank[root_x]) {
                parent[root_y] = root_x;
            } else {
                parent[root_y] = root_x;
                rank[root_x]++;
            }
        }
    }

    void update(int x, int value, bool update_group) {
        std::cout << "Update called for " << x << " with " << value<< " in group." << std::endl;
        if (update_group) {
            int root = find(x);
            for (int i = 0; i < parent.size(); i++) {
                if (find(i) == root) {
                    values[i] = value;
                }
            }
        } else {
            values[x] = value;
        }
    }

    int get(int x) {
        return values[x];
    }

  private:
    std::vector<int> parent;
    std::vector<int> rank;
    std::unordered_map<int, int> values;
};

// int main()
// {
//     int n = 5;
//     DisjointSet ds(n);
//     ds.merge(0, 1);
//     ds.merge(2, 3);
//     ds.merge(3, 4);

//     int x = 2;
//     ds.update(x, 100, true);
//     for (int i = 0; i < n; i++)
//     {
//         std::cout << "Element " << i << ": " << ds.get(i) << std::endl;
//     }
//     return 0;
// }
}  // namespace motion_planning::DisjointSet