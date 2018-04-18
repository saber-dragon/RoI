#ifndef ROI_HPP
#define ROI_HPP
#include <vector>
#include <cassert>
#include <limits>

using namespace std;

namespace saber {
    using Graph = vector<vector<bool> >;
    using ReachabilityMatrix = vector<vector<bool> >;

    ReachabilityMatrix FloydWarshall(const Graph& g){
        ReachabilityMatrix reachable(g);
        size_t n = g.size();

        for(size_t k = 0;k < n;++ k) reachable[k][k] = true;

        for (size_t k = 0;k < n;++ k){
            for (size_t i = 0;i < n;++ i){
                for (size_t j = 0;j < n;++ j){
                    if (reachable[i][k] && reachable[k][j])
                        reachable[i][j] = true;
                }
            }
        }
        return reachable;
    }

    inline void TraceBack(size_t s, size_k v,
                          vector<size_t>& parent,
                          vector<bool>& inRoI){
        while (v != numeric_limits<size_t>::max() && v != s && !inRoI[v]){
            inRoI[v] = true;
            v = parent[v];
        }
    }
    void DfsVisit(const Graph& g, size_t s, size_t c, const set<size_t>& ts,
                  vector<bool>& visited,
                  vector<size_t>& parent, vector<bool>& inRoI){
        if (ts.count(c) > 0) {
            inRoI[c] = true;
            auto v = parent[c];
            TraceBack(s, v, parent, inRoI);
            return;
        }
        for (size_t k = 0;k < g.size();++ k){
            if (k != c && g[c][k]){
                if (!visited[k]){
                    parent[k] = c;
                    visited[k] = true;
                    DfsVisit(g, s, k, ts, visited, parent, inRoI);
                } else {
                    if (inRoI[k]){
                        TraceBack(s, c, parent, inRoI);
                    }
                }
            }
        }

    }
    set<size_t> RoIByDFS(const Graph& g, size_t s, const set<size_t>& ts){
          size_t n = g.size();
          vector<bool>& visited(n, false);
          vector<size_t> parent(n, numeric_limits<size_t>::max());
          vector<bool> inRoI(n, false);
          DfsVisit(g, s, s, ts, visited, parent, inRoI);

          set<size_t> RoI;
          for (size_t k = 0;k < n;++ k){
              if (inRoI[k]) RoI.insert(k);
          }
          return RoI;
    }
    set<size_t> RoIByDefinition(const Graph& g, size_t s, const set<size_t>& ts){
        ReachabilityMatrix Reachable = FloydWarshall(g);
        set<size_t> RoI;
        for (const auto& u: ts){
            for (size_t n = 0;n < g.size();++ n){
                if (Reachable[s][n] && Reachable[n][u]){
                    RoI.insert(n);
                }
            }
        }
        return RoI;
    }

}
#endif // ROI_HPP
