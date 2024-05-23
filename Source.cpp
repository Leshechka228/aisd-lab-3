#include <vector>
#include <memory>
#include <algorithm>
#include <iterator>
#include <functional>
#include <numeric>
#include <iostream>
#include <limits>
#include <unordered_map>
#include <queue>
#include <stack>

using namespace std;

template<typename Vertex, typename Distance = double>
class Graph {
public:
    struct Edge {
        Vertex from;
        Vertex to;
        Distance distance;
    };

    bool has_vertex(const Vertex& v) const {
        return find(vertices_.begin(), vertices_.end(), v) != vertices_.end();
    }

    void add_vertex(const Vertex& v) {
        if (!has_vertex(v)) {
            vertices_.push_back(v);
        }
    }

    bool remove_vertex(const Vertex& v) {
        auto it = find(vertices_.begin(), vertices_.end(), v);
        if (it != vertices_.end()) {
            vertices_.erase(it);
            edges_.erase(remove_if(edges_.begin(), edges_.end(),
                [v](const Edge& e) { return e.from == v || e.to == v; }), edges_.end());
            return true;
        }
        return false;
    }

    vector<Vertex> vertices() const {
        return vertices_;
    }

    void add_edge(const Vertex& from, const Vertex& to, const Distance& d) {
        if (has_vertex(from) && has_vertex(to)) {
            edges_.push_back({ from, to, d });
        }
    }

    bool remove_edge(const Vertex& from, const Vertex& to) {
        auto it = remove_if(edges_.begin(), edges_.end(), [from, to](const Edge& e) {
            return e.from == from && e.to == to;
            });
        if (it != edges_.end()) {
            edges_.erase(it, edges_.end());
            return true;
        }
        return false;
    }

    bool has_edge(const Vertex& from, const Vertex& to) const {
        return any_of(edges_.begin(), edges_.end(), [from, to](const Edge& e) {
            return e.from == from && e.to == to;
            });
    }

    vector<Edge> edges(const Vertex& vertex) const{
        vector<Edge> outgoing_edges;
        for (const auto& edge : edges_) {
            if (edge.from == vertex) {
                outgoing_edges.push_back(edge);
            }
        }
        return outgoing_edges;
    }

    size_t order() const {
        return vertices_.size();
    }

    size_t degree(const Vertex& v) const {
        return count_if(edges_.begin(), edges_.end(), [v](const Edge& e) {
            return e.from == v;
            });
    }

    pair<vector<Edge>, Distance> shortest_path(const Vertex& from, const Vertex& to) const {
        priority_queue<pair<Distance, Vertex>, vector<pair<Distance, Vertex>>, greater<>> pq;
        unordered_map<Vertex, Distance> distances;
        unordered_map<Vertex, Vertex> previous;

        distances[from] = 0;
        pq.push({ 0, from });

        while (!pq.empty()) {
            auto current = pq.top().second;
            pq.pop();

            if (current == to) {
                vector<Edge> path;
                Vertex p = to;
                while (previous.count(p)) {
                    path.insert(path.begin(), { previous[p], p, distances[p] });
                    p = previous[p];
                }
                Distance total_distance = distances[to];
                return make_pair(path, total_distance);
            }

            for (const auto& edge : edges(current)) {
                Vertex next = edge.to;
                Distance weight = edge.distance;
                Distance new_distance = distances[current] + weight;

                if (!distances.count(next) || new_distance < distances[next]) {
                    distances[next] = new_distance;
                    previous[next] = current;
                    pq.push({ new_distance, next });
                }
            }
        }

        return make_pair(vector<Edge>(), numeric_limits<Distance>::infinity());
    }


    vector<Vertex> walk(const Vertex& start_vertex) const {
        vector<Vertex> result;
        stack<Vertex> stk;
        unordered_map<Vertex, bool> visited;

        for (const auto& vertex : vertices_) {
            visited[vertex] = false;
        }

        stk.push(start_vertex);

        while (!stk.empty()) {
            Vertex current = stk.top();
            stk.pop();

            if (!visited[current]) {
                result.push_back(current);
                visited[current] = true;

                for (const Edge& edge : edges(current)) {
                    if (!visited[edge.to]) {
                        stk.push(edge.to);
                    }
                }
            }
        }

        return result;
    }

private:
    vector<Vertex> vertices_;
    vector<Edge> edges_;
};


int main() {

    Graph<string, double> graph;

    graph.add_vertex("A");
    graph.add_vertex("B");
    graph.add_vertex("C");
    graph.add_vertex("D");

    graph.add_edge("A", "B", 1.0);
    graph.add_edge("B", "C", 2.0);
    graph.add_edge("C", "A", 3.0);
    graph.add_edge("B", "D", 1.5);

    auto walkresult = graph.walk("B");
    cout << "depth-first traversal of the graph starting from the vertex B:" << endl;
    for (const auto& vertex : walkresult) {
        cout << vertex << " ";
    }
    cout << endl;

    Graph<string, double> g;

    g.add_vertex("A");
    g.add_vertex("B");
    g.add_vertex("C");
    g.add_vertex("D");
    g.add_vertex("E");

    g.add_edge("A", "B", 2.0);
    g.add_edge("A", "C", 4.0);
    g.add_edge("B", "C", 1.0);
    g.add_edge("B", "D", 7.0);
    g.add_edge("C", "D", 3.0);
    g.add_edge("C", "E", 2.0);
    g.add_edge("D", "E", 5.0);

    string from = "A";
    string to = "E";

    pair<vector<Graph<string, double>::Edge>, double> result = g.shortest_path(from, to);

    if (!result.first.empty()) {
        cout << "Shortest path between vertices " << from << " and " << to << ":\n";
        for (const auto& edge : result.first) {
            cout << edge.from << " -> " << edge.to << " : " << edge.distance << endl;
        }
        cout << "Total path length: " << result.second << endl;
    }
    else {
        cout << "Shortest path between vertices " << from << " and " << to << " not found.\n";
    }

    return 0;
}