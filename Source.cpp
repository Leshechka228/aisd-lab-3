#include <vector>
#include <memory>
#include <algorithm>
#include <iterator>
#include <functional>
#include <numeric>
#include <iostream>

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

    vector<Edge> edges(const Vertex& vertex) {
        return vector<Edge>(
            make_move_iterator(find_if(edges_.begin(), edges_.end(), [vertex](const Edge& e) {
                return e.from == vertex;
                })),
            make_move_iterator(find_if_not(edges_.begin(), edges_.end(), [vertex](const Edge& e) {
                    return e.from == vertex;
                }))
                    );
    }

    size_t order() const {
        return vertices_.size();
    }

    size_t degree(const Vertex& v) const {
        return count_if(edges_.begin(), edges_.end(), [v](const Edge& e) {
            return e.from == v;
            });
    }

    vector<Edge> shortest_path(const Vertex& from, const Vertex& to) const {
        // Реализация алгоритма Дейкстры для поиска кратчайшего пути
        // ...
        return vector<Edge>();
    }

    vector<Vertex> walk(const Vertex& start_vertex) const {
        // Реализация метода обхода графа в глубину
        // ...
        return vector<Vertex>();
    }

private:
    vector<Vertex> vertices_;
    vector<Edge> edges_;
};

int main() {
    Graph<int, double> graph; // Создаем объект графа с целочисленными вершинами и вещественными расстояниями

    // Добавляем вершины
    graph.add_vertex(1);
    graph.add_vertex(2);
    graph.add_vertex(3);

    // Добавляем рёбра
    graph.add_edge(1, 2, 1.5);
    graph.add_edge(2, 3, 2.0);
    graph.add_edge(3, 1, 2.5);

    // Проверяем наличие вершин и рёбер
    cout << "Has vertex 1: " << graph.has_vertex(1) << endl;
    cout << "Has edge from 2 to 3: " << graph.has_edge(2, 3) << endl;

    // Выводим список вершин и степень вершины
    auto vertices = graph.vertices();
    for (const auto& v : vertices) {
        cout << "Vertex: " << v << ", Degree: " << graph.degree(v) << endl;
    }

    // Удаляем вершину и ребро
    graph.remove_vertex(2);
    graph.remove_edge(3, 1);

    // Выводим обновленный список вершин и проверяем удаление
    cout << "Has vertex 2 after removal: " << graph.has_vertex(2) << endl;
    cout << "Has edge from 3 to 1 after removal: " << graph.has_edge(3, 1) << endl;

    return 0;
}