//
// Created by juan-diego on 3/29/24.
//

#ifndef HOMEWORK_GRAPH_PATH_FINDING_MANAGER_H
#define HOMEWORK_GRAPH_PATH_FINDING_MANAGER_H


#include "window_manager.h"
#include "graph.h"
#include <unordered_map>
#include <set>
#include <queue>
#include <cmath>


// Este enum sirve para identificar el algoritmo que el usuario desea simular
enum Algorithm {
    None,
    Dijkstra,
    BestFirstSearch,
    AStar
};


//* --- PathFindingManager ---
//
// Esta clase sirve para realizar las simulaciones de nuestro grafo.
//
// Variables miembro
//     - path           : Contiene el camino resultante del algoritmo que se desea simular
//     - visited_edges  : Contiene todas las aristas que se visitaron en el algoritmo, notar que 'path'
//                        es un subconjunto de 'visited_edges'.
//     - window_manager : Instancia del manejador de ventana, es utilizado para dibujar cada paso del algoritmo
//     - src            : Nodo incial del que se parte en el algoritmo seleccionado
//     - dest           : Nodo al que se quiere llegar desde 'src'
//*
class PathFindingManager {
    WindowManager *window_manager;
    std::vector<sfLine> path;
    std::vector<sfLine> visited_edges;

    struct Entry {
        Node* node;
        double dist;

        bool operator < (const Entry& other) const {
            return dist < other.dist;
        }
        
        bool operator > (const Entry& other) const {
            return dist > other.dist;
        }
    };

    void dijkstra(Graph &graph) {
        std::unordered_map<Node *, Node *> parent;
        
        // Cola de prioridad para Dijkstra
        std::priority_queue<Entry, std::vector<Entry>, std::greater<Entry>> pq;
        
        // Mapa de distancias
        std::unordered_map<Node*, double> distances;
        
        // Inicializar distancias
        for (auto& [id, node] : graph.nodes) {
            distances[node] = std::numeric_limits<double>::max();
        }
        
        // Distancia del nodo origen a sí mismo es 0
        distances[src] = 0.0;
        pq.push({src, 0.0});
        
        while (!pq.empty()) {
            Entry current = pq.top();
            pq.pop();
            
            Node* u = current.node;
            double dist = current.dist;
            
            // Si ya procesamos este nodo con una distancia menor, continuamos
            if (dist > distances[u]) continue;
            
            // Marcar el nodo como visitado (cambiar color)
            u->color = sf::Color::Yellow;
            u->radius = 2.0f;
            
            // Si llegamos al destino, terminamos
            if (u == dest) break;
            
            // Procesar todos los vecinos
            for (Edge* edge : u->edges) {
                Node* v = (edge->src == u) ? edge->dest : edge->src;
                
                // Calcular nueva distancia
                double new_dist = distances[u] + edge->length;
                
                if (new_dist < distances[v]) {
                    distances[v] = new_dist;
                    parent[v] = u;
                    pq.push({v, new_dist});
                    
                    // Marcar la arista como visitada
                    edge->color = sf::Color::Blue;
                    edge->thickness = 1.5f;
                    visited_edges.emplace_back(edge->src->coord, edge->dest->coord, sf::Color::Blue, 1.5f);
                }
            }
            
            // Renderizar el progreso
            render();
        }

        set_final_path(parent);
    }

    void best_first_search(Graph &graph) {
        std::unordered_map<Node *, Node *> parent;
        
        // Cola de prioridad para Best First Search (ordenada por heurística)
        std::priority_queue<Entry, std::vector<Entry>, std::greater<Entry>> pq;
        
        // Set para nodos visitados
        std::set<Node*> visited;
        
        // Inicializar con el nodo origen
        pq.push({src, heuristic(src)});
        parent[src] = nullptr;
        
        while (!pq.empty()) {
            Entry current = pq.top();
            pq.pop();
            
            Node* u = current.node;
            
            // Si ya visitamos este nodo, continuamos
            if (visited.find(u) != visited.end()) continue;
            
            // Marcar como visitado
            visited.insert(u);
            u->color = sf::Color::Yellow;
            u->radius = 2.0f;
            
            // Si llegamos al destino, terminamos
            if (u == dest) break;
            
            // Procesar todos los vecinos
            for (Edge* edge : u->edges) {
                Node* v = (edge->src == u) ? edge->dest : edge->src;
                
                // Si no hemos visitado este nodo
                if (visited.find(v) == visited.end()) {
                    parent[v] = u;
                    pq.push({v, heuristic(v)});
                    
                    // Marcar la arista como visitada
                    edge->color = sf::Color::Blue;
                    edge->thickness = 1.5f;
                    visited_edges.emplace_back(edge->src->coord, edge->dest->coord, sf::Color::Blue, 1.5f);
                }
            }
            
            // Renderizar el progreso
            render();
        }

        set_final_path(parent);
    }

    void a_star(Graph &graph) {
        std::unordered_map<Node *, Node *> parent;
        
        // Cola de prioridad para A* (ordenada por f(n) = g(n) + h(n))
        std::priority_queue<Entry, std::vector<Entry>, std::greater<Entry>> pq;
        
        // Mapa de costos reales g(n)
        std::unordered_map<Node*, double> g_cost;
        
        // Inicializar costos
        for (auto& [id, node] : graph.nodes) {
            g_cost[node] = std::numeric_limits<double>::max();
        }
        
        // Costo del nodo origen a sí mismo es 0
        g_cost[src] = 0.0;
        pq.push({src, g_cost[src] + heuristic(src)});
        parent[src] = nullptr;
        
        while (!pq.empty()) {
            Entry current = pq.top();
            pq.pop();
            
            Node* u = current.node;
            double f_cost = current.dist;
            
            // Si ya procesamos este nodo con un costo menor, continuamos
            if (f_cost > g_cost[u] + heuristic(u)) continue;
            
            // Marcar el nodo como visitado
            u->color = sf::Color::Yellow;
            u->radius = 2.0f;
            
            // Si llegamos al destino, terminamos
            if (u == dest) break;
            
            // Procesar todos los vecinos
            for (Edge* edge : u->edges) {
                Node* v = (edge->src == u) ? edge->dest : edge->src;
                
                // Calcular nuevo costo real
                double new_g_cost = g_cost[u] + edge->length;
                
                if (new_g_cost < g_cost[v]) {
                    g_cost[v] = new_g_cost;
                    parent[v] = u;
                    pq.push({v, new_g_cost + heuristic(v)});
                    
                    // Marcar la arista como visitada
                    edge->color = sf::Color::Blue;
                    edge->thickness = 1.5f;
                    visited_edges.emplace_back(edge->src->coord, edge->dest->coord, sf::Color::Blue, 1.5f);
                }
            }
            
            // Renderizar el progreso
            render();
        }

        set_final_path(parent);
    }

    // Función heurística: distancia euclidiana al nodo destino
    double heuristic(Node* node) {
        if (dest == nullptr) return 0.0;
        double dx = node->coord.x - dest->coord.x;
        double dy = node->coord.y - dest->coord.y;
        return std::sqrt(dx*dx + dy*dy);
    }

    //* --- render ---
    // En cada iteración de los algoritmos esta función es llamada para dibujar los cambios en el 'window_manager'
    void render() {
        sf::sleep(sf::milliseconds(10));
        
        // Limpiar la ventana
        window_manager->clear();
        
        // Dibujar el grafo
        // Nota: Esto debería llamarse desde la GUI, pero por ahora lo dejamos así
    }

    //* --- set_final_path ---
    // Esta función se usa para asignarle un valor a 'this->path' al final de la simulación del algoritmo.
    // 'parent' es un std::unordered_map que recibe un puntero a un vértice y devuelve el vértice anterior a el,
    // formando así el 'path'.
    //
    // ej.
    //     parent(a): b
    //     parent(b): c
    //     parent(c): d
    //     parent(d): NULL
    //
    // Luego, this->path = [Line(a.coord, b.coord), Line(b.coord, c.coord), Line(c.coord, d.coord)]
    //
    // Este path será utilizado para hacer el 'draw()' del 'path' entre 'src' y 'dest'.
    //*
    void set_final_path(std::unordered_map<Node *, Node *> &parent) {
        Node* current = dest;
        
        // Limpiar el path anterior
        path.clear();
        
        // Reconstruir el camino desde dest hasta src
        while (current != nullptr && parent.find(current) != parent.end()) {
            Node* prev = parent[current];
            if (prev != nullptr) {
                // Crear una línea desde prev hasta current
                path.emplace_back(prev->coord, current->coord, sf::Color::Red, 3.0f);
            }
            current = prev;
        }
    }

public:
    Node *src = nullptr;
    Node *dest = nullptr;

    explicit PathFindingManager(WindowManager *window_manager) : window_manager(window_manager) {}

    void exec(Graph &graph, Algorithm algorithm) {
        if (src == nullptr || dest == nullptr) {
            return;
        }

        // Limpiar el estado anterior
        path.clear();
        visited_edges.clear();
        
        // Ejecutar el algoritmo seleccionado
        switch (algorithm) {
            case Dijkstra:
                dijkstra(graph);
                break;
            case BestFirstSearch:
                best_first_search(graph);
                break;
            case AStar:
                a_star(graph);
                break;
            default:
                break;
        }
    }

    void reset() {
        path.clear();
        visited_edges.clear();

        if (src) {
            src->reset();
            src = nullptr;
            // ^^^ Pierde la referencia luego de restaurarlo a sus valores por defecto
        }
        if (dest) {
            dest->reset();
            dest = nullptr;
            // ^^^ Pierde la referencia luego de restaurarlo a sus valores por defecto
        }
    }

    void draw(bool draw_extra_lines) {
        // Dibujar todas las aristas visitadas
        if (draw_extra_lines) {
            for (sfLine &line: visited_edges) {
                line.draw(window_manager->get_window(), sf::RenderStates::Default);
            }
        }

        // Dibujar el camino resultante entre 'str' y 'dest'
        for (sfLine &line: path) {
            line.draw(window_manager->get_window(), sf::RenderStates::Default);
        }

        // Dibujar el nodo inicial
        if (src != nullptr) {
            src->draw(window_manager->get_window());
        }

        // Dibujar el nodo final
        if (dest != nullptr) {
            dest->draw(window_manager->get_window());
        }
    }
};


#endif //HOMEWORK_GRAPH_PATH_FINDING_MANAGER_H
