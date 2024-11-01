#ifndef GRAFO_H
#define GRAFO_H

#include <vector>
#include <list>

class Grafo {
private:
    std::vector<std::list<int>> adyacencias; // Lista de adyacencia

public:
    // Constructor
    Grafo(int nodos);

    // Método para añadir una conexión (arista)
    void agregarArista(int origen, int destino);

    // Método para mostrar el grafo
    void mostrarGrafo() const;
};

#endif // GRAFO_H
