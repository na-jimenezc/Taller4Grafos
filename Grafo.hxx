#include "Grafo.h"
#include <iostream>

// Constructor
Grafo::Grafo(int nodos) : adyacencias(nodos) {}

// Método para añadir una conexión (arista)
void Grafo::agregarArista(int origen, int destino) {
    adyacencias[origen].push_back(destino);
    adyacencias[destino].push_back(origen); // Grafo no dirigido
}

// Método para mostrar el grafo
void Grafo::mostrarGrafo() const {
    for (int i = 0; i < adyacencias.size(); ++i) {
        std::cout << "Agujero " << i << ": ";
        for (int vecino : adyacencias[i]) {
            std::cout << vecino << " ";
        }
        std::cout << std::endl;
    }
}
