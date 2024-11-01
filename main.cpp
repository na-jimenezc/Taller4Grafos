#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <list>
#include "Coordenada.h"
#include "Grafo.h"

void abrirArchivo(const std::string& nombreArchivo);

int main() {
    std::string nombreArchivo;

    std::cout << "¡Bienvenido a nuestro taller número 4!" << std::endl;
    std::cout << "Ingrese el nombre del archivo: ";
    std::cin >> nombreArchivo;

    abrirArchivo(nombreArchivo);

    return 0;
}

void abrirArchivo(const std::string& nombreArchivo) {
    std::ifstream archivo(nombreArchivo);
    
    if (!archivo.is_open()) {
        std::cerr << "No fue posible abrir el archivo" << std::endl;
        return;
    } else {
        std::cout << "El archivo se abrió con éxito" << std::endl;
    }

    int n; // Número de circuitos
    archivo >> n;
    std::vector<std::list<Coordenada>> circuitos(n);
    std::vector<Grafo> grafos; // Vector para almacenar los grafos

    for (int i = 0; i < n; ++i) {
        int m; // Número de agujeros para el circuito i
        archivo >> m;

        grafos.emplace_back(m); // Crear un grafo para el circuito

        for (int j = 0; j < m; ++j) {
            double x, y; // Coordenadas del agujero
            archivo >> x; // Leer el primer argumento
            archivo >> y; // Leer el segundo argumento
            circuitos[i].push_back(Coordenada(x, y)); // Almacenar las coordenadas en la lista

            // Conectar este agujero a todos los demás (en un grafo completo)
            for (int k = 0; k < j; ++k) {
                grafos[i].agregarArista(j, k);
            }
        }
    }

    // Mostrar los datos leídos y los grafos
    for (int i = 0; i < n; ++i) {
        std::cout << "Circuito " << (i + 1) << ": " << circuitos[i].size() << " agujeros\n";
        for (const Coordenada& agujero : circuitos[i]) { // Usar tipo explícito
            std::cout << "Agujero: (" << agujero.obtenerX() << ", " << agujero.obtenerY() << ")\n"; // Imprimir directamente
        }
        std::cout << "Grafo del circuito " << (i + 1) << ":\n";
        grafos[i].mostrarGrafo(); // Mostrar el grafo
    }

    archivo.close();
}
