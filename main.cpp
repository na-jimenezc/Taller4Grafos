     #include <iostream>
    #include <fstream>
    #include <string>
    #include <vector>
    #include <list>
    #include <cmath> 
    #include "GrafoML.h"
    #include "Coordenada.h"
    
    void recuperarDatos(const std::string& nombreArchivoEntrada, std::vector<GrafoML<Coordenada, double>>& grafos);
    double calcularDistanciaEuclidiana(const Coordenada& c1, const Coordenada& c2);
    
    int main(int argc, char* argv[]) {
        if (argc != 3) {
            std::cerr << "Uso: " << argv[0] << " archivo_entrada.txt archivo_salida.txt" << std::endl;
            return 1;
        }
    
        std::string nombreArchivoEntrada = argv[1];
        std::string nombreArchivoSalida = argv[2];
    
        // Vector de grafos para almacenar los datos
        std::vector<GrafoML<Coordenada, double>> grafos;
    
        recuperarDatos(nombreArchivoEntrada, grafos);
        
        for(int=0; i<grafos.size();i++){
            grafos[i].imprimir();
        }
    
        return 0;
    }
    
    void recuperarDatos(const std::string& nombreArchivoEntrada, std::vector<GrafoML<Coordenada, double>>& grafos) {
        std::ifstream archivoEntrada(nombreArchivoEntrada);
    
        if (!archivoEntrada.is_open()) {
            std::cerr << "No fue posible abrir el archivo de entrada" << std::endl;
            return;
        } else {
            std::cout << "El archivo de entrada se abrió con éxito" << std::endl;
        }
    
        int numCircuitos;
        archivoEntrada >> numCircuitos;
        grafos.resize(numCircuitos);
    
        for (int i = 0; i < numCircuitos; ++i) {
            int numAgujeros;
            archivoEntrada >> numAgujeros;
    
            GrafoML<Coordenada, double> grafo;
    
            // Vector temporal para almacenar coordenadas del circuito actual
            std::vector<Coordenada> agujeros;
    
            for (int j = 0; j < numAgujeros; ++j) {
                double x, y;
                archivoEntrada >> x >> y;
                Coordenada coord(x, y);
    
                agujeros.push_back(coord);
                grafo.insertarVertice(coord);  // Insertar Coordenada como vértice
            }
    
            // Crear las aristas para el grafo
            for (int k = 0; k < numAgujeros; ++k) {
                for (int l = k + 1; l < numAgujeros; ++l) {
                    double costo = calcularDistanciaEuclidiana(agujeros[k], agujeros[l]);
                    grafo.insertarAristaNoDirigida(agujeros[k], agujeros[l], costo);  // Usar coordenadas como vértices
                }
            }
    
    
            grafos[i] = grafo;
        }
    
        archivoEntrada.close();
        std::cout << "Datos cargados exitosamente." << std::endl;
    }
    
    double calcularDistanciaEuclidiana(const Coordenada& c1, const Coordenada& c2) {
        double deltaX = c1.obtenerX() - c2.obtenerX();
        double deltaY = c1.obtenerY() - c2.obtenerY();
        return std::sqrt(deltaX * deltaX + deltaY * deltaY);
    }