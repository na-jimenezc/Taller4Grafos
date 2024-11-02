#ifndef COORDENADA_H
#define COORDENADA_H

#include <iostream>

class Coordenada {
private:
    double x;
    double y;

public:
    Coordenada(double x, double y);
    double obtenerX(); 
    double obtenerY(); 
    
    // Cambiar a pasar el objeto por valor
    bool operator==(Coordenada other); 
};

// Sobrecarga del operador <<
std::ostream& operator<<(std::ostream& os, Coordenada coord);

#endif // COORDENADA_H
