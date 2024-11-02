#include "Coordenada.h"

// Constructor
Coordenada::Coordenada(double x, double y) : x(x), y(y) {}

// Métodos para obtener los valores de x e y
double Coordenada::obtenerX() {
    return x; 
}

double Coordenada::obtenerY() {
    return y;
}

// Sobrecarga del operador == (mantener por valor)
bool Coordenada::operator==(Coordenada other) {
    return (x == other.x && y == other.y);
}

// Sobrecarga del operador <<
std::ostream& operator<<(std::ostream& os, Coordenada coord) {
    os << "(" << coord.obtenerX() << ", " << coord.obtenerY() << ")";
    return os;
}
