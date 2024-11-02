    #include "Coordenada.h"
    
    // Constructor
    Coordenada::Coordenada(double x, double y) : x(x), y(y) {}
    
    // Métodos para obtener los valores de x e y
    double Coordenada::obtenerX() const {
        return x;
    }
    
    double Coordenada::obtenerY() const {
        return y;
    }
    
    // Sobrecarga del operador ==
    bool Coordenada::operator==(const Coordenada& other) const {
        return (x == other.x && y == other.y);
    }

    std::ostream& operator<<(std::ostream& os, const Coordenada& coord) {
    os << "(" << coord.obtenerX() << ", " << coord.obtenerY() << ")";
    return os;
    }