#ifndef COORDENADA_H
#define COORDENADA_H

class Coordenada {
private:
    double x; // Componente x
    double y; // Componente y

public:
    // Constructor
    Coordenada(double x, double y);

    // Métodos para obtener los valores de x e y
    double obtenerX() const;
    double obtenerY() const;
};

#endif // COORDENADA_H
