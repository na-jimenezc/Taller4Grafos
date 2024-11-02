    // Coordenada.h
    
    #ifndef COORDENADA_H
    #define COORDENADA_H
    #include<iostream>
    
    class Coordenada {
    private:
        double x, y;
    
    public:
        Coordenada(double x_val, double y_val);
        double obtenerX() const;
        double obtenerY() const;
        bool operator==(const Coordenada& other) const;
        friend std::ostream& operator<<(std::ostream& os, const Coordenada& coord);
    };
    
    #endif