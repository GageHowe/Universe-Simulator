#define _USE_MATH_DEFINES
#include <GL\glew.h>
#include <vector>
#include <cmath>

#include <math.h>

std::vector<GLfloat> vertices;
std::vector<GLuint> indices;

void createSphere(float radius, unsigned int rings, unsigned int sectors)
{
    float const R = 1.0f/(float)(rings-1);
    float const S = 1.0f/(float)(sectors-1);

    for(unsigned int r = 0; r < rings; ++r) {
        for(unsigned int s = 0; s < sectors; ++s) {
            float const y = sin( -M_PI_2 + M_PI * r * R );
            float const x = cos(2*M_PI * s * S) * sin( M_PI * r * R );
            float const z = sin(2*M_PI * s * S) * sin( M_PI * r * R );

            vertices.push_back(x * radius);
            vertices.push_back(y * radius);
            vertices.push_back(z * radius);

            vertices.push_back(x);
            vertices.push_back(y);
            vertices.push_back(z);

            vertices.push_back(s*S);
            vertices.push_back(r*R);
        }
    }

    for(unsigned int r = 0; r < rings-1; ++r) {
        for(unsigned int s = 0; s < sectors-1; ++s) {
            indices.push_back(r * sectors + s);
            indices.push_back(r * sectors + (s+1));
            indices.push_back((r+1) * sectors + (s+1));

            indices.push_back(r * sectors + s);
            indices.push_back((r+1) * sectors + (s+1));
            indices.push_back((r+1) * sectors + s);
        }
    }
}

// Usage
//float radius = 1.0f;
//unsigned int rings = 20;
//unsigned int sectors = 20;
//createSphere(radius, rings, sectors);