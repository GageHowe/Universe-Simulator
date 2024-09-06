#include "MatrixStack.h"

#include <stdio.h>
#include <cassert>
#include <vector>

#define GLM_FORCE_RADIANS
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>

using namespace std;

MatrixStack::MatrixStack()
{
	mstack = make_shared< stack<glm::mat4> >();
	mstack->push(glm::mat4(1.0));
}

MatrixStack::~MatrixStack()
{
}

void MatrixStack::pushMatrix()
{
	const glm::mat4 &top = mstack->top();
	mstack->push(top);
	assert(mstack->size() < 100);
}

void MatrixStack::popMatrix()
{
	assert(!mstack->empty());
	mstack->pop();
	// There should always be one matrix left.
	assert(!mstack->empty());
}

void MatrixStack::loadIdentity()
{
	glm::mat4 &top = mstack->top();
	top = glm::mat4(1.0);
}

void MatrixStack::translate(const glm::vec3 &t)
{
	glm::mat4 translationMatrix(1.0f);

	translationMatrix = glm::translate(glm::mat4(1.0f), t);


	multMatrix(translationMatrix);
}

void MatrixStack::scale(const glm::vec3 &s)
{
	glm::mat4 scaleMatrix(1.0f);

	scaleMatrix = glm::scale(glm::mat4(1.0f), s);

	multMatrix(scaleMatrix);
}

void MatrixStack::rotateX(float angle)
{
	glm::mat4 rotationMatrix(1.0f);

	rotationMatrix = glm::rotate(glm::mat4(1.0f), angle, glm::vec3(1.0f, 0.0f, 0.0f));

	multMatrix(rotationMatrix);
}

void MatrixStack::rotateY(float angle)
{
	glm::mat4 rotationMatrix(1.0f);

	rotationMatrix = glm::rotate(glm::mat4(1.0f), angle, glm::vec3(0.0f, 1.0f, 0.0f));

	multMatrix(rotationMatrix);
}

void MatrixStack::rotateZ(float angle)
{
	glm::mat4 rotationMatrix(1.0f);

	rotationMatrix = glm::rotate(glm::mat4(1.0f), angle, glm::vec3(0.0f, 0.0f, 1.0f));

	multMatrix(rotationMatrix);
}

void MatrixStack::multMatrix(glm::mat4 &matrix)
{
	glm::mat4 &top = mstack->top();

	top *= matrix;

	// Getting a pointer to the glm::mat4 matrix
	float* topArray = glm::value_ptr(top);
	float* matrixArray = glm::value_ptr(matrix);
}

void MatrixStack::Perspective(float fovy, float aspect, float near, float far)
{
	glm::mat4 projectionMatrix(0.0f);

	projectionMatrix = glm::perspective(fovy, aspect, near, far);

	multMatrix(projectionMatrix);
}

void MatrixStack::LookAt(glm::vec3 eye, glm::vec3 center, glm::vec3 up)
{
	glm::mat4 viewMatrix(1.0f);

	viewMatrix = glm::lookAt(eye, center, up);

	multMatrix(viewMatrix);
}


void MatrixStack::translate(float x, float y, float z)
{
	translate(glm::vec3(x, y, z));
}

void MatrixStack::scale(float x, float y, float z)
{
	scale(glm::vec3(x, y, z));
}

void MatrixStack::scale(float s)
{
	scale(glm::vec3(s, s, s));
}

glm::mat4 &MatrixStack::topMatrix()
{
	return mstack->top();
}

void MatrixStack::print(const glm::mat4 &mat, const char *name)
{
	if(name) {
		printf("%s = [\n", name);
	}
	for(int i = 0; i < 4; ++i) {
		for(int j = 0; j < 4; ++j) {
			// mat[j] returns the jth column
			printf("%- 5.2f ", mat[j][i]);
		}
		printf("\n");
	}
	if(name) {
		printf("];");
	}
	printf("\n");
}

void MatrixStack::print(const char *name) const
{
	print(mstack->top(), name);
}
