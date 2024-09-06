#version 120

attribute vec3 position;
attribute vec3 color;
varying vec3 fragColor;
uniform mat4 mvp;
float a=0.1;
float b=0.3;
float h=1.0;
uniform int time;

void main() {
	vec3 newp = position;
	newp.y = position.y+h*sin(time + a * position.x) + h * sin(time + b * position.y);
	gl_Position = mvp * vec4(newp, 1.0);
		fragColor = color;
}
