#include <chrono>
#include <fstream>
#include <iostream>
#include <stdlib.h>
#include <time.h>
#include <vector>

#include "GLFW\glfw3.h"
#pragma comment(lib, "opengl32.lib")
#pragma comment(lib, "GLFW/glfw3dll.lib")

#include <glm\glm.hpp>

#include <Windows.h>
#undef max

using namespace std;
using namespace glm;

const int windowW = 600;
const int windowH = 600;

const int mapW = 128;
const int mapH = 128;

const float dt = 1.f / 60;

float PI = 3.14159;

struct particle
{
	vec2 pos;
	vec2 vel;
	float density;
	float pressure;
};
vector<particle> parts;

vector<int> grid[mapW][mapH];

int walls[mapW][mapH];

GLFWwindow* window;

float k = 10;
float k_near = 20;
float rho_0 = 2;
float h = 20;

float nrand()
{
	return (float)rand() / RAND_MAX;
}

// W(r) = 15 / (PI * h^6) * (h - r)^3
float Wspiky(vec2 r)
{
	float coef = 15 / (PI * pow(h, 6));

	float len_r = r.length();
	
	if (len_r <= h)
		return coef * pow(h - len_r, 3);
	return 0;
}
vec2 del_Wspiky(vec2 r)
{
	float len_r = r.length();
	len_r = max(0.01f, len_r);
	
	float coef = -45 * pow(h - len_r, 2) / (PI * pow(h, 6) * len_r);

	if (len_r <= h)
		return r * coef;
	return vec2();
}

// W(r) = 315 / (64 * PI * h^9) * (h^2 - r^2)^3
float Wpoly6(vec2 r)
{
	float coef = 315 / (64 * PI * pow(h, 9));

	float len_r_2 = dot(r, r);

	if (len_r_2 <= h * h)
		return coef * pow(h * h - len_r_2, 3);
	return 0;
}
vec2 del_Wpoly6(vec2 r)
{
	float len_r_2 = dot(r, r);

	float coef = -945 * pow(h*h - len_r_2, 2) / (32 * PI * pow(h, 9));

	if (len_r_2 <= h * h)
		return r * coef;
	return vec2();
}

void setup()
{
	float dx = 2;
	for (float x = 0; x < mapW / 4; x += dx)
	{
		for (float y = 0; y < mapH / 1; y += dx)
		{
			particle p;
			p.pos = vec2(x + dx / 2, y + dx / 2) + vec2(nrand(), nrand()) / 10.f;
			p.vel = vec2();

			parts.push_back(p);
		}
	}

	for (int y = 0; y < mapH; ++y)
	{
		for (int x = 0; x < mapW; ++x)
		{
			grid[x][y].reserve(20);
		}
	}
}

void pressureComputation()
{
	for (int i = 0; i < parts.size(); ++i)
		parts[i].density = 0;
	
	for (int i = 0; i < parts.size(); ++i)
	{
		for (int j = 0; j < parts.size(); ++j)
		{
			if (i == j)
				continue;

			parts[i].density += Wpoly6(parts[i].pos - parts[j].pos);
		}
		//parts[i].pressure = k * (pow(parts[i].density / rho_0, 7) - 1);
		parts[i].pressure = k * (parts[i].density - rho_0);
	}

	for (int i = 0; i < parts.size(); ++i)
	{
		vec2 del_p = vec2();

		if (parts[i].density <= 0.005)
			continue;

		for (int j = 0; j < parts.size(); ++j)
		{
			if (i == j)
				continue;

			vec2 r = parts[i].pos - parts[j].pos;
			if (dot(r, r) > h * h)
				continue;
			if (parts[j].density <= 0.005)
				continue;

			del_p = del_p + (parts[i].pressure + parts[j].pressure) / (2 * parts[i].density) *del_Wspiky(r);

			//del_p += parts[j].pressure / parts[j].density * del_Wspiky(parts[i].pos - parts[j].pos);
			//del_p += (parts[j].pressure / (parts[j].density * parts[j].density) + parts[i].pressure / (parts[i].density * parts[i].density)) * del_Wspiky(parts[i].pos - parts[j].pos);
		}

		//printf("%f\n", del_p.y);
		parts[i].vel += (-del_p / parts[i].density) * dt;
	}
}

void findNeighbors()
{
	/*for (int y = 0; y < mapH; ++y)
	{
		for (int x = 0; x < mapW; ++x)
		{
			grid[x][y].clear();
		}
	}

	for (int i = 0; i < parts.size(); ++i)
	{
		grid[(int)(parts[i].pos.x / h)][(int)(parts[i].pos.y / h)].push_back(i);
	}*/

	/*for (int i = 0; i < parts.size(); ++i)
		parts[i].neighbors.clear();

	for (int i = 0; i < parts.size(); ++i)
	{
		for (int j = i + 1; j < parts.size(); ++j)
		{
			vec2 r = parts[j].pos - parts[i].pos;
			float len = dot(r, r) / (h * h);
			if (len < 1)
			{
				parts[j].neighbors.push_back(i);
				parts[i].neighbors.push_back(j);
			}
		}
	}*/
}

void enforceBoundary()
{
	for (particle& p : parts)
	{
		//printf("%f\n", p.pos.y);
		if (p.pos.y < 0.0)
		{
			p.pos.y = 0.0;
		}
		if (p.pos.y > mapH - 0.01)
			p.pos.y = mapH - 0.1;
		if (p.pos.x < 0.01)
			p.pos.x = 0.01;
		if (p.pos.x > mapW - 0.01)
			p.pos.x = mapW - 0.01;

		/*static float iter = 0;
		//vec2 c = vec2(mapW / 2 + cos(iter) * 30, mapH / 8 + 10);
		vec2 c = vec2(3 * mapW / 4, mapH / 8 + 10);

		float r = mapH / 8;
		vec2 dist = p.pos - c;
		if (dot(dist, dist) < r * r)
		{
			p.pos = c + normalize(dist) * (r + nrand() / 50);
		}
		iter += 0.000005;*/
	}
}

int iter = 0;
void update()
{
	/*if (iter == 0)
	{
		if (!GetAsyncKeyState(VK_RETURN))
			return;
		++iter;
	}*/

	//findNeighbors();

	pressureComputation();

	// apply gravity
	vec2 grav = normalize(vec2(0, -1)) * 9.81f;
	for (particle& p : parts)
		p.vel += grav * dt / p.density;

	// advect
	for (particle& p : parts)
	{
		p.pos = p.pos + p.vel * dt;
		//p.vel *= 0.99f;
	}

	enforceBoundary();

	/*if (GetAsyncKeyState('K'))
		k += 0.2;
	if (GetAsyncKeyState('M'))
		k -= 0.2;

	if (GetAsyncKeyState('J'))
		k_near += 0.2;
	if (GetAsyncKeyState('N'))
		k_near -= 0.2;

	if (GetAsyncKeyState('H'))
		h += 0.2;
	if (GetAsyncKeyState('B'))
		h -= 0.2;

	if (GetAsyncKeyState('G'))
		rho_0 += 0.2;
	if (GetAsyncKeyState('V'))
		rho_0 -= 0.2;

	printf("k: %f\n", k);
	printf("k_near: %f\n", k_near);
	printf("rho_0: %f\n", rho_0);
	printf("h: %f\n", h);
	printf("-----------------\n");*/

	// diagnostics
	{
		double rx, ry;
		glfwGetCursorPos(window, &rx, &ry);
		rx /= windowW / mapW; ry /= windowH / mapH;
		ry = mapH - ry;

		//printf("%i, %i \n", (int)rx, (int)ry);

		/*if (0 <= rx && rx < mapW &&
			0 <= ry && ry < mapH)
		{
			printf("-----------------------------\n");
			printf("V: %f, %f\n", u->lerp(rx, ry), v->lerp(rx, ry));
			printf("P: %f\n", p[(int)rx][(int)ry]);
			printf("-----------------------------\n\n");
		}*/

		static boolean isPressed = false;

		if (0 <= rx && rx < mapW &&
			0 <= ry && ry < mapH)
		{
			if (isPressed && glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_LEFT) == 0)
			{
			}
		}

		if (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_LEFT))
			isPressed = true;
		isPressed = false;
	}
}

void draw()
{
	glViewport(0, 0, windowW, windowH);
	glClear(GL_COLOR_BUFFER_BIT);

	glLoadIdentity();
	glTranslatef(-1, -1, 0);
	glScalef(2, 2, 1);
	glScalef(1.f / mapW, 1.f / mapH, 1);

	glPointSize(2);
	glColor3f(1, 1, 1);
	glBegin(GL_POINTS);
	{
		for (particle p : parts)
		{
			float f = p.density / 50;
			glColor3f(f, 0, 1 - f);
			glVertex2f(p.pos.x, p.pos.y);
			//printf("%f\n", p.pos.x);
		}
	}
	glEnd();
}

int main()
{
	srand(time(0));

	if (!glfwInit())
	{
		printf("couldn't initialize GLFW");
		return 0;
	}

	// no window hints. don't really care
	window = glfwCreateWindow(windowW, windowH, "lulz", NULL, NULL);
	if (!window)
	{
		printf("failed to open glfw window");
		return 0;
	}

	glfwMakeContextCurrent(window);
	glfwSwapInterval(1);

	glEnable(GL_BLEND);
	glBlendFunc(GL_SRC_ALPHA, GL_ONE);

	setup();

	// main loop
	auto currentTime = chrono::high_resolution_clock::now();
	float accumulator = 0;
	while (!glfwWindowShouldClose(window))
	{
		auto newTime = chrono::high_resolution_clock::now();
		float frameTime = chrono::duration_cast<chrono::nanoseconds>(newTime - currentTime).count() / 1.0e9;
		currentTime = newTime;

		accumulator += frameTime;

		if (accumulator >= dt)
		{
			update();

			accumulator = 0;
		}

		draw();

		//Sleep(500);

		glfwSwapBuffers(window);
		glfwPollEvents();
	}
	printf("%d", window);
	glfwDestroyWindow(window);
	glfwTerminate();

	return 0;
}