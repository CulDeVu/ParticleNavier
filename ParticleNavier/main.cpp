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
	vec2 prev;
	vec2 vel;
};
vector<particle> parts;

vector<int> grid[mapW][mapH];

int walls[mapW][mapH];

GLFWwindow* window;

float nrand()
{
	return (float)rand() / RAND_MAX;
}

void setup()
{
	float dx = 2;
	for (float x = 0; x < mapW / 2; x += dx)
	{
		for (float y = 0; y < mapH; y += dx)
		{
			particle p;
			p.pos = p.prev = vec2(x + dx / 2, y + dx / 2) + vec2(nrand(), nrand()) / 10.f;
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

void spawnSquare(int xpos, int ypos, int r)
{
	for (int x = xpos - r; x < xpos + r; x += 4)
	{
		for (int y = ypos - r; y < ypos + r; y += 4)
		{
			particle p;
			p.pos = p.prev = vec2(x + 0.5, y + 0.5);
			p.vel = vec2();

			parts.push_back(p);
		}
	}
}

float k = 10;
float k_near = 20;
float rho_0 = 3;
float h = 5;
void doubleDensityRelaxation()
{ 
	for (int i = 0; i < parts.size(); ++i)
	{
		particle& p_i = parts[i];
		
		float rho = 0;
		float rho_near = 0;
		
		// compute density and near-density
		for (int y = max(0.f, p_i.pos.y / h - 1); y <= min(mapH - 1, p_i.pos.y / h + 1); ++y)
		{
			for (int x = max(0.f, p_i.pos.x / h - 1); x <= min(mapW - 1, p_i.pos.x / h + 1); ++x)
			{
				for (int j : grid[x][y])
				{
					if (i == j)
						continue;

					float q = length(p_i.pos - parts[j].pos) / h;
					if (q < 1)
					{
						rho = rho + (1 - q) * (1 - q);
						rho_near = rho_near + pow(1 - q, 3);
					}
				}
			}
		}

		// compute pressure and near-pressure
		float P = k * (rho - rho_0);
		float P_near = k_near * rho_near;

		// apply displacements
		vec2 dx = vec2();
		for (int y = max(0.f, p_i.pos.y / h - 1); y <= min(mapH - 1, p_i.pos.y / h + 1); ++y)
		{
			for (int x = max(0.f, p_i.pos.x / h - 1); x <= min(mapW - 1, p_i.pos.x / h + 1); ++x)
			{
				for (int j : grid[x][y])
				{
					if (i == j)
						continue;

					float q = length(parts[j].pos - p_i.pos) / h;
					if (q < 1)
					{
						vec2 D = dt * dt * (P * (1 - q) + P_near * (1 - q) * (1 - q)) * (parts[j].pos - p_i.pos);
						parts[j].pos += D / 2.f;
						dx -= D / 2.f;

					}
				}
			}
		}

		p_i.pos += dx;
	}
}

void findNeighbors()
{
	for (int y = 0; y < mapH; ++y)
	{
		for (int x = 0; x < mapW; ++x)
		{
			grid[x][y].clear();
		}
	}

	for (int i = 0; i < parts.size(); ++i)
	{
		grid[(int)(parts[i].pos.x / h)][(int)(parts[i].pos.y / h)].push_back(i);
	}
}

void enforceBoundary()
{
	for (particle& p : parts)
	{
		if (p.pos.y < 0)
			p.pos.y = 0.01;
		if (p.pos.y > mapH - 0.01)
			p.pos.y = mapH - 0.1;
		if (p.pos.x < 0.01)
			p.pos.x = 0.01;
		if (p.pos.x > mapW - 0.01)
			p.pos.x = mapW - 0.01;

		/*int wall = walls[(int)p.pos.x][(int)p.pos.y];
		if (wall != 0)
		{
			float xRel = p.pos.x - (int)p.pos.x;
			float yRel = p.pos.y - (int)p.pos.y;
			if (yRel > xRel && yRel < 1 - xRel)
				p.pos.x = (int)p.pos.x;
			if (yRel < xRel && yRel > 1 - xRel)
				p.pos.x = (int)p.pos.x + 1;
			if (yRel > xRel && yRel > 1 - xRel)
				p.pos.y = (int)p.pos.y + 1;
		}*/
		/*float l = mapW / 2;
		float r = mapW / 2 + 20;
		float b = 0;
		float t = mapH / 4;
		float slope = (t - b) / (r - l);
		if (l < p.pos.x && p.pos.x < r &&
			b < p.pos.y && p.pos.y < t)
		{
			float xRel = p.pos.x - l;
			float yRel = p.pos.y - b;

			if (yRel >= slope * xRel && yRel <= slope * (1 - xRel))
				p.pos.x = l;
		}*/
		static float iter = 0;
		//vec2 c = vec2(mapW / 2 + cos(iter) * 30, mapH / 8 + 10);
		vec2 c = vec2(3 * mapW / 4, mapH / 8 + 10);

		float r = mapH / 8;
		vec2 dist = p.pos - c;
		if (dot(dist, dist) < r * r)
		{
			p.pos = c + normalize(dist) * (r + nrand() / 50);
		}
		iter += 0.000005;
	}
}

int iter = 0;
void update()
{
	if (iter == 0)
	{
		if (!GetAsyncKeyState(VK_RETURN))
			return;
		++iter;
	}

	// apply gravity
	vec2 grav = normalize(vec2(0, -1)) * 9.81f;
	for (particle& p : parts)
		p.vel = p.vel + grav * dt;

	// apply viscosity

	// advect
	for (particle& p : parts)
	{
		p.prev = p.pos;
		p.pos = p.pos + p.vel * dt;
	}

	findNeighbors();

	// relaxation
	doubleDensityRelaxation();

	// resolve collisions

	enforceBoundary();

	// correct velocity
	for (particle& p : parts)
	{
		p.vel = (p.pos - p.prev) / dt;
	}

	if (GetAsyncKeyState('K'))
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
	printf("-----------------\n");

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
				spawnSquare(rx, ry, 20);
				//vel[(int)rx][(int)ry] += p;
			}
		}

		if (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_LEFT))
			isPressed = true;
		isPressed = false;
	}
}

void drawCircle(vec2 pos, float r)
{
	glPushMatrix();
	{
		glTranslatef(pos.x, pos.y, 0);

		glBegin(GL_LINE_STRIP);
		{
			glVertex2f(r, 0);
			for (int i = 1; i < 8; ++i)
				glVertex2f(r * cos(2 * PI * i / 8), r * sin(2 * PI * i / 8));
			glVertex2f(r, 0);
		}
		glEnd();
	}
	glPopMatrix();
}
float fluid[mapW][mapH];
void draw()
{
	glViewport(0, 0, windowW, windowH);
	glClear(GL_COLOR_BUFFER_BIT);

	glLoadIdentity();
	glTranslatef(-1, -1, 0);
	glScalef(2, 2, 1);
	glScalef(1.f / mapW, 1.f / mapH, 1);

	for (int y = 0; y < mapH; ++y)
		for (int x = 0; x < mapW; ++x)
			fluid[x][y] = 0;
	for (particle p : parts)
	{
		float rad = h * 5;
		for (int x = max(0.f, p.pos.x - rad); x <= min(mapW - 0.001, p.pos.x + rad); ++x)
		{
			for (int y = max(0.f, p.pos.y - rad); y <= min(mapH - 0.001, p.pos.y + rad); ++y)
			{
				fluid[x][y] += pow(1 - min(1.f, length(vec2(x, y) - p.pos) / h), 2);
			}
		}
	}
	for (int y = 0; y < mapH; ++y)
		for (int x = 0; x < mapW; ++x)
			fluid[x][y] = sqrt(fluid[x][y]);

	glColor3f(0, 0, 1);
	glBegin(GL_QUADS);
	{
		for (int y = 0; y < mapH; ++y)
		{
			for (int x = 0; x < mapW; ++x)
			{
				float cutoff = 0.8;
				if (fluid[x][y] > cutoff)
				{
					glColor3f(1 - fluid[x][y], 1 - fluid[x][y], 1);
					glVertex2f(x, y);
					glVertex2f(x + 1, y);
					glVertex2f(x + 1, y + 1);
					glVertex2f(x, y + 1);
				}
				/*else
				{
					if (y > 0 && x > 0)
					{
						if (fluid[x][y - 1] > cutoff && fluid[x - 1][y] > cutoff)
						{
							float f = (fluid[x][y - 1] + fluid[x - 1][y]) / 2;
							glColor3f(1 - f, 1 - f, 1);
							glVertex2f(x, y);
							glVertex2f(x + 1, y);
							glVertex2f(x, y + 1);
							glVertex2f(x, y);
						}
					}
					if (y > 0 && x < mapW)
					{
						if (fluid[x][y - 1] > cutoff && fluid[x + 1][y] > cutoff)
						{
							glColor3f(0, 0, (fluid[x][y - 1] + fluid[x + 1][y]) / 2);
							glVertex2f(x, y);
							glVertex2f(x + 1, y);
							glVertex2f(x + 1, y + 1);
							glVertex2f(x, y);
						}
					}
					if (y < mapH && x > 0)
					{
						if (fluid[x][y + 1] > cutoff && fluid[x - 1][y] > cutoff)
						{
							glColor3f(0, 0, (fluid[x][y + 1] + fluid[x - 1][y]) / 2);
							glVertex2f(x, y);
							glVertex2f(x + 1, y + 1);
							glVertex2f(x, y + 1);
							glVertex2f(x, y);
						}
					}
					if (y < mapH && x < mapW)
					{
						if (fluid[x][y + 1] > cutoff && fluid[x + 1][y] > cutoff)
						{
							glColor3f(0, 0, (fluid[x][y + 1] + fluid[x + 1][y]) / 2);
							glVertex2f(x + 1, y);
							glVertex2f(x + 1, y + 1);
							glVertex2f(x, y + 1);
							glVertex2f(x + 1, y);
						}
					}
				}*/
			}
		}
	}
	glEnd();

	/*glColor3f(0, 1, 0);
	glBegin(GL_QUADS);
	{
		for (int y = 0; y < mapH; ++y)
		{
			for (int x = 0; x < mapW; ++x)
			{
				if (walls[x][y] > 0)
				{
					glVertex2f(x, y);
					glVertex2f(x + 1, y);
					glVertex2f(x + 1, y + 1);
					glVertex2f(x, y + 1);
				}
			}
		}
	}
	glEnd();*/

	/*glPointSize(2);
	glColor3f(1, 1, 1);
	glBegin(GL_POINTS);
	{
		for (particle p : parts)
			glVertex2f(p.pos.x, p.pos.y);
	}
	glEnd();
	/*for (particle p : parts)
		drawCircle(p.pos, 1.5);*/
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
		float frameTime = chrono::duration_cast<chrono::milliseconds>(newTime - currentTime).count();

		if (frameTime >= dt)
		{
			update();

			currentTime = newTime;
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