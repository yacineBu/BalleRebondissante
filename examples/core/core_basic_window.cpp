/*******************************************************************************************
*
*   raylib [core] example - Basic window
*
*   Welcome to raylib!
*
*   To test examples, just press F6 and execute raylib_compile_execute script
*   Note that compiled executable is placed in the same folder as .c file
*
*   You can find all basic examples on C:\raylib\raylib\examples folder or
*   raylib official webpage: www.raylib.com
*
*   Enjoy using raylib. :)
*
*   This example has been created using raylib 1.0 (www.raylib.com)
*   raylib is licensed under an unmodified zlib/libpng license (View raylib.h for details)
*
*   Copyright (c) 2014 Ramon Santamaria (@raysan5)
*
********************************************************************************************/

#include "raylib.h"
#include <raymath.h>
#include "rlgl.h"
#include <math.h>
#include <float.h>
#include <vector>
#include <iostream>

#if defined(PLATFORM_DESKTOP)
#define GLSL_VERSION            330
#else   // PLATFORM_RPI, PLATFORM_ANDROID, PLATFORM_WEB
#define GLSL_VERSION            100
#endif

#define EPSILON 1.e-6f

template <typename T> int sgn(T val) {
	return (T(0) < val) - (val < T(0));
}

struct Cylindrical {
	float rho;
	float theta;
	float y;
};

struct Spherical {
	float rho;
	float theta;
	float phi;
};

struct Line {
	Vector3 pt;
	Vector3 dir;
};

struct Segment {
	Vector3 pt1;
	Vector3 pt2;
};

struct Triangle {
	Vector3 pt1;
	Vector3 pt2;
	Vector3 pt3;
};

struct Plane {
	Vector3 n;
	float d;
};

struct ReferenceFrame {
	Vector3 origin;
	Vector3 i, j, k;
	Quaternion q;

	ReferenceFrame()
	{
		origin = { 0,0,0 };
		i = { 1,0,0 };
		j = { 0,1,0 };
		k = { 0,0,1 };
		q = QuaternionIdentity();
	}

	ReferenceFrame(Vector3 origin, Quaternion q)
	{
		this->q = q;
		this->origin = origin;
		i = Vector3RotateByQuaternion({ 1,0,0 }, q);
		j = Vector3RotateByQuaternion({ 0,1,0 }, q);
		k = Vector3RotateByQuaternion({ 0,0,1 }, q);
	}

	void Translate(Vector3 vect)
	{
		this->origin = Vector3Add(this->origin, vect);
	}

	void RotateByQuaternion(Quaternion qRot)
	{
		q = QuaternionMultiply(qRot, q);
		i = Vector3RotateByQuaternion({ 1,0,0 }, q);
		j = Vector3RotateByQuaternion({ 0,1,0 }, q);
		k = Vector3RotateByQuaternion({ 0,0,1 }, q);
	}
};

struct Quad {
	ReferenceFrame ref;
	Vector3 extents;		// extents.y n’est pas utilisé
};

struct Disk {
	ReferenceFrame ref;
	float radius;
};

struct Box {
	ReferenceFrame ref;
	Vector3 extents;
};

struct Sphere {
	ReferenceFrame ref;
	float radius;
};

struct InfiniteCylinder {
	ReferenceFrame ref;
	float radius;
};

struct Cylinder {
	ReferenceFrame ref;
	float halfHeight;
	float radius;
};

struct Capsule {
	ReferenceFrame ref;
	float halfHeight;
	float radius;
};

struct RoundedBox {
	ReferenceFrame ref;
	Vector3 extents;
	float radius;
};

Vector3 CylindricalToCartesian(Cylindrical cyl) {
	return { cyl.rho * sinf(cyl.theta), cyl.y, cyl.rho * cosf(cyl.theta) };
}

Cylindrical CartesianToCylindrical(Vector3 cart) {
	Cylindrical cyl = {
		sqrt((double)powf(cart.x, 2) + (double)powf(cart.z, 2)),
		atan2(cart.x, cart.z),
		cart.y
	};

	if (cyl.theta <= 0)
		cyl.theta += 2 * PI;
	return cyl;
}

Vector3 SphericalToCartesian(Spherical sph) {
	return {
		sph.rho * sinf(sph.phi) * sinf(sph.theta),
		sph.rho * cosf(sph.phi),
		sph.rho * sinf(sph.phi) * cosf(sph.theta)
	};
}

Spherical CartesianToSpherical(Vector3 cart) {
	Spherical sph = {
		sqrt((double)pow(cart.x, 2) + (double)pow(cart.y, 2) + (double)pow(cart.z, 2)),
		atan2(cart.x, cart.z),
		acosf(cart.y / sph.rho)
	};

	if (sph.theta <= 0)
		sph.theta += 2 * PI;
	return sph;
}

void MyUpdateOrbitalCamera(Camera* camera, float deltaTime)
{
	static Spherical sphPos = { 10.0f, PI / 4.0f, PI / 88.0f };

	Spherical sphSpeed = { 2.0f, 0.04f, 0.04f };
	Spherical sphDelta;

	float rhoMin = 4;
	float rhoMax = 40;

	Vector2 mousePos;
	Vector2 mouseVect;
	static Vector2 prevMousePos = { 0,0 };

	// Gestion de la souris 
	mousePos = GetMousePosition(); // 1) récupération de la position de la souris
	mouseVect = Vector2Subtract(mousePos, prevMousePos); // 2) calcul du vecteur de déplacement de la souris entre la position courante de la souris et la précédente
	prevMousePos = mousePos; // 3) mise à jour de la position précédente de la souris avec la position courante

	// 4) calcul du vecteur de déplacement de la caméra en coordonnées sphériques
	sphDelta = CartesianToSpherical(camera->position);


	sphDelta.rho += (GetMouseWheelMove() * deltaTime * sphSpeed.rho * sphPos.rho);
	sphDelta.rho = Clamp(sphDelta.rho, rhoMin, rhoMax);

	if (IsMouseButtonDown(MOUSE_RIGHT_BUTTON)) {
		sphDelta.theta += mouseVect.x * deltaTime * sphSpeed.theta * sphPos.theta;
	}

	if (IsMouseButtonDown(MOUSE_LEFT_BUTTON)) {
		sphDelta.phi += mouseVect.y * deltaTime * sphSpeed.phi;
		sphDelta.phi = Clamp(sphDelta.phi * RAD2DEG, 1, 179);

		sphDelta.phi *= DEG2RAD;
	}

	camera->position = SphericalToCartesian(sphDelta);
}

/*
 * FONCTIONS DE DESSIN
 */

 //QUAD
void MyDrawPolygonQuad(Quad quad, Color color = LIGHTGRAY)
{
	int numVertex = 6;
	if (rlCheckBufferLimit(numVertex)) rlglDraw();

	rlPushMatrix();

	// BEGINNING OF SPACE TRANSFORMATION INDUCED BY THE LOCAL REFERENCE FRAME (on touche au référentiel)
	// methods should be called in this order: rlTranslatef, rlRotatef & rlScalef
	// so that transformations occur in the opposite order: scale, then rotation, then translation

	//TRANSLATION
	rlTranslatef(quad.ref.origin.x, quad.ref.origin.y, quad.ref.origin.z);

	//ROTATION
	Vector3 vect;
	float angle;
	QuaternionToAxisAngle(quad.ref.q, &vect, &angle);
	rlRotatef(angle * RAD2DEG, vect.x, vect.y, vect.z);

	//SCALING
	rlScalef(quad.extents.x, 1, quad.extents.z);
	// END OF SPACE TRANSFORMATION INDUCED BY THE LOCAL REFERENCE FRAME

	rlBegin(RL_TRIANGLES);		// 1 triangle = 3 points. Tout les 3 appels à rlVertex3f(), un triangle se dessine.
	rlColor4ub(color.r, color.g, color.b, color.a);
	rlVertex3f(1, 0, 1);
	rlVertex3f(1, 0, -1);
	rlVertex3f(-1, 0, -1);
	rlVertex3f(1, 0, 1);
	rlVertex3f(-1, 0, -1);
	rlVertex3f(-1, 0, 1);
	rlEnd();

	//EVERY rlPushMatrix method call should be followed by a rlPopMatrix method call
	rlPopMatrix();
}

void MyDrawWireframeQuad(Quad quad, Color color = DARKGRAY)
{
	int numVertex = 10;
	if (rlCheckBufferLimit(numVertex)) rlglDraw();

	rlPushMatrix();

	rlTranslatef(quad.ref.origin.x, quad.ref.origin.y, quad.ref.origin.z);
	Vector3 vect;
	float angle;
	QuaternionToAxisAngle(quad.ref.q, &vect, &angle);
	rlRotatef(angle * RAD2DEG, vect.x, vect.y, vect.z);
	rlScalef(quad.extents.x, 1, quad.extents.z);

	rlBegin(RL_LINES);		// 1 droite = 2 points. Tout les 2 appels à rlVertex3f(), une droite se dessine.
	rlColor4ub(color.a, color.g, color.b, color.a);
	rlVertex3f(1, 0, 1);
	rlVertex3f(1, 0, -1);
	rlVertex3f(1, 0, -1);
	rlVertex3f(-1, 0, -1);
	rlVertex3f(-1, 0, -1);
	rlVertex3f(1, 0, 1);
	rlVertex3f(-1, 0, -1);
	rlVertex3f(-1, 0, 1);
	rlVertex3f(-1, 0, 1);
	rlVertex3f(1, 0, 1);
	rlEnd();

	rlPopMatrix();
}

void MyDrawQuad(Quad quad, bool drawPolygon = true, bool drawWireframe = true,
	Color polygonColor = LIGHTGRAY, Color wireframeColor = DARKGRAY)
{
	if (drawPolygon) MyDrawPolygonQuad(quad, polygonColor);
	if (drawWireframe) MyDrawWireframeQuad(quad, wireframeColor);
}

// DISK
void MyDrawPolygonDisk(Disk disk, int nSectors, Color color = LIGHTGRAY) {
	// Minimum 3 secteur de disque pour pouvoir tracer le disque
	if (nSectors < 3) {
		return;
	}

	int numVertex = nSectors * 3;		// nbr total d'appel à rlVertx3f : 3 appels par secteur pour dessiner un triangle
	if (rlCheckBufferLimit(numVertex)) rlglDraw();

	rlPushMatrix();

	// On touche au référentiel. On met à la bonne place le "crayon" pour qu'il puisse dessiner le disk tel que celui décrit dans l'objet "disk" reçu en paramètre
	rlTranslatef(disk.ref.origin.x, disk.ref.origin.y, disk.ref.origin.z);
	Vector3 vect;
	float angle;
	QuaternionToAxisAngle(disk.ref.q, &vect, &angle);
	rlRotatef(angle * RAD2DEG, vect.x, vect.y, vect.z);			// rotation du "crayon" en mode quaterninon, bon ici on applique la rotation à partir d'un axe/vecteur (vect.x, vect.y, vect.z) et d'un angle, pour simplifier les choses
	rlScalef(disk.radius, 1, disk.radius);

	// Création du tab qui contient les points du disque (nbr de points en fnc de nSectors). On fait ca en utilisant le système de coordonnées cylindrique (ca facilite les choses, seulement la coordonnée teta est à modifier)
	float pitch = 2 * PI / nSectors;
	std::vector<Cylindrical> diskPointsAtPerim;
	for (int i = 0; i < nSectors; i++) {
		diskPointsAtPerim.push_back({ 1, i * pitch, 0 });
	}
	diskPointsAtPerim.push_back({ 1, 0, 0 });		// On termine le tableau par une duplication du tout premier point du disk (pratique pour le moment où on dessine le disk)

	// Enfin, on dessine le disque, en utilisant les méthodes de conversion pour repasser au sys cartésien
	rlBegin(RL_TRIANGLES);			// 1 triangle = 3 points. Tout les 3 appels à rlVertex3f(), un triangle se dessine.
	rlColor4ub(color.r, color.g, color.b, color.a);
	Vector3 carPt, carPt2;
	for (int i = 0; i < diskPointsAtPerim.size() - 1; i++) {
		rlVertex3f(0, 0, 0);
		carPt = CylindricalToCartesian(diskPointsAtPerim[i]);
		rlVertex3f(carPt.x, carPt.y, carPt.z);
		carPt2 = CylindricalToCartesian(diskPointsAtPerim[i+1]);
		rlVertex3f(carPt2.x, carPt2.y, carPt2.z);
	}
	rlEnd();

	rlPopMatrix();
}

void MyDrawWireframeDisk(Disk disk, int nSectors, Color color = DARKGRAY) {
	if (nSectors < 3) {
		return;
	}

	int numVertex = nSectors * 4;		// nbr total d'appel à rlVertx3f : 4 appels par secteur pour dessiner les côtés des triangles
	if (rlCheckBufferLimit(numVertex)) rlglDraw();

	rlPushMatrix();

	rlTranslatef(disk.ref.origin.x, disk.ref.origin.y, disk.ref.origin.z);
	Vector3 vect;
	float angle;
	QuaternionToAxisAngle(disk.ref.q, &vect, &angle);
	rlRotatef(angle * RAD2DEG, vect.x, vect.y, vect.z);
	rlScalef(disk.radius, 1, disk.radius);

	float pitch = 2 * PI / nSectors;
	std::vector<Cylindrical> diskPointsAtPerim;
	for (int i = 0; i < nSectors; i++) {
		diskPointsAtPerim.push_back({ 1, i * pitch, 0 });
	}
	diskPointsAtPerim.push_back({ 1, 0, 0 });

	rlBegin(RL_LINES);
	rlColor4ub(color.r, color.g, color.b, color.a);
	Vector3 carPt, carPt2;
	for (int i = 0; i < diskPointsAtPerim.size() - 1; i++) {
		rlVertex3f(0, 0, 0);
		carPt = CylindricalToCartesian(diskPointsAtPerim[i]);
		rlVertex3f(carPt.x, carPt.y, carPt.z);
		rlVertex3f(carPt.x, carPt.y, carPt.z);
		carPt2 = CylindricalToCartesian(diskPointsAtPerim[i + 1]);
		rlVertex3f(carPt2.x, carPt2.y, carPt2.z);
	}
	rlEnd();

	rlPopMatrix();
}

void MyDrawDisk(Disk disk, int nSectors, bool drawPolygon = true, bool drawWireframe = true, Color polygonColor = LIGHTGRAY, Color wireframeColor = DARKGRAY) {
	if (drawPolygon) MyDrawPolygonDisk(disk, nSectors, polygonColor);
	if (drawWireframe) MyDrawWireframeDisk(disk, nSectors, wireframeColor);
}

// SPHERE

// void DrawTop();

// écrire code de test pour chaque meth de tracage 
void MyDrawPolygonSphere(Sphere sphere, int nMeridians, int nParallels, Color
	color = LIGHTGRAY) {
	if (nMeridians < 2 || nParallels < 1)
		return;

	int numVertex = 1000;		// !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
	if (rlCheckBufferLimit(numVertex)) rlglDraw();

	rlPushMatrix();

	rlTranslatef(sphere.ref.origin.x, sphere.ref.origin.y, sphere.ref.origin.z);
	Vector3 vect;
	float angle;
	QuaternionToAxisAngle(sphere.ref.q, &vect, &angle);
	rlRotatef(angle * RAD2DEG, vect.x, vect.y, vect.z);
	rlScalef(sphere.radius, sphere.radius, sphere.radius);

	float thetaPitch = PI / nMeridians;				// permet pendant le dessin de passer d'un méridien à un autre
	float phiPitch = PI / (nParallels + 1);			// permet pendant le dessin de passer d'un parallèle à un autre
	Vector3 top = SphericalToCartesian({ 1, 0, 0 });
	Vector3 bottom = SphericalToCartesian({ 1, 0, PI });
	
	// Pour le dessin, j'ai le choix entre 
	//	- pré-calculer les points de la sphère, les stocker dans un tableau, puis dessiner la sphère en utilisant ce tableau
	//	- tout faire en live : pendant le dessin calculer les points
	// Les 2 reviennent au même normalement.
	// J'avais pris la 1ère option pour dessiner un disk. Ici, je prends l'option 2

	// L'algo commence par dessiner le haut de la sphère et termine par le bas, en prennant soin d'effectuer un tour complet sur chaque parallèle
	rlBegin(RL_TRIANGLES);			
	rlColor4ub(color.r, color.g, color.b, color.a);

	Spherical sphPt;
	Vector3 carPt, carPt_save;

	// Dessin du haut (un cone)
	rlVertex3f(top.x, top.y, top.z);
	sphPt = { 1, 0, phiPitch };
	carPt = SphericalToCartesian(sphPt);
	rlVertex3f(carPt.x, carPt.y, carPt.z);
	sphPt = { 1, thetaPitch, phiPitch };
	carPt = SphericalToCartesian(sphPt);
	carPt_save = { carPt.x, carPt.y, carPt.z };				// Le 2ème point du prochain triangle sera le même, on le sauvegarde au lieu de le recalculer
	rlVertex3f(carPt.x, carPt.y, carPt.z);
	for (int i = 1 ; i < nMeridians*2 ; i++) {
		rlVertex3f(top.x, top.y, top.z);
		rlVertex3f(carPt_save.x, carPt_save.y, carPt_save.z);
		sphPt = { 1, thetaPitch * (i+1), phiPitch };
		carPt = SphericalToCartesian(sphPt);
		carPt_save = { carPt.x, carPt.y, carPt.z };			// Idem
		rlVertex3f(carPt.x, carPt.y, carPt.z);
	}

	// Dessin du corps
	for (int i = 1; i < nParallels; i++) {
		// Une partie du corps (un anneau, un peu près)
		for (int j = 0; j < nMeridians * 2; j++) {
			sphPt = { 1, thetaPitch * j, phiPitch * i };
			carPt = SphericalToCartesian(sphPt);
			rlVertex3f(carPt.x, carPt.y, carPt.z);

			sphPt = { 1, thetaPitch * j, phiPitch * (i + 1) };
			carPt = SphericalToCartesian(sphPt);
			carPt_save = { carPt.x, carPt.y, carPt.z };						// Le 6ème point sera le même, on le sauvegarde au lieu de le recalculer
			rlVertex3f(carPt.x, carPt.y, carPt.z);

			sphPt = { 1, thetaPitch * (j + 1), phiPitch * i };
			carPt = SphericalToCartesian(sphPt);
			rlVertex3f(carPt.x, carPt.y, carPt.z);

			sphPt = { 1, thetaPitch * j, phiPitch * (i + 1) };
			carPt = SphericalToCartesian(sphPt);
			rlVertex3f(carPt.x, carPt.y, carPt.z);

			sphPt = { 1, thetaPitch * (j + 1), phiPitch * (i + 1) };
			carPt = SphericalToCartesian(sphPt);
			rlVertex3f(carPt.x, carPt.y, carPt.z);

			sphPt = { 1, thetaPitch * (j + 1), phiPitch * i };
			carPt = SphericalToCartesian(sphPt);
			rlVertex3f(carPt.x, carPt.y, carPt.z);
		}
	}

	// Dessin du bas (un cone renversé)
	sphPt = { 1, 0, PI - phiPitch };
	carPt = SphericalToCartesian(sphPt);
	rlVertex3f(carPt.x, carPt.y, carPt.z);
	rlVertex3f(bottom.x, bottom.y, bottom.z);
	sphPt = { 1, thetaPitch, PI - phiPitch };
	carPt = SphericalToCartesian(sphPt);
	carPt_save = { carPt.x, carPt.y, carPt.z };				// Le 1ème point du prochain triangle sera le même, on le sauvegarde au lieu de le recalculer
	rlVertex3f(carPt.x, carPt.y, carPt.z);
	for (int i = 1; i < nMeridians * 2; i++) {
		rlVertex3f(carPt_save.x, carPt_save.y, carPt_save.z);
		rlVertex3f(bottom.x, bottom.y, bottom.z);
		sphPt = { 1, thetaPitch * (i + 1), PI - phiPitch };
		carPt = SphericalToCartesian(sphPt);
		carPt_save = { carPt.x, carPt.y, carPt.z };			// Idem
		rlVertex3f(carPt.x, carPt.y, carPt.z);
	}

	rlEnd();

	rlPopMatrix();
}

void MyDrawWireframeSphere(Sphere sphere, int nMeridians, int nParallels, Color
	color = DARKGRAY) {
	if (nMeridians < 2 || nParallels < 1)
		return;

}

void MyDrawSphere(Sphere sphere, int nMeridians, int nParallels, bool
	drawPolygon = true, bool drawWireframe = true, Color polygonColor = LIGHTGRAY,
	Color wireframeColor = DARKGRAY) {

}



int main(int argc, char* argv[])
{
	// Initialization
	//--------------------------------------------------------------------------------------
	float screenSizeCoef = .9f;
	const int screenWidth = 1920 * screenSizeCoef;
	const int screenHeight = 1080 * screenSizeCoef;

	InitWindow(screenWidth, screenHeight, "ESIEE - E3FI - 2022 - 2023 - Maths 3D");

	SetTargetFPS(60);

	//CAMERA
	Vector3 cameraPos = { 8.0f, 15.0f, 14.0f };
	Camera camera = { 0 };
	camera.position = cameraPos;
	camera.target = { 0.0f, 0.0f, 0.0f };
	camera.up = { 0.0f, 1.0f, 0.0f };
	camera.fovy = 45.0f;
	camera.type = CAMERA_PERSPECTIVE;
	SetCameraMode(camera, CAMERA_CUSTOM);  // Set an orbital camera mode


	//--------------------------------------------------------------------------------------

	// Main game loop
	while (!WindowShouldClose())    // Detect window close button or ESC key
	{
		// Update
		//----------------------------------------------------------------------------------
		// TODO: Update your variables here
		//----------------------------------------------------------------------------------

		float deltaTime = GetFrameTime();
		float time = (float)GetTime();

		MyUpdateOrbitalCamera(&camera, deltaTime);

		// Draw
		//----------------------------------------------------------------------------------
		BeginDrawing();

		ClearBackground(RAYWHITE);

		BeginMode3D(camera);
		{			
			//3D REFERENTIAL
			DrawGrid(20, 1.0f);        // Draw a grid
			DrawLine3D({ 0 }, { 0,10,0 }, DARKGRAY);
			DrawSphere({ 10,0,0 }, .2f, RED);
			DrawSphere({ 0,10,0 }, .2f, GREEN);
			DrawSphere({ 0,0,10 }, .2f, BLUE);

			// QUAD DISPLAY TEST
			//ReferenceFrame ref = ReferenceFrame(
			//	{ 0,2,0 },
			//	QuaternionFromAxisAngle(Vector3Normalize({ 1,1,1 }), PI / 4));
			//Quad quad = { ref,{3,1,5} };
			//MyDrawQuad(quad);

			// DISK DISPLAY TEST
			//ReferenceFrame ref2 = ReferenceFrame(
			//	{3, 1, 0},
			//	QuaternionFromAxisAngle(Vector3Normalize({ 1,0,0 }), PI / 3));
			//Disk d = { ref2, 5 };
			//MyDrawDisk(d, 30);

			// SPHERE DISPLAY TEST
			ReferenceFrame ref2 = ReferenceFrame(
				{0, 0, 0},
				QuaternionFromAxisAngle(Vector3Normalize({ 1,0,0 }), 0));
			Sphere s = { ref2, 4 };
			MyDrawPolygonSphere(s, 10, 10);

		}
		EndMode3D();

		EndDrawing();
		//----------------------------------------------------------------------------------
	}

	// De-Initialization
	//--------------------------------------------------------------------------------------   
	CloseWindow();        // Close window and OpenGL context
	//--------------------------------------------------------------------------------------

	return 0;
}