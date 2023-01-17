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
#include <map>

#if defined(PLATFORM_DESKTOP)
#define GLSL_VERSION            330
#else   // PLATFORM_RPI, PLATFORM_ANDROID, PLATFORM_WEB
#define GLSL_VERSION            100
#endif

#define EPSILON 1.e-6f

template <typename T> int sgn(T val) {
	return (T(0) < val) - (val < T(0));
}

#pragma region Structure;
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
	Vector3 normal;
	float d;		// d est la distance entre le point "au centre du plan" et l'origine du monde, sur l'axe du vecteur normal
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
#pragma endregion;

#pragma region Tools;
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

/// <summary>
/// Exprime un vecteur défini en coordonnées globales en coordonnées locales
/// </summary>
/// <param name="globalVect">Le vecteur à convertir</param>
/// <param name="localRef">Référentiel local cible</param>
/// <returns>Vecteur exprimé en coordonnées locales</returns>
Vector3 GlobalToLocalVect(Vector3 globalVect, ReferenceFrame localRef) {
	return { 
		Vector3DotProduct(globalVect, localRef.i),
		Vector3DotProduct(globalVect, localRef.j),
		Vector3DotProduct(globalVect, localRef.k)
	};
}

/// <summary>
/// Exprime un vecteur défini en coordonnées locales en coordonnées globales
/// </summary>
/// <param name="localVect">Le vecteur à convertir</param>
/// <param name="localRef">Référentiel local source</param>
/// <returns>Vecteur exprimé en coordonnées globales</returns>
Vector3 LocalToGlobalVect(Vector3 localVect, ReferenceFrame localRef) {
	Vector3 xVect = Vector3Scale(localRef.i, localVect.x);
	Vector3 yVect = Vector3Scale(localRef.j, localVect.y);
	Vector3 zVect = Vector3Scale(localRef.k, localVect.z);

	return Vector3Add(Vector3Add(xVect, yVect), zVect);
}

/// <summary>
/// Exprime un point défini en coordonnées globales en coordonnées locales
/// </summary>
/// <param name="globalPos">Le point à convertir</param>
/// <param name="localRef">Référentiel local cible</param>
/// <returns>Point exprimé en coordonnées locales</returns>
Vector3 GlobalToLocalPos(Vector3 globalPos, ReferenceFrame localRef)
{
	Vector3 worldOrigin = { 0,0,0 };
	Vector3 globalVect = Vector3Subtract(Vector3Subtract(globalPos, worldOrigin), Vector3Subtract(localRef.origin, worldOrigin));

	return GlobalToLocalVect(globalVect, localRef);
}

/// <summary>
/// Exprime un point défini en coordonnées locales en coordonnées globales
/// </summary>
/// <param name="localPos">Le point à convertir</param>
/// <param name="localRef">Référentiel local source</param>
/// <returns>Point exprimé en coordonnées globales</returns>
Vector3 LocalToGlobalPos(Vector3 localPos, ReferenceFrame localRef) {
	Vector3 worldOrigin = { 0,0,0 };

	return Vector3Add(Vector3Subtract(localRef.origin, worldOrigin), LocalToGlobalVect(localPos, localRef));
}
#pragma endregion;

#pragma region PrimitiveDraw;
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

// BOX
void MyDrawPolygonBox(Box box, Color color = LIGHTGRAY) {

	rlPushMatrix();

	rlTranslatef(box.ref.origin.x, box.ref.origin.y, box.ref.origin.z);
	Vector3 vect;
	float angle;
	QuaternionToAxisAngle(box.ref.q, &vect, &angle);
	rlRotatef(angle * RAD2DEG, vect.x, vect.y, vect.z);
	rlScalef(box.extents.x, box.extents.y, box.extents.z);
	
	float piOn2 = PI / 2;
	Vector3 extents = { 1, 0, 1 };
	// La face normale à l'axe X, situé dans la partie positive de l'axe
	Quad sideNormalToXPositive = { {{1, 0, 0}, QuaternionFromAxisAngle({0, 0, 1}, -piOn2)}, extents };
	// La face normale à l'axe X, situé dans la partie négative de l'axe
	Quad sideNormalToXNegative = { {{-1, 0, 0}, QuaternionFromAxisAngle({0, 0, 1}, piOn2)}, extents };
	// etc...
	Quad sideNormalToYPositive = { {{0, 1, 0}, QuaternionIdentity()}, extents };
	Quad sideNormalToYNegative = { {{0, -1, 0}, QuaternionFromAxisAngle({0, 0, 1}, PI)}, extents };
	Quad sideNormalToZPositive = { {{0, 0, 1 }, QuaternionFromAxisAngle({1, 0, 0}, piOn2)}, extents };
	Quad sideNormalToZNegative = { {{0, 0, -1 }, QuaternionFromAxisAngle({1, 0, 0}, -piOn2)}, extents };

	MyDrawPolygonQuad(sideNormalToXPositive, color);
	MyDrawPolygonQuad(sideNormalToXNegative, color);
	MyDrawPolygonQuad(sideNormalToYPositive, color);
	MyDrawPolygonQuad(sideNormalToYNegative, color);
	MyDrawPolygonQuad(sideNormalToZPositive, color);
	MyDrawPolygonQuad(sideNormalToZNegative, color);

	rlPopMatrix();
}

void MyDrawWireframeBox(Box box, Color color = DARKGRAY) {
	rlPushMatrix();

	rlTranslatef(box.ref.origin.x, box.ref.origin.y, box.ref.origin.z);
	Vector3 vect;
	float angle;
	QuaternionToAxisAngle(box.ref.q, &vect, &angle);
	rlRotatef(angle * RAD2DEG, vect.x, vect.y, vect.z);
	rlScalef(box.extents.x, box.extents.y, box.extents.z);

	float piOn2 = PI / 2;
	Vector3 extents = { 1, 0, 1 };
	Quad sideNormalToXPositive = { {{1, 0, 0}, QuaternionFromAxisAngle({0, 0, 1}, -piOn2)}, extents };
	Quad sideNormalToXNegative = { {{-1, 0, 0}, QuaternionFromAxisAngle({0, 0, 1}, piOn2)}, extents };
	Quad sideNormalToYPositive = { {{0, 1, 0}, QuaternionIdentity()}, extents };
	Quad sideNormalToYNegative = { {{0, -1, 0}, QuaternionFromAxisAngle({0, 0, 1}, PI)}, extents };
	Quad sideNormalToZPositive = { {{0, 0, 1 }, QuaternionFromAxisAngle({1, 0, 0}, piOn2)}, extents };
	Quad sideNormalToZNegative = { {{0, 0, -1 }, QuaternionFromAxisAngle({1, 0, 0}, -piOn2)}, extents };

	MyDrawWireframeQuad(sideNormalToXPositive, color);
	MyDrawWireframeQuad(sideNormalToXNegative, color);
	MyDrawWireframeQuad(sideNormalToYPositive, color);
	MyDrawWireframeQuad(sideNormalToYNegative, color);
	MyDrawWireframeQuad(sideNormalToZPositive, color);
	MyDrawWireframeQuad(sideNormalToZNegative, color);

	rlPopMatrix();
}

void MyDrawBox(Box box, bool drawPolygon = true, bool drawWireframe = true,
	Color polygonColor = LIGHTGRAY, Color wireframeColor = DARKGRAY) {
	if (drawPolygon) MyDrawPolygonBox(box, polygonColor);
	if (drawWireframe) MyDrawWireframeBox(box, wireframeColor);
}

/// <summary>
/// Créer le tableau qui contient les points du disque. Le nombre de points est en fonction de nSectors.
/// On fait ca en utilisant le système de coordonnées cylindrique (ca facilite les choses, seulement la coordonnée teta est à modifier).
/// Méthode utilisée par les méthodes de dessin du disque et du cylindre.
/// </summary>
/// <param name="nSectors">Le nombre de secteurs voulu être dessinés pour un disque ou un cylindre</param>
/// <returns>Tableau de points en coordonnées cylindriques</returns>
std::vector<Cylindrical> computeDiskPoints(int nSectors) {
	float pitch = 2 * PI / nSectors;
	std::vector<Cylindrical> diskPointsAtPerim;
	for (int i = 0; i < nSectors; i++) {
		diskPointsAtPerim.push_back({ 1, i * pitch, 0 });
	}
	diskPointsAtPerim.push_back({ 1, 0, 0 });		// On termine le tableau par une duplication du tout premier point (pratique pour le moment où on dessine)

	return diskPointsAtPerim;
}

// DISQUE
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

	std::vector<Cylindrical> diskPointsAtPerim = computeDiskPoints(nSectors);

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


void MyDrawPolygonCylinder(Cylinder cylinder, int nSectors, bool drawCaps =
	false, Color color = LIGHTGRAY) {
	if (nSectors < 3) {
		return;
	}

	// Uniquement pour le cylindre, les points du disk sont déjà gérés par la méthode de dessin du disk
	int numVertex = nSectors * 6;
	if (rlCheckBufferLimit(numVertex)) rlglDraw();

	rlPushMatrix();

	rlTranslatef(cylinder.ref.origin.x, cylinder.ref.origin.y, cylinder.ref.origin.z);
	Vector3 vect;
	float angle;
	QuaternionToAxisAngle(cylinder.ref.q, &vect, &angle);
	rlRotatef(angle * RAD2DEG, vect.x, vect.y, vect.z);
	rlScalef(cylinder.radius, cylinder.halfHeight, cylinder.radius);

	std::vector<Cylindrical> diskPointsAtPerim = computeDiskPoints(nSectors);

	rlBegin(RL_TRIANGLES);
	rlColor4ub(color.r, color.g, color.b, color.a);
	Vector3 carPt, carPt2;
	for (int i = 0; i < diskPointsAtPerim.size() - 1; i++) {
		carPt = CylindricalToCartesian(diskPointsAtPerim[i]);
		rlVertex3f(carPt.x, carPt.y + 1, carPt.z);
		rlVertex3f(carPt.x, carPt.y - 1, carPt.z);
		carPt2 = CylindricalToCartesian(diskPointsAtPerim[i + 1]);
		rlVertex3f(carPt2.x, carPt2.y - 1, carPt2.z);
		rlVertex3f(carPt.x, carPt.y + 1, carPt.z);
		rlVertex3f(carPt2.x, carPt2.y - 1, carPt2.z);
		rlVertex3f(carPt2.x, carPt2.y + 1, carPt2.z);
	}
	rlEnd();

	// Imbrication de la transformation dans l'espace
	if (drawCaps) {
		Disk top = { {{0, 1, 0},  QuaternionIdentity()}, 1};
		Disk bottom = { {{0, -1, 0}, QuaternionFromAxisAngle({1, 0, 0}, PI)}, 1};

		MyDrawPolygonDisk(top, nSectors);
		MyDrawPolygonDisk(bottom, nSectors);
	}

	rlPopMatrix();
}

void MyDrawWireframeCylinder(Cylinder cylinder, int nSectors, bool drawCaps =
	false, Color color = LIGHTGRAY) {
	if (nSectors < 3) {
		return;
	}

	int numVertex = nSectors * 10;
	if (rlCheckBufferLimit(numVertex)) rlglDraw();

	rlPushMatrix();

	rlTranslatef(cylinder.ref.origin.x, cylinder.ref.origin.y, cylinder.ref.origin.z);
	Vector3 vect;
	float angle;
	QuaternionToAxisAngle(cylinder.ref.q, &vect, &angle);
	rlRotatef(angle * RAD2DEG, vect.x, vect.y, vect.z);
	rlScalef(cylinder.radius, cylinder.halfHeight, cylinder.radius);

	std::vector<Cylindrical> diskPointsAtPerim = computeDiskPoints(nSectors);

	rlBegin(RL_LINES);
	rlColor4ub(color.r, color.g, color.b, color.a);
	Vector3 carPt, carPt2;
	for (int i = 0; i < diskPointsAtPerim.size() - 1; i++) {
		carPt = CylindricalToCartesian(diskPointsAtPerim[i]);
		rlVertex3f(carPt.x, carPt.y + 1, carPt.z);
		rlVertex3f(carPt.x, carPt.y - 1, carPt.z);
		rlVertex3f(carPt.x, carPt.y - 1, carPt.z);
		carPt2 = CylindricalToCartesian(diskPointsAtPerim[i + 1]);
		rlVertex3f(carPt2.x, carPt2.y - 1, carPt2.z);
		rlVertex3f(carPt.x, carPt.y + 1, carPt.z);
		rlVertex3f(carPt2.x, carPt2.y - 1, carPt2.z);
		rlVertex3f(carPt2.x, carPt2.y - 1, carPt2.z);
		rlVertex3f(carPt2.x, carPt2.y + 1, carPt2.z);
		rlVertex3f(carPt2.x, carPt2.y + 1, carPt2.z);
		rlVertex3f(carPt.x, carPt.y + 1, carPt.z);
	}
	rlEnd();

	// Imbrication de la transformation dans l'espace
	if (drawCaps) {
		Disk top = { {{0, 1, 0},  QuaternionIdentity()}, 1 };
		Disk bottom = { {{0, -1, 0}, QuaternionFromAxisAngle({1, 0, 0}, PI)}, 1 };

		MyDrawWireframeDisk(top, nSectors, color);
		MyDrawWireframeDisk(bottom, nSectors, color);
	}

	rlPopMatrix();
}

void MyDrawCylinder(Cylinder cylinder, int nSectors, bool drawCaps = false, bool drawPolygon = true, bool drawWireframe = true,
	Color polygonColor = LIGHTGRAY, Color wireframeColor = DARKGRAY) {
	if (drawPolygon) MyDrawPolygonCylinder(cylinder, nSectors, drawCaps, polygonColor);
	if (drawWireframe) MyDrawWireframeCylinder(cylinder, nSectors, drawCaps, wireframeColor);
}

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

void MyDrawPolygonCapsule(Capsule capsule, int nSectors, int nParallels, Color
	color = LIGHTGRAY)
{
	rlPushMatrix();

	rlTranslatef(capsule.ref.origin.x, capsule.ref.origin.y, capsule.ref.origin.z);
	Vector3 vect;
	float angle;
	QuaternionToAxisAngle(capsule.ref.q, &vect, &angle);
	rlRotatef(angle * RAD2DEG, vect.x, vect.y, vect.z);
	rlScalef(capsule.radius, capsule.halfHeight, capsule.radius);

	Vector3 bottom = { 0, -1, 0 };
	Vector3 top = { 0, 1, 0 };

	Cylinder cyl = { ReferenceFrame(), 1, 1 };
	MyDrawPolygonCylinder(cyl, nSectors, false, color);

	// On ne souhaite pas que les sphères soient ovales à cause du redimentionnement de rlScalef().
	// On crée alors une nouvelle matrice, identique à la précédente, à l'exception qu'elle n'inclut pas
	// de transformation de scaling
	rlPopMatrix();
	rlPushMatrix();
	rlTranslatef(capsule.ref.origin.x, capsule.ref.origin.y, capsule.ref.origin.z);
	rlRotatef(angle * RAD2DEG, vect.x, vect.y, vect.z);

	Sphere sphereBottom = { ReferenceFrame({ 0, -capsule.halfHeight, 0 }, QuaternionIdentity()), capsule.radius };
	Sphere sphereTop = { ReferenceFrame({ 0, capsule.halfHeight, 0 }, QuaternionIdentity()), capsule.radius };
	MyDrawPolygonSphere(sphereTop, nSectors, nParallels, color);
	MyDrawPolygonSphere(sphereBottom, nSectors, nParallels, color);

	rlPopMatrix();


}
#pragma endregion;

#pragma region Intersection;
bool IntersectLinePlane(Line line, Plane plane, float& t, Vector3& interPt, Vector3&
	interNormal)
{
	// no intersection if line is parallel to the plane
	float dotProd = Vector3DotProduct(plane.normal, line.dir);
	if (fabsf(dotProd) < EPSILON) return false;
	// intersection: t, interPt & interNormal
	t = (plane.d - Vector3DotProduct(plane.normal, line.pt)) / dotProd;
	interPt = Vector3Add(line.pt, Vector3Scale(line.dir, t)); // OM = OA+tAB
	interNormal = Vector3Scale(plane.normal,
		Vector3DotProduct(Vector3Subtract(line.pt, interPt), plane.normal) < 0 ? -1.f : 1.f);
	return true;
}

bool IntersectSegmentPlane(Segment seg, Plane plane, float& t, Vector3& interPt,
	Vector3& interNormal) {
	Vector3 dir = Vector3Subtract(seg.pt1, seg.pt2);			// peut importe l'ordre de pt1 et pt2 dans la soustr
	float dotProd = Vector3DotProduct(plane.normal, dir);
	if (fabsf(dotProd) < EPSILON) return false;

	t = fabsf((plane.d - Vector3DotProduct(plane.normal, seg.pt1)) / dotProd);
	//std::cout << "t=" << t << "\n";
	if (t < 0 || t > 1) return false;
	interPt = Vector3Subtract(seg.pt1, Vector3Scale(dir, t)); // OM = OA+tAB
	interNormal = Vector3Scale(plane.normal,
		Vector3DotProduct(Vector3Subtract(seg.pt1, interPt), plane.normal) < 0 ? -1.f : 1.f);
	return true;
}

/// <summary>
/// Indique s'il y a intersection entre le segment et le Quad donnés
/// </summary>
/// <param name="seg">Le segment à tester</param>
/// <param name="quad">Le Quad à tester</param>
/// <param name="t">Si intersection detecté, indique la position du point d'intersection dans le segment. t est compris entre 0 et 1</param>
/// <param name="interPt">Si intersection detecté, indique la position du point d'intersection dans le monde</param>
/// <param name="interNormal">Si intersection detecté, indique le vecteur normal à la surface de la primitive, où est localisé interPt</param>
/// <returns>True si l'intersection existe, False sinon</returns>
bool IntersectSegmentQuad(Segment seg, Quad quad, float& t, Vector3& interPt, Vector3& interNormal) {
	Plane superimposedPlane;		
	Vector3 n = quad.ref.j;	
	float d = Vector3DotProduct(n, quad.ref.origin);
	superimposedPlane = { n, d};

	bool isIntersection = IntersectSegmentPlane(seg, superimposedPlane, t, interPt, interNormal);
	if (isIntersection) {
		Vector3 localPos = GlobalToLocalPos(interPt, quad.ref);
		return ((fabsf(localPos.x) <= quad.extents.x) && (fabsf(localPos.z) <= quad.extents.z));
	}
	else
		return false;
}

/// <summary>
/// Blabla
/// Méthode utilisé également pour le test de l'OBB.
/// </summary>
/// <param name="seg"></param>
/// <param name="box"></param>
/// <param name="t"></param>
/// <param name="interPt"></param>
/// <param name="interNormal"></param>
/// <returns></returns>
bool IntersectSegmentBox(Segment seg, Box box, float& t, Vector3& interPt,
	Vector3& interNormal) {

	// La Map permet de stocker les Quads de la Box ainsi que leur distance entre leur origines et le pt1 du segment.
	// Au moment du test d'intersection, le Quad qui a la plus courte distance sera traité en premier, et celui qui a la plus longue en dernier.
	std::map<float, Quad> boxQuads;
	float piOn2 = PI / 2;
	Vector3 originForNextQuad;
	float distanceQuadPt1ForNextQuad;
	
	// Construction des 6 Quads qui repésentent la Box
	Vector3 extentsForQuadsOnX = { box.extents.y, 0, box.extents.z };
	originForNextQuad = Vector3Add(box.ref.origin, Vector3Scale(box.ref.i, box.extents.x));
	distanceQuadPt1ForNextQuad = Vector3Distance(seg.pt1, originForNextQuad);
	Quad sideNormalToXPositive = { {originForNextQuad, QuaternionMultiply(box.ref.q, QuaternionFromAxisAngle({0, 0, 1}, -piOn2))}, extentsForQuadsOnX };
	boxQuads.insert(std::pair<float, Quad>(distanceQuadPt1ForNextQuad, sideNormalToXPositive));

	originForNextQuad = Vector3Subtract(box.ref.origin, Vector3Scale(box.ref.i, box.extents.x));
	distanceQuadPt1ForNextQuad = Vector3Distance(seg.pt1, originForNextQuad);
	Quad sideNormalToXNegative = { {originForNextQuad, QuaternionMultiply(box.ref.q, QuaternionFromAxisAngle({0, 0, 1}, piOn2))}, extentsForQuadsOnX };
	boxQuads.insert(std::pair<float, Quad>(distanceQuadPt1ForNextQuad, sideNormalToXNegative));

	Vector3 extentsForQuadsOnY = { box.extents.x, 0, box.extents.z };
	originForNextQuad = Vector3Add(box.ref.origin, Vector3Scale(box.ref.j, box.extents.y));
	distanceQuadPt1ForNextQuad = Vector3Distance(seg.pt1, originForNextQuad);
	Quad sideNormalToYPositive = { {originForNextQuad, box.ref.q}, extentsForQuadsOnY };
	boxQuads.insert(std::pair<float, Quad>(distanceQuadPt1ForNextQuad, sideNormalToYPositive));

	originForNextQuad = Vector3Subtract(box.ref.origin, Vector3Scale(box.ref.j, box.extents.y));
	distanceQuadPt1ForNextQuad = Vector3Distance(seg.pt1, originForNextQuad);
	Quad sideNormalToYNegative = { {originForNextQuad, QuaternionMultiply(box.ref.q, QuaternionFromAxisAngle({0, 0, 1}, PI))}, extentsForQuadsOnY };
	boxQuads.insert(std::pair<float, Quad>(distanceQuadPt1ForNextQuad, sideNormalToYNegative));

	Vector3 extentsForQuadsOnZ = { box.extents.x, 0, box.extents.y };
	originForNextQuad = Vector3Add(box.ref.origin, Vector3Scale(box.ref.k, box.extents.z));
	distanceQuadPt1ForNextQuad = Vector3Distance(seg.pt1, originForNextQuad);
	Quad sideNormalToZPositive = { {originForNextQuad, QuaternionMultiply(box.ref.q, QuaternionFromAxisAngle({1, 0, 0}, piOn2))}, extentsForQuadsOnZ };
	boxQuads.insert(std::pair<float, Quad>(distanceQuadPt1ForNextQuad, sideNormalToZPositive));

	originForNextQuad = Vector3Subtract(box.ref.origin, Vector3Scale(box.ref.k, box.extents.z));
	distanceQuadPt1ForNextQuad = Vector3Distance(seg.pt1, originForNextQuad);
	Quad sideNormalToZNegative = { {originForNextQuad, QuaternionMultiply(box.ref.q, QuaternionFromAxisAngle({1, 0, 0}, -piOn2))}, extentsForQuadsOnZ };
	boxQuads.insert(std::pair<float, Quad>(distanceQuadPt1ForNextQuad, sideNormalToZNegative));

	for (std::map<float, Quad>::iterator it = boxQuads.begin(); it != boxQuads.end(); ++it) {

		if (IntersectSegmentQuad(seg, it->second, t, interPt, interNormal)) {
			return true;
		}
	}

	return false;
}

// réparer la signature de la méthode (* -> &) pour les pts de qualité du code
bool IntersectSegmentSphere(Segment seg, Sphere sph, float& t, Vector3& interPt, Vector3& interNormal) {

	Vector3 vecteurAB = Vector3Subtract(seg.pt2, seg.pt1); // vecteur AB
	Vector3 vecteur2AB = Vector3Scale(vecteurAB, 2.0); // vecteur 2AB
	Vector3 vecteurOmegaA = Vector3Subtract(seg.pt1, sph.ref.origin); // vecteur Oméga A
	float omegaADotOmegaA = Vector3DotProduct(vecteurOmegaA, vecteurOmegaA); // Oméga A . Oméga A
	float rAuCarre = pow(sph.radius, 2.0);

	float a = Vector3DotProduct(vecteurAB, vecteurAB); // a = AB . AB

	float b = Vector3DotProduct(vecteur2AB, vecteurOmegaA); // b = 2AB . Oméga A

	float c = omegaADotOmegaA - rAuCarre; // c = OmégaA² - r²

	float delta = pow(b, 2.0) - (4 * a * c);

	if (delta < 0) return false;

	t = (-b - sqrtf(delta)) / (2 * a);

	std::cout << "t=" << t << "\n";

	if (t < 0 || t > 1) return false;

	interPt = Vector3Add(seg.pt1, Vector3Scale(vecteurAB, t));

	interNormal = Vector3Subtract(sph.ref.origin, interPt);

	return true;
}

// todo : rendre fonctionnel l'argument t
// ptet aussi que y'a moyen de la faire fonctionner avec plane
/*
bool IntersectSegmentDisk(Segment seg, Disk disk, float& t, Vector3& interPt, Vector3& interNormal) {
	float distanceDiskOrigin = Vector3Distance({ 0,0,0 }, disk.ref.origin);
	Plane planeOfDisk = { disk.ref.j, distanceDiskOrigin };

	if (IntersectSegmentQuad(seg, quadOnDisk, interPt, interNormal)) {
		if (Vector3Distance(plane.position, interPt) <= disk.radius * .5f) {
			return true;
		}
	}

	return false;
}
*/

// penser à tester l'argument t
// Attention : l'intersection avec les chapeaux ne marchent pas (après ca se trouve c'est même pas à faire..)
bool IntersectSegmentCylinder(Segment seg, Cylinder cyl, float& t, Vector3& interPt, Vector3& interNormal) {

	Vector3 cylDir = Vector3RotateByQuaternion({ 0,1,0 }, cyl.ref.q);

	Vector3 A = Vector3Add(cyl.ref.origin, Vector3Scale(cylDir, -cyl.halfHeight));		// bas du cyl
	Vector3 B = Vector3Add(cyl.ref.origin, Vector3Scale(cylDir, cyl.halfHeight));		// haut du cyl
	float r = cyl.radius;			// rayon
	Vector3 start = seg.pt1;
	Vector3 dir = Vector3Normalize(Vector3Subtract(seg.pt2, seg.pt1));

	double cxmin, cymin, czmin, cxmax, cymax, czmax;
	if (A.z < B.z) {
		czmin = A.z - r; czmax = B.z + r;
	}
	else {
		czmin = B.z - r; czmax = A.z + r;
	}
	if (A.y < B.y) {
		cymin = A.y - r; cymax = B.y + r;
	}
	else {
		cymin = B.y - r; cymax = A.y + r;
	}
	if (A.x < B.x) {
		cxmin = A.x - r; cxmax = B.x + r;
	}
	else {
		cxmin = B.x - r; cxmax = A.x + r;
	}

	Vector3 AB = Vector3Subtract(B, A);
	Vector3 AO = Vector3Subtract(start, A);
	Vector3 AOxAB = Vector3CrossProduct(AO, AB);
	Vector3 VxAB = Vector3CrossProduct(dir, AB);
	double ab2 = Vector3DotProduct(AB, AB);
	double a = Vector3DotProduct(VxAB, VxAB);
	double b = 2 * Vector3DotProduct(VxAB, AOxAB);
	double c = Vector3DotProduct(AOxAB, AOxAB) - (r * r * ab2);
	double d = b * b - 4 * a * c;
	if (d <= 0) return false;
	t = (-b - sqrt(d)) / (2 * a);		// tester
	//std::cout << "t=" << t << "\n";

	if (time < 0) return false;

	interPt = Vector3Add(start, Vector3Scale(dir, t));
	Vector3 projection = Vector3Add(A, Vector3Scale(AB, (Vector3DotProduct(AB, Vector3Subtract(interPt, A)) / ab2)));
	if (Vector3Length(Vector3Subtract(projection, A)) + Vector3Length(Vector3Subtract(B, projection)) > Vector3Length(AB) || Vector3Distance(start, interPt) > Vector3Distance(seg.pt1, seg.pt2)) return false;
	interNormal = Vector3Normalize(Vector3Subtract(interPt, projection));
	return true;
}

/// <summary>
/// Indique si il y a intersection entre un segment et une capsule
/// </summary>
/// <param name="seg"></param>
/// <param name="capsule"></param>
/// <param name="t"></param>
/// <param name="interPt"></param>
/// <param name="interNormal"></param>
/// <returns>True pour intersection, False sinon</returns>
bool IntersectSegmentCapsule(Segment seg, Capsule capsule, float& t, Vector3& interPt, Vector3& interNormal) {
	Vector3 up = LocalToGlobalPos({ 0, capsule.halfHeight, 0 }, capsule.ref);
	Vector3 down = LocalToGlobalPos({ 0, - capsule.halfHeight, 0 }, capsule.ref);

	ReferenceFrame ref = ReferenceFrame(capsule.ref.origin, capsule.ref.q);
	Cylinder cylinder = { ref, capsule.halfHeight, capsule.radius };
	Sphere sphereUp = { {up, QuaternionIdentity() }, capsule.radius };
	Sphere sphereDown = { {down, QuaternionIdentity()}, capsule.radius };

	Vector3 interPtCurrent;
	Vector3 interNormalCurrent;
	float tmpT;

	// OBB
	ReferenceFrame refObb = ReferenceFrame(capsule.ref.origin, capsule.ref.q);
	Vector3 extentsObb = {capsule.radius, capsule.halfHeight + capsule.radius, capsule.radius};
	Box obb = { refObb, extentsObb};
	if (!IntersectSegmentBox(seg, obb, t, interPt, interNormal))
		return false;

	// Un autre système plus léger pour définir le premier point d'intersection rencontré par le segment.
	// Pour une intersection trouvée, on vérifie si la distance entre le pt1 du segment et le point
	// d'intersection trouvé est inférieure à celle avec le dernier point d'intersection déterminé (interPt).
	// On initialise interPt avec FLT_MAX, de manière à ce qu'il soit le plus éloigné possible
	bool isInter = false;
	bool isInterCurrent;
	interPt = { FLT_MAX, FLT_MAX, FLT_MAX };

	isInterCurrent = IntersectSegmentCylinder(seg, cylinder, tmpT, interPtCurrent, interNormalCurrent);
	if (isInterCurrent && Vector3Distance(interPtCurrent, seg.pt1) < Vector3Distance(interPt, seg.pt1)) {
		interPt = { interPtCurrent.x, interPtCurrent.y, interPtCurrent.z };
		interNormal = { interNormalCurrent.x, interNormalCurrent.y, interNormalCurrent.z };
		isInter = true;
	}

	isInterCurrent = IntersectSegmentSphere(seg, sphereUp, tmpT, interPtCurrent, interNormalCurrent);
	if (isInterCurrent && Vector3Distance(interPtCurrent, seg.pt1) < Vector3Distance(interPt, seg.pt1)) {
		interPt = { interPtCurrent.x, interPtCurrent.y, interPtCurrent.z };
		interNormal = { interNormalCurrent.x, interNormalCurrent.y, interNormalCurrent.z };
		isInter = true;
	}

	isInterCurrent = IntersectSegmentSphere(seg, sphereDown, tmpT, interPtCurrent, interNormalCurrent);
	if (isInterCurrent && Vector3Distance(interPtCurrent, seg.pt1) < Vector3Distance(interPt, seg.pt1)) {
		interPt = { interPtCurrent.x, interPtCurrent.y, interPtCurrent.z };
		interNormal = { interNormalCurrent.x, interNormalCurrent.y, interNormalCurrent.z };
		isInter = true;
	}

	return isInter;
}

#pragma endregion;

#pragma region Camera;
void MyUpdateOrbitalCamera(Camera* camera, float deltaTime)
{
	static Spherical sphPos = { 10.0f, PI / 4.0f, PI / 88.0f };

	Spherical sphSpeed = { 4.0f, 0.08f, 0.08f };
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
#pragma endregion;

int main(int argc, char* argv[])
{
	// Initialization
	//--------------------------------------------------------------------------------------
	float screenSizeCoef = .9f;
	const int screenWidth = 1920 * screenSizeCoef;
	const int screenHeight = 1080 * screenSizeCoef;
	InitWindow(screenWidth, screenHeight, "ESIEE - E3FI - 2022 - 2023 - Maths 3D");
	SetTargetFPS(60);

	// CAMERA
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
			DrawGrid(20, 1.0f);
			DrawLine3D({ 0 }, { 0,10,0 }, DARKGRAY);
			DrawSphere({ 10,0,0 }, .2f, RED);
			DrawSphere({ 0,10,0 }, .2f, GREEN);
			DrawSphere({ 0,0,10 }, .2f, BLUE);

			ReferenceFrame refBase = ReferenceFrame(
				{ 0, 0, 0 },
				QuaternionIdentity()
			);
			ReferenceFrame ref1 = ReferenceFrame(
				{ 0,2,0 },
				QuaternionFromAxisAngle(Vector3Normalize({ 1,1,1 }), PI / 4)
			);
			ReferenceFrame ref2 = ReferenceFrame(
				{3, 1, 0},
				QuaternionFromAxisAngle(Vector3Normalize({ 1,0,0 }), PI / 3)
			);
			ReferenceFrame ref2QReversed = ReferenceFrame(
				{ 3, 1, 0 },
				QuaternionFromAxisAngle(Vector3Normalize({ 1,0,0 }), PI / 3 + PI)
			);
			ReferenceFrame ref3 = ReferenceFrame(
				{ 0, 0, 0 },
				QuaternionFromAxisAngle(Vector3Normalize({ 1,0,0 }), PI / 2)
			);
			ReferenceFrame ref4 = ReferenceFrame(
				{ 0, 0, 2 },
				QuaternionIdentity()
			);
			ReferenceFrame refALouest = ReferenceFrame(
				{ 0, 0, 20 },
				QuaternionIdentity()
			);
			ReferenceFrame ref6 = ReferenceFrame(
				{ 6, 0, 4 },
				QuaternionIdentity()
			);


			// TESTS AFFICHAGE PRIMITIVES 3D
			// Pour tester, décommenter les parties de code suivantes
			// QUAD DISPLAY TEST
			//Quad quad = { ref1,{4,0,4} };
			//MyDrawQuad(quad);

			// BOX DISPLAY TEST
			//Box box = { ref2, {4, 4, 8} };
			//MyDrawBox(box, true, true);

			// DISK DISPLAY TEST
			//Disk d = { ref2, 5 };
			//MyDrawDisk(d, 30);

			// CYLINDER DISPLAY TEST
			//Cylinder c = { ref3, 2, 1};
			//MyDrawCylinder(c, 10, true);

			// SPHERE DISPLAY TEST
			//Sphere s = { ref2, 4 };
			//MyDrawPolygonSphere(s, 10, 10);

			// CAPSULE DISPLAY TEST
			//Capsule cap = { ref3, 3, 1 };
			//MyDrawPolygonCapsule(cap, 15, 10);

			
			//TESTS INTERSECTIONS
			Vector3 interPt;
			Vector3 interNormal;
			float t;

			// THE SEGMENT (on ne peut pas dessiner de droite avec raylib, c'est pour ca qu'on créer un segment)
			// une expression entre acollades sigifie qu'un nouvel objet du bon type est crée (comme new en java)
			Segment segment = { {-5,8,0},{5,-8,3} };
			DrawLine3D(segment.pt1, segment.pt2, BLACK);
			MyDrawPolygonSphere({ {segment.pt1,QuaternionIdentity()},.15f }, 16, 8, RED);
			MyDrawPolygonSphere({ {segment.pt2,QuaternionIdentity()},.15f }, 16, 8, GREEN);

			// TEST LINE PLANE INTERSECTION
			//Plane plane = { Vector3RotateByQuaternion({0,1,0}, QuaternionFromAxisAngle({1,0,0},time
			//* .5f)), 2 };
			//// (on ne peut pas dessiner un plan avec raylib, du coup on rpz ca avec un quad)
			//ReferenceFrame refQuad = { Vector3Scale(plane.normal, plane.d), QuaternionFromVector3ToVector3({0,1,0},plane.normal) };
			//Quad quad = { refQuad,{10,1,10} };
			//MyDrawQuad(quad);
			//Line line = { segment.pt1,Vector3Subtract(segment.pt2,segment.pt1) };

			//if (IntersectLinePlane(line, plane, t, interPt, interNormal))
			//{
			//	MyDrawPolygonSphere({ {interPt,QuaternionIdentity()},.1f }, 16, 8, RED);
			//	DrawLine3D(interPt, Vector3Add(Vector3Scale(interNormal, 1), interPt), RED);
			//}
			// 
			//// TEST SEGM PLANE INTERSECTION
			/*if (IntersectSegmentPlane(segment, plane, t, interPt, interNormal))
			{
				MyDrawPolygonSphere({ {interPt,QuaternionIdentity()},.1f }, 16, 8, RED);
				DrawLine3D(interPt, Vector3Add(Vector3Scale(interNormal, 1), interPt), RED);
			}*/

			// TEST SEGM QUAD INTERSECTION
			//Quad quad = { refBase, {10, 0, 2} };
			//MyDrawQuad(quad);
			//if (IntersectSegmentQuad(segment, quad, t, interPt, interNormal)) {
			//	MyDrawPolygonSphere({ {interPt,QuaternionIdentity()},.1f }, 16, 8, RED);
			//	DrawLine3D(interPt, Vector3Add(Vector3Scale(interNormal, 1), interPt), RED);
			//}

			// TEST SEGM BOX INTERSECTION
			//Box box = { refBase, {3, 6, 4} };
			//MyDrawBox(box);
			//if (IntersectSegmentBox(segment, box, t, interPt, interNormal)) {
			//	MyDrawPolygonSphere({ {interPt,QuaternionIdentity()},.1f }, 16, 8, RED);
			//	DrawLine3D(interPt, Vector3Add(Vector3Scale(interNormal, 1), interPt), RED);
			//}

			// TEST SEGM SPHERE INTERSECTION
			//Sphere s = { refBase, 4 };
			//MyDrawPolygonSphere(s, 10, 10);
			//if (IntersectSegmentSphere(segment, s, t, interPt, interNormal)) {
			//	MyDrawPolygonSphere({ {interPt,QuaternionIdentity()},.1f }, 16, 8, RED);
			//	DrawLine3D(interPt, Vector3Add(Vector3Scale(interNormal, 1), interPt), RED);
			//}

			// TEST SEGM CYLINDER INTERSECTION
			/*
			Cylinder c = { ref3, 4, 2};
			MyDrawCylinder(c, 10, true);
			if (IntersectSegmentCylinder(segment, c, t, interPt, interNormal)) {
				MyDrawPolygonSphere({ {interPt,QuaternionIdentity()},.1f }, 16, 8, RED);
				DrawLine3D(interPt, Vector3Add(Vector3Scale(interNormal, 1), interPt), RED);
			}
			*/

			// TEST SEGM CAPSULE INTERSECTION
			//Capsule cap = { ref2, 8, 5 };
			//MyDrawPolygonCapsule(cap, 15, 15);
			//Segment segment2 = { {2,15,15},{3,-15,-15} };
			//DrawLine3D(segment2.pt1, segment2.pt2, BLACK);
			//MyDrawPolygonSphere({ {segment2.pt1,QuaternionIdentity()},.15f }, 16, 8, RED);
			//MyDrawPolygonSphere({ {segment2.pt2,QuaternionIdentity()},.15f }, 16, 8, GREEN);
			//if (IntersectSegmentCapsule(segment2, cap, t, interPt, interNormal)) {
			//	MyDrawPolygonSphere({ {interPt,QuaternionIdentity()},.1f }, 16, 8, RED);
			//	DrawLine3D(interPt, Vector3Add(Vector3Scale(interNormal, 1), interPt), RED);
			//}
			

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