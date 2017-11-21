#pragma once
#include "scene.h"
#include "objects.h"
#include "tinyxml.h"



Node rootNode;
Camera camera;
RenderImage renderImage;
Sphere theSphere;
LightList lights;
MaterialList materials;
Plane thePlane;
ObjFileList objList;
TexturedColor background;
TexturedColor environment;
TextureList textureList;


const int samplePerPixel = 128;
const int minsamplePerPixel = 32;


int rendID = 0;

