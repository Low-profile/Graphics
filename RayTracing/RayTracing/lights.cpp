#include "lights.h"
#include <iostream>

const int samplePerShadowRay = 4;

extern Node rootNode;

bool ShadowTraceNode(const Node &node, const Ray &r, HitInfo &hInfo, float t_max)
{
	float t_bias = 0.0001;
	Ray curNodeRay = node.ToNodeCoords(r);;
	if (node.GetNodeObj() != nullptr)
	{
		if (node.GetNodeObj()->IntersectRay(curNodeRay, hInfo))
		{
			if ((hInfo.z<t_max) && (hInfo.z > t_bias))
				return true;
		}
	}
	for (int childIndex = 0; childIndex < node.GetNumChild(); childIndex++)
	{
		if (ShadowTraceNode(*node.GetChild(childIndex), curNodeRay, hInfo, t_max))
			return true;
	}
	return false;
}

bool ShadowTrace(const Ray &r, HitInfo &hInfo, float t_max)
{
	return ShadowTraceNode(rootNode, r, hInfo, t_max);
}

float GenLight::Shadow(Ray ray, float t_max)
{
	HitInfo hit_Info;
	if (ShadowTrace(ray, hit_Info, t_max))
	{
		return 0;
	}
	return 1;
}

Color PointLight::Illuminate(const Point3 &p, const Point3 &N) const
{
	float total = 0;
	for (int sample_i = 0; sample_i < samplePerShadowRay; sample_i++)
	{
		float c_r = size * sqrt(static_cast <float> (rand()) / static_cast <float> (RAND_MAX));
		float c_theta = static_cast <float> (rand()) / (static_cast <float> (RAND_MAX / (2 * M_PI)));

		float c_x = c_r * cos(c_theta);
		float c_y = c_r * sin(c_theta);

		Point3 l_rand = Point3(static_cast <float> (rand()) / static_cast <float> (RAND_MAX), static_cast <float> (rand()) / static_cast <float> (RAND_MAX), static_cast <float> (rand()) / static_cast <float> (RAND_MAX));
		while (l_rand % (position - p) == 0)
			l_rand = Point3(static_cast <float> (rand()) / static_cast <float> (RAND_MAX), static_cast <float> (rand()) / static_cast <float> (RAND_MAX), static_cast <float> (rand()) / static_cast <float> (RAND_MAX));
		Point3 l_x = (position - p) ^ l_rand;
		Point3 l_y = (position - p) ^ l_x;

		Point3 light_p = position + l_x.GetNormalized() * c_x + l_y.GetNormalized() *c_y;
		total += Shadow(Ray(p, light_p - p), 1);
	}
	total /= samplePerShadowRay;
	return total * intensity;
}