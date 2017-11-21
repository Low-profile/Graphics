#pragma once

#define IMPORTANCE_SAMPLING

#include "materials.h"
#include "lights.h"
#include <list>
#include <random>
#include <iostream>

extern Node rootNode;
extern TexturedColor environment;
const int g_bounceCount = 3;
const int GI_samples = 1;

std::random_device                  rand_dev;
std::mt19937                        generator(rand_dev());
std::uniform_real_distribution<float>  dist_z(0, 1);
std::uniform_real_distribution<float>  dist_phi(0, 2 * (float)M_PI);



bool RefractionTraceNode(const Node &node, const Ray &r, HitInfo &hInfo)
{
	std::vector<HitInfo> childinfos;
	Ray curNodeRay = node.ToNodeCoords(r);
	if (node.GetNodeObj() != nullptr)
	{
		HitInfo tmp_HitInfo = hInfo;
		if (node.GetNodeObj()->IntersectRay(curNodeRay, hInfo))
		{
			if (hInfo.z > tmp_HitInfo.z)
			{
				hInfo = tmp_HitInfo;
			}
			else
			{
				hInfo.node = &node;
			}
		}
	}
	for (int childIndex = 0; childIndex < node.GetNumChild(); childIndex++)
	{
		childinfos.resize(node.GetNumChild());
		RefractionTraceNode(*node.GetChild(childIndex), curNodeRay, childinfos[childIndex]);
	}
	for (size_t i = 0; i < childinfos.size(); i++)
	{
		if (hInfo.z > childinfos[i].z)
			hInfo = childinfos[i];
	}
	node.FromNodeCoords(hInfo);
	if (hInfo.z < BIGFLOAT)
	{
		return true;
	}
	else
		return false;
}

bool RefractionTrace(const Ray &r, HitInfo &hInfo)
{
	return RefractionTraceNode(rootNode, r, hInfo);
}

bool ReflectionTraceNode(const Node &node, const Ray &r, HitInfo &hInfo, const Node& curNode)
{
	std::vector<HitInfo> childinfos;
	Ray curNodeRay = node.ToNodeCoords(r);
	if (node.GetNodeObj() != nullptr)
	{
		HitInfo tmp_HitInfo = hInfo;
		if (node.GetNodeObj()->IntersectRay(curNodeRay, hInfo))
		{
			if (hInfo.z > tmp_HitInfo.z)
			{
				hInfo = tmp_HitInfo;
			}
			else
			{
				hInfo.node = &node;
			}
		}

	}
	for (int childIndex = 0; childIndex < node.GetNumChild(); childIndex++)
	{
		childinfos.resize(node.GetNumChild());
		ReflectionTraceNode(*node.GetChild(childIndex), curNodeRay, childinfos[childIndex], curNode);
	}
	for (size_t i = 0; i < childinfos.size(); i++)
	{
		if (hInfo.z > childinfos[i].z)
			hInfo = childinfos[i];
	}
	node.FromNodeCoords(hInfo);
	if (hInfo.z < BIGFLOAT)
	{
		return true;
	}
	else
		return false;
}

bool ReflectionTrace(const Ray &r, HitInfo &hInfo, const Node& curNode)
{
	return ReflectionTraceNode(rootNode, r, hInfo, curNode);
}


Color MtlBlinn::Shade(const Ray &ray, const HitInfo &hInfo, const LightList &lights, int bounceCount) const
{
	Point3 N = hInfo.N;
	Point3 P = hInfo.p;
	Point3 V = -(ray.dir.GetNormalized());
	Color s;
	s.SetBlack();
	Color nodecolor;
	nodecolor.SetBlack();

	Color k_d = diffuse.Sample(hInfo.uvw);
	Color k_s = specular.Sample(hInfo.uvw);
	float alpha = glossiness;

	for (LightList::const_iterator it = lights.begin(); it != lights.end(); it++)
	{
		if (!((*it)->IsAmbient()))
		{
			Point3 L = -(*it)->Direction(P);
			if (L % N > 0)
			{
				Color I_i = (*it)->Illuminate(P, N);
				Point3 H_i = (V + L).GetNormalized();
				nodecolor += I_i*(k_d + k_s*(pow(N%H_i, alpha)))*(N%L);
			}
		}
		//else
		//{
		//	Color k_a = diffuse.Sample(hInfo.uvw);
		//	Color I_a = (*it)->Illuminate(P, N);
		//	nodecolor += I_a *k_a;
		//}
	}

	Color indirectionColor;
	indirectionColor.SetBlack();
	if (bounceCount == g_bounceCount)
	{
		for (int i = 0; i < GI_samples; i++)
		{
			float phi = dist_phi(generator);
			float n_z = dist_z(generator);
#ifdef IMPORTANCE_SAMPLING
			float theta = asinf(sqrt(n_z));
#else
			float theta = acosf(n_z);
#endif

			Point3 rand_vec{ 0,0,1 };
			if (abs(N % rand_vec) > (1 - 0.001))
				rand_vec = Point3(0, 1, 0);
			
			Point3 T = N ^ rand_vec;
			Point3 S = T ^ N;
			Point3 n_r = N * cos(theta) + T * sin(theta) * cos(phi) + S * sin(theta) * sin(phi);

			//float n_x = dist(generator);
			//float n_y = dist(generator);
			//float n_z = dist(generator);
			//Point3 n_r(n_x, n_y, n_z);
			//while (n_r.Length() > 1)
			//{
			//	n_x = dist(generator);
			//	n_y = dist(generator);
			//	n_z = dist(generator);
			//	n_r = Point3(n_x, n_y, n_z);
			//}
			//if (n_r % N < 0)
			//	n_r = -n_r;
			n_r = n_r.GetNormalized();


			if (n_r % N < 0)
				std::cout << " n_r % N < 0!" << std::endl;


			HitInfo hitInfo_reflect;
			Ray ray_reflect(P, n_r);
			bool hit = ReflectionTrace(ray_reflect, hitInfo_reflect, *hInfo.node);

#ifdef IMPORTANCE_SAMPLING
			float geometryTerm = 1;
#else
			float geometryTerm = N % n_r;
#endif

			if (hit)
			{
				indirectionColor += k_d * hitInfo_reflect.node->GetMaterial()->Shade(ray_reflect, hitInfo_reflect, lights, bounceCount - 1) * geometryTerm / (float)GI_samples;
			}
			else
				indirectionColor += k_d * environment.SampleEnvironment(ray_reflect.dir) * geometryTerm / (float)GI_samples;
		}
	}
	else if (bounceCount > 0)
	{
		//float n_x = dist(generator);
		//float n_y = dist(generator);
		//float n_z = dist(generator);
		//Point3 n_r(n_x, n_y, n_z);
		//while (n_r.Length() > 1)
		//{
		//	n_x = dist(generator);
		//	n_y = dist(generator);
		//	n_z = dist(generator);
		//	n_r = Point3(abs(n_x), abs(n_y), abs(n_z));
		//}
		//if (n_r % N < 0)
		//	n_r = -n_r;
		float phi = dist_phi(generator);
		float n_z = dist_z(generator);
#ifdef IMPORTANCE_SAMPLING
		float theta = asinf(sqrt(n_z));
#else
		float theta = acosf(n_z);
#endif

		Point3 rand_vec{ 0,0,1 };
		if (abs(N % rand_vec) > (1 - 0.001))
			rand_vec = Point3(0, 1, 0);

		Point3 T = N ^ rand_vec;
		Point3 S = T ^ N;
		Point3 n_r = N * cos(theta) + T * sin(theta) * cos(phi) + S * sin(theta) * sin(phi);
		n_r = n_r.GetNormalized();

		if (n_r % N <= 0)
			std::cout << " n_r % N < 0!" << std::endl;

		HitInfo hitInfo_reflect;
		Ray ray_reflect(P, n_r);
		bool hit = ReflectionTrace(ray_reflect, hitInfo_reflect, *hInfo.node);

#ifdef IMPORTANCE_SAMPLING
		float geometryTerm = 1;
#else
		float geometryTerm = N % n_r;
#endif

		if (hit)
		{
			indirectionColor += k_d * hitInfo_reflect.node->GetMaterial()->Shade(ray_reflect, hitInfo_reflect, lights, bounceCount - 1) * geometryTerm;
		}
		else
			indirectionColor += k_d * environment.SampleEnvironment(ray_reflect.dir) * geometryTerm;
	}

	//if (indirectionColor.r < 0 || indirectionColor.g < 0 || indirectionColor.b < 0)
	//	std::cout << indirectionColor.r << " " << indirectionColor.g << " " << indirectionColor.b << std::endl;

	if (refraction.GetColor() != Color(0, 0, 0))
	{
		if (hInfo.front && N % V > 0)
		{
			Color absorption_e;
			absorption_e.SetBlack();

			float x = static_cast <float> (rand()) / (static_cast <float> (RAND_MAX));
			float y = static_cast <float> (rand()) / (static_cast <float> (RAND_MAX));
			float z = static_cast <float> (rand()) / (static_cast <float> (RAND_MAX));
			Point3 n_p(x, y, z);
			while (n_p.Length() > 1)
			{
				x = static_cast <float> (rand()) / (static_cast <float> (RAND_MAX));
				y = static_cast <float> (rand()) / (static_cast <float> (RAND_MAX));
				z = static_cast <float> (rand()) / (static_cast <float> (RAND_MAX));
				n_p = Point3(x, y, z);
			}
			n_p *= refractionGlossiness;

			Point3 N_glossy = (N + n_p).GetNormalized();
			float cos_theta1 = N_glossy % V;
			while (cos_theta1 < 0)
			{
				while (n_p.Length() > 1)
				{
					x = static_cast <float> (rand()) / (static_cast <float> (RAND_MAX));
					y = static_cast <float> (rand()) / (static_cast <float> (RAND_MAX));
					z = static_cast <float> (rand()) / (static_cast <float> (RAND_MAX));
					n_p = Point3(x, y, z);
				}
				n_p *= refractionGlossiness;
				N_glossy = (N + n_p).GetNormalized();

				cos_theta1 = N_glossy % V;
			}
			Point3 S = (N_glossy ^ (N_glossy^V)) / (N_glossy^V).Length();
			//Point3 S = (N ^ (N^V)) / (N^V).Length();
			//float cos_theta1 = N % V;
			float r_0 = pow((ior - 1) / (ior + 1), 2);
			float shlicks_reflection_factor = r_0 + (1 - r_0) * pow((1 - cos_theta1), 5);
			float refraction_factor = 1 - shlicks_reflection_factor;
			float sin_theta1 = 0;
			if (pow(cos_theta1, 2) < 1)
				sin_theta1 = sqrt(1 - pow(cos_theta1, 2));
			float sin_theta2 = 1 / ior * sin_theta1;
			float cos_theta2 = sqrt(1 - pow(sin_theta2, 2));
			//Point3 T = (-N*cos_theta2 + S*sin_theta2).GetNormalized();
			Point3 T = (-N_glossy*cos_theta2 + S*sin_theta2).GetNormalized();
			HitInfo hitInfo_refract;
			Ray ray_reflect(P, T);
			bool hit = RefractionTrace(ray_reflect, hitInfo_refract);
			absorption_e = Color(exp(-hitInfo_refract.z * absorption.r), exp(-hitInfo_refract.z * absorption.g), exp(-hitInfo_refract.z * absorption.b));
			if (hit)
			{
				s += absorption_e * refraction_factor * refraction.GetColor() * hitInfo_refract.node->GetMaterial()->Shade(ray_reflect, hitInfo_refract, lights, bounceCount);
			}
			else
				s += absorption_e * refraction_factor * refraction.GetColor() * environment.SampleEnvironment(ray_reflect.dir);

			if (bounceCount > 0 && V % N >= 0)
			{
				float x = static_cast <float> (rand()) / (static_cast <float> (RAND_MAX));
				float y = static_cast <float> (rand()) / (static_cast <float> (RAND_MAX));
				float z = static_cast <float> (rand()) / (static_cast <float> (RAND_MAX));
				Point3 n_p(x, y, z);
				while (n_p.Length() > 1)
				{
					x = static_cast <float> (rand()) / (static_cast <float> (RAND_MAX));
					y = static_cast <float> (rand()) / (static_cast <float> (RAND_MAX));
					z = static_cast <float> (rand()) / (static_cast <float> (RAND_MAX));
					n_p = Point3(x, y, z);
				}
				n_p *= reflectionGlossiness;

				Point3 N_glossy = (N + n_p).GetNormalized();
				Point3 V_reflect = (2 * (V%N_glossy / (N_glossy%N_glossy)*N_glossy) - V).GetNormalized();
				while (V_reflect % N < 0)
				{
					while (n_p.Length() > 1)
					{
						x = static_cast <float> (rand()) / (static_cast <float> (RAND_MAX));
						y = static_cast <float> (rand()) / (static_cast <float> (RAND_MAX));
						z = static_cast <float> (rand()) / (static_cast <float> (RAND_MAX));
						n_p = Point3(x, y, z);
					}
					n_p *= reflectionGlossiness;
					N_glossy = (N + n_p).GetNormalized();
					V_reflect = (2 * (V%N_glossy / (N_glossy%N_glossy)*N_glossy) - V).GetNormalized();
				}
				//Point3 V_reflect = (2 * (V%N / (N%N)*N) - V).GetNormalized();
				HitInfo hitInfo_reflect;
				Ray ray_reflect(P, V_reflect);
				bool hit = ReflectionTrace(ray_reflect, hitInfo_reflect, *hInfo.node);
				if (hit)
				{
					s += refraction.GetColor() * shlicks_reflection_factor * hitInfo_reflect.node->GetMaterial()->Shade(ray_reflect, hitInfo_reflect, lights, bounceCount - 1);
				}
				else
					s += refraction.GetColor() * shlicks_reflection_factor * environment.SampleEnvironment(ray_reflect.dir);
			}

			s += nodecolor;

			return s;

		}
		else if( (!hInfo.front) && -N % V > 0)
		{
			Point3 S = (-N ^ (-N^V)) / (-N^V).Length();
			float cos_theta1 = -N % V;
			float r_0 = pow((ior - 1) / (ior + 1), 2);
			float shlicks_reflection_factor = r_0 + (1 - r_0) * pow((1 - cos_theta1), 5);
			float refraction_factor = 1 - shlicks_reflection_factor;
			float sin_theta1 = 0;
			if (pow(cos_theta1, 2) < 1)
				sin_theta1 = sqrt(1 - pow(cos_theta1, 2));
			float sin_theta2 = ior * sin_theta1;
			if (sin_theta2 > 1)
				sin_theta2 = 1;
			float cos_theta2 = 0;
			if (pow(sin_theta2, 2) < 1)
				cos_theta2 = sqrt(1 - pow(sin_theta2, 2));

			Point3 T = (N*cos_theta2 + S*sin_theta2).GetNormalized();
			HitInfo hitInfo_refract;
			Ray ray_reflect(P, T);
			bool hit = RefractionTrace(ray_reflect, hitInfo_refract);
			if (hit)
			{
				return refraction_factor * refraction.GetColor() * hitInfo_refract.node->GetMaterial()->Shade(ray_reflect, hitInfo_refract, lights, bounceCount);
			}
			else
				return refraction_factor * refraction.GetColor() * environment.SampleEnvironment(ray_reflect.dir);
		}
	}

	if (reflection.GetColor() != Color(0, 0, 0))
	{
		if (bounceCount > 0 && V % N >= 0)
		{
			float x = static_cast <float> (rand()) / (static_cast <float> (RAND_MAX));
			float y = static_cast <float> (rand()) / (static_cast <float> (RAND_MAX));
			float z = static_cast <float> (rand()) / (static_cast <float> (RAND_MAX));
			Point3 n_p(x, y, z);
			while (n_p.Length() > 1)
			{
				x = static_cast <float> (rand()) / (static_cast <float> (RAND_MAX));
				y = static_cast <float> (rand()) / (static_cast <float> (RAND_MAX));
				z = static_cast <float> (rand()) / (static_cast <float> (RAND_MAX));
				n_p = Point3(x, y, z);
			}
			n_p *= reflectionGlossiness;

			Point3 N_glossy = (N + n_p).GetNormalized();
			Point3 V_reflect = (2 * (V%N_glossy / (N_glossy%N_glossy)*N_glossy) - V).GetNormalized();
			while (V_reflect % N < 0)
			{
				while (n_p.Length() > 1)
				{
					x = static_cast <float> (rand()) / (static_cast <float> (RAND_MAX));
					y = static_cast <float> (rand()) / (static_cast <float> (RAND_MAX));
					z = static_cast <float> (rand()) / (static_cast <float> (RAND_MAX));
					n_p = Point3(x, y, z);
				}
				n_p *= reflectionGlossiness;
				N_glossy = (N + n_p).GetNormalized();
				V_reflect = (2 * (V%N_glossy / (N_glossy%N_glossy)*N_glossy) - V).GetNormalized();
			}
			//Point3 V_reflect = (2 * (V%N / (N%N)*N) - V).GetNormalized();
			HitInfo hitInfo_reflect;
			Ray ray_reflect(P, V_reflect);
			bool hit = ReflectionTrace(ray_reflect, hitInfo_reflect, *hInfo.node);
			if (hit)
			{
				s += reflection.GetColor() * hitInfo_reflect.node->GetMaterial()->Shade(ray_reflect, hitInfo_reflect, lights, bounceCount - 1);
			}
			else
				s += reflection.GetColor() * environment.SampleEnvironment(ray_reflect.dir);
		}
		else
			s += environment.SampleEnvironment(ray.dir);
	}

	s += nodecolor + indirectionColor;

	return s;
}
