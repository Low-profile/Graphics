#pragma once

#define GAMMA_CORRECTION

#include "global.h"
#include <ctime>
#include <iostream>
#include <thread>
#include <list>
#include <string>


class PixelIterator
{
private:
	std::atomic<int> ix;
	int width;
	int height;
public:
	int rid;
	void Init() {
		ix = 0;
		rid = 1;
		width = renderImage.GetWidth();
		height = renderImage.GetHeight();
	}
	bool getPixel(int &x, int &y);
};

bool PixelIterator::getPixel(int &x, int &y)
{
	int i = ix++;
	if (i >= width*height)
		return false;
	x = i % width;
	y = i / width;
	return true;
}

clock_t begin;
PixelIterator pixelIt;

//loading functions
int LoadScene(const char *filename);
void BeginRender();	// Called to start rendering (renderer must run in a separate thread)
void StopRender();	// Called to end rendering (if it is not already finished)
void ShowViewport();

bool TraceNode(const Node &node, const Ray &r, HitInfo &hInfo);

bool Trace(const Ray &r, HitInfo &hInfo);

void TraceToNode(const Node &node, const Ray &r, HitInfo &hInfo, const Node &findingNode);

void RenderPixel(PixelIterator &it, int rid)
{
	int i, j;
	while (it.getPixel(j, i))
	{
		if (i == 599 & j == 799)
		{
			std::cout << i << "\t" << j << std::endl;
			clock_t end = clock();
			double elapsed_secs = double(end - begin) / CLOCKS_PER_SEC;

			std::cout << elapsed_secs << std::endl;
		}

		int imgHeight = renderImage.GetHeight();
		int imgWidht = renderImage.GetWidth();
		float aspectRatio = 1.0f * imgWidht / imgHeight;
		float l = camera.focaldist;
		float thetaFov = camera.fov;
		float h = tan(thetaFov / 2 * M_PI / 180) * 2 * l;
		float w = aspectRatio * h;

		Point3 C = camera.pos;
		Point3 Z = -camera.dir;
		Point3 Y = camera.up;
		Point3 X = Y ^ Z;

		Point3 B = C + l * (-Z) + h / 2 * Y;
		Point3 A = B + w / 2 * (-X);

		Point3 U = X * w / (imgWidht * 1.0f);
		Point3 V = (-Y) * h / (imgHeight * 1.0f);



		std::vector<Color> shading_v = {};
		float color_sum = 0;
		float c_iSquareSum = 0;
		float c_bar = 0;


		for (int sample_i = 1; sample_i < samplePerPixel + 1; sample_i++)
		{

			if (sample_i > minsamplePerPixel)
			{
				if (c_iSquareSum / shading_v.size() - pow(c_bar, 2) < 0.001)
				{
					break;
				}
			}


			float c_r = camera.dof * sqrt(static_cast <float> (rand()) / static_cast <float> (RAND_MAX));
			float c_theta = static_cast <float> (rand()) / (static_cast <float> (RAND_MAX / (2 * M_PI)));


			float c_x = c_r * cos(c_theta);
			float c_y = c_r * sin(c_theta);
			//float c_x = 0;
			//float c_y = 0;

			Point3 P = C + X * c_x + Y *c_y;

			float x = Halton(sample_i, 2);
			float y = Halton(sample_i, 3);
			//float x = 0.5;
			//float y = 0.5;

			Point3 Q_ij = A + (i + y)*V + (j + x)*U;
			Point3 D = (Q_ij - P) /*/ Point3(Q_ij - C).Length()*/;
			Ray ray = Ray(P, D);

			HitInfo hitInfos;
			hitInfos.Init();
			Color shading_c;
			shading_c.SetBlack();
			bool hit = Trace(ray, hitInfos);
			if (hit)
			{
				shading_c = hitInfos.node->GetMaterial()->Shade(ray, hitInfos, lights, 3);
			}
			else
			{
				shading_c = background.Sample(Point3(j*1.0 / imgWidht, i*1.0 / imgHeight, 0));
			}
			shading_v.push_back(shading_c);

			float cur_color = shading_c.r + shading_c.g + shading_c.b;
			c_iSquareSum += pow(cur_color, 2);
			color_sum += cur_color;
			c_bar = color_sum / shading_v.size();
				
		}

		Color shading_res;
		shading_res.SetBlack();
		for (int sample_i = 0; sample_i < shading_v.size(); sample_i++)
		{
			shading_res += shading_v[sample_i] / shading_v.size();
		}

		Color24 final_color(shading_res);

#ifdef GAMMA_CORRECTION
		final_color.r = 255 * pow(final_color.r / 255.f, 1 / 2.2);
		final_color.g = 255 * pow(final_color.g / 255.f, 1 / 2.2);
		final_color.b = 255 * pow(final_color.b / 255.f, 1 / 2.2);
#endif // GAMMA_CORRECTION


		renderImage.GetPixels()[i*imgWidht + j] = final_color;

		//std::cout << i*imgWidht + j << std::endl;
		renderImage.GetSampleCount()[i*imgWidht + j] = shading_v.size();
		if (rid == rendID)
			return;
	}
}

//multithread implementtion
void Render()
{
	int n = std::thread::hardware_concurrency();
	if (n <= 0)
		n = 1;

	pixelIt.Init();
	for (int i = 0; i < n; i++)
	{
		std::thread(RenderPixel, std::ref(pixelIt), 1).detach();
	}
}

void StopRender()
{
	rendID++;
}


//single thread
void BeginRender()
{
	int imgHeight = renderImage.GetHeight();
	int imgWidht = renderImage.GetWidth();
	float aspectRatio = 1.0f * imgWidht / imgHeight;
	float l = 1;
	float thetaFov = camera.fov;
	float h = tan(thetaFov / 2 * M_PI / 180) * 2 * l;
	float w = aspectRatio * h;

	Point3 C = camera.pos;
	Point3 Z = -camera.dir;
	Point3 Y = camera.up;
	Point3 X = Y ^ Z;

	Point3 B = C + l * (-Z) + h / 2 * Y;
	Point3 A = B + w / 2 * (-X);

	Point3 U = X * w / (imgWidht * 1.0f);
	Point3 V = (-Y) * h / (imgHeight * 1.0f);

	Point3 P = C;

	for (int i = 0; i < imgHeight; i++)
		for (int j = 0; j < imgWidht; j++)
		{
			std::vector<Color> shading_v;

			for (int sample_i = 1; sample_i < 4 + 1; sample_i++)
			{
				float x = Halton(sample_i, 2);
				float y = Halton(sample_i, 3);

				Point3 Q_ij = A + (i + y)*V + (j + x)*U;
				Point3 D = (Q_ij - C) / Point3(Q_ij - C).Length();
				Ray ray = Ray(P, D);

				HitInfo hitInfos;
				Color shading_c;
				shading_c.SetBlack();
				bool hit = Trace(ray, hitInfos);
				//renderImage.GetZBuffer()[i*imgWidht + j] = hitInfos.z;
				if (hit)
				{
					shading_c = hitInfos.node->GetMaterial()->Shade(ray, hitInfos, lights, 5);
				}
				else
				{
					shading_c = background.Sample(Point3(j*1.0 / imgWidht, i*1.0 / imgHeight, 0));
				}
				shading_v.push_back(shading_c);

			}


			float color_sum = 0;
			float c_iSquareSum = 0;
			for (int cur_sample_i = 0; cur_sample_i < shading_v.size(); cur_sample_i++)
			{
				float cur_color = shading_v[cur_sample_i].r + shading_v[cur_sample_i].g + shading_v[cur_sample_i].b;
				c_iSquareSum += pow(cur_color, 2);
				color_sum += cur_color;
			}
			float c_bar = color_sum / shading_v.size();

			for (int sample_i = 5; sample_i < samplePerPixel + 1; sample_i++)
			{
				if (c_iSquareSum / shading_v.size() - pow(c_bar, 2) < 0.3)
				{
					break;
				}


				float x = Halton(sample_i, 2);
				float y = Halton(sample_i, 3);

				Point3 Q_ij = A + (i + y)*V + (j + x)*U;
				Point3 D = (Q_ij - C) / Point3(Q_ij - C).Length();
				Ray ray = Ray(P, D);

				HitInfo hitInfos;
				hitInfos.Init();
				Color shading_c;
				shading_c.SetBlack();
				bool hit = Trace(ray, hitInfos);
				//renderImage.GetZBuffer()[i*imgWidht + j] = hitInfos.z;
				if (hit)
				{
					shading_c = hitInfos.node->GetMaterial()->Shade(ray, hitInfos, lights, 5);
				}
				else
				{
					shading_c = background.Sample(Point3(j*1.0 / imgWidht, i*1.0 / imgHeight, 0));
				}
				shading_v.push_back(shading_c);

				float cur_color = shading_c.r + shading_c.g + shading_c.b;
				c_iSquareSum += pow(cur_color, 2);
				color_sum += cur_color;
				c_bar = color_sum / shading_v.size();

			}

			Color shading_res;
			shading_res.SetBlack();
			for (int sample_i = 0; sample_i < shading_v.size(); sample_i++)
			{
				shading_res += shading_v[sample_i] / shading_v.size();
			}
			renderImage.GetSampleCount()[i*imgWidht + j] = shading_v.size();
			renderImage.GetPixels()[i*imgWidht + j] = Color24(shading_res);
		}
}

bool TraceNode(const Node &node, const Ray &r, HitInfo &hInfo)
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
		TraceNode(*node.GetChild(childIndex), curNodeRay, childinfos[childIndex]);
	}
	for (int i = 0; i < childinfos.size(); i++)
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

bool Trace(const Ray &r, HitInfo &hInfo)
{
	return TraceNode(rootNode, r, hInfo);
}

void TracePixel(int x, int y)
{
	int imgHeight = renderImage.GetHeight();
	int imgWidht = renderImage.GetWidth();
	float aspectRatio = 1.0f * imgWidht / imgHeight;
	float l = camera.focaldist;
	float thetaFov = camera.fov;
	float h = tan(thetaFov / 2 * M_PI / 180) * 2 * l;
	float w = aspectRatio * h;

	Point3 C = camera.pos;
	Point3 Z = -camera.dir;
	Point3 Y = camera.up;
	Point3 X = Y ^ Z;

	Point3 MIDDLE = C + l * (-Z);

	Point3 B = C + l * (-Z) + h / 2 * Y;
	Point3 A = B + w / 2 * (-X);

	Point3 U = X * w / (imgWidht * 1.0f);
	Point3 V = (-Y) * h / (imgHeight * 1.0f);

	std::vector<Color> shading_v = {};
	float color_sum = 0;
	float c_iSquareSum = 0;
	float c_bar = 0;


	for (int sample_i = 1; sample_i < samplePerPixel + 1; sample_i++)
	{

		if (sample_i > minsamplePerPixel)
		{
			if (c_iSquareSum / shading_v.size() - pow(c_bar, 2) < 0.0001)
			{
				break;
			}
		}


		float c_r = camera.dof * sqrt(static_cast <float> (rand()) / static_cast <float> (RAND_MAX));
		float c_theta = static_cast <float> (rand()) / (static_cast <float> (RAND_MAX / (2 * M_PI)));

		//if (c_r > camera.dof)
		//	std::cout << c_r;
		//if (c_theta > 2 * M_PI)
		//	std::cout << c_theta;

		float c_x = c_r * cos(c_theta);
		float c_y = c_r * sin(c_theta);
		//float c_x = 0;
		//float c_y = 0;

		Point3 P = C + X * c_x + Y *c_y;

		//float x = Halton(sample_i, 2);
		//float y = Halton(sample_i, 3);
		float s_x = 0.5;
		float s_y = 0.5;

		Point3 Q_ij = A + (y + s_y)*V + (x + s_x)*U;
		Point3 D = (Q_ij - P) /*/ Point3(Q_ij - C).Length()*/;
		Ray ray = Ray(P, D);

		HitInfo hitInfos;
		hitInfos.Init();
		Color shading_c;
		shading_c.SetBlack();
		bool hit = Trace(ray, hitInfos);
		//std::cout << hitInfos.z << std::endl;
		//std::cout << std::to_string(hitInfos.p.x) + "," + std::to_string(hitInfos.p.y) + "," + std::to_string(hitInfos.p.z) << std::endl;
		if (hit)
		{
			shading_c = hitInfos.node->GetMaterial()->Shade(ray, hitInfos, lights, 5);
		}
		else
		{
			shading_c = background.Sample(Point3(y*1.0 / imgWidht, x*1.0 / imgHeight, 0));
		}
		shading_v.push_back(shading_c);

		float cur_color = shading_c.r + shading_c.g + shading_c.b;
		c_iSquareSum += pow(cur_color, 2);
		color_sum += cur_color;
		c_bar = color_sum / shading_v.size();

	}

	Color shading_res;
	shading_res.SetBlack();
	for (int sample_i = 0; sample_i < shading_v.size(); sample_i++)
	{
		shading_res += shading_v[sample_i] / shading_v.size();
	}

	renderImage.GetPixels()[x*imgWidht + y] = Color24(shading_res);

	//std::cout << i*imgWidht + j << std::endl;
	renderImage.GetSampleCount()[x*imgWidht + y] = shading_v.size();

}

void TraceToNode(const Node &node, const Ray &r, HitInfo &hInfo, const Node &findingNode)
{
	Ray curNodeRay = node.ToNodeCoords(r);
	if (node.GetNodeObj() == findingNode.GetNodeObj())
	{
		node.GetNodeObj()->IntersectRay(curNodeRay, hInfo);
		return;

	}
	for (int childIndex = 0; childIndex < node.GetNumChild(); childIndex++)
	{
		TraceToNode(*node.GetChild(childIndex), curNodeRay, hInfo, findingNode);
	}
	return;
}

int main()
{
	//loaddata
	if (!LoadScene("scene.xml"))
		return false;
	//render
	begin = clock();

	Render();
	//BeginRender();//single
	//TracePixel(110, 66);
	//TracePixel(109, 66);
	//TracePixel(108, 66);
	//TracePixel(107, 66);
	//StopRender();


	int tmp;
	std::cin >> tmp;
	renderImage.SaveImage("scene_32_imp_gamma.png");
	int m = renderImage.ComputeSampleCountImage();

	renderImage.SaveSampleCountImage("scene_32_imp_gamma_sample.png");


	//renderImage.ComputeZBufferImage();
	//renderImage.SaveZImage("sceneZ2.png");





}