#pragma once

#include "objects.h"

const float t_bias = 0.001;


void swap(float& a, float& b)
{
	float temp = b;
	b = a;
	a = temp;
}

bool boxIntersectRay(const Ray &ray, const float* box, Point2& range)
{
	float p_x{ ray.p.x }, dir_x{ ray.dir.x },
		p_y{ ray.p.y }, dir_y{ ray.dir.y },
		p_z{ ray.p.z }, dir_z{ ray.dir.z };
	float t_x0 = (box[0] - p_x) / dir_x;
	float t_x1 = (box[3] - p_x) / dir_x;
	float t_y0 = (box[1] - p_y) / dir_y;
	float t_y1 = (box[4] - p_y) / dir_y;
	float t_z0 = (box[2] - p_z) / dir_z;
	float t_z1 = (box[5] - p_z) / dir_z;
	if (t_x0 > t_x1)
		swap(t_x0, t_x1);
	if (t_y0 > t_y1)
		swap(t_y0, t_y1);
	if (t_z0 > t_z1)
		swap(t_z0, t_z1);

	float t_entry = max(max(t_x0, t_y0), t_z0);
	float t_exit = min(min(t_x1, t_y1), t_z1);
	if (t_exit < 0)
		return false;
	if (t_entry <= t_exit)
	{
		range.x = t_entry;
		range.y = t_exit;
		return true;
	}
	else
		return false;
}


bool Sphere::IntersectRay(const Ray &ray, HitInfo &hInfo, int hitSide) const
{

	Point3 P = ray.p;
	Point3 D = ray.dir;
	Point3 C = { 0,0,0 };

	float r = 1;
	float a = D % D;
	float b = 2 * (P - C) % D;
	float c = (P - C) % (P - C) - r*r;
	float discriminant = b*b - 4 * a*c;


	if (discriminant < 0)
		return false;
	else if (discriminant == 0)
	{
		float root = (-b + sqrt(discriminant)) / (2 * a);
		if (root > 0)
		{
			if (root > t_bias)
			{
				hInfo.z = root;
				hInfo.p = ray.p + hInfo.z * ray.dir;
				hInfo.N = hInfo.p;
				hInfo.uvw = Point3(
					0.5 - atan2(hInfo.p.x, hInfo.p.y) / (2 * M_PI),
					0.5 + asin(hInfo.p.z) / M_PI,
					0);
				return true;
			}
			else
				return false;
		}
		else
			return false;
	}
	else
	{
		float root_1 = (-b + sqrt(discriminant)) / (2 * a);
		float root_2 = (-b - sqrt(discriminant)) / (2 * a);
		if (root_1 < 0)
		{
			if (root_2 > 0)
			{
				if (root_2 > t_bias)
				{
					hInfo.z = root_2;
					hInfo.front = false;
					hInfo.p = ray.p + hInfo.z * ray.dir;
					hInfo.N = hInfo.p;
					hInfo.uvw = Point3(
						0.5 - atan2(hInfo.p.x, hInfo.p.y) / (2 * M_PI),
						0.5 + asin(hInfo.p.z) / M_PI,
						0);
					return true;
				}
				else
					return false;
			}
			else
				return false;
		}
		else
		{
			if (root_2 < 0)
			{
				if (root_1 > t_bias)
				{
					hInfo.z = root_1;
					hInfo.p = ray.p + hInfo.z * ray.dir;
					hInfo.N = hInfo.p;
					hInfo.front = false;
					hInfo.uvw = Point3(
						0.5 - atan2(hInfo.p.x, hInfo.p.y) / (2 * M_PI),
						0.5 + asin(hInfo.p.z) / M_PI,
						0);
					return true;
				}
				else
					return false;
			}
			else
			{
				if (root_1 < root_2)
				{
					if (root_1 > t_bias)
					{
						hInfo.z = root_1;
						hInfo.p = ray.p + hInfo.z * ray.dir;
						hInfo.N = hInfo.p;
						hInfo.uvw = Point3(
							0.5 - atan2(hInfo.p.x, hInfo.p.y) / (2 * M_PI),
							0.5 + asin(hInfo.p.z) / M_PI,
							0);
						return true;
					}
					else
					{
						if (root_2 > t_bias)
						{
							hInfo.z = root_2;
							hInfo.front = false;
							hInfo.p = ray.p + hInfo.z * ray.dir;
							hInfo.N = hInfo.p;
							hInfo.uvw = Point3(
								0.5 - atan2(hInfo.p.x, hInfo.p.y) / (2 * M_PI),
								0.5 + asin(hInfo.p.z) / M_PI,
								0);
							return true;
						}
						else
							return false;
					}

				}
				else
				{
					if (root_2 > t_bias)
					{
						hInfo.z = root_2;
						hInfo.p = ray.p + hInfo.z * ray.dir;
						hInfo.N = hInfo.p;
						hInfo.uvw = Point3(
							0.5 - atan2(hInfo.p.x, hInfo.p.y) / (2 * M_PI),
							0.5 + asin(hInfo.p.z) / M_PI,
							0);
						return true;
					}
					else
					{
						if (root_1 > t_bias)
						{
							hInfo.front = false;
							hInfo.z = root_1;
							hInfo.p = ray.p + hInfo.z * ray.dir;
							hInfo.N = hInfo.p;
							hInfo.uvw = Point3(
								0.5 - atan2(hInfo.p.x, hInfo.p.y) / (2 * M_PI),
								0.5 + asin(hInfo.p.z) / M_PI,
								0);
							return true;
						}
						else
							return false;

					}
				}
			}
		}
	}
}

bool Plane::IntersectRay(const Ray &ray, HitInfo &hInfo, int hitSide) const
{
	float p_x{ ray.p.x }, dir_x{ ray.dir.x },
		p_y{ ray.p.y }, dir_y{ ray.dir.y },
		p_z{ ray.p.z }, dir_z{ ray.dir.z };
	Box curplane = GetBoundBox();
	float t_x0 = (curplane.pmin.x - p_x) / dir_x;
	float t_x1 = (curplane.pmax.x - p_x) / dir_x;
	float t_y0 = (curplane.pmin.y - p_y) / dir_y;
	float t_y1 = (curplane.pmax.y - p_y) / dir_y;
	float t_z0 = (curplane.pmin.z - p_z) / dir_z;
	float t_z1 = (curplane.pmax.z - p_z) / dir_z;
	if (t_x0 > t_x1)
		swap(t_x0, t_x1);
	if (t_y0 > t_y1)
		swap(t_y0, t_y1);
	if (t_z0 > t_z1)
		swap(t_z0, t_z1);

	float t_entry = max(max(t_x0, t_y0), t_z0);
	float t_exit = min(min(t_x1, t_y1), t_z1);

	if (t_exit < t_bias)
		return false;

	if (t_entry <= t_exit)
	{
		hInfo.z = t_entry;
		hInfo.p = ray.p + hInfo.z * ray.dir;
		hInfo.N = Point3(0, 0, 1);
		hInfo.uvw = Point3((hInfo.p.x + 1) / 2, (hInfo.p.y + 1) / 2, 0);
		return true;
	}
	else
		return false;

}

bool TriObj::IntersectRay(const Ray &ray, HitInfo &hInfo, int hitSide) const
{
	if (!GetBoundBox().IntersectRay(ray, 0))
	{
		return false;
	}


	hInfo.z = BIGFLOAT;
	int rootnodeID = bvh.GetRootNodeID();
	return TraceBVHNode(ray, hInfo, hitSide, rootnodeID);

	//hInfo.z = BIGFLOAT;
	//for (int nums = 0; nums < NF(); nums++)
	//{
	//	HitInfo tmp_HitInfo = hInfo;
	//	if (IntersectTriangle(ray, hInfo, hitSide, nums))
	//	{
	//		if (hInfo.z > tmp_HitInfo.z)
	//		{
	//			hInfo = tmp_HitInfo;
	//		}
	//	}
	//}
	//if (hInfo.z < BIGFLOAT)
	//	return true;
	//else
	//	return false;
}

bool TriObj::IntersectTriangle(const Ray &ray, HitInfo &hInfo, int hitSide, unsigned int faceID) const
{
	TriFace curFace = F(faceID);
	Point3 A = V(curFace.v[0]);
	Point3 B = V(curFace.v[1]);
	Point3 C = V(curFace.v[2]);

	Point3 N = (B - A) ^ (C - A) / ((B - A) ^ (C - A)).Length();
	float t = (A - ray.p) % N / (ray.dir%N);
	if (t < t_bias)
		return false;

	Point3 p_temp = ray.p + t * ray.dir;
	Point2 projectionRay;
	Point2 A_projection;
	Point2 B_projection;
	Point2 C_projection;
	if (N.x > N.y)
	{
		if (N.x > N.z)
		{
			projectionRay = Point2(p_temp.y, p_temp.z);
			A_projection = Point2(A.y, A.z);
			B_projection = Point2(B.y, B.z);
			C_projection = Point2(C.y, C.z);
		}
		else
		{
			projectionRay = Point2(p_temp.x, p_temp.y);
			A_projection = Point2(A.x, A.y);
			B_projection = Point2(B.x, B.y);
			C_projection = Point2(C.x, C.y);
		}

	}
	else
	{
		if (N.z > N.y)
		{
			projectionRay = Point2(p_temp.x, p_temp.y);
			A_projection = Point2(A.x, A.y);
			B_projection = Point2(B.x, B.y);
			C_projection = Point2(C.x, C.y);
		}
		else
		{
			projectionRay = Point2(p_temp.x, p_temp.z);
			A_projection = Point2(A.x, A.z);
			B_projection = Point2(B.x, B.z);
			C_projection = Point2(C.x, C.z);
		}

	}
	float S_A = (B_projection - projectionRay) ^ (C_projection - projectionRay);
	float SA_ABC = (B_projection - A_projection) ^ (C_projection - A_projection);
	float S_B = (C_projection - projectionRay) ^ (A_projection - projectionRay);
	float SB_ABC = (C_projection - B_projection) ^ (A_projection - B_projection);
	float S_C = (A_projection - projectionRay) ^ (B_projection - projectionRay);
	float SC_ABC = (A_projection - C_projection) ^ (B_projection - C_projection);


	float alpha = (S_A) / (SA_ABC);
	float beta = (S_B) / (SB_ABC);
	float gamma = (S_C) / (SC_ABC);

	if (alpha < 0 || beta < 0 || gamma < 0)
	{
		return false;
	}

	hInfo.p = GetPoint(faceID, cy::Point3f(alpha, beta, gamma));
	hInfo.N = GetNormal(faceID, cy::Point3f(alpha, beta, gamma));
	hInfo.z = t;
	hInfo.uvw = GetTexCoord(faceID, cy::Point3f(alpha, beta, gamma));

	return true;
}

bool Box::IntersectRay(const Ray &r, float t_max) const
{
	Point3 leftbottom = Corner(0);
	Point3 righttop = Corner(7);

	float p_x{ r.p.x }, dir_x{ r.dir.x },
		p_y{ r.p.y }, dir_y{ r.dir.y },
		p_z{ r.p.z }, dir_z{ r.dir.z };
	float t_x0 = (leftbottom.x - p_x) / dir_x;
	float t_x1 = (righttop.x - p_x) / dir_x;
	float t_y0 = (leftbottom.y - p_y) / dir_y;
	float t_y1 = (righttop.y - p_y) / dir_y;
	float t_z0 = (leftbottom.z - p_z) / dir_z;
	float t_z1 = (righttop.z - p_z) / dir_z;
	if (t_x0 > t_x1)
		swap(t_x0, t_x1);
	if (t_y0 > t_y1)
		swap(t_y0, t_y1);
	if (t_z0 > t_z1)
		swap(t_z0, t_z1);

	float t_entry = max(max(t_x0, t_y0), t_z0);
	float t_exit = min(min(t_x1, t_y1), t_z1);

	if (t_exit < t_bias)
		return false;

	if (t_entry <= t_exit)
	{
		return true;
	}
	else
		return false;
}

bool TriObj::TraceBVHNode(const Ray &ray, HitInfo &hInfo, int hitSide, unsigned int nodeID) const
{
	if (bvh.IsLeafNode(nodeID))
	{
		const unsigned int * curMeshes = bvh.GetNodeElements(nodeID);
		unsigned int curMeshesCounts = bvh.GetNodeElementCount(nodeID);
		for (int nums = 0; nums < curMeshesCounts; nums++)
		{
			//std::cout << curMeshes[nums] << std::endl;
			HitInfo tmp_HitInfo = hInfo;
			if (IntersectTriangle(ray, hInfo, hitSide, curMeshes[nums]))
			{
				if (hInfo.z > tmp_HitInfo.z)
				{
					hInfo = tmp_HitInfo;
				}
			}
		}
		if (hInfo.z < BIGFLOAT)
		{
			return true;
		}
		else
			return false;
	}
	else
	{
		unsigned int child_1, child_2;
		bvh.GetChildNodes(nodeID, child_1, child_2);
		Point2 child_1_range; 
		Point2 child_2_range;
		bool child_1_hit = boxIntersectRay(ray, bvh.GetNodeBounds(child_1), child_1_range);
		bool child_2_hit = boxIntersectRay(ray, bvh.GetNodeBounds(child_2), child_2_range);
		if (child_1_hit)
		{
			if (child_2_hit)
			{
				//method 1

				//if (child_1_range.y < child_2_range.y)
				//{
				//	if (child_1_range.y > child_2_range.x)
				//	{
				//		if (TraceBVHNode(ray, hInfo, hitSide, child_1))
				//		{
				//			TraceBVHNode(ray, hInfo, hitSide, child_2);
				//			return true;
				//		}
				//		else
				//		{
				//			return TraceBVHNode(ray, hInfo, hitSide, child_2);
				//		}
				//		
				//	}
				//	if (TraceBVHNode(ray, hInfo, hitSide, child_1))
				//		return true;
				//	else
				//		return TraceBVHNode(ray, hInfo, hitSide, child_2);
				//}
				//else
				//{
				//	if (child_2_range.y > child_1_range.x)
				//	{
				//		if (TraceBVHNode(ray, hInfo, hitSide, child_2))
				//		{
				//			TraceBVHNode(ray, hInfo, hitSide, child_1);
				//			return true;
				//		}
				//		else
				//		{
				//			return TraceBVHNode(ray, hInfo, hitSide, child_1);
				//		}
				//	}
				//	if (TraceBVHNode(ray, hInfo, hitSide, child_2))
				//		return true;
				//	else
				//		return TraceBVHNode(ray, hInfo, hitSide, child_1);
				//}

				//method 2

				//if (child_1_range.y < child_2_range.x)
				//{
				//	if (child_1_range.x <0)
				//		int nmb = 0;
				//	if (TraceBVHNode(ray, hInfo, hitSide, child_1))
				//		return true;
				//	else
				//		return TraceBVHNode(ray, hInfo, hitSide, child_2);
				//}
				//if (child_2_range.y < child_1_range.x)
				//{
				//	if (child_2_range.x <0)
				//		int nmb = 0;
				//	if (TraceBVHNode(ray, hInfo, hitSide, child_2))
				//		return true;
				//	else
				//		return TraceBVHNode(ray, hInfo, hitSide, child_1);
				//}
				bool intersect_child1 = TraceBVHNode(ray, hInfo, hitSide, child_1);
				bool intersect_child2 = TraceBVHNode(ray, hInfo, hitSide, child_2);
				return intersect_child1 || intersect_child2;
			}
			else
			{
				return TraceBVHNode(ray, hInfo, hitSide, child_1);
			}
		}
		else
		{
			if (child_2_hit)
			{
				return TraceBVHNode(ray, hInfo, hitSide, child_2);
			}
			else
			{
				return false;
			}
		}
	}
}