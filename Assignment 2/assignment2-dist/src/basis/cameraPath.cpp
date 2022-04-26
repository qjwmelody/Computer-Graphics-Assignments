#include "cameraPath.h"
#include "extra.h"
#include <iostream>
#include <array>

using namespace FW;


Mat4f FW::cameraPath::GetOrientation(float t)
{
	if (orientationMode)
	{
		int count = orientationPoints.size();
		int start = int(t * count);

		std::array<Vec4f, 4> controlPoints = orientationPoints[start % orientationPoints.size()];

		// YOUR CODE HERE (extra)
		// Use the De Casteljau construction with spherical interpolation (slerp) to interpolate between the orientation control point
		// quaternions in the points array, and convert the interpolated quaternion to an orientation matrix.
		
		std::array<Vec4f, 4> cpLeft, cpRight;
		int p = controlPoints.size() - 1;
		std::copy(controlPoints.begin(), controlPoints.end(), cpRight.begin());
		cpLeft[0] = cpRight[0];
		// Iteration
		for (int i = 0; i < p; ++i)
		{
			for (int j = 0; j < p - i; ++j)
				cpRight[j] = 0.5f * cpRight[j] + 0.5f * cpRight[j + 1];
			cpRight[i + 1] = cpRight[0];
		}
		// Convert quaternion to matrix
		Mat4f controlPointsLeft = makeMat4f(cpLeft[0], cpLeft[1], cpLeft[2], cpLeft[3]);
		Mat4f controlPointsRight = makeMat4f(cpRight[0], cpRight[1], cpRight[2], cpRight[3]);

		//return Mat4f();
		return controlPointsRight;
	}
	else
	{
		Mat4f orientation;
		int i = int(t * positionPath.size());

		orientation.setCol(0, -Vec4f(positionPath[i].B, 0));
		orientation.setCol(1, -Vec4f(positionPath[i].N, 0));
		orientation.setCol(2, -Vec4f(positionPath[i].T, .0f));
		return orientation.transposed();
	}
}

Mat4f FW::cameraPath::GetTranslation(float t)
{
	int i = int(t * positionPath.size());

	Vec3f pos = positionPath[i].V;
	return Mat4f::translate(-pos);
}

Mat4f FW::cameraPath::GetWorldToCam(float t)
{
	return GetOrientation(t) * GetTranslation(t);
}

void FW::cameraPath::Draw(float t, GLContext* gl, Mat4f projection)
{
	mesh->draw(gl, GetWorldToCam(t), projection);
	glUseProgram(0);
}
