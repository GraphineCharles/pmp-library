#pragma once
#include <pmp/algorithms/DifferentialGeometry.h>
#include <pmp/algorithms/SurfaceNormals.h>
#include <unordered_set>
#include "ShortestPath.h"
#include "SurfaceSimplificationWithAncestors.h"
#include "MeshHelpers.h"
#include "PatchParameterization.h"

class PatchBaker
{
public:

	static void BakePatches(SurfaceMesh &inputMesh, const std::vector<std::vector<Face>> &patches)
	{
		SurfaceNormals::compute_vertex_normals(inputMesh);
		MeshHelpers::NormalsToBake(inputMesh);

		char namebuff[256];

		int outputSize = (int)ceil(sqrt(patches.size()));
		int resolution = 128;
		Texture2D output_p;
		output_p.Initialize(outputSize*resolution, outputSize*resolution, 64, nullptr);
		Texture2D output_n;
		output_n.Initialize(outputSize*resolution, outputSize*resolution, 64, nullptr);

		int patchCount = 0;
		for (auto faceList : patches)
		{
			int destX = (patchCount % outputSize) * resolution;
			int destY = (patchCount / outputSize) * resolution;

			// Make a mini mesh for the patch
			SurfaceMesh sub;
			MeshHelpers::ExtractMesh(inputMesh, faceList, sub);

			// Unwrap patch to unit rectangle
			PatchParameterization sp(sub);
			sp.harmonic();

			MeshHelpers::BakeMesh(sub, resolution, output_n, destX, destY);

			MeshHelpers::PositionsToBakeNormalized(sub, inputMesh.bounds());
			MeshHelpers::BakeMesh(sub, resolution, output_p, destX, destY);
			patchCount++;
		}

		sprintf(namebuff, "out_p.png");
		output_p.Save(namebuff, false);
		std::cout << "Saved bake: " << namebuff << std::endl;

		sprintf(namebuff, "out_n.png");
		output_n.Save(namebuff, false);
		std::cout << "Saved bake: " << namebuff <<std::endl;
	}
};
