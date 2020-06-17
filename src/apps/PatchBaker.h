#pragma once
#include <pmp/algorithms/DifferentialGeometry.h>
#include <pmp/algorithms/SurfaceNormals.h>
#include <unordered_set>
#include "ShortestPath.h"
#include "SurfaceSimplificationWithAncestors.h"
#include "MeshHelpers.h"
#include "PatchParameterization.h"
#include "PatchQuadrangulator.h"
#include "PatchHelpers.h"

class PatchBaker
{
public:

	static void QuadPatches(std::vector<SurfaceMesh*> &patches, std::vector<PatchQuadrangulator::QuadrangularPatch*> &quadPatches)
	{
		SurfaceMesh aggregate;

		std::vector< PatchQuadrangulator::PolygonPatch> polyPatches;
		PatchQuadrangulator::SetupPolyPatches(patches, polyPatches);

		// This can run in parallel for all patches
		int patchCount = 0;
		for (auto sub : polyPatches)
		{
			std::cout << "Quadrangulate Patch." << std::endl;
			PatchQuadrangulator::QuadrangulatePatch(sub, quadPatches);
		}

		std::cout << "Quadrangulated " << patches.size() << " into " << quadPatches.size() << "\n";

		//Finish setting up neighboring information correctly, sequential
		std::cout << "Neighbour patching\n";
		PatchQuadrangulator::PatchUpNeighbours(quadPatches);

		// Assign UVs to the quadrangular patches, parallel again
		for (auto patch : quadPatches)
		{
			std::cout << "Unwrap Patch\n";

			auto tex = patch->mesh.vertex_property<TexCoord>("v:tex");
			auto points = patch->mesh.vertex_property<Point>("v:point");

			// Initialize all texture coordinates to the origin.
			for (auto v : patch->mesh.vertices())
				tex[v] = TexCoord(0.5, 0.5);

			//assert(patch->edges.size() == 4.0f);

			// Map each of the four edges uniformly
			int edgeIdx = 0;
			for (auto strip : patch->edges)
			{
				float length = 0.0f;
				for (auto edge : strip.edges)
				{
					length += distance(points[patch->mesh.from_vertex(edge)], points[patch->mesh.to_vertex(edge)]);
				}

				float dst = 0.0f;
				for (auto edge : strip.edges)
				{
					float range = dst / length;// from 0-1 over the course of the patch edge
					switch (edgeIdx)
					{
					case 0:
						tex[patch->mesh.from_vertex(edge)] = TexCoord(range, 0.0f);
						break;
					case 1:
						tex[patch->mesh.from_vertex(edge)] = TexCoord(1.0f, range);
						break;
					case 2:
						tex[patch->mesh.from_vertex(edge)] = TexCoord(1.0f - range, 1.0f);
						break;
					case 3:
						tex[patch->mesh.from_vertex(edge)] = TexCoord(0.0f, 1.0f - range);
						break;
					default:
						// invalid too many edges
						tex[patch->mesh.from_vertex(edge)] = TexCoord(0.0f, 0.0f);
					}
					dst += distance(points[patch->mesh.from_vertex(edge)], points[patch->mesh.to_vertex(edge)]);
				}

				// last vert on edge
				Halfedge last = strip.edges[strip.edges.size() - 1];
				Vertex lastV = patch->mesh.to_vertex(last);
				switch (edgeIdx)
				{
				case 0:
					tex[lastV] = TexCoord(1.0f, 0.0f);
					break;
				case 1:
					tex[lastV] = TexCoord(1.0f, 1.0f);
					break;
				case 2:
					tex[lastV] = TexCoord(0.0f, 1.0f);
					break;
				case 3:
					tex[lastV] = TexCoord(0.0f, 0.0f);
					break;
				default:
					// invalid too many edges
					tex[lastV] = TexCoord(0.0f, 0.0f);
				}
				edgeIdx++;
			}

			// Unwrap patch to unit rectangle
			PatchParameterization sp(patch->mesh);
			sp.harmonic(patch->edges.size() == 4.0f); //If not 4 edges we just map the whole boundary loop uniformly to the uv quad. This will lead to cracks but is generally robust.
			if (patch->edges.size() != 4.0f)
			{
				std::cout << "Non quad patch, " << patch->edges.size() << " sides, unwrapping full boundary.\n";
				MeshHelpers::ErrorMesh(patch->mesh, " Non quad full unwrap");
			}
		}
	}

	static void *SampleEdge(Texture2D &image, PatchQuadrangulator::QuadrangularPatch* patch, int edgeIndex, int edgeResolution, int pixelIndex)
	{
		int x, y;
		switch (edgeIndex)
		{
		case 0:
			x = patch->destX + pixelIndex;
			y = 0;// patch->destY;
			break;
		case 1:
			x = patch->destX + edgeResolution-1;
			y = /*patch->destY +*/ pixelIndex;
			break;
		case 2:
			x = patch->destX + edgeResolution-1 - pixelIndex;
			y = /*patch->destY +*/ edgeResolution-1;
			break;
		case 3:
			x = patch->destX;
			y = /*patch->destY +*/ edgeResolution-1 - pixelIndex;
			break;
		default:
			x = 0;
			y = 0;
			break;
		}

		// Sigh, patches are y flipped, blame opengl...
		y = edgeResolution - 1 - y;
		y += patch->destY;

		return image.image + (y * image.width + x) * (image.bpp / 8);
	}

	static void BakePatches(std::vector<PatchQuadrangulator::QuadrangularPatch*> &quadPatches, Texture2D *albeido,  BoundingBox &normalizationBox, const char *namePrefix)
	{
		char namebuff[256];

		int outputSize = (int)ceil(sqrt(quadPatches.size()));
		int resolution = 128;
		Texture2D output_p;
		output_p.Initialize(outputSize*resolution, outputSize*resolution, 128, nullptr);
		Texture2D output_n;
		output_n.Initialize(outputSize*resolution, outputSize*resolution, 64, nullptr);

		Texture2D output_t;
		int textureResolutionScale = 4;
		int textureResolution = resolution * textureResolutionScale;
		if (albeido)
		{
			output_t.Initialize(outputSize*textureResolution, outputSize*textureResolution, 32, nullptr);
		}

		int patchCount = 0;
		for (auto sub : quadPatches)
		{
			sub->destX = (patchCount % outputSize) * resolution;
			sub->destY = (patchCount / outputSize) * resolution;

			MeshHelpers::BackupPositions(sub->mesh);
			
			std::cout << "Bake Patch Positions\n";
			MeshHelpers::PositionsToBakeNormalized(sub->mesh, normalizationBox);
			MeshHelpers::TexCoordsToPositions(sub->mesh);
			MeshHelpers::BakeMesh(sub->mesh, NULL, resolution, output_p, sub->destX, sub->destY);

			std::cout << "Bake Patch Normals\n";
			MeshHelpers::NormalsToBake(sub->mesh);
			MeshHelpers::BakeMesh(sub->mesh, NULL, resolution, output_n, sub->destX, sub->destY);

			if (albeido)
			{
				std::cout << "Bake Patch Albedo\n";
				MeshHelpers::BakeMesh(sub->mesh, albeido, textureResolution, output_t, sub->destX * textureResolutionScale, sub->destY * textureResolutionScale);
			}

			MeshHelpers::RestorePositions(sub->mesh);

			patchCount++;
		}
		
		for (int i=0;i<quadPatches.size();i++)
		{
			auto sub = quadPatches[i];
			int edgeIndex=0;
			for (auto edge : sub->edges)
			{
				//Invalid or
				//Already set up by other neighbour
				if (edge.neighbour < 0 || edge.neighbour < sub->patchID)
				{
					edgeIndex++;
					continue; 
				}

				PatchQuadrangulator::QuadrangularPatch* otherPatch = NULL;
				for (auto other : quadPatches)
				{
					if (other->patchID == edge.neighbour)
					{
						otherPatch = other;
						break;
					}
				}

				if (otherPatch == nullptr)
				{
					edgeIndex++;
					std::cout << "Could not find neighbour patch\n";
					continue;
				}

				int otherEdgeIdx = -1;
				for (int j = 0; j < otherPatch->edges.size(); j++)
				{
					if (otherPatch->edges[j].neighbour == sub->patchID)
					{
						otherEdgeIdx = j;
						break;
					}
				}

				assert(otherPatch->edges[otherEdgeIdx].neighbour == sub->patchID);

				if (otherEdgeIdx < 0)
				{
					edgeIndex++;
					std::cout << "Could not determine neighbors edge for patch edge\n";
					continue;
				}

				std::cout << "Stitch edge " << sub->patchID << "(" << sub->originalPatchID << ")" << "," << edgeIndex << " " << otherPatch->patchID << "(" << otherPatch->originalPatchID << ")" << "," << otherEdgeIdx << "\n";

				if (sub->edges.size() != 4 || otherPatch->edges.size() != 4)
				{
					std::cout << "Skipping stitch, non quad edges.\n";
					edgeIndex++;
					continue;;
				}

				for (int p = 0; p < resolution; p++)
				{
					int otherP = resolution - 1 - p;

					float *a = (float *)SampleEdge(output_p, sub, edgeIndex, resolution, p);
					float *b = (float *)SampleEdge(output_p, otherPatch, otherEdgeIdx, resolution, otherP);

					unsigned short *a_n = (unsigned short *)SampleEdge(output_n, sub, edgeIndex, resolution, p);
					unsigned short *b_n = (unsigned short *)SampleEdge(output_n, otherPatch, otherEdgeIdx, resolution, otherP);

					for (int c = 0; c < 4; c++)
					{
						b[c] = a[c];
						b_n[c] = a_n[c];
						/*float average = (a[c] + b[c]) * 0.5f;
						a[c] = average;
						b[c] = average;

						int average_s = (a_n[c] + b_n[c]) / 2; //(c & 1) ? 0xFFFFF : 0;
						a_n[c] = average_s;
						b_n[c] = average_s;*/
					}
				}

				edgeIndex++;
			}
		}

		sprintf(namebuff, "%s_out_p.exr", namePrefix);
		output_p.Save(namebuff, false);
		std::cout << "Saved bake: " << namebuff << std::endl;

		sprintf(namebuff, "%s_out_n.png", namePrefix);
		output_n.Save(namebuff, false);
		std::cout << "Saved bake: " << namebuff << std::endl;

		if (albeido)
		{
			sprintf(namebuff, "%s_out_t.png", namePrefix);
			output_t.Save(namebuff, false);
			std::cout << "Saved bake: " << namebuff << std::endl;
		}
	}

	static void BakePatches(std::vector<SurfaceMesh> &patches, Texture2D *albeido, const BoundingBox &normalizationBox)
	{
		char namebuff[256];

		int outputSize = (int)ceil(sqrt(patches.size()));
		int resolution = 128;
		Texture2D output_p;
		output_p.Initialize(outputSize*resolution, outputSize*resolution, 64, nullptr);
		Texture2D output_n;
		output_n.Initialize(outputSize*resolution, outputSize*resolution, 64, nullptr);

		int patchCount = 0;
		for (auto sub : patches)
		{
			int destX = (patchCount % outputSize) * resolution;
			int destY = (patchCount / outputSize) * resolution;

			// Unwrap patch to unit rectangle
			PatchParameterization sp(sub);
			sp.harmonic();

			MeshHelpers::BakeMesh(sub, albeido, resolution, output_n, destX, destY);

			MeshHelpers::PositionsToBakeNormalized(sub, normalizationBox);
			MeshHelpers::BakeMesh(sub, albeido, resolution, output_p, destX, destY);
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
