#pragma once
#include <pmp/algorithms/DifferentialGeometry.h>
#include <pmp/algorithms/SurfaceNormals.h>
#include <unordered_set>
#include <iostream>
#include <filesystem>
#include "ShortestPath.h"
#include "SurfaceSimplificationWithAncestors.h"
#include "MeshHelpers.h"
#include "PatchParameterization.h"
#include "PatchQuadrangulator.h"
#include "PatchHelpers.h"

#include <Eigen/Core>
#include <Eigen/Dense>
#include <algorithm>

#define ALL_MAPS_SAME_SIZE 1

class PatchBaker
{
public:

	struct ShortPixel
	{
		ShortPixel() {
			channels[0] = channels[1] = channels[2] = channels[3] = 0;
		}

		ShortPixel(int init) {
			channels[0] = channels[1] = channels[2] = channels[3] = init;
		}

		ShortPixel(const Vector4 &init) {
			for (int i = 0; i < 4; i++)
			{
				channels[i] = std::min(std::max(0, (int)(init[i]+0.5f)), 0xFFFF - 1);
			}
		}

		// Operator * promites to float precision
		Vector4 operator*(float scalar) const
		{
			Vector4 ret;
			for (int i = 0; i < 4; i++)
			{
				ret[i] = channels[i] * scalar;
			}
			return ret;
		}

		ShortPixel operator+(ShortPixel& a) const
		{
			ShortPixel r(0);
			for (int i = 0; i < 4; i++)
			{
				int l = (int)(a.channels[i] + channels[i]);
				r.channels[i] = std::min(std::max(0, l), 0xFFFF - 1);
			}
			return r;
		}

		ShortPixel operator-(ShortPixel& a) const
		{
			ShortPixel r(0);
			for (int i = 0; i < 4; i++)
			{
				int l = (int)(a.channels[i] - channels[i]);
				r.channels[i] = std::min(std::max(0, l), 0xFFFF - 1);
			}
			return r;
		}

		ShortPixel operator*(ShortPixel& a) const
		{
			ShortPixel r(0);
			for (int i = 0; i < 4; i++)
			{
				int l = (int)(a.channels[i]*channels[i]);
				r.channels[i] = std::min(std::max(0, l), 0xFFFF - 1);
			}
			return r;
		}

		unsigned short channels[4];
	};

	struct BytePixel
	{
		BytePixel() {
			channels[0] = channels[1] = channels[2] = channels[3] = 0;
		}

		BytePixel(int init) {
			channels[0] = channels[1] = channels[2] = channels[3] = init;
		}

		BytePixel(const Vector4 &init) {
			for (int i = 0; i < 4; i++)
			{
				channels[i] = std::min(std::max(0, (int)(init[i] + 0.5f)), 0xFF - 1);
			}
		}

		// Operator * promites to float precision
		Vector4 operator*(float scalar) const
		{
			Vector4 ret;
			for (int i = 0; i < 4; i++)
			{
				ret[i] = channels[i]*scalar;
			}
			return ret;
		}

		BytePixel operator+(BytePixel& a) const
		{
			BytePixel r(0);
			for (int i = 0; i < 4; i++)
			{
				int l = (int)(a.channels[i] + channels[i]);
				r.channels[i] = std::min(std::max(0, l), 0xFF - 1);
			}
			return r;
		}

		BytePixel operator-(BytePixel& a) const
		{
			BytePixel r(0);
			for (int i = 0; i < 4; i++)
			{
				int l = (int)(a.channels[i] - channels[i]);
				r.channels[i] = std::min(std::max(0, l), 0xFF - 1);
			}
			return r;
		}

		BytePixel operator*(BytePixel& a) const
		{
			BytePixel r(0);
			for (int i = 0; i < 4; i++)
			{
				int l = (int)(a.channels[i]*channels[i]);
				r.channels[i] = std::min(std::max(0, l), 0xFF - 1);
			}
			return r;
		}

		unsigned char channels[4];
	};

	static ShortPixel LerpPixel(ShortPixel &a, ShortPixel &b, float lerp)
	{
		float oneMinus = 1.0f - lerp;
		ShortPixel r(0);
		for (int i = 0; i < 4; i++)
		{
			int l = (int)(a.channels[i] * oneMinus + b.channels[i] * lerp + 0.5f);
			r.channels[i] = std::min(std::max(0, l), 0xFFFF - 1);

		}
		return r;
	}

	static BytePixel LerpPixel(BytePixel &a, BytePixel &b, float lerp)
	{
		float oneMinus = 1.0f - lerp;
		BytePixel r(0);
		for (int i = 0; i < 4; i++)
		{
			int l = (int)(a.channels[i] * oneMinus + b.channels[i] * lerp + 0.5f);
			r.channels[i] = std::min(std::max(0, l), 0xFF - 1);

		}
		return r;
	}

	static Vector4 LerpPixel(Vector4 &a, Vector4 &b, float lerp)
	{
		float oneMinus = 1.0f - lerp;
		return a * oneMinus + b * lerp;
	}

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
			x = patch->gridX*edgeResolution + pixelIndex;
			y = patch->gridY*edgeResolution;
			break;
		case 1:
			x = patch->gridX*edgeResolution + edgeResolution-1;
			y = patch->gridY*edgeResolution + pixelIndex;
			break;
		case 2:
			x = patch->gridX*edgeResolution + edgeResolution-1 - pixelIndex;
			y = patch->gridY*edgeResolution + edgeResolution-1;
			break;
		case 3:
			x = patch->gridX*edgeResolution;
			y = patch->gridY*edgeResolution + edgeResolution-1 - pixelIndex;
			break;
		default:
			x = 0;
			y = 0;
			break;
		}

		return image.image + (y * image.width + x) * (image.bpp / 8);
	}

	typedef std::function<void(PatchQuadrangulator::QuadrangularPatch* patch, int edge, int pixel)> CornerVisitor;

	// Visit all the patches touching the corner of a given input patch
	// This may seem too simple but it works out it's similar to iterating
	// halfedges around a vertex works. Try it out on paper...
	static void VisitCorners(std::vector<PatchQuadrangulator::QuadrangularPatch*> &quadPatches, int startQuad, int startEdge, CornerVisitor visit)
	{
		int currentQuad = startQuad;
		int currentEdge = startEdge;
		int currentPixel = 0;

		// Two cases a center vertex with an number of edges
		//		q1	|  q2
		//		  --*--
		//       q3  \   q4
		// We visit all the corners that touch this
		//
		// An edge of the mesh we visit all the corners we have to handle this specially since we may ot start at the edge
		// so we first walk backwards to find an edge then forwards from that edge if found
		//     q1 |  q2
		//     ---*-----
		//       empty space
		//

		// walk backwards untill we reach start or an edge
		// then walk forward from that edge until we reach that corner or an edge
		while (true)
		{
			PatchQuadrangulator::QuadrangularPatch* q = quadPatches[currentQuad];

			// Step to previous
			int prevEdge = currentEdge - 1;
			if (prevEdge < 0) prevEdge += quadPatches[currentQuad]->edges.size();

			// Reached a border edge use this one as the start
			if (q->edges[prevEdge].neighbour == -1)
			{
				break;
			}

			// Walk backwards to the next edge
			currentQuad = q->edges[prevEdge].neighbour;
			currentEdge = q->edges[prevEdge].neighbourEdgeIndex;

			// Full circle or no neighbour -> break
			if (currentQuad == startQuad)
			{
				if (currentEdge != startEdge)
				{
					std::cout << "Went around but not to the same edge :-(\n";
				}
				break;
			}
		}

		if (currentQuad < 0)
		{
			std::cout << "VisitCorners: invalid start\n";
		}

		while (true)
		{
			if (currentQuad < 0 || currentQuad >= quadPatches.size())
			{
				std::cout << "VisitCorners: patch index " << currentQuad << "\n";
				return;
			}

			PatchQuadrangulator::QuadrangularPatch* q = quadPatches[currentQuad];
			visit(q, currentEdge, currentPixel);

			// Step to next
			currentQuad = q->edges[currentEdge].neighbour;

			// Full circle or no neighbour -> break
			if (currentQuad == startQuad || currentQuad < 0) break;

			currentEdge = (q->edges[currentEdge].neighbourEdgeIndex+1)%quadPatches[currentQuad]->edges.size();
		}
	}

	// Samples with magic border stitching i.e. out of bounds coords will magcially sample the borders
	template <class PixelType> static PixelType SamplePatch(Texture2D &image, std::vector<PatchQuadrangulator::QuadrangularPatch*> &quadPatches, int patchSize, int patchIndex, int x, int y)
	{
		PatchQuadrangulator::QuadrangularPatch *patch = quadPatches[patchIndex];
		Vector4 average(0);
		int firstVisit = true;
		int averageCount = 0;

		//Note: This code takes special care to behave cleanly for patches which have non quad edges
		// it simply clamps in that case. But if the quadrangulation worked it should never happen.
		// but yet sometimes it happens so we handle this cleanly ;-)

		// Uncomment to disable all neighbour fetching logic
		//x = std::max(0, std::min(x, patchSize - 1));
		//y = std::max(0, std::min(y, patchSize - 1));

		if (patch->edges.size() != 4)
		{
			// Clamp and fall through
			x = std::max(0, std::min(x, patchSize - 1));
			y = std::max(0, std::min(y, patchSize - 1));
		}

		if (x < 0)
		{
			if (y < 0)
			{
				VisitCorners(quadPatches, patchIndex, 0, [&](PatchQuadrangulator::QuadrangularPatch* visitPatch, int edge, int pixel)
				{
					if (patch == visitPatch) return;
					average += *(PixelType*)SampleEdge(image, visitPatch, edge, patchSize, pixel)*1.0f;
					averageCount++;
				});
				return PixelType(average * (1.0f / averageCount));
			}
			if (y >= patchSize && patch->edges.size() >= 4)
			{
				VisitCorners(quadPatches, patchIndex, 3, [&](PatchQuadrangulator::QuadrangularPatch* visitPatch, int edge, int pixel)
				{
					if (patch == visitPatch) return;
					average += *(PixelType*)SampleEdge(image, visitPatch, edge, patchSize, pixel)*1.0f;
					averageCount++;
				});
				return PixelType(average * (1.0f / averageCount));
			}

			if (patch->edges.size() >= 4 && patch->edges[3].neighbour >= 0)
			{
				PatchQuadrangulator::QuadrangularPatch *edge = quadPatches[patch->edges[3].neighbour];
				if (edge->edges.size() == 4)
				{
					int otherP = y;// patchSize - 1 - y;
					return *(PixelType*)SampleEdge(image, edge, patch->edges[3].neighbourEdgeIndex, patchSize, otherP);
				}
			}

			// Clamp and fall through
			x = std::max(0, std::min(x, patchSize - 1));
			y = std::max(0, std::min(y, patchSize - 1));
		}
		else if (x >= patchSize)
		{
			if (y < 0)
			{
				VisitCorners(quadPatches, patchIndex, 1, [&](PatchQuadrangulator::QuadrangularPatch* visitPatch, int edge, int pixel)
				{
					if (patch == visitPatch) return;
					average += *(PixelType*)SampleEdge(image, visitPatch, edge, patchSize, pixel)*1.0f;
					averageCount++;
				});
				return PixelType(average * (1.0f / averageCount));
			}
			if (y >= patchSize)
			{
				VisitCorners(quadPatches, patchIndex, 2, [&](PatchQuadrangulator::QuadrangularPatch* visitPatch, int edge, int pixel)
				{
					if (patch == visitPatch) return;
					average += *(PixelType*)SampleEdge(image, visitPatch, edge, patchSize, pixel)*1.0f;
					averageCount++;
				});
				return PixelType(average * (1.0f / averageCount));
			}

			if (patch->edges[1].neighbour >= 0)
			{
				PatchQuadrangulator::QuadrangularPatch *edge = quadPatches[patch->edges[1].neighbour];
				if (edge->edges.size() == 4)
				{
					int otherP = patchSize - 1 - y;
					return *(PixelType*)SampleEdge(image, edge, patch->edges[1].neighbourEdgeIndex, patchSize, otherP);
				}
			}

			// Clamp and fall through
			x = std::max(0, std::min(x, patchSize - 1));
			y = std::max(0, std::min(y, patchSize - 1));
		}

		if (y < 0)
		{
			if (patch->edges[0].neighbour >= 0)
			{
				PatchQuadrangulator::QuadrangularPatch *edge = quadPatches[patch->edges[0].neighbour];
				if (edge->edges.size() == 4)
				{
					int otherP = patchSize - 1 - x;
					return *(PixelType*)SampleEdge(image, edge, patch->edges[0].neighbourEdgeIndex, patchSize, otherP);
				}
			}

			// Clamp and fall through
			x = std::max(0, std::min(x, patchSize - 1));
			y = std::max(0, std::min(y, patchSize - 1));
		}
		else if (y >= patchSize)
		{
			if (patch->edges[2].neighbour >= 0)
			{
				PatchQuadrangulator::QuadrangularPatch *edge = quadPatches[patch->edges[2].neighbour];
				if (edge->edges.size() == 4)
				{
					int otherP = x;// patchSize - 1 - x;
					return *(PixelType*)SampleEdge(image, edge, patch->edges[2].neighbourEdgeIndex, patchSize, otherP);
				}
			}

			// Clamp and fall through
			x = std::max(0, std::min(x, patchSize - 1));
			y = std::max(0, std::min(y, patchSize - 1));
		}

		//The normal case: aka disappontingly simple
		PixelType* base = (PixelType*)image.image;
		return *(base + (x + patch->gridX*patchSize) + (y + patch->gridY*patchSize) * image.width);
	}

	template <class PixelType> static void BlurPatch(std::vector<PatchQuadrangulator::QuadrangularPatch*> &quadPatches, int resolution, int patchIndex,Texture2D &image, Texture2D &out)
	{
		// Use http://dev.theomader.com/gaussian-kernel-calculator/
		// Sigma 0.3
		/*float gauss[9] = {
			0.002284,	0.043222,	0.002284,
			0.043222,	0.817976,	0.043222,
			0.002284,	0.043222,	0.002284
		};*/
		// Sigma 0.5
		float gauss[9] = {
			0.024879,	0.107973,	0.024879,
			0.107973,	0.468592,	0.107973,
			0.024879,	0.107973,	0.024879
		};

		PatchQuadrangulator::QuadrangularPatch*patch = quadPatches[patchIndex];
		for (int y = 0; y< resolution; y++)
		{
			for (int x = 0; x < resolution; x++)
			{
				Vector4 c(0);
				int xo = 1; int yo = 1;
				for (int yo = -1; yo < 2; yo++)
				{
					for (int xo = -1; xo < 2; xo++)
					{
						Vector4 p = SamplePatch<PixelType>(image, quadPatches, resolution, patchIndex, x+xo, y+yo) * gauss[(xo+1)+(yo+1)*3];
						c += p;
					}
				}

				*out.GetPixelPtr<PixelType>(x + patch->gridX*resolution, y + patch->gridY*resolution) = PixelType(c);
			}
		}
	}

	template <class PixelType> static void MipPatch(std::vector<PatchQuadrangulator::QuadrangularPatch*> &quadPatches, int resolution, int patchIndex, Texture2D &image, Texture2D &mip)
	{
		float mipResolution = resolution / 2;
		float sampleScale = (float)(resolution - 1) / (float)(mipResolution - 1);

		PatchQuadrangulator::QuadrangularPatch*patch = quadPatches[patchIndex];
		for (int y = 0; y < mipResolution; y++)
		{
			for (int x = 0; x < mipResolution; x++)
			{
				float highx = x * sampleScale;
				float highy = y * sampleScale;

				int hix0 = (int)highx;
				int hix1 = hix0 + 1;

				int hiy0 = (int)highy;
				int hiy1 = hiy0 + 1;

				assert(hix0 >= 0 && hix0 < resolution);
				assert(hiy0 >= 0 && hiy0 < resolution);
				assert(hix1 >= 0 && hix1 <= resolution);
				assert(hiy1 >= 0 && hiy1 <= resolution);

				float lerpx = highx - hix0;
				float lerpy = highy - hiy0;

				PixelType p00 = SamplePatch<PixelType>(image, quadPatches, resolution, patchIndex, hix0, hiy0);
				PixelType p01 = SamplePatch<PixelType>(image, quadPatches, resolution, patchIndex, hix0, hiy1);
				PixelType p11 = SamplePatch<PixelType>(image, quadPatches, resolution, patchIndex, hix1, hiy1);
				PixelType p10 = SamplePatch<PixelType>(image, quadPatches, resolution, patchIndex, hix1, hiy0);

				PixelType p0 = LerpPixel(p00, p10, lerpx);
				PixelType p1 = LerpPixel(p01, p11, lerpx);

				PixelType p = LerpPixel(p0, p1, lerpy);

				*mip.GetPixelPtr<PixelType>(x + patch->gridX*mipResolution, y + patch->gridY*mipResolution) = p;
			}
		}
	}

	template <class PixelType> static void MipPatches(std::vector<PatchQuadrangulator::QuadrangularPatch*> &quadPatches, int resolution, Texture2D& image, const char *namePrefix, const char *namePostfix)
	{
		Texture2D* prevMip = &image;
		for (int mipResolution = resolution / 2, mipCount = 1; mipResolution >= 4; mipResolution = mipResolution / 2, mipCount++)
		{
			int prevResolution = mipResolution * 2;

			Texture2D output_blur;
			output_blur.Initialize(prevMip->width, prevMip->height, prevMip->bpp, nullptr);

			for (int i = 0; i < quadPatches.size(); i++)
			{
				BlurPatch<PixelType>(quadPatches, prevResolution, i, *prevMip, output_blur);
			}

			Texture2D *output_mip = new Texture2D();
			output_mip->Initialize(prevMip->width/2, prevMip->height/2, prevMip->bpp, nullptr);

			for (int i = 0; i < quadPatches.size(); i++)
			{
				MipPatch<PixelType>(quadPatches, prevResolution, i, output_blur, *output_mip);
			}

			StitchPatches<PixelType>(quadPatches, *output_mip, mipResolution);

			char namebuff[512];
			sprintf(namebuff, "%s_mip_%i_%s", namePrefix, mipCount, namePostfix);
			output_mip->Save(namebuff, false, true);
			std::cout << "Saved mip: " << namebuff << std::endl;

			// Don't free the originally passed in image
			if (mipCount != 1)
			{
				delete prevMip;
			}
			prevMip = output_mip;
		}
	}

	static std::pair<Eigen::Vector3f, Eigen::Vector3f> FitPlane3D(const std::vector<Eigen::Vector3f> & c)
	{
		// copy coordinates to  matrix in Eigen format
		size_t num_atoms = c.size();
		Eigen::Matrix<Eigen::Vector3f::Scalar, Eigen::Dynamic, Eigen::Dynamic > coord(3, num_atoms);
		for (size_t i = 0; i < num_atoms; ++i) coord.col(i) = c[i];

		// calculate centroid
		Eigen::Vector3f centroid(coord.row(0).mean(), coord.row(1).mean(), coord.row(2).mean());

		// subtract centroid
		coord.row(0).array() -= centroid(0); coord.row(1).array() -= centroid(1); coord.row(2).array() -= centroid(2);

		// we only need the left-singular matrix here
		//  http://math.stackexchange.com/questions/99299/best-fitting-plane-given-a-set-of-points
		auto svd = coord.jacobiSvd(Eigen::ComputeThinU | Eigen::ComputeThinV);
		Eigen::Vector3f plane_normal = svd.matrixU().rightCols<1>();
		return std::make_pair(centroid, plane_normal);
	}

	static
	std::pair<Vector4, Vector4> GetPlane3D(Vector4 p00, Vector4 p01, Vector4 p11, Vector4 p10)
	{
		std::pair<Eigen::Vector3f, Eigen::Vector3f>
			planeInfo = FitPlane3D(
				{
					Eigen::Vector3f(p00(0, 0), p00(1, 0), p00(2, 0)),
					Eigen::Vector3f(p01(0, 0), p01(1, 0), p01(2, 0)),
					Eigen::Vector3f(p11(0, 0), p11(1, 0), p11(2, 0)),
					Eigen::Vector3f(p10(0, 0), p10(1, 0), p10(2, 0))
				});

		// first: centroid, second: normal
		return
			{
				Vector4(planeInfo.first.x(), planeInfo.first.y(), planeInfo.first.z(), 1.0f),
				Vector4(planeInfo.second.x(), planeInfo.second.y(), planeInfo.second.z(), 1.0f)
			};
	}

	static
	Vector4 GetHeights(
		std::vector<PatchQuadrangulator::QuadrangularPatch*> & quadPatches,
		int resolution,
		int patchIndex,
		float hix0, float hiy0,
		float hix1, float hiy1,
		Texture2D& positions)
	{
		Vector4 p00, p01, p11, p10;
		Gather<Vector4>(p00, p01, p11, p10, quadPatches, resolution, patchIndex, hix0, hiy0, hix1, hiy1, positions);

		auto planeInfo = GetPlane3D(p00, p01, p11, p10);

		float d00 = abs(dot(p00, planeInfo.second));
		float d01 = abs(dot(p00, planeInfo.second));
		float d11 = abs(dot(p00, planeInfo.second));
		float d10 = abs(dot(p00, planeInfo.second));

		return Vector4(d00, d01, d11, d10);
	}

	template <class PixelType>
	static PixelType Variance4(PixelType a, PixelType b, PixelType c, PixelType d)
	{
		PixelType mean = (a + b + c + d);

		return PixelType(((a*a + b*b + c*c + d*d) - (mean*mean))*(1.0f/4.0f));
	}


	template <class PixelType>
	static PixelType Variance4(
		std::vector<PatchQuadrangulator::QuadrangularPatch*>& quadPatches,
		int resolution,
		int patchIndex,
		float hix0, float hiy0,
		float hix1, float hiy1,
		Texture2D& image)
	{
		PixelType p00, p01, p11, p10;
		Gather<PixelType>(p00, p01, p11, p10, quadPatches, resolution, patchIndex, hix0, hiy0, hix1, hiy1, image);

		return Variance4(p00, p01, p11, p10);
	}

	template <class PixelType>
	static void Gather(
		PixelType& p00,
		PixelType& p01,
		PixelType& p11,
		PixelType& p10,
		std::vector<PatchQuadrangulator::QuadrangularPatch*>& quadPatches,
		int resolution,
		int patchIndex,
		float hix0, float hiy0,
		float hix1, float hiy1,
		Texture2D& image)
	{
		p00 = SamplePatch<PixelType>(image, quadPatches, resolution, patchIndex, hix0, hiy0);
		p01 = SamplePatch<PixelType>(image, quadPatches, resolution, patchIndex, hix0, hiy1);
		p11 = SamplePatch<PixelType>(image, quadPatches, resolution, patchIndex, hix1, hiy1);
		p10 = SamplePatch<PixelType>(image, quadPatches, resolution, patchIndex, hix1, hiy0);
	}

	template <class PixelType>
	static PixelType GetSum(
		std::vector<PatchQuadrangulator::QuadrangularPatch*>& quadPatches,
		int resolution,
		int patchIndex,
		float hix0, float hiy0,
		float hix1, float hiy1,
		Texture2D& image)
	{
		PixelType p00, p01, p11, p10;
		Gather<PixelType>(p00, p01, p11, p10, quadPatches, resolution, patchIndex, hix0, hiy0, hix1, hiy1, image);

		return p00 + p01 + p11 + p10;
	}

	template <class PixelType>
	static PixelType Bilinear(
		std::vector<PatchQuadrangulator::QuadrangularPatch*>& quadPatches,
		int resolution,
		int patchIndex,
		float hix0, float hiy0,
		float hix1, float hiy1,
		float lerpx, float lerpy,
		Texture2D& image)
	{
		PixelType p00, p01, p11, p10;
		Gather<PixelType>(p00, p01, p11, p10, quadPatches, resolution, patchIndex, hix0, hiy0, hix1, hiy1, image);

		PixelType p0 = LerpPixel(p00, p10, lerpx);
		PixelType p1 = LerpPixel(p01, p11, lerpx);

		return LerpPixel(p0, p1, lerpy);
	}

	template <class PixelType, class PreProcess, class PostProcess>
	static PixelType BilinearWithProcess(
		std::vector<PatchQuadrangulator::QuadrangularPatch*>& quadPatches,
		int resolution,
		int patchIndex,
		float hix0, float hiy0,
		float hix1, float hiy1,
		float lerpx, float lerpy,
		Texture2D& image,
		PreProcess preProcess,
		PostProcess postProcess)
	{
		PixelType p00, p01, p11, p10;
		Gather<PixelType>(p00, p01, p11, p10, quadPatches, resolution, patchIndex, hix0, hiy0, hix1, hiy1, image);

		PixelType p0 = LerpPixel(p00, p10, lerpx);
		PixelType p1 = LerpPixel(p01, p11, lerpx);

		return postProcess(LerpPixel(p0, p1, lerpy));
	}

	static void MipPatchAll(
		std::vector<PatchQuadrangulator::QuadrangularPatch*>& quadPatches,
		int resolution,
		int patchIndex,
		Texture2D& positions,
		Texture2D& normals,
		Texture2D& albedo,
		Texture2D& roughness,
		Texture2D& positionsMip,
		Texture2D& normalsMip,
		Texture2D& albedoMip,
		Texture2D& roughnessMip)
	{
		float mipResolution = resolution / 2;
		float sampleScale = (float)(resolution - 1) / (float)(mipResolution - 1);

		PatchQuadrangulator::QuadrangularPatch* patch = quadPatches[patchIndex];
		for (int y = 0; y < mipResolution; y++)
		{
			for (int x = 0; x < mipResolution; x++)
			{
				float highx = x * sampleScale;
				float highy = y * sampleScale;

				int hix0 = (int)highx;
				int hix1 = hix0 + 1;

				int hiy0 = (int)highy;
				int hiy1 = hiy0 + 1;

				assert(hix0 >= 0 && hix0 < resolution);
				assert(hiy0 >= 0 && hiy0 < resolution);
				assert(hix1 >= 0 && hix1 <= resolution);
				assert(hiy1 >= 0 && hiy1 <= resolution);

				float lerpx = highx - hix0;
				float lerpy = highy - hiy0;

				Vector4 heights = GetHeights(quadPatches, resolution, patchIndex, hix0, hiy0, hix1, hiy0, positions);
				float varH = std::clamp(Variance4(heights(0, 0), heights(1, 0), heights(2, 0), heights(3, 0)), 0.0f, 1.0f);
				float varH2 = varH*varH;

				// Fit vMF to NDF
				ShortPixel nTotal = GetSum<ShortPixel>(quadPatches, resolution, patchIndex, hix0, hiy0, hix1, hiy1, normals);
				Vector4 nTotalf = (nTotal*(0.25f/float(0xFFFF - 1)));

				float roughnessOffset = varH2;
				float fNorm = norm(nTotalf);
				if (fNorm < 1.0f)
				{
					float kappa = (3.0f*fNorm - fNorm*fNorm*fNorm)/(1.0f - fNorm*fNorm);
					roughnessOffset += 1.0f/kappa;
				}

				*positionsMip.GetPixelPtr<Vector4>(x + patch->gridX * mipResolution, y + patch->gridY * mipResolution) =
					Bilinear<Vector4>(quadPatches, resolution, patchIndex, hix0, hiy0, hix1, hiy0, lerpx, lerpy, positions);

				*normalsMip.GetPixelPtr<ShortPixel>(x + patch->gridX * mipResolution, y + patch->gridY * mipResolution) =
					Bilinear<ShortPixel>(quadPatches, resolution, patchIndex, hix0, hiy0, hix1, hiy0, lerpx, lerpy, normals);

				*albedoMip.GetPixelPtr<BytePixel>(x + patch->gridX * mipResolution, y + patch->gridY * mipResolution) =
					Bilinear<BytePixel>(quadPatches, resolution, patchIndex, hix0, hiy0, hix1, hiy0, lerpx, lerpy, albedo);

				*roughnessMip.GetPixelPtr<BytePixel>(x + patch->gridX * mipResolution, y + patch->gridY * mipResolution) =
					BilinearWithProcess<BytePixel>(quadPatches, resolution, patchIndex, hix0, hiy0, hix1, hiy0, lerpx, lerpy, roughness,
							[](BytePixel x)
							{
								Vector4 y = x*(1.0f/255.f);
								for (int i = 0; i < 4; ++i)
								{
									y(i, 0) = sqrt(y(i, 0))*255.0f;
								}
								return BytePixel(y);
							},
							[roughnessOffset](BytePixel x)
							{
								Vector4 y = (x*(1.0f/255.0f) + Vector4(roughnessOffset, roughnessOffset, roughnessOffset, roughnessOffset));
								Vector4 z(y(0,0)*y(0,0), y(1,0)*y(1,0), y(2,0)*y(2,0), y(3,0)*y(3,0));
								return BytePixel(z*255.0f);
							}
						);
			}
		}
	}

	static void MipPatchesAll(
		std::vector<PatchQuadrangulator::QuadrangularPatch*> &quadPatches,
		int resolution,
		Texture2D& positions,
		Texture2D& normals,
		Texture2D& albedo,
		Texture2D& roughness,
		const char *namePrefix)
	{
		// sprintf(namebuff, "%s_out_", namePrefix);

		Texture2D* prevPosMip = &positions;
		Texture2D* prevNormalMip = &normals;
		Texture2D* prevAlbedoMip = &albedo;
		Texture2D* prevRoughnessMip = &roughness;
		for (int mipResolution = resolution / 2, mipCount = 1; mipResolution >= 4; mipResolution = mipResolution / 2, mipCount++)
		{
			int prevResolution = mipResolution * 2;

			Texture2D outputPos_blur;
			outputPos_blur.Initialize(prevPosMip->width, prevPosMip->height, prevPosMip->bpp, nullptr);
			Texture2D outputNormal_blur;
			outputNormal_blur.Initialize(prevNormalMip->width, prevNormalMip->height, prevNormalMip->bpp, nullptr);
			Texture2D outputAlbedo_blur;
			outputAlbedo_blur.Initialize(prevAlbedoMip->width, prevAlbedoMip->height, prevAlbedoMip->bpp, nullptr);
			Texture2D outputRoughness_blur;
			outputRoughness_blur.Initialize(prevRoughnessMip->width, prevRoughnessMip->height, prevRoughnessMip->bpp, nullptr);

			for (int i = 0; i < quadPatches.size(); i++)
			{
				BlurPatch<Vector4>(quadPatches, prevResolution, i, *prevPosMip, outputPos_blur);
				BlurPatch<ShortPixel>(quadPatches, prevResolution, i, *prevNormalMip, outputNormal_blur);
				BlurPatch<BytePixel>(quadPatches, prevResolution, i, *prevAlbedoMip, outputAlbedo_blur);
				BlurPatch<BytePixel>(quadPatches, prevResolution, i, *prevRoughnessMip, outputRoughness_blur);
			}

			Texture2D *outputPos_mip = new Texture2D();
			outputPos_mip->Initialize(prevPosMip->width/2, prevPosMip->height/2, prevPosMip->bpp, nullptr);
			Texture2D *outputNormal_mip = new Texture2D();
			outputNormal_mip->Initialize(prevNormalMip->width/2, prevNormalMip->height/2, prevNormalMip->bpp, nullptr);
			Texture2D *outputAlbedo_mip = new Texture2D();
			outputAlbedo_mip->Initialize(prevAlbedoMip->width/2, prevAlbedoMip->height/2, prevAlbedoMip->bpp, nullptr);
			Texture2D *outputRoughness_mip = new Texture2D();
			outputRoughness_mip->Initialize(prevRoughnessMip->width/2, prevRoughnessMip->height/2, prevRoughnessMip->bpp, nullptr);

			for (int i = 0; i < quadPatches.size(); i++)
			{
				MipPatchAll(quadPatches, prevResolution, i,
					outputPos_blur, outputNormal_blur, outputAlbedo_blur, outputRoughness_blur,
					*outputPos_mip, *outputNormal_mip, *outputAlbedo_mip, *outputRoughness_mip);
			}

			StitchPatches<Vector4>(quadPatches, *outputPos_mip, mipResolution);
			StitchPatches<ShortPixel>(quadPatches, *outputNormal_mip, mipResolution);
			StitchPatches<BytePixel>(quadPatches, *outputAlbedo_mip, mipResolution);
			StitchPatches<BytePixel>(quadPatches, *outputRoughness_mip, mipResolution);

			char namebuffPos[512];
			char namebuffNor[512];
			char namebuffAlb[512];
			char namebuffRou[512];
			sprintf(namebuffPos, "%s_p_all_mip_%i_%s", namePrefix, mipCount, ".exr");
			sprintf(namebuffNor, "%s_n_all_mip_%i_%s", namePrefix, mipCount, ".png");
			sprintf(namebuffAlb, "%s_t_all_mip_%i_%s", namePrefix, mipCount, ".png");
			sprintf(namebuffRou, "%s_r_all_mip_%i_%s", namePrefix, mipCount, ".png");
			outputPos_mip->Save(namebuffPos, false);
			outputNormal_mip->Save(namebuffNor, false);
			outputAlbedo_mip->Save(namebuffAlb, true);
			outputRoughness_mip->Save(namebuffRou, false);
			std::cout << "Saved mip: " << outputPos_mip << std::endl;
			std::cout << "Saved mip: " << outputNormal_mip << std::endl;
			std::cout << "Saved mip: " << outputAlbedo_mip << std::endl;
			std::cout << "Saved mip: " << outputRoughness_mip << std::endl;

			// Don't free the originally passed in image
			if (mipCount != 1)
			{
				delete prevPosMip;
				delete prevNormalMip;
				delete prevAlbedoMip;
				delete prevRoughnessMip;
			}
			prevPosMip = outputPos_mip;
			prevNormalMip = outputNormal_mip;
			prevAlbedoMip = outputAlbedo_mip;
			prevRoughnessMip = outputRoughness_mip;
		}
	}

	// Make clean patch id's starting from zero that can be used to directly index in the vector.
	// also adjusts the neighbour pointers accordingly.
	static void ReassignPatchIDs(std::vector<PatchQuadrangulator::QuadrangularPatch*> &quadPatches)
	{
		std::unordered_map<int, int> reodering;
		int patchCount = 0;
		for (auto sub : quadPatches)
		{
			reodering[sub->patchID] = patchCount;
			sub->patchID = patchCount;
			patchCount++;
		}

		// Patch neighbours
		for (auto &sub : quadPatches)
		{
			for (auto &edge : sub->edges)
			{
				if (edge.neighbour > -1)
				{
					auto f = reodering.find(edge.neighbour);
					if (f != reodering.end())
					{
						edge.neighbour = f->second;
					}
					else
					{
						std::cout << "Could not find neighbour patch with id " << edge.neighbour << "\n";
						edge.neighbour = -1;
					}
				}
				else
				{
					edge.neighbour = -1;
				}
			}
		}
	}

	template<class PixelType> static void StitchPatches(std::vector<PatchQuadrangulator::QuadrangularPatch*> &quadPatches, Texture2D &image, int resolution)
	{
		// Stitch neighbour patches up so the border pixel values match exactly
		// Also fill the neighbour info texture
		for (int i = 0; i < quadPatches.size(); i++)
		{
			auto sub = quadPatches[i];
			int edgeIndex = 0;

			for (auto &edge : sub->edges)
			{
				//No neighbour nothing to stitch.
				if (edge.neighbour < 0)
				{
					edgeIndex++;
					continue;
				}

				PatchQuadrangulator::QuadrangularPatch* otherPatch = quadPatches[edge.neighbour];
				int otherEdgeIdx = edge.neighbourEdgeIndex;

				//Already stitched up by other neighbour
				if (edge.neighbour < 0 || edge.neighbour < sub->patchID)
				{
					edgeIndex++;
					continue;
				}

				std::cout << "Stitch edge " << sub->patchID << "(" << sub->originalPatchID << ")" << "," << edgeIndex << " " << otherPatch->patchID << "(" << otherPatch->originalPatchID << ")" << "," << otherEdgeIdx << "\n";

				if (sub->edges.size() != 4 || otherPatch->edges.size() != 4)
				{
					std::cout << "Skipping stitch, non quad edges.\n";
					edgeIndex++;
					continue;;
				}

				for (int p = 1; p < resolution-1; p++)
				{
					int otherP = resolution - 1 - p;

					PixelType *a = (PixelType *)SampleEdge(image, sub, edgeIndex, resolution, p);
					PixelType *b = (PixelType *)SampleEdge(image, otherPatch, otherEdgeIdx, resolution, otherP);

					//*b = *a;

					PixelType average = LerpPixel(*a, *b, 0.5f);//(*a + *b) * 0.5f;
					*a = average;
					*b = average;
				}

				Vector4 average(0);
				int count = 0;
				VisitCorners(quadPatches, i, edgeIndex, [&](PatchQuadrangulator::QuadrangularPatch* visitPatch, int edge, int pixel)
				{
					PixelType pix = *(PixelType *)SampleEdge(image, visitPatch, edge, resolution, pixel);
					average += (pix * 1.0f);
					count++;
				});

				average *= (1.0f / count);

				VisitCorners(quadPatches, i, edgeIndex, [&](PatchQuadrangulator::QuadrangularPatch* visitPatch, int edge, int pixel)
				{
					PixelType *pix = (PixelType *)SampleEdge(image, visitPatch, edge, resolution, pixel);
					*pix = PixelType(average);
				});

				edgeIndex++;
			}
		}
	}

	static void BakePatches(
		std::vector<PatchQuadrangulator::QuadrangularPatch*> &quadPatches,
		Texture2D* diffuseAlbedo,
		Texture2D* roughness,
		BoundingBox &normalizationBox,
		const char *namePrefix)
	{
		ReassignPatchIDs(quadPatches);

		char namebuff[256];
		char postionName[256];
		char normalName[256];
		char diffuseAlbedoName[256];
		char roughnessName[256];
		char neighboursName[256];
		char patchBoundsName[256];
		char cornerNeighboursName[256];

		int outputSize = (int)ceil(sqrt(quadPatches.size()));
		int resolution = 128;

		Texture2D output_t;
		Texture2D output_r;
		int textureResolutionScale = 4;
#if ALL_MAPS_SAME_SIZE
		int textureResolution = resolution;
#else
		int textureResolution = resolution*textureResolutionScale;
#endif
		if (diffuseAlbedo)
		{
			output_t.Initialize(outputSize*textureResolution, outputSize*textureResolution, 32, nullptr);
		}
		if (roughness)
		{
			output_r.Initialize(outputSize*textureResolution, outputSize*textureResolution, 32, nullptr);
		}

		Texture2D output_p;
		output_p.Initialize(outputSize*resolution, outputSize*resolution, 128, nullptr);
		Texture2D output_n;
		output_n.Initialize(outputSize*resolution, outputSize*resolution, 64, nullptr);

		// Bounding box texture x: mins, maxs, y: patchid
		Texture2D output_bounds;
		output_bounds.Initialize(2, outputSize*outputSize, 128, nullptr);

		// Neighbour texture: x : neighbour, edge, y : patchid
		Texture2D output_neighbours;
		output_neighbours.Initialize(2, outputSize*outputSize, 128, nullptr);

		// Neighbour corners texture: x : patchid, y : n-th neighbour (0 = num neighbors)
		static const int MAX_CORNER_NEIGHBOURS = 15;
		Texture2D output_cneighbours;
		output_cneighbours.Initialize(MAX_CORNER_NEIGHBOURS+1, outputSize*outputSize, 128, nullptr);

		sprintf(namebuff, "%s_bounds.txt", namePrefix);
		FILE *boundsTxt = fopen(namebuff, "w");

		for (auto sub : quadPatches)
		{
			sub->gridX = sub->patchID % outputSize;
			sub->gridY = sub->patchID / outputSize;

			// Save bounding box
			auto bounds = sub->mesh.bounds();
			*output_bounds.GetPixelPtr<Vector4>(0, sub->patchID) = Vector4(bounds.min(), 1.0f);
			*output_bounds.GetPixelPtr<Vector4>(1, sub->patchID) = Vector4(bounds.max(), 1.0f);

			fprintf(boundsTxt, "min %f %f %f max %f %f %f\n",
				bounds.min()[0], bounds.min()[1], bounds.min()[2],
				bounds.max()[0], bounds.max()[1], bounds.max()[2]
				);

			MeshHelpers::BackupPositions(sub->mesh);

			std::cout << "Bake Patch Positions\n";
			MeshHelpers::PositionsToBakeNormalized(sub->mesh, normalizationBox);
			MeshHelpers::TexCoordsToPositions(sub->mesh);
			MeshHelpers::BakeMesh(sub->mesh, NULL, resolution, output_p, sub->gridX*resolution, sub->gridY*resolution);

			std::cout << "Bake Patch Normals\n";
			MeshHelpers::NormalsToBake(sub->mesh);
			MeshHelpers::BakeMesh(sub->mesh, NULL, resolution, output_n, sub->gridX*resolution, sub->gridY*resolution);

			if (diffuseAlbedo)
			{
				std::cout << "Bake Patch Albedo\n";
				MeshHelpers::BakeMesh(sub->mesh, diffuseAlbedo, textureResolution, output_t, sub->gridX * textureResolution, sub->gridY * textureResolution);
			}
			if (roughness)
			{
				std::cout << "Bake Patch Roughness\n";
				MeshHelpers::BakeMesh(sub->mesh, roughness, textureResolution, output_r, sub->gridX * textureResolution, sub->gridY * textureResolution);
			}

			MeshHelpers::RestorePositions(sub->mesh);
		}

		fclose(boundsTxt);

		// Stitch up edges
		StitchPatches<Vector4>(quadPatches, output_p, resolution);
		StitchPatches<ShortPixel>(quadPatches, output_n, resolution);
		if (diffuseAlbedo)
		{
			StitchPatches<BytePixel>(quadPatches, output_t, textureResolution);
		}
		if (roughness)
		{
			StitchPatches<BytePixel>(quadPatches, output_r, textureResolution);
		}

		// Fill in neighbour data to ouptut
		for (int i=0;i<quadPatches.size();i++)
		{
			auto sub = quadPatches[i];
			int edgeIndex=0;
			Vector4 neighbourIds;
			Vector4 neighbourEdges;

			for (auto edge : sub->edges)
			{
				if (edgeIndex < 4)
				{
					neighbourIds[edgeIndex] = edge.neighbour;
					neighbourEdges[edgeIndex] = edge.neighbourEdgeIndex;

					VisitCorners(quadPatches, i, edgeIndex, [&](PatchQuadrangulator::QuadrangularPatch* visitPatch, int edge, int pixel)
					{
						if ( visitPatch == sub) return; //skip self
						Vector4 &neighboursCounts = *output_cneighbours.GetPixelPtr<Vector4>(0, sub->patchID);
						if (neighboursCounts[edgeIndex] < MAX_CORNER_NEIGHBOURS)
						{
							Vector4 &destNeighbours = *output_cneighbours.GetPixelPtr<Vector4>(neighboursCounts[edgeIndex] + 1, sub->patchID);
							destNeighbours[edgeIndex] = visitPatch->patchID;
							neighboursCounts[edgeIndex]++;
						}
						else
						{
							std::cout << "Corner has more than " << MAX_CORNER_NEIGHBOURS << "neighbours. Ignoring extra neighbours\n";
						}
					});
				}
				edgeIndex++;
			}

			*output_neighbours.GetPixelPtr<Vector4>(0, sub->patchID) = neighbourIds;
			*output_neighbours.GetPixelPtr<Vector4>(1, sub->patchID) = neighbourEdges;
		}

		sprintf(postionName, "%s_p.exr", namePrefix);
		output_p.Save(postionName, false, true);
		std::cout << "Saved bake: " << postionName << std::endl;

		sprintf(namebuff, "%s_p", namePrefix);
		MipPatches<Vector4>(quadPatches, resolution, output_p, namebuff, ".exr");

		sprintf(normalName, "%s_n.png", namePrefix);
		output_n.Save(normalName, false, true);
		std::cout << "Saved bake: " << normalName << std::endl;

		sprintf(namebuff, "%s_n", namePrefix);
		MipPatches<ShortPixel>(quadPatches, resolution, output_n, namebuff, ".png");

		if (diffuseAlbedo)
		{
			sprintf(diffuseAlbedoName, "%s_t.png", namePrefix);
			output_t.Save(diffuseAlbedoName, false, true);
			std::cout << "Saved bake: " << diffuseAlbedoName << std::endl;

			sprintf(namebuff, "%s_t", namePrefix);
			MipPatches<BytePixel>(quadPatches, textureResolution, output_t, namebuff, ".png");
		}
		else
		{
			sprintf(diffuseAlbedoName, "");
		}

		if (roughness)
		{
			sprintf(roughnessName, "%s_r.png", namePrefix);
			output_r.Save(roughnessName, false, true);
			std::cout << "Saved bake: " << roughnessName << std::endl;

			sprintf(namebuff, "%sr", namePrefix);
			MipPatches<BytePixel>(quadPatches, textureResolution, output_r, namebuff, ".png");
		}
		else
		{
			sprintf(roughnessName, "");
		}

		if (diffuseAlbedo && roughnessName)
		{
			sprintf(namebuff, "%s_out_", namePrefix);
			MipPatchesAll(quadPatches, textureResolution, output_p, output_n, output_t, output_r, namebuff);
		}

		sprintf(patchBoundsName, "%s_bounds.exr", namePrefix);
		output_bounds.Save(patchBoundsName, false, true);
		std::cout << "Saved bake: " << patchBoundsName << std::endl;

		sprintf(neighboursName, "%s_neighbours.exr", namePrefix);
		output_neighbours.Save(neighboursName, false, true);
		std::cout << "Saved bake: " << neighboursName << std::endl;

		sprintf(cornerNeighboursName, "%s_cneighbours.exr", namePrefix);
		output_cneighbours.Save(cornerNeighboursName, false, true);
		std::cout << "Saved bake: " << cornerNeighboursName << std::endl;

		Point delta = normalizationBox.max() - normalizationBox.min();

		char jsonBuff[4096];

		const char *jsonTemplate = R"(
			{
				"positions": "%s",
				"normals": "%s",
				"albedo": "%s",
				"roughness": "%s",
				"neighbours": "%s",
				"patchBounds": "%s",
				"cornerNeighbours": "%s",
				"scale": {
					"x": %f,
					"y": %f,
					"z": %f
				},
				"bias": {
					"x": %f,
					"y": %f,
					"z": %f
				},
				"numPatches": %i,
				"gridWidth": %i,
				"gridHeight": %i,
				"gridSize": %i,
				"boundsMin": {
					"x": %f,
					"y": %f,
					"z": %f
				},
				"boundsMax": {
					"x": %f,
					"y": %f,
					"z": %f
				}
			}
		)";

		sprintf(jsonBuff, jsonTemplate,
			std::filesystem::path(postionName).filename().string().c_str(),
			std::filesystem::path(normalName).filename().string().c_str(),
			std::filesystem::path(diffuseAlbedoName).filename().string().c_str(),
			std::filesystem::path(roughnessName).filename().string().c_str(),
			std::filesystem::path(neighboursName).filename().string().c_str(),
			std::filesystem::path(patchBoundsName).filename().string().c_str(),
			std::filesystem::path(cornerNeighboursName).filename().string().c_str(),
			delta[0], delta[1], delta[2],
			normalizationBox.min()[0], normalizationBox.min()[1], normalizationBox.min()[2],
			quadPatches.size(),
			outputSize,
			outputSize,
			resolution,
			normalizationBox.min()[0], normalizationBox.min()[1], normalizationBox.min()[2],
			normalizationBox.max()[0], normalizationBox.max()[1], normalizationBox.max()[2]
		);

		sprintf(namebuff, "%s.vmesh", namePrefix);
		FILE *f = fopen(namebuff, "w");
		fprintf(f, "%s", jsonBuff);
		fclose(f);
		std::cout << "Saved json: " << namebuff << std::endl;

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
		output_p.Save(namebuff, false, true);
		std::cout << "Saved bake: " << namebuff << std::endl;

		sprintf(namebuff, "out_n.png");
		output_n.Save(namebuff, false, true);
		std::cout << "Saved bake: " << namebuff <<std::endl;
	}
};
