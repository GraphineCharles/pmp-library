#pragma once
#include <pmp/algorithms/DifferentialGeometry.h>
#include <pmp/algorithms/SurfaceNormals.h>
#include <unordered_set>
#include "ShortestPath.h"
#include "SurfaceSimplificationWithAncestors.h"
#include "MeshHelpers.h"
#include "PatchParameterization.h"
#include "PatchBaker.h"

class ClusteringExp
{
public:

	ClusteringExp(SurfaceMesh& setMesh) : inputMesh(setMesh) {}


	void process()
	{
		simplified = inputMesh; //deep copy

#if 1
		// Edge collapse
		SurfaceSimplificationWithAncestors ss(simplified);
		ss.initialize();
		ss.simplify(simplified.n_vertices()/1000);

		FaceProperty<std::vector<Face>> original_faces = simplified.face_property<std::vector<Face>>("f:orignalFaces");
		auto prop = inputMesh.add_face_property<Color>("f:color", Color(0, 0, 0));
		auto prop_simp = simplified.add_face_property<Color>("f:color", Color(0, 0, 0));

		srand(2202);
		for (Face f : simplified.faces())
		{
			vec3 color = vec3((rand() % 255) / 255.0f,
				(rand() % 255) / 255.0f,
				(rand() % 255) / 255.0f);

			for (Face of : original_faces[f])
			{
				prop[Face(of)] = color;
			}

			prop_simp[f] = color;
		}

		SurfaceMesh sub;
		std::vector<std::vector<Face>> patches;
		patches.resize(simplified.n_faces());
		// first face
		for (Face f : simplified.faces())
		{
			patches[f.idx()] = original_faces[f];
		}

		//SurfaceNormals::compute_vertex_normals(inputMesh);
		//MeshHelpers::NormalsToBake(inputMesh);
		//MeshHelpers::ExtractMesh(inputMesh, toExtract, sub);
		//SurfaceParameterizationBis sp(sub);
		//sp.harmonic();
		//MeshHelpers::TexCoordsToPositions(sub);
		//MeshHelpers::BakeMesh(sub,"test.png");

		PatchBaker::BakePatches(inputMesh, patches);
		glViewport(0, 0, 512, 512);

//		SurfaceParameterizationBis sp2(inputMesh);
//		sp2.harmonic();
//		MeshHelpers::PositionsToBakeNormalized(inputMesh, inputMesh.bounds());
//		MeshHelpers::BakeMesh(inputMesh, "test2.png");

		//inputMesh = sub;
		//inputMesh = simplified;
#endif

#if 0
		// Reverse loop
		Color white = Color(1, 1, 1);

		auto colorsSimp = simplified.add_face_property<Color>("f:color", Color(0, 0, 0));
		auto colorsOrig = inputMesh.add_face_property<Color>("f:color", Color(0, 0, 0));
		FaceProperty<std::vector<int>> original_faces = simplified.face_property<std::vector<int>>("f:orignalFaces");

		for (auto f : simplified.faces())
		{
			original_faces[f].clear();
			original_faces[f].push_back(f.idx());
		}

		for (int iter = 0; iter<20; iter++)
		{
			for (int i = 0; i < 190; i++)
			{
				// Find smallest face
				float min = 1e23;
				Face minFace;
				for (Face f : simplified.faces())
				{
					bool isGood = true;
					if (original_faces[f].size() > iter)
					{
						isGood = false;
					}

					float area = 00.0f;
					if (isGood)
					{
						area = triangle_area(simplified, f);

						for (Halfedge h : simplified.halfedges(f))
						{
							area += triangle_area(simplified, simplified.face(simplified.opposite_halfedge(h)));
							Halfedge opo = simplified.opposite_halfedge(h);
							Halfedge toRemove = simplified.next_halfedge(opo);
							if (!simplified.is_collapse_ok(toRemove))
							{
								isGood = false;
								break;
							}

							if (original_faces[simplified.face(opo)].size() > iter)
							{
								isGood = false;
								break;
							}
						}
					}

					if (isGood && area < min)
					{
						min = area;
						minFace = f;
					}
				}

				if (!minFace.is_valid())
				{
					break;
				}

				// This does inverse loop subdivision, four triangles, a center triangle and it's  three neighbours
				// become one large triangle
				// Collapse 3 edges this removes three triangles and keeps the current face
				std::vector<Halfedge> removeList;
				for (Halfedge h : simplified.halfedges(minFace))
				{
					Halfedge opo = simplified.opposite_halfedge(h);
					Halfedge toRemove = simplified.next_halfedge(opo);
					Face f = simplified.face(toRemove);
					removeList.push_back(toRemove);
					original_faces[minFace].insert(original_faces[minFace].end(), original_faces[f].begin(), original_faces[f].end());

					Halfedge toRemoveOpo = simplified.opposite_halfedge(toRemove);
					Face fOpo = simplified.face(toRemoveOpo);
					//original_faces[minFace].insert(original_faces[minFace].end(), original_faces[fOpo].begin(), original_faces[fOpo].end());

					Halfedge whatever = inputMesh.opposite_halfedge(simplified.prev_halfedge(toRemoveOpo));
					Face wFace = simplified.face(whatever);
					original_faces[wFace].insert(original_faces[wFace].end(), original_faces[fOpo].begin(), original_faces[fOpo].end());
				}

				bool failedCollapse = false;
				for (Halfedge r : removeList)
				{
					if (!simplified.is_collapse_ok(r))
					{
						failedCollapse = true;
					}
					simplified.collapse(r);
				}

				if (failedCollapse) break;

				if (simplified.n_faces() <= 10) break;
			}
		}

		srand(2202);
		for (Face f : simplified.faces())
		{
			vec3 color = vec3((rand() % 255) / 255.0f,
				(rand() % 255) / 255.0f,
				(rand() % 255) / 255.0f);

			for (int of : original_faces[f])
			{
				colorsOrig[Face(of)] = color;
			}

			colorsSimp[f] = color;
		}
		//inputMesh = simplified;
#endif
	}

private:
    SurfaceMesh& inputMesh;
	SurfaceMesh simplified;
};
