#pragma once
#include <pmp/algorithms/DifferentialGeometry.h>
#include <pmp/algorithms/SurfaceNormals.h>
#include <unordered_set>
#include <string>
#include "ShortestPath.h"
#include "SurfaceSimplificationWithAncestors.h"
#include "MeshHelpers.h"
#include "PatchParameterization.h"
#include "PatchBaker.h"
#include "ElCheapoClust.h"

class ClusteringExp
{
public:

	ClusteringExp(SurfaceMesh &setOutputMesh) : outputMesh(setOutputMesh){}


	enum class ClusterMethod
	{
		Simplification,
		ElCheapo
	};

	struct JobInfo
	{
		std::string name;
		std::string fileName; // input mesh
		std::string textureFileName; // albeido texture
		bool cluster = false;
		bool bake = true;
		ClusterMethod method = ClusterMethod::ElCheapo;
	};

	void process(JobInfo &job)
	{
		/*bool cluster = false;
		bool bake = true;
		ClusterMethod method = ClusterMethod::ElCheapo;*/

		std::string boundsName = job.name + "_out.bounds";
		std::string polyPatchPrefix = job.name + "_patch";

		// Edge collapse
		if (job.cluster)
		{
			SurfaceMesh inputMesh;
			std::cout << "Parsing input mesh: " << job.fileName << "\n";
			if (!inputMesh.read(job.fileName))
			{
				std::cout << "Failed reading: " << job.fileName << "\n";
				return;
			}

			assert(inputMesh.n_faces());
			assert(inputMesh.n_vertices());
			SurfaceMesh simplified = inputMesh; //deep copy

			std::vector<std::vector<Face>> patches;

			std::cout << "Clustering\n";
			if (job.method == ClusterMethod::Simplification)
			{

				SurfaceSimplificationWithAncestors ss(simplified);
				auto delta = simplified.bounds().max() - simplified.bounds().max();
				//float aspectRatio = 0.0f;
				//float edgeLength = std::max(std::max(delta[0], delta[1]), delta[2]) / 128.0f;
				ss.initialize();
				ss.simplify(simplified.n_vertices() / 1000);

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

				patches.resize(simplified.n_faces());
				// first face
				for (Face f : simplified.faces())
				{
					patches[f.idx()] = original_faces[f];
				}

			}
			else if (job.method == ClusterMethod::ElCheapo)
			{

				ElCheapoClust clust(inputMesh);
				clust.process(12);
				patches = clust.clusters;
			}

			SurfaceNormals::compute_vertex_normals(inputMesh);
			PatchHelpers::BakePatchIdsToFacesEdges(inputMesh, patches);
			PatchHelpers::BakePatchIdsToColors(inputMesh, patches);

			// For every halfedge save the id on the original mesh, This will then get passed along to the patches
			// so they can find out where they came from originally
			auto originalHalfedges = inputMesh.halfedge_property<Halfedge>("h:orig_id");
			for (Halfedge e : inputMesh.halfedges())
			{
				originalHalfedges[e] = e;
			}

			std::vector<SurfaceMesh> meshPatches;
			PatchHelpers::MakePatches(inputMesh, patches, meshPatches);

			// Save and load for testing
			PatchHelpers::SavePatches(meshPatches, polyPatchPrefix.c_str());

			FILE *f = fopen(boundsName.c_str(),"wb");
			auto bounds = inputMesh.bounds();
			fwrite(&bounds, sizeof(bounds), 1, f);
			fclose(f);

			outputMesh = inputMesh;
		}

		if (job.bake)
		{
			FILE *f = fopen(boundsName.c_str(), "rb");
			BoundingBox bounds;
			fread(&bounds, sizeof(bounds), 1, f);
			fclose(f);

			std::vector<SurfaceMesh*> meshPatchesBis;
			PatchHelpers::LoadPatches(polyPatchPrefix.c_str(), meshPatchesBis);

			//		PatchBaker::BakePatches(meshPatches, inputMesh.bounds());
			std::vector<PatchQuadrangulator::QuadrangularPatch*> quadPatches;
			PatchBaker::QuadPatches(meshPatchesBis, quadPatches);

			Texture2D tex;
			if (job.textureFileName.size() && tex.Load(job.textureFileName.c_str()))
			{
				PatchBaker::BakePatches(quadPatches, &tex, bounds, job.name.c_str());
			}
			else
			{
				PatchBaker::BakePatches(quadPatches, NULL, bounds, job.name.c_str());
			}


			SurfaceMesh agg;
			for (PatchQuadrangulator::QuadrangularPatch* quad : quadPatches)
			{
				MeshHelpers::AppendMesh(agg, quad->mesh);
			}

			outputMesh = agg;
		}

		glViewport(0, 0, 512, 512);

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

	SurfaceMesh &outputMesh;
};
