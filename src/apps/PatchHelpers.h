#pragma once
#include <pmp/algorithms/DifferentialGeometry.h>
#include <pmp/algorithms/SurfaceNormals.h>
#include <unordered_set>
#include "ShortestPath.h"
#include "SurfaceSimplificationWithAncestors.h"
#include "MeshHelpers.h"
#include "PatchParameterization.h"
#include "PatchQuadrangulator.h"

class PatchHelpers
{
public:

	static int AllocatePatchID()
	{
		static int patchID = 1;
		return patchID++;
	}

	static void BakePatchIdsToColors(SurfaceMesh &inputMesh, const std::vector<std::vector<Face>> &patches)
	{
		std::vector<vec3> colors;
		colors.reserve(patches.size());
		for (int i = 0; i < patches.size(); i++)
		{
			vec3 color = vec3((rand() % 255) / 255.0f,
				(rand() % 255) / 255.0f,
				(rand() % 255) / 255.0f);
			colors.push_back(color);
		}

		// mark every face with the patch it belongs to
		auto prop = inputMesh.face_property<Color>("f:color");
		int patch = 0;
		for (auto faceList : patches)
		{
			for (auto face : faceList)
			{
				prop[face] = colors[patch];
			}
			patch++;
		}
	}

	static void BakePatchIdsToFacesEdges(SurfaceMesh &inputMesh, const std::vector<std::vector<Face>> &patches)
	{
		auto faceIDs = inputMesh.face_property<int>("f:patch_id");
		auto edgeIDs = inputMesh.halfedge_property<int>("h:patch_id");
		assert(faceIDs);
		assert(edgeIDs);

		// mark every face with the patch it belongs to
		for (auto faceList : patches)
		{
			int patchID = PatchHelpers::AllocatePatchID();
			for (auto face : faceList)
			{
				faceIDs[face] = patchID;
			}
		}

		// mark every edge with the patch it's opposite belongs to
		for (auto edge : inputMesh.halfedges())
		{
			edgeIDs[edge] = faceIDs[inputMesh.face(inputMesh.opposite_halfedge(edge))];
		}
	}

private:
	// helper function
	template <typename T>
	static void tfwrite(FILE* out, const T& t)
	{
		size_t n_items = fwrite((char*)&t, 1, sizeof(t), out);
		PMP_ASSERT(n_items > 0);
	}

	// helper function
	template <typename T>
	static void tfread(FILE* in, const T& t)
	{
		size_t n_items = fread((char*)&t, 1, sizeof(t), in);
		PMP_ASSERT(n_items > 0);
	}

public:

	static const int Version = 1;

	static bool WritePatch(const SurfaceMesh& mesh, const char *fileName)
	{
		// open file (in binary mode)
		FILE* out = fopen(fileName, "wb");
		if (!out)
			return false;

		// get properties
		auto vconn = mesh.get_vertex_property<SurfaceMesh::VertexConnectivity>(
			"v:connectivity");
		auto hconn = mesh.get_halfedge_property<SurfaceMesh::HalfedgeConnectivity>(
			"h:connectivity");
		auto fconn =
			mesh.get_face_property<SurfaceMesh::FaceConnectivity>("f:connectivity");
		auto point = mesh.get_vertex_property<Point>("v:point");
		auto htex = mesh.get_halfedge_property<TexCoord>("h:tex");
		auto faceIDs = mesh.get_face_property<int>("f:patch_id");
		auto edgeIDs = mesh.get_halfedge_property<int>("h:patch_id");
		auto originalEdges = mesh.get_halfedge_property<Halfedge>("h:orig_id");
		auto normals = mesh.get_vertex_property<Normal>("v:normal");

		assert(faceIDs);
		assert(edgeIDs);
		assert(originalEdges);
		assert(normals);

		// how many elements?
		unsigned int nv, ne, nh, nf;
		nv = mesh.n_vertices();
		ne = mesh.n_edges();
		nh = mesh.n_halfedges();
		nf = mesh.n_faces();

		// write header
		tfwrite(out, Version);
		tfwrite(out, nv);
		tfwrite(out, ne);
		tfwrite(out, nf);
		tfwrite(out, (bool)htex);

		// write properties to file
		fwrite((char*)vconn.data(), sizeof(SurfaceMesh::VertexConnectivity), nv,
			out);
		fwrite((char*)hconn.data(), sizeof(SurfaceMesh::HalfedgeConnectivity), nh,
			out);
		fwrite((char*)fconn.data(), sizeof(SurfaceMesh::FaceConnectivity), nf, out);
		fwrite((char*)point.data(), sizeof(Point), nv, out);
		fwrite((char*)faceIDs.data(), sizeof(int), nf, out);
		fwrite((char*)edgeIDs.data(), sizeof(int), nh, out);
		fwrite((char*)originalEdges.data(), sizeof(Halfedge), nh, out);
		fwrite((char*)normals.data(), sizeof(Normal), nv, out);

		// texture coordinates
		if (htex)
			fwrite((char*)htex.data(), sizeof(TexCoord), nh, out);

		fclose(out);
		return true;
	}

	static bool ReadPatch(SurfaceMesh& mesh, const char *fileName)
	{
		// open file (in binary mode)
		FILE* in = fopen(fileName, "rb");
		if (!in)
			return false;

		// how many elements?
		unsigned int nv(0), ne(0), nh(0), nf(0);
		int version;
		tfread(in, version);
		assert(version == Version);
		if (version != Version)
		{
			std::cout << "invalid version";
			return false;
		}
		tfread(in, nv);
		tfread(in, ne);
		tfread(in, nf);
		nh = 2 * ne;

		// texture coordinates?
		bool has_htex(false);
		tfread(in, has_htex);

		// resize containers
		mesh.vprops_.resize(nv);
		mesh.hprops_.resize(nh);
		mesh.eprops_.resize(ne);
		mesh.fprops_.resize(nf);

		// get properties
		auto vconn =
			mesh.vertex_property<SurfaceMesh::VertexConnectivity>("v:connectivity");
		auto hconn = mesh.halfedge_property<SurfaceMesh::HalfedgeConnectivity>(
			"h:connectivity");
		auto fconn =
			mesh.face_property<SurfaceMesh::FaceConnectivity>("f:connectivity");
		auto point = mesh.vertex_property<Point>("v:point");

		// read properties from file
		size_t nvc = fread((char*)vconn.data(),
			sizeof(SurfaceMesh::VertexConnectivity), nv, in);
		size_t nhc = fread((char*)hconn.data(),
			sizeof(SurfaceMesh::HalfedgeConnectivity), nh, in);
		size_t nfc = fread((char*)fconn.data(),
			sizeof(SurfaceMesh::FaceConnectivity), nf, in);
		size_t np = fread((char*)point.data(), sizeof(Point), nv, in);
		PMP_ASSERT(nvc == nv);
		PMP_ASSERT(nhc == nh);
		PMP_ASSERT(nfc == nf);
		PMP_ASSERT(np == nv);

		auto faceIDs = mesh.face_property<int>("f:patch_id");
		auto edgeIDs = mesh.halfedge_property<int>("h:patch_id");
		auto originalEdges = mesh.halfedge_property<Halfedge>("h:orig_id");
		auto normals = mesh.vertex_property<Normal>("v:normal");

		size_t nfi = fread((char*)faceIDs.data(), sizeof(int), nf, in);
		PMP_ASSERT(nfi == nf);
		size_t nhi = fread((char*)edgeIDs.data(), sizeof(int), nh, in);
		PMP_ASSERT(nhi == nh);
		size_t nho = fread((char*)originalEdges.data(), sizeof(Halfedge), nh, in);
		PMP_ASSERT(nho == nh);
		size_t nn = fread((char*)normals.data(), sizeof(Normal), nv, in);
		PMP_ASSERT(nn == nv);

		// read texture coordiantes
		if (has_htex)
		{
			auto htex = mesh.halfedge_property<TexCoord>("h:tex");
			size_t nhtc = fread((char*)htex.data(), sizeof(TexCoord), nh, in);
			PMP_ASSERT(nhtc == nh);
		}

		fclose(in);
		return true;
	}

	static void SavePatches(std::vector<SurfaceMesh> &patches,const char *prefix)
	{
		int idx = 0;
		for (auto mesh : patches)
		{
			//skip emtpy
			if ( mesh.n_vertices() == 0 || mesh.n_faces() == 0) continue;
			char buffer[512];
			sprintf(buffer, "%s_%i.patch", prefix, idx);
			WritePatch(mesh, buffer);
			idx++;			
		}
	}

	static void LoadPatches(const char *prefix, std::vector<SurfaceMesh*> &outPatches)
	{
		outPatches.clear();

		int idx = 0;
		while (true)
		{
			char buffer[512];
			sprintf(buffer, "%s_%i.patch", prefix, idx);
			SurfaceMesh *sm = new SurfaceMesh();
			bool ok = ReadPatch(*sm, buffer);
			if (!ok) break;
			outPatches.emplace_back(sm);
			idx++;

			// Hack debug single patch
			//return;
		}

		/*
		To debug a specific patch id
		SurfaceMesh temp;
		for (SurfaceMesh &m : outPatches)
		{
			auto facePatchIDs = m.get_face_property<int>("f:patch_id");
			assert(facePatchIDs);
			int originalPatchID = facePatchIDs[*m.faces_begin()];
			if (originalPatchID == 9)
			{
				temp = m;
				break;
			}
		}

		outPatches.clear();
		outPatches.push_back(temp);*/
	}

	static void MakePatches(SurfaceMesh &inputMesh, const std::vector<std::vector<Face>> &patches, std::vector<SurfaceMesh> &outAppendTo)
	{
		int idx = 0;
		for (auto faceList : patches)
		{
			// Make a mini mesh for the patch
			SurfaceMesh sub;
			MeshHelpers::ExtractMesh(inputMesh, faceList, sub);

			outAppendTo.emplace_back(sub);
		}
	}
};
