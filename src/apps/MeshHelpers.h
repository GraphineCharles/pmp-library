#pragma once
#include <pmp/algorithms/DifferentialGeometry.h>
#include <unordered_set>
#include "ShortestPath.h"
#include <unordered_set>
#include <unordered_map>
#include "FrameBuffer.h"
#include "lodepng/lodepng.h"

/**
 * Allow pmp::Handles and derived classes to be stored in std containers like
 * std::unordered_map and std::unordered_set
 */
namespace std {
	template <> struct hash<pmp::Handle>
	{
		size_t operator()(const pmp::Handle & x) const
		{
			return hash<int>()(x.idx());
		}
	};

	template <> struct hash<pmp::Vertex>
	{
		size_t operator()(const pmp::Vertex & x) const
		{
			return hash<pmp::Handle>()(x);
		}
	};

	template <> struct hash<pmp::Edge>
	{
		size_t operator()(const pmp::Edge & x) const
		{
			return hash<pmp::Handle>()(x);
		}
	};

	template <> struct hash<pmp::Halfedge>
	{
		size_t operator()(const pmp::Halfedge & x) const
		{
			return hash<pmp::Handle>()(x);
		}
	};

	template <> struct hash<pmp::Face>
	{
		size_t operator()(const pmp::Face & x) const
		{
			return hash<pmp::Handle>()(x);
		}
	};
}

#define myassert(pred) if (!(pred)) {	__debugbreak();	}

class MeshHelpers
{
public:

	static void triangulate_with_properties(SurfaceMesh &mesh, Face f)
	{
		// Split an arbitrary face into triangles by connecting each vertex of face
		// f after its second to vertex v. Face f will remain valid (it will become
		// one of the triangles). The halfedge handles of the new triangles will
		// point to the old halfedges.

		Halfedge baseH = mesh.halfedge(f);
		Vertex startV = mesh.from_vertex(baseH);
		Halfedge nextH = mesh.next_halfedge(baseH);

		while (mesh.to_vertex(mesh.next_halfedge(nextH)) != startV)
		{
			Halfedge nextNextH(mesh.next_halfedge(nextH));

			Face newF = mesh.new_face();
			CopyFaceProperties(mesh, f, newF);

			mesh.set_halfedge(newF, baseH);

			Halfedge newH = mesh.new_edge(mesh.to_vertex(nextH), startV);
			assert(mesh.to_vertex(nextH) == mesh.to_vertex(mesh.opposite_halfedge(newH)));
			CopyHalfEdgeProperties(mesh, nextH, newH);

			mesh.set_next_halfedge(baseH, nextH);
			mesh.set_next_halfedge(nextH, newH);
			mesh.set_next_halfedge(newH, baseH);

			mesh.set_face(baseH, newF);
			mesh.set_face(nextH, newF);
			mesh.set_face(newH, newF);

			baseH = mesh.opposite_halfedge(newH);
			nextH = nextNextH;
		}
		mesh.set_halfedge(f, baseH); //the last face takes the handle baseH

		mesh.set_next_halfedge(baseH, nextH);
		mesh.set_next_halfedge(mesh.next_halfedge(nextH), baseH);

		mesh.set_face(baseH, f);
	}

	static void AppendMesh(SurfaceMesh &appendTo, const SurfaceMesh &toAppend)
	{
		std::unordered_map<Vertex, Vertex> vertexRemap;
		std::unordered_map<Face, Face> faceRemap;
		appendTo.add_properties(toAppend);

		for (Vertex v : toAppend.vertices())
		{
			Vertex newV = appendTo.add_vertex(toAppend.position(v));
			vertexRemap.insert({ v, newV });
		}

		for (Face f : toAppend.faces())
		{
			std::vector<Vertex> verts;
			for (Vertex v : toAppend.vertices(f))
			{
				verts.push_back(Vertex(vertexRemap[v]));
			}
			if ( verts.size() < 3) continue;
			Face added = appendTo.add_face(verts);
			if (added.is_valid())
			{
			//	assert(added.is_valid());
				faceRemap.insert({ f, added });
			}
		}

		CopyVertexProperties(toAppend, appendTo, vertexRemap);
		CopyFaceProperties(toAppend, appendTo, faceRemap);
		CopyHalfEdgeProperties(toAppend, appendTo, faceRemap);
		CopyEdgeProperties(toAppend, appendTo, faceRemap);
	}

	static void  ExtractMesh(const SurfaceMesh &source, const std::vector<Face> &sourceFaces, SurfaceMesh &dest)
	{
		std::unordered_map<Vertex, Vertex> vertexRemap;
		std::unordered_map<Face, Face> faceRemap;
		dest.add_properties(source);

		vertexRemap.reserve(sourceFaces.size() * 3);
		faceRemap.reserve(sourceFaces.size());

		std::vector<Vertex> verts;
		verts.reserve(16);

		for (Face f : sourceFaces)
		{
			verts.clear();
			// Add new vertices if not existing yet
			for (Vertex v : source.vertices(f))
			{
				if (vertexRemap.find(v) == vertexRemap.end())
				{
					Vertex remapped = dest.add_vertex(source.position(v));
					vertexRemap.insert({ v, remapped });
				}
				verts.push_back(vertexRemap[v.idx()]);
			}

			// Filter out bad faces
			if (verts.size() > 2)
			{
				Face added = dest.add_face(verts);
				faceRemap.insert({ f, added });
			}
		}

		CopyVertexProperties(source, dest, vertexRemap);
		CopyFaceProperties(source, dest, faceRemap);
		CopyHalfEdgeProperties(source, dest, faceRemap);
		CopyEdgeProperties(source, dest, faceRemap);
	}

	static void  CopyVertexProperties(const SurfaceMesh &source, SurfaceMesh &dest, std::unordered_map<Vertex, Vertex> vertexRemap)
	{
		auto vprops = source.vertex_properties();
		for (auto pname : vprops)
		{
			if (dest.is_reserved_property(pname)) continue;

			auto srcprop = source.get_vertex_property_base(pname);
			auto dstprop = dest.get_vertex_property_base(pname);

			for (auto mapped : vertexRemap)
			{
				srcprop->copy_to(mapped.first.idx(), dstprop, mapped.second.idx());
			}
		}
	}

	// SLOW! use CopyFaceProperties with a list if you need to to a lot of copying to make it faster
	static void CopyFaceProperties(SurfaceMesh &source, Face a, Face b)
	{
		auto fprops = source.face_properties();
		for (auto pname : fprops)
		{
			if (source.is_reserved_property(pname)) continue;
			auto srcprop = source.get_face_property_base(pname);
			srcprop->copy_to(a.idx(), srcprop, b.idx());
		}
	}

	static void CopyFaceProperties(const SurfaceMesh &source, SurfaceMesh &dest, std::unordered_map<Face, Face> vertexRemap)
	{
		auto fprops = source.face_properties();
		for (auto pname : fprops)
		{
			if (dest.is_reserved_property(pname)) continue;

			auto srcprop = source.get_face_property_base(pname);
			auto dstprop = dest.get_face_property_base(pname);

			for (auto mapped : vertexRemap)
			{
				srcprop->copy_to(mapped.first.idx(), dstprop, mapped.second.idx());
			}
		}
	}

	static void CopyHalfEdgeProperties(SurfaceMesh &source, Halfedge src, Halfedge dst)
	{
		auto eprops = source.halfedge_properties();
		for (auto pname : eprops)
		{
			if (source.is_reserved_property(pname)) continue;
			auto srcprop = source.get_halfedge_property_base(pname);
			srcprop->copy_to(src.idx(), srcprop, dst.idx());
		}
	}

	static void CopyHalfEdgeProperties(const SurfaceMesh &source, SurfaceMesh &dest, std::unordered_map<Face, Face> faceRemap)
	{
		auto eprops = source.halfedge_properties();
		for (auto pname : eprops)
		{
			if (dest.is_reserved_property(pname)) continue;

			auto srcprop = source.get_halfedge_property_base(pname);
			auto dstprop = dest.get_halfedge_property_base(pname);

			for (auto mapped : faceRemap)
			{
				for (auto srcEdge = source.halfedges(mapped.first).begin(), dstEdge = dest.halfedges(mapped.second).begin();
					srcEdge != source.halfedges(mapped.first).end() &&
					dstEdge != dest.halfedges(mapped.second).end();
					++srcEdge, ++dstEdge)
				{
					srcprop->copy_to( (*srcEdge).idx(), dstprop, (*dstEdge).idx());
					Halfedge dstOpo = dest.opposite_halfedge(*dstEdge);
					// Take special care to copy from boundary haledges also. There is no face for those to remap
					// but the halfedges on the boundary to actualy exist
					if (dest.is_boundary(dstOpo))
					{
						Halfedge srcOpo = source.opposite_halfedge(*srcEdge);
						srcprop->copy_to(srcOpo.idx(), dstprop, dstOpo.idx());
					}
				}
			}
		}
	}

	static void CopyEdgeProperties(const SurfaceMesh &source, SurfaceMesh &dest, std::unordered_map<Face, Face> faceRemap)
	{
		// FIXME: This will copy everything twice for shared halfedges
		auto eprops = source.edge_properties();
		for (auto pname : eprops)
		{
			if (dest.is_reserved_property(pname)) continue;

			auto srcprop = source.get_edge_property_base(pname);
			auto dstprop = dest.get_edge_property_base(pname);

			for (auto mapped : faceRemap)
			{
				for (auto srcEdge = source.halfedges(mapped.first).begin(), dstEdge = dest.halfedges(mapped.second).begin();
					srcEdge != source.halfedges(mapped.first).end() &&
					dstEdge != dest.halfedges(mapped.second).end();
					++srcEdge, ++dstEdge)
				{
					Edge srcE = source.edge(*srcEdge);
					Edge dstE = dest.edge(*dstEdge);
					srcprop->copy_to(srcE.idx(), dstprop, dstE.idx());
				}
			}
		}
	}

	static void BackupPositions(SurfaceMesh &source)
	{
		auto points = source.vertex_property<Point>("v:point");
		auto backup = source.vertex_property<Point>("v:backup");

		for (Vertex v : source.vertices())
		{
			backup[v] = points[v];
		}
	}

	static void RestorePositions(SurfaceMesh &source)
	{
		auto points = source.vertex_property<Point>("v:point");
		auto backup = source.vertex_property<Point>("v:backup");

		for (Vertex v : source.vertices())
		{
			points[v] = backup[v];
		}
	}


	static void TexCoordsToPositions(SurfaceMesh &source)
	{
		auto points = source.vertex_property<Point>("v:point");
		auto tex = source.vertex_property<TexCoord>("v:tex");

		for (Vertex v : source.vertices())
		{
			TexCoord t = tex[v];
			Point p = Point(t[0], t[1], 0.0f);
			points[v] = p;
		}
	}

	static void NormalsToBake(SurfaceMesh &source)
	{
		auto normals = source.vertex_property<Normal>("v:normal");
		auto tex = source.vertex_property<Vector4>("v:bake");

		for (Vertex v : source.vertices())
		{
			Normal n = normals[v];
			
			tex[v] = Vector4(n[0] * 0.5f + 0.5f, n[1] * 0.5f + 0.5f, n[2] * 0.5f + 0.5f, 1.0);
		}
	}

	static void  TexCoordsToBake(SurfaceMesh &source)
	{
		auto tex = source.vertex_property<TexCoord>("v:tex");
		auto bake = source.vertex_property<Vector4>("v:bake");

		for (Vertex v : source.vertices())
		{
			TexCoord n = tex[v];
			bake[v] = Vector4(n[0], n[1], 0.0f, 1.0);
		}
	}

	static void  PositionsToBakeNormalized(SurfaceMesh &source, const BoundingBox &normalizationBox)
	{
		auto points = source.vertex_property<Point>("v:point");
		auto bake = source.vertex_property<Vector4>("v:bake");

		Point offset = -normalizationBox.min();
		Point size = normalizationBox.max() - normalizationBox.min();
		Point scale = Point(1.0f / size[0], 1.0f / size[1], 1.0f / size[2]);


		for (Vertex v : source.vertices())
		{
			Point n = (points[v] + offset);
			n[0] *= scale[0];
			n[1] *= scale[1];
			n[2] *= scale[2];
			bake[v] = Vector4(n[0], n[1], n[2], 1.0);
		}
	}

	static void BakeMesh(SurfaceMesh &source, Texture2D *optionalSourceTexture, int resolution, Texture2D &destination, int destX, int destY)
	{
		std::cout << "Baking mesh " << std::endl;
		FrameBuffer f;
		f.Initialize(resolution, resolution, destination.bpp);
		f.BindForRendering();

		SurfaceMeshGL glCopy;
		SurfaceMesh &asSimple = glCopy;
		asSimple = source;

		float halfPixel = 0.5f / resolution;
		float halfPixelInsetWidth = 1.0f - halfPixel * 2.0f;

		mat4 transform = translation_matrix(vec3(halfPixel, halfPixel, halfPixel)) * scaling_matrix(vec3(halfPixelInsetWidth, halfPixelInsetWidth, halfPixelInsetWidth));
		transform = translation_matrix(vec3(-1.0f,-1.0f,-1.0f)) * scaling_matrix(vec3(2.0f, 2.0f, 2.0f)) * transform;
		if (optionalSourceTexture)
		{
			glCopy.set_texture(optionalSourceTexture->handle);
			glCopy.draw(mat4::identity(), transform, "BakeTex");
		}
		else
		{
			glCopy.draw(mat4::identity(), transform, "Bake");
		}
		f.ReadPixels();


		size_t pixelSize = (destination.bpp / 8);
		size_t scanlineSize = resolution * pixelSize;
		size_t destSanlineSize = destination.width * pixelSize;
		unsigned char *dest = destination.image + destY * destSanlineSize + destX * pixelSize;
		unsigned char *src = f.image.image;
		for (int line = 0; line < resolution; line++)
		{
			memcpy(dest, src, scanlineSize);
			dest += destSanlineSize;
			src += scanlineSize;
		}
	}

	static std::vector<SurfaceMesh> errorMeshes;

	static void ErrorEdge(SurfaceMesh &mesh, Halfedge e, const std::string &message)
	{
		ErrorEdge(mesh, mesh.edge(e), message);
	}

	static void ErrorEdge(SurfaceMesh &mesh, Edge e, const std::string &message)
	{
		errorMeshes.push_back(SurfaceMesh());
		SurfaceMesh &errorMesh = errorMeshes[errorMeshes.size() - 1];
		errorMesh = mesh;
		auto efeature = errorMesh.edge_property<bool>("e:feature");
		efeature[e] = true;

		Face f = errorMesh.face(errorMesh.halfedge(e, 0));
		if (!f.is_valid())
		{
			f = errorMesh.face(errorMesh.halfedge(e, 1));
		}

		auto htex = errorMesh.halfedge_property<TexCoord>("h:tex");
		errorMesh.remove_halfedge_property(htex);

		auto faceColor = errorMesh.face_property<Color>("f:color");
		for (Face f : errorMesh.faces())
		{
			faceColor[f] = Color(0.7f, 0.7f, 0.7f);
		}

		if (f.is_valid())
		{
			faceColor[f] = Color(1.0f, 0.0, 0.0f);
		}

	}

	static void ErrorMesh(SurfaceMesh &mesh, const std::string &message)
	{
		errorMeshes.push_back(SurfaceMesh());
		SurfaceMesh &errorMesh = errorMeshes[errorMeshes.size() - 1];
		errorMesh = mesh;

		auto htex = errorMesh.halfedge_property<TexCoord>("h:tex");
		errorMesh.remove_halfedge_property(htex);

		auto faceColor = errorMesh.face_property<Color>("f:color");
		for (Face f : errorMesh.faces())
		{
			faceColor[f] = Color(0.7f, 0.7f, 0.7f);
		}
	}

	static void ErrorMesh(SurfaceMesh &mesh, SurfaceMesh &mesh2, const std::string &message)
	{
		errorMeshes.push_back(SurfaceMesh());
		SurfaceMesh &errorMesh = errorMeshes[errorMeshes.size() - 1];
		errorMesh = mesh;

	/*	auto faceColor = errorMesh.face_property<Color>("f:color");
		for (Face f : errorMesh.faces())
		{
			faceColor[f] = Color(0.7f, 0.7f, 0.7f);
		}*/

		AppendMesh(errorMesh, mesh2);

		auto htex = errorMesh.halfedge_property<TexCoord>("h:tex");
		errorMesh.remove_halfedge_property(htex);
	

	}
};