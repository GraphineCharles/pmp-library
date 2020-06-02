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


class MeshHelpers
{
public:

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
			Face added = appendTo.add_face(verts);
			faceRemap.insert({ f, added });
		}

		CopyVertexProperties(toAppend, appendTo, vertexRemap);
		CopyFaceProperties(toAppend, appendTo, faceRemap);
	}

	static void  ExtractMesh(const SurfaceMesh &source, const std::vector<Face> &sourceFaces, SurfaceMesh &dest)
	{
		std::unordered_map<Vertex, Vertex> vertexRemap;
		std::unordered_map<Face, Face> faceRemap;
		dest.add_properties(source);

		for (Face f : sourceFaces)
		{
			std::vector<Vertex> verts;
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
			Face added = dest.add_face(verts);
			faceRemap.insert({ f, added });
		}

		CopyVertexProperties(source, dest, vertexRemap);
		CopyFaceProperties(source, dest, faceRemap);
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

	static void BakeMesh(SurfaceMesh &source, int resolution, Texture2D &destination, int destX, int destY)
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
		glCopy.draw(mat4::identity(), transform, "Bake");
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
};