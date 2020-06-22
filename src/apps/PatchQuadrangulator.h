#pragma once
#include <pmp/algorithms/DifferentialGeometry.h>
#include <pmp/algorithms/SurfaceNormals.h>
#include <unordered_set>
#include "ShortestPath.h"
#include "SurfaceSimplificationWithAncestors.h"
#include "MeshHelpers.h"
#include "PatchParameterization.h"
#include "PatchHelpers.h"

class PatchQuadrangulator
{
public:

	// First create a list of cluster/cluser edges
	struct PatchEdge
	{
		int neighbour;
		int neighbourEdgeIndex; // edge number on the neighbour
		Vertex center;
		std::vector<Halfedge> edges; // half edge path on cluster A between the two clusters
	};

	struct PolygonPatch
	{
		int patchID; // id of this quad patch
		SurfaceMesh *mesh;
		std::vector<PatchEdge> edges;
	};

	struct QuadrangularPatch
	{
		int patchID; // id of this quad patch
		int originalPatchID; // id of the patch before quadrangulation
		SurfaceMesh mesh;
		std::vector<PatchEdge> edges;
		int gridX;
		int gridY;
	};

	static void GenerateBoundaryInfo(SurfaceMesh &patch, std::vector<PatchEdge> &strips)
	{
		strips.clear();
		Halfedge firstEdge;
		auto edgeIDs = patch.halfedge_property<int>("h:patch_id");
		if (!edgeIDs)
		{
			std::cerr << "Mesh has no edge ids." << std::endl;
			assert(false);
			return;
		}

		// find 1st boundary vertex
		for (auto edge : patch.halfedges())
		{
			if (patch.is_boundary(edge))
			{
				firstEdge = edge;
				break;
			}
		}

		// no boundary found ?
		if (!firstEdge.is_valid())
		{
			std::cerr << "Mesh has no boundary." << std::endl;
			assert(false);
			return;
		}

		// Create strips of edges that have the same neighbour patch id
		Halfedge edge = firstEdge; 
		PatchEdge current;
		current.neighbour = edgeIDs[patch.opposite_halfedge(edge)];
		int numEdges = 0;
		do 
		{
			Halfedge opo = patch.opposite_halfedge(edge);
			// A new neighbour strip has started
			if (edgeIDs[opo] != current.neighbour)
			{
				strips.push_back(current);
				current.edges.clear();
				current.edges.push_back(edge);
				current.neighbour = edgeIDs[opo];
			}
			else
			{
				// add to current strip
				current.edges.push_back(edge);
			}
			edge = patch.next_halfedge(edge);
			numEdges++;
		} while (edge != firstEdge);

		// If the intial vertex we found on the edge above. Is not the first in the strip. The final bit of the strip will contain the remaining
		// verts of that first strip so we merge them.
		if (strips.size() && current.neighbour == strips[0].neighbour)
		{
			current.edges.insert(current.edges.end(), strips[0].edges.begin(), strips[0].edges.end());
			strips[0].edges = current.edges;
		}
		else if ( current.edges.size() )
		{
			strips.push_back(current);
		}

		// Validate results
#if 1
		int total = 0;
		int count = 0;
		for (auto strip : strips)
		{
			total += strip.edges.size();

			auto next = strips[(count + 1) % strips.size()];
			assert(patch.to_vertex(strip.edges[strip.edges.size() - 1]) == patch.from_vertex(next.edges[0]));
			count++;
		}
#endif

		assert(total == numEdges);
	}

	static void MarkVertex(SurfaceMesh m, Vertex v)
	{
		// Debug mark edges around vert as feature so we can see it in viewer
		auto efeature = m.edge_property<bool>("e:feature");
		for (auto e : m.halfedges(v))
		{
			efeature[m.edge(e)] = true;
		}
	}

	// find vertex nearest to uv center
	static Vertex FindCenterVertex(SurfaceMesh &patch)
	{
		auto tex = patch.get_vertex_property<TexCoord>("v:tex");
		assert(tex);
		float minDist = 1e23;
		Vertex found;

		for (Vertex v : patch.vertices())
		{
			if (patch.is_isolated(v)) continue;
			if (patch.is_boundary(v)) continue;
			TexCoord centered = tex[v] - TexCoord(0.5f, 0.5f);
			float dist = norm(centered);
			if (dist < minDist)
			{
				minDist = dist;
				found = v;
			}
		}

/*		int valence = patch.valence(found);
		int origValence = valence;

		std::vector<Vertex> neighneigh;

		for (Vertex neigh : patch.vertices(found))
		{
			int newVal = patch.valence(neigh);
			if (newVal < valence)
			{
				valence = newVal;
				found = neigh;
			}

			for (Vertex nn : patch.vertices(neigh))
			{
				neighneigh.push_back(nn);
			}
		}

		for (Vertex neigh : neighneigh)
		{
			int newVal = patch.valence(neigh);
			if (newVal > valence)
			{
				valence = newVal;
				found = neigh;
			}
		}

		if (valence < origValence)
		{
			std::cout << "Updated center vertex valence from " << origValence << " to " << valence << "\n";
		}*/

		MarkVertex(patch, found);
		return found;
	}

	static Vertex FindCenterVertex(SurfaceMesh &patch, PatchEdge &strip)
	{
		// We again differ from the paper we don't find the center in 3D distance but in edge counts.
		// If you have a pretty uniformly tessellated mesh this is pretty much the same though.

		// odd number of edges == even verts so the center is ill defined
		// depending on the order in which we walk the edge (which is opposite for the adjacent patch)
		// so we tie break by something order independent hare arbitrarily the x coord of the vertex
		// We could obviously communicate the choice to the neighbor etc but this makes it more parallelizable
		if ( (strip.edges.size() & 1) != 0)
		{
			int mid = strip.edges.size() / 2;
			Vertex v1 = patch.from_vertex(strip.edges[mid]);
			Vertex v2 = patch.to_vertex(strip.edges[mid]);
			if (patch.position(v1)[0] != patch.position(v2)[0])
			{
				return (patch.position(v1)[0] < patch.position(v2)[0]) ? v1 : v2;
			}
			else if (patch.position(v1)[1] != patch.position(v2)[1])
			{
				return (patch.position(v1)[1] < patch.position(v2)[1]) ? v1 : v2;
			}
			else if (patch.position(v1)[2] != patch.position(v2)[2])
			{
				return (patch.position(v1)[2] < patch.position(v2)[2]) ? v1 : v2;
			}
			else
			{
				std::cout << "Identical vertices, center vertex tie breaking will fail";
				return v1;
			}
		}
		// even number of edges == odd verts so the center one is well defined
		else
		{
			int mid = strip.edges.size() / 2;
			return patch.from_vertex(strip.edges[mid]);
		}

	}

	static void FloodFillFaces(SurfaceMesh &patch, Face f, std::unordered_set<Halfedge> excludedEdges, std::vector<Face> &outFaces)
	{
		std::unordered_set<Face> visited;
		std::vector<Face> toProcess;
		toProcess.push_back(f);

		while (toProcess.size() != 0)
		{
			Face f = toProcess[toProcess.size() - 1];
			toProcess.pop_back();
			visited.insert(f);
			outFaces.push_back(f);

			for (Halfedge e : patch.halfedges(f))
			{
				if (excludedEdges.find(e) == excludedEdges.end())
				{
					Face newF = patch.face(patch.opposite_halfedge(e));
					if (!newF.is_valid()) continue;
					if (visited.find(newF) == visited.end())
					{
						toProcess.push_back(newF);
					}
				}
			}
		}
	}

	// https://stackoverflow.com/questions/563198/how-do-you-detect-where-two-line-segments-intersect
	// Returns 1 if the lines intersect, otherwise 0. In addition, if the lines 
	// intersect the intersection point may be stored in the floats i_x and i_y.
	static bool GetIntersection(TexCoord p0, TexCoord p1,
		TexCoord p2, TexCoord p3, TexCoord &out)
	{
		TexCoord s1, s2;
		s1[0] = p1[0] - p0[0];     s1[1] = p1[1] - p0[1];
		s2[0] = p3[0] - p2[0];     s2[1] = p3[1] - p2[1];

		float s, t;
		s = (-s1[1] * (p0[0] - p2[0]) + s1[0] * (p0[1] - p2[1])) / (-s2[0] * s1[1] + s1[0] * s2[1]);
		t = (s2[0] * (p0[1] - p2[1]) - s2[1] * (p0[0] - p2[0])) / (-s2[0] * s1[1] + s1[0] * s2[1]);

		if (s >= 0 && s <= 1 && t >= 0 && t <= 1)
		{
			// Collision detected
			out[0] = p0[0] + (t * s1[0]);
			out[1] = p0[1] + (t * s1[1]);
			return true;
		}

		return false; // No collision
	}

#if 0
	static bool CarvePath(SurfaceMesh &patch, Vertex v1, Vertex v2, std::vector<Halfedge> &newEdges)
	{
		newEdges.clear();

		auto tex = patch.get_vertex_property<TexCoord>("v:tex");
		auto htex = patch.get_halfedge_property<TexCoord>("h:tex");
		auto normals = patch.vertex_property<Normal>("v:normal");

		TexCoord uv0 = tex[v1];
		TexCoord uv1 = tex[v2];

		struct CarveEdge
		{
			Halfedge h;
			TexCoord intersect;
			float dist;
			float lerp;
		};

		auto efeature = patch.edge_property<bool>("e:feature");

		std::vector<CarveEdge> edgesToSplit;

		for (Edge e : patch.edges())
		{
			Halfedge h = patch.halfedge(e, 0);
			TexCoord euv0 = tex[patch.from_vertex(h)];
			TexCoord euv1 = tex[patch.to_vertex(h)];
			TexCoord inter;
			if (GetIntersection(uv0, uv1, euv0, euv1, inter))
			{
				if (patch.from_vertex(h) == v1 ||
					patch.to_vertex(h) == v1 ||
					patch.from_vertex(h) == v2 ||
					patch.to_vertex(h) == v2)
				{
					float lerp = norm(euv0 - inter) / norm(euv0 - euv1);
					// Check that we're interescting at the end ponts which is ok but otherwise something's wrong
					if (lerp > 0.01f && lerp < 0.99f)
					{
						std::cout << "Intersected start/end edge at invalid lerp " << lerp << "\n";
					}
					continue;
				}

				if (patch.is_boundary(e))
				{
					std::cout << "Intersected boundary edge.\n";
				}

				CarveEdge save;
				save.h = h;
				save.intersect = inter;
				save.dist = norm(uv0 - inter);
				save.lerp = norm(euv0 - inter) / norm(euv0 - euv1);
				if (save.lerp > 0.99f) save.lerp = 0.99f;
				edgesToSplit.push_back(save);
				//efeature[e] = true;
			}
		}

		if (edgesToSplit.size() == 0)
		{
			std::cout << "No edges to split.\n";
		}
		else
		{
			std::cout << "Carving " << edgesToSplit.size() << " edges.\n";
		}

		std::sort(edgesToSplit.begin(), edgesToSplit.end(), [](const CarveEdge &a, const CarveEdge &b) { return a.dist < b.dist; });

		std::vector<Face> toTriangulate;

		Vertex previous = v1;
		for (CarveEdge &e : edgesToSplit)
		{
			Face face;
			Halfedge toSplit;
			for (Face ring : patch.faces(previous))
			{
				if (ring == patch.face(e.h))
				{
					face = ring;
					toSplit = e.h;
					break;
				}
				else if (ring == patch.face(patch.opposite_halfedge(e.h)))
				{
					face = ring;
					toSplit = patch.opposite_halfedge(e.h);
					break;
				}
			}

			if (!face.is_valid())
			{
				MeshHelpers::ErrorEdge(patch, e.h, "Could not find next face, aborting carve.");
				return false;
			}

			myassert(face.is_valid());
			myassert(toSplit.is_valid());

			Halfedge fromEdge;
			for (Halfedge fe : patch.halfedges(face))
			{
				if (patch.to_vertex(fe) == previous)
				{
					fromEdge = fe;
					break;
				}
			}

			myassert(fromEdge.is_valid());
			myassert(patch.face(fromEdge) == patch.face(toSplit));
			
			// Calculate interpolated properties
			//Point p = (patch.position(patch.from_vertex(e.h)) + patch.position(patch.to_vertex(e.h))) * 0.5f;
			//TexCoord t = (tex[patch.from_vertex(e.h)] + tex[patch.to_vertex(e.h)]) * 0.5f;
			float oneMinus = 1.0f - e.lerp;
			Point p = patch.position(patch.from_vertex(e.h)) * oneMinus + patch.position(patch.to_vertex(e.h)) *e.lerp;
			TexCoord t = tex[patch.from_vertex(e.h)] * oneMinus + tex[patch.to_vertex(e.h)] * e.lerp;

			Normal n = normalize(normals[patch.from_vertex(e.h)] + normals[patch.to_vertex(e.h)]);
			TexCoord ht0 = (htex[patch.prev_halfedge(e.h)] + htex[e.h]) * 0.5f;
			TexCoord ht1 = (htex[patch.prev_halfedge(patch.opposite_halfedge(e.h))] + htex[patch.opposite_halfedge(e.h)]) * 0.5f;

			// Add vertex to the edge and make sure all halfedge properties are moved over
			Vertex newVert = patch.add_vertex(p);
			tex[newVert] = t;
			normals[newVert] = n;
			Halfedge newEdge = patch.insert_vertex(toSplit, newVert);

			TexCoord backup = htex[toSplit];
			htex[toSplit] = ht0;
			htex[newEdge] = ht1;
			htex[patch.opposite_halfedge(newEdge)] = backup;

			// Split the face again making sure halfedge properties are moved
			Halfedge newCrossEdge = patch.insert_edge(fromEdge, toSplit);
			htex[newCrossEdge] = htex[toSplit];
			htex[patch.opposite_halfedge(newCrossEdge)] = htex[fromEdge];

			efeature[patch.edge(newCrossEdge)] = true;


			newEdges.push_back(newCrossEdge);

			// Triangulate both new created faces
			toTriangulate.push_back(patch.face(newCrossEdge));
			toTriangulate.push_back(patch.face(patch.opposite_halfedge(newCrossEdge)));

			//assert(patch.to_vertex(newEdge) == patch.to_vertex(newCrossEdge));

			previous = patch.to_vertex(newEdge);
		}

		// Final step from previous to v2
		{
			Face face;
			for (Face ring1 : patch.faces(previous))
			{
				for (Face ring2 : patch.faces(v2))
				{
					if (ring1 == ring2)
					{
						face = ring1;
					}
				}
			}

//			assert(face.is_valid());
			if (!face.is_valid())
			{
				std::cout << "Could not find final face, aborting carve.\n";
				return false;
			}

			Halfedge fromEdge;
			Halfedge toEdge;
			for (Halfedge fe : patch.halfedges(face))
			{
				if (patch.to_vertex(fe) == previous)
				{
					fromEdge = fe;
					break;
				}
			}

			for (Halfedge fe : patch.halfedges(face))
			{
				if (patch.to_vertex(fe) == v2)
				{
					toEdge = fe;
					break;
				}
			}

			myassert(toEdge.is_valid());
			myassert(fromEdge.is_valid());

			Halfedge newCrossEdge = patch.insert_edge(fromEdge, toEdge);
			htex[newCrossEdge] = htex[toEdge];
			htex[patch.opposite_halfedge(newCrossEdge)] = htex[fromEdge];

			newEdges.push_back(newCrossEdge);

			toTriangulate.push_back(patch.face(newCrossEdge));
			toTriangulate.push_back(patch.face(patch.opposite_halfedge(newCrossEdge)));
			//MeshHelpers::triangulate_with_properties(patch, patch.face(newCrossEdge));
			//MeshHelpers::triangulate_with_properties(patch, patch.face(patch.opposite_halfedge(newCrossEdge)));
		}

		for (Face f : toTriangulate)
		{
			if (patch.valence(f) > 3)
			{
				MeshHelpers::triangulate_with_properties(patch, f);
			}
		}

		std::cout << "New edges " << newEdges.size() << "\n";
		return true;
	}
#endif
	static bool CarvePath(SurfaceMesh &patch, Vertex v1, Vertex v2, std::vector<Halfedge> &newEdges)
	{
		newEdges.clear();

		auto tex = patch.get_vertex_property<TexCoord>("v:tex");
		auto htex = patch.get_halfedge_property<TexCoord>("h:tex");
		auto normals = patch.vertex_property<Normal>("v:normal");

		TexCoord uv0 = tex[v1];
		TexCoord uv1 = tex[v2];

		struct CarveEdge
		{
			Halfedge h;
			TexCoord intersect;
			float dist;
			float lerp;
		};

		auto efeature = patch.edge_property<bool>("e:feature");

		//std::vector<CarveEdge> edgesToSplit;

/*
		for (Edge e : patch.edges())
		{
			Halfedge h = patch.halfedge(e, 0);
			TexCoord euv0 = tex[patch.from_vertex(h)];
			TexCoord euv1 = tex[patch.to_vertex(h)];
			TexCoord inter;
			if (GetIntersection(uv0, uv1, euv0, euv1, inter))
			{
				if (patch.from_vertex(h) == v1 ||
					patch.to_vertex(h) == v1 ||
					patch.from_vertex(h) == v2 ||
					patch.to_vertex(h) == v2)
				{
					float lerp = norm(euv0 - inter) / norm(euv0 - euv1);
					// Check that we're interescting at the end ponts which is ok but otherwise something's wrong
					if (lerp > 0.01f && lerp < 0.99f)
					{
						std::cout << "Intersected start/end edge at invalid lerp " << lerp << "\n";
					}
					continue;
				}

				if (patch.is_boundary(e))
				{
					std::cout << "Intersected boundary edge.\n";
				}

				CarveEdge save;
				save.h = h;
				save.intersect = inter;
				save.dist = norm(uv0 - inter);
				save.lerp = norm(euv0 - inter) / norm(euv0 - euv1);
				if (save.lerp > 0.99f) save.lerp = 0.99f;
				edgesToSplit.push_back(save);
				//efeature[e] = true;
			}
		}*/

		/*if (edgesToSplit.size() == 0)
		{
			std::cout << "No edges to split.\n";
		}
		else
		{
			std::cout << "Carving " << edgesToSplit.size() << " edges.\n";
		}*/

		//std::sort(edgesToSplit.begin(), edgesToSplit.end(), [](const CarveEdge &a, const CarveEdge &b) { return a.dist < b.dist; });

		std::vector<Face> toTriangulate;

		Vertex previous = v1;
		Face nextFace;
		Halfedge prevEdge;
		for (Face f : patch.faces(v1))
		{
			for (Halfedge h : patch.halfedges(f))
			{
				TexCoord euv0 = tex[patch.from_vertex(h)];
				TexCoord euv1 = tex[patch.to_vertex(h)];
				TexCoord inter;

				if (patch.from_vertex(h) == v2 ||
					patch.to_vertex(h) == v2)
				{
					// v2 is part of a face that also contains V1
					// return this single edge
					Edge e = patch.find_edge(v1, v2);
					newEdges.push_back(patch.halfedge(e, 0));
					std::cout << "Carve path, source and destination are part of a single triangle, returned the existing edge.\n";
					return true;
				}

				if (GetIntersection(uv0, uv1, euv0, euv1, inter))
				{
					if (patch.from_vertex(h) == v1 ||
						patch.to_vertex(h) == v1)
					{
						// dismiss edges that have one vert equal to the start/end vertices
						continue;
					}
					prevEdge = h;
					nextFace = f;
					break;
				}
			}
			if (nextFace.is_valid()) break;
		}

		if (!nextFace.is_valid())
		{
			std::cout << "Could not find initial face\n";
			return false;
		}
		
		std::vector<Face> v2faces;
		std::vector<std::vector<Vertex>> v2faceverts;
		for (Face f : patch.faces(v2))
		{
			v2faces.push_back(f);
			std::vector<Vertex> faceverts;
			for (Vertex v : patch.vertices(f))
			{
				faceverts.push_back(v);
			}
			for (Halfedge h : patch.halfedges(f))
			{
				Edge e = patch.edge(h);
				assert(!patch.is_boundary(e));
			}
			v2faceverts.push_back(faceverts);
		}

		while (true)
		{
			Halfedge nextToSplit;
			float lerp;

			for (Halfedge h : patch.halfedges(nextFace))
			{
				TexCoord euv0 = tex[patch.from_vertex(h)];
				TexCoord euv1 = tex[patch.to_vertex(h)];
				TexCoord inter;

				if (patch.from_vertex(h) == v2 ||
					patch.to_vertex(h) == v2)
				{
					std::cout << "Reached vt by accident";
				}

				if (GetIntersection(uv0, uv1, euv0, euv1, inter))
				{
					if (patch.from_vertex(h) == v1 ||
						patch.to_vertex(h) == v1 ||
						patch.from_vertex(h) == previous ||
						patch.to_vertex(h) == previous )
					{
						//std::cout << "Edge containing start vertex of previous vertex was intrersected strange...\n";
						continue;
					}
					nextToSplit = h;
					lerp = norm(euv0 - inter) / norm(euv0 - euv1);
					break;
				}
			}
			 
			if (!nextToSplit.is_valid())
			{
				MeshHelpers::ErrorEdge(patch, prevEdge, "Next to split not found");
				return false;
			}

			Halfedge fromEdge;
			for (Halfedge fe : patch.halfedges(patch.face(nextToSplit)))
			{
				if (patch.to_vertex(fe) == previous)
				{
					fromEdge = fe;
					break;
				}
			}

			assert(fromEdge.is_valid());

			float oneMinus = 1.0f - lerp;
			Point p = patch.position(patch.from_vertex(nextToSplit)) * oneMinus + patch.position(patch.to_vertex(nextToSplit)) *lerp;
			TexCoord t = tex[patch.from_vertex(nextToSplit)] * oneMinus + tex[patch.to_vertex(nextToSplit)] * lerp;

			Normal n = normalize(normals[patch.from_vertex(nextToSplit)] + normals[patch.to_vertex(nextToSplit)]);
			TexCoord ht0 = (htex[patch.prev_halfedge(nextToSplit)] + htex[nextToSplit]) * 0.5f;
			TexCoord ht1 = (htex[patch.prev_halfedge(patch.opposite_halfedge(nextToSplit))] + htex[patch.opposite_halfedge(nextToSplit)]) * 0.5f;

			// Add vertex to the edge and make sure all halfedge properties are moved over
			Vertex newVert = patch.add_vertex(p);
			tex[newVert] = t;
			normals[newVert] = n;
			Halfedge newEdge = patch.insert_vertex(nextToSplit, newVert);
			
			TexCoord backup = htex[nextToSplit];
			htex[nextToSplit] = ht0;
			htex[newEdge] = ht1;
			htex[patch.opposite_halfedge(newEdge)] = backup;

			// Split the face again making sure halfedge properties are moved
			Halfedge newCrossEdge = patch.insert_edge(fromEdge, nextToSplit);
			htex[newCrossEdge] = htex[nextToSplit];
			htex[patch.opposite_halfedge(newCrossEdge)] = htex[fromEdge];

			efeature[patch.edge(newCrossEdge)] = true;


			newEdges.push_back(newCrossEdge);

			// Triangulate both new created faces
			toTriangulate.push_back(patch.face(newCrossEdge));
			toTriangulate.push_back(patch.face(patch.opposite_halfedge(newCrossEdge)));

			//assert(patch.to_vertex(newEdge) == patch.to_vertex(newCrossEdge));

			nextFace = patch.face(newEdge);
			previous = patch.to_vertex(newEdge);
			prevEdge = nextToSplit;

			// Check if we found a face that contains the destination vertex
			bool reachedEnd = false;
			for (Halfedge h : patch.halfedges(nextFace))
			{
				if (patch.from_vertex(h) == v2 ||
					patch.to_vertex(h) == v2)
				{
					reachedEnd = true;
				}
			}
			if (reachedEnd) break;
		}

		// Final step from previous to v2
		{
			/*Face face;
			for (Face ring1 : patch.faces(previous))
			{
				for (Face ring2 : patch.faces(v2))
				{
					if (ring1 == ring2)
					{
						face = ring1;
					}
				}
			}*/

			//			assert(face.is_valid());
			if (!nextFace.is_valid())
			{
				std::cout << "Could not find final face, aborting carve.\n";
				return false;
			}

			Halfedge fromEdge;
			Halfedge toEdge;
			for (Halfedge fe : patch.halfedges(nextFace))
			{
				if (patch.to_vertex(fe) == previous)
				{
					fromEdge = fe;
					break;
				}
			}

			for (Halfedge fe : patch.halfedges(nextFace))
			{
				if (patch.to_vertex(fe) == v2)
				{
					toEdge = fe;
					break;
				}
			}

			myassert(toEdge.is_valid());
			myassert(fromEdge.is_valid());

			Halfedge newCrossEdge = patch.insert_edge(fromEdge, toEdge);
			htex[newCrossEdge] = htex[toEdge];
			htex[patch.opposite_halfedge(newCrossEdge)] = htex[fromEdge];

			newEdges.push_back(newCrossEdge);

			toTriangulate.push_back(patch.face(newCrossEdge));
			toTriangulate.push_back(patch.face(patch.opposite_halfedge(newCrossEdge)));
			//MeshHelpers::triangulate_with_properties(patch, patch.face(newCrossEdge));
			//MeshHelpers::triangulate_with_properties(patch, patch.face(patch.opposite_halfedge(newCrossEdge)));
		}

		for (Face f : toTriangulate)
		{
			if (patch.valence(f) > 3)
			{
				MeshHelpers::triangulate_with_properties(patch, f);
			}
		}

		std::cout << "New edges " << newEdges.size() << "\n";
		return true;
	}

	static void TestCarve(SurfaceMeshGL &mesh)
	{
		auto v0 = mesh.add_vertex(Point(0.0f, 0.0f, 0.0f));
		auto v1 = mesh.add_vertex(Point(1.0f, 1.0f, 0.0f));
		auto v2 = mesh.add_vertex(Point(0.0f, 1.0f, 0.0f));
		auto v3 = mesh.add_vertex(Point(1.0f, 0.0f, 0.0f));

		auto tex = mesh.vertex_property<TexCoord>("v:tex");
		tex[v0] = TexCoord(0.0f, 0.0f);
		tex[v1] = TexCoord(1.0f, 1.0f);
		tex[v2] = TexCoord(0.0f, 1.0f);
		tex[v3] = TexCoord(1.0f, 0.0f);

		std::vector<Vertex> f0v = { v0,v1,v2 };
		std::vector<Vertex> f1v = { v0,v3,v1 };
		Face f0 = mesh.add_face(f0v);
		Face f1 = mesh.add_face(f1v);

		auto htex = mesh.halfedge_property<TexCoord>("h:tex");

		for (Halfedge h : mesh.halfedges(f0))
		{
			Point x = mesh.position(mesh.to_vertex(h));
			htex[h] = TexCoord(x[0], x[1]) * 0.25f + TexCoord(0.1f, 0.3f);
		}
		htex[*mesh.halfedges(f0)] = TexCoord(1.0f, 0.0f);

		for (Halfedge h : mesh.halfedges(f1))
		{
			Point x = mesh.position(mesh.to_vertex(h));
			htex[h] = TexCoord(x[0], x[1]) * 0.25f + TexCoord(0.5f, 0.5f);
		}

		std::vector<Halfedge> edges;
		CarvePath(mesh, v2, v3, edges);
	}

	static void QuadrangulatePatch(PolygonPatch &polyPatch, std::vector<QuadrangularPatch*> &outQuadPatches)
	{
		// Step1 : find the center vertex.
		// We hack this up compared to the paper. We simply map the patch to a quad using the uv unwrapper
		// then use the center vertex in 2D space (0.5, 0.5) as patch centre.
		// This seems to work well in practice and is a lot easier to implement and reuses more code :-)

		// Step2 : Find shortest paths from the center of the edges to the center vertex
		// We do this pretty much as in the paper except that we just do them in order not
		// do the "binary opposing sides" order.

		/*auto facePatchIDs = patch.get_face_property<int>("f:patch_id");
		assert(facePatchIDs);
		int originalPatchID = facePatchIDs[*patch.faces_begin()];

		// Extract patch edges
		std::vector<PatchEdge> strips;
		GenerateBoundaryInfo(patch, strips);*/

		std::cout << "Quadrangulating patch " << polyPatch.patchID  << ", detected " << polyPatch.edges.size() << "edges.\n";
		auto &strips = polyPatch.edges;
		SurfaceMesh &patch = *polyPatch.mesh;

		auto facePatchIDs = patch.get_face_property<int>("f:patch_id");
		assert(facePatchIDs);

		// Set up lists for the fearch
		std::unordered_set<Face> includedFaces;
		std::unordered_set<Halfedge> excludedEdges;
		std::unordered_set<Vertex> excludedVerts;

		// Exclude boundary edges from shortest path search
		for (auto strip : strips)
		{
			for (auto edge : strip.edges)
			{
				excludedEdges.insert(edge);
				excludedEdges.insert(patch.opposite_halfedge(edge));
				excludedVerts.insert(patch.from_vertex(edge));
			}
		}

		// Find all edges that are not in the excluded list
		std::vector<Edge> edges;
		for (Edge e : patch.edges())
		{
			Halfedge he0 = patch.halfedge(e, 0);
			Halfedge he1 = patch.halfedge(e, 1);
			if (excludedEdges.find(he0) == excludedEdges.end() &&
				excludedEdges.find(he1) == excludedEdges.end())
			{		
				edges.push_back(e);
			}
		}

		auto faceColor = patch.face_property<Color>("f:color");
		auto normals = patch.vertex_property<Normal>("v:normal");
		auto htex = patch.halfedge_property<TexCoord>("h:tex");

		// Split any edges not  in the excluded list
		// this means edges internally to the patch
		// this means we have more edges to walk on during shortest path search increasing the chance of finding a path
		// this is dumber than in the paper where they (vaguely) suggest they only split certain edges when they fail to find a path
		// we just split a bunch up front. As this is patch local only it doesn't really matter that this increases the polycount
		// or anything.
	/*	for (Edge e : edges)
		{
			Halfedge he = patch.halfedge(e,0);
			Point p = (patch.position(patch.from_vertex(he)) + patch.position(patch.to_vertex(he))) * 0.5f;
			Normal n = normalize(normals[patch.from_vertex(he)] + normals[patch.to_vertex(he)]);
			TexCoord t0 = (htex[he] + htex[patch.prev_halfedge(he)])*0.5f;
			TexCoord t1 = (htex[patch.opposite_halfedge(he)] + htex[patch.prev_halfedge(patch.opposite_halfedge(he))])*0.5f;
			Halfedge sp = patch.split(e, p);
			Face f0 = patch.face(sp);
			Face f1 = patch.face(patch.opposite_halfedge(sp));
			if (!f0.is_valid() || !f1.is_valid()) continue;
			if (f0.is_valid()) faceColor[f0] = faceColor[patch.face(he)];
			if (f1.is_valid()) faceColor[f1] = faceColor[patch.face(he)];
			normals[patch.to_vertex(sp)] = n;

			// Set up edge props for the new edge and edges of vertices around it
			// This doesn't really seem to make too much sense it was reverse engineerd by looking at the edges post split
			// See TestSplit at the bottom of this file for a simple case you can debug in the viewer
			Halfedge newEdge = sp;
			htex[patch.next_halfedge(newEdge)] = htex[patch.prev_halfedge(patch.opposite_halfedge(patch.next_halfedge(newEdge)))];
			htex[patch.opposite_halfedge(newEdge)] = htex[patch.prev_halfedge(newEdge)];

			htex[newEdge] = t0;
			htex[patch.opposite_halfedge(patch.next_halfedge(newEdge))] = t0;

			// Other side of the new edge
			Halfedge opoStart = patch.prev_halfedge(patch.opposite_halfedge(newEdge));

			htex[patch.next_halfedge(opoStart)] = htex[patch.prev_halfedge(patch.opposite_halfedge(opoStart))];
			htex[patch.opposite_halfedge(opoStart)] = htex[patch.prev_halfedge(opoStart)];

			htex[opoStart] = t1;
			htex[patch.prev_halfedge(patch.opposite_halfedge(opoStart))] = t1;
		}*/

		// Unwrap patch to unit rectangle
		PatchParameterization sp(patch);
		sp.harmonic();

		// Find Center in uv
		Vertex center = FindCenterVertex(patch);

		auto efeature = patch.edge_property<bool>("e:feature");

		//A better loop like compared to the previous naive edge split but not working yet :-) should give a cleaner subdivided mesh
#if 0
		for (Edge e : edges)
		{
			Halfedge he = patch.halfedge(e, 0);
			Point p = (patch.position(patch.from_vertex(he)) + patch.position(patch.to_vertex(he))) * 0.5f;
			patch.insert_vertex(e, p);
		}

		// split faces
		Halfedge h;
		for (auto f : patch.faces())
		{
			auto c = prop[f];
			h = patch.halfedge(f);
			if (excludedEdges.find(h) == excludedEdges.end() && h != patch.next_halfedge(patch.next_halfedge(h)))
			{
				Halfedge sp = patch.insert_edge(h, patch.next_halfedge(patch.next_halfedge(h)));
				Face f0 = patch.face(sp);
				Face f1 = patch.face(patch.opposite_halfedge(sp));
				prop[f0] = prop[f1] = c;
			}
			
			h = patch.next_halfedge(h);
			if (excludedEdges.find(h) == excludedEdges.end() && h != patch.next_halfedge(patch.next_halfedge(h)))
			{
				Halfedge sp = patch.insert_edge(h, patch.next_halfedge(patch.next_halfedge(h)));
				Face f0 = patch.face(sp);
				Face f1 = patch.face(patch.opposite_halfedge(sp));
				prop[f0] = prop[f1] = c;
			}

			h = patch.next_halfedge(h);
			if (excludedEdges.find(h) == excludedEdges.end() && h != patch.next_halfedge(patch.next_halfedge(h)))
			{
				Halfedge sp = patch.insert_edge(h, patch.next_halfedge(patch.next_halfedge(h)));
				Face f0 = patch.face(sp);
				Face f1 = patch.face(patch.opposite_halfedge(sp));
				prop[f0] = prop[f1] = c;
			}
		}
#endif

		if (patch.valence(center) < strips.size())
		{
			std::cout << "Center vertex valence it to low have " << patch.valence(center) << " need " << strips.size() <<". Quadrangulation will fail.\n";
		}

		std::vector<Face> seedFaces;
		std::vector<Halfedge> carvedEdges;

		for (auto strip : strips)
		{
#if 0
			Vertex stripCenter = FindCenterVertex(patch, strip);

			// Temporarily allow this vert so we can start from it :-)
			excludedVerts.erase(stripCenter);

			// Dijkstra on mesh
			ShortestPath sp(patch);
			sp.process(stripCenter, center, includedFaces, excludedEdges, excludedVerts);

			if (sp.result.size() == 0)
			{
				std::cout << "Failed to find a patch from the edge to the center.\n";
			}

			// Exclude "used" edges from future shortest path searches
			for (Halfedge onPath : sp.result)
			{
				excludedEdges.insert(onPath);
				excludedEdges.insert(patch.opposite_halfedge(onPath));
				excludedVerts.insert(patch.from_vertex(onPath));
				// Debug mark edge
				efeature[patch.edge(onPath)] = true;
			}

			// First edge on the path will provide a seed face for later floodfilling of the new patch triangle lists
			if (sp.result.size())
			{
				seedFaces.push_back(patch.face(sp.result[0]));
			}

			// Disallow the vert again
			excludedVerts.insert(stripCenter);
#endif
			Vertex stripCenter = FindCenterVertex(patch, strip);
			if (CarvePath(patch, stripCenter, center, carvedEdges))
			{

				if (carvedEdges.size() == 0)
				{
					std::cout << "Carved edge list is empty.\n";
					continue;
				}

				assert(patch.from_vertex(carvedEdges[0]) == stripCenter);
				seedFaces.push_back(patch.face(carvedEdges[0]));

				for (Halfedge onPath : carvedEdges)
				{
					excludedEdges.insert(onPath);
					excludedEdges.insert(patch.opposite_halfedge(onPath));
					efeature[patch.edge(onPath)] = true;
				}
			}
			else
			{
				std::cout << "Carve failed.\n";
			}

		}

		if (seedFaces.size() == 0)
		{
			std::cout << "Patch seed list is empty, patch will disappear in result. Adding arbitraty face to seed to fil. Patch will not be quadrilateral.\n";
			seedFaces.push_back(*patch.faces_begin());
		}

		// Flood fill from seed faces to get the full face list of the new quad-patches
		std::vector<std::vector<Face>> paches;
		for (Face seed : seedFaces)
		{
			std::vector<Face> newPatch;
			FloodFillFaces(patch, seed, excludedEdges, newPatch);

			int id = PatchHelpers::AllocatePatchID();
			vec3 color = vec3((rand() % 255) / 255.0f,
				(rand() % 255) / 255.0f,
				(rand() % 255) / 255.0f);

			for (Face f : newPatch)
			{
				facePatchIDs[f] = id;
				faceColor[f] = color;
			}

			paches.emplace_back(newPatch);
		}

		// Spill face patch ids to edge patch ids
		auto edgePatchIDs = patch.get_halfedge_property<int>("h:patch_id");
		assert(edgePatchIDs);
		for (auto edge : patch.halfedges())
		{
			Face neighbour = patch.face(patch.opposite_halfedge(edge));
			if (neighbour.is_valid())
			{
				edgePatchIDs[edge] = facePatchIDs[patch.face(patch.opposite_halfedge(edge))];
			}
			else
			{
				// This is a border edge this points to the edge patch in the original polygonal non quad patch
				// this is ok for edge extraction but not if we actually need the neighbour for filtering etc
				// We mark it a negative here so the edge-patch up later recognises it can can set it up
				// to the correct neighbour quad patch
				edgePatchIDs[edge] = -edgePatchIDs[edge];
			}
		}

		// Add the new quad patches to the global list
		for (auto faceList : paches)
		{
			QuadrangularPatch *qPatch = new QuadrangularPatch();
			MeshHelpers::ExtractMesh(patch, faceList, qPatch->mesh);
			qPatch->originalPatchID = polyPatch.patchID;
			qPatch->patchID = facePatchIDs[*faceList.begin()];
			outQuadPatches.push_back(qPatch);
		}

	}

	static void PatchUpNeighbours(std::vector<PatchQuadrangulator::QuadrangularPatch*> &quadPatches)
	{
		// Set
		for (auto patch : quadPatches)
		{
			auto edgePatchIDs = patch->mesh.get_halfedge_property<int>("h:patch_id");
			auto edgeIDsOnOriginalMesh = patch->mesh.get_halfedge_property<Halfedge>("h:orig_id");
			assert(edgePatchIDs);
			assert(edgeIDsOnOriginalMesh);
			bool error = false;

			for (auto edge : patch->mesh.halfedges())
			{
				// still pointing to an old non-quad neighbour patch? fix it upper
				if (edgePatchIDs[edge] < 0)
				{
					int originalNeighbourEdgePatchID = -edgePatchIDs[edge];
					assert(edgeIDsOnOriginalMesh[edge].is_valid());

					// Use the edge numbers on the original mesh we started from to match edges in the patches
					auto toFind = edgeIDsOnOriginalMesh[edge];
					bool found = false;
					assert(toFind.is_valid());
				
					for (auto potentialNeighbour : quadPatches)
					{
						// Optimize: only scan patches that came from the original patch we're interested in
						if (potentialNeighbour->originalPatchID != originalNeighbourEdgePatchID) continue;
						if (potentialNeighbour == patch) continue;
						
						auto edgeIDsOnOriginalMeshNeigh = potentialNeighbour->mesh.get_halfedge_property<Halfedge>("h:orig_id");
						assert(edgeIDsOnOriginalMeshNeigh);
						for (auto neighEdge : potentialNeighbour->mesh.halfedges())
						{
							if (edgeIDsOnOriginalMeshNeigh[neighEdge] == toFind)
							{
								edgePatchIDs[edge] = potentialNeighbour->patchID;
								found = true;
								break;
							}
						}

						if (found && potentialNeighbour->originalPatchID != originalNeighbourEdgePatchID)
						{
							std::cout << "mismatch";
						}

						if (found) break;
					}

					//assert(found);
					if (!found)
					{
						error = true;
					}
				}
			}

			if (error)
			{
				std::cout << "Patch" << patch->patchID << ": Some edges could not find their neighbour.\n";
				MeshHelpers::ErrorMesh(patch->mesh, "Neighbours not found");
			}

			// now the neighbours are patched we can finally generate boundary info
			GenerateBoundaryInfo(patch->mesh, patch->edges);
			//assert(patch->edges.size() <= 4);
		}

		// Find the neighbour edge indexes
		for (auto patch : quadPatches)
		{
			for (auto &edge : patch->edges)
			{
				//No neighbour nothing find here
				if (edge.neighbour < 0)
				{
					edge.neighbourEdgeIndex = -1;
					continue;
				}

				PatchQuadrangulator::QuadrangularPatch* otherPatch = nullptr;
				for (auto p : quadPatches)
				{
					if (p->patchID == edge.neighbour)
					{
						otherPatch = p;
						break;
					}
				}

				if (otherPatch == nullptr)
				{
					std::cout << "Could not find neighbour edge. Neighbor data corrupt marking as no-neighbour.\n";
					edge.neighbourEdgeIndex = -1;
					edge.neighbour = -1;
					continue;
				}

				// Find the index of the edge on the other patch that points to this patch.
				int otherEdgeIdx = -1;
				for (int j = 0; j < otherPatch->edges.size(); j++)
				{
					if (otherPatch->edges[j].neighbour == patch->patchID)
					{
						otherEdgeIdx = j;
						break;
					}
				}

				if (otherEdgeIdx < 0)
				{
					std::cout << "Could not find index of neighbour edge. Neighbor data corrupt marking as no-neighbour.\n";
					edge.neighbourEdgeIndex = -1;
					edge.neighbour = -1;
				}
				else
				{
					edge.neighbourEdgeIndex = otherEdgeIdx;
				}
			}
		}
	}

	static void SetupPolyPatches(std::vector<SurfaceMesh*> &patches, std::vector<PolygonPatch> &polyPatches)
	{
		for (SurfaceMesh *mesh : patches)
		{
			// Extract patch edges
			std::vector<PatchEdge> strips;
			GenerateBoundaryInfo(*mesh, strips);

			for (PatchEdge &edge : strips)
			{
				Vertex v = FindCenterVertex(*mesh, edge);
				edge.center = v;
			}

			PolygonPatch p;
			p.edges = strips;
			p.mesh = mesh;

			auto facePatchIDs = mesh->get_face_property<int>("f:patch_id");
			assert(facePatchIDs);
			p.patchID = facePatchIDs[*mesh->faces_begin()];

			polyPatches.push_back(p);
		}

		// Validate edges and centers
		for (PolygonPatch &patch : polyPatches)
		{
			for (int ei=0; ei<patch.edges.size(); ei++)
			{
				int pachID = patch.patchID;
				int neighbourID = patch.edges[ei].neighbour;
				Halfedge neighbourFirstEdge = patch.mesh->opposite_halfedge(patch.edges[ei].edges.back());

				auto neighbour = std::find_if(polyPatches.begin(), polyPatches.end(), [neighbourID](PolygonPatch &candidate) { return candidate.patchID == neighbourID; });
				if (neighbour == polyPatches.end())
				{
					std::cout << "Could not find neighbour patch.\n";
					continue;
				}

				auto neighbourEdge = std::find_if(neighbour->edges.begin(), neighbour->edges.end(), [pachID](PatchEdge &candidate) { return candidate.neighbour == pachID; });
				if (neighbourEdge == neighbour->edges.end())
				{
					std::cout << "Could not find neighbour edge.\n";
					continue;
				}

				if (patch.edges[ei].edges.size() != neighbourEdge->edges.size())
				{
					std::cout << "Patch: " << pachID << "Mismatching edge counts.\n" << patch.edges[ei].edges.size() << " vs " << neighbourEdge->edges.size();

					auto thisColor = patch.mesh->face_property<Color>("f:color");
					for (Face f : patch.mesh->faces())
					{
						thisColor[f] = Color(0.7f, 0.7f, 0.7f);
					}

					auto efeature = patch.mesh->edge_property<bool>("e:feature");
					for (Halfedge h : patch.edges[ei].edges)
					{
						Edge e = patch.mesh->edge(h);
						efeature[e] = true;
					}

					auto otherColor = neighbour->mesh->face_property<Color>("f:color");
					for (Face f : neighbour->mesh->faces())
					{
						otherColor[f] = Color(0.7f, 0.3f, 0.7f);
					}

					auto otherefeature = neighbour->mesh->edge_property<bool>("e:feature");
					for (Halfedge h : neighbourEdge->edges)
					{
						Edge e = neighbour->mesh->edge(h);
						otherefeature[e] = true;
					}


					MeshHelpers::ErrorMesh(*patch.mesh, *neighbour->mesh, "Mismatching edge counts");
				}

				if ( patch.mesh->position(patch.edges[ei].center) != neighbour->mesh->position(neighbourEdge->center))
				{
					std::cout << "Mismatching edge centers";
				}
			}
		}
	}

	static void TestSplit(SurfaceMeshGL &mesh)
	{
		auto v0 = mesh.add_vertex(Point(0.0f, 0.0f, 0.0f));
		auto v1 = mesh.add_vertex(Point(1.0f, 1.0f, 0.0f));
		auto v2 = mesh.add_vertex(Point(0.0f, 1.0f, 0.0f));
		auto v3 = mesh.add_vertex(Point(1.0f, 0.0f, 0.0f));

		std::vector<Vertex> f0v = { v0,v1,v2 };
		std::vector<Vertex> f1v = { v0,v3,v1 };
		Face f0 = mesh.add_face(f0v);
		Face f1 = mesh.add_face(f1v);

		auto htex = mesh.halfedge_property<TexCoord>("h:tex");

		for (Halfedge h : mesh.halfedges(f0))
		{
			Point x = mesh.position(mesh.to_vertex(h));
			htex[h] = TexCoord(x[0], x[1]) * 0.25f + TexCoord(0.2f, 0.7f);
		}

		for (Halfedge h : mesh.halfedges(f1))
		{
			Point x = mesh.position(mesh.to_vertex(h));
			htex[h] = TexCoord(x[0], x[1]) * 0.25f + TexCoord(0.5f, 0.5f);
		}

		Edge toSplit = mesh.find_edge(v0, v1);
		Halfedge he = mesh.halfedge(toSplit, 0);
		Point p = (mesh.position(mesh.from_vertex(he)) + mesh.position(mesh.to_vertex(he))) * 0.5f;

		TexCoord t0 = (htex[he] + htex[mesh.prev_halfedge(he)])*0.5f;
		TexCoord t1 = (htex[mesh.opposite_halfedge(he)] + htex[mesh.prev_halfedge(mesh.opposite_halfedge(he))])*0.5f;

		// Set up edge props for the new edge and edges of vertices around it
		Halfedge newEdge = mesh.split(toSplit, p);
		htex[mesh.next_halfedge(newEdge)] = htex[mesh.prev_halfedge(mesh.opposite_halfedge(mesh.next_halfedge(newEdge)))];
		htex[mesh.opposite_halfedge(newEdge)] = htex[mesh.prev_halfedge(newEdge)];

		htex[newEdge] = t0;
		htex[mesh.opposite_halfedge(mesh.next_halfedge(newEdge))] = t0;

		// Other side of the new edge
		Halfedge opoStart = mesh.prev_halfedge(mesh.opposite_halfedge(newEdge));

		htex[mesh.next_halfedge(opoStart)] = htex[mesh.prev_halfedge(mesh.opposite_halfedge(opoStart))];
		htex[mesh.opposite_halfedge(opoStart)] = htex[mesh.prev_halfedge(opoStart)];

		htex[opoStart] = t1;
		htex[mesh.prev_halfedge(mesh.opposite_halfedge(opoStart))] = t1;


	}
};
