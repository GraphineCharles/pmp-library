#pragma once
#include <pmp/algorithms/DifferentialGeometry.h>
#include <pmp/algorithms/SurfaceNormals.h>
#include <unordered_set>
#include "ShortestPath.h"
#include "SurfaceSimplificationWithAncestors.h"
#include "MeshHelpers.h"
#include "PatchParameterization.h"
#include "PatchBaker.h"

class ElCheapoClust
{
public:

	ElCheapoClust(SurfaceMesh& setMesh) : inputMesh(setMesh) {}

	Vertex nearest(Point p, VertexProperty<Point> &vp)
	{
		float smallest = 1e23f;
		Vertex found;
		for (Vertex v : inputMesh.vertices())
		{
			float dist = norm(vp[v] - p);
			if (dist < smallest)
			{
				smallest = dist;
				found = v;
			}
		}
		assert(found.is_valid());
		return found;
	}

	// Cluster into numClusters
	// Something I just cooked up
	// Works by finding numclusters verts on the mesh. Greedily by picking the vert furthest away from existing verts
	// (to make the seeds it uniform across the mesh)
	// Then grow sets ov verts connected to those verts
	// Eventually map these sets to faces by selecting the first vert of the face and looking it up in the vertex set
	// for uniform "sphere blobl" meshes this shoudl give us resonably equally spaced patches
	// improvement would be distance on the mesh but that would greatly compicate things
	// this should be fast for exremely large meshes
	void process(int numClusters)
	{
		std::vector<Vertex> clusterSeeds;
		auto points = inputMesh.vertex_property<Point>("v:point");

		// The first seed is vertex 0
/*		clusterSeeds.push_back(*inputMesh.vertices_begin());


		// Second seed = vertex furthest away from existing seed
		Point c0 = points[clusterSeeds[0]];
		float bestDist = 0.0f;
		Vertex c1;
		for (Vertex v : inputMesh.vertices())
		{
			float dist = norm(c0 - points[v]);
			if (dist > bestDist)
			{
				bestDist = dist;
				c1 = v;
			}
		}
		clusterSeeds.push_back(c1);
			   
		// Third sees variance based
		for (int i = 2; i < numClusters; i++)
		{
			float bestCost = 1e23f;
			Vertex found;

			// Find vertex furthest away from existing verts
			for (Vertex v : inputMesh.vertices())
			{
				float averageDist = 0.0f;
				for (Vertex c : clusterSeeds)
				{
					float dist = norm(points[c] - points[v]);
					//totalDist += pow(dist,0.1);// Prefer many small dists over one large e.g.
					averageDist += dist;
					//totalDist *= dist;
				}
				averageDist /= (float)clusterSeeds.size();

				float cost = 0.0f;
				for (Vertex c : clusterSeeds)
				{
					float dist = norm(points[c] - points[v]) - averageDist;
					//totalDist += pow(dist,0.1);// Prefer many small dists over one large e.g.
					cost += dist* dist;
					//totalDist *= dist;
				}

				if (cost < bestCost)
				{
					bestCost = cost;
					found = v;
				}
			}

			assert(found.is_valid());
			clusterSeeds.push_back(found);
		}
*/

		// Genetic seed vertex algorithm:
		// Pick random seeds. Then replace the one which has the smallest distance to one of the other seeds.
		// with a new random seed. If the smallest distance to the others is better keep.
		// Repeat...

		clusterSeeds.reserve(numClusters);

		// Add verts based on bounding box corners
		auto b = inputMesh.bounds();

		for (int i = 0; i < 8; i++)
		{
			Point p = Point(
				(i & 1) ? b.min()[0] : b.max()[0],
				(i & 2) ? b.min()[1] : b.max()[1],
				(i & 4) ? b.min()[2] : b.max()[2]
			);
			Vertex v = nearest(p, points);
			if (std::find(clusterSeeds.begin(), clusterSeeds.end(), v) == clusterSeeds.end())
			{
				clusterSeeds.push_back(v);
			}
		}

		// Add verts based on bounding box face centers
		Point center = b.center();
		for (int i = 0; i < 6; i++)
		{
			int axis = i / 2;
			int offs = i % 2;

			Point p = center;
			p[axis] = (offs) ? b.min()[axis] : b.max()[axis];

			Vertex v = nearest(p, points);
			if (std::find(clusterSeeds.begin(), clusterSeeds.end(), v) == clusterSeeds.end())
			{
				clusterSeeds.push_back(v);
			}
		}

		// Add random unique vertices until enough
		while (clusterSeeds.size() < numClusters)
		{
			Vertex v = Vertex(rand() % inputMesh.n_vertices());
			if (std::find(clusterSeeds.begin(), clusterSeeds.end(), v) == clusterSeeds.end())
			{
				clusterSeeds.push_back(v);
			}
		}


		float smallest = 1e23;
		std::pair<int, int> update{ -1,-1 };
		bool search = true;

		for (int iter = 0; iter < 100000; iter++)
		{
			if (search)
			{
				for (int i = 0; i < numClusters; i++)
				{
					for (int j = i + 1; j < numClusters; j++)
					{
						float dist = norm(points[clusterSeeds[i]] - points[clusterSeeds[j]]);
						if (dist < smallest)
						{
							smallest = dist;
							update = { i,j };
						}
					}
				}
				assert(update.first >= 0);
				search = false;
			}
			Vertex newPoint = Vertex(rand() % inputMesh.n_vertices());
			int toReplace = (rand() & 1) ? update.first : update.second;

			// Calculate the new minimum distance after replacing the point
			float newSmallest = 1e23f;
			for (int i = 0; i < numClusters; i++)
			{
				// Skip the point we are about to replace
				if ( i==toReplace ) continue;

				// Don't allow mapping it to one of the existing points
				if (newPoint == clusterSeeds[i])
				{
					newSmallest =  -1.0f;
					break;
				}
				float dist = norm(points[newPoint] - points[clusterSeeds[i]]);
				if (dist < newSmallest)
				{
					newSmallest = dist;
				}
			}

			// if it became better, accept it and search a new point
			// if not better just try another point but skip the search
			if (newSmallest < smallest)
			{
				clusterSeeds[toReplace] = newPoint;
				search = true;
			}
		}


#if 1
		auto visited = inputMesh.add_face_property<int>("f:visited");
		assert(visited);

		clusters.resize(clusterSeeds.size());
		std::deque<Face> work;
		//fronts.resize(clusterSeeds.size());

		// Initialize initial front around verts
		for (int cluster=0;cluster<clusters.size(); cluster++)
		{
			Face seedFace;
			for (Halfedge h : inputMesh.halfedges(clusterSeeds[cluster]))
			{
				seedFace = inputMesh.face(h);
				if (seedFace.is_valid()) break;
			}

			if (!seedFace.is_valid())
			{
				std::cout << "Seedface has no adjacent faces. Probably isolated, skipping.\n";
				continue;
			}

			work.push_back(seedFace);
			MarkVertex(inputMesh, clusterSeeds[cluster]);
			visited[seedFace] = cluster+1;// zero is unvisited
		}

		while (!work.size() == 0)
		{
			Face toCheck = *work.begin();
			work.pop_front();
			assert(visited[toCheck] != 0);

			// Iterate neighboring faces
			for (Halfedge h : inputMesh.halfedges(toCheck))
			{
				Face f = inputMesh.face(inputMesh.opposite_halfedge(h));
				if ( !f.is_valid()) continue; // mesh border
				if (visited[f] == 0)
				{
					visited[f] = visited[toCheck];
					work.push_back(f);
				}
			}
		}
		
		// Extract face lists just take first vert of the face
		for (Face f : inputMesh.faces())
		{
			int cluster = visited[f]-1;
			if (cluster < 0 ) continue;
			clusters[cluster].push_back(f);
		}

		inputMesh.remove_face_property(visited);
#else
		auto visited = inputMesh.add_vertex_property<int>("v:visited");
		assert(visited);

		clusters.resize(clusterSeeds.size());
		std::deque<Vertex> work;
		//fronts.resize(clusterSeeds.size());

		// Initialize initial front around verts
		for (int cluster = 0; cluster < clusterSeeds.size(); cluster++)
		{
			Vertex seedVert = clusterSeeds[cluster];
			work.push_back(seedVert);
			MarkVertex(inputMesh, clusterSeeds[cluster]);
			visited[seedVert] = cluster + 1;// zero is unvisited
		}

		while (!work.size() == 0)
		{
			Vertex toCheck = *work.begin();
			work.pop_front();
			assert(visited[toCheck] != 0);

			// Iterate neighboring faces
			for (Vertex h : inputMesh.vertices(toCheck))
			{
				if (visited[h] == 0)
				{
					visited[h] = visited[toCheck];
					work.push_back(h);
				}
			}
		}

		// Assign face ids based on the vertex id
		auto fvisited = inputMesh.add_face_property<int>("f:visited");
		for (Face f : inputMesh.faces())
		{
			//int cluster = visited[f] - 1;
			//if (cluster < 0) continue;
			//clusters[cluster].push_back(f);
			int lowestClust = 1000000;
			for (Vertex v : inputMesh.vertices(f))
			{
				int cluster = visited[v] - 1;
				assert(cluster >= 0); //no cluster was assigned to this vert !?
				if (cluster < lowestClust)
				{
					lowestClust = cluster;
				}
			}
			assert(lowestClust < clusters.size());
			fvisited[f] = lowestClust;
		}

		// Clean up isolated triangles by adding them to the cluster of one of the neighbours
		// This shouldn't be needed but it sometimes seems to happen because we flood from vertices instead of flooding from faces
		// We could flood from faces but flooding vertices gives smoother edges.
		for (Face f : inputMesh.faces())
		{
			int identCount = 0;
			for (Halfedge h : inputMesh.halfedges(f))
			{
				Face fo = inputMesh.face(inputMesh.opposite_halfedge(h));
				if (!fo.is_valid() || fvisited[f] != fvisited[fo])
				{
					continue;
				}
				identCount++;
			}

			if (identCount == 0)
			{
				std::cout << "Found isolated triangle assigning to one of neighbours\n";

				for (Halfedge h : inputMesh.halfedges(f))
				{
					Face fo = inputMesh.face(inputMesh.opposite_halfedge(h));
					if (fo.is_valid())
					{
						fvisited[f] = fvisited[fo];
						break;
					}
				}
			}
		}

		// Extract face list
		for (Face f : inputMesh.faces())
		{
			int clust = fvisited[f];
			clusters[clust].push_back(f);
		}

		inputMesh.remove_vertex_property(visited);
		inputMesh.remove_face_property(fvisited);
#endif
	}

	std::vector<std::vector<Face>> clusters;

private:
    SurfaceMesh& inputMesh;
	SurfaceMesh simplified;

	static void MarkVertex(SurfaceMesh &m, Vertex v)
	{
		// Debug mark edges around vert as feature so we can see it in viewer
		auto efeature = m.edge_property<bool>("e:feature");
		for (auto e : m.halfedges(v))
		{
			efeature[m.edge(e)] = true;
		}
	}

};
