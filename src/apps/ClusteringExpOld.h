#pragma once
#include <pmp/algorithms/DifferentialGeometry.h>
#include <unordered_set>
#include "ShortestPath.h"

class ClusteringExpOld
{
public:

	ClusteringExpOld(SurfaceMesh& setMesh) : mesh(setMesh) {}

	struct Cluster
	{
		// Empty cluster
		Cluster() : isValid(true)
		{
			color = vec3((rand() % 255) / 255.0f,
				(rand() % 255) / 255.0f,
				(rand() % 255) / 255.0f);
		}

		// Single face cluster
		Cluster(SurfaceMesh& mesh, Face f)
		{
			isValid = true;
			faces.insert(f.idx());
			for (Halfedge h : mesh.halfedges(f))
			{
				Halfedge opo = mesh.opposite_halfedge(h);
				Face opoFace = mesh.face(opo);
				if (opoFace.is_valid())
				{
					neighbours.insert(opoFace.idx());
				}
				halfEdges.insert(h.idx());
			}
			color = vec3((rand() % 255) / 255.0f,
				(rand() % 255) / 255.0f,
				(rand() % 255) / 255.0f);
			RecalcEdge(mesh);
			RecalcSurface(mesh);
		}

		typedef std::function<void(SurfaceMesh &, Halfedge &)> EdgeWalker;

		bool IsClusterEdge(const SurfaceMesh &mesh, Halfedge h)
		{
			// Edge is part of face of current cluster
			Face cf = mesh.face(h);
			if (faces.find(cf.idx()) == faces.end())
			{
				return false;
			}

			Halfedge opo = mesh.opposite_halfedge(h);
			// Real boundary return immediately
			if (!mesh.face(opo).is_valid())
			{
				return true;
			}
			else
			{
				// Check if oposing face is not in the current cluster if so -> an edge is found
				Face f = mesh.face(opo);
				if (faces.find(f.idx()) == faces.end())
				{
					return true;
				}
			}
			return false;
		}

		// Enumerate edges in a single connected loop order
		void WalkClusterEdges(SurfaceMesh &mesh, EdgeWalker walker)
		{
			// Find an intial edge on the boundary
			// Either the real mesh boundary or the cluster boundary
			Halfedge initialEdge;
			for (int f : faces)
			{
				for (Halfedge h : mesh.halfedges(Face(f)))
				{
					if (IsClusterEdge(mesh, h))
					{
						initialEdge = h;
						break;
					}
				}
				if (initialEdge.is_valid()) break;
			}

			assert(initialEdge.is_valid());

			walker(mesh, initialEdge);
			Halfedge current = initialEdge;
			while (true)
			{
				// Find the next edge
				Halfedge next = mesh.next_halfedge(current);				
				if (IsClusterEdge(mesh, next))
				{
					assert(mesh.to_vertex(current) == mesh.from_vertex(next));
					current = next;

					// We did a full circle dont walk it twice
					if (current == initialEdge) break;

					walker(mesh, current);
				} 
				else
				{
					// Must be on an other triangle 
					Vertex v = mesh.to_vertex(current);
					for (Halfedge h : mesh.halfedges(v))
					{
						if (h == next) continue;// Already checked it in first part of if don't check again
						if (IsClusterEdge(mesh, h))
						{
							assert(mesh.to_vertex(current) == mesh.from_vertex(h));
							current = h;

							// We did a full circle dont walk it twice
							if (current == initialEdge) break;

							walker(mesh, current);
							break;
						}
					}
				}

				// We did a full circle
				if (current == initialEdge) break;
			}
		}

		// Enumerate edges in a single connected loop order
		void WalkSharedEdges(SurfaceMesh &mesh, Cluster &other, EdgeWalker walker)
		{
			WalkClusterEdges(mesh, [&other, &walker](SurfaceMesh &mesh, Halfedge edge)
			{
				Halfedge opo = mesh.opposite_halfedge(Halfedge(edge));
				auto found = other.halfEdges.find(opo.idx());
				if (found != other.halfEdges.end())
				{
					walker(mesh, edge);
				}
			});
		}

		bool IsSimpleEdge(SurfaceMesh &mesh, Cluster &other)
		{
			bool result = true;
			bool inLine = false;
			bool lineEnded = false;
			WalkClusterEdges(mesh, [&other, &inLine, &result, &lineEnded](SurfaceMesh &mesh, Halfedge edge)
			{
				Halfedge opo = mesh.opposite_halfedge(Halfedge(edge));
				auto found = other.halfEdges.find(opo.idx());
				if (found != other.halfEdges.end())
				{
					// A previous segment was ended and now we start a new > fail the reuslt
					if (lineEnded)
					{
						result = false;
					}
					// Start a segment
					inLine = true;
				}
				else
				{
					// A segment has ended
					if (inLine)
					{
						inLine = false;
						lineEnded = true;
					}
				}
			});
			return result;
		}
		
		// Enumerate edges in no particular order
		void IterateClusterEdges(SurfaceMesh &mesh, EdgeWalker walker)
		{
			for (int edge : halfEdges)
			{
				walker(mesh, Halfedge(edge));
			}
		}

		//  Enumerate shared edges on the 'this' side in no particular order
		void IterateSharedEdges(SurfaceMesh &mesh, Cluster &other, EdgeWalker walker)
		{
			for (int e : other.halfEdges)
			{
				Halfedge opo = mesh.opposite_halfedge(Halfedge(e));
				auto found = halfEdges.find(opo.idx());
				if (found != halfEdges.end())
				{
					walker(mesh, opo);
				}
			}
		}

		void MarkClusterEdges(SurfaceMesh &mesh)
		{
			auto efeature = mesh.add_edge_property<bool>("e:feature");
			IterateClusterEdges(mesh, [&efeature](SurfaceMesh &mesh, Halfedge edge) { efeature[mesh.edge(edge)] = true; });
		}

		void MarkSharedClusterEdges(SurfaceMesh &mesh, Cluster &other)
		{
			auto efeature = mesh.add_edge_property<bool>("e:feature");
			IterateSharedEdges(mesh, other, [&efeature](SurfaceMesh &mesh, Halfedge edge) { efeature[mesh.edge(edge)] = true; });
		}

		void RecalcEdge(SurfaceMesh &mesh)
		{
			float total = 0.0f;
			IterateClusterEdges(mesh,[&total] (SurfaceMesh &mesh, Halfedge edge) {
				total += mesh.edge_length(mesh.edge(edge));
			});
			edge = total;
		}

		void RecalcSurface(SurfaceMesh &mesh)
		{
			float total = 0.0f;
			for (int face : faces)
			{
				total += triangle_area(mesh, Face(face));
			}
			surface = total;
		}

		// Merge face lists but does not initialize neighbours correctly
		Cluster PartialMerge(Cluster &b)
		{
			Cluster res;

			res.faces.reserve(this->faces.size() + b.faces.size());
			for (int f : this->faces)
			{
				res.faces.insert(f);
			}
			for (int f : b.faces)
			{
				res.faces.insert(f);
			}

			return res;
		}

		bool isValid;
		//std::vector<Face> faces;
		std::unordered_set<int> faces;
		std::unordered_set<int> neighbours;
		std::unordered_set<int> halfEdges;
		vec3 color;
		float surface;
		float edge;
	};

	void process()
	{
		clusters.clear();

		// Init cluster per face
		clusters.reserve(mesh.n_faces());
		for(Face f : mesh.faces())
		{
			clusters.push_back(Cluster(mesh, f));
		}

		// Try to merge clusters
		int activeClusters = clusters.size();
		int targetClusters = activeClusters / 100;
		while (activeClusters > targetClusters)
		{
			float min = 1e25f;
			int minIdxA = -1;
			int minIdxB = -1;
			float cost = -0.14644680f;

			for (int i = 0; i < clusters.size(); i++)
			{
				if (!clusters[i].isValid) continue;

				for (int neighbour : clusters[i].neighbours)
				{
					cost = MergedClusterMetric(clusters[i], clusters[neighbour]);
					if (cost < min)
					{
						if (clusters[i].IsSimpleEdge(mesh, clusters[neighbour]))
						{
							minIdxA = i;
							minIdxB = neighbour;
							min = cost;
						}
					}
				}
			}

			if (minIdxA < 0) {
				break;
			}

			MergeClusters(minIdxA, minIdxB);
			activeClusters -= 1;
		}

		// Color cluster
		auto prop = mesh.add_face_property<Color>("f:color", Color(0, 0, 0));
		for (Cluster &clust : clusters)
		{
			if ( !clust.isValid) continue;
			for (int f : clust.faces)
			{
				prop[Face(f)] = clust.color;
			}
		}

	    StraightenEdges();

		//ShortestPath p(mesh);
		//p.process(Vertex(0), Vertex(250));


		// Mark edge
		//clusters[0].MarkClusterEdges(mesh);
		//clusters[0].MarkSharedClusterEdges(mesh, clusters[1]);
	}

	template <class T> static void add_unique(std::vector<T> &list, T &el)
	{
		// Only add non existing neighbours
		if (std::find(list.begin(), list.end(), el) == list.end())
		{
			// someName not in name, add it
			list.push_back(el);
		}
	}

	void MergeClusters(int a, int b)
	{
		assert(clusters[a].isValid);
		assert(clusters[b].isValid);
		assert(a != b);

		// Append face lists
		clusters[a].faces.reserve(clusters[a].faces.size() + clusters[b].faces.size());
		for (int f : clusters[b].faces)
		{
			clusters[a].faces.insert(f);
		}

		// Append neighbour lists except self
		clusters[a].neighbours.reserve(clusters[a].neighbours.size() + clusters[b].neighbours.size());
		for (int n : clusters[b].neighbours)
		{
			if ( n == a ) continue; // don't add yourself to the neighbours :-)
			clusters[a].neighbours.insert(n);
		}

		// Append edge lists except shared edges
		clusters[a].halfEdges.reserve(clusters[a].halfEdges.size() + clusters[b].halfEdges.size());
		for (int e : clusters[b].halfEdges)
		{
			Halfedge opo = mesh.opposite_halfedge(Halfedge(e));
			auto found = clusters[a].halfEdges.find(opo.idx());
			if (found == clusters[a].halfEdges.end())
			{
				clusters[a].halfEdges.insert(e);
			}
			else
			{
				clusters[a].halfEdges.erase(found);
			}
		}

		clusters[a].RecalcEdge(mesh);
		clusters[a].RecalcSurface(mesh);

		// Mark old as invalid
		clusters[b].isValid = false;

		// Patch references to old so it points to the merged
		for (int i = 0; i < clusters.size(); i++)
		{
			if (!clusters[i].isValid) continue;

			auto &neigh = clusters[i].neighbours;
			auto found = neigh.find(b);
			if (found != neigh.end())
			{
				neigh.erase(found);
				if (neigh.size() == 0)
				{
					int a = 5;
				}
				if (i != a) // this removes an refs to b from a
				{
					neigh.insert(a);
				}
				assert(neigh.size() >= 1);
			}
		}

	}

	void StraightenEdges()
	{
		// First create a list of cluster/cluser edges
		struct ClusterEdge
		{
			int clusterA;
			int clusterB;
			std::vector<int> edges; // half edge path on cluster A between the two clusters
			std::vector<int> verts;
		};

		std::vector<ClusterEdge> clusterEdges;
		auto efeature = mesh.edge_property<bool>("e:feature");

		for (int i = 0; i < clusters.size(); i++)
		{
			Cluster &clustA = clusters[i];
			if ( !clustA.isValid ) continue;
			for (auto iter = clustA.neighbours.begin();  iter!= clustA.neighbours.end(); iter++)
			{
				Cluster &clustB = clusters[*iter];
				assert(clustB.isValid);
				if (i > *iter) {
					continue; //will aready have been visited when the loop reached *iter previously as it's lower than i
				}
				ClusterEdge clusterEdge;
				clusterEdge.clusterA = i;
				clusterEdge.clusterB = *iter;

				clustA.WalkSharedEdges(mesh, clustB, [&clusterEdge,&efeature](SurfaceMesh &mesh, Halfedge edge) {
					clusterEdge.verts.push_back(mesh.from_vertex(edge).idx());
					clusterEdge.verts.push_back(mesh.to_vertex(edge).idx());
					/*if (clusterEdge.edges.size() > 0)
					{
						Halfedge h = Halfedge(clusterEdge.edges[clusterEdge.edges.size() - 1]);
						assert(mesh.to_vertex(h) == mesh.from_vertex(edge));
					}*/
					clusterEdge.edges.push_back(edge.idx());
					//efeature[mesh.edge(edge)] = true;
				});

				clustA.IterateSharedEdges(mesh, clustB, [&clusterEdge, &efeature](SurfaceMesh &mesh, Halfedge edge) {
					//efeature[mesh.edge(edge)] = true;
				});

				clusterEdges.emplace_back(clusterEdge);
			}
		}

		//ClusterEdge &currentEdge = clusterEdges[0];
		for (ClusterEdge &currentEdge : clusterEdges)
		{
			std::unordered_set<int> excludedEdges;
			for (int edge : clusters[currentEdge.clusterA].halfEdges)
			{
				excludedEdges.insert(edge);
			}
			for (int edge : clusters[currentEdge.clusterA].halfEdges)
			{
				excludedEdges.insert(edge);
			}

			for (int edge : currentEdge.edges)
			{
				excludedEdges.erase(edge);
				excludedEdges.erase(mesh.opposite_halfedge(Halfedge(edge)).idx());
			}

			std::unordered_set<int> includedFaces;
			for (int edge : clusters[currentEdge.clusterA].faces)
			{
				includedFaces.insert(edge);
			}
			for (int edge : clusters[currentEdge.clusterA].faces)
			{
				includedFaces.insert(edge);
			}

			Vertex from = mesh.from_vertex(Halfedge(currentEdge.edges[0]));
			Vertex to = mesh.to_vertex(Halfedge(currentEdge.edges[currentEdge.edges.size()-1]));

			ShortestPath s(mesh);
			s.process(from, to, includedFaces, excludedEdges);
			assert(mesh.from_vertex(s.result[0]) == from);
			assert(mesh.to_vertex(s.result[s.result.size()-1]) == to);

			for (Halfedge e : s.result)
			{
				efeature[mesh.edge(e)] = true;
			}
		}

		// Debug mark first edge
		//auto efeature = mesh.add_edge_property<bool>("e:feature");
		//for (int edge : clusterEdges[30].edges)
		//{
		//	efeature[mesh.edge(Halfedge(edge))] = true;
		//}
	}

	float MergedClusterMetric(Cluster &a, Cluster &b)
	{
#if 0
		float total = 0.0f;

		for (Face f : a.faces)
		{
			total += triangle_area(mesh, f);
		}

		for (Face f : b.faces)
		{
			total += triangle_area(mesh, f);
		}

		return total;
#else
		float surface = a.surface + b.surface;
		float edge = a.edge + b.edge;
		float shared = 0.0f;
		a.IterateSharedEdges(mesh, b, [&shared](SurfaceMesh &mesh, Halfedge edge) { shared += mesh.edge_length(mesh.edge(edge)); });
		edge -= shared;
		return ( (edge * edge) / ( 4.0f * ((float)M_PI) * surface ));
#endif
	}
	
private:
    // mesh and properties
    SurfaceMesh& mesh;
	std::vector<Cluster> clusters;
};
