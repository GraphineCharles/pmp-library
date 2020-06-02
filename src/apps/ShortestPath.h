#pragma once
#include <queue>

class ShortestPath
{
public:

	ShortestPath(SurfaceMesh& setMesh) : mesh(setMesh) {
		distance = mesh.vertex_property<Scalar>("shortest:distance");
		processed = mesh.vertex_property<bool>("shortest:processed");
		previous = mesh.vertex_property<int>("shortest:previous");
	}

	void process(Vertex start, Vertex end, const std::unordered_set<int> &includedFaces, const std::unordered_set<int> &excludedEdges)
	{
		result.clear();

		for (Vertex v : mesh.vertices())
		{
			distance[v] = 1e23;
			processed[v] = false;
			previous[v] = PMP_MAX_INDEX;
		}

		distance[start] = 0.0f;
		processed[start] = true;
		previous[start] = PMP_MAX_INDEX;

		std::vector<Vertex> workQueue;
		workQueue.push_back(start);

		bool reachedGoal = false;

		while (workQueue.size() != 0)
		{
			// Find min
			float min = distance[workQueue[0]];
			auto best = workQueue.begin();
			for(auto iter = workQueue.begin(); iter != workQueue.end(); iter++)
			{
				if (distance[*iter] < min)
				{
					min = distance[*iter];
					best = iter;
				}
			}

			Vertex v = *best;
			workQueue.erase(best);

			if (v == end)
			{
				assert(previous[v] != PMP_MAX_INDEX);
				assert(mesh.to_vertex(Halfedge(previous[v])) == end); // The last edge reached this vertex
				reachedGoal = true;
				break;
			}

			for (Halfedge edge : mesh.halfedges(v))
			{
				// Face is not in our allowed list, skip this edge
				if (includedFaces.find(mesh.face(edge).idx()) == includedFaces.end())
				{
					continue;
				}
				// Edge is in our exclude list, skip this edge
				if (excludedEdges.find(edge.idx()) != excludedEdges.end())
				{
					continue;
				}

				assert(mesh.from_vertex(edge) == v);
				float alt = distance[v] + mesh.edge_length(mesh.edge(edge));
				Vertex to = mesh.to_vertex(edge);
				if (alt < distance[to])
				{
					distance[to] = alt;
					previous[to] = edge.idx();
				}
				if (!processed[to])
				{
					processed[to] = true;
					workQueue.push_back(to);
				}
			}

			if (reachedGoal) break;
		}

		auto efeature = mesh.edge_property<bool>("e:feature");
		Halfedge e = Halfedge(previous[end]);
		assert(mesh.to_vertex(e) == end);
		while (true)
		{
			if (!e.is_valid())
			{
				break;
			}
			result.insert(result.begin(), e);
			e = Halfedge(previous[mesh.from_vertex(e)]);
		}
	}

	std::vector<Halfedge> result;

private:
    // mesh and properties
    SurfaceMesh& mesh;
	VertexProperty<Scalar> distance;
	VertexProperty<int> previous;
	VertexProperty<bool> processed;
};
