// Copyright 2011-2020 the Polygon Mesh Processing Library developers.
// Modified by Cholleme version of standard "SurfaceParameterization" class tailored for mesh patches

#include "PatchParameterization.h"

#include <cmath>

#include <Eigen/Dense>
#include <Eigen/Sparse>

#include "pmp/algorithms/DifferentialGeometry.h"

namespace pmp {

PatchParameterization::PatchParameterization(SurfaceMesh& mesh)
    : mesh_(mesh)
{
}

bool PatchParameterization::setup_boundary_constraints()
{
    // get properties
    auto points = mesh_.vertex_property<Point>("v:point");
    auto tex = mesh_.vertex_property<TexCoord>("v:tex");
	auto efeature = mesh_.edge_property<bool>("e:feature");

    SurfaceMesh::VertexIterator vit, vend = mesh_.vertices_end();
    Vertex vh;
    Halfedge hh;
    std::vector<Vertex> loop;

    // Initialize all texture coordinates to the origin.
    for (auto v : mesh_.vertices())
        tex[v] = TexCoord(0.5, 0.5);

    // find 1st boundary vertex
    for (vit = mesh_.vertices_begin(); vit != vend; ++vit)
        if (mesh_.is_boundary(*vit))
            break;

    // no boundary found ?
    if (vit == vend)
    {
        std::cerr << "Mesh has no boundary." << std::endl;
        return false;
    }

    // collect boundary loop
    vh = *vit;
    hh = mesh_.halfedge(vh);
    do
    {
        loop.push_back(mesh_.to_vertex(hh));
        hh = mesh_.next_halfedge(hh);
		efeature[mesh_.edge(hh)] = true;// Mark the uv boundary
    } while (hh != mesh_.halfedge(vh));

    // map boundary loop to unit circle in texture domain
    unsigned int i, n = loop.size();
    Scalar angle, l, length;
    TexCoord t;

    // compute length of boundary loop
    for (i = 0, length = 0.0; i < n; ++i)
        length += distance(points[loop[i]], points[loop[(i + 1) % n]]);

    // map length intervalls to unit quad intervals
	int prevAngle = -1;
    for (i = 0, l = 0.0; i < n;)
    {
        // Unit quad has boundary length 4
        angle = 4.0f * (l / length);

		// Clamp first vert on a new edge to the corner
		// nearest would be better?
		int anglei = (int)angle;
		if (anglei != prevAngle)
		{
			angle = anglei;
			prevAngle = anglei;
		}


		if (angle <= 1.0f)
		{
			t = TexCoord(angle, 0.0f);
		}
		else if (angle <= 2.0f)
		{
			t = TexCoord(1.0f, angle-1.0f);
		}
		else if (angle <= 3.0f)
		{
			t = TexCoord(1.0f - (angle - 2.0f), 1.0f);
		}
		else
		{
			t = TexCoord(0.0f, 1.0f - (angle - 3.0f));
		}


        tex[loop[i]] = t;

        ++i;
        if (i < n)
        {
            l += distance(points[loop[i]], points[loop[(i + 1) % n]]);
        }
    }

    return true;
}

void PatchParameterization::harmonic(bool use_uniform_weights)
{
    // map boundary to circle
    if (!setup_boundary_constraints())
    {
        std::cerr << "Could not perform setup of boundary constraints.\n";
        return;
    }

    // get properties
    auto tex = mesh_.vertex_property<TexCoord>("v:tex");
    auto eweight = mesh_.add_edge_property<Scalar>("e:param");
    auto idx = mesh_.add_vertex_property<int>("v:idx", -1);

    // compute Laplace weight per edge: cotan or uniform
    for (auto e : mesh_.edges())
    {
        eweight[e] =
            use_uniform_weights ? 1.0 : std::max(0.0, cotan_weight(mesh_, e));
    }

    // collect free (non-boundary) vertices in array free_vertices[]
    // assign indices such that idx[ free_vertices[i] ] == i
    unsigned i = 0;
    std::vector<Vertex> free_vertices;
    free_vertices.reserve(mesh_.n_vertices());
    for (auto v : mesh_.vertices())
    {
        if (!mesh_.is_boundary(v))
        {
            idx[v] = i++;
            free_vertices.push_back(v);
        }
    }

    // setup matrix A and rhs B
    const unsigned int n = free_vertices.size();
    Eigen::SparseMatrix<double> A(n, n);
    Eigen::MatrixXd B(n, 2);
    std::vector<Eigen::Triplet<double>> triplets;
    dvec2 b;
    double w, ww;
    Vertex v, vv;
    Edge e;
    for (i = 0; i < n; ++i)
    {
        v = free_vertices[i];

        // rhs row
        b = dvec2(0.0);

        // lhs row
        ww = 0.0;
        for (auto h : mesh_.halfedges(v))
        {
            vv = mesh_.to_vertex(h);
            e = mesh_.edge(h);
            w = eweight[e];
            ww += w;

            if (mesh_.is_boundary(vv))
            {
                b -= -w * static_cast<dvec2>(tex[vv]);
            }
            else
            {
                triplets.emplace_back(i, idx[vv], -w);
            }
        }
        triplets.emplace_back(i, i, ww);
        B.row(i) = (Eigen::Vector2d)b;
    }

    // build sparse matrix from triplets
    A.setFromTriplets(triplets.begin(), triplets.end());

    // solve A*X = B
    Eigen::SimplicialLDLT<Eigen::SparseMatrix<double>> solver(A);
    Eigen::MatrixXd X = solver.solve(B);
    if (solver.info() != Eigen::Success)
    {
        std::cerr << "PatchParameterization: Could not solve linear system\n";
    }
    else
    {
        // copy solution
        for (i = 0; i < n; ++i)
        {
            tex[free_vertices[i]] = X.row(i);
        }
    }

    // clean-up
    mesh_.remove_vertex_property(idx);
    mesh_.remove_edge_property(eweight);
}

} // namespace pmp
