// Copyright 2011-2019 the Polygon Mesh Processing Library developers.
// Distributed under a MIT-style license, see LICENSE.txt for details.

#include "MeshProcessingViewer.h"

#include <pmp/algorithms/SurfaceSubdivision.h>
#include <pmp/algorithms/SurfaceFeatures.h>
#include <pmp/algorithms/SurfaceSimplification.h>
#include <pmp/algorithms/SurfaceFairing.h>
#include <pmp/algorithms/SurfaceRemeshing.h>
#include <pmp/algorithms/SurfaceCurvature.h>
#include <pmp/algorithms/SurfaceGeodesic.h>
#include <pmp/algorithms/SurfaceHoleFilling.h>
#include <pmp/algorithms/DifferentialGeometry.h>

#include <imgui.h>

#include "ClusteringExpOld.h"
#include "ClusteringExp.h"

MeshProcessingViewer::MeshProcessingViewer(const char* title, int width,
                                           int height)
    : MeshViewer(title, width, height)/*, smoother_(meshes_.emplace_back())*/
{
    //crease_angle_ = 90.0;
    //set_draw_mode("Hidden Line");

    // add help items
    add_help_item("O", "Flip mesh orientation", 5);
}

void TestSplit(SurfaceMeshGL &mesh)
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

std::vector<SurfaceMesh> MeshHelpers::errorMeshes;

bool MeshProcessingViewer::load_mesh(const char* filename)
{
	//bool res = pmp::MeshViewer::load_mesh(filename);
	//ClusteringExpOld g(mesh_);
	meshes_.push_back(SurfaceMeshGL());
	ClusteringExp g(meshes_[0]);

	ClusteringExp::JobInfo job;
	job.bake = true;
	job.cluster = true;
	job.method = ClusteringExp::ClusterMethod::ElCheapo;
#if 1
	job.name = "quixel";
	job.fileName = "G:\\Downloads\\Stone_Carved_tjtpfb2fa_8K_3d_ms\\tjtpfb2fa_High.obj";
	job.textureFileName = "G:\\Downloads\\Stone_Carved_tjtpfb2fa_8K_3d_ms\\tjtpfb2fa_8K_Albedo.png";
#endif
#if 0
	job.name = "sphere_low";
	job.fileName = "H:\\Pmp\\pmp-library\\external\\pmp-data\\off\\sphere_low.off";
	job.textureFileName = "";
#endif

	g.process(job);
	//TestSplit(mesh_);
	//PatchQuadrangulator::TestCarve(mesh_);

	for (SurfaceMesh m : MeshHelpers::errorMeshes)
	{
		meshes_.push_back(SurfaceMeshGL());
		SurfaceMesh &asSimple = meshes_[meshes_.size() - 1];
		asSimple = m;
	}

	set_scene((vec3)meshes_[0].bounds().center(), 0.5 * meshes_[0].bounds().size());

	// compute face & vertex normals, update face indices
	update_mesh();

	// set draw mode
	if (meshes_[0].n_faces() == 0)
	{
		set_draw_mode("Points");
	}

	return true;
}

void MeshProcessingViewer::keyboard(int key, int scancode, int action, int mods)
{
    if (action != GLFW_PRESS && action != GLFW_REPEAT)
        return;

    switch (key)
    {
        case GLFW_KEY_O: // change face orientation
        {
            SurfaceMeshGL new_mesh;
            for (auto v : meshes_[0].vertices())
            {
                new_mesh.add_vertex(meshes_[0].position(v));
            }
            for (auto f : meshes_[0].faces())
            {
                std::vector<Vertex> vertices;
                for (auto v : meshes_[0].vertices(f))
                {
                    vertices.push_back(v);
                }
                std::reverse(vertices.begin(), vertices.end());
                new_mesh.add_face(vertices);
            }
			meshes_[0] = new_mesh;
            update_mesh();
            break;
        }
        case GLFW_KEY_M: // merge two faces incident to longest edge
        {
            Scalar l, ll(0);
            Edge ee;
            for (auto e : meshes_[0].edges())
            {
                Vertex v0 = meshes_[0].vertex(e, 0);
                Vertex v1 = meshes_[0].vertex(e, 1);
                Point p0 = meshes_[0].position(v0);
                Point p1 = meshes_[0].position(v1);
                l = distance(p0, p1);
                if (l > ll && meshes_[0].is_removal_ok(e))
                {
                    ll = l;
                    ee = e;
                }
            }

            if (ee.is_valid())
            {
                std::cout << "Merge faces incident to edge " << ee << std::endl;
				meshes_[0].remove_edge(ee);
                update_mesh();
            }
            break;
        }
        default:
        {
            MeshViewer::keyboard(key, scancode, action, mods);
            break;
        }
    }
}

void MeshProcessingViewer::process_imgui()
{
    MeshViewer::process_imgui();

    ImGui::Spacing();
    ImGui::Spacing();

    if (ImGui::CollapsingHeader("Curvature"))
    {
        if (ImGui::Button("Mean Curvature"))
        {
            SurfaceCurvature analyzer(meshes_[0]);
            analyzer.analyze_tensor(1, true);
            analyzer.mean_curvature_to_texture_coordinates();
			meshes_[0].use_cold_warm_texture();
            update_mesh();
            set_draw_mode("Texture");
        }
        if (ImGui::Button("Gauss Curvature"))
        {
            SurfaceCurvature analyzer(meshes_[0]);
            analyzer.analyze_tensor(1, true);
            analyzer.gauss_curvature_to_texture_coordinates();
			meshes_[0].use_cold_warm_texture();
            update_mesh();
            set_draw_mode("Texture");
        }
        if (ImGui::Button("Abs. Max. Curvature"))
        {
            SurfaceCurvature analyzer(meshes_[0]);
            analyzer.analyze_tensor(1, true);
            analyzer.max_curvature_to_texture_coordinates();
			meshes_[0].use_cold_warm_texture();
            update_mesh();
            set_draw_mode("Texture");
        }
    }

    ImGui::Spacing();
    ImGui::Spacing();

    if (ImGui::CollapsingHeader("Smoothing"))
    {
        static int iterations = 10;
        ImGui::PushItemWidth(100);
        ImGui::SliderInt("Iterations", &iterations, 1, 100);
        ImGui::PopItemWidth();

        if (ImGui::Button("Explicit Smoothing"))
        {
            //smoother_.explicit_smoothing(iterations);
            update_mesh();
        }

        ImGui::Spacing();

        static float timestep = 0.001;
        float lb = 0.001;
        float ub = 0.1;
        ImGui::PushItemWidth(100);
        ImGui::SliderFloat("TimeStep", &timestep, lb, ub);
        ImGui::PopItemWidth();

        if (ImGui::Button("Implicit Smoothing"))
        {
            Scalar dt = timestep * radius_ * radius_;
            //smoother_.implicit_smoothing(dt);
            update_mesh();
        }
    }

    ImGui::Spacing();
    ImGui::Spacing();

    if (ImGui::CollapsingHeader("Decimation"))
    {
        static int target_percentage = 10;
        ImGui::PushItemWidth(100);
        ImGui::SliderInt("Percentage", &target_percentage, 1, 99);
        ImGui::PopItemWidth();

        static int normal_deviation = 135;
        ImGui::PushItemWidth(100);
        ImGui::SliderInt("Normal Deviation", &normal_deviation, 1, 135);
        ImGui::PopItemWidth();

        static int aspect_ratio = 10;
        ImGui::PushItemWidth(100);
        ImGui::SliderInt("Aspect Ratio", &aspect_ratio, 1, 10);
        ImGui::PopItemWidth();

        if (ImGui::Button("Decimate it!"))
        {
            SurfaceSimplification ss(meshes_[0]);
            ss.initialize(aspect_ratio, 0.0, 0.0, normal_deviation, 0.0);
            ss.simplify(meshes_[0].n_vertices() * 0.01 * target_percentage);
            update_mesh();
        }
    }

    ImGui::Spacing();
    ImGui::Spacing();

    if (ImGui::CollapsingHeader("Subdivision"))
    {
        if (ImGui::Button("Loop Subdivision"))
        {
            SurfaceSubdivision(meshes_[0]).loop();
            update_mesh();
        }

        if (ImGui::Button("Sqrt(3) Subdivision"))
        {
            SurfaceSubdivision(meshes_[0]).sqrt3();
            update_mesh();
        }
    }

    ImGui::Spacing();
    ImGui::Spacing();

    if (ImGui::CollapsingHeader("Remeshing"))
    {
        if (ImGui::Button("Adaptive Remeshing"))
        {
            auto bb = meshes_[0].bounds().size();
            SurfaceRemeshing(meshes_[0]).adaptive_remeshing(
                0.001 * bb,  // min length
                1.0 * bb,    // max length
                0.001 * bb); // approx. error
            update_mesh();
        }

        if (ImGui::Button("Uniform Remeshing"))
        {
            Scalar l(0);
            for (auto eit : meshes_[0].edges())
                l += distance(meshes_[0].position(meshes_[0].vertex(eit, 0)),
					meshes_[0].position(meshes_[0].vertex(eit, 1)));
            l /= (Scalar)meshes_[0].n_edges();
            SurfaceRemeshing(meshes_[0]).uniform_remeshing(l);
            update_mesh();
        }
    }

    ImGui::Spacing();
    ImGui::Spacing();

    if (ImGui::CollapsingHeader("Hole Filling"))
    {
        if (ImGui::Button("Close smallest hole"))
        {
            // find smallest hole
            Halfedge hmin;
            unsigned int lmin(meshes_[0].n_halfedges());
            for (auto h : meshes_[0].halfedges())
            {
                if (meshes_[0].is_boundary(h))
                {
                    Scalar l(0);
                    Halfedge hh = h;
                    do
                    {
                        ++l;
                        if (!meshes_[0].is_manifold(meshes_[0].to_vertex(hh)))
                        {
                            l += 123456;
                            break;
                        }
                        hh = meshes_[0].next_halfedge(hh);
                    } while (hh != h);

                    if (l < lmin)
                    {
                        lmin = l;
                        hmin = h;
                    }
                }
            }

            // close smallest hole
            if (hmin.is_valid())
            {
                SurfaceHoleFilling hf(meshes_[0]);
                hf.fill_hole(hmin);
                update_mesh();
            }
            else
            {
                std::cerr << "No manifold boundary loop found\n";
            }
        }
    }
}

void MeshProcessingViewer::mouse(int button, int action, int mods)
{
    if (action == GLFW_PRESS && button == GLFW_MOUSE_BUTTON_MIDDLE &&
        shift_pressed())
    {
        double x, y;
        cursor_pos(x, y);
        Vertex v = pick_vertex(x, y);
        if (meshes_[0].is_valid(v))
        {
            // setup seed
            std::vector<Vertex> seed;
            seed.push_back(v);

            // compute geodesic distance
            SurfaceGeodesic geodist(meshes_[0]);
            geodist.compute(seed);

            // setup texture coordinates for visualization
            geodist.distance_to_texture_coordinates();
			meshes_[0].use_checkerboard_texture();
            update_mesh();
            set_draw_mode("Texture");
        }
    }
    else
    {
        MeshViewer::mouse(button, action, mods);
    }
}
