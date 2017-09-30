//=============================================================================
// Copyright (C) 2017 The pmp-library developers
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// * Redistributions of source code must retain the above copyright notice, this
//   list of conditions and the following disclaimer.
// * Redistributions in binary form must reproduce the above copyright notice,
//   this list of conditions and the following disclaimer in the documentation
//   and/or other materials provided with the distribution.
// * Neither the name of the copyright holder nor the names of its
//   contributors may be used to endorse or promote products derived from
//   this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
//=============================================================================

#include <pmp/gl/EdgeSetGL.h>
#include <pmp/gl/phong_shader.h>
#include <float.h>

//=============================================================================

namespace pmp {

//=============================================================================

EdgeSetGL::EdgeSetGL()
{
    // initialize GL buffers to zero
    vertex_array_object_ = 0;
    vertex_buffer_       = 0;
    edge_buffer_         = 0;

    // initialize buffer sizes
    n_vertices_ = 0;
    n_edges_    = 0;
}

//-----------------------------------------------------------------------------

EdgeSetGL::~EdgeSetGL()
{
    // delete OpenGL buffers
    glDeleteBuffers(1, &vertex_buffer_);
    glDeleteBuffers(1, &edge_buffer_);
    glDeleteVertexArrays(1, &vertex_array_object_);
}

//-----------------------------------------------------------------------------

void EdgeSetGL::updateOpenGLBuffers()
{
    // are buffers already initialized?
    if (!vertex_array_object_)
    {
        glGenVertexArrays(1, &vertex_array_object_);
        glBindVertexArray(vertex_array_object_);
        glGenBuffers(1, &vertex_buffer_);
        glGenBuffers(1, &edge_buffer_);
    }

    // activate VAO
    glBindVertexArray(vertex_array_object_);

    auto positions = getVertexProperty<Point>("v:point");
    // vertices
    glBindBuffer(GL_ARRAY_BUFFER, vertex_buffer_);
    glBufferData(GL_ARRAY_BUFFER, nVertices() * 3 * sizeof(float),
                 positions.data(), GL_STATIC_DRAW);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 0, 0);
    glEnableVertexAttribArray(0);
    n_vertices_ = nVertices();

    // edge indices
    std::vector<unsigned int> edgeArray;
    edgeArray.reserve(nEdges());
    for (auto e : edges())
    {
        edgeArray.push_back(vertex(e, 0).idx());
        edgeArray.push_back(vertex(e, 1).idx());
    }
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, edge_buffer_);
    glBufferData(GL_ELEMENT_ARRAY_BUFFER,
                 edgeArray.size() * sizeof(unsigned int), edgeArray.data(),
                 GL_STATIC_DRAW);
    n_edges_ = edgeArray.size();

    // unbind vertex array
    glBindVertexArray(0);
}

//-----------------------------------------------------------------------------

void EdgeSetGL::draw(const mat4& projectionMatrix, const mat4& modelviewMatrix,
                     const std::string drawMode)
{
    // did we generate buffers already?
    if (!vertex_array_object_)
    {
        updateOpenGLBuffers();
    }

    // load shader?
    if (!m_phongShader.isValid())
    {
        m_phongShader.source(phong_vshader, phong_fshader);
        m_phongShader.use();
        m_phongShader.bind_attrib("v_position", 0);
    }

    // empty edge set?
    if (isEmpty())
        return;

    // setup matrices
    mat4 mv_matrix  = modelviewMatrix;
    mat4 mvp_matrix = projectionMatrix * modelviewMatrix;
    mat3 n_matrix   = inverse(transpose(mat3(mv_matrix)));

    // setup shader
    m_phongShader.use();
    m_phongShader.set_uniform("modelview_projection_matrix", mvp_matrix);
    m_phongShader.set_uniform("modelview_matrix", mv_matrix);
    m_phongShader.set_uniform("normal_matrix", n_matrix);
    m_phongShader.set_uniform("light1", vec3(1.0, 1.0, 1.0));
    m_phongShader.set_uniform("light2", vec3(-1.0, 1.0, 1.0));
    m_phongShader.set_uniform("front_color", vec3(0.6, 0.6, 0.6));
    m_phongShader.set_uniform("back_color", vec3(0.3, 0.0, 0.0));
    m_phongShader.set_uniform("use_lighting", true);

    glBindVertexArray(vertex_array_object_);

    if (drawMode == "Wireframe")
    {
        // draw points
#ifndef __EMSCRIPTEN__
        glPointSize(5.0);
#endif
        glDrawArrays(GL_POINTS, 0, n_vertices_);

        // draw edges
        m_phongShader.set_uniform("front_color", vec3(0.1, 0.1, 0.1));
        m_phongShader.set_uniform("back_color", vec3(0.1, 0.1, 0.1));
        m_phongShader.set_uniform("use_lighting", false);
        glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, edge_buffer_);
        glDrawElements(GL_LINES, n_edges_, GL_UNSIGNED_INT, NULL);
    }

    glBindVertexArray(0);
    glCheckError();
}

//=============================================================================
} // namespace pmp
//=============================================================================