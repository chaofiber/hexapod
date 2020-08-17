#include <gui/model.h>

#define STB_IMAGE_IMPLEMENTATION
#include <stb_image.h>

#define TINYOBJLOADER_IMPLEMENTATION // define this in only *one* .cc
#include <tiny_obj_loader.h>

#include <unordered_map>
#include <glm/gtx/intersect.hpp>

#ifdef CRL_USE_ASSIMP
#include <assimp/Importer.hpp> 
#include <assimp/scene.h>
#include <assimp/postprocess.h>
#endif

unsigned int textureFromFile(const char *path, const std::string &directory);

namespace std {
    template<> struct hash<Vertex> {
        size_t operator()(Vertex const& vertex) const {
            return ((hash<glm::vec3>()(vertex.position) ^
                   (hash<glm::vec3>()(vertex.normal) << 1)) >> 1) ^
                   (hash<glm::vec2>()(vertex.texCoords) << 1);
        }
    };
}


Model::Model() {
}

Model::Model(const std::string &path) {
    loadModel(path);
}

void Model::draw(const Shader &shader, const V3D &color, const glm::mat4 &transform) const {
    shader.use();
    shader.setMat4("model", transform);
    shader.setVec3("objectColor", toGLM(color));
    for(unsigned int i = 0; i < meshes.size(); i++)
        meshes[i].draw(shader);
}

void Model::draw(const Shader &shader, const V3D &color) const {
    shader.use();
    shader.setMat4("model", getTransform());
    shader.setVec3("objectColor", toGLM(color));
    for(unsigned int i = 0; i < meshes.size(); i++)
        meshes[i].draw(shader);
}

void Model::draw(const Shader &shader) const {
    draw(shader, color);
}

glm::mat4 Model::getTransform() const{
    return getGLMTransform(scale, orientation, position);
}

void Model::loadModel(const std::string &path_) {
    mName = path_;

# ifdef CRL_USE_ASSIMP
    Assimp::Importer importer;
    const aiScene *scene = importer.ReadFile(path_, 
        aiProcess_Triangulate               |
        aiProcess_JoinIdenticalVertices
    );
    if(!scene) {
        throw std::runtime_error("unable to load mesh: " + std::string(importer.GetErrorString()));
    }

    // Assimp transformation
    // Matrix3x3 assimpR;
    // assimpR << 
    //     scene->mRootNode->mTransformation.a1, scene->mRootNode->mTransformation.a2, scene->mRootNode->mTransformation.a3,
    //     scene->mRootNode->mTransformation.b1, scene->mRootNode->mTransformation.b2, scene->mRootNode->mTransformation.b3,
    //     scene->mRootNode->mTransformation.c1, scene->mRootNode->mTransformation.c2, scene->mRootNode->mTransformation.c3;
    // std::cout << assimpR << std::endl;
    // this->orientationOffset = Quaternion(assimpR);
    // this->positionOffset = V3D(
    //     scene->mRootNode->mTransformation.a4, 
    //     scene->mRootNode->mTransformation.b4, 
    //     scene->mRootNode->mTransformation.c4
    // );

    // TODO 
    // material

    for(int i = 0; i < scene->mNumMeshes; ++i) {
        auto mesh = scene->mMeshes[i];
        
        std::vector<Vertex> vertices;
        std::vector<unsigned int> indices;
        Mesh::TextureMap textures;

        if(mesh->HasPositions()) {

            for(int j = 0; j < mesh->mNumVertices; j++) {
                Vertex vertex;

                vertex.position[0] = mesh->mVertices[j].x;
                vertex.position[1] = mesh->mVertices[j].y;
                vertex.position[2] = mesh->mVertices[j].z;

                vertex.normal[0] = mesh->mNormals[j].x;
                vertex.normal[1] = mesh->mNormals[j].y;
                vertex.normal[2] = mesh->mNormals[j].z;

                // vertex.texCoords[0] = mesh->mTextureCoords[0].x;
                // vertex.texCoords[1] = mesh->mTextureCoords[0].y;

                vertices.push_back(vertex);
            }
        }

        if(mesh->HasFaces()) {
            for(int j = 0; j < mesh->mNumFaces; j++) {
                if(mesh->mFaces[j].mNumIndices == 3) {
                    indices.push_back(mesh->mFaces[j].mIndices[0]);
                    indices.push_back(mesh->mFaces[j].mIndices[1]);
                    indices.push_back(mesh->mFaces[j].mIndices[2]);
                }
            }
        }

        meshes.push_back(Mesh(vertices, indices, textures));
	}
# else
    std::string newPath = path_;
    std::replace(newPath.begin(), newPath.end(), '\\', '/');
    std::string directory = newPath.substr(0, newPath.find_last_of('/'));

    tinyobj::attrib_t attrib;
    std::vector<tinyobj::shape_t> shapes;
    std::vector<tinyobj::material_t> materials;

    std::string warn;
    std::string err;

    //we shall retriangulate the mesh...
    bool ret = tinyobj::LoadObj(&attrib, &shapes, &materials, &warn, &err, newPath.c_str(), directory.c_str(), true);

    if (!warn.empty()) {
        std::cout << warn << std::endl;
    }

    if (!err.empty()) {
        throw std::runtime_error("error loading obj: " + err);
    }

    if (!ret) {
        throw std::runtime_error("error loading obj.");
    }

    // Load materials
    material_textures.resize(materials.size());
    for (uint i = 0; i < materials.size(); ++i) {
        const auto &mat = materials[i];
        if(!mat.diffuse_texname.empty())
            material_textures[i][Mesh::DIFFUSE].push_back({textureFromFile(mat.diffuse_texname.c_str(), directory), mat.diffuse_texname.c_str()});
        //			if(!mat.specular_texname.empty())
        //				material_textures[i].push_back({textureFromFile(mat.specular_texname.c_str(), directory), mat.specular_texname.c_str(), "texture_specular"});
        //			if(!mat.normal_texname.empty())
        //				material_textures[i].push_back({textureFromFile(mat.normal_texname.c_str(), directory), mat.normal_texname.c_str(), "texture_normal"});
        //			if(!mat.ambient_texname.empty())
        //				material_textures[i].push_back({textureFromFile(mat.ambient_texname.c_str(), directory), mat.ambient_texname.c_str(), "texture_height"});
    }

    // Loop over shapes
    for (size_t s = 0; s < shapes.size(); s++) {

        std::vector<Vertex> vertices;
        std::vector<unsigned int> indices;
        Mesh::TextureMap textures;

        // Loop over faces(polygon)
        size_t index_offset = 0;
        for (size_t f = 0; f < shapes[s].mesh.num_face_vertices.size(); f++) {

            // To avoid duplicating vertices (as we know each vertex will appear once for each triangle that contains it) we'll use this hash map that lets us know if a vertex has already been seen...
            std::unordered_map<Vertex, uint32_t> uniqueVertices;

            int fv = shapes[s].mesh.num_face_vertices[f];

            // Loop over vertices in the face.
            for (size_t v = 0; v < (size_t)fv; v++) {
                // access to vertex
                tinyobj::index_t idx = shapes[s].mesh.indices[index_offset + v];

                Vertex vertex;

                for (int i=0;i<3;i++){
                    vertex.position[i] = attrib.vertices[3 * idx.vertex_index+i];
					if (attrib.normals.size() > 3 * idx.normal_index + i)
						vertex.normal[i]   =  attrib.normals[3 * idx.normal_index+i];
                }

                if(attrib.texcoords.size() > 0){
                    tinyobj::real_t tx = attrib.texcoords[2*idx.texcoord_index+0];
                    tinyobj::real_t ty = attrib.texcoords[2*idx.texcoord_index+1];
                    vertex.texCoords = {tx, 1-ty};
                }

                if(uniqueVertices.count(vertex) == 0){
                    uniqueVertices[vertex] = static_cast<uint32_t>(vertices.size());
                    vertices.push_back(vertex);
                }
                indices.push_back(uniqueVertices[vertex]);
            }
            index_offset += fv;
        }

        int matId = shapes[s].mesh.material_ids[0];
        if(matId >= 0) {
            textures = material_textures[matId];
            if(textures.count(Mesh::DIFFUSE) == 0)
                std::cout << "no diffuse texture found for shape `" << shapes[s].name << "`" << std::endl;
        }
        meshes.push_back(Mesh(vertices, indices, textures));
    }
#endif
}

bool Model::hitByRay(const P3D &r_o, const V3D &r_v, P3D &hitPoint, double &t, V3D &n) const {

    using namespace glm;

    glm::vec3 orig = toGLM(r_o);
    glm::vec3 dir = toGLM(r_v);

    auto modelInv = glm::inverse(getTransform());
    vec4 origModelTmp = modelInv * vec4(orig, 1);
    vec3 origModel = vec3(origModelTmp / origModelTmp.w);
    vec3 dirModel  = vec3(modelInv * vec4(dir, 0));

    bool hit = false;
    t = HUGE_VALF;
    vec2 bary;
    for(const auto &m : meshes){
        for (unsigned int i = 0; i < m.indices.size()/3; ++i) {
            vec3 v0 = m.vertices[m.indices[3*i+0]].position;
            vec3 v1 = m.vertices[m.indices[3*i+1]].position;
            vec3 v2 = m.vertices[m.indices[3*i+2]].position;

            float t_ = 0;
            bool tHit = glm::intersectRayTriangle(origModel, dirModel, v0, v1, v2, bary, t_);

            if(tHit && t_ > 1e-8 && t_ < t){
                hit = true;
                t = t_;

                //handle the scaling here, otherwise the normal is a bit messed up...
                for (int idx=0;idx<3;idx++){
                    v0[idx] *= scale[idx];
                    v1[idx] *= scale[idx];
                    v2[idx] *= scale[idx];
                }

                hitPoint = toP3D(v0 * (1 - bary.x - bary.y) + v1 * bary.x + v2 * bary.y);
                n = toV3D(v1-v0).cross(toV3D(v2-v0)).normalized();
            }
        }
    }

    if (hit){
        //the point is now in local coordinates, so switch it over to world coords...
        hitPoint = position + V3D(orientation * V3D(P3D(), hitPoint));
        n = orientation * n;
    }
    return hit;
}

bool Model::hitByRay(const P3D &r_o, const V3D &r_v) const {

    P3D hitPoint(0,0,0);
    V3D n(0,0,0);
    double t = 0;

    return hitByRay(r_o, r_v, hitPoint, t, n);
}

bool Model::hitByRay(const P3D &r_o, const V3D &r_v, double &t) const {

    P3D hitPoint(0,0,0);
    V3D n(0,0,0);

    return hitByRay(r_o, r_v, hitPoint, t, n);
}

bool Model::hitByRay(const P3D &r_o, const V3D &r_v, P3D &hitPoint) const {

    double t = 0;
    V3D n(0,0,0);

    return hitByRay(r_o, r_v, hitPoint, t, n);
}

bool Model::hitByRay(const P3D &r_o, const V3D &r_v, P3D &hitPoint, V3D &hitNormal) const {

    double t = 0;

    return hitByRay(r_o, r_v, hitPoint, t, hitNormal);
}

unsigned int textureFromFile(const char *path, const std::string &directory){
    std::string filename = std::string(path);
    filename = directory + '/' + filename;

    unsigned int textureID;
    glGenTextures(1, &textureID);

    int width, height, nrComponents;
    unsigned char *data = stbi_load(filename.c_str(), &width, &height, &nrComponents, 0);
    if (data) {
        GLenum format = 0;
        if (nrComponents == 1)
            format = GL_RED;
        else if (nrComponents == 3)
            format = GL_RGB;
        else if (nrComponents == 4)
            format = GL_RGBA;

        glBindTexture(GL_TEXTURE_2D, textureID);
        glTexImage2D(GL_TEXTURE_2D, 0, format, width, height, 0, format, GL_UNSIGNED_BYTE, data);
        glGenerateMipmap(GL_TEXTURE_2D);

        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR_MIPMAP_LINEAR);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);

        stbi_image_free(data);
    }
    else {
        std::cout << "Texture failed to load at path: " << path << std::endl;
        stbi_image_free(data);
    }

    return textureID;
}
