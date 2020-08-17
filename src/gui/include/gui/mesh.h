#pragma once

#include <gui/shader.h>

//possible loss of data in conversion between double and float
#pragma warning( disable : 4244)
//deprecated/unsafe functions such as fopen
#pragma warning( disable : 4996)

#define GLM_ENABLE_EXPERIMENTAL
#include <glm/gtx/hash.hpp>

#include <vector>
#include <map>

struct Vertex {
	glm::vec3 position;
	glm::vec3 normal;
	glm::vec2 texCoords;

	bool operator==(const Vertex &other) const {
		return position == other.position && normal == other.normal
				&& texCoords == other.texCoords;
	}
};

struct Texture {
    unsigned int id;
	std::string path;
};

class Mesh {
public:
	enum TextureType {
		DIFFUSE, SPECULAR, NORMAL, AMBIENT
	};
	typedef std::map<TextureType, std::vector<Texture>> TextureMap;

	std::vector<Vertex> vertices;
	std::vector<unsigned int> indices;
	TextureMap textures;

	Mesh(std::vector<Vertex> vertices, std::vector<unsigned int> indices, std::map<TextureType, std::vector<Texture>> textures)
		: vertices(vertices), indices(indices), textures(textures){
        setupMesh();
    }

	Mesh(std::vector<Vertex> vertices, std::vector<unsigned int> indices)
		: vertices(vertices), indices(indices) {
		setupMesh();
	}


	// render the mesh using `shader`
	void draw(Shader shader) const {
		if(textures.find(DIFFUSE) != textures.end()){
			shader.setBool("use_textures", true);
			// bind appropriate textures
			for(unsigned int i = 0; i < textures.at(DIFFUSE).size(); i++) {
				glActiveTexture(GL_TEXTURE0 + i); // active proper texture unit before binding
				// retrieve texture number (the N in diffuse_textureN)
				const Texture &texture = textures.at(DIFFUSE)[i];
				// now set the sampler to the correct texture unit
				glUniform1i(glGetUniformLocation(shader.ID, ("texture_diffuse" + std::to_string(i+1)).c_str()), i);
				// and finally bind the texture
				glBindTexture(GL_TEXTURE_2D, texture.id);
			}
		}
		else {
			//there are no textures... indicate as much...
			shader.setBool("use_textures", false);
		}

        // draw mesh
        glBindVertexArray(VAO);
		glDrawElements(GL_TRIANGLES, (GLsizei)indices.size(), GL_UNSIGNED_INT, nullptr);
        glBindVertexArray(0);

        // always good practice to set everything back to defaults once configured.
        glActiveTexture(GL_TEXTURE0);
    }

	void reinitialize(std::vector<Vertex> vertices, std::vector<unsigned int> indices) {
		glDeleteVertexArrays(1, &VAO);
		glDeleteBuffers(1, &VBO);
		glDeleteBuffers(1, &EBO);
		this->vertices = vertices;
		this->indices = indices;
		setupMesh();
	}

private:
	unsigned int VAO, VBO, EBO;

    // initializes all the buffer objects/arrays
	void setupMesh() {
		//TODO: should the arrays generated here be destroyed/deleted at some point? If so, care must be taken with copy constructors/operators and all that...

		// create buffers/arrays
        glGenVertexArrays(1, &VAO);
        glGenBuffers(1, &VBO);
        glGenBuffers(1, &EBO);

        glBindVertexArray(VAO);
        // load data into vertex buffers
        glBindBuffer(GL_ARRAY_BUFFER, VBO);
        // A great thing about structs is that their memory layout is sequential for all its items.
        // The effect is that we can simply pass a pointer to the struct and it translates perfectly to a glm::vec3/2 array which
        // again translates to 3/2 floats which translates to a byte array.
        glBufferData(GL_ARRAY_BUFFER, vertices.size() * sizeof(Vertex), &vertices[0], GL_STATIC_DRAW);  

        glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, EBO);
        glBufferData(GL_ELEMENT_ARRAY_BUFFER, indices.size() * sizeof(unsigned int), &indices[0], GL_STATIC_DRAW);

        // set the vertex attribute pointers
        // vertex Positions
        glEnableVertexAttribArray(0);	
		glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(Vertex), (void*)nullptr);
        // vertex normals
        glEnableVertexAttribArray(1);	
		glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, sizeof(Vertex), (void*)offsetof(Vertex, normal));
        // vertex texture coords
        glEnableVertexAttribArray(2);	
		glVertexAttribPointer(2, 2, GL_FLOAT, GL_FALSE, sizeof(Vertex), (void*)offsetof(Vertex, texCoords));

        glBindVertexArray(0);
    }
};
