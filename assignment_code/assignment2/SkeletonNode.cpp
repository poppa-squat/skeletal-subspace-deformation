#include "SkeletonNode.hpp"

#include "gloo/utils.hpp"
#include "gloo/InputManager.hpp"
#include "gloo/MeshLoader.hpp"

#include "gloo/components/RenderingComponent.hpp"
#include "gloo/shaders/PhongShader.hpp"
#include "gloo/debug/PrimitiveFactory.hpp"

namespace GLOO {
SkeletonNode::SkeletonNode(const std::string& filename)
    : SceneNode(), draw_mode_(DrawMode::Skeleton) {
  std::cout << "print ctor" << std::endl;
  LoadAllFiles(filename);

  // Force initial update.
  OnJointChanged();
}

void SkeletonNode::ToggleDrawMode() {
  draw_mode_ =
      draw_mode_ == DrawMode::Skeleton ? DrawMode::SSD : DrawMode::Skeleton;
  
  if (draw_mode_ == DrawMode::SSD) {
    if (mesh_node_ptr_ != nullptr) {
      mesh_node_ptr_->SetActive(true);
    }
  } else {
    if (mesh_node_ptr_ != nullptr) {
      mesh_node_ptr_->SetActive(false);
    }
  }
}

void SkeletonNode::Update(double delta_time) {
  // Prevent multiple toggle.
  static bool prev_released = true;
  if (InputManager::GetInstance().IsKeyPressed('S')) {
    if (prev_released) {
      ToggleDrawMode();
    }
    prev_released = false;
  } else if (InputManager::GetInstance().IsKeyReleased('S')) {
    prev_released = true;
  }
}

void SkeletonNode::OnJointChanged() {
  // TODO: this method is called whenever the values of UI sliders change.
  // The new Euler angles (represented as EulerAngle struct) can be retrieved
  // from linked_angles_ (a std::vector of EulerAngle*).
  // The indices of linked_angles_ align with the order of the joints in .skel
  // files. For instance, *linked_angles_[0] corresponds to the first line of
  // the .skel file.

  const size_t num_controlled = std::min(linked_angles_.size(), nodes_.size());
  for (size_t i = 0; i < num_controlled; ++i) {
    EulerAngle* angle = linked_angles_[i];

    glm::quat quat_angle = glm::quat(glm::vec3(angle->rx, angle->ry, angle->rz));

    nodes_[i]->GetTransform().SetRotation(quat_angle);
  }

  UpdateTransformMatrices();
  DeformVertices();

}

void SkeletonNode::ComputeBindNormals() {

  const auto& positions = mesh_vertex_obj_->GetPositions();
  const auto& indices = mesh_vertex_obj_->GetIndices();
  auto norms = make_unique<NormalArray>();
  norms->resize(positions.size(), glm::vec3(0.f));

  for (size_t i = 0; i < indices.size(); i+=3) {
    auto p_0 = positions[indices[i]];
    auto p_1 = positions[indices[i+1]];
    auto p_2 = positions[indices[i+2]];

    auto e_1 = p_1 - p_0;
    auto e_2 = p_2 - p_0;

    auto face_cross = .5f * glm::cross(e_1, e_2);

    (*norms)[indices[i]] += face_cross;
    (*norms)[indices[i+1]] += face_cross;
    (*norms)[indices[i+2]] += face_cross;

  }

  for (size_t i = 0; i < norms->size(); i++){
    (*norms)[i] = glm::normalize((*norms)[i]);
  }

  mesh_vertex_obj_->UpdateNormals(std::move(norms));

}

void SkeletonNode::ComputeDeformNormals() {

  const auto& positions = deform_vertex_obj_->GetPositions();
  const auto& indices = deform_vertex_obj_->GetIndices();
  auto norms = make_unique<NormalArray>();
  norms->resize(positions.size(), glm::vec3(0.f));

  for (size_t i = 0; i < indices.size(); i+=3) {
    auto p_0 = positions[indices[i]];
    auto p_1 = positions[indices[i+1]];
    auto p_2 = positions[indices[i+2]];

    auto e_1 = p_1 - p_0;
    auto e_2 = p_2 - p_0;

    auto face_cross = .5f * glm::cross(e_1, e_2);

    (*norms)[indices[i]] += face_cross;
    (*norms)[indices[i+1]] += face_cross;
    (*norms)[indices[i+2]] += face_cross;

  }

  for (size_t i = 0; i < norms->size(); i++){
    (*norms)[i] = glm::normalize((*norms)[i]);
  }

  deform_vertex_obj_->UpdateNormals(std::move(norms));
}


void SkeletonNode::LinkRotationControl(const std::vector<EulerAngle*>& angles) {
  linked_angles_ = angles;
}

void SkeletonNode::LoadSkeletonFile(const std::string& path) {
  // Each line is "x y z parent_index", where the position is relative to the
  // parent joint and parent_index refers to an earlier line (-1 for the root).
  std::fstream file(path, std::ios::in);
  if (!file) {
    std::cerr << "Failed to open skeleton file: " << path << std::endl;
    return;
  }

  std::string line;
  while (std::getline(file, line)) {
    std::stringstream iss(line);

    glm::vec3 position;
    int parent_index;
    if (!(iss >> position.x >> position.y >> position.z >> parent_index)) {
      continue;  // Skip blank or malformed lines.
    }

    auto current_node = make_unique<SceneNode>();
    current_node->GetTransform().SetPosition(position);
    SceneNode* node_ptr = current_node.get();

    if (parent_index < 0) {
      this->AddChild(std::move(current_node));
    } else {
      nodes_[parent_index]->AddChild(std::move(current_node));
    }

    nodes_.push_back(node_ptr);
    bind_mats_.push_back(glm::mat4(1.0f));
    T_mats_.push_back(glm::mat4(1.0f));
  }

  ComputeBindPoseMatrices();

  auto shader = std::make_shared<PhongShader>();

  for (auto* node_ : nodes_) {

    if (node_->GetChildrenCount()){
      std::vector<std::unique_ptr<GLOO::SceneNode>> bones;
      for (size_t i = 0; i < node_->GetChildrenCount(); ++i) {
        const glm::vec3 p_to_c = node_->GetChild(i).GetTransform().GetPosition();
        const float length_parent_to_child = glm::length(p_to_c);

        const glm::vec3 dir = p_to_c / length_parent_to_child;

        const glm::vec3 from = glm::vec3(0.f, 1.f, 0.f);
        glm::quat q;
        const float dot_val = glm::clamp(glm::dot(from, dir), -1.0f, 1.0f);

        glm::vec3 axis = glm::cross(from, dir);
        const float axis_len = glm::length(axis);
        if (axis_len > 1e-6f) {
          axis /= axis_len;
        } else {
          axis = glm::vec3(1, 0, 0);
        }
        const float angle = std::acos(dot_val);
        q = glm::angleAxis(angle, axis);

        const float radius = 0.02f;
        const glm::vec3 scale(radius, length_parent_to_child, radius);

        auto bone_node = make_unique<GLOO::SceneNode>();
        bone_node->GetTransform().SetRotation(q);
        bone_node->GetTransform().SetScale(scale); 

        auto cylinder_mesh = PrimitiveFactory::CreateCylinder(0.4f, 1.0f, 25);

        auto& cylinder_shading = bone_node->CreateComponent<ShadingComponent>(shader);
        auto& cylinder_rc = bone_node->CreateComponent<RenderingComponent>(move(cylinder_mesh));

        bones.push_back(std::move(bone_node));
      }

      for (auto& bone : bones) {
        node_->AddChild(std::move(bone));
      }
      
    }

    auto sphere_mesh = PrimitiveFactory::CreateSphere(.018f, 25, 25);
    auto sphere_node = make_unique<SceneNode>();
    auto& shading = sphere_node->CreateComponent<ShadingComponent>(shader);
    auto& rc = sphere_node->CreateComponent<RenderingComponent>(move(sphere_mesh));

    node_->AddChild(std::move(sphere_node));
    
  }

}

void SkeletonNode::LoadMeshFile(const std::string& filename) {

  mesh_vertex_obj_ = MeshLoader::Import(filename).vertex_obj;
  if (mesh_vertex_obj_ == nullptr) {
    std::cerr << "Failed to load mesh: " << filename << std::endl;
    return;
  }

  ComputeBindNormals();

  deform_vertex_obj_ = std::make_shared<VertexObject>();

  deform_vertex_obj_->UpdatePositions(make_unique<PositionArray>(mesh_vertex_obj_->GetPositions()));
  deform_vertex_obj_->UpdateIndices(make_unique<IndexArray>(mesh_vertex_obj_->GetIndices()));
  
  if (mesh_vertex_obj_->HasNormals()) deform_vertex_obj_->UpdateNormals(make_unique<NormalArray>(mesh_vertex_obj_->GetNormals()));
  
  auto mesh_node = make_unique<SceneNode>();
  mesh_node_ptr_ = mesh_node.get();

  mesh_node->CreateComponent<ShadingComponent>(std::make_shared<PhongShader>());
  mesh_node->CreateComponent<RenderingComponent>(deform_vertex_obj_);
  this->AddChild(std::move(mesh_node));

  mesh_node_ptr_->SetActive(false);

}

void SkeletonNode::LoadAttachmentWeights(const std::string& path) {
  std::fstream file(path, std::ios::in);
  std::string line;
  while (std::getline(file, line)) {
    std::stringstream iss(line);
    
    float value;
    std::vector<float> values;

    while (iss >> value) values.push_back(value);

    weight_matrix_.push_back(values);
  }

}

void SkeletonNode::LoadAllFiles(const std::string& prefix) {
  std::string prefix_full = GetAssetDir() + prefix;
  LoadSkeletonFile(prefix_full + ".skel");
  // MeshLoader::Import prepends the asset dir itself, so it takes the
  // relative prefix rather than prefix_full.
  LoadMeshFile(prefix + ".obj");
  LoadAttachmentWeights(prefix_full + ".attach");
}

void SkeletonNode::ComputeBindPoseMatrices() {

  inv_bind_mats_.resize(nodes_.size());
  for (size_t i = 0; i < nodes_.size(); ++i) {
    bind_mats_[i] = nodes_[i]->GetTransform().GetLocalToWorldMatrix();
    // The bind pose never changes, so invert once here rather than per vertex.
    inv_bind_mats_[i] = glm::inverse(bind_mats_[i]);
  }
}

void SkeletonNode::UpdateTransformMatrices() {

  T_mats_.clear();
  for (size_t i = 0; i < nodes_.size(); ++i) {
    T_mats_.push_back(nodes_[i]->GetTransform().GetLocalToWorldMatrix());
  }
}

void SkeletonNode::DeformVertices() {

  auto deformed = make_unique<PositionArray>();
  const auto& positions = mesh_vertex_obj_->GetPositions();

  // Per-joint skinning matrices depend only on the pose, not on the vertex.
  std::vector<glm::mat4> skinning_mats(nodes_.size());
  for (size_t j = 0; j < nodes_.size(); ++j) {
    skinning_mats[j] = T_mats_[j] * inv_bind_mats_[j];
  }

  deformed->reserve(positions.size());
  for (size_t v = 0; v < positions.size(); ++v) {
    auto deformed_pos = glm::vec4(0.f);
    const auto& p = glm::vec4(positions[v], 1.f);

    const size_t num_weights =
        std::min(weight_matrix_[v].size(), nodes_.size() - 1);
    for (size_t j = 1; j <= num_weights; ++j) {
      float weight = weight_matrix_[v][j - 1];
      if (weight > 0.0f) {
        deformed_pos += weight * (skinning_mats[j] * p);
      }
    }
    deformed->push_back(glm::vec3(deformed_pos));
  }
  deform_vertex_obj_->UpdatePositions(std::move(deformed));
  ComputeDeformNormals();

}
}  // namespace GLOO
