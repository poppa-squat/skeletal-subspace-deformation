#ifndef SKELETON_NODE_H_
#define SKELETON_NODE_H_

#include "gloo/SceneNode.hpp"
#include "gloo/VertexObject.hpp"
#include "gloo/components/ShadingComponent.hpp"

#include <fstream>
#include <string>
#include <vector>

namespace GLOO {
class SkeletonNode : public SceneNode {
 public:
  enum class DrawMode { Skeleton, SSD };
  struct EulerAngle {
    float rx, ry, rz;
  };

  SkeletonNode(const std::string& filename);
  void LinkRotationControl(const std::vector<EulerAngle*>& angles);
  void Update(double delta_time) override;
  void OnJointChanged();

 private:
  void LoadAllFiles(const std::string& prefix);
  void LoadSkeletonFile(const std::string& path);
  void LoadMeshFile(const std::string& filename);
  void LoadAttachmentWeights(const std::string& path);

  void ToggleDrawMode();
  //void DecorateTree();
  void ComputeBindPoseMatrices();
  void UpdateTransformMatrices();
  void DeformVertices();

  void ComputeBindNormals();
  void ComputeDeformNormals();

  DrawMode draw_mode_;
  std::vector<EulerAngle*> linked_angles_;

  std::vector<SceneNode*> nodes_;

  std::shared_ptr<VertexObject> mesh_vertex_obj_;
  std::shared_ptr<VertexObject> deform_vertex_obj_;
  SceneNode* mesh_node_ptr_;

  std::vector<std::vector<float>> weight_matrix_;
  std::vector<glm::mat4> bind_mats_;
  std::vector<glm::mat4> inv_bind_mats_;
  std::vector<glm::mat4> T_mats_;

};
}  // namespace GLOO

#endif
