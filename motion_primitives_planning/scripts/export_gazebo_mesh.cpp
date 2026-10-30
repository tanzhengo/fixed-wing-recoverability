#include <gazebo/common/MeshManager.hh>
#include <gazebo/common/Mesh.hh>
#include <iostream>
#include <iomanip>
int main(int n,char** a){
 if(n<2)return 1; auto m=gazebo::common::MeshManager::Instance()->Load(a[1]);if(!m)return 2;
 std::cout<<std::setprecision(12)<<"{\"submeshes\":[";
 for(unsigned j=0;j<m->GetSubMeshCount();++j){auto s=m->GetSubMesh(j);if(j)std::cout<<",";std::cout<<"{\"vertices\":[";
 for(unsigned i=0;i<s->GetVertexCount();++i){auto v=s->Vertex(i);if(i)std::cout<<",";std::cout<<"["<<v.X()<<","<<v.Y()<<","<<v.Z()<<"]";}
 std::cout<<"],\"indices\":[";for(unsigned i=0;i<s->GetIndexCount();++i){if(i)std::cout<<",";std::cout<<s->GetIndex(i);}std::cout<<"]}";}
 std::cout<<"]}\n";
}
