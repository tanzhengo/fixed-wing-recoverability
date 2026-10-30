#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/SystemPaths.hh>
#include <iostream>
#include <fstream>
#include <iomanip>
int main(int argc,char**argv){
 if(argc != 3){std::cerr<<"Usage: inspect_gazebo_world WORLD OUTPUT_JSON\n";return 1;}
 gazebo::setupServer();auto w=gazebo::loadWorld(argv[1]);if(!w)return 2;w->SetPaused(true);
 std::ofstream o(argv[2]);o<<std::setprecision(12)<<"{\"models\":[";bool first=true;
 for(auto m:w->Models()){if(!first)o<<",";first=false;auto pose=m->WorldPose();auto sc=m->Scale();o<<"{\"name\":\""<<m->GetName()<<"\",\"pose\":["<<pose.Pos().X()<<","<<pose.Pos().Y()<<","<<pose.Pos().Z()<<","<<pose.Rot().Roll()<<","<<pose.Rot().Pitch()<<","<<pose.Rot().Yaw()<<"],\"scale\":["<<sc.X()<<","<<sc.Y()<<","<<sc.Z()<<"],\"collisions\":[";bool f=true;
 for(auto l:m->GetLinks())for(auto c:l->GetCollisions()){if(!f)o<<",";f=false;auto b=c->BoundingBox();auto cp=c->WorldPose();o<<"{\"name\":\""<<c->GetScopedName()<<"\",\"min\":["<<b.Min().X()<<","<<b.Min().Y()<<","<<b.Min().Z()<<"],\"max\":["<<b.Max().X()<<","<<b.Max().Y()<<","<<b.Max().Z()<<"],\"pose\":["<<cp.Pos().X()<<","<<cp.Pos().Y()<<","<<cp.Pos().Z()<<","<<cp.Rot().Roll()<<","<<cp.Rot().Pitch()<<","<<cp.Rot().Yaw()<<"]}";}o<<"]}";
 }o<<"]}\n";o.close();gazebo::shutdown();return 0;
}
