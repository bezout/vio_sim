#include "long_motion/long_motion.hpp"
#include <limits>
#include <queue>
#include <numeric>

namespace long_motion {
static bool overlaps(Aabb a,Aabb b) { for(int i=0;i<3;++i)if(a.hi[i]<b.lo[i]||a.lo[i]>b.hi[i])return false;return true; }
static bool contained(Aabb a,Aabb b) { for(int i=0;i<3;++i)if(a.lo[i]<b.lo[i]||a.hi[i]>b.hi[i])return false;return true; }
CollisionWorld::CollisionWorld(const World& w,const Config& c):depth(c.collision_depth) {
    validate(w,c);
    double xy=c.body_radius,down=c.body_radius,up=c.body_radius;
    if(c.mode==Mode::Mower) {
        xy=c.mower_radius;down=c.mower_imu_height;up=c.mower_top_above_imu;
        for(auto p:c.camera_offsets) {
            p=c.initial_rotation*p;
            xy=std::max(xy,std::hypot(p.x,p.y)+c.camera_radius);
            down=std::max(down,-p.z+c.camera_radius);up=std::max(up,p.z+c.camera_radius);
        }
    } else {
        for(auto p:c.camera_offsets)xy=std::max(xy,norm(p)+c.camera_radius);
        up=down=xy;
    }
    double m=c.clearance+c.numerical_guard;
    allowed={w.room.lo+Vec3{xy+m,xy+m,down+m},w.room.hi-Vec3{xy+m,xy+m,up+m}};
    allowed.lo.z=std::max(w.room.lo.z,w.floor_z)+down+m;
    // Ground contact is intended for the chassis, unlike wall/obstacle contact.
    if(c.mode==Mode::Mower) allowed.lo.z=std::max(w.room.lo.z,w.floor_z)+down;
    for(int i=0;i<3;++i)if(allowed.lo[i]>allowed.hi[i])throw std::invalid_argument("Rig/chassis does not fit inside the room");
    for(auto b:w.obstacles) forbidden.push_back({b.lo-Vec3{xy+m,xy+m,up+m},b.hi+Vec3{xy+m,xy+m,down+m}});
    std::sort(forbidden.begin(),forbidden.end(),[](Aabb a,Aabb b){for(int k=0;k<6;++k){double x=k<3?a.lo[k]:a.hi[k-3],y=k<3?b.lo[k]:b.hi[k-3];if(x!=y)return x<y;}return false;});
    if(!point_free(c.start)) throw std::invalid_argument("Initial rig/chassis intersects an obstacle or exceeds the allowed room (no silent relocation)");
    if(c.mode==Mode::Circular&&c.circle_fixed_center) {
        Vec3 offset=c.initial_rotation*c.circle_reference_offset_body;
        Vec3 center=c.circle_center-Vec3{0,0,offset.z};
        Vec3 delta=c.start-center;
        if(!arc_free(center,std::hypot(delta.x,delta.y),std::atan2(delta.y,delta.x),2*pi))
            throw std::invalid_argument("Requested fixed camera circle cannot be certified collision-free for the whole rig");
    }
}
int CollisionWorld::classify(Aabb b) const {
    // Room bounds are convex. If a control-hull box goes outside, subdivision
    // may still resolve it; return unknown unless completely outside a slab.
    bool inside=contained(b,allowed);
    for(int i=0;i<3;++i)if(b.hi[i]<allowed.lo[i]||b.lo[i]>allowed.hi[i])return -1;
    bool uncertain=!inside;
    for(auto o:forbidden) {
        if(contained(b,o))return -1;
        if(overlaps(b,o))uncertain=true;
    }
    return uncertain?0:1;
}
bool CollisionWorld::point_free(Vec3 p) const { return finite(p)&&classify({p,p})==1; }
bool CollisionWorld::line_free(Vec3 a,Vec3 b) const {
    if(!point_free(a)||!point_free(b))return false;
    Vec3 d=b-a;
    for(auto o:forbidden) {
        double lo=0,hi=1;
        for(int k=0;k<3;++k) {
            if(std::abs(d[k])<1e-15) { if(a[k]<o.lo[k]||a[k]>o.hi[k]){lo=2;break;} }
            else { double x=(o.lo[k]-a[k])/d[k],y=(o.hi[k]-a[k])/d[k];if(x>y)std::swap(x,y);lo=std::max(lo,x);hi=std::min(hi,y); }
        }
        if(lo<=hi)return false;
    }
    return true;
}
bool CollisionWorld::bezier_recursive(const std::array<Vec3,4>& p,int remaining) const {
    Aabb box{p[0],p[0]};for(auto v:p){box.lo=vmin(box.lo,v);box.hi=vmax(box.hi,v);}
    int s=classify(box);if(s==1)return true;if(s==-1||remaining==0)return false;
    Vec3 a=(p[0]+p[1])*.5,b=(p[1]+p[2])*.5,c=(p[2]+p[3])*.5,d=(a+b)*.5,e=(b+c)*.5,f=(d+e)*.5;
    return bezier_recursive({p[0],a,d,f},remaining-1)&&bezier_recursive({f,e,c,p[3]},remaining-1);
}
bool CollisionWorld::bezier_free(const std::array<Vec3,4>& p) const {
    for(auto v:p)if(!finite(v))return false;
    if(!point_free(p[0])||!point_free(p[3]))return false;
    return bezier_recursive(p,depth);
}
static std::pair<double,double> trig_interval(double a,double b,bool sine) {
    if(a>b)std::swap(a,b);
    if(b-a>=2*pi)return {-1,1};
    auto fn=[&](double x){return sine?std::sin(x):std::cos(x);};
    double lo=std::min(fn(a),fn(b)),hi=std::max(fn(a),fn(b));
    const double phase=sine?pi/2:0;
    int k0=int(std::ceil((a-phase)/pi)),k1=int(std::floor((b-phase)/pi));
    for(int k=k0;k<=k1;++k){double v=k%2==0?1.:-1.;lo=std::min(lo,v);hi=std::max(hi,v);}
    // Explicit roundoff expansion. The environment also has a configurable guard.
    return {std::max(-1.,lo-2e-14),std::min(1.,hi+2e-14)};
}
bool CollisionWorld::arc_recursive(Vec3 c,double r,double a,double b,int remaining) const {
    auto x=trig_interval(a,b,false),y=trig_interval(a,b,true);
    Aabb box{c+Vec3{r*x.first,r*y.first,0},c+Vec3{r*x.second,r*y.second,0}};
    int s=classify(box);if(s==1)return true;if(s==-1||remaining==0)return false;
    double h=(a+b)*.5;
    return arc_recursive(c,r,a,h,remaining-1)&&arc_recursive(c,r,h,b,remaining-1);
}
bool CollisionWorld::arc_free(Vec3 c,double r,double a,double d) const {
    if(!finite(c)||!std::isfinite(r)||r<=0||!std::isfinite(a)||!std::isfinite(d)||std::abs(d)>4*pi)return false;
    a=std::remainder(a,2*pi);
    return arc_recursive(c,r,a,a+d,depth);
}
bool CollisionWorld::segment_free(const Segment& s) const {
    return s.kind==SegmentKind::Arc?arc_free(s.center,s.radius,s.angle0,s.angle_delta):bezier_free(s.control);
}

std::vector<Vec3> Generator::navigation(Vec3 start,Vec3 goal) const {
    const auto& space=*collision_;
    // Navigation is intentionally a horizontal grid at the CURRENT sensor height.
    // Local vertical gestures are certified separately. No 3-D flying planner.
    if(std::abs(start.z-goal.z)>1e-7||!space.point_free(goal))return {};
    if(space.line_free(start,goal))return {goal};
    auto b=space.allowed;
    // Keep navigational knots away from the numerical certificate boundary.
    b.lo.x+=.001;b.lo.y+=.001;b.hi.x-=.001;b.hi.y-=.001;
    double resolution=config_.nav_resolution;
    double nx_d=std::floor((b.hi.x-b.lo.x)/resolution)+1,ny_d=std::floor((b.hi.y-b.lo.y)/resolution)+1;
    if(nx_d*ny_d>config_.nav_max_nodes||nx_d<2||ny_d<2)throw std::runtime_error("Navigation grid budget exceeded or grid too small; change nav_resolution/nav_max_nodes");
    int nx=int(nx_d),ny=int(ny_d),n=nx*ny;
    auto point=[&](int id){return Vec3{b.lo.x+(id%nx)*resolution,b.lo.y+(id/nx)*resolution,start.z};};
    std::vector<uint8_t> free(n,0),closed(n,0);
    for(int i=0;i<n;++i)free[i]=space.point_free(point(i));
    // Link arbitrary endpoints to nearby cells with exact swept-line checks.
    auto connectors=[&](Vec3 p){
        std::vector<std::pair<double,int>> c;
        int x=int(std::llround((p.x-b.lo.x)/resolution)),y=int(std::llround((p.y-b.lo.y)/resolution));
        for(int dy=-3;dy<=3;++dy)for(int dx=-3;dx<=3;++dx){int ix=x+dx,iy=y+dy;if(ix<0||ix>=nx||iy<0||iy>=ny)continue;int id=iy*nx+ix;if(free[id]&&space.line_free(p,point(id)))c.emplace_back(norm(p-point(id)),id);}
        std::sort(c.begin(),c.end());return c;
    };
    auto starts=connectors(start),ends=connectors(goal);if(starts.empty()||ends.empty())return {};
    std::vector<double> cost(n,std::numeric_limits<double>::infinity());std::vector<int> parent(n,-1);
    std::vector<double> end_cost(n,std::numeric_limits<double>::infinity());for(auto p:ends)end_cost[p.second]=p.first;
    struct Node{double f,g;int id;};
    struct Greater{bool operator()(const Node&a,const Node&b)const{if(a.f!=b.f)return a.f>b.f;if(a.g!=b.g)return a.g>b.g;return a.id>b.id;}};
    std::priority_queue<Node,std::vector<Node>,Greater> queue;
    for(auto p:starts){cost[p.second]=p.first;queue.push({p.first+norm(point(p.second)-goal),p.first,p.second});}
    const int dxs[8]={1,0,-1,0,1,-1,-1,1},dys[8]={0,1,0,-1,1,1,-1,-1};
    int found=-1;double best=std::numeric_limits<double>::infinity();
    while(!queue.empty()){
        auto top=queue.top();queue.pop();if(top.f>best)break;int id=top.id;
        if(closed[id]||top.g!=cost[id]) continue;
        closed[id]=1;
        double total=top.g+end_cost[id];if(total<best){best=total;found=id;}
        int x=id%nx,y=id/nx;
        for(int k=0;k<8;++k){int xx=x+dxs[k],yy=y+dys[k];if(xx<0||xx>=nx||yy<0||yy>=ny)continue;int next=yy*nx+xx;if(!free[next]||closed[next]||!space.line_free(point(id),point(next)))continue;
            double g=top.g+norm(point(next)-point(id));if(g<cost[next]){cost[next]=g;parent[next]=id;queue.push({g+norm(point(next)-goal),g,next});}}
    }
    if(found<0)return {};
    std::vector<Vec3> raw;for(int i=found;i>=0;i=parent[i])raw.push_back(point(i));std::reverse(raw.begin(),raw.end());raw.push_back(goal);
    std::vector<Vec3> smoothed;Vec3 from=start;size_t k=0;
    while(k<raw.size()){
        size_t far=k;for(size_t j=k+1;j<raw.size();++j)if(space.line_free(from,raw[j]))far=j;
        if(!space.line_free(from,raw[far]))return {};
        if(norm(raw[far]-from)>1e-7) smoothed.push_back(raw[far]);
        from=raw[far];k=far+1;
    }
    return smoothed;
}
} // namespace long_motion
