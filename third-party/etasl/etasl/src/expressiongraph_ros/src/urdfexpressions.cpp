/* Author: Erwin Aertbelien */
#include <expressiongraph/urdfexpressions.hpp>
#include <kdl/frames_io.hpp>
#include <kdl/tree.hpp>
#include <urdf/model.h>
#include <urdf_parser/urdf_parser.h>
#include <ros/ros.h>
#include <stdlib.h>
#include <expressiongraph/fileutils.hpp>
#include <utility>
#include <boost/tuple/tuple.hpp>

#include <expressiongraph/context_aux.hpp>

using namespace std;


// for backward compatibility with indigo:
#ifndef URDF_TYPEDEF_CLASS_POINTER
    #define URDF_TYPEDEF_CLASS_POINTER(Class) \
    class Class; \
    typedef boost::shared_ptr<Class> Class##SharedPtr; \
    typedef boost::shared_ptr<const Class> Class##ConstSharedPtr; \
    typedef boost::weak_ptr<Class> Class##WeakPtr

    namespace urdf{
        URDF_TYPEDEF_CLASS_POINTER(Joint);
        URDF_TYPEDEF_CLASS_POINTER(Link);
        URDF_TYPEDEF_CLASS_POINTER(ModelInterface);
    }
#endif


namespace KDL {

DefaultUrdfJointsAndConstraintsGenerator::DefaultUrdfJointsAndConstraintsGenerator(double _vel_scale, double _K_limit) {
    vel_scale=_vel_scale;
    K_limit = _K_limit;
}

 
boost::tuple< Expression<double>::Ptr, std::string> 
DefaultUrdfJointsAndConstraintsGenerator::create(
    Context::Ptr& ctx, 
    const std::string& name, 
    double vel_lower, double vel_upper, double pos_lower, double pos_upper,
    double effort_lower, double effort_upper, double dt
) {
    KDL::Expression<double>::Ptr inp = ctx->getScalarExpr(name);
    KDL::Expression<double>::Ptr inpd = ctx->getScalarExpr(name+"_dot");
    if (!inp) {
        inp  = ctx->addScalarVariable(name,"robot",0.0, Constant(1.0));
        if (vel_lower <= vel_upper) {
            int robotvar=ctx->getScalarNdx(name);
            ctx->addBoxConstraint( name + ":velocitylimit", robotvar,
                                   vel_scale*vel_lower,
                                   vel_scale*vel_upper);
        }
        if (pos_lower <= pos_upper) {
            addInequalityConstraint(ctx, name+":positionlimit",
                                    inp,
                                    pos_lower, Constant<double>(K_limit),
                                    pos_upper, Constant<double>(K_limit), Constant<double>(1.0), 1);
        }
        inpd  = ctx->addDerivedVariable(name+"_dot","robot",0.0, ctx->next_varnr-1, dt, Constant(1.0));
        if (effort_lower <= effort_upper) {
            int robotvar_dot=ctx->getScalarNdx(name+"_dot");
            ctx->addBoxConstraint( name+":accelerationlimit", robotvar_dot,
                                   vel_scale*effort_lower,
                                   vel_scale*effort_upper);
            addInequalityConstraint(ctx, name+":accvelpair",
                                    inpd,
                                    0, Constant<double>(K_limit),
                                    0, Constant<double>(K_limit), Constant<double>(1.0), 0);
        }
    }
    return boost::make_tuple(inp,name);
}; 



// Expressions::LinkPtr p
//    p->name
//    p->inertial
//    p->inertial->origin.position.x/y/z
//    p->inertial->origin.rotation.x/.y/.z/.w
//    p->inertial->mass/Ixx/etc.... 
class UrdfExpressions3 {
public:
    typedef urdf::LinkConstSharedPtr                                                LinkPtr;
    typedef urdf::JointConstSharedPtr                                               JointPtr;
    typedef std::map<std::string, LinkPtr>                                    LinkMap;
    typedef std::map<std::string, typename KDL::Expression<KDL::Frame>::Ptr > ExprMap;
    typedef std::pair<LinkPtr,LinkPtr>                                        Transform;
    typedef std::vector<Transform >                                           TransformList;
    typedef std::vector<LinkPtr>                                              LinkList; 
    typedef std::set< LinkPtr >                                               LinkSet;
    typedef std::map< std::string,  std::pair<Expression<double>::Ptr, std::string> > VarList;

    LinkMap  linkmap;
    urdf::LinkSharedPtr                    root_link;
    TransformList                    transformlist;
    LinkList                         rootlist;
    double                           velocity_scale;
    bool                             poslimits;
    bool                             vellimits;
    bool                             effortlimits;
    double                           dt;
    UrdfJointsAndConstraintsGenerator::Ptr jc_gen; 
    ExprMap                                jnt_cache;
    VarList                                varlist;
    
    
    void addUrdfJointName(const urdf::LinkSharedPtr& link, std::vector<std::string>& names);

    void hash_names( const urdf::LinkSharedPtr&  link, int level=0);


    KDL::Vector toKdl(urdf::Vector3 v) {
        return KDL::Vector(v.x, v.y, v.z);
    }

    KDL::Rotation toKdl(urdf::Rotation r) {
        return KDL::Rotation::Quaternion(r.x, r.y, r.z, r.w);
    }

    KDL::Frame toKdl(urdf::Pose p) {
        return KDL::Frame(toKdl(p.rotation), toKdl(p.position));
    }

    KDL::Expression<double>::Ptr 
    create_if_not_exists( 
        Context::Ptr& ctx, const std::string& name, double vel_lower, double vel_upper, double pos_lower, double pos_upper,
        double effort_lower, double effort_upper, double dt
    );

    KDL::Expression<KDL::Frame>::Ptr  toKdl(Context::Ptr& ctx, const JointPtr& jnt);
 
    KDL::Expression<KDL::Frame>::Ptr compose_tree(
        Context::Ptr& ctx, 
        const LinkPtr& p, 
        const LinkPtr& p_root
    );

    UrdfExpressions3(UrdfJointsAndConstraintsGenerator::Ptr jc_gen, bool poslimits=true, bool vellimits=true, bool effortlimits=true, double _dt=0.001);

    bool readFromFile( const std::string& filename );
    bool readFromParam(const std::string& parameter);

    //bool read_fromFile(const std::string& filename);
    bool read_fromString(const std::string& xml_string);
 

    bool addTransform(const std::string& frame, const std::string& base);

    void getAllJointNames(std::vector<std::string>& names);
    void getAllUrdfJointNames(std::vector<std::string>& names);

    void getAllJointExpressions(std::vector<Expression<double>::Ptr>& exprs);

    KDL::Expression<KDL::Frame>::Ptr getExpression(Context::Ptr& ctx, int i);

    void getAllLinkProperties( std::vector<link_property>& props, UrdfExpressions3::LinkPtr p_root=UrdfExpressions3::LinkPtr());
};


UrdfExpressions3::UrdfExpressions3(UrdfJointsAndConstraintsGenerator::Ptr _jc_gen,bool _poslimits, bool _vellimits, bool _effortlimits, double _dt) {
    poslimits = _poslimits;
    vellimits = _vellimits;
    effortlimits = _effortlimits;
    dt = _dt;
    jc_gen    = _jc_gen;
    jnt_cache.clear();
    varlist.clear();
}

bool UrdfExpressions3::addTransform(const std::string& ee, const std::string& base) {
    //std::cout << "addTransform(" << ee << "," << base << ")"<<endl;
    LinkMap::iterator it_ee, it_base;
    it_ee = linkmap.find(ee);
    if (it_ee==linkmap.end()) {
        return false;
    }
    it_base = linkmap.find(base);
    if (it_base==linkmap.end()) {
        return false;
    }
    transformlist.push_back(Transform( it_ee->second, it_base->second) );
    LinkSet linkset;
    LinkPtr p = it_ee->second; 
    while (p) {
        linkset.insert(p);
        p = p->getParent();
    }
    p = it_base->second;
    while (p) {
        LinkSet::iterator it = linkset.find(p);
        if (it!=linkset.end()) {
            rootlist.push_back(*it);
            //std::cout << "\troot: " << (*it)->name << std::endl; 
            break;
        }
        p = p->getParent();
    }
    if (!p) {
        transformlist.pop_back();
        return false; 
    }
    return true; 
}

void UrdfExpressions3::getAllLinkProperties( std::vector<link_property>& props, UrdfExpressions3::LinkPtr p_root){
    if (p_root) {
        link_property L;
        L.name = p_root->name;
        if (p_root->inertial) {
            L.mass = p_root->inertial->mass;
            L.Ixx = p_root->inertial->ixx;
            L.Ixy = p_root->inertial->ixy;
            L.Ixz = p_root->inertial->ixz;
            L.Iyy = p_root->inertial->iyy;
            L.Iyz = p_root->inertial->iyz;
            L.Izz = p_root->inertial->izz;
            L.origin = toKdl(p_root->inertial->origin);
            props.push_back(L);
            //std::cout << L.name << "\t" << L.mass << std::endl;
        }
        for (size_t i=0; i< p_root->child_links.size();++i) {
            getAllLinkProperties(props,p_root->child_links[i]);
        }
    }
}


Expression<Frame>::Ptr UrdfExpressions3::compose_tree(
        Context::Ptr& ctx, 
        const UrdfExpressions3::LinkPtr& p, 
        const UrdfExpressions3::LinkPtr& p_root
) {
   if (p==p_root) {
            return Constant(Frame::Identity());
    } else {
            std::string name;
            Expression<Frame>::Ptr e;
            if (p->parent_joint) {
                name = p->parent_joint->name;
                e    = toKdl(ctx,p->parent_joint);
            } else {
                e    = Constant(Frame::Identity());
            }
            return cached<Frame>(
                        name,
                        compose_tree(ctx,p->getParent(),p_root)*e
                    );
    }
}


Expression<Frame>::Ptr UrdfExpressions3::getExpression(Context::Ptr& ctx,int i) {
    //std::cout << "getExpression(ctx,"<< i << ")"<< endl;
    assert( (0<=i) && ( i < (int)transformlist.size() ) );
    LinkPtr p_ee   = transformlist[i].first;
    LinkPtr p_base = transformlist[i].second;
    LinkPtr p_root = rootlist[i];
    Expression<Frame>::Ptr e_root_ee;
    Expression<Frame>::Ptr e_root_base;
    e_root_ee   = compose_tree(ctx,p_ee,p_root);
    e_root_base = compose_tree(ctx,p_base,p_root);

    return cached<Frame>(inv(e_root_base)*e_root_ee);
}

bool UrdfExpressions3::readFromParam(const std::string& parameter) {
    std::string xml_string;
    if (ros::param::get(parameter,xml_string)) {
        return read_fromString(xml_string);
    } else {
        ROS_WARN("could not find robot model");
        return false;
    }
}

bool UrdfExpressions3::readFromFile(const std::string& filename) {
  std::string xml_string;
  std::fstream xml_file(filename.c_str(), std::fstream::in);
  if (xml_file.good()) {
      while ( xml_file.good() ) {
        std::string line;
        std::getline( xml_file, line);
        xml_string += (line + "\n");
      }
      xml_file.close();
      if (!read_fromString(xml_string)) {
            return false;
      };
      return true;
  } else {
      return false;
  }
}

bool UrdfExpressions3::read_fromString(const std::string& xml_string) {
  using namespace urdf;
  urdf::ModelInterfaceSharedPtr robot = parseURDF(xml_string);
  if (!robot){
    ROS_ERROR("ERROR: Model Parsing the xml failed");
    return false;
  }
  //std::cout << "robot name is: " << robot->getName() << std::endl;
  // get info from parser
  //std::cout << "---------- Successfully Parsed XML ---------------" << std::endl;
  // get root link
  root_link=robot->root_link_;
  if (!root_link) return false;
  //std::cout << "root Link: " << root_link->name << " has " << root_link->child_links.size() << " child(ren)" << std::endl;
  hash_names(root_link);
  return true;
}


void UrdfExpressions3::hash_names( const urdf::LinkSharedPtr& link, int level) {
    using namespace urdf;
    linkmap[ link->name ] = link;  
    for (std::vector<urdf::LinkSharedPtr >::const_iterator child = link->child_links.begin(); child != link->child_links.end(); child++) {
        if (*child) {
            // indent:
            // for(int j=0;j<level;j++) std::cout << "\t"; 
            // debug output:
            // std::cout << "child(" << (*child)->name <<  ")" << std::endl;
            hash_names(*child,level+1);
        } else {
            std::cerr << "root link: " << link->name << " has a null child!" << *child << std::endl;
        }
    }
}

Expression<double>::Ptr 
UrdfExpressions3::create_if_not_exists( Context::Ptr& ctx, const std::string& name, double vel_lower, double vel_upper, double pos_lower, double pos_upper,
                                        double effort_lower, double effort_upper, double dt) {
    Expression<double>::Ptr inp;
    std::string used_name;
    if (varlist.find(name)==varlist.end()) { 
        tie(inp,used_name) = jc_gen->create(ctx, name, vel_lower,vel_upper,pos_lower,pos_upper, effort_lower, effort_upper, dt);
        varlist[name] = std::make_pair(inp,used_name);
    } else {
        inp = varlist[name].first;
    }
    return inp;
}

Expression<Frame>::Ptr  UrdfExpressions3::toKdl(Context::Ptr& ctx, const UrdfExpressions3::JointPtr& jnt) {
    Expression<Frame>::Ptr expr;
    Frame F_parent_jnt = toKdl(jnt->parent_to_joint_origin_transform);
    double pos_lower=1E10;
    double pos_upper=-1E10;
    double vel_lower=1E10;
    double vel_upper=-1E10;
    double effort_lower=1E10;
    double effort_upper=-1E10;
    switch (jnt->type){
        case urdf::Joint::FIXED : {
            return  Constant(F_parent_jnt);
        }
        case urdf::Joint::REVOLUTE : {
            if (jnt->limits) {
                if (vellimits) {
                     vel_lower = -jnt->limits->velocity;
                     vel_upper =  jnt->limits->velocity;
                }
                if (poslimits) {
                    pos_lower = jnt->limits->lower;
                    pos_upper = jnt->limits->upper;
                }
                if (effortlimits) {
                    effort_lower = -jnt->limits->effort;
                    effort_upper = jnt->limits->effort;
                }
            }
            Expression<double>::Ptr inp = create_if_not_exists(ctx, jnt->name, vel_lower,vel_upper,pos_lower,pos_upper,effort_lower, effort_upper, dt);
            Vector axis = toKdl(jnt->axis);
            return Constant(F_parent_jnt)*frame(rot(axis,inp));
        }
        case urdf::Joint::CONTINUOUS : {
            if (jnt->limits) {
                if (vellimits) {
                     vel_lower = -jnt->limits->velocity;
                     vel_upper =  jnt->limits->velocity;
                }
                if (effortlimits) {
                    effort_lower = -jnt->limits->effort;
                    effort_upper = jnt->limits->effort;
                }
            }
            Expression<double>::Ptr inp = create_if_not_exists(ctx, jnt->name, vel_lower,vel_upper,pos_lower,pos_upper,effort_lower, effort_upper, dt);
            Vector axis = toKdl(jnt->axis);
            return Constant(F_parent_jnt)*frame(rot(axis,inp));
        }
        case urdf::Joint::PRISMATIC : {
            if (jnt->limits) {
                if (vellimits) {
                     vel_lower = -jnt->limits->velocity;
                     vel_upper =  jnt->limits->velocity;
                }
                if (poslimits) {
                    pos_lower = jnt->limits->lower; 
                    pos_upper = jnt->limits->upper; 
                }
                if (effortlimits) {
                    effort_lower = -jnt->limits->effort;
                    effort_upper = jnt->limits->effort;
                }
            }
            Expression<double>::Ptr inp = create_if_not_exists(ctx, jnt->name, vel_lower,vel_upper,pos_lower,pos_upper,effort_lower, effort_upper, dt);
            Vector axis = toKdl(jnt->axis);
            return Constant(F_parent_jnt)*frame(Constant(axis)*inp);
        }
        default : {
            ROS_WARN("Converting unknown joint type of joint '%s' into a fixed joint", jnt->name.c_str());
            return  Constant(F_parent_jnt);
        }
    }
    return Constant(F_parent_jnt);
}

void UrdfExpressions3::addUrdfJointName(const urdf::LinkSharedPtr& link, std::vector<std::string>& names){
    using namespace std;
    // add joint:
    if ((link->parent_joint)  &&
        ( (link->parent_joint->type == urdf::Joint::PRISMATIC) || 
          (link->parent_joint->type == urdf::Joint::CONTINUOUS) || 
          (link->parent_joint->type == urdf::Joint::REVOLUTE) 
        ) 
       )
    {
        names.push_back(link->parent_joint->name);
        //std::cout << "joint name : " << link->parent_joint->name << endl;
    }
    // add children:
    for (std::vector<urdf::LinkSharedPtr >::const_iterator child = link->child_links.begin();
         child!= link->child_links.end();
        ++child) {
            if (*child) {
                addUrdfJointName(*child,names);
            }
     } 
}

void UrdfExpressions3::getAllUrdfJointNames(std::vector<std::string>& names) {
    addUrdfJointName(root_link,names);
}


void UrdfExpressions3::getAllJointNames(std::vector<std::string>& names) {
    names.clear();
    for( VarList::iterator it=varlist.begin();it!=varlist.end();++it) {
        names.push_back( it->second.second );
    }
}

void UrdfExpressions3::getAllJointExpressions(std::vector<Expression<double>::Ptr>& exprs) {
    exprs.clear();
    for( VarList::iterator it=varlist.begin();it!=varlist.end();++it) {
        exprs.push_back( it->second.first );
    }
}


//====================================================================================================
// UrdfExpr implementation (uses the above defined class)
//====================================================================================================


void UrdfExpr3::readFromFile(const std::string& filename ) {
    if (!reader) {
        reader.reset( new UrdfExpressions3(jc_gen,poslimits, vellimits, effortlimits, dt) );
    }
    std::string full_filename;
    std::string path = getEnvVar("URDF_PATH");
    std::string fullfilename = getFullFileName(path, filename);
    bool result=reader->readFromFile(fullfilename);
    if (!result) throw std::invalid_argument("UrdfExpr: could not open file");
}

void UrdfExpr3::readFromParam(const std::string& filename ) {
    if (!reader) {
        reader.reset( new UrdfExpressions3(jc_gen,poslimits, vellimits, effortlimits, dt) );
    }
    bool result=reader->readFromParam(filename);
    if (!result) throw std::invalid_argument("UrdfExpr: could not open using ros parameter");
}

void UrdfExpr3::readFromString(const std::string& xmlstring ) {
    if (!reader) {
        reader.reset( new UrdfExpressions3(jc_gen,poslimits, vellimits, effortlimits, dt) );
    }
    bool result=reader->read_fromString(xmlstring);
    if (!result) throw std::invalid_argument("UrdfExpr: could not open using string input");
}

void UrdfExpr3::addTransform(const std::string& name, const std::string& frame, const std::string& base) {
    if (!reader) {
        throw std::invalid_argument("UrdfExpr: addTransform called before urdf file was read"); 
    }
    //std::cout << "add transform " << name << " with number " << count-1 << std::endl;
    bool result=reader->addTransform(frame,base);
    if (result) {
        name_to_number[name] = count;
        count++;
    } else {
        throw std::invalid_argument("UrdfExpr: could not add transform");
    }
}

ExpressionMap UrdfExpr3::getExpressions(Context::Ptr& ctx) {
    using namespace std;
    typedef map<string, int> M;
    if (!reader) {
        return ExpressionMap();
    }
    ExpressionMap em;
    //cout << "getExpression started " << endl;
    for (M::iterator it = name_to_number.begin(); it!= name_to_number.end(); ++it) {
        //cout << "Expression " << it->first << " with number " << it->second << endl;
        Expression<Frame>::Ptr e = reader->getExpression(ctx,it->second);
        if (!e) {
            //cout << it->first << "  :  returned value was NIL" << endl;
            continue;
        }
        //cout << it->first << " : "; e->print(cout);cout << endl;
        em[ it->first ] = e;
    } 
    //cout << "getExpression ended " << endl;
    return em;
}

std::vector<std::string> UrdfExpr3::getLinkNames() {
    std::vector<std::string> s;
    if (!reader) {
        return s;
    }
    for (UrdfExpressions3::LinkMap::iterator it = reader->linkmap.begin();it!=reader->linkmap.end();++it) {
        s.push_back(it->first);
    }
    return s;
}

void UrdfExpr3::getAllUrdfJointNames(std::vector<std::string>& names) {
    if (reader) {
        reader->getAllUrdfJointNames(names);
    }
}

void UrdfExpr3::getAllJointNames(std::vector<std::string>& names) {
    if (reader) {
        reader->getAllJointNames(names);
    }
}

void UrdfExpr3::getAllJointExpressions(std::vector<Expression<double>::Ptr>& exprs) {
    if (reader) {
        reader->getAllJointExpressions(exprs);
    }
}
void UrdfExpr3::getAllLinkProperties(std::vector<link_property>& props ) {
    if (reader) {
        reader->getAllLinkProperties(props,reader->root_link);
    }
}

std::ostream& operator << ( std::ostream& os, const UrdfExpr3& u) {
    if (!u.reader) {
        return os;
    }
    for (UrdfExpressions3::LinkMap::iterator it = u.reader->linkmap.begin();it!=u.reader->linkmap.end();++it) {
        os << it->first << "  ,  ";
    }
    return os;
}

}// namespace KDL
