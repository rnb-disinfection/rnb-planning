#ifndef EXPRESSIONGRAPH_HELPER_SOLVER_HPP
#define EXPRESSIONGRAPH_HELPER_SOLVER_HPP



#include <expressiongraph_tf/context.hpp>
#include <expressiongraph_tf/outputs_matlab.hpp>


using namespace KDL;
using namespace std;
 Context::Ptr dummyCtx1()
 {
	 Context::Ptr ctx = create_context();

	 // set-up necessary for robot control problems:
	 ctx->addType("robot");
	 ctx->addType("feature");
	 ctx->addType("time");
	 ctx->addScalarVariable("time","time",0.0, Constant(1.0) );

	 // define the problem: a 1-dof robot in 2d-space
	 // not the best way, just an illustration
	 double K = 4;
	 Expression<double>::Ptr q     = ctx->addScalarVariable("q","robot");
	 Expression<double>::Ptr f     = ctx->addScalarVariable("f","feature");
	 Expression<double>::Ptr L     = Constant(0.2);
	 Expression<double>::Ptr ee_x  = (L+f)*cos(q);
	 Expression<double>::Ptr ee_y  = (L+f)*sin(q);
	 Expression<double>::Ptr des_x = Constant(0.2);
	 Expression<double>::Ptr des_y = Constant(0.2);
	 //ctx->addConstraint("tracking_x",  ee_x - des_x, K, 1.0, 1);
	 //ctx->addConstraint("tracking_y",  ee_y - des_y, K, 1.0, 1);

	 ControllerRegistry reg;
	 reg.register_controller(create_controller_proportional());
	 reg.register_controller(create_controller_proportional_saturated());
	 Controller::Ptr c = reg.lookupPrototype("proportional");
	 c->setParameter("K", Constant<double>(10.0));

	 ctx->addInequalityConstraint("tracking_x",
			 ee_x - des_x,    ee_x - des_x,
			 0.0, 0.0, c->clone(), c->clone(),
			 Constant<double>(1.0), 1);

	 ctx->addInequalityConstraint("tracking_y",
			 ee_y - des_y,    ee_y - des_y,
			 0.0, 0.0, c->clone(), c->clone(),
			 Constant<double>(1.0), 1);

	 ctx->addInequalityConstraint("tracking_y0",
			 ee_y - des_y,    ee_y - des_y,
			 0.0, 0.0, c->clone(), c->clone(),
			 Constant<double>(1.0), 0);
	 ctx->addInequalityConstraint("tracking_y00",
			 ee_y - des_y,    ee_y - des_y,
			 0.0, 0.0, c->clone(), c->clone(),
			 Constant<double>(1.0), 0);


	 ctx->addInequalityConstraint("tracking_y2",
			 ee_y - des_y,    ee_y - des_y,
			 0.0, 0.0, c->clone(), c->clone(),
			 Constant<double>(1.0), 2);

	 // specification of the output:
	 ctx->addOutput<double>("q","matlab",q);
	 ctx->addOutput<double>("f","matlab",f);
	 ctx->addOutput<double>("dx","matlab",ee_x - des_x);
	 ctx->addOutput<double>("dy","matlab",ee_y - des_y);
	 return ctx;
 }
 Context::Ptr dummyCtx2(  )
 {
	 Context::Ptr ctx = create_context();

	 // set-up necessary for robot control problems:
	 ctx->addType("robot");
	 ctx->addType("feature");
	 ctx->addType("time");
	 ctx->addScalarVariable("time","time",0.0, Constant(1.0) );


	 double K = 4;
	  Expression<double>::Ptr t     = ctx->addScalarVariable("t","time");
	 Expression<double>::Ptr q1     = ctx->addScalarVariable("q1","robot");
	 Expression<double>::Ptr q2     = ctx->addScalarVariable("q2","robot");
	 Expression<double>::Ptr q3     = ctx->addScalarVariable("q3","robot");
	 Expression<double>::Ptr q4     = ctx->addScalarVariable("q4","robot");
	 Expression<double>::Ptr f      = ctx->addScalarVariable("f","feature");

	 Expression<Vector>::Ptr L      = Constant(Vector(0,0,0.2));
	 Expression<Vector>::Ptr L0=  Constant(Vector(0,0,0.0));
	 Expression<Rotation>::Ptr R0=  Constant(Rotation(KDL::Rotation::Identity()));
	 Expression<Frame>::Ptr w_T_l1  =       frame( rot_z(q1), L0)*frame( R0, L);
	 Expression<Frame>::Ptr w_T_l2 = w_T_l1*frame( rot_x(q2), L0)*frame( R0, L);
	 Expression<Frame>::Ptr w_T_l3 = w_T_l2*frame( rot_x(q3), L0)*frame( R0, L);
	 Expression<Frame>::Ptr w_T_ee = w_T_l3*frame( rot_z(q4), L0)*frame( R0, L);
	 Expression<double>::Ptr ee_x = coord_x(origin(w_T_ee));
	 Expression<double>::Ptr ee_y = coord_y(origin(w_T_ee));

	 Expression<double>::Ptr des_x = Constant(0.2);
	 Expression<double>::Ptr des_y = Constant(0.2);
	 Expression<double>::Ptr des_f = ee_x;

	 ctx->setOutputExpression<double>("errorX",ee_x - des_x);
	 ctx->setOutputExpression<double>("errorY",ee_x - des_x);
	 ctx->setOutputExpression<double>("errorF",ee_x - des_x);


	 ControllerRegistry reg;
	 reg.register_controller(create_controller_proportional());
	 reg.register_controller(create_controller_proportional_saturated());
	 Controller::Ptr c = reg.lookupPrototype("proportional");
	 c->setParameter("K", Constant<double>(10.0));

	 ctx->addInequalityConstraint("tracking_x",
			 ee_x - des_x,    ee_x - des_x,
			 0.0, 0.0, c->clone(), c->clone(),
			 Constant<double>(1.0), 2);

	 ctx->addBoxConstraint("q1lim",1,-3.14/2,3.14/2);

	 ctx->addInequalityConstraint("tracking_y",
			 ee_y - des_y,    ee_y - des_y,
			 0.0, 0.0, c->clone(), c->clone(),
			 Constant<double>(1.0), 2);

	 ctx->addInequalityConstraint("tracking_f",
			 f - des_f,    f - des_f,
			 0.0, 0.0, c->clone(), c->clone(),
			 Constant<double>(1.0), 0);

	 return ctx;
 }

 Context::Ptr dummyCtxSotScalarWithFeature(  )
 {
	 Context::Ptr ctx = create_context();

	 // set-up necessary for robot control problems:
	 ctx->addType("robot");
	 ctx->addType("feature");
	 ctx->addType("time");
	 ctx->addScalarVariable("time","time",0.0, Constant(1.0) );


	 double K = 4;
	  Expression<double>::Ptr t     = ctx->addScalarVariable("t","time");
	 Expression<double>::Ptr q1     = ctx->addScalarVariable("q1","robot");
	 Expression<double>::Ptr q2     = ctx->addScalarVariable("q2","robot");
	 Expression<double>::Ptr q3     = ctx->addScalarVariable("q3","robot");
	 Expression<double>::Ptr q4     = ctx->addScalarVariable("q4","robot");
	 Expression<double>::Ptr f      = ctx->addScalarVariable("f","feature");

	 Expression<Vector>::Ptr L      = Constant(Vector(0,0,0.2));
	 Expression<Vector>::Ptr L0=  Constant(Vector(0,0,0.0));
	 Expression<Rotation>::Ptr R0=  Constant(Rotation(KDL::Rotation::Identity()));
	 Expression<Frame>::Ptr w_T_l1  =       frame( rot_z(q1), L0)*frame( R0, L);
	 Expression<Frame>::Ptr w_T_l2 = w_T_l1*frame( rot_x(q2), L0)*frame( R0, L);
	 Expression<Frame>::Ptr w_T_l3 = w_T_l2*frame( rot_x(q3), L0)*frame( R0, L);
	 Expression<Frame>::Ptr w_T_ee = w_T_l3*frame( rot_z(q4), L0)*frame( R0, L);
	 Expression<double>::Ptr ee_x = coord_x(origin(w_T_ee));
	 Expression<double>::Ptr ee_y = coord_y(origin(w_T_ee));

	 Expression<double>::Ptr des_x = Constant(0.2);
	 Expression<double>::Ptr des_y = Constant(0.2);
	 Expression<double>::Ptr des_f = ee_x;

	 ctx->setOutputExpression<double>("errorX",ee_x - des_x);
	 ctx->setOutputExpression<double>("errorY",ee_y - des_y);
	 ctx->setOutputExpression<double>("errorF",f - des_f);


	 ControllerRegistry reg;
	 reg.register_controller(create_controller_proportional());
	 reg.register_controller(create_controller_proportional_saturated());
	 Controller::Ptr c = reg.lookupPrototype("proportional");
	 c->setParameter("K", Constant<double>(10.0));

	 ctx->addInequalityConstraint("tracking_x",
			 ee_x - des_x,    ee_x - des_x,
			 0.0, 0.0, c->clone(), c->clone(),
			 Constant<double>(1.0), 1);



	 ctx->addInequalityConstraint("tracking_y",
			 ee_y - des_y,    ee_y - des_y,
			 0.0, 0.0, c->clone(), c->clone(),
			 Constant<double>(1.0), 1);

	 ctx->addInequalityConstraint("tracking_f",
			 f - des_f,    f - des_f,
			 0.0, 0.0, c->clone(), c->clone(),
			 Constant<double>(1.0), 0);

	 return ctx;
 }

 Context::Ptr dummyCtxSotScalarWithoutFeature(  )
 {
	 Context::Ptr ctx = create_context();

	 // set-up necessary for robot control problems:
	 ctx->addType("robot");
	 ctx->addType("time");
	 ctx->addScalarVariable("time","time",0.0, Constant(1.0) );

	 double K = 4;
	  Expression<double>::Ptr t     = ctx->addScalarVariable("t","time");
	 Expression<double>::Ptr q1     = ctx->addScalarVariable("q1","robot");
	 Expression<double>::Ptr q2     = ctx->addScalarVariable("q2","robot");
	 Expression<double>::Ptr q3     = ctx->addScalarVariable("q3","robot");
	 Expression<double>::Ptr q4     = ctx->addScalarVariable("q4","robot");

	 Expression<Vector>::Ptr L      = Constant(Vector(0,0,0.2));
	 Expression<Vector>::Ptr L0=  Constant(Vector(0,0,0.0));
	 Expression<Rotation>::Ptr R0=  Constant(Rotation(KDL::Rotation::Identity()));
	 Expression<Frame>::Ptr w_T_l1  =       frame( rot_z(q1), L0)*frame( R0, L);
	 Expression<Frame>::Ptr w_T_l2 = w_T_l1*frame( rot_x(q2), L0)*frame( R0, L);
	 Expression<Frame>::Ptr w_T_l3 = w_T_l2*frame( rot_x(q3), L0)*frame( R0, L);
	 Expression<Frame>::Ptr w_T_ee = w_T_l3*frame( rot_z(q4), L0)*frame( R0, L);
	 Expression<double>::Ptr ee_x = coord_x(origin(w_T_ee));
	 Expression<double>::Ptr ee_y = coord_y(origin(w_T_ee));

	 Expression<double>::Ptr des_x = Constant(0.2);
	 Expression<double>::Ptr des_y = Constant(0.2);

	 ctx->setOutputExpression<double>("errorX",ee_x - des_x);
	 ctx->setOutputExpression<double>("errorY",ee_y - des_y);

	 ControllerRegistry reg;
	 reg.register_controller(create_controller_proportional());
	 reg.register_controller(create_controller_proportional_saturated());
	 Controller::Ptr c = reg.lookupPrototype("proportional");
	 c->setParameter("K", Constant<double>(10.0));

	 ctx->addInequalityConstraint("tracking_x",
			 ee_x - des_x,    ee_x - des_x,
			 0.0, 0.0, c->clone(), c->clone(),
			 Constant<double>(1.0), 2);

	 ctx->addInequalityConstraint("tracking_y",
			 ee_y - des_y,    ee_y - des_y,
			 0.0, 0.0, c->clone(), c->clone(),
			 Constant<double>(1.0), 2);

	 return ctx;
 }
 Context::Ptr dummyCtxSotScalarWithoutFeatureBox(  )
 {
	 Context::Ptr ctx = create_context();

	 // set-up necessary for robot control problems:
	 ctx->addType("robot");
	 ctx->addType("time");
	 ctx->addScalarVariable("time","time",0.0, Constant(1.0) );

	 double K = 4;

	 Expression<double>::Ptr q1     = ctx->addScalarVariable("q1","robot");
	 Expression<double>::Ptr q2     = ctx->addScalarVariable("q2","robot");
	 Expression<double>::Ptr q3     = ctx->addScalarVariable("q3","robot");
	 Expression<double>::Ptr q4     = ctx->addScalarVariable("q4","robot");


	 std::vector<int> qindx;
	 ctx->getScalarsOfType("robot",qindx);
	 /*cout<<"ctx->getScalarsOfType(Robot);"<<endl;

	 for (unsigned int i=0;i<qindx.size();i++)
	 {
		 cout<<qindx[i];
		 if(i!=qindx.size()-1) cout<<"\t";
	 }
	 cout<<endl;*/
	 Expression<Vector>::Ptr L      = Constant(Vector(0,0,0.2));
	 Expression<Vector>::Ptr L0=  Constant(Vector(0,0,0.0));
	 Expression<Rotation>::Ptr R0=  Constant(Rotation(KDL::Rotation::Identity()));
	 Expression<Frame>::Ptr w_T_l1  =       frame( rot_z(q1), L0)*frame( R0, L);
	 Expression<Frame>::Ptr w_T_l2 = w_T_l1*frame( rot_x(q2), L0)*frame( R0, L);
	 Expression<Frame>::Ptr w_T_l3 = w_T_l2*frame( rot_x(q3), L0)*frame( R0, L);
	 Expression<Frame>::Ptr w_T_ee = w_T_l3*frame( rot_z(q4), L0)*frame( R0, L);
	 Expression<double>::Ptr ee_x = coord_x(origin(w_T_ee));
	 Expression<double>::Ptr ee_y = coord_y(origin(w_T_ee));

	 Expression<double>::Ptr des_x = Constant(0.2);
	 Expression<double>::Ptr des_y = Constant(0.2);

	 ctx->setOutputExpression<double>("errorX",ee_x - des_x);
	 ctx->setOutputExpression<double>("errorY",ee_y - des_y);

	 ControllerRegistry reg;
	 reg.register_controller(create_controller_proportional());
	 reg.register_controller(create_controller_proportional_saturated());
	 Controller::Ptr c = reg.lookupPrototype("proportional");
	 c->setParameter("K", Constant<double>(10.0));

	 ctx->addInequalityConstraint("tracking_x",
			 ee_x - des_x,    ee_x - des_x,
			 0.0, 0.0, c->clone(), c->clone(),
			 Constant<double>(1.0), 2);

	 ctx->addInequalityConstraint("tracking_y",
			 ee_y - des_y,    ee_y - des_y,
			 0.0, 0.0, c->clone(), c->clone(),
			 Constant<double>(1.0), 2);

	 ctx->addBoxConstraint("q1_lim",qindx[0],-1.51,1.57);
	 ctx->addBoxConstraint("q2_lim",qindx[1],-1.52,1.57);
	 ctx->addBoxConstraint("q3_lim",qindx[2],-1.53,1.57);
	 ctx->addBoxConstraint("q4_lim",qindx[3],-1.54,1.57);

	 return ctx;
 }

 Context::Ptr dummyCtxSotScalarWithoutFeatureBoxConflict(  )
 {
	 Context::Ptr ctx = create_context();

	 // set-up necessary for robot control problems:
	 ctx->addType("robot");
	 ctx->addType("time");
	 ctx->addScalarVariable("time","time",0.0, Constant(1.0) );

	 double K = 4;

	 Expression<double>::Ptr q1     = ctx->addScalarVariable("q1","robot",0.1,4.0);
	 Expression<double>::Ptr q2     = ctx->addScalarVariable("q2","robot",0.2,3.0);
	 Expression<double>::Ptr q3     = ctx->addScalarVariable("q3","robot",0.3,2.0);
	 Expression<double>::Ptr q4     = ctx->addScalarVariable("q4","robot",0.4,1.0);


	 std::vector<int> qindx;
	 ctx->getScalarsOfType("robot",qindx);
	 /*cout<<"ctx->getScalarsOfType(Robot);"<<endl;

	 for (unsigned int i=0;i<qindx.size();i++)
	 {
		 cout<<qindx[i];
		 if(i!=qindx.size()-1) cout<<"\t";
	 }
	 cout<<endl;*/
	 Expression<Vector>::Ptr L      = Constant(Vector(0,0,0.2));
	 Expression<Vector>::Ptr L0=  Constant(Vector(0,0,0.0));
	 Expression<Rotation>::Ptr R0=  Constant(Rotation(KDL::Rotation::Identity()));
	 Expression<Frame>::Ptr w_T_l1  =       frame( rot_z(q1), L0)*frame( R0, L);
	 Expression<Frame>::Ptr w_T_l2 = w_T_l1*frame( rot_x(q2), L0)*frame( R0, L);
	 Expression<Frame>::Ptr w_T_l3 = w_T_l2*frame( rot_x(q3), L0)*frame( R0, L);
	 Expression<Frame>::Ptr w_T_ee = w_T_l3*frame( rot_z(q4), L0)*frame( R0, L);
	 Expression<double>::Ptr ee_x = coord_x(origin(w_T_ee));
	 Expression<double>::Ptr ee_y = coord_y(origin(w_T_ee));

	 Expression<double>::Ptr des_x1 = Constant(0.2);
	 Expression<double>::Ptr des_x2 = Constant(0.1);
	 Expression<double>::Ptr des_y = Constant(0.2);

	 ctx->setOutputExpression<double>("errorX1",ee_x - des_x1);
	 ctx->setOutputExpression<double>("errorX2",ee_x - des_x2);
	 ctx->setOutputExpression<double>("errorY",ee_y - des_y);


	 ControllerRegistry reg;
	 reg.register_controller(create_controller_proportional());
	 reg.register_controller(create_controller_proportional_saturated());
	 Controller::Ptr c = reg.lookupPrototype("proportional");
	 c->setParameter("K", Constant<double>(10.0));

	 ctx->addInequalityConstraint("tracking_x1",
			 ee_x - des_x1,    ee_x - des_x1,
			 0.0, 0.0, c->clone(), c->clone(),
			 Constant<double>(1.0), 2);

	 ctx->addInequalityConstraint("tracking_x2",
			 ee_x - des_x2,    ee_x - des_x2,
			 0.0, 0.0, c->clone(), c->clone(),
			 Constant<double>(4.0), 2);


	 ctx->addInequalityConstraint("tracking_y",
			 ee_y - des_y,    ee_y - des_y,
			 0.0, 0.0, c->clone(), c->clone(),
			 Constant<double>(1.0), 2);

	 ctx->addBoxConstraint("q1_lim",qindx[0],-1.51,1.57);
	 ctx->addBoxConstraint("q2_lim",qindx[1],-1.52,1.57);
	 ctx->addBoxConstraint("q3_lim",qindx[2],-1.53,1.57);
	 ctx->addBoxConstraint("q4_lim",qindx[3],-1.54,1.57);

	 return ctx;
 }


 Context::Ptr dummyCtxSotScalarWithFeatureWithBox(  )
 {
	 Context::Ptr ctx = create_context();

	 // set-up necessary for robot control problems:
	 ctx->addType("robot");
	 ctx->addType("time");
	 ctx->addType("feature");


	 double K = 4;

	 Expression<double>::Ptr q1     = ctx->addScalarVariable("q1","robot",0.1,4.0);
	 Expression<double>::Ptr q2     = ctx->addScalarVariable("q2","robot",0.2,3.0);
	 Expression<double>::Ptr q3     = ctx->addScalarVariable("q3","robot",0.3,2.0);
	 Expression<double>::Ptr q4     = ctx->addScalarVariable("q4","robot",0.4,1.0);
	 Expression<double>::Ptr f      = ctx->addScalarVariable("f","feature");



	 std::vector<int> qindx;
	 ctx->getScalarsOfType("robot",qindx);

	 Expression<Vector>::Ptr L      = Constant(Vector(0,0,0.2));
	 Expression<Vector>::Ptr L0=  Constant(Vector(0,0,0.0));
	 Expression<Rotation>::Ptr R0=  Constant(Rotation(KDL::Rotation::Identity()));
	 Expression<Frame>::Ptr w_T_l1  =       frame( rot_z(q1), L0)*frame( R0, L);
	 Expression<Frame>::Ptr w_T_l2 = w_T_l1*frame( rot_x(q2), L0)*frame( R0, L);
	 Expression<Frame>::Ptr w_T_l3 = w_T_l2*frame( rot_x(q3), L0)*frame( R0, L);
	 Expression<Frame>::Ptr w_T_ee = w_T_l3*frame( rot_z(q4), L0)*frame( R0, L);
	 Expression<double>::Ptr ee_x = coord_x(origin(w_T_ee));
	 Expression<double>::Ptr ee_y = coord_y(origin(w_T_ee));

	 Expression<double>::Ptr des_x = Constant(0.2);
	 Expression<double>::Ptr des_y = Constant(0.2);
	 Expression<double>::Ptr des_f = ee_x;

	 ctx->setOutputExpression<double>("errorX",ee_x - des_x);
	 ctx->setOutputExpression<double>("errorY",ee_y - des_y);
	 ctx->setOutputExpression<double>("errorF",f - des_f);
	 ControllerRegistry reg;
	 reg.register_controller(create_controller_proportional());

	 Controller::Ptr c = reg.lookupPrototype("proportional");
	 c->setParameter("K", Constant<double>(10.0));

	 ctx->addInequalityConstraint("tracking_x",
			 ee_x - des_x,    ee_x - des_x,
			 0.0, 0.0, c->clone(), c->clone(),
			 Constant<double>(1.0), 2);

	 ctx->addInequalityConstraint("tracking_y",
			 ee_y - des_y,    ee_y - des_y,
			 0.0, 0.0, c->clone(), c->clone(),
			 Constant<double>(1.0), 2);


	 ctx->addInequalityConstraint("tracking_f",
			 f - des_f,    f - des_f,
			 0.0, 0.0, c->clone(), c->clone(),
			 Constant<double>(1.0), 0);

	 ctx->addBoxConstraint("q1_lim",qindx[0],-1.51,1.57);
	 ctx->addBoxConstraint("q2_lim",qindx[1],-1.52,1.57);
	 ctx->addBoxConstraint("q3_lim",qindx[2],-1.53,1.57);
	 ctx->addBoxConstraint("q4_lim",qindx[3],-1.54,1.57);

	 return ctx;
 }

 inline Context::Ptr dummyCtxSotRotAndScalar(  )
 {
	 Context::Ptr ctx = create_context();

	 // set-up necessary for robot control problems:
	 ctx->addType("robot");
	 ctx->addType("feature");
	 ctx->addType("time");
	 ctx->addScalarVariable("time","time",0.0, Constant(1.0) );


	 double K = 4;
	  Expression<double>::Ptr t     = ctx->addScalarVariable("t","time");
	 Expression<double>::Ptr q1     = ctx->addScalarVariable("q1","robot");
	 Expression<double>::Ptr q2     = ctx->addScalarVariable("q2","robot");
	 Expression<double>::Ptr q3     = ctx->addScalarVariable("q3","robot");
	 Expression<double>::Ptr q4     = ctx->addScalarVariable("q4","robot");
	 Expression<double>::Ptr f      = ctx->addScalarVariable("f","feature");

	 Expression<Vector>::Ptr L    = Constant(Vector(0,0,0.2));
	 Expression<Vector>::Ptr L0   = Constant(Vector(0,0,0.0));
	 Expression<Rotation>::Ptr R0 = Constant(Rotation(KDL::Rotation::Identity()));
	 Expression<Frame>::Ptr w_T_l1 =        frame( rot_z(q1), L0)*frame( R0, L);
	 Expression<Frame>::Ptr w_T_l2 = w_T_l1*frame( rot_x(q2), L0)*frame( R0, L);
	 Expression<Frame>::Ptr w_T_l3 = w_T_l2*frame( rot_x(q3), L0)*frame( R0, L);
	 Expression<Frame>::Ptr w_T_ee = w_T_l3*frame( rot_z(q4), L0)*frame( R0, L);
	 Expression<Rotation>::Ptr ee_r = rotation(w_T_ee);


	 Expression<Rotation>::Ptr des_r = Constant(KDL::Rotation::RotZ(0.5));

	 Expression<double>::Ptr des_f = coord_x(origin(w_T_ee));

	 ctx->setOutputExpression<Rotation>("errorR",ee_r *inv(des_r));

	 ctx->setOutputExpression<double>("errorF",f - des_f);


	 ControllerRegistry reg;
	 reg.register_controller(create_controller_proportional());

	 Controller::Ptr c = reg.lookupPrototype("proportional");
	 c->setParameter("K", Constant<double>(10.0));


	 ctx->addConstraint("tracking_R",ee_r *inv(des_r),K,0.1,1);



	 ctx->addInequalityConstraint("tracking_f",
			 f - des_f,    f - des_f,
			 0.0, 0.0, c->clone(), c->clone(),
			 Constant<double>(1.0), 0);

	 return ctx;
 }
#endif

