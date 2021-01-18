#include <gtest/gtest.h>




#include <expressiongraph_tf/qpoases_solver.hpp>
#include "SolverTestsHelpers.hpp"


using namespace KDL;

//run with ```catkin_make run_tests_expressiongraph_context```

 // Declare a test
TEST(qpoasesSolvers, generalTest)
{
	using namespace KDL;

	Context::Ptr ctx=dummyCtx2();



	KDL::Expression<double>::Ptr errorX =ctx->getOutputExpression<double>("errorX");
	KDL::Expression<double>::Ptr errorY =ctx->getOutputExpression<double>("errorY");
	KDL::Expression<double>::Ptr errorF =ctx->getOutputExpression<double>("errorF");
	bool errorXisNullPointer=!errorX;
	bool errorYisNullPointer=!errorY;
	bool errorFisNullPointer=!errorF;
	ASSERT_FALSE(errorXisNullPointer);
	ASSERT_FALSE(errorYisNullPointer);
	ASSERT_FALSE(errorFisNullPointer);



	qpOASESSolver solver(100,0.0, 1E-8);
	ASSERT_EQ(solver.prepareInitialization(ctx),0);

	double timeStep=0.01;
	for (int i=0;i<100;++i) {
		int solverErrorInUpdateInitialization=solver.updateStep(timeStep);
		ASSERT_EQ( solverErrorInUpdateInitialization, 0);
	}
	ASSERT_EQ(solver.prepareExecution(ctx),0);

	//outgen->init(ctx);
	for (int i=0;i<500;++i) {
		int solverErrorInUpdateExecution=solver.updateStep(timeStep);
		ASSERT_EQ( solverErrorInUpdateExecution, 0);
	//	outgen->update(ctx);
	}
//	outgen->finish(ctx);






	double errorXvalue=errorX->value();
	EXPECT_NEAR(errorXvalue, 0.0, 0.000001);
	double errorYvalue=errorY->value();
	EXPECT_NEAR(errorYvalue, 0.0, 0.000001);
	double errorFvalue=errorF->value();
	EXPECT_NEAR(errorFvalue, 0.0, 0.000001);
}


// Run all the tests that were declared with TEST()
int main(int argc, char **argv){
	testing::InitGoogleTest(&argc, argv);
	return RUN_ALL_TESTS();
}




