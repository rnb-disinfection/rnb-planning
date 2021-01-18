#ifndef EXPRESSIONGRAPH_CONTEXT_AUX_HPP
#define EXPRESSIONGRAPH_CONTEXT_AUX_HPP
namespace KDL{

inline int addConstraint(
        Context::Ptr                ctx,
        const std::string&          name,
        Expression<double>::Ptr     expr,
        Expression<double>::Ptr     K, 
        Expression<double>::Ptr     weight,
        int                         priority
    ) {
    Controller::Ptr cL = ctx->createDefaultController();    
    cL->setParameter("K", K );
    Controller::Ptr cU = ctx->createDefaultController();    
    cU->setParameter("K", K );
    return ctx->addInequalityConstraint( name, 
        expr, expr, 
        0.0, 0.0, 
        cL, cU, 
        weight,
        priority );
}

inline int addConstraint(
        Context::Ptr                ctx,
        const std::string&          name,
        Expression<double>::Ptr     expr,
        Expression<double>::Ptr     K, 
        double                      weight,
        int                         priority
    ) {
    Controller::Ptr cL = ctx->createDefaultController();    
    cL->setParameter("K", K );
    Controller::Ptr cU = ctx->createDefaultController();    
    cU->setParameter("K", K );
    return ctx->addInequalityConstraint( name, 
        expr, expr, 
        0.0, 0.0, 
        cL, cU, 
        Constant<double>(weight),
        priority );
}

inline int addConstraint(
        Context::Ptr                ctx,
        const std::string&          name,
        Expression<double>::Ptr     expr,
        double                      K, 
        Expression<double>::Ptr     weight,
        int                         priority
    ) {
    Controller::Ptr cL = ctx->createDefaultController();    
    cL->setParameter("K", Constant<double>(K) );
    Controller::Ptr cU = ctx->createDefaultController();    
    cU->setParameter("K", Constant<double>(K) );
    return ctx->addInequalityConstraint( name, 
        expr, expr, 
        0.0, 0.0, 
        cL, cU, 
        weight,
        priority );
}

inline int addConstraint(
        Context::Ptr                ctx,
        const std::string&          name,
        Expression<double>::Ptr     expr,
        double                      K, 
        double                      weight,
        int                         priority
    ) {
    Controller::Ptr cL = ctx->createDefaultController();    
    cL->setParameter("K", Constant<double>(K) );
    Controller::Ptr cU = ctx->createDefaultController();    
    cU->setParameter("K", Constant<double>(K) );
    return ctx->addInequalityConstraint( name, 
        expr, expr, 
        0.0, 0.0, 
        cL, cU, 
        Constant<double>(weight),
        priority );
}


inline int addInequalityConstraint(
        Context::Ptr                ctx,
        const std::string&          name,
        Expression<double>::Ptr     expr,
        double                      target_lower,
        Expression<double>::Ptr     K_lower, 
        double                      target_upper,
        Expression<double>::Ptr     K_upper,
        Expression<double>::Ptr     weight,
        int                         priority
    ) {
    Controller::Ptr cL = ctx->createDefaultController();    
    cL->setParameter("K", K_lower );
    Controller::Ptr cU = ctx->createDefaultController();    
    cU->setParameter("K", K_upper );
    return ctx->addInequalityConstraint( name, 
        expr, expr, 
        target_lower, target_upper, 
        cL, cU, 
        weight,
        priority );

}

inline int addInequalityConstraint(
        Context::Ptr                ctx,
        const std::string&          name,
        Expression<double>::Ptr     expr,
        double                      target_lower,
        double                      K_lower, 
        double                      target_upper,
        double                      K_upper,
        Expression<double>::Ptr     weight,
        int                         priority
    ) {
    Controller::Ptr cL = ctx->createDefaultController();    
    cL->setParameter("K", Constant<double>(K_lower) );
    Controller::Ptr cU = ctx->createDefaultController();    
    cU->setParameter("K", Constant<double>(K_upper) );
    return ctx->addInequalityConstraint( name, 
        expr, expr, 
        target_lower, target_upper, 
        cL, cU, 
        weight,
        priority );
}



inline int addInequalityConstraint(
        Context::Ptr                ctx,
        const std::string&          name,
        Expression<double>::Ptr     expr,
        double                      target_lower,
        Expression<double>::Ptr     K_lower, 
        double                      target_upper,
        Expression<double>::Ptr     K_upper,
        double                      weight,
        int                         priority
    ) {
    Controller::Ptr cL = ctx->createDefaultController();    
    cL->setParameter("K", K_lower );
    Controller::Ptr cU = ctx->createDefaultController();    
    cU->setParameter("K", K_upper );
    return ctx->addInequalityConstraint( name, 
        expr, expr, 
        target_lower, target_upper, 
        cL, cU, 
        Constant<double>(weight),
        priority );

}

inline int addInequalityConstraint(
        Context::Ptr                ctx,
        const std::string&          name,
        Expression<double>::Ptr     expr,
        double                      target_lower,
        double                      K_lower, 
        double                      target_upper,
        double                      K_upper,
        double                      weight,
        int                         priority
    ) {
    Controller::Ptr cL = ctx->createDefaultController();    
    cL->setParameter("K", Constant<double>(K_lower) );
    Controller::Ptr cU = ctx->createDefaultController();    
    cU->setParameter("K", Constant<double>(K_upper) );
    return ctx->addInequalityConstraint( name, 
        expr, expr, 
        target_lower, target_upper, 
        cL, cU, 
        Constant<double>(weight),
        priority );
}










}// namespace KDL
#endif
