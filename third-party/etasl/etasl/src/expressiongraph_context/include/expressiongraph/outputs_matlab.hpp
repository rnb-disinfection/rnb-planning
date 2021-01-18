#ifndef EXPRESSIONGRAPH_OUTPUTS_MATLAB_HPP
#define EXPRESSIONGRAPH_OUTPUTS_MATLAB_HPP

#include <expressiongraph/context.hpp>
#include <iostream>
#include <string>

namespace KDL {

/**
 * create an OutputGenerator to output outputs with the given type to the given stream in a Matlab/Octave format.
 * \param [in] os stream to write to.
 * \param [in] type string that indicates which types of outputs will be written to matlab.
 * \param [in] next (optional) another OutputGenerator to handle different types of output.
 */
OutputGenerator::Ptr create_matlab_output( 
    std::ostream& os, 
    const std::string& type, 
    OutputGenerator::Ptr next
);
OutputGenerator::Ptr create_matlab_output( 
    std::ostream& os, 
    const std::string& type
);




};//namespace
#endif
