/*    Copyright (c) 2010-2016, Delft University of Technology
 *    All rights reserved.
 *
 *    Redistribution and use in source and binary forms, with or without modification, are
 *    permitted provided that the following conditions are met:
 *      - Redistributions of source code must retain the above copyright notice, this list of
 *        conditions and the following disclaimer.
 *      - Redistributions in binary form must reproduce the above copyright notice, this list of
 *        conditions and the following disclaimer in the documentation and/or other materials
 *        provided with the distribution.
 *      - Neither the name of the Delft University of Technology nor the names of its contributors
 *        may be used to endorse or promote products derived from this software without specific
 *        prior written permission.
 *
 *    THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS
 *    OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 *    MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *    COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 *    EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
 *    GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 *    AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
 *    NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED
 *    OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 *    Changelog
 *      YYMMDD    Author            Comment
 *      101111    K. Kumar          File created.
 *      110113    K. Kumar          Scenario updated to use latest version of code; added file
 *                                  header and footer.
 *      110202    K. Kumar          Scenario updated to use latest version of code.
 *      110216    K. Kumar          Migrated to applications namespace.
 *      110217    K. Kumar          Function name changed.
 *      110815    K. Kumar          Updated with mass of Asterix.
 *      111024    K. Kumar          Modified to be executable program with main-function as
 *                                  suggested by M. Persson.
 *      120221    K. Kumar          Rewrote application from scratch; now propagates two
 *                                  satellites.
 *      120502    K. Kumar          Updated code to use shared pointers.
 *      121030    K. Kumar          Updated code to use new state derivative models.
 *      130107    K. Kumar          Updated license in file header.
 *      130225    K. Kumar          Updated gravitational acceleration model references; renamed
 *                                  file; fixed error in assigning Obelix state derivative model;
 *                                  made variables const-correct.
 *
 *      160322    R. Hoogendoorn    GSL Library example
 *
 *    References
 *
 *    https://github.com/ampl/gsl
 *    http://www.gnu.org/software/gsl/
 *
 *    Notes
 *
 */

#include <iostream> // cout sometimes needs this

#include <cstdio>
#include <vector>
#include <fstream>
#include <iostream>

#include <iomanip> // setw

#include <algorithm>
#include <utility>

#include <Eigen/Core>

#include <boost/assign/list_of.hpp>
#include <boost/bind.hpp>
#include <boost/test/unit_test.hpp>

// NRLMSISE00
#include "Tudat/Astrodynamics/Aerodynamics/nrlmsise00Atmosphere.h"
#include "Tudat/Astrodynamics/Aerodynamics/tabulatedAtmosphere.h"

// Input Output
#include "Tudat/InputOutput/basicInputOutput.h"

//// ============================ Header / Declarations ====================== ////

//tudat::aerodynamics::NRLMSISE00Input data ; // Input data

using tudat::aerodynamics::NRLMSISE00Input;
using tudat::aerodynamics::NRLMSISE00Atmosphere;

// Global variable to be changed by tests and function.
NRLMSISE00Input data;

// Define input data generic (or almost completely) for all tests.
NRLMSISE00Input gen_data(0, 172, 29000.0, 16.0, 150.0, 150.0, 4.0);
std::vector< double > gen_input = boost::assign::list_of(400.0)(-70.0)(60.0)(0.0);

NRLMSISE00Input function(double altitude, double longitude,
                                              double latitude, double time,
                                              bool computeLocalSolarTime,
                                              bool invariableLower ) {
    // Functionality encountered in the original wrapper class, these
    // have been moved out of the Tudat space and now into the
    // application space. This is given here as an example of how to
    // solve these problems and to keep the code for future generations :).
    // NOTE: both these functions are switched of for testing.
    if (computeLocalSolarTime) {
        data.localSolarTime = data.secondOfTheDay/3600.0 + longitude/15.0;
    }
    if (invariableLower && altitude < 80000.0) {
        data.f107        = 150.0;
        data.f107a       = 150.0;
        data.apDaily     = 4.0;
        data.switches[9] = 1;
    }
    return data;
}

//// =================== Start Main code ==================== ////
int main()
{
    // Define tolerance for equality
//    double tol = 1.0E-18;

    // Create the model
    NRLMSISE00Atmosphere model(boost::bind(&function, _1, _2, _3, _4, false, false));

    // Create local copy of input and define variations
    data = gen_data;
    std::vector< double > input = gen_input;
    // Define variations from the standard case
    // First case is the default case

    // Get full output and extract density and temperature
    std::pair<std::vector<double>, std::vector<double>> output
            = model.getFullOutput(input[0], input[1], input[2], input[3]);
    double density1     = output.first[5]*1000.0;
    double temperature1 = output.second[1];

    // Get density and temperature from functions
    double density2     = model.getDensity(input[0], input[1], input[2], input[3]);
    double temperature2 = model.getTemperature(input[0], input[1], input[2], input[3]);

    std::cout << "Density 1     : " << density1 << std::endl;
    std::cout << "Temperature 1 : " << temperature1 << std::endl;

    std::cout << "Density 2     : " << density2 << std::endl;
    std::cout << "Temperature 2 : " << temperature2 << std::endl;

  return 0;
}
